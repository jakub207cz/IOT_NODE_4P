#include "definitions.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MCP9808_ADDR 0x1C
#define CONFIG_MAGIC 0x434F4E46 // "CONF"

// Spusteni testu pri startu (bring-up EEPROM + test preteceni bufferu).
// Pro ostry provoz lze nastavit na 0.
#define RUN_SELFTESTS 1

// EEPROM 25CSM04 - parametry a mapa pameti
#define EEPROM_SIZE 0x080000UL       // 4 Mbit = 512 KiB
#define EEPROM_PAGE_SIZE 256U        // zapis nesmi prekrocit hranici stranky
#define EEPROM_SPI_FREQ_HZ 8000000UL // SPI Mode 0, 8 MHz (rozsah 8-12 MHz)

#define EEPROM_CMD_WREN 0x06
#define EEPROM_CMD_WRITE 0x02
#define EEPROM_CMD_READ 0x03
#define EEPROM_CMD_RDSR 0x05
#define EEPROM_SR_WIP 0x01 // Write In Progress
#define EEPROM_SR_WEL 0x02 // Write Enable Latch
#define EEPROM_SR_BP 0x0C  // Block Protect bity

// Mapa pameti:
//   0x000000 - 0x0000FF  konfigurace DeviceConfig_t
//   0x000100 - 0x0001FF  scratch (test kruhoveho bufferu)
//   0x000200 - 0x0002FF  metadata kruhoveho bufferu
//   0x000300 - 0x07FFFF  data kruhoveho bufferu (zaznamy po 6 B)
#define EEPROM_CONFIG_ADDR 0x000000UL
#define EEPROM_SCRATCH_ADDR 0x000100UL
#define RB_META_ADDR 0x000200UL
#define RB_DATA_ADDR 0x000300UL

// Timeout cekani na WIP (zapis trva max ~5 ms, nechame velkou rezervu)
#define EEPROM_BUSY_TIMEOUT 50000UL

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t perioda_mereni;
  uint16_t perioda_odesilani;
  char wifi_ssid[32];
  char wifi_pass[64];
  char mqtt_broker[64];
  uint16_t mqtt_port;
  uint32_t device_id;
  uint16_t crc;
} DeviceConfig_t;

static volatile bool rtc_alarm_fired = false;
static uint32_t system_uptime_sec = 0;

// Vypocet CRC-16-CCITT pro overeni integrity dat
static uint16_t CRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
  }
  return crc;
}

// =====================================================================
// 1. HODINA: SPI BRING-UP & OVLADAC EEPROM 25CSM04
// =====================================================================
static inline void EEPROM_CS(bool enable) {
  if (enable) {
    PORT_PinClear(PORT_PIN_PA14);
  } else {
    PORT_PinSet(PORT_PIN_PA14);
  }
}

// Explicitni nastaveni SPI: Mode 0 (CPOL=0, CPHA=0), 8 bitu, 8 MHz.
// (MCC uz to ma stejne, ale takto je nastaveni jiste i po zmene v MCC.)
static bool EEPROM_SpiSetup(void) {
  SPI_TRANSFER_SETUP setup = {
      .clockFrequency = EEPROM_SPI_FREQ_HZ,
      .clockPhase = SPI_CLOCK_PHASE_LEADING_EDGE,
      .clockPolarity = SPI_CLOCK_POLARITY_IDLE_LOW,
      .dataBits = SPI_DATA_BITS_8,
  };
  return SERCOM2_SPI_TransferSetup(&setup, 0); // 0 = GCLK z MCC (48 MHz)
}

static void SPI_WriteBlocking(const void *buf, size_t len) {
  SERCOM2_SPI_Write((void *)buf, len);
  while (SERCOM2_SPI_IsBusy())
    ;
}

static void SPI_ReadBlocking(void *buf, size_t len) {
  SERCOM2_SPI_Read(buf, len);
  while (SERCOM2_SPI_IsBusy())
    ;
}

uint8_t EEPROM_ReadStatus(void) {
  uint8_t cmd = EEPROM_CMD_RDSR;
  uint8_t status;
  EEPROM_CS(true);
  SPI_WriteBlocking(&cmd, 1);
  SPI_ReadBlocking(&status, 1);
  EEPROM_CS(false);
  return status;
}

// Polling bitu WIP s timeoutem - pri chybejici/vadne EEPROM se MCU nezasekne
static bool EEPROM_WaitBusy(void) {
  for (uint32_t i = 0; i < EEPROM_BUSY_TIMEOUT; i++) {
    if ((EEPROM_ReadStatus() & EEPROM_SR_WIP) == 0) {
      return true;
    }
  }
  printf("[EEPROM] CHYBA: timeout WIP (EEPROM neodpovida)\r\n");
  return false;
}

bool EEPROM_Read(uint32_t addr, void *buf, size_t len) {
  uint8_t hdr[4] = {EEPROM_CMD_READ, (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)addr};
  if (!EEPROM_WaitBusy()) {
    return false;
  }
  EEPROM_CS(true);
  SPI_WriteBlocking(hdr, 4);
  SPI_ReadBlocking(buf, len);
  EEPROM_CS(false);
  return true;
}

// Zapis v ramci jedne stranky (addr..addr+len nesmi prekrocit hranici 256 B)
static bool EEPROM_WritePage(uint32_t addr, const uint8_t *buf, size_t len) {
  uint8_t wren = EEPROM_CMD_WREN;
  uint8_t hdr[4] = {EEPROM_CMD_WRITE, (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)addr};

  if (!EEPROM_WaitBusy()) {
    return false;
  }

  // 1. Povoleni zapisu (WREN) + kontrola, ze se WEL opravdu nastavil
  EEPROM_CS(true);
  SPI_WriteBlocking(&wren, 1);
  EEPROM_CS(false);
  if ((EEPROM_ReadStatus() & EEPROM_SR_WEL) == 0) {
    printf("[EEPROM] CHYBA: WEL se nenastavil po WREN\r\n");
    return false;
  }

  // 2. Samotny zapis dat
  EEPROM_CS(true);
  SPI_WriteBlocking(hdr, 4);
  SPI_WriteBlocking(buf, len);
  EEPROM_CS(false);

  // 3. Pockat na dokonceni interniho zapisu
  return EEPROM_WaitBusy();
}

// Obecny zapis - automaticky deli data podle hranic stranek.
// (Bez toho by zapis pres hranici stranky "pretekl" na jeji zacatek.)
bool EEPROM_Write(uint32_t addr, const void *buf, size_t len) {
  const uint8_t *p = (const uint8_t *)buf;
  while (len > 0) {
    size_t room = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
    size_t chunk = (len < room) ? len : room;
    if (!EEPROM_WritePage(addr, p, chunk)) {
      return false;
    }
    addr += chunk;
    p += chunk;
    len -= chunk;
  }
  return true;
}

// Bring-up test: zapis testovaciho pole na nahodnou adresu a kontrolni cteni.
// Adresa je zvolena tak, aby zapis prekracoval hranici stranky (test deleni).
// Puvodni obsah se pred testem zalohuje a po nem vrati -> test je
// nedestruktivni.
bool EEPROM_BringUpTest(void) {
  uint8_t backup[32], pattern[32], readback[32];

  uint8_t sr = EEPROM_ReadStatus();
  printf("[EEPROM] Status registr: 0x%02X\r\n", sr);
  if (sr == 0xFF) {
    printf("[EEPROM] CHYBA: status 0xFF - EEPROM nepripojena?\r\n");
    return false;
  }
  if (sr & EEPROM_SR_BP) {
    printf("[EEPROM] VAROVANI: aktivni Block Protect (BP=%u)\r\n",
           (sr & EEPROM_SR_BP) >> 2);
  }

  // "Nahodna" stranka v datove oblasti (z citace RTC), zapis 16 B pred koncem
  // stranky -> 16 B do jedne stranky + 16 B do nasledujici.
  uint32_t pages = (EEPROM_SIZE - RB_DATA_ADDR) / EEPROM_PAGE_SIZE - 1;
  uint32_t seed = RTC_Timer32CounterGet() * 1103515245UL + 12345UL;
  uint32_t page = (RB_DATA_ADDR / EEPROM_PAGE_SIZE) + (seed % pages);
  uint32_t addr = (page + 1) * EEPROM_PAGE_SIZE - 16;

  for (uint8_t i = 0; i < sizeof(pattern); i++) {
    pattern[i] = (uint8_t)(0xA5 ^ (i * 7) ^ (uint8_t)seed);
  }

  bool ok = EEPROM_Read(addr, backup, sizeof(backup)) &&
            EEPROM_Write(addr, pattern, sizeof(pattern)) &&
            EEPROM_Read(addr, readback, sizeof(readback));
  ok = ok && (memcmp(pattern, readback, sizeof(pattern)) == 0);

  EEPROM_Write(addr, backup, sizeof(backup)); // obnova puvodnich dat

  printf("[EEPROM] Test zapis/cteni 32 B na 0x%06lX (pres hranici stranky): "
         "%s\r\n",
         addr, ok ? "OK" : "CHYBA");
  if (!ok) {
    printf("[EEPROM]   zapsano: ");
    for (uint8_t i = 0; i < 8; i++)
      printf("%02X ", pattern[i]);
    printf("...\r\n[EEPROM]   precteno: ");
    for (uint8_t i = 0; i < 8; i++)
      printf("%02X ", readback[i]);
    printf("...\r\n");
  }
  return ok;
}

// =====================================================================
// 2. HODINA: KONFIGURACE ZARIZENI
// =====================================================================
void Config_Init(DeviceConfig_t *cfg) {
  EEPROM_Read(EEPROM_CONFIG_ADDR, cfg, sizeof(DeviceConfig_t));
  uint16_t calculated_crc =
      CRC16((uint8_t *)cfg, sizeof(DeviceConfig_t) - sizeof(uint16_t));

  // Kontrola Magic wordu a kontrolniho souctu CRC
  if (cfg->magic != CONFIG_MAGIC || cfg->crc != calculated_crc) {
    printf("[CFG] Neplatna konfigurace v EEPROM (CRC nesouhlasi). Ukladam "
           "default...\r\n");

    memset(cfg, 0, sizeof(DeviceConfig_t));
    cfg->magic = CONFIG_MAGIC;
    cfg->perioda_mereni = 15;
    cfg->perioda_odesilani = 60;
    strcpy(cfg->wifi_ssid, "IoT_Explorer_Net");
    strcpy(cfg->wifi_pass, "spseplzen123");
    strcpy(cfg->mqtt_broker, "192.168.1.100");
    cfg->mqtt_port = 1883;
    cfg->device_id = 0x00010048;
    cfg->crc = CRC16((uint8_t *)cfg, sizeof(DeviceConfig_t) - sizeof(uint16_t));

    // Zapis vychozich parametru do EEPROM
    EEPROM_Write(EEPROM_CONFIG_ADDR, cfg, sizeof(DeviceConfig_t));
  } else {
    printf("[CFG] Nactena platna konfigurace (ID: 0x%08lX, Perioda: %u s, "
           "SSID: %s)\r\n",
           cfg->device_id, cfg->perioda_mereni, cfg->wifi_ssid);
  }
}

// =====================================================================
// 3. HODINA: KRUHOVY BUFFER (RING BUFFER) V EEPROM
// =====================================================================
#define RB_MAGIC 0x52494E47 // "RING"

// Zaznam mereni: 6 B
typedef struct __attribute__((packed)) {
  uint32_t timestamp; // cas [s]
  int16_t teplota;    // teplota v setinach C
} Record_t;

// Metadata ulozena na adrese 0x000200
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t capacity;   // pocet slotu (pri zmene layoutu se buffer resetuje)
  uint32_t write_idx;  // pozice zapisu (sem pujde dalsi zaznam)
  uint32_t oldest_idx; // pozice nejstarsiho zaznamu
  uint32_t count;      // pocet platnych zaznamu v bufferu
  uint32_t unsent;     // pocet neodeslanych zaznamu (= nejnovejsich N)
  uint16_t crc;
} RingMeta_t;

typedef struct {
  uint32_t meta_addr;
  uint32_t data_addr;
  uint32_t capacity;
  RingMeta_t m;
} RingBuffer_t;

static RingBuffer_t ring;

#define RB_CAPACITY ((EEPROM_SIZE - RB_DATA_ADDR) / sizeof(Record_t)) // 87253

static uint32_t RB_SlotAddr(const RingBuffer_t *rb, uint32_t idx) {
  return rb->data_addr + idx * (uint32_t)sizeof(Record_t);
}

static bool RB_SaveMeta(RingBuffer_t *rb) {
  rb->m.crc = CRC16((uint8_t *)&rb->m, sizeof(RingMeta_t) - sizeof(uint16_t));
  return EEPROM_Write(rb->meta_addr, &rb->m, sizeof(RingMeta_t));
}

static bool RB_MetaValid(const RingBuffer_t *rb) {
  const RingMeta_t *m = &rb->m;
  if (m->magic != RB_MAGIC || m->capacity != rb->capacity)
    return false;
  if (m->crc != CRC16((const uint8_t *)m, sizeof(RingMeta_t) - 2))
    return false;
  if (m->write_idx >= rb->capacity || m->oldest_idx >= rb->capacity)
    return false;
  if (m->count > rb->capacity || m->unsent > m->count)
    return false;
  // Konzistence: oldest + count musi ukazovat na write
  return ((m->oldest_idx + m->count) % rb->capacity) == m->write_idx;
}

// Vymaze buffer (jen metadata - stara data se proste prepisou)
bool RB_Format(RingBuffer_t *rb) {
  memset(&rb->m, 0, sizeof(RingMeta_t));
  rb->m.magic = RB_MAGIC;
  rb->m.capacity = rb->capacity;
  return RB_SaveMeta(rb);
}

// Nacteni metadat po startu; pri neplatnych metadatech se buffer zalozi znovu
bool RB_Init(RingBuffer_t *rb, uint32_t meta_addr, uint32_t data_addr,
             uint32_t capacity) {
  rb->meta_addr = meta_addr;
  rb->data_addr = data_addr;
  rb->capacity = capacity;

  if (EEPROM_Read(meta_addr, &rb->m, sizeof(RingMeta_t)) && RB_MetaValid(rb)) {
    return true;
  }
  return RB_Format(rb);
}

// Pridani zaznamu. Pri plnem bufferu se prepise nejstarsi zaznam.
// Poradi zapisu: nejdriv data, pak metadata -> pri vypadku napajeni
// mezi nimi zustane buffer konzistentni (jen se ztrati posledni zaznam).
bool RB_Push(RingBuffer_t *rb, const Record_t *rec) {
  RingMeta_t *m = &rb->m;

  if (!EEPROM_Write(RB_SlotAddr(rb, m->write_idx), rec, sizeof(Record_t))) {
    return false;
  }

  m->write_idx = (m->write_idx + 1) % rb->capacity;
  if (m->count < rb->capacity) {
    m->count++;
  } else {
    // Buffer plny -> nejstarsi zaznam byl prepsan
    m->oldest_idx = (m->oldest_idx + 1) % rb->capacity;
  }
  if (m->unsent < rb->capacity) {
    m->unsent++; // pokud uz bylo vse neodeslane, prisli jsme o nejstarsi
  }
  return RB_SaveMeta(rb);
}

// Precteni i-teho nejstarsiho ulozeneho zaznamu (0 = nejstarsi)
bool RB_ReadAt(const RingBuffer_t *rb, uint32_t i, Record_t *rec) {
  if (i >= rb->m.count)
    return false;
  uint32_t idx = (rb->m.oldest_idx + i) % rb->capacity;
  return EEPROM_Read(RB_SlotAddr(rb, idx), rec, sizeof(Record_t));
}

// Precteni i-teho neodeslaneho zaznamu (0 = nejstarsi neodeslany)
bool RB_ReadUnsent(const RingBuffer_t *rb, uint32_t i, Record_t *rec) {
  if (i >= rb->m.unsent)
    return false;
  return RB_ReadAt(rb, rb->m.count - rb->m.unsent + i, rec);
}

// Oznaceni n nejstarsich neodeslanych zaznamu jako odeslanych
bool RB_MarkSent(RingBuffer_t *rb, uint32_t n) {
  if (n > rb->m.unsent)
    n = rb->m.unsent;
  rb->m.unsent -= n;
  return RB_SaveMeta(rb);
}

#if RUN_SELFTESTS
// Test preteceni na malem bufferu (kapacita 5) v oblasti scratch,
// aby se nemuselo zapisovat 87 tisic zaznamu a neprisli jsme o data.
#define RBT_CAP 5
#define RBT_CHECK(cond)                                                        \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("[RB-TEST] SELHALO: %s (radek %d)\r\n", #cond, __LINE__);         \
      return false;                                                            \
    }                                                                          \
  } while (0)

bool RB_SelfTest(void) {
  RingBuffer_t t;
  Record_t r;

  RB_Init(&t, EEPROM_SCRATCH_ADDR, EEPROM_SCRATCH_ADDR + 0x20, RBT_CAP);
  RBT_CHECK(RB_Format(&t));

  // 8 zaznamu do bufferu o kapacite 5 -> zaznamy 1..3 se prepisou
  for (uint32_t i = 1; i <= 8; i++) {
    Record_t w = {.timestamp = i, .teplota = (int16_t)(2000 + i)};
    RBT_CHECK(RB_Push(&t, &w));
  }
  RBT_CHECK(t.m.count == RBT_CAP);
  RBT_CHECK(t.m.unsent == RBT_CAP);
  RBT_CHECK(t.m.write_idx == 3 && t.m.oldest_idx == 3);

  // V bufferu musi byt zaznamy 4..8 ve spravnem poradi
  for (uint32_t i = 0; i < RBT_CAP; i++) {
    RBT_CHECK(RB_ReadAt(&t, i, &r));
    RBT_CHECK(r.timestamp == 4 + i && r.teplota == (int16_t)(2004 + i));
  }

  // Odeslani 2 zaznamu -> prvni neodeslany je ts=6
  RBT_CHECK(RB_MarkSent(&t, 2));
  RBT_CHECK(RB_ReadUnsent(&t, 0, &r) && r.timestamp == 6);

  // Dalsi zaznam prepise nejstarsi (ts=4), neodeslane: 6,7,8,9
  Record_t w = {.timestamp = 9, .teplota = 2009};
  RBT_CHECK(RB_Push(&t, &w));
  RBT_CHECK(t.m.unsent == 4);
  RBT_CHECK(RB_ReadAt(&t, 0, &r) && r.timestamp == 5);
  RBT_CHECK(RB_ReadUnsent(&t, 0, &r) && r.timestamp == 6);

  // Simulace restartu: metadata se musi spravne nacist z EEPROM
  RingBuffer_t t2;
  RB_Init(&t2, EEPROM_SCRATCH_ADDR, EEPROM_SCRATCH_ADDR + 0x20, RBT_CAP);
  RBT_CHECK(memcmp(&t.m, &t2.m, sizeof(RingMeta_t)) == 0);
  RBT_CHECK(RB_ReadAt(&t2, RBT_CAP - 1, &r) && r.timestamp == 9);

  printf("[RB-TEST] Test preteceni bufferu: OK\r\n");
  return true;
}
#endif

// SENZOR MCP9808
void MCP9808_Shutdown(bool shutdown) {
  uint8_t buf[3] = {0x01, shutdown ? 0x01 : 0x00,
                    0x00}; // Registr 0x01: Bit 8 = SHDN
  SERCOM0_I2C_Write(MCP9808_ADDR, buf, 3);
  while (SERCOM0_I2C_IsBusy())
    ;
}

int16_t MCP9808_ReadTemp(void) {
  uint8_t reg = 0x05;
  uint8_t data[2];

  SERCOM0_I2C_Write(MCP9808_ADDR, &reg, 1);
  while (SERCOM0_I2C_IsBusy())
    ;

  SERCOM0_I2C_Read(MCP9808_ADDR, data, 2);
  while (SERCOM0_I2C_IsBusy())
    ;

  uint16_t raw = (((uint16_t)data[0] << 8) | data[1]) & 0x1FFF;
  if (raw & 0x1000) {
    return (int16_t)(-((int32_t)(raw & 0x0FFF) * 100) / 16);
  }
  return (int16_t)(((int32_t)raw * 100) / 16);
}

// Callback preruseni pri dosazeni Compare hodnoty v RTC
static void RTC_AlarmCallback(RTC_TIMER32_INT_MASK intCause,
                              uintptr_t context) {
  if (intCause & RTC_TIMER32_INT_MASK_CMP0) {
    rtc_alarm_fired = true;
  }
}

int main(void) {
  SYS_Initialize(NULL);
  EEPROM_CS(false);

  // SPI bring-up EEPROM
  if (!EEPROM_SpiSetup()) {
    printf("[EEPROM] CHYBA: nelze nastavit SPI na %lu Hz\r\n",
           EEPROM_SPI_FREQ_HZ);
  }

  // RTC spustime hned, citac slouzi i jako seed pro nahodnou adresu testu
  RTC_Timer32CounterSet(0);
  RTC_Timer32Start();

#if RUN_SELFTESTS
  EEPROM_BringUpTest();
  RB_SelfTest();
#endif

  // Nacteni a overeni konfigurace z EEPROM
  DeviceConfig_t config;
  Config_Init(&config);

  // Nacteni kruhoveho bufferu
  RB_Init(&ring, RB_META_ADDR, RB_DATA_ADDR, RB_CAPACITY);
  printf("[RB] Kapacita: %lu zaznamu | ulozeno: %lu | neodeslano: %lu\r\n",
         ring.capacity, ring.m.count, ring.m.unsent);

  // Pokracovat v case od posledniho ulozeneho zaznamu
  if (ring.m.count > 0) {
    Record_t last;
    if (RB_ReadAt(&ring, ring.m.count - 1, &last)) {
      system_uptime_sec = last.timestamp;
    }
  }

  // Nastaveni RTC alarmu na periodu mereni
  uint32_t rtc_freq = RTC_Timer32FrequencyGet();
  if (rtc_freq == 0) {
    rtc_freq = 1024;
  }
  uint32_t compare_ticks = (uint32_t)config.perioda_mereni * rtc_freq;

  RTC_Timer32Stop();
  RTC_Timer32CallbackRegister(RTC_AlarmCallback, (uintptr_t)NULL);
  RTC_Timer32CompareSet(compare_ticks);
  RTC_Timer32CounterSet(0);
  RTC_Timer32InterruptEnable(RTC_TIMER32_INT_MASK_CMP0);
  RTC_Timer32Start();

  printf("[RTC] Perioda probouzeni: %u s (%lu tiku pri %lu Hz)\r\n\r\n",
         config.perioda_mereni, compare_ticks, rtc_freq);

  while (1) {
    // Probudit senzor MCP9808 z Shutdown modu
    MCP9808_Shutdown(false);

    // Kratka pauza na dokonceni prevodu teploty senzoru (~250 ms)
    for (volatile uint32_t i = 0; i < 300000; i++)
      ;

    // Zmerit teplotu
    int16_t temp = MCP9808_ReadTemp();
    system_uptime_sec += config.perioda_mereni;
    printf("[MERENI] Cas: %lu s | Teplota: %d.%02d C\r\n", system_uptime_sec,
           temp / 100, (temp < 0 ? -temp : temp) % 100);

    // Uvest MCP9808 do Shutdown modu (proud senzoru < 1 uA)
    MCP9808_Shutdown(true);

    // Ulozit mereni do kruhoveho bufferu v EEPROM
    Record_t rec = {.timestamp = system_uptime_sec, .teplota = temp};
    bool was_full = (ring.m.count == ring.capacity);
    if (RB_Push(&ring, &rec)) {
      printf("[RB] Ulozeno (zapis=%lu, nejstarsi=%lu, neodeslano=%lu)%s\r\n",
             ring.m.write_idx, ring.m.oldest_idx, ring.m.unsent,
             was_full ? " - prepsan nejstarsi zaznam" : "");
    } else {
      printf("[RB] CHYBA zapisu do EEPROM!\r\n");
    }

    printf("[SPANEK] Prechod do Standby rezimu...\r\n");
    while (SERCOM1_USART_TransmitComplete() == false)
      ;

    rtc_alarm_fired = false;
    PM_StandbyModeEnter();

    printf("[WAKEUP] MCU probuzeno RTC alarmem!\r\n");
  }
}