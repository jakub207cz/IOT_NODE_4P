#include "definitions.h"
#include <stdint.h>
#include <stdio.h>


int main(void) {
  SYS_Initialize(NULL);

  uint8_t reg;
  uint8_t data[2];

  // 1. Vyčtení ID (registr 0x07) z adresy 0x1C
  reg = 0x07;
  SERCOM0_I2C_Write(0x1C, &reg, 1);
  while (SERCOM0_I2C_IsBusy())
    ;

  SERCOM0_I2C_Read(0x1C, data, 2);
  while (SERCOM0_I2C_IsBusy())
    ;

  printf("Dev ID: 0x%02X%02X\r\n", data[0], data[1]);

  // 2. Periodické čtení teploty (registr 0x05)
  while (1) {
    reg = 0x05;
    SERCOM0_I2C_Write(0x1C, &reg, 1);
    while (SERCOM0_I2C_IsBusy())
      ;

    SERCOM0_I2C_Read(0x1C, data, 2);
    while (SERCOM0_I2C_IsBusy())
      ;

    // Spojení 2 bajtů a oříznutí příznaků (bity 15-13)
    uint16_t raw = (((uint16_t)data[0] << 8) | data[1]) & 0x1FFF;

    // Převod na setiny °C: raw * 100 / 16 (1 LSB = 0.0625 °C)
    int32_t setiny;
    if (raw & 0x1000) { // Záporná teplota (bit 12)
      setiny = -((int32_t)(raw & 0x0FFF) * 100) / 16;
    } else {
      setiny = ((int32_t)raw * 100) / 16;
    }

    printf("Teplota: %ld.%02ld C\r\n", setiny / 100,
           (setiny < 0 ? -setiny : setiny) % 100);

    for (volatile uint32_t i = 0; i < 2000000; i++)
      ;
  }
}
