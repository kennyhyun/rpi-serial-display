#include "driver.h"
#include <Wire.h>

#define SSD1306_I2C_ADDR 0x3C

void SSD1306Driver::updateRegion(int page, int startCol, int endCol,
                                 uint8_t *data) {
  Wire.beginTransmission(SSD1306_I2C_ADDR);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.write(startCol);
  Wire.write(endCol - 1);
  Wire.endTransmission();

  Wire.beginTransmission(SSD1306_I2C_ADDR);
  Wire.write(0x00);
  Wire.write(0x22);
  Wire.write(page);
  Wire.write(page);
  Wire.endTransmission();

  Wire.beginTransmission(SSD1306_I2C_ADDR);
  Wire.write(0x40);
  for (int i = 0; i < (endCol - startCol); i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

void SSD1306Driver::init() {
  // Implementation in main file
}