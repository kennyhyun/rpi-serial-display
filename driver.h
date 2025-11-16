#pragma once
#include <cstdint>

class DisplayDriver {
public:
  virtual void updateRegion(int page, int startCol, int endCol,
                            uint8_t *data) = 0;
  virtual void setPosition(int page, int col) = 0;
  virtual void writeData(uint8_t *data, int length) = 0;
  virtual void init() = 0;
};

class SSD1306Driver : public DisplayDriver {
public:
  void updateRegion(int page, int startCol, int endCol, uint8_t *data) override;
  void setPosition(int page, int col) override;
  void writeData(uint8_t *data, int length) override;
  void init() override;
};