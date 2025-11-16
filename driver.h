#pragma once
#include <cstdint>

class DisplayDriver {
public:
  virtual void updateRegion(int page, int startCol, int endCol,
                            uint8_t *data) = 0;
  virtual void init() = 0;
};

class SSD1306Driver : public DisplayDriver {
public:
  void updateRegion(int page, int startCol, int endCol, uint8_t *data) override;
  void init() override;
};