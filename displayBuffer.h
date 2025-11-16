#pragma once
#include <U8g2lib.h>
#include <cstdint>

class DisplayDriver;

enum Orientation {
  VERTICAL = 0,  // Default: vertical orientation (SSD1306 native)
  HORIZONTAL = 1 // Horizontal orientation (icon data format) - Not implemented
};

class DisplayBuffer {
public:
  const int width, height, bitsPerPixel;
  const int bufferWidth, bufferSize;
  uint8_t *current, *previous;
  bool *dirtyRegions;
  Orientation orientation;
  U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C u8g2;

  DisplayBuffer(int w = 128, int h = 64, int bpp = 1,
                Orientation orient = VERTICAL)
      : width(w), height(h), bitsPerPixel(bpp), orientation(orient),
        bufferWidth(w), bufferSize(w * h / (8 / bpp)),
        u8g2(U8G2_R0, U8X8_PIN_NONE) {
    current = new uint8_t[bufferSize];
    previous = new uint8_t[bufferSize];
    dirtyRegions = new bool[h / (8 / bpp)];
  }

  void initText() {
    u8g2.begin();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.enableUTF8Print();
  }

  void setPixel(int x, int y, bool on);
  void draw8x8Icon(int x, int y,
                   const uint8_t *verticalIcon); // 8x8 vertical icon only
  void drawText(int x, int y, const char *text);
  void orByteColumn(int x, int page, uint8_t data);
  void clear();

  // Utility function: transform horizontal icon to vertical format
  static void transform8x8Icon(const uint8_t *horizontalIcon,
                               uint8_t *verticalIcon);
  void swap();
  void updateDisplay(DisplayDriver &driver);
  bool *getDirtyRegions() { return dirtyRegions; }
  uint8_t *getBuffer() { return current; }
  int getWidth() const { return width; }
  int getHeight() const { return height; }
};