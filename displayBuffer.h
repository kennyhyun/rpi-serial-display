#pragma once
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
  uint8_t *current, *previous, *chunkData;
  bool *dirtyRegions;
  Orientation orientation;
  int linesToSkip, pageToSkip, chunkSize;

  DisplayBuffer(int w = 128, int h = 64, int bpp = 1,
                Orientation orient = VERTICAL, int skipLines = 0,
                int skipFromPage = 0, int chunkSz = 16)
      : width(w), height(h), bitsPerPixel(bpp), orientation(orient),
        bufferWidth(w), bufferSize(w * ((h + skipLines + 7) / 8)),
        linesToSkip(skipLines), pageToSkip(skipFromPage), chunkSize(chunkSz) {
    current = new uint8_t[bufferSize];
    previous = new uint8_t[bufferSize];
    chunkData = new uint8_t[chunkSize];
    dirtyRegions = new bool[(h + skipLines + 7) / 8];
  }

  ~DisplayBuffer() {
    delete[] current;
    delete[] previous;
    delete[] chunkData;
    delete[] dirtyRegions;
  }

  void setPixel(int x, int y, bool on);
  void draw8x8Icon(int x, int y,
                   const uint8_t *verticalIcon); // 8x8 vertical icon only

  void orByteColumn(int x, int page, uint8_t data);
  void mergeBufferRegion(uint8_t *srcBuffer, int startPage, int endPage,
                         int startCol, int endCol);
  void drawBorder();
  void drawTestPattern();
  void clear();

  // Utility function: transform horizontal icon to vertical format
  static void transform8x8Icon(const uint8_t *horizontalIcon,
                               uint8_t *verticalIcon);
  void swap();
  void updateDisplay(DisplayDriver &driver);
  bool *getDirtyRegions() { return dirtyRegions; }
  uint8_t *getBuffer() { return current; }
  int getWidth() const { return width; }
  int getHeight() const { return height + linesToSkip; }
};