#pragma once
#include <Adafruit_GFX.h>
#include <cstdint>

// Separate text rendering buffer with dynamic allocation
class TextBuffer : public Adafruit_GFX {
public:
  uint8_t *buffer;
  int bufferSize;
  int allocatedSize;

  TextBuffer()
      : Adafruit_GFX(0, 0), buffer(nullptr), bufferSize(0), allocatedSize(0) {}

  ~TextBuffer() {
    if (buffer)
      delete[] buffer;
  }

  void resize(int w, int h) {
    int newSize = w * h / 8;

    if (newSize > allocatedSize) {
      // Need larger buffer - reallocate
      if (buffer)
        delete[] buffer;
      buffer = new uint8_t[newSize];
      allocatedSize = newSize;
    }

    // Update dimensions and buffer size using Adafruit_GFX constructor
    _width = w;
    _height = h;
    bufferSize = newSize;
    clear();
  }

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || x >= _width || y < 0 || y >= _height || !buffer)
      return;

    int byteIndex = x + (y / 8) * _width;
    int bitIndex = y % 8;

    if (color > 0) {
      buffer[byteIndex] |= (1 << bitIndex);
    } else {
      buffer[byteIndex] &= ~(1 << bitIndex);
    }
  }

  void clear() {
    if (buffer)
      memset(buffer, 0, bufferSize);
  }

  int getWidth() const { return _width; }
  int getHeight() const { return _height; }
};

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
        bufferWidth(w), bufferSize(w * h / (8 / bpp)), linesToSkip(skipLines),
        pageToSkip(skipFromPage), chunkSize(chunkSz) {
    current = new uint8_t[bufferSize];
    previous = new uint8_t[bufferSize];
    chunkData = new uint8_t[chunkSize];
    dirtyRegions = new bool[h / (8 / bpp)];
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
  void mergeTextBuffer(const TextBuffer &textBuf, int destX,
                       int destY); // Merge text at specific position
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