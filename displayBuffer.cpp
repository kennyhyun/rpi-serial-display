#include "displayBuffer.h"
#include "driver.h"
#include <Arduino.h>
#include <U8g2lib.h>

void DisplayBuffer::setPixel(int x, int y, bool on) {
  if (x < 0 || x >= width || y < 0 || y >= height)
    return;

  int byteIndex = x + (y / (8 / bitsPerPixel)) * width;
  int bitIndex = y % (8 / bitsPerPixel);

  if (on) {
    current[byteIndex] |= (1 << bitIndex);
  } else {
    current[byteIndex] &= ~(1 << bitIndex);
  }
}

// 8x8 vertical icon only - direct byte manipulation
void DisplayBuffer::draw8x8Icon(int x, int y, const uint8_t *verticalIcon) {
  if (x < 0 || y < 0 || x + 8 > width || y + 8 > height)
    return;

  int startPage = y / 8;
  int yOffset = y % 8;

  if (yOffset == 0) {
    // Aligned to page boundary: direct byte OR
    for (int col = 0; col < 8; col++) {
      orByteColumn(x + col, startPage, verticalIcon[col]);
    }
  } else {
    // Crosses page boundary: split into 2 pages
    for (int col = 0; col < 8; col++) {
      uint8_t iconByte = verticalIcon[col];

      // Upper page
      orByteColumn(x + col, startPage, iconByte << yOffset);

      // Lower page
      if (startPage + 1 < height / 8) {
        orByteColumn(x + col, startPage + 1, iconByte >> (8 - yOffset));
      }
    }
  }
}

// Utility: transform horizontal 8x8 icon to vertical format
void DisplayBuffer::transform8x8Icon(const uint8_t *horizontalIcon,
                                     uint8_t *verticalIcon) {
  for (int col = 0; col < 8; col++) {
    uint8_t verticalByte = 0;
    for (int row = 0; row < 8; row++) {
      if (horizontalIcon[row] & (1 << (7 - col))) {
        verticalByte |= (1 << row);
      }
    }
    verticalIcon[col] = verticalByte;
  }
}

void DisplayBuffer::drawText(int x, int y, const char *text) {
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.setCursor(x, y + 8);
  u8g2.print(text);

  // Copy u8g2 buffer to our buffer (u8g2 won't send to I2C)
  uint8_t *u8g2_buffer = u8g2.getBufferPtr();
  for (int i = 0; i < width * height / 8; i++) {
    current[i] |= u8g2_buffer[i];
  }
}

void DisplayBuffer::orByteColumn(int x, int page, uint8_t data) {
  if (x < 0 || x >= width || page < 0 || page >= height / 8)
    return;

  int byteIndex = x + page * width;
  current[byteIndex] |= data;
}

void DisplayBuffer::clear() { memset(current, 0, bufferSize); }

void DisplayBuffer::swap() {
  uint8_t *temp = current;
  current = previous;
  previous = temp;
}

void DisplayBuffer::updateDisplay(DisplayDriver &driver) {
  memset(dirtyRegions, false, height / (8 / bitsPerPixel));

  for (int page = 0; page < height / (8 / bitsPerPixel); page++) {
    int pageStart = page * width;
    for (int col = 0; col < width; col++) {
      if (current[pageStart + col] != previous[pageStart + col]) {
        dirtyRegions[page] = true;
        break;
      }
    }
  }

  for (int page = 0; page < height / (8 / bitsPerPixel); page++) {
    if (dirtyRegions[page]) {
      for (int startCol = 0; startCol < width; startCol += 16) {
        int endCol = min(startCol + 16, width);
        driver.updateRegion(page, startCol, endCol,
                            &current[page * width + startCol]);
      }
    }
  }

  swap();
}