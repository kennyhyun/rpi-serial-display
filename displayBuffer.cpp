#include "displayBuffer.h"
#include "driver.h"
#include <Arduino.h>

void DisplayBuffer::setPixel(int x, int y, bool on) {
  if (x < 0 || x >= width || y < 0 || y >= getHeight())
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
  // Allow partial drawing - only skip if completely outside
  if (x + 8 <= 0 || y + 8 <= 0 || x >= width || y >= getHeight())
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
      if (startPage + 1 < (getHeight() + 7) / 8) {
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

void DisplayBuffer::orByteColumn(int x, int page, uint8_t data) {
  if (x < 0 || x >= width || page < 0 || page >= (height + linesToSkip + 7) / 8)
    return;

  int byteIndex = x + page * width;
  current[byteIndex] |= data;
}

void DisplayBuffer::mergeBufferRegion(uint8_t *srcBuffer, int startPage,
                                      int endPage, int startCol, int endCol) {
  for (int page = startPage; page < endPage && page < height / 8; page++) {
    for (int col = startCol; col < endCol && col < width; col++) {
      int idx = page * width + col;
      current[idx] |= srcBuffer[idx];
    }
  }
}

void DisplayBuffer::drawBorder() {
  int bufferHeight = getHeight();
  // Draw top and bottom borders
  for (int x = 0; x < width; x++) {
    setPixel(x, 0, true);                // Top border
    setPixel(x, bufferHeight - 1, true); // Bottom border (use physical height)
  }

  // Draw left and right borders
  for (int y = 0; y < bufferHeight; y += 2) {
    setPixel(0, y, true);         // Left border
    setPixel(width - 1, y, true); // Right border
  }
}

void DisplayBuffer::drawTestPattern() {
  int bufferHeight = getHeight();

  // Draw top and bottom borders
  for (int x = 0; x < width; x++) {
    setPixel(x, 0, true);                // Top border
    setPixel(x, bufferHeight - 1, true); // Bottom border (use physical height)
  }

  // Draw left and right borders
  for (int y = 0; y < bufferHeight; y += 2) {
    setPixel(0 + (y / 10), y, true);         // Left border
    setPixel(width - 1 - (y / 10), y, true); // Right border
  }
}

void DisplayBuffer::clear() { memset(current, 0, bufferSize); }

void DisplayBuffer::swap() {
  uint8_t *temp = current;
  current = previous;
  previous = temp;
}

void DisplayBuffer::updateDisplay(DisplayDriver &driver) {
  int numPages = (getHeight() + 7) / 8;
  for (int page = 0; page < numPages; page++) {
    for (int chunkStart = 0; chunkStart < width; chunkStart += chunkSize) {
      bool chunk_changed = false;
      int chunk_end = min(chunkStart + chunkSize, width);

      for (int col = chunkStart; col < chunk_end; col++) {
        int idx = page * width + col;

        if (linesToSkip > 0 && page >= pageToSkip) {
          uint16_t curr_16 = (uint16_t)current[idx];
          uint16_t prev_16 = (uint16_t)previous[idx];

          if (page < numPages - 1) {
            int next_idx = (page + 1) * width + col;
            curr_16 |= (uint16_t)current[next_idx] << 8;
            prev_16 |= (uint16_t)previous[next_idx] << 8;
          }

          uint16_t mask = 0x00FF << linesToSkip;
          curr_16 &= mask;
          prev_16 &= mask;

          if (curr_16 != prev_16) {
            chunk_changed = true;
            break;
          }
        } else {
          if (current[idx] != previous[idx]) {
            chunk_changed = true;
            break;
          }
        }
      }

      if (chunk_changed) {
        driver.setPosition(page, chunkStart);

        for (int col = chunkStart; col < chunk_end; col++) {
          int idx = page * width + col;

          if (linesToSkip > 0 && page >= pageToSkip) {
            uint16_t data_16 = (uint16_t)current[idx];
            if (page < numPages - 1) {
              int next_idx = (page + 1) * width + col;
              data_16 |= (uint16_t)current[next_idx] << 8;
            }
            chunkData[col - chunkStart] = data_16 >> linesToSkip;
          } else {
            chunkData[col - chunkStart] = current[idx];
          }
        }

        driver.writeData(chunkData, chunk_end - chunkStart);
      }
    }
  }
  swap();
}