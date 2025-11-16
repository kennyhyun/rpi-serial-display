#include "movable.h"
#include <Arduino.h>

BouncingPoint::BouncingPoint(float posX, float posY, float velX, float velY,
                             int scale)
    : Movable(posX, posY, velX, velY, scale) {}

void BouncingPoint::update(unsigned long currentTime,
                           const DisplayBuffer &buffer) {
  if (lastUpdateTime == 0)
    lastUpdateTime = currentTime;

  int dt = currentTime - lastUpdateTime;
  lastUpdateTime = currentTime;

  x += vx * dt / movementScale;
  y += vy * dt / movementScale;

  if (x <= 0 || x >= buffer.getWidth() - 1)
    vx = -vx;
  if (y <= 0 || y >= buffer.getHeight() - 1)
    vy = -vy;

  x = constrain(x, 0.0f, (float)(buffer.getWidth() - 1));
  y = constrain(y, 0.0f, (float)(buffer.getHeight() - 1));
}

void BouncingPoint::draw(DisplayBuffer &buffer) { buffer.setPixel(x, y, true); }

FallingIcon::FallingIcon(float posX, float posY, const uint8_t *icon, int w,
                         int h, int scale)
    : Movable(posX, posY, 0.0, 0.0, scale), iconData(icon), initialEnergy(0.0),
      iconWidth(w), iconHeight(h) {}

void FallingIcon::update(unsigned long currentTime,
                         const DisplayBuffer &buffer) {
  if (lastUpdateTime == 0)
    lastUpdateTime = currentTime;

  int dt = currentTime - lastUpdateTime;
  lastUpdateTime = currentTime;

  // Apply gravity
  vy += gravity * dt / (float)movementScale;

  // Update position
  x += vx * dt / (float)movementScale;
  y += vy * dt / (float)movementScale;

  // Bounce off walls (perfect elastic collision)
  if (x <= 0) {
    vx = -vx;
    x = 0.0f;
  } else if (x >= buffer.getWidth() - iconWidth) {
    vx = -vx;
    x = (float)(buffer.getWidth() - iconWidth);
  }

  // Only bounce off bottom (ground)
  if (y >= buffer.getHeight() - iconHeight) {
    vy = -vy;
    y = (float)(buffer.getHeight() - iconHeight);

    // Energy restoration: store and restore initial bounce velocity
    if (initialEnergy == 0.0) {
      initialEnergy = abs(vy); // Store initial bounce velocity
    } else {
      vy = vy > 0 ? initialEnergy : -initialEnergy; // Restore energy
    }
  }
}

void FallingIcon::draw(DisplayBuffer &buffer) {
  // Only draw if icon is visible on screen
  if (y >= -iconHeight && y < buffer.getHeight() && x >= -iconWidth &&
      x < buffer.getWidth()) {
    buffer.draw8x8Icon((int)x, (int)y, iconData);
  }
}