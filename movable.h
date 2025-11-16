#pragma once
#include "displayBuffer.h"
#include <cstdint>

class Movable {
public:
  float x, y, vx, vy;
  int movementScale;
  unsigned long lastUpdateTime;

  Movable(float posX = 0, float posY = 0, float velX = 0, float velY = 0,
          int scale = 16)
      : x(posX), y(posY), vx(velX), vy(velY), movementScale(scale),
        lastUpdateTime(0) {}
  virtual void update(unsigned long currentTime,
                      const DisplayBuffer &buffer) = 0;
  virtual void draw(DisplayBuffer &buffer) = 0;
};

class BouncingPoint : public Movable {
public:
  BouncingPoint(float posX = 0, float posY = 0, float velX = 1, float velY = 1,
                int scale = 16);
  void update(unsigned long currentTime, const DisplayBuffer &buffer) override;
  void draw(DisplayBuffer &buffer) override;
};

class FallingIcon : public Movable {
public:
  const uint8_t *iconData;
  const float gravity = 9.8; // Standard gravity acceleration
  float initialEnergy;       // Track initial energy for restoration
  int iconWidth, iconHeight; // Actual icon dimensions

  FallingIcon(float posX, float posY, const uint8_t *icon, int w = 8, int h = 8,
              int scale = 1000);
  void update(unsigned long currentTime, const DisplayBuffer &buffer) override;
  void draw(DisplayBuffer &buffer) override;
};
