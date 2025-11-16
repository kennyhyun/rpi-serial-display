#include <U8g2lib.h>
#include <Wire.h>
#include <hardware/adc.h>

#include "displayBuffer.h"
#include "driver.h"
#include "logging.h"
#include "movable.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define LINES_PER_PAGE 8
#define SSD1306_I2C_ADDR 0x3C
#define COLUMN_SIZE 16 // 한 번에 전송할 컬럼 수
// #define COLUMN_SIZE 128 // 한 번에 전송할 컬럼 수

#define NUM_POINTS 4 // 동시에 움직일 점 개수

// Heart icon data (horizontal format)
const uint8_t heartIcon_horizontal[8] = {0b01100110, 0b11111111, 0b11111111,
                                         0b11111111, 0b01111110, 0b00111100,
                                         0b00011000, 0b00000000};
uint8_t heartIcon_vertical[8];

Movable *objects[NUM_POINTS + 1];
DisplayBuffer displayBuffer;
SSD1306Driver driver;
Logger logger;

float readVoltage(int pin) {
  int rawValue = analogRead(pin);
  return (rawValue * 3.3) / 4095.0;
}

unsigned long getRandomValue() {
  float tempC = analogReadTemp();

  char msg[64];
  snprintf(msg, sizeof(msg), "Internal Temp: %.2f(C)", tempC);
  logger.addLog(msg);

  // 외부 ADC 읽기 (전압으로 변환)
  float voltA0 = readVoltage(A0);
  float voltA1 = readVoltage(A1);
  float voltA2 = readVoltage(A2);
  float voltA3 = readVoltage(A3);

  snprintf(msg, sizeof(msg), "A0:%.2fV A1:%.2fV A2:%.2fV A3:%.2fV", voltA0,
           voltA1, voltA2, voltA3);
  logger.addLog(msg);

  int valA0 = analogRead(A0);
  int valA1 = analogRead(A1);
  int valA2 = analogRead(A2);
  int valA3 = analogRead(A3);

  unsigned long val = tempC + valA0 + valA1 + valA2 + valA3;
  snprintf(msg, sizeof(msg), "Sum: %lu", val);
  logger.addLog(msg);

  // 시간 기반 엔트로피 추가
  unsigned long time_entropy = micros();
  snprintf(msg, sizeof(msg), "Time entropy: %lu", time_entropy);
  logger.addLog(msg);

  return (unsigned long)val + time_entropy;
}

void i2c_reset() {
  logger.addLog("Initializing I2C...");
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(1000000); // 1MHz
  delay(10);
}

void ssd1306_init() {
  logger.addLog("Initializing SSD1306...");

  uint8_t init_cmds[] = {
      0xAE,       // Display OFF
      0xD5, 0x80, // Set display clock divide ratio
      0xA8, 0x3F, // Set multiplex ratio (64)
      0xD3, 0x00, // Set display offset
      0x40,       // Set start line
      0x8D, 0x14, // Charge pump setting (enable)
      0x20, 0x00, // Memory addressing mode (horizontal)
      0xA1,       // Set segment re-map
      0xC8,       // Set COM output scan direction
      0xDA, 0x12, // Set COM pins hardware configuration
      0x81, 0xCF, // Set contrast control
      0xD9, 0xF1, // Set pre-charge period
      0xDB, 0x40, // Set VCOMH deselect level
      0xA4,       // Entire display ON (resume to RAM content)
      0xA6,       // Set normal display
      0x2E,       // Deactivate scroll
      0xAF        // Display ON
  };

  for (int i = 0; i < sizeof(init_cmds); i++) {
    Wire.beginTransmission(SSD1306_I2C_ADDR);
    Wire.write(0x00); // Command mode
    Wire.write(init_cmds[i]);
    byte error = Wire.endTransmission();

    if (error != 0) {
      char msg[64];
      snprintf(msg, sizeof(msg), "I2C error at cmd %d: %d", i, error);
      logger.addLog(msg);

      // 에러 시 재시도
      if (error == 5) { // timeout
        logger.addLog("Timeout detected, resetting I2C...");
        i2c_reset();
        delay(10);
        // 재시도
        Wire.beginTransmission(SSD1306_I2C_ADDR);
        Wire.write(0x00);
        Wire.write(init_cmds[i]);
        error = Wire.endTransmission();
        if (error == 0) {
          logger.addLog("Retry successful");
        }
      }
    }
  }

  logger.addLog("SSD1306 initialization complete");
}

void setup() {
  Serial.begin(115200);

  adc_init();
  analogReadResolution(12);

  i2c_reset();

  logger.addLog("Starting OLED bouncing icon...");

  // Transform heart icon to vertical format
  DisplayBuffer::transform8x8Icon(heartIcon_horizontal, heartIcon_vertical);

  randomSeed(getRandomValue());

  ssd1306_init();

  // Initialize text system after I2C is ready
  displayBuffer.initText();

  // 초기화 완료 후 고속 모드로 전환
  Wire.setClock(1000000); // 1MHz (최고 속도)
  logger.addLog("I2C speed set to 1MHz");

  // 객체들 초기화
  for (int i = 0; i < NUM_POINTS; i++) {
    int x = random(0, SCREEN_WIDTH);
    int y = random(0, SCREEN_HEIGHT);
    int vx = random(-3, 4);
    int vy = random(-3, 4);
    if (vx == 0)
      vx = 1;
    if (vy == 0)
      vy = 1;
    objects[i] = new BouncingPoint(x, y, vx, vy);
  }
  // Create FallingIcon with actual size (8x7) and movementScale 100
  FallingIcon *heart =
      new FallingIcon(SCREEN_WIDTH / 2 - 4, 10, heartIcon_vertical, 8, 7, 100);
  heart->vx = random(-20, 21);
  heart->vy = 0.0; // Ensure vy starts at 0

  char heartMsg[64];
  snprintf(heartMsg, sizeof(heartMsg), "Heart: pos(%.1f,%.1f) vel(%.1f,%.1f)",
           heart->x, heart->y, heart->vx, heart->vy);
  logger.addLog(heartMsg);

  objects[NUM_POINTS] = heart;

  displayBuffer.clear();

  delay(100);
}

unsigned long timestamp = millis();
unsigned long temp_debug_timer = 0;
bool allowSerial = false;
int frameCount = 0;
float currentTemp = 0;
float currentFPS = 0;

void loop() {
  displayBuffer.clear();
  int dt = millis() - timestamp; // ms 단위
  timestamp = millis();
  frameCount++;

  if (millis() - temp_debug_timer >= 3000) {
    float cpuTemp = analogReadTemp();
    float fps = frameCount / 3.0;
    char tempMsg[64];
    snprintf(tempMsg, sizeof(tempMsg), "CPU: %.2f(C), FPS: %.1f", cpuTemp, fps);
    logger.addLog(tempMsg);
    temp_debug_timer = millis();
    frameCount = 0;

    currentTemp = cpuTemp;
    currentFPS = fps;
  }

  // 모든 객체 업데이트 및 그리기
  for (int i = 0; i <= NUM_POINTS; i++) {
    objects[i]->update(timestamp, displayBuffer);
    objects[i]->draw(displayBuffer);
  }

  // OLED에 온도와 FPS 표시 (통합된 displayBuffer 시스템 사용)
  char tempStr[16], fpsStr[16];
  snprintf(tempStr, sizeof(tempStr), "%.1f°C", currentTemp);
  snprintf(fpsStr, sizeof(fpsStr), "%.1f FPS", currentFPS);
  displayBuffer.drawText(0, -1, tempStr);
  displayBuffer.drawText(0, 8, fpsStr);

  if (timestamp > 1000 || allowSerial) {
    // 저장된 로그 출력 (시리얼 버퍼에 여유가 있을 때)
    logger.flushLogs();
    allowSerial = true;
  }

  displayBuffer.updateDisplay(driver);

  delay(1000 / 60);
}
