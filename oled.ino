#include <U8g2lib.h>
#include <Wire.h>
#include <hardware/adc.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define LINES_PER_PAGE 8
#define SSD1306_I2C_ADDR 0x3C
#define COLUMN_SIZE 16 // 한 번에 전송할 컬럼 수
// #define COLUMN_SIZE 128 // 한 번에 전송할 컬럼 수

#define NUM_POINTS 4 // 동시에 움직일 점 개수

// 8x8 아이콘 배열 (하트 모양)
const uint8_t icon8x8[8] = {0b01100110, 0b11111111, 0b11111111, 0b11111111,
                            0b01111110, 0b00111100, 0b00011000, 0b00000000};

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

class FrameBuffer {
public:
  uint8_t *buffer1;
  uint8_t *buffer2;
  uint8_t *current;
  uint8_t *previous;
  int bufferHeight;
  int pageToSkip;
  int linesToSkip;

  FrameBuffer(int skipLines = 0, int skipPage = 0) {
    linesToSkip = skipLines;
    pageToSkip = skipPage;
    if (linesToSkip > 0) {
      bufferHeight =
          SCREEN_HEIGHT + (LINES_PER_PAGE / linesToSkip) * LINES_PER_PAGE;
    } else {
      bufferHeight = SCREEN_HEIGHT;
    }

    int bufferSize = SCREEN_WIDTH * bufferHeight / LINES_PER_PAGE;
    buffer1 = new uint8_t[bufferSize];
    buffer2 = new uint8_t[bufferSize];
    current = buffer1;
    previous = buffer2;
  }

  ~FrameBuffer() {
    delete[] buffer1;
    delete[] buffer2;
  }

  void swap() {
    uint8_t *temp = current;
    current = previous;
    previous = temp;
  }

  void clear(bool clearCurrent = true, uint8_t clearValue = 0x00) {
    uint8_t *target = clearCurrent ? current : previous;
    for (int i = 0; i < SCREEN_WIDTH * bufferHeight / LINES_PER_PAGE; i++) {
      target[i] = clearValue;
    }
  }

  void setPixel(int x, int y, bool on) {
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= bufferHeight)
      return;
    int page = y / LINES_PER_PAGE;
    int bit = y % LINES_PER_PAGE;
    if (on)
      current[page * SCREEN_WIDTH + x] |= (1 << bit);
    else
      current[page * SCREEN_WIDTH + x] &= ~(1 << bit);
  }

  void drawIcon(int x, int y, const uint8_t *icon) {
    for (int row = 0; row < 8; row++) {
      uint8_t line = icon[row];
      for (int col = 0; col < 8; col++) {
        if (line & (0x80 >> col)) {
          setPixel(x + col, y + row, true);
        }
      }
    }
  }

  void drawText(int x, int y, const char *text) {
    // U8g2 버퍼에 텍스트 그리기
    u8g2.setDrawColor(1);
    u8g2.setCursor(x, y + 8); // U8g2는 baseline 기준
    u8g2.print(text);

    // U8g2 버퍼를 우리 버퍼로 복사
    uint8_t *u8g2_buffer = u8g2.getBufferPtr();
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT / LINES_PER_PAGE; i++) {
      current[i] |= u8g2_buffer[i];
    }
  }

  void updateDisplay() {
    for (int page = 0; page < SCREEN_HEIGHT / LINES_PER_PAGE; page++) {
      for (int chunk = 0; chunk < SCREEN_WIDTH; chunk += COLUMN_SIZE) {
        bool chunk_changed = false;
        int chunk_end = min(chunk + COLUMN_SIZE, SCREEN_WIDTH);

        for (int col = chunk; col < chunk_end; col++) {
          int idx = page * SCREEN_WIDTH + col;

          if (linesToSkip > 0 && page >= pageToSkip) {
            // 16비트 데이터 구성 (LSB가 상단)
            uint16_t curr_16 = (uint16_t)current[idx];
            uint16_t prev_16 = (uint16_t)previous[idx];

            if (page < SCREEN_HEIGHT / LINES_PER_PAGE - 1) {
              int next_idx = (page + 1) * SCREEN_WIDTH + col;
              curr_16 |= (uint16_t)current[next_idx] << LINES_PER_PAGE;
              prev_16 |= (uint16_t)previous[next_idx] << LINES_PER_PAGE;
            }

            // 마스크: 하위 linesToSkip 비트 제거
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
          Wire.beginTransmission(SSD1306_I2C_ADDR);
          Wire.write(0x00);
          Wire.write(0xB0 + page);
          Wire.write(chunk & 0x0F);
          Wire.write(0x10 | (chunk >> 4));
          Wire.endTransmission();

          Wire.beginTransmission(SSD1306_I2C_ADDR);
          Wire.write(0x40);
          for (int col = chunk; col < chunk_end; col++) {
            int idx = page * SCREEN_WIDTH + col;
            uint8_t data;

            if (linesToSkip > 0 && page >= pageToSkip) {
              // 16비트 데이터 구성 (LSB가 상단)
              uint16_t data_16 = (uint16_t)current[idx];
              if (page < SCREEN_HEIGHT / LINES_PER_PAGE - 1) {
                int next_idx = (page + 1) * SCREEN_WIDTH + col;
                data_16 |= (uint16_t)current[next_idx] << LINES_PER_PAGE;
              }

              // linesToSkip만큼 시프트하여 라인 제거
              data = data_16 >> linesToSkip;
            } else {
              data = current[idx];
            }

            Wire.write(data);
            if ((col - chunk + 1) % 32 == 0) {
              Wire.endTransmission();
              Wire.beginTransmission(SSD1306_I2C_ADDR);
              Wire.write(0x40);
            }
          }
          Wire.endTransmission();
        }
      }
    }
    swap();
  }
};

FrameBuffer frameBuffer(2, 2); // Skip 2 lines, start from page 2

class Point {
public:
  int x, y;
  int vx, vy;

  Point(int posX = 0, int posY = 0, int velX = 1, int velY = 1) {
    x = posX;
    y = posY;
    vx = velX;
    vy = velY;
  }

  void update(int dt) {
    int dx = vx * dt / 16;
    int dy = vy * dt / 16;
    x += dx;
    y += dy;

    if (x < 0) {
      x = 0;
      vx = -vx;
    }
    if (x >= SCREEN_WIDTH) {
      x = SCREEN_WIDTH - 1;
      vx = -vx;
    }
    if (y < 0) {
      y = 0;
      vy = -vy;
    }
    if (y >= SCREEN_HEIGHT) {
      y = SCREEN_HEIGHT - 1;
      vy = -vy;
    }
  }
};

class Icon {
public:
  float x, y;
  float vx, vy;
  const uint8_t *iconData;

  Icon(float posX = 60, float posY = 0) {
    x = posX;
    y = posY;
    vx = 0;
    vy = -30;
    iconData = icon8x8;
  }

  void update(float dt) {
    vy += 9.8 * dt;
    x += vx * dt;
    y += vy * dt;

    if (x < 0) {
      x = 0;
      vx = -vx;
    }
    if (x > SCREEN_WIDTH - 8) {
      x = SCREEN_WIDTH - 8;
      vx = -vx;
    }
    float bottomLimit = SCREEN_HEIGHT - 2 - 8;
    if (y >= bottomLimit) {
      y = bottomLimit;
      vy = -vy;
    }
    if (y < 0) {
      y = 0;
      vy = -vy;
    }
  }
};

Point *points[NUM_POINTS];
Icon bouncingIcon;

// 로그 버퍼링 시스템
#define LOG_BUFFER_SIZE 10
struct LogEntry {
  unsigned long timestamp;
  char message[64];
};

LogEntry logBuffer[LOG_BUFFER_SIZE];
int logWriteIndex = 0;
int logReadIndex = 0;
int logCount = 0;

void addLog(const char *msg) {
  if (logCount < LOG_BUFFER_SIZE) {
    logBuffer[logWriteIndex].timestamp = millis();
    strncpy(logBuffer[logWriteIndex].message, msg, 63);
    logBuffer[logWriteIndex].message[63] = '\0';
    logWriteIndex = (logWriteIndex + 1) % LOG_BUFFER_SIZE;
    logCount++;
  }
}

void flushLogs() {
  while (logCount > 0 && Serial.availableForWrite() > 64) {
    Serial.print("[");
    Serial.print(logBuffer[logReadIndex].timestamp);
    Serial.print("] ");
    Serial.println(logBuffer[logReadIndex].message);

    logReadIndex = (logReadIndex + 1) % LOG_BUFFER_SIZE;
    logCount--;
  }
}

float readVoltage(int pin) {
  int rawValue = analogRead(pin);
  return (rawValue * 3.3) / 4095.0;
}

unsigned long getRandomValue() {
  float tempC = analogReadTemp();

  char msg[64];
  snprintf(msg, sizeof(msg), "Internal Temp: %.2f(C)", tempC);
  addLog(msg);

  // 외부 ADC 읽기 (전압으로 변환)
  float voltA0 = readVoltage(A0);
  float voltA1 = readVoltage(A1);
  float voltA2 = readVoltage(A2);
  float voltA3 = readVoltage(A3);

  snprintf(msg, sizeof(msg), "A0:%.2fV A1:%.2fV A2:%.2fV A3:%.2fV", voltA0,
           voltA1, voltA2, voltA3);
  addLog(msg);

  int valA0 = analogRead(A0);
  int valA1 = analogRead(A1);
  int valA2 = analogRead(A2);
  int valA3 = analogRead(A3);

  unsigned long val = tempC + valA0 + valA1 + valA2 + valA3;
  snprintf(msg, sizeof(msg), "Sum: %lu", val);
  addLog(msg);

  // 시간 기반 엔트로피 추가
  unsigned long time_entropy = micros();
  snprintf(msg, sizeof(msg), "Time entropy: %lu", time_entropy);
  addLog(msg);

  return (unsigned long)val + time_entropy;
}

void i2c_reset() {
  addLog("Initializing I2C...");
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(1000000); // 1MHz
  delay(10);
}

void ssd1306_init() {
  addLog("Initializing SSD1306...");

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
      addLog(msg);

      // 에러 시 재시도
      if (error == 5) { // timeout
        addLog("Timeout detected, resetting I2C...");
        i2c_reset();
        delay(10);
        // 재시도
        Wire.beginTransmission(SSD1306_I2C_ADDR);
        Wire.write(0x00);
        Wire.write(init_cmds[i]);
        error = Wire.endTransmission();
        if (error == 0) {
          addLog("Retry successful");
        }
      }
    }
  }

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  addLog("SSD1306 initialization complete");
}

void setup() {
  Serial.begin(115200);

  adc_init();
  analogReadResolution(12);

  i2c_reset();

  addLog("Starting OLED bouncing icon...");

  randomSeed(getRandomValue());

  ssd1306_init();

  // 초기화 완료 후 고속 모드로 전환
  Wire.setClock(1000000); // 1MHz (최고 속도)
  addLog("I2C speed set to 1MHz");

  // Point들 초기화
  for (int i = 0; i < NUM_POINTS; i++) {
    int x = random(0, SCREEN_WIDTH);
    int y = random(0, SCREEN_HEIGHT);
    int vx = random(-3, 4);
    int vy = random(-3, 4);
    if (vx == 0)
      vx = 1;
    if (vy == 0)
      vy = 1;
    points[i] = new Point(x, y, vx, vy);
  }

  // 아이콘 초기화 (화면 중앙 상단에서 10픽셀 아래)
  bouncingIcon = Icon(SCREEN_WIDTH / 2 - 4, 10);

  frameBuffer.clear(false, 0xff);

  delay(100);
}

unsigned long timestamp = millis();
unsigned long temp_debug_timer = 0;
bool allowSerial = false;
int frameCount = 0;
float currentTemp = 0;
float currentFPS = 0;

void loop() {
  frameBuffer.clear();
  float dt = (millis() - timestamp) / 1000.0; // 초 단위로 변환
  timestamp = millis();
  frameCount++;

  if (millis() - temp_debug_timer >= 3000) {
    float cpuTemp = analogReadTemp();
    float fps = frameCount / 3.0;
    char tempMsg[64];
    snprintf(tempMsg, sizeof(tempMsg), "CPU: %.2f(C), FPS: %.1f", cpuTemp, fps);
    addLog(tempMsg);
    temp_debug_timer = millis();
    frameCount = 0;

    currentTemp = cpuTemp;
    currentFPS = fps;
  }

  // OLED에 온도와 FPS 표시
  u8g2.clearBuffer();
  char tempStr[16], fpsStr[16];
  snprintf(tempStr, sizeof(tempStr), "%.1f°C", currentTemp);
  snprintf(fpsStr, sizeof(fpsStr), "%.1fFPS", currentFPS);
  frameBuffer.drawText(0, -1, tempStr);
  frameBuffer.drawText(0, 8, fpsStr);

  if (timestamp > 1000 || allowSerial) {
    // 저장된 로그 출력 (시리얼 버퍼에 여유가 있을 때)
    flushLogs();
    allowSerial = true;
  }

  // Point들 업데이트 및 그리기
  int dt_ms = dt * 1000; // Point는 ms 단위 사용
  for (int i = 0; i < NUM_POINTS; i++) {
    points[i]->update(dt_ms);
    frameBuffer.setPixel(points[i]->x, points[i]->y, true);
  }

  // 아이콘 업데이트 및 그리기 (시간 배율 적용)
  bouncingIcon.update(dt * 5.0); // 5배 빠른 시간
  frameBuffer.drawIcon((int)bouncingIcon.x, (int)bouncingIcon.y,
                       bouncingIcon.iconData);

  frameBuffer.updateDisplay();

  delay(1000 / 60);
}
