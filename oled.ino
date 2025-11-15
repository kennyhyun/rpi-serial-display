#include <Wire.h>
#include <hardware/adc.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SSD1306_I2C_ADDR 0x3C
#define COLUMN_SIZE 16 // 한 번에 전송할 컬럼 수

#define NUM_POINTS 9 // 동시에 움직일 점 개수

#define PAGE_TO_SHIFT 2   // 프레임 갱신 생략할 라인이 속한 페이지 (0부터)
#define LINES_TO_DELETE 2 // 프레임 갱신 생략할 라인 수

class FrameBuffer {
public:
  uint8_t buffer1[SCREEN_WIDTH * SCREEN_HEIGHT / 8];
  uint8_t buffer2[SCREEN_WIDTH * SCREEN_HEIGHT / 8];
  uint8_t *current;
  uint8_t *previous;

  FrameBuffer() {
    current = buffer1;
    previous = buffer2;
  }

  void swap() {
    uint8_t *temp = current;
    current = previous;
    previous = temp;
  }

  void clear(bool clearCurrent = true, uint8_t clearValue = 0x00) {
    uint8_t *target = clearCurrent ? current : previous;
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT / 8; i++) {
      target[i] = clearValue;
    }
  }

  void setPixel(int x, int y, bool on) {
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT)
      return;
    int page = y / 8;
    int bit = y % 8;
    if (on)
      current[page * SCREEN_WIDTH + x] |= (1 << bit);
    else
      current[page * SCREEN_WIDTH + x] &= ~(1 << bit);
  }

  void updateDisplay() {
    for (int page = 0; page < SCREEN_HEIGHT / 8; page++) {
      for (int chunk = 0; chunk < SCREEN_WIDTH; chunk += COLUMN_SIZE) {
        bool chunk_changed = false;
        int chunk_end = min(chunk + COLUMN_SIZE, SCREEN_WIDTH);

        for (int col = chunk; col < chunk_end; col++) {
          int idx = page * SCREEN_WIDTH + col;

          if (page >= PAGE_TO_SHIFT) {
            // 16비트 데이터 구성 (LSB가 상단)
            uint16_t curr_16 = (uint16_t)current[idx];
            uint16_t prev_16 = (uint16_t)previous[idx];

            if (page < SCREEN_HEIGHT / 8 - 1) {
              int next_idx = (page + 1) * SCREEN_WIDTH + col;
              curr_16 |= (uint16_t)current[next_idx] << 8;
              prev_16 |= (uint16_t)previous[next_idx] << 8;
            }

            // 마스크: 하위 LINES_TO_DELETE 비트 제거
            uint16_t mask = 0x00FF << LINES_TO_DELETE;
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

            if (page >= PAGE_TO_SHIFT) {
              // 16비트 데이터 구성 (LSB가 상단)
              uint16_t data_16 = (uint16_t)current[idx];
              if (page < SCREEN_HEIGHT / 8 - 1) {
                int next_idx = (page + 1) * SCREEN_WIDTH + col;
                data_16 |= (uint16_t)current[next_idx] << 8;
              }

              // LINES_TO_DELETE만큼 시프트하여 라인 제거
              data = data_16 >> LINES_TO_DELETE;
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

FrameBuffer frameBuffer;

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
    int dx = vx * dt / 16; // 16ms 기준으로 정규화
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

Point *points[NUM_POINTS];

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

unsigned long getRandomValue() {
  float tempC = analogReadTemp();

  char msg[64];
  snprintf(msg, sizeof(msg), "Internal Temp: %.2f°C", tempC);
  addLog(msg);

  // 외부 ADC 읽기
  int valA0 = analogRead(A0);
  int valA1 = analogRead(A1);
  int valA2 = analogRead(A2);
  int valA3 = analogRead(A3);

  snprintf(msg, sizeof(msg), "A0:%d A1:%d A2:%d A3:%d", valA0, valA1, valA2,
           valA3);
  addLog(msg);

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
  addLog("Resetting I2C bus...");
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(1000000); // 1MHz
  delay(10);
}

void ssd1306_init() {
  addLog("Initializing SSD1306...");

  // I2C 버스 리셋
  i2c_reset();

  // 완전한 초기화 시퀀스
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

  addLog("SSD1306 initialization complete");
}

void setup() {
  Serial.begin(115200);

  // ADC 초기화
  adc_init();
  analogReadResolution(12);

  // I2C 초기화 (안정성 우선)
  Wire.begin();
  Wire.setClock(100000); // 100kHz 초기화용
  // delay(600); // 안정화 대기

  addLog("Starting OLED bouncing balls...");

  randomSeed(getRandomValue());

  ssd1306_init();

  // 초기화 완료 후 고속 모드로 전환
  Wire.setClock(1000000); // 1MHz (최고 속도)
  addLog("I2C speed set to 1MHz");

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

  frameBuffer.clear(false, 0xff);

  delay(100);
}

unsigned long timestamp = millis();
unsigned long temp_debug_timer = 0;
bool allowSerial = false;

void loop() {
  frameBuffer.clear();
  int dt = millis() - timestamp;
  timestamp = millis();

  // 3초마다 CPU 온도 로그 저장
  if (millis() - temp_debug_timer >= 3000) {
    float cpuTemp = analogReadTemp();
    char tempMsg[64];
    snprintf(tempMsg, sizeof(tempMsg), "CPU Temperature: %.2f(C)", cpuTemp);
    addLog(tempMsg);
    temp_debug_timer = millis();
  }

  if (timestamp > 1000 || allowSerial) {
    // 저장된 로그 출력 (시리얼 버퍼에 여유가 있을 때)
    flushLogs();
    allowSerial = true;
  }

  for (int i = 0; i < NUM_POINTS; i++) {
    points[i]->update(dt);
    frameBuffer.setPixel(points[i]->x, points[i]->y, true);
  }

  frameBuffer.updateDisplay();
  delay(1000 / 60);
}
