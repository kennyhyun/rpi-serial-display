#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SSD1306_I2C_ADDR 0x3C
#define COLUMN_SIZE 16  // 한 번에 전송할 컬럼 수

#define NUM_POINTS 3  // 동시에 움직일 점 개수

#define PAGE_TO_SHIFT 2 // 프레임 갱신 생략할 라인이 속한 페이지 (0부터)
#define LINES_TO_DELETE 2 // 프레임 갱신 생략할 라인 수


class FrameBuffer {
public:
  uint8_t buffer1[SCREEN_WIDTH * SCREEN_HEIGHT / 8];
  uint8_t buffer2[SCREEN_WIDTH * SCREEN_HEIGHT / 8];
  uint8_t* current;
  uint8_t* previous;

  FrameBuffer() {
    current = buffer1;
    previous = buffer2;
  }

  void swap() {
    uint8_t* temp = current;
    current = previous;
    previous = temp;
  }

  void clear(bool clearCurrent = true) {
    uint8_t* target = clearCurrent ? current : previous;
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT / 8; i++) {
      target[i] = 0x00;
    }
  }

  void setPixel(int x, int y, bool on) {
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) return;
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
    int dx = vx * dt / 16;  // 16ms 기준으로 정규화
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

Point* points[NUM_POINTS];

void ssd1306_init() {
  Wire.begin();
  Wire.beginTransmission(SSD1306_I2C_ADDR);
  Wire.write(0x00); // 명령 모드
  Wire.write(0xAE); // Display OFF
  Wire.write(0x20); Wire.write(0x00); // Memory addressing mode: Horizontal
  Wire.write(0x21); Wire.write(0); Wire.write(SCREEN_WIDTH - 1); // Column
  Wire.write(0x22); Wire.write(0); Wire.write(SCREEN_HEIGHT / 8 - 1); // Page
  Wire.write(0xAF); // Display ON
  Wire.endTransmission();
}

void setup() {
  randomSeed(analogRead(0));
  Wire.begin();
  Wire.setClock(400000); // 400kHz
  ssd1306_init();

  for (int i = 0; i < NUM_POINTS; i++) {
    int x = random(0, SCREEN_WIDTH);
    int y = random(0, SCREEN_HEIGHT);
    int vx = random(-3, 4);
    int vy = random(-3, 4);
    if (vx == 0) vx = 1;
    if (vy == 0) vy = 1;
    points[i] = new Point(x, y, vx, vy);
  }

  frameBuffer.clear(true);
  frameBuffer.clear(false);
}

unsigned long timestamp = millis();
void loop() {
  frameBuffer.clear();
  int dt = millis() - timestamp;
  timestamp = millis();
  for (int i = 0; i < NUM_POINTS; i++) {
    points[i]->update(dt);
    frameBuffer.setPixel(points[i]->x, points[i]->y, true);
  }
  frameBuffer.updateDisplay();
  delay(30);
}
