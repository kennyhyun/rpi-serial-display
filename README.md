# rpi-serial-display
Raspberry Pi pico example for gm009605 2 color serial graphic display



https://github.com/user-attachments/assets/d7e0fda1-83b2-4453-89bd-3c5fe038e99b

# Optimized SSD1306 OLED Display with Bouncing Balls

A high-performance Arduino sketch for Raspberry Pi Pico that displays animated bouncing balls on an SSD1306 OLED display with real-time temperature and FPS monitoring.

## Dependencies

### Required Libraries
- **[U8g2lib](https://github.com/olikraus/u8g2)** - Graphics library for monochrome displays
- **Wire** - I2C communication library (Arduino core)
- **hardware/adc** - Raspberry Pi Pico ADC hardware access

### Hardware Requirements
- Raspberry Pi Pico
- SSD1306 128x64 OLED Display (I2C)
- I2C connections: SDA, SCL, VCC, GND

## Key Features

### Performance Optimizations
- **No SSD1306 library dependency** - Direct I2C communication for maximum control
- **Double buffering with partial updates** - Only updates changed screen regions for optimal performance
- **Line skip optimization** - Removes unwanted gaps in the frame buffer by skipping specified lines for more natural positioning
- **Chunked display updates** - Configurable chunk size (16 columns) to balance speed and memory
- **1MHz I2C clock speed** - Maximum supported speed for fast data transfer

### Advanced Features
- **ADC-based random seed generation** - Uses internal temperature sensor and external ADC pins (A0-A3) for true randomness
- **Asynchronous log queue system** - Non-blocking serial output with buffered logging to eliminate delays
- **Real-time monitoring** - Displays CPU temperature and FPS on screen
- **Error recovery** - Automatic I2C bus reset and retry on communication errors

### Animation System
- **Physics-based bouncing balls** - Configurable number of points with velocity and collision detection
- **Delta-time animation** - Frame-rate independent movement calculations
- **Boundary collision** - Realistic ball bouncing off screen edges

## Configuration

```cpp
#define NUM_POINTS 9        // Number of bouncing balls
#define COLUMN_SIZE 16      // I2C chunk size for updates
#define LOG_BUFFER_SIZE 10  // Serial log buffer size

// FrameBuffer constructor
FrameBuffer frameBuffer(2, 2);   // Skip 2 lines starting from page 2
// FrameBuffer frameBuffer(0, 0);   // Disable optimization (standard mode)
```

## Performance

- Achieves 60 FPS target with optimized partial screen updates
- Minimal memory footprint with efficient double buffering
- Non-blocking serial communication prevents frame drops
- Hardware-accelerated random number generation
