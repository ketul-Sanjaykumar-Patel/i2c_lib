# i2c_lib — Universal I2C Driver Library for Embedded C

[![Build](https://github.com/yourusername/i2c_lib/actions/workflows/build.yml/badge.svg)](https://github.com/yourusername/i2c_lib/actions/workflows/build.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](CHANGELOG.md)
[![C Standard](https://img.shields.io/badge/C-C11-orange.svg)](https://en.wikipedia.org/wiki/C11_(C_standard_revision))

A portable, production-grade I2C master driver for bare-metal and RTOS embedded targets. Works on any platform with GPIO-controllable pins via a simple HAL callback interface.

---

## Features

- **Software (bit-bang) I2C** — works on any MCU with 2 GPIO pins
- **Hardware I2C abstraction** — plug in your platform's HAL (STM32, ESP32, RP2040, AVR)
- **Clock stretching** — slave hold detection with configurable timeout
- **Bus recovery** — 9-clock-pulse recovery sequence per I2C spec §3.1.16
- **Arbitration loss detection** — multi-master bus conflict handling
- **7-bit and 10-bit addressing** support
- **Register shorthand API** — `i2c_reg_write8`, `i2c_reg_read16_be`, bit set/clear/update
- **Bus diagnostics** — scan, device probe, statistics counters
- **Zero dynamic allocation** — static pool, no heap, RTOS-friendly
- **8 production device drivers** included (see below)

---

## Supported Devices (included drivers)

| Device | Category | Real-world use |
|--------|----------|----------------|
| BME280 | Temperature / Humidity / Pressure | Google Pixel, DJI drones, weather stations |
| MPU-6050 | 6-axis IMU | Drone flight controllers, VR headsets, wearables |
| VEML7700 | Ambient light sensor | Auto screen brightness (phones, laptops) |
| SSD1306 | 128×64 OLED display | Smart watches, 3D printers, instruments |
| INA226 | Current / power monitor | Server PDUs, EVs, solar chargers |
| DS3231 | Real-time clock | Data loggers, Raspberry Pi HATs |
| AT24C32 | 32Kbit EEPROM | Config storage, EDID, sensor calibration |
| PCF8574 | 8-bit GPIO expander | LCD backpack (HD44780), industrial relay boards |

---

## Quick Start

### 1 — Implement the 7 HAL callbacks for your platform

```c
#include "i2c.h"

static void my_sda_dir  (bool output, void *ctx) { /* set pin direction */ }
static void my_sda_write(bool level,  void *ctx) { /* write pin */         }
static bool my_sda_read (void *ctx)              { /* read pin */  return false; }
static void my_scl_write(bool level,  void *ctx) { /* write pin */         }
static bool my_scl_read (void *ctx)              { /* read pin */  return false; }
static void my_delay_us (uint32_t us)            { /* spin/timer delay */  }
static uint32_t my_millis(void)                  { /* millisecond tick */  return 0; }

static i2c_hal_t hal = {
    .sda_dir   = my_sda_dir,
    .sda_write = my_sda_write,
    .sda_read  = my_sda_read,
    .scl_write = my_scl_write,
    .scl_read  = my_scl_read,
    .delay_us  = my_delay_us,
    .millis    = my_millis,
};
```

### 2 — Initialize the bus

```c
i2c_bus_t bus;
i2c_config_t cfg = {
    .mode       = I2C_MODE_SOFTWARE,
    .speed      = I2C_SPEED_FAST,      /* 400 kHz */
    .timeout_ms = 50,
    .hal        = &hal,
};
i2c_init(&bus, &cfg);
```

### 3 — Communicate

```c
/* Single register read */
uint8_t who_am_i;
i2c_reg_read8(bus, 0x68, 0x75, &who_am_i);     /* MPU-6050 WHO_AM_I */

/* Burst read (14 bytes = all MPU-6050 sensor data) */
uint8_t raw[14];
i2c_write_reg_read(bus, 0x68, 0x3B, raw, 14);

/* Modify a single bit */
i2c_reg_set_bits(bus, 0x68, 0x6B, 0x40);       /* set SLEEP bit */
i2c_reg_clear_bits(bus, 0x68, 0x6B, 0x40);     /* clear SLEEP bit */
```

### 4 — Use a device driver

```c
#include "i2c_devices.h"

bme280_t sensor;
bme280_init(&sensor, bus, BME280_ADDR_LOW);

int32_t  temp;   /* hundredths of °C,  e.g. 2350 = 23.50°C  */
uint32_t press;  /* Pa,                e.g. 101325 = 1013.25 hPa */
uint32_t hum;    /* 1024 × %RH,        e.g. 51200 = 50.0% RH  */
bme280_read(&sensor, &temp, &press, &hum);
```

---

## File Structure

```
i2c_lib/
├── include/
│   └── i2c.h              ← Core API (all types, functions, docs)
├── src/
│   └── i2c.c              ← Software bit-bang implementation
├── drivers/
│   ├── i2c_devices.h      ← 8 device driver headers
│   └── i2c_devices.c      ← 8 device driver implementations
├── examples/
│   └── example_all_devices.c  ← Full demo: all drivers on one bus
├── CMakeLists.txt
├── CHANGELOG.md
├── CONTRIBUTING.md
└── LICENSE
```

---

## API Reference

### Initialization

| Function | Description |
|----------|-------------|
| `i2c_init(bus, cfg)` | Initialize bus instance |
| `i2c_deinit(bus)` | Release resources, send STOP |
| `i2c_set_speed(bus, speed)` | Change clock speed at runtime |

### Core I/O

| Function | Description |
|----------|-------------|
| `i2c_write(bus, addr, data, len)` | Multi-byte write |
| `i2c_read(bus, addr, data, len)` | Multi-byte read |
| `i2c_write_reg_read(bus, addr, reg, data, len)` | Write register, then read (combined transaction) |
| `i2c_write_reg(bus, addr, reg, data, len)` | Write register + data |

### Register Shorthand

| Function | Description |
|----------|-------------|
| `i2c_reg_write8(bus, addr, reg, val)` | Write 1 byte to register |
| `i2c_reg_read8(bus, addr, reg, &val)` | Read 1 byte from register |
| `i2c_reg_read16_be(bus, addr, reg, &val)` | Read 16-bit big-endian |
| `i2c_reg_read16_le(bus, addr, reg, &val)` | Read 16-bit little-endian |
| `i2c_reg_set_bits(bus, addr, reg, mask)` | Set bits (RMW) |
| `i2c_reg_clear_bits(bus, addr, reg, mask)` | Clear bits (RMW) |
| `i2c_reg_update_bits(bus, addr, reg, mask, val)` | Set field value (RMW) |

### Diagnostics

| Function | Description |
|----------|-------------|
| `i2c_scan(bus, addrs, &count)` | Discover all devices (0x08–0x77) |
| `i2c_device_present(bus, addr)` | Probe a single address |
| `i2c_recover(bus)` | 9-clock bus recovery |
| `i2c_get_state(bus)` | IDLE / BUSY / ERROR / SUSPENDED |
| `i2c_get_stats(bus, &stats)` | Transaction, error, NACK counters |
| `i2c_err_to_string(err)` | Human-readable error description |

---

## Platform Porting

Implement the 7 callbacks in `i2c_hal_t`. Full examples for common platforms:

<details>
<summary>STM32 (HAL)</summary>

```c
static void sda_dir(bool output, void *ctx) {
    GPIO_InitTypeDef g = {
        .Pin  = GPIO_PIN_8,
        .Mode = output ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL,
    };
    HAL_GPIO_Init(GPIOB, &g);
}
static void sda_write(bool level, void *ctx) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static bool sda_read(void *ctx) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET;
}
static void scl_write(bool level, void *ctx) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static bool scl_read(void *ctx) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET;
}
static void delay_us(uint32_t us) { /* DWT or TIM-based */ }
static uint32_t millis(void) { return HAL_GetTick(); }
```
</details>

<details>
<summary>ESP32 (ESP-IDF)</summary>

```c
#include "driver/gpio.h"
#include "esp_timer.h"

static void sda_dir(bool output, void *ctx) {
    gpio_set_direction(GPIO_NUM_21, output ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_INPUT);
}
static void sda_write(bool level, void *ctx) { gpio_set_level(GPIO_NUM_21, level); }
static bool sda_read(void *ctx)  { return gpio_get_level(GPIO_NUM_21); }
static void scl_write(bool level, void *ctx) { gpio_set_level(GPIO_NUM_22, level); }
static bool scl_read(void *ctx)  { return gpio_get_level(GPIO_NUM_22); }
static void delay_us(uint32_t us){ esp_rom_delay_us(us); }
static uint32_t millis(void)     { return (uint32_t)(esp_timer_get_time() / 1000); }
```
</details>

<details>
<summary>Arduino</summary>

```cpp
static void sda_dir(bool output, void *ctx) {
    pinMode(SDA, output ? OUTPUT : INPUT);
}
static void sda_write(bool level, void *ctx) { digitalWrite(SDA, level ? HIGH : LOW); }
static bool sda_read(void *ctx)  { return digitalRead(SDA) == HIGH; }
static void scl_write(bool level, void *ctx) { digitalWrite(SCL, level ? HIGH : LOW); }
static bool scl_read(void *ctx)  { return digitalRead(SCL) == HIGH; }
static void delay_us(uint32_t us){ delayMicroseconds(us); }
static uint32_t millis_fn(void)  { return millis(); }
```
</details>

<details>
<summary>Raspberry Pi (Linux /dev/i2c-N)</summary>

```c
#include <fcntl.h>
#include <linux/i2c-dev.h>
/* For Linux targets, use hardware mode with the kernel I2C driver instead.
 * Open /dev/i2c-1, use ioctl(fd, I2C_SLAVE, addr), then read()/write(). */
```
</details>

---

## Error Handling

Every function returns `i2c_err_t`. Always check it:

```c
i2c_err_t err = i2c_write_reg_read(bus, 0x68, 0x75, &id, 1);
if (err != I2C_OK) {
    printf("Error: %s\n", i2c_err_to_string(err));
    if (err == I2C_ERR_BUS_BUSY || err == I2C_ERR_BUS_ERROR) {
        i2c_recover(bus);   /* Attempt 9-clock recovery */
    }
}
```

| Code | Meaning |
|------|---------|
| `I2C_OK` | Success |
| `I2C_ERR_NACK_ADDR` | Device not found at address |
| `I2C_ERR_NACK_DATA` | Device rejected data |
| `I2C_ERR_ARB_LOST` | Multi-master arbitration loss |
| `I2C_ERR_BUS_BUSY` | SDA or SCL stuck LOW |
| `I2C_ERR_TIMEOUT` | Operation exceeded timeout |
| `I2C_ERR_CLK_STRETCH` | Slave held SCL too long |

---

## Build

```bash
# With CMake
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

# Direct gcc (for quick testing)
gcc -std=c11 -I include -I drivers \
    src/i2c.c drivers/i2c_devices.c examples/example_all_devices.c \
    -o demo -lm
```

---

## Configuration

Override in `i2c_config.h` before including `i2c.h`:

```c
#define I2C_MAX_INSTANCES       4     /* Max simultaneous buses */
#define I2C_TIMEOUT_DEFAULT_MS  100   /* Default timeout */
#define I2C_RETRY_COUNT         3     /* Retries on NACK */
#define I2C_BUS_RECOVERY_CLOCKS 9     /* Recovery pulse count */
#define I2C_ENABLE_RTOS         1     /* Enable FreeRTOS mutex */
```

---

## Roadmap

- [ ] **v2.1** — FreeRTOS mutex protection (`I2C_ENABLE_RTOS`)
- [ ] **v2.1** — DMA transfer support (STM32 HAL)
- [ ] **v2.2** — Async / callback API for non-blocking transfers
- [ ] **v2.2** — TCA9548A I2C multiplexer driver
- [ ] **v2.3** — SMBus / PMBus compatibility layer
- [ ] **v2.3** — 10-bit address mode implementation
- [ ] **v3.0** — PEC (packet error checking) support
- [ ] **v3.0** — Logic analyser export (.csv / .sal format)
- More device drivers: LIS2DH12, VL53L1X, MAX30105, TAS5825M, PCA9685

See [CHANGELOG.md](CHANGELOG.md) for past changes.

---

## Contributing

Contributions welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) first.

- Bug reports → [GitHub Issues](https://github.com/yourusername/i2c_lib/issues)
- New device driver → submit a PR against `drivers/`
- Platform HAL example → add to the `examples/hal/` directory

---

## References

- [NXP UM10204](https://www.nxp.com/docs/en/user-guide/UM10204.pdf) — I2C-bus specification and user manual (the authoritative spec)
- [SMBus Specification](http://smbus.org/specs/) — System Management Bus (I2C subset)
- Device datasheets linked in `drivers/i2c_devices.h`
