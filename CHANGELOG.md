# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Planned
- FreeRTOS mutex protection for thread-safe multi-task bus access
- DMA transfer mode for STM32 hardware I2C
- Async/callback API for non-blocking transfers
- TCA9548A I2C multiplexer driver
- SMBus / PMBus compatibility layer (PEC checksum, timeout enforcement)
- 10-bit addressing implementation
- Logic analyser trace export (.csv / .sal)
- Unit test suite with mock HAL (CMock / Unity)

---

## [2.0.0] - 2024-01-01

### Added
- Full software (bit-bang) I2C implementation with clock stretching support
- Hardware I2C abstraction layer via `i2c_hal_t` callbacks
- 7-bit addressing, arbitration loss detection
- Bus recovery sequence (9 SCL clock pulses per NXP UM10204 §3.1.16)
- Register shorthand API: `i2c_reg_write8/read8/read16_be/read16_le`
- Bit field manipulation: `i2c_reg_set_bits`, `i2c_reg_clear_bits`, `i2c_reg_update_bits`
- Bus scan across all valid addresses (0x08–0x77)
- Device probe (`i2c_device_present`)
- Bus statistics: transaction count, error count, NACK count, timeout count
- `i2c_err_to_string()` for human-readable error messages
- Static instance pool (`I2C_MAX_INSTANCES`, no heap allocation)
- Device drivers: BME280, MPU-6050, VEML7700, SSD1306, INA226, DS3231, AT24C32, PCF8574
- BME280: official Bosch integer compensation formulas (temp, pressure, humidity)
- MPU-6050: burst 14-byte read for data consistency, WHO_AM_I verification
- VEML7700: non-linear lux correction for >1000 lux per application note
- SSD1306: standard init sequence, pixel framebuffer, flush via low-level API
- INA226: calibration register setup, current/power LSB calculation
- DS3231: BCD conversion, temperature sensor read (0.25°C resolution)
- AT24C32: automatic page-boundary split writes, ACK polling for write cycle
- PCF8574: single-pin get/set, cached output state
- CMakeLists.txt for CMake builds
- `example_all_devices.c` demonstrating all drivers on one bus

---

## [1.0.0] - Initial release

### Added
- Basic software I2C START/STOP/byte write/byte read
- No error handling, no statistics, no device drivers
