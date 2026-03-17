/**
 * @file    i2c_devices.h
 * @brief   Ready-to-use drivers for common real-world I2C devices
 *
 * This file provides minimal, documented drivers for widely-used I2C sensors
 * and peripherals. Each driver shows how the device is used in the real world
 * and demonstrates the I2C library API patterns.
 *
 * COVERED DEVICES:
 * ================
 *
 *  [ENVIRONMENT SENSING]
 *   1. BME280  — Temperature, Humidity, Pressure (Bosch)
 *      Used in: Weather stations, HVAC, drones, smartphones (Google Pixel etc.)
 *
 *   2. MPU-6050 — 6-axis IMU (Accelerometer + Gyroscope) (InvenSense/TDK)
 *      Used in: Drone flight controllers, VR headsets, robotics, wearables,
 *               smartphone screen rotation, gaming controllers
 *
 *   3. VEML7700 — Ambient Light Sensor (Vishay)
 *      Used in: Auto display brightness (phones, laptops), street lighting,
 *               industrial light metering
 *
 *  [DISPLAY]
 *   4. SSD1306  — 128×64 OLED display controller (Solomon Systech)
 *      Used in: Smart watches, IoT dashboards, wearables, dev boards,
 *               medical devices, portable instruments
 *
 *  [POWER MANAGEMENT]
 *   5. INA226   — High-side current/power monitor (Texas Instruments)
 *      Used in: Battery management, server PSUs, solar chargers, EVs,
 *               rack power distribution units (PDUs)
 *
 *  [REAL-TIME CLOCK]
 *   6. DS3231   — Extremely accurate RTC (Maxim/Analog Devices)
 *      Used in: Data loggers, industrial timers, embedded Linux boards
 *               (Raspberry Pi HATs), access control systems
 *
 *  [MEMORY]
 *   7. AT24C32  — 32Kbit I2C EEPROM (Atmel/Microchip)
 *      Used in: Configuration storage, calibration data, serial number storage,
 *               EDID data in monitors, SPD in RAM modules
 *
 *  [GPIO EXPANSION]
 *   8. PCF8574  — 8-bit I/O Expander (NXP/TI)
 *      Used in: Industrial PLCs, LCD backpacks (HD44780 via I2C),
 *               keyboard matrix expansion, relay boards
 */

#ifndef I2C_DEVICES_H
#define I2C_DEVICES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

/* ============================================================================
 * 1. BME280 — Temperature / Humidity / Pressure Sensor
 *    Datasheet: https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/
 *
 *    DEFAULT I2C ADDRESS: 0x76 (SDO=GND) or 0x77 (SDO=VCC)
 *
 *    REAL-WORLD USAGE:
 *    - Google Pixel phones (barometric pressure for GPS elevation)
 *    - DJI drones (altitude hold)
 *    - Commercial weather stations
 *    - Home automation hubs
 *    - HVAC control systems (humidity feedback)
 * ========================================================================== */

#define BME280_ADDR_LOW     0x76    /**< SDO pin = GND */
#define BME280_ADDR_HIGH    0x77    /**< SDO pin = VCC */

#define BME280_REG_CHIP_ID  0xD0    /**< Should read 0x60 */
#define BME280_REG_RESET    0xE0    /**< Write 0xB6 to reset */
#define BME280_REG_CTRL_HUM 0xF2    /**< Humidity oversampling */
#define BME280_REG_STATUS   0xF3    /**< Measuring/updating status */
#define BME280_REG_CTRL_MEAS 0xF4   /**< Temp/pressure oversampling + mode */
#define BME280_REG_CONFIG   0xF5    /**< Standby time, filter, SPI 3-wire */
#define BME280_REG_PRESS_MSB 0xF7   /**< Start of 8-byte data block */
#define BME280_REG_CALIB_00  0x88   /**< Factory calibration data (trimming) */
#define BME280_REG_CALIB_26  0xE1   /**< Humidity calibration data */

/**
 * @brief BME280 oversampling setting
 *
 * Higher oversampling = more accuracy, more current draw, slower update rate.
 * For HVAC: ×1 is fine. For weather stations: ×4 or ×8.
 */
typedef enum {
    BME280_OVERSAMPLE_SKIP = 0x00,  /**< Output = 0x80000 (invalid) */
    BME280_OVERSAMPLE_1    = 0x01,  /**< ×1 oversampling */
    BME280_OVERSAMPLE_2    = 0x02,  /**< ×2 oversampling */
    BME280_OVERSAMPLE_4    = 0x03,  /**< ×4 oversampling */
    BME280_OVERSAMPLE_8    = 0x04,  /**< ×8 oversampling */
    BME280_OVERSAMPLE_16   = 0x05,  /**< ×16 oversampling */
} bme280_oversample_t;

/**
 * @brief BME280 sensor mode
 */
typedef enum {
    BME280_MODE_SLEEP  = 0x00,  /**< No measurements */
    BME280_MODE_FORCED = 0x01,  /**< Single measurement, then sleep */
    BME280_MODE_NORMAL = 0x03,  /**< Continuous measurements */
} bme280_mode_t;

/** BME280 calibration data (loaded once from device NVM at init) */
typedef struct {
    uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
    uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3;
    int16_t  dig_P4; int16_t dig_P5; int16_t dig_P6;
    int16_t  dig_P7; int16_t dig_P8; int16_t dig_P9;
    uint8_t  dig_H1; int16_t dig_H2; uint8_t dig_H3;
    int16_t  dig_H4; int16_t dig_H5; int8_t  dig_H6;
    int32_t  t_fine;  /**< Shared between compensation formulas */
} bme280_calib_t;

/** BME280 driver handle */
typedef struct {
    i2c_bus_t   bus;
    uint8_t     addr;
    bme280_calib_t calib;
    bool        initialized;
} bme280_t;

/**
 * @brief Initialize BME280 sensor
 *
 * Verifies chip ID, reads calibration data from NVM, configures oversampling.
 *
 * @param dev   Pointer to BME280 handle (user-allocated)
 * @param bus   I2C bus handle
 * @param addr  I2C address (BME280_ADDR_LOW or BME280_ADDR_HIGH)
 * @return I2C_OK on success, I2C_ERR_NACK_ADDR if sensor not found
 *
 * @example
 * @code
 * bme280_t sensor;
 * if (bme280_init(&sensor, bus, BME280_ADDR_LOW) != I2C_OK) {
 *     printf("BME280 not found!\n");
 * }
 * @endcode
 */
i2c_err_t bme280_init(bme280_t *dev, i2c_bus_t bus, uint8_t addr);

/**
 * @brief Read compensated temperature, pressure, and humidity
 *
 * Uses Bosch's official compensation formulas from the BME280 datasheet.
 * Integer math only — no floating point required.
 *
 * @param dev          BME280 handle
 * @param[out] temp    Temperature in hundredths of °C (e.g., 2350 = 23.50°C)
 * @param[out] press   Pressure in Pa (e.g., 101325 = 1013.25 hPa)
 * @param[out] hum     Humidity in 1024ths of % RH (e.g., 51200 = 50.0% RH)
 * @return I2C_OK on success
 */
i2c_err_t bme280_read(bme280_t *dev, int32_t *temp, uint32_t *press, uint32_t *hum);

/* Internal helper — load calibration from device NVM */
/* Internal — implemented in i2c_devices.c */

/* ============================================================================
 * 2. MPU-6050 — 6-Axis IMU (Accelerometer + Gyroscope)
 *    Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *
 *    DEFAULT I2C ADDRESS: 0x68 (AD0=GND) or 0x69 (AD0=VCC)
 *
 *    REAL-WORLD USAGE:
 *    - Drone flight controllers (ArduPilot, Betaflight): pitch/roll/yaw
 *    - VR/AR headsets: head tracking orientation
 *    - Prosthetic limbs: gesture recognition
 *    - Smartphone screen auto-rotate
 *    - Balance robots (segway-style)
 *    - Sports performance trackers
 * ========================================================================== */

#define MPU6050_ADDR_LOW        0x68    /**< AD0 pin = GND (default) */
#define MPU6050_ADDR_HIGH       0x69    /**< AD0 pin = VCC */

/** MPU-6050 Register Map */
#define MPU6050_REG_SELF_TEST_X 0x0D
#define MPU6050_REG_SMPLRT_DIV  0x19    /**< Sample rate divider */
#define MPU6050_REG_CONFIG      0x1A    /**< DLPF (digital low-pass filter) */
#define MPU6050_REG_GYRO_CONFIG 0x1B    /**< Gyro full-scale range */
#define MPU6050_REG_ACCEL_CONFIG 0x1C   /**< Accel full-scale range */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B  /**< First of 14 data bytes */
#define MPU6050_REG_TEMP_OUT_H  0x41
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_PWR_MGMT_1  0x6B   /**< Power management / clock source */
#define MPU6050_REG_WHO_AM_I    0x75   /**< Should return 0x68 */

#define MPU6050_WHO_AM_I_VAL    0x68   /**< Expected chip ID */

/**
 * @brief MPU-6050 accelerometer full-scale range
 *
 * Choose based on application:
 *   - Drones: ±8g (fast maneuvers)
 *   - Pedometer: ±2g (walking)
 *   - Crash detection: ±16g
 */
typedef enum {
    MPU6050_ACCEL_2G  = 0x00,   /**< ±2g,  sensitivity: 16384 LSB/g */
    MPU6050_ACCEL_4G  = 0x08,   /**< ±4g,  sensitivity:  8192 LSB/g */
    MPU6050_ACCEL_8G  = 0x10,   /**< ±8g,  sensitivity:  4096 LSB/g */
    MPU6050_ACCEL_16G = 0x18,   /**< ±16g, sensitivity:  2048 LSB/g */
} mpu6050_accel_range_t;

/**
 * @brief MPU-6050 gyroscope full-scale range
 *
 *   - Slow gestures / tilt: ±250°/s
 *   - Walking robots: ±500°/s
 *   - Racing drones: ±2000°/s
 */
typedef enum {
    MPU6050_GYRO_250  = 0x00,   /**< ±250°/s,  sensitivity: 131.0 LSB/°/s */
    MPU6050_GYRO_500  = 0x08,   /**< ±500°/s,  sensitivity:  65.5 LSB/°/s */
    MPU6050_GYRO_1000 = 0x10,   /**< ±1000°/s, sensitivity:  32.8 LSB/°/s */
    MPU6050_GYRO_2000 = 0x18,   /**< ±2000°/s, sensitivity:  16.4 LSB/°/s */
} mpu6050_gyro_range_t;

/** MPU-6050 raw sensor data (all values are signed 16-bit) */
typedef struct {
    int16_t accel_x;    /**< Raw accelerometer X */
    int16_t accel_y;    /**< Raw accelerometer Y */
    int16_t accel_z;    /**< Raw accelerometer Z */
    int16_t temp_raw;   /**< Raw temperature (temp_C = raw/340.0 + 36.53) */
    int16_t gyro_x;     /**< Raw gyroscope X */
    int16_t gyro_y;     /**< Raw gyroscope Y */
    int16_t gyro_z;     /**< Raw gyroscope Z */
} mpu6050_raw_t;

/** MPU-6050 driver handle */
typedef struct {
    i2c_bus_t           bus;
    uint8_t             addr;
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t  gyro_range;
    bool                initialized;
} mpu6050_t;

/**
 * @brief Initialize MPU-6050
 *
 * Wakes the device (exits sleep mode), sets clock source to PLL with
 * X-axis gyro reference (recommended by datasheet), configures ranges.
 *
 * @param dev         MPU-6050 handle
 * @param bus         I2C bus
 * @param addr        I2C address (MPU6050_ADDR_LOW or HIGH)
 * @param accel_range Accelerometer full-scale range
 * @param gyro_range  Gyroscope full-scale range
 * @return I2C_OK on success
 *
 * @example
 * @code
 * mpu6050_t imu;
 * mpu6050_init(&imu, bus, MPU6050_ADDR_LOW,
 *              MPU6050_ACCEL_8G, MPU6050_GYRO_500);
 * @endcode
 */
i2c_err_t mpu6050_init(mpu6050_t *dev, i2c_bus_t bus, uint8_t addr,
                        mpu6050_accel_range_t accel_range,
                        mpu6050_gyro_range_t gyro_range);

/**
 * @brief Read all raw sensor data in one burst read (14 bytes)
 *
 * Using a burst read (one transaction for all 14 bytes) ensures all values
 * are from the same sample — critical for flight control and sensor fusion.
 *
 * @param dev      MPU-6050 handle
 * @param[out] raw Pointer to raw data structure
 * @return I2C_OK on success
 */
i2c_err_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw);

/**
 * @brief Get temperature in hundredths of degrees Celsius
 *
 * Formula from datasheet: temp_C = (raw / 340.0) + 36.53
 * Integer version: temp_100 = (raw * 100) / 340 + 3653
 *
 * @param raw_temp  Temperature raw value from mpu6050_read_raw()
 * @return Temperature in units of 0.01°C (e.g., 2500 = 25.00°C)
 */
int32_t mpu6050_raw_to_temp(int16_t raw_temp);

/* ============================================================================
 * 3. VEML7700 — Ambient Light Sensor
 *    Datasheet: https://www.vishay.com/docs/84286/veml7700.pdf
 *
 *    FIXED I2C ADDRESS: 0x10
 *
 *    REAL-WORLD USAGE:
 *    - Automatic screen brightness on phones/tablets/laptops
 *    - Smart building lighting control (saves ~30% energy)
 *    - Street lamp dimming (dawn/dusk detection)
 *    - Industrial light measurement
 *    - Photography: automatic exposure assist
 * ========================================================================== */

#define VEML7700_ADDR               0x10

/** VEML7700 registers */
#define VEML7700_REG_ALS_CONF       0x00    /**< Config: gain, integration time, power */
#define VEML7700_REG_ALS_WH         0x01    /**< High threshold window */
#define VEML7700_REG_ALS_WL         0x02    /**< Low threshold window */
#define VEML7700_REG_POWER_SAVING   0x03    /**< Power saving mode */
#define VEML7700_REG_ALS            0x04    /**< ALS output data (16-bit) */
#define VEML7700_REG_WHITE          0x05    /**< White channel output */
#define VEML7700_REG_ALS_INT        0x06    /**< Interrupt status */

/**
 * @brief VEML7700 gain setting
 * Higher gain needed in dark environments, lower gain in bright light.
 */
typedef enum {
    VEML7700_GAIN_1   = 0x00,   /**< ×1 gain (default) */
    VEML7700_GAIN_2   = 0x01,   /**< ×2 gain */
    VEML7700_GAIN_1_8 = 0x02,   /**< ×1/8 gain (bright environments) */
    VEML7700_GAIN_1_4 = 0x03,   /**< ×1/4 gain */
} veml7700_gain_t;

/**
 * @brief VEML7700 integration time
 * Longer = more accurate, but slower updates.
 */
typedef enum {
    VEML7700_IT_25MS  = 0x0C,   /**< 25ms  integration */
    VEML7700_IT_50MS  = 0x08,   /**< 50ms  integration */
    VEML7700_IT_100MS = 0x00,   /**< 100ms integration (default) */
    VEML7700_IT_200MS = 0x01,   /**< 200ms integration */
    VEML7700_IT_400MS = 0x02,   /**< 400ms integration */
    VEML7700_IT_800MS = 0x03,   /**< 800ms integration */
} veml7700_it_t;

/** VEML7700 driver handle */
typedef struct {
    i2c_bus_t       bus;
    veml7700_gain_t gain;
    veml7700_it_t   it;
    float           resolution;  /**< lux per count, depends on gain + IT */
    bool            initialized;
} veml7700_t;

/**
 * @brief Initialize VEML7700 ambient light sensor
 *
 * @param dev   VEML7700 handle
 * @param bus   I2C bus
 * @param gain  Gain setting (start with VEML7700_GAIN_1)
 * @param it    Integration time (start with VEML7700_IT_100MS)
 * @return I2C_OK on success
 */
i2c_err_t veml7700_init(veml7700_t *dev, i2c_bus_t bus,
                         veml7700_gain_t gain, veml7700_it_t it);

/**
 * @brief Read ambient light in lux
 *
 * Applies resolution correction formula from VEML7700 application note.
 * For very high lux (>1000), applies a non-linear correction factor.
 *
 * @param dev        VEML7700 handle
 * @param[out] lux   Illuminance in lux (0.0 = dark, ~100000 = full sunlight)
 * @return I2C_OK on success
 */
i2c_err_t veml7700_read_lux(veml7700_t *dev, float *lux);

/* ============================================================================
 * 4. SSD1306 — 128×64 OLED Display Controller
 *    Datasheet: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 *
 *    DEFAULT I2C ADDRESS: 0x3C (SA0=GND) or 0x3D (SA0=VCC)
 *
 *    REAL-WORLD USAGE:
 *    - Smart watch displays (pre-AMOLED era)
 *    - Raspberry Pi / Arduino project displays
 *    - Medical device status displays (portable ECG monitors)
 *    - 3D printer control panels (Creality, Prusa)
 *    - Network equipment front-panel displays
 *    - Portable instruments (oscilloscopes, analyzers)
 * ========================================================================== */

#define SSD1306_ADDR_LOW    0x3C    /**< SA0=GND */
#define SSD1306_ADDR_HIGH   0x3D    /**< SA0=VCC */

#define SSD1306_WIDTH       128     /**< Display width in pixels */
#define SSD1306_HEIGHT      64      /**< Display height in pixels */
#define SSD1306_PAGES       8       /**< 64 rows / 8 bits per page */
#define SSD1306_BUF_SIZE    (SSD1306_WIDTH * SSD1306_PAGES) /**< 1024 bytes */

/** SSD1306 control bytes (I2C specific) */
#define SSD1306_CTRL_CMD    0x00    /**< Next byte(s) are commands */
#define SSD1306_CTRL_DATA   0x40    /**< Next byte(s) are display data */

/** SSD1306 fundamental commands */
#define SSD1306_CMD_DISPLAY_OFF     0xAE
#define SSD1306_CMD_DISPLAY_ON      0xAF
#define SSD1306_CMD_SET_CONTRAST    0x81    /**< Followed by contrast value 0–255 */
#define SSD1306_CMD_ENTIRE_ON       0xA5    /**< All pixels ON (test mode) */
#define SSD1306_CMD_NORMAL_DISPLAY  0xA4    /**< Normal display from RAM */
#define SSD1306_CMD_INVERT_OFF      0xA6    /**< Normal (0=black, 1=white) */
#define SSD1306_CMD_INVERT_ON       0xA7    /**< Inverted (0=white, 1=black) */
#define SSD1306_CMD_SET_MUX_RATIO   0xA8    /**< Set multiplex ratio */
#define SSD1306_CMD_SET_OFFSET      0xD3    /**< Set display vertical offset */
#define SSD1306_CMD_SET_START_LINE  0x40    /**< Set start line (0x40–0x7F) */
#define SSD1306_CMD_CHARGE_PUMP     0x8D    /**< Enable charge pump */
#define SSD1306_CMD_MEM_ADDR_MODE   0x20    /**< Set memory addressing mode */
#define SSD1306_CMD_COL_ADDR        0x21    /**< Set column address range */
#define SSD1306_CMD_PAGE_ADDR       0x22    /**< Set page address range */
#define SSD1306_CMD_REMAP_SEG_0     0xA0    /**< Column 0 → SEG0 */
#define SSD1306_CMD_REMAP_SEG_127   0xA1    /**< Column 127 → SEG0 (mirror X) */
#define SSD1306_CMD_COM_SCAN_INC    0xC0    /**< Normal COM scan */
#define SSD1306_CMD_COM_SCAN_DEC    0xC8    /**< Remapped COM scan (mirror Y) */
#define SSD1306_CMD_SET_CLK_DIV     0xD5    /**< Set clock divide ratio/frequency */
#define SSD1306_CMD_SET_COM_PINS    0xDA    /**< Set COM pins hardware config */
#define SSD1306_CMD_SET_VCOM_DESEL  0xDB    /**< Set VCOMH deselect level */
#define SSD1306_CMD_PRECHARGE       0xD9    /**< Set pre-charge period */

/** SSD1306 driver handle */
typedef struct {
    i2c_bus_t   bus;
    uint8_t     addr;
    uint8_t     framebuf[SSD1306_BUF_SIZE]; /**< Local frame buffer (draw here, then flush) */
    bool        initialized;
} ssd1306_t;

/**
 * @brief Initialize SSD1306 OLED display
 *
 * Sends the standard initialization sequence: charge pump, timing, COM config.
 * Display starts blank and OFF. Call ssd1306_display_on() to turn on.
 *
 * @param dev   SSD1306 handle
 * @param bus   I2C bus
 * @param addr  I2C address (SSD1306_ADDR_LOW or HIGH)
 * @return I2C_OK on success
 */
i2c_err_t ssd1306_init(ssd1306_t *dev, i2c_bus_t bus, uint8_t addr);

/**
 * @brief Turn display on/off
 */
i2c_err_t ssd1306_display_on(ssd1306_t *dev, bool on);

/**
 * @brief Clear the frame buffer (fill with 0)
 *
 * Does NOT update the display — call ssd1306_flush() after.
 */
void ssd1306_clear(ssd1306_t *dev);

/**
 * @brief Set a single pixel in the frame buffer
 *
 * @param dev    SSD1306 handle
 * @param x      Column (0–127)
 * @param y      Row (0–63)
 * @param on     true = pixel on (white), false = pixel off (black)
 */
void ssd1306_set_pixel(ssd1306_t *dev, uint8_t x, uint8_t y, bool on);

/**
 * @brief Send the entire frame buffer to the OLED via I2C
 *
 * Transfers all 1024 bytes of display data. Takes ~10ms at 400kHz.
 * Called after all drawing operations are complete.
 *
 * @param dev  SSD1306 handle
 * @return I2C_OK on success
 */
i2c_err_t ssd1306_flush(ssd1306_t *dev);

/**
 * @brief Set display contrast (brightness)
 *
 * @param dev       SSD1306 handle
 * @param contrast  0 (dim) to 255 (max brightness)
 * @return I2C_OK on success
 */
i2c_err_t ssd1306_set_contrast(ssd1306_t *dev, uint8_t contrast);

/* ============================================================================
 * 5. INA226 — Current/Power Monitor
 *    Datasheet: https://www.ti.com/lit/ds/symlink/ina226.pdf
 *
 *    DEFAULT I2C ADDRESS: Programmable 0x40–0x4F via A0/A1 pins
 *
 *    REAL-WORLD USAGE:
 *    - Server rack PDUs: monitor each blade's power consumption
 *    - Electric vehicle battery management (cell balancing)
 *    - Solar charge controllers (track power harvest)
 *    - Industrial UPS systems
 *    - Satellite subsystems (NASA JPL uses similar parts)
 *    - Medical power monitoring (defibrillator charging)
 * ========================================================================== */

#define INA226_ADDR_BASE    0x40    /**< A1=GND, A0=GND */

/** INA226 registers */
#define INA226_REG_CONFIG   0x00
#define INA226_REG_SHUNT_V  0x01    /**< Shunt voltage (2.5µV/LSB) */
#define INA226_REG_BUS_V    0x02    /**< Bus voltage (1.25mV/LSB) */
#define INA226_REG_POWER    0x03    /**< Power (depends on calibration) */
#define INA226_REG_CURRENT  0x04    /**< Current (depends on calibration) */
#define INA226_REG_CALIB    0x05    /**< Calibration register */
#define INA226_REG_MASK_EN  0x06    /**< Alert configuration */
#define INA226_REG_ALERT_L  0x07    /**< Alert limit */
#define INA226_REG_MFR_ID   0xFE    /**< Manufacturer ID (should be 0x5449 = 'TI') */
#define INA226_REG_DIE_ID   0xFF    /**< Die ID (should be 0x2260) */

/** INA226 driver handle */
typedef struct {
    i2c_bus_t   bus;
    uint8_t     addr;
    float       current_lsb;     /**< Amps per LSB (set by calibration) */
    float       power_lsb;       /**< Watts per LSB = 25 × current_lsb */
    bool        initialized;
} ina226_t;

/**
 * @brief Initialize INA226 and calibrate for a given shunt resistor
 *
 * Calibration formula: Cal = 0.00512 / (current_lsb × r_shunt)
 *
 * @param dev          INA226 handle
 * @param bus          I2C bus
 * @param addr         I2C address
 * @param r_shunt_ohm  Shunt resistor value in ohms (e.g., 0.1 for 100mΩ)
 * @param max_amps     Maximum expected current (e.g., 5.0 for 5A)
 * @return I2C_OK on success
 *
 * @example
 * @code
 * ina226_t pmon;
 * // 100mΩ shunt, max 5A — typical for power bank monitoring
 * ina226_init(&pmon, bus, INA226_ADDR_BASE, 0.1f, 5.0f);
 * @endcode
 */
i2c_err_t ina226_init(ina226_t *dev, i2c_bus_t bus, uint8_t addr,
                       float r_shunt_ohm, float max_amps);

/**
 * @brief Read bus voltage in millivolts
 *
 * @param dev      INA226 handle
 * @param[out] mv  Bus voltage in millivolts
 * @return I2C_OK on success
 */
i2c_err_t ina226_read_voltage(ina226_t *dev, int32_t *mv);

/**
 * @brief Read current in milliamps
 *
 * @param dev      INA226 handle
 * @param[out] ma  Current in milliamps (negative = reverse current)
 * @return I2C_OK on success
 */
i2c_err_t ina226_read_current(ina226_t *dev, int32_t *ma);

/**
 * @brief Read power in milliwatts
 *
 * @param dev      INA226 handle
 * @param[out] mw  Power in milliwatts
 * @return I2C_OK on success
 */
i2c_err_t ina226_read_power(ina226_t *dev, uint32_t *mw);

/* ============================================================================
 * 6. DS3231 — Extremely Accurate Real-Time Clock
 *    Datasheet: https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
 *
 *    FIXED I2C ADDRESS: 0x68
 *
 *    REAL-WORLD USAGE:
 *    - Data loggers (environmental monitoring, industrial)
 *    - Raspberry Pi HATs (Pi has no RTC; DS3231 adds one)
 *    - Network time appliances and edge routers
 *    - Access control systems (door locks with schedules)
 *    - Smart meters (energy usage timestamps)
 *    - Seismograph stations (precise event timestamps)
 * ========================================================================== */

#define DS3231_ADDR         0x68

/** DS3231 register map */
#define DS3231_REG_SECONDS  0x00    /**< BCD: seconds (00–59) */
#define DS3231_REG_MINUTES  0x01    /**< BCD: minutes (00–59) */
#define DS3231_REG_HOURS    0x02    /**< BCD: hours (00–23 or 1–12 + AM/PM) */
#define DS3231_REG_DAY      0x03    /**< BCD: day of week (1–7) */
#define DS3231_REG_DATE     0x04    /**< BCD: day of month (01–31) */
#define DS3231_REG_MONTH    0x05    /**< BCD: month (01–12), bit7 = century */
#define DS3231_REG_YEAR     0x06    /**< BCD: year (00–99) */
#define DS3231_REG_CTRL     0x0E    /**< Control register */
#define DS3231_REG_STATUS   0x0F    /**< Status register */
#define DS3231_REG_TEMP_MSB 0x11    /**< Temperature MSB (signed, 1°C/bit) */
#define DS3231_REG_TEMP_LSB 0x12    /**< Temperature LSB (bits 7–6: 0.25°C/bit) */

/** DS3231 datetime structure */
typedef struct {
    uint8_t seconds;    /**< 0–59 */
    uint8_t minutes;    /**< 0–59 */
    uint8_t hours;      /**< 0–23 */
    uint8_t day;        /**< 1–7 (day of week) */
    uint8_t date;       /**< 1–31 */
    uint8_t month;      /**< 1–12 */
    uint8_t year;       /**< 0–99 (add 2000 for full year) */
} ds3231_datetime_t;

/** DS3231 driver handle */
typedef struct {
    i2c_bus_t   bus;
    bool        initialized;
} ds3231_t;

/**
 * @brief Initialize DS3231 RTC
 *
 * Clears oscillator stop flag if set (power failure indicator).
 * Enables the 32kHz output if desired.
 *
 * @param dev  DS3231 handle
 * @param bus  I2C bus
 * @return I2C_OK on success
 */
i2c_err_t ds3231_init(ds3231_t *dev, i2c_bus_t bus);

/**
 * @brief Set the current date and time
 *
 * @param dev  DS3231 handle
 * @param dt   Datetime to set
 * @return I2C_OK on success
 */
i2c_err_t ds3231_set_time(ds3231_t *dev, const ds3231_datetime_t *dt);

/**
 * @brief Read the current date and time
 *
 * @param dev      DS3231 handle
 * @param[out] dt  Datetime structure to fill
 * @return I2C_OK on success
 */
i2c_err_t ds3231_get_time(ds3231_t *dev, ds3231_datetime_t *dt);

/**
 * @brief Read the internal temperature sensor
 *
 * DS3231 has a built-in TCXO temperature sensor used for oscillator
 * compensation. Resolution: 0.25°C. Accuracy: ±3°C.
 *
 * @param dev          DS3231 handle
 * @param[out] temp_c4 Temperature in units of 0.25°C (divide by 4 for °C)
 * @return I2C_OK on success
 */
i2c_err_t ds3231_read_temp(ds3231_t *dev, int16_t *temp_c4);

/* ============================================================================
 * 7. AT24C32 — 32Kbit (4096 byte) I2C EEPROM
 *    Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/doc0336.pdf
 *
 *    DEFAULT I2C ADDRESS: 0x50–0x57 (A0/A1/A2 pins)
 *
 *    REAL-WORLD USAGE:
 *    - Monitor EDID storage (display capabilities communicated to GPU)
 *    - DDR RAM SPD chips (memory timing parameters)
 *    - Network card MAC address storage
 *    - Calibration coefficients for sensors
 *    - Product serial numbers and manufacturing data
 *    - Firmware configuration backup
 * ========================================================================== */

#define AT24C32_ADDR_BASE   0x50    /**< A2=A1=A0=GND */
#define AT24C32_PAGE_SIZE   32      /**< 32-byte write page (must align writes) */
#define AT24C32_CAPACITY    4096    /**< Total bytes */
#define AT24C32_WRITE_CYCLE_MS 5    /**< Internal write cycle time */

/** AT24C32 driver handle */
typedef struct {
    i2c_bus_t   bus;
    uint8_t     addr;
    bool        initialized;
} at24c32_t;

/**
 * @brief Initialize AT24C32 EEPROM
 */
i2c_err_t at24c32_init(at24c32_t *dev, i2c_bus_t bus, uint8_t addr);

/**
 * @brief Read bytes from EEPROM
 *
 * Uses 16-bit memory addressing (MSB first).
 *
 * @param dev         AT24C32 handle
 * @param mem_addr    Starting memory address (0–4095)
 * @param[out] data   Buffer to store read data
 * @param len         Number of bytes to read (up to 4096–mem_addr)
 * @return I2C_OK on success
 */
i2c_err_t at24c32_read(at24c32_t *dev, uint16_t mem_addr, uint8_t *data, size_t len);

/**
 * @brief Write bytes to EEPROM (handles page boundary splits automatically)
 *
 * Splits writes that cross 32-byte page boundaries. Waits for write
 * cycle completion between pages (5ms each).
 *
 * IMPORTANT: EEPROM cells have ~1 million write cycle endurance.
 * Do not write the same address in a tight loop!
 *
 * @param dev       AT24C32 handle
 * @param mem_addr  Starting memory address (0–4095)
 * @param data      Data to write
 * @param len       Number of bytes to write
 * @return I2C_OK on success
 */
i2c_err_t at24c32_write(at24c32_t *dev, uint16_t mem_addr,
                         const uint8_t *data, size_t len);

/* ============================================================================
 * 8. PCF8574 — 8-bit Remote I/O Expander
 *    Datasheet: https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf
 *
 *    DEFAULT I2C ADDRESS: 0x20–0x27 (PCF8574) or 0x38–0x3F (PCF8574A)
 *
 *    REAL-WORLD USAGE:
 *    - HD44780 LCD backpack (enables LCD with only 2 I2C wires)
 *    - Industrial PLC relay expansion boards
 *    - LED indicator panels
 *    - Button/keypad input expansion
 *    - Fan and pump control in industrial equipment
 * ========================================================================== */

#define PCF8574_ADDR_BASE   0x20    /**< A2=A1=A0=GND */
#define PCF8574A_ADDR_BASE  0x38    /**< PCF8574A variant */

/** PCF8574 driver handle */
typedef struct {
    i2c_bus_t   bus;
    uint8_t     addr;
    uint8_t     out_state;  /**< Cached output state */
    bool        initialized;
} pcf8574_t;

/**
 * @brief Initialize PCF8574 GPIO expander
 *
 * Sets all 8 pins to HIGH (input mode — quasi-bidirectional).
 *
 * @param dev   PCF8574 handle
 * @param bus   I2C bus
 * @param addr  I2C address (PCF8574_ADDR_BASE + pin combination 0–7)
 * @return I2C_OK on success
 */
i2c_err_t pcf8574_init(pcf8574_t *dev, i2c_bus_t bus, uint8_t addr);

/**
 * @brief Write all 8 GPIO pins at once
 *
 * PCF8574 is quasi-bidirectional: pins driven HIGH act as inputs.
 * For outputs: drive HIGH or LOW. For inputs: drive HIGH (then read).
 *
 * @param dev   PCF8574 handle
 * @param pins  Bitmask: 1 = HIGH, 0 = LOW
 * @return I2C_OK on success
 */
i2c_err_t pcf8574_write(pcf8574_t *dev, uint8_t pins);

/**
 * @brief Read all 8 GPIO pins
 *
 * Pins must be driven HIGH before reading (quasi-bidirectional property).
 *
 * @param dev      PCF8574 handle
 * @param[out] pins  Pin states: 1 = HIGH, 0 = LOW
 * @return I2C_OK on success
 */
i2c_err_t pcf8574_read(pcf8574_t *dev, uint8_t *pins);

/**
 * @brief Set a single pin HIGH or LOW
 *
 * @param dev   PCF8574 handle
 * @param pin   Pin number (0–7)
 * @param high  true = HIGH, false = LOW
 * @return I2C_OK on success
 */
i2c_err_t pcf8574_set_pin(pcf8574_t *dev, uint8_t pin, bool high);

/**
 * @brief Read a single pin state
 *
 * @param dev       PCF8574 handle
 * @param pin       Pin number (0–7)
 * @param[out] high true if pin is HIGH
 * @return I2C_OK on success
 */
i2c_err_t pcf8574_get_pin(pcf8574_t *dev, uint8_t pin, bool *high);

#ifdef __cplusplus
}
#endif

#endif /* I2C_DEVICES_H */
