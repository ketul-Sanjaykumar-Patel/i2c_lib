/**
 * @file    example_all_devices.c
 * @brief   Complete example: I2C library with all included device drivers
 *
 * This example shows how to wire everything together on a typical
 * embedded system. It demonstrates:
 *
 *   1. Bus initialization (software bit-bang on any GPIO)
 *   2. Bus scanning (debug/discovery)
 *   3. BME280  → temperature / humidity / pressure
 *   4. MPU-6050 → accelerometer + gyroscope raw data
 *   5. VEML7700 → ambient light in lux
 *   6. SSD1306  → OLED display output
 *   7. INA226   → current and power monitoring
 *   8. DS3231   → real-time clock read/set
 *   9. AT24C32  → EEPROM write/read
 *  10. PCF8574  → GPIO expansion
 *
 * HARDWARE SETUP (example with STM32/Arduino pins):
 * =================================================
 *   SDA → PA8 (or any open-drain capable GPIO)
 *   SCL → PA9
 *   Pull-up resistors: 4.7kΩ to 3.3V on both SDA and SCL
 *
 * ALL DEVICES connected to the same 2-wire bus (different addresses):
 *   0x3C = SSD1306 OLED
 *   0x40 = INA226
 *   0x50 = AT24C32 EEPROM
 *   0x10 = VEML7700
 *   0x20 = PCF8574
 *   0x68 = DS3231 RTC and MPU-6050 IMU (different boards!)
 *   0x76 = BME280
 *
 * NOTE: DS3231 and MPU-6050 both use 0x68 by default.
 *       If using both, move one to 0x69 (change AD0/A0 pin).
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "i2c_devices.h"

/* ============================================================================
 * PLATFORM HAL IMPLEMENTATION
 * Replace these functions with your platform's GPIO API.
 * Examples shown for a generic embedded target.
 * ========================================================================== */

/**
 * Platform-specific GPIO control.
 * Replace with: HAL_GPIO_WritePin / gpio_set_level / digitalWrite / etc.
 */

/** GPIO pin numbers — change for your hardware */
#define SDA_PIN  8  /* PA8 on STM32, GPIO8 on ESP32, pin 8 on Arduino */
#define SCL_PIN  9

/* Simulated GPIO state for demonstration */
static bool s_sda_out = true;
static bool s_scl_out = true;
static bool s_sda_is_output = false;

static void platform_sda_dir(bool output, void *ctx)
{
    (void)ctx;
    s_sda_is_output = output;
    /* Example STM32:
     * GPIO_InitTypeDef g = {.Pin=GPIO_PIN_8, .Mode=output?GPIO_MODE_OUTPUT_OD:GPIO_MODE_INPUT};
     * HAL_GPIO_Init(GPIOA, &g);
     */
}

static void platform_sda_write(bool level, void *ctx)
{
    (void)ctx;
    s_sda_out = level;
    /* Example: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, level ? GPIO_PIN_SET : GPIO_PIN_RESET); */
}

static bool platform_sda_read(void *ctx)
{
    (void)ctx;
    return s_sda_out;  /* In real code: HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET */
}

static void platform_scl_write(bool level, void *ctx)
{
    (void)ctx;
    s_scl_out = level;
    /* Example: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, level ? GPIO_PIN_SET : GPIO_PIN_RESET); */
}

static bool platform_scl_read(void *ctx)
{
    (void)ctx;
    return s_scl_out; /* In real code: HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET */
}

static void platform_delay_us(uint32_t us)
{
    /* Example STM32:  DWT_Delay_us(us);
     * Example Arduino: delayMicroseconds(us);
     * Example Linux:   usleep(us);
     */
    volatile uint32_t count = us * 10;  /* Rough spin delay for demo */
    while (count--) {}
}

static uint32_t platform_millis(void)
{
    /* Example STM32:  return HAL_GetTick();
     * Example Arduino: return millis();
     * Example Linux:   clock_gettime + convert
     */
    static uint32_t ms = 0;
    return ms++;
}

/* ============================================================================
 * HELPER: Print error if not OK
 * ========================================================================== */

static bool check(const char *op, i2c_err_t err)
{
    if (err != I2C_OK) {
        printf("ERROR [%s]: %s\n", op, i2c_err_to_string(err));
        return false;
    }
    return true;
}

/* ============================================================================
 * EXAMPLE: BUS SCAN
 * ========================================================================== */

static void example_bus_scan(i2c_bus_t bus)
{
    printf("\n=== I2C Bus Scan ===\n");

    uint8_t found[112];
    uint8_t count;
    i2c_scan(bus, found, &count);

    if (count == 0) {
        printf("No devices found. Check wiring and pull-ups!\n");
        return;
    }

    printf("Found %u device(s):\n", count);
    for (uint8_t i = 0; i < count; i++) {
        uint8_t addr = found[i];
        /* Print known device names */
        const char *name = "Unknown";
        if (addr == 0x3C || addr == 0x3D) name = "SSD1306 OLED";
        else if (addr == 0x40)            name = "INA226 Power Monitor";
        else if (addr == 0x50)            name = "AT24C32 EEPROM";
        else if (addr == 0x10)            name = "VEML7700 Light Sensor";
        else if (addr == 0x20)            name = "PCF8574 GPIO Expander";
        else if (addr == 0x68)            name = "DS3231 RTC / MPU-6050 IMU";
        else if (addr == 0x69)            name = "MPU-6050 IMU (AD0=HIGH)";
        else if (addr == 0x76 || addr == 0x77) name = "BME280 Env Sensor";
        printf("  0x%02X — %s\n", addr, name);
    }
}

/* ============================================================================
 * EXAMPLE: BME280 — Environment Sensing
 * ========================================================================== */

static void example_bme280(i2c_bus_t bus)
{
    printf("\n=== BME280: Temperature / Humidity / Pressure ===\n");

    bme280_t sensor;
    if (!check("bme280_init", bme280_init(&sensor, bus, BME280_ADDR_LOW))) return;

    int32_t  temp;
    uint32_t press, hum;
    if (!check("bme280_read", bme280_read(&sensor, &temp, &press, &hum))) return;

    printf("  Temperature : %d.%02d °C\n", temp / 100, temp % 100);
    printf("  Pressure    : %u.%02u hPa\n", press / 100, press % 100);
    printf("  Humidity    : %u.%03u %%RH\n", hum / 1024, (hum % 1024) * 1000 / 1024);
}

/* ============================================================================
 * EXAMPLE: MPU-6050 — IMU Data
 * ========================================================================== */

static void example_mpu6050(i2c_bus_t bus)
{
    printf("\n=== MPU-6050: 6-Axis IMU ===\n");

    mpu6050_t imu;
    if (!check("mpu6050_init",
               mpu6050_init(&imu, bus, MPU6050_ADDR_LOW,
                            MPU6050_ACCEL_4G, MPU6050_GYRO_500))) return;

    mpu6050_raw_t raw;
    if (!check("mpu6050_read", mpu6050_read_raw(&imu, &raw))) return;

    /* Convert raw to physical units */
    /* Accel ±4g: 1g = 8192 LSB. Multiply by 1000 for mg (milli-g) */
    int32_t ax_mg = (int32_t)raw.accel_x * 1000 / 8192;
    int32_t ay_mg = (int32_t)raw.accel_y * 1000 / 8192;
    int32_t az_mg = (int32_t)raw.accel_z * 1000 / 8192;

    /* Gyro ±500°/s: 1°/s = 65.5 LSB */
    int32_t gx_dps10 = (int32_t)raw.gyro_x * 10 / 655;
    int32_t gy_dps10 = (int32_t)raw.gyro_y * 10 / 655;
    int32_t gz_dps10 = (int32_t)raw.gyro_z * 10 / 655;

    int32_t temp = mpu6050_raw_to_temp(raw.temp_raw);

    printf("  Accel: X=%d mg  Y=%d mg  Z=%d mg\n", ax_mg, ay_mg, az_mg);
    printf("  Gyro:  X=%d.%d°/s  Y=%d.%d°/s  Z=%d.%d°/s\n",
           gx_dps10 / 10, (gx_dps10 < 0 ? -gx_dps10 : gx_dps10) % 10,
           gy_dps10 / 10, (gy_dps10 < 0 ? -gy_dps10 : gy_dps10) % 10,
           gz_dps10 / 10, (gz_dps10 < 0 ? -gz_dps10 : gz_dps10) % 10);
    printf("  Temp:  %d.%02d °C\n", temp / 100, temp % 100);
}

/* ============================================================================
 * EXAMPLE: VEML7700 — Ambient Light
 * ========================================================================== */

static void example_veml7700(i2c_bus_t bus)
{
    printf("\n=== VEML7700: Ambient Light Sensor ===\n");

    veml7700_t light;
    if (!check("veml7700_init",
               veml7700_init(&light, bus, VEML7700_GAIN_1, VEML7700_IT_100MS))) return;

    float lux;
    if (!check("veml7700_read", veml7700_read_lux(&light, &lux))) return;

    printf("  Illuminance: %.1f lux\n", (double)lux);

    /* Context: typical lux values */
    const char *context;
    if      (lux < 1)     context = "(very dark)";
    else if (lux < 100)   context = "(indoor dim)";
    else if (lux < 500)   context = "(indoor bright)";
    else if (lux < 2000)  context = "(overcast outside)";
    else if (lux < 20000) context = "(daylight)";
    else                  context = "(direct sunlight)";
    printf("  Context    : %s\n", context);
}

/* ============================================================================
 * EXAMPLE: SSD1306 — OLED Display
 * ========================================================================== */

static void example_ssd1306(i2c_bus_t bus)
{
    printf("\n=== SSD1306: 128×64 OLED Display ===\n");

    ssd1306_t display;
    if (!check("ssd1306_init", ssd1306_init(&display, bus, SSD1306_ADDR_LOW))) return;

    /* Draw a simple cross pattern to test display */
    ssd1306_clear(&display);

    /* Draw horizontal line at y=32 */
    for (uint8_t x = 0; x < SSD1306_WIDTH; x++) {
        ssd1306_set_pixel(&display, x, 32, true);
    }

    /* Draw vertical line at x=64 */
    for (uint8_t y = 0; y < SSD1306_HEIGHT; y++) {
        ssd1306_set_pixel(&display, 64, y, true);
    }

    /* Draw border */
    for (uint8_t x = 0; x < SSD1306_WIDTH; x++) {
        ssd1306_set_pixel(&display, x, 0, true);
        ssd1306_set_pixel(&display, x, SSD1306_HEIGHT - 1, true);
    }
    for (uint8_t y = 0; y < SSD1306_HEIGHT; y++) {
        ssd1306_set_pixel(&display, 0, y, true);
        ssd1306_set_pixel(&display, SSD1306_WIDTH - 1, y, true);
    }

    check("ssd1306_flush", ssd1306_flush(&display));
    printf("  Sent cross pattern + border to OLED (1024 bytes)\n");
}

/* ============================================================================
 * EXAMPLE: INA226 — Power Monitoring
 * ========================================================================== */

static void example_ina226(i2c_bus_t bus)
{
    printf("\n=== INA226: Current/Power Monitor ===\n");

    ina226_t pmon;
    /* 100mΩ shunt resistor, max 5A measurement range */
    if (!check("ina226_init",
               ina226_init(&pmon, bus, INA226_ADDR_BASE, 0.1f, 5.0f))) return;

    int32_t  mv, ma;
    uint32_t mw;
    check("ina226_voltage", ina226_read_voltage(&pmon, &mv));
    check("ina226_current", ina226_read_current(&pmon, &ma));
    check("ina226_power",   ina226_read_power(&pmon, &mw));

    printf("  Voltage : %d.%03d V\n", mv / 1000, mv % 1000);
    printf("  Current : %d.%03d A\n", ma / 1000, (ma < 0 ? -ma : ma) % 1000);
    printf("  Power   : %u.%03u W\n", mw / 1000, mw % 1000);
}

/* ============================================================================
 * EXAMPLE: DS3231 — Real-Time Clock
 * ========================================================================== */

static void example_ds3231(i2c_bus_t bus)
{
    printf("\n=== DS3231: Real-Time Clock ===\n");

    ds3231_t rtc;
    if (!check("ds3231_init", ds3231_init(&rtc, bus))) return;

    /* Set time (do this once, then comment out) */
    ds3231_datetime_t set_dt = {
        .seconds = 0,
        .minutes = 30,
        .hours   = 14,
        .day     = 2,      /* Monday */
        .date    = 17,
        .month   = 3,
        .year    = 26      /* 2026 */
    };
    check("ds3231_set_time", ds3231_set_time(&rtc, &set_dt));

    /* Read it back */
    ds3231_datetime_t dt;
    if (!check("ds3231_get_time", ds3231_get_time(&rtc, &dt))) return;

    printf("  Date/Time: 20%02u-%02u-%02u %02u:%02u:%02u\n",
           dt.year, dt.month, dt.date, dt.hours, dt.minutes, dt.seconds);

    int16_t temp_c4;
    check("ds3231_temp", ds3231_read_temp(&rtc, &temp_c4));
    printf("  RTC Temp : %d.%02u °C\n", temp_c4 / 4,
           (uint8_t)((temp_c4 % 4) * 25));
}

/* ============================================================================
 * EXAMPLE: AT24C32 — EEPROM Storage
 * ========================================================================== */

static void example_at24c32(i2c_bus_t bus)
{
    printf("\n=== AT24C32: 32Kbit EEPROM ===\n");

    at24c32_t eeprom;
    if (!check("at24c32_init", at24c32_init(&eeprom, bus, AT24C32_ADDR_BASE))) return;

    /* Write a 16-byte calibration record at address 0 */
    uint8_t cal_data[16] = {
        0xCA, 0xFE,           /* Magic header */
        0x01,                 /* Version */
        0x00,                 /* Reserved */
        0x12, 0x34, 0x56, 0x78, /* Sensor offset (example) */
        0xAB, 0xCD, 0xEF, 0x01, /* Gain value */
        0x00, 0x00, 0x00, 0x00  /* CRC placeholder */
    };

    printf("  Writing 16 bytes to address 0x0000...\n");
    if (!check("at24c32_write", at24c32_write(&eeprom, 0x0000, cal_data, 16))) return;

    /* Read it back and verify */
    uint8_t read_back[16] = {0};
    if (!check("at24c32_read", at24c32_read(&eeprom, 0x0000, read_back, 16))) return;

    bool match = (memcmp(cal_data, read_back, 16) == 0);
    printf("  Read back: %s\n", match ? "OK (data matches)" : "FAIL (mismatch!)");
    printf("  First 4 bytes: 0x%02X 0x%02X 0x%02X 0x%02X\n",
           read_back[0], read_back[1], read_back[2], read_back[3]);
}

/* ============================================================================
 * EXAMPLE: PCF8574 — GPIO Expansion
 * ========================================================================== */

static void example_pcf8574(i2c_bus_t bus)
{
    printf("\n=== PCF8574: 8-bit GPIO Expander ===\n");

    pcf8574_t gpio;
    if (!check("pcf8574_init", pcf8574_init(&gpio, bus, PCF8574_ADDR_BASE))) return;

    /* Toggle LED on pin 0 (example: relay board control) */
    check("pcf8574_pin0_low",  pcf8574_set_pin(&gpio, 0, false));
    printf("  Pin 0 → LOW  (relay energized / LED on)\n");

    check("pcf8574_pin0_high", pcf8574_set_pin(&gpio, 0, true));
    printf("  Pin 0 → HIGH (relay released / LED off)\n");

    /* Read all pins (e.g., read DIP switches on pins 4–7) */
    uint8_t pins;
    check("pcf8574_read", pcf8574_read(&gpio, &pins));
    printf("  Pin states: 0b%u%u%u%u%u%u%u%u\n",
           (pins >> 7) & 1, (pins >> 6) & 1, (pins >> 5) & 1, (pins >> 4) & 1,
           (pins >> 3) & 1, (pins >> 2) & 1, (pins >> 1) & 1, pins & 1);
}

/* ============================================================================
 * EXAMPLE: BUS STATISTICS
 * ========================================================================== */

static void example_stats(i2c_bus_t bus)
{
    printf("\n=== I2C Bus Statistics ===\n");

    i2c_stats_t stats;
    i2c_get_stats(bus, &stats);

    printf("  Transactions : %u\n",   stats.transaction_count);
    printf("  Bytes TX     : %u\n",   stats.tx_count);
    printf("  Bytes RX     : %u\n",   stats.rx_count);
    printf("  Errors       : %u\n",   stats.error_count);
    printf("  NACK (addr)  : %u\n",   stats.nack_count);
    printf("  Timeouts     : %u\n",   stats.timeout_count);
    printf("  Arb lost     : %u\n",   stats.arb_lost_count);
    printf("  Recoveries   : %u\n",   stats.recovery_count);
}

/* ============================================================================
 * MAIN
 * ========================================================================== */

int main(void)
{
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║  Universal I2C Library v%s — Full Demo      ║\n", i2c_version());
    printf("╚══════════════════════════════════════════════╝\n");

    /* ---- Build HAL callbacks for software I2C ---- */
    i2c_hal_t hal = {
        .sda_dir   = platform_sda_dir,
        .sda_write = platform_sda_write,
        .sda_read  = platform_sda_read,
        .scl_write = platform_scl_write,
        .scl_read  = platform_scl_read,
        .delay_us  = platform_delay_us,
        .millis    = platform_millis,
        .user_ctx  = NULL,
    };

    /* ---- Initialize I2C bus ---- */
    i2c_config_t cfg = {
        .mode        = I2C_MODE_SOFTWARE,
        .speed       = I2C_SPEED_FAST,      /* 400 kbps */
        .addr_size   = I2C_ADDR_7BIT,
        .timeout_ms  = 50,
        .hal         = &hal,
    };

    i2c_bus_t bus;
    i2c_err_t err = i2c_init(&bus, &cfg);
    if (err != I2C_OK) {
        printf("Fatal: i2c_init failed: %s\n", i2c_err_to_string(err));
        return 1;
    }
    printf("\nI2C bus initialized at 400kHz (software mode)\n");

    /* ---- Run examples ---- */
    example_bus_scan(bus);
    example_bme280(bus);
    example_mpu6050(bus);
    example_veml7700(bus);
    example_ssd1306(bus);
    example_ina226(bus);
    example_ds3231(bus);
    example_at24c32(bus);
    example_pcf8574(bus);
    example_stats(bus);

    /* ---- Demonstrate bus recovery ---- */
    printf("\n=== Bus Recovery Test ===\n");
    if (i2c_get_state(bus) == I2C_STATE_ERROR) {
        if (i2c_recover(bus) == I2C_OK)
            printf("  Bus recovered successfully.\n");
        else
            printf("  Bus unrecoverable — power cycle needed.\n");
    } else {
        printf("  Bus is healthy (state: IDLE)\n");
    }

    /* ---- Cleanup ---- */
    i2c_deinit(bus);
    printf("\nDone. Bus deinitialized.\n");

    return 0;
}
