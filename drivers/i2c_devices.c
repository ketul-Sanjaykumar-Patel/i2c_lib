/**
 * @file    i2c_devices.c
 * @brief   I2C device driver implementations
 *
 * Implements drivers for: BME280, MPU-6050, VEML7700, SSD1306,
 *                          INA226, DS3231, AT24C32, PCF8574
 */

#include "i2c_devices.h"
#include <string.h>

/* ============================================================================
 * HELPER MACROS
 * ========================================================================== */

/** BCD-to-binary conversion (used by RTC) */
#define BCD2BIN(bcd)    (((bcd) >> 4) * 10 + ((bcd) & 0x0F))
/** Binary-to-BCD conversion */
#define BIN2BCD(bin)    ((((bin) / 10) << 4) | ((bin) % 10))

/** Early-return macro for error propagation */
#define CHECK(x)  do { i2c_err_t _e = (x); if (_e != I2C_OK) return _e; } while(0)

/* ============================================================================
 * BME280 IMPLEMENTATION
 * ========================================================================== */

static i2c_err_t bme280_load_calib(bme280_t *dev)
{
    uint8_t buf[26];
    /* Read first calibration block (0x88–0x9F, 26 bytes) */
    CHECK(i2c_write_reg_read(dev->bus, dev->addr, BME280_REG_CALIB_00, buf, 26));

    dev->calib.dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    dev->calib.dig_T2 = (int16_t) (buf[3] << 8 | buf[2]);
    dev->calib.dig_T3 = (int16_t) (buf[5] << 8 | buf[4]);
    dev->calib.dig_P1 = (uint16_t)(buf[7] << 8 | buf[6]);
    dev->calib.dig_P2 = (int16_t) (buf[9] << 8 | buf[8]);
    dev->calib.dig_P3 = (int16_t) (buf[11] << 8 | buf[10]);
    dev->calib.dig_P4 = (int16_t) (buf[13] << 8 | buf[12]);
    dev->calib.dig_P5 = (int16_t) (buf[15] << 8 | buf[14]);
    dev->calib.dig_P6 = (int16_t) (buf[17] << 8 | buf[16]);
    dev->calib.dig_P7 = (int16_t) (buf[19] << 8 | buf[18]);
    dev->calib.dig_P8 = (int16_t) (buf[21] << 8 | buf[20]);
    dev->calib.dig_P9 = (int16_t) (buf[23] << 8 | buf[22]);
    dev->calib.dig_H1 = buf[25];

    /* Read humidity calibration block (0xE1–0xE7, 7 bytes) */
    CHECK(i2c_write_reg_read(dev->bus, dev->addr, BME280_REG_CALIB_26, buf, 7));

    dev->calib.dig_H2 = (int16_t)(buf[1] << 8 | buf[0]);
    dev->calib.dig_H3 = buf[2];
    dev->calib.dig_H4 = (int16_t)((buf[3] << 4) | (buf[4] & 0x0F));
    dev->calib.dig_H5 = (int16_t)((buf[5] << 4) | (buf[4] >> 4));
    dev->calib.dig_H6 = (int8_t)buf[6];

    return I2C_OK;
}

i2c_err_t bme280_init(bme280_t *dev, i2c_bus_t bus, uint8_t addr)
{
    if (!dev || !bus) return I2C_ERR_INVALID_ARG;

    dev->bus  = bus;
    dev->addr = addr;

    /* Verify chip ID */
    uint8_t chip_id;
    CHECK(i2c_reg_read8(bus, addr, BME280_REG_CHIP_ID, &chip_id));
    if (chip_id != 0x60) return I2C_ERR_PLATFORM;  /* Not a BME280 */

    /* Software reset */
    CHECK(i2c_reg_write8(bus, addr, BME280_REG_RESET, 0xB6));
    /* Wait for reset to complete (~2ms) */

    /* Load factory calibration data */
    CHECK(bme280_load_calib(dev));

    /*
     * Configure:
     *   Humidity oversampling ×1 (must be set BEFORE ctrl_meas)
     *   Temp oversampling ×2, Pressure oversampling ×16, Normal mode
     *   Standby 0.5ms, filter coefficient 16 (good for indoor navigation)
     */
    CHECK(i2c_reg_write8(bus, addr, BME280_REG_CTRL_HUM,  0x01));
    CHECK(i2c_reg_write8(bus, addr, BME280_REG_CONFIG,    0xA0));
    CHECK(i2c_reg_write8(bus, addr, BME280_REG_CTRL_MEAS, 0x57)); /* osrs_t=×2, osrs_p=×16, mode=normal */

    dev->initialized = true;
    return I2C_OK;
}

i2c_err_t bme280_read(bme280_t *dev, int32_t *temp, uint32_t *press, uint32_t *hum)
{
    if (!dev || !dev->initialized) return I2C_ERR_NOT_INIT;

    /* Read 8-byte burst starting at 0xF7 (press MSB → hum LSB) */
    uint8_t raw[8];
    CHECK(i2c_write_reg_read(dev->bus, dev->addr, BME280_REG_PRESS_MSB, raw, 8));

    /* Unpack 20-bit ADC values */
    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] << 8)  | raw[7];

    bme280_calib_t *c = &dev->calib;

    /* === Temperature compensation (Bosch official formula) ===
     * Produces t_fine (used by pressure and humidity formulas too)
     * Output: temperature in units of 0.01°C (DegC × 100)
     */
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)c->dig_T1 << 1))) * c->dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)c->dig_T1) *
              ((adc_T >> 4) - (int32_t)c->dig_T1)) >> 12) * c->dig_T3) >> 14;
    c->t_fine = var1 + var2;

    if (temp) *temp = (c->t_fine * 5 + 128) >> 8;

    /* === Pressure compensation (Bosch formula, 64-bit integer version) ===
     * Output: pressure in Pa (e.g., 101325 Pa = 1013.25 hPa = sea level)
     */
    if (press) {
        int64_t p;
        int64_t v1 = (int64_t)c->t_fine - 128000;
        int64_t v2 = v1 * v1 * (int64_t)c->dig_P6;
        v2 += (v1 * (int64_t)c->dig_P5) << 17;
        v2 += (int64_t)c->dig_P4 << 35;
        v1  = ((v1 * v1 * (int64_t)c->dig_P3) >> 8) +
              ((v1 * (int64_t)c->dig_P2) << 12);
        v1  = ((((int64_t)1 << 47) + v1) * (int64_t)c->dig_P1) >> 33;
        if (v1 == 0) {
            *press = 0;  /* Avoid division by zero */
        } else {
            p = (((int64_t)1048576 - adc_P) << 31) - v2;
            p = (p * 3125) / v1;
            v1 = ((int64_t)c->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
            v2 = ((int64_t)c->dig_P8 * p) >> 19;
            p = ((p + v1 + v2) >> 8) + ((int64_t)c->dig_P7 << 4);
            *press = (uint32_t)(p >> 8);  /* Divide by 256 to get Pa */
        }
    }

    /* === Humidity compensation (Bosch formula) ===
     * Output: relative humidity × 1024 (e.g., 51200 = 50.0% RH)
     */
    if (hum) {
        int32_t h = c->t_fine - 76800;
        h = (((((adc_H << 14) - ((int32_t)c->dig_H4 << 20) -
                ((int32_t)c->dig_H5 * h)) + 16384) >> 15) *
             (((((((h * (int32_t)c->dig_H6) >> 10) *
                 (((h * (int32_t)c->dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
               (int32_t)c->dig_H2 + 8192) >> 14));
        h -= (((((h >> 15) * (h >> 15)) >> 7) * (int32_t)c->dig_H1) >> 4);
        h = (h < 0) ? 0 : (h > 419430400) ? 419430400 : h;
        *hum = (uint32_t)(h >> 12);
    }

    return I2C_OK;
}

/* ============================================================================
 * MPU-6050 IMPLEMENTATION
 * ========================================================================== */

i2c_err_t mpu6050_init(mpu6050_t *dev, i2c_bus_t bus, uint8_t addr,
                        mpu6050_accel_range_t accel_range,
                        mpu6050_gyro_range_t gyro_range)
{
    if (!dev || !bus) return I2C_ERR_INVALID_ARG;

    dev->bus         = bus;
    dev->addr        = addr;
    dev->accel_range = accel_range;
    dev->gyro_range  = gyro_range;

    /* Verify WHO_AM_I register */
    uint8_t who;
    CHECK(i2c_reg_read8(bus, addr, MPU6050_REG_WHO_AM_I, &who));
    if (who != MPU6050_WHO_AM_I_VAL) return I2C_ERR_PLATFORM;

    /* Wake up: clear SLEEP bit (bit 6), select PLL with X-axis gyro ref (recommended) */
    CHECK(i2c_reg_write8(bus, addr, MPU6050_REG_PWR_MGMT_1, 0x01));

    /* Configure digital low-pass filter: ~44Hz BW (balanced noise/latency) */
    CHECK(i2c_reg_write8(bus, addr, MPU6050_REG_CONFIG, 0x03));

    /* Set sample rate: 1kHz / (1 + SMPLRT_DIV) = 200Hz */
    CHECK(i2c_reg_write8(bus, addr, MPU6050_REG_SMPLRT_DIV, 0x04));

    /* Configure accelerometer full-scale range */
    CHECK(i2c_reg_write8(bus, addr, MPU6050_REG_ACCEL_CONFIG, (uint8_t)accel_range));

    /* Configure gyroscope full-scale range */
    CHECK(i2c_reg_write8(bus, addr, MPU6050_REG_GYRO_CONFIG, (uint8_t)gyro_range));

    dev->initialized = true;
    return I2C_OK;
}

i2c_err_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw)
{
    if (!dev || !dev->initialized || !raw) return I2C_ERR_INVALID_ARG;

    /*
     * Burst read 14 bytes starting at ACCEL_XOUT_H (0x3B):
     *   [0..1]  = ACCEL_X  [2..3]  = ACCEL_Y  [4..5]  = ACCEL_Z
     *   [6..7]  = TEMP     [8..9]  = GYRO_X   [10..11] = GYRO_Y
     *   [12..13] = GYRO_Z
     *
     * CRITICAL: Always read all 14 bytes in one transaction to ensure
     * all values are from the same internal sample (data consistency).
     */
    uint8_t buf[14];
    CHECK(i2c_write_reg_read(dev->bus, dev->addr, MPU6050_REG_ACCEL_XOUT_H, buf, 14));

    raw->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    raw->temp_raw= (int16_t)((buf[6]  << 8) | buf[7]);
    raw->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return I2C_OK;
}

int32_t mpu6050_raw_to_temp(int16_t raw_temp)
{
    /* Datasheet formula: Temp_degC = raw / 340.0 + 36.53
     * Integer version (×100 for centidegrees):
     *   temp_100 = (raw * 100) / 340 + 3653
     */
    return ((int32_t)raw_temp * 100) / 340 + 3653;
}

/* ============================================================================
 * VEML7700 IMPLEMENTATION
 * ========================================================================== */

/**
 * @brief Resolution (lux per count) lookup table
 * Indexed by [gain][integration_time]
 * Source: VEML7700 Application Note, Table 1
 */
static const float veml7700_res_lut[4][6] = {
    /* IT: 25ms    50ms    100ms   200ms   400ms   800ms */
    {0.2304f, 0.1152f, 0.0576f, 0.0288f, 0.0144f, 0.0072f}, /* gain x1   */
    {0.1152f, 0.0576f, 0.0288f, 0.0144f, 0.0072f, 0.0036f}, /* gain x2   */
    {1.8432f, 0.9216f, 0.4608f, 0.2304f, 0.1152f, 0.0576f}, /* gain x1/8 */
    {0.9216f, 0.4608f, 0.2304f, 0.1152f, 0.0576f, 0.0288f}, /* gain x1/4 */
};

i2c_err_t veml7700_init(veml7700_t *dev, i2c_bus_t bus,
                         veml7700_gain_t gain, veml7700_it_t it)
{
    if (!dev || !bus) return I2C_ERR_INVALID_ARG;

    dev->bus  = bus;
    dev->gain = gain;
    dev->it   = it;

    /* Build config register: [15:13]=gain, [9:6]=IT, [3]=INT_EN, [0]=SD */
    uint16_t conf = ((uint16_t)gain << 11) | ((uint16_t)it << 6);
    /* conf bit 0 = 0: power ON (SD = 0 means powered) */

    /* VEML7700 uses 16-bit LE register writes */
    uint8_t buf[2] = { (uint8_t)(conf & 0xFF), (uint8_t)(conf >> 8) };
    CHECK(i2c_write_reg(bus, VEML7700_ADDR, VEML7700_REG_ALS_CONF, buf, 2));

    /* Look up resolution for chosen gain/IT combination */
    int it_idx = 0;
    switch (it) {
        case VEML7700_IT_25MS:  it_idx = 0; break;
        case VEML7700_IT_50MS:  it_idx = 1; break;
        case VEML7700_IT_100MS: it_idx = 2; break;
        case VEML7700_IT_200MS: it_idx = 3; break;
        case VEML7700_IT_400MS: it_idx = 4; break;
        case VEML7700_IT_800MS: it_idx = 5; break;
        default: it_idx = 2;
    }
    dev->resolution = veml7700_res_lut[(int)gain][it_idx];
    dev->initialized = true;

    return I2C_OK;
}

i2c_err_t veml7700_read_lux(veml7700_t *dev, float *lux)
{
    if (!dev || !dev->initialized || !lux) return I2C_ERR_INVALID_ARG;

    uint16_t raw;
    CHECK(i2c_reg_read16_le(dev->bus, VEML7700_ADDR, VEML7700_REG_ALS, &raw));

    float result = raw * dev->resolution;

    /*
     * Non-linear correction for high lux values (>1000 lux).
     * From VEML7700 application note: correction polynomial.
     * Improves accuracy from ±10% to ±3% at high light levels.
     */
    if (result > 1000.0f) {
        result = (6.0135e-13f * result * result * result * result)
               - (9.3924e-9f  * result * result * result)
               + (8.1488e-5f  * result * result)
               + (1.0023f     * result);
    }

    *lux = result;
    return I2C_OK;
}

/* ============================================================================
 * SSD1306 IMPLEMENTATION
 * ========================================================================== */

/** Standard SSD1306 initialization sequence */
static const uint8_t ssd1306_init_cmds[] = {
    SSD1306_CMD_DISPLAY_OFF,        /* 0. Turn off display during init */
    SSD1306_CMD_SET_CLK_DIV, 0x80, /* 1. Fosc = 8, divide ratio = 1 */
    SSD1306_CMD_SET_MUX_RATIO, 63, /* 2. 1/64 duty cycle (64 rows) */
    SSD1306_CMD_SET_OFFSET, 0x00,  /* 3. No vertical offset */
    SSD1306_CMD_SET_START_LINE,    /* 4. Start line = 0 */
    SSD1306_CMD_CHARGE_PUMP, 0x14, /* 5. Enable charge pump (3.3V supply) */
    SSD1306_CMD_MEM_ADDR_MODE, 0x00, /* 6. Horizontal addressing mode */
    SSD1306_CMD_REMAP_SEG_127,     /* 7. Mirror X (column 127→SEG0) */
    SSD1306_CMD_COM_SCAN_DEC,      /* 8. Mirror Y (scan from bottom) */
    SSD1306_CMD_SET_COM_PINS, 0x12,/* 9. Alt COM pin config, no remap */
    SSD1306_CMD_SET_CONTRAST, 0xCF,/* 10. Medium-high contrast */
    SSD1306_CMD_PRECHARGE, 0xF1,   /* 11. Phase 1=15 DCLK, Phase 2=1 */
    SSD1306_CMD_SET_VCOM_DESEL,0x40,/*12. VCOMH = 0.77 × Vcc */
    SSD1306_CMD_NORMAL_DISPLAY,    /* 13. Normal display (not ALL ON) */
    SSD1306_CMD_INVERT_OFF,        /* 14. Non-inverted */
    SSD1306_CMD_DISPLAY_ON,        /* 15. Turn on */
};

/**
 * @brief Send a command byte to SSD1306
 *
 * I2C command format: [0x3C] [0x00 = control: all commands] [cmd] [STOP]
 */
static i2c_err_t ssd1306_cmd(ssd1306_t *dev, uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CTRL_CMD, cmd };
    return i2c_write(dev->bus, dev->addr, buf, 2);
}

i2c_err_t ssd1306_init(ssd1306_t *dev, i2c_bus_t bus, uint8_t addr)
{
    if (!dev || !bus) return I2C_ERR_INVALID_ARG;

    dev->bus  = bus;
    dev->addr = addr;
    memset(dev->framebuf, 0, SSD1306_BUF_SIZE);

    /* Send initialization sequence */
    for (size_t i = 0; i < sizeof(ssd1306_init_cmds); i++) {
        CHECK(ssd1306_cmd(dev, ssd1306_init_cmds[i]));
    }

    dev->initialized = true;
    return I2C_OK;
}

i2c_err_t ssd1306_display_on(ssd1306_t *dev, bool on)
{
    if (!dev || !dev->initialized) return I2C_ERR_NOT_INIT;
    return ssd1306_cmd(dev, on ? SSD1306_CMD_DISPLAY_ON : SSD1306_CMD_DISPLAY_OFF);
}

void ssd1306_clear(ssd1306_t *dev)
{
    if (dev) memset(dev->framebuf, 0, SSD1306_BUF_SIZE);
}

void ssd1306_set_pixel(ssd1306_t *dev, uint8_t x, uint8_t y, bool on)
{
    if (!dev || x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    /* SSD1306 memory layout: pages of 8 rows
     * byte index = page * 128 + col, bit = row within page */
    uint16_t idx = (y / 8) * SSD1306_WIDTH + x;
    uint8_t  bit = 1 << (y % 8);

    if (on) dev->framebuf[idx] |=  bit;
    else    dev->framebuf[idx] &= ~bit;
}

i2c_err_t ssd1306_flush(ssd1306_t *dev)
{
    if (!dev || !dev->initialized) return I2C_ERR_NOT_INIT;

    /* Set write window: all columns (0–127), all pages (0–7) */
    CHECK(ssd1306_cmd(dev, SSD1306_CMD_COL_ADDR));
    CHECK(ssd1306_cmd(dev, 0));    /* Column start */
    CHECK(ssd1306_cmd(dev, 127)); /* Column end */
    CHECK(ssd1306_cmd(dev, SSD1306_CMD_PAGE_ADDR));
    CHECK(ssd1306_cmd(dev, 0));    /* Page start */
    CHECK(ssd1306_cmd(dev, 7));   /* Page end */

    /*
     * Send 1024 bytes of pixel data.
     * I2C data format: [0x3C] [0x40 = control: data stream] [1024 bytes] [STOP]
     *
     * We prepend the 0x40 control byte to the frame buffer area by
     * writing it as a single byte header, then the buffer.
     * (Batch using a local transmission to avoid extra malloc)
     */
    /* Use low-level API for a combined transfer */
    i2c_err_t err;
    err = i2c_start(dev->bus, dev->addr, false);
    if (err != I2C_OK) return err;

    err = i2c_write_byte(dev->bus, SSD1306_CTRL_DATA);
    if (err != I2C_OK) { i2c_stop(dev->bus); return err; }

    for (int i = 0; i < SSD1306_BUF_SIZE; i++) {
        err = i2c_write_byte(dev->bus, dev->framebuf[i]);
        if (err != I2C_OK) { i2c_stop(dev->bus); return err; }
    }

    return i2c_stop(dev->bus);
}

i2c_err_t ssd1306_set_contrast(ssd1306_t *dev, uint8_t contrast)
{
    if (!dev || !dev->initialized) return I2C_ERR_NOT_INIT;
    CHECK(ssd1306_cmd(dev, SSD1306_CMD_SET_CONTRAST));
    return ssd1306_cmd(dev, contrast);
}

/* ============================================================================
 * INA226 IMPLEMENTATION
 * ========================================================================== */

i2c_err_t ina226_init(ina226_t *dev, i2c_bus_t bus, uint8_t addr,
                       float r_shunt_ohm, float max_amps)
{
    if (!dev || !bus || r_shunt_ohm <= 0.0f || max_amps <= 0.0f)
        return I2C_ERR_INVALID_ARG;

    dev->bus  = bus;
    dev->addr = addr;

    /* Verify manufacturer ID */
    uint16_t mfr;
    CHECK(i2c_reg_read16_be(bus, addr, INA226_REG_MFR_ID, &mfr));
    if (mfr != 0x5449) return I2C_ERR_PLATFORM;  /* Not TI INA226 */

    /*
     * Calibration:
     *   current_LSB = max_amps / 2^15  (smallest representable current)
     *   Cal = 0.00512 / (current_LSB × r_shunt)
     *
     * Example: 100mΩ shunt, 5A max:
     *   current_LSB = 5 / 32768 ≈ 0.0001526 A/LSB = 152.6 µA/LSB
     *   Cal = 0.00512 / (0.0001526 × 0.1) ≈ 335
     */
    dev->current_lsb = max_amps / 32768.0f;
    dev->power_lsb   = dev->current_lsb * 25.0f;

    uint16_t cal = (uint16_t)(0.00512f / (dev->current_lsb * r_shunt_ohm));
    CHECK(i2c_reg_write16_be(bus, addr, INA226_REG_CALIB, cal));

    /*
     * Config register:
     *   Mode = Shunt+Bus continuous (111)
     *   VBUS CT = 1.1ms (100)
     *   VSH CT = 1.1ms (100)
     *   AVG = 16 samples (011)
     */
    CHECK(i2c_reg_write16_be(bus, addr, INA226_REG_CONFIG, 0x4327));

    dev->initialized = true;
    return I2C_OK;
}

i2c_err_t ina226_read_voltage(ina226_t *dev, int32_t *mv)
{
    if (!dev || !dev->initialized || !mv) return I2C_ERR_INVALID_ARG;
    uint16_t raw;
    CHECK(i2c_reg_read16_be(dev->bus, dev->addr, INA226_REG_BUS_V, &raw));
    /* Bus voltage LSB = 1.25mV, result × 1.25 = result + result/4 */
    *mv = (int32_t)raw + ((int32_t)raw >> 2);  /* ≈ ×1.25 in integer */
    return I2C_OK;
}

i2c_err_t ina226_read_current(ina226_t *dev, int32_t *ma)
{
    if (!dev || !dev->initialized || !ma) return I2C_ERR_INVALID_ARG;
    int16_t raw;
    CHECK(i2c_reg_read16_be(dev->bus, dev->addr, INA226_REG_CURRENT, (uint16_t*)&raw));
    /* current = raw × current_LSB, convert to milliamps */
    *ma = (int32_t)(raw * dev->current_lsb * 1000.0f);
    return I2C_OK;
}

i2c_err_t ina226_read_power(ina226_t *dev, uint32_t *mw)
{
    if (!dev || !dev->initialized || !mw) return I2C_ERR_INVALID_ARG;
    uint16_t raw;
    CHECK(i2c_reg_read16_be(dev->bus, dev->addr, INA226_REG_POWER, &raw));
    /* power = raw × power_LSB, convert to milliwatts */
    *mw = (uint32_t)(raw * dev->power_lsb * 1000.0f);
    return I2C_OK;
}

/* ============================================================================
 * DS3231 IMPLEMENTATION
 * ========================================================================== */

i2c_err_t ds3231_init(ds3231_t *dev, i2c_bus_t bus)
{
    if (!dev || !bus) return I2C_ERR_INVALID_ARG;
    dev->bus = bus;

    /* Clear oscillator stop flag (OSF) in status register
     * OSF is set when power was lost — timestamps after this are invalid */
    CHECK(i2c_reg_clear_bits(bus, DS3231_ADDR, DS3231_REG_STATUS, 0x80));

    /* Configure: enable oscillator on battery, disable 32kHz output, no alarm */
    CHECK(i2c_reg_write8(bus, DS3231_ADDR, DS3231_REG_CTRL, 0x1C));

    dev->initialized = true;
    return I2C_OK;
}

i2c_err_t ds3231_set_time(ds3231_t *dev, const ds3231_datetime_t *dt)
{
    if (!dev || !dev->initialized || !dt) return I2C_ERR_INVALID_ARG;

    uint8_t buf[7] = {
        BIN2BCD(dt->seconds),
        BIN2BCD(dt->minutes),
        BIN2BCD(dt->hours),    /* 24-hour format (bit 6 = 0) */
        BIN2BCD(dt->day),
        BIN2BCD(dt->date),
        BIN2BCD(dt->month),
        BIN2BCD(dt->year),
    };

    return i2c_write_reg(dev->bus, DS3231_ADDR, DS3231_REG_SECONDS, buf, 7);
}

i2c_err_t ds3231_get_time(ds3231_t *dev, ds3231_datetime_t *dt)
{
    if (!dev || !dev->initialized || !dt) return I2C_ERR_INVALID_ARG;

    uint8_t buf[7];
    CHECK(i2c_write_reg_read(dev->bus, DS3231_ADDR, DS3231_REG_SECONDS, buf, 7));

    dt->seconds = BCD2BIN(buf[0] & 0x7F);
    dt->minutes = BCD2BIN(buf[1] & 0x7F);
    dt->hours   = BCD2BIN(buf[2] & 0x3F);  /* Mask out 12/24hr bit */
    dt->day     = BCD2BIN(buf[3] & 0x07);
    dt->date    = BCD2BIN(buf[4] & 0x3F);
    dt->month   = BCD2BIN(buf[5] & 0x1F);  /* Mask out century bit */
    dt->year    = BCD2BIN(buf[6]);

    return I2C_OK;
}

i2c_err_t ds3231_read_temp(ds3231_t *dev, int16_t *temp_c4)
{
    if (!dev || !dev->initialized || !temp_c4) return I2C_ERR_INVALID_ARG;

    uint8_t buf[2];
    CHECK(i2c_write_reg_read(dev->bus, DS3231_ADDR, DS3231_REG_TEMP_MSB, buf, 2));

    /* MSB: signed integer °C, LSB: bits [7:6] = fractional 0.25°C increments */
    int16_t msb = (int8_t)buf[0];
    uint8_t lsb = buf[1] >> 6;

    *temp_c4 = (msb * 4) + lsb;  /* Units: 0.25°C (divide by 4 for °C) */

    return I2C_OK;
}

/* ============================================================================
 * AT24C32 IMPLEMENTATION
 * ========================================================================== */

i2c_err_t at24c32_init(at24c32_t *dev, i2c_bus_t bus, uint8_t addr)
{
    if (!dev || !bus) return I2C_ERR_INVALID_ARG;

    dev->bus  = bus;
    dev->addr = addr;

    /* Verify device is present */
    if (!i2c_device_present(bus, addr)) return I2C_ERR_NACK_ADDR;

    dev->initialized = true;
    return I2C_OK;
}

i2c_err_t at24c32_read(at24c32_t *dev, uint16_t mem_addr, uint8_t *data, size_t len)
{
    if (!dev || !dev->initialized || !data) return I2C_ERR_INVALID_ARG;
    if (mem_addr + len > AT24C32_CAPACITY)  return I2C_ERR_OVERFLOW;

    /*
     * AT24C32 read sequence:
     *   START | ADDR W | ACK | addr_MSB | ACK | addr_LSB | ACK |
     *   RSTART | ADDR R | ACK | data... | NACK | STOP
     */
    i2c_err_t err;
    err = i2c_start(dev->bus, dev->addr, false);
    if (err != I2C_OK) return err;

    err = i2c_write_byte(dev->bus, (uint8_t)(mem_addr >> 8));
    if (err != I2C_OK) { i2c_stop(dev->bus); return err; }

    err = i2c_write_byte(dev->bus, (uint8_t)(mem_addr & 0xFF));
    if (err != I2C_OK) { i2c_stop(dev->bus); return err; }

    err = i2c_restart(dev->bus, dev->addr, true);
    if (err != I2C_OK) return err;

    for (size_t i = 0; i < len; i++) {
        err = i2c_read_byte(dev->bus, &data[i], (i < len - 1));
        if (err != I2C_OK) { i2c_stop(dev->bus); return err; }
    }

    return i2c_stop(dev->bus);
}

i2c_err_t at24c32_write(at24c32_t *dev, uint16_t mem_addr,
                         const uint8_t *data, size_t len)
{
    if (!dev || !dev->initialized || !data) return I2C_ERR_INVALID_ARG;
    if (mem_addr + len > AT24C32_CAPACITY)  return I2C_ERR_OVERFLOW;

    size_t written = 0;

    while (written < len) {
        /* Calculate how many bytes fit in the current page */
        uint16_t curr_addr = mem_addr + written;
        uint8_t  page_off  = curr_addr % AT24C32_PAGE_SIZE;
        size_t   page_rem  = AT24C32_PAGE_SIZE - page_off;
        size_t   chunk     = (len - written < page_rem) ? (len - written) : page_rem;

        /* Write chunk */
        i2c_err_t err;
        err = i2c_start(dev->bus, dev->addr, false);
        if (err != I2C_OK) return err;

        err = i2c_write_byte(dev->bus, (uint8_t)(curr_addr >> 8));
        if (err != I2C_OK) { i2c_stop(dev->bus); return err; }
        err = i2c_write_byte(dev->bus, (uint8_t)(curr_addr & 0xFF));
        if (err != I2C_OK) { i2c_stop(dev->bus); return err; }

        for (size_t i = 0; i < chunk; i++) {
            err = i2c_write_byte(dev->bus, data[written + i]);
            if (err != I2C_OK) { i2c_stop(dev->bus); return err; }
        }

        i2c_stop(dev->bus);

        /* Wait for internal write cycle to complete.
         * AT24C32 typically finishes in 3–5ms, max 10ms.
         * Poll with ACK check (faster than fixed delay). */
        uint32_t attempts = 0;
        while (!i2c_device_present(dev->bus, dev->addr)) {
            if (++attempts > 20) return I2C_ERR_TIMEOUT;
        }

        written += chunk;
    }

    return I2C_OK;
}

/* ============================================================================
 * PCF8574 IMPLEMENTATION
 * ========================================================================== */

i2c_err_t pcf8574_init(pcf8574_t *dev, i2c_bus_t bus, uint8_t addr)
{
    if (!dev || !bus) return I2C_ERR_INVALID_ARG;

    dev->bus       = bus;
    dev->addr      = addr;
    dev->out_state = 0xFF;  /* All pins HIGH (input/idle state) */

    /* Verify device is present, then set all pins HIGH */
    CHECK(pcf8574_write(dev, 0xFF));

    dev->initialized = true;
    return I2C_OK;
}

i2c_err_t pcf8574_write(pcf8574_t *dev, uint8_t pins)
{
    if (!dev) return I2C_ERR_INVALID_ARG;

    /* PCF8574 write: single byte = pin states */
    i2c_err_t err = i2c_write(dev->bus, dev->addr, &pins, 1);
    if (err == I2C_OK) dev->out_state = pins;
    return err;
}

i2c_err_t pcf8574_read(pcf8574_t *dev, uint8_t *pins)
{
    if (!dev || !pins) return I2C_ERR_INVALID_ARG;

    /* PCF8574 read: single byte = all pin states */
    return i2c_read(dev->bus, dev->addr, pins, 1);
}

i2c_err_t pcf8574_set_pin(pcf8574_t *dev, uint8_t pin, bool high)
{
    if (!dev || pin > 7) return I2C_ERR_INVALID_ARG;

    uint8_t state = dev->out_state;
    if (high) state |=  (1 << pin);
    else      state &= ~(1 << pin);

    return pcf8574_write(dev, state);
}

i2c_err_t pcf8574_get_pin(pcf8574_t *dev, uint8_t pin, bool *high)
{
    if (!dev || !high || pin > 7) return I2C_ERR_INVALID_ARG;

    uint8_t pins;
    CHECK(pcf8574_read(dev, &pins));
    *high = (pins >> pin) & 0x01;
    return I2C_OK;
}
