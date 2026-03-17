/**
 * @file    i2c.c
 * @brief   Universal I2C Driver — Software (bit-bang) implementation
 *
 * This file implements the core I2C protocol in pure software using GPIO
 * callbacks. It works on ANY microcontroller that can:
 *   - Toggle a GPIO pin HIGH/LOW
 *   - Read a GPIO pin state
 *   - Provide microsecond delays
 *
 * HOW BIT-BANG I2C WORKS:
 * ========================
 * The master manually toggles SCL and SDA according to I2C timing rules:
 *
 *    SDA: ‾‾\__/‾‾\__/‾‾\__/‾‾\__/‾‾\__/‾‾\__/‾‾\__/‾‾\__/‾‾\___/‾‾‾
 *    SCL: ___/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_______________________
 *         | S | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |ACK|        STOP
 *
 * Timing (for 100kHz standard mode):
 *   - Half-period (t_half) = 5µs
 *   - SCL HIGH and LOW each hold for t_half
 *   - SDA must be stable before SCL rises
 *
 * CLOCK STRETCHING:
 * A slave can hold SCL LOW to pause the master. We detect this by reading
 * SCL back after driving it HIGH and waiting if it's still LOW.
 *
 * OPEN-DRAIN EMULATION:
 * Since most MCU GPIO can be push-pull or open-drain, we emulate open-drain
 * by: driving LOW = push LOW, driving HIGH = set as input (let pull-up do it)
 * This prevents bus contention and enables clock stretching detection.
 */

#include "i2c.h"
#include <stdlib.h>
#include <string.h>

/* ============================================================================
 * INTERNAL STRUCTURES
 * ========================================================================== */

/**
 * @brief Internal I2C bus instance
 *
 * One per bus handle. Contains configuration, state, statistics, and
 * timing parameters computed from the requested speed.
 */
struct i2c_bus_s {
    i2c_config_t    cfg;        /**< User configuration (copied on init) */
    i2c_state_t     state;      /**< Current bus state */
    i2c_stats_t     stats;      /**< Accumulated statistics */
    uint32_t        t_half_us;  /**< Half clock period in microseconds */
    bool            initialized;/**< Safety flag: true after successful init */

    /* Computed timing parameters */
    uint32_t        t_su_dat;   /**< SDA setup time (µs) before SCL rise */
    uint32_t        t_hd_dat;   /**< SDA hold time (µs) after SCL fall */
    uint32_t        t_su_sta;   /**< Setup time for (repeated) START */
    uint32_t        t_hd_sta;   /**< Hold time for START condition */
    uint32_t        t_su_sto;   /**< Setup time for STOP condition */
    uint32_t        t_buf;      /**< Bus free time between STOP and START */
};

/* ============================================================================
 * STATIC STORAGE
 * ========================================================================== */

/** Pool of bus instances — avoids heap fragmentation on embedded targets */
static struct i2c_bus_s s_bus_pool[I2C_MAX_INSTANCES];
static bool             s_bus_used[I2C_MAX_INSTANCES];

/* ============================================================================
 * TIMING HELPERS
 * ========================================================================== */

/**
 * @brief Compute timing parameters from clock speed
 *
 * I2C timing specs (from NXP UM10204 rev7):
 *   Standard (100kHz): t_HD;STA ≥ 4.0µs, t_SU;DAT ≥ 250ns, t_BUF ≥ 4.7µs
 *   Fast    (400kHz): t_HD;STA ≥ 0.6µs, t_SU;DAT ≥ 100ns, t_BUF ≥ 1.3µs
 *   Fast+   (1MHz):   t_HD;STA ≥ 0.26µs, t_SU;DAT ≥ 50ns,  t_BUF ≥ 0.5µs
 *
 * We round up to whole microseconds for software implementation safety.
 */
static void compute_timing(struct i2c_bus_s *bus)
{
    uint32_t hz = (uint32_t)bus->cfg.speed;

    /* Half-period: 1/speed / 2, in microseconds */
    bus->t_half_us = (1000000UL / hz) / 2;
    if (bus->t_half_us == 0) bus->t_half_us = 1;

    if (hz <= 100000) {
        /* Standard mode */
        bus->t_su_dat  = 1;     /* 250ns → round to 1µs */
        bus->t_hd_dat  = 0;
        bus->t_su_sta  = 5;
        bus->t_hd_sta  = 4;
        bus->t_su_sto  = 4;
        bus->t_buf     = 5;
    } else if (hz <= 400000) {
        /* Fast mode */
        bus->t_su_dat  = 1;
        bus->t_hd_dat  = 0;
        bus->t_su_sta  = 1;
        bus->t_hd_sta  = 1;
        bus->t_su_sto  = 1;
        bus->t_buf     = 2;
    } else {
        /* Fast mode plus or high speed — minimal delays */
        bus->t_su_dat  = 0;
        bus->t_hd_dat  = 0;
        bus->t_su_sta  = 1;
        bus->t_hd_sta  = 1;
        bus->t_su_sto  = 1;
        bus->t_buf     = 1;
    }
}

/* ============================================================================
 * LOW-LEVEL PIN CONTROL
 * Inline helpers to drive SDA and SCL through HAL callbacks.
 * Open-drain emulation: HIGH = input (let pull-up), LOW = output low
 * ========================================================================== */

static inline void sda_high(struct i2c_bus_s *b)
{
    b->cfg.hal->sda_dir(false, b->cfg.hal->user_ctx);   /* input = release */
}

static inline void sda_low(struct i2c_bus_s *b)
{
    b->cfg.hal->sda_write(false, b->cfg.hal->user_ctx); /* drive LOW */
    b->cfg.hal->sda_dir(true, b->cfg.hal->user_ctx);    /* output */
}

static inline bool sda_read(struct i2c_bus_s *b)
{
    return b->cfg.hal->sda_read(b->cfg.hal->user_ctx);
}

static inline void scl_high(struct i2c_bus_s *b)
{
    b->cfg.hal->scl_write(true, b->cfg.hal->user_ctx);
}

static inline void scl_low(struct i2c_bus_s *b)
{
    b->cfg.hal->scl_write(false, b->cfg.hal->user_ctx);
}

static inline bool scl_read(struct i2c_bus_s *b)
{
    return b->cfg.hal->scl_read(b->cfg.hal->user_ctx);
}

static inline void delay(struct i2c_bus_s *b, uint32_t us)
{
    if (us > 0)
        b->cfg.hal->delay_us(us);
}

/* ============================================================================
 * SCL CLOCK PULSE WITH CLOCK STRETCH SUPPORT
 * ========================================================================== */

/**
 * @brief Drive SCL HIGH, wait for slave to release it, then drive LOW
 *
 * Handles clock stretching: slave may hold SCL LOW to pause the master.
 * We spin-wait up to timeout_ms for SCL to go HIGH.
 *
 * @return I2C_OK, or I2C_ERR_CLK_STRETCH on timeout
 */
static i2c_err_t scl_clock_pulse(struct i2c_bus_s *b)
{
    uint32_t start;

    /* SCL goes HIGH */
    scl_high(b);

    /* Wait for clock stretch — slave may hold SCL LOW */
    if (b->cfg.hal->scl_read) {
        uint32_t timeout_ms = b->cfg.timeout_ms;
        start = b->cfg.hal->millis();
        while (!scl_read(b)) {
            if ((b->cfg.hal->millis() - start) >= timeout_ms) {
                b->stats.timeout_count++;
                return I2C_ERR_CLK_STRETCH;
            }
        }
    }

    delay(b, b->t_half_us);   /* Hold SCL HIGH for half period */
    scl_low(b);               /* SCL falls */
    delay(b, b->t_half_us);   /* Hold SCL LOW for half period */

    return I2C_OK;
}

/* ============================================================================
 * BIT-LEVEL I2C PRIMITIVES
 * ========================================================================== */

/**
 * @brief Generate START condition
 *
 * Assumes bus is IDLE (both SDA and SCL HIGH).
 * START: SDA HIGH→LOW while SCL is HIGH
 *
 *   SDA: ‾‾‾‾\___
 *   SCL: ‾‾‾‾‾\__
 *          ^ START here
 */
static void bb_start(struct i2c_bus_s *b)
{
    sda_high(b);
    scl_high(b);
    delay(b, b->t_buf);      /* Bus free time */
    sda_low(b);              /* SDA falls while SCL HIGH = START */
    delay(b, b->t_hd_sta);
    scl_low(b);              /* Bring SCL low to prepare for data */
    delay(b, b->t_half_us);
}

/**
 * @brief Generate REPEATED START condition
 *
 * Similar to START but SCL is LOW beforehand.
 * SCL must go HIGH before SDA transitions.
 */
static void bb_restart(struct i2c_bus_s *b)
{
    sda_high(b);
    delay(b, b->t_su_sta);
    scl_high(b);
    delay(b, b->t_su_sta);
    sda_low(b);              /* START: SDA HIGH→LOW while SCL HIGH */
    delay(b, b->t_hd_sta);
    scl_low(b);
    delay(b, b->t_half_us);
}

/**
 * @brief Generate STOP condition
 *
 * STOP: SDA LOW→HIGH while SCL HIGH
 *
 *   SDA: ___/‾‾‾
 *   SCL: __/‾‾‾
 *           ^ STOP here
 */
static void bb_stop(struct i2c_bus_s *b)
{
    sda_low(b);
    delay(b, b->t_su_sto);
    scl_high(b);
    delay(b, b->t_su_sto);
    sda_high(b);             /* SDA rises while SCL HIGH = STOP */
    delay(b, b->t_buf);
}

/**
 * @brief Transmit one byte, check ACK
 *
 * Shifts out 8 bits MSB-first, then releases SDA and reads ACK.
 *
 * @return I2C_OK, I2C_ERR_NACK_DATA, I2C_ERR_ARB_LOST, I2C_ERR_CLK_STRETCH
 */
static i2c_err_t bb_write_byte(struct i2c_bus_s *b, uint8_t byte)
{
    i2c_err_t err;

    for (int bit = 7; bit >= 0; bit--) {
        /* Set SDA to the next data bit */
        if (byte & (1 << bit)) {
            sda_high(b);
        } else {
            sda_low(b);
        }
        delay(b, b->t_su_dat);

        /* Check for arbitration loss: if we drove SDA HIGH but it reads LOW,
         * another master is driving the bus — we lost arbitration */
        if ((byte & (1 << bit)) && !sda_read(b)) {
            b->stats.arb_lost_count++;
            b->state = I2C_STATE_ERROR;
            return I2C_ERR_ARB_LOST;
        }

        err = scl_clock_pulse(b);
        if (err != I2C_OK) return err;
    }

    /* Release SDA to let slave drive ACK */
    sda_high(b);
    delay(b, b->t_su_dat);

    /* Clock the ACK bit */
    scl_high(b);
    delay(b, b->t_half_us);

    bool ack = !sda_read(b);   /* ACK = SDA LOW, NACK = SDA HIGH */

    scl_low(b);
    delay(b, b->t_half_us);

    return ack ? I2C_OK : I2C_ERR_NACK_DATA;
}

/**
 * @brief Receive one byte, send ACK or NACK
 *
 * Clocks in 8 bits MSB-first, then sends ACK/NACK.
 *
 * @param ack  true = send ACK (continue), false = send NACK (last byte)
 */
static i2c_err_t bb_read_byte(struct i2c_bus_s *b, uint8_t *out, bool ack)
{
    i2c_err_t err;
    uint8_t byte = 0;

    sda_high(b);   /* Release SDA — slave will drive data */

    for (int bit = 7; bit >= 0; bit--) {
        delay(b, b->t_su_dat);

        scl_high(b);
        delay(b, b->t_half_us);

        if (sda_read(b)) {
            byte |= (1 << bit);
        }

        scl_low(b);
        delay(b, b->t_half_us);

        (void)err; /* suppress unused warning from loop */
    }

    *out = byte;

    /* Send ACK (LOW) or NACK (HIGH) */
    if (ack) {
        sda_low(b);
    } else {
        sda_high(b);
    }
    delay(b, b->t_su_dat);

    err = scl_clock_pulse(b);

    sda_high(b);   /* Release SDA after ACK/NACK */
    return err;
}

/**
 * @brief Transmit 7-bit address with R/W bit, check for ACK
 *
 * Address byte format: [A6][A5][A4][A3][A2][A1][A0][R/W]
 *
 * @param read  true = READ (R/W=1), false = WRITE (R/W=0)
 * @return I2C_OK, I2C_ERR_NACK_ADDR (no device), I2C_ERR_ARB_LOST
 */
static i2c_err_t bb_send_addr(struct i2c_bus_s *b, uint8_t addr, bool read)
{
    uint8_t addr_byte = (addr << 1) | (read ? 1 : 0);
    i2c_err_t err = bb_write_byte(b, addr_byte);

    if (err == I2C_ERR_NACK_DATA) {
        /* NACK on address = no device at this address */
        b->stats.nack_count++;
        return I2C_ERR_NACK_ADDR;
    }

    return err;
}

/* ============================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================== */

i2c_err_t i2c_init(i2c_bus_t *bus_out, const i2c_config_t *cfg)
{
    if (!bus_out || !cfg) return I2C_ERR_INVALID_ARG;

    if (cfg->mode == I2C_MODE_SOFTWARE) {
        /* Software mode requires HAL callbacks */
        if (!cfg->hal ||
            !cfg->hal->sda_dir   ||
            !cfg->hal->sda_write ||
            !cfg->hal->sda_read  ||
            !cfg->hal->scl_write ||
            !cfg->hal->delay_us  ||
            !cfg->hal->millis) {
            return I2C_ERR_INVALID_ARG;
        }
    }

    /* Find a free slot in the pool */
    struct i2c_bus_s *bus = NULL;
    for (int i = 0; i < I2C_MAX_INSTANCES; i++) {
        if (!s_bus_used[i]) {
            bus = &s_bus_pool[i];
            s_bus_used[i] = true;
            break;
        }
    }
    if (!bus) return I2C_ERR_PLATFORM;  /* No free instances */

    /* Copy config and initialize */
    memset(bus, 0, sizeof(*bus));
    bus->cfg = *cfg;

    /* Apply default timeout if not set */
    if (bus->cfg.timeout_ms == 0)
        bus->cfg.timeout_ms = I2C_TIMEOUT_DEFAULT_MS;

    /* Compute timing from speed */
    compute_timing(bus);

    /* Initialize bus to idle state (both lines HIGH) */
    if (cfg->mode == I2C_MODE_SOFTWARE) {
        sda_high(bus);
        scl_high(bus);
        delay(bus, bus->t_buf);
    }

    bus->state       = I2C_STATE_IDLE;
    bus->initialized = true;

    *bus_out = bus;
    return I2C_OK;
}

i2c_err_t i2c_deinit(i2c_bus_t bus)
{
    if (!bus || !bus->initialized) return I2C_ERR_NOT_INIT;

    /* Send STOP if mid-transaction */
    if (bus->state == I2C_STATE_BUSY) {
        bb_stop(bus);
    }

    /* Mark slot as free */
    for (int i = 0; i < I2C_MAX_INSTANCES; i++) {
        if (&s_bus_pool[i] == bus) {
            s_bus_used[i] = false;
            break;
        }
    }

    memset(bus, 0, sizeof(*bus));
    return I2C_OK;
}

i2c_err_t i2c_set_speed(i2c_bus_t bus, i2c_speed_t speed)
{
    if (!bus || !bus->initialized) return I2C_ERR_NOT_INIT;
    if (bus->state == I2C_STATE_BUSY)  return I2C_ERR_BUS_BUSY;

    bus->cfg.speed = speed;
    compute_timing(bus);
    return I2C_OK;
}

/* -------------------------------------------------------------------------- */

i2c_err_t i2c_start(i2c_bus_t bus, uint8_t addr, bool read)
{
    if (!bus || !bus->initialized) return I2C_ERR_NOT_INIT;
    if (!i2c_addr_is_valid(addr))  return I2C_ERR_ADDR_RANGE;

    bus->state = I2C_STATE_BUSY;
    bb_start(bus);

    i2c_err_t err = bb_send_addr(bus, addr, read);
    if (err != I2C_OK) {
        bb_stop(bus);
        bus->state = I2C_STATE_IDLE;
        bus->stats.error_count++;
    }
    return err;
}

i2c_err_t i2c_restart(i2c_bus_t bus, uint8_t addr, bool read)
{
    if (!bus || !bus->initialized) return I2C_ERR_NOT_INIT;
    if (!i2c_addr_is_valid(addr))  return I2C_ERR_ADDR_RANGE;

    bb_restart(bus);

    i2c_err_t err = bb_send_addr(bus, addr, read);
    if (err != I2C_OK) {
        bb_stop(bus);
        bus->state = I2C_STATE_IDLE;
        bus->stats.error_count++;
    }
    return err;
}

i2c_err_t i2c_stop(i2c_bus_t bus)
{
    if (!bus || !bus->initialized) return I2C_ERR_NOT_INIT;
    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;
    bus->stats.transaction_count++;
    return I2C_OK;
}

i2c_err_t i2c_write_byte(i2c_bus_t bus, uint8_t byte)
{
    if (!bus || !bus->initialized) return I2C_ERR_NOT_INIT;

    i2c_err_t err = bb_write_byte(bus, byte);
    bus->stats.tx_count++;
    if (err != I2C_OK) bus->stats.error_count++;
    return err;
}

i2c_err_t i2c_read_byte(i2c_bus_t bus, uint8_t *byte, bool ack)
{
    if (!bus || !bus->initialized || !byte) return I2C_ERR_INVALID_ARG;

    i2c_err_t err = bb_read_byte(bus, byte, ack);
    bus->stats.rx_count++;
    if (err != I2C_OK) bus->stats.error_count++;
    return err;
}

/* -------------------------------------------------------------------------- */

i2c_err_t i2c_write(i2c_bus_t bus, uint8_t addr, const uint8_t *data, size_t len)
{
    if (!bus || !bus->initialized || !data) return I2C_ERR_INVALID_ARG;
    if (!i2c_addr_is_valid(addr))           return I2C_ERR_ADDR_RANGE;

    i2c_err_t err;

    bus->state = I2C_STATE_BUSY;
    bb_start(bus);

    err = bb_send_addr(bus, addr, false);
    if (err != I2C_OK) goto fail;

    for (size_t i = 0; i < len; i++) {
        err = bb_write_byte(bus, data[i]);
        if (err != I2C_OK) goto fail;
        bus->stats.tx_count++;
    }

    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;
    bus->stats.transaction_count++;
    return I2C_OK;

fail:
    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;
    bus->stats.error_count++;
    return err;
}

i2c_err_t i2c_read(i2c_bus_t bus, uint8_t addr, uint8_t *data, size_t len)
{
    if (!bus || !bus->initialized || !data || len == 0) return I2C_ERR_INVALID_ARG;
    if (!i2c_addr_is_valid(addr))                       return I2C_ERR_ADDR_RANGE;

    i2c_err_t err;

    bus->state = I2C_STATE_BUSY;
    bb_start(bus);

    err = bb_send_addr(bus, addr, true);
    if (err != I2C_OK) goto fail;

    for (size_t i = 0; i < len; i++) {
        bool ack = (i < len - 1);   /* ACK all except last byte */
        err = bb_read_byte(bus, &data[i], ack);
        if (err != I2C_OK) goto fail;
        bus->stats.rx_count++;
    }

    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;
    bus->stats.transaction_count++;
    return I2C_OK;

fail:
    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;
    bus->stats.error_count++;
    return err;
}

i2c_err_t i2c_write_reg_read(i2c_bus_t bus, uint8_t addr, uint8_t reg,
                               uint8_t *data, size_t len)
{
    if (!bus || !bus->initialized || !data) return I2C_ERR_INVALID_ARG;

    i2c_err_t err;
    bus->state = I2C_STATE_BUSY;

    /* Phase 1: Write register address */
    bb_start(bus);
    err = bb_send_addr(bus, addr, false);
    if (err != I2C_OK) goto fail;

    err = bb_write_byte(bus, reg);
    if (err != I2C_OK) goto fail;
    bus->stats.tx_count++;

    /* Phase 2: Repeated START, read data */
    bb_restart(bus);
    err = bb_send_addr(bus, addr, true);
    if (err != I2C_OK) goto fail;

    for (size_t i = 0; i < len; i++) {
        bool ack = (i < len - 1);
        err = bb_read_byte(bus, &data[i], ack);
        if (err != I2C_OK) goto fail;
        bus->stats.rx_count++;
    }

    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;
    bus->stats.transaction_count++;
    return I2C_OK;

fail:
    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;
    bus->stats.error_count++;
    return err;
}

i2c_err_t i2c_write_reg(i2c_bus_t bus, uint8_t addr, uint8_t reg,
                         const uint8_t *data, size_t len)
{
    if (!bus || !bus->initialized || !data) return I2C_ERR_INVALID_ARG;

    i2c_err_t err;
    bus->state = I2C_STATE_BUSY;

    bb_start(bus);
    err = bb_send_addr(bus, addr, false);
    if (err != I2C_OK) goto fail;

    /* Write register byte first */
    err = bb_write_byte(bus, reg);
    if (err != I2C_OK) goto fail;
    bus->stats.tx_count++;

    /* Write data bytes */
    for (size_t i = 0; i < len; i++) {
        err = bb_write_byte(bus, data[i]);
        if (err != I2C_OK) goto fail;
        bus->stats.tx_count++;
    }

    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;
    bus->stats.transaction_count++;
    return I2C_OK;

fail:
    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;
    bus->stats.error_count++;
    return err;
}

/* ============================================================================
 * REGISTER SHORTHAND
 * ========================================================================== */

i2c_err_t i2c_reg_write8(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint8_t val)
{
    return i2c_write_reg(bus, addr, reg, &val, 1);
}

i2c_err_t i2c_reg_read8(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_write_reg_read(bus, addr, reg, val, 1);
}

i2c_err_t i2c_reg_write16_be(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_write_reg(bus, addr, reg, buf, 2);
}

i2c_err_t i2c_reg_read16_be(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    i2c_err_t err = i2c_write_reg_read(bus, addr, reg, buf, 2);
    if (err == I2C_OK)
        *val = ((uint16_t)buf[0] << 8) | buf[1];
    return err;
}

i2c_err_t i2c_reg_read16_le(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    i2c_err_t err = i2c_write_reg_read(bus, addr, reg, buf, 2);
    if (err == I2C_OK)
        *val = ((uint16_t)buf[1] << 8) | buf[0];
    return err;
}

/* ============================================================================
 * BIT MANIPULATION
 * ========================================================================== */

i2c_err_t i2c_reg_set_bits(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint8_t mask)
{
    uint8_t val;
    i2c_err_t err = i2c_reg_read8(bus, addr, reg, &val);
    if (err != I2C_OK) return err;
    val |= mask;
    return i2c_reg_write8(bus, addr, reg, val);
}

i2c_err_t i2c_reg_clear_bits(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint8_t mask)
{
    uint8_t val;
    i2c_err_t err = i2c_reg_read8(bus, addr, reg, &val);
    if (err != I2C_OK) return err;
    val &= ~mask;
    return i2c_reg_write8(bus, addr, reg, val);
}

i2c_err_t i2c_reg_update_bits(i2c_bus_t bus, uint8_t addr, uint8_t reg,
                                uint8_t mask, uint8_t value)
{
    uint8_t val;
    i2c_err_t err = i2c_reg_read8(bus, addr, reg, &val);
    if (err != I2C_OK) return err;
    val = (val & ~mask) | (value & mask);
    return i2c_reg_write8(bus, addr, reg, val);
}

/* ============================================================================
 * DIAGNOSTIC FUNCTIONS
 * ========================================================================== */

i2c_err_t i2c_scan(i2c_bus_t bus, uint8_t *found_addrs, uint8_t *count)
{
    if (!bus || !bus->initialized || !found_addrs || !count)
        return I2C_ERR_INVALID_ARG;

    *count = 0;

    for (uint8_t addr = I2C_ADDR_MIN; addr <= I2C_ADDR_MAX; addr++) {
        bus->state = I2C_STATE_BUSY;
        bb_start(bus);
        i2c_err_t err = bb_send_addr(bus, addr, false);
        bb_stop(bus);
        bus->state = I2C_STATE_IDLE;

        if (err == I2C_OK) {
            found_addrs[(*count)++] = addr;
        }

        /* Brief delay between probes to avoid disturbing devices */
        delay(bus, bus->t_buf);
    }

    return I2C_OK;
}

bool i2c_device_present(i2c_bus_t bus, uint8_t addr)
{
    if (!bus || !bus->initialized) return false;
    if (!i2c_addr_is_valid(addr))  return false;

    bus->state = I2C_STATE_BUSY;
    bb_start(bus);
    i2c_err_t err = bb_send_addr(bus, addr, false);
    bb_stop(bus);
    bus->state = I2C_STATE_IDLE;

    return (err == I2C_OK);
}

i2c_err_t i2c_recover(i2c_bus_t bus)
{
    if (!bus || !bus->initialized) return I2C_ERR_NOT_INIT;

    /*
     * I2C Bus Recovery Procedure (per NXP UM10204 §3.1.16):
     * 1. Assert 9 SCL clock pulses — this forces any slave that was mid-byte
     *    to release SDA regardless of where it was in its bit sequence.
     * 2. Generate a STOP condition.
     * 3. Verify SDA and SCL are both HIGH.
     */

    /* Step 1: 9 clock pulses */
    for (int i = 0; i < I2C_BUS_RECOVERY_CLOCKS; i++) {
        scl_low(bus);
        delay(bus, bus->t_half_us);
        scl_high(bus);
        delay(bus, bus->t_half_us);
    }

    /* Step 2: STOP condition */
    sda_low(bus);
    delay(bus, bus->t_hd_sta);
    scl_high(bus);
    delay(bus, bus->t_su_sto);
    sda_high(bus);
    delay(bus, bus->t_buf);

    /* Step 3: Verify bus is free */
    if (!sda_read(bus)) {
        bus->state = I2C_STATE_ERROR;
        return I2C_ERR_BUS_BUSY;  /* SDA still stuck — power cycle needed */
    }

    bus->state = I2C_STATE_IDLE;
    bus->stats.recovery_count++;
    return I2C_OK;
}

i2c_state_t i2c_get_state(i2c_bus_t bus)
{
    if (!bus || !bus->initialized) return I2C_STATE_ERROR;
    return bus->state;
}

i2c_err_t i2c_get_stats(i2c_bus_t bus, i2c_stats_t *stats)
{
    if (!bus || !bus->initialized || !stats) return I2C_ERR_INVALID_ARG;
    *stats = bus->stats;
    return I2C_OK;
}

i2c_err_t i2c_reset_stats(i2c_bus_t bus)
{
    if (!bus || !bus->initialized) return I2C_ERR_NOT_INIT;
    memset(&bus->stats, 0, sizeof(bus->stats));
    return I2C_OK;
}

/* ============================================================================
 * UTILITY
 * ========================================================================== */

const char *i2c_err_to_string(i2c_err_t err)
{
    switch (err) {
        case I2C_OK:              return "Success";
        case I2C_ERR_NACK_ADDR:   return "No ACK on address (device not found)";
        case I2C_ERR_NACK_DATA:   return "No ACK during data transfer";
        case I2C_ERR_ARB_LOST:    return "Arbitration lost (multi-master conflict)";
        case I2C_ERR_BUS_BUSY:    return "Bus busy (SDA or SCL stuck LOW)";
        case I2C_ERR_TIMEOUT:     return "Operation timed out";
        case I2C_ERR_INVALID_ARG: return "Invalid argument (NULL pointer or bad value)";
        case I2C_ERR_NOT_INIT:    return "Bus not initialized (call i2c_init first)";
        case I2C_ERR_OVERFLOW:    return "Buffer overflow";
        case I2C_ERR_BUS_ERROR:   return "Bus error (unexpected START/STOP detected)";
        case I2C_ERR_PLATFORM:    return "Platform-specific hardware error";
        case I2C_ERR_NO_DEVICE:   return "No device found on bus";
        case I2C_ERR_ADDR_RANGE:  return "Address out of valid range (0x08-0x77)";
        case I2C_ERR_CLK_STRETCH: return "Clock stretch timeout (slave holding SCL)";
        case I2C_ERR_DMA:         return "DMA configuration or transfer error";
        default:                  return "Unknown error";
    }
}

bool i2c_addr_is_valid(uint8_t addr)
{
    return (addr >= I2C_ADDR_MIN && addr <= I2C_ADDR_MAX);
}

const char *i2c_version(void)
{
    return I2C_LIB_VERSION_STRING;
}
