# Contributing to i2c_lib

Thank you for considering a contribution! Here is everything you need to know.

---

## Ways to contribute

- **Bug report** — open a GitHub Issue with the bug report template
- **New device driver** — add a new chip driver under `drivers/`
- **Platform HAL example** — add a porting example under `examples/hal/`
- **Documentation fix** — fix typos, clarify comments, improve the README
- **New feature** — open an issue first to discuss the design before coding

---

## Code style

- C11 standard, no compiler extensions
- Snake_case for all identifiers
- `i2c_` prefix for all public symbols
- All public functions documented with `@brief`, `@param`, `@return`
- Error propagation: always return `i2c_err_t`, never assert/abort in library code
- No dynamic memory allocation (`malloc` / `calloc`) in library code
- No global mutable state outside the `s_bus_pool` array

---

## Adding a device driver

1. Add the driver header (`typedef struct`, function prototypes, `@brief` with real-world usage) to `drivers/i2c_devices.h`
2. Add the implementation to `drivers/i2c_devices.c`
3. Add a usage example to `examples/example_all_devices.c`
4. Update the device table in `README.md`
5. Add an entry under `[Unreleased]` in `CHANGELOG.md`

Each driver must:
- Verify chip identity on `_init()` (read WHO_AM_I or equivalent)
- Use `i2c_write_reg_read()` for combined transactions
- Return `i2c_err_t` on every function
- Not use `float` unless absolutely necessary (prefer fixed-point)
- Include a `@example` code block in the header comment

---

## Pull request checklist

- [ ] Code compiles with `gcc -Wall -Wextra -std=c11` without warnings
- [ ] All public API additions are documented in the header
- [ ] `CHANGELOG.md` updated under `[Unreleased]`
- [ ] Example updated if a new driver was added
- [ ] No new heap allocations introduced

---

## Commit messages

Use the conventional commits format:

```
feat(drivers): add VL53L1X time-of-flight driver
fix(i2c): correct clock stretch timeout comparison
docs(readme): add RP2040 HAL example
```

---

## Issue templates

When filing a bug, please include:
- MCU and toolchain (e.g. STM32F4, arm-none-eabi-gcc 12.2)
- I2C mode (software/hardware) and speed setting
- Device being communicated with
- Minimal reproducing code snippet
- Actual vs expected behaviour
- Logic analyser capture if available (paste as text or attach `.sal`)
