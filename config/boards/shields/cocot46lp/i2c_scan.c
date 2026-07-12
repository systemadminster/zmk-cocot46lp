/*
 * DIAGNOSTIC ONLY — remove for production.
 *
 * One-shot I2C bus scan on i2c1 (TWIM1), run at APPLICATION init (after the
 * bus is ready). Logs every address that ACKs so we can tell whether the
 * AZ1UBALL trackball (addr 0x0A) is electrically present / responding at all.
 *
 *   - 0x0A ACKs  -> trackball is powered and its I2C chip is alive
 *   - nothing    -> no power / broken wiring / dead module
 *
 * Deferred logging only (TWIM + CONFIG_LOG_MODE_IMMEDIATE deadlocks the boot).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_scan, LOG_LEVEL_INF);

static int i2c_scan_init(void)
{
    const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));

    if (!device_is_ready(bus)) {
        LOG_ERR("i2c1 bus not ready");
        return 0;
    }

    LOG_INF("=== I2C1 scan start (looking for trackball @ 0x0A) ===");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy;
        int ret = i2c_read(bus, &dummy, 1, addr);
        if (ret == 0) {
            LOG_INF("I2C device ACKed at 0x%02x", addr);
            found++;
        }
    }
    LOG_INF("=== I2C1 scan done: %d device(s) found ===", found);

    return 0;
}

SYS_INIT(i2c_scan_init, APPLICATION, 90);
