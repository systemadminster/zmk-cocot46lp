/*
 * Live I2C bus scanner (trackball wiring diagnosis).
 *
 * The az1uball driver reports "Failed to set turbo mode" = no ACK at 0x0A.
 * To find out whether ANYTHING answers on the bus (and at which address),
 * scan 0x01..0x7F every 5 s and log the ACKed addresses. Runs forever so the
 * user can re-seat/swap wires and watch the result live without reflashing.
 *
 * Safe pairing note: TWIM + deferred logging is proven fine (fw78); never
 * combine TWIM with CONFIG_LOG_MODE_IMMEDIATE.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_scan, LOG_LEVEL_INF);

static void i2c_scan_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	if (!device_is_ready(bus)) {
		LOG_ERR("I2CSCAN bus not ready");
		return;
	}

	while (1) {
		char found[64] = "";
		int n = 0;
		uint8_t dummy;

		for (uint8_t addr = 0x01; addr < 0x78; addr++) {
			if (i2c_read(bus, &dummy, 1, addr) == 0) {
				int len = strlen(found);
				snprintf(found + len, sizeof(found) - len, " 0x%02X", addr);
				n++;
				if (n >= 8) {
					break;
				}
			}
		}

		if (n > 0) {
			LOG_INF("I2CSCAN found %d device(s):%s", n, found);
		} else {
			LOG_WRN("I2CSCAN no ACK on any address (power/wiring)");
		}

		k_sleep(K_SECONDS(5));
	}
}

K_THREAD_DEFINE(i2c_scan_tid, 1536, i2c_scan_thread, NULL, NULL, NULL, 7, 0, 3000);
