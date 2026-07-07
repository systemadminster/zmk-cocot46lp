/*
 * I2C bus level probe (SWD-free diagnosis).
 *
 * fw72/73 (AZ1UBALL driver enabled) dies before USB init, and SWD contact is
 * unreliable, so let the firmware itself report the I2C bus state: log the
 * raw levels of SDA (P0.18) and SCL (P0.16) every 3 s over USB serial.
 *
 * Expected: both HIGH at idle. If either reads LOW with no transfer running,
 * the bus is being clamped (unpowered/miswired trackball or level-shifter
 * back-feed) and any blocking I2C transfer would hang forever — which is the
 * leading theory for the boot hang.
 *
 * Reads the GPIO IN register directly; works regardless of whether TWIM owns
 * the pins.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx.h>

LOG_MODULE_REGISTER(bus_probe, LOG_LEVEL_INF);

static void bus_probe_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		uint32_t in = NRF_P0->IN;
		int sda = (in >> 18) & 1; /* P0.18 = BMP PIN5 */
		int scl = (in >> 16) & 1; /* P0.16 = BMP PIN6 */

		if (sda && scl) {
			LOG_INF("BUSPROBE SDA(P0.18)=%d SCL(P0.16)=%d -- idle HIGH, bus OK",
				sda, scl);
		} else {
			LOG_WRN("BUSPROBE SDA(P0.18)=%d SCL(P0.16)=%d -- LINE STUCK LOW (clamped: power/wiring)",
				sda, scl);
		}

		k_sleep(K_SECONDS(3));
	}
}

K_THREAD_DEFINE(bus_probe_tid, 1024, bus_probe_thread, NULL, NULL, NULL, 7, 0, 2000);
