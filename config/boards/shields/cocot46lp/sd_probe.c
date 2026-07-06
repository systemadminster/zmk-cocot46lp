/*
 * SoftDevice presence probe (SWD-free diagnosis).
 *
 * The SIO/SWDIO pad tore off, so we can no longer inspect flash with the
 * ST-Link. Instead the firmware itself reads the low flash region and logs it
 * over USB serial (BLE is off, so USB/CDC stays alive).
 *
 * Hypothesis: this board previously ran QMK BLE, which uses a Nordic
 * SoftDevice living at 0x1000. UF2 flashing of ZMK never touches that region,
 * so a stale SoftDevice may still be there and fight ZMK's own (SoftDevice-less)
 * BLE stack for the radio -> the total hang we see with BLE=y.
 *
 * A Nordic SoftDevice publishes an "info struct" at MBR_SIZE(0x1000)+0x2000 =
 * 0x3000, whose magic word at 0x3004 is 0x51B1E5DB. If we see that magic, a
 * SoftDevice IS present. 0x3010 then holds the SoftDevice id (e.g. 140).
 *
 * Logged on a repeating thread (every 3 s) so the value is easy to capture no
 * matter when the serial port is opened.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sd_probe, LOG_LEVEL_INF);

#define RD(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define SD_MAGIC 0x51B1E5DBUL

static void sd_probe_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		uint32_t magic = RD(0x3004);

		LOG_INF("SDPROBE f1000=%08x f1004=%08x magic=%08x sdsize=%08x id=%u ver=%u nrffw0=%08x",
			RD(0x1000), RD(0x1004), magic, RD(0x3008),
			RD(0x3010), RD(0x3014), RD(0x10001014));

		if (magic == SD_MAGIC) {
			LOG_WRN("SDPROBE => SoftDevice PRESENT at 0x1000 (id=%u) -- likely BLE conflict",
				RD(0x3010));
		} else {
			LOG_INF("SDPROBE => no SoftDevice magic (region empty/erased)");
		}

		k_sleep(K_SECONDS(3));
	}
}

K_THREAD_DEFINE(sd_probe_tid, 1024, sd_probe_thread, NULL, NULL, NULL, 7, 0, 2000);
