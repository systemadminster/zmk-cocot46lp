/*
 * Erase the stale QMK-era Nordic SoftDevice (S140) that still sits at
 * 0x1000..0x26000.
 *
 * The SDPROBE log confirmed a SoftDevice info-struct magic (0x51B1E5DB, id 140)
 * at 0x3004. UF2 flashing of ZMK only writes the app (code_partition at
 * 0x26000) and never clears this region, so the old SoftDevice remains and is
 * the prime suspect for ZMK's total hang when BLE is enabled (ZMK uses its own
 * SoftDevice-less controller and fights the leftover SD for the radio).
 *
 * We erase pages 0x1000..0x25000 only. The MBR (0x0..0x1000) and the ZMK app
 * (>= 0x26000) are left untouched. Raw NVMC page erase (no flash driver — that
 * hung the board before). Guarded on the SoftDevice magic so it runs exactly
 * once, then a reset boots the board with a clean MBR+app layout.
 *
 * RECOVERY NOTE: if the bootloader turns out to compute the app start from the
 * (now-erased) SoftDevice size rather than a fixed 0x26000, the board may not
 * boot after this. That is fully recoverable via the UF2 bootloader (double the
 * usual flash) — just reflash.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <nrfx.h>

#define RD(a)     (*(volatile uint32_t *)(uintptr_t)(a))
#define SD_MAGIC  0x51B1E5DBUL
#define SD_START  0x00001000UL
#define SD_END    0x00026000UL   /* ZMK app begins here; never erase at/above */
#define PAGE_SZ   0x00001000UL

static int erase_softdevice(void)
{
	/* Only act while a SoftDevice is actually present; after we erase it the
	 * magic reads 0xFFFFFFFF and this becomes a no-op (no reset loop, never
	 * touches the app region). */
	if (RD(0x3004) != SD_MAGIC) {
		return 0;
	}

	unsigned int key = irq_lock();

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;   /* erase enable */
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}

	for (uint32_t addr = SD_START; addr < SD_END; addr += PAGE_SZ) {
		NRF_NVMC->ERASEPAGE = addr;
		__DSB();
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
		}
	}

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;   /* back to read-only */
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}

	irq_unlock(key);

	NVIC_SystemReset();
	return 0;
}
/* Run after approtect_unlock (both PRE_KERNEL_1); ordering between them does not
 * matter since each guards on its own condition and resets. */
SYS_INIT(erase_softdevice, PRE_KERNEL_1, 1);
