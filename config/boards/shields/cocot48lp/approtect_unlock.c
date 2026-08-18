/*
 * Firmware-side APPROTECT unlock for the BLE Micro Pro (nRF52840).
 *
 * WHY: ST-Link is a "high-level adapter" (HLA) and cannot access the Nordic
 * proprietary CTRL-AP, so it can NOT run the ERASEALL/recover that clears
 * APPROTECT. But the UF2 bootloader still works, so we can flash firmware
 * that unlocks the chip itself: write UICR.APPROTECT = 0x5A (HwDisabled) and
 * reset. With CONFIG_NRF_APPROTECT_USE_UICR=y, the SoC init reads that on the
 * next boot and disables CTRL-AP APPROTECT, exposing the normal MEM-AP so the
 * ST-Link can attach for SWD debugging. The UF2 bootloader is untouched
 * (it lives in code flash, not UICR).
 *
 * SAFETY: uses only raw NVMC registers (no flash driver — the driver-based
 * storage_erase.c hook previously hung the board). A direct write can only
 * clear bits, so it succeeds when UICR.APPROTECT is in its erased state
 * (0xFFFFFFFF). If the field was previously programmed to 0x00, the write
 * won't take; we detect that and DON'T reset, to avoid an infinite reset loop
 * (in that case a CTRL-AP capable probe such as CMSIS-DAP/J-Link is required).
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <nrfx.h>

LOG_MODULE_REGISTER(approtect_unlock, LOG_LEVEL_INF);

#define UICR_APPROTECT_HWDISABLED 0x5AUL

/* Very early: program UICR.APPROTECT and reset if needed. */
static int approtect_unlock_early(void)
{
	if (NRF_UICR->APPROTECT == UICR_APPROTECT_HWDISABLED) {
		return 0; /* already unlocked */
	}

	/* Enable write to non-volatile memory (UICR). */
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}

	NRF_UICR->APPROTECT = UICR_APPROTECT_HWDISABLED;
	__DSB();
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}

	/* Back to read-only. */
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}

	/* Only reset if the write actually stuck; otherwise leave the board
	 * running normally (no reset loop). */
	if (NRF_UICR->APPROTECT == UICR_APPROTECT_HWDISABLED) {
		NVIC_SystemReset();
	}

	return 0;
}
SYS_INIT(approtect_unlock_early, PRE_KERNEL_1, 0);

/* Later, once logging is up, report the value over USB serial so we can
 * confirm success (0x5A) or diagnose failure (e.g. 0x00 => needs a CTRL-AP
 * probe). */
static int approtect_report(void)
{
	LOG_INF("UICR.APPROTECT = 0x%08x (want 0x0000005A = unlocked)",
		NRF_UICR->APPROTECT);
	return 0;
}
SYS_INIT(approtect_report, APPLICATION, 99);
