/*
 * Custom fatal-error hook to capture the BLE crash over SWD.
 *
 * When BLE is enabled the board dies and USB never comes up. Over SWD, memory
 * reads worked but the core could not be halted (looked like a reboot/retry
 * loop). To catch the crash we override Zephyr's weak k_sys_fatal_error_handler:
 * on any fatal error we stash the reason, the exception-stack-frame pointer, and
 * the ARM fault status registers into a magic-tagged RAM slot, then busy-spin
 * (no reset, no sleep) so the core stays halt-able and the RAM slot stays
 * readable over SWD.
 *
 * Recovery: with the ST-Link, scan RAM (0x20000000..0x20040000) for the two
 * magic words 0xFA017A17 0x0BADF00D; the following words are:
 *   [2] reason, [3] esf pointer (stacked PC is at *(esf+0x18)),
 *   [4] CFSR, [5] HFSR, [6] BFAR, [7] MMFAR.
 */

#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <cmsis_core.h>

/* .noinit so it is not wiped by C startup; 'used' so it is never GC'd. */
volatile uint32_t g_fault[8] __attribute__((used, section(".noinit")));

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	g_fault[0] = 0xFA017A17UL;              /* magic 1 */
	g_fault[1] = 0x0BADF00DUL;              /* magic 2 */
	g_fault[2] = reason;
	g_fault[3] = (uint32_t)(uintptr_t)esf;  /* stacked PC lives at *(esf + 0x18) */
	g_fault[4] = SCB->CFSR;
	g_fault[5] = SCB->HFSR;
	g_fault[6] = SCB->BFAR;
	g_fault[7] = SCB->MMFAR;
	__DSB();

	/* Busy-spin: keep the core running (halt-able) and the RAM readable,
	 * without resetting or entering low power. */
	for (;;) {
		/* nothing */
	}
}
