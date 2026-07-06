/*
 * Custom fatal-error hook to capture the BLE crash over SWD.
 *
 * When BLE is enabled the board dies and USB never comes up. Over SWD, short
 * memory reads work but long operations (full-RAM scan) get cut off by the
 * flaky ST-Link joint, so we cannot scan for a magic tag. Instead we write the
 * fault info to a FIXED, known RAM address that Zephyr never touches: the DT
 * shrinks sram0 by 256 bytes (ends at 0x2003FF00) and we stash the info at
 * 0x2003FF00, so recovery is a single short read of 8 words at that address.
 *
 * Slot layout at 0x2003FF00:
 *   [0]=0xFA017A17 magic, [1]=0x0BADF00D magic, [2]=reason,
 *   [3]=esf ptr (stacked PC is at *(esf+0x18)),
 *   [4]=CFSR, [5]=HFSR, [6]=BFAR, [7]=MMFAR
 */

#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <cmsis_core.h>

#define FAULT_SLOT ((volatile uint32_t *)0x2003FF00UL)

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	FAULT_SLOT[0] = 0xFA017A17UL;               /* magic 1 */
	FAULT_SLOT[1] = 0x0BADF00DUL;               /* magic 2 */
	FAULT_SLOT[2] = reason;
	FAULT_SLOT[3] = (uint32_t)(uintptr_t)esf;   /* stacked PC at *(esf + 0x18) */
	FAULT_SLOT[4] = SCB->CFSR;
	FAULT_SLOT[5] = SCB->HFSR;
	FAULT_SLOT[6] = SCB->BFAR;
	FAULT_SLOT[7] = SCB->MMFAR;
	__DSB();

	/* Busy-spin: keep the core running (halt-able) and RAM readable, with no
	 * reset and no low-power entry. */
	for (;;) {
		/* nothing */
	}
}
