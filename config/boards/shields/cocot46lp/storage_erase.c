/*
 * ONE-OFF maintenance build: erase the storage_partition (BLE bonding /
 * settings NVS) before continuing BLE testing. Dozens of test cycles
 * with interrupted/crashed BLE builds may have left NVS in a corrupted
 * or inconsistent state that doesn't get cleared by normal UF2 reflashing
 * (UF2 only rewrites code_partition, never touches storage_partition).
 *
 * Delete this file (and its CMakeLists.txt reference) once the erase
 * has run once — it is NOT meant to run on every boot.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static int erase_storage_partition(void)
{
    const struct flash_area *fa;
    int rc = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa);
    if (rc) {
        LOG_ERR("storage_erase: open failed: %d", rc);
        return rc;
    }

    LOG_INF("storage_erase: erasing %u bytes at 0x%lx",
             (unsigned int)fa->fa_size, (unsigned long)fa->fa_off);

    rc = flash_area_erase(fa, 0, fa->fa_size);
    LOG_INF("storage_erase: erase result: %d", rc);

    flash_area_close(fa);
    return rc;
}

SYS_INIT(erase_storage_partition, POST_KERNEL, 95);
