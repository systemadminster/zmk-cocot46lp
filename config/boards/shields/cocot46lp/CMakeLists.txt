zephyr_library()
zephyr_library_sources(kscan_duplex.c)
# approtect_unlock.c is kept: it is a no-op once UICR.APPROTECT is 0x5A, and it
# auto-unlocks any freshly flashed BMP for SWD debugging.
zephyr_library_sources(approtect_unlock.c)
# DIAG: one-shot I2C bus scan to check whether the trackball (0x0A) responds.
# Remove this line for production once the trackball is confirmed working.
zephyr_library_sources(i2c_scan.c)
