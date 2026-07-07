zephyr_library()
zephyr_library_sources(kscan_duplex.c)
# approtect_unlock.c is kept: it is a no-op once UICR.APPROTECT is 0x5A, and it
# auto-unlocks any freshly flashed BMP for SWD debugging.
zephyr_library_sources(approtect_unlock.c)
zephyr_library_sources(bus_probe.c)
