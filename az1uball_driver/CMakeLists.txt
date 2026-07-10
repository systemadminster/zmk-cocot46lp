zephyr_library()

# Add source file only if CONFIG_AZ1UBALL is enabled
zephyr_library_sources_ifdef(CONFIG_AZ1UBALL src/az1uball.c)

# Add include directories
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
