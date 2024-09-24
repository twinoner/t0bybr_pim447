zephyr_library()

# Include the application include directory
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

# Include the behaviors subdirectory
zephyr_library_sources(
  src/behaviors/behavior_trackball_adjust.c
)

if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  zephyr_include_directories(include)
  zephyr_library_sources_ifdef(CONFIG_ZMK_SENSOR_PIM447 src/pim447.c)
endif()
