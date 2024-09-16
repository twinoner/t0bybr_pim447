zephyr_library()

if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  zephyr_include_directories(include)
  zephyr_library_sources_ifdef(CONFIG_ZMK_SENSOR_PIM447 src/pim447.c)
endif()