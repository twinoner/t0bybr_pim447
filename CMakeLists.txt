if(CONFIG_PIMORONI_PIM447)
  if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    zephyr_library()

    # Include the application include directory
    zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

    if(CONFIG_PIMORONI_PIM447_BEHAVIORS)
      # Include the behaviors subdirectory
      zephyr_library_sources(
        src/behaviors/pim447_behaviors.c
      )
    endif()

    zephyr_include_directories(include)
    zephyr_library_sources_ifdef(CONFIG_PIMORONI_PIM447 src/pim447.c)
  endif()
endif()