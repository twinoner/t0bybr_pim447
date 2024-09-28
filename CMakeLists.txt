if(CONFIG_ZMK_PIM447)
  if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    zephyr_library()

    # Include the application include directory
    zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

    if(CONFIG_ZMK_PIM447_TB_ACTIONS)
      # Include the behaviors subdirectory
      zephyr_library_sources(
        src/behaviors/behavior_tb_actions.c
      )
    endif()

    zephyr_include_directories(include)
    zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_PIM447 src/pim447.c)
  endif()
endif()