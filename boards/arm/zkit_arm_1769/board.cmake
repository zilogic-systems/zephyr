set_ifndef(LPCLINK_FW jlink)

if(LPCLINK_FW STREQUAL jlink)
  set_ifndef(BOARD_DEBUG_RUNNER jlink)
  set_ifndef(BOARD_FLASH_RUNNER jlink)
endif()

board_runner_args(jlink "--device=ZKIT_ARM_1768")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
