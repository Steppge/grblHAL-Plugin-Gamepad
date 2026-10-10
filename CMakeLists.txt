add_library(gamepad INTERFACE)

target_sources(gamepad INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/gamepad_plugin.c
 ${CMAKE_CURRENT_LIST_DIR}/joystick_plugin.c
)

target_include_directories(gamepad INTERFACE ${CMAKE_CURRENT_LIST_DIR})
