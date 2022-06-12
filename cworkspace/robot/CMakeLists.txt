add_executable(robot
  robot.c
)

pico_enable_stdio_usb(robot 1)
pico_enable_stdio_uart(robot 1)
pico_add_extra_outputs(robot)
target_link_libraries(robot pico_stdlib)
