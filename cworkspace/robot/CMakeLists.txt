add_executable(robot
  robot.cpp
  robotlib.cpp
  AX12.cpp
)

pico_enable_stdio_usb(robot 0)
pico_enable_stdio_uart(robot 1)
pico_add_extra_outputs(robot)
target_link_libraries(robot pico_stdlib)
