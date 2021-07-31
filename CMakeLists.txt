cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(pico_synth)
pico_sdk_init()
add_executable(pico_synth pico_synth.c)
pico_set_binary_type(pico_synth copy_to_ram)
pico_enable_stdio_usb(pico_synth 1)
pico_enable_stdio_uart(pico_synth 1)
pico_add_extra_outputs(pico_synth)
target_link_libraries(pico_synth pico_stdlib
  pico_multicore
  hardware_irq hardware_pwm hardware_sync)
