cmake_minimum_required(VERSION 3.18)

# ── Pico SDK bootstrap ─────────────────────────────────────────
# Set PICO_SDK_PATH in your environment or pass -DPICO_SDK_PATH=…
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rp2350_efi C CXX ASM)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ── Generate PIO headers ──────────────────────────────────────
pico_generate_pio_header(rp2350_efi
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pio/crank_trigger.pio
)

# ── Target ───────────────────────────────────────────────────
add_executable(rp2350_efi
    src/main.cpp
    src/drivers/crank_sensor.cpp
    src/drivers/adc_sensors.cpp
    src/fusion/kalman_filter.cpp
    src/efi/fuel_calc.cpp
)

target_include_directories(rp2350_efi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ── Link pico-sdk libraries ───────────────────────────────────
target_link_libraries(rp2350_efi
    pico_stdlib
    pico_multicore
    hardware_adc
    hardware_dma
    hardware_gpio
    hardware_irq
    hardware_pio
    hardware_pwm
    hardware_timer
    hardware_uart
    hardware_clocks
    m               # libm for sinf/logf/etc.
)

# ── Output formats ────────────────────────────────────────────
pico_add_extra_outputs(rp2350_efi)   # generates .uf2, .hex, .bin, .map

# ── Build options ─────────────────────────────────────────────
target_compile_options(rp2350_efi PRIVATE
    -Wall
    -Wextra
    -O2
    -ffast-math         # allow reciprocal/FMA optimisations
    -DPICO_RP2350=1     # inform SDK we target RP2350
)

# ── USB or UART stdio ─────────────────────────────────────────
pico_enable_stdio_usb  (rp2350_efi 0)
pico_enable_stdio_uart (rp2350_efi 1)   # debug printf on UART0

message(STATUS "rp2350_efi target configured for RP2350")
