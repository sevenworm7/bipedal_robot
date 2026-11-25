cmake_minimum_required(VERSION 3.13)
# robot v2 on-board runtime for the MANGO X4 (RP2350). Build with build_firmware.sh (sets the SDK, toolchain and
# EXPORT_DIR = an outputs/robot_v2/deploy/export_* folder from deploy/export/export_policy.py).

set(PICO_BOARD mango_x4 CACHE STRING "Board")
list(APPEND PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_LIST_DIR}/boards")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")
if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if(NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR "Pico SDK not found: -DPICO_SDK_PATH=<dir>")
endif()
set(PICO_NO_PICOTOOL 1 CACHE BOOL "UF2 conversion is done by build_firmware.sh" FORCE)
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(biped_v2_runtime C CXX ASM)
pico_sdk_init()

if(NOT EXPORT_DIR OR NOT EXISTS "${EXPORT_DIR}/robot_config.h")
    message(FATAL_ERROR "EXPORT_DIR must be an export folder with robot_config.h / policy_int8.h")
endif()
option(POLICY_FP32 "float32 weights in flash instead of int8 in SRAM (slow, for comparison)" OFF)
option(ALLOW_UNVERIFIED_CALIB "allow INIT with an unverified calibration (bench only)" OFF)
set(FW_BUILD_ID "unknown" CACHE STRING "firmware_build folder name, shown by STATUS (build_firmware.sh)")

add_executable(biped_runtime
    src/main_rp2350.c src/policy.c src/obs.c src/attitude.c src/joints.c src/feetech.c src/controller.c
    vendor/bmi270/bmi2.c vendor/bmi270/bmi270.c)
target_include_directories(biped_runtime PRIVATE src vendor/bmi270 "${EXPORT_DIR}")
target_compile_options(biped_runtime PRIVATE -Wall -Wextra -O2)
target_compile_definitions(biped_runtime PRIVATE PICO_BUILD=1
    PICO_STDIO_USB_DEFAULT_CRLF=0 PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1 PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE=1200
    USBD_PRODUCT="BIPED V2 RUNTIME" FW_BUILD_ID="${FW_BUILD_ID}")
if(NOT POLICY_FP32)
    target_compile_definitions(biped_runtime PRIVATE POLICY_INT8=1)
endif()
if(ALLOW_UNVERIFIED_CALIB)
    target_compile_definitions(biped_runtime PRIVATE ALLOW_UNVERIFIED_CALIB=1)
endif()
target_link_libraries(biped_runtime pico_stdlib hardware_uart hardware_spi m)
pico_enable_stdio_usb(biped_runtime 1)
pico_enable_stdio_uart(biped_runtime 0)
pico_add_extra_outputs(biped_runtime)
