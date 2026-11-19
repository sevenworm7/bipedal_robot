// Minimal UART-only MANGO X4 configuration. Source: supplied X4-intro.pdf/pinout.xlsx.
// pico_cmake_set PICO_PLATFORM = rp2350
// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (2 * 1024 * 1024)
// pico_cmake_set_default PICO_RP2350_A2_SUPPORTED = 1
#ifndef MANGO_X4_H
#define MANGO_X4_H

// The supplied board document specifies RP2350B (48 GPIOs).
#define PICO_RP2350A 0
#define PICO_DEFAULT_UART 0
#define PICO_DEFAULT_UART_TX_PIN 28
#define PICO_DEFAULT_UART_RX_PIN 29

// Use standard SPI READ, avoiding assumptions about the flash's quad-enable bit.
// Reserve only the first 2 MiB; this diagnostic image is much smaller.
#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 1
#define PICO_FLASH_SPI_CLKDIV 8
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif
#define PICO_RP2350_A2_SUPPORTED 1

// Pico SDK's default 12 MHz external crystal is used for initial bring-up.
// No LED, motor, power-switch, or PSRAM GPIO is configured.
#endif
