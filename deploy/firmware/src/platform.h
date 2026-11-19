/* Small portability layer: the same sources build for the RP2350 (Pico SDK) and for host unit tests (gcc). */
#pragma once

#ifdef PICO_BUILD
#include "pico.h"
/* Hot loops run from SRAM, not from XIP flash. */
#define RAMFUNC __not_in_flash("policy")
/* int8 policy weights: explicit .data section so they are copied to SRAM at start-up */
#define POL_SRAM __attribute__((section(".data.policy")))
#else
#define RAMFUNC
#define POL_SRAM
#endif
