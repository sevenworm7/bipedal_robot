/* Student policy inference (interface v2: 210 -> 512 -> 256 -> 128 -> 10, ELU, input normaliser).
 * Weights come from the generated header of deploy/export/export_policy.py:
 *   POLICY_INT8 defined -> policy_int8.h (int8 weights, one scale per output row, in SRAM)
 *   otherwise           -> policy_fp32.h (float32 weights, flash on the RP2350)
 * Portable C, no hardware access. */
#pragma once
#include <stdbool.h>

#define POLICY_OBS_DIM 210
#define POLICY_ACT_DIM 10

/* obs: student observation (5 x 42, newest frame first, SI units, not normalised).
 * act: raw policy mean (normalised action, not clipped). Returns false (act untouched) on non-finite input/output. */
bool policy_forward(const float *obs, float *act);

/* Expected action for obs = normaliser mean (BENCH self-test) and the normaliser mean itself. */
const float *policy_test_obs(void);
const float *policy_test_act(void);
const char *policy_kind(void);
