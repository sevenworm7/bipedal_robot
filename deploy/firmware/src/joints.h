/* Servo position counts <-> model joint angles (portable C).
 *   angle [rad] = SERVO_SIGN * (count - zero) * 2 pi / 4096          (robot_config.h, from the calibration JSON)
 * The servos run in multi-turn mode (angle limits 0/0, set by the teammate on 2026-11-15) and report the position
 * within one turn after power-up, so the stored zero is shifted by whole turns (4096 counts) to the equivalent
 * nearest to the current reading (joints_align_turns). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "robot_config.h"

#define COUNTS_PER_TURN 4096
#define SERVO_POS_LIMIT 28671 /* signed-magnitude goal register range used by the teammate's firmware */

typedef struct {
    int32_t zero[NUM_JOINTS]; /* turn-aligned zero counts */
    bool aligned;
} joint_map;

void joints_init(joint_map *m);
/* Choose, per joint, the whole-turn shift of the stored zero that puts the current angle closest to 0 (all joint
 * ranges are within +-180 deg). Returns false if any resulting angle is outside its limits by more than tol_rad. */
bool joints_align_turns(joint_map *m, const int32_t counts[NUM_JOINTS], float tol_rad, int *bad_joint);
float joints_count_to_rad(const joint_map *m, int j, int32_t count);
/* Clamps to [JOINT_LOWER, JOINT_UPPER]; returns the goal count. */
int32_t joints_rad_to_count(const joint_map *m, int j, float rad);
