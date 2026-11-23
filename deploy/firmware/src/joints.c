#include "joints.h"

#include <math.h>

#define TWO_PI 6.28318530717958647692f

void joints_init(joint_map *m) {
    for (int j = 0; j < NUM_JOINTS; ++j) m->zero[j] = SERVO_ZERO[j];
    m->aligned = false;
}

float joints_count_to_rad(const joint_map *m, int j, int32_t count) {
    return (float)SERVO_SIGN[j] * (float)(count - m->zero[j]) * (TWO_PI / COUNTS_PER_TURN);
}

int32_t joints_rad_to_count(const joint_map *m, int j, float rad) {
    /* safety range, not the training limits: INIT must start from where a joint really is (2026-11-23: the
     * drooping tail at -62 deg was clamped to -45 deg and jumped 17 deg at torque-on) */
    /* + RANGE_TOL_RAD (the alignment tolerance): a joint resting slightly outside, e.g. the left knee at -4.7 deg,
     * must not jump to the limit at torque-on (2026-11-23 A2: 4.9 deg jump) */
    const float lo = RANGE_LOWER[j] - RANGE_TOL_RAD, hi = RANGE_UPPER[j] + RANGE_TOL_RAD;
    if (!(rad >= lo)) rad = lo; /* also catches NaN */
    if (rad > hi) rad = hi;
    const float c = (float)m->zero[j] + (float)SERVO_SIGN[j] * rad * (COUNTS_PER_TURN / TWO_PI);
    int32_t v = (int32_t)lrintf(c);
    if (v > SERVO_POS_LIMIT) v = SERVO_POS_LIMIT;
    if (v < -SERVO_POS_LIMIT) v = -SERVO_POS_LIMIT;
    return v;
}

bool joints_align_turns(joint_map *m, const int32_t counts[NUM_JOINTS], float tol_rad, int *bad_joint) {
    bool ok = true;
    if (bad_joint) *bad_joint = -1;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        int32_t d = counts[j] - SERVO_ZERO[j];
        int32_t turns = (int32_t)lrintf((float)d / COUNTS_PER_TURN);
        m->zero[j] = SERVO_ZERO[j] + turns * COUNTS_PER_TURN;
        const float q = joints_count_to_rad(m, j, counts[j]);
        if (q < RANGE_LOWER[j] - tol_rad || q > RANGE_UPPER[j] + tol_rad) {
            if (ok && bad_joint) *bad_joint = j;
            ok = false;
        }
    }
    m->aligned = ok;
    return ok;
}
