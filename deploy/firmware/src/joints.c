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
    if (!(rad >= JOINT_LOWER[j])) rad = JOINT_LOWER[j]; /* also catches NaN */
    if (rad > JOINT_UPPER[j]) rad = JOINT_UPPER[j];
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
        if (q < JOINT_LOWER[j] - tol_rad || q > JOINT_UPPER[j] + tol_rad) {
            if (ok && bad_joint) *bad_joint = j;
            ok = false;
        }
    }
    m->aligned = ok;
    return ok;
}
