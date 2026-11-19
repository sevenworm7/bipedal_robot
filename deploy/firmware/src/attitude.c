#include "attitude.h"

#include <math.h>
#include <string.h>

void attitude_init(attitude *a) {
    memset(a, 0, sizeof(*a));
    a->q[0] = 1.0f;
    a->kp = 1.0f;
    a->ki = 0.02f;
    a->acc_gate = 0.35f;
}

float wrap_angle(float x) { return atan2f(sinf(x), cosf(x)); }

/* world "up" (0,0,1) expressed in the body frame: third row of R(q) */
static void up_in_body(const float q[4], float v[3]) {
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    v[0] = 2.0f * (x * z - w * y);
    v[1] = 2.0f * (y * z + w * x);
    v[2] = 1.0f - 2.0f * (x * x + y * y);
}

static void level_from_acc(attitude *a, const float acc[3]) {
    /* roll / pitch that align body "up" with the measured specific force, yaw = 0 */
    const float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (n < 1e-6f) return;
    const float ax = acc[0] / n, ay = acc[1] / n, az = acc[2] / n;
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f), cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    a->q[0] = cr * cp;
    a->q[1] = sr * cp;
    a->q[2] = cr * sp;
    a->q[3] = -sr * sp;
    a->initialised = true;
}

void attitude_update(attitude *a, const float gyro[3], const float acc[3], float dt) {
    if (!a->initialised) {
        level_from_acc(a, acc);
        return;
    }
    float w[3] = {gyro[0] - a->bias[0], gyro[1] - a->bias[1], gyro[2] - a->bias[2]};
    const float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (n > 1e-6f && fabsf(n - 1.0f) < a->acc_gate) {
        float v[3];
        up_in_body(a->q, v);
        const float ax = acc[0] / n, ay = acc[1] / n, az = acc[2] / n;
        const float e[3] = {ay * v[2] - az * v[1], az * v[0] - ax * v[2], ax * v[1] - ay * v[0]};
        for (int i = 0; i < 3; ++i) {
            a->integral[i] += a->ki * e[i] * dt;
            w[i] += a->kp * e[i] + a->integral[i];
        }
    }
    float *q = a->q;
    const float hw = 0.5f * dt;
    const float dq0 = -q[1] * w[0] - q[2] * w[1] - q[3] * w[2];
    const float dq1 = q[0] * w[0] + q[2] * w[2] - q[3] * w[1];
    const float dq2 = q[0] * w[1] - q[1] * w[2] + q[3] * w[0];
    const float dq3 = q[0] * w[2] + q[1] * w[1] - q[2] * w[0];
    q[0] += hw * dq0;
    q[1] += hw * dq1;
    q[2] += hw * dq2;
    q[3] += hw * dq3;
    const float qn = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; ++i) q[i] /= qn;
}

void attitude_gravity(const attitude *a, float g[3]) {
    float v[3];
    up_in_body(a->q, v);
    g[0] = -v[0];
    g[1] = -v[1];
    g[2] = -v[2];
}

float attitude_heading(const attitude *a) {
    const float w = a->q[0], x = a->q[1], y = a->q[2], z = a->q[3];
    /* body x axis in the world frame: first column of R(q) */
    const float fx = 1.0f - 2.0f * (y * y + z * z);
    const float fy = 2.0f * (x * y + w * z);
    return atan2f(fy, fx);
}
