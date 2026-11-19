/* IMU attitude estimate for the policy observation (portable C).
 *
 * The simulation feeds the policy the true base angular velocity, the true projected gravity and the heading.
 * On the robot: angular velocity = bias-corrected gyro; gravity and heading come from a Mahony complementary filter
 * (gyro integration, accelerometer corrects roll / pitch only; yaw drifts, there is no magnetometer).
 * All vectors here are already in the canonical body frame (+X forward, +Y left, +Z up); the sensor -> body rotation
 * is applied by the caller (IMU_SENSOR_TO_BODY). */
#pragma once
#include <stdbool.h>

typedef struct {
    float q[4];          /* body -> world quaternion (w, x, y, z) */
    float bias[3];       /* gyro bias [rad/s] */
    float integral[3];   /* Mahony integral term */
    float kp, ki;        /* accelerometer correction gains */
    float acc_gate;      /* ignore the accelerometer when | |a| - 1 g | exceeds this [g] */
    bool initialised;
} attitude;

void attitude_init(attitude *a);
/* gyro [rad/s] (raw, bias is subtracted here), acc [g], dt [s]. The first call levels the filter from acc. */
void attitude_update(attitude *a, const float gyro[3], const float acc[3], float dt);
/* Unit gravity direction in the body frame (upright: 0, 0, -1), like Isaac's projected_gravity_b. */
void attitude_gravity(const attitude *a, float g[3]);
/* Yaw [rad] of the body x axis projected on the ground, like Isaac's heading_w. */
float attitude_heading(const attitude *a);
float wrap_angle(float x);
