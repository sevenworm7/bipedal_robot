/* Student observation builder, identical to core/interface_v2.py (student group):
 *   frame (42) = base_ang_vel(3) projected_gravity(3) cmd_vx gait_freq heading_error
 *                joint_pos_rel(J) joint_vel(J) last_action(J) sin(2 pi phase) cos(2 pi phase) clearance_cmd
 *   observation = the last OBS_HISTORY frames, newest first; the first frame after a reset fills the whole history.
 * Joint order is ACTION_JOINT_NAMES (robot_config.h). Portable C. */
#pragma once
#include <stdbool.h>

#include "robot_config.h"

typedef struct {
    float ang_vel[3];       /* IMU angular velocity, canonical body frame [rad/s] */
    float gravity[3];       /* unit gravity direction, body frame (upright: 0, 0, -1) */
    float cmd_vx;           /* [m/s] */
    float gait_freq;        /* [Hz], 0 while standing */
    float heading_error;    /* wrap(target - heading) [rad] */
    float joint_pos_rel[NUM_JOINTS]; /* joint angle - DEFAULT_POS [rad] */
    float joint_vel[NUM_JOINTS];     /* [rad/s] */
    float last_action[NUM_JOINTS];   /* previous clipped normalised action */
    float gait_phase;       /* [0, 1) */
    float clearance_cmd;    /* [m] */
} obs_input;

typedef struct {
    float hist[OBS_HISTORY][OBS_FRAME_DIM];
    bool primed;
} obs_history;

void obs_reset(obs_history *h);
void obs_frame(const obs_input *in, float frame[OBS_FRAME_DIM]);
/* Push a new frame and write the flattened observation (OBS_DIM floats, newest first). */
void obs_push(obs_history *h, const obs_input *in, float *obs_out);
