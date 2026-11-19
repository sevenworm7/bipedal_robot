#include "obs.h"

#include <math.h>
#include <string.h>

#if OBS_FRAME_DIM != 12 + 3 * NUM_JOINTS || OBS_DIM != OBS_HISTORY * OBS_FRAME_DIM
#error "robot_config.h does not match the interface v2 frame layout"
#endif

void obs_reset(obs_history *h) { memset(h, 0, sizeof(*h)); }

void obs_frame(const obs_input *in, float f[OBS_FRAME_DIM]) {
    int k = 0;
    for (int i = 0; i < 3; ++i) f[k++] = in->ang_vel[i];
    for (int i = 0; i < 3; ++i) f[k++] = in->gravity[i];
    f[k++] = in->cmd_vx;
    f[k++] = in->gait_freq;
    f[k++] = in->heading_error;
    for (int i = 0; i < NUM_JOINTS; ++i) f[k++] = in->joint_pos_rel[i];
    for (int i = 0; i < NUM_JOINTS; ++i) f[k++] = in->joint_vel[i];
    for (int i = 0; i < NUM_JOINTS; ++i) f[k++] = in->last_action[i];
    const float ph = 6.28318530717958647692f * in->gait_phase;
    f[k++] = sinf(ph);
    f[k++] = cosf(ph);
    f[k++] = in->clearance_cmd;
}

void obs_push(obs_history *h, const obs_input *in, float *obs_out) {
    float f[OBS_FRAME_DIM];
    obs_frame(in, f);
    if (!h->primed) {
        for (int t = 0; t < OBS_HISTORY; ++t) memcpy(h->hist[t], f, sizeof(f));
        h->primed = true;
    } else {
        memmove(h->hist[1], h->hist[0], sizeof(h->hist[0]) * (OBS_HISTORY - 1));
        memcpy(h->hist[0], f, sizeof(f));
    }
    memcpy(obs_out, h->hist, sizeof(h->hist));
}
