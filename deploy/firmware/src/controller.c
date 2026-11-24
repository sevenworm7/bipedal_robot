#include "controller.h"

#include <math.h>
#include <string.h>

#include "attitude.h"
#include "policy.h"

#define DEG (3.14159265358979f / 180.0f)

void ctl_default_params(ctl_params *p) {
    p->init_peak_deg_s = 10.0f;
    p->init_min_s = 2.0f;
    p->stand_ramp_s = 0.5f;
    p->tilt_limit_deg = 40.0f;
    p->v_min_stop = 6.5f;
    p->temp_stop = 60;
    p->track_limit_deg = 25.0f;
    p->track_ticks = 3;
    p->range_tol_deg = 10.0f;
    p->sensor_fail_ticks = 3;
    p->cmd_vx_max = CMD_VX_MAX;  /* training range of the exported model (robot_config.h) */
    p->freq_min = GAIT_FREQ_MIN;
    p->freq_max = GAIT_FREQ_MAX;
}

void ctl_init(controller *c) {
    memset(c, 0, sizeof(*c));
    ctl_default_params(&c->p);
    c->state = ST_OFF;
    c->fault_joint = -1;
    obs_reset(&c->hist);
}

const char *ctl_state_name(ctl_state s) {
    static const char *n[] = {"OFF", "INIT", "STAND", "WALK", "FAULT"};
    return (unsigned)s < 5 ? n[s] : "?";
}

const char *ctl_fault_name(ctl_fault f) {
    static const char *n[] = {"NONE", "SENSOR", "TILT", "VOLTAGE", "TEMP", "TRACKING", "JOINT_RANGE",
                              "POLICY", "OVERRUN", "HOST", "SERVO_STATUS"};
    return (unsigned)f < sizeof(n) / sizeof(n[0]) ? n[f] : "?";
}

bool ctl_cmd_init(controller *c, const char **why) {
    if (c->state != ST_OFF) { *why = "INIT only from OFF (STOP clears a FAULT)"; return false; }
#if !CALIB_VERIFIED
#ifndef ALLOW_UNVERIFIED_CALIB
    *why = "calibration not verified (deploy/calibration/robot_v2.json)";
    return false;
#endif
#endif
    c->pending_init = true;
    return true;
}

static bool check_cmd(const controller *c, float vx, float freq, const char **why) {
    if (!(vx >= 0.0f && vx <= c->p.cmd_vx_max)) { *why = "vx outside 0..CMD_VX_MAX of this model (STATUS shows it)"; return false; }
    if (!(freq >= c->p.freq_min && freq <= c->p.freq_max)) { *why = "freq outside GAIT_FREQ_MIN..MAX of this model (STATUS shows it)"; return false; }
    return true;
}

bool ctl_cmd_walk(controller *c, float vx, float freq, float clearance, const char **why) {
    if (c->state != ST_STAND) { *why = "WALK only from STAND"; return false; }
    if (!check_cmd(c, vx, freq, why)) return false;
    if (!(clearance >= 0.0f && clearance <= CLEARANCE_MAX)) { *why = "clearance outside 0..CLEARANCE_MAX"; return false; }
    c->pend_vx = vx; c->pend_freq = freq; c->pend_clr = clearance;
    c->pending_walk = true;
    return true;
}

bool ctl_cmd_set(controller *c, float vx, float freq, const char **why) {
    if (c->state != ST_WALK) { *why = "SET only while walking"; return false; }
    if (!check_cmd(c, vx, freq, why)) return false;
    c->cmd_vx = vx;
    c->gait_freq = freq;
    return true;
}

bool ctl_cmd_stand(controller *c, const char **why) {
    if (c->state != ST_WALK) { *why = "STAND only from WALK"; return false; }
    c->pending_stand = true;
    return true;
}

void ctl_cmd_stop(controller *c) { c->pending_stop = true; }

static void enter(controller *c, ctl_state s) {
    c->state = s;
    c->ticks_in_state = 0;
    c->t_state = 0.0f;
}

static void fault(controller *c, ctl_fault f, int joint) {
    if (c->state == ST_OFF || c->state == ST_FAULT) return;
    c->fault = f;
    c->fault_joint = joint;
    enter(c, ST_FAULT);
}

void ctl_host_fault(controller *c, ctl_fault f) { fault(c, f, -1); }

static float smoothstep(float x) {
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return 1.0f;
    return x * x * (3.0f - 2.0f * x);
}

/* stand / walk observation of this period (also used to keep the history warm while standing) */
static void build_obs(controller *c, const ctl_sensors *s, bool walking) {
    obs_input in;
    memcpy(in.ang_vel, s->ang_vel, sizeof(in.ang_vel));
    memcpy(in.gravity, s->gravity, sizeof(in.gravity));
    in.cmd_vx = walking ? c->cmd_vx : 0.0f;
    in.gait_freq = walking ? c->gait_freq : 0.0f;
    in.heading_error = walking ? wrap_angle(c->target_heading - s->heading) : 0.0f;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        in.joint_pos_rel[j] = s->q[j] - DEFAULT_POS[j];
        in.joint_vel[j] = s->qd[j];
        in.last_action[j] = walking ? c->last_action[j] : 0.0f;
    }
    in.gait_phase = walking ? c->phase : 0.0f;
    in.clearance_cmd = walking ? c->clearance : 0.0f;
    obs_push(&c->hist, &in, c->obs);
}

static void safety(controller *c, const ctl_sensors *s) {
    if (c->state == ST_OFF || c->state == ST_FAULT) return;
    if (!s->ok) {
        if (++c->sensor_bad >= c->p.sensor_fail_ticks) fault(c, FAULT_SENSOR, -1);
        return;
    }
    c->sensor_bad = 0;
    if (s->gravity[2] > -cosf(c->p.tilt_limit_deg * DEG)) { fault(c, FAULT_TILT, -1); return; }
    if (s->v_min < c->p.v_min_stop) { fault(c, FAULT_VOLTAGE, -1); return; }
    if (s->temp_max >= c->p.temp_stop) { fault(c, FAULT_TEMP, -1); return; }
    if (s->status_or) { fault(c, FAULT_SERVO_STATUS, -1); return; }
    const float tol = c->p.range_tol_deg * DEG;
    bool bad_track = false;
    int track_joint = -1;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (s->q[j] < RANGE_LOWER[j] - tol || s->q[j] > RANGE_UPPER[j] + tol) { fault(c, FAULT_JOINT_RANGE, j); return; }
        if (c->have_target && fabsf(s->q[j] - c->last_target[j]) > c->p.track_limit_deg * DEG) {
            bad_track = true;
            track_joint = j;
        }
    }
    c->track_bad = bad_track ? c->track_bad + 1 : 0;
    if (c->track_bad >= c->p.track_ticks) fault(c, FAULT_TRACKING, track_joint);
}

void ctl_step(controller *c, const ctl_sensors *s, float dt, ctl_output *out) {
    memset(out, 0, sizeof(*out));

    /* commands */
    if (c->pending_stop) {
        c->pending_stop = c->pending_init = c->pending_walk = c->pending_stand = false;
        enter(c, ST_OFF);
        c->fault = FAULT_NONE;
        c->fault_joint = -1;
        c->have_target = false;
    }
    if (c->pending_init && c->state == ST_OFF && s->ok) {
        c->pending_init = false;
        float far = 0.0f;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            c->init_from[j] = s->q[j];
            far = fmaxf(far, fabsf(DEFAULT_POS[j] - s->q[j]));
        }
        /* smoothstep peak speed = 1.5 x distance / T */
        c->init_T = fmaxf(c->p.init_min_s, 1.5f * far / (c->p.init_peak_deg_s * DEG));
        c->track_bad = c->sensor_bad = 0;
        c->have_target = false;
        obs_reset(&c->hist);
        enter(c, ST_INIT);
    }
    if (c->pending_walk && c->state == ST_STAND) {
        c->pending_walk = false;
        c->cmd_vx = c->pend_vx;
        c->gait_freq = c->pend_freq;
        c->clearance = c->pend_clr;
        c->phase = 0.0f;
        c->target_heading = s->heading;
        memset(c->last_action, 0, sizeof(c->last_action));
        enter(c, ST_WALK);
    }
    if (c->pending_stand && c->state == ST_WALK) {
        c->pending_stand = false;
        memcpy(c->ramp_from, c->last_target, sizeof(c->ramp_from));
        enter(c, ST_STAND);
    }

    safety(c, s);

    switch (c->state) {
    case ST_OFF:
    case ST_FAULT:
        out->torque = false;
        c->have_target = false;
        break;
    case ST_INIT: {
        const float x = smoothstep(c->t_state / c->init_T);
        for (int j = 0; j < NUM_JOINTS; ++j) out->target[j] = c->init_from[j] + x * (DEFAULT_POS[j] - c->init_from[j]);
        out->torque = true;
        if (s->ok) build_obs(c, s, false);
        if (c->t_state >= c->init_T) {
            memcpy(c->ramp_from, DEFAULT_POS, sizeof(c->ramp_from));
            enter(c, ST_STAND);
        }
        break;
    }
    case ST_STAND: {
        const float x = c->p.stand_ramp_s > 0.0f ? smoothstep(c->t_state / c->p.stand_ramp_s) : 1.0f;
        for (int j = 0; j < NUM_JOINTS; ++j) out->target[j] = c->ramp_from[j] + x * (DEFAULT_POS[j] - c->ramp_from[j]);
        out->torque = true;
        if (s->ok) build_obs(c, s, false);
        break;
    }
    case ST_WALK: {
        out->torque = true;
        if (!s->ok) { /* hold the previous targets for this period; safety() faults after repeated failures */
            memcpy(out->target, c->last_target, sizeof(out->target));
            break;
        }
        build_obs(c, s, true);
        float a[NUM_JOINTS];
        if (!policy_forward(c->obs, a)) {
            fault(c, FAULT_POLICY, -1);
            out->torque = false;
            break;
        }
        memcpy(c->raw_action, a, sizeof(a));
        for (int j = 0; j < NUM_JOINTS; ++j) {
            const float ac = fminf(fmaxf(a[j], -ACTION_CLIP), ACTION_CLIP);
            c->last_action[j] = ac;
            /* policy targets stay inside the training limits (the servo-side clamp uses the wider RANGE_*) */
            out->target[j] = fminf(fmaxf(DEFAULT_POS[j] + ACTION_SCALE * ac, JOINT_LOWER[j]), JOINT_UPPER[j]);
        }
        out->policy_ran = true;
        c->phase = fmodf(c->phase + c->gait_freq * dt, 1.0f);
        break;
    }
    }

    if (out->torque) {
        memcpy(c->last_target, out->target, sizeof(c->last_target));
        c->have_target = true;
    }
    ++c->ticks_in_state;
    c->t_state += dt;
}
