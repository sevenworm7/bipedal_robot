/* Robot-level state machine (portable C; deploy/README.md step 4, decisions #24):
 *
 *   OFF --INIT--> INIT (slow smoothstep from the current pose to the default stance) --> STAND
 *   STAND: fixed stance (targets = DEFAULT_POS, i.e. action 0); the policy is NOT run, but the observation history
 *          keeps being filled (cmd 0, gait frequency 0, last action 0), exactly as in evaluation/load_analysis/
 *          stand_handover.py, which passed 100 % in simulation.
 *   STAND --WALK vx f--> WALK: the student policy drives all 10 joints, gait clock starts at 0, target heading =
 *          heading at the hand-over.
 *   WALK --STAND--> STAND (targets ramp to the stance over STAND_RAMP_S; not tested in simulation yet)
 *   any --STOP / fault--> OFF / FAULT: torque off (the robot is no longer held!)
 *
 * The caller (main_rp2350.c) owns all I/O: it fills ctl_sensors each control period, calls ctl_step, then applies
 * ctl_output (torque enable + goal positions). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "obs.h"
#include "robot_config.h"

typedef enum { ST_OFF = 0, ST_INIT, ST_STAND, ST_WALK, ST_FAULT } ctl_state;

typedef enum {
    FAULT_NONE = 0, FAULT_SENSOR, FAULT_TILT, FAULT_VOLTAGE, FAULT_TEMP, FAULT_TRACKING, FAULT_JOINT_RANGE,
    FAULT_POLICY, FAULT_OVERRUN, FAULT_HOST, FAULT_SERVO_STATUS,
} ctl_fault;

typedef struct {
    bool ok;                     /* every servo and the IMU answered this period */
    float q[NUM_JOINTS];         /* joint angles [rad], model convention */
    float qd[NUM_JOINTS];        /* joint velocities [rad/s] */
    float ang_vel[3];            /* body frame [rad/s], bias removed */
    float gravity[3];            /* body frame unit gravity */
    float heading;               /* [rad] */
    float v_min;                 /* lowest servo voltage [V] */
    int temp_max;                /* hottest servo [C] */
    uint8_t status_or;           /* OR of the servo status bytes */
} ctl_sensors;

typedef struct {
    bool torque;                 /* torque must be enabled on all servos */
    float target[NUM_JOINTS];    /* goal angles [rad] (valid when torque) */
    bool policy_ran;
} ctl_output;

typedef struct {
    float init_peak_deg_s;       /* INIT smoothstep peak joint speed */
    float init_min_s;
    float stand_ramp_s;          /* WALK -> STAND blend */
    float tilt_limit_deg;        /* fall test, as in simulation (40 deg) */
    float v_min_stop;            /* [V] */
    int temp_stop;               /* [C] */
    float track_limit_deg;       /* |q - previous target| */
    int track_ticks;             /* consecutive periods over the limit */
    float range_tol_deg;         /* allowed excursion beyond the URDF joint limits */
    int sensor_fail_ticks;
    float cmd_vx_max, freq_min, freq_max;
} ctl_params;

typedef struct {
    ctl_params p;
    ctl_state state;
    ctl_fault fault;
    int fault_joint;
    uint32_t ticks_in_state;
    float t_state;               /* [s] in the current state */
    float init_from[NUM_JOINTS], init_T;
    float ramp_from[NUM_JOINTS];
    float last_target[NUM_JOINTS];
    bool have_target;
    float last_action[NUM_JOINTS];
    float cmd_vx, gait_freq, clearance;
    float phase, target_heading;
    int track_bad, sensor_bad;
    obs_history hist;
    float obs[OBS_DIM];
    float raw_action[NUM_JOINTS];
    bool pending_init, pending_walk, pending_stand, pending_stop;
    float pend_vx, pend_freq, pend_clr;
} controller;

void ctl_default_params(ctl_params *p);
void ctl_init(controller *c);
/* Commands (applied at the next ctl_step). Return false with a reason if not allowed in the current state. */
bool ctl_cmd_init(controller *c, const char **why);
bool ctl_cmd_walk(controller *c, float vx, float freq, float clearance, const char **why);
bool ctl_cmd_set(controller *c, float vx, float freq, const char **why); /* change the command while walking */
bool ctl_cmd_stand(controller *c, const char **why);
void ctl_cmd_stop(controller *c);
void ctl_host_fault(controller *c, ctl_fault f); /* e.g. USB lost, loop overrun */
/* One control period. dt = actual period [s]. */
void ctl_step(controller *c, const ctl_sensors *s, float dt, ctl_output *out);
const char *ctl_state_name(ctl_state s);
const char *ctl_fault_name(ctl_fault f);
