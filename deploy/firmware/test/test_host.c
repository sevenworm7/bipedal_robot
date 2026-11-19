/* Host unit tests for the portable firmware modules (gcc on the PC; no hardware).
 *   test_host <export_dir>
 * Uses golden_mlp.bin / golden_rollout.bin written by deploy/export/export_policy.py and record_golden.py.
 * Built twice by build_host_tests.sh: fp32 policy and int8 policy (-DPOLICY_INT8). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "attitude.h"
#include "controller.h"
#include "feetech.h"
#include "joints.h"
#include "obs.h"
#include "policy.h"

static int failures;
#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            ++failures;                                   \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
        }                                                 \
    } while (0)

static float *load(const char *dir, const char *name, int *hdr, int nhdr, size_t *count) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    FILE *f = fopen(path, "rb");
    if (!f) { printf("  cannot open %s\n", path); ++failures; return NULL; }
    if (fread(hdr, sizeof(int), (size_t)nhdr, f) != (size_t)nhdr) { fclose(f); ++failures; return NULL; }
    fseek(f, 0, SEEK_END);
    long end = ftell(f);
    fseek(f, (long)(nhdr * sizeof(int)), SEEK_SET);
    *count = (size_t)(end - nhdr * (long)sizeof(int)) / sizeof(float);
    float *d = malloc(*count * sizeof(float));
    if (fread(d, sizeof(float), *count, f) != *count) { free(d); fclose(f); ++failures; return NULL; }
    fclose(f);
    return d;
}

#ifdef POLICY_INT8
#define EXPECT_COL 1
#else
#define EXPECT_COL 0
#endif

static void test_golden_mlp(const char *dir) {
    printf("[mlp] golden vectors (%s)\n", policy_kind());
    int n;
    size_t cnt;
    float *d = load(dir, "golden_mlp.bin", &n, 1, &cnt);
    if (!d) return;
    const int row = POLICY_OBS_DIM + 2 * POLICY_ACT_DIM;
    CHECK(cnt == (size_t)n * row, "size %zu", cnt);
    double max_err = 0.0;
    for (int k = 0; k < n; ++k) {
        const float *r = d + (size_t)k * row;
        const float *want = r + POLICY_OBS_DIM + EXPECT_COL * POLICY_ACT_DIM;
        float a[POLICY_ACT_DIM];
        CHECK(policy_forward(r, a), "forward failed at %d", k);
        for (int j = 0; j < POLICY_ACT_DIM; ++j) max_err = fmax(max_err, fabs(a[j] - want[j]));
    }
    printf("  %d vectors, max |C - reference| = %.3g\n", n, max_err);
    CHECK(max_err < 1e-4, "max error %.3g", max_err);
    /* self-test vector used by the BENCH command */
    float a[POLICY_ACT_DIM];
    policy_forward(policy_test_obs(), a);
    double e = 0;
    for (int j = 0; j < POLICY_ACT_DIM; ++j) e = fmax(e, fabs(a[j] - policy_test_act()[j]));
    CHECK(e < 1e-4, "BENCH self-test vector error %.3g", e);
    /* NaN input is rejected without touching the output */
    float bad[POLICY_OBS_DIM];
    memcpy(bad, d, sizeof(bad));
    bad[17] = NAN;
    float keep[POLICY_ACT_DIM] = {42.0f};
    CHECK(!policy_forward(bad, keep) && keep[0] == 42.0f, "NaN not rejected");
    free(d);
}

static void test_golden_rollout(const char *dir) {
    printf("[obs] golden rollout (flat_v1, stand -> walk)\n");
    int h[4];
    size_t cnt;
    float *d = load(dir, "golden_rollout.bin", h, 4, &cnt);
    if (!d) return;
    const int steps = h[0], envs = h[1], J = h[2], od = h[3];
    CHECK(J == NUM_JOINTS && od == OBS_DIM, "J %d obs %d", J, od);
    const int row = 11 + 3 * J + od + J;
    CHECK(cnt == (size_t)steps * envs * row, "size");
    double obs_err = 0, act_err = 0, act_err_walk = 0;
    for (int e = 0; e < envs; ++e) {
        obs_history hist;
        obs_reset(&hist);
        for (int t = 0; t < steps; ++t) {
            const float *r = d + ((size_t)t * envs + e) * row;
            obs_input in;
            memcpy(in.ang_vel, r, 3 * sizeof(float));
            memcpy(in.gravity, r + 3, 3 * sizeof(float));
            in.heading_error = r[6];
            in.cmd_vx = r[7];
            in.gait_freq = r[8];
            in.gait_phase = r[9];
            in.clearance_cmd = r[10];
            for (int j = 0; j < J; ++j) {
                in.joint_pos_rel[j] = r[11 + j] - DEFAULT_POS[j];
                in.joint_vel[j] = r[11 + J + j];
                in.last_action[j] = r[11 + 2 * J + j];
            }
            float obs[OBS_DIM], a[NUM_JOINTS];
            obs_push(&hist, &in, obs);
            const float *want_obs = r + 11 + 3 * J;
            const float *want_act = want_obs + od;
            for (int i = 0; i < od; ++i) obs_err = fmax(obs_err, fabs(obs[i] - want_obs[i]));
            policy_forward(obs, a);
            for (int j = 0; j < J; ++j) {
                act_err = fmax(act_err, fabs(a[j] - want_act[j]));
                if (in.gait_freq > 0) act_err_walk = fmax(act_err_walk, fabs(a[j] - want_act[j]));
            }
        }
    }
    printf("  %d envs x %d steps: max obs error %.3g, max action error %.3g (walking %.3g, %s policy vs PyTorch fp32)\n",
           envs, steps, obs_err, act_err, act_err_walk, policy_kind());
    CHECK(obs_err < 2e-5, "observation mismatch %.3g", obs_err);
#ifdef POLICY_INT8
    CHECK(act_err < 0.1, "int8 action error %.3g", act_err);
#else
    CHECK(act_err < 1e-4, "action mismatch %.3g", act_err);
#endif
    free(d);
}

static void test_protocol(void) {
    printf("[feetech] packets\n");
    uint8_t b[64];
    size_t n = ft_build_read(b, 1, 56, 15);
    const uint8_t want_read[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x0F, 0xB1};
    CHECK(n == 8 && !memcmp(b, want_read, 8), "read packet");
    const uint8_t ping[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}; /* teammate's README: PING id 1 */
    uint8_t s = 0;
    for (int i = 2; i < 5; ++i) s = (uint8_t)(s + ping[i]);
    CHECK((uint8_t)~s == ping[5], "checksum rule");
    uint8_t lo, hi;
    ft_encode_signed(-129, &lo, &hi);
    CHECK(ft_decode_signed(lo, hi) == -129 && hi == 0x80, "signed position -129");
    ft_encode_signed(3967, &lo, &hi);
    CHECK(ft_decode_signed(lo, hi) == 3967, "signed position 3967");
    const uint8_t ids[3] = {1, 2, 3};
    const int32_t goals[3] = {100, -5, 4000};
    n = ft_build_goal_sync(b, ids, goals, 3);
    CHECK(n == 7 + 9 + 1 && b[2] == 0xFE && b[3] == 13 && b[4] == 0x83 && b[5] == 42 && b[6] == 2 && b[7] == 1 &&
              b[8] == 100 && b[11] == 5 && b[12] == 0x80, "sync write layout");
    /* status parse with echo in front */
    uint8_t tx[8];
    size_t tl = ft_build_read(tx, 7, 56, 2);
    uint8_t rx[32];
    memcpy(rx, tx, tl);
    uint8_t st[] = {0xFF, 0xFF, 7, 4, 0, 0x26, 0x01, 0};
    uint8_t sum = 0;
    for (int i = 2; i < 7; ++i) sum = (uint8_t)(sum + st[i]);
    st[7] = (uint8_t)~sum;
    memcpy(rx + tl, st, sizeof(st));
    uint8_t data[2], err = 9;
    CHECK(ft_parse_status(rx, tl + sizeof(st), tx, tl, 7, 2, data, &err) == 1 && err == 0 && data[0] == 0x26,
          "parse after echo");
    CHECK(ft_parse_status(rx, tl + sizeof(st) - 1, tx, tl, 7, 2, data, &err) == 0, "incomplete packet");
    rx[tl + 5] ^= 1;
    CHECK(ft_parse_status(rx, tl + sizeof(st), tx, tl, 7, 2, data, &err) == 0, "bad checksum accepted");
    uint8_t sr[16];
    n = ft_build_sync_read(sr, 56, 15, ids, 3);
    CHECK(n == 11 && sr[3] == 7 && sr[4] == 0x82 && sr[5] == 56 && sr[6] == 15 && sr[9] == 3, "sync read layout");
}

static void test_joints(void) {
    printf("[joints] counts <-> radians, turn alignment\n");
    joint_map m;
    joints_init(&m);
    int32_t c[NUM_JOINTS];
    for (int j = 0; j < NUM_JOINTS; ++j) c[j] = joints_rad_to_count(&m, j, DEFAULT_POS[j]);
    for (int j = 0; j < NUM_JOINTS; ++j)
        CHECK(fabsf(joints_count_to_rad(&m, j, c[j]) - DEFAULT_POS[j]) < 0.002f, "round trip joint %d", j);
    /* the teammate's ID2 case: stored zero 3967, power-up reading 38 (one turn lower than stored) */
    int32_t now[NUM_JOINTS];
    for (int j = 0; j < NUM_JOINTS; ++j) now[j] = SERVO_ZERO[j] + 4096 * ((j % 3) - 1) + 10;
    int bad;
    CHECK(joints_align_turns(&m, now, 0.17f, &bad), "alignment failed at %d", bad);
    for (int j = 0; j < NUM_JOINTS; ++j)
        CHECK(fabsf(joints_count_to_rad(&m, j, now[j])) < 0.02f, "aligned angle joint %d", j);
    now[2] = m.zero[2] + 2048; /* half a turn away: outside every joint range */
    CHECK(!joints_align_turns(&m, now, 0.17f, &bad) && bad == 2, "out-of-range pose not rejected");
    joints_init(&m);
    CHECK(joints_rad_to_count(&m, 0, 10.0f) == joints_rad_to_count(&m, 0, JOINT_UPPER[0]), "target clamp");
    CHECK(joints_rad_to_count(&m, 0, NAN) == joints_rad_to_count(&m, 0, JOINT_LOWER[0]), "NaN target");
}

static void test_attitude(void) {
    printf("[attitude] levelling, gyro integration\n");
    attitude a;
    attitude_init(&a);
    /* body pitched nose-down by 10 deg about +y: specific force (up) in body = (-sin, 0, cos) with pitch +10 */
    const float th = 10.0f * 3.14159265f / 180.0f;
    const float acc[3] = {-sinf(th), 0.0f, cosf(th)}, w0[3] = {0, 0, 0};
    attitude_update(&a, w0, acc, 0.02f);
    float g[3];
    attitude_gravity(&a, g);
    CHECK(fabsf(g[0] - sinf(th)) < 1e-4f && fabsf(g[2] + cosf(th)) < 1e-4f, "levelled gravity %.4f %.4f %.4f", g[0], g[1], g[2]);
    /* start level, then hold a 10 deg roll in the accelerometer: the filter converges within ~5 s */
    attitude_init(&a);
    const float up[3] = {0, 0, 1};
    attitude_update(&a, w0, up, 0.02f);
    const float ph = 10.0f * 3.14159265f / 180.0f, rolled[3] = {0.0f, sinf(ph), cosf(ph)};
    for (int k = 0; k < 250; ++k) attitude_update(&a, w0, rolled, 0.02f);
    attitude_gravity(&a, g);
    CHECK(fabsf(g[1] + sinf(ph)) < 0.01f, "roll convergence g_y %.4f (want %.4f)", g[1], -sinf(ph));
    /* yaw rate 0.5 rad/s for 2 s -> heading 1 rad; accelerometer cannot correct yaw */
    attitude_init(&a);
    attitude_update(&a, w0, up, 0.02f);
    const float wz[3] = {0, 0, 0.5f};
    for (int k = 0; k < 100; ++k) attitude_update(&a, wz, up, 0.02f);
    CHECK(fabsf(attitude_heading(&a) - 1.0f) < 0.01f, "heading %.4f", attitude_heading(&a));
    /* gyro bias is removed */
    attitude_init(&a);
    a.bias[2] = 0.5f;
    attitude_update(&a, w0, up, 0.02f);
    for (int k = 0; k < 100; ++k) attitude_update(&a, wz, up, 0.02f);
    CHECK(fabsf(attitude_heading(&a)) < 1e-3f, "bias not removed: heading %.4f", attitude_heading(&a));
    /* pitch rate integrates with the right sign: +y rotation tips the nose down -> gravity x becomes positive */
    attitude_init(&a);
    a.kp = a.ki = 0.0f;
    attitude_update(&a, w0, up, 0.02f);
    const float wy[3] = {0, 0.5f, 0};
    for (int k = 0; k < 10; ++k) attitude_update(&a, wy, up, 0.02f);
    attitude_gravity(&a, g);
    CHECK(fabsf(g[0] - sinf(0.1f)) < 1e-3f, "pitch integration g_x %.4f (want %.4f)", g[0], sinf(0.1f));
}

static void sensors_at(ctl_sensors *s, const float *q) {
    memset(s, 0, sizeof(*s));
    s->ok = true;
    memcpy(s->q, q, sizeof(s->q));
    s->gravity[2] = -1.0f;
    s->v_min = 7.6f;
    s->temp_max = 35;
}

static void test_controller(void) {
    printf("[controller] state machine\n");
    controller c;
    ctl_init(&c);
    const char *why = "";
    ctl_output out;
    ctl_sensors s;
    float q[NUM_JOINTS];
    for (int j = 0; j < NUM_JOINTS; ++j) q[j] = DEFAULT_POS[j] + 0.2f * ((j & 1) ? 1 : -1);
    q[2] = 0.3f; /* keep the knee in range */
    sensors_at(&s, q);
    CHECK(!ctl_cmd_walk(&c, 0.1f, 1.6f, 0.0f, &why), "WALK from OFF accepted");
    CHECK(ctl_cmd_init(&c, &why), "INIT rejected: %s", why);
    ctl_step(&c, &s, 0.02f, &out);
    CHECK(c.state == ST_INIT && out.torque, "INIT not entered");
    CHECK(fabsf(out.target[0] - q[0]) < 1e-4f, "INIT must start at the current pose");
    int k = 0;
    while (c.state == ST_INIT && k++ < 2000) {
        for (int j = 0; j < NUM_JOINTS; ++j) s.q[j] = out.target[j]; /* perfect tracking */
        ctl_step(&c, &s, 0.02f, &out);
    }
    CHECK(c.state == ST_STAND, "INIT did not finish (%s)", ctl_state_name(c.state));
    printf("  INIT took %.2f s (planned %.2f s)\n", k * 0.02f, c.init_T);
    for (int j = 0; j < NUM_JOINTS; ++j) CHECK(fabsf(out.target[j] - DEFAULT_POS[j]) < 1e-5f, "stand target %d", j);
    CHECK(!out.policy_ran, "policy ran while standing");
    CHECK(!ctl_cmd_walk(&c, 0.3f, 1.6f, 0.0f, &why), "vx 0.3 accepted");
    CHECK(!ctl_cmd_walk(&c, 0.1f, 3.0f, 0.0f, &why), "freq 3.0 accepted");
    CHECK(ctl_cmd_walk(&c, 0.1f, 1.6f, 0.0f, &why), "WALK rejected: %s", why);
    sensors_at(&s, DEFAULT_POS);
    ctl_step(&c, &s, 0.02f, &out);
    CHECK(c.state == ST_WALK && out.policy_ran && out.torque, "WALK not entered");
    /* history: the newest frame carries the walk command, the older ones the stand frames (cmd 0) */
    CHECK(c.obs[6] == 0.1f && c.obs[7] == 1.6f && c.obs[OBS_FRAME_DIM + 6] == 0.0f, "history at the hand-over");
    for (k = 0; k < 10; ++k) {
        for (int j = 0; j < NUM_JOINTS; ++j) s.q[j] = out.target[j];
        ctl_step(&c, &s, 0.02f, &out);
    }
    CHECK(fabsf(c.phase - fmodf(11 * 0.02f * 1.6f, 1.0f)) < 1e-4f, "gait clock %.4f", c.phase);
    /* tilt > 40 deg -> FAULT, torque off */
    s.gravity[0] = sinf(0.8f);
    s.gravity[2] = -cosf(0.8f);
    ctl_step(&c, &s, 0.02f, &out);
    CHECK(c.state == ST_FAULT && c.fault == FAULT_TILT && !out.torque, "tilt fault (%s)", ctl_fault_name(c.fault));
    CHECK(!ctl_cmd_init(&c, &why), "INIT from FAULT accepted");
    ctl_cmd_stop(&c);
    sensors_at(&s, DEFAULT_POS);
    ctl_step(&c, &s, 0.02f, &out);
    CHECK(c.state == ST_OFF && !out.torque, "STOP did not clear the fault");
    /* sensor dropouts: three consecutive failures fault */
    ctl_cmd_init(&c, &why);
    ctl_step(&c, &s, 0.02f, &out);
    s.ok = false;
    for (k = 0; k < 3; ++k) ctl_step(&c, &s, 0.02f, &out);
    CHECK(c.state == ST_FAULT && c.fault == FAULT_SENSOR, "sensor fault (%s)", ctl_fault_name(c.fault));
    /* voltage */
    ctl_cmd_stop(&c);
    sensors_at(&s, DEFAULT_POS);
    ctl_step(&c, &s, 0.02f, &out);
    ctl_cmd_init(&c, &why);
    s.v_min = 6.2f;
    ctl_step(&c, &s, 0.02f, &out);
    CHECK(c.state == ST_FAULT && c.fault == FAULT_VOLTAGE && !out.torque, "voltage fault");
    /* tracking: a joint stuck 30 deg away for 3 periods */
    ctl_cmd_stop(&c);
    sensors_at(&s, DEFAULT_POS);
    ctl_step(&c, &s, 0.02f, &out);
    ctl_cmd_init(&c, &why);
    for (k = 0; k < 200 && c.state != ST_STAND; ++k) ctl_step(&c, &s, 0.02f, &out);
    s.q[3] += 0.55f;
    for (k = 0; k < 3; ++k) ctl_step(&c, &s, 0.02f, &out);
    CHECK(c.state == ST_FAULT && c.fault == FAULT_TRACKING && c.fault_joint == 3, "tracking fault (%s)",
          ctl_fault_name(c.fault));
    /* WALK -> STAND ramps from the last walking targets */
    ctl_cmd_stop(&c);
    sensors_at(&s, DEFAULT_POS);
    ctl_step(&c, &s, 0.02f, &out);
    ctl_cmd_init(&c, &why);
    for (k = 0; k < 200 && c.state != ST_STAND; ++k) ctl_step(&c, &s, 0.02f, &out);
    ctl_cmd_walk(&c, 0.1f, 1.6f, 0.0f, &why);
    for (k = 0; k < 5; ++k) {
        ctl_step(&c, &s, 0.02f, &out);
        for (int j = 0; j < NUM_JOINTS; ++j) s.q[j] = out.target[j];
    }
    float last[NUM_JOINTS];
    memcpy(last, out.target, sizeof(last));
    CHECK(ctl_cmd_stand(&c, &why), "STAND rejected: %s", why);
    ctl_step(&c, &s, 0.02f, &out);
    CHECK(c.state == ST_STAND && !out.policy_ran, "STAND not entered");
    for (int j = 0; j < NUM_JOINTS; ++j) CHECK(fabsf(out.target[j] - last[j]) < 0.02f, "STAND jumped at joint %d", j);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("usage: test_host <export_dir>\n");
        return 2;
    }
    printf("policy %s, calibration verified %d\n", policy_kind(), CALIB_VERIFIED);
    test_protocol();
    test_joints();
    test_attitude();
    test_golden_mlp(argv[1]);
    test_golden_rollout(argv[1]);
    test_controller();
    printf(failures ? "FAILED: %d check(s)\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}
