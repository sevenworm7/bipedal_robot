/* robot v2 on-board runtime for the MANGO X4 (RP2350): 50 Hz closed loop
 *   read 10 servos + BMI270 -> attitude -> controller (fixed stance / student policy) -> sync-write goal positions.
 *
 * Hardware access follows the teammate's validated firmware (references/mango_x4_handoff_20261117,
 * mango_feetech/scan_readonly.c, right_leg_motion.h, stance_setup/imu_reference.h): FEETECH bus on UART1 GPIO8/9 at
 * 1 Mbps half duplex (the receiver sees our own echo), BMI270 on SPI0 (MISO20, CS21, SCK22, MOSI23; TFT CS33 held high),
 * USB CDC console. Everything robot-specific comes from the generated robot_config.h.
 *
 * USB commands (one per line):
 *   PING | STATUS | TELEM 0|1 | INIT | WALK <vx> <freq> [clearance] | SET <vx> <freq> | STAND | STOP (also ESC / Ctrl-C)
 *   BENCH (policy self-test + timing, OFF only) | BUSTEST <rounds> (read timing, OFF only) | IMUCAL (gyro bias, OFF / STAND)
 *   SCAN [max_id] (read-only bus scan of IDs 0..max_id, default 253, OFF only)
 *   ALIGN (redo the turn alignment, OFF only)
 *   PLAY <rate> [loops] (slow open-loop preview of the simulated gait, STAND only)
 *   VSTOP <volts> (voltage stop threshold 5.5..7.0 V, default 6.5, OFF only, not saved)
 *   JOG <joint_name> <delta_deg> [speed_deg_s] (one servo, +-15 deg, <= 20 deg/s, out-hold-back-hold, OFF only)
 * Torque is switched off on STOP, any fault and USB disconnect: the robot is then NOT held. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/spi.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/uart.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "attitude.h"
#include "bmi270.h"
#include "controller.h"
#include "feetech.h"
#include "joints.h"
#include "policy.h"
#include "robot_config.h"

#define FW_VERSION "BIPED_V2_RUNTIME 0.1"
#define BUS uart1
#define PERIOD_US 20000u
#define READ_TIMEOUT_US 3000u
#define SYNC_READ_TIMEOUT_US 6000u
#define OVERRUN_TICKS 5
#define IMU_CS 21

/* ------------------------------------------------------------------ servo bus */
static uint8_t rxbuf[512];

static void bus_flush(void) {
    while (uart_is_readable(BUS)) (void)uart_getc(BUS);
    uart_get_hw(BUS)->rsr = 0;
}

static void bus_send(const uint8_t *tx, size_t n) {
    bus_flush();
    uart_write_blocking(BUS, tx, n);
    uart_tx_wait_blocking(BUS);
}

/* collect bytes until `want` bytes after the echo arrived or the timeout expires; returns bytes received */
static size_t bus_collect(size_t want, uint32_t timeout_us, bool *uart_err) {
    size_t used = 0;
    const uint64_t until = time_us_64() + timeout_us;
    *uart_err = false;
    while (time_us_64() < until && used < want && used < sizeof(rxbuf)) {
        if (uart_is_readable(BUS)) {
            const uint32_t v = uart_get_hw(BUS)->dr;
            if ((v >> 8) & 15) *uart_err = true;
            rxbuf[used++] = (uint8_t)v;
        }
    }
    return used;
}

static bool read_reg(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out) {
    uint8_t tx[8], err;
    const size_t tl = ft_build_read(tx, id, addr, len);
    bus_send(tx, tl);
    bool uerr;
    const size_t used = bus_collect(tl + 6 + len, READ_TIMEOUT_US, &uerr);
    return !uerr && ft_parse_status(rxbuf, used, tx, tl, id, len, out, &err) == 1 && err == 0;
}

static bool write_verify(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n) {
    uint8_t tx[16], back[4];
    const size_t tl = ft_build_write(tx, id, addr, data, n);
    for (int attempt = 0; attempt < 3; ++attempt) {
        bus_send(tx, tl);
        sleep_us(1500); /* the status reply of the write, discarded by the next flush */
        if (read_reg(id, addr, n, back) && !memcmp(back, data, n)) return true;
    }
    return false;
}

static const uint8_t *ids(void) { return SERVO_ID; }

static bool use_sync_read;
static ft_feedback fb[NUM_JOINTS];
static unsigned bus_fail_total;

static bool read_feedback_seq(void) {
    bool ok = true;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        uint8_t d[FT_FEEDBACK_LEN];
        if (read_reg(SERVO_ID[j], FT_REG_FEEDBACK, FT_FEEDBACK_LEN, d)) ft_decode_feedback(d, &fb[j]);
        else ok = false;
    }
    return ok;
}

static bool read_feedback_sync(void) {
    uint8_t tx[32], d[FT_FEEDBACK_LEN], err;
    const size_t tl = ft_build_sync_read(tx, FT_REG_FEEDBACK, FT_FEEDBACK_LEN, ids(), NUM_JOINTS);
    bus_send(tx, tl);
    bool uerr;
    const size_t used = bus_collect(tl + (size_t)NUM_JOINTS * (6 + FT_FEEDBACK_LEN), SYNC_READ_TIMEOUT_US, &uerr);
    if (uerr) return false;
    bool ok = true;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (ft_parse_status(rxbuf, used, tx, tl, SERVO_ID[j], FT_FEEDBACK_LEN, d, &err) == 1 && err == 0)
            ft_decode_feedback(d, &fb[j]);
        else ok = false;
    }
    return ok;
}

static bool read_feedback(void) {
    const bool ok = use_sync_read ? read_feedback_sync() : read_feedback_seq();
    if (!ok) ++bus_fail_total;
    return ok;
}

static void write_goals(const int32_t *goals) {
    uint8_t tx[64];
    const size_t n = ft_build_goal_sync(tx, ids(), goals, NUM_JOINTS);
    bus_send(tx, n); /* broadcast: no reply */
}

static bool torque_all(uint8_t on) {
    bool all = true;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        const bool ok = write_verify(SERVO_ID[j], FT_REG_TORQUE, &on, 1);
        if (!ok) printf("TORQUE %s id=%u UNCONFIRMED\n", on ? "ON" : "OFF", SERVO_ID[j]);
        all &= ok;
    }
    return all;
}

/* identity 10/31, own ID, position mode, torque off */
static bool precheck(void) {
    bool ok = true;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        uint8_t ident[3], mode, torque;
        const uint8_t id = SERVO_ID[j];
        if (!read_reg(id, FT_REG_MODEL_L, 3, ident) || ident[0] != 10 || ident[1] != 31 || ident[2] != id ||
            !read_reg(id, FT_REG_MODE, 1, &mode) || mode != 4 || !read_reg(id, FT_REG_TORQUE, 1, &torque) || torque) {
            printf("ERROR PRECHECK id=%u (%s)\n", id, JOINT_NAMES[j]);
            ok = false;
        }
    }
    return ok;
}

/* ------------------------------------------------------------------ IMU (BMI270) */
static struct bmi2_dev imu;
static bool imu_ready;

static BMI2_INTF_RETURN_TYPE spi_read_cb(uint8_t reg, uint8_t *data, uint32_t len, void *ctx) {
    (void)ctx;
    gpio_put(IMU_CS, 0);
    busy_wait_us_32(1);
    const int a = spi_write_blocking(spi0, &reg, 1);
    const int b = spi_read_blocking(spi0, 0, data, len);
    busy_wait_us_32(1);
    gpio_put(IMU_CS, 1);
    return a == 1 && b == (int)len ? BMI2_INTF_RET_SUCCESS : -1;
}

static BMI2_INTF_RETURN_TYPE spi_write_cb(uint8_t reg, const uint8_t *data, uint32_t len, void *ctx) {
    (void)ctx;
    gpio_put(IMU_CS, 0);
    busy_wait_us_32(1);
    const int a = spi_write_blocking(spi0, &reg, 1);
    const int b = spi_write_blocking(spi0, data, len);
    busy_wait_us_32(1);
    gpio_put(IMU_CS, 1);
    return a == 1 && b == (int)len ? BMI2_INTF_RET_SUCCESS : -1;
}

static void delay_cb(uint32_t us, void *ctx) { (void)ctx; sleep_us(us); }

static bool imu_init(void) {
    gpio_init(IMU_CS); gpio_put(IMU_CS, 1); gpio_set_dir(IMU_CS, GPIO_OUT);
    gpio_init(33); gpio_put(33, 1); gpio_set_dir(33, GPIO_OUT); /* TFT chip select shares SPI0 */
    spi_init(spi0, 1000000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(20, GPIO_FUNC_SPI); gpio_set_function(22, GPIO_FUNC_SPI); gpio_set_function(23, GPIO_FUNC_SPI);
    sleep_ms(10);
    memset(&imu, 0, sizeof(imu));
    imu.intf = BMI2_SPI_INTF; imu.read = spi_read_cb; imu.write = spi_write_cb; imu.delay_us = delay_cb;
    imu.read_write_len = 32;
    if (bmi270_init(&imu) != BMI2_OK || imu.chip_id != BMI270_CHIP_ID) return false;
    if (bmi2_set_adv_power_save(BMI2_DISABLE, &imu) != BMI2_OK) return false;
    struct bmi2_sens_config cfg[2] = {{0}};
    cfg[0].type = BMI2_ACCEL; cfg[1].type = BMI2_GYRO;
    if (bmi270_get_sensor_config(cfg, 2, &imu) != BMI2_OK) return false;
    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ; cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4; cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ; cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_500;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE; cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    if (bmi270_set_sensor_config(cfg, 2, &imu) != BMI2_OK) return false;
    const uint8_t sensors[] = {BMI2_ACCEL, BMI2_GYRO};
    if (bmi270_sensor_enable(sensors, 2, &imu) != BMI2_OK) return false;
    sleep_ms(100);
    return true;
}

/* body-frame gyro [rad/s] and accelerometer [g] */
static bool imu_read(float gyro[3], float acc[3]) {
    if (!imu_ready) return false;
    struct bmi2_sens_data d = {0};
    if (bmi2_get_sensor_data(&d, &imu) != BMI2_OK) return false;
    const float gs[3] = {d.gyr.x / 65.536f * 0.0174532925f, d.gyr.y / 65.536f * 0.0174532925f,
                         d.gyr.z / 65.536f * 0.0174532925f};
    const float as[3] = {d.acc.x / 8192.0f, d.acc.y / 8192.0f, d.acc.z / 8192.0f};
    for (int i = 0; i < 3; ++i) {
        gyro[i] = IMU_SENSOR_TO_BODY[i][0] * gs[0] + IMU_SENSOR_TO_BODY[i][1] * gs[1] + IMU_SENSOR_TO_BODY[i][2] * gs[2];
        acc[i] = IMU_SENSOR_TO_BODY[i][0] * as[0] + IMU_SENSOR_TO_BODY[i][1] * as[1] + IMU_SENSOR_TO_BODY[i][2] * as[2];
    }
    return true;
}

/* ------------------------------------------------------------------ state */
static controller ctl;
static joint_map jmap;
static attitude att;
static bool torque_on, telemetry, have_prev_q;
static float prev_q[NUM_JOINTS];
static float last_gyro[3], last_acc[3];
static uint32_t loop_us, bus_us, ctl_us;
static int overrun_count;

static void gyro_calibrate(float seconds) {
    float sum[3] = {0}, g[3], a[3];
    int n = 0;
    const uint64_t end = time_us_64() + (uint64_t)(seconds * 1e6f);
    while (time_us_64() < end) {
        if (imu_read(g, a)) { for (int i = 0; i < 3; ++i) sum[i] += g[i]; ++n; }
        sleep_ms(5);
    }
    if (n > 10) for (int i = 0; i < 3; ++i) att.bias[i] = sum[i] / n;
    printf("IMUCAL samples=%d bias_rad_s=%.5f %.5f %.5f (robot must be still)\n", n, att.bias[0], att.bias[1], att.bias[2]);
}

static void sense(ctl_sensors *s, float dt) {
    memset(s, 0, sizeof(*s));
    const uint64_t t0 = time_us_64();
    const bool bus_ok = read_feedback();
    bus_us = (uint32_t)(time_us_64() - t0);
    float gyro[3], acc[3];
    const bool imu_ok = imu_read(gyro, acc);
    if (imu_ok) {
        attitude_update(&att, gyro, acc, dt);
        memcpy(last_gyro, gyro, sizeof(gyro));
        memcpy(last_acc, acc, sizeof(acc));
    }
    s->ok = bus_ok && imu_ok && jmap.aligned;
    for (int i = 0; i < 3; ++i) s->ang_vel[i] = last_gyro[i] - att.bias[i];
    attitude_gravity(&att, s->gravity);
    s->heading = attitude_heading(&att);
    s->v_min = 99.0f;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        s->q[j] = joints_count_to_rad(&jmap, j, fb[j].position);
        s->qd[j] = (have_prev_q && dt > 0.0f) ? (s->q[j] - prev_q[j]) / dt : 0.0f; /* JOINT_VEL_SOURCE 0 */
        s->v_min = fminf(s->v_min, fb[j].voltage_raw * 0.1f);
        if (fb[j].temp_c > s->temp_max) s->temp_max = fb[j].temp_c;
        s->status_or |= fb[j].status;
    }
    if (bus_ok) { memcpy(prev_q, s->q, sizeof(prev_q)); have_prev_q = true; }
}

static void apply(const ctl_output *o, const ctl_sensors *s) {
    if (o->torque) {
        int32_t goals[NUM_JOINTS];
        for (int j = 0; j < NUM_JOINTS; ++j) goals[j] = joints_rad_to_count(&jmap, j, o->target[j]);
        write_goals(goals);
        if (!torque_on) {
            torque_on = true; /* set first: a partial enable must still be switched off */
            if (!torque_all(1)) {
                ctl_host_fault(&ctl, FAULT_SERVO_STATUS);
                puts("ERROR TORQUE_ON_VERIFY");
            } else puts("TORQUE ON");
        }
    } else if (torque_on) {
        const bool off = torque_all(0);
        torque_on = !off;
        printf("TORQUE OFF %s state=%s fault=%s joint=%d\n", off ? "TORQUE_OFF_VERIFIED" : "UNCONFIRMED",
               ctl_state_name(ctl.state), ctl_fault_name(ctl.fault), ctl.fault_joint);
    }
    (void)s;
}

static void print_status(const ctl_sensors *s) {
    printf("STATUS %s state=%s fault=%s joint=%d torque=%d calib_verified=%d imu=%d sync_read=%d aligned=%d "
           "loop_us=%lu bus_us=%lu ctl_us=%lu bus_fail=%u vmin=%.1f tmax=%d policy=%s\n",
           FW_VERSION, ctl_state_name(ctl.state), ctl_fault_name(ctl.fault), ctl.fault_joint, torque_on, CALIB_VERIFIED,
           imu_ready, use_sync_read, jmap.aligned, (unsigned long)loop_us, (unsigned long)bus_us, (unsigned long)ctl_us,
           bus_fail_total, s->v_min, s->temp_max, policy_kind());
    printf("POSE");
    for (int j = 0; j < NUM_JOINTS; ++j) printf(" %s=%d/%.1fdeg", JOINT_NAMES[j], (int)fb[j].position, s->q[j] * 57.2958f);
    printf("\nIMU gravity=%.3f %.3f %.3f heading_deg=%.1f\n", s->gravity[0], s->gravity[1], s->gravity[2],
           s->heading * 57.2958f);
}

static void telemetry_line(const ctl_sensors *s, const ctl_output *o) {
    printf("T %lu %d %lu %lu %lu", (unsigned long)(time_us_64() / 1000), ctl.state, (unsigned long)loop_us,
           (unsigned long)bus_us, (unsigned long)ctl_us);
    for (int i = 0; i < 3; ++i) printf(" %.4f", s->ang_vel[i]);
    for (int i = 0; i < 3; ++i) printf(" %.4f", s->gravity[i]);
    printf(" %.4f %.3f", s->heading, ctl.phase);
    for (int j = 0; j < NUM_JOINTS; ++j) printf(" %.4f", s->q[j]);
    for (int j = 0; j < NUM_JOINTS; ++j) printf(" %.4f", o->target[j]);
    for (int j = 0; j < NUM_JOINTS; ++j) printf(" %ld", (long)fb[j].speed_raw);
    printf(" %.1f %d", s->v_min, s->temp_max);
    /* 2026-11-23: per-servo load (60-61) and current (69-70) registers, raw (units not verified) */
    for (int j = 0; j < NUM_JOINTS; ++j) printf(" %ld", (long)fb[j].load_raw);
    for (int j = 0; j < NUM_JOINTS; ++j) printf(" %ld", (long)fb[j].current_raw);
    printf("\n");
}

static void bench(void) {
    float a[POLICY_ACT_DIM];
    uint32_t worst = 0, total = 0;
    const int n = 20;
    bool ok = true;
    for (int k = 0; k < n; ++k) {
        const uint64_t t0 = time_us_64();
        ok &= policy_forward(policy_test_obs(), a);
        const uint32_t us = (uint32_t)(time_us_64() - t0);
        total += us;
        if (us > worst) worst = us;
    }
    float err = 0.0f;
    for (int j = 0; j < POLICY_ACT_DIM; ++j) err = fmaxf(err, fabsf(a[j] - policy_test_act()[j]));
    printf("BENCH policy=%s ok=%d max_abs_err=%.3g mean_us=%lu max_us=%lu %s\n", policy_kind(), ok, err,
           (unsigned long)(total / n), (unsigned long)worst, ok && err < POLICY_REF_TOL ? "PASS" : "FAIL");
}

static void bustest(int rounds) {
    for (int mode = 0; mode < 2; ++mode) {
        const bool saved = use_sync_read;
        use_sync_read = mode == 1;
        uint32_t worst = 0, total = 0;
        int fails = 0;
        for (int k = 0; k < rounds; ++k) {
            const uint64_t t0 = time_us_64();
            if (!(use_sync_read ? read_feedback_sync() : read_feedback_seq())) ++fails;
            const uint32_t us = (uint32_t)(time_us_64() - t0);
            total += us;
            if (us > worst) worst = us;
            sleep_ms(2);
        }
        printf("BUSTEST mode=%s rounds=%d fails=%d mean_us=%lu max_us=%lu\n", mode ? "sync_read" : "sequential", rounds,
               fails, (unsigned long)(total / rounds), (unsigned long)worst);
        use_sync_read = saved;
    }
}

/* Single-joint bench move (2026-11-23): torque on ONE servo only, goal ramps current -> current + delta -> back.
 * Safety: OFF state only, calibration aligned, |delta| <= 15 deg, speed <= 20 deg/s, the target inside the joint
 * limits, goal set to the present position before torque-on, any byte from the host aborts with torque off.
 * Prints "JOG t_ms goal_count pos_count speed_raw load_raw volt_raw" at 100 Hz for tracking / latency analysis. */
#define JOG_MAX_DEG 15.0f
#define JOG_MAX_SPEED 20.0f
static bool write_goal_one(uint8_t id, int32_t goal) {
    uint8_t tx[16], d[2];
    ft_encode_signed(goal, &d[0], &d[1]);
    bus_send(tx, ft_build_write(tx, id, FT_REG_GOAL, d, 2));
    sleep_us(400); /* status reply of the write, dropped by the next flush */
    return true;
}

static void jog(const char *name, float delta_deg, float speed_deg_s) {
    int j = -1;
    for (int k = 0; k < NUM_JOINTS; ++k)
        if (!strcmp(name, JOINT_NAMES[k])) j = k;
    if (j < 0) { puts("ERROR JOG unknown joint"); return; }
    if (!jmap.aligned) { puts("ERROR JOG calibration not aligned"); return; }
    if (fabsf(delta_deg) > JOG_MAX_DEG || fabsf(delta_deg) < 0.5f) { puts("ERROR JOG |delta| must be 0.5..15 deg"); return; }
    if (speed_deg_s <= 0.0f || speed_deg_s > JOG_MAX_SPEED) { puts("ERROR JOG speed must be 0..20 deg/s"); return; }
    const uint8_t id = SERVO_ID[j];
    uint8_t d[FT_FEEDBACK_LEN], torque = 1;
    if (!read_reg(id, FT_REG_TORQUE, 1, &torque) || torque) { puts("ERROR JOG torque already on / no reply"); return; }
    if (!read_reg(id, FT_REG_FEEDBACK, FT_FEEDBACK_LEN, d)) { puts("ERROR JOG feedback"); return; }
    ft_feedback f;
    ft_decode_feedback(d, &f);
    const float q0 = joints_count_to_rad(&jmap, j, f.position);
    const float q1 = q0 + delta_deg * 0.0174533f;
    if (q1 < RANGE_LOWER[j] || q1 > RANGE_UPPER[j]) {
        printf("ERROR JOG target %.1f deg outside [%.1f, %.1f]\n", q1 * 57.2958f, RANGE_LOWER[j] * 57.2958f,
               RANGE_UPPER[j] * 57.2958f);
        return;
    }
    const int32_t c0 = f.position, c1 = joints_rad_to_count(&jmap, j, q1);
    write_goal_one(id, c0);
    const uint8_t on = 1, off = 0;
    if (!write_verify(id, FT_REG_TORQUE, &on, 1)) { (void)write_verify(id, FT_REG_TORQUE, &off, 1); puts("ERROR JOG torque on"); return; }
    printf("JOG start joint=%s id=%u q0=%.2fdeg target=%.2fdeg c0=%ld c1=%ld speed=%.1fdeg/s\n", name, id, q0 * 57.2958f,
           q1 * 57.2958f, (long)c0, (long)c1, speed_deg_s);
    const float ramp_s = fabsf(delta_deg) / speed_deg_s, hold_s = 1.0f;
    const float phases[4] = {ramp_s, hold_s, ramp_s, hold_s}; /* out, hold, back, hold */
    const uint64_t t0 = time_us_64();
    bool aborted = false;
    int fails = 0;
    float t_end = 0.0f;
    for (int p = 0; p < 4 && !aborted; ++p) {
        const float tp0 = t_end;
        t_end += phases[p];
        for (;;) {
            const uint64_t tick = time_us_64();
            const float t = (float)(tick - t0) * 1e-6f;
            if (t >= t_end) break;
            if (getchar_timeout_us(0) >= 0) { aborted = true; break; }
            float a = (t - tp0) / phases[p];
            float frac = p == 0 ? a : p == 1 ? 1.0f : p == 2 ? 1.0f - a : 0.0f;
            const int32_t goal = c0 + (int32_t)lrintf((float)(c1 - c0) * frac);
            write_goal_one(id, goal);
            if (read_reg(id, FT_REG_FEEDBACK, FT_FEEDBACK_LEN, d)) {
                ft_decode_feedback(d, &f);
                printf("JOG %lu %ld %ld %ld %ld %u\n", (unsigned long)((tick - t0) / 1000), (long)goal, (long)f.position,
                       (long)f.speed_raw, (long)f.load_raw, f.voltage_raw);
            } else if (++fails >= 5) { aborted = true; break; }
            sleep_until(from_us_since_boot(tick + 10000));
        }
    }
    const bool off_ok = write_verify(id, FT_REG_TORQUE, &off, 1);
    printf("JOG done %s fails=%d torque_off=%s\n", aborted ? "ABORTED" : "OK", fails, off_ok ? "VERIFIED" : "UNCONFIRMED");
}

/* Slow open-loop preview of the simulated walking gait (2026-11-23): STAND only (torque on). Ramps the stance to the
 * first frame (1.5 s), plays PLAYBACK (deploy/export/make_playback.py) at `rate` x real time `loops` times, ramps back
 * to the stance (1.5 s) and stays in STAND. Aborts with torque off on any host byte, 3 bus failures, voltage below the
 * controller limit, servo temperature limit or tracking error > track limit for 3 ticks.
 * Prints "PL t_ms frame v_min max_err_deg worst_joint" at 10 Hz and the per-joint max tracking error at the end. */
#if defined(__has_include)
#if __has_include("playback.h")
#include "playback.h"
#define HAVE_PLAYBACK 1
#endif
#endif
static void play(float rate, int loops) {
#ifndef HAVE_PLAYBACK
    (void)rate;
    (void)loops;
    puts("ERROR PLAY no playback.h in this build (deploy/export/make_playback.py)");
#else
    if (ctl.state != ST_STAND || !torque_on) { puts("ERROR PLAY only in STAND (INIT first)"); return; }
    if (!(rate >= 0.05f && rate <= 1.0f) || loops < 1 || loops > 5) { puts("ERROR PLAY rate 0.05..1, loops 1..5"); return; }
    const float ramp_s = 1.5f, play_s = (float)PLAYBACK_FRAMES / 50.0f / rate * (float)loops;
    const float total_s = 2.0f * ramp_s + play_s;
    printf("PLAY start rate=%.2f loops=%d frames=%d duration=%.1fs\n", rate, loops, PLAYBACK_FRAMES, total_s);
    float max_err[NUM_JOINTS] = {0};
    float v_low = 99.0f;
    int fails = 0, track_bad = 0, tick = 0;
    const char *why = NULL;
    const uint64_t t0 = time_us_64();
    for (;;) {
        const uint64_t now = time_us_64();
        const float t = (float)(now - t0) * 1e-6f;
        if (t >= total_s) break;
        if (getchar_timeout_us(0) >= 0) { why = "host"; break; }
        float target[NUM_JOINTS];
        int frame = 0;
        if (t < ramp_s || t >= ramp_s + play_s) {
            const bool in = t < ramp_s;
            const float a = in ? t / ramp_s : (t - ramp_s - play_s) / ramp_s;
            const float x = a * a * (3.0f - 2.0f * a);
            const float *edge = in ? PLAYBACK[0] : PLAYBACK[PLAYBACK_FRAMES - 1];
            for (int j = 0; j < NUM_JOINTS; ++j)
                target[j] = in ? DEFAULT_POS[j] + x * (edge[j] - DEFAULT_POS[j]) : edge[j] + x * (DEFAULT_POS[j] - edge[j]);
            frame = in ? 0 : PLAYBACK_FRAMES - 1;
        } else {
            const float f = fmodf((t - ramp_s) * rate * 50.0f, (float)PLAYBACK_FRAMES);
            const int k = (int)f;
            const int k1 = k + 1 < PLAYBACK_FRAMES ? k + 1 : k;
            const float w = f - (float)k;
            for (int j = 0; j < NUM_JOINTS; ++j) target[j] = PLAYBACK[k][j] + w * (PLAYBACK[k1][j] - PLAYBACK[k][j]);
            frame = k;
        }
        int32_t goals[NUM_JOINTS];
        for (int j = 0; j < NUM_JOINTS; ++j) goals[j] = joints_rad_to_count(&jmap, j, target[j]);
        write_goals(goals);
        if (!read_feedback()) {
            if (++fails >= 3) { why = "bus"; break; }
        } else {
            fails = 0;
            float worst = 0.0f, vmin = 99.0f;
            int wj = 0, tmax = 0;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                const float e = fabsf(joints_count_to_rad(&jmap, j, fb[j].position) - target[j]) * 57.2958f;
                if (e > max_err[j]) max_err[j] = e;
                if (e > worst) { worst = e; wj = j; }
                const float v = fb[j].voltage_raw * 0.1f;
                if (v < vmin) vmin = v;
                if (fb[j].temp_c > tmax) tmax = fb[j].temp_c;
            }
            if (vmin < v_low) v_low = vmin;
            track_bad = worst > ctl.p.track_limit_deg ? track_bad + 1 : 0;
            if (vmin < ctl.p.v_min_stop) { why = "voltage"; break; }
            if (tmax >= ctl.p.temp_stop) { why = "temperature"; break; }
            if (track_bad >= 3) { why = "tracking"; break; }
            if (tick % 5 == 0)
                printf("PL %lu %d %.1f %.2f %s\n", (unsigned long)((now - t0) / 1000), frame, vmin, worst, JOINT_NAMES[wj]);
        }
        ++tick;
        sleep_until(from_us_since_boot(now + PERIOD_US));
    }
    if (why) {
        ctl_cmd_stop(&ctl);
        const bool off = torque_all(0);
        torque_on = !off;
        printf("PLAY ABORTED reason=%s torque_off=%s\n", why, off ? "VERIFIED" : "UNCONFIRMED");
    }
    printf("PLAY done v_min=%.1f max_err_deg", v_low);
    for (int j = 0; j < NUM_JOINTS; ++j) printf(" %s=%.2f", JOINT_NAMES[j], max_err[j]);
    printf("\n");
#endif
}

/* read-only bus scan: every responding ID with identity, mode, torque flag and feedback (no writes) */
static void scan(int max_id) {
    int found = 0;
    if (max_id < 0 || max_id > 253) max_id = 253;
    for (int id = 0; id <= max_id; ++id) {
        uint8_t ident[3], mode = 255, torque = 255, d[FT_FEEDBACK_LEN];
        if (!read_reg((uint8_t)id, FT_REG_MODEL_L, 3, ident)) continue;
        ft_feedback f;
        memset(&f, 0, sizeof(f));
        (void)read_reg((uint8_t)id, FT_REG_MODE, 1, &mode);
        (void)read_reg((uint8_t)id, FT_REG_TORQUE, 1, &torque);
        const bool fok = read_reg((uint8_t)id, FT_REG_FEEDBACK, FT_FEEDBACK_LEN, d);
        if (fok) ft_decode_feedback(d, &f);
        printf("SCAN id=%d model=%u/%u reg_id=%u mode=%u torque=%u pos=%ld speed=%ld volt=%.1f temp=%u status=%u%s\n",
               id, ident[0], ident[1], ident[2], mode, torque, (long)f.position, (long)f.speed_raw,
               f.voltage_raw * 0.1f, f.temp_c, f.status, fok ? "" : " feedback=FAIL");
        ++found;
    }
    printf("SCAN done found=%d range=0..%d\n", found, max_id);
}

static void dispatch(char *line, const ctl_sensors *s) {
    char *argv[6];
    int argc = 0;
    for (char *p = strtok(line, " \t"); p && argc < 6; p = strtok(NULL, " \t")) argv[argc++] = p;
    if (!argc) return;
    const char *why = "";
    const char *c = argv[0];
    if (!strcmp(c, "STOP")) { ctl_cmd_stop(&ctl); puts("ACK STOP"); return; }
    if (!strcmp(c, "PING")) { printf("PONG %s\n", FW_VERSION); return; }
    if (!strcmp(c, "STATUS")) { print_status(s); return; }
    if (!strcmp(c, "TELEM") && argc == 2) { telemetry = atoi(argv[1]) != 0; printf("ACK TELEM %d\n", telemetry); return; }
    if (!strcmp(c, "STAND")) { printf(ctl_cmd_stand(&ctl, &why) ? "ACK STAND\n" : "ERROR STAND %s\n", why); return; }
    if (!strcmp(c, "SET") && argc == 3) {
        printf(ctl_cmd_set(&ctl, strtof(argv[1], NULL), strtof(argv[2], NULL), &why) ? "ACK SET\n" : "ERROR SET %s\n", why);
        return;
    }
    if (!strcmp(c, "WALK") && (argc == 3 || argc == 4)) {
        const float clr = argc == 4 ? strtof(argv[3], NULL) : 0.0f;
        printf(ctl_cmd_walk(&ctl, strtof(argv[1], NULL), strtof(argv[2], NULL), clr, &why) ? "ACK WALK\n" : "ERROR WALK %s\n",
               why);
        return;
    }
    if (!strcmp(c, "PLAY") && (argc == 2 || argc == 3)) {
        play(strtof(argv[1], NULL), argc == 3 ? atoi(argv[2]) : 1);
        return;
    }
    if (ctl.state != ST_OFF && strcmp(c, "IMUCAL")) { puts("ERROR BUSY (STOP first)"); return; }
    if (!strcmp(c, "IMUCAL")) {
        if (ctl.state == ST_WALK || ctl.state == ST_INIT) { puts("ERROR IMUCAL only in OFF / STAND"); return; }
        gyro_calibrate(2.0f);
        return;
    }
    if (!strcmp(c, "BENCH")) { bench(); return; }
    if (!strcmp(c, "VSTOP") && argc == 2) { /* OFF only, RAM only (6.5 V again after reboot); for hanging tests */
        const float v = strtof(argv[1], NULL);
        if (!(v >= 5.5f && v <= 7.0f)) { puts("ERROR VSTOP 5.5..7.0 V"); return; }
        ctl.p.v_min_stop = v;
        printf("ACK VSTOP %.2f\n", v);
        return;
    }
    if (!strcmp(c, "BUSTEST")) { bustest(argc == 2 ? atoi(argv[1]) : 100); return; }
    if (!strcmp(c, "SCAN")) { scan(argc == 2 ? atoi(argv[1]) : 253); return; }
    if (!strcmp(c, "ALIGN")) { /* redo the boot-time turn alignment (OFF only), e.g. after moving a joint into range */
        int bad = -1;
        if (read_feedback()) {
            int32_t counts[NUM_JOINTS];
            for (int j = 0; j < NUM_JOINTS; ++j) counts[j] = fb[j].position;
            joints_align_turns(&jmap, counts, RANGE_TOL_RAD, &bad);
        }
        printf("ALIGN aligned=%d bad_joint=%s\n", jmap.aligned, bad >= 0 ? JOINT_NAMES[bad] : "none");
        return;
    }
    if (!strcmp(c, "JOG") && (argc == 3 || argc == 4)) {
        jog(argv[1], strtof(argv[2], NULL), argc == 4 ? strtof(argv[3], NULL) : 10.0f);
        return;
    }
    if (!strcmp(c, "INIT")) {
        if (!precheck()) { puts("ERROR INIT precheck failed"); return; }
        int32_t counts[NUM_JOINTS];
        if (!read_feedback()) { puts("ERROR INIT feedback"); return; }
        for (int j = 0; j < NUM_JOINTS; ++j) counts[j] = fb[j].position;
        int bad;
        if (!joints_align_turns(&jmap, counts, RANGE_TOL_RAD, &bad)) {
            printf("ERROR INIT joint %s outside its range (check calibration / pose)\n", JOINT_NAMES[bad]);
            return;
        }
        printf(ctl_cmd_init(&ctl, &why) ? "ACK INIT\n" : "ERROR INIT %s\n", why);
        return;
    }
    puts("ERROR SYNTAX");
}

int main(void) {
    stdio_init_all();
    uart_init(BUS, 1000000);
    gpio_set_function(8, GPIO_FUNC_UART);
    gpio_set_function(9, GPIO_FUNC_UART);
    uart_set_hw_flow(BUS, false, false);
    uart_set_format(BUS, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(BUS, true);
    pads_bank0_hw->io[8] = 0x56; /* as in the teammate's firmware */
    pads_bank0_hw->io[9] = 0x56;

    ctl_init(&ctl);
    joints_init(&jmap);
    attitude_init(&att);
    imu_ready = imu_init();
    use_sync_read = read_feedback_sync(); /* falls back to sequential reads if the servos ignore SYNC READ */
    if (read_feedback()) {
        int32_t counts[NUM_JOINTS];
        int bad;
        for (int j = 0; j < NUM_JOINTS; ++j) counts[j] = fb[j].position;
        joints_align_turns(&jmap, counts, RANGE_TOL_RAD, &bad);
    }
    gyro_calibrate(2.0f);

    char line[96];
    unsigned used = 0;
    bool connected = false;
    ctl_sensors s;
    ctl_output o;
    memset(&s, 0, sizeof(s));
    memset(&o, 0, sizeof(o));
    uint64_t next = time_us_64(), last = next;
    for (;;) {
        sleep_until(from_us_since_boot(next));
        const uint64_t t0 = time_us_64();
        /* a blocking command (PLAY, JOG, BENCH ...) leaves a long gap: integrate it as one normal period, otherwise the
         * attitude filter multiplies the gyro by the whole gap (2026-11-23: TILT fault right after a 19 s PLAY) */
        float dt = (float)(t0 - last) * 1e-6f;
        if (dt > 0.1f) dt = (float)PERIOD_US * 1e-6f;
        last = t0;
        next += PERIOD_US;

        const bool usb = stdio_usb_connected();
        if (usb && !connected) {
            sleep_ms(50);
            printf("READY %s robot_v%d policy=%s calib_verified=%d imu=%d sync_read=%d aligned=%d\n", FW_VERSION,
                   ROBOT_VERSION, policy_kind(), CALIB_VERIFIED, imu_ready, use_sync_read, jmap.aligned);
        }
        if (!usb && connected) ctl_host_fault(&ctl, FAULT_HOST);
        connected = usb;

        for (int n = 0; n < 96; ++n) {
            const int ch = getchar_timeout_us(0);
            if (ch < 0) break;
            if (ch == 3 || ch == 27) { ctl_cmd_stop(&ctl); used = 0; puts("ACK STOP"); continue; }
            if (ch == '\r') continue;
            if (ch == '\n') { line[used] = 0; dispatch(line, &s); used = 0; continue; }
            if (used + 1 < sizeof(line)) line[used++] = (char)ch;
        }

        sense(&s, dt);
        const uint64_t t1 = time_us_64();
        ctl_step(&ctl, &s, dt, &o);
        ctl_us = (uint32_t)(time_us_64() - t1);
        apply(&o, &s);
        loop_us = (uint32_t)(time_us_64() - t0);
        if (torque_on && loop_us > PERIOD_US) {
            if (++overrun_count >= OVERRUN_TICKS) ctl_host_fault(&ctl, FAULT_OVERRUN);
        } else overrun_count = 0;
        if (time_us_64() > next + PERIOD_US) next = time_us_64(); /* do not try to catch up after a long command */
        if (telemetry && usb) telemetry_line(&s, &o);
    }
}
