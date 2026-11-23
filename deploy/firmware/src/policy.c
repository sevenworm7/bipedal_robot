#include "policy.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef POLICY_INT8
#include "policy_int8.h"
#else
#include "policy_fp32.h"
#endif

#if POL_D0 != POLICY_OBS_DIM || POL_D4 != POLICY_ACT_DIM || POL_LAYERS != 4
#error "policy header does not match the 210 -> ... -> 10 student layout"
#endif

#define POL_MAX_HIDDEN 512
#if POL_D1 > POL_MAX_HIDDEN || POL_D2 > POL_MAX_HIDDEN || POL_D3 > POL_MAX_HIDDEN
#error "hidden layer larger than the work buffers"
#endif

static float buf_a[POL_MAX_HIDDEN];
static float buf_b[POL_MAX_HIDDEN];

static inline float elu(float v) { return v < 0.0f ? expm1f(v) : v; }

#ifdef POLICY_INT8
/* int8 weights x int16 activations with integer accumulation (2026-11-23: the float version took 15.5 ms on the
 * RP2350, over the 20 ms budget together with the 6.4 ms bus read).
 * Each layer input is quantized per vector: q = round(x * Q_MAX / max|x|), Q_MAX = 16383 keeps 512 terms of
 * 127 * 16383 inside int32. Pairs are packed for the Cortex-M33 DSP instructions: SXTB16 splits a weight word
 * (w0 w1 w2 w3) into int16 lanes (w0, w2) and, rotated by 8, (w1, w3); SMLAD multiplies two lanes and accumulates.
 * The activation words are therefore packed as (x0, x2), (x1, x3). The host build emulates both instructions in C. */
#define Q_MAX 16383.0f
#ifdef PICO_BUILD
#include <arm_acle.h>
static inline uint32_t sx_lo(uint32_t w) { return __sxtb16(w); }
static inline uint32_t sx_hi(uint32_t w) { return __sxtb16(__ror(w, 8)); }
static inline int32_t mac2(uint32_t a, uint32_t b, int32_t acc) { return __smlad(a, b, acc); }
#else
static inline uint32_t sx_pair(uint8_t lo, uint8_t hi) {
    return (uint16_t)(int16_t)(int8_t)lo | ((uint32_t)(uint16_t)(int16_t)(int8_t)hi << 16);
}
static inline uint32_t sx_lo(uint32_t w) { return sx_pair((uint8_t)w, (uint8_t)(w >> 16)); }
static inline uint32_t sx_hi(uint32_t w) { return sx_pair((uint8_t)(w >> 8), (uint8_t)(w >> 24)); }
static inline int32_t mac2(uint32_t a, uint32_t b, int32_t acc) {
    const int32_t p = (int32_t)(int16_t)(a & 0xFFFF) * (int16_t)(b & 0xFFFF) + (int32_t)(int16_t)(a >> 16) * (int16_t)(b >> 16);
    return (int32_t)((uint32_t)acc + (uint32_t)p);
}
#endif

static uint32_t xq[(POL_MAX_HIDDEN + 3) / 4 * 2];

static inline uint32_t pack16(int16_t lo, int16_t hi) { return (uint16_t)lo | ((uint32_t)(uint16_t)hi << 16); }
static inline int16_t qround(float v) { return (int16_t)(v >= 0.0f ? (int32_t)(v + 0.5f) : -(int32_t)(0.5f - v)); }

/* quantize and pack the layer input; returns the activation scale (x ~= q * scale) */
static float RAMFUNC quantize_input(const float *in, int ni) {
    float m = 0.0f;
    for (int i = 0; i < ni; ++i) m = fmaxf(m, fabsf(in[i]));
    const float inv = m > 0.0f ? Q_MAX / m : 0.0f;
    for (int g = 0; g < (ni + 3) / 4; ++g) {
        int16_t v[4];
        for (int k = 0; k < 4; ++k) v[k] = 4 * g + k < ni ? qround(in[4 * g + k] * inv) : 0;
        xq[2 * g] = pack16(v[0], v[2]);
        xq[2 * g + 1] = pack16(v[1], v[3]);
    }
    return m / Q_MAX;
}

static void RAMFUNC dense(const float *in, float *out, const int8_t *q, const float *s, const float *b,
                          int ni, int no, int act) {
    const float sx = quantize_input(in, ni);
    const int full = ni / 4, rem = ni - 4 * full;
    for (int j = 0; j < no; ++j) {
        const uint8_t *r = (const uint8_t *)q + (size_t)j * ni;
        int32_t a0 = 0, a1 = 0;
        const uint32_t *x = xq;
        for (int g = 0; g < full; ++g, r += 4, x += 2) {
            uint32_t w;
            memcpy(&w, r, 4); /* rows of 210 bytes are not word aligned; the M33 handles unaligned LDR */
            a0 = mac2(sx_lo(w), x[0], a0);
            a1 = mac2(sx_hi(w), x[1], a1);
        }
        if (rem) {
            uint32_t w = 0;
            for (int k = 0; k < rem; ++k) w |= (uint32_t)r[k] << (8 * k);
            a0 = mac2(sx_lo(w), x[0], a0);
            a1 = mac2(sx_hi(w), x[1], a1);
        }
        const float v = (float)(a0 + a1) * (sx * s[j]) + b[j];
        out[j] = act ? elu(v) : v;
    }
}
#define LAYER(k, in, out, act) dense(in, out, POL_Q##k, POL_S##k, POL_B##k, POL_D##k, POL_D##k##_NEXT, act)
#else
static void RAMFUNC dense(const float *in, float *out, const float *w, const float *b, int ni, int no, int act) {
    for (int j = 0; j < no; ++j) {
        const float *r = w + (size_t)j * ni;
        float a0 = 0.0f, a1 = 0.0f, a2 = 0.0f, a3 = 0.0f;
        int i = 0;
        for (; i + 4 <= ni; i += 4) {
            a0 += r[i] * in[i];
            a1 += r[i + 1] * in[i + 1];
            a2 += r[i + 2] * in[i + 2];
            a3 += r[i + 3] * in[i + 3];
        }
        for (; i < ni; ++i) a0 += r[i] * in[i];
        float v = ((a0 + a1) + (a2 + a3)) + b[j];
        out[j] = act ? elu(v) : v;
    }
}
#define LAYER(k, in, out, act) dense(in, out, POL_W##k, POL_B##k, POL_D##k, POL_D##k##_NEXT, act)
#endif

#define POL_D0_NEXT POL_D1
#define POL_D1_NEXT POL_D2
#define POL_D2_NEXT POL_D3
#define POL_D3_NEXT POL_D4

bool policy_forward(const float *obs, float *act) {
    if (!obs || !act) return false;
    for (int i = 0; i < POL_D0; ++i) {
        if (!isfinite(obs[i])) return false;
        buf_a[i] = (obs[i] - POL_MEAN[i]) * POL_INV_STD[i];
    }
    float out[POL_D4];
    LAYER(0, buf_a, buf_b, 1);
    LAYER(1, buf_b, buf_a, 1);
    LAYER(2, buf_a, buf_b, 1);
    LAYER(3, buf_b, out, 0);
    for (int i = 0; i < POL_D4; ++i)
        if (!isfinite(out[i])) return false;
    memcpy(act, out, sizeof(out));
    return true;
}

const float *policy_test_obs(void) { return POL_MEAN; }
const float *policy_test_act(void) { return POL_TEST_ACT; }
const char *policy_kind(void) {
#ifdef POLICY_INT8
    return "int8";
#else
    return "fp32";
#endif
}
