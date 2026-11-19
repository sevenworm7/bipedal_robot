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
static void RAMFUNC dense(const float *in, float *out, const int8_t *q, const float *s, const float *b,
                          int ni, int no, int act) {
    for (int j = 0; j < no; ++j) {
        const int8_t *r = q + (size_t)j * ni;
        float a0 = 0.0f, a1 = 0.0f, a2 = 0.0f, a3 = 0.0f;
        int i = 0;
        for (; i + 4 <= ni; i += 4) {
            a0 += (float)r[i] * in[i];
            a1 += (float)r[i + 1] * in[i + 1];
            a2 += (float)r[i + 2] * in[i + 2];
            a3 += (float)r[i + 3] * in[i + 3];
        }
        for (; i < ni; ++i) a0 += (float)r[i] * in[i];
        float v = ((a0 + a1) + (a2 + a3)) * s[j] + b[j];
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
