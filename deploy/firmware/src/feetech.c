#include "feetech.h"

#include <string.h>

int32_t ft_decode_signed(uint8_t lo, uint8_t hi) {
    const uint32_t v = (uint32_t)lo | ((uint32_t)hi << 8);
    return (v & 0x8000u) ? -(int32_t)(v & 0x7fffu) : (int32_t)v;
}

void ft_encode_signed(int32_t v, uint8_t *lo, uint8_t *hi) {
    uint32_t e = v < 0 ? ((uint32_t)(-v) & 0x7fffu) | 0x8000u : (uint32_t)v & 0x7fffu;
    *lo = (uint8_t)e;
    *hi = (uint8_t)(e >> 8);
}

void ft_decode_feedback(const uint8_t d[FT_FEEDBACK_LEN], ft_feedback *fb) {
    fb->position = ft_decode_signed(d[0], d[1]);
    fb->speed_raw = ft_decode_signed(d[2], d[3]);
    const uint32_t load = (uint32_t)d[4] | ((uint32_t)d[5] << 8);
    fb->load_raw = (load & 0x400u) ? -(int32_t)(load & 0x3ffu) : (int32_t)(load & 0x3ffu);
    fb->voltage_raw = d[6];
    fb->temp_c = d[7];
    fb->status = d[9];
    fb->moving = d[10];
    fb->current_raw = ft_decode_signed(d[13], d[14]);
}

static size_t finish(uint8_t *buf, size_t n_before_sum) {
    uint8_t s = 0;
    for (size_t i = 2; i < n_before_sum; ++i) s = (uint8_t)(s + buf[i]);
    buf[n_before_sum] = (uint8_t)~s;
    return n_before_sum + 1;
}

size_t ft_build_read(uint8_t *buf, uint8_t id, uint8_t addr, uint8_t len) {
    buf[0] = 0xFF; buf[1] = 0xFF; buf[2] = id; buf[3] = 4; buf[4] = FT_INST_READ; buf[5] = addr; buf[6] = len;
    return finish(buf, 7);
}

size_t ft_build_write(uint8_t *buf, uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n) {
    buf[0] = 0xFF; buf[1] = 0xFF; buf[2] = id; buf[3] = (uint8_t)(n + 3); buf[4] = FT_INST_WRITE; buf[5] = addr;
    memcpy(buf + 6, data, n);
    return finish(buf, 6 + (size_t)n);
}

size_t ft_build_sync_write(uint8_t *buf, uint8_t addr, uint8_t len, const uint8_t *ids, const uint8_t *data, int n_ids) {
    buf[0] = 0xFF; buf[1] = 0xFF; buf[2] = FT_BROADCAST; buf[3] = (uint8_t)((len + 1) * n_ids + 4);
    buf[4] = FT_INST_SYNC_WRITE; buf[5] = addr; buf[6] = len;
    size_t k = 7;
    for (int i = 0; i < n_ids; ++i) {
        buf[k++] = ids[i];
        memcpy(buf + k, data + (size_t)i * len, len);
        k += len;
    }
    return finish(buf, k);
}

size_t ft_build_sync_read(uint8_t *buf, uint8_t addr, uint8_t len, const uint8_t *ids, int n_ids) {
    buf[0] = 0xFF; buf[1] = 0xFF; buf[2] = FT_BROADCAST; buf[3] = (uint8_t)(n_ids + 4);
    buf[4] = FT_INST_SYNC_READ; buf[5] = addr; buf[6] = len;
    memcpy(buf + 7, ids, (size_t)n_ids);
    return finish(buf, 7 + (size_t)n_ids);
}

size_t ft_build_goal_sync(uint8_t *buf, const uint8_t *ids, const int32_t *goals, int n_ids) {
    uint8_t data[2 * 32];
    if (n_ids > 32) return 0;
    for (int i = 0; i < n_ids; ++i) ft_encode_signed(goals[i], &data[2 * i], &data[2 * i + 1]);
    return ft_build_sync_write(buf, FT_REG_GOAL, 2, ids, data, n_ids);
}

int ft_parse_status(const uint8_t *rx, size_t used, const uint8_t *tx, size_t tx_len, uint8_t id, uint8_t len,
                    uint8_t *data, uint8_t *err) {
    int found = 0;
    for (size_t i = 0; i + 6 <= used; ++i) {
        if (rx[i] != 0xFF || rx[i + 1] != 0xFF || rx[i + 2] != id) continue;
        if (tx && i + tx_len <= used && !memcmp(rx + i, tx, tx_len)) { /* our own echo */
            i += tx_len - 1;
            continue;
        }
        const size_t n = rx[i + 3];
        if (n != (size_t)len + 2 || i + n + 4 > used) continue;
        uint8_t s = 0;
        for (size_t j = 2; j < n + 4; ++j) s = (uint8_t)(s + rx[i + j]);
        if (s != 0xFF) continue;
        if (found) return -1;
        found = 1;
        *err = rx[i + 4];
        memcpy(data, rx + i + 5, len);
        i += n + 3;
    }
    return found;
}
