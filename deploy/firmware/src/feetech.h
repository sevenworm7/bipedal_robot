/* Feetech FT-SCS packets for the HD-1910-C001 (portable C: builders and parsers only, no I/O).
 * Packet: FF FF id len instr params... checksum, len = params + 2, checksum = ~(id + len + instr + params).
 * The X4 serial-servo port is half duplex: the receiver also sees our own transmission (echo), which the parsers
 * skip. Register map (HLS memory table, confirmed on the robot by the teammate's firmware):
 *   3,4 model (10, 31)  5 id  33 mode (4 = position)  40 torque enable  42-43 goal position (sign bit 15)
 *   48-49 torque limit  50 P  51 D  52 I
 *   56-70 feedback: 56-57 position, 58-59 speed, 60-61 load, 62 voltage [0.1 V], 63 temperature [C],
 *         64 async flag, 65 status, 66 moving, 67-68 goal, 69-70 current (signed magnitude, unit unverified) */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    FT_INST_PING = 0x01, FT_INST_READ = 0x02, FT_INST_WRITE = 0x03, FT_INST_SYNC_READ = 0x82, FT_INST_SYNC_WRITE = 0x83,
    FT_BROADCAST = 0xFE,
    FT_REG_MODEL_L = 3, FT_REG_ID = 5, FT_REG_MODE = 33, FT_REG_TORQUE = 40, FT_REG_GOAL = 42,
    FT_REG_TORQUE_LIMIT = 48, FT_REG_P = 50, FT_REG_FEEDBACK = 56, FT_FEEDBACK_LEN = 15,
};

typedef struct {
    int32_t position;   /* signed counts */
    int32_t speed_raw;  /* signed magnitude, unit not verified */
    int32_t load_raw;
    uint8_t voltage_raw; /* 0.1 V */
    uint8_t temp_c;
    uint8_t status;
    uint8_t moving;
    int32_t current_raw;
} ft_feedback;

int32_t ft_decode_signed(uint8_t lo, uint8_t hi);          /* bit 15 = sign */
void ft_encode_signed(int32_t v, uint8_t *lo, uint8_t *hi);
void ft_decode_feedback(const uint8_t d[FT_FEEDBACK_LEN], ft_feedback *fb);

size_t ft_build_read(uint8_t *buf, uint8_t id, uint8_t addr, uint8_t len);
size_t ft_build_write(uint8_t *buf, uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n);
/* data: n_ids x len bytes, in the order of ids */
size_t ft_build_sync_write(uint8_t *buf, uint8_t addr, uint8_t len, const uint8_t *ids, const uint8_t *data, int n_ids);
size_t ft_build_sync_read(uint8_t *buf, uint8_t addr, uint8_t len, const uint8_t *ids, int n_ids);
/* sync write of goal positions (counts) */
size_t ft_build_goal_sync(uint8_t *buf, const uint8_t *ids, const int32_t *goals, int n_ids);

/* Find a valid status packet of `id` with `len` data bytes in rx (skipping the echo `tx` if present).
 * Returns 1 found (data copied, *err = servo error byte), 0 not found / incomplete, -1 more than one reply. */
int ft_parse_status(const uint8_t *rx, size_t used, const uint8_t *tx, size_t tx_len, uint8_t id, uint8_t len,
                    uint8_t *data, uint8_t *err);
