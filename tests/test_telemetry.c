#include "telemetry.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 三组预期报文直接列成固定字节，检查顺序、符号、大端、校验和以及实际帧长。
 * 预期值独立于编码函数，可防止编码和解码同时使用同一错误规则而漏检。 */
static void golden_a1(void)
{
    uint8_t frame[TELEMETRY_MAX_SIZE];
    /* 此姿态的航向和倾角均为 90°。线上航向 −900 为 0x8384，
     * 两份倾角 +900 都是 0x0384。198 次/秒对应 0x00C6。 */
    const float q[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    const uint8_t want[] = {
        0xa5, 0x5a, 0x12, 0xa1, 0x83, 0x84, 0x03, 0x84, 0x03, 0x84,
        0xa5, 0xa5, 0x48, 0x01, 0x00, 0x00, 0x00, 0xc6, 0x21, 0xaa
    };
    assert(telemetry_a1(frame, q, 198) == sizeof want);
    assert(memcmp(frame, want, sizeof want) == 0);
    const float negative_yaw[4] = {0.5f, 0.5f, -0.5f, -0.5f};
    telemetry_a1(frame, negative_yaw, 200);
    assert(frame[4] == 0x8a && frame[5] == 0x8c); /* −90° 先折回 270°，取负并乘 10 后为 −2700。 */
}

static void golden_a2(void)
{
    uint8_t frame[TELEMETRY_MAX_SIZE];
    TelemetryMotion m;
    telemetry_motion_init(&m);
    /* 同时覆盖 −1、−32768 和 +1 g。旧 ARM 编码的 −32768 会回绕成 0x0000。 */
    const int16_t accel[3] = {-1, INT16_MIN, 16384};
    const int16_t gyro[3] = {80, -41, 0};
    /* 按 16.4 计数/(°/s) 换算，零偏为 (16,−8,0)，扣除后应输出 (64,−33,0)。 */
    const float bias[3] = {0.97560976f, -0.48780488f, 0};
    telemetry_motion_update(&m, accel, gyro);
    const uint8_t want[] = {
        0xa5, 0x5a, 0x16, 0xa2, 0x80, 0x01, 0x00, 0x00, 0x40, 0x00,
        0x00, 0x40, 0x80, 0x21, 0x00, 0x00, 0x00, 0x10, 0x80, 0x08,
        0x00, 0x00, 0xf2, 0xaa
    };
    assert(telemetry_a2(frame, &m, bias) == sizeof want);
    assert(memcmp(frame, want, sizeof want) == 0);
    const int16_t no_motion[3] = {0};
    const float no_bias[3] = {0};
    /* 首样本后补 9 个零，验证十点均值。再补一个零，首样本应从窗口移出。 */
    for (int i = 0; i < 9; ++i)
        telemetry_motion_update(&m, no_motion, no_motion);
    telemetry_a2(frame, &m, no_bias);
    assert(frame[8] == 0x06 && frame[9] == 0x66); /* 16384 除以 10 向零截断为 1638，即 0x0666。 */
    assert(frame[10] == 0 && frame[11] == 8);
    telemetry_motion_update(&m, no_motion, no_motion);
    telemetry_a2(frame, &m, no_bias);
    assert(frame[8] == 0 && frame[9] == 0);
    assert(frame[10] == 0 && frame[11] == 0);
}

static void golden_a3(void)
{
    uint8_t frame[TELEMETRY_MAX_SIZE];
    /* 分量 ±0.5 乘 10000 后分别编码为 0x1388 和 0x9388。 */
    const float q[4] = {0.5f, -0.5f, 0.5f, -0.5f};
    const uint8_t want[] = {
        0xa5, 0x5a, 0x0c, 0xa3, 0x13, 0x88, 0x93, 0x88,
        0x13, 0x88, 0x93, 0x88, 0x1b, 0xaa
    };
    assert(telemetry_a3(frame, q) == sizeof want);
    assert(memcmp(frame, want, sizeof want) == 0);
}

int main(void)
{
    golden_a1();
    golden_a2();
    golden_a3();
    puts("telemetry tests passed");
    return 0;
}
