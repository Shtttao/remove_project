#include "telemetry.h"
#include "icm42688.h"

#include <math.h>

enum {
    VERSION_HEAD = 0xa5a5,
    VERSION_BOARD_MAJOR = 0x4801, /* 高字节 0x48 是本台架约定的板卡标记，低字节 1 为主版本。 */
    VERSION_MINOR_PATCH = 0x0000
};

/* 旧 UART2 报文的负数用最高位标明符号，其余 15 位放绝对值，例如 −1 发 0x8001。
 * 先按旧固件的 int16_t 字段宽度保留低 16 位，再复现负数转换。这样即使陀螺计数
 * 减去零偏后跨过有符号边界，字节也一致。特别地，−32768 在线上编码为 0x0000。 */
static uint16_t sign_magnitude(int32_t value)
{
    uint16_t bits = (uint16_t)value;
    return (bits & 0x8000u) ? (uint16_t)(0x18000u - bits) : bits;
}

/* 三种帧共用帧头 A5 5A 和帧尾 AA。16 位字段先发高字节。校验只累加
 * 长度、类型和数据的每个字节，保留和的低 8 位，所以两端帧标记均不参与校验。 */
static size_t frame(uint8_t *out, uint8_t type, const uint16_t *words, size_t count)
{
    out[0] = 0xa5;
    out[1] = 0x5a;
    out[2] = (uint8_t)(2*count + 4); /* 长度从本字节算到帧尾，线上总字节数还要加 2。 */
    out[3] = type;
    uint8_t sum = (uint8_t)(out[2] + type);
    size_t n = 4;
    for (size_t i = 0; i < count; ++i) {
        uint8_t hi = (uint8_t)(words[i] >> 8);
        uint8_t lo = (uint8_t)words[i];
        out[n++] = hi;
        out[n++] = lo;
        sum = (uint8_t)(sum + hi + lo);
    }
    out[n++] = sum;
    out[n++] = 0xaa;
    return n;
}

void telemetry_motion_init(TelemetryMotion *m)
{
    *m = (TelemetryMotion){0};
}

void telemetry_motion_update(TelemetryMotion *m, const int16_t accel_raw[3],
                             const int16_t gyro_raw[3])
{
    /* sum 加新值、减被覆盖的旧值即可，无需为每一帧重新扫描整个窗口。 */
    int16_t *old = m->history[m->next];
    for (int i = 0; i < 6; ++i) {
        int16_t sample = i < 3 ? accel_raw[i] : gyro_raw[i-3];
        m->sum[i] += (int32_t)sample - old[i];
        old[i] = sample;
    }
    m->next = (uint8_t)((m->next + 1) % TELEMETRY_MOTION_WINDOW);
    if (m->count < TELEMETRY_MOTION_WINDOW)
        ++m->count;
}

size_t telemetry_a1(uint8_t *out, const float q[4], uint16_t update_hz)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float rad_to_deg = 57.29577951308232f;
    /* atan2 给出带正负号的航向角，旧接收端约定先折回 [0°,360°)，
     * 打包时再取负并换成 0.1° 单位。例如 −90° 对应线上的 −2700。 */
    float yaw = atan2f(2*(w*z+x*y), 1-2*(y*y+z*z)) * rad_to_deg;
    if (yaw < 0)
        yaw += 360.0f;
    /* (zx,zy,zz) 为基座 +Z 轴在地面坐标中的方向。用其水平投影长度和竖直分量
     * 求与向上方向的夹角，范围 0°～180°。旧帧的两个倾角位置都填这个值。 */
    const float zx = 2*(x*z+w*y), zy = 2*(y*z-w*x), zz = 1-2*(x*x+y*y);
    float tilt = atan2f(sqrtf(zx*zx+zy*zy), zz) * rad_to_deg;
    const uint16_t tilt_tenths = (uint16_t)(tilt * 10.0f);
    /* 七个字段依次是航向、两份倾角、三个版本字、实测姿态更新频率。 */
    const uint16_t words[] = {
        sign_magnitude((int32_t)(-yaw * 10.0f)), tilt_tenths, tilt_tenths,
        VERSION_HEAD, VERSION_BOARD_MAJOR, VERSION_MINOR_PATCH, update_hz
    };
    return frame(out, 0xa1, words, 7);
}

size_t telemetry_a2(uint8_t *out, const TelemetryMotion *m, const float bias_dps[3])
{
    if (m->count == 0)
        return 0;
    /* 九个字段按三轴成组。先写加速度均值，再写扣除零偏的陀螺计数均值，
     * 最后三项是陀螺零偏。按传感器灵敏度将 °/s 换为计数后四舍五入。
     * 运动数据均值按旧实现做整数除法，向零截断。 */
    uint16_t words[9];
    for (int i = 0; i < 3; ++i) {
        int32_t bias = (int32_t)roundf(bias_dps[i] * ICM42688_GYRO_LSB_PER_DPS);
        words[i] = sign_magnitude(m->sum[i] / m->count);
        words[3+i] = sign_magnitude(m->sum[3+i] / m->count - bias);
        words[6+i] = sign_magnitude(bias);
    }
    return frame(out, 0xa2, words, 9);
}

size_t telemetry_a3(uint8_t *out, const float q[4])
{
    /* w、x、y、z 各乘 10000 后向零截断，按协议的符号与幅值写为四个 16 位字。 */
    uint16_t words[4];
    for (int i = 0; i < 4; ++i)
        words[i] = sign_magnitude((int32_t)(q[i] * 10000.0f));
    return frame(out, 0xa3, words, 4);
}
