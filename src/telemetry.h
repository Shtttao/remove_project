#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

/* A2 是最长的帧，共 24 字节。旧模块的运动数据取最近 10 组采样的均值。 */
enum { TELEMETRY_MAX_SIZE = 24, TELEMETRY_MOTION_WINDOW = 10 };

typedef struct {
    int16_t history[TELEMETRY_MOTION_WINDOW][6]; /* 环形窗口，每行依次为加速度和角速度各三轴。 */
    int32_t sum[6]; /* 10 个 16 位读数的和可能超过 16 位，按轴用 32 位累计。 */
    uint8_t count;  /* 刚开始未攒满时，按实际有效样本数求均值。 */
    uint8_t next;   /* 下一次写入的槽位，窗口满后它指向最旧样本。 */
} TelemetryMotion;

void telemetry_motion_init(TelemetryMotion *m);
/* 每收到一组新样本就更新窗口，用于下一帧 A2。输入按芯片自身的 XYZ 轴排列，
 * 直接使用 ±2 g、±2000 °/s 量程下的寄存器计数值，与旧模块的名义量程相符。 */
void telemetry_motion_update(TelemetryMotion *m, const int16_t accel_raw[3],
                             const int16_t gyro_raw[3]);

/* 调用者分配至少 TELEMETRY_MAX_SIZE 字节的 out，按返回长度发送。
 * A1 和 A3 接受开机找平后的基座到地面的单位四元数，顺序为 (w,x,y,z)。
 * A1 的 update_hz 表示每秒实际成功的姿态更新次数。A2 的 bias_dps 单位为 °/s，
 * 编码时换回计数，窗口为空时返回 0。 */
size_t telemetry_a1(uint8_t *out, const float base_q[4], uint16_t update_hz);
size_t telemetry_a2(uint8_t *out, const TelemetryMotion *m, const float bias_dps[3]);
size_t telemetry_a3(uint8_t *out, const float base_q[4]);

#endif
