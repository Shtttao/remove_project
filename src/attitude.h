#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* 两个姿态都用右手系的 Hamilton 单位四元数，排列为 (w,x,y,z)。
     * 地面坐标系的 +Z 竖直向上，水平朝向取决于开机姿态。 */
    bool calibrated;  /* 静止标定完成后，bias_dps 和姿态才可使用。 */
    float q[4];       /* 把传感器坐标中的向量旋转到地面坐标。 */
    float base_q[4];  /* 把基座坐标旋转到地面坐标，对应旧协议的 Q_eb，标定完成时为单位四元数。 */
    float bias_dps[3]; /* XYZ 三轴静止角速度的均值，单位 °/s。 */

    /* 每个实例单独保存以下状态，连续采样期间由 attitude_update 维护。 */
    float base_to_sensor[4]; /* 开机找平后固定，描述基座轴与传感器轴的关系。 */
    float mean_accel[3];     /* 当前静止窗口内的加速度均值，单位 g。 */
    float mean_gyro[3];      /* 同一窗口内的角速度均值，单位 °/s。 */
    uint32_t still_samples;
    uint32_t still_since_us;
    uint32_t last_us;
    bool have_timestamp;
} Attitude;

/* 上电或重连时清除标定历史，以单位四元数开始。 */
void attitude_init(Attitude *a);

/* 每读到一组新样本调用一次，两个数组都按芯片自身的右手 XYZ 轴排列。
 * 加速度以 g 为单位，静止时测到向上的比力，模长约为 1 g。
 * 陀螺仪以 °/s 为单位，正向按右手定则。now_us 来自可回绕的 32 位微秒计数器。
 * 返回 true 表示本次产生了已标定的姿态。相邻时间戳超过 25 ms 时放弃该段积分。 */
bool attitude_update(Attitude *a, const float accel_g[3],
                     const float gyro_dps[3], uint32_t now_us);

#endif
