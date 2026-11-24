#include "attitude.h"

#include <math.h>

#define DEG_TO_RAD 0.01745329251994329577f /* π/180，将角度转换为弧度。 */

/* 200 Hz 下每隔 5 ms 采样，200 点首尾相距约 995 ms。另设 950 ms 下限，
 * 允许定时抖动，也避免短时间内收到足量样本就误判静止。25 ms 相当于五个采样周期。 */
enum {
    STILL_SAMPLES = 200,
    MIN_STILL_US = 950000,
    MAX_INTERVAL_US = 25000
};

static float length3(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

/* 四元数乘法用来组合旋转。对于这里的坐标变换，a*b 表示先应用 b，再应用 a。
 * 先保存两个输入的分量，调用者便能把结果直接写回任一输入数组。 */
static void multiply(float out[4], const float a[4], const float b[4])
{
    const float w = a[0], x = a[1], y = a[2], z = a[3];
    const float r = b[0], i = b[1], j = b[2], k = b[3];
    out[0] = w*r - x*i - y*j - z*k;
    out[1] = w*i + x*r + y*k - z*j;
    out[2] = w*j - x*k + y*r + z*i;
    out[3] = w*k + x*j - y*i + z*r;
}

void attitude_init(Attitude *a)
{
    *a = (Attitude){.q = {1, 0, 0, 0}, .base_q = {1, 0, 0, 0}};
}

/* 用开机静止时测得的比力方向找平。设它与地面 +Z 的夹角为 θ，
 * 则下方的 z 等于 cos(θ)，最短旋转的轴沿 (x,y,z) × (0,0,1)。
 * 正放时从 cos(θ/2) 求四元数。接近倒置时改用 sin(θ/2)，避开 1+z 的相消误差。
 * 恰好倒置时水平旋转轴有无穷多个，约定选传感器 +Y。 */
static void align_to_gravity(Attitude *a)
{
    const float mag = length3(a->mean_accel);
    const float x = a->mean_accel[0] / mag, y = a->mean_accel[1] / mag;
    const float z = a->mean_accel[2] / mag;
    if (z >= 0) {
        a->q[0] = sqrtf((1 + z) * 0.5f);
        a->q[1] = y / (2 * a->q[0]);
        a->q[2] = -x / (2 * a->q[0]);
    } else {
        const float horizontal = sqrtf(x*x + y*y);
        const float sin_half = sqrtf((1 - z) * 0.5f);
        a->q[0] = horizontal / (2 * sin_half);
        a->q[1] = horizontal > 0 ? y / horizontal * sin_half : 0;
        a->q[2] = horizontal > 0 ? -x / horizontal * sin_half : 1;
    }
    a->q[3] = 0;
    /* 单位四元数的逆等于共轭，即保留 w 并取反 x、y、z。
     * 保存这次找平的逆变换，后续可由传感器姿态算出基座姿态。 */
    a->base_to_sensor[0] = a->q[0];
    for (int k = 1; k < 4; ++k)
        a->base_to_sensor[k] = -a->q[k];
}

static bool calibrate(Attitude *a, const float accel[3], const float gyro[3],
                      uint32_t now_us)
{
    /* 静止时比力应接近 1 g，陀螺仪读数应接近零。0.95～1.05 g 和
     * 3 °/s 是台架标定用的门限。满足门限的缓慢匀速偏航仍可能被计入零偏。 */
    const float accel_norm = length3(accel);
    if (!(accel_norm >= 0.95f && accel_norm <= 1.05f) || length3(gyro) > 3.0f) {
        a->still_samples = 0;
        return false;
    }
    /* 即使模长合适，某一轴相对窗口均值波动超过 0.75 °/s 或 0.035 g，
     * 也可能正在振动或搬动。丢弃前面的样本，以当前样本重新开窗。 */
    for (int k = 0; a->still_samples && k < 3; ++k) {
        if (fabsf(gyro[k] - a->mean_gyro[k]) > 0.75f ||
            fabsf(accel[k] - a->mean_accel[k]) > 0.035f)
            a->still_samples = 0;
    }
    if (a->still_samples == 0)
        a->still_since_us = now_us;
    /* 第 n 个样本的在线均值为 旧均值 + (样本−旧均值)/n，只需保存均值。 */
    float weight = 1.0f / (float)++a->still_samples;
    for (int k = 0; k < 3; ++k) {
        a->mean_accel[k] += (accel[k] - a->mean_accel[k]) * weight;
        a->mean_gyro[k] += (gyro[k] - a->mean_gyro[k]) * weight;
    }
    if (a->still_samples < STILL_SAMPLES || now_us - a->still_since_us < MIN_STILL_US)
        return false;
    for (int k = 0; k < 3; ++k)
        a->bias_dps[k] = a->mean_gyro[k];
    align_to_gravity(a);
    a->calibrated = true;
    return true;
}

/* 减去零偏，再乘实际间隔 dt（秒），得到传感器坐标中的旋转向量（弧度）。
 * 向量长度是转角 θ，方向是转轴 u。假设这段时间角速度恒定，
 * 增量四元数为 (cos(θ/2), u·sin(θ/2))，从右侧乘入当前姿态。 */
static void predict(Attitude *a, const float gyro[3], float dt)
{
    float rotation[3];
    for (int k = 0; k < 3; ++k)
        rotation[k] = (gyro[k] - a->bias_dps[k]) * (dt * DEG_TO_RAD);
    const float angle = length3(rotation);
    if (angle == 0)
        return;
    const float scale = sinf(angle * 0.5f) / angle;
    const float step[4] = {cosf(angle * 0.5f), rotation[0]*scale,
                           rotation[1]*scale, rotation[2]*scale};
    multiply(a->q, a->q, step);
}

static void correct_tilt(float q[4], const float accel[3], float dt)
{
    const float mag = length3(accel);
    const float deviation = fabsf(mag - 1.0f);
    if (!(deviation < 0.15f))
        return; /* 偏离 1 g 达 0.15 g 或出现非有限值时只信陀螺仪，例如自由落体。 */

    /* 把地面向上方向投到传感器坐标中得到预测值 g，再用归一化的实测比力 a
     * 求叉积 a×g。其方向给出倾角修正轴，长度等于两者夹角的正弦。
     * 绕竖直轴转动并不改变向上方向，因此重力观测无法校正航向漂移。 */
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float gx = 2*(x*z-w*y), gy = 2*(y*z+w*x), gz = 1-2*(x*x+y*y);
    const float ax = accel[0]/mag, ay = accel[1]/mag, az = accel[2]/mag;
    float error[3] = {ay*gz-az*gy, az*gx-ax*gz, ax*gy-ay*gx};
    if (ax*gx + ay*gy + az*gz < 0) {
        /* 点积为负表示夹角超过 90°。此时把修正轴归一化，防止接近 180° 时
         * 叉积趋于零，姿态卡在倒置方向。两向量几乎反向时，选与 g 不平行
         * 的 X 或 Y 轴求叉积，构造一个可用的垂直转轴。 */
        float length = length3(error);
        if (length < 0.000001f) {
            if (fabsf(gx) < 0.9f) {
                error[0] = 0; error[1] = gz; error[2] = -gy;
            } else {
                error[0] = -gz; error[1] = 0; error[2] = gx;
            }
            length = length3(error);
        }
        for (int k = 0; k < 3; ++k)
            error[k] /= length;
    }
    /* 偏离 1 g 不超过 0.05 g 时给实测方向全部权重，再到 0.15 g 线性降为零。
     * 这能减轻平动加速度的干扰，长期接近 1 g 的合加速度仍可能引入倾角误差。 */
    const float weight = fminf(1.0f, (0.15f-deviation)/0.10f);
    const float scale = dt * weight; /* 小角度四元数用半角，比例增益 2 乘 dt 后再除以 2。 */
    const float step[4] = {1, scale*error[0], scale*error[1], scale*error[2]};
    multiply(q, q, step);
}

bool attitude_update(Attitude *a, const float accel_g[3],
                     const float gyro_dps[3], uint32_t now_us)
{
    /* uint32_t 减法按 2³² 取模，正常采样间隔跨越一次计数器回绕仍能正确计算。 */
    const uint32_t elapsed = now_us - a->last_us;
    if (a->have_timestamp && elapsed == 0)
        return false;
    const bool contiguous = a->have_timestamp && elapsed <= MAX_INTERVAL_US;
    a->last_us = now_us;
    a->have_timestamp = true;
    /* 驱动配置量程为 ±2000 °/s。按有效范围做比较也能拦住 NaN，
     * 遇到坏数据就丢弃时间基准，随后从新的有效时间戳开始。 */
    for (int k = 0; k < 3; ++k) {
        if (!(fabsf(gyro_dps[k]) <= 2000.0f)) {
            a->still_samples = 0;
            a->have_timestamp = false;
            return false;
        }
    }
    if (!contiguous)
        a->still_samples = 0;
    if (!a->calibrated)
        return calibrate(a, accel_g, gyro_dps, now_us);
    if (!contiguous)
        return false;

    /* 已标定后先靠角速度预测，再用比力方向修正倾角，最后归一化，
     * 消除乘法和小角度修正造成的模长偏差。 */
    const float dt = (float)elapsed * 0.000001f;
    predict(a, gyro_dps, dt);
    correct_tilt(a->q, accel_g, dt);
    const float *q = a->q;
    const float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int k = 0; k < 4; ++k)
        a->q[k] /= norm;
    /* 基座到地面 = 传感器到地面 × 基座到传感器。旧模块以此作为串口姿态。 */
    multiply(a->base_q, a->q, a->base_to_sensor);
    return true;
}
