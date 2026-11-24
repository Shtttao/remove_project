#include "attitude.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/* 模拟水平静置，Z 轴测得 +1 g。bias 是预先给定的三轴零偏（°/s），
 * 可以直接拿它和标定结果比较。所有测试使用模拟时钟，不需要接开发板。 */
static const float up[3] = {0, 0, 1};
static const float zero[3] = {0, 0, 0};
static const float bias[3] = {0.5f, -0.75f, 0.25f};

static void near(float value, float expected, float tolerance)
{
    assert(fabsf(value - expected) <= tolerance);
}

/* 每次推进 5000 µs，模拟 200 Hz。前 199 次应保持未标定，第 200 次才有有效姿态。 */
static void calibrate(Attitude *a, const float accel[3], uint32_t *time)
{
    for (int i = 0; i < 200; ++i) {
        *time += 5000;
        assert(attitude_update(a, accel, bias, *time) == (i == 199));
        assert(a->calibrated == (i == 199));
    }
    for (int axis = 0; axis < 3; ++axis)
        near(a->bias_dps[axis], bias[axis], 0.0001f);
}

static void startup_requires_sustained_stationarity(void)
{
    Attitude a;
    attitude_init(&a);
    uint32_t time = 0;
    for (int i = 0; i < 180; ++i) {
        time += 5000;
        assert(!attitude_update(&a, up, bias, time));
    }
    const float moving[3] = {0, 0, 12};
    time += 5000;
    assert(!attitude_update(&a, up, moving, time));
    /* 交替输入 ±2 °/s，均值接近零，逐样本波动却很大，应持续拒绝标定。 */
    for (int i = 0; i < 220; ++i) {
        const float noisy[3] = {i % 2 ? 2.0f : -2.0f, 0, 0};
        time += 5000;
        assert(!attitude_update(&a, up, noisy, time));
    }
    /* 再输入一次明确的运动，确保随后的标定从完整的静止窗口重新计数。 */
    time += 5000;
    assert(!attitude_update(&a, up, moving, time));
    calibrate(&a, up, &time);
    near(a.q[0], 1.0f, 0.0001f);
    /* 标定完成后继续静置 5 秒。扣掉零偏后应保持单位四元数。 */
    for (int i = 0; i < 1000; ++i) {
        time += 5000;
        assert(attitude_update(&a, up, bias, time));
    }
    near(a.q[0], 1.0f, 0.0001f);
    for (int axis = 1; axis < 4; ++axis)
        near(a.q[axis], 0.0f, 0.0001f);
}

static void six_face_alignment_and_base_zero(void)
{
    /* 六面静置使 +1 g 依次落在 ±X、±Y、±Z 上，另加一个近乎完全倒置的样本。 */
    const float faces[][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
        {0, 0, 1}, {0, 0, -1}, {0.0001f, -0.0002f, -1}
    };
    for (size_t i = 0; i < sizeof faces / sizeof faces[0]; ++i) {
        Attitude a;
        attitude_init(&a);
        uint32_t time = 0;
        calibrate(&a, faces[i], &time);
        float w = a.q[0], x = a.q[1], y = a.q[2], z = a.q[3];
        /* 若 R 将传感器坐标转到地面，则 R 的第三行正好列出向上的
         * 单位向量在传感器三轴上的分量。找平后的基座在开机时总应为零姿态。 */
        near(2*(x*z-w*y), faces[i][0], 0.00003f);
        near(2*(y*z+w*x), faces[i][1], 0.00003f);
        near(1-2*(x*x+y*y), faces[i][2], 0.00003f);
        near(a.base_q[0], 1, 0.00001f);
        for (int axis = 1; axis < 4; ++axis)
            near(a.base_q[axis], 0, 0.00001f);
    }
}

static void known_turn_with_tilted_mount(void)
{
    Attitude a;
    attitude_init(&a);
    uint32_t time = UINT32_MAX - 1200000;
    const float side[3] = {1, 0, 0};
    calibrate(&a, side, &time);
    /* 侧放使传感器 +X 对准地面 +Z。减去零偏后，绕 +X 旋转 90 °/s，
     * 3.5/6.5 ms 两种间隔交替，累计恰好 1 秒，且中途 32 位时间戳回绕。
     * 基座应绕 +Z 转过 90°，所以 w=cos(45°)、z=sin(45°)。 */
    const float turning[3] = {90.5f, -0.75f, 0.25f};
    for (int i = 0; i < 200; ++i) {
        time += i % 2 ? 6500 : 3500;
        assert(attitude_update(&a, side, turning, time));
    }
    near(a.base_q[0], 0.7071068f, 0.0001f);
    near(a.base_q[1], 0, 0.0001f);
    near(a.base_q[2], 0, 0.0001f);
    near(a.base_q[3], 0.7071068f, 0.0001f);
    /* 重复时间戳和随后 250 ms 的空档都不能继续累加转角。 */
    assert(!attitude_update(&a, side, turning, time));
    time += 250000;
    assert(!attitude_update(&a, side, turning, time));
    near(a.base_q[3], 0.7071068f, 0.0001f);
    assert(attitude_update(&a, side, bias, time + 5000));
    near(a.base_q[3], 0.7071068f, 0.0001f);
}

static void rejects_linear_acceleration_and_recovers_tilt(void)
{
    Attitude a;
    attitude_init(&a);
    uint32_t time = 0;
    calibrate(&a, up, &time);
    /* 水平额外增加 1.5 g，使模长明显超过 1 g。此时应保持原姿态。 */
    const float shock[3] = {1.5f, 0, 1};
    for (int i = 0; i < 200; ++i) {
        time += 5000;
        assert(attitude_update(&a, shock, bias, time));
    }
    near(a.q[0], 1, 0.0001f);
    /* (0,sin30°,cos30°) 模拟绕 X 倾斜 30° 后静止。即使陀螺只有零偏，
     * 重力方向也应逐步把估计拉到此角度，四元数使用半角 15°。 */
    const float tilt30[3] = {0, 0.5f, 0.8660254f};
    for (int i = 0; i < 1000; ++i) {
        time += 5000;
        assert(attitude_update(&a, tilt30, bias, time));
    }
    near(a.q[0], 0.9659258f, 0.001f);
    near(a.q[1], 0.2588190f, 0.001f);
    near(a.q[2], 0, 0.0001f);
    near(a.q[3], 0, 0.0001f);
    time += 5000;
    assert(attitude_update(&a, zero, bias, time));
    /* 比力为零时仍能靠陀螺更新。陀螺本身无效时应保留上一有效姿态。 */
    const float invalid_gyro[3] = {NAN, 0, 0};
    assert(!attitude_update(&a, up, invalid_gyro, time + 5000));
    near(a.q[1], 0.2588190f, 0.001f);
}

static void short_or_interrupted_windows_do_not_calibrate(void)
{
    Attitude a;
    attitude_init(&a);
    uint32_t time = 0;
    /* 300 点仅覆盖约 30 ms，达不到静止时长。随后的 100 ms 空档会重置窗口。 */
    for (int i = 0; i < 300; ++i) {
        time += 100;
        assert(!attitude_update(&a, up, bias, time));
    }
    time += 100000;
    calibrate(&a, up, &time);
}

static void gravity_recovers_after_an_unobserved_half_turn(void)
{
    Attitude a;
    attitude_init(&a);
    uint32_t time = 0;
    calibrate(&a, up, &time);
    /* 模拟在 50 ms 漏采期间翻转半圈。恢复后实测与预测方向完全相反，
     * 检验算法能否重新找到竖直方向。仅凭重力无法验证绕竖直轴的航向。 */
    const float inverted[3] = {0, 0, -1};
    time += 50000;
    assert(!attitude_update(&a, inverted, bias, time));
    for (int i = 0; i < 1000; ++i) {
        time += 5000;
        assert(attitude_update(&a, inverted, bias, time));
    }
    near(1 - 2*(a.q[1]*a.q[1]+a.q[2]*a.q[2]), -1, 0.0001f);
}

int main(void)
{
    startup_requires_sustained_stationarity();
    six_face_alignment_and_base_zero();
    known_turn_with_tilted_mount();
    rejects_linear_acceleration_and_recovers_tilt();
    short_or_interrupted_windows_do_not_calibrate();
    gravity_recovers_after_an_unobserved_half_turn();
    puts("attitude tests passed");
    return 0;
}
