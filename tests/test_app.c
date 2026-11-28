#include "app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 在电脑上运行真实的 app_step。这里替换芯片寄存器、微秒时钟和串口，
 * 使上电、转为离线和恢复都能在同一次测试里重复。所有时间均来自模拟时钟。 */
static uint8_t regs[128];
static uint32_t tick;
static uint32_t last_sample;
static bool connected, data_stalled, miso_high, tx_failed;
static unsigned frames, kinds[4];
static uint8_t sequence[3];

/* 驱动请求毫秒延时时直接推进 tick；测试不必等现实时间流逝。 */
static uint32_t clock_us(void) { return tick; }
static void delay_ms(void *ctx, uint32_t ms) { (void)ctx; tick += ms * 1000u; }

static bool read_reg(void *ctx, uint8_t reg, uint8_t *out, size_t len) {
    (void)ctx;
    if (!connected) return false;
    if (miso_high) {
        memset(out, 0xff, len); /* 模拟 MISO 断线被拉高，SPI 主机仍能完成收发。 */
        return true;
    }
    /* 200 Hz 对应每 5000 µs 一个就绪事件；读 INT_STATUS 后清除事件位。 */
    if (reg == 0x2d && !data_stalled && tick - last_sample >= 5000u) {
        regs[reg] |= 0x08;
        last_sample = tick;
    }
    memcpy(out, &regs[reg], len);
    if (reg == 0x2d) regs[reg] = 0;
    return true;
}

static bool write_reg(void *ctx, uint8_t reg, uint8_t value) {
    (void)ctx;
    if (!connected) return false;
    if (reg == 0x11 && (value & 1u)) {
        memset(regs, 0, sizeof regs);
        regs[0x75] = 0x47;
        regs[0x2d] = 0x10;
        regs[0x4c] = 0x30;
        regs[0x23] = 0x40; /* Z 轴加速度高字节为 0x40，低字节为零；平放静止读出 1 g。 */
    } else {
        regs[reg] = value;
    }
    return true;
}

/* 像接收端那样检查完整包。静止时 A2 的 Z 轴应为 0x4000，
 * A3 的单位四元数首项 w=1 编码为 10000，即 0x2710。 */
static bool uart_tx(const uint8_t *p, size_t len) {
    if (tx_failed) return false;
    assert(len >= 6 && len == (size_t)p[2] + 2);
    assert(p[0] == 0xa5 && p[1] == 0x5a && p[len-1] == 0xaa);
    uint8_t sum = 0;
    for (size_t i = 2; i < len - 2; ++i) sum += p[i];
    assert(sum == p[len-2]);
    unsigned type = p[3] - 0xa0;
    assert(type >= 1 && type <= 3);
    if (frames < 3) sequence[frames] = p[3];
    if (type == 2) assert(p[8] == 0x40 && p[9] == 0);
    if (type == 3) assert(p[4] == 0x27 && p[5] == 0x10);
    kinds[type]++;
    frames++;
    return true;
}

/* 每 1 ms 调用一次真实主循环逻辑；数据来源仍按上面 5 ms 的周期更新。 */
static void run_ms(ImuApp *app, const AppIo *io, unsigned milliseconds) {
    for (unsigned i = 0; i < milliseconds; i++) {
        tick += 1000;
        app_step(app, io);
    }
}

int main(void) {
    ImuApp app;
    const AppIo io = {{read_reg, write_reg, delay_ms, NULL}, clock_us, uart_tx};
    /* 从回绕前 0.8 秒开始，让后面的校准跨过 32 位微秒计数器的最大值。 */
    tick = UINT32_MAX - 800000u;
    app_init(&app);
    run_ms(&app, &io, 100); /* 上电时不接 IMU，应该保持静默，接入后再开始校准。 */
    assert(!app.online && frames == 0);
    assert(app_status_led_on(&app, 0u) && !app_status_led_on(&app, 131072u));
    connected = true;
    regs[0x75] = 0x47;
    run_ms(&app, &io, 600);
    assert(app.online && !app.attitude.calibrated && frames == 0);
    assert(app_status_led_on(&app, 131072u) && app_status_led_on(&app, 524288u));
    run_ms(&app, &io, 2600);
    assert(app.online && app.attitude.calibrated);
    assert(app_status_led_on(&app, 131072u) && !app_status_led_on(&app, 524288u));
    assert(!app_status_led_on(&app, UINT32_MAX) && app_status_led_on(&app, 0u));
    assert(frames > 120 && sequence[0] == 0xa1 && sequence[1] == 0xa2 && sequence[2] == 0xa3);
    /* 三种帧轮流发送，截在任意时刻 A1 的帧数最多比 A3 多一帧。 */
    assert(kinds[1] >= kinds[3] && kinds[1] <= kinds[3] + 1);
    assert(app.update_hz >= 198 && app.update_hz <= 202);
    tx_failed = true;
    run_ms(&app, &io, 20);
    assert(app.tx_errors > 0);
    tx_failed = false;
    /* 总线仍可访问，但就绪位停止更新。旧样本不能被当成新姿态继续发送。 */
    unsigned previous = frames;
    data_stalled = true;
    run_ms(&app, &io, 200);
    assert(!app.online && previous == frames);
    assert(!app_status_led_on(&app, 131072u)); /* 掉线后优先显示寻址状态。 */
    data_stalled = false;
    run_ms(&app, &io, 2300);
    assert(app.online && frames > previous && app.attitude.calibrated);
    miso_high = true;
    previous = frames;
    run_ms(&app, &io, 30);
    assert(!app.online && previous == frames);
    miso_high = false;
    run_ms(&app, &io, 2300);
    assert(app.online && frames > previous);
    regs[0x2d] |= 0x10; /* 运行中独立复位。此位读后清零，单次出现就要重新初始化。 */
    run_ms(&app, &io, 1);
    assert(!app.online && app.last_fault == IMU_RESET_ERROR);
    run_ms(&app, &io, 2300);
    assert(app.online);
    /* 与前面的 MISO 全高情形分别验证：这里由底层传输函数直接返回失败。 */
    connected = false;
    previous = frames;
    run_ms(&app, &io, 50);
    assert(!app.online && previous == frames && app.bus_errors > 0);
    puts("PASS integrated firmware: retry, wraparound, calibration, status LED, A1/A2/A3, timing, TX failure, stale data, reconnect");
}
