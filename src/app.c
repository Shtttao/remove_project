#include "app.h"

/* 每 1 ms 最多查一次就绪位，IMU 在 200 Hz 下约每 5 ms 产生一组新数据。
 * 串口共用 10 ms 的发送时隙；一轮三种帧约 30 ms。其余时间值用于恢复和测速。 */
enum {
    POLL_US = 1000, FRAME_US = 10000, STALE_US = 100000,
    RETRY_US = 500000, RATE_WINDOW_US = 1000000
};

void app_init(ImuApp *app) {
    *app = (ImuApp){0};
}

bool app_status_led_on(const ImuApp *app, uint32_t now)
{
    /* TIM2 每微秒加一。第 17 位和第 19 位分别提供约 0.13 s 和 0.52 s 的半周期，
     * 计数器回绕时仍按同样的节奏闪烁。掉线优先使用快闪，即使旧校准结果仍在。 */
    if (!app->online) {
        return (now & (1u << 17)) == 0u;
    }
    return !app->attitude.calibrated || (now & (1u << 19)) == 0u;
}

static void connect_sensor(ImuApp *app, const AppIo *io, uint32_t now) {
    /* 首次上电立即尝试；失败或掉线后至少间隔 500 ms，给传感器和接线恢复的时间。
     * 微秒时钟约每 71.6 分钟回绕一次，两个 uint32_t 时间戳相减仍能求短时间间隔。 */
    if (app->init_attempts && now - app->last_attempt < RETRY_US) return;
    app->init_attempts++;
    ImuStatus result = icm42688_init(&io->bus);
    app->last_attempt = io->micros(); /* 驱动内部等待了复位和启动，这里取返回后的时间。 */
    if (result != IMU_OK) {
        app->last_fault = result;
        if (result == IMU_BUS_ERROR) app->bus_errors++;
        return;
    }
    /* 重连期间的运动无法补算。清空姿态和 A2 平均窗口，从新样本重新静止校准。 */
    attitude_init(&app->attitude);
    telemetry_motion_init(&app->motion);
    app->last_sample = app->last_attempt;
    app->last_tx = app->last_attempt;
    app->last_poll = app->last_attempt;
    app->rate_started = false;
    app->update_hz = 0;
    app->consecutive_errors = 0;
    app->slot = 0;
    app->online = true;
}

static void update_rate(ImuApp *app, uint32_t now) {
    /* 首次得到可用姿态的时刻作为起点；后续成功更新才计入这个时间窗口。 */
    if (!app->rate_started) {
        app->rate_started = true;
        app->rate_start = now;
        app->updates = 0;
        return;
    }
    ++app->updates;
    uint32_t elapsed = now - app->rate_start;
    if (elapsed >= RATE_WINDOW_US) {
        /* 次数 × 每秒微秒数 ÷ 实际经过的微秒数。先扩为 64 位再乘。 */
        app->update_hz = (uint16_t)((app->updates * (uint64_t)RATE_WINDOW_US) / elapsed);
        app->rate_start = now;
        app->updates = 0;
    }
}

static void send_next_frame(ImuApp *app, const AppIo *io, uint32_t now) {
    uint32_t elapsed = now - app->last_tx;
    if (elapsed < FRAME_US) return;
    /* 锚定最近的 10 ms 时隙边界。例如晚到 2 ms，这 2 ms 不会累积进后续周期。
     * 若错过多个时隙，这一轮只发一帧。数据类型仍按成功发送的顺序轮换。 */
    app->last_tx = now - elapsed % FRAME_US;
    uint8_t frame[TELEMETRY_MAX_SIZE];
    size_t size;
    switch (app->slot) {
    case 0:  size = telemetry_a1(frame, app->attitude.base_q, app->update_hz); break;
    case 1:  size = telemetry_a2(frame, &app->motion, app->attitude.bias_dps); break;
    default: size = telemetry_a3(frame, app->attitude.base_q); break;
    }
    /* 只有发送成功才换类型；失败则记数，在下一个时隙用最新样本重发该类型。 */
    if (io->transmit(frame, size)) app->slot = (uint8_t)((app->slot + 1u) % 3u);
    else ++app->tx_errors;
}

void app_step(ImuApp *app, const AppIo *io) {
    uint32_t now = io->micros();
    if (!app->online) {
        connect_sensor(app, io, now);
        return;
    }
    if (now - app->last_poll < POLL_US) return;
    app->last_poll = now;
    ImuSample sample;
    ImuStatus result = icm42688_read(&io->bus, &sample);
    if (result != IMU_OK) {
        /* 两个采样时刻之间读到 NO_DATA 很常见。它会清零连续读错计数，
         * 但不会刷新 last_sample，所以传感器长时间停止更新仍能被发现。 */
        if (result != IMU_NO_DATA) {
            app->last_fault = result;
            if (result == IMU_BUS_ERROR) ++app->bus_errors;
            ++app->consecutive_errors;
        } else {
            app->consecutive_errors = 0;
        }
        /* 运行中复位意味着量程配置已经失效；其余情况在连续 3 次读错或
         * 100 ms 没有有效样本时转为离线。这个分支直接返回，避免旧姿态继续发出。 */
        if (result == IMU_RESET_ERROR || app->consecutive_errors >= 3 ||
            now - app->last_sample >= STALE_US) {
            if (now - app->last_sample >= STALE_US) {
                ++app->stalls;
                if (result == IMU_NO_DATA) app->last_fault = IMU_NO_DATA;
            }
            app->online = false;
            app->last_attempt = now;
        }
        return;
    }
    app->consecutive_errors = 0;
    app->last_sample = now;
    ++app->samples;
    /* 同一份读数分两路：原始计数进入旧协议的滑动平均，换算后的 g、°/s 用于融合。
     * 静止校准完成且本次姿态有效时，才参与 A1 的频率统计和串口上报。 */
    telemetry_motion_update(&app->motion, sample.accel_raw, sample.gyro_raw);
    if (attitude_update(&app->attitude, sample.accel_g, sample.gyro_dps, now)) {
        update_rate(app, now);
        send_next_frame(app, io, now);
    }
}
