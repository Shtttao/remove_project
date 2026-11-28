#ifndef IMU_APP_H
#define IMU_APP_H

#include "attitude.h"
#include "icm42688.h"
#include "telemetry.h"

/* 应用从这里取得三个外部能力：访问 IMU、读取微秒时钟、发送一帧串口数据。 */
typedef struct {
    ImuBus bus;
    uint32_t (*micros)(void);
    bool (*transmit)(const uint8_t *frame, size_t length);
} AppIo;

/* 持续跨主循环保存的状态。计数可通过 SWD 观察，UART2 继续传输二进制协议帧。 */
typedef struct {
    Attitude attitude;        /* 包含静止校准进度、陀螺零偏和姿态四元数。 */
    TelemetryMotion motion;   /* 保存 A2 运动帧使用的十点滑动窗口。 */
    ImuStatus last_fault;     /* 最近一次故障原因；重连成功后仍保留，便于追查。 */
    /* 累计初始化尝试数、有效样本数、总线错误数、发送错误数、采样停滞次数。 */
    uint32_t init_attempts, samples, bus_errors, tx_errors, stalls;
    /* 单位均为 µs。分别记录重试基准、轮询、最近有效样本、发送时隙、测速窗口起点。 */
    uint32_t last_attempt, last_poll, last_sample, last_tx, rate_start;
    /* updates 是本窗口的姿态更新次数；update_hz 是上一完整窗口测出的次数/秒。 */
    uint16_t updates, update_hz;
    /* slot 按 0、1、2 轮换，对应 A1、A2、A3；连续读错计数用于触发重连。 */
    uint8_t slot, consecutive_errors;
    /* online 表示传感器已配置成功；姿态能否上报还要看 attitude 的校准状态。 */
    bool online, rate_started;
} ImuApp;

/* 上电时清零全部状态。传感器初始化由首次 app_step 发起。 */
void app_init(ImuApp *app);
/* 由同一个前台循环反复调用；成功连接后，每轮最多处理一组新样本、发送一帧。 */
void app_step(ImuApp *app, const AppIo *io);
/* 当前时刻的期望灯态：离线快闪，在线校准时常亮，校准完成后慢闪。 */
bool app_status_led_on(const ImuApp *app, uint32_t now);

#endif
