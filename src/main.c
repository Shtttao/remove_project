#include "app.h"
#include "board.h"

/* 在 VS Code 调试器中展开 imu_app，可同时看到连接状态、校准结果和收发计数。 */
ImuApp imu_app;

int main(void) {
    /* 复位入口已准备好 C 运行环境。这里才配置本板的计时器、SPI 和串口。 */
    board_init();
    /* 把板级读写函数交给应用层。同一套 app_step 在主机测试中接的是模拟器。 */
    const AppIo io =
    {
        .bus = {.read = board_read, .write = board_write, .delay_ms = board_delay_ms},
        .micros = board_micros,
        .transmit = board_uart_tx
    };
    app_init(&imu_app);
    bool led_on = false; /* board_init 已熄灯，灯态变化时才写 GPIO。 */
    /* 每轮推进一次状态机。app_step 自己检查时间和数据就绪位，
     * 因此主循环执行次数、传感器采样数和串口发帧数可以各不相同。 */
    for (;;) {
        app_step(&imu_app, &io);
        bool next_led_on = app_status_led_on(&imu_app, board_micros());
        if (next_led_on != led_on) {
            board_status_led(next_led_on);
            led_on = next_led_on;
        }
    }
}
