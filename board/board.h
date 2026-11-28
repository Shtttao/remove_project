#ifndef IMU_BOARD_H
#define IMU_BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* main 启动时先调用一次，准备系统时钟、TIM2、SPI1、片选、USART2 和状态灯。 */
void board_init(void);
/* 初始化后调用，on 表示点亮。PC0 的电平极性由板级代码处理。 */
void board_status_led(bool on);
/* 初始化后 TIM2 自由计数，每格 1 µs，约 71.6 分钟回绕一次。
   测量不足一个回绕周期的间隔时，用两个 uint32_t 时间戳直接相减。 */
uint32_t board_micros(void);
/* 传给传感器驱动的阻塞延时回调。context 用于匹配回调签名，本板使用固定外设。 */
void board_delay_ms(void *context, uint32_t ms);

/* 由前台代码串行调用。reg 是 7 位地址，SPI1 在一次片选期间完成整个事务。
   read 接收 len 个字节，write 写一个字节。返回 false 时应放弃本次传输结果。 */
bool board_read(void *context, uint8_t reg, uint8_t *data, size_t len);
bool board_write(void *context, uint8_t reg, uint8_t value);
/* 发送二进制数据，len 包括值为 0 的字节。最后一个停止位发完才返回 true。 */
bool board_uart_tx(const uint8_t *data, size_t len);

#endif
