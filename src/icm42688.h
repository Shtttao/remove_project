#ifndef ICM42688_H
#define ICM42688_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* DS-000347 §3.1–3.2 给出的灵敏度，须与初始化里选的量程配套。
 * ±2 g 时 16384 个计数对应 1 g；±2000 °/s 时 164 个计数约为 10 °/s。 */
#define ICM42688_GYRO_LSB_PER_DPS 16.4f
#define ICM42688_ACCEL_LSB_PER_G 16384.0f

/* 驱动向回调传 7 位寄存器地址。以读 WHO_AM_I 为例，传给 read 的是 0x75，
 * 板级 SPI 将最高位置 1，线上发出 0xF5。地址和后续数据共用一次低电平片选。
 * read/write 以 true 表示整次传输成功；delay_ms 至少等待指定毫秒数。
 * context 会原样传回各函数，实板可以不用它，测试用它定位模拟器实例。 */
typedef struct {
    bool (*read)(void *context, uint8_t reg, uint8_t *data, size_t len);
    bool (*write)(void *context, uint8_t reg, uint8_t value);
    void (*delay_ms)(void *context, uint32_t ms);
    void *context;
} ImuBus;

typedef struct {
    /* 数组下标依次为芯片 X、Y、Z。保留计数用于旧协议，保留物理单位用于解算。 */
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    float accel_g[3];
    float gyro_dps[3];  /* °/s，绕对应轴的角速度。 */
    float temp_c;       /* 芯片内部温度，单位 °C。 */
} ImuSample;

typedef enum {
    IMU_OK,            /* 初始化成功，或已经得到一份完整样本。 */
    IMU_NO_DATA,       /* 尚无新数据，或本次样本带有无效标记。 */
    IMU_BUS_ERROR,     /* 总线传输失败，或状态寄存器返回断线特征值 0xff。 */
    IMU_ID_MISMATCH,   /* 读到的器件 ID 与 ICM-42688-P 的 0x47 不符。 */
    IMU_RESET_ERROR,   /* 初始化时缺少复位完成标志，或运行中出现了新的复位。 */
    IMU_CONFIG_ERROR,  /* 写入的配置与有效位读回值不符。 */
    IMU_BAD_ARGUMENT   /* 缺少必要的回调或输出缓冲区。 */
} ImuStatus;

/* 调用方负责串行访问这一颗器件，避免另一段代码中途切换寄存器组或发起传输。 */
ImuStatus icm42688_init(const ImuBus *bus);

/* 通过用户接口数据寄存器读取最新一组样本。仅 IMU_OK 时更新 *out。 */
ImuStatus icm42688_read(const ImuBus *bus, ImuSample *out);

#endif
