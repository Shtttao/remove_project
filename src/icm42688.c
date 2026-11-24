#include "icm42688.h"

/* DS-000347 的用户寄存器组 0。地址标识读写位置，下面的配置值决定采样方式。 */
enum {
    DEVICE_CONFIG = 0x11,
    TEMP_DATA1 = 0x1d,
    INT_STATUS = 0x2d,
    PWR_MGMT0 = 0x4e,
    GYRO_CONFIG0 = 0x4f,
    ACCEL_CONFIG0 = 0x50,
    WHO_AM_I = 0x75,
    REG_BANK_SEL = 0x76,
    EXPECTED_ID = 0x47,
    RESET_DONE = 0x10,         /* INT_STATUS bit 4，复位完成，读取状态后清除。 */
    DATA_READY = 0x08,         /* INT_STATUS bit 3，数据寄存器有更新。 */
    GYRO_2000DPS_200HZ = 0x07, /* CONFIG0 高 3 位选 ±2000 °/s，低 4 位选 200 Hz。 */
    ACCEL_2G_200HZ = 0x67,     /* 高 3 位为 011，选 ±2 g；采样率同为 200 Hz。 */
    BOTH_LOW_NOISE = 0x0f      /* PWR_MGMT0 的陀螺仪和加速度计模式都设为低噪声。 */
};

/* SPI 写操作成功只说明字节已送出；读回并用 mask 取有效位，才能确认配置生效。 */
static ImuStatus verify_config(const ImuBus *bus, uint8_t reg, uint8_t mask, uint8_t expected)
{
    uint8_t actual;
    if (!bus->read(bus->context, reg, &actual, 1)) {
        return IMU_BUS_ERROR;
    }
    return (actual & mask) == expected ? IMU_OK : IMU_CONFIG_ERROR;
}

ImuStatus icm42688_init(const ImuBus *bus)
{
    if (!bus || !bus->read || !bus->write || !bus->delay_ms) {
        return IMU_BAD_ARGUMENT;
    }
    /* 手册规定上电至串行接口可用最多 1 ms，这里留到 2 ms。
     * 先切回 Bank 0，兼容 MCU 单独重启、传感器仍停在其他 Bank 的情形。 */
    bus->delay_ms(bus->context, 2);
    uint8_t id;
    if (!bus->write(bus->context, REG_BANK_SEL, 0) ||
        !bus->read(bus->context, WHO_AM_I, &id, 1)) {
        return IMU_BUS_ERROR;
    }
    if (id != EXPECTED_ID) {
        return IMU_ID_MISMATCH;
    }
    /* 确认 ID 后写 DEVICE_CONFIG bit 0 触发软复位，重新建立已知的寄存器状态。 */
    if (!bus->write(bus->context, DEVICE_CONFIG, 0x01)) {
        return IMU_BUS_ERROR;
    }
    /* 软复位期间至少 1 ms 不访问寄存器，等待后检查 RESET_DONE。 */
    bus->delay_ms(bus->context, 2);
    uint8_t status;
    if (!bus->read(bus->context, INT_STATUS, &status, 1)) {
        return IMU_BUS_ERROR;
    }
    if ((status & RESET_DONE) == 0) {
        return IMU_RESET_ERROR;
    }

    if (!bus->write(bus->context, GYRO_CONFIG0, GYRO_2000DPS_200HZ) ||
        !bus->write(bus->context, ACCEL_CONFIG0, ACCEL_2G_200HZ)) {
        return IMU_BUS_ERROR;
    }
    /* 两个 CONFIG0 的 bit 4 是保留位，掩码 0xef 只比较量程和输出数据率。 */
    ImuStatus check = verify_config(bus, GYRO_CONFIG0, 0xef, GYRO_2000DPS_200HZ);
    if (check != IMU_OK) {
        return check;
    }
    check = verify_config(bus, ACCEL_CONFIG0, 0xef, ACCEL_2G_200HZ);
    if (check != IMU_OK) {
        return check;
    }
    if (!bus->write(bus->context, PWR_MGMT0, BOTH_LOW_NOISE)) {
        return IMU_BUS_ERROR;
    }
    /* 陀螺仪典型启动时间为 30 ms，开启后至少保持 45 ms。
     * 这里等待 50 ms，同时满足切换模式后至少 200 µs 不写寄存器的要求。 */
    bus->delay_ms(bus->context, 50);
    return verify_config(bus, PWR_MGMT0, 0x3f, BOTH_LOW_NOISE);
}

/* 芯片先传高字节。先把高字节还原为有符号值再拼接，例如 FF 9C 对应 −100。 */
static int16_t signed_be16(const uint8_t *bytes)
{
    int high = bytes[0];
    if (high >= 128) {
        high -= 256;
    }
    return (int16_t)(high * 256 + bytes[1]);
}

ImuStatus icm42688_read(const ImuBus *bus, ImuSample *out)
{
    if (!bus || !bus->read || !out) {
        return IMU_BAD_ARGUMENT;
    }
    uint8_t status;
    if (!bus->read(bus->context, INT_STATUS, &status, 1)) {
        return IMU_BUS_ERROR;
    }
    /* 即使从设备拔掉，SPI 主机仍能产生时钟。MISO 带上拉时可能“读成功”却得到全 1。
     * RESET_DONE 在初始化中已读清；运行时若再出现，之前的量程和采样率需重新配置。 */
    if (status == 0xff) {
        return IMU_BUS_ERROR;
    }
    if (status & RESET_DONE) {
        return IMU_RESET_ERROR;
    }
    if ((status & DATA_READY) == 0) {
        return IMU_NO_DATA;
    }
    /* 200 Hz 下约每 5 ms 更新一次。先判就绪，再从 0x1d 连读 14 字节，
     * 一次片选取出本组温度和六轴，避免分别读各轴时跨过一次数据更新。 */
    uint8_t raw[14];
    if (!bus->read(bus->context, TEMP_DATA1, raw, sizeof(raw))) {
        return IMU_BUS_ERROR;
    }
    int16_t words[7]; /* 寄存器顺序为温度、加速度 XYZ、角速度 XYZ，每项两个字节。 */
    for (size_t i = 0; i < 7; ++i) {
        words[i] = signed_be16(&raw[2 * i]);
        /* 复位默认大端；0x8000 为无效标记。整组检查成功后再写 out，避免上层收到半组数据。 */
        if (words[i] == INT16_MIN) {
            return IMU_NO_DATA;
        }
    }
    /* 直接数据寄存器的温度换算以 25 °C 为零点，灵敏度 132.48 计数/°C。 */
    out->temp_c = 25.0f + words[0] / 132.48f;
    for (size_t axis = 0; axis < 3; ++axis) {
        out->accel_raw[axis] = words[1 + axis];
        out->gyro_raw[axis] = words[4 + axis];
        out->accel_g[axis] = out->accel_raw[axis] / ICM42688_ACCEL_LSB_PER_G;
        out->gyro_dps[axis] = out->gyro_raw[axis] / ICM42688_GYRO_LSB_PER_DPS;
    }
    return IMU_OK;
}
