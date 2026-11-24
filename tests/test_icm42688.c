#include "icm42688.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "%s:%d: failed: %s\n", __FILE__, __LINE__, #expr); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

/* 模拟器直接使用手册给出的地址和字节，避免测试与驱动复用同一个写错的常量。
 * ms 记录虚拟时间；fail_transfer、ignore_write 分别用来注入传输失败和写入未生效。 */
typedef struct {
    uint8_t reg[128];
    uint8_t bank;
    uint32_t ms;
    uint32_t accessible_at_ms;
    uint32_t powered_at_ms;
    unsigned transfers;
    unsigned fail_transfer;
    uint8_t ignore_write;
    unsigned resets;
    unsigned bursts;
    bool suppress_reset_done;
} FakeImu;

static FakeImu new_device(void)
{
    FakeImu dev = {0};
    dev.reg[0x75] = 0x47;
    dev.accessible_at_ms = 1;
    dev.bank = 2; /* 模拟 MCU 已重启，而传感器仍停留在之前的寄存器组。 */
    return dev;
}

static bool fake_read(void *context, uint8_t reg, uint8_t *data, size_t len)
{
    FakeImu *dev = context;
    CHECK(dev->ms >= dev->accessible_at_ms);
    CHECK((size_t)reg + len <= sizeof(dev->reg));
    if (++dev->transfers == dev->fail_transfer) {
        data[0] = 0x5a; /* 故意留一个已传输字节，检验突发读中途失败时上层输出的完整性。 */
        return false;
    }
    CHECK(dev->bank == 0);
    if (reg == 0x1d) {
        CHECK(len == 14);
        dev->bursts++;
    } else {
        CHECK(len == 1);
    }
    memcpy(data, &dev->reg[reg], len);
    if (reg == 0x2d) {
        dev->reg[reg] = 0; /* 模拟 INT_STATUS 的读清特性。 */
    }
    return true;
}

/* 单独处理切换 Bank、软复位和两种传感器上电。其余写操作保存在寄存器数组中。 */
static bool fake_write(void *context, uint8_t reg, uint8_t value)
{
    FakeImu *dev = context;
    CHECK(dev->ms >= dev->accessible_at_ms);
    CHECK(reg < sizeof(dev->reg));
    if (++dev->transfers == dev->fail_transfer) {
        return false;
    }
    if (reg == 0x76) {
        dev->bank = value;
        return true;
    }
    CHECK(dev->bank == 0);
    if (reg == 0x11 && value == 1) {
        dev->resets++;
        dev->accessible_at_ms = dev->ms + 1;
        dev->reg[0x2d] = dev->suppress_reset_done ? 0x00 : 0x10;
        dev->reg[0x4e] = 0x00;
        dev->reg[0x4f] = 0x06;
        dev->reg[0x50] = 0x06;
    } else if (reg != dev->ignore_write) {
        dev->reg[reg] = value;
        if (reg == 0x4e && value == 0x0f) {
            CHECK(dev->reg[0x4f] == 0x07 && dev->reg[0x50] == 0x67);
            dev->powered_at_ms = dev->ms;
        }
    }
    return true;
}

/* 推进虚拟时间。读写回调会检查复位期间有没有过早访问寄存器。 */
static void fake_delay(void *context, uint32_t ms)
{
    FakeImu *dev = context;
    dev->ms += ms;
}

/* 用 context 把三个回调绑定到同一个模拟芯片，驱动使用的公开接口与实板一致。 */
static ImuBus bus_for(FakeImu *dev)
{
    ImuBus bus = { .read = fake_read, .write = fake_write, .delay_ms = fake_delay,
                   .context = dev };
    return bus;
}

/* 成功标准包括软复位次数、上电等待时长以及低噪声、量程、输出速率的最终值。 */
static void test_init(void)
{
    FakeImu dev = new_device();
    ImuBus bus = bus_for(&dev);
    CHECK(icm42688_init(&bus) == IMU_OK);
    CHECK(dev.resets == 1 && dev.powered_at_ms >= 2);
    CHECK(dev.ms - dev.powered_at_ms >= 45);
    CHECK(dev.reg[0x4e] == 0x0f && dev.reg[0x4f] == 0x07 && dev.reg[0x50] == 0x67);
}

static void test_init_errors(void)
{
    FakeImu dev = new_device();
    ImuBus bus = bus_for(&dev);
    dev.reg[0x75] = 0x46;
    CHECK(icm42688_init(&bus) == IMU_ID_MISMATCH);
    CHECK(dev.resets == 0);
    dev = new_device();
    dev.suppress_reset_done = true;
    CHECK(icm42688_init(&bus) == IMU_RESET_ERROR);
    uint8_t config_regs[] = {0x4e, 0x4f, 0x50};
    for (size_t i = 0; i < sizeof(config_regs); ++i) {
        dev = new_device();
        dev.ignore_write = config_regs[i];
        CHECK(icm42688_init(&bus) == IMU_CONFIG_ERROR);
    }
    dev = new_device();
    CHECK(icm42688_init(&bus) == IMU_OK);
    /* 先从成功路径取得总传输次数，然后令第 1、2、… 次传输逐一失败；
     * 每次都应在当场报错并停止继续写寄存器。 */
    unsigned total = dev.transfers;
    for (unsigned stage = 1; stage <= total; ++stage) {
        dev = new_device();
        dev.fail_transfer = stage;
        CHECK(icm42688_init(&bus) == IMU_BUS_ERROR);
        CHECK(dev.transfers == stage);
    }
}

/* 物理量经过浮点除法，以容差比较；上面的寄存器和下面的原始计数则精确比较。 */
static bool near(float actual, float expected, float tolerance)
{
    return actual >= expected - tolerance && actual <= expected + tolerance;
}

static void test_read(void)
{
    FakeImu dev = new_device();
    ImuBus bus = bus_for(&dev);
    ImuSample out = {0};
    CHECK(icm42688_init(&bus) == IMU_OK);
    const uint8_t wire[] = {
        0xfc, 0x1e,              /* 温度原始值 −994 个计数。 */
        0xc0, 0x00, 0x20, 0x00, 0x80, 0x02, /* 加速度分别为 −16384、8192、−32766。 */
        0x06, 0x68, 0xf9, 0x98, 0x7f, 0xff  /* 角速度分别为 1640、−1640、32767。 */
    };
    memcpy(&dev.reg[0x1d], wire, sizeof(wire));
    dev.reg[0x2d] = 0x28; /* 就绪位与无关状态位同时置位，驱动仍应正确读取。 */
    CHECK(icm42688_read(&bus, &out) == IMU_OK);
    CHECK(dev.bursts == 1);
    CHECK(out.accel_raw[0] == -16384 && out.accel_raw[1] == 8192 && out.accel_raw[2] == -32766);
    CHECK(out.gyro_raw[0] == 1640 && out.gyro_raw[1] == -1640 && out.gyro_raw[2] == 32767);
    CHECK(out.accel_g[0] == -1.0f && out.accel_g[1] == 0.5f);
    CHECK(near(out.accel_g[2], -1.999878f, 0.00001f));
    CHECK(near(out.gyro_dps[0], 100.0f, 0.001f) && near(out.gyro_dps[1], -100.0f, 0.001f));
    CHECK(near(out.gyro_dps[2], 1997.988f, 0.001f));
    CHECK(near(out.temp_c, 17.49698f, 0.0001f));
    dev.reg[0x2d] = 0x20;
    CHECK(icm42688_read(&bus, &out) == IMU_NO_DATA);
    CHECK(dev.bursts == 1);
}

static void test_incomplete_read(void)
{
    FakeImu dev = new_device();
    ImuBus bus = bus_for(&dev);
    ImuSample out;
    unsigned char original[sizeof(out)];
    /* 先给输出填入可辨认的字节。无新数据、读失败或无效字出现时都须逐字节保留。 */
    memset(&out, 0x5a, sizeof(out));
    memcpy(original, &out, sizeof(out));
    CHECK(icm42688_init(&bus) == IMU_OK);
    CHECK(icm42688_read(&bus, &out) == IMU_NO_DATA);
    CHECK(memcmp(original, &out, sizeof(out)) == 0);
    for (unsigned stage = 1; stage <= 2; ++stage) {
        dev.reg[0x2d] = 0x08;
        dev.fail_transfer = dev.transfers + stage;
        CHECK(icm42688_read(&bus, &out) == IMU_BUS_ERROR);
        CHECK(memcmp(original, &out, sizeof(out)) == 0);
    }
    for (size_t offset = 0; offset < 14; offset += 2) {
        dev.reg[0x2d] = 0x08;
        dev.reg[0x1d + offset] = 0x80; /* 逐个将七个数据字设为无效值 0x8000。 */
        CHECK(icm42688_read(&bus, &out) == IMU_NO_DATA);
        CHECK(memcmp(original, &out, sizeof(out)) == 0);
        dev.reg[0x1d + offset] = 0;
    }
}

/* 空指针或缺少必要回调时立即报参数错误，模拟芯片上的传输计数应始终为零。 */
static void test_arguments(void)
{
    FakeImu dev = new_device();
    ImuBus bus = bus_for(&dev);
    ImuSample out;
    CHECK(icm42688_init(NULL) == IMU_BAD_ARGUMENT);
    CHECK(icm42688_read(NULL, &out) == IMU_BAD_ARGUMENT);
    CHECK(icm42688_read(&bus, NULL) == IMU_BAD_ARGUMENT);
    bus.write = NULL;
    CHECK(icm42688_init(&bus) == IMU_BAD_ARGUMENT);
    bus = bus_for(&dev);
    bus.delay_ms = NULL;
    CHECK(icm42688_init(&bus) == IMU_BAD_ARGUMENT);
    bus = bus_for(&dev);
    bus.read = NULL;
    CHECK(icm42688_init(&bus) == IMU_BAD_ARGUMENT);
    CHECK(icm42688_read(&bus, &out) == IMU_BAD_ARGUMENT);
    CHECK(dev.transfers == 0);
}

int main(void)
{
    test_init();
    test_init_errors();
    test_read();
    test_incomplete_read();
    test_arguments();
    puts("PASS icm42688: startup, identity, readback, bus faults, data ready, raw axes, units, invalid samples");
    return EXIT_SUCCESS;
}
