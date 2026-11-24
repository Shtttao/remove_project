/* 主机端逐字节对照测试。telemetry.h 声明新版编码接口，旧版发送函数
 * 由 check_legacy.py 从原工程提取，放进下方包含的 .inc 文件。 */
#include "telemetry.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t reference[24];
static size_t sent;
/* 替代旧固件的串口写操作。每次调用把一个字节记入参考帧，最长的 A2 帧占 24 字节。 */
static void UART2_Put_Char(uint8_t byte) {
    assert(sent < sizeof reference);
    reference[sent++] = byte;
}
#include "legacy_uart_functions.inc"

/* 同时比较帧长和所有字节，包含字段顺序、校验和与帧尾。随后清空计数供下一帧使用。 */
static void match(const uint8_t *actual, size_t size) {
    assert(size == sent && !memcmp(actual, reference, size));
    sent = 0;
}

int main(void) {
    uint8_t out[TELEMETRY_MAX_SIZE];
    /* 先用旧函数编码手工算出的 A1 字段，再由新函数从四元数编码同一姿态。
     * 角度字段以 0.1 度计，故 -900 表示 -90 度。再用水平姿态覆盖零值。 */
    const float diagonal[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    UART2_ReportIMU(-900, 900, 900, 0xa5a5, 0x4801, 0, 200);
    match(out, telemetry_a1(out, diagonal, 200));
    const float level[4] = {1, 0, 0, 0};
    UART2_ReportIMU(0, 0, 0, 0xa5a5, 0x4801, 0, 0);
    match(out, telemetry_a1(out, level, 0));
    /* A3 按 w、x、y、z 发送四个分量。旧函数接收放大 10000 倍后的有符号整数。 */
    const float q[4] = {0.5f, -0.5f, 0.5f, -0.5f};
    UART2_ReportQua(5000, -5000, 5000, -5000);
    match(out, telemetry_a3(out, q));
    /* 让 v 遍历所有 65536 个 16 位有符号值。各轴取不同数据，
     * 覆盖字节顺序、校验和回绕和 -32768 的旧式编码。 */
    for (int32_t v = INT16_MIN; v <= INT16_MAX; ++v) {
        int16_t a[3] = {(int16_t)v, (int16_t)(-1 - v), 16384};
        int16_t g[3] = {13, (int16_t)v, -324};
        /* 新接口接收的零偏单位是度/秒。在 ±2000 度/秒量程下，
         * ±1 度/秒取整为 ±16 计数，所以旧函数的校正值分别为 13-16 和 -324+16。 */
        const float bias[3] = {1, 0, -1};
        UART2_ReportMotion(a[0], a[1], a[2], -3, g[1], -308, 16, 0, -16);
        TelemetryMotion m;
        telemetry_motion_init(&m);
        telemetry_motion_update(&m, a, g);
        match(out, telemetry_a2(out, &m, bias));
    }
    puts("PASS legacy UART2 differential: 65536 motion inputs and A1/A3 fixtures");
}
