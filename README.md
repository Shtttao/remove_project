# 机器人 IMU 模块 · STM32H563VIT6 / ICM-42688-P

在 VS Code 中打开父目录 `remove_project`。固件源码、构建输出和文档位于 `project/`，原始资料与项目工具链位于其上一级。已实现传感器采集、开机陀螺零偏校准、六轴姿态融合和旧上位机使用的三种 UART2 遥测帧。目标为独立启动的台架开发板。首次阅读可先看[实施计划与资料依据](docs/实施计划.md)，再看下方接线与验收步骤。

## 第一次读代码：跟踪一组静止读数

先打开 [main.c](src/main.c)。`main` 把 SPI、微秒时钟和串口回调交给 `app_step`。[整机测试](tests/test_app.c) 把这三个接口换成虚拟设备，模拟板子水平静止、芯片 Z 轴朝上。可以沿 `ACCEL_Z` 的两个字节看懂整条链。

| 在 VS Code 中查找 | 留意这个值 |
| --- | --- |
| [test_app.c](tests/test_app.c) 的 `write_reg` | 模拟器复位时设 `regs[0x23] = 0x40`，相邻低字节为零。Z 轴原始字为 `40 00`。 |
| [icm42688.c](src/icm42688.c) 的 `icm42688_read` | 同一次读取包含温度和六轴。两个字节按大端拼成 `0x4000 = 16384`，再除以 16384 LSB/g，得到 `accel_g[2] = 1.0`。 |
| [app.c](src/app.c) 的 `app_step` | 同一份样本走两条路。原始计数进入 A2 的平均窗口；单位为 g 和 °/s 的数组进入姿态估计。 |
| [attitude.c](src/attitude.c) 的 `calibrate` 和 `attitude_update` | 连续静止样本建立开机基准。在这个模拟场景里，校准后持续静止，基座相对起始姿态的四元数为 `(1,0,0,0)`。 |
| [telemetry.c](src/telemetry.c) 的 `telemetry_a2`、`telemetry_a3` | A2 的 Z 轴仍发送 `40 00`；A3 的 w 乘以 10000 后发送 `27 10`。回到测试的 `uart_tx`，可以看到这两条逐字节断言。 |

想往 `main` 之前追，可在 [ST 启动文件](third_party/stm32h5/startup_stm32h563xx.s) 查找 `Reset_Handler`。它设置栈，调用 [SystemInit](third_party/stm32h5/system_stm32h5xx.c)，把有初值的全局变量从 Flash 复制到 RAM，清零 `.bss`，再通过 C 运行库初始化进入 `main`。这些区段的地址由[链接脚本](board/stm32h563vi.ld)定义。

想对照移植前后的实现，可看[旧 F103 主循环](../资料文件/old_model/User_Src/main.cpp)如何轮换三类帧，再看[旧 UART2 发送函数](../资料文件/old_model/HardWareDriver/UART2/UART2.c)如何逐字节打包。资料包里的 [STC 传感器示例](../资料文件/ICM42688资料包/DmxLibraries/DmxDevice/dmx_icm42688.c)可用来对照寄存器操作；H563 实际运行的 SPI 收发位于 [board.c](board/board.c)。

## 在 VS Code 中复现

快捷键通过工作区的 [tasks.json](../.vscode/tasks.json) 在 `project/` 目录调用 [Makefile](Makefile)。[c_cpp_properties.json](../.vscode/c_cpp_properties.json) 告诉编辑器使用哪套编译器、芯片宏和头文件来解释 C 代码；[launch.json](../.vscode/launch.json) 让调试器用构建出的 ELF 符号附加到目标板。

| 操作 | VS Code 入口 | 在 `project/` 目录下的等效命令 |
| --- | --- | --- |
| 构建和检查镜像 | macOS 按 ⇧⌘B | `make firmware verify` |
| 跑主机回归 | “终端 → 运行任务”选择“IMU · 运行主机测试” | `make test` |
| 与旧代码比对 | 任务“IMU · 与旧 UART2 源码逐字节比对” | `make legacy-check` |
| 附加调试 | “运行和调试”选择“STM32H563VIT6 · 附加并暂停（SWD）” | 需要匹配镜像、SWD 探针与实板；此配置会暂停目标 |
| 离线查看抓包 | 在集成终端指定原始二进制串口文件 | `python3 tools/decode_capture.py capture.bin` |

构建产物在 `project/build/firmware/`，包括可查看符号的 `robot-imu.elf`、纯二进制 `robot-imu.bin` 和内存分布 `robot-imu.map`。当前 Mac 使用项目内的 Arm GNU 14.3.Rel1、系统 Make/Clang、VS Code C/C++ 和 Cortex-Debug。调试采用 pyOCD 0.46.1 和 Keil STM32H5xx_DFP 2.3.1。在另一台 Apple Silicon Mac 上可用 `sh tools/setup-arm.sh` 和 `sh tools/setup-debug.sh` 重建环境。调试器识别目标名称 `stm32h563vitx`。

## 从原始数据到串口

1. [板级代码](board/board.c) 配置 HSI、TIM2 微秒计数、SPI1 与 USART2。SPI 采用四线模式 0、1 MHz。USART2 为 115200、8N1。
2. [传感器驱动](src/icm42688.c) 校验 `WHO_AM_I = 0x47`，软复位、等复位完成、写入并读回量程与 200 Hz ODR。确认新数据后从 0x1D 一次取 14 字节，得到温度、加速度 XYZ、角速度 XYZ；单位分别为 °C、g、°/s。
3. [姿态算法](src/attitude.c) 用至少 200 个连续静止样本估算陀螺零偏并记录开机时的水平基准，通常约 1 秒。采用按实际时间间隔的四元数积分与重力方向校正。线性加速度明显时减弱或关闭重力校正。六轴系统给出相对航向，重力无法区分匀速缓慢自转与陀螺零偏，启动时请保持静止。
4. [遥测编码](src/telemetry.c) 在校准后轮流发送 A1、A2、A3。目标为总计约 100 帧/秒，每类约 33 帧/秒。A1 为负向航向、两份相同的倾角、版本字和实际更新率（首个完整测量窗口前为零）。旧 UART2 将两次 A1 之间的循环次数乘以 50，台架版按实际秒数统计。A2 为十点平均的加速度、去零偏角速度和陀螺零偏，均为计数值。A3 为按 10000 缩放的 w/x/y/z。台架版本为 `A5A5 4801 0000`，上位机如果筛选版本，需要核对。
5. [采样与恢复](src/app.c) 只为新样本计算姿态。连续传输故障、传感器意外复位或 100 ms 无有效样本会停报，约 500 ms 后重试并重新静止校准。调试器可以观察 `imu_app.online`、`last_fault`、`samples`、`stalls`、`tx_errors`、`update_hz` 和 `attitude.bias_dps`。

本版以重力校正的四元数滤波重新实现姿态估计；输入时刻、滤波响应和新器件灵敏度均与旧模块有差异。A1 倾角按旧的上位机字段定义输出。最终的机器人安装偏航角需要实物确定。旧 UART1、上位机下行控制、里程计辅助、Flash 参数、IAP 和产品激活流程尚待集成。旧工程配置和新板 Bootloader 布局未在所给资料中找到。

## 首次台架接线

以下是本固件选择的接线。图中板卡型号明确，资料未给出 X Pulse 原理图。接线前依据两块实物丝印确认引脚已引出、板载器件占用、供电范围和电平。建议先在独立开发板上验证。

| STM32H563VIT6 | IMU 小板或仪器 | 用途 |
| --- | --- | --- |
| PA4 | CS / nCS | 软件片选，空闲高 |
| PA5 | SCK / SCLK | SPI1 时钟 |
| PA6 | SDO / MISO | 传感器至 MCU |
| PA7 | SDI / MOSI | MCU 至传感器 |
| PA2 | 3.3 V USB 转串口的 RX | 旧 UART2 遥测输出 |
| GND | IMU、串口适配器、探针 GND | 共地 |
| 与模块标称输入匹配的电源 | IMU 供电脚 | 裸芯片的 VDD/VDDIO 依 TDK 手册；确认小板稳压和电平后接电 |
| PA13 / PA14 | 探针 SWDIO / SWCLK | 调试，还需目标电压参考；NRST 依探针接线 |

镜像以 **0x08000000** 为起始地址，使用 2 MiB 内部 Flash、256 KiB SRAM1 和关闭 TrustZone 的独立启动配置。装有 Bootloader 或启用 TrustZone 的板需要核对现有布局并调整链接和启动文件。原 F103 的 0x08005800 偏移仅属于旧工程。

## 实板验收顺序

| 顺序 | 台架操作 | 观察与记录 |
| --- | --- | --- |
| 1 | 确认实物型号、供电和接线，读取 MCU 状态与既有固件布局 | 调试器能识别目标。确认可用的独立开发板和备份后再部署 |
| 2 | 用逻辑分析仪看 CS、SCK、SDI、SDO | CS 低期间发 0xF5、返回 0x47；记录配置读回与数据突发；若持续全高/全低则先查供电和总线 |
| 3 | 通电静置，完成校准后采集 60 秒 | 记录实际采样率、串口每类帧率、校验失败数与重连次数；设计目标 200 Hz、每类约 33 Hz |
| 4 | 六面静置，将 ±X、±Y、±Z 依次朝上 | 在 A2 中检查对应轴约 ±16384 计数，其余两轴接近零。记录各面实测均值；由项目验收方确定误差容限 |
| 5 | 绕已标记方向旋转 90°，随后长时间静置 | 记录 A3 姿态变化、A1 航向符号、四元数范数及每分钟航向漂移；固定并记录安装方向 |
| 6 | 在台架制造传感器掉线并恢复，与旧模块在同样动作下抓包 | 中断期间停报，重新静止后恢复。用离线解码器比较字段、版本过滤、方向、延迟与噪声；用真实上位机检查接收情况 |

## 已验证的范围

- 主机运行传感器、姿态、遥测、整机四组 C 回归；另有 Python 抓包解码测试。覆盖寄存器故障、字节序与单位、静止校准、已知转角、时间戳回绕、数据中断和重连。用测试代替总线时，执行的是与固件相同的信号处理和编码代码。
- `make legacy-check` 从资料中的旧 UART2 源码提取三个实际发送函数作参照。A2 遍历 65,536 个输入字，A1/A3 采用已知姿态样例。它验证帧字段和字节表示；真实传感器误差、运动轨迹及接收端行为需要实测。
- 交叉编译和镜像检查覆盖 Cortex-M33、硬浮点 ABI、Flash 向量表、栈顶、Thumb 复位地址，以及数据段与栈的边界。当前未检测到调试探针，尚无实板采样、烧录或机器人联调结果。

硬件与依赖来源见[实施计划](docs/实施计划.md)及[第三方源码和版本](third_party/README.md)。
