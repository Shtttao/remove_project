# 固件使用的第三方源码

| 目录 | 上游版本 | 本项目使用的文件 | 许可证 |
| --- | --- | --- | --- |
| `stm32h5/` | [STMicroelectronics/cmsis-device-h5 v1.7.0](https://github.com/STMicroelectronics/cmsis-device-h5/tree/884b8dc78e41cbfca008363342b17f4a9e8641f7) | `Include/` 下的 STM32H563、H5 选择与系统头文件，以及 `Source/Templates/` 下的系统初始化和 GNU STM32H563 启动文件 | [Apache-2.0](stm32h5/LICENSE.md) |
| `cmsis/` | [Arm CMSIS_5 5.9.0](https://github.com/ARM-software/CMSIS_5/tree/2b7495b8535bdcb306dac29b9ded4cfb679d7e5c) | `CMSIS/Core/Include/` 下的 Cortex-M33、版本、编译器选择、GCC 与 Armv8-M MPU 头文件 | [Apache-2.0](cmsis/LICENSE.txt) |

以上源码逐字保留上游文件及版权声明。工程按 GNU Arm 裸机工具链构建，设备选择宏为 `STM32H563xx`。启动文件设置 MSP 和 MSPLIM，调用 `SystemInit`，复制 `.data`，清零 `.bss`，再调用 C 运行库初始化和 `main`。

芯片引脚与复用功能可按 [ST 的 LQFP100 封装数据](https://github.com/STMicroelectronics/STM32_open_pin_data/blob/45c04599e2e0a788c5ed9b6218dc5a28a36041f7/mcu/STM32H563VITx.xml) 和 [GPIO AF 表](https://github.com/STMicroelectronics/STM32_open_pin_data/blob/45c04599e2e0a788c5ed9b6218dc5a28a36041f7/mcu/IP/GPIO-STM32H56x_gpio_v1_0_Modes.xml) 复核。数据手册给出复位后的 HSI/2 配置。板级初始化先确认 Flash 为 3 个等待周期，再将 HSI 配为不分频；SPI1 使用 HSI 经 CLKP 后分频 64。SPI 结束处理保留最后一拍的下降沿。[DS14258 数据手册](https://www.st.com/resource/en/datasheet/stm32h563vi.pdf) · [RM0481 参考手册](https://www.st.com/resource/en/reference_manual/rm0481-stm32h52333xx-stm32h56263xx-and-stm32h573xx-armbased-32bit-mcus-stmicroelectronics.pdf) · [ES0565 芯片勘误](https://www.st.com/resource/en/errata_sheet/es0565-stm32h562xx563xx573xx-device-errata-stmicroelectronics.pdf)
