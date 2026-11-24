# 不指定目标时 make 执行 all。伪目标即使与同名文件重名，也照常执行命令。
.DEFAULT_GOAL := all
.PHONY: all firmware test verify legacy-check
# 分别选择运行在本机和 MCU 上的编译器。?= 允许调用者指定变量，
# CROSS 优先选工作区工具链，否则从 PATH 查找 arm-none-eabi-*。
HOST_CC ?= cc
CROSS ?= $(if $(wildcard ../.tools/arm-gnu/bin/arm-none-eabi-gcc),../.tools/arm-gnu/bin/arm-none-eabi-,arm-none-eabi-)
ARM_CC := $(CROSS)gcc
# Thumb、单精度浮点指令和硬浮点调用约定要在编译与链接时保持一致。
CPU := -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
WARN := -Wall -Wextra -Werror
CORE := src/icm42688.c src/attitude.c src/telemetry.c src/app.c
FW_DIR := build/firmware
HOST_DIR := build/host
FW_BASE := $(FW_DIR)/robot-imu
INCLUDES := -Isrc -Iboard -Ithird_party/stm32h5/include -Ithird_party/cmsis/include
# -Og 和 -g3 方便源码调试。按函数、数据分段，链接时可裁掉未用部分。
FW_CFLAGS := $(CPU) -std=c11 -Og -g3 -ffunction-sections -fdata-sections -fno-common $(WARN)
# $(call ARM_CRT,文件名) 将其参数代入 $(1)，请 GCC 找到匹配此架构的运行库对象。
ARM_CRT = $(shell $(ARM_CC) $(CPU) -print-file-name=$(1))
# 设备宏让 ST 公共头文件选择 H563 的寄存器定义。主机测试只需可移植的 src。
FW_CPPFLAGS := $(INCLUDES) -DSTM32H563xx
HOST_FLAGS := -std=c11 -O2 -g -Isrc $(WARN) $(HOST_EXTRA_FLAGS)
FW_C := $(CORE) src/main.c board/board.c third_party/stm32h5/system_stm32h5xx.c
STARTUP := third_party/stm32h5/startup_stm32h563xx.s
# 把源文件的扩展名改为 .o，再添加输出目录前缀，在 build 中保留源文件层级。
FW_OBJ := $(addprefix $(FW_DIR)/,$(FW_C:.c=.o) $(STARTUP:.s=.o))
HOST_TESTS := $(addprefix $(HOST_DIR)/,test_icm42688 test_attitude test_telemetry test_app)

all: test firmware verify

# BIN 是连续的 Flash 字节。ELF 另含符号、段地址等信息，size 据此汇总内存占用。
firmware: $(FW_BASE).bin
	$(CROSS)size $(FW_BASE).elf

# 模式规则中 % 匹配源文件名。$@ 是当前目标，$(@D) 是其父目录，
# $< 是首个依赖。在这里它就是待编译的 C 源文件或汇编文件。
# -MMD 为 C 源文件生成头文件依赖表 .d，-MP 为其中的头文件生成占位目标。
$(FW_DIR)/%.o: %.c Makefile
	@mkdir -p "$(@D)"
	$(ARM_CC) $(FW_CFLAGS) $(FW_CPPFLAGS) -MMD -MP -c "$<" -o "$@"

$(FW_DIR)/%.o: %.s Makefile
	@mkdir -p "$(@D)"
	$(ARM_CC) $(CPU) -g3 -c "$<" -o "$@"

# 使用 ST 的复位入口，所以以 -nostartfiles 跳过默认启动文件。
# crti.o 与 crtn.o 提供运行库初始化段的首尾。-T 选择本板内存布局。
# nano.specs 选择精简 newlib，nosys.specs 提供裸机系统调用桩。
# --gc-sections 去除未用的输入段，MAP 记录每个符号与段最终放置的位置。
$(FW_BASE).elf: $(FW_OBJ) board/stm32h563vi.ld
	$(ARM_CC) $(CPU) -nostartfiles $(call ARM_CRT,crti.o) $(FW_OBJ) -Tboard/stm32h563vi.ld --specs=nano.specs --specs=nosys.specs \
	    -Wl,--gc-sections,-Map,$(FW_BASE).map -Wl,--start-group -lm -lc -lgcc -Wl,--end-group $(call ARM_CRT,crtn.o) -o "$@"

# objcopy 取出可装载内容制成 BIN。校验脚本同时读取 BIN、ELF 并比对启动向量。
$(FW_BASE).bin: $(FW_BASE).elf
	$(CROSS)objcopy -O binary "$<" "$@"

verify: $(FW_BASE).bin
	python3 tools/verify_image.py "$(CROSS)" "$(FW_BASE)"

# 先编译并依次运行主机测试，任一失败就退出。$$ 把一个 $ 传给 shell，
# 因而 $$t 在执行时表示当前循环的测试程序。最后验证 Python 抓包解码工具。
test: $(HOST_TESTS)
	@set -e; for t in $(HOST_TESTS); do "$$t"; done
	python3 tests/test_capture.py

# 旧工程资料的对照检查单独运行，可用 make legacy-check 调用。
legacy-check:
	python3 tests/check_legacy.py

# 每条规则列出一个测试所需的源码，共用下方的编译命令。
# $^ 是去重后的全部依赖，filter 只把 .c 交给编译器。头文件变更仍会触发重编。
$(HOST_DIR)/test_icm42688: tests/test_icm42688.c src/icm42688.c
$(HOST_DIR)/test_attitude: tests/test_attitude.c src/attitude.c
$(HOST_DIR)/test_telemetry: tests/test_telemetry.c src/telemetry.c
$(HOST_DIR)/test_app: tests/test_app.c $(CORE)
$(HOST_TESTS): Makefile $(wildcard src/*.h)
	@mkdir -p "$(@D)"
	$(HOST_CC) $(HOST_FLAGS) $(filter %.c,$^) -lm -o "$@"

# 首次构建时尚无 .d，前导减号允许忽略缺失文件。之后会自动跟踪头文件变更。
-include $(FW_OBJ:.o=.d)
