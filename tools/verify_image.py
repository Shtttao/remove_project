"""Check that a little-endian Cortex-M33 firmware image can boot at the planned address."""
# 离线检查启动布局。ELF 给出符号和目标架构，BIN 给出将放到 Flash 的实际字节。
import struct
import subprocess
import sys
from pathlib import Path

# Make 传入工具链前缀（如 arm-none-eabi-）和不带扩展名的固件路径。
prefix, base = sys.argv[1:]
elf = base + ".elf"
binary = Path(base + ".bin").read_bytes()


def require(condition, message):
    # 任一条件失败就带原因退出，make verify 会将其报告为失败。
    if not condition:
        raise SystemExit("Image verification failed: " + message)


def tool(name, *args):
    # 用构建固件的同一套工具读 ELF。check_output 会传播子进程失败。
    return subprocess.check_output([prefix + name, *args, elf], text=True)


# -A 读 Arm 架构属性。分别核对 v8-M Mainline 指令体系和浮点参数传递约定。
attributes = tool("readelf", "-A")
require("Tag_CPU_arch: v8-M.mainline" in attributes, "Cortex-M33 architecture")
require("Tag_ABI_VFP_args: VFP registers" in attributes, "hard-float ABI")
syms = {}
# nm -n 按地址列符号。带地址的记录通常依次为十六进制地址、类型、名称。
for line in tool("nm", "-n").splitlines():
    fields = line.split()
    if len(fields) == 3:
        syms[fields[2]] = int(fields[0], 16)

# 这里的末地址都取区间上界，不含上界本身。检查范围与本板链接脚本一致。
flash, flash_end = 0x08000000, 0x08200000
ram, ram_end = 0x20000000, 0x20040000
require(8 <= len(binary) <= flash_end - flash, "binary length")
# Cortex-M 复位时取向量表的前两个字。<II 按小端序读取两个 32 位无符号数，
# 对应初始栈指针 MSP 和复位向量。MSP 初值在 SRAM1 顶端并按 8 字节对齐。
stack, reset = struct.unpack_from("<II", binary)
require(stack == syms["_estack"] == ram_end and stack % 8 == 0, "initial SP")
require(syms["g_pfnVectors"] == flash, "vector base")
# 复位向量最低位必须为 1，表示执行 Thumb 指令。去掉状态位后应等于
# ELF 中 Reset_Handler 的地址，并且复位入口必须落在 BIN 实际覆盖的范围内。
require((reset & 1) and reset & ~1 == syms["Reset_Handler"], "Thumb reset vector")
require(flash <= reset < flash + len(binary), "reset handler outside image")
# 启动代码把 .data 复制进 RAM，随后清零 .bss。两个区间不能越界或侵入栈。
require(ram <= syms["_sdata"] <= syms["_edata"] <= syms["_sbss"] <= syms["_ebss"]
        <= syms["_sstack"] < stack, "SRAM sections overlap stack or are out of bounds")
# _sidata 是 Flash 中的复制源，复制长度由 RAM 中 .data 的起止地址之差决定。
# 连最后一个初值字节也要落在 BIN 内，才能发现镜像尾部遭截断的情况。
require(flash <= syms["_sidata"] <= flash + len(binary), "data initialization image")
require(syms["_sidata"] + syms["_edata"] - syms["_sdata"] <= flash + len(binary),
        "truncated data initialization image")
# 抽查应用入口、板级初始化、调度和传感器初始化符号均落在此镜像内。
for symbol in ("main", "board_init", "app_step", "icm42688_init"):
    require(flash <= syms[symbol] < flash + len(binary), "missing executable code: " + symbol)
print(f"PASS Cortex-M33 hard-float image, {len(binary)} bytes, "
      f"vectors 0x{flash:08X}, initial SP 0x{stack:08X}, reset 0x{reset:08X}")
