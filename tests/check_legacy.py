"""Compare new transmit bytes to the three transcribed-from-file legacy UART2 functions."""
import os
import re
import subprocess
import tempfile
from pathlib import Path

# 无需连接 MCU。此脚本从资料目录读旧发送函数，再用主机 C 编译器构建比较程序。
# 可用 HOST_CC 指定主机编译器。成功时打印比较结果，编译失败或断言失败时退出。
root = Path(__file__).resolve().parents[1]
old = root.parent / "资料文件/old_model/HardWareDriver/UART2/UART2.c"
text = old.read_bytes().decode("utf-8", errors="replace")
# 先剔除注释，以免把注释中保留的旧函数版本或花括号误当作有效代码。
text = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)
bodies = []
for name in ("UART2_ReportIMU", "UART2_ReportMotion", "UART2_ReportQua"):
    # 只匹配形参后面紧跟左花括号的函数定义。三个名称依次对应 A1、A2、A3 帧。
    function = re.search(r"\bvoid\s+" + name + r"\s*\([^)]*\)\s*\{", text)
    if function is None:
        raise SystemExit(f"Missing reference function {name} in {old}")
    depth = 1
    end = function.end()
    # 从函数的左花括号后开始扫描。嵌套块增加深度，回到零即得到完整函数体。
    while depth and end < len(text):
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    if depth:
        raise SystemExit(f"Incomplete reference function {name}")
    bodies.append(text[function.start():end])
with tempfile.TemporaryDirectory() as temp:
    # 将提取的三个定义放进临时 .inc 文件，legacy_uart.c 通过 #include 直接调用它们。
    # -I 分别指定临时目录和新版头文件目录。-funsigned-char 匹配旧发送函数的字节类型。
    tmp = Path(temp)
    (tmp / "legacy_uart_functions.inc").write_text("\n".join(bodies))
    exe = tmp / "compare"
    subprocess.run([os.environ.get("HOST_CC", "cc"), "-std=c11", "-O2", "-funsigned-char",
                    "-I", str(tmp), "-I", str(root / "src"), str(root / "tests/legacy_uart.c"),
                    str(root / "src/telemetry.c"), "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
