#!/bin/sh
# 准备 VS Code 的 SWD 调试工具。无需参数或连接开发板，文件写到工作区根目录的 .tools。
# -e 使失败的命令中止安装，-u 使未定义的变量触发错误。
set -eu
cd "$(dirname "$0")/../.."

venv=.tools/debug-venv
# pyOCD 负责与调试探针通信，Keil 设备包提供 STM32H563 的目标定义。
pyocd_version=0.46.1
pack=Keil.STM32H5xx_DFP.2.3.1.pack
digest=906b2f8802845ed21c4bc31512a816065991bd9839f1441d0126a667a50347b1

if [ ! -x "$venv/bin/python" ]; then
    # 在独立的 Python 环境中安装依赖，重复运行时可以复用这个目录。
    python3 -m venv "$venv"
fi
if [ "$("$venv/bin/python" -m pyocd --version 2>/dev/null || true)" != "$pyocd_version" ]; then
    # 初次安装时版本查询可能失败。缺少 pyOCD 或版本不符时安装指定版本。
    "$venv/bin/python" -m pip --disable-pip-version-check install "pyocd==$pyocd_version"
fi
check_pack() {
    # 参数是待检设备包的路径。把预期摘要和路径交给 shasum，退出码表示校验结果。
    printf '%s  %s\n' "$digest" "$1" | shasum -a 256 --status -c -
}
if [ ! -f ".tools/$pack" ] || ! check_pack ".tools/$pack"; then
    # 先校验已下载的设备包，缺失或校验失败时才重新下载。新包也通过校验后才替换。
    tmp=$(mktemp .tools/dfp-download.XXXXXX)
    trap 'rm -f "$tmp"' EXIT
    curl -fL --retry 2 --connect-timeout 10 --max-time 180 "https://www.keil.com/pack/$pack" -o "$tmp"
    check_pack "$tmp"
    mv "$tmp" ".tools/$pack"
fi

# 生成 VS Code 使用的 .tools/pyocd 启动脚本。它按自身位置找到解释器，
# 即使工作区移动或路径含空格，仍可把原命令的全部参数交给 pyOCD。
cat > .tools/pyocd <<'EOF'
#!/bin/sh
exec "$(dirname "$0")/debug-venv/bin/python" -m pyocd "$@"
EOF
chmod +x .tools/pyocd
.tools/pyocd --version
# 从设备包枚举目标，要求型号和来源同时匹配。找到时输出相应行，找不到时返回失败。
targets=$(.tools/pyocd list --targets --pack ".tools/$pack" --name stm32h563vitx)
printf '%s\n' "$targets" | awk '$1 == "stm32h563vitx" && $NF == "pack" { print; found=1 } END { exit !found }'
