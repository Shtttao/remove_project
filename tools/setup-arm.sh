#!/bin/sh
# 在 Apple Silicon 的 macOS 上安装裸机交叉编译器。脚本无需参数。
# 从脚本位置定位工作区根目录，让 Make 能从 .tools/arm-gnu 找到编译器。
# set -e 遇到命令失败就退出，-u 遇到未定义变量就退出。
set -eu
cd "$(dirname "$0")/../.."
release=14.3.rel1
# 包名固定了主机平台和目标架构。sha 是此版本压缩包的 SHA-256 校验值。
archive="arm-gnu-toolchain-${release}-darwin-arm64-arm-none-eabi.tar.xz"
sha=30f4d08b219190a37cded6aa796f4549504902c53cfc3c7e044a8490b6eba1f7
base="https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/${release}/binrel"
if [ -x .tools/arm-gnu/bin/arm-none-eabi-gcc ]; then
    # 已有本地编译器时打印版本并结束，保留当前安装。
    .tools/arm-gnu/bin/arm-none-eabi-gcc --version
    exit 0
fi
if [ "$(uname -s)-$(uname -m)" != Darwin-arm64 ]; then
    # 其他主机按提示安装对应版本到 PATH，Make 也能使用系统上的交叉编译器。
    echo 'Install Arm GNU Toolchain (arm-none-eabi) on PATH for this host.' >&2
    exit 1
fi
mkdir -p .tools
# 先下载到临时目录，脚本退出时清理。连接超时为 15 秒，下载失败最多重试 3 次。
stage=$(mktemp -d .tools/arm-install.XXXXXX)
trap 'rm -rf "$stage"' EXIT
curl --fail --location --retry 3 --connect-timeout 15 --output "$stage/$archive" "$base/$archive"
# 校验成功才解压。去掉压缩包的顶层目录，使最终路径固定为 .tools/arm-gnu/bin。
printf '%s  %s\n' "$sha" "$stage/$archive" | shasum -a 256 -c -
mkdir "$stage/toolchain"
tar -xf "$stage/$archive" -C "$stage/toolchain" --strip-components 1
mv "$stage/toolchain" .tools/arm-gnu
# 打印实际安装的编译器版本，供核对构建环境。
.tools/arm-gnu/bin/arm-none-eabi-gcc --version
