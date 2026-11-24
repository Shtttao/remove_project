"""Decode raw legacy UART2 records from a bench capture as JSON lines."""
import argparse
import json
import struct
import sys

# 每帧以 A5 5A 开头。长度字节计算从长度字段到 AA 帧尾的字节数。
# 因此，下面的 18、22、12 都要加上两个帧头字节，才是串口上的总长度。
LENGTH = {0xA1: 18, 0xA2: 22, 0xA3: 12}
HEADER = b"\xa5\x5a"


def signed(word):
    # 旧协议把符号放在最高位，低 15 位存绝对值。例如 0x8384 表示 -900。
    return -(word & 0x7fff) if word & 0x8000 else word


def decode(frame):
    # inspect 已验过帧。略过帧头、长度和类型，再去掉校验和与帧尾。
    # “>H”按高字节在前的顺序，每两个字节读出一个无符号整数。
    kind = frame[3]
    raw = [word for (word,) in struct.iter_unpack(">H", frame[4:-2])]
    fields = [signed(v) for v in raw]
    row = {"type": f"{kind:02X}"}
    if kind == 0xa1:
        # 前三个字段以 0.1 度计。旧协议在两个倾角字段里发送同一个值。
        # 三个版本字按原始位值保存，最后一个字保存发送端给出的更新频率。
        row.update(yaw_deg=fields[0] / 10, tilt1_deg=fields[1] / 10, tilt2_deg=fields[2] / 10,
                   version_words=raw[3:6], update_hz=raw[6])
    elif kind == 0xa2:
        # 每组三轴都按 X、Y、Z 排列，单位是传感器计数。陀螺仪数据已减去末组零偏。
        row.update(accel_counts=fields[:3], gyro_corrected_counts=fields[3:6],
                   gyro_bias_counts=fields[6:])
    else:
        # 剩下的有效类型是 A3。发送端把无量纲四元数 w、x、y、z 各乘以 10000。
        row["q_wxyz"] = [v / 10000 for v in fields]
    return row


def inspect(stream):
    # 串口抓包可能从一帧中间开始。逐块读取，保留未收全的字节到下一轮。
    data = bytearray()
    valid = bad = skipped = 0
    while True:
        chunk = stream.read(8192)
        data.extend(chunk)
        pos = 0
        while pos < len(data):
            start = data.find(HEADER, pos)
            if start < 0:
                # 块末的 A5 可能和下块首字节 5A 组成帧头，单独留下它。
                end = len(data) - int(data[-1] == HEADER[0])
                skipped += end - pos
                pos = end
                break
            skipped += start - pos
            pos = start
            # 至少要收到两个帧头字节、长度和类型，才能查表判断整帧大小。
            if len(data) - pos < 4:
                break
            size, kind = data[pos + 2:pos + 4]
            if LENGTH.get(kind) != size:
                # 候选帧头可能出现在噪声或坏帧里。向前挪一个字节后重新搜索。
                pos += 1
                bad += 1
                continue
            if len(data) - pos < size + 2:
                break
            frame = data[pos:pos + size + 2]
            # 累加长度、类型和全部载荷字节，取低 8 位与倒数第二个字节比较。
            if frame[-1] != 0xaa or sum(frame[2:-2]) & 0xff != frame[-2]:
                pos += 1
                bad += 1
                continue
            print(json.dumps(decode(frame), separators=(",", ":")))
            valid += 1
            pos += len(frame)
        del data[:pos]
        if not chunk:
            break
    # 标准输出每行一条有效帧的 JSON。统计写入标准错误，可单独重定向。
    # truncated 为 1 表示末尾还有未收全的帧或帧头。读到至少一条有效帧即返回 0。
    print(f"valid={valid} bad={bad} skipped_bytes={skipped} truncated={int(bool(data))}", file=sys.stderr)
    return 0 if valid else 1


if __name__ == "__main__":
    # 输入可以是二进制抓包文件，也可以用 “-” 从标准输入读取。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=argparse.FileType("rb"), help="raw 115200 8N1 capture, or - for stdin")
    sys.exit(inspect(parser.parse_args().capture))
