import pathlib
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
# 两条已知正确的旧协议帧作为测试基准。A3 的 0x2710 表示四元数的 w = 1。
# A1 的 0x8384 表示航向 -90.0 度，0x00c6 表示更新率 198 Hz。
LEVEL = bytes.fromhex("a55a 0c a3 2710 0000 0000 0000 e6 aa")
TILT = bytes.fromhex("a55a 12 a1 8384 0384 0384 a5a5 4801 0000 00c6 21 aa")


class CaptureTest(unittest.TestCase):
    def invoke(self, blob):
        # 把给定字节写入临时抓包文件，通过命令行运行解码器。
        # 返回退出码、标准输出和标准错误，测试可以分别核对解码值和错误统计。
        with tempfile.TemporaryDirectory() as t:
            capture = pathlib.Path(t) / "serial.bin"
            capture.write_bytes(blob)
            return subprocess.run([sys.executable, str(ROOT / "tools/decode_capture.py"), str(capture)],
                                  text=True, capture_output=True)

    def test_resync_and_signs(self):
        import json
        bad = bytearray(LEVEL)
        # 仅翻转校验和的一位，使这条帧的长度和帧尾正确，但校验失败。
        bad[-2] ^= 0x80
        # 输入按顺序包含伪帧头、坏帧、两条好帧和不完整的帧。
        # 解码器应跳过损坏部分，在下一个帧头恢复，并报告末尾截断。
        run = self.invoke(b'\xa5\x00' + bytes(bad) + TILT + LEVEL + LEVEL[:5])
        self.assertEqual(run.returncode, 0, run.stderr)
        frames = [json.loads(line) for line in run.stdout.splitlines()]
        self.assertEqual([f["type"] for f in frames], ["A1", "A3"])
        self.assertEqual(frames[0]["yaw_deg"], -90)
        self.assertEqual(frames[0]["update_hz"], 198)
        self.assertEqual(frames[1]["q_wxyz"], [1, 0, 0, 0])
        self.assertIn("bad=1", run.stderr)
        self.assertIn("truncated=1", run.stderr)

    def test_no_valid_frames(self):
        # 整个输入都没有有效帧时，命令行退出码应让调用者识别失败。
        self.assertNotEqual(self.invoke(b'not a binary IMU capture').returncode, 0)


if __name__ == "__main__":
    unittest.main()
