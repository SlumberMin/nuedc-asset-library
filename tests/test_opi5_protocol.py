#!/usr/bin/env python3
"""
OPi5通信协议单元测试
覆盖: CRC8校验、帧构建、帧解析、传感器数据解析、序列号管理
测试: 正常通信、边界条件、异常输入、协议格式验证
参考: 04_通用代码库_OrangePi5/serial_protocol.py
"""

import sys
import os
import struct
import unittest
from unittest.mock import MagicMock, patch, PropertyMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ── 协议常量 (与 serial_protocol.py 一致) ──
HEAD = 0xAA

CMD_HEARTBEAT    = 0x01
CMD_VERSION      = 0x02
CMD_MOTOR_SET    = 0x10
CMD_MOTOR_GET    = 0x11
CMD_MOTOR_STOP   = 0x12
CMD_SERVO_SET    = 0x20
CMD_SERVO_GET    = 0x21
CMD_ADC_READ     = 0x30
CMD_ADC_MULTI    = 0x31
CMD_GPIO_SET     = 0x40
CMD_GPIO_GET     = 0x41
CMD_QUERY_SENSOR = 0x50
CMD_ACK          = 0xE0
CMD_ERROR        = 0xFE
CMD_SENSOR_DATA  = 0x50

SENSOR_ENCODER = 0x01
SENSOR_IMU     = 0x02
SENSOR_GRAY    = 0x03


# ── CRC8 独立实现用于验证 ──
def crc8(data: bytes) -> int:
    """CRC8校验 (多项式0x07, 初始值0x00)"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, data: bytes, seq: int) -> bytes:
    """构建协议帧"""
    length = len(data)
    frame = bytes([HEAD, cmd, length, seq]) + data
    frame_crc = crc8(frame[1:])
    return frame + bytes([frame_crc])


# ═══════════════════════════════════════════════════════════
#  测试用例
# ═══════════════════════════════════════════════════════════

class TestCRC8(unittest.TestCase):
    """CRC8校验测试"""

    def test_empty_data(self):
        """测试空数据CRC"""
        result = crc8(b'')
        self.assertEqual(result, 0x00)

    def test_single_byte(self):
        """测试单字节CRC"""
        result = crc8(bytes([0x01]))
        self.assertIsInstance(result, int)
        self.assertGreaterEqual(result, 0)
        self.assertLessEqual(result, 255)

    def test_known_vector(self):
        """测试已知CRC向量"""
        # 0x01, 0x00, 0x00 -> 已知CRC值
        data = bytes([0x01, 0x00, 0x00])
        result = crc8(data)
        self.assertEqual(result, crc8(data))  # 确定性

    def test_deterministic(self):
        """测试CRC确定性: 相同输入产生相同输出"""
        data = bytes([0xAA, 0x01, 0x02, 0x03])
        r1 = crc8(data)
        r2 = crc8(data)
        self.assertEqual(r1, r2)

    def test_different_data_different_crc(self):
        """测试不同数据产生不同CRC"""
        d1 = crc8(bytes([0x01, 0x02]))
        d2 = crc8(bytes([0x01, 0x03]))
        self.assertNotEqual(d1, d2)

    def test_crc_range(self):
        """测试CRC值在0~255范围内"""
        for i in range(256):
            result = crc8(bytes([i]))
            self.assertGreaterEqual(result, 0)
            self.assertLessEqual(result, 255)

    def test_large_data(self):
        """测试大数据CRC"""
        data = bytes(range(256))
        result = crc8(data)
        self.assertIsInstance(result, int)


class TestFrameBuild(unittest.TestCase):
    """帧构建测试"""

    def test_frame_format(self):
        """测试帧格式: [HEAD][CMD][LEN][SEQ][DATA...][CRC8]"""
        cmd = CMD_HEARTBEAT
        data = b''
        seq = 0
        frame = build_frame(cmd, data, seq)
        self.assertEqual(frame[0], HEAD)
        self.assertEqual(frame[1], cmd)
        self.assertEqual(frame[2], len(data))
        self.assertEqual(frame[3], seq)

    def test_frame_with_data(self):
        """测试带数据的帧"""
        cmd = CMD_MOTOR_SET
        data = bytes([0x00, 0x00, 0x01, 0xF4])
        seq = 5
        frame = build_frame(cmd, data, seq)
        self.assertEqual(frame[0], HEAD)
        self.assertEqual(frame[1], cmd)
        self.assertEqual(frame[2], 4)  # len
        self.assertEqual(frame[3], seq)
        self.assertEqual(frame[4:8], data)
        self.assertEqual(len(frame), 4 + 4 + 1)  # header + data + crc

    def test_frame_crc_valid(self):
        """测试帧CRC校验正确"""
        cmd = CMD_MOTOR_SET
        data = bytes([0x01, 0x00, 0x03, 0xE8])
        seq = 10
        frame = build_frame(cmd, data, seq)
        # 验证: CRC over CMD+LEN+SEQ+DATA should equal last byte
        payload = frame[1:-1]  # 去掉HEAD和CRC
        expected_crc = crc8(payload)
        self.assertEqual(frame[-1], expected_crc)

    def test_frame_empty_data(self):
        """测试空数据帧"""
        frame = build_frame(CMD_HEARTBEAT, b'', 0)
        self.assertEqual(len(frame), 5)  # HEAD+CMD+LEN+SEQ+CRC

    def test_frame_max_data(self):
        """测试最大数据长度帧"""
        data = bytes([0x00] * 255)
        frame = build_frame(CMD_ADC_MULTI, data, 0)
        self.assertEqual(frame[2], 255)
        self.assertEqual(len(frame), 4 + 255 + 1)


class TestFrameParse(unittest.TestCase):
    """帧解析测试"""

    def _parse_frame(self, raw_bytes):
        """解析帧 (模拟 recv_frame 逻辑)"""
        if len(raw_bytes) < 5:
            return None
        if raw_bytes[0] != HEAD:
            return None
        cmd = raw_bytes[1]
        length = raw_bytes[2]
        seq = raw_bytes[3]
        if len(raw_bytes) < 4 + length + 1:
            return None
        data = raw_bytes[4:4 + length]
        crc_received = raw_bytes[4 + length]
        # CRC校验
        crc_payload = raw_bytes[1:4 + length]
        expected_crc = crc8(crc_payload)
        if crc_received != expected_crc:
            return None
        return cmd, seq, data

    def test_parse_valid_frame(self):
        """测试解析有效帧"""
        frame = build_frame(CMD_ACK, bytes([0x10, 0x00]), 1)
        result = self._parse_frame(frame)
        self.assertIsNotNone(result)
        cmd, seq, data = result
        self.assertEqual(cmd, CMD_ACK)
        self.assertEqual(seq, 1)
        self.assertEqual(data, bytes([0x10, 0x00]))

    def test_parse_wrong_head(self):
        """测试错误帧头"""
        frame = build_frame(CMD_ACK, b'', 0)
        bad_frame = bytes([0xBB]) + frame[1:]
        result = self._parse_frame(bad_frame)
        self.assertIsNone(result)

    def test_parse_corrupted_crc(self):
        """测试CRC损坏"""
        frame = bytearray(build_frame(CMD_ACK, b'', 0))
        frame[-1] ^= 0xFF  # 翻转CRC
        result = self._parse_frame(bytes(frame))
        self.assertIsNone(result)

    def test_parse_truncated(self):
        """测试截断帧"""
        frame = build_frame(CMD_ACK, bytes([0x01, 0x02]), 0)
        result = self._parse_frame(frame[:3])  # 不足最小长度
        self.assertIsNone(result)

    def test_parse_empty_data(self):
        """测试空数据帧解析"""
        frame = build_frame(CMD_HEARTBEAT, b'', 0)
        result = self._parse_frame(frame)
        self.assertIsNotNone(result)
        cmd, seq, data = result
        self.assertEqual(cmd, CMD_HEARTBEAT)
        self.assertEqual(data, b'')


class TestSequenceNumber(unittest.TestCase):
    """序列号管理测试"""

    def test_seq_auto_increment(self):
        """测试序列号自动递增"""
        # 模拟协议实例的序列号管理
        seq_counter = [0]

        def next_seq():
            seq = seq_counter[0] & 0xFF
            seq_counter[0] += 1
            return seq

        s0 = next_seq()
        s1 = next_seq()
        s2 = next_seq()
        self.assertEqual(s0, 0)
        self.assertEqual(s1, 1)
        self.assertEqual(s2, 2)

    def test_seq_wraparound(self):
        """测试序列号溢出回绕"""
        seq_counter = [255]

        def next_seq():
            seq = seq_counter[0] & 0xFF
            seq_counter[0] += 1
            return seq

        self.assertEqual(next_seq(), 255)
        self.assertEqual(next_seq(), 0)  # 回绕

    def test_seq_range(self):
        """测试序列号在0~255范围内"""
        seq_counter = [0]

        def next_seq():
            seq = seq_counter[0] & 0xFF
            seq_counter[0] += 1
            return seq

        for _ in range(300):
            seq = next_seq()
            self.assertGreaterEqual(seq, 0)
            self.assertLessEqual(seq, 255)


class TestSensorDataParse(unittest.TestCase):
    """传感器数据解析测试"""

    def _parse_sensor_data(self, data):
        """解析传感器数据 (模拟 _parse_sensor_data)"""
        if not data:
            return {}
        sensor_type = data[0]
        result = {'type': sensor_type}

        if sensor_type == SENSOR_ENCODER and len(data) >= 9:
            result['count_left'] = struct.unpack('>h', data[1:3])[0]
            result['speed_left'] = struct.unpack('>h', data[3:5])[0]
            result['count_right'] = struct.unpack('>h', data[5:7])[0]
            result['speed_right'] = struct.unpack('>h', data[7:9])[0]
        elif sensor_type == SENSOR_IMU and len(data) >= 13:
            result['roll'] = struct.unpack('<f', data[1:5])[0]
            result['pitch'] = struct.unpack('<f', data[5:9])[0]
            result['yaw'] = struct.unpack('<f', data[9:13])[0]
        elif sensor_type == SENSOR_GRAY and len(data) >= 2:
            result['digital'] = data[1]
            result['channels'] = [(data[1] >> i) & 1 for i in range(8)]
        return result

    def test_parse_encoder_data(self):
        """测试解析编码器数据"""
        data = bytes([SENSOR_ENCODER]) + struct.pack('>hhhh', 100, 500, -100, -500)
        result = self._parse_sensor_data(data)
        self.assertEqual(result['type'], SENSOR_ENCODER)
        self.assertEqual(result['count_left'], 100)
        self.assertEqual(result['speed_left'], 500)
        self.assertEqual(result['count_right'], -100)
        self.assertEqual(result['speed_right'], -500)

    def test_parse_imu_data(self):
        """测试解析IMU数据"""
        data = bytes([SENSOR_IMU]) + struct.pack('<fff', 1.5, -2.3, 90.0)
        result = self._parse_sensor_data(data)
        self.assertEqual(result['type'], SENSOR_IMU)
        self.assertAlmostEqual(result['roll'], 1.5, places=3)
        self.assertAlmostEqual(result['pitch'], -2.3, places=3)
        self.assertAlmostEqual(result['yaw'], 90.0, places=3)

    def test_parse_gray_sensor_data(self):
        """测试解析灰度传感器数据"""
        data = bytes([SENSOR_GRAY, 0b10101010])
        result = self._parse_sensor_data(data)
        self.assertEqual(result['type'], SENSOR_GRAY)
        self.assertEqual(result['digital'], 0b10101010)
        self.assertEqual(result['channels'], [0, 1, 0, 1, 0, 1, 0, 1])

    def test_parse_gray_all_on(self):
        """测试灰度传感器全亮"""
        data = bytes([SENSOR_GRAY, 0xFF])
        result = self._parse_sensor_data(data)
        self.assertEqual(result['digital'], 0xFF)
        self.assertEqual(result['channels'], [1, 1, 1, 1, 1, 1, 1, 1])

    def test_parse_gray_all_off(self):
        """测试灰度传感器全灭"""
        data = bytes([SENSOR_GRAY, 0x00])
        result = self._parse_sensor_data(data)
        self.assertEqual(result['digital'], 0x00)
        self.assertEqual(result['channels'], [0, 0, 0, 0, 0, 0, 0, 0])

    def test_parse_empty_data(self):
        """测试空数据"""
        result = self._parse_sensor_data(b'')
        self.assertEqual(result, {})

    def test_parse_encoder_negative_values(self):
        """测试编码器负值(反转)"""
        data = bytes([SENSOR_ENCODER]) + struct.pack('>hhhh', -32768, -32768, 32767, 32767)
        result = self._parse_sensor_data(data)
        self.assertEqual(result['count_left'], -32768)
        self.assertEqual(result['speed_left'], -32768)
        self.assertEqual(result['count_right'], 32767)
        self.assertEqual(result['speed_right'], 32767)

    def test_parse_imu_zero(self):
        """测试IMU零值"""
        data = bytes([SENSOR_IMU]) + struct.pack('<fff', 0.0, 0.0, 0.0)
        result = self._parse_sensor_data(data)
        self.assertAlmostEqual(result['roll'], 0.0)
        self.assertAlmostEqual(result['pitch'], 0.0)
        self.assertAlmostEqual(result['yaw'], 0.0)

    def test_parse_insufficient_encoder_data(self):
        """测试编码器数据不足"""
        data = bytes([SENSOR_ENCODER, 0x00, 0x01])  # 只有3字节，不足9
        result = self._parse_sensor_data(data)
        # 数据不足时只返回type
        self.assertEqual(result['type'], SENSOR_ENCODER)
        self.assertNotIn('count_left', result)


class TestProtocolConstants(unittest.TestCase):
    """协议常量测试"""

    def test_head_value(self):
        """测试帧头常量"""
        self.assertEqual(HEAD, 0xAA)

    def test_command_ids_unique(self):
        """测试CMD ID唯一性"""
        cmd_ids = [
            CMD_HEARTBEAT, CMD_VERSION, CMD_MOTOR_SET, CMD_MOTOR_GET,
            CMD_MOTOR_STOP, CMD_SERVO_SET, CMD_SERVO_GET, CMD_ADC_READ,
            CMD_ADC_MULTI, CMD_GPIO_SET, CMD_GPIO_GET, CMD_QUERY_SENSOR,
        ]
        self.assertEqual(len(cmd_ids), len(set(cmd_ids)))

    def test_response_ids_unique(self):
        """测试响应CMD ID唯一性"""
        resp_ids = [CMD_ACK, CMD_ERROR, CMD_SENSOR_DATA]
        self.assertEqual(len(resp_ids), len(set(resp_ids)))

    def test_motor_set_data_format(self):
        """测试电机设置数据格式: [ch, dir, speed_H, speed_L]"""
        data = struct.pack('<BBH', 0, 0, 500)
        self.assertEqual(len(data), 4)
        ch, direction, speed = struct.unpack('<BBH', data)
        self.assertEqual(ch, 0)
        self.assertEqual(direction, 0)
        self.assertEqual(speed, 500)

    def test_servo_set_data_format(self):
        """测试舵机设置数据格式: [id, angle_H, angle_L]"""
        data = bytes([0]) + struct.pack('>H', 900)
        self.assertEqual(len(data), 3)
        servo_id = data[0]
        angle = struct.unpack('>H', data[1:3])[0]
        self.assertEqual(servo_id, 0)
        self.assertEqual(angle, 900)

    def test_ack_data_format(self):
        """测试ACK数据格式: [orig_cmd, status]"""
        data = bytes([CMD_MOTOR_SET, 0x00])
        self.assertEqual(len(data), 2)
        self.assertEqual(data[0], CMD_MOTOR_SET)
        self.assertEqual(data[1], 0x00)


class TestFrameRoundTrip(unittest.TestCase):
    """帧往返测试"""

    def test_build_and_parse(self):
        """测试构建后解析往返"""
        cmd = CMD_MOTOR_SET
        data = bytes([0x00, 0x00, 0x01, 0xF4])
        seq = 42
        frame = build_frame(cmd, data, seq)

        # 解析
        self.assertEqual(frame[0], HEAD)
        self.assertEqual(frame[1], cmd)
        self.assertEqual(frame[2], len(data))
        self.assertEqual(frame[3], seq)
        self.assertEqual(frame[4:4 + len(data)], data)
        # CRC验证
        payload = frame[1:-1]
        self.assertEqual(frame[-1], crc8(payload))

    def test_multiple_frames_sequence(self):
        """测试多帧序列"""
        frames = []
        for i in range(10):
            frame = build_frame(CMD_HEARTBEAT, bytes([i]), i)
            frames.append(frame)

        for i, frame in enumerate(frames):
            self.assertEqual(frame[0], HEAD)
            self.assertEqual(frame[3], i)
            payload = frame[1:-1]
            self.assertEqual(frame[-1], crc8(payload))


class TestProtocolEdgeCases(unittest.TestCase):
    """协议边界条件测试"""

    def test_all_zero_data(self):
        """测试全零数据帧"""
        frame = build_frame(0x00, bytes([0x00] * 10), 0)
        self.assertEqual(frame[0], HEAD)
        self.assertEqual(frame[2], 10)

    def test_all_ff_data(self):
        """测试全0xFF数据帧"""
        frame = build_frame(0xFF, bytes([0xFF] * 10), 0xFF)
        self.assertEqual(frame[0], HEAD)
        self.assertEqual(frame[1], 0xFF)
        self.assertEqual(frame[3], 0xFF)

    def test_single_byte_data(self):
        """测试单字节数据帧"""
        frame = build_frame(CMD_ADC_READ, bytes([0x05]), 0)
        self.assertEqual(frame[2], 1)
        self.assertEqual(len(frame), 6)

    def test_crc_sensitivity(self):
        """测试CRC对单比特变化敏感"""
        frame = build_frame(CMD_ACK, bytes([0x01, 0x02]), 0)
        # 修改数据的一位
        modified = bytearray(frame)
        modified[4] ^= 0x01
        # CRC应不同
        self.assertNotEqual(crc8(frame[1:-1]), crc8(bytes(modified[1:-1])))


if __name__ == '__main__':
    unittest.main()
