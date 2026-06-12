#!/usr/bin/env python3
"""
协议分析器 - 解析UART/SPI/I2C数据包
=============================================
功能：
  - 解析UART数据帧（自定义波特率、数据位、校验位）
  - 解析SPI数据流（CPOL/CPHA模式）
  - 解析I2C数据包（地址、读写、数据）
  - 数据包解码和字段提取
  - CRC校验（CRC8/CRC16/CRC32）
  - 协议统计和错误检测

用法：
  python protocol_analyzer.py uart --data "55 AA 01 03 00 01 XX" --baud 115200
  python protocol_analyzer.py i2c --data "D0 75 00" --addr 0x68
  python protocol_analyzer.py spi --mosi "FF 00 AB" --miso "00 3C 00"
  python protocol_analyzer.py decode --protocol modbus --data "01 03 00 00 00 0A"
"""

import argparse
import struct
import sys
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict


# ============================================================
# CRC 校验算法
# ============================================================

class CRC:
    """CRC校验计算工具"""

    @staticmethod
    def crc8(data: bytes, poly=0x07, init=0x00) -> int:
        """CRC-8 校验"""
        crc = init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ poly) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    @staticmethod
    def crc16_modbus(data: bytes) -> int:
        """CRC-16/MODBUS 校验"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def crc16_ccitt(data: bytes) -> int:
        """CRC-16/CCITT 校验"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def crc32(data: bytes) -> int:
        """CRC-32 校验"""
        crc = 0xFFFFFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xEDB88320
                else:
                    crc >>= 1
        return crc ^ 0xFFFFFFFF

    @staticmethod
    def xor_checksum(data: bytes) -> int:
        """异或校验和"""
        result = 0
        for byte in data:
            result ^= byte
        return result

    @staticmethod
    def sum_checksum(data: bytes) -> int:
        """累加和校验"""
        return sum(data) & 0xFF


# ============================================================
# UART 协议分析器
# ============================================================

@dataclass
class UARTFrame:
    """UART数据帧"""
    start_bit: int = 0
    data_bits: List[int] = None
    parity_bit: int = 0
    stop_bit: int = 1
    raw_byte: int = 0
    has_error: bool = False
    error_msg: str = ""


class UARTAnalyzer:
    """UART协议分析器"""

    # 常用波特率
    BAUD_RATES = [300, 1200, 2400, 4800, 9600, 19200, 38400,
                  57600, 115200, 230400, 460800, 921600, 1000000, 2000000]

    def __init__(self, baud_rate: int = 115200, data_bits: int = 8,
                 parity: str = 'none', stop_bits: float = 1.0):
        self.baud_rate = baud_rate
        self.data_bits = data_bits
        self.parity = parity  # none, even, odd, mark, space
        self.stop_bits = stop_bits

    @property
    def bit_time_us(self) -> float:
        """单比特时间(微秒)"""
        return 1_000_000 / self.baud_rate

    @property
    def frame_time_us(self) -> float:
        """单帧时间(微秒)"""
        total_bits = 1 + self.data_bits + (1 if self.parity != 'none' else 0) + self.stop_bits
        return total_bits * self.bit_time_us

    def parse_byte(self, byte_val: int) -> UARTFrame:
        """解析单个字节为UART帧"""
        frame = UARTFrame(raw_byte=byte_val)
        frame.start_bit = 0

        # 数据位（LSB先发）
        frame.data_bits = []
        for i in range(self.data_bits):
            frame.data_bits.append((byte_val >> i) & 1)

        # 计算校验位
        data_ones = sum(frame.data_bits)
        if self.parity == 'even':
            frame.parity_bit = 0 if data_ones % 2 == 0 else 1
        elif self.parity == 'odd':
            frame.parity_bit = 1 if data_ones % 2 == 0 else 0
        elif self.parity == 'mark':
            frame.parity_bit = 1
        elif self.parity == 'space':
            frame.parity_bit = 0
        else:
            frame.parity_bit = None

        frame.stop_bit = 1
        return frame

    def parse_hex_string(self, hex_str: str) -> List[dict]:
        """解析十六进制字符串为UART数据流"""
        hex_str = hex_str.replace("0x", "").replace(",", " ").replace("-", " ")
        parts = hex_str.split()
        bytes_data = [int(p, 16) for p in parts]

        results = []
        for i, byte_val in enumerate(bytes_data):
            frame = self.parse_byte(byte_val)

            # 检查数据有效性
            errors = []
            if byte_val < 0 or byte_val > 255:
                errors.append("字节值超出范围")

            bit_stream = []
            # 起始位
            bit_stream.append(("START", 0))
            # 数据位
            for j, bit in enumerate(frame.data_bits):
                bit_stream.append((f"D{j}", bit))
            # 校验位
            if frame.parity_bit is not None:
                bit_stream.append(("PARITY", frame.parity_bit))
            # 停止位
            bit_stream.append(("STOP", 1))

            results.append({
                "index": i,
                "hex": f"0x{byte_val:02X}",
                "dec": byte_val,
                "bin": f"{byte_val:08b}",
                "ascii": chr(byte_val) if 32 <= byte_val < 127 else '.',
                "bit_stream": bit_stream,
                "frame_time_us": round(self.frame_time_us, 2),
                "errors": errors,
            })
        return results

    def analyze_timing(self, hex_str: str) -> str:
        """分析时序特性"""
        n = len(hex_str.replace("0x", "").replace(",", " ").replace("-", " ").split())
        total_time = n * self.frame_time_us
        throughput = (n * self.data_bits) / (total_time / 1_000_000) if total_time > 0 else 0

        lines = [
            f"  波特率: {self.baud_rate} bps",
            f"  比特时间: {self.bit_time_us:.2f} μs",
            f"  帧时间: {self.frame_time_us:.2f} μs",
            f"  数据帧数: {n}",
            f"  总传输时间: {total_time/1000:.3f} ms",
            f"  有效数据速率: {throughput:.0f} bps",
        ]
        return "\n".join(lines)


# ============================================================
# I2C 协议分析器
# ============================================================

class I2CAnalyzer:
    """I2C协议分析器"""

    # 常用I2C设备地址映射
    KNOWN_DEVICES = {
        0x50: "EEPROM (AT24C系列)",
        0x68: "RTC (DS3231) / IMU (MPU6050)",
        0x76: "气压传感器 (BMP280)",
        0x77: "气压传感器 (BMP180/BME280)",
        0x23: "光照传感器 (BH1750)",
        0x48: "ADC (ADS1115/PCF8591)",
        0x27: "LCD (PCF8574)",
        0x3C: "OLED (SSD1306)",
        0x44: "温湿度传感器 (SHT30)",
        0x5A: "红外测温 (MLX90614)",
        0x60: "温湿度传感器 (SHT31)",
        0x62: "DAC (MCP4725)",
        0x69: "IMU (ICM-20948)",
    }

    def parse_transaction(self, data_str: str, address_hint: int = None) -> List[dict]:
        """解析I2C事务"""
        hex_str = data_str.replace("0x", "").replace(",", " ")
        parts = hex_str.split()
        bytes_data = [int(p, 16) for p in parts]

        if not bytes_data:
            return []

        results = []
        i = 0

        while i < len(bytes_data):
            byte = bytes_data[i]
            addr_7bit = byte >> 1
            rw = byte & 1  # 0=写, 1=读
            device = self.KNOWN_DEVICES.get(addr_7bit, "未知设备")

            entry = {
                "type": "ADDRESS",
                "raw": f"0x{byte:02X}",
                "address_7bit": f"0x{addr_7bit:02X}",
                "address_10bit": addr_7bit,  # 简化处理
                "read_write": "读(R)" if rw else "写(W)",
                "device": device,
                "ack": True,  # 假设ACK
            }
            results.append(entry)
            i += 1

            # 解析后续数据字节
            data_count = 0
            while i < len(bytes_data) and bytes_data[i] != bytes_data[0]:
                data_byte = bytes_data[i]
                data_entry = {
                    "type": "DATA",
                    "raw": f"0x{data_byte:02X}",
                    "dec": data_byte,
                    "bin": f"{data_byte:08b}",
                    "ascii": chr(data_byte) if 32 <= data_byte < 127 else '.',
                    "reg_name": self._guess_register(addr_7bit, data_count, rw),
                    "ack": True,
                }
                results.append(data_entry)
                data_count += 1
                i += 1

        return results

    def _guess_register(self, device_addr: int, byte_index: int, rw: int) -> str:
        """猜测寄存器名称"""
        # MPU6050 寄存器
        if device_addr == 0x68:
            mpu_regs = {
                0x75: "WHO_AM_I",
                0x6B: "PWR_MGMT_1",
                0x1B: "GYRO_CONFIG",
                0x1C: "ACCEL_CONFIG",
                0x3B: "ACCEL_XOUT_H",
                0x41: "TEMP_OUT_H",
            }
            if byte_index == 0 and rw == 0:
                return "寄存器地址"
            return mpu_regs.get(byte_index, f"数据[{byte_index}]")
        return f"字节[{byte_index}]"

    def format_i2c_speed(self, speed: str) -> dict:
        """获取I2C速度规格"""
        speeds = {
            "standard": {"name": "标准模式", "rate": 100_000, "min_pulse": 4.0},
            "fast": {"name": "快速模式", "rate": 400_000, "min_pulse": 0.6},
            "fast_plus": {"name": "快速模式+", "rate": 1_000_000, "min_pulse": 0.26},
            "high": {"name": "高速模式", "rate": 3_400_000, "min_pulse": 0.06},
        }
        return speeds.get(speed, speeds["standard"])


# ============================================================
# SPI 协议分析器
# ============================================================

class SPIAnalyzer:
    """SPI协议分析器"""

    def __init__(self, mode: int = 0, bits: int = 8, msb_first: bool = True):
        """
        SPI模式:
        mode 0: CPOL=0, CPHA=0 (空闲低电平, 第一个边沿采样)
        mode 1: CPOL=0, CPHA=1 (空闲低电平, 第二个边沿采样)
        mode 2: CPOL=1, CPHA=0 (空闲高电平, 第一个边沿采样)
        mode 3: CPOL=1, CPHA=1 (空闲高电平, 第二个边沿采样)
        """
        self.mode = mode
        self.cpol = (mode >> 1) & 1
        self.cpha = mode & 1
        self.bits = bits
        self.msb_first = msb_first

    def parse_transaction(self, mosi_str: str, miso_str: str = None) -> List[dict]:
        """解析SPI事务"""
        mosi_hex = mosi_str.replace("0x", "").replace(",", " ")
        mosi_bytes = [int(p, 16) for p in mosi_hex.split()]

        if miso_str:
            miso_hex = miso_str.replace("0x", "").replace(",", " ")
            miso_bytes = [int(p, 16) for p in miso_hex.split()]
        else:
            miso_bytes = [0x00] * len(mosi_bytes)

        # 对齐长度
        max_len = max(len(mosi_bytes), len(miso_bytes))
        mosi_bytes.extend([0x00] * (max_len - len(mosi_bytes)))
        miso_bytes.extend([0x00] * (max_len - len(miso_bytes)))

        results = []
        for i in range(max_len):
            mosi_val = mosi_bytes[i]
            miso_val = miso_bytes[i]

            entry = {
                "index": i,
                "mosi_hex": f"0x{mosi_val:02X}",
                "mosi_bin": f"{mosi_val:08b}",
                "miso_hex": f"0x{miso_val:02X}",
                "miso_bin": f"{miso_val:08b}",
                "mosi_ascii": chr(mosi_val) if 32 <= mosi_val < 127 else '.',
                "miso_ascii": chr(miso_val) if 32 <= miso_val < 127 else '.',
                "comment": self._guess_spi_command(i, mosi_val),
            }
            results.append(entry)
        return results

    def _guess_spi_command(self, index: int, value: int) -> str:
        """猜测SPI命令含义"""
        if index == 0:
            # 常见SPI命令
            commands = {
                0x9F: "读JEDEC ID",
                0x06: "写使能(WREN)",
                0x04: "写禁止(WRDI)",
                0x05: "读状态寄存器(RDSR)",
                0x01: "写状态寄存器(WRSR)",
                0x03: "读数据(READ)",
                0x02: "写数据(PROGRAM)",
                0x20: "扇区擦除(4KB)",
                0x52: "块擦除(32KB)",
                0xD8: "块擦除(64KB)",
                0xC7: "芯片擦除",
                0x00: "NOP",
                0xFF: "虚拟字节",
            }
            return commands.get(value, f"命令: 0x{value:02X}")
        return f"数据字节[{index}]"

    def get_mode_info(self) -> str:
        """获取SPI模式信息"""
        return (f"  SPI Mode {self.mode}: CPOL={self.cpol}, CPHA={self.cpha}\n"
                f"  空闲电平: {'高' if self.cpol else '低'}\n"
                f"  采样边沿: {'第二个' if self.cpha else '第一个'}\n"
                f"  数据位序: {'MSB先发' if self.msb_first else 'LSB先发'}\n"
                f"  数据宽度: {self.bits} bit")


# ============================================================
# 通用协议解码器
# ============================================================

class ProtocolDecoder:
    """通用协议帧解码器"""

    @staticmethod
    def decode_modbus_rtu(data_str: str) -> dict:
        """解码Modbus RTU帧"""
        hex_str = data_str.replace("0x", "").replace(",", " ")
        bytes_data = bytes([int(p, 16) for p in hex_str.split()])

        if len(bytes_data) < 4:
            return {"error": "数据太短，至少需要4字节"}

        slave_addr = bytes_data[0]
        func_code = bytes_data[1]

        functions = {
            0x01: "读线圈状态",
            0x02: "读输入状态",
            0x03: "读保持寄存器",
            0x04: "读输入寄存器",
            0x05: "写单个线圈",
            0x06: "写单个寄存器",
            0x0F: "写多个线圈",
            0x10: "写多个寄存器",
        }

        result = {
            "slave_address": f"0x{slave_addr:02X} ({slave_addr})",
            "function_code": f"0x{func_code:02X} - {functions.get(func_code, '未知功能')}",
            "data": [f"0x{b:02X}" for b in bytes_data[2:-2]],
        }

        # CRC校验
        if len(bytes_data) >= 3:
            data_part = bytes_data[:-2]
            received_crc = bytes_data[-2] | (bytes_data[-1] << 8)
            calc_crc = CRC.crc16_modbus(data_part)
            result["crc_received"] = f"0x{received_crc:04X}"
            result["crc_calculated"] = f"0x{calc_crc:04X}"
            result["crc_valid"] = received_crc == calc_crc

        # 解析读请求
        if func_code in (0x01, 0x02, 0x03, 0x04) and len(bytes_data) >= 8:
            start_addr = (bytes_data[2] << 8) | bytes_data[3]
            quantity = (bytes_data[4] << 8) | bytes_data[5]
            result["start_address"] = f"0x{start_addr:04X} ({start_addr})"
            result["quantity"] = quantity

        return result

    @staticmethod
    def decode_custom_frame(data_str: str, header: str = "55 AA",
                             length_offset: int = 2, length_size: int = 1) -> dict:
        """解码自定义协议帧"""
        hex_str = data_str.replace("0x", "").replace(",", " ")
        bytes_data = [int(p, 16) for p in hex_str.split()]

        header_bytes = [int(h, 16) for h in header.split()]

        result = {"raw_hex": " ".join(f"{b:02X}" for b in bytes_data)}

        # 帧头检查
        if bytes_data[:len(header_bytes)] != header_bytes:
            result["error"] = f"帧头不匹配，期望 {' '.join(f'{h:02X}' for h in header_bytes)}"
            return result
        result["header"] = "OK"

        # 长度字段
        if length_size == 1:
            payload_len = bytes_data[length_offset]
        elif length_size == 2:
            payload_len = (bytes_data[length_offset] << 8) | bytes_data[length_offset + 1]
        else:
            payload_len = len(bytes_data) - len(header_bytes) - length_size - 1

        result["payload_length"] = payload_len

        # 提取负载
        data_start = len(header_bytes) + length_size
        payload = bytes_data[data_start:data_start + payload_len]
        result["payload"] = " ".join(f"{b:02X}" for b in payload)

        # 校验和
        if data_start + payload_len < len(bytes_data):
            checksum_byte = bytes_data[data_start + payload_len]
            calc_sum = sum(bytes_data[:data_start + payload_len]) & 0xFF
            result["checksum"] = f"0x{checksum_byte:02X}"
            result["checksum_calc"] = f"0x{calc_sum:02X}"
            result["checksum_valid"] = checksum_byte == calc_sum

        return result


# ============================================================
# CLI 接口
# ============================================================

def cmd_uart(args):
    """UART分析命令"""
    analyzer = UARTAnalyzer(
        baud_rate=args.baud,
        data_bits=args.databits,
        parity=args.parity,
        stop_bits=args.stopbits,
    )

    print(f"\n  UART 协议分析")
    print(f"  {'=' * 50}")
    print(f"  配置: {args.baud} bps, {args.databits}{args.parity[0].upper()}{args.stopbits}")
    print()

    results = analyzer.parse_hex_string(args.data)

    print(f"  {'位置':>4s} {'HEX':>6s} {'DEC':>5s} {'BIN':>10s} {'ASCII':>5s} {'帧时间':>10s}")
    print(f"  {'-' * 50}")
    for r in results:
        print(f"  {r['index']:4d} {r['hex']:>6s} {r['dec']:5d} {r['bin']:>10s} "
              f"  {r['ascii']:>3s}  {r['frame_time_us']:>8.1f}μs")
        if r['errors']:
            for e in r['errors']:
                print(f"       ⚠ {e}")

    print()
    print(analyzer.analyze_timing(args.data))

    # 解码为ASCII
    ascii_text = "".join(r['ascii'] for r in results)
    printable = ascii_text.replace('.', '')
    if len(printable) > 0:
        print(f"\n  ASCII解码: '{ascii_text}'")


def cmd_i2c(args):
    """I2C分析命令"""
    analyzer = I2CAnalyzer()

    print(f"\n  I2C 协议分析")
    print(f"  {'=' * 50}")

    results = analyzer.parse_transaction(args.data, args.addr)

    print(f"\n  {'类型':>8s} {'HEX':>6s} {'地址':>8s} {'读/写':>6s} {'设备/数据':>25s} {'ACK':>4s}")
    print(f"  {'-' * 60}")

    for r in results:
        if r['type'] == 'ADDRESS':
            print(f"  {'地址':>8s} {r['raw']:>6s} {r['address_7bit']:>8s} "
                  f"{r['read_write']:>6s} {r['device']:>25s} {'✓':>4s}")
        else:
            print(f"  {'数据':>8s} {r['raw']:>6s} {'':>8s} {'':>6s} "
                  f"{r['reg_name']:>25s} {'✓':>4s}")

    print(f"\n  已知I2C设备:")
    for addr, name in sorted(I2CAnalyzer.KNOWN_DEVICES.items()):
        print(f"    0x{addr:02X} (7-bit) / 0x{addr*2:02X} (写) / 0x{addr*2+1:02X} (读) - {name}")


def cmd_spi(args):
    """SPI分析命令"""
    analyzer = SPIAnalyzer(mode=args.mode, bits=args.bits)

    print(f"\n  SPI 协议分析")
    print(f"  {'=' * 50}")
    print(analyzer.get_mode_info())

    results = analyzer.parse_transaction(args.mosi, args.miso)

    print(f"\n  {'位置':>4s} {'MOSI':>8s} {'MISO':>8s} {'注释':>25s}")
    print(f"  {'-' * 50}")
    for r in results:
        print(f"  {r['index']:4d} {r['mosi_hex']:>8s} {r['miso_hex']:>8s} {r['comment']:>25s}")


def cmd_decode(args):
    """协议解码命令"""
    if args.protocol == "modbus":
        result = ProtocolDecoder.decode_modbus_rtu(args.data)
        print(f"\n  Modbus RTU 解码")
        print(f"  {'=' * 40}")
        for k, v in result.items():
            print(f"  {k}: {v}")

    elif args.protocol == "custom":
        result = ProtocolDecoder.decode_custom_frame(
            args.data, header=args.header or "55 AA")
        print(f"\n  自定义协议解码")
        print(f"  {'=' * 40}")
        for k, v in result.items():
            print(f"  {k}: {v}")

    else:
        # 通用解析
        hex_str = args.data.replace("0x", "").replace(",", " ")
        bytes_data = [int(p, 16) for p in hex_str.split()]
        print(f"\n  通用数据解析")
        print(f"  {'=' * 40}")
        print(f"  原始HEX: {' '.join(f'{b:02X}' for b in bytes_data)}")
        print(f"  字节数: {len(bytes_data)}")
        print(f"  CRC-8: 0x{CRC.crc8(bytes(bytes_data)):02X}")
        print(f"  CRC-16/Modbus: 0x{CRC.crc16_modbus(bytes(bytes_data)):04X}")
        print(f"  CRC-16/CCITT: 0x{CRC.crc16_ccitt(bytes(bytes_data)):04X}")
        print(f"  CRC-32: 0x{CRC.crc32(bytes(bytes_data)):08X}")
        print(f"  XOR校验: 0x{CRC.xor_checksum(bytes(bytes_data)):02X}")
        print(f"  累加校验: 0x{CRC.sum_checksum(bytes(bytes_data)):02X}")

        # 尝试ASCII解码
        ascii_text = "".join(chr(b) if 32 <= b < 127 else '.' for b in bytes_data)
        print(f"  ASCII: {ascii_text}")


def main():
    parser = argparse.ArgumentParser(
        description="协议分析器 - 解析UART/SPI/I2C数据包",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    subparsers = parser.add_subparsers(dest="command", help="协议类型")

    # UART子命令
    uart_parser = subparsers.add_parser("uart", help="UART协议分析")
    uart_parser.add_argument("--data", "-d", required=True, help="十六进制数据 (如: '55 AA 01 03')")
    uart_parser.add_argument("--baud", "-b", type=int, default=115200, help="波特率 (默认: 115200)")
    uart_parser.add_argument("--databits", type=int, default=8, choices=[5, 6, 7, 8], help="数据位")
    uart_parser.add_argument("--parity", default="none", choices=["none", "even", "odd", "mark", "space"])
    uart_parser.add_argument("--stopbits", type=float, default=1.0, choices=[1.0, 1.5, 2.0])

    # I2C子命令
    i2c_parser = subparsers.add_parser("i2c", help="I2C协议分析")
    i2c_parser.add_argument("--data", "-d", required=True, help="I2C数据字节")
    i2c_parser.add_argument("--addr", "-a", type=lambda x: int(x, 0), default=None, help="设备地址")

    # SPI子命令
    spi_parser = subparsers.add_parser("spi", help="SPI协议分析")
    spi_parser.add_argument("--mosi", required=True, help="MOSI数据")
    spi_parser.add_argument("--miso", default=None, help="MISO数据")
    spi_parser.add_argument("--mode", type=int, default=0, choices=[0, 1, 2, 3], help="SPI模式")
    spi_parser.add_argument("--bits", type=int, default=8, help="数据位宽")

    # 通用解码子命令
    decode_parser = subparsers.add_parser("decode", help="通用协议解码")
    decode_parser.add_argument("--protocol", "-p", default="auto",
                               choices=["modbus", "custom", "auto"], help="协议类型")
    decode_parser.add_argument("--data", "-d", required=True, help="十六进制数据")
    decode_parser.add_argument("--header", default=None, help="自定义帧头 (如: '55 AA')")

    args = parser.parse_args()

    if args.command == "uart":
        cmd_uart(args)
    elif args.command == "i2c":
        cmd_i2c(args)
    elif args.command == "spi":
        cmd_spi(args)
    elif args.command == "decode":
        cmd_decode(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
