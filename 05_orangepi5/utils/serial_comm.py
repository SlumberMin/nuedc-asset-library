"""
串口通信模块 — 与 STM32 通信协议
适用于 Orange Pi 5 (RK3588S) UART, 支持自定义帧协议
"""
import struct
import time
import threading
import logging
from typing import Callable, Optional
from collections import deque

logger = logging.getLogger(__name__)


class FrameProtocol:
    """
    串口帧协议

    帧格式:
    | HEAD(1B) | LEN(1B) | CMD(1B) | DATA(NB) | CRC16(2B) | TAIL(1B) |

    HEAD: 0xAA
    TAIL: 0x55
    LEN:  CMD + DATA 长度
    CRC16: CRC-16/MODBUS

    可自定义帧头帧尾和 CRC 算法。
    """

    HEAD = 0xAA
    TAIL = 0x55

    def __init__(self, head: int = 0xAA, tail: int = 0x55):
        self.HEAD = head
        self.TAIL = tail

    @staticmethod
    def crc16_modbus(data: bytes) -> int:
        """CRC-16/MODBUS 计算"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def pack(self, cmd: int, data: bytes = b'') -> bytes:
        """
        打包一帧数据

        Parameters
        ----------
        cmd : int
            命令字 (1 字节)
        data : bytes
            数据负载

        Returns
        -------
        bytes : 完整帧
        """
        length = 1 + len(data)  # CMD + DATA
        payload = bytes([length, cmd]) + data
        crc = self.crc16_modbus(payload)
        frame = bytes([self.HEAD]) + payload + struct.pack('<H', crc) + bytes([self.TAIL])
        return frame

    def unpack(self, buffer: bytes) -> list:
        """
        从缓冲区解析帧

        Parameters
        ----------
        buffer : bytes
            接收缓冲区

        Returns
        -------
        list of (cmd, data, consumed_bytes)
            解析出的帧列表
        """
        frames = []
        i = 0
        while i < len(buffer):
            # 寻找帧头
            if buffer[i] != self.HEAD:
                i += 1
                continue

            if i + 2 >= len(buffer):
                break  # 数据不足

            length = buffer[i + 1]
            frame_len = 1 + 1 + length + 2 + 1  # HEAD + LEN + PAYLOAD + CRC + TAIL

            if i + frame_len > len(buffer):
                break  # 数据不足

            # 检查帧尾
            if buffer[i + frame_len - 1] != self.TAIL:
                i += 1
                continue

            # CRC 校验
            payload = buffer[i + 1:i + 1 + length + 1]  # LEN + CMD + DATA
            received_crc = struct.unpack('<H', buffer[i + 1 + length + 1:i + 1 + length + 3])[0]
            calculated_crc = self.crc16_modbus(payload)

            if received_crc != calculated_crc:
                logger.warning(f"CRC 校验失败: recv={received_crc:#06x}, calc={calculated_crc:#06x}")
                i += 1
                continue

            cmd = buffer[i + 2]
            data = buffer[i + 3:i + 1 + length]
            frames.append((cmd, data))
            i += frame_len

        return frames, i  # 返回解析的帧和消费的字节数


class SerialProtocol:
    """
    串口通信协议封装

    Parameters
    ----------
    port : str
        串口设备路径 (如 '/dev/ttyS1', '/dev/ttyUSB0')
    baudrate : int
        波特率
    timeout : float
        读超时 (秒)
    frame_protocol : FrameProtocol, optional
        帧协议实例
    """

    def __init__(
        self,
        port: str = '/dev/ttyS1',
        baudrate: int = 115200,
        timeout: float = 0.1,
        frame_protocol: FrameProtocol = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.protocol = frame_protocol or FrameProtocol()

        self._serial = None
        self._buffer = bytearray()
        self._lock = threading.Lock()
        self._running = False
        self._recv_thread = None
        self._handlers = {}  # cmd -> callback
        self._recv_queue = deque(maxlen=100)

    def open(self):
        """打开串口"""
        import serial
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        logger.info(f"串口已打开: {self.port} @ {self.baudrate}")

    def close(self):
        """关闭串口"""
        self.stop_receiving()
        if self._serial and self._serial.is_open:
            self._serial.close()
            logger.info("串口已关闭")

    def send(self, cmd: int, data: bytes = b''):
        """
        发送一帧数据

        Parameters
        ----------
        cmd : int
            命令字
        data : bytes
            数据负载
        """
        frame = self.protocol.pack(cmd, data)
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.write(frame)
                logger.debug(f"发送: {frame.hex()}")

    def send_raw(self, data: bytes):
        """发送原始数据 (不封装帧)"""
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.write(data)

    def receive(self, timeout: float = None) -> list:
        """
        同步接收并解析帧

        Returns
        -------
        list of (cmd, data)
        """
        if not self._serial or not self._serial.is_open:
            return []

        remaining = self._serial.in_waiting
        if remaining > 0:
            self._buffer.extend(self._serial.read(remaining))

        frames, consumed = self.protocol.unpack(bytes(self._buffer))
        if consumed > 0:
            self._buffer = self._buffer[consumed:]

        return frames

    def register_handler(self, cmd: int, callback: Callable):
        """
        注册命令回调

        Parameters
        ----------
        cmd : int
            命令字
        callback : Callable(cmd, data)
            回调函数
        """
        self._handlers[cmd] = callback

    def start_receiving(self):
        """启动后台接收线程"""
        if self._running:
            return
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        logger.info("串口后台接收已启动")

    def stop_receiving(self):
        """停止后台接收"""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)
            self._recv_thread = None

    def _recv_loop(self):
        """后台接收循环"""
        while self._running:
            try:
                frames = self.receive()
                for cmd, data in frames:
                    self._recv_queue.append((cmd, data, time.monotonic()))
                    if cmd in self._handlers:
                        try:
                            self._handlers[cmd](cmd, data)
                        except Exception as e:
                            logger.error(f"命令回调异常: {e}")
            except Exception as e:
                if self._running:
                    logger.error(f"串口接收异常: {e}")
                    time.sleep(0.1)

    def get_received(self) -> tuple:
        """获取接收到的帧 (非阻塞)"""
        if self._recv_queue:
            return self._recv_queue.popleft()
        return None

    def wait_for_cmd(self, cmd: int, timeout: float = 2.0) -> bytes:
        """
        等待指定命令

        Parameters
        ----------
        cmd : int
            期望的命令字
        timeout : float
            超时 (秒)

        Returns
        -------
        bytes : 数据负载
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for item in list(self._recv_queue):
                if item[0] == cmd:
                    self._recv_queue.remove(item)
                    return item[1]
            time.sleep(0.01)
        raise TimeoutError(f"等待命令 {cmd:#04x} 超时")

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()

    def __repr__(self):
        return f"SerialProtocol(port={self.port}, baudrate={self.baudrate})"
