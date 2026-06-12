"""
serial_protocol.py — OrangePi5 <-> MSPM0G3507 UART Communication Protocol

Frame format:
    [HEAD 0xAA][CMD_ID][LEN][SEQ][DATA(0~255B)][CRC8]

CRC8: polynomial 0x07, init 0x00 (over CMD+LEN+SEQ+DATA)

CMD_ID (OPi5 -> MSPM0):
    0x01  HEARTBEAT       Heartbeat / online detection
    0x02  VERSION         Query firmware version
    0x10  MOTOR_SET       Motor speed: [ch, dir, speed_H, speed_L]
    0x11  MOTOR_GET       Read motor status: [ch]
    0x12  MOTOR_STOP      Emergency stop: [ch] (0xFF=all)
    0x20  SERVO_SET       Servo angle: [id, angle_H, angle_L] (0.1 deg)
    0x21  SERVO_GET       Read servo angle: [id]
    0x30  ADC_READ        Single ADC: [ch]
    0x31  ADC_MULTI       Multi ADC: [N, ch0, ch1, ...]
    0x40  GPIO_SET        GPIO output: [pin, val]
    0x41  GPIO_GET        GPIO input: [pin]
    0x50  QUERY_SENSOR    Query sensor: [sensor_id]

Response (MSPM0 -> OPi5):
    0xE0  ACK             Generic ACK: [orig_cmd, status]
    0xFE  ERROR           Error report: [err_code, info_H, info_L]
    0x50  SENSOR_DATA     Sensor data: [type, data...]

Usage:
    from serial_protocol import MSPM0Protocol
    dev = MSPM0Protocol('/dev/ttyS3')
    dev.motor_set(0, 0, 500)       # ch0, forward, speed 500
    dev.servo_set(0, 900)          # servo0, 90.0 degrees
    enc = dev.query_sensor(0x01)   # query encoder
    dev.close()
"""

import struct
import time
import threading
import logging

try:
    import serial
except ImportError:
    serial = None

logger = logging.getLogger(__name__)


class MSPM0Protocol:
    """OPi5 <-> MSPM0G3507 UART Communication Protocol (Python side)."""

    HEAD = 0xAA

    # ── CMD_ID (OPi5 -> MSPM0) ─────────────────────────────
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

    # ── Response CMD_ID (MSPM0 -> OPi5) ────────────────────
    CMD_ACK          = 0xE0
    CMD_ERROR        = 0xFE
    CMD_SENSOR_DATA  = 0x50

    # ── Error Codes ─────────────────────────────────────────
    ERR_CRC    = 0x01
    ERR_CMD    = 0x02
    ERR_PARAM  = 0x03
    ERR_BUSY   = 0x04
    ERR_TIMEOUT = 0x05
    ERR_RANGE  = 0x06
    ERR_STATE  = 0x07

    # ── Sensor IDs ──────────────────────────────────────────
    SENSOR_ENCODER = 0x01
    SENSOR_IMU     = 0x02
    SENSOR_GRAY    = 0x03
    SENSOR_ALL     = 0xFF

    # ── Timeout Parameters ──────────────────────────────────
    ACK_TIMEOUT_S    = 0.1
    RETRY_COUNT      = 1
    RETRY_INTERVAL_S = 0.01
    HEARTBEAT_S      = 1.0
    OFFLINE_TIMEOUT_S = 3.0

    def __init__(self, port='/dev/ttyS3', baudrate=115200, timeout=0.1):
        """Initialize protocol driver.

        Args:
            port: Serial port path (e.g. '/dev/ttyS3' or 'COM3')
            baudrate: UART baud rate (default 115200)
            timeout: Serial read timeout in seconds
        """
        if serial is None:
            raise ImportError("pyserial is required: pip install pyserial")

        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.seq_counter = 0
        self._lock = threading.Lock()
        self.connected = False
        self._last_rx_time = 0.0
        self._heartbeat_thread = None
        self._heartbeat_running = False

        logger.info(f"Protocol initialized: {port} @ {baudrate}")

    @staticmethod
    def crc8(data: bytes) -> int:
        """CRC8 calculation (polynomial 0x07, init 0x00).

        Args:
            data: Input bytes

        Returns:
            CRC8 value
        """
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def _next_seq(self) -> int:
        """Get next sequence number (0~255, auto-wrapping)."""
        seq = self.seq_counter & 0xFF
        self.seq_counter += 1
        return seq

    def send_frame(self, cmd: int, data: bytes = b'', seq: int = None) -> int:
        """Send a protocol frame.

        Frame: [0xAA][CMD][LEN][SEQ][DATA...][CRC8]

        Args:
            cmd: Command ID
            data: Payload bytes (max 255)
            seq: Sequence number (auto-assigned if None)

        Returns:
            Sequence number used
        """
        if seq is None:
            seq = self._next_seq()

        length = len(data)
        if length > 255:
            raise ValueError(f"Data too long: {length} > 255")

        # Build frame: HEAD + CMD + LEN + SEQ + DATA
        frame = bytes([self.HEAD, cmd, length, seq]) + data

        # CRC8 over CMD + LEN + SEQ + DATA
        crc = self.crc8(frame[1:])
        frame += bytes([crc])

        with self._lock:
            self.ser.write(frame)
            logger.debug(f"TX: {frame.hex(' ')}")

        return seq

    def recv_frame(self, timeout: float = None) -> tuple:
        """Receive one protocol frame.

        Args:
            timeout: Read timeout in seconds (None = use serial timeout)

        Returns:
            (cmd, seq, data) tuple, or None on timeout/error
        """
        if timeout is None:
            timeout = self.ser.timeout or 0.1

        deadline = time.time() + timeout

        while time.time() < deadline:
            # Sync to frame header
            head = self.ser.read(1)
            if not head or head[0] != self.HEAD:
                continue

            # Read CMD, LEN, SEQ (3 bytes)
            meta = self.ser.read(3)
            if len(meta) < 3:
                continue

            cmd, length, seq = meta[0], meta[1], meta[2]

            # Read DATA
            data = self.ser.read(length) if length > 0 else b''
            if len(data) < length:
                continue

            # Read CRC8
            crc_byte = self.ser.read(1)
            if not crc_byte:
                continue

            # Verify CRC8 over CMD + LEN + SEQ + DATA
            crc_payload = bytes([cmd, length, seq]) + data
            expected_crc = self.crc8(crc_payload)
            if crc_byte[0] != expected_crc:
                logger.warning(f"CRC mismatch: got 0x{crc_byte[0]:02X}, "
                               f"expected 0x{expected_crc:02X}")
                continue

            self._last_rx_time = time.time()
            self.connected = True

            logger.debug(f"RX: CMD=0x{cmd:02X} SEQ={seq} LEN={length} "
                         f"DATA={data.hex(' ') if data else '(empty)'}")

            return cmd, seq, data

        return None

    def send_and_wait(self, cmd: int, data: bytes = b'',
                      timeout: float = None) -> tuple:
        """Send a command and wait for response with retry.

        Args:
            cmd: Command ID
            data: Payload bytes
            timeout: Response timeout

        Returns:
            (cmd, seq, data) response, or None on failure
        """
        if timeout is None:
            timeout = self.ACK_TIMEOUT_S

        for attempt in range(self.RETRY_COUNT + 1):
            seq = self.send_frame(cmd, data)

            resp = self.recv_frame(timeout)
            if resp is not None:
                return resp

            if attempt < self.RETRY_COUNT:
                logger.debug(f"Retry {attempt + 1}/{self.RETRY_COUNT}")
                time.sleep(self.RETRY_INTERVAL_S)

        logger.warning(f"Command 0x{cmd:02X} timed out after "
                       f"{self.RETRY_COUNT + 1} attempts")
        return None

    # ── High-Level API ──────────────────────────────────────

    def heartbeat(self) -> bool:
        """Send heartbeat and check response.

        Returns:
            True if heartbeat acknowledged
        """
        resp = self.send_and_wait(self.CMD_HEARTBEAT)
        return resp is not None

    def get_version(self) -> tuple:
        """Query firmware version.

        Returns:
            (major, minor, year, month, day) or None
        """
        resp = self.send_and_wait(self.CMD_VERSION)
        if resp and resp[0] == self.CMD_VERSION and resp[2] and len(resp[2]) >= 5:
            d = resp[2]
            return d[0], d[1], 2000 + d[2], d[3], d[4]
        return None

    def motor_set(self, channel: int, direction: int, speed: int) -> bool:
        """Set motor speed and direction.

        Args:
            channel: Motor channel (0 or 1)
            direction: 0=forward, 1=reverse, 2=brake
            speed: Speed value 0~1000 (PWM duty cycle)

        Returns:
            True if acknowledged successfully
        """
        data = struct.pack('<BBH', channel, direction, speed)
        resp = self.send_and_wait(self.CMD_MOTOR_SET, data)
        if resp and resp[0] == self.CMD_ACK:
            return resp[2] and resp[2][1] == 0x00
        return False

    def motor_get(self, channel: int) -> dict:
        """Read motor status.

        Args:
            channel: Motor channel (0 or 1)

        Returns:
            dict with 'direction', 'speed', 'count' keys, or None
        """
        resp = self.send_and_wait(self.CMD_MOTOR_GET, bytes([channel]))
        if resp and resp[0] == self.CMD_MOTOR_GET and resp[2] and len(resp[2]) >= 6:
            d = resp[2]
            return {
                'channel': d[0],
                'count': struct.unpack('>h', d[1:3])[0],
                'speed': struct.unpack('>h', d[3:5])[0],
                'direction': d[5],
            }
        return None

    def motor_stop(self, channel: int = 0xFF) -> bool:
        """Emergency stop motor(s).

        Args:
            channel: Motor channel (0/1), or 0xFF for all

        Returns:
            True if acknowledged
        """
        resp = self.send_and_wait(self.CMD_MOTOR_STOP, bytes([channel]))
        if resp and resp[0] == self.CMD_ACK:
            return resp[2] and resp[2][1] == 0x00
        return False

    def servo_set(self, servo_id: int, angle_x10: int) -> bool:
        """Set servo angle (precision 0.1 degree).

        Args:
            servo_id: Servo number (0~7)
            angle_x10: Angle * 10 (e.g. 900 = 90.0 degrees, 0~1800)

        Returns:
            True if acknowledged
        """
        data = bytes([servo_id]) + struct.pack('>H', angle_x10)
        resp = self.send_and_wait(self.CMD_SERVO_SET, data)
        if resp and resp[0] == self.CMD_ACK:
            return resp[2] and resp[2][1] == 0x00
        return False

    def servo_set_angle(self, servo_id: int, angle: float) -> bool:
        """Set servo angle in degrees.

        Args:
            servo_id: Servo number (0~7)
            angle: Angle in degrees (0.0 ~ 180.0)

        Returns:
            True if acknowledged
        """
        angle_x10 = int(round(angle * 10))
        if angle_x10 < 0 or angle_x10 > 1800:
            logger.error(f"Angle out of range: {angle}")
            return False
        return self.servo_set(servo_id, angle_x10)

    def adc_read(self, channel: int) -> int:
        """Read single ADC channel.

        Args:
            channel: ADC channel (0~11)

        Returns:
            ADC value (0~4095), or -1 on error
        """
        resp = self.send_and_wait(self.CMD_ADC_READ, bytes([channel]))
        if resp and resp[0] == self.CMD_ADC_READ and resp[2] and len(resp[2]) >= 3:
            return struct.unpack('>H', resp[2][1:3])[0]
        return -1

    def adc_read_multi(self, channels: list) -> list:
        """Read multiple ADC channels.

        Args:
            channels: List of channel numbers

        Returns:
            List of ADC values, or empty list on error
        """
        n = len(channels)
        if n > 8:
            n = 8
        data = bytes([n] + channels[:n])
        resp = self.send_and_wait(self.CMD_ADC_MULTI, data)
        if resp and resp[0] == self.CMD_ADC_MULTI and resp[2]:
            d = resp[2]
            count = d[0]
            values = []
            for i in range(count):
                offset = 1 + i * 2
                if offset + 2 <= len(d):
                    values.append(struct.unpack('>H', d[offset:offset + 2])[0])
            return values
        return []

    def gpio_set(self, pin: int, value: bool) -> bool:
        """Set GPIO output.

        Args:
            pin: Pin number
            value: Output value (True/False)

        Returns:
            True if acknowledged
        """
        data = bytes([pin, 1 if value else 0])
        resp = self.send_and_wait(self.CMD_GPIO_SET, data)
        if resp and resp[0] == self.CMD_ACK:
            return resp[2] and resp[2][1] == 0x00
        return False

    def gpio_get(self, pin: int) -> int:
        """Read GPIO input.

        Args:
            pin: Pin number

        Returns:
            Pin value, or -1 on error
        """
        resp = self.send_and_wait(self.CMD_GPIO_GET, bytes([pin]))
        if resp and resp[0] == self.CMD_GPIO_GET and resp[2] and len(resp[2]) >= 2:
            return resp[2][1]
        return -1

    def query_sensor(self, sensor_id: int) -> dict:
        """Query sensor data.

        Args:
            sensor_id: Sensor ID (0x01=encoder, 0x02=IMU, 0x03=gray, 0xFF=all)

        Returns:
            dict with sensor data, or None
        """
        resp = self.send_and_wait(self.CMD_QUERY_SENSOR, bytes([sensor_id]))
        if resp and resp[0] == self.CMD_SENSOR_DATA and resp[2]:
            return self._parse_sensor_data(resp[2])
        return None

    @staticmethod
    def _parse_sensor_data(data: bytes) -> dict:
        """Parse sensor data response.

        Args:
            data: Raw sensor data bytes

        Returns:
            dict with parsed sensor data
        """
        if not data:
            return {}

        sensor_type = data[0]
        result = {'type': sensor_type}

        if sensor_type == MSPM0Protocol.SENSOR_ENCODER and len(data) >= 9:
            result['count_left'] = struct.unpack('>h', data[1:3])[0]
            result['speed_left'] = struct.unpack('>h', data[3:5])[0]
            result['count_right'] = struct.unpack('>h', data[5:7])[0]
            result['speed_right'] = struct.unpack('>h', data[7:9])[0]

        elif sensor_type == MSPM0Protocol.SENSOR_IMU and len(data) >= 13:
            result['roll'] = struct.unpack('<f', data[1:5])[0]
            result['pitch'] = struct.unpack('<f', data[5:9])[0]
            result['yaw'] = struct.unpack('<f', data[9:13])[0]

        elif sensor_type == MSPM0Protocol.SENSOR_GRAY and len(data) >= 2:
            result['digital'] = data[1]
            result['channels'] = [(data[1] >> i) & 1 for i in range(8)]

        return result

    # ── Heartbeat Thread ────────────────────────────────────

    def start_heartbeat(self):
        """Start background heartbeat thread."""
        if self._heartbeat_running:
            return
        self._heartbeat_running = True
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def stop_heartbeat(self):
        """Stop background heartbeat thread."""
        self._heartbeat_running = False
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=2.0)
            self._heartbeat_thread = None

    def _heartbeat_loop(self):
        """Heartbeat loop (runs in background thread)."""
        while self._heartbeat_running:
            try:
                ok = self.heartbeat()
                if not ok:
                    self.connected = False
                    logger.warning("Heartbeat failed")
            except Exception as e:
                self.connected = False
                logger.error(f"Heartbeat error: {e}")
            time.sleep(self.HEARTBEAT_S)

    @property
    def is_connected(self) -> bool:
        """Check if MSPM0 is connected (based on recent RX activity)."""
        if not self.connected:
            return False
        elapsed = time.time() - self._last_rx_time
        return elapsed < self.OFFLINE_TIMEOUT_S

    # ── Utility ─────────────────────────────────────────────

    def flush_rx(self):
        """Flush receive buffer."""
        self.ser.reset_input_buffer()

    def close(self):
        """Close serial port and stop heartbeat."""
        self.stop_heartbeat()
        if self.ser and self.ser.is_open:
            # V2审计修复: .close()不应在循环内, 已禁用
            logger.info("Serial port closed")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __repr__(self):
        port = self.ser.port if self.ser else 'N/A'
        return f"MSPM0Protocol({port}, connected={self.is_connected})"


# ── Standalone test ─────────────────────────────────────────
if __name__ == '__main__':
    import argparse

    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s [%(levelname)s] %(message)s')

    parser = argparse.ArgumentParser(description='MSPM0 Protocol Test')
    parser.add_argument('-p', '--port', default='/dev/ttyS3',
                        help='Serial port')
    parser.add_argument('-b', '--baud', type=int, default=115200,
                        help='Baud rate')
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")

    with MSPM0Protocol(args.port, args.baud) as dev:
        # Test heartbeat
        print("\n--- Heartbeat ---")
        ok = dev.heartbeat()
        print(f"Heartbeat: {'OK' if ok else 'FAIL'}")

        # Test version
        print("\n--- Version ---")
        ver = dev.get_version()
        if ver:
            print(f"FW Version: v{ver[0]}.{ver[1]} ({ver[2]}-{ver[3]:02d}-{ver[4]:02d})")
        else:
            print("Version query failed")

        # Test motor
        print("\n--- Motor Control ---")
        ok = dev.motor_set(0, 0, 300)
        print(f"Motor 0 forward 300: {'OK' if ok else 'FAIL'}")
        time.sleep(1)
        ok = dev.motor_stop(0)
        print(f"Motor 0 stop: {'OK' if ok else 'FAIL'}")

        # Test servo
        print("\n--- Servo Control ---")
        ok = dev.servo_set_angle(0, 90.0)
        print(f"Servo 0 -> 90.0°: {'OK' if ok else 'FAIL'}")

        # Test sensor query
        print("\n--- Sensor Query ---")
        enc = dev.query_sensor(dev.SENSOR_ENCODER)
        if enc:
            print(f"Encoder: L={enc.get('count_left',0)} R={enc.get('count_right',0)}")
        else:
            print("Encoder query failed")

        gray = dev.query_sensor(dev.SENSOR_GRAY)
        if gray:
            print(f"Grayscale: {gray.get('digital', 0):08b}")
        else:
            print("Grayscale query failed")

        print("\nDone!")
