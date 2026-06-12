# OPi5 与 MSPM0 通信协议规范

## 1. 概述

- **物理层**：UART，波特率 115200，8N1（8数据位，无校验，1停止位）
- **逻辑层**：自定义二进制帧协议，支持命令请求/应答、数据上报、错误通知
- **连接方式**：OPi5 UART3_TX → MSPM0 UART0_RX，OPi5 UART3_RX → MSPM0 UART0_TX，共地

---

## 2. 帧格式

### 2.1 通用帧结构

```
┌──────┬──────┬────────┬──────┬──────────┬──────┐
│ HEAD │ LEN  │ CMD_ID │ SEQ  │  DATA    │ CRC  │
│ 0xAA │ 1B   │  1B    │  1B  │ 0~255B   │ 2B   │
└──────┴──────┴────────┴──────┴──────────┴──────┘
```

| 字段 | 长度 | 说明 |
|------|------|------|
| HEAD | 1B | 帧头，固定 `0xAA` |
| LEN | 1B | 数据段长度（不含帧头、LEN、CRC），范围 0~255 |
| CMD_ID | 1B | 命令编号，见 §3 |
| SEQ | 1B | 序列号 0~255，每发一帧递增，用于应答匹配 |
| DATA | 0~255B | 命令参数/响应数据，格式由 CMD_ID 定义 |
| CRC | 2B | CRC-16/MODBUS，对 HEAD 到 DATA 全部字节计算 |

### 2.2 CRC-16/MODBUS 计算

```python
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc  # 低字节在前发送
```

### 2.3 帧定界与转义

- 接收方以 `0xAA` 为帧头同步
- **无转义机制**：LEN 字段明确数据长度，接收方按长度读取
- 帧间隔 ≥ 2ms（即两个字节之间的静默时间超过 2ms 则认为新帧）

---

## 3. 命令定义

### 3.1 命令总览

| CMD_ID | 名称 | 方向 | 说明 |
|--------|------|------|------|
| 0x01 | HEARTBEAT | 双向 | 心跳/在线检测 |
| 0x02 | VERSION | OPi5→MSPM0 | 查询固件版本 |
| 0x10 | MOTOR_SET | OPi5→MSPM0 | 设置电机速度 |
| 0x11 | MOTOR_GET | OPi5→MSPM0 | 读取电机状态 |
| 0x12 | MOTOR_STOP | OPi5→MSPM0 | 紧急停止电机 |
| 0x20 | SERVO_SET | OPi5→MSPM0 | 设置舵机角度 |
| 0x21 | SERVO_GET | OPi5→MSPM0 | 读取舵机角度 |
| 0x30 | ADC_READ | OPi5→MSPM0 | 读取ADC值 |
| 0x31 | ADC_MULTI | OPi5→MSPM0 | 多通道ADC读取 |
| 0x40 | GPIO_SET | OPi5→MSPM0 | 设置GPIO输出 |
| 0x41 | GPIO_GET | OPi5→MSPM0 | 读取GPIO输入 |
| 0x50 | SENSOR_DATA | MSPM0→OPi5 | 主动上报传感器数据 |
| 0xE0 | ACK | 双向 | 通用应答 |
| 0xFE | ERROR | 双向 | 错误报告 |

### 3.2 命令详细定义

#### 0x01 HEARTBEAT — 心跳
- **请求**：DATA 为空
- **应答**：DATA 为空
- **超时**：3 秒无心跳则认为对端离线

#### 0x02 VERSION — 版本查询
- **请求**：DATA 为空
- **应答**：
  ```
  DATA[0]   = 主版本号
  DATA[1]   = 次版本号
  DATA[2:4] = 编译日期 BCD（年月日）
  ```
- **示例**：`AA 03 02 01 01 02 26 06 11 CRC` → V1.02, 2026-06-11

#### 0x10 MOTOR_SET — 电机设置
- **请求**：
  ```
  DATA[0]    = 电机编号（0~3）
  DATA[1]    = 方向（0=正转, 1=反转）
  DATA[2:4]  = 速度（uint16, 0~1000 对应占空比 0~100%）
  ```
- **应答**：ACK（0xE0），DATA[0] = 0x00 表示成功

#### 0x11 MOTOR_GET — 电机状态
- **请求**：DATA[0] = 电机编号
- **应答**：
  ```
  DATA[0]    = 方向
  DATA[2:4]  = 当前速度（uint16）
  DATA[4:6]  = 编码器脉冲计数（int16，自上次查询以来的增量）
  ```

#### 0x12 MOTOR_STOP — 紧急停止
- **请求**：DATA[0] = 电机编号（0xFF = 全部停止）
- **应答**：ACK

#### 0x20 SERVO_SET — 舵机设置
- **请求**：
  ```
  DATA[0]    = 舵机编号（0~7）
  DATA[2:4]  = 角度（uint16, 0~1800 = 0.0°~180.0°，精度0.1°）
  ```
- **应答**：ACK

#### 0x30 ADC_READ — 单通道ADC
- **请求**：DATA[0] = 通道号（0~11）
- **应答**：
  ```
  DATA[0]    = 通道号
  DATA[1:3]  = ADC值（uint16, 0~4095）
  ```

#### 0x31 ADC_MULTI — 多通道ADC
- **请求**：
  ```
  DATA[0]     = 通道数 N（1~8）
  DATA[1:N+1] = 通道号列表
  ```
- **应答**：
  ```
  DATA[0]       = 通道数 N
  DATA[1:3]     = CH0 值（uint16）
  DATA[3:5]     = CH1 值（uint16）
  ...
  DATA[2N-1:2N+1] = CH(N-1) 值
  ```

#### 0x50 SENSOR_DATA — 传感器上报
- **方向**：MSPM0 → OPi5（主动发送）
- **DATA 格式**：
  ```
  DATA[0]      = 传感器类型（0=IMU, 1=超声波, 2=红外, 3=光电）
  DATA[1:]     = 类型相关数据
  ```

#### 0xE0 ACK — 通用应答
```
DATA[0] = 原命令 CMD_ID
DATA[1] = 状态码（0x00=成功, 其他见§4）
```

#### 0xFE ERROR — 错误报告
```
DATA[0] = 错误码（见§4）
DATA[1:3] = 附加信息（可选）
```

---

## 4. 错误处理

### 4.1 错误码定义

| 错误码 | 名称 | 说明 |
|--------|------|------|
| 0x01 | ERR_CRC | CRC校验失败 |
| 0x02 | ERR_CMD | 未知命令 |
| 0x03 | ERR_PARAM | 参数非法 |
| 0x04 | ERR_BUSY | 设备忙 |
| 0x05 | ERR_TIMEOUT | 内部超时（如ADC转换超时） |
| 0x06 | ERR_RANGE | 超出范围（如角度/速度超限） |
| 0x07 | ERR_STATE | 状态错误（如电机未使能时设置速度） |

### 4.2 错误处理策略

```
发送方                              接收方
  │                                    │
  │─── 命令帧 ───────────────────────→│
  │                                    │ CRC校验失败 → 发送 ERR_CRC
  │                                    │ 命令解析失败 → 发送 ERR_CMD
  │                                    │ 参数校验失败 → 发送 ERR_PARAM
  │←── ACK(成功) 或 ERROR ───────────│
  │                                    │
  │ 超时(100ms)无应答 → 重试1次       │
  │ 重试仍无应答 → 上报上层通信故障    │
```

### 4.3 超时与重试

| 参数 | 值 | 说明 |
|------|------|------|
| 应答超时 | 100ms | 发送命令后等待应答的时间 |
| 重试次数 | 1 | 超时后重试1次 |
| 重试间隔 | 10ms | 重试前的等待时间 |
| 心跳间隔 | 1000ms | 正常心跳发送周期 |
| 离线判定 | 3000ms | 连续3次心跳无应答 |

### 4.4 通信状态机

```
         初始化
           │
           ▼
    ┌──────────────┐
    │   DISCONNECTED│ ←──── 超时/错误过多
    └──────┬───────┘
           │ 收到心跳应答
           ▼
    ┌──────────────┐
    │   CONNECTED  │ ←──── 正常工作
    └──────┬───────┘
           │ 连续3次心跳失败
           ▼
    ┌──────────────┐
    │   ERROR      │ → 尝试重连 → DISCONNECTED
    └──────────────┘
```

---

## 5. 代码实现参考

### 5.1 MSPM0 端（C语言）

```c
#define FRAME_HEAD  0xAA
#define CMD_HEARTBEAT 0x01
#define CMD_MOTOR_SET 0x10
#define CMD_ACK       0xE0
#define CMD_ERROR     0xFE

typedef struct {
    uint8_t head;
    uint8_t len;
    uint8_t cmd;
    uint8_t seq;
    uint8_t data[255];
    uint16_t crc;
} frame_t;

typedef enum {
    STATE_IDLE,
    STATE_LEN,
    STATE_CMD,
    STATE_SEQ,
    STATE_DATA,
    STATE_CRC_L,
    STATE_CRC_H
} rx_state_t;

static rx_state_t rx_state = STATE_IDLE;
static frame_t rx_frame;
static uint8_t rx_idx = 0;

void uart_rx_isr(uint8_t byte) {
    switch (rx_state) {
    case STATE_IDLE:
        if (byte == FRAME_HEAD) {
            rx_frame.head = byte;
            rx_state = STATE_LEN;
        }
        break;
    case STATE_LEN:
        rx_frame.len = byte;
        rx_idx = 0;
        rx_state = STATE_CMD;
        break;
    case STATE_CMD:
        rx_frame.cmd = byte;
        rx_state = STATE_SEQ;
        break;
    case STATE_SEQ:
        rx_frame.seq = byte;
        if (rx_frame.len > 0) {
            rx_state = STATE_DATA;
        } else {
            rx_state = STATE_CRC_L;
        }
        break;
    case STATE_DATA:
        rx_frame.data[rx_idx++] = byte;
        if (rx_idx >= rx_frame.len) {
            rx_state = STATE_CRC_L;
        }
        break;
    case STATE_CRC_L:
        rx_frame.crc = byte;
        rx_state = STATE_CRC_H;
        break;
    case STATE_CRC_H:
        rx_frame.crc |= (byte << 8);
        // 验证CRC，处理命令
        uint8_t buf[259];
        buf[0] = rx_frame.head;
        buf[1] = rx_frame.len;
        buf[2] = rx_frame.cmd;
        buf[3] = rx_frame.seq;
        memcpy(&buf[4], rx_frame.data, rx_frame.len);
        if (crc16_modbus(buf, 4 + rx_frame.len) == rx_frame.crc) {
            process_frame(&rx_frame);
        } else {
            send_error(rx_frame.seq, ERR_CRC);
        }
        rx_state = STATE_IDLE;
        break;
    }
}
```

### 5.2 OPi5 端（Python）

```python
import struct
import time

class MSPM0Protocol:
    HEAD = 0xAA
    
    def __init__(self, port='/dev/ttyS3', baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.seq = 0
    
    @staticmethod
    def crc16_modbus(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def send(self, cmd: int, data: bytes = b'') -> tuple:
        seq = self.seq & 0xFF
        self.seq += 1
        payload = bytes([self.HEAD, len(data), cmd, seq]) + data
        crc = self.crc16_modbus(payload)
        self.ser.write(payload + struct.pack('<H', crc))
        return cmd, seq
    
    def recv(self, timeout=0.1):
        deadline = time.time() + timeout
        while time.time() < deadline:
            head = self.ser.read(1)
            if not head or head[0] != self.HEAD:
                continue
            meta = self.ser.read(3)
            if len(meta) < 3:
                continue
            length, cmd, seq = meta[0], meta[1], meta[2]
            data = self.ser.read(length) if length > 0 else b''
            crc_bytes = self.ser.read(2)
            if len(crc_bytes) < 2:
                continue
            payload = head + meta + data
            expected = struct.pack('<H', self.crc16_modbus(payload))
            if crc_bytes != expected:
                continue
            return cmd, seq, data
        return None
    
    def heartbeat(self):
        self.send(0x01)
        return self.recv(0.05) is not None
    
    def motor_set(self, motor: int, direction: int, speed: int):
        data = struct.pack('<BBH', motor, direction, speed)
        self.send(0x10, data)
        resp = self.recv(0.1)
        return resp and resp[2][1] == 0x00
    
    def adc_read(self, channel: int) -> int:
        self.send(0x30, bytes([channel]))
        resp = self.recv(0.05)
        if resp and resp[0] == 0x30:
            return struct.unpack('<H', resp[2][1:3])[0]
        return -1
    
    def close(self):
        self.ser.close()
```

---

## 6. 调试建议

1. **抓包工具**：用 USB-TTL 转接器 + 串口助手同时监听，查看实际收发数据
2. **十六进制显示**：串口助手务必切换到 HEX 模式
3. **逐条测试**：先测心跳（0x01），再测简单命令，最后测复杂命令
4. **日志记录**：在 OPi5 端用 `logging` 模块记录所有收发数据
5. **逻辑分析仪**：用 Saleae/DSLogic 捕获 UART 波形，验证时序
