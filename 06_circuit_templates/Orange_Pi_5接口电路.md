# Orange Pi 5 接口电路设计指南

## 1. 芯片概述

Orange Pi 5 搭载 Rockchip RK3588S SoC，提供丰富的外设接口。工作电压域：3.3V（主GPIO）、1.8V（部分引脚复用）。

### 关键参数
| 参数 | 规格 |
|------|------|
| CPU | 4×A76 + 4×A55 |
| GPIO | 最多 28×GPIO（2.54mm排针） |
| 电平 | 3.3V（大部分），1.8V（部分） |
| USB | 1×USB3.0 + 2×USB2.0 + 1×Type-C |
| 供电 | 5V/4A Type-C PD |

---

## 2. GPIO 接口电路

### 2.1 基本 GPIO 输出驱动

```
Orange Pi 5 GPIO (3.3V)
        │
        ├──[220Ω]──→ LED ──→ GND
        │
        ├──[1kΩ]──→ NPN基极(S8050) ──→ 继电器线圈 ──→ 12V
        │                                           │
        │                                    续流二极管(1N4007)
```

**注意事项：**
- GPIO 最大灌/拉电流约 20mA，驱动负载务必外接三极管/MOS管
- 驱动感性负载（继电器、电机）必须加续流二极管
- 推荐 S8050(NPN) 或 2N7000(NMOS) 做开关驱动

### 2.2 GPIO 输入保护

```
外部信号 ──[100Ω]──┬──→ OPi GPIO
                    │
                  [TVS 3.3V]  (SMBJ3.3A)
                    │
                   GND
```

**电平转换（3.3V ↔ 5V）：**
- 上拉到3.3V，外接电平转换芯片（TXS0108E / SN74LVC4245A）
- 简单场景可用分压电阻：5V信号 → [1kΩ] → GPIO ← [2kΩ] → GND

### 2.3 推荐 GPIO 引脚分配（电赛常用）

| 引脚编号 | 功能建议 | 说明 |
|----------|----------|------|
| PIN_7 | GPIO输出 | 控制/使能信号 |
| PIN_11 | GPIO输入 | 按键/限位开关 |
| PIN_13 | GPIO输出 | LED指示灯 |
| PIN_15 | GPIO输入 | 传感器中断 |
| PIN_16 | GPIO输出 | 电机使能 |
| PIN_18 | GPIO输入 | 编码器A相 |
| PIN_22 | GPIO输入 | 编码器B相 |

---

## 3. UART 接口电路

### 3.1 默认 UART（调试串口）

```
OPi UART_TX (PIN_8)  ──→ USB-TTL模块 RXD
OPi UART_RX (PIN_10) ←── USB-TTL模块 TXD
OPi GND (PIN_6)      ──→ USB-TTL模块 GND
```

**参数：** 115200 bps, 8N1, 3.3V电平

### 3.2 UART 连接外部 MCU/传感器

```
OPi UART2_TX ──[100Ω]──→ 外部设备 RX
OPi UART2_RX ←──[100Ω]── 外部设备 TX
               │
             [10kΩ上拉至3.3V]
```

**RS-485 长距离通信方案：**
```
OPi UART_TX ──→ MAX485 DI
OPi UART_RX ←── MAX485 RO
OPi GPIO   ──→ MAX485 DE/RE（方向控制）
MAX485 A/B ──→ 双绞线(120Ω终端匹配)
```

### 3.3 常用波特率与配置

```bash
# Linux下配置串口
stty -F /dev/ttyS1 115200 cs8 -cstopb -parenb
# Python测试
import serial
ser = serial.Serial('/dev/ttyS1', 115200, timeout=1)
```

---

## 4. SPI 接口电路

### 4.1 基本 SPI 连接（3.3V外设）

```
OPi SPI0_CLK (PIN_23) ──→ 从设备 SCK
OPi SPI0_MOSI(PIN_19) ──→ 从设备 MOSI
OPi SPI0_MISO(PIN_21) ←── 从设备 MISO
OPi SPI0_CS0 (PIN_24) ──→ 从设备 CS
```

### 4.2 驱动 5V SPI 设备（如 NRF24L01）

```
         3.3V ──┬──[10kΩ]──→ NRF VCC (3.3V直接供电)
                │
OPi SPI_CLK ──[100Ω]──→ NRF SCK
OPi SPI_MOSI──[100Ω]──→ NRF MOSI
OPi SPI_MISO←────────── NRF MISO （3.3V兼容）
OPi GPIO_CS ──────────→ NRF CSN
OPi GPIO_CE ──────────→ NRF CE
```

### 4.3 SPI DAC 输出电路

```
OPi SPI ──→ MCP4922(SPI DAC) ──→ [RC滤波] ──→ [运放缓冲] ──→ 输出
              VREF = 2.048V      R=1kΩ,C=100nF   OPA340
```

---

## 5. I2C 接口电路

### 5.1 基本 I2C 总线

```
                    3.3V
                     │
                [4.7kΩ] [4.7kΩ]
                     │     │
OPi I2C_SDA (PIN_3)──┼─────┤──→ 从设备 SDA
OPi I2C_SCL (PIN_5)──┼─────┤──→ 从设备 SCL
                     │     │
                  从设备2 SDA/SCL （总线并联）
```

**关键设计要点：**
- 上拉电阻 4.7kΩ（3.3V），总线速率 ≤400kHz 时推荐
- 设备多或线长时降至 2.2kΩ
- 总线电容 < 400pF（含走线和器件寄生电容）
- 每个从设备地址不冲突

### 5.2 I2C 电平转换（3.3V ↔ 5V）

```
3.3V ──[4.7kΩ]──┬── I2C_SDA_3V3
                 │
MOSFET(2N7002)  Gate→SDA_3V3
                 │
5V ──[4.7kΩ]───┬── I2C_SDA_5V
               │
          （SCL同理）
```

### 5.3 常用 I2C 外设地址

| 设备 | 地址(7bit) | 用途 |
|------|-----------|------|
| OLED SSD1306 | 0x3C | 显示屏 |
| MPU6050 | 0x68 | 六轴IMU |
| BMP280 | 0x76 | 气压计 |
| ADS1115 | 0x48 | 16位ADC |
| PCF8574 | 0x27 | IO扩展 |
| INA219 | 0x40 | 电流检测 |

---

## 6. PWM 接口电路

### 6.1 PWM 直接驱动（3.3V电平）

```
OPi PWM_OUT ──[100Ω]──→ 负载（舵机信号线/LED）
```

### 6.2 PWM 驱动舵机

```
OPi PWM ──→ 舵机信号线（橙色）
5V/6V  ──→ 舵机电源线（红色）  ← 独立供电！
GND    ──→ 舵机地线（棕色）    ← 必须共地！
```

**舵机控制参数：**
- 周期：20ms（50Hz）
- 脉宽：0.5ms(0°) ~ 2.5ms(180°)
- Python: `echo 1500 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle`（单位ns）

### 6.3 PWM 控制电机（经MOSFET）

```
OPi PWM ──[100Ω]──→ IRF540N Gate
                     Source → GND
                     Drain ──→ 电机(-)
电机(+) ──→ 12V
        ── 续流二极管(MBR2045) 反并联 ──
```

---

## 7. 供电设计要点

```
5V/4A PD充电器 ──→ Type-C ──→ OPi 5
                    │
              大电流外设（电机/舵机）独立供电
              共地！共地！共地！
```

- **OPi 5 推荐 5V/4A**，使用支持PD协议的充电器
- GPIO 3.3V 输出电流有限，外设一律外部供电
- 电机/舵机电源与 OPi 电源**共地但不共电源**
- 长距离供电加 100μF 电解 + 100nF 陶瓷退耦

---

## 8. 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| GPIO无输出 | 引脚未export/复用冲突 | 检查设备树overlay |
| UART乱码 | 波特率/电平不匹配 | 确认3.3V电平+波特率一致 |
| I2C无应答 | 上拉缺失/地址错误 | 加4.7kΩ上拉，i2cdetect扫描 |
| SPI数据错位 | 时钟极性/相位错 | 调整CPOL/CPHA |
| 舵机抖动 | 供电不足 | 独立5V/2A以上供电 |

---

*最后更新：2026年电赛备赛*
