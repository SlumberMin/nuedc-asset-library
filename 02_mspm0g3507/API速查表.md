# MSPM0G3507 驱动 API 速查表

> 适用于 TI MSPM0G3507（Cortex-M0+, 80MHz, 128KB Flash, 32KB SRAM）

---

## 目录

1. [平台层 (platform)](#1-平台层)
2. [电机驱动](#2-电机驱动)
3. [编码器](#3-编码器)
4. [舵机](#4-舵机)
5. [JY901S IMU](#5-jy901s-imu)
6. [灰度传感器](#6-灰度传感器)
7. [超声波传感器](#7-超声波传感器)
8. [颜色传感器 TCS34725](#8-颜色传感器)
9. [蓝牙 HC-05](#9-蓝牙-hc-05)
10. [OLED SSD1306](#10-oled-ssd1306)
11. [EEPROM AT24C02](#11-eeprom-at24c02)
12. [PCA9685 舵机驱动板](#12-pca9685)
13. [DriverLib 原生 API](#13-driverlib-原生-api)

---

## 1. 平台层

**头文件**: `platform/driverlib_mspm0.h`

| 宏/函数 | 说明 |
|---------|------|
| `GPIO_SET(port, pin)` | GPIO 置高 |
| `GPIO_CLR(port, pin)` | GPIO 置低 |
| `GPIO_TOGGLE(port, pin)` | GPIO 翻转 |
| `GPIO_READ(port, pin)` | GPIO 读取（返回 0/非0） |
| `PWM_SET_DUTY(timer, ch, val)` | 设置 PWM 占空比 |
| `ADC_START(adc)` | 启动 ADC 转换 |
| `ADC_READ(adc, mem)` | 读取 ADC 结果 |
| `UART_TX_BYTE(uart, byte)` | UART 发送一字节 |
| `UART_RX_BYTE(uart)` | UART 接收一字节 |
| `DELAY_MS(ms)` | 阻塞延时（毫秒） |
| `DELAY_US(us)` | 阻塞延时（微秒） |

---

## 2. 电机驱动

### TB6612 (`drivers/tb6612.h`)

```c
void TB6612_Init(TB6612_Config *cfg);
void TB6612_SetSpeed(TB6612_Channel ch, int16_t speed);  // -1000~+1000
void TB6612_Brake(TB6612_Channel ch);
void TB6612_Standby(void);  // STNDBY 拉低，进入待机
```

| 参数 | 说明 |
|------|------|
| `speed > 0` | 正转 |
| `speed < 0` | 反转 |
| `speed = 0` | 停止（滑行） |

### L298N (`drivers/l298n.h`)

```c
void L298N_Init(L298N_Config *cfg);
void L298N_SetSpeed(L298N_Channel ch, int16_t speed);  // -1000~+1000
void L298N_Brake(L298N_Channel ch);
```

---

## 3. 编码器

### GPIO 编码器 (`drivers/encoder_gpio_mspm0.h`) — 推荐 N20 电机

```c
void EncoderGpio_Init(EncoderGpioConfig *cfg);
int32_t EncoderGpio_Read(EncoderGpio_Channel ch);       // 累计脉冲数
int32_t EncoderGpio_GetSpeed(EncoderGpio_Channel ch);   // 脉冲/采样周期
void EncoderGpio_Reset(EncoderGpio_Channel ch);
void EncoderGpio_SetInverted(EncoderGpio_Channel ch, uint8_t inv);
```

### 定时器编码器 (`drivers/encoder_mspm0.h`) — 高速编码器

```c
void Encoder_Init(TIMER_Regs *timer, uint32_t period);
int32_t Encoder_Read(Encoder_Channel ch);
void Encoder_SetInverted(Encoder_Channel ch, uint8_t inv);
```

---

## 4. 舵机

**头文件**: `drivers/servo_mspm0.h`

```c
void Servo_Init(TIMER_Regs *timer, DL_TIMER_CC_INDEX ch);
void Servo_SetAngle(float angle);           // 0~180°
void Servo_SetRange(uint16_t min_us, uint16_t max_us);  // 校准脉宽（默认 500~2500μs）
void Servo_SetPulse(uint16_t pulse_us);     // 直接设置脉宽
```

---

## 5. JY901S IMU

**头文件**: `drivers/jy901s_mspm0.h`

```c
void JY901S_Init(JY901S_Config *cfg);
uint8_t JY901S_IsDataReady(void);
void JY901S_ClearDataReady(void);
void JY901S_GetAngle(float *pitch, float *roll, float *yaw);       // 单位：度
void JY901S_GetAccel(float *ax, float *ay, float *az);             // 单位：g
void JY901S_GetGyro(float *gx, float *gy, float *gz);              // 单位：°/s
void JY901S_GetQuaternion(float *q0, float *q1, float *q2, float *q3);
```

| 字段 | 说明 |
|------|------|
| `cfg->uart` | UART 实例，如 `UART_0_INST` |
| `cfg->baudrate` | 波特率，默认 9600 |
| `cfg->auto_calib` | 上电自动校准（1=开启） |

---

## 6. 灰度传感器

**头文件**: `drivers/grayscale_mspm0.h`

```c
void Grayscale_Init(GrayscaleConfig *cfg);
void Grayscale_Calibrate(uint16_t *white_val, uint16_t *black_val);
void Grayscale_Read(void);
int16_t Grayscale_GetTrackError(void);          // 加权偏差（负=偏左，正=偏右）
uint8_t Grayscale_IsOffTrack(void);             // 1=全部脱线
uint8_t Grayscale_IsAllBlack(void);             // 1=十字/起始线
uint16_t Grayscale_GetChannel(uint8_t ch);      // 单通道原始值（0~7）
```

---

## 7. 超声波传感器

**头文件**: `drivers/ultrasonic_mspm0.h`

```c
void Ultrasonic_Init(UltrasonicConfig *cfg);
float Ultrasonic_Measure(void);                 // 返回距离（cm），失败返回 -1
float Ultrasonic_MeasureAvg(uint8_t n);         // 多次测量取平均
```

| 参数 | 说明 |
|------|------|
| `cfg->type` | `ULTRASONIC_SR04` 或 `ULTRASONIC_US016` |
| `cfg->filter_size` | 中值滤波窗口大小 |

---

## 8. 颜色传感器

**头文件**: `drivers/tcs34725.h`

```c
void TCS34725_Init(TCS34725_Config *cfg);
void TCS34725_Read(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);  // 原始值
void TCS34725_GetRGB(uint8_t *r, uint8_t *g, uint8_t *b);                // 0~255
uint8_t TCS34725_GetColor(void);   // 返回颜色枚举：COLOR_RED/GREEN/BLUE/WHITE/BLACK
void TCS34725_SetIntegrationTime(uint8_t time);  // 0xFF=2.4ms, 0x00=700ms
void TCS34725_SetGain(uint8_t gain);             // 0=1x, 1=4x, 2=16x, 3=60x
```

---

## 9. 蓝牙 HC-05

**头文件**: `drivers/bluetooth_hc05_mspm0.h`

```c
void BT_HC05_Init(BT_HC05_Config *cfg);
uint8_t BT_HC05_IsConnected(void);                      // 1=已连接
void BT_HC05_SendString(const char *str);
void BT_HC05_SendData(const uint8_t *data, uint16_t len);
uint16_t BT_HC05_GetReceivedData(uint8_t *buf, uint16_t max_len);
uint8_t BT_HC05_IsDataReceived(void);
void BT_HC05_ClearRxBuffer(void);
```

---

## 10. OLED SSD1306

**头文件**: `drivers/oled_ssd1306_mspm0.h`

```c
void OLED_Init(uint8_t i2c_instance);
void OLED_Clear(void);
void OLED_Refresh(void);                                 // 刷新到屏幕
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size, uint8_t mode);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t size, uint8_t mode);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t mode); // mode: 1=亮, 0=灭
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t index);  // 预存字模
```

| 参数 | 说明 |
|------|------|
| `size` | 字号：12=12×6, 16=16×8 |
| `mode` | 0=反色, 1=正常 |

---

## 11. EEPROM AT24C02

**头文件**: `drivers/at24c02.h`

```c
void AT24C02_Init(uint8_t i2c_instance);
uint8_t AT24C02_ReadByte(uint16_t addr);                // 0x00~0xFF
void AT24C02_WriteByte(uint16_t addr, uint8_t data);
void AT24C02_Read(uint16_t addr, uint8_t *buf, uint16_t len);
void AT24C02_Write(uint16_t addr, const uint8_t *buf, uint16_t len);
uint8_t AT24C02_IsReady(void);                           // 写入完成检测
```

> **注意**: AT24C02 单页 8 字节，跨页写入需分页处理。驱动内部已处理。

---

## 12. PCA9685

**头文件**: `drivers/pca9685.h`

```c
void PCA9685_Init(PCA9685_Config *cfg);
void PCA9685_SetPWM(uint8_t ch, uint16_t on, uint16_t off);  // 0~4095
void PCA9685_SetServoAngle(uint8_t ch, float angle);           // 0~180°
void PCA9685_SetFreq(uint16_t freq);                           // 24~1526 Hz
void PCA9685_Sleep(void);
void PCA9685_Wakeup(void);
```

---

## 13. DriverLib 原生 API

```c
// GPIO
DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0);
DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_0);
DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_0);
uint32_t val = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_0);
DL_GPIO_enableInterrupt(GPIOA, DL_GPIO_PIN_0);

// UART
DL_UART_main_transmitData(UART0, 0x55);
uint8_t rx = DL_UART_main_receiveData(UART0);
bool empty = DL_UART_isTXFIFOEmpty(UART0);

// I2C
DL_I2C_startControllerTransfer(I2C0, addr, DL_I2C_CONTROLLER_DIRECTION_TX, len);
DL_I2C_transmitControllerData(I2C0, data);
DL_I2C_startControllerTransfer(I2C0, addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);
uint8_t rx = DL_I2C_receiveControllerData(I2C0);

// Timer PWM
DL_TimerA_setCaptureCompareValue(TIMG0, duty, DL_TIMER_CC_0_INDEX);
DL_TimerA_setLoadValue(TIMG0, period);

// ADC
DL_ADC12_startConversion(ADC0);
while (!DL_ADC12_getRawInterruptStatus(ADC0, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED)) {}
uint16_t result = DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
DL_ADC12_clearInterruptStatus(ADC0, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

// 延时
DL_Common_delayMilliseconds(100);
DL_Common_delayMicroseconds(10);
```

---

## SysConfig 引脚速查

| 外设 | SysConfig 模块 | 常用引脚 |
|------|---------------|---------|
| GPIO 输出 | `/ti/driverlib/GPIO` | PA0~PA31, PB0~PB31 |
| UART | `/ti/driverlib/UART` | TX: PA10, RX: PA11 (UART0) |
| I2C | `/ti/driverlib/I2C` | SCL: PB2, SDA: PB3 (I2C0) |
| SPI | `/ti/driverlib/SPI` | CLK/MOSI/MISO 按需分配 |
| Timer PWM | `/ti/driverlib/TIMER` | TIMG0~TIMG7 |
| ADC12 | `/ti/driverlib/ADC12` | PA25~PA28 等 |

---

*最后更新: 2026-06-11*
