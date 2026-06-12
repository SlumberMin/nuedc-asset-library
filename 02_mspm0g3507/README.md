# MSPM0G3507 电赛通用代码库

> 全国大学生电子设计竞赛 · TI MSPM0G3507 专用驱动库

---

## 芯片概览

| 参数 | 值 |
|------|-----|
| 内核 | ARM Cortex-M0+ |
| 主频 | 80 MHz |
| Flash | 128 KB |
| SRAM | 32 KB |
| ADC | 12-bit, 4 MSPS |
| PWM | TIMG0~TIMG7, 最高 80MHz 输入时钟 |
| UART | 4 路 (UART0~UART3) |
| I2C | 2 路 (I2C0~I2C1) |
| SPI | 2 路 (SPI0~SPI1) |
| GPIO | 最多 60 个 |
| 封装 | LQFP-64 (G3507) |

---

## 目录结构

```
02_mspm0g3507/
├── platform/
│   └── driverlib_mspm0.h              # DriverLib 封装宏
├── drivers/
│   ├── tb6612.c/.h                    # TB6612 电机驱动
│   ├── l298n.c/.h                     # L298N 电机驱动
│   ├── motor_mspm0.h/.c               # 通用电机驱动（TB6612FNG）
│   ├── servo_mspm0.h                  # 舵机驱动（50Hz PWM）
│   ├── encoder_mspm0.c/.h             # 定时器 QEI 编码器
│   ├── encoder_gpio_mspm0.c/.h        # GPIO 中断编码器（N20 电机推荐）
│   ├── encoder.c/.h                   # 编码器通用接口
│   ├── jy901s_mspm0.c/.h             # JY901S 九轴 IMU (UART)
│   ├── jy901s.c/.h                    # JY901S 通用接口
│   ├── grayscale_mspm0.c/.h           # 感为 8 路灰度传感器 (ADC)
│   ├── grayscale.c/.h                 # 灰度传感器通用接口
│   ├── ultrasonic_mspm0.c/.h          # SR04/US-016 超声波传感器
│   ├── ultrasonic.c/.h                # 超声波通用接口
│   ├── tcs34725.c/.h                  # TCS34725 颜色传感器 (I2C)
│   ├── bluetooth_hc05_mspm0.c/.h      # HC-05 蓝牙模块 (UART)
│   ├── bluetooth.c/.h                 # 蓝牙通用接口
│   ├── oled_ssd1306_mspm0.c/.h        # SSD1306 OLED 显示 (I2C)
│   ├── oled_ssd1306.c/.h              # OLED 通用接口
│   ├── at24c02.c/.h                   # AT24C02 EEPROM (I2C)
│   ├── pca9685.c/.h                   # PCA9685 16 路舵机驱动 (I2C)
│   └── sensor_ir_mspm0.h/.c           # 红外循迹传感器
├── examples/
│   ├── motor_encoder_test.c           # 电机+编码器测试
│   ├── motor_encoder.syscfg           # SysConfig 配置
│   └── motor_encoder_pid_example.c    # PID 闭环控制示例
├── tests/
│   ├── test_communication.c           # 通信测试
│   ├── test_ultrasonic.c              # 超声波测试
│   ├── test_grayscale.c               # 灰度传感器测试
│   ├── test_jy901s.c                  # IMU 测试
│   └── test_encoder_gpio.c            # GPIO 编码器测试
├── src/
│   └── l298n_test.syscfg              # L298N SysConfig 配置
├── API速查表.md                        # 驱动 API 速查
├── STM32_vs_MSPM0_PLATFORM_DIFF.md    # 平台差异对照表
└── README.md                          # 本文件
```

---

## 已适配器件清单

| 序号 | 器件 | 驱动文件 | 通信接口 | 用途 |
|------|------|---------|---------|------|
| 1 | TB6612FNG | `tb6612.c/.h` | PWM+GPIO | 双路直流电机驱动 |
| 2 | L298N | `l298n.c/.h` | PWM+GPIO | 双路直流电机驱动（大电流） |
| 3 | SG90 舵机 | `servo_mspm0.h` | PWM | 舵机角度控制 |
| 4 | N20 编码电机 | `encoder_gpio_mspm0.c/.h` | GPIO 中断 | 速度/位置反馈 |
| 5 | JY901S IMU | `jy901s_mspm0.c/.h` | UART | 姿态角/加速度/角速度 |
| 6 | 感为 8 路灰度 | `grayscale_mspm0.c/.h` | ADC+GPIO | 循迹/路径检测 |
| 7 | SR04 超声波 | `ultrasonic_mspm0.c/.h` | GPIO Timer | 距离测量/避障 |
| 8 | TCS34725 | `tcs34725.c/.h` | I2C | 颜色识别 |
| 9 | HC-05 蓝牙 | `bluetooth_hc05_mspm0.c/.h` | UART | 无线通信 |
| 10 | SSD1306 OLED | `oled_ssd1306_mspm0.c/.h` | I2C | 显示调试信息 |
| 11 | AT24C02 | `at24c02.c/.h` | I2C | 参数存储 |
| 12 | PCA9685 | `pca9685.c/.h` | I2C | 16 路舵机扩展驱动 |

---

## 快速开始

### 1. 开发环境

- **IDE**: Code Composer Studio (CCS) 12+
- **SDK**: MSPM0 SDK 2.04+ (ti.com 下载)
- **配置工具**: SysConfig 1.22+（图形化引脚/外设配置）
- **调试器**: XDS110 (LaunchPad 自带)

### 2. 添加到工程

1. 将 `platform/` 和 `drivers/` 复制到工程目录
2. 在 SysConfig 中配置引脚映射
3. `#include` 对应头文件即可使用

### 3. 基本用法

```c
#include "ti_msp_dl_config.h"      // SysConfig 生成
#include "drivers/tb6612.h"
#include "drivers/encoder_gpio_mspm0.h"

int main(void)
{
    SYSCFG_DL_init();  // SysConfig 初始化

    // 初始化电机
    TB6612_Init(&tb6612_cfg);
    TB6612_SetSpeed(TB6612_CH_A, 500);  // 正转 50%

    // 初始化编码器
    EncoderGpio_Init(&enc_cfg);

    while (1) {
        int32_t speed = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT);
        DELAY_MS(10);
    }
}
```

---

## 驱动详细说明

### TB6612 电机驱动

双路 H 桥驱动，最大持续电流 1.2A，峰值 3.2A。

```c
TB6612_Config cfg = {
    .ch_a = { .in1_port=GPIOA, .in1_pin=DL_GPIO_PIN_0, .in2_port=GPIOA, .in2_pin=DL_GPIO_PIN_1,
              .pwm_timer=TIMG0, .pwm_ch=DL_TIMER_CC_0_INDEX, .pwm_period=2000 },
    .ch_b = { /* 同上 */ }
};
TB6612_Init(&cfg);
TB6612_SetSpeed(TB6612_CH_A, 800);   // 正转 80%
TB6612_SetSpeed(TB6612_CH_A, -500);  // 反转 50%
TB6612_Brake(TB6612_CH_A);           // 刹车
```

### L298N 电机驱动

大电流 H 桥驱动，最大持续电流 2A。

```c
L298N_Init(&l298n_cfg);
L298N_SetSpeed(L298N_CH_A, 600);
L298N_Brake(L298N_CH_A);
```

### SG90 舵机

50Hz PWM 控制，脉宽 500~2500μs 对应 0~180°。

```c
Servo_Init(TIMG6, DL_TIMER_CC_0_INDEX);
Servo_SetAngle(90);       // 居中
Servo_SetAngle(45);       // 左转
Servo_SetRange(500, 2500); // 校准
```

### GPIO 编码器 (N20 电机推荐)

四倍频解码，支持方向检测。

```c
EncoderGpioConfig cfg[ENCODER_GPIO_MAX] = {
    [ENCODER_GPIO_LEFT]  = { .port=GPIOB, .pin_a=DL_GPIO_PIN_0, .pin_b=DL_GPIO_PIN_1, .inverted=0 },
    [ENCODER_GPIO_RIGHT] = { .port=GPIOB, .pin_a=DL_GPIO_PIN_2, .pin_b=DL_GPIO_PIN_3, .inverted=1 }
};
EncoderGpio_Init(cfg);

int32_t count = EncoderGpio_Read(ENCODER_GPIO_LEFT);   // 累计脉冲
int32_t speed = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT); // 脉冲/周期
EncoderGpio_Reset(ENCODER_GPIO_LEFT);
```

### JY901S IMU

九轴惯性测量单元，UART 输出姿态角。

```c
JY901S_Config cfg = { .uart=UART_0_INST, .baudrate=9600, .auto_calib=1 };
JY901S_Init(&cfg);

while (1) {
    if (JY901S_IsDataReady()) {
        JY901S_ClearDataReady();
        float p, r, y;
        JY901S_GetAngle(&p, &r, &y);
    }
}
```

### 感为 8 路灰度传感器

ADC 采集 + 8 选 1 模拟开关。

```c
Grayscale_Init(&gray_cfg);
Grayscale_Calibrate(white_cal, black_cal);

Grayscale_Read();
int16_t error = Grayscale_GetTrackError();  // 加权偏差
uint8_t off = Grayscale_IsOffTrack();       // 脱线检测
```

### SR04 超声波

```c
Ultrasonic_Init(&ultra_cfg);
float dist = Ultrasonic_Measure();        // cm，失败返回 -1
float avg  = Ultrasonic_MeasureAvg(5);    // 5 次平均
```

### TCS34725 颜色传感器

```c
TCS34725_Init(&tcs_cfg);
uint8_t r, g, b;
TCS34725_GetRGB(&r, &g, &b);
uint8_t color = TCS34725_GetColor();  // COLOR_RED/GREEN/BLUE/WHITE/BLACK
```

### HC-05 蓝牙

```c
BT_HC05_Init(&bt_cfg);
if (BT_HC05_IsConnected()) {
    BT_HC05_SendString("Hello!\n");
    if (BT_HC05_IsDataReceived()) {
        uint8_t buf[64];
        uint16_t len = BT_HC05_GetReceivedData(buf, sizeof(buf));
        BT_HC05_ClearRxBuffer();
    }
}
```

### SSD1306 OLED

```c
OLED_Init(I2C_0_INST);
OLED_Clear();
OLED_ShowString(0, 0, "Speed:", 16, 1);
OLED_ShowNum(0, 16, 1234, 4, 16, 1);
OLED_Refresh();
```

### AT24C02 EEPROM

```c
AT24C02_Init(I2C_0_INST);
AT24C02_WriteByte(0x00, 0xAA);
uint8_t val = AT24C02_ReadByte(0x00);
// 批量读写
AT24C02_Write(0x10, data_buf, 16);
AT24C02_Read(0x10, read_buf, 16);
```

### PCA9685 16 路舵机驱动

```c
PCA9685_Init(&pca_cfg);
PCA9685_SetFreq(50);                      // 50Hz 舵机模式
PCA9685_SetServoAngle(0, 90);             // 通道 0，角度 90°
PCA9685_SetPWM(1, 0, 2048);              // 通道 1，直接 PWM
```

---

## 系统集成示例

### 智能小车完整架构

```c
#include "ti_msp_dl_config.h"
#include "drivers/tb6612.h"
#include "drivers/encoder_gpio_mspm0.h"
#include "drivers/grayscale_mspm0.h"
#include "drivers/ultrasonic_mspm0.h"
#include "drivers/bluetooth_hc05_mspm0.h"
#include "drivers/oled_ssd1306_mspm0.h"

// PID 控制器
typedef struct { float kp, ki, kd, integral, prev_error, output_max; } PID;
float PID_Calc(PID *p, float target, float actual) {
    float err = target - actual;
    p->integral += err;
    float deriv = err - p->prev_error;
    p->prev_error = err;
    float out = p->kp*err + p->ki*p->integral + p->kd*deriv;
    if (out > p->output_max) out = p->output_max;
    if (out < -p->output_max) out = -p->output_max;
    return out;
}

int main(void)
{
    SYSCFG_DL_init();

    // 初始化所有外设
    TB6612_Init(&motor_cfg);
    EncoderGpio_Init(&enc_cfg);
    Grayscale_Init(&gray_cfg);
    Ultrasonic_Init(&ultra_cfg);
    BT_HC05_Init(&bt_cfg);
    OLED_Init(I2C_0_INST);

    PID pid_track = { .kp=15, .ki=0.1, .kd=8, .output_max=400 };
    PID pid_speed_l = { .kp=5, .ki=0.5, .kd=0, .output_max=1000 };
    PID pid_speed_r = { .kp=5, .ki=0.5, .kd=0, .output_max=1000 };

    int16_t base_speed = 400;

    while (1) {
        // 1. 读取传感器
        Grayscale_Read();
        float distance = Ultrasonic_Measure();
        int32_t spd_l = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT);
        int32_t spd_r = EncoderGpio_GetSpeed(ENCODER_GPIO_RIGHT);

        // 2. 避障
        if (distance > 0 && distance < 15.0f) {
            TB6612_Brake(TB6612_CH_A);
            TB6612_Brake(TB6612_CH_B);
            continue;
        }

        // 3. 循迹 PID
        float track_err = (float)Grayscale_GetTrackError();
        float track_out = PID_Calc(&pid_track, 0, track_err);

        // 4. 速度闭环
        float out_l = PID_Calc(&pid_speed_l, base_speed + track_out, (float)spd_l);
        float out_r = PID_Calc(&pid_speed_r, base_speed - track_out, (float)spd_r);

        TB6612_SetSpeed(TB6612_CH_A, (int16_t)out_l);
        TB6612_SetSpeed(TB6612_CH_B, (int16_t)out_r);

        // 5. OLED 显示
        OLED_Clear();
        OLED_ShowString(0, 0, "Track:", 16, 1);
        OLED_ShowNum(48, 0, (int32_t)track_err, 4, 16, 1);
        OLED_ShowString(0, 16, "Dist:", 16, 1);
        OLED_ShowFloat(40, 16, distance, 3, 1, 16, 1);
        OLED_Refresh();

        // 6. 蓝牙上报
        if (BT_HC05_IsConnected()) {
            char msg[32];
            // sprintf(msg, "%ld,%ld\n", spd_l, spd_r);
            // BT_HC05_SendString(msg);
        }

        DELAY_MS(5);
    }
}
```

---

## 常见问题解答 (FAQ)

### Q1: 编译报错 "Cannot open source file: ti_msp_dl_config.h"

**原因**: SysConfig 未生成配置文件

**解决**:
1. 确认 `.syscfg` 文件存在于工程中
2. 右键 `.syscfg` 文件 → SysConfig → Generate Output
3. 确认 `build/ti_msp_dl_config.h` 已生成
4. 重新编译

### Q2: 编码器读数为 0 或方向反了

**原因**: GPIO 中断未配置或接线反了

**解决**:
1. 在 SysConfig 中为编码器引脚启用 GPIO 中断（双边沿）
2. 确认 `pin_a` 和 `pin_b` 接线正确
3. 尝试 `EncoderGpio_SetInverted(ch, 1)` 翻转方向
4. 确认中断优先级合理（不要被其他高优先级中断阻塞）

### Q3: 舵机抖动

**原因**: PWM 频率不稳或电源不足

**解决**:
1. 确认 PWM 频率为 50Hz（周期 20ms）
2. 舵机独立供电，不要从 LaunchPad 取电
3. 增加 100μF 电容滤波
4. 使用 `Servo_SetRange()` 校准脉宽范围

### Q4: 超声波测距不准或无返回

**原因**: 触发信号时序或接线问题

**解决**:
1. 确认 Trig 引脚输出、Echo 引脚输入配置正确
2. 测量前确保上一个测量周期已完成（至少 60ms 间隔）
3. 检查供电电压（SR04 需 5V）
4. 增大 `filter_size` 参数进行中值滤波

### Q5: I2C 设备通信失败

**原因**: 上拉电阻缺失或地址错误

**解决**:
1. SCL/SDA 线上加 4.7kΩ 上拉电阻到 3.3V
2. 确认 I2C 地址正确（7-bit 格式）
3. 降低 I2C 时钟频率（100kHz 起步）
4. 用逻辑分析仪抓波形确认

### Q6: UART 接收丢数据

**原因**: 中断处理不及时或缓冲区溢出

**解决**:
1. 在 SysConfig 中启用 UART RX 中断和 FIFO
2. 中断服务函数中尽快读取数据
3. 增大软件接收缓冲区
4. 降低波特率测试

### Q7: ADC 采样值跳动大

**原因**: 参考电压不稳或采样时间太短

**解决**:
1. 增加采样电容（100nF 在 ADC 输入引脚）
2. 多次采样取平均
3. 确认 ADC 时钟配置合理
4. 避免数字信号线靠近模拟输入线

### Q8: MSPM0 没有 FPU，浮点运算会不会太慢？

**说明**: Cortex-M0+ 确实没有硬件 FPU，软件浮点运算耗时约 10~50μs。

**优化建议**:
1. PID 控制等实时任务使用定点数（Q16.16 格式）
2. 将浮点乘法转换为整数乘法 + 移位
3. 避免在中断中使用浮点运算
4. 姿态解算等复杂运算放在主循环，控制周期适当放宽

---

## 最佳实践

### 1. SysConfig 配置规范

- 每个外设单独配置，不要混在一个实例中
- 引脚分配记录在 SysConfig 注释中
- 修改引脚后必须重新生成代码
- `.syscfg` 文件纳入版本管理

### 2. 中断管理

- Cortex-M0+ 仅 4 级优先级（2-bit）
- 编码器中断设为最高优先级
- UART 接收中断次之
- Timer 中断用于周期性任务
- 避免在中断中做复杂运算

### 3. 电源设计

- LaunchPad 3.3V 仅供电给 MCU 和小信号器件
- 电机/舵机必须独立供电（7.4V 锂电池 + 稳压模块）
- 共地连接：所有模块 GND 必须连在一起
- 大功率器件并联 100~470μF 电解电容

### 4. 代码规范

- 每个驱动模块独立 `.c/.h` 文件
- 使用 `#ifndef` 头文件保护
- 配置结构体传参，避免硬编码引脚
- 关键函数添加中文注释（功能、参数、返回值）

### 5. 调试技巧

- UART0 作为调试串口（printf 重定向）
- OLED 实时显示关键变量
- 蓝牙无线调试（避免线缆束缚）
- 用逻辑分析仪抓 I2C/SPI/UART 波形

### 6. 性能优化

- 主循环周期控制在 1~10ms
- 传感器读取和控制计算分离
- 使用 DMA 减少 CPU 占用（ADC、UART）
- 避免频繁 `printf`（阻塞且耗时）

---

## 平台差异（对比 STM32）

| 特性 | STM32F103 | MSPM0G3507 |
|------|-----------|------------|
| 内核 | Cortex-M3 | Cortex-M0+ |
| 主频 | 72 MHz | 80 MHz |
| FPU | 无 | 无 |
| 中断优先级 | 16 级 (4-bit) | 4 级 (2-bit) |
| HAL 库 | STM32 HAL/LL | TI DriverLib |
| 配置工具 | CubeMX | SysConfig |
| PWM 配置 | TIM_Init() 结构体 | SysConfig 图形化 |
| ADC 配置 | ADC_Init() 结构体 | SysConfig 图形化 |
| GPIO 配置 | GPIO_Init() 结构体 | SysConfig 图形化 |

详细差异见 `STM32_vs_MSPM0_PLATFORM_DIFF.md`。

---

## 相关资源

- [MSPM0 SDK 下载](https://ti.com/tool/MSPM0-SDK)
- [MSPM0G3507 数据手册](https://ti.com/lit/ds/symlink/mspm0g3507.pdf)
- [DriverLib API 文档](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/)
- [SysConfig 使用指南](https://ti.com/tool/SYSCONFIG)
- [LaunchPad 购买](https://ti.com/tool/LP-MSPM0G3507)

---

*最后更新: 2026-06-11*
