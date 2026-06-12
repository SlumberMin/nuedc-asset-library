# 电赛常用DAC电路设计

> nuedc-asset-library · 电路模板库 · R-2R电阻串、电阻串DAC、电流舵DAC、Σ-Δ DAC、输出滤波电路

---

## 目录

1. [DAC基础概念](#1-dac基础概念)
2. [R-2R梯形电阻DAC](#2-r-2r梯形电阻dac)
3. [电阻串DAC](#3-电阻串dac)
4. [电流舵DAC](#4-电流舵dac)
5. [Σ-Δ DAC](#5-σ-δ-dac)
6. [PWM型DAC](#6-pwm型dac)
7. [输出滤波电路设计](#7-输出滤波电路设计)
8. [DAC芯片选型指南](#8-dac芯片选型指南)
9. [PCB设计要点](#9-pcb设计要点)
10. [电赛典型应用电路](#10-电赛典型应用电路)

---

## 1. DAC基础概念

### 1.1 DAC关键参数

| 参数 | 符号 | 含义 | 典型值 |
|------|------|------|--------|
| 分辨率 | N | 输出位数 | 8/10/12/16位 |
| 精度 | INL/DNL | 积分/微分非线性 | ±0.5 LSB |
| 建立时间 | ts | 输出达到终值±0.5LSB时间 | 1~10μs |
| 转换速率 | SR | 输出最大变化率 | 1~100V/μs |
| 输出范围 | Vout | 输出电压范围 | 0~Vref 或 ±Vref |
| 参考电压 | Vref | 基准电压 | 2.5V, 5V |
| 功耗 | P | 静态功耗 | 0.1~10mW |
| 接口 | - | 数字接口类型 | SPI, I2C, 并行 |

### 1.2 DAC架构对比

| 架构 | 分辨率 | 速度 | 精度 | 成本 | 应用 |
|------|--------|------|------|------|------|
| R-2R梯形 | 8~16位 | 中 | 中 | 低 | 通用 |
| 电阻串 | 8~16位 | 中 | 高 | 中 | 高精度 |
| 电流舵 | 8~16位 | 高 | 高 | 中 | 高速通信 |
| Σ-Δ | 16~24位 | 低 | 最高 | 中 | 音频/高精度 |
| PWM | 8~12位 | 低 | 低 | 最低 | 低成本 |

### 1.3 DAC输出类型

| 输出类型 | 特点 | 驱动能力 | 应用 |
|---------|------|---------|------|
| 电压输出 | 直接输出电压 | 弱（需缓冲） | 通用 |
| 电流输出 | 输出电流 | 需I-V转换 | 高速应用 |
| 差分输出 | 差分电压/电流 | 需差分转单端 | 抗干扰 |

---

## 2. R-2R梯形电阻DAC

### 2.1 原理

```
R-2R梯形网络（4位示例）：

        D3      D2      D1      D0
        │       │       │       │
        2R      2R      2R      2R
        │       │       │       │
Vref──┬─┤──R──┬─┤──R──┬─┤──R──┬─┤
      │ │     │ │     │ │     │ │
      R │     R │     R │     R │
      │ │     │ │     │ │     │ │
      ├─┘     ├─┘     ├─┘     ├─┘
      │       │       │       │
      └───────┴───────┴───────┴──→ Vout

输出电压：Vout = Vref × (D/2^N)
例：4位，Vref=5V，D=1010(10)
Vout = 5 × 10/16 = 3.125V
```

### 2.2 分立元件实现

```c
// 8位R-2R DAC电路
// 使用精度匹配的电阻（0.1%或更好）

// 元件清单：
// - R: 10kΩ × 9个（0.1%精度）
// - 2R: 20kΩ × 8个（0.1%精度，或用2个10kΩ串联）
// - 运放: OPA2188或AD8608（低偏移、低噪声）

// 电路连接：
// 1. R-2R网络输出接到运放同相输入端
// 2. 运放配置为电压跟随器（或同相放大器）
// 3. Vref使用精密基准（如REF3025）

// 优缺点：
// + 简单，元件少
// + 适合8位及以下分辨率
// - 精度受电阻匹配度限制
// - 位数增加时电阻数量线性增加
```

### 2.3 R-2R DAC精度分析

```
误差来源：
1. 电阻值误差：ΔR/R
2. 电阻匹配误差：|R1-R2|/Ravg
3. 运放偏移电压：Vos
4. 运放增益误差
5. 参考电压误差

精度估算（8位R-2R）：
- 电阻匹配精度0.1%时，INL ≈ ±0.5 LSB
- 电阻匹配精度0.5%时，INL ≈ ±2 LSB
- 需要更高精度时使用12位以上集成DAC

建议：
- 8位以下使用分立R-2R
- 10位以上使用集成DAC芯片
```

### 2.4 R-2R DAC驱动代码

```c
// GPIO驱动R-2R DAC
#define DAC_BITS 8
#define DAC_PORT GPIOA
#define DAC_MASK 0xFF  // PA0~PA7

void r2r_dac_init(void) {
    // 配置PA0~PA7为推挽输出
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                          GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void r2r_dac_write(uint8_t value) {
    // 直接写入GPIO
    DAC_PORT->ODR = (DAC_PORT->ODR & ~DAC_MASK) | value;
}

// 使用DMA输出波形
void r2r_dac_dma_output(uint8_t *waveform, uint16_t length, uint32_t sample_rate) {
    // 配置定时器触发DMA
    // 每个定时器周期自动将下一个采样值写入GPIO
    HAL_DMA_Start(&hdma, (uint32_t)waveform, (uint32_t)&DAC_PORT->ODR, length);
    HAL_TIM_Base_Start(&htim);
}
```

---

## 3. 电阻串DAC

### 3.1 原理

```
电阻串DAC（3位示例）：

Vref ──┤├──┤├──┤├──┤├──┤├──┤├──┤├── GND
       R   R   R   R   R   R   R
       │   │   │   │   │   │   │
       7   6   5   4   3   2   1   0
       │   │   │   │   │   │   │
       └───┴───┴───┼───┴───┴───┘
                   │
            多路选择器（MUX）
                   │
                  Vout

输出电压：Vout = Vref × D/2^N
每个节点电压递增 Vref/2^N

优点：
- 单调性保证（即使电阻有误差）
- 适合低分辨率高精度
- INL受电阻匹配度影响小

缺点：
- 位数增加时电阻数量指数增长（2^N个）
- 8位需要256个电阻
```

### 3.2 电阻串DAC实现

```c
// 集成电阻串DAC芯片（如AD5620）
// 通常通过SPI接口控制

// AD5620 12位串行DAC
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    float vref;
} AD5620;

void ad5620_init(AD5620 *dac, SPI_HandleTypeDef *hspi, 
                  GPIO_TypeDef *cs_port, uint16_t cs_pin, float vref) {
    dac->hspi = hspi;
    dac->cs_port = cs_port;
    dac->cs_pin = cs_pin;
    dac->vref = vref;
    
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

void ad5620_write(AD5620 *dac, uint16_t value) {
    uint8_t tx[3];
    tx[0] = 0x00; // 命令：写入并更新
    tx[1] = (value >> 4) & 0xFF;
    tx[2] = (value << 4) & 0xF0;
    
    HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dac->hspi, tx, 3, 100);
    HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_SET);
}

float ad5620_get_voltage(AD5620 *dac, uint16_t value) {
    return dac->vref * value / 4096.0f;
}
```

---

## 4. 电流舵DAC

### 4.1 原理

```
电流舵DAC（4位示例）：

        D3   D2   D1   D0
        │    │    │    │
      ┌─▼─┐┌─▼─┐┌─▼─┐┌─▼─┐
Iref──┤ 8I├┤ 4I├┤ 2I├┤ 1I├──→ Iout
      └───┘└───┘└───┘└───┘
        开关控制电流方向

每个电流源由对应数字位控制：
- Dn=1：电流源连接到Iout输出
- Dn=0：电流源连接到GND（或互补输出）

Iout = Iref × (D/2^N)

需要I-V转换：
Vout = Iout × Rf
```

### 4.2 电流舵DAC特点

| 特性 | 说明 |
|------|------|
| 速度 | 最快（可达GHz级） |
| 精度 | 高（良好的匹配性） |
| 分辨率 | 通常8~16位 |
| 功耗 | 中等 |
| 应用 | 通信、雷达、波形发生 |

### 4.3 电流舵DAC芯片驱动

```c
// AD9708 8位高速电流舵DAC
// 最大采样率125MSPS

typedef struct {
    // 并行数据接口
    GPIO_TypeDef *data_port;
    uint16_t data_mask;
    GPIO_TypeDef *clk_port;
    uint16_t clk_pin;
    GPIO_TypeDef *wrt_port;
    uint16_t wrt_pin;
    
    float iout_full_scale; // 满量程输出电流（通常2mA~20mA）
    float r_load;          // 负载电阻
} AD9708;

void ad9708_init(AD9708 *dac) {
    // 配置GPIO为高速推挽输出
    // 时钟和数据走线等长
}

void ad9708_write(AD9708 *dac, uint8_t value) {
    // 设置数据
    dac->data_port->ODR = (dac->data_port->ODR & ~dac->data_mask) | value;
    
    // 产生WRT脉冲
    HAL_GPIO_WritePin(dac->wrt_port, dac->wrt_pin, GPIO_PIN_SET);
    __NOP(); __NOP(); // 保持最小脉宽
    HAL_GPIO_WritePin(dac->wrt_port, dac->wrt_pin, GPIO_PIN_RESET);
}

// I-V转换电路
// 使用运放将电流输出转为电压
// Vout = -Iout × Rf
// Rf选择：考虑输出范围和带宽
```

### 4.4 电流舵DAC输出电路

```
电流舵DAC输出I-V转换：

                Rf (反馈电阻)
DAC Iout ──┬──/\/\/──┬── Vout
           │         │
           └──→ (-)  │
              [运放]──┘
           ──→ (+)
              │
              GND

Vout = -Iout × Rf

例：Iout = 2mA, Rf = 2.5kΩ
Vout = -2mA × 2.5kΩ = -5V

使用反相放大器配置：
- 输入阻抗低（电流输入）
- 输出阻抗低（电压输出）
- 带宽由运放决定
```

---

## 5. Σ-Δ DAC

### 5.1 原理

```
Σ-Δ DAC架构：

数字输入 ──→ [数字滤波器] ──→ [Σ-Δ调制器] ──→ [1位DAC] ──→ [模拟滤波器] ──→ 输出
             (插值+滤波)      (噪声整形)      (开关电容)     (低通滤波)

工作过程：
1. 数字滤波器对输入进行插值（提高采样率）
2. Σ-Δ调制器将高分辨率数据转为低分辨率（1位）高速数据
3. 量化噪声被整形到高频
4. 1位DAC简单精确（只有两个电平）
5. 模拟低通滤波器滤除高频噪声
```

### 5.2 Σ-Δ DAC关键参数

| 参数 | 说明 | 典型值 |
|------|------|--------|
| 过采样比 | OSR | 64~256 |
| 噪声整形阶数 | - | 1~5阶 |
| 有效位数 | ENOB | 16~24位 |
| 带宽 | - | DC~20kHz（音频） |
| SNR | 信噪比 | 90~120dB |
| THD | 总谐波失真 | -80~-100dB |

### 5.3 音频Σ-Δ DAC驱动

```c
// PCM5102A 32位音频DAC
// I2S接口，SNR=112dB

typedef struct {
    I2S_HandleTypeDef *hi2s;
    int16_t *buffer;
    uint16_t buffer_size;
} PCM5102A;

void pcm5102a_init(PCM5102A *dac, I2S_HandleTypeDef *hi2s) {
    dac->hi2s = hi2s;
    
    // PCM5102A不需要I2C配置
    // 上电后自动工作
    // 硬件引脚配置：
    // FMT = 0: I2S格式
    // XSMT = 1: 正常工作（非静音）
}

void pcm5102a_play(PCM5102A *dac, int16_t *audio, uint16_t samples) {
    // DMA方式播放
    HAL_I2S_Transmit_DMA(dac->hi2s, (uint16_t*)audio, samples);
}

// 正弦波生成（用于测试）
void generate_sine(int16_t *buffer, int samples, float freq, float sample_rate) {
    for (int i = 0; i < samples; i++) {
        float t = (float)i / sample_rate;
        buffer[i] = (int16_t)(32767 * sinf(2 * M_PI * freq * t));
    }
}
```

### 5.4 高精度Σ-Δ DAC

```c
// AD5791 20位高精度DAC
// SPI接口，±0.5 LSB INL

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port, *sync_port, *clr_port;
    uint16_t cs_pin, sync_pin, clr_pin;
    float vref_pos, vref_neg;
} AD5791;

void ad5791_init(AD5791 *dac) {
    // 复位
    HAL_GPIO_WritePin(dac->clr_port, dac->clr_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(dac->clr_port, dac->clr_pin, GPIO_PIN_SET);
    
    // 配置寄存器
    ad5791_write_reg(dac, 0x02, 0x000000); // 控制寄存器：正常模式
}

void ad5791_set_voltage(AD5791 *dac, float voltage) {
    // 计算DAC码值
    float range = dac->vref_pos - dac->vref_neg;
    float normalized = (voltage - dac->vref_neg) / range;
    uint32_t code = (uint32_t)(normalized * 1048575); // 20位
    
    if (code > 1048575) code = 1048575;
    
    // 写入DAC寄存器
    ad5791_write_reg(dac, 0x01, code);
}

void ad5791_write_reg(AD5791 *dac, uint8_t reg, uint32_t data) {
    uint8_t tx[4];
    tx[0] = reg;
    tx[1] = (data >> 16) & 0xFF;
    tx[2] = (data >> 8) & 0xFF;
    tx[3] = data & 0xFF;
    
    HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dac->hspi, tx, 4, 100);
    HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_SET);
}
```

---

## 6. PWM型DAC

### 6.1 原理

```
PWM DAC：

PWM信号 ──→ [RC低通滤波器] ──→ 模拟电压

占空比 D = Ton / Tperiod
平均电压 Vout = Vcc × D

例：Vcc=3.3V, 占空比=50%
Vout = 3.3 × 0.5 = 1.65V
```

### 6.2 PWM DAC电路

```
PWM输出 ──┬── R1 (10kΩ) ──┬── Vout
          │               │
          │              C1 (100nF)
          │               │
          │              GND
          │
         GPIO (PWM输出)

滤波器设计：
截止频率 fc = 1/(2π × R1 × C1)
fc = 1/(2π × 10kΩ × 100nF) = 159Hz

选择原则：
- fc << PWM频率（通常1kHz~100kHz）
- fc >> 信号带宽
- 纹波 = Vcc / (2^N × √(3) × fc × T)
```

### 6.3 PWM DAC高分辨率实现

```c
// PWM DAC实现
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t resolution;  // PWM分辨率（如1024 = 10位）
    float vref;
} PWM_DAC;

void pwm_dac_init(PWM_DAC *dac, TIM_HandleTypeDef *htim, uint32_t ch, 
                   uint32_t resolution, float vref) {
    dac->htim = htim;
    dac->channel = ch;
    dac->resolution = resolution;
    dac->vref = vref;
    
    // 设置PWM分辨率
    __HAL_TIM_SET_AUTORELOAD(htim, resolution - 1);
    HAL_TIM_PWM_Start(htim, ch);
}

void pwm_dac_set_value(PWM_DAC *dac, uint32_t value) {
    if (value >= dac->resolution) value = dac->resolution - 1;
    __HAL_TIM_SET_COMPARE(dac->htim, dac->channel, value);
}

void pwm_dac_set_voltage(PWM_DAC *dac, float voltage) {
    uint32_t value = (uint32_t)(voltage / dac->vref * dac->resolution);
    pwm_dac_set_value(dac, value);
}

// 提高PWM DAC精度：过采样
// 使用DMA在每个PWM周期微调占空比
// 例：10位PWM + 4倍过采样 = 12位等效精度
```

### 6.4 PWM DAC纹波分析

```
PWM DAC纹波计算：

一阶RC滤波器：
Vripple = Vcc × D × (1-D) / (1 + 2π×fc×RC)

对于高分辨率（D≈0.5时纹波最大）：
Vripple_max ≈ Vcc / (4 × 2π × fpwm × R × C)

减小纹波方法：
1. 提高PWM频率
2. 增大RC时间常数
3. 使用二阶或多阶滤波器
4. 过采样+数字滤波

例：Vcc=3.3V, fpwm=1kHz, R=10kΩ, C=10μF
Vripple = 3.3 / (4 × 2π × 1000 × 10000 × 0.00001)
Vripple ≈ 1.3mV
```

---

## 7. 输出滤波电路设计

### 7.1 一阶RC低通滤波器

```
最简单的DAC输出滤波：

DAC输出 ── R ──┬── Vout
               │
               C
               │
              GND

传递函数：H(s) = 1/(1+sRC)
截止频率：fc = 1/(2πRC)

设计示例（10kHz截止）：
R = 1.6kΩ, C = 10nF
fc = 1/(2π × 1600 × 10e-9) = 9.95kHz

优点：简单，成本低
缺点：滚降慢（-20dB/dec），高频抑制不足
```

### 7.2 二阶Sallen-Key低通滤波器

```
Sallen-Key拓扑（Butterworth响应）：

DAC输出 ── R1 ──┬── R2 ──┬── (+)    ┌── Vout
                │        │   [运放]──┘
               C1        ├── (-)
                │        C2   │
               GND       │   └──反馈──┘
                        GND

参数计算（Butterworth，Q=0.707）：
fc = 1/(2π√(R1×R2×C1×C2))
Q = √(R1×R2×C1×C2)/(C2×(R1+R2))

设计示例（10kHz Butterworth）：
R1 = R2 = 10kΩ
C1 = 1.5nF, C2 = 680pF
fc ≈ 10kHz, Q ≈ 0.707

传递函数：H(s) = 1/(s²R1R2C1C2 + s(R1C2+R2C2+R1C1(1-K)) + 1)
K=1（跟随器时）

优点：滚降-40dB/dec，过渡带陡峭
缺点：需要运放，有相位延迟
```

### 7.3 二阶MFB低通滤波器

```
多反馈（MFB）拓扑：

DAC输出 ── R1 ──┬── C1 ──┬── 输出
                │        │
               R2        │
                │       (-)
                ├── C2 ──[运放]
                │       (+)
               GND       │
                        GND

参数计算：
fc = 1/(2π√(R1×R2×C1×C2))
Q = √(R1×R2×C1×C2)/(C1×(R1+R2))
增益 K = -C1/C2

优点：反相输入，无共模问题
缺点：增益为负（需要反相）
```

### 7.4 有源滤波器设计实例

```c
// 有源滤波器参数计算工具
typedef struct {
    float fc;       // 截止频率
    float Q;        // 品质因数
    float gain;     // 增益
    float R1, R2, R3;
    float C1, C2;
} ActiveFilter;

// Sallen-Key 低通滤波器设计
void design_sallen_key_lp(ActiveFilter *f, float fc, float Q) {
    f->fc = fc;
    f->Q = Q;
    
    // 选择C1和计算C2
    float C1 = 10e-9; // 10nF（先选择一个值）
    float C2 = C1 / (4 * Q * Q); // Butterworth: C2 = C1/2
    
    // 计算电阻
    float w0 = 2 * M_PI * fc;
    float R = 1 / (w0 * sqrtf(C1 * C2));
    
    f->R1 = R;
    f->R2 = R;
    f->C1 = C1;
    f->C2 = C2;
}

// 计算实际截止频率
float calc_cutoff_freq(float R1, float R2, float C1, float C2) {
    return 1.0f / (2 * M_PI * sqrtf(R1 * R2 * C1 * C2));
}
```

### 7.5 滤波器阶数选择

| 阶数 | 滚降速率 | 适用场景 | 复杂度 |
|------|---------|---------|--------|
| 1阶 | -20dB/dec | 简单应用 | 最低 |
| 2阶 | -40dB/dec | 一般应用 | 低 |
| 3阶 | -60dB/dec | 较高要求 | 中 |
| 4阶 | -80dB/dec | 高精度应用 | 高 |
| 5阶+ | -100dB/dec+ | 专业应用 | 很高 |

```
滤波器类型选择：
- Butterworth：最平坦通带，过渡带适中
- Chebyshev I型：过渡带陡峭，通带有纹波
- Chebyshev II型：过渡带陡峭，阻带有纹波
- Bessel：线性相位，群延迟恒定，过渡带缓
- Elliptic：过渡带最陡，通带和阻带都有纹波
```

---

## 8. DAC芯片选型指南

### 8.1 选型决策表

| 需求 | 推荐方案 | 推荐芯片 |
|------|---------|---------|
| 低成本8位 | PWM DAC | MCU内置PWM |
| 通用12位 | 集成DAC | MCP4921, AD5620 |
| 高速波形 | 电流舵DAC | AD9708, DAC908 |
| 高精度16位 | Σ-Δ DAC | AD5660, DAC8551 |
| 超高精度20位 | 高精度DAC | AD5791, DAC1282 |
| 音频输出 | 音频DAC | PCM5102A, CS4344 |
| 多通道 | 多通道DAC | MCP4728(4ch), AD5754(4ch) |

### 8.2 电赛常用DAC芯片

| 型号 | 分辨率 | 通道 | 接口 | 速度 | 价格 | 特点 |
|------|--------|------|------|------|------|------|
| MCP4921 | 12位 | 1 | SPI | - | ¥5 | 简单易用 |
| MCP4922 | 12位 | 2 | SPI | - | ¥8 | 双通道 |
| MCP4728 | 12位 | 4 | I2C | - | ¥10 | 四通道，内部EEPROM |
| AD5620 | 12位 | 1 | SPI | - | ¥15 | 高精度 |
| DAC8551 | 16位 | 1 | SPI | - | ¥20 | 16位高精度 |
| AD9708 | 8位 | 1 | 并行 | 125MSPS | ¥30 | 高速波形 |
| DAC908 | 8位 | 1 | 并行 | 165MSPS | ¥40 | 超高速 |
| PCM5102A | 32位 | 2 | I2S | 384kHz | ¥15 | 音频DAC |
| AD5791 | 20位 | 1 | SPI | - | ¥200 | 超高精度 |

### 8.3 DAC接口电路

```
SPI接口DAC连接（以MCP4921为例）：

MCU                    MCP4921
────                   ───────
SPI_SCK ──────────────→ SCK
SPI_MOSI ─────────────→ SDI
GPIO_CS ──────────────→ CS
GPIO_LDAC ────────────→ LDAC (可选)
                        VOUT ──→ 输出

SPI时序：
CS下降沿开始传输
16位数据：4位命令 + 12位数据
MSB先发
CS上升沿锁存数据

I2S接口DAC连接（以PCM5102A为例）：

MCU                    PCM5102A
────                   ────────
I2S_SCK ──────────────→ BCK
I2S_WS ───────────────→ LRCK
I2S_SD ───────────────→ DIN
                        VOUTL ──→ 左声道输出
                        VOUTR ──→ 右声道输出
```

---

## 9. PCB设计要点

### 9.1 DAC电源设计

```
电源设计原则：
1. 模拟电源(AVDD)和数字电源(DVDD)分开
2. AGND和DGND单点连接
3. 每个电源引脚加100nF+10μF去耦
4. 电源走线尽量粗

电源连接图：
VCC ──→ LDO ──→ AVDD ──┬── 100nF ──┬── AGND
                        │          │
                        └── 10μF ──┘

VCC ──→ LDO ──→ DVDD ──┬── 100nF ──┬── DGND
                        │          │
                        └── 10μF ──┘

AGND与DGND单点连接（在DAC芯片下方）
```

### 9.2 接地设计

```
接地策略：
1. 模拟地和数字地分开铺铜
2. 在DAC芯片正下方单点连接
3. 使用磁珠或0Ω电阻连接AGND和DGND
4. 高精度DAC使用4层板，独立地层

PCB布局示例：
┌────────────────────────────────┐
│  模拟区域      │  数字区域      │
│                │               │
│  ┌────────┐    │               │
│  │  DAC   │    │    MCU        │
│  │        │    │               │
│  └────────┘    │               │
│       │        │               │
│    AGND铜皮    │   DGND铜皮    │
│                │               │
└────────────────┴───────────────┘
           │单点连接│
           └───────┘
```

### 9.3 信号完整性

```
高速DAC信号完整性要求：
1. SPI/I2S走线等长
2. 时钟信号远离模拟输出
3. 输出走线短且阻抗匹配
4. 差分输出走线等长等距

布局建议：
- DAC输出走线直接到滤波器
- 滤波器输出到连接器走线短
- 避免输出走线跨越地分割
- 使用包地保护关键信号
```

---

## 10. 电赛典型应用电路

### 10.1 信号发生器

```c
// 基于MCP4922的双通道信号发生器
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    float vref;
    uint16_t waveform_a[256]; // 波形表A
    uint16_t waveform_b[256]; // 波形表B
    uint16_t phase_a, phase_b;
    uint16_t freq_a, freq_b;
} SignalGenerator;

void sig_gen_init(SignalGenerator *gen) {
    // 生成正弦波表
    for (int i = 0; i < 256; i++) {
        gen->waveform_a[i] = (uint16_t)(2047 + 2047 * sinf(2 * M_PI * i / 256));
        gen->waveform_b[i] = gen->waveform_a[i];
    }
}

void sig_gen_set_freq(SignalGenerator *gen, uint8_t channel, float freq) {
    // 频率控制字 = freq × 256 / sample_rate
    if (channel == 0) {
        gen->freq_a = (uint16_t)(freq * 256 / SAMPLE_RATE);
    } else {
        gen->freq_b = (uint16_t)(freq * 256 / SAMPLE_RATE);
    }
}

// 定时器中断中更新输出
void sig_gen_update(SignalGenerator *gen) {
    // 更新相位
    gen->phase_a += gen->freq_a;
    gen->phase_b += gen->freq_b;
    
    // 查表输出
    uint8_t idx_a = gen->phase_a >> 8;
    uint8_t idx_b = gen->phase_b >> 8;
    
    // 写入DAC
    mcp4922_write(gen, 0, gen->waveform_a[idx_a]); // 通道A
    mcp4922_write(gen, 1, gen->waveform_b[idx_b]); // 通道B
}
```

### 10.2 精密电压源

```c
// 基于AD5791的精密电压源
typedef struct {
    AD5791 dac;
    float voltage;         // 当前输出电压
    float vref_pos;        // 正参考电压
    float vref_neg;        // 负参考电压
    float resolution;      // 电压分辨率
} PrecisionVoltageSource;

void pvs_init(PrecisionVoltageSource *pvs, float vref_pos, float vref_neg) {
    ad5791_init(&pvs->dac);
    pvs->vref_pos = vref_pos;
    pvs->vref_neg = vref_neg;
    pvs->resolution = (vref_pos - vref_neg) / 1048576.0f; // 20位
}

float pvs_set_voltage(PrecisionVoltageSource *pvs, float target) {
    // 限幅
    if (target > pvs->vref_pos) target = pvs->vref_pos;
    if (target < pvs->vref_neg) target = pvs->vref_neg;
    
    // 设置DAC
    ad5791_set_voltage(&pvs->dac, target);
    pvs->voltage = target;
    
    return target;
}

// 电压步进测试
void pvs_sweep(PrecisionVoltageSource *pvs, float start, float stop, float step) {
    for (float v = start; v <= stop; v += step) {
        pvs_set_voltage(pvs, v);
        HAL_Delay(100); // 等待建立
        // 可以在此读取ADC验证输出
    }
}
```

### 10.3 音频DAC应用

```c
// 基于PCM5102A的音频播放器
typedef struct {
    PCM5102A dac;
    int16_t audio_buffer[4096]; // 双缓冲
    uint16_t buffer_size;
    uint8_t buffer_index;
    volatile uint8_t buffer_ready;
} AudioPlayer;

void audio_play_sine(AudioPlayer *player, float freq, float amplitude, 
                      float duration) {
    int samples = (int)(SAMPLE_RATE * duration);
    
    for (int i = 0; i < samples; i++) {
        float t = (float)i / SAMPLE_RATE;
        float value = amplitude * sinf(2 * M_PI * freq * t);
        
        // 左右声道相同
        player->audio_buffer[i * 2] = (int16_t)(value * 32767);
        player->audio_buffer[i * 2 + 1] = (int16_t)(value * 32767);
    }
    
    // DMA播放
    pcm5102a_play(&player->dac, player->audio_buffer, samples * 2);
}

// 音频文件播放（从SD卡读取）
void audio_play_from_sd(AudioPlayer *player, const char *filename) {
    // 1. 打开WAV文件
    // 2. 解析WAV头
    // 3. 双缓冲DMA读取和播放
    // 4. 处理播放完成和循环
}
```

### 10.4 DDS波形发生器

```c
// DDS（直接数字频率合成）波形发生器
typedef struct {
    uint32_t phase_acc;     // 相位累加器
    uint32_t freq_word;     // 频率控制字
    uint32_t sample_rate;   // 采样率
    uint16_t *wave_table;   // 波形查找表
    uint16_t table_size;    // 表大小
    DAC_HandleTypeDef *hdac;
} DDS_Generator;

void dds_init(DDS_Generator *dds, uint32_t sample_rate, uint16_t *table, 
               uint16_t table_size) {
    dds->phase_acc = 0;
    dds->freq_word = 0;
    dds->sample_rate = sample_rate;
    dds->wave_table = table;
    dds->table_size = table_size;
}

void dds_set_freq(DDS_Generator *dds, float freq) {
    // 频率控制字 = freq × 2^32 / sample_rate
    dds->freq_word = (uint32_t)(freq * 4294967296.0f / dds->sample_rate);
}

// 定时器中断中调用
uint16_t dds_update(DDS_Generator *dds) {
    dds->phase_acc += dds->freq_word;
    
    // 取相位累加器高位作为表索引
    uint16_t index = dds->phase_acc >> (32 - 16); // 16位表大小
    index = index % dds->table_size;
    
    return dds->wave_table[index];
}

// 生成波形表
void dds_gen_sine_table(uint16_t *table, int size) {
    for (int i = 0; i < size; i++) {
        float angle = 2 * M_PI * i / size;
        table[i] = (uint16_t)(2047 + 2047 * sinf(angle)); // 12位DAC
    }
}

void dds_gen_triangle_table(uint16_t *table, int size) {
    for (int i = 0; i < size; i++) {
        if (i < size / 2) {
            table[i] = (uint16_t)(4095 * i / (size / 2));
        } else {
            table[i] = (uint16_t)(4095 * (size - i) / (size / 2));
        }
    }
}

void dds_gen_square_table(uint16_t *table, int size) {
    for (int i = 0; i < size; i++) {
        table[i] = (i < size / 2) ? 4095 : 0;
    }
}
```

### 10.5 PID控制DAC输出

```c
// DAC作为PID控制输出
typedef struct {
    DAC_HandleTypeDef *hdac;
    float Kp, Ki, Kd;
    float integral, prev_error;
    float output_min, output_max;
    uint16_t dac_max;
} PID_DAC_Controller;

float pid_dac_update(PID_DAC_Controller *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    
    pid->integral += error;
    // 积分限幅
    if (pid->integral > pid->output_max) pid->integral = pid->output_max;
    if (pid->integral < pid->output_min) pid->integral = pid->output_min;
    
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // 输出限幅
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    // 转换为DAC值
    uint16_t dac_value = (uint16_t)((output - pid->output_min) / 
                          (pid->output_max - pid->output_min) * pid->dac_max);
    
    HAL_DAC_SetValue(pid->hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
    
    return output;
}
```

---

## 参考资料

1. 《数据转换器应用设计》- 科学出版社
2. 《模拟与数字转换器手册》
3. ADI DAC选型指南
4. TI数据转换器应用笔记
5. MCP4921/MCP4922 Datasheet
6. AD5791 Datasheet
7. PCM5102A Datasheet

---

*文档版本: v1.0 | 最后更新: 2025-01*
