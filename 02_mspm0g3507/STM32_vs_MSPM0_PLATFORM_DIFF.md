# STM32 vs MSPM0G3507 平台差异对比表

> 从 STM32F103/F407 迁移到 MSPM0G3507 的关键差异

## 1. 核心架构

| 特性 | STM32F103 (Cortex-M3) | STM32F407 (Cortex-M4F) | MSPM0G3507 (Cortex-M0+) |
|------|----------------------|------------------------|--------------------------|
| 内核 | Cortex-M3 | Cortex-M4F | Cortex-M0+ |
| 主频 | 72 MHz | 168 MHz | **80 MHz** |
| FPU | 无 | 有 (单精度) | **无** |
| DSP指令 | 无 | 有 | **无** |
| Flash | 64-512KB | 512KB-1MB | **128KB** |
| SRAM | 20-64KB | 128-192KB | **32KB** |
| 中断优先级 | 16级 (4-bit) | 16级 (4-bit) | **4级 (2-bit)** |

## 2. 时钟系统

| 特性 | STM32 | MSPM0G3507 |
|------|-------|------------|
| HSE | 外部晶振 8MHz | 内部 32MHz 振荡器 (无需外部晶振) |
| PLL | HSE×9=72MHz | SYSPLL: 32MHz→80MHz |
| 外设时钟 | APB1(36MHz)/APB2(72MHz) | **BUSCLK 40MHz** |
| 配置方式 | RCC 寄存器操作 | **SysConfig 自动生成 + DL_init** |

```c
// STM32
RCC->CR |= RCC_CR_HSEON;
RCC->CFGR = RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9;

// MSPM0G3507 — SysConfig 自动完成
SYSCFG_DL_init();  // 一行搞定
```

## 3. GPIO 差异

| 特性 | STM32 | MSPM0G3507 |
|------|-------|------------|
| 端口 | GPIOA~GPIOG | **GPIOA, GPIOB** (仅2组) |
| 引脚复用 | AFIO + 寄存器配置 | **SysConfig 引脚映射** |
| 输出速度 | 2/10/50MHz | 固定 (无需配置) |
| 操作方式 | `HAL_GPIO_WritePin()` / 寄存器 | **`DL_GPIO_setPins()`** |

```c
// STM32
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

// MSPM0G3507
DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0);
// 或用封装宏:
GPIO_SET(GPIOA, DL_GPIO_PIN_0);
```

## 4. PWM (定时器)

| 特性 | STM32 | MSPM0G3507 |
|------|-------|------------|
| 定时器 | TIM1~TIM14 | **TIMG0~TIMG7, TIMA0/TIMA1** |
| PWM 模式 | PWM1/PWM2 | **Edge-aligned / Center-aligned** |
| 最大频率 | 72MHz (TIM1) | **80MHz (TIMG)** |
| 通道数 | 每定时器4通道 | **每定时器4通道 (CC0~CC3)** |
| 配置方式 | CubeMX 或寄存器 | **SysConfig + DL_Timer** |

```c
// STM32
htim3.Instance->CCR1 = duty;
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);

// MSPM0G3507
DL_TimerA_setCaptureCompareValue(TIMG0, duty, DL_TIMER_CC_0_INDEX);
// 或:
PWM_SET_DUTY(TIMG0, DL_TIMER_CC_0_INDEX, duty);
```

## 5. ADC

| 特性 | STM32 | MSPM0G3507 |
|------|-------|------------|
| 精度 | 12-bit | **12-bit** |
| 采样速率 | 1 MSPS | **4 MSPS** (更优) |
| 通道数 | 最多16路 | **10路外部 + 内部通道** |
| DMA | 支持 | **支持 (DMA0~DMA3)** |
| 触发方式 | 软件/定时器 | **软件/硬件触发** |

```c
// STM32
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1, 10);
uint16_t val = HAL_ADC_GetValue(&hadc1);

// MSPM0G3507
DL_ADC12_startConversion(ADC0);
while (!DL_ADC12_getRawInterruptStatus(ADC0,
    DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED)) {}
uint16_t val = DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
```

## 6. UART

| 特性 | STM32 | MSPM0G3507 |
|------|-------|------------|
| 实例 | USART1~3, UART4~5 | **UART0, UART1, UART2** |
| 波特率配置 | BRR 寄存器 | **SysConfig 自动计算** |
| FIFO | 无 (单字节) | **有 (硬件 FIFO)** |

## 7. 编码器接口

| 特性 | STM32 | MSPM0G3507 |
|------|-------|------------|
| 模式 | TIM 编码器模式 | **TIMG QEI 模式** |
| 解码 | 硬件自动 A/B 解码 | **硬件自动 A/B 解码** |
| 配置 | `HAL_TIM_Encoder_Start()` | **`DL_Timer` + QEI 模式** |

## 8. 编程模型差异

| 方面 | STM32 (HAL) | MSPM0G3507 (DriverLib) |
|------|-------------|------------------------|
| API 风格 | `HAL_Xxx_Function()` | **`DL_Xxx_function()`** |
| 句柄 | HAL 句柄结构体 | **直接传寄存器基地址** |
| 配置工具 | STM32CubeMX (.ioc) | **SysConfig (.syscfg)** |
| 代码生成 | HAL 库 + LL 库 | **DL 库 + ti_msp_dl_config** |
| 中断处理 | `HAL_XXX_IRQHandler()` | **直接写 IRQ Handler 名** |

## 9. 关键迁移检查清单

### 必须修改
- [ ] **无 FPU**: 所有 `float` 运算为软件模拟，PID 计算耗时增加 ~10x
- [ ] **中断优先级仅 4 级**: 需重新规划中断优先级分配
- [ ] **GPIO 端口减少**: 仅 GPIOA/GPIOB，引脚数 < STM32
- [ ] **无 TIM 编码器模式命名**: 改用 QEI 相关 API
- [ ] **头文件替换**: `stm32f1xx_hal.h` → `ti/driverlib/driverlib.h`

### 无需修改
- [x] PID 算法逻辑 (纯数学，平台无关)
- [x] 赛题逻辑代码
- [x] 通信协议 (UART/SPI 数据格式)

## 10. 性能对比

| 操作 | STM32F103 @72MHz | MSPM0G3507 @80MHz |
|------|------------------|---------------------|
| GPIO 切换 | ~14ns | **~12.5ns** (略快) |
| ADC 采样 | 1μs | **0.25μs** (4x 快) |
| 浮点乘法 | ~0.01μs (M3 硬件) | **~0.5μs** (软件模拟) |
| PID 计算 | ~1μs | **~10-50μs** |
| 中断响应 | ~12 周期 | **~16 周期** |

## 11. 常见坑

1. **SysConfig 是必须的**: 不像 STM32 可以纯手写寄存器，MSPM0 强烈依赖 SysConfig
2. **外设时钟使能**: MSPM0 默认外设时钟关闭，需在 SysConfig 中启用
3. **引脚冲突**: GPIO 引脚较少，多功能复用需仔细规划
4. **无硬件除法**: `int / int` 由软件完成，避免在高频中断中使用
5. **Flash 较小**: 128KB 足够电赛，但需注意库占用

## 12. 迁移实战技巧

### 12.1 快速迁移步骤

```
1. 头文件替换:
   #include "stm32f1xx_hal.h" → #include "ti/driverlib/driverlib.h"
   
2. 初始化替换:
   HAL_Init() + SystemClock_Config() → SYSCFG_DL_init() (一行搞定)
   
3. GPIO替换:
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
   → DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0)
   
4. 定时器替换:
   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty)
   → DL_TimerA_setCaptureCompareValue(TIMG0, duty, DL_TIMER_CC_0_INDEX)
   
5. 编译运行 → 解决编译错误 → 调试
```

### 12.2 电赛常用模块迁移对照表

| STM32模块 | MSPM0对应模块 | 关键差异 |
|-----------|-------------|---------|
| PWM电机控制 | TIMG0/TIMG6 PWM | 无FPU，PID用定点数优化 |
| 编码器 | TIMG QEI模式 | API名不同，功能相同 |
| I2C OLED | UART0/I2C模拟 | MSPM0无硬件I2C(用模拟) |
| SPI通信 | SPI0/SPI1 | 时钟配置不同 |
| ADC采集 | ADC0(12bit 4MSPS) | 采样率更高，注意DMA配置 |
| UART调试 | UART0/UART1 | 有硬件FIFO，更稳定 |

### 12.3 性能优化建议

1. **PID计算优化**: 用Q15定点数替代float，速度提升10x
2. **中断优先级**: 仅4级，关键中断(编码器)设为0
3. **DMA使用**: ADC+DMA采集释放CPU时间
4. **代码优化**: -O2编译，使用__attribute__((packed))减小结构体
