# STM32 vs TM4C123 平台差异对比表

本文档帮助从STM32平台迁移到TM4C123平台的开发者快速理解差异。

## 📊 核心参数对比

| 特性 | STM32F103C8T6 | STM32F407VET6 | TM4C123GH6PZT7 |
|------|---------------|---------------|-----------------|
| **内核** | Cortex-M3 | Cortex-M4F | Cortex-M4F |
| **主频** | 72MHz | 168MHz | 80MHz |
| **Flash** | 64KB | 512KB | 256KB |
| **SRAM** | 20KB | 192KB | 32KB |
| **FPU** | ❌ 无 | ✅ 硬件FPU | ✅ 硬件FPU |
| **GPIO** | 37个 | 82个 | 69个 |
| **ADC** | 2×12bit, 1MSPS | 3×12bit, 2.4MSPS | 2×12bit, 1MSPS |
| **定时器** | 4+2 | 14 | 6 |
| **PWM** | TIM输出 | TIM输出 | 专用PWM模块(16路) |
| **编码器** | TIM编码器模式 | TIM编码器模式 | **专用QEI模块** |
| **UART** | 3 | 6 | 8 |
| **I2C** | 2 | 3 | 4 |
| **SPI** | 2 | 3 | 4 |
| **USB** | 从机 | OTG | 从机/主机 |
| **CAN** | 1 | 2 | 2 |
| **价格** | ~¥8 | ~¥25 | ~¥20 |

## 🔧 开发环境对比

| 项目 | STM32 | TM4C123 |
|------|-------|---------|
| **IDE** | Keil MDK / STM32CubeIDE | CCS (Code Composer Studio) / Keil |
| **HAL库** | STM32 HAL / StdPeriph | TivaWare (driverlib) |
| **调试器** | ST-Link | Stellaris ICDI / J-Link |
| **烧录** | SWD | SWD / JTAG |
| **配置工具** | STM32CubeMX | 无图形化工具 (手动配置) |
| **包管理** | STM32Cube包 | TivaWare SDK |

## ⚙️ 外设驱动API差异

### GPIO

| 操作 | STM32 HAL | TM4C (TivaWare) |
|------|-----------|-----------------|
| 输出高 | `HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)` | `GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5)` |
| 输出低 | `HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)` | `GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0)` |
| 读取 | `HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)` | `GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5)` |
| 翻转 | `HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5)` | `无内置, 用读-取反-写` |
| 模式配置 | `GPIO_InitStruct.Mode/Speed/Pull` | `GPIOPinTypeGPIOOutput()` / `GPIOPadConfigSet()` |

### PWM

| 项目 | STM32 | TM4C123 |
|------|-------|---------|
| 实现方式 | TIMx_CHy 输出比较 | 专用PWM模块 (PWM0/PWM1) |
| 频率设置 | `TIM_Prescaler` + `TIM_Period` | `PWMGenPeriodSet()` |
| 占空比 | `__HAL_TIM_SET_COMPARE()` | `PWMPulseWidthSet()` |
| 分辨率 | 取决于ARR | 取决于Load寄存器 |
| 优势 | 灵活, 每个定时器独立 | **专用模块, 16路独立输出** |

**关键差异:** TM4C123有独立PWM模块(PWM0/PWM1), 每个模块8个输出, 不占用通用定时器。

### 编码器

| 项目 | STM32 | TM4C123 |
|------|-------|---------|
| 实现方式 | TIM编码器模式 | **专用QEI模块** |
| 配置复杂度 | 需配置TIM, GPIO | `QEIConfigure()` 一步到位 |
| 速度测量 | 需软件定时读取 | **硬件自动测量** |
| 4倍频 | 自动 | 自动 |
| CPU开销 | 低 | **零** (硬件解码) |
| 优势 | 通用 | **专用硬件, 无CPU占用** |

**关键差异:** TM4C123的QEI是真正的硬件正交解码器, 包含速度测量寄存器, 完全不占用CPU。

### ADC

| 项目 | STM32 | TM4C123 |
|------|-------|---------|
| 配置方式 | ADC_InitTypeDef + CubeMX | `ADCSequenceConfigure()` |
| 触发方式 | 软件/定时器/外部 | 软件/定时器/PWM |
| DMA支持 | ✅ | ✅ |
| 序列扫描 | 最多16通道 | 每个序列最多8步 |
| 中断 | EOC/EOCS | 序列完成中断 |

### 定时器

| 项目 | STM32 | TM4C123 |
|------|-------|---------|
| 数量 | 4通用 + 2基本 + 2高级 | 6个 (Timer0~5) |
| 位宽 | 16bit | 16/32bit (每个可选) |
| 编码器模式 | ✅ TIM编码器 | ❌ 用QEI模块 |
| PWM输出 | ✅ | ❌ 用PWM模块 |
| 输入捕获 | ✅ | ✅ |

### 串口(UART)

| 项目 | STM32 | TM4C123 |
|------|-------|---------|
| 数量 | 3~6 | 8个 |
| FIFO | ❌ (需DMA) | **✅ 16字节硬件FIFO** |
| 波特率配置 | `USART_Init()` | `UARTConfigSet()` |

**优势:** TM4C123有8个UART, 每个带16字节FIFO, 适合多串口应用。

## 🔌 引脚映射差异

### GPIO端口命名

| STM32 | TM4C123 |
|-------|---------|
| GPIOA~GPIOG | GPIO_PORTA~GPIO_PORTH |
| `GPIO_Pin_5` | `GPIO_PIN_5` |
| `GPIO_Pin_All` | `0xFF` |

### 引脚复用

| 项目 | STM32 | TM4C123 |
|------|-------|---------|
| 复用配置 | `GPIO_PinAFConfig()` | `GPIOPinConfigure()` |
| 复用表 | 每个引脚有AF映射表 | 固定映射 (如PB6=M0PWM0) |
| 灵活性 | **高** (任意引脚可复用) | 低 (固定引脚映射) |

**关键差异:** STM32引脚复用更灵活, TM4C123外设引脚映射是固定的。

## ⚡ 性能对比

| 指标 | STM32F103 | STM32F407 | TM4C123 |
|------|-----------|-----------|---------|
| Dhrystone MIPS | 90 | 210 | 100 |
| CoreMark | 150 | 550 | 340 |
| 浮点运算 | ❌ 软浮点 | ✅ 硬件 | ✅ 硬件 |
| PID计算耗时(估) | ~10μs(软浮点) | ~1μs | ~2μs |
| ADC采样率 | 1MSPS | 2.4MSPS | 1MSPS |

## 🏆 TM4C123电赛优势

| 优势 | 说明 |
|------|------|
| **专用QEI** | 硬件编码器接口, 零CPU占用, 自动速度测量 |
| **专用PWM模块** | 16路独立PWM, 不占用定时器资源 |
| **8个UART** | 多传感器串口通信无压力 |
| **硬件FPU** | PID浮点运算加速 |
| **丰富GPIO** | 69个IO, 引脚充足 |
| **性价比** | 性能/价格比高, 适合电赛预算 |

## ⚠️ TM4C123注意事项

| 问题 | 解决方案 |
|------|----------|
| 引脚复用不灵活 | 设计电路时严格按数据手册引脚表 |
| 无图形化配置工具 | 手动编写初始化代码 (参考本代码库) |
| 社区资源较少 | 参考TI官方例程和论坛 |
| 固件库较旧 | TivaWare 2.2.0 仍可用, API稳定 |
| GPIO无内置上拉 | 需外接上拉电阻或用 `GPIOPadConfigSet()` |

## 📚 快速迁移检查清单

从STM32迁移到TM4C123:

- [ ] 安装CCS或Keil for TM4C
- [ ] 下载TivaWare SDK
- [ ] 替换HAL库调用为TivaWare API
- [ ] 移除STM32的TIM编码器代码, 改用QEI
- [ ] 移除STM32的TIM PWM代码, 改用PWM模块
- [ ] 检查引脚复用映射是否匹配硬件
- [ ] 更新中断处理函数名称
- [ ] 验证时钟配置 (80MHz vs STM32的72/168MHz)
- [ ] 测试FPU浮点运算精度

## 🔗 参考资料

- [TM4C123GH6PZ数据手册](https://www.ti.com/lit/ds/symlink/tm4c123gh6pz.pdf)
- [TivaWare API参考](https://www.ti.com/lit/ug/spmu298/spmu298.pdf)
- [TM4C vs STM32社区讨论](https://e2e.ti.com/support/microcontrollers/)
