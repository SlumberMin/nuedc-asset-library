# 电赛高频Bug排查手册

> 电赛中常见的软件Bug现象、原因分析与解决方法

---

## 目录

1. [定时器相关问题](#1-定时器相关问题)
2. [ADC采集问题](#2-adc采集问题)
3. [串口通信问题](#3-串口通信问题)
4. [PWM输出问题](#4-pwm输出问题)
5. [中断问题](#5-中断问题)
6. [PID控制问题](#6-pid控制问题)
7. [显示驱动问题](#7-显示驱动问题)
8. [内存与栈溢出](#8-内存与栈溢出)
9. [电源与复位问题](#9-电源与复位问题)
10. [编译与链接问题](#10-编译与链接问题)

---

## 1. 定时器相关问题

### Bug 1.1: 定时器中断不触发
**现象**: 配置了定时器但中断回调函数不执行
**原因**:
- 未使能NVIC中断: `HAL_NVIC_EnableIRQ(TIMx_IRQn)`
- 未启动定时器: `HAL_TIM_Base_Start_IT(&htimx)`
- 时钟未使能: `__HAL_RCC_TIMx_CLK_ENABLE()`
- 预分频/重载值为0

**排查步骤**:
```c
// 1. 检查时钟使能
__HAL_RCC_TIM2_CLK_ENABLE();

// 2. 检查NVIC使能
HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
HAL_NVIC_EnableIRQ(TIM2_IRQn);

// 3. 检查定时器启动
HAL_TIM_Base_Start_IT(&htim2);

// 4. 在中断回调函数中翻转LED验证
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}
```

### Bug 1.2: 定时器中断频率不准
**现象**: 示波器测量中断频率与计算值不符
**原因**:
- APB总线时钟频率理解错误（APB1通常为APB主频的一半）
- 预分频寄存器是 `PSC+1` 分频
- 重载寄存器是 `ARR+1` 计数

**公式**:
```
中断频率 = TIM_CLK / ((PSC+1) * (ARR+1))
```

### Bug 1.3: 多个定时器中断互相干扰
**现象**: 开启定时器B后，定时器A的中断变慢或丢失
**原因**: 中断优先级配置不当，高优先级中断抢占了低优先级中断
**解决**: 合理配置抢占优先级和子优先级

---

## 2. ADC采集问题

### Bug 2.1: ADC读数跳变/毛刺大
**现象**: ADC读数不稳定，跳变幅度大（几十甚至上百个LSB）
**原因与解决**:
| 原因 | 解决方法 |
|------|----------|
| 未加滤波电容 | 在ADC输入引脚并联100nF~1μF电容 |
| 采样时间太短 | 增加采样周期: `ADC_SAMPLETIME_239CYCLES_5` |
| 电源纹波大 | ADC供电加LC滤波 |
| 通道间串扰 | 增加通道间延迟或使用非连续模式 |
| 数字/模拟地未分离 | 单点接地，模拟地与数字地分开布线 |

### Bug 2.2: ADC读数始终为0或4095
**现象**: ADC读数饱和
**原因**:
- GPIO配置错误（未配置为模拟输入模式）
- ADC通道与实际引脚不对应
- 参考电压未接

**排查**:
```c
// 检查GPIO配置
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;  // 必须是模拟模式！
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

### Bug 2.3: 多通道ADC扫描顺序错误
**现象**: 读取到的通道数据错位
**原因**: DMA缓冲区顺序与ADC扫描顺序不匹配
**解决**: 按Rank顺序配置通道，DMA缓冲区严格对应

---

## 3. 串口通信问题

### Bug 3.1: 串口只能发不能收
**现象**: 上位机发送数据，MCU无响应
**原因**:
- 未使能接收中断: `HAL_UART_Receive_IT(&huart1, &rx_byte, 1)`
- 中断回调函数中未重新启动接收
- 波特率不匹配
- TX/RX接反

**解决**:
```c
// 在接收完成回调中重新启动接收
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 处理数据...
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  // 重新启动！
    }
}
```

### Bug 3.2: 串口接收数据乱码
**原因**: 波特率误差过大或系统时钟配置错误
**解决**: 检查系统时钟配置，确保波特率误差<2%

### Bug 3.3: printf重定向不工作
**原因**: 未重写`fputc`函数或未勾选"Use MicroLIB"
```c
// 标准重定向
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
    return ch;
}
```

---

## 4. PWM输出问题

### Bug 4.1: PWM无输出
**现象**: 示波器测量PWM引脚无波形
**排查清单**:
- [ ] GPIO配置为复用推挽输出(`GPIO_MODE_AF_PP`)
- [ ] 定时器时钟已使能
- [ ] `HAL_TIM_PWM_Start()`已调用
- [ ] ARR和CCR值合理（CCR > 0 且 CCR ≤ ARR）
- [ ] 引脚复用映射正确（特别是重映射引脚）

### Bug 4.2: PWM频率不正确
**公式**: `f_PWM = TIM_CLK / ((PSC+1) * (ARR+1))`
**注意**: 高级定时器(TIM1/TIM8)需要使能主输出: `HAL_TIM_PWM_Start()` 而不是 `HAL_TIM_Base_Start()`

### Bug 4.3: PWM占空比方向相反
**现象**: 设置50%占空比但输出看起来像5%
**原因**: CCR与ARR的比较方向，有些配置是递减计数
**解决**: 检查计数方向配置

---

## 5. 中断问题

### Bug 5.1: 中断处理时间过长导致系统卡死
**现象**: 进入某个中断后系统不响应其他事件
**原因**: 中断服务函数中执行了大量计算或延时
**解决**:
```c
// 错误做法
void EXTI0_IRQHandler(void) {
    // ... 大量计算
    delay_ms(100);  // 绝对禁止！
}

// 正确做法: 设置标志位，主循环处理
volatile uint8_t flag_exti0 = 0;
void EXTI0_IRQHandler(void) {
    flag_exti0 = 1;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
}

// 主循环
while(1) {
    if (flag_exti0) {
        flag_exti0 = 0;
        // 处理逻辑...
    }
}
```

### Bug 5.2: 共享变量数据竞争
**现象**: 中断和主循环同时访问变量导致数据错误
**解决**: 使用 `volatile` 关键字，必要时关中断保护

### Bug 5.3: 中断嵌套导致栈溢出
**原因**: 中断优先级配置导致多层嵌套
**解决**: 合理规划优先级，避免不必要的嵌套；增大栈空间

---

## 6. PID控制问题

### Bug 6.1: PID输出饱和（积分饱和）
**现象**: 误差变号后输出很久才响应
**解决**: 加入积分限幅(anti-windup)
```c
if (pid->output > OUT_MAX) {
    pid->integral -= error * dt;  // 停止积分累加
    pid->output = OUT_MAX;
}
```

### Bug 6.2: PID输出抖动
**原因**: 微分项对噪声敏感
**解决**: 对微分项加低通滤波
```c
derivative = alpha * derivative_raw + (1-alpha) * derivative_prev;
```

### Bug 6.3: 串级PID内外环冲突
**现象**: 两个环互相打架，系统振荡
**原因**: 内外环带宽比不够（通常需要5倍以上）
**解决**: 内环响应速度 >> 外环响应速度

---

## 7. 显示驱动问题

### Bug 7.1: OLED显示花屏/白屏
**原因**:
- I2C/SPI时序问题
- 初始化序列不正确
- 显示缓冲区未正确刷新

### Bug 7.2: TFT屏颜色错误
**原因**: 颜色格式不匹配（RGB565 vs RGB888）
**解决**: 确认驱动芯片的颜色格式定义

### Bug 7.3: 显示刷新闪烁
**原因**: 全屏刷新太慢
**解决**: 局部刷新，只更新变化区域

---

## 8. 内存与栈溢出

### Bug 8.1: 程序运行一段时间后死机
**原因**: 栈溢出覆盖了其他变量
**排查**:
```c
// 在链接脚本中增大栈空间
_Min_Heap_Size = 0x400;
_Min_Stack_Size = 0x800;  // 增大到2KB
```

### Bug 8.2: 局部大数组导致栈溢出
**解决**: 使用 `static` 关键字或全局数组
```c
void process() {
    static uint8_t buffer[1024];  // 放在BSS段，不占栈
    // ...
}
```

### Bug 8.3: malloc/free导致内存碎片
**解决**: 电赛建议尽量使用静态分配，避免动态内存

---

## 9. 电源与复位问题

### Bug 9.1: 程序下载后不运行
**原因**:
- BOOT引脚配置错误
- 复位电路异常
- 电源不稳定
- 看门狗未喂狗

### Bug 9.2: 运行中随机复位
**原因**:
- 电源纹波过大触发复位
- 未使用独立看门狗导致死循环
- EMI干扰

---

## 10. 编译与链接问题

### Bug 10.1: 未定义引用(undefined reference)
**原因**: 缺少源文件或库文件
**解决**: 检查项目文件列表，添加缺失的.c/.a文件

### Bug 10.2: 头文件找不到
**原因**: Include路径未配置
**解决**: Project → Options → C/C++ → Include Paths 添加路径

### Bug 10.3: 优化等级导致程序行为异常
**现象**: Debug模式正常，Release模式异常（或反之）
**原因**: 编译器优化可能改变代码执行顺序
**解决**: 对关键变量使用 `volatile`；检查时序敏感代码

---

## 快速排查流程图

```
Bug出现
  │
  ├─ 能复现吗？
  │   ├─ 能 → 用断点/打印定位问题代码行
  │   └─ 偶发 → 检查中断/共享变量/时序问题
  │
  ├─ 硬件还是软件？
  │   ├─ 换一块板子试试
  │   └─ 用示波器观察信号
  │
  └─ 最近改了什么？
      └─ 用版本控制回退对比 (git diff)
```

---

*最后更新: 2025年电赛备战*
