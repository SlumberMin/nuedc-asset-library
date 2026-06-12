# 嵌入式PID实现完全指南

## 目录

1. [PID控制基础](#1-pid控制基础)
2. [离散化方法](#2-离散化方法)
3. [定点化实现](#3-定点化实现)
4. [抗积分饱和](#4-抗积分饱和)
5. [微分项处理](#5-微分项处理)
6. [参数整定方法](#6-参数整定方法)
7. [嵌入式优化技巧](#7-嵌入式优化技巧)
8. [常见问题与解决方案](#8-常见问题与解决方案)
9. [代码模板](#9-代码模板)
10. [调试方法](#10-调试方法)

---

## 1. PID控制基础

### 1.1 连续PID公式

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt
```

- **P (比例)**: 快速响应误差,但存在稳态误差
- **I (积分)**: 消除稳态误差,但可能导致超调和积分饱和
- **D (微分)**: 预测误差趋势,抑制超调,但放大噪声

### 1.2 离散化公式

**位置式PID:**
```
u[k] = Kp·e[k] + Ki·Ts·Σe[j] + Kd·(e[k]-e[k-1])/Ts
```

**增量式PID:**
```
Δu[k] = Kp·(e[k]-e[k-1]) + Ki·Ts·e[k] + Kd·(e[k]-2e[k-1]+e[k-2])/Ts
u[k] = u[k-1] + Δu[k]
```

### 1.3 位置式 vs 增量式

| 特性 | 位置式 | 增量式 |
|------|--------|--------|
| 输出 | 绝对值 | 增量 |
| 积分饱和 | 需要处理 | 天然抗饱和 |
| 初始化 | 需要跟踪积分项 | 只需上一次输出 |
| 执行器故障 | 可能大幅跳变 | 自动跟踪 |
| 实现复杂度 | 简单 | 稍复杂 |

**推荐**: 电机控制用增量式,温度控制用位置式。

---

## 2. 离散化方法

### 2.1 前向欧拉 (Forward Euler)

```
s ≈ (z-1)/(z·Ts)
```

- 最简单,但高频失真大
- 条件稳定,需要足够小的Ts

### 2.2 后向欧拉 (Backward Euler) ⭐推荐

```
s ≈ (z-1)/Ts
```

- 无条件稳定
- 频率畸变适中
- 嵌入式最常用

### 2.3 双线性变换 (Tustin)

```
s ≈ (2/Ts)·(z-1)/(z+1)
```

- 频率畸变最小
- 计算量稍大
- 精度要求高时使用

### 2.4 采样周期选择

```
经验法则: Ts ≤ Tp / (10 ~ 20)
```

其中Tp是被控对象主导时间常数。

| 应用 | 典型Ts |
|------|--------|
| 电机速度环 | 1~5 ms |
| 电机位置环 | 0.5~2 ms |
| 温度控制 | 100~1000 ms |
| 电压/电流环 | 50~200 μs |

---

## 3. 定点化实现

### 3.1 Q格式选择

| Q格式 | 整数范围 | 分辨率 | 适用场景 |
|-------|----------|--------|----------|
| Q16.16 | ±32767 | 1/65536 | 通用PID |
| Q8.24 | ±127 | 1/16777216 | 高精度小信号 |
| Q4.28 | ±7 | 1/268435456 | 超高精度 |
| Q0.32 | ±0 | 1/4294967296 | 纯小数 |

### 3.2 定点运算注意

```c
// Q16乘法
int32_t q16_mul(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b) >> 16);
}

// Q16除法
int32_t q16_div(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a << 16) / b);
}
```

**关键点**:
- 乘法中间结果用64位防止溢出
- 除法前左移扩大精度
- 注意有符号数的符号扩展

### 3.3 定点化误差分析

- Q16.16分辨率: 1.5×10⁻⁵
- 8-bit ADC分辨率: 3.3V/256 ≈ 12.9 mV
- 12-bit ADC分辨率: 3.3V/4096 ≈ 0.8 mV

**结论**: 对于12-bit ADC系统,Q16.16定点精度足够。

---

## 4. 抗积分饱和

### 4.1 积分饱和现象

当执行器饱和(如PWM占空比已达100%)时,误差持续累积导致积分项过大,即使误差反向也需要很长时间才能"退饱和",造成大超调。

### 4.2 抗饱和策略

#### 策略1: 条件积分 (Conditional Integration)

```c
// 仅在输出未饱和时积分
if (output > out_min && output < out_max) {
    integral += ki * error * dt;
}
```

**优点**: 简单  
**缺点**: 饱和边界处积分不连续

#### 策略2: 积分限幅 (Integral Clamping)

```c
integral += ki * error * dt;
integral = CLAMP(integral, integral_min, integral_max);
```

**优点**: 简单可靠  
**缺点**: 需要经验设置限幅值

#### 策略3: 反馈退饱和 (Back-Calculation) ⭐推荐

```c
unclamped_output = p + i + d;
output = CLAMP(unclamped_output, out_min, out_max);

// 退饱和: 将多余的输出反馈回积分项
if (unclamped_output != output) {
    integral -= kb * (unclamped_output - output) * dt;
}
```

**优点**: 效果最好,响应平滑  
**缺点**: 需要整定退饱和增益kb

#### 策略4: 积分分离 (Integral Separation)

```c
if (fabs(error) < threshold) {
    integral += ki * error * dt;  // 误差小时启用积分
} else {
    integral = 0;  // 误差大时关闭积分
}
```

**优点**: 大信号响应快  
**缺点**: 阈值需要经验设置

#### 策略5: 速率限幅 (Rate Limiting)

```c
delta = output - prev_output;
if (delta > rate_limit) delta = rate_limit;
if (delta < -rate_limit) delta = -rate_limit;
output = prev_output + delta;
```

### 4.3 策略组合建议

| 应用场景 | 推荐组合 |
|----------|----------|
| 电机控制 | 条件积分 + 积分限幅 |
| 温度控制 | 反馈退饱和 + 积分限幅 |
| 位置伺服 | 积分分离 + 速率限幅 |
| 电压调节 | 反馈退饱和 |

---

## 5. 微分项处理

### 5.1 微分放大噪声问题

微分项对高频噪声极为敏感。ADC量化噪声会被微分放大。

### 5.2 解决方案

#### 方案1: 低通滤波

```c
// 一阶IIR滤波
d_raw = kd * (error - prev_error) / dt;
d_filtered = alpha * d_raw + (1 - alpha) * prev_d;
prev_d = d_filtered;
```

alpha推荐值: 0.1 ~ 0.3

#### 方案2: 不完全微分

```c
// 在PID传递函数中加入低通滤波器
// C(s) = Kp + Ki/s + Kd·s/(1 + Tf·s)
// 离散化:
d_term = (kd / (dt + tf)) * ((error - prev_error) + (tf/dt) * (error - prev_error));
```

#### 方案3: 微分作用于PV (过程变量) ⭐推荐

```c
// 不对误差微分,而是对测量值微分
d_term = -kd * (measurement - prev_measurement) / dt;
```

**优点**: 
- 设定值跳变时不会产生微分冲击
- 天然抗噪声

---

## 6. 参数整定方法

### 6.1 Ziegler-Nichols临界比例法

1. 令Ki=0, Kd=0
2. 逐渐增大Kp直到系统等幅振荡
3. 记录临界增益Ku和振荡周期Tu
4. 按下表计算:

| 方法 | Kp | Ki | Kd |
|------|----|----|-----|
| Z-N经典 | 0.6Ku | 1.2Ku/Tu | 0.075Ku·Tu |
| Pessen | 0.7Ku | 1.75Ku/Tu | 0.105Ku·Tu |
| 少超调 | 0.33Ku | 0.66Ku/Tu | 0.11Ku·Tu |
| 无超调 | 0.2Ku | 0.4Ku/Tu | 0.066Ku·Tu |

### 6.2 Cohen-Coon方法

适用于带延迟的一阶系统: G(s) = K·e^(-θs) / (τs+1)

```
Kp = (τ/(K·θ))·(1 + θ/(3τ))
Ki = Kp / (τ·(1 + 2θ/(3τ))^(-1))
Kd = Kp·τ·(4θ/(3τ))/(1 + 2θ/(3τ))
```

### 6.3 SIMC (Skogestad IMC) 方法 ⭐推荐

1. 建立过程模型: G(s) = K·e^(-θs) / (τs+1)
2. 选择闭环时间常数 τc (一般取 τc = θ)
3. 计算:
```
Kp = τ / (K·(τc + θ))
Ki = Kp / min(τ, 4·(τc + θ))
Kd = 0
```

### 6.4 手动整定步骤

1. 先只用P控制,从小Kp开始增大,直到响应合理
2. 加入I控制,从小Ki开始,消除稳态误差
3. 如果超调大,加入D控制
4. 反复微调,平衡快速性和稳定性

---

## 7. 嵌入式优化技巧

### 7.1 计算优化

```c
// 1. 使用移位代替除法 (2的幂次)
// Kp = 0.5 → kp_fixed = value >> 1;

// 2. 预计算常数
// 不要每次计算 ki * dt, 而是预计算 ki_dt = ki * dt

// 3. 使用查表代替复杂运算
// 对于非线性增益调度

// 4. 使用DMA+中断实现后台采样
```

### 7.2 内存优化

```c
// 使用位域压缩标志
typedef struct {
    uint8_t first_run   : 1;
    uint8_t aw_enable   : 1;
    uint8_t mode        : 2;
    uint8_t reserved    : 4;
} pid_flags_t;
```

### 7.3 实时性保证

- PID计算放在定时器中断中
- 确保中断周期稳定
- 使用原子操作保护共享变量
- PID计算时间应 < 10% 的采样周期

### 7.4 多环控制

```
位置环 (1-5ms)
  └─ 速度环 (0.5-2ms)
      └─ 电流环 (50-200μs)
```

- 内环必须比外环快5~10倍
- 内环先整定,外环后整定

---

## 8. 常见问题与解决方案

### 8.1 输出振荡

| 原因 | 解决方案 |
|------|----------|
| Kp过大 | 减小Kp |
| Ts过大 | 减小采样周期 |
| 传感器噪声 | 加滤波,减小Kd |
| 机械共振 | 加陷波滤波器 |

### 8.2 响应太慢

| 原因 | 解决方案 |
|------|----------|
| Kp过小 | 增大Kp |
| Ki过小 | 增大Ki |
| 滤波过强 | 减小滤波系数 |

### 8.3 大超调

| 原因 | 解决方案 |
|------|----------|
| Ki过大 | 减小Ki,或用积分分离 |
| 积分饱和 | 用抗饱和策略 |
| 缺少D项 | 加入微分控制 |

### 8.4 稳态误差

| 原因 | 解决方案 |
|------|----------|
| Ki过小 | 增大Ki |
| 积分限幅太紧 | 放宽积分限幅 |
| 执行器死区 | 加前馈补偿 |

---

## 9. 代码模板

### 9.1 最小PID模板

```c
typedef struct {
    float kp, ki, kd;
    float integral, prev_error;
    float out_min, out_max;
} pid_t;

float pid_update(pid_t *pid, float sp, float pv, float dt) {
    float error = sp - pv;
    
    // P
    float p = pid->kp * error;
    
    // I
    pid->integral += pid->ki * error * dt;
    float i = pid->integral;
    
    // D
    float d = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // 合成并限幅
    float output = p + i + d;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    
    return output;
}
```

### 9.2 增量式PID模板

```c
typedef struct {
    float kp, ki, kd;
    float prev_error, prev2_error, prev_output;
    float out_min, out_max;
} pid_inc_t;

float pid_inc_update(pid_inc_t *pid, float sp, float pv, float dt) {
    float error = sp - pv;
    
    float delta = pid->kp * (error - pid->prev_error)
                + pid->ki * error * dt
                + pid->kd * (error - 2*pid->prev_error + pid->prev2_error) / dt;
    
    float output = pid->prev_output + delta;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    
    pid->prev2_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_output = output;
    
    return output;
}
```

---

## 10. 调试方法

### 10.1 阶跃响应测试

```c
// 1. 输出设定值阶跃
// 2. 记录: 设定值, 测量值, 控制输出
// 3. 分析: 上升时间, 超调量, 调整时间, 稳态误差
```

### 10.2 扰动测试

```c
// 1. 系统稳定后,突然改变负载
// 2. 观察恢复时间和最大偏差
```

### 10.3 实时监控变量

建议通过串口/DAC输出以下变量:

- 设定值 (setpoint)
- 测量值 (measurement)
- 控制输出 (output)
- P, I, D 各分量
- 误差 (error)
- 积分项 (integral) - 监控是否饱和

### 10.4 调试工具

- **串口绘图器**: 用Python matplotlib实时显示
- **逻辑分析仪**: 检查PWM波形
- **示波器**: 检查实际信号
- **J-Scope**: Segger的实时变量监控工具

---

## 参考资料

1. Åström, K.J. & Hägglund, T. "PID Controllers: Theory, Design, and Tuning"
2. Franklin, G.F. "Feedback Control of Dynamic Systems"
3. 本资源库: `common/pid_discrete.c` - 定点化PID实现
4. 本资源库: `common/pid_anti_windup_v2.c` - 多策略抗饱和PID
5. 本资源库: `simulation/digital_pid_simulation.py` - 量化效应仿真
6. 本资源库: `simulation/robust_pid_simulation.py` - 鲁棒性仿真
