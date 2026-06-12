# 串级PID调参实战指南

> 从入门到精通：速度环 + 位置环 + 电流环全覆盖

---

## 一、串级PID基本概念

### 1.1 为什么需要串级PID？

单级PID在面对复杂系统时存在以下问题：
- **响应慢**：单环无法同时兼顾快速性和稳定性
- **抗干扰差**：外部扰动直接作用在被控量上
- **超调大**：单环PID难以平衡响应速度和超调量

串级PID通过**内环+外环**的结构，将复杂控制问题分解：
- 内环负责快速响应（消除扰动）
- 外环负责精确跟踪（保证精度）

### 1.2 串级PID基本结构

```
目标值 ──→ [外环PID] ──→ 内环目标 ──→ [内环PID] ──→ 执行器 ──→ 被控对象
  ↑                                                        │
  │                        ←←←←←← 反馈 ←←←←←←←←←←←←←←←←←←←←
```

### 1.3 双环 vs 三环

| 结构 | 应用场景 | 典型应用 |
|------|----------|----------|
| 双环（速度+位置） | 直线运动控制 | 电机直线运动、平衡小车 |
| 双环（电流+速度） | 力矩精确控制 | 机械臂、四旋翼 |
| 三环（电流+速度+位置） | 高精度位置控制 | 伺服系统、精密定位 |

---

## 二、三环PID详解

### 2.1 电流环（最内环）

**作用**：控制电机电流，即控制力矩

**特点**：
- 带宽最高（响应最快），通常 > 1kHz
- 消除电机电气特性带来的延迟
- 保护电机不过流

**参数范围**（经验值）：
```
Kp: 0.5 ~ 5.0
Ki: 0.01 ~ 0.5
Kd: 0 ~ 0.1（电流环通常不用D项）
```

**调参要点**：
1. 先只调Kp，观察电流跟踪波形
2. 加入Ki消除稳态误差
3. 电流环一般不需要D项（噪声放大问题严重）

### 2.2 速度环（中间环）

**作用**：控制电机转速

**特点**：
- 带宽中等，通常 100~500Hz
- 电流环作为其执行环节
- 消除负载变化带来的速度波动

**参数范围**（经验值）：
```
Kp: 1.0 ~ 20.0
Ki: 0.1 ~ 5.0
Kd: 0 ~ 1.0
```

**调参要点**：
1. 确保电流环已调好
2. 先调Kp使速度能跟上目标
3. 加Ki消除速度稳态误差
4. 加Kd抑制速度超调

### 2.3 位置环（最外环）

**作用**：控制电机位置/角度

**特点**：
- 带宽最低，通常 10~100Hz
- 速度环作为其执行环节
- 保证最终位置精度

**参数范围**（经验值）：
```
Kp: 0.5 ~ 10.0
Ki: 0 ~ 1.0（位置环Ki慎用）
Kd: 0.1 ~ 5.0
```

**调参要点**：
1. 确保速度环已调好
2. Kp决定位置跟踪速度
3. Kd用于抑制振荡和超调
4. 位置环通常**不需要大Ki**，否则积分饱和

---

## 三、调参步骤（从内到外）

### 步骤1：调电流环

```c
// 伪代码
while (1) {
    current_error = target_current - measured_current;
    output = Kp_i * current_error + Ki_i * integral_i;
    integral_i += current_error * dt;
    set_pwm(output);
    wait_for_next_cycle(1000);  // 1kHz
}
```

**操作步骤**：
1. 设定一个较小的目标电流（如额定的20%）
2. 先设 Ki=0, Kd=0，逐步增大 Kp
3. 观察电流波形，直到出现轻微振荡
4. Kp取振荡值的60%~70%
5. 逐步加入Ki，消除稳态误差
6. 若出现振荡，适当减小Ki

### 步骤2：调速度环

```c
while (1) {
    speed_error = target_speed - measured_speed;
    speed_output = Kp_v * speed_error + Ki_v * integral_v + Kd_v * derivative_v;
    integral_v += speed_error * dt;
    derivative_v = (speed_error - last_speed_error) / dt;
    last_speed_error = speed_error;
    
    // 速度环输出作为电流环目标
    target_current = speed_output;
    wait_for_next_cycle(2000);  // 500Hz
}
```

**操作步骤**：
1. 设定一个适中的目标速度
2. 先设 Ki=0, Kd=0，逐步增大 Kp
3. Kp取到速度能快速响应但不振荡
4. 加入Ki消除稳态误差
5. 加入Kd抑制超调

### 步骤3：调位置环

```c
while (1) {
    pos_error = target_pos - measured_pos;
    pos_output = Kp_p * pos_error + Ki_p * integral_p + Kd_p * derivative_p;
    
    // 位置环输出作为速度环目标
    target_speed = pos_output;
    wait_for_next_cycle(5000);  // 200Hz
}
```

**操作步骤**：
1. 设定一个目标位置
2. 先设 Ki=0, Kd=0，逐步增大 Kp
3. 观察位置跟踪曲线，调整到无超调或可接受超调
4. 加入Kd进一步抑制超调
5. 位置环一般不加Ki或只加很小的Ki

---

## 四、常见问题与解决方案

### 4.1 振荡问题

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| 内环振荡 | Kp过大 | 减小内环Kp |
| 外环振荡 | 外环Kp过大或内环响应不足 | 减小外环Kp或优化内环 |
| 高频振荡 | 传感器噪声被D项放大 | 加低通滤波或减小Kd |
| 低频振荡 | 系统延迟过大 | 检查采样频率，减小积分时间 |

### 4.2 响应慢

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| 整体响应慢 | 内环带宽不足 | 提高内环Kp |
| 跟踪有静差 | 积分系数不足 | 适当增大Ki |
| 启动慢 | 积分项积累不足 | 使用积分分离或前馈 |

### 4.3 积分饱和

```c
// 积分限幅法
if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

// 积分分离法
if (fabs(error) < ERROR_THRESHOLD) {
    integral += error * dt;  // 只在误差小时积分
}

// 条件积分法（Anti-Windup）
if (fabs(output) < OUTPUT_MAX) {
    integral += error * dt;  // 只在输出未饱和时积分
}
```

---

## 五、进阶技巧

### 5.1 前馈控制

在串级PID基础上加入前馈，提高响应速度：

```c
// 速度前馈
target_speed = pos_output + Kff_pos * (target_pos - last_target_pos) / dt;

// 电流前馈
target_current = speed_output + Kff_speed * (target_speed - last_target_speed) / dt;
```

### 5.2 变参数PID

根据误差大小切换参数：

```c
if (fabs(error) > BIG_ERROR) {
    Kp = Kp_aggressive;  // 大误差用激进参数
} else {
    Kp = Kp_gentle;       // 小误差用温和参数
}
```

### 5.3 微分先行PID

只对反馈值微分，避免目标突变引起输出跳变：

```c
// 标准微分
derivative = (error - last_error) / dt;

// 微分先行
derivative = -(feedback - last_feedback) / dt;
```

---

## 六、调试工具推荐

| 工具 | 用途 | 推荐 |
|------|------|------|
| 串口波形助手 | 实时观察PID输出波形 | ★★★★★ |
| MATLAB/Simulink | 系统建模与仿真 | ★★★★☆ |
| 示波器 | 观测实际信号 | ★★★★★ |
| 上位机 | 参数在线调整 | ★★★★☆ |

---

## 七、实战案例：平衡小车双环PID

### 7.1 系统结构

```
目标角度(0°) ──→ [角度环PID] ──→ 目标速度 ──→ [速度环PID] ──→ PWM输出 ──→ 电机
     ↑                                                                  │
     └────────────────←←←←←←←←← 陀螺仪+加速度计 ←←←←←←←←←←←←←←←←←←←←←←
```

### 7.2 代码实现

```c
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float last_error;
    float output;
    float integral_max;
    float output_max;
} PID_t;

float PID_Calc(PID_t *pid, float error, float dt) {
    pid->integral += error * dt;
    // 积分限幅
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    
    float derivative = (error - pid->last_error) / dt;
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // 输出限幅
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;
    
    pid->last_error = error;
    return pid->output;
}

// 角度环（外环）
PID_t angle_pid = {5.0, 0, 2.0, 0, 0, 0, 100, 200};
// 速度环（内环）
PID_t speed_pid = {3.0, 0.5, 0.5, 0, 0, 0, 500, 1000};

void Balance_Control(float target_angle, float current_angle, float current_speed, float dt) {
    // 外环：角度 -> 目标速度
    float angle_error = target_angle - current_angle;
    float target_speed = PID_Calc(&angle_pid, angle_error, dt);
    
    // 内环：速度 -> PWM
    float speed_error = target_speed - current_speed;
    float pwm_output = PID_Calc(&speed_pid, speed_error, dt);
    
    Set_Motor_PWM((int)pwm_output);
}
```

### 7.3 推荐调参顺序

1. **先调角度环**：设速度环Kp=1, Ki=0, Kd=0（比例直通）
2. **调角度环Kp**：从小到大，直到车能基本直立
3. **调角度环Kd**：抑制摆动
4. **再调速度环Kp**：让车能保持静止不漂移
5. **调速度环Ki**：消除速度静差

---

## 八、总结

串级PID调参口诀：

> **从内到外，先P后I再D；**
> **内环要快，外环要稳；**
> **振荡减P，静差加I，超调加D；**
> **积分限幅防饱和，微分滤波防噪声。**

---

*本文档为nuedc-asset-library的一部分，持续更新中。*
