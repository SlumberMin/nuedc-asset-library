# PID调参实战手册 - 从入门到精通

## 一、PID基础公式

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

| 参数 | 作用 | 增大效果 | 典型副作用 |
|------|------|----------|-----------|
| Kp | 比例：快速响应误差 | 响应加快 | 超调增大、振荡 |
| Ki | 积分：消除稳态误差 | 消除静差 | 响应变慢、积分饱和 |
| Kd | 微分：抑制变化率 | 减小超调 | 放大噪声、抖动 |

## 二、离散PID（嵌入式常用）

```c
// 增量式PID（推荐，无积分饱和问题）
float error = setpoint - measurement;
float delta = Kp*(error - last_error) + Ki*error + Kd*(error - 2*last_error + prev_error);
output += delta;
prev_error = last_error;
last_error = error;

// 位置式PID
float integral += error * dt;
float derivative = (error - last_error) / dt;
output = Kp*error + Ki*integral + Kd*derivative;
last_error = error;
```

**关键技巧：**
- 积分限幅：`integral = CLAMP(integral, -I_MAX, I_MAX)`
- 输出限幅：`output = CLAMP(output, -OUT_MAX, OUT_MAX)`
- 死区处理：`if(fabs(error) < DEADZONE) error = 0;`
- 微分滤波：`derivative = alpha*derivative + (1-alpha)*(error-last_error)/dt`

## 三、Ziegler-Nichols临界比例法（经典方法）

### 步骤：
1. **设Ki=0, Kd=0**，仅用P控制
2. **逐渐增大Kp**，直到系统出现等幅持续振荡
3. **记录临界增益Ku和振荡周期Tu**
4. **查表计算参数：**

| 控制器 | Kp | Ki | Kd |
|--------|-----|-----|-----|
| 纯P | 0.5Ku | - | - |
| PI | 0.45Ku | 0.54Ku/Tu | - |
| PID | 0.6Ku | 1.2Ku/Tu | 0.075Ku·Tu |
| Pessen积分 | 0.7Ku | 1.75Ku/Tu | 0.105Ku·Tu |
| 有些超调 | 0.33Ku | 0.66Ku/Tu | 0.11Ku·Tu |
| 无超调 | 0.2Ku | 0.4Ku/Tu | 0.066Ku·Tu |

### 适用条件：
- 系统能产生振荡（电赛中电机速度、温度控制常用）
- 响应时间合理（不能太慢）

## 四、继电反馈法（Relay Feedback，更实用）

当系统无法用Z-N法（如某些积分型系统）时，用继电反馈自动整定：

### 原理：
```
if(measurement < setpoint) output = +d;  // 继电器输出
else output = -d;
```
系统会在设定值附近产生自振荡，测量振幅a和周期Tu。

### 参数计算：
```
Ku = 4d / (π·a)    // d为继电器幅值，a为振荡幅值
Tu = 振荡周期

// 然后用Z-N表或改进公式：
Kp = 0.6·Ku
Ki = 1.2·Ku / Tu
Kd = 0.075·Ku·Tu
```

### C代码实现：
```c
#define RELAY_D 500  // 继电器幅值（PWM值）

int relay_state = 1;
float osc_amplitude = 0, osc_period = 0;
uint32_t last_cross_time = 0;
float max_val = -99999, min_val = 99999;

float relay_pid(float setpoint, float measurement) {
    if(relay_state) {
        // 继电整定阶段
        if(measurement > setpoint) output = -RELAY_D;
        else output = +RELAY_D;
        
        // 检测振荡周期和幅值
        static int cross_count = 0;
        static uint32_t first_cross;
        if((measurement > setpoint && last_meas <= setpoint) ||
           (measurement < setpoint && last_meas >= setpoint)) {
            cross_count++;
            if(cross_count == 1) first_cross = HAL_GetTick();
            if(cross_count == 5) {
                osc_period = (HAL_GetTick() - first_cross) / 2.0f;
                // 计算Ku并切换到PID
                relay_state = 0;
                compute_zn_params();
            }
        }
        max_val = fmax(max_val, measurement);
        min_val = fmin(min_val, measurement);
        osc_amplitude = (max_val - min_val) / 2.0f;
        last_meas = measurement;
    } else {
        // 正常PID阶段
        output = pid_compute(setpoint, measurement);
    }
    return CLAMP(output, -1000, 1000);
}
```

## 五、手动调参法（电赛最实用）

### 调参口诀：先P后I再D，逐步修正

```
第一步：设Ki=0, Kd=0
  → 从小到大调Kp，直到系统有较快响应且振荡可接受
  → 如果不需要消除静差，到此结束

第二步：引入Ki
  → 从小到大调Ki，直到静差消除
  → Ki过大 → 振荡加剧、响应变慢 → 减小

第三步：引入Kd
  → 从小到大调Kd，减小超调
  → Kd过大 → 高频抖动、噪声放大 → 减小

第四步：微调
  → 观察响应曲线，三者协同微调
```

### 常见现象速查：

| 现象 | 原因 | 解决 |
|------|------|------|
| 响应太慢 | Kp太小 | 增大Kp |
| 振荡不收敛 | Kp太大 | 减小Kp |
| 有静差 | 缺积分项 | 加Ki |
| 积分饱和 | Ki太大 | 减Ki，加积分限幅 |
| 超调过大 | Kp或Ki偏大 | 减Kp/Ki，加Kd |
| 高频抖动 | Kd过大或噪声 | 减Kd，加滤波 |
| 启动冲击 | 积分累积 | 用增量式PID或积分分离 |

## 六、电赛高频场景调参经验

### 电机速度环（典型参数范围）
```
Kp = 2~10,  Ki = 0.5~3,  Kd = 0~0.5
采样周期: 10~50ms
建议: 增量式PID + 低通滤波
```

### 电机位置环
```
Kp = 5~20,  Ki = 0~1,  Kd = 1~5
建议: 位置环+速度环双闭环
外环(位置)输出→内环(速度)设定值
```

### 温度控制
```
Kp = 5~50,  Ki = 0.1~2,  Kd = 5~20
采样周期: 500ms~2s
注意: 温度响应慢，耐心等待
```

### 平衡/姿态控制
```
Kp = 10~100,  Ki = 0~5,  Kd = 1~10
采样周期: 1~5ms（越快越好）
建议: 加互补滤波或卡尔曼滤波
```

## 七、自动整定技巧（电赛实战）

### 抗积分饱和（Anti-Windup）
```c
// 条件积分法
if(fabs(error) < THRESHOLD && fabs(output) < OUT_MAX) {
    integral += error * dt;
}
// 反馈退饱和法
integral += (error + (output - output_clamped) / Kb) * dt;
```

### 积分分离
```c
if(fabs(error) > SEPARATION_THRESHOLD) {
    // 大误差时不积分
    output = Kp * error + Kd * derivative;
} else {
    output = Kp * error + Ki * integral + Kd * derivative;
}
```

### 变增益PID（分段PID）
```c
if(fabs(error) > 100) { Kp_use = Kp * 2; Ki_use = 0; }
else if(fabs(error) > 20) { Kp_use = Kp; Ki_use = Ki * 0.5; }
else { Kp_use = Kp; Ki_use = Ki; }
```

## 八、调试工具建议

1. **串口输出**：每周期输出 `时间,设定值,实际值,输出` → 导入Excel画图
2. **关键指标**：上升时间、超调量、稳态误差、调节时间
3. **示波器**：观察PWM输出波形、编码器反馈波形

## 九、调参检查清单

- [ ] 确认反馈信号正确（方向、单位、量程）
- [ ] 确认执行器限幅已设置
- [ ] 确认采样周期合理且稳定
- [ ] 确认PID输出方向正确（正误差→正输出）
- [ ] 先P后I再D，不要同时调
- [ ] 每次只改一个参数
- [ ] 记录每组参数的效果
