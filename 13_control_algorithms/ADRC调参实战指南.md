# ADRC调参实战指南 —— 从入门到精通

> 适用：电赛中电机控制、小车平衡、倒立摆等需要高精度快速响应的场景

---

## 一、ADRC是什么？为什么选它？

### 1.1 PID的痛点

| 问题 | 表现 |
|------|------|
| 超调 | 快速性与超调的矛盾 |
| 抗扰差 | 外部扰动需要等误差产生后才纠正 |
| 参数耦合 | Kp/Ki/Kd互相影响，调参靠经验 |

### 1.2 ADRC的核心思想

**把一切未知扰动（内扰+外扰）统一估计并补偿掉。**

```
参考输入 → [跟踪微分器TD] → [扩张状态观测器ESO] → [非线性反馈NLSEF] → 控制量 → 被控对象
                                    ↑                                    ↓
                                    ←←←←←←←← 状态估计 ←←←←←←←←←←←←←←←←←
```

三个核心模块：
1. **TD（跟踪微分器）**：安排过渡过程，解决快速性与超调矛盾
2. **ESO（扩张状态观测器）**：估计系统状态 + 总扰动
3. **NLSEF（非线性状态误差反馈）**：产生控制量

---

## 二、二阶ADRC完整数学推导

### 2.1 被控对象

$$\ddot{y} = f(y, \dot{y}, w, t) + bu$$

其中 $f$ 是总扰动（含内部未建模动态 + 外部扰动），$b$ 是增益。

### 2.2 离散TD

```
v1(k+1) = v1(k) + h * v2(k)
v2(k+1) = v2(k) + h * fhan(v1(k) - v0, v2(k), r, h0)
```

`fhan` 函数（最速综合函数）：

```c
float fhan(float x1, float x2, float r, float h) {
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;
    float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    float a2 = a0 + sign(y) * (a1 - d) * 0.5f;
    float sy = (sign(y + d) - sign(y - d)) * 0.5f;
    float a = (a0 + y - a2) * sy + a2;
    float sa = (sign(a + d) - sign(a - d)) * 0.5f;
    return -r * (a / d - sign(a)) * sa - r * sign(a);
}
```

### 2.3 离散ESO（核心！）

```
e = z1(k) - y(k)
z1(k+1) = z1(k) + h * (z2(k) - β01 * e)
z2(k+1) = z2(k) + h * (z3(k) - β02 * fal(e, α1, δ) + b * u(k))
z3(k+1) = z3(k) + h * (-β03 * fal(e, α2, δ))
```

其中 `fal` 函数：

```c
float fal(float e, float alpha, float delta) {
    if (fabsf(e) < delta) {
        return e / powf(delta, 1.0f - alpha);
    } else {
        return sign(e) * powf(fabsf(e), alpha);
    }
}
```

**状态含义：**
- `z1` → 估计位置
- `z2` → 估计速度
- `z3` → **估计总扰动**（这是ADRC的精髓）

### 2.4 NLSEF + 扰动补偿

```
e1 = v1(k) - z1(k)    // 位置误差
e2 = v2(k) - z2(k)    // 速度误差
u0 = β1 * fal(e1, α1, δ) + β2 * fal(e2, α2, δ)
u = (u0 - z3) / b      // 扰动补偿！
```

---

## 三、完整C实现（可直接移植到STM32）

```c
/* adrc.h */
#ifndef __ADRC_H
#define __ADRC_H

typedef struct {
    /* TD参数 */
    float r;        // 跟踪速度因子
    float h;        // 采样周期
    
    /* ESO参数 */
    float beta01, beta02, beta03;  // 观测器增益
    float alpha1, alpha2;          // fal函数指数
    float delta;                   // fal函数线性区宽度
    float b;                       // 系统增益估计
    
    /* NLSEF参数 */
    float beta1, beta2;            // 反馈增益
    float alpha3, alpha4;          // fal函数指数
    
    /* 状态变量 */
    float v1, v2;     // TD输出
    float z1, z2, z3;  // ESO输出
    float u;           // 控制输出
} ADRC_Controller;

void ADRC_Init(ADRC_Controller *adrc, float h);
void ADRC_Update(ADRC_Controller *adrc, float ref, float y);
void ADRC_Reset(ADRC_Controller *adrc);

/* 电机控制专用快速整定 */
void ADRC_MotorTune(ADRC_Controller *adrc, float ts, float b_est);

#endif
```

```c
/* adrc.c */
#include "adrc.h"
#include <math.h>

static float sign(float x) {
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

static float fhan(float x1, float x2, float r, float h) {
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;
    float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    float a2 = a0 + sign(y) * (a1 - d) * 0.5f;
    float sy = (sign(y + d) - sign(y - d)) * 0.5f;
    float a = (a0 + y - a2) * sy + a2;
    float sa = (sign(a + d) - sign(a - d)) * 0.5f;
    return -r * (a / d - sign(a)) * sa - r * sign(a);
}

static float fal(float e, float alpha, float delta) {
    if (fabsf(e) < delta) {
        return e / powf(delta, 1.0f - alpha);
    }
    return sign(e) * powf(fabsf(e), alpha);
}

void ADRC_Init(ADRC_Controller *adrc, float h) {
    adrc->h = h;
    adrc->r = 100.0f;
    adrc->beta01 = 100.0f;
    adrc->beta02 = 300.0f;
    adrc->beta03 = 1000.0f;
    adrc->alpha1 = 0.5f;
    adrc->alpha2 = 0.25f;
    adrc->delta = 0.01f;
    adrc->b = 1.0f;
    adrc->beta1 = 10.0f;
    adrc->beta2 = 5.0f;
    adrc->alpha3 = 0.5f;
    adrc->alpha4 = 0.75f;
    adrc->v1 = 0; adrc->v2 = 0;
    adrc->z1 = 0; adrc->z2 = 0; adrc->z3 = 0;
    adrc->u = 0;
}

void ADRC_Update(ADRC_Controller *adrc, float ref, float y) {
    /* === TD === */
    float h0 = adrc->h;
    float fv = fhan(adrc->v1 - ref, adrc->v2, adrc->r, h0);
    adrc->v1 += adrc->h * adrc->v2;
    adrc->v2 += adrc->h * fv;

    /* === ESO === */
    float e = adrc->z1 - y;
    adrc->z1 += adrc->h * (adrc->z2 - adrc->beta01 * e);
    adrc->z2 += adrc->h * (adrc->z3 - adrc->beta02 * fal(e, adrc->alpha1, adrc->delta) + adrc->b * adrc->u);
    adrc->z3 += adrc->h * (-adrc->beta03 * fal(e, adrc->alpha2, adrc->delta));

    /* === NLSEF + 补偿 === */
    float e1 = adrc->v1 - adrc->z1;
    float e2 = adrc->v2 - adrc->z2;
    float u0 = adrc->beta1 * fal(e1, adrc->alpha3, adrc->delta) 
              + adrc->beta2 * fal(e2, adrc->alpha4, adrc->delta);
    adrc->u = (u0 - adrc->z3) / adrc->b;
}

void ADRC_Reset(ADRC_Controller *adrc) {
    adrc->v1 = adrc->v2 = 0;
    adrc->z1 = adrc->z2 = adrc->z3 = 0;
    adrc->u = 0;
}

void ADRC_MotorTune(ADRC_Controller *adrc, float ts, float b_est) {
    /* 快速整定公式（经验值，电机控制常用） */
    float wc = 4.0f / ts;          // 闭环带宽
    float wo = (3.0f ~ 5.0f) * wc; // 观测器带宽（取3~5倍）
    
    adrc->b = b_est;
    adrc->r = 1.0f / (ts * ts);    // TD跟踪速度
    adrc->beta01 = 3.0f * wo;
    adrc->beta02 = 3.0f * wo * wo;
    adrc->beta03 = wo * wo * wo;
    adrc->beta1 = wc * wc;
    adrc->beta2 = 2.0f * wc;
}
```

---

## 四、参数整定方法（核心！）

### 4.1 "分离整定法"——推荐流程

**Step 1：整定ESO（最关键）**

ESO是ADRC的灵魂，先调ESO再调其他。

```
观测器带宽 ωo 选取：
  - ωo 越大 → 观测越快 → 但对噪声敏感
  - 经验公式：ωo = (3 ~ 10) × 期望闭环带宽 ωc

β01 = 3ωo
β02 = 3ωo²  
β03 = ωo³
```

**调试方法：**
```c
// 只开ESO，不开NLSEF，看z3能否跟踪总扰动
adrc.u = 0;  // 开环
// 观察 z3 是否能正确估计扰动
// 用串口打印 z3 和 实际扰动 对比
```

**判断标准：** z3波形应平滑且快速跟踪真实扰动。

**Step 2：整定NLSEF**

```
β1 = ωc²（位置环增益）
β2 = 2ωc（速度环增益）
```

**Step 3：整定TD**

```
r 越大 → 跟踪越快 → 过渡时间越短
经验：r = 系统最大加速度 × 2
```

### 4.2 带宽法参数速查表

| 期望响应时间ts | ωc | ωo | β01 | β02 | β03 | β1 | β2 |
|---|---|---|---|---|---|---|---|
| 0.5s | 8 | 32 | 96 | 3072 | 32768 | 64 | 16 |
| 0.2s | 20 | 80 | 240 | 19200 | 512000 | 400 | 40 |
| 0.1s | 40 | 160 | 480 | 76800 | 4096000 | 1600 | 80 |
| 0.05s | 80 | 320 | 960 | 307200 | 32768000 | 6400 | 160 |

### 4.3 参数影响速查

| 参数 | 增大效果 | 减小效果 | 常见问题 |
|------|---------|---------|---------|
| β01 | ESO跟踪更快 | 跟踪滞后 | 过大→z1振荡 |
| β02 | 速度估计更准 | 速度估计滞后 | 过大→z2抖动 |
| β03 | **扰动估计更快** | 扰动估计滞后 | **过大→z3剧烈振荡！** |
| β1 | 响应更快 | 响应更慢 | 过大→超调/振荡 |
| β2 | 阻尼更大 | 阻尼小 | 过大→响应变慢 |
| r | 过渡过程更快 | 过渡更平缓 | 过大→TD输出振荡 |
| b | 控制量变小 | 控制量变大 | 偏差大→控制效果差 |

---

## 五、常见问题与解决

### Q1：z3（扰动估计）剧烈振荡

**原因：** β03过大或fal函数δ太小

**解决：**
```c
// 方案1：增大δ（线性区）
adrc.delta = 0.1f;  // 从0.01增大到0.1

// 方案2：降低β03
adrc.beta03 *= 0.5f;

// 方案3：对z3做一阶低通滤波
static float z3_filtered = 0;
float alpha_lpf = 0.3f;
z3_filtered = alpha_lpf * adrc.z3 + (1 - alpha_lpf) * z3_filtered;
adrc.u = (u0 - z3_filtered) / adrc.b;
```

### Q2：控制量饱和

**解决：** 加输出限幅 + 抗积分饱和

```c
float u_max = 100.0f;  // PWM最大值
if (adrc.u > u_max) adrc.u = u_max;
if (adrc.u < -u_max) adrc.u = -u_max;
```

### Q3：采样频率太低导致ADRC失效

**经验法则：** 采样频率 ≥ 10× 闭环带宽

```
如果闭环带宽ωc = 40 rad/s → 采样频率 ≥ 400 rad/s → ≥ 64Hz
实际建议：≥ 200Hz（电机控制常见1kHz~10kHz）
```

### Q4：b值（系统增益）不知道怎么办

```c
// 方法1：粗略估计
// 电机：b ≈ Kt / J（转矩系数/转动惯量）
// 温度：b ≈ 加热功率 / 热容

// 方法2：自适应b
// 用ESO的z3在线估计b的修正量
float b_adapt = 1.0f;
float adapt_rate = 0.01f;
b_adapt += adapt_rate * adrc.z3 * adrc.u;
adrc.b = b_adapt;
```

---

## 六、仿真验证（Python）

```python
import numpy as np
import matplotlib.pyplot as plt

# 被控对象：二阶系统 + 扰动
# y'' = -10*y' - 100*y + u + d(t)

dt = 0.001  # 1ms
T = 2.0
N = int(T / dt)

# ADRC参数
r = 1000
beta01, beta02, beta03 = 150, 7500, 125000
b = 1.0
beta1, beta2 = 400, 40
delta = 0.1

# 状态
y, dy = 0, 0
v1, v2 = 0, 0
z1, z2, z3 = 0, 0, 0
u = 0

# 记录
t_log = []
y_log = []
ref_log = []
u_log = []

ref = 1.0  # 目标值

def sign(x):
    return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)

def fhan(x1, x2, r, h):
    d = r * h * h
    a0 = h * x2
    y = x1 + a0
    a1 = np.sqrt(d * (d + 8 * abs(y)))
    a2 = a0 + sign(y) * (a1 - d) * 0.5
    sy = (sign(y + d) - sign(y - d)) * 0.5
    a = (a0 + y - a2) * sy + a2
    sa = (sign(a + d) - sign(a - d)) * 0.5
    return -r * (a / d - sign(a)) * sa - r * sign(a)

def fal(e, alpha, delta):
    if abs(e) < delta:
        return e / (delta ** (1 - alpha))
    return sign(e) * (abs(e) ** alpha)

for k in range(N):
    t = k * dt
    
    # 扰动：t>1s时加50%负载
    d = 50.0 if t > 1.0 else 0.0
    
    # 被控对象
    ddy = -10 * dy - 100 * y + u + d
    dy += dtdy
    y += dt * dy
    
    # ADRC
    # TD
    fv = fhan(v1 - ref, v2, r, dt)
    v1 += dt * v2
    v2 += dt * fv
    
    # ESO
    e_eso = z1 - y
    z1 += dt * (z2 - beta01 * e_eso)
    z2 += dt * (z3 - beta02 * fal(e_eso, 0.5, delta) + b * u)
    z3 += dt * (-beta03 * fal(e_eso, 0.25, delta))
    
    # NLSEF
    e1 = v1 - z1
    e2 = v2 - z2
    u0 = beta1 * fal(e1, 0.5, delta) + beta2 * fal(e2, 0.75, delta)
    u = (u0 - z3) / b
    
    # 限幅
    u = max(-200, min(200, u))
    
    t_log.append(t)
    y_log.append(y)
    ref_log.append(ref)
    u_log.append(u)

# 绘图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
ax1.plot(t_log, ref_log, 'r--', label='Reference')
ax1.plot(t_log, y_log, 'b-', label='Output')
ax1.set_ylabel('Position')
ax1.legend()
ax1.grid(True)

ax2.plot(t_log, u_log, 'g-', label='Control')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control Signal')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.savefig('adrc_response.png', dpi=150)
plt.show()
```

---

## 七、ADRC vs PID 对比测试

### 测试场景

| 场景 | PID表现 | ADRC表现 |
|------|---------|---------|
| 阶跃响应 | 有超调，需折中 | TD安排过渡，几乎无超调 |
| 突加负载 | 等误差产生后纠正 | ESO提前估计，快速补偿 |
| 参数变化 | 需重新整定 | 自动补偿（ESO估计） |
| 调参难度 | 三个参数互相耦合 | 带宽法，物理意义清晰 |

### 何时选ADRC vs PID

- **选PID：** 系统简单、变化慢、调试时间短
- **选ADRC：** 系统非线性强、扰动大、需要高性能

---

## 八、电赛实战模板

### 8.1 电机速度ADRC控制

```c
/* 1ms定时器中断 */
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // 读编码器
        int32_t speed = TIM3->CNT;
        TIM3->CNT = 0;
        
        // ADRC更新
        ADRC_Update(&motor_adrc, target_speed, (float)speed);
        
        // 输出PWM
        int16_t pwm = (int16_t)motor_adrc.u;
        Set_PWM(pwm);
    }
}
```

### 8.2 平衡车直立ADRC

```c
/* 同时控制角度和速度 */
void Balance_Control(void) {
    float angle = Get_Angle();      // MPU6050融合角度
    float gyro = Get_Gyro();        // 角速度
    
    // 角度环ADRC
    ADRC_Update(&angle_adrc, 0.0f, angle);
    
    // 输出给电机
    int16_t pwm = (int16_t)angle_adrc.u;
    Set_Motor(pwm, pwm);
}
```

---

## 九、进阶：线性ADRC（LADRC）

对于工程应用，线性ADRC更易实现且稳定性好：

```c
/* LADRC - 用线性函数替代fal */
void LADRC_Update(ADRC_Controller *adrc, float ref, float y) {
    /* L-TD */
    float fv = fhan(adrc->v1 - ref, adrc->v2, adrc->r, adrc->h);
    adrc->v1 += adrc->h * adrc->v2;
    adrc->v2 += adrc->h * fv;
    
    /* L-ESO */
    float e = adrc->z1 - y;
    float wo = adrc->beta01 / 3.0f;  // 观测器带宽
    adrc->z1 += adrc->h * (adrc->z2 - 3 * wo * e);
    adrc->z2 += adrc->h * (adrc->z3 - 3 * wo * wo * e + adrc->b * adrc->u);
    adrc->z3 += adrc->h * (-wo * wo * wo * e);
    
    /* L-SEF */
    float e1 = adrc->v1 - adrc->z1;
    float e2 = adrc->v2 - adrc->z2;
    float wc = sqrtf(adrc->beta1);
    float u0 = wc * wc * e1 + 2 * wc * e2;
    adrc->u = (u0 - adrc->z3) / adrc->b;
}
```

**LADRC只有两个调参：ωo（观测器带宽）和 ωc（控制器带宽）**

---

## 十、总结：ADRC调参口诀

```
ESO是灵魂，先调观测器；
β03别太大，否则z3炸；
带宽法最实用，ωo取3到5倍ωc；
b值要估准，偏差别超半；
fal的δ别太小，噪声会进家；
输出要限幅，保护执行器。
```

---

*最后更新：2026年6月 | nuedc-asset-library*
