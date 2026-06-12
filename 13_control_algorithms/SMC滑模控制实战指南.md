# SMC滑模控制实战指南 - 从入门到精通

> 电赛控制类题目核心算法，抗干扰能力强，实现简单

---

## 一、滑模控制原理（10分钟速通）

### 1.1 什么是滑模控制？

滑模控制（Sliding Mode Control）是一种**变结构控制**方法：
- 设计一个**滑模面** s(x) = 0
- 无论系统初始状态如何，都驱使状态到达滑模面
- 到达滑模面后，沿滑模面滑动到平衡点

**核心优势**：对参数摄动和外部干扰具有**不变性**（鲁棒性极强）

### 1.2 二阶系统经典滑模

对于系统：
```
ẍ = f(x) + b·u + d(t)
```

定义滑模面：
```
s = ė + λ·e    （其中 e = x_desired - x）
```

控制律：
```
u = (1/b) · [ -f(x) + ẍ_desired + λ·ė - η·sgn(s) ]
```

其中：
- λ > 0：滑模面斜率，决定收敛速度
- η > 0：切换增益，必须 > |d(t)| 上界
- sgn(s)：符号函数，造成抖振

---

## 二、STM32完整实现

### 2.1 基础滑模控制器

```c
/* ========== SMC滑模控制器 ========== */
typedef struct {
    float lambda;       // 滑模面参数 λ
    float eta;          // 切换增益 η
    float epsilon;      // 饱和函数边界层厚度
    float prev_error;   // 上一次误差
    float prev_s;       // 上一次滑模面值
    float output;       // 控制输出
} SMC_t;

void SMC_Init(SMC_t *smc, float lambda, float eta, float epsilon)
{
    smc->lambda = lambda;
    smc->eta = eta;
    smc->epsilon = epsilon;
    smc->prev_error = 0;
    smc->prev_s = 0;
    smc->output = 0;
}

/* 饱和函数替代符号函数，消除抖振 */
float sat(float s, float epsilon)
{
    if (s > epsilon) return 1.0f;
    if (s < -epsilon) return -1.0f;
    return s / epsilon;
}

/* 等速趋近律滑模 */
float SMC_Calc(SMC_t *smc, float target, float current, float dt)
{
    float error = target - current;
    float d_error = (error - smc->prev_error) / dt;  // 误差微分

    // 滑模面: s = ė + λ·e
    float s = d_error + smc->lambda * error;

    // 控制律: u = λ·ė + η·sat(s/ε)
    smc->output = smc->lambda * d_error + smc->eta * sat(s, smc->epsilon);

    smc->prev_error = error;
    smc->prev_s = s;

    return smc->output;
}
```

### 2.2 电机速度环滑模控制

```c
/* ========== 电机速度SMC ========== */
typedef struct {
    SMC_t smc;
    float Kp_speed;     // 速度环比例（可选混合）
    float integral;     // 积分项（消除稳态误差）
    float Ki;
    float max_output;
} Motor_SMC_t;

void MotorSMC_Init(Motor_SMC_t *m, float lambda, float eta, float epsilon)
{
    SMC_Init(&m->smc, lambda, eta, epsilon);
    m->Kp_speed = 0;
    m->integral = 0;
    m->Ki = 0.01f;
    m->max_output = 999;
}

float MotorSMC_Calc(Motor_SMC_t *m, float target_rpm, float actual_rpm, float dt)
{
    float error = target_rpm - actual_rpm;

    // 滑模主控
    float u_smc = SMC_Calc(&m->smc, target_rpm, actual_rpm, dt);

    // 积分补偿（消除稳态误差）
    m->integral += error * dt * m->Ki;
    if (m->integral > 100) m->integral = 100;
    if (m->integral < -100) m->integral = -100;

    float output = u_smc + m->integral;

    // 限幅
    if (output > m->max_output) output = m->max_output;
    if (output < -m->max_output) output = -m->max_output;

    return output;
}

/* ========== 使用示例 ========== */
Motor_SMC_t motor1_smc;

void Control_Init(void)
{
    // 参数整定经验：
    // lambda = 5~20 (越大收敛越快)
    // eta = 50~500 (必须大于最大干扰)
    // epsilon = 0.1~5 (越大抖振越小，但跟踪精度下降)
    MotorSMC_Init(&motor1_smc, 10.0f, 200.0f, 1.0f);
}

// 10ms定时器中断
void TIM2_IRQHandler(void)
{
    static float target_speed = 300.0f;  // 目标转速 rpm
    float actual_speed = Encoder_GetSpeed();

    float pwm = MotorSMC_Calc(&motor1_smc, target_speed, actual_speed, 0.01f);

    Motor_SetPWM((int16_t)pwm);
}
```

### 2.3 位置环滑模控制（适合平衡小车）

```c
/* ========== 位置+角度双环SMC ========== */
typedef struct {
    SMC_t angle_smc;    // 角度环（内环）
    SMC_t pos_smc;      // 位置环（外环）
    float max_angle_ref; // 最大角度参考值
} BalanceSMC_t;

void BalanceSMC_Init(BalanceSMC_t *b)
{
    SMC_Init(&b->angle_smc, 15.0f, 300.0f, 2.0f);   // 内环快
    SMC_Init(&b->pos_smc, 5.0f, 100.0f, 3.0f);       // 外环慢
    b->max_angle_ref = 15.0f;  // 最大倾角15度
}

float BalanceSMC_Calc(BalanceSMC_t *b,
                      float target_pos, float actual_pos,
                      float actual_angle, float actual_gyro,
                      float dt)
{
    // 外环：位置 → 目标角度
    float angle_ref = SMC_Calc(&b->pos_smc, target_pos, actual_pos, dt);
    // 限幅
    if (angle_ref > b->max_angle_ref) angle_ref = b->max_angle_ref;
    if (angle_ref < -b->max_angle_ref) angle_ref = -b->max_angle_ref;

    // 内环：角度 → PWM
    // 注意：这里用陀螺仪角速度做 ė，而非数值微分
    float error_angle = angle_ref - actual_angle;
    float s = actual_gyro + b->angle_smc.lambda * error_angle;
    float u = b->angle_smc.lambda * (-actual_gyro)
            + b->angle_smc.eta * sat(s, b->angle_smc.epsilon);

    return u;
}
```

---

## 三、关键参数整定指南

### 3.1 参数对性能的影响

| 参数 | 增大效果 | 减小效果 | 典型范围 |
|------|---------|---------|---------|
| λ (lambda) | 收敛更快 | 收敛更慢 | 5~30 |
| η (eta) | 抗干扰更强 | 抗干扰弱 | 50~500 |
| ε (epsilon) | 抖振更小 | 跟踪更精确 | 0.5~5 |

### 3.2 调参步骤（推荐）

```
第1步：设 ε 较大（如5），η 较小（如50），λ 中等（如10）
第2步：观察跟踪效果，增大 λ 直到响应足够快
第3步：施加干扰（如手动拨动电机），增大 η 直到能抗住
第4步：逐步减小 ε，观察抖振，在精度和抖振间平衡
第5步：加入积分项消除稳态误差
```

### 3.3 常见问题与解决

| 问题 | 原因 | 解决 |
|------|------|------|
| 输出剧烈抖振 | ε太小或η太大 | 增大ε，用饱和函数替代sgn |
| 稳态误差 | 系统存在未建模扰动 | 加积分补偿 |
| 超调过大 | λ太大 | 减小λ或加入趋近律 |
| 响应太慢 | λ太小 | 增大λ |

---

## 四、趋近律变种

### 4.1 指数趋近律（推荐）

```c
/* s_dot = -ε·sgn(s) - k·s  （快速趋近+等速趋近） */
float SMC_ExponentialApproach(SMC_t *smc, float error, float d_error, float dt)
{
    float s = d_error + smc->lambda * error;
    float k = 5.0f;   // 指数趋近系数
    float eps = smc->eta;

    // 趋近律: s_dot = -eps*sgn(s) - k*s
    float s_dot = -eps * sat(s, smc->epsilon) - k * s;

    // 控制量积分
    smc->output += s_dot * dt;
    return smc->output;
}
```

### 4.2 幂次趋近律（平滑）

```c
/* s_dot = -η·|s|^α·sgn(s)  (0<α<1, 接近平衡点时减速) */
float s_abs_alpha = powf(fabsf(s), 0.5f);  // α = 0.5
float s_dot = -smc->eta * s_abs_alpha * sat(s, smc->epsilon);
```

---

## 五、电赛实战技巧

### 5.1 差速驱动小车

```
左轮PWM = 直行速度SMC + 转向SMC
右轮PWM = 直行速度SMC - 转向SMC
```

### 5.2 倒立摆/平衡车

```
外环(位置SMC) → 目标角度
内环(角度SMC) → PWM输出
角度环频率 ≥ 100Hz，位置环频率 10~50Hz
```

### 5.3 调参口诀

```
"λ定速度，η定抗扰，ε定平滑"
"先粗后细，先外后内"
"宁可慢一点，不要抖起来"
```

---

*最后更新：2025年电赛备赛*
