# 2013年C题 旋转倒立摆 - LQR最优控制应用

## 一、为什么倒立摆适合用LQR？

### 1.1 系统特点
- 多状态变量：摆杆角度θ、角速度ω、旋转臂角度α、角速度α'
- 非线性系统，但在倒立点附近可线性化
- 需要同时控制多个状态

### 1.2 LQR vs PID 对比

| 指标 | 多环PID | LQR |
|------|---------|-----|
| 设计方法 | 试凑 | 系统化（Riccati方程） |
| 最优性 | 无保证 | 最优（最小化代价函数） |
| 稳定性 | 需仔细调参 | 理论保证 |
| 多变量 | 分解为多个单环 | 统一处理 |
| 抗扰 | 积分补偿 | 状态观测器 |

## 二、状态空间模型

### 2.1 状态向量
x = [θ-π, ω, α, α']ᵀ

其中：
- θ-π：摆杆偏离倒立点的角度
- ω：摆杆角速度
- α：旋转臂角度
- α'：旋转臂角速度

### 2.2 线性化模型（倒立点附近）
x' = Ax + Bu

A = [0  1   0   0  ]
    [g/L 0  0   0  ]
    [0  0   0   1  ]
    [0  0   0   0  ]

B = [0]
    [-L1/L]
    [0]
    [1]

### 2.3 权重矩阵
Q = diag([100, 10, 10, 1])  // 状态权重
R = [1]                      // 控制量权重

## 三、MATLAB求解K

```matlab
% 系统参数
L = 0.15;   % 摆杆长度(m)
L1 = 0.20;  % 旋转臂长度(m)
g = 9.8;

% 状态空间模型
A = [0 1 0 0; g/L 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0; -L1/L; 0; 1];

% 权重矩阵
Q = diag([100, 10, 10, 1]);
R = [1];

% 求解LQR增益
[K, S, e] = lqr(A, B, Q, R);

% K = [-31.6, -8.2, -3.2, -1.8] (示例值)
```

## 四、STM32实现

```c
LQR_t lqr_pendulum;

void LQR_Init_Pendulum(void)
{
    LQR_Init(&lqr_pendulum, 4, 1, -999, 999);
    
    // MATLAB计算的K值
    float K[4] = {-31.6f, -8.2f, -3.2f, -1.8f};
    LQR_SetGain(&lqr_pendulum, K);
}

void Control_Loop(void)
{
    // 读取状态
    float theta = Encoder_GetAngle() - PI;  // 偏离倒立点
    float omega = Encoder_GetAngularVelocity();
    float alpha = Motor_GetArmAngle();
    float alpha_dot = Motor_GetArmVelocity();
    
    // 设置状态
    LQR_SetState(&lqr_pendulum, 0, theta);
    LQR_SetState(&lqr_pendulum, 1, omega);
    LQR_SetState(&lqr_pendulum, 2, alpha);
    LQR_SetState(&lqr_pendulum, 3, alpha_dot);
    
    // 计算控制量
    float pwm = LQR_Calculate(&lqr_pendulum);
    
    // 输出
    Motor_SetPWM((int16_t)pwm);
}
```

## 五、预期改进效果

1. 摆起倒立更平稳（最优控制）
2. 倒立保持时间更长（理论保证稳定）
3. 抗干扰能力更强（多状态统一控制）
4. 参数整定更系统化（MATLAB辅助设计）
