# MPC模型预测控制实战指南 - 从入门到精通

> 电赛高端控制算法，适合有约束优化场景

---

## 一、MPC原理速通

### 1.1 核心思想

MPC在每个采样时刻：
1. 测量当前状态
2. 基于**模型**预测未来N步的状态
3. 求解**优化问题**，找到最优控制序列
4. 只执行第一步控制量，下一时刻重复

```
时刻k: 测量 → 预测N步 → 优化 → 取u(k) → 执行
时刻k+1: 测量 → 预测N步 → 优化 → 取u(k+1) → 执行
```

### 1.2 为什么电赛要用MPC？

- **可加约束**：PWM范围、角度范围、速度上限（PID/SMC不行）
- **多目标优化**：跟踪精度+控制量平滑+状态约束
- **前馈能力**：已知参考轨迹时提前规划

### 1.3 代价函数（核心）

```
J = Σ[Q·(x-xref)² + R·Δu²] + P·(xN-xref_N)²
    i=0→N-1

Q: 状态跟踪权重（跟踪精度）
R: 控制增量权重（平滑性）
P: 终端权重（稳定性）
N: 预测步长（通常5~20）
```

---

## 二、STM32轻量级MPC实现

### 2.1 一阶系统MPC（适合温度/速度控制）

```c
/* ========== 轻量MPC（一阶系统） ========== */
#define MPC_N  10   // 预测步长

typedef struct {
    float A, B;         // 模型参数: x(k+1) = A*x(k) + B*u(k)
    float Q, R, P;      // 权重
    float u_min, u_max; // 控制量约束
    float du_max;       // 控制增量约束
    float x_pred[MPC_N+1];  // 预测序列
    float u_opt[MPC_N];     // 最优控制序列
} MPC_1st_t;

void MPC1_Init(MPC_1st_t *mpc, float A, float B, float Q, float R)
{
    mpc->A = A;
    mpc->B = B;
    mpc->Q = Q;
    mpc->R = R;
    mpc->P = Q * 5;      // 终端权重通常取Q的倍数
    mpc->u_min = -999;
    mpc->u_max = 999;
    mpc->du_max = 50;     // 每步最大变化量
}

/* 简单网格搜索求解（适合单输入单输出） */
float MPC1_Solve(MPC_1st_t *mpc, float x_now, float ref, float u_prev)
{
    float best_cost = 1e12f;
    float best_u = u_prev;

    // 网格搜索：在 [u_min, u_max] 之间搜索
    int steps = 50;
    float u_range = mpc->u_max - mpc->u_min;
    float du;

    for (int i = 0; i <= steps; i++) {
        float u_try = mpc->u_min + u_range * i / steps;

        // 约束：控制增量限制
        du = u_try - u_prev;
        if (du > mpc->du_max) continue;
        if (du < -mpc->du_max) continue;

        // 预测N步
        float cost = 0;
        float x = x_now;
        float u = u_try;

        for (int k = 0; k < MPC_N; k++) {
            float e = ref - x;
            cost += mpc->Q * e * e + mpc->R * du * du;

            // 一阶模型预测
            x = mpc->A * x + mpc->B * u;
            // 假设后续控制量不变
        }
        // 终端代价
        float e_N = ref - x;
        cost += mpc->P * e_N * e_N;

        if (cost < best_cost) {
            best_cost = cost;
            best_u = u_try;
        }
    }

    return best_u;
}

/* ========== 使用示例：电机速度MPC ========== */
MPC_1st_t speed_mpc;
float u_prev = 0;

void MPC_Speed_Init(void)
{
    // 电机模型辨识: v(k+1) = 0.9*v(k) + 0.5*u(k)
    // 通过阶跃响应辨识得到A和B
    MPC1_Init(&speed_mpc, 0.92f, 0.45f, 1.0f, 0.01f);
    speed_mpc.u_min = -800;
    speed_mpc.u_max = 800;
    speed_mpc.du_max = 100;
}

// 10ms调用一次
float MPC_Speed_Control(float target_rpm, float actual_rpm)
{
    float u = MPC1_Solve(&speed_mpc, actual_rpm, target_rpm, u_prev);
    u_prev = u;
    return u;
}
```

### 2.2 二阶系统MPC（适合位置/角度控制）

```c
/* ========== 二阶MPC ========== */
typedef struct {
    float A[2][2];  // 状态矩阵
    float B[2];     // 输入矩阵
    float Q[2];     // 状态权重 [位置权重, 速度权重]
    float R;        // 控制权重
    float P[2];     // 终端权重
    float u_min, u_max;
    float du_max;
    int N;          // 预测步长
} MPC_2nd_t;

void MPC2_Init(MPC_2nd_t *mpc)
{
    // 典型二阶离散模型 (采样时间Ts=0.01s)
    // x = [位置, 速度]'
    mpc->A[0][0] = 1.0f;  mpc->A[0][1] = 0.01f;
    mpc->A[1][0] = 0.0f;  mpc->A[1][1] = 0.95f;
    mpc->B[0] = 0.00005f;
    mpc->B[1] = 0.01f;

    mpc->Q[0] = 100.0f;   // 位置跟踪权重（大→精确跟踪）
    mpc->Q[1] = 1.0f;     // 速度权重
    mpc->R = 0.1f;         // 控制量权重（大→平滑）
    mpc->P[0] = 500.0f;
    mpc->P[1] = 5.0f;

    mpc->u_min = -900;
    mpc->u_max = 900;
    mpc->du_max = 80;
    mpc->N = 15;
}

float MPC2_Solve(MPC_2nd_t *mpc, float pos, float vel,
                 float ref_pos, float ref_vel, float u_prev)
{
    float best_cost = 1e15f;
    float best_u = u_prev;
    int steps = 40;
    float u_range = mpc->u_max - mpc->u_min;

    for (int i = 0; i <= steps; i++) {
        float u_try = mpc->u_min + u_range * i / steps;
        float du = u_try - u_prev;
        if (fabsf(du) > mpc->du_max) continue;

        float x0 = pos, x1 = vel;
        float cost = 0;
        float u = u_try;

        for (int k = 0; k < mpc->N; k++) {
            float e0 = ref_pos - x0;
            float e1 = ref_vel - x1;
            cost += mpc->Q[0] * e0 * e0 + mpc->Q[1] * e1 * e1
                  + mpc->R * du * du;

            // 状态预测
            float new_x0 = mpc->A[0][0]*x0 + mpc->A[0][1]*x1 + mpc->B[0]*u;
            float new_x1 = mpc->A[1][0]*x0 + mpc->A[1][1]*x1 + mpc->B[1]*u;
            x0 = new_x0;
            x1 = new_x1;
        }
        // 终端代价
        float e0_N = ref_pos - x0;
        float e1_N = ref_vel - x1;
        cost += mpc->P[0]*e0_N*e0_N + mpc->P[1]*e1_N*e1_N;

        if (cost < best_cost) {
            best_cost = cost;
            best_u = u_try;
        }
    }
    return best_u;
}
```

---

## 三、模型辨识方法

### 3.1 阶跃响应法（最简单）

```c
/*
 * 步骤：
 * 1. 给系统施加阶跃输入 u = step_val
 * 2. 记录输出 x(k) 序列
 * 3. 计算模型参数
 *
 * 一阶系统: x(k+1) = A*x(k) + B*u
 * A = x(k+1) / x(k)  (稳态前)
 * B = x_ss / (step_val * (1-A))  或  B = (x(k+1) - A*x(k)) / step_val
 */

// 数据采集示例
#define STEP_DATA_LEN 200
float step_response[STEP_DATA_LEN];

void Collect_StepResponse(float step_input)
{
    Motor_SetPWM((int)step_input);
    for (int i = 0; i < STEP_DATA_LEN; i++) {
        step_response[i] = Encoder_GetSpeed();
        HAL_Delay(10);  // 10ms采样
    }
}

// 离线计算A和B（可PC端做）
// A ≈ x[10] / x[9]
// B ≈ (x[10] - A * x[9]) / step_input
```

### 3.2 最小二乘法辨识

```python
# Python辅助辨识脚本
import numpy as np

# 采集的数据
u_data = np.array([...])  # 输入序列
x_data = np.array([...])  # 输出序列

# x(k+1) = A*x(k) + B*u(k)
# 构建 Y = X @ theta
X = np.column_stack([x_data[:-1], u_data[:-1]])
Y = x_data[1:]

theta = np.linalg.lstsq(X, Y, rcond=None)[0]
A, B = theta[0], theta[1]
print(f"A = {A:.4f}, B = {B:.6f}")
```

---

## 四、参数整定指南

### 4.1 调参优先级

```
1. N（预测步长）：先设 N=10~15，过大则计算量大
2. Q（跟踪权重）：从Q=1开始，增大→跟踪更紧
3. R（控制权重）：从R=0.1开始，增大→控制更平滑
4. du_max：设为PWM最大变化量的10%~20%
5. u_max/u_min：设为实际PWM限幅
```

### 4.2 Q/R比值对性能的影响

| Q/R比值 | 跟踪性 | 平滑性 | 适用场景 |
|---------|--------|--------|---------|
| 大（1000+） | 精确 | 差，抖动 | 精确定位 |
| 中（10~100） | 均衡 | 中等 | 大多数场景 |
| 小（<1） | 差 | 好，平滑 | 对平滑性要求高 |

### 4.3 计算量优化（STM32关键！）

```
问题：MPC每步都要搜索，计算量大
解决：
1. 减少搜索步数（50→20）
2. 用上一帧最优解做初始猜测，缩小搜索范围
3. 预测步长N不要超过15
4. 用定点数替代浮点数（高级技巧）
5. 降低MPC调用频率（如50Hz而非100Hz）
```

```c
/* 智能搜索：以上次最优解为中心搜索 */
float MPC_SmartSearch(MPC_1st_t *mpc, float x_now, float ref,
                      float u_prev, float u_last_opt)
{
    float best_cost = 1e12f;
    float best_u = u_last_opt;

    // 在上次最优解 ±2*du_max 范围搜索
    float center = u_last_opt;
    float range = mpc->du_max * 3;
    int steps = 20;

    for (int i = 0; i <= steps; i++) {
        float u_try = (center - range) + 2 * range * i / steps;
        if (u_try < mpc->u_min || u_try > mpc->u_max) continue;
        // ... 同上计算代价 ...
    }
    return best_u;
}
```

---

## 五、电赛实战应用

### 5.1 平衡小车MPC

```c
// 状态: [角度, 角速度, 位置, 速度]
// 输入: 电机PWM
// 约束: |角度|<30°, |PWM|<999

void Balance_MPC_Loop(void)
{
    float angle = MPU6050_GetAngle();
    float gyro = MPU6050_GetGyro();
    float pos = Encoder_GetPosition();
    float vel = Encoder_GetVelocity();

    float u = MPC2_Solve(&balance_mpc, pos, vel,
                         target_pos, 0, u_prev);
    u_prev = u;
    Motor_SetPWM((int)u);
}
```

### 5.2 轨迹跟踪MPC（电磁循迹）

```c
// 预设参考轨迹点
float ref_traj[100] = {...};  // 预存轨迹
int traj_idx = 0;

void Trajectory_MPC_Loop(void)
{
    // 提前N步获取参考轨迹
    float ref_now = ref_traj[traj_idx];
    float ref_ahead = ref_traj[(traj_idx + 5) % 100];

    float u = MPC1_Solve(&traj_mpc, actual_pos, ref_ahead, u_prev);
    u_prev = u;
    traj_idx = (traj_idx + 1) % 100;
}
```

---

## 六、MPC vs PID vs SMC 选择

| 特性 | PID | SMC | MPC |
|------|-----|-----|-----|
| 实现难度 | ★☆☆ | ★★☆ | ★★★ |
| 计算量 | 小 | 小 | 大 |
| 抗干扰 | 一般 | 强 | 一般 |
| 约束处理 | 不行 | 不行 | **天然支持** |
| 轨迹跟踪 | 一般 | 好 | **最好** |
| 电赛推荐度 | 必备 | 推荐 | 进阶可选 |

**建议**：PID是基础，SMC用于对抗扰要求高的场景，MPC用于有约束或轨迹跟踪的高端题。

---

*最后更新：2025年电赛备赛*
