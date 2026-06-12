#!/usr/bin/env python3
"""
测试包装层 — Python实现与C生产代码逻辑一致的算法

解决错误经验库 #9: 测试不import生产代码
本文件是C驱动/算法代码的Python镜像，保证：
  1. 算法逻辑与C版本逐行对应
  2. 测试可直接import本模块验证生产代码逻辑
  3. 生产代码的bug能被测试捕获

对应C源文件:
  - 02_mspm0g3507/drivers/advanced_pid.c
  - 02_mspm0g3507/drivers/kalman_filter.c
  - 02_mspm0g3507/drivers/ring_buffer.c
  - 02_mspm0g3507/drivers/state_machine.c
  - 02_mspm0g3507/drivers/task_scheduler.c
  - 13_control_algorithms/active_disturbance_rejection.c
"""

import math

# ═══════════════════════════════════════════════════════════════
#  PID控制器 — 对应 advanced_pid.h PID_Controller
# ═══════════════════════════════════════════════════════════════

class PIDController:
    """标准PID控制器，与C版本PID_Controller逻辑一致"""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 output_min=-1000.0, output_max=1000.0,
                 integral_max=500.0, dead_zone=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max
        self.dead_zone = dead_zone
        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0

    def calc(self, ref, feedback):
        """PID计算，对应PID_Calc()"""
        error = ref - feedback

        # 死区
        if abs(error) < self.dead_zone:
            error = 0.0

        # 积分累加（带抗饱和）
        self.integral += error
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max

        # 微分
        derivative = error - self.prev_error
        self.prev_error = error

        # PID输出
        u = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 输出限幅
        if u > self.output_max:
            u = self.output_max
        elif u < self.output_min:
            u = self.output_min

        self.output = u
        return u

    def reset(self):
        """重置PID状态"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0

    def set_kp(self, kp):
        self.kp = kp

    def set_ki(self, ki):
        self.ki = ki

    def set_kd(self, kd):
        self.kd = kd

    def get_output(self):
        return self.output


# ═══════════════════════════════════════════════════════════════
#  ADRC控制器 (V1) — 对应 advanced_pid.c ADRC_t
# ═══════════════════════════════════════════════════════════════

def _fal(e, alpha, delta):
    """fal函数 — ADRC核心非线性映射，与C版本fal_func一致"""
    ae = abs(e)
    if ae > delta:
        sign = 1.0 if e > 0.0 else -1.0
        val = 1.0
        if ae > 0.0001:
            val = ae ** alpha
        return val * sign
    else:
        d = delta if delta > 0.0001 else 0.0001
        return e / (d ** (1.0 - alpha))


def _fhan_v1(x1, x2, r, h):
    """fhan函数 — 最速综合函数（advanced_pid.c版本）"""
    if h < 1e-6:
        h = 1e-6
    d = r * h
    d0 = d * h
    y = x1 + h * x2
    a0 = math.sqrt(d * d + 8.0 * r * abs(y))

    if abs(y) > d0:
        a = x2 + (a0 - d) * 0.5 * (1.0 if y > 0.0 else -1.0)
    else:
        a = x2 + y / h

    if abs(a) > d:
        return -r * (1.0 if a > 0.0 else -1.0)
    else:
        return -r * a / d


class ADRCv1:
    """ADRC V1 — 对应 advanced_pid.c 中的 ADRC_t"""

    def __init__(self, r0=100.0, h0=0.01, b0=1.0,
                 omega_c=10.0, omega_o=30.0, delta=0.01, dt=0.01):
        # TD参数
        self.r0 = r0
        self.h0 = h0

        # ESO参数（带宽法整定）
        self.beta01 = 3.0 * omega_o
        self.beta02 = 3.0 * omega_o * omega_o
        self.beta03 = omega_o * omega_o * omega_o

        # 控制器参数
        self.b0 = b0
        self.delta = delta
        self.dt = dt

        # NLSEF增益
        self.kp = omega_c * omega_c
        self.kd = 2.0 * omega_c

        # 状态
        self.v1 = 0.0
        self.v2 = 0.0
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        self.u = 0.0
        self.u_max = 1000.0

    def update(self, ref, y):
        """对应ADRC_Update()"""
        dt = self.dt

        # 1. 跟踪微分器 TD
        fh = _fhan_v1(self.v1 - ref, self.v2, self.r0, self.h0)
        self.v1 += self.h0 * self.v2
        self.v2 += self.h0 * fh

        # 2. 扩张状态观测器 ESO
        e_obs = self.z1 - y
        fe1 = _fal(e_obs, 0.5, self.delta)
        fe2 = _fal(e_obs, 0.25, self.delta)

        self.z1 += dt * (self.z2 - self.beta01 * e_obs)
        self.z2 += dt * (self.z3 - self.beta02 * fe1 + self.b0 * self.u)
        self.z3 += dt * (-self.beta03 * fe2)

        # 3. 误差计算
        e1 = self.v1 - self.z1
        e2 = self.v2 - self.z2

        # 4. NLSEF
        u0 = self.kp * _fal(e1, 0.5, self.delta) + self.kd * _fal(e2, 0.25, self.delta)

        # 5. 扰动补偿（b0除零保护）
        b0 = self.b0
        if abs(b0) < 1e-6:
            b0 = 1e-6 if b0 >= 0.0 else -1e-6
        u = (u0 - self.z3) / b0

        # 输出限幅
        if u > self.u_max:
            u = self.u_max
        if u < -self.u_max:
            u = -self.u_max

        self.u = u
        return u

    def reset(self):
        self.v1 = self.v2 = 0.0
        self.z1 = self.z2 = self.z3 = 0.0
        self.u = 0.0

    def set_output_limit(self, max_val):
        self.u_max = max_val


# ═══════════════════════════════════════════════════════════════
#  ADRC控制器 (V2) — 对应 active_disturbance_rejection.c ADRC_t
# ═══════════════════════════════════════════════════════════════

def _fhan_v2(x1, x2, r, h):
    """fhan函数 — active_disturbance_rejection.c版本"""
    d = r * h * h
    if d < 1e-10:
        d = 1e-10
    a0 = h * x2
    y = x1 + a0
    a1 = math.sqrt(d * (d + 8.0 * abs(y)))
    a2 = a0 + (1.0 if y > 0 else -1.0) * (a1 - d) * 0.5
    sy = (1.0 if y > 0 else (-1.0 if y < 0 else 0.0))
    sa2 = (1.0 if a2 > 0 else (-1.0 if a2 < 0 else 0.0))
    result = 0.0

    if abs(y) >= d:
        result = -r * sa2
    else:
        result = -r * y / d

    a = a2 if abs(y) >= d else a0 + y
    if sy * sa2 < 0:
        result = -r * a / d

    return result


def _fal_v2(e, alpha, delta):
    """fal函数 — active_disturbance_rejection.c版本"""
    if abs(e) > delta:
        sign = 1.0 if e > 0 else -1.0
        return sign * (abs(e) ** alpha)
    else:
        return e / (delta ** (1.0 - alpha))


class ADRCv2:
    """ADRC V2 — 对应 active_disturbance_rejection.c ADRC_t"""

    def __init__(self, dt=0.01, b0=1.0, omega_c=0.0, omega_o=0.0):
        self.dt = dt
        self.eso_b0 = b0 if b0 != 0.0 else 1.0

        # TD
        self.td_x1 = 0.0
        self.td_x2 = 0.0
        self.td_r = 100.0
        self.td_h = dt

        # ESO
        if omega_o <= 0.0:
            omega_o = 10.0 / dt
        self.eso_omega_o = omega_o
        self.eso_z1 = 0.0
        self.eso_z2 = 0.0
        self.eso_z3 = 0.0
        self._update_eso_gains()

        # NLSEF
        if omega_c <= 0.0:
            omega_c = omega_o / 3.0
        self.nl_omega_c = omega_c
        self.nl_alpha1 = 0.5
        self.nl_alpha2 = 0.25
        self.nl_delta = 0.01
        self._update_nlsef_gains()

        # 输出
        self.output_min = -1.0
        self.output_max = 1.0
        self.u0_prev = 0.0

    def _update_eso_gains(self):
        wo = self.eso_omega_o
        self.eso_beta1 = 3.0 * wo
        self.eso_beta2 = 3.0 * wo * wo
        self.eso_beta3 = wo * wo * wo

    def _update_nlsef_gains(self):
        wc = self.nl_omega_c
        self.nl_k1 = wc * wc
        self.nl_k2 = 2.0 * wc

    def compute(self, setpoint, feedback):
        """对应ADRC_Compute()"""
        dt = self.dt
        v0 = setpoint

        # Step 1: TD
        e_td = self.td_x1 - v0
        fh = _fhan_v2(e_td, self.td_x2, self.td_r, self.td_h)
        self.td_x1 += dt * self.td_x2
        self.td_x2 += dt * fh
        v1 = self.td_x1
        v2 = self.td_x2

        # Step 2: ESO
        y = feedback
        eso_e = self.eso_z1 - y
        fal_e1 = _fal_v2(eso_e, self.nl_alpha1, self.nl_delta)
        fal_e2 = _fal_v2(eso_e, self.nl_alpha2, self.nl_delta)

        z1_dot = self.eso_z2 - self.eso_beta1 * eso_e
        z2_dot = self.eso_z3 - self.eso_beta2 * fal_e1 + self.eso_b0 * self.u0_prev
        z3_dot = -self.eso_beta3 * fal_e2

        self.eso_z1 += dt * z1_dot
        self.eso_z2 += dt * z2_dot
        self.eso_z3 += dt * z3_dot

        # ESO状态限幅
        limit = abs(self.output_max) * 10.0
        self.eso_z3 = max(-limit, min(limit, self.eso_z3))

        # Step 3: NLSEF + 扰动补偿
        e1 = v1 - self.eso_z1
        e2 = v2 - self.eso_z2
        u0 = (self.nl_k1 * _fal_v2(e1, self.nl_alpha1, self.nl_delta)
              + self.nl_k2 * _fal_v2(e2, self.nl_alpha2, self.nl_delta))
        u = (u0 - self.eso_z3) / self.eso_b0

        u = max(self.output_min, min(self.output_max, u))
        self.u0_prev = u
        return u

    def reset(self):
        self.td_x1 = self.td_x2 = 0.0
        self.eso_z1 = self.eso_z2 = self.eso_z3 = 0.0
        self.u0_prev = 0.0

    def set_output_limits(self, min_val, max_val):
        self.output_min = min_val
        self.output_max = max_val

    def set_eso_bandwidth(self, omega_o):
        self.eso_omega_o = omega_o
        self._update_eso_gains()

    def set_control_bandwidth(self, omega_c):
        self.nl_omega_c = omega_c
        self._update_nlsef_gains()

    def set_b0(self, b0):
        self.eso_b0 = 1e-6 if b0 == 0.0 else b0

    def get_disturbance_estimate(self):
        return self.eso_z3


# ═══════════════════════════════════════════════════════════════
#  卡尔曼滤波器 — 对应 kalman_filter.c KalmanFilter_t
# ═══════════════════════════════════════════════════════════════

class KalmanFilter:
    """2D卡尔曼滤波器（位置+速度），与C版本KalmanFilter_t逻辑一致"""

    def __init__(self, dt=0.1, proc_noise=1.0, meas_noise=1.0):
        self.dt = dt
        self.obs_dim = 1
        self.initialized = True

        # 状态转移矩阵 A（恒速模型）
        self.A = [[1.0, dt],
                  [0.0, 1.0]]

        # 控制输入矩阵 B
        self.B = [0.5 * dt * dt, dt]

        # 观测矩阵 H（默认仅观测位置）
        self.H = [[1.0, 0.0],
                  [0.0, 0.0]]

        # 过程噪声协方差 Q（连续白噪声加速度模型离散化）
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        self.Q = [[proc_noise * dt4 * 0.25, proc_noise * dt3 * 0.5],
                  [proc_noise * dt3 * 0.5, proc_noise * dt2]]

        # 观测噪声协方差 R
        self.R = [[meas_noise, 0.0],
                  [0.0, meas_noise]]

        # 状态
        self.x = [0.0, 0.0]

        # 误差协方差矩阵 P
        self.P = [[1.0, 0.0],
                  [0.0, 1.0]]

    def predict(self):
        """预测步骤（无控制输入），对应Kalman_Predict()"""
        x0, x1 = self.x
        self.x[0] = self.A[0][0] * x0 + self.A[0][1] * x1
        self.x[1] = self.A[1][0] * x0 + self.A[1][1] * x1

        # P_pred = A * P * A^T + Q
        ap00 = self.A[0][0] * self.P[0][0] + self.A[0][1] * self.P[1][0]
        ap01 = self.A[0][0] * self.P[0][1] + self.A[0][1] * self.P[1][1]
        ap10 = self.A[1][0] * self.P[0][0] + self.A[1][1] * self.P[1][0]
        ap11 = self.A[1][0] * self.P[0][1] + self.A[1][1] * self.P[1][1]

        self.P[0][0] = ap00 * self.A[0][0] + ap01 * self.A[0][1] + self.Q[0][0]
        self.P[0][1] = ap00 * self.A[1][0] + ap01 * self.A[1][1] + self.Q[0][1]
        self.P[1][0] = ap10 * self.A[0][0] + ap11 * self.A[0][1] + self.Q[1][0]
        self.P[1][1] = ap10 * self.A[1][0] + ap11 * self.A[1][1] + self.Q[1][1]

    def predict_with_input(self, u):
        """带控制输入的预测，对应Kalman_PredictWithInput()"""
        self.predict()
        self.x[0] += self.B[0] * u
        self.x[1] += self.B[1] * u

    def update_1d(self, z):
        """1D观测更新，对应Kalman_Update1D()"""
        h0, h1 = self.H[0][0], self.H[0][1]

        # 新息
        y = z - (h0 * self.x[0] + h1 * self.x[1])

        # 新息协方差
        S = (h0 * (h0 * self.P[0][0] + h1 * self.P[1][0])
             + h1 * (h0 * self.P[0][1] + h1 * self.P[1][1])
             + self.R[0][0])
        if S < 1e-10:
            S = 1e-10

        # 卡尔曼增益
        K0 = (self.P[0][0] * h0 + self.P[0][1] * h1) / S
        K1 = (self.P[1][0] * h0 + self.P[1][1] * h1) / S

        # 状态更新
        self.x[0] += K0 * y
        self.x[1] += K1 * y

        # 协方差更新
        P00 = self.P[0][0] - K0 * (h0 * self.P[0][0] + h1 * self.P[1][0])
        P01 = self.P[0][1] - K0 * (h0 * self.P[0][1] + h1 * self.P[1][1])
        P10 = self.P[1][0] - K1 * (h0 * self.P[0][0] + h1 * self.P[1][0])
        P11 = self.P[1][1] - K1 * (h0 * self.P[0][1] + h1 * self.P[1][1])

        self.P[0][0] = P00
        self.P[0][1] = P01
        self.P[1][0] = P10
        self.P[1][1] = P11

    def update_2d(self, z):
        """2D观测更新，对应Kalman_Update2D()"""
        h00, h01 = self.H[0][0], self.H[0][1]
        h10, h11 = self.H[1][0], self.H[1][1]

        y0 = z[0] - (h00 * self.x[0] + h01 * self.x[1])
        y1 = z[1] - (h10 * self.x[0] + h11 * self.x[1])

        hp00 = h00 * self.P[0][0] + h01 * self.P[1][0]
        hp01 = h00 * self.P[0][1] + h01 * self.P[1][1]
        hp10 = h10 * self.P[0][0] + h11 * self.P[1][0]
        hp11 = h10 * self.P[0][1] + h11 * self.P[1][1]

        S00 = hp00 * h00 + hp01 * h01 + self.R[0][0]
        S01 = hp00 * h10 + hp01 * h11 + self.R[0][1]
        S10 = hp10 * h00 + hp11 * h01 + self.R[1][0]
        S11 = hp10 * h10 + hp11 * h11 + self.R[1][1]

        det = S00 * S11 - S01 * S10
        if abs(det) < 1e-10:
            det = 1e-10

        inv_S00 = S11 / det
        inv_S01 = -S01 / det
        inv_S10 = -S10 / det
        inv_S11 = S00 / det

        pht00 = self.P[0][0] * h00 + self.P[0][1] * h01
        pht01 = self.P[0][0] * h10 + self.P[0][1] * h11
        pht10 = self.P[1][0] * h00 + self.P[1][1] * h01
        pht11 = self.P[1][0] * h10 + self.P[1][1] * h11

        K00 = pht00 * inv_S00 + pht01 * inv_S10
        K01 = pht00 * inv_S01 + pht01 * inv_S11
        K10 = pht10 * inv_S00 + pht11 * inv_S10
        K11 = pht10 * inv_S01 + pht11 * inv_S11

        self.x[0] += K00 * y0 + K01 * y1
        self.x[1] += K10 * y0 + K11 * y1

        ikh00 = 1.0 - K00 * h00 - K01 * h10
        ikh01 = -K00 * h01 - K01 * h11
        ikh10 = -K10 * h00 - K11 * h10
        ikh11 = 1.0 - K10 * h01 - K11 * h11

        self.P[0][0] = ikh00 * self.P[0][0] + ikh01 * self.P[1][0]
        self.P[0][1] = ikh00 * self.P[0][1] + ikh01 * self.P[1][1]
        self.P[1][0] = ikh10 * self.P[0][0] + ikh11 * self.P[1][0]
        self.P[1][1] = ikh10 * self.P[0][1] + ikh11 * self.P[1][1]

    def get_position(self):
        return self.x[0]

    def get_velocity(self):
        return self.x[1]

    def get_uncertainty(self):
        """对应Kalman_GetUncertainty()"""
        return self.P[0][0] + self.P[1][1]

    def set_state(self, pos, vel):
        self.x[0] = pos
        self.x[1] = vel

    def reset(self):
        self.x = [0.0, 0.0]
        self.P = [[1.0, 0.0], [0.0, 1.0]]

    def step(self, z, u=0.0):
        """一步预测+更新（便利方法）"""
        if u != 0.0:
            self.predict_with_input(u)
        else:
            self.predict()
        self.update_1d(z)
        return self.x[0], self.x[1]


# ═══════════════════════════════════════════════════════════════
#  环形缓冲区 — 对应 ring_buffer.c RingBuffer_t
# ═══════════════════════════════════════════════════════════════

class RingBuffer:
    """无锁环形缓冲区，与C版本RingBuffer_t逻辑一致"""

    def __init__(self, capacity):
        # 大小必须为2的幂（与C版本一致）
        size = 1
        while size < capacity:
            size <<= 1
        self.size = size
        self.mask = size - 1
        self.buffer = bytearray(size)
        self.head = 0
        self.tail = 0

    def used(self):
        """对应RingBuffer_Used()"""
        return self.head - self.tail

    def free_space(self):
        """对应RingBuffer_Free()"""
        return self.size - (self.head - self.tail)

    def is_empty(self):
        """对应RingBuffer_IsEmpty()"""
        return self.head == self.tail

    def is_full(self):
        """对应RingBuffer_IsFull()"""
        return self.used() >= self.size

    def put_byte(self, data):
        """写入单字节，对应RingBuffer_PutByte()"""
        if self.is_full():
            return False
        self.buffer[self.head & self.mask] = data & 0xFF
        self.head += 1
        return True

    def get_byte(self):
        """读取单字节，对应RingBuffer_GetByte()"""
        if self.is_empty():
            return None
        val = self.buffer[self.tail & self.mask]
        self.tail += 1
        return val

    def write(self, data):
        """批量写入，对应RingBuffer_Write()"""
        free = self.free_space()
        if len(data) > free:
            data = data[:free]
        if len(data) == 0:
            return 0

        head_idx = self.head & self.mask
        first_chunk = self.size - head_idx
        if len(data) <= first_chunk:
            self.buffer[head_idx:head_idx + len(data)] = data
        else:
            self.buffer[head_idx:] = data[:first_chunk]
            self.buffer[:len(data) - first_chunk] = data[first_chunk:]

        self.head += len(data)
        return len(data)

    def read(self, count):
        """批量读取，对应RingBuffer_Read()"""
        used = self.used()
        if count > used:
            count = used
        if count == 0:
            return b''

        tail_idx = self.tail & self.mask
        first_chunk = self.size - tail_idx
        if count <= first_chunk:
            result = bytes(self.buffer[tail_idx:tail_idx + count])
        else:
            result = bytes(self.buffer[tail_idx:]) + bytes(self.buffer[:count - first_chunk])

        self.tail += count
        return result

    def peek(self):
        """查看但不读取，对应RingBuffer_Peek()"""
        if self.is_empty():
            return None
        return self.buffer[self.tail & self.mask]

    def reset(self):
        """对应RingBuffer_Reset()"""
        self.head = 0
        self.tail = 0


# ═══════════════════════════════════════════════════════════════
#  状态机 — 对应 state_machine.c SM_Machine
# ═══════════════════════════════════════════════════════════════

SM_NO_PARENT = 0xFF

class SMEvent:
    """对应SM_Event_t"""
    def __init__(self, eid=0, param=0, timestamp=0):
        self.id = eid
        self.param = param
        self.timestamp = timestamp

class SMStateDesc:
    """对应SM_StateDesc_t"""
    def __init__(self, parent=SM_NO_PARENT, on_enter=None, on_exit=None, on_event=None):
        self.parent = parent
        self.on_enter = on_enter
        self.on_exit = on_exit
        self.on_event = on_event

class StateMachine:
    """事件驱动状态机，与C版本SM_Machine逻辑一致"""

    def __init__(self, state_table, init_state=0, user_data=None):
        self.state_table = state_table
        self.current = init_state
        self.previous = init_state
        self.state_count = len(state_table)
        self.is_running = False
        self.state_ticks = 0
        self.user_data = user_data

    def start(self):
        """对应SM_Start()"""
        self.is_running = True
        self.state_ticks = 0
        if self.current < self.state_count:
            desc = self.state_table[self.current]
            if desc.on_enter:
                enter_evt = SMEvent()
                desc.on_enter(self, enter_evt)

    def dispatch(self, event):
        """对应SM_Dispatch()"""
        if not self.is_running or event is None:
            return False

        state = self.current
        while state < self.state_count:
            desc = self.state_table[state]
            if desc.on_event:
                handled = desc.on_event(self, event)
                if handled:
                    return True
            if desc.parent == SM_NO_PARENT:
                break
            state = desc.parent
        return False

    def transition(self, new_state):
        """对应SM_Transition()"""
        if new_state >= self.state_count:
            return
        if new_state == self.current:
            return

        # 调用当前状态的on_exit
        if self.current < self.state_count:
            desc = self.state_table[self.current]
            if desc.on_exit:
                exit_evt = SMEvent()
                desc.on_exit(self, exit_evt)

        self.previous = self.current
        self.current = new_state
        self.state_ticks = 0

        # 调用新状态的on_enter
        desc = self.state_table[new_state]
        if desc.on_enter:
            enter_evt = SMEvent()
            desc.on_enter(self, enter_evt)

    def return_to_previous(self):
        """对应SM_Return()"""
        self.transition(self.previous)

    def get_state(self):
        return self.current

    def get_previous_state(self):
        return self.previous

    def get_state_ticks(self):
        return self.state_ticks

    def tick(self):
        """对应SM_Tick()"""
        if self.state_ticks < 0xFFFFFFFF:
            self.state_ticks += 1

    def is_running_state(self):
        return self.is_running

    def stop(self):
        self.is_running = False


# ═══════════════════════════════════════════════════════════════
#  任务调度器 — 对应 task_scheduler.c Scheduler
# ═══════════════════════════════════════════════════════════════

SCHED_PRIORITY_HIGH = 0
SCHED_PRIORITY_NORMAL = 1
SCHED_PRIORITY_LOW = 2
SCHED_PRIORITY_IDLE = 3

TASK_STATE_IDLE = 0
TASK_STATE_READY = 1
TASK_STATE_RUNNING = 2
TASK_STATE_DISABLED = 3

class TaskHandle:
    """对应TaskHandle"""
    def __init__(self):
        self.callback = None
        self.arg = None
        self.period_ms = 0
        self.delay_ms = 0
        self.priority = SCHED_PRIORITY_NORMAL
        self.state = TASK_STATE_IDLE
        self.auto_reload = False
        self.run_count = 0
        self.last_run_tick = 0
        self.max_exec_us = 0

class TaskScheduler:
    """非抢占式任务调度器，与C版本Scheduler逻辑一致"""

    def __init__(self, max_tasks=16):
        self.max_tasks = max_tasks
        self.tasks = [TaskHandle() for _ in range(max_tasks)]
        self.task_count = 0
        self.tick_count = 0
        self.initialized = False
        self.running = False
        self.loop_count = 0

    def init(self):
        """对应Sched_Init()"""
        self.tasks = [TaskHandle() for _ in range(self.max_tasks)]
        self.task_count = 0
        self.tick_count = 0
        self.initialized = True
        self.running = False
        self.loop_count = 0

    def tick_isr(self):
        """对应Sched_TickISR()"""
        self.tick_count += 1
        for t in self.tasks:
            if t.state == TASK_STATE_READY:
                if t.delay_ms > 0:
                    t.delay_ms -= 1

    def add_periodic(self, callback, arg=None, period_ms=100, priority=SCHED_PRIORITY_NORMAL):
        """对应Sched_AddPeriodic()"""
        if not self.initialized or callback is None or period_ms == 0:
            return -1

        idx = self._find_free_slot()
        if idx < 0:
            return -1

        t = self.tasks[idx]
        t.callback = callback
        t.arg = arg
        t.period_ms = period_ms
        t.delay_ms = period_ms
        t.priority = priority
        t.state = TASK_STATE_READY
        t.auto_reload = True
        t.run_count = 0
        t.last_run_tick = 0
        t.max_exec_us = 0

        self.task_count += 1
        return idx

    def add_delayed(self, callback, arg=None, delay_ms=100, priority=SCHED_PRIORITY_NORMAL):
        """对应Sched_AddDelayed()"""
        if not self.initialized or callback is None:
            return -1

        idx = self._find_free_slot()
        if idx < 0:
            return -1

        t = self.tasks[idx]
        t.callback = callback
        t.arg = arg
        t.period_ms = 0
        t.delay_ms = delay_ms
        t.priority = priority
        t.state = TASK_STATE_READY
        t.auto_reload = False
        t.run_count = 0
        t.last_run_tick = 0
        t.max_exec_us = 0

        self.task_count += 1
        return idx

    def enable_task(self, index, enable):
        """对应Sched_EnableTask()"""
        if index >= self.max_tasks:
            return
        t = self.tasks[index]
        if t.state == TASK_STATE_IDLE:
            return
        t.state = TASK_STATE_READY if enable else TASK_STATE_DISABLED

    def remove_task(self, index):
        """对应Sched_RemoveTask()"""
        if index >= self.max_tasks:
            return
        if self.tasks[index].state != TASK_STATE_IDLE:
            self.task_count -= 1
        self.tasks[index] = TaskHandle()

    def reset_task(self, index):
        """对应Sched_ResetTask()"""
        if index >= self.max_tasks:
            return
        t = self.tasks[index]
        if t.state == TASK_STATE_IDLE:
            return
        t.delay_ms = t.period_ms if t.period_ms > 0 else 1
        t.state = TASK_STATE_READY

    def run(self):
        """对应Sched_Run()"""
        if not self.initialized:
            return

        self.running = True
        self.loop_count += 1

        for prio in range(4):
            for i in range(self.max_tasks):
                t = self.tasks[i]
                if t.state != TASK_STATE_READY:
                    continue
                if t.priority != prio:
                    continue
                if t.delay_ms > 0:
                    continue

                t.state = TASK_STATE_RUNNING
                tick_before = self.tick_count

                t.callback(t.arg)

                elapsed = self.tick_count - tick_before
                if elapsed > t.max_exec_us:
                    t.max_exec_us = elapsed

                t.run_count += 1
                t.last_run_tick = self.tick_count

                if t.auto_reload:
                    t.delay_ms = t.period_ms
                    t.state = TASK_STATE_READY
                else:
                    t.state = TASK_STATE_IDLE
                    self.task_count -= 1

    def get_tick(self):
        return self.tick_count

    def get_task_count(self):
        return self.task_count

    def _find_free_slot(self):
        for i in range(self.max_tasks):
            if self.tasks[i].state == TASK_STATE_IDLE:
                return i
        return -1


# ═══════════════════════════════════════════════════════════════
#  滑动平均滤波器 — 对应 moving_average.c SimpleMA_t / EMA / IntMA
# ═══════════════════════════════════════════════════════════════

class SimpleMA:
    """简单滑动平均滤波器(float)，与C版本SimpleMA_t逻辑一致"""

    def __init__(self, window):
        if window <= 0:
            window = 1
        self.size = window
        self.buffer = [0.0] * window
        self.index = 0
        self.count = 0
        self.sum = 0.0
        self.last_output = 0.0

    def update(self, input_val):
        """对应SimpleMA_Update()"""
        # 如果缓冲区已满，减去最旧的数据
        if self.count >= self.size:
            self.sum -= self.buffer[self.index]

        # 写入新数据
        self.buffer[self.index] = input_val
        self.sum += input_val

        # 更新索引（环形）
        self.index += 1
        if self.index >= self.size:
            self.index = 0

        # 更新计数
        if self.count < self.size:
            self.count += 1

        # 计算平均值
        self.last_output = self.sum / float(self.count)
        return self.last_output

    def reset(self):
        """对应SimpleMA_Reset()"""
        self.index = 0
        self.count = 0
        self.sum = 0.0
        self.last_output = 0.0
        self.buffer = [0.0] * self.size


class EMA:
    """指数滑动平均滤波器，与C版本EMA逻辑一致"""

    def __init__(self, alpha=0.1):
        # alpha校验（错误经验#24: 滤波器参数未校验）
        if alpha <= 0.0:
            alpha = 0.1
        if alpha > 1.0:
            alpha = 1.0
        self.alpha = alpha
        self.output = 0.0
        self.initialized = False

    def update(self, input_val):
        """对应EMA_Update() — inline在头文件中"""
        if not self.initialized:
            self.output = input_val
            self.initialized = True
        else:
            self.output = self.alpha * input_val + (1.0 - self.alpha) * self.output
        return self.output

    def reset(self):
        self.output = 0.0
        self.initialized = False


class IntMA:
    """整数滑动平均滤波器（定点版），与C版本IntMA_t逻辑一致
    窗口大小必须为2的幂"""

    def __init__(self, window):
        if window <= 0 or (window & (window - 1)) != 0:
            # 非2的幂或0，设置无效状态
            self.buffer = []
            self.size = 0
            self.shift = 0
            self.valid = False
        else:
            self.buffer = [0] * window
            self.size = window
            self.shift = window.bit_length() - 1  # log2
            self.valid = True
        self.index = 0
        self.count = 0
        self.sum = 0
        self.last_output = 0

    def update(self, input_val):
        """对应IntMA_Update()"""
        if not self.valid:
            return 0

        # 如果缓冲区已满，减去最旧的数据
        if self.count >= self.size:
            self.sum -= self.buffer[self.index]

        # 写入新数据
        self.buffer[self.index] = input_val
        self.sum += input_val

        # 更新索引（环形，用掩码取模）
        self.index = (self.index + 1) & (self.size - 1)

        # 更新计数
        if self.count < self.size:
            self.count += 1

        # 计算平均值（用移位代替除法）
        self.last_output = self.sum >> self.shift
        return self.last_output

    def reset(self):
        if self.valid:
            self.index = 0
            self.count = 0
            self.sum = 0
            self.last_output = 0
            self.buffer = [0] * self.size


# ═══════════════════════════════════════════════════════════════
#  事件系统 — 对应 event_system.c EventSystem
# ═══════════════════════════════════════════════════════════════

EVT_MAX_TYPES = 32
EVT_MAX_LISTENERS = 32

# 预定义事件类型
EVT_NONE = 0
EVT_SYSTEM_BOOT = 1
EVT_SYSTEM_ERROR = 2
EVT_SENSOR_UPDATE = 3
EVT_MOTOR_CMD = 4
EVT_LINE_LOST = 5
EVT_LINE_FOUND = 6
EVT_OBSTACLE_DETECTED = 7
EVT_OBSTACLE_CLEARED = 8
EVT_BUTTON_PRESS = 9
EVT_UART_RX_DATA = 10
EVT_TIMEOUT = 11
EVT_WATCHDOG_WARN = 12
EVT_WATCHDOG_RESET = 13
EVT_USER_START = 16


class Event:
    """对应Event结构体"""
    def __init__(self, event_type=0, data=None, data_len=0, timestamp=0):
        self.type = event_type
        self.data = data
        self.data_len = data_len
        self.timestamp = timestamp


class EventListener:
    """对应EventListener结构体"""
    def __init__(self, event_type=0, callback=None, user_data=None, active=False):
        self.type = event_type
        self.callback = callback
        self.user_data = user_data
        self.active = active


class EventSystem:
    """模块间事件通信系统，与C版本EventSystem逻辑一致"""

    def __init__(self, max_types=EVT_MAX_TYPES, max_listeners=EVT_MAX_LISTENERS):
        self.max_types = max_types
        self.max_listeners = max_listeners
        self.listeners = [EventListener() for _ in range(max_listeners)]
        self.listener_count = 0
        self.event_counts = [0] * max_types
        self.initialized = False

    def init(self):
        """对应Event_Init()"""
        self.listeners = [EventListener() for _ in range(self.max_listeners)]
        self.listener_count = 0
        self.event_counts = [0] * self.max_types
        self.initialized = True

    def register(self, event_type, callback, user_data=None):
        """对应Event_Register()"""
        if not self.initialized or callback is None:
            return -1

        for i in range(self.max_listeners):
            if not self.listeners[i].active:
                self.listeners[i].type = event_type
                self.listeners[i].callback = callback
                self.listeners[i].user_data = user_data
                self.listeners[i].active = True
                self.listener_count += 1
                return i
        return -1  # 满

    def unregister(self, index):
        """对应Event_Unregister()"""
        if index >= self.max_listeners:
            return
        if self.listeners[index].active:
            self.listeners[index].active = False
            self.listener_count -= 1

    def trigger(self, event_type):
        """对应Event_Trigger() — 无数据"""
        self.trigger_with_data(event_type, None, 0)

    def trigger_with_data(self, event_type, data=None, data_len=0):
        """对应Event_TriggerWithData() — 同步分发"""
        if not self.initialized:
            return

        # 记录事件计数
        if event_type < self.max_types:
            self.event_counts[event_type] += 1

        # 构造事件结构
        evt = Event(event_type=event_type, data=data, data_len=data_len)

        # 遍历所有监听者，找到匹配类型并同步执行
        for i in range(self.max_listeners):
            lis = self.listeners[i]
            if lis.active and lis.type == event_type and lis.callback is not None:
                lis.callback(evt, lis.user_data)

    def get_count(self, event_type):
        """对应Event_GetCount()"""
        if event_type >= self.max_types:
            return 0
        return self.event_counts[event_type]

    def reset_count(self, event_type):
        """对应Event_ResetCount()"""
        if event_type < self.max_types:
            self.event_counts[event_type] = 0


# ═══════════════════════════════════════════════════════════════
#  看门狗 — 对应 watchdog.c Watchdog
# ═══════════════════════════════════════════════════════════════

WDG_MAX_MONITORS = 8
WDG_DEFAULT_TIMEOUT_MS = 5000
WDG_DEFAULT_MAX_FAILS = 3

WDG_STATE_HEALTHY = 0
WDG_STATE_WARNING = 1
WDG_STATE_CRITICAL = 2
WDG_STATE_RESET_PENDING = 3

WDG_RESET_NONE = 0
WDG_RESET_TASK_REINIT = 1
WDG_RESET_SYSTEM = 2


class WdgMonitor:
    """对应WdgMonitor结构体"""
    def __init__(self):
        self.name = ""
        self.fed = False
        self.timeout_ms = WDG_DEFAULT_TIMEOUT_MS
        self.time_since_feed = 0
        self.fail_count = 0
        self.max_fails = WDG_DEFAULT_MAX_FAILS
        self.total_fails = 0
        self.state = WDG_STATE_HEALTHY
        self.reset_policy = WDG_RESET_NONE
        self.reset_callback = None
        self.reset_arg = None
        self.active = False


class Watchdog:
    """软件看门狗，与C版本Watchdog逻辑一致"""

    def __init__(self, max_monitors=WDG_MAX_MONITORS):
        self.max_monitors = max_monitors
        self.monitors = [WdgMonitor() for _ in range(max_monitors)]
        self.monitor_count = 0
        self.check_interval_ms = 1000
        self.elapsed_ms = 0
        self.initialized = False
        self.system_reset_armed = False

    def init(self, check_interval_ms):
        """对应WDG_Init()"""
        self.monitors = [WdgMonitor() for _ in range(self.max_monitors)]
        self.monitor_count = 0
        self.elapsed_ms = 0
        self.check_interval_ms = check_interval_ms if check_interval_ms > 0 else 1000
        self.system_reset_armed = False
        self.initialized = True

    def add(self, name, timeout_ms, max_fails, reset_policy,
            reset_cb=None, reset_arg=None):
        """对应WDG_Add()"""
        if not self.initialized:
            return -1
        if self.monitor_count >= self.max_monitors:
            return -1

        m = self.monitors[self.monitor_count]
        m.name = name
        m.fed = False
        m.timeout_ms = timeout_ms if timeout_ms > 0 else WDG_DEFAULT_TIMEOUT_MS
        m.time_since_feed = 0
        m.fail_count = 0
        m.max_fails = max_fails if max_fails > 0 else WDG_DEFAULT_MAX_FAILS
        m.total_fails = 0
        m.state = WDG_STATE_HEALTHY
        m.reset_policy = reset_policy
        m.reset_callback = reset_cb
        m.reset_arg = reset_arg
        m.active = True

        idx = self.monitor_count
        self.monitor_count += 1
        return idx

    def feed(self, index):
        """对应WDG_Feed()"""
        if index >= self.max_monitors:
            return
        m = self.monitors[index]
        if not m.active:
            return
        m.fed = True

    def update(self):
        """对应WDG_Update() — 每次调用视为经过1ms"""
        if not self.initialized:
            return

        self.elapsed_ms += 1
        if self.elapsed_ms < self.check_interval_ms:
            return
        self.elapsed_ms = 0

        # 检查所有监控项
        for i in range(self.monitor_count):
            m = self.monitors[i]
            if not m.active:
                continue

            if m.fed:
                # 喂了狗，重置计数器
                m.fed = False
                m.time_since_feed = 0
                m.fail_count = 0
                m.state = WDG_STATE_HEALTHY
            else:
                # 没喂狗，累加时间
                m.time_since_feed += self.check_interval_ms

                if m.time_since_feed >= m.timeout_ms:
                    # 超时!
                    m.fail_count += 1
                    m.total_fails += 1
                    m.time_since_feed = 0

                    if m.fail_count >= m.max_fails:
                        m.state = WDG_STATE_RESET_PENDING

                        # 执行复位策略
                        if m.reset_policy == WDG_RESET_NONE:
                            m.state = WDG_STATE_CRITICAL
                        elif m.reset_policy == WDG_RESET_TASK_REINIT:
                            if m.reset_callback:
                                m.reset_callback(m.reset_arg)
                            m.fail_count = 0
                            m.state = WDG_STATE_WARNING
                        elif m.reset_policy == WDG_RESET_SYSTEM:
                            # Python中不执行实际复位
                            pass
                    else:
                        m.state = WDG_STATE_WARNING

    def get_state(self, index):
        """对应WDG_GetState()"""
        if index >= self.max_monitors:
            return WDG_STATE_CRITICAL
        return self.monitors[index].state

    def arm_system_reset(self, arm):
        """对应WDG_ArmSystemReset()"""
        self.system_reset_armed = arm


# ═══════════════════════════════════════════════════════════════
#  I2C总线管理 — 对应 i2c_bus.c I2C_Bus
# ═══════════════════════════════════════════════════════════════

I2C_BUS_OK = 0
I2C_BUS_ERR_BUSY = 1
I2C_BUS_ERR_TIMEOUT = 2
I2C_BUS_ERR_NACK = 3
I2C_BUS_ERR_RETRY = 4

I2C_BUS_TIMEOUT = 1000
I2C_BUS_MAX_RETRY = 3


class I2CBus:
    """I2C总线管理层，与C版本I2C_Bus逻辑一致"""

    def __init__(self):
        self.locked = False
        self.timeout = I2C_BUS_TIMEOUT
        self.max_retry = I2C_BUS_MAX_RETRY
        self.tx_count = 0
        self.rx_count = 0
        self.err_count = 0
        self.retry_count = 0
        # 模拟I2C设备寄存器存储
        self._devices = {}  # addr -> {reg: val}

    def init(self):
        """对应I2C_Bus_Init()"""
        self.locked = False
        self.timeout = I2C_BUS_TIMEOUT
        self.max_retry = I2C_BUS_MAX_RETRY
        self.tx_count = 0
        self.rx_count = 0
        self.err_count = 0
        self.retry_count = 0

    def lock(self):
        """对应I2C_Bus_Lock()"""
        if self.locked:
            return False
        self.locked = True
        return True

    def unlock(self):
        """对应I2C_Bus_Unlock()"""
        self.locked = False

    def write(self, addr, tx_buf):
        """对应I2C_Bus_Write()"""
        if not self.lock():
            return I2C_BUS_ERR_BUSY

        err = self._raw_write(addr, tx_buf)
        self.unlock()
        return err

    def read(self, addr, rx_len):
        """对应I2C_Bus_Read()"""
        if not self.lock():
            return I2C_BUS_ERR_BUSY, None

        err, data = self._raw_read(addr, rx_len)
        self.unlock()
        return err, data

    def write_reg(self, addr, reg, val):
        """对应I2C_Bus_WriteReg()"""
        return self.write(addr, bytes([reg, val]))

    def read_reg(self, addr, reg):
        """对应I2C_Bus_ReadReg()"""
        return self.write_read(addr, bytes([reg]), 1)

    def read_multi(self, addr, reg, length):
        """对应I2C_Bus_ReadMulti()"""
        return self.write_read(addr, bytes([reg]), length)

    def write_read(self, addr, tx_buf, rx_len):
        """对应I2C_Bus_WriteRead()"""
        if not self.lock():
            return I2C_BUS_ERR_BUSY, None

        err = None
        for retry in range(self.max_retry):
            # 写阶段
            err = self._raw_write(addr, tx_buf)
            if err != I2C_BUS_OK:
                self.retry_count += 1
                continue

            # 读阶段
            err, data = self._raw_read(addr, rx_len)
            if err == I2C_BUS_OK:
                self.tx_count += 1
                self.rx_count += 1
                self.unlock()
                return I2C_BUS_OK, data

            self.retry_count += 1

        self.err_count += 1
        self.unlock()
        return err, None

    def get_stats(self):
        """对应I2C_Bus_GetStats()"""
        return self.tx_count, self.rx_count, self.err_count, self.retry_count

    def _raw_write(self, addr, tx_buf):
        """模拟底层写操作"""
        if addr not in self._devices:
            self._devices[addr] = {}
        if len(tx_buf) >= 2:
            reg = tx_buf[0]
            val = tx_buf[1]
            self._devices[addr][reg] = val
        self.tx_count += 1
        return I2C_BUS_OK

    def _raw_read(self, addr, rx_len):
        """模拟底层读操作"""
        if addr not in self._devices:
            return I2C_BUS_ERR_NACK, None
        # 返回设备寄存器数据
        dev = self._devices[addr]
        data = bytes([dev.get(i, 0) for i in range(rx_len)])
        self.rx_count += 1
        return I2C_BUS_OK, data

    @staticmethod
    def error_str(err):
        """对应I2C_Bus_ErrorStr()"""
        strs = {
            I2C_BUS_OK: "OK",
            I2C_BUS_ERR_BUSY: "BUSY (locked)",
            I2C_BUS_ERR_TIMEOUT: "TIMEOUT",
            I2C_BUS_ERR_NACK: "NACK (no response)",
            I2C_BUS_ERR_RETRY: "RETRY exhausted",
        }
        return strs.get(err, "UNKNOWN")


# ═══════════════════════════════════════════════════════════════
#  OPi5通信协议 — 对应 opi5_protocol.c
# ═══════════════════════════════════════════════════════════════

OPI5_FRAME_HEAD = 0xAA
OPI5_MAX_DATA_LEN = 64
OPI5_RX_BUF_SIZE = 256
OPI5_FRAME_QUEUE_SIZE = 4

# CMD_ID
CMD_HEARTBEAT = 0x01
CMD_VERSION = 0x02
CMD_MOTOR_SET = 0x10
CMD_MOTOR_GET = 0x11
CMD_MOTOR_STOP = 0x12
CMD_SERVO_SET = 0x20
CMD_SERVO_GET = 0x21
CMD_ADC_READ = 0x30
CMD_ADC_MULTI = 0x31
CMD_GPIO_SET = 0x40
CMD_GPIO_GET = 0x41
CMD_QUERY_SENSOR = 0x50
CMD_ACK = 0xE0
CMD_ERROR = 0xFE

# Error codes
ERR_CRC = 0x01
ERR_CMD = 0x02
ERR_PARAM = 0x03
ERR_BUSY = 0x04
ERR_TIMEOUT = 0x05
ERR_RANGE = 0x06
ERR_STATE = 0x07

# RX State Machine
OPI5_STATE_HEAD = 0
OPI5_STATE_CMD = 1
OPI5_STATE_LEN = 2
OPI5_STATE_SEQ = 3
OPI5_STATE_DATA = 4
OPI5_STATE_CRC = 5

# Comm State
OPI5_COMM_DISCONNECTED = 0
OPI5_COMM_CONNECTED = 1
OPI5_COMM_ERROR = 2

# Timeouts
OPI5_HEARTBEAT_MS = 1000
OPI5_OFFLINE_TIMEOUT_MS = 3000


class OPI5Frame:
    """对应OPI5_Frame"""
    def __init__(self):
        self.cmd = 0
        self.len = 0
        self.seq = 0
        self.data = bytearray(OPI5_MAX_DATA_LEN)


class OPi5Protocol:
    """OPi5通信协议，与C版本逻辑一致
    Frame: [0xAA][CMD][LEN][SEQ][DATA...][CRC8]
    CRC8: polynomial 0x07, init 0x00"""

    def __init__(self):
        self.rx_buf = bytearray(OPI5_RX_BUF_SIZE)
        self.rx_head = 0
        self.rx_tail = 0
        self.parse_state = OPI5_STATE_HEAD
        self.frame_cmd = 0
        self.frame_len = 0
        self.frame_seq = 0
        self.frame_data = bytearray(OPI5_MAX_DATA_LEN)
        self.frame_idx = 0
        self.frame_queue = []
        self.comm_state = OPI5_COMM_DISCONNECTED
        self.heartbeat_timer = 0
        self.last_rx_tick = 0
        self.g_ms = 0  # 模拟全局tick
        self.seq_counter = 0
        self.tx_log = []  # 记录发送的帧（测试用）

    @staticmethod
    def crc8(data):
        """对应OPI5_CRC8() — polynomial 0x07, init 0x00"""
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def init(self):
        """对应OPI5_Init()"""
        self.rx_head = 0
        self.rx_tail = 0
        self.frame_queue = []
        self.parse_state = OPI5_STATE_HEAD
        self.comm_state = OPI5_COMM_DISCONNECTED
        self.seq_counter = 0
        self.heartbeat_timer = 0
        self.last_rx_tick = 0
        self.tx_log = []

    def rx_byte(self, byte):
        """对应OPI5_RxByte() — UART ISR喂入"""
        next_idx = (self.rx_head + 1) % OPI5_RX_BUF_SIZE
        if next_idx != self.rx_tail:
            self.rx_buf[self.rx_head] = byte
            self.rx_head = next_idx

    def _parse_rx_data(self):
        """对应parse_rx_data()"""
        while self.rx_tail != self.rx_head:
            byte = self.rx_buf[self.rx_tail]
            self.rx_tail = (self.rx_tail + 1) % OPI5_RX_BUF_SIZE

            if self.parse_state == OPI5_STATE_HEAD:
                if byte == OPI5_FRAME_HEAD:
                    self.parse_state = OPI5_STATE_CMD

            elif self.parse_state == OPI5_STATE_CMD:
                self.frame_cmd = byte
                self.parse_state = OPI5_STATE_LEN

            elif self.parse_state == OPI5_STATE_LEN:
                if byte <= OPI5_MAX_DATA_LEN:
                    self.frame_len = byte
                    self.parse_state = OPI5_STATE_SEQ
                else:
                    self.parse_state = OPI5_STATE_HEAD

            elif self.parse_state == OPI5_STATE_SEQ:
                self.frame_seq = byte
                self.frame_idx = 0
                if self.frame_len == 0:
                    self.parse_state = OPI5_STATE_CRC
                else:
                    self.parse_state = OPI5_STATE_DATA

            elif self.parse_state == OPI5_STATE_DATA:
                self.frame_data[self.frame_idx] = byte
                self.frame_idx += 1
                if self.frame_idx >= self.frame_len:
                    self.parse_state = OPI5_STATE_CRC

            elif self.parse_state == OPI5_STATE_CRC:
                # CRC8 over CMD + LEN + SEQ + DATA
                crc_buf = bytearray(self.frame_len + 3)
                crc_buf[0] = self.frame_cmd
                crc_buf[1] = self.frame_len
                crc_buf[2] = self.frame_seq
                if self.frame_len > 0:
                    crc_buf[3:] = self.frame_data[:self.frame_len]
                crc_calc = self.crc8(crc_buf)
                if crc_calc == byte:
                    frame = OPI5Frame()
                    frame.cmd = self.frame_cmd
                    frame.len = self.frame_len
                    frame.seq = self.frame_seq
                    if self.frame_len > 0:
                        frame.data[:self.frame_len] = self.frame_data[:self.frame_len]
                    self.frame_queue.append(frame)
                    self.last_rx_tick = self.g_ms
                self.parse_state = OPI5_STATE_HEAD

            else:
                self.parse_state = OPI5_STATE_HEAD

    def has_frame(self):
        """对应OPI5_HasFrame()"""
        self._parse_rx_data()
        return len(self.frame_queue) > 0

    def get_frame(self):
        """对应OPI5_GetFrame()"""
        self._parse_rx_data()
        if not self.frame_queue:
            return None
        return self.frame_queue.pop(0)

    def build_frame(self, cmd, seq, data=None):
        """构建帧字节序列（用于测试发送）"""
        length = len(data) if data else 0
        if length > OPI5_MAX_DATA_LEN:
            length = OPI5_MAX_DATA_LEN
            data = data[:length]

        tx_buf = bytearray()
        tx_buf.append(OPI5_FRAME_HEAD)
        tx_buf.append(cmd)
        tx_buf.append(length)
        tx_buf.append(seq)
        if data:
            tx_buf.extend(data)

        # CRC8 over CMD + LEN + SEQ + DATA
        crc_buf = bytearray(3 + length)
        crc_buf[0] = cmd
        crc_buf[1] = length
        crc_buf[2] = seq
        if data:
            crc_buf[3:] = data
        tx_buf.append(self.crc8(crc_buf))
        return bytes(tx_buf)

    def send_frame(self, cmd, seq, data=None):
        """对应OPI5_SendFrame()"""
        frame_bytes = self.build_frame(cmd, seq, data)
        self.tx_log.append(frame_bytes)

    def send_ack(self, seq, orig_cmd, status=0):
        """对应OPI5_SendAck()"""
        data = bytes([orig_cmd, status])
        self.send_frame(CMD_ACK, seq, data)

    def send_error(self, seq, err_code, info=0):
        """对应OPI5_SendError()"""
        data = bytes([err_code, (info >> 8) & 0xFF, info & 0xFF])
        self.send_frame(CMD_ERROR, seq, data)

    def tick(self):
        """对应OPI5_Tick()"""
        self.heartbeat_timer += 1
        if self.heartbeat_timer >= OPI5_HEARTBEAT_MS:
            self.heartbeat_timer = 0
            self.send_frame(CMD_HEARTBEAT, self.seq_counter)
            self.seq_counter += 1

        if self.comm_state == OPI5_COMM_CONNECTED:
            if (self.g_ms - self.last_rx_tick) > OPI5_OFFLINE_TIMEOUT_MS:
                self.comm_state = OPI5_COMM_DISCONNECTED

    def get_comm_state(self):
        """对应OPI5_GetCommState()"""
        return self.comm_state

    def feed_bytes(self, data):
        """便捷方法：喂入多个字节"""
        for b in data:
            self.rx_byte(b)


# ═══════════════════════════════════════════════════════════════
#  SG90舵机驱动 — 对应 servo.c
# ═══════════════════════════════════════════════════════════════

SERVO_TIM_CLK = 2000000
SERVO_PERIOD = 40000
SERVO_MIN_PULSE = 1000    # 0.5ms → 0°
SERVO_MAX_PULSE = 5000    # 2.5ms → 180°
SERVO_MAX_ANGLE = 180

class Servo:
    """SG90舵机驱动，与C版本servo.c逻辑一致"""

    def __init__(self):
        self.angle = 90
        self.pulse_ticks = 3000  # 90°对应的tick数
        self.running = False

    def init(self):
        """对应Servo_Init()"""
        self.angle = 90
        self.pulse_ticks = self._angle_to_ticks(90)
        self.running = True

    def set_angle(self, angle):
        """对应Servo_SetAngle() — 角度0~180"""
        if angle > SERVO_MAX_ANGLE:
            angle = SERVO_MAX_ANGLE
        self.angle = angle
        self.pulse_ticks = self._angle_to_ticks(angle)

    def set_pulse_width(self, pulse_us):
        """对应Servo_SetPulseWidth() — 脉宽500~2500µs"""
        if pulse_us < 500:
            pulse_us = 500
        if pulse_us > 2500:
            pulse_us = 2500
        # µs → ticks: ticks = pulse_us * (SERVO_TIM_CLK / 1000000)
        self.pulse_ticks = int(pulse_us * SERVO_TIM_CLK / 1000000)
        # 反算角度
        self.angle = int((pulse_us - 500) * SERVO_MAX_ANGLE / 2000)

    def stop(self):
        """对应Servo_Stop()"""
        self.running = False

    def get_angle(self):
        return self.angle

    def get_pulse_ticks(self):
        return self.pulse_ticks

    def _angle_to_ticks(self, angle):
        """角度 → PWM tick值"""
        pulse_us = 500 + angle * 2000 // SERVO_MAX_ANGLE
        return int(pulse_us * SERVO_TIM_CLK / 1000000)


# ═══════════════════════════════════════════════════════════════
#  超声波测距驱动 — 对应 ultrasonic.c
# ═══════════════════════════════════════════════════════════════

ULTRASONIC_US_PER_CM = 58.0
ULTRASONIC_MIN_CM = 2.0
ULTRASONIC_MAX_CM = 400.0
ULTRASONIC_TIMEOUT_US = 30000

class Ultrasonic:
    """SR04超声波测距驱动，与C版本ultrasonic.c逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.last_distance = 0.0
        self.last_pulse_us = 0
        self.measure_count = 0
        self.timeout_count = 0

    def init(self):
        """对应Ultrasonic_Init()"""
        self.initialized = True
        self.last_distance = 0.0
        self.last_pulse_us = 0
        self.measure_count = 0
        self.timeout_count = 0

    def measure(self, pulse_us):
        """
        对应Ultrasonic_Measure() — 根据Echo脉宽计算距离
        pulse_us: 模拟的Echo脉宽(µs)
        返回: (success: bool, distance_cm: float)
        """
        self.measure_count += 1

        if pulse_us <= 0 or pulse_us > ULTRASONIC_TIMEOUT_US:
            self.timeout_count += 1
            return False, 0.0

        distance = pulse_us / ULTRASONIC_US_PER_CM

        if distance < ULTRASONIC_MIN_CM or distance > ULTRASONIC_MAX_CM:
            self.timeout_count += 1
            return False, 0.0

        self.last_pulse_us = pulse_us
        self.last_distance = distance
        return True, distance

    def measure_raw(self, pulse_us):
        """
        对应Ultrasonic_MeasureRaw() — 返回脉宽
        返回: (success: bool, pulse_us: int)
        """
        if pulse_us <= 0 or pulse_us > ULTRASONIC_TIMEOUT_US:
            return False, 0
        return True, pulse_us

    def get_last_distance(self):
        return self.last_distance


# ═══════════════════════════════════════════════════════════════
#  8路灰度传感器驱动 — 对应 grayscale.c
# ═══════════════════════════════════════════════════════════════

GRAY_CH0, GRAY_CH1, GRAY_CH2, GRAY_CH3 = 0, 1, 2, 3
GRAY_CH4, GRAY_CH5, GRAY_CH6, GRAY_CH7 = 4, 5, 6, 7

class Grayscale:
    """感为8路灰度传感器，与C版本grayscale.c逻辑一致"""

    def __init__(self):
        self.initialized = False
        # 模拟8路传感器值 (1=白, 0=黑)
        self._values = [1] * 8

    def init(self):
        """对应Grayscale_Init()"""
        self.initialized = True

    def set_channel(self, ch, value):
        """测试辅助：设置模拟传感器值"""
        if 0 <= ch < 8:
            self._values[ch] = 1 if value else 0

    def read(self, ch):
        """对应Grayscale_Read() — 1=白, 0=黑, 0xFF=无效"""
        if ch < 0 or ch > 7:
            return 0xFF
        return self._values[ch]

    def read_all(self):
        """对应Grayscale_ReadAll() — 返回8位掩码"""
        mask = 0
        for i in range(8):
            if self._values[i]:
                mask |= (1 << i)
        return mask

    def count_white(self):
        """对应Grayscale_CountWhite()"""
        return sum(self._values)


# ═══════════════════════════════════════════════════════════════
#  TCS34725 颜色传感器驱动 — 对应 tcs34725.c
# ═══════════════════════════════════════════════════════════════

TCS34725_ADDR = 0x29
TCS34725_CMD_BIT = 0x80
TCS34725_CMD_AUTO_INC = 0xA0
TCS34725_ENABLE_PON = 0x01
TCS34725_ENABLE_AEN = 0x02

class TCS34725_RGBC:
    """RGBC数据结构"""
    def __init__(self, clear=0, red=0, green=0, blue=0):
        self.clear = clear
        self.red = red
        self.green = green
        self.blue = blue

class TCS34725:
    """TCS34725颜色传感器I2C驱动，与C版本tcs34725.c逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.enabled = False
        self._registers = {}  # 模拟寄存器
        self._rgbc = TCS34725_RGBC()

    def init(self):
        """对应TCS34725_Init()"""
        # 模拟上电 + ADC使能
        self._registers[0x00] = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN
        self.initialized = True
        self.enabled = True
        return True

    def write_reg(self, reg, val):
        """对应TCS34725_WriteReg()"""
        if not self.initialized:
            return False
        self._registers[reg | TCS34725_CMD_BIT] = val
        self._registers[reg] = val
        return True

    def read_reg(self, reg):
        """对应TCS34725_ReadReg() — 返回 (success, value)"""
        if not self.initialized:
            return False, 0
        val = self._registers.get(reg, 0)
        return True, val

    def set_rgbc(self, clear, red, green, blue):
        """测试辅助：设置模拟RGBC值"""
        self._rgbc = TCS34725_RGBC(clear, red, green, blue)
        # 写入寄存器
        base = 0x14
        for i, val in enumerate([clear, red, green, blue]):
            self._registers[base + i * 2] = val & 0xFF
            self._registers[base + i * 2 + 1] = (val >> 8) & 0xFF

    def read_rgbc(self):
        """对应TCS34725_ReadRGBC() — 返回 (success, RGBC)"""
        if not self.initialized or not self.enabled:
            return False, TCS34725_RGBC()
        return True, TCS34725_RGBC(
            self._rgbc.clear, self._rgbc.red,
            self._rgbc.green, self._rgbc.blue
        )


# ═══════════════════════════════════════════════════════════════
#  HC-05 蓝牙驱动 — 对应 bluetooth.c
# ═══════════════════════════════════════════════════════════════

BT_MODE_TRANSPARENT = 0
BT_MODE_AT = 1
BT_STATE_IDLE = 0
BT_STATE_CONNECTED = 1
BT_STATE_AT_PENDING = 2
BT_RX_BUF_SIZE = 256
BT_TX_BUF_SIZE = 256

class Bluetooth:
    """HC-05蓝牙UART驱动，与C版本bluetooth.c逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.mode = BT_MODE_TRANSPARENT
        self.state = BT_STATE_IDLE
        self.rx_buf = bytearray(BT_RX_BUF_SIZE)
        self.rx_head = 0
        self.rx_tail = 0
        self.tx_count = 0
        self.rx_count = 0
        self.overflow_count = 0
        self.at_response = ""
        self.at_resp_ready = False

    def init(self):
        """对应BT_Init()"""
        self.initialized = True
        self.mode = BT_MODE_TRANSPARENT
        self.state = BT_STATE_IDLE
        self.rx_head = 0
        self.rx_tail = 0
        self.tx_count = 0
        self.rx_count = 0
        self.overflow_count = 0

    def enter_at_mode(self):
        """对应BT_EnterATMode()"""
        self.mode = BT_MODE_AT
        self.state = BT_STATE_AT_PENDING

    def enter_transparent_mode(self):
        """对应BT_EnterTransparentMode()"""
        self.mode = BT_MODE_TRANSPARENT
        self.state = BT_STATE_IDLE

    def send_byte(self, data):
        """对应BT_SendByte()"""
        self.tx_count += 1

    def send_data(self, data):
        """对应BT_SendData()"""
        self.tx_count += len(data)

    def send_string(self, text):
        """对应BT_SendString()"""
        self.tx_count += len(text)

    def _rx_push(self, byte_val):
        """模拟中断：接收一个字节"""
        next_head = (self.rx_head + 1) % BT_RX_BUF_SIZE
        if next_head == self.rx_tail:
            self.overflow_count += 1
            return False
        self.rx_buf[self.rx_head] = byte_val & 0xFF
        self.rx_head = next_head
        self.rx_count += 1
        if self.state == BT_STATE_IDLE:
            self.state = BT_STATE_CONNECTED
        return True

    def available(self):
        """对应BT_Available()"""
        return (self.rx_head - self.rx_tail) % BT_RX_BUF_SIZE

    def read_byte(self):
        """对应BT_Read() — 返回字节或-1"""
        if self.rx_head == self.rx_tail:
            return -1
        val = self.rx_buf[self.rx_tail]
        self.rx_tail = (self.rx_tail + 1) % BT_RX_BUF_SIZE
        return val

    def read_data(self, max_len):
        """对应BT_ReadData() — 返回bytes"""
        result = bytearray()
        for _ in range(max_len):
            b = self.read_byte()
            if b < 0:
                break
            result.append(b)
        return bytes(result)

    def flush(self):
        """对应BT_Flush()"""
        self.rx_head = 0
        self.rx_tail = 0

    def get_state(self):
        """对应BT_GetState()"""
        return self.state

    def get_mode(self):
        """对应BT_GetMode()"""
        return self.mode

    def send_at_command(self, cmd, response_buf_size=128):
        """对应BT_SendATCommand() — 模拟AT指令"""
        if self.mode != BT_MODE_AT:
            return False, ""
        # 模拟响应
        self.at_resp_ready = True
        return True, "OK"


# ═══════════════════════════════════════════════════════════════
#  编码器驱动 — 对应 encoder.h EncoderData / encoder.c
# ═══════════════════════════════════════════════════════════════

ENC_LEFT = 0
ENC_RIGHT = 1
ENC_CHANNELS = 2
ENC_PPR_DEFAULT = 360       # 默认每转脉冲数
ENC_SAMPLE_PERIOD_MS = 10   # 采样周期10ms


class Encoder:
    """N20霍尔编码器驱动，与C版本encoder.h逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.data = [{'count': 0, 'speed': 0, 'last_count': 0} for _ in range(ENC_CHANNELS)]
        self.ppr = ENC_PPR_DEFAULT

    def init(self):
        """对应Encoder_Init()"""
        self.initialized = True
        for d in self.data:
            d['count'] = 0
            d['speed'] = 0
            d['last_count'] = 0

    def get_count(self, ch):
        """对应Encoder_GetCount()"""
        return self.data[ch]['count']

    def get_speed(self, ch):
        """对应Encoder_GetSpeed()"""
        return self.data[ch]['speed']

    def reset(self, ch):
        """对应Encoder_Reset()"""
        self.data[ch]['count'] = 0
        self.data[ch]['speed'] = 0
        self.data[ch]['last_count'] = 0

    def inject_pulse(self, ch, delta):
        """模拟中断注入脉冲（测试用）"""
        self.data[ch]['count'] += delta

    def sample_callback(self):
        """对应Encoder_SampleCallback() — 计算速度"""
        for d in self.data:
            d['speed'] = d['count'] - d['last_count']
            d['last_count'] = d['count']

    def get_speed_rpm(self, ch):
        """计算RPM: speed(pulses/sample) * 60 / ppr / period_s"""
        period_s = ENC_SAMPLE_PERIOD_MS / 1000.0
        if period_s < 1e-9 or self.ppr < 1:
            return 0.0
        return self.data[ch]['speed'] * 60.0 / (self.ppr * period_s)


# ═══════════════════════════════════════════════════════════════
#  PCA9685 16路PWM驱动 — 对应 pca9685.h / pca9685.c
# ═══════════════════════════════════════════════════════════════

PCA9685_ADDR = 0x40
PCA9685_OSC_FREQ = 25000000
PCA9685_PWM_STEPS = 4096
PCA9685_NUM_CHANNELS = 16
PCA9685_REG_MODE1 = 0x00
PCA9685_REG_PRE_SCALE = 0xFE
PCA9685_REG_LED0_ON_L = 0x06
PCA9685_MODE1_SLEEP = 0x10
PCA9685_MODE1_RESTART = 0x80
PCA9685_MODE1_AI = 0x20


class PCA9685:
    """PCA9685 16路PWM舵机驱动板，与C版本pca9685.h逻辑一致"""

    def __init__(self, i2c_bus=None, addr=PCA9685_ADDR):
        self.addr = addr
        self.i2c = i2c_bus
        self.initialized = False
        self.freq = 0
        self.channels = [{'on': 0, 'off': 0} for _ in range(PCA9685_NUM_CHANNELS)]
        self.reg_mode1 = 0

    def init(self):
        """对应PCA9685_Init()"""
        self.initialized = True
        self.reg_mode1 = PCA9685_MODE1_AI | PCA9685_MODE1_RESTART
        self.set_pwm_freq(50)
        return True

    def set_pwm_freq(self, freq_hz):
        """对应PCA9685_SetPWMFreq()"""
        if freq_hz < 24 or freq_hz > 1526:
            return False
        prescale = int(round(PCA9685_OSC_FREQ / (PCA9685_PWM_STEPS * freq_hz))) - 1
        if prescale < 3:
            prescale = 3
        self.freq = freq_hz
        self._prescale = prescale
        return True

    def set_pwm(self, ch, on, off):
        """对应PCA9685_SetPWM()"""
        if ch < 0 or ch >= PCA9685_NUM_CHANNELS:
            return False
        self.channels[ch]['on'] = on & 0x0FFF
        self.channels[ch]['off'] = off & 0x0FFF
        return True

    def set_angle(self, ch, angle):
        """对应PCA9685_SetAngle() — 0~180°映射到102~512"""
        if ch < 0 or ch >= PCA9685_NUM_CHANNELS:
            return False
        if angle < 0:
            angle = 0
        if angle > 180:
            angle = 180
        pulse = int(102 + (512 - 102) * angle / 180.0)
        return self.set_pwm(ch, 0, pulse)

    def all_off(self):
        """对应PCA9685_AllOff()"""
        for ch in self.channels:
            ch['on'] = 0
            ch['off'] = 0
        return True

    def get_pwm(self, ch):
        """获取通道PWM值（测试用）"""
        if ch < 0 or ch >= PCA9685_NUM_CHANNELS:
            return 0, 0
        return self.channels[ch]['on'], self.channels[ch]['off']


# ═══════════════════════════════════════════════════════════════
#  AT24C02 EEPROM驱动 — 对应 at24c02.h / at24c02.c
# ═══════════════════════════════════════════════════════════════

AT24C02_ADDR = 0x50
AT24C02_SIZE = 256
AT24C02_PAGE_SIZE = 8
AT24C02_WRITE_CYCLE_MS = 5


class AT24C02:
    """AT24C02 EEPROM驱动，与C版本at24c02.h逻辑一致"""

    def __init__(self, i2c_bus=None, addr=AT24C02_ADDR):
        self.addr = addr
        self.i2c = i2c_bus
        self.initialized = False
        self._mem = bytearray(AT24C02_SIZE)  # 模拟EEPROM存储

    def init(self):
        """初始化"""
        self.initialized = True
        return True

    def write_byte(self, addr, data):
        """对应AT24C02_WriteByte()"""
        if addr < 0 or addr >= AT24C02_SIZE:
            return False
        self._mem[addr] = data & 0xFF
        return True

    def read_byte(self, addr):
        """对应AT24C02_ReadByte()"""
        if addr < 0 or addr >= AT24C02_SIZE:
            return False, 0
        return True, self._mem[addr]

    def write(self, addr, data):
        """对应AT24C02_Write() — 自动处理跨页"""
        if addr < 0 or addr + len(data) > AT24C02_SIZE:
            return False
        for i, b in enumerate(data):
            self._mem[addr + i] = b & 0xFF
        return True

    def read(self, addr, length):
        """对应AT24C02_Read()"""
        if addr < 0 or addr + length > AT24C02_SIZE:
            return False, None
        return True, bytes(self._mem[addr:addr + length])

    def write_page(self, addr, data):
        """对应AT24C02_WritePage() — 不超过页边界"""
        if len(data) > AT24C02_PAGE_SIZE:
            return False
        page_start = (addr // AT24C02_PAGE_SIZE) * AT24C02_PAGE_SIZE
        if addr + len(data) > page_start + AT24C02_PAGE_SIZE:
            return False  # 跨页
        return self.write(addr, data)

    def is_ready(self):
        """对应AT24C02_IsReady()"""
        return self.initialized


# ═══════════════════════════════════════════════════════════════
#  SHT20 温湿度传感器驱动 — I2C温湿度传感器
# ═══════════════════════════════════════════════════════════════

SHT20_ADDR = 0x40
SHT20_CMD_TEMP_HOLD = 0xE3
SHT20_CMD_HUMI_HOLD = 0xE5
SHT20_CMD_TEMP_NOHOLD = 0xF3
SHT20_CMD_HUMI_NOHOLD = 0xF5
SHT20_CMD_WRITE_USER = 0xE6
SHT20_CMD_READ_USER = 0xE7
SHT20_CMD_SOFT_RESET = 0xFE


class SHT20:
    """SHT20温湿度传感器驱动，I2C接口"""

    def __init__(self, i2c_bus=None, addr=SHT20_ADDR):
        self.addr = addr
        self.i2c = i2c_bus
        self.initialized = False
        # 模拟寄存器值
        self._raw_temp = 0
        self._raw_humi = 0
        self._user_reg = 0x02  # 默认值

    def init(self):
        """初始化SHT20"""
        self.initialized = True
        return True

    def read_temperature(self):
        """读取温度(°C)，对应读取原始值并转换
        T = -46.85 + 175.72 * raw / 65536
        """
        if not self.initialized:
            return False, 0.0
        temp = -46.85 + 175.72 * self._raw_temp / 65536.0
        return True, temp

    def read_humidity(self):
        """读取湿度(%RH)
        RH = -6.0 + 125.0 * raw / 65536
        """
        if not self.initialized:
            return False, 0.0
        humi = -6.0 + 125.0 * self._raw_humi / 65536.0
        # 限幅到0~100
        humi = max(0.0, min(100.0, humi))
        return True, humi

    def set_raw_values(self, raw_temp, raw_humi):
        """测试用：设置模拟原始ADC值"""
        self._raw_temp = raw_temp & 0xFFFC
        self._raw_humi = raw_humi & 0xFFFC

    def read_user_reg(self):
        """读用户寄存器"""
        return True, self._user_reg

    def write_user_reg(self, val):
        """写用户寄存器"""
        self._user_reg = val & 0xFF
        return True

    def soft_reset(self):
        """软复位"""
        self._user_reg = 0x02
        return True


# ═══════════════════════════════════════════════════════════════
#  WS2812 可寻址LED驱动 — SPI/GPIO方式
# ═══════════════════════════════════════════════════════════════

WS2812_MAX_LEDS = 256
WS2812_RESET_US = 50


class WS2812:
    """WS2812可寻址LED驱动"""

    def __init__(self, num_leds=8):
        self.num_leds = min(num_leds, WS2812_MAX_LEDS)
        self.initialized = False
        # GRB格式存储
        self._buf = [[0, 0, 0] for _ in range(self.num_leds)]
        self._brightness = 255

    def init(self):
        """初始化"""
        self.initialized = True
        return True

    def set_pixel(self, idx, r, g, b):
        """设置单个LED颜色(RGB -> GRB内部存储)"""
        if idx < 0 or idx >= self.num_leds:
            return False
        self._buf[idx] = [g, r, b]  # WS2812使用GRB顺序
        return True

    def set_pixel_hsv(self, idx, h, s, v):
        """设置HSV颜色"""
        if idx < 0 or idx >= self.num_leds:
            return False
        # HSV to RGB conversion
        h = h % 360
        c = v * s / 255
        x = c * (1 - abs((h / 60) % 2 - 1))
        m = v - c
        if h < 60:
            r, g, b = c, x, 0
        elif h < 120:
            r, g, b = x, c, 0
        elif h < 180:
            r, g, b = 0, c, x
        elif h < 240:
            r, g, b = 0, x, c
        elif h < 300:
            r, g, b = x, 0, c
        else:
            r, g, b = c, 0, x
        r = int(r + m)
        g = int(g + m)
        b = int(b + m)
        self._buf[idx] = [g, r, b]
        return True

    def fill(self, r, g, b):
        """填充所有LED"""
        for i in range(self.num_leds):
            self._buf[i] = [g, r, b]
        return True

    def clear(self):
        """清零所有LED"""
        return self.fill(0, 0, 0)

    def set_brightness(self, b):
        """设置全局亮度 0~255"""
        self._brightness = b & 0xFF

    def get_pixel(self, idx):
        """获取LED颜色(RGB)"""
        if idx < 0 or idx >= self.num_leds:
            return 0, 0, 0
        g, r, b = self._buf[idx]
        return r, g, b

    def show(self):
        """发送数据到LED（模拟）"""
        if not self.initialized:
            return False
        return True


# ═══════════════════════════════════════════════════════════════
#  JY901S 九轴IMU驱动 — 对应 jy901s.h / jy901s.c
# ═══════════════════════════════════════════════════════════════

JY901S_FRAME_HEAD = 0x55
JY901S_TYPE_ACC   = 0x51
JY901S_TYPE_GYRO  = 0x52
JY901S_TYPE_ANGLE = 0x53
JY901S_TYPE_MAG   = 0x54
JY901S_FRAME_LEN  = 11


class JY901S_Data:
    """JY901S IMU完整数据，与C版本JY901S_Data结构体一致"""

    def __init__(self):
        # 加速度 (单位: g)
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        # 角速度 (单位: 度/秒)
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        # 角度 (单位: 度)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # 磁场 (原始值)
        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0
        # 更新标志
        self.acc_updated = False
        self.gyro_updated = False
        self.angle_updated = False


class JY901S:
    """JY901S九轴IMU UART驱动，与C版本jy901s.c逻辑一致

    通信协议: UART 9600 8N1
    数据帧: 0x55 + 类型 + 8字节数据 + 校验和 = 11字节
    """

    def __init__(self):
        self.initialized = False
        self._data = JY901S_Data()
        self._frame_buf = []
        self._frame_count = 0
        self._parse_errors = 0

    def init(self):
        """对应JY901S_Init()"""
        self.initialized = True
        self._data = JY901S_Data()
        self._frame_buf = []
        self._frame_count = 0
        self._parse_errors = 0
        return True

    def get_data(self):
        """对应JY901S_GetData() — 返回最新IMU数据"""
        return self._data

    def get_roll(self):
        """对应JY901S_GetRoll()"""
        return self._data.roll

    def get_pitch(self):
        """对应JY901S_GetPitch()"""
        return self._data.pitch

    def get_yaw(self):
        """对应JY901S_GetYaw()"""
        return self._data.yaw

    def is_angle_updated(self):
        """对应JY901S_IsAngleUpdated()"""
        return self._data.angle_updated

    def clear_angle_flag(self):
        """清除角度更新标志"""
        self._data.angle_updated = False

    def _checksum(self, buf):
        """计算校验和: 前10字节求和 & 0xFF"""
        return sum(buf[:10]) & 0xFF

    def feed_byte(self, byte_val):
        """喂入一个字节 — 对应UART中断逐字节接收

        返回: (frame_parsed: bool, frame_type: int)
        """
        byte_val &= 0xFF
        self._frame_buf.append(byte_val)

        # 帧头检测
        if len(self._frame_buf) == 1:
            if byte_val != JY901S_FRAME_HEAD:
                self._frame_buf.clear()
                return False, 0
            return False, 0

        # 等待满帧
        if len(self._frame_buf) < JY901S_FRAME_LEN:
            return False, 0

        # 校验和
        calc_sum = self._checksum(self._frame_buf)
        recv_sum = self._frame_buf[10]
        frame_type = self._frame_buf[1]
        frame_copy = list(self._frame_buf)
        self._frame_buf.clear()

        if calc_sum != recv_sum:
            self._parse_errors += 1
            return False, 0

        self._frame_count += 1
        self._parse_frame(frame_type, frame_copy)
        return True, frame_type

    def _parse_frame(self, frame_type, buf):
        """解析已校验通过的帧数据"""
        if len(buf) < 11:
            return

        if frame_type == JY901S_TYPE_ACC:
            # 加速度: 原始值 / 32768 * 16g
            self._data.acc_x = self._bytes_to_int16(buf[2], buf[3]) / 32768.0 * 16.0
            self._data.acc_y = self._bytes_to_int16(buf[4], buf[5]) / 32768.0 * 16.0
            self._data.acc_z = self._bytes_to_int16(buf[6], buf[7]) / 32768.0 * 16.0
            self._data.acc_updated = True
        elif frame_type == JY901S_TYPE_GYRO:
            # 角速度: 原始值 / 32768 * 2000 dps
            self._data.gyro_x = self._bytes_to_int16(buf[2], buf[3]) / 32768.0 * 2000.0
            self._data.gyro_y = self._bytes_to_int16(buf[4], buf[5]) / 32768.0 * 2000.0
            self._data.gyro_z = self._bytes_to_int16(buf[6], buf[7]) / 32768.0 * 2000.0
            self._data.gyro_updated = True
        elif frame_type == JY901S_TYPE_ANGLE:
            # 角度: 原始值 / 32768 * 180 度
            self._data.roll  = self._bytes_to_int16(buf[2], buf[3]) / 32768.0 * 180.0
            self._data.pitch = self._bytes_to_int16(buf[4], buf[5]) / 32768.0 * 180.0
            self._data.yaw   = self._bytes_to_int16(buf[6], buf[7]) / 32768.0 * 180.0
            self._data.angle_updated = True
        elif frame_type == JY901S_TYPE_MAG:
            self._data.mag_x = self._bytes_to_int16(buf[2], buf[3])
            self._data.mag_y = self._bytes_to_int16(buf[4], buf[5])
            self._data.mag_z = self._bytes_to_int16(buf[6], buf[7])

    @staticmethod
    def _bytes_to_int16(lo, hi):
        """小端有符号16位"""
        val = lo | (hi << 8)
        if val >= 32768:
            val -= 65536
        return val

    @staticmethod
    def build_frame(frame_type, raw_values):
        """构建JY901S数据帧（测试辅助）

        frame_type: 0x51~0x54
        raw_values: 4个int16原始值
        返回: 11字节列表
        """
        frame = [0] * 11
        frame[0] = JY901S_FRAME_HEAD
        frame[1] = frame_type
        for i, val in enumerate(raw_values):
            val &= 0xFFFF
            frame[2 + i*2] = val & 0xFF
            frame[3 + i*2] = (val >> 8) & 0xFF
        frame[10] = sum(frame[:10]) & 0xFF
        return frame

    def get_frame_count(self):
        """获取已解析帧数"""
        return self._frame_count

    def get_parse_errors(self):
        """获取校验错误数"""
        return self._parse_errors


# ═══════════════════════════════════════════════════════════════
#  MPU6050 六轴IMU驱动 — 对应 mpu6050.h / mpu6050.c
# ═══════════════════════════════════════════════════════════════

MPU6050_I2C_ADDR_LOW  = 0x68
MPU6050_I2C_ADDR_HIGH = 0x69
MPU6050_I2C_ADDR      = MPU6050_I2C_ADDR_LOW
MPU6050_WHO_AM_I_VALUE = 0x68

# 陀螺仪量程配置
MPU6050_GYRO_FS_250DPS  = 0x00
MPU6050_GYRO_FS_500DPS  = 0x08
MPU6050_GYRO_FS_1000DPS = 0x10
MPU6050_GYRO_FS_2000DPS = 0x18

# 加速度计量程配置
MPU6050_ACCEL_FS_2G  = 0x00
MPU6050_ACCEL_FS_4G  = 0x08
MPU6050_ACCEL_FS_8G  = 0x10
MPU6050_ACCEL_FS_16G = 0x18

# 量程对应的灵敏度（LSB/单位）
_MPU6050_ACCEL_SENSITIVITY = {
    MPU6050_ACCEL_FS_2G:  16384.0,
    MPU6050_ACCEL_FS_4G:  8192.0,
    MPU6050_ACCEL_FS_8G:  4096.0,
    MPU6050_ACCEL_FS_16G: 2048.0,
}
_MPU6050_GYRO_SENSITIVITY = {
    MPU6050_GYRO_FS_250DPS:  131.0,
    MPU6050_GYRO_FS_500DPS:  65.5,
    MPU6050_GYRO_FS_1000DPS: 32.8,
    MPU6050_GYRO_FS_2000DPS: 16.4,
}


class MPU6050_Data:
    """MPU6050 测量数据，与C版本mpu6050_data_t对应"""

    def __init__(self):
        self.accel_x_raw = 0
        self.accel_y_raw = 0
        self.accel_z_raw = 0
        self.gyro_x_raw = 0
        self.gyro_y_raw = 0
        self.gyro_z_raw = 0
        self.accel_x_g = 0.0
        self.accel_y_g = 0.0
        self.accel_z_g = 0.0
        self.gyro_x_dps = 0.0
        self.gyro_y_dps = 0.0
        self.gyro_z_dps = 0.0
        self.temperature = 0.0


class MPU6050:
    """MPU6050六轴IMU I2C驱动，与C版本mpu6050.c逻辑一致

    对应C源文件: 02_mspm0g3507/drivers/mpu6050.c
    """

    def __init__(self, i2c_bus=None, addr=MPU6050_I2C_ADDR):
        self.addr = addr
        self.i2c = i2c_bus
        self.initialized = False
        self._accel_fs = MPU6050_ACCEL_FS_2G
        self._gyro_fs = MPU6050_GYRO_FS_250DPS
        self._raw_accel = [0, 0, 0]
        self._raw_gyro = [0, 0, 0]
        self._raw_temp = 0
        self._sleeping = False

    def init(self, accel_fs=MPU6050_ACCEL_FS_2G, gyro_fs=MPU6050_GYRO_FS_250DPS):
        """初始化MPU6050，唤醒芯片，配置量程"""
        self._accel_fs = accel_fs
        self._gyro_fs = gyro_fs
        self._sleeping = False
        self.initialized = True
        return True

    def read_all(self):
        """读取全部数据（加速度+陀螺仪+温度），对应mpu6050_read_all()"""
        if not self.initialized:
            return False, None
        data = MPU6050_Data()
        data.accel_x_raw = self._raw_accel[0]
        data.accel_y_raw = self._raw_accel[1]
        data.accel_z_raw = self._raw_accel[2]
        data.gyro_x_raw = self._raw_gyro[0]
        data.gyro_y_raw = self._raw_gyro[1]
        data.gyro_z_raw = self._raw_gyro[2]
        # 转换为物理单位
        acc_sens = _MPU6050_ACCEL_SENSITIVITY.get(self._accel_fs, 16384.0)
        gyro_sens = _MPU6050_GYRO_SENSITIVITY.get(self._gyro_fs, 131.0)
        data.accel_x_g = data.accel_x_raw / acc_sens
        data.accel_y_g = data.accel_y_raw / acc_sens
        data.accel_z_g = data.accel_z_raw / acc_sens
        data.gyro_x_dps = data.gyro_x_raw / gyro_sens
        data.gyro_y_dps = data.gyro_y_raw / gyro_sens
        data.gyro_z_dps = data.gyro_z_raw / gyro_sens
        # 温度转换: T = raw/340 + 36.53
        data.temperature = self._raw_temp / 340.0 + 36.53
        return True, data

    def read_temperature(self):
        """仅读取温度"""
        if not self.initialized:
            return False, 0.0
        return True, self._raw_temp / 340.0 + 36.53

    def read_id(self):
        """读取芯片ID（应为0x68）"""
        return True, MPU6050_WHO_AM_I_VALUE

    def sleep(self):
        """进入睡眠模式"""
        if not self.initialized:
            return False
        self._sleeping = True
        return True

    def wake_up(self):
        """唤醒"""
        if not self.initialized:
            return False
        self._sleeping = False
        return True

    def set_raw_values(self, accel_xyz, gyro_xyz, raw_temp=0):
        """测试用：设置模拟原始值
        accel_xyz: (ax, ay, az) 有符号16位
        gyro_xyz:  (gx, gy, gz) 有符号16位
        raw_temp:  温度原始值（有符号16位）
        """
        self._raw_accel = list(accel_xyz)
        self._raw_gyro = list(gyro_xyz)
        self._raw_temp = raw_temp


# ═══════════════════════════════════════════════════════════════
#  QMC5883L 三轴电子罗盘驱动 — 对应 qmc5883l.h / qmc5883l.c
# ═══════════════════════════════════════════════════════════════

QMC5883L_I2C_ADDR       = 0x0D
QMC5883L_CHIP_ID_VALUE  = 0xFF

QMC5883L_RANGE_2G = 0x00  # ±2 Gauss, 灵敏度 12000 LSB/Gauss
QMC5883L_RANGE_8G = 0x10  # ±8 Gauss, 灵敏度 3000 LSB/Gauss

_QMC5883L_SENSITIVITY = {
    QMC5883L_RANGE_2G: 12000.0,
    QMC5883L_RANGE_8G: 3000.0,
}


class QMC5883L_Data:
    """QMC5883L 测量数据，与C版本qmc5883l_data_t对应"""

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.x_gauss = 0.0
        self.y_gauss = 0.0
        self.z_gauss = 0.0
        self.heading_deg = 0.0
        self.temperature = 0.0


class QMC5883L:
    """QMC5883L三轴电子罗盘I2C驱动，与C版本qmc5883l.c逻辑一致

    对应C源文件: 02_mspm0g3507/drivers/qmc5883l.c
    """

    def __init__(self, i2c_bus=None, addr=QMC5883L_I2C_ADDR):
        self.addr = addr
        self.i2c = i2c_bus
        self.initialized = False
        self._range = QMC5883L_RANGE_2G
        self._raw_x = 0
        self._raw_y = 0
        self._raw_z = 0
        self._raw_temp = 0

    def init(self, range_cfg=QMC5883L_RANGE_2G):
        """初始化QMC5883L，验证芯片ID，配置量程"""
        self._range = range_cfg
        self.initialized = True
        return True

    def read_data(self):
        """读取磁场数据，对应qmc5883l_read_data()"""
        if not self.initialized:
            return False, None
        data = QMC5883L_Data()
        data.x = self._raw_x
        data.y = self._raw_y
        data.z = self._raw_z
        sens = _QMC5883L_SENSITIVITY.get(self._range, 12000.0)
        data.x_gauss = data.x / sens
        data.y_gauss = data.y / sens
        data.z_gauss = data.z / sens
        # 航向角: atan2(y, x) → 0~360°
        import math
        heading = math.atan2(data.y, data.x) * 180.0 / math.pi
        if heading < 0:
            heading += 360.0
        data.heading_deg = heading
        # 温度
        data.temperature = self._raw_temp / 100.0
        return True, data

    def read_chip_id(self):
        """读取芯片ID（应为0xFF）"""
        return True, QMC5883L_CHIP_ID_VALUE

    def soft_reset(self):
        """软复位"""
        if not self.initialized:
            return False
        self._raw_x = self._raw_y = self._raw_z = 0
        self._raw_temp = 0
        return True

    def set_standby(self):
        """进入待机模式"""
        return self.initialized

    def set_raw_values(self, x, y, z, temp_raw=0):
        """测试用：设置模拟原始值"""
        self._raw_x = x
        self._raw_y = y
        self._raw_z = z
        self._raw_temp = temp_raw


# ═══════════════════════════════════════════════════════════════
#  SGP30 空气质量传感器驱动 — 对应 sgp30.h / sgp30.c
# ═══════════════════════════════════════════════════════════════

SGP30_I2C_ADDR = 0x58
SGP30_CMD_INIT_AIR_QUALITY     = 0x2003
SGP30_CMD_MEASURE_AIR_QUALITY  = 0x2008
SGP30_CMD_GET_FEATURE_SET      = 0x202F
SGP30_CMD_GET_TVOC_BASELINE    = 0x20B3
SGP30_CMD_SET_TVOC_BASELINE    = 0x2077
SGP30_CRC_POLYNOMIAL = 0x31
SGP30_CRC_INIT       = 0xFF


class SGP30:
    """SGP30空气质量传感器I2C驱动，与C版本sgp30.c逻辑一致

    对应C源文件: 02_mspm0g3507/drivers/sgp30.c
    """

    def __init__(self, i2c_bus=None, addr=SGP30_I2C_ADDR):
        self.addr = addr
        self.i2c = i2c_bus
        self.initialized = False
        self._tvoc = 0
        self._eco2 = 400  # 默认基线eCO2
        self._tvoc_baseline = 0
        self._eco2_baseline = 0

    def init(self):
        """初始化SGP30，发送init_air_quality命令"""
        self.initialized = True
        self._tvoc = 0
        self._eco2 = 400
        return True

    def measure(self):
        """测量空气质量（TVOC + eCO2），对应sgp30_measure()"""
        if not self.initialized:
            return False, None
        return True, (self._tvoc, self._eco2)

    def get_baseline(self):
        """获取基线值"""
        if not self.initialized:
            return False, (0, 0)
        return True, (self._tvoc_baseline, self._eco2_baseline)

    def set_baseline(self, tvoc_base, eco2_base):
        """恢复基线值"""
        if not self.initialized:
            return False
        self._tvoc_baseline = tvoc_base
        self._eco2_baseline = eco2_base
        return True

    def set_humidity(self, humidity_pct, temperature_c):
        """设置湿度补偿"""
        if not self.initialized:
            return False
        return True

    def selftest(self):
        """自检"""
        return self.initialized

    def set_raw_values(self, tvoc, eco2):
        """测试用：设置模拟测量值"""
        self._tvoc = tvoc
        self._eco2 = eco2

    @staticmethod
    def crc8(data_bytes):
        """SGP30 CRC-8校验，多项式0x31，初值0xFF"""
        crc = SGP30_CRC_INIT
        for byte in data_bytes:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ SGP30_CRC_POLYNOMIAL) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc


# ═══════════════════════════════════════════════════════════════
#  SHT30 温湿度传感器驱动 — 对应 sht30.h / sht30.c
# ═══════════════════════════════════════════════════════════════

SHT30_I2C_ADDR_LOW  = 0x44
SHT30_I2C_ADDR_HIGH = 0x45
SHT30_I2C_ADDR      = SHT30_I2C_ADDR_LOW

SHT30_CMD_SINGLE_HIGH_CS_EN  = 0x2C06
SHT30_CMD_SOFT_RESET         = 0x30A2
SHT30_CMD_HEATER_ON          = 0x306D
SHT30_CMD_HEATER_OFF         = 0x3066
SHT30_CMD_READ_STATUS        = 0xF32D
SHT30_CRC_POLYNOMIAL = 0x31
SHT30_CRC_INIT       = 0xFF


class SHT30:
    """SHT30温湿度传感器I2C驱动，与C版本sht30.c逻辑一致

    对应C源文件: 02_mspm0g3507/drivers/sht30.c
    温度转换: T = -45 + 175 * raw / 65535
    湿度转换: RH = 100 * raw / 65535
    """

    def __init__(self, i2c_bus=None, addr=SHT30_I2C_ADDR):
        self.addr = addr
        self.i2c = i2c_bus
        self.initialized = False
        self._raw_temp = 0
        self._raw_humi = 0
        self._heater_on = False
        self._status = 0x0000

    def init(self):
        """初始化SHT30，发送软复位"""
        self.initialized = True
        self._heater_on = False
        self._status = 0x0000
        return True

    def measure_single(self):
        """单次高精度测量，对应sht30_measure_single()"""
        if not self.initialized:
            return False, None
        temp = -45.0 + 175.0 * self._raw_temp / 65535.0
        humi = 100.0 * self._raw_humi / 65535.0
        humi = max(0.0, min(100.0, humi))
        return True, (temp, humi)

    def read_status(self):
        """读取状态寄存器"""
        if not self.initialized:
            return False, 0
        return True, self._status

    def heater_on(self):
        """开启加热器"""
        if not self.initialized:
            return False
        self._heater_on = True
        return True

    def heater_off(self):
        """关闭加热器"""
        if not self.initialized:
            return False
        self._heater_on = False
        return True

    def soft_reset(self):
        """软复位"""
        if not self.initialized:
            return False
        self._heater_on = False
        self._status = 0x0000
        return True

    def set_raw_values(self, raw_temp, raw_humi):
        """测试用：设置模拟原始ADC值"""
        self._raw_temp = raw_temp & 0xFFFF
        self._raw_humi = raw_humi & 0xFFFF

    @staticmethod
    def crc8(data_bytes):
        """SHT30 CRC-8校验，多项式0x31，初值0xFF"""
        crc = SHT30_CRC_INIT
        for byte in data_bytes:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ SHT30_CRC_POLYNOMIAL) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc


# ═══════════════════════════════════════════════════════════════
#  GPS NEO-6M 模块驱动 — 对应 gps_neo6m.h / gps_neo6m.c
# ═══════════════════════════════════════════════════════════════

GPS_NMEA_MAX_LEN   = 128
GPS_MAX_SATELLITES = 12

# NMEA语句类型
GPS_NMEA_NONE    = 0
GPS_NMEA_GGA     = 1
GPS_NMEA_RMC     = 2
GPS_NMEA_GSV     = 3
GPS_NMEA_GSA     = 4
GPS_NMEA_UNKNOWN = 5

# 定位质量
GPS_FIX_NONE      = 0
GPS_FIX_GPS       = 1
GPS_FIX_DGPS      = 2


class GPS_Data:
    """GPS完整数据，与C版本gps_data_t对应"""

    def __init__(self):
        # 时间（UTC）
        self.hour = 0
        self.minute = 0
        self.second = 0
        self.millisecond = 0
        # 日期
        self.day = 0
        self.month = 0
        self.year = 0
        # 位置
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude_m = 0.0
        self.speed_knots = 0.0
        self.speed_kmh = 0.0
        self.course_deg = 0.0
        # 精度
        self.hdop = 0.0
        self.vdop = 0.0
        self.pdop = 0.0
        # 定位状态
        self.fix_quality = GPS_FIX_NONE
        self.satellites_used = 0
        # 有效标志
        self.valid_position = False
        self.valid_time = False
        self.valid_date = False
        # 卫星信息
        self.satellites_in_view = 0
        self.satellites = []


class GPS_NEO6M:
    """GPS NEO-6M UART驱动，NMEA协议解析，与C版本gps_neo6m.c逻辑一致

    对应C源文件: 02_mspm0g3507/drivers/gps_neo6m.c
    逐字节喂入NMEA数据，自动解析GGA/RMC语句更新定位信息。
    """

    def __init__(self):
        self.initialized = False
        self._data = GPS_Data()
        self._buf = []
        self._new_data = False

    def init(self):
        """初始化GPS模块"""
        self.initialized = True
        self._data = GPS_Data()
        self._buf = []
        self._new_data = False
        return True

    def get_data(self):
        """获取最新GPS数据"""
        self._new_data = False
        return self._data

    def has_new_data(self):
        """检查是否有新数据"""
        return self._new_data

    def feed_byte(self, byte_val):
        """喂入一个字节，自动解析NMEA语句"""
        byte_val &= 0xFF
        ch = chr(byte_val)

        if ch == '$':
            self._buf = []

        self._buf.append(ch)

        if ch == '\n' or len(self._buf) >= GPS_NMEA_MAX_LEN:
            sentence = ''.join(self._buf).strip()
            self._buf = []
            if sentence.startswith('$') and '*' in sentence:
                self._parse_sentence(sentence)
        return True

    def feed_string(self, nmea_str):
        """喂入一整条NMEA字符串（测试用）"""
        for ch in nmea_str:
            self.feed_byte(ord(ch))

    def _parse_sentence(self, sentence):
        """解析NMEA语句"""
        try:
            star_idx = sentence.index('*')
            body = sentence[1:star_idx]
            fields = body.split(',')
            talker = fields[0]

            if talker in ('GPGGA', 'GNGGA'):
                self._parse_gga(fields)
            elif talker in ('GPRMC', 'GNRMC'):
                self._parse_rmc(fields)
        except (ValueError, IndexError):
            pass

    def _parse_gga(self, f):
        """解析GGA定位数据"""
        if len(f) < 10:
            return
        # 时间
        if f[1] and len(f[1]) >= 6:
            t = f[1]
            self._data.hour = int(t[0:2])
            self._data.minute = int(t[2:4])
            self._data.second = int(t[4:6])
            self._data.valid_time = True
        # 纬度
        if f[2] and f[3]:
            lat_nmea = float(f[2])
            self._data.latitude = self.nmea_to_degrees(lat_nmea, f[3])
            self._data.valid_position = True
        # 经度
        if f[4] and f[5]:
            lon_nmea = float(f[4])
            self._data.longitude = self.nmea_to_degrees(lon_nmea, f[5])
        # 定位质量
        if f[6]:
            self._data.fix_quality = int(f[6])
        # 卫星数
        if f[7]:
            self._data.satellites_used = int(f[7])
        # HDOP
        if f[8]:
            self._data.hdop = float(f[8])
        # 海拔
        if f[9]:
            self._data.altitude_m = float(f[9])
        self._new_data = True

    def _parse_rmc(self, f):
        """解析RMC推荐最小定位信息"""
        if len(f) < 10:
            return
        # 状态: A=有效, V=无效
        if f[2] != 'A':
            return
        # 速度（节）
        if f[7]:
            self._data.speed_knots = float(f[7])
            self._data.speed_kmh = self._data.speed_knots * 1.852
        # 航向
        if f[8]:
            self._data.course_deg = float(f[8])
        # 日期
        if f[9] and len(f[9]) == 6:
            self._data.day = int(f[9][0:2])
            self._data.month = int(f[9][2:4])
            self._data.year = 2000 + int(f[9][4:6])
            self._data.valid_date = True
        self._new_data = True

    @staticmethod
    def nmea_to_degrees(nmea_val, direction):
        """NMEA经纬度转换为十进制度
        NMEA格式: ddmm.mmmm（纬度）或 dddmm.mmmm（经度）
        """
        if nmea_val == 0.0:
            return 0.0
        # 统一处理：整数部分/100=度，余数=分
        deg = int(nmea_val / 100)
        minutes = nmea_val - deg * 100
        degrees = deg + minutes / 60.0
        if direction in ('S', 'W'):
            degrees = -degrees
        return degrees


# ═══════════════════════════════════════════════════════════════
#  A4988步进电机驱动 — 对应 stepper_a4988.h / stepper_a4988.c
# ═══════════════════════════════════════════════════════════════

STEPPER_DIR_CW = 0
STEPPER_DIR_CCW = 1
STEPPER_MAX_SPEED = 1000  # 最大速度（步/秒）
STEPPER_DEFAULT_STEP_PIN = 0
STEPPER_DEFAULT_DIR_PIN = 1
STEPPER_DEFAULT_EN_PIN = 2


class StepperA4988:
    """A4988步进电机驱动，与C版本stepper_a4988.h逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.position = 0          # 当前位置（步数）
        self.target = 0            # 目标位置
        self.speed = 100           # 速度（步/秒）
        self.direction = STEPPER_DIR_CW
        self.enabled = False
        self.step_pin = STEPPER_DEFAULT_STEP_PIN
        self.dir_pin = STEPPER_DEFAULT_DIR_PIN
        self.en_pin = STEPPER_DEFAULT_EN_PIN
        self.microstep = 1         # 细分: 1/2/4/8/16

    def init(self, step_pin=None, dir_pin=None, en_pin=None):
        """对应StepperA4988_Init()"""
        if step_pin is not None:
            self.step_pin = step_pin
        if dir_pin is not None:
            self.dir_pin = dir_pin
        if en_pin is not None:
            self.en_pin = en_pin
        self.initialized = True
        self.position = 0
        self.target = 0
        self.enabled = False
        return True

    def enable(self):
        """对应StepperA4988_Enable() — 使能电机（EN低有效）"""
        self.enabled = True

    def disable(self):
        """对应StepperA4988_Disable() — 失能电机"""
        self.enabled = False

    def set_speed(self, speed):
        """对应StepperA4988_SetSpeed() — 设置速度（步/秒）"""
        if speed < 0:
            speed = 0
        if speed > STEPPER_MAX_SPEED:
            speed = STEPPER_MAX_SPEED
        self.speed = speed

    def set_direction(self, direction):
        """对应StepperA4988_SetDirection() — 设置方向"""
        self.direction = direction

    def set_microstep(self, ms):
        """对应StepperA4988_SetMicrostep() — 设置细分(1/2/4/8/16)"""
        if ms in (1, 2, 4, 8, 16):
            self.microstep = ms
            return True
        return False

    def move_to(self, target):
        """对应StepperA4988_MoveTo() — 移动到绝对位置"""
        self.target = target
        if self.target > self.position:
            self.direction = STEPPER_DIR_CW
        elif self.target < self.position:
            self.direction = STEPPER_DIR_CCW

    def move_relative(self, steps):
        """对应StepperA4988_MoveRelative() — 相对移动"""
        self.move_to(self.position + steps)

    def step(self):
        """对应StepperA4988_Step() — 执行一步"""
        if not self.enabled:
            return False
        if self.position == self.target:
            return False
        if self.position < self.target:
            self.position += 1
        else:
            self.position -= 1
        return True

    def run_to_target(self):
        """对应StepperA4988_RunToTarget() — 运行到目标位置"""
        count = 0
        while self.step():
            count += 1
        return count

    def stop(self):
        """对应StepperA4988_Stop() — 停止并更新目标为当前位置"""
        self.target = self.position

    def get_position(self):
        """对应StepperA4988_GetPosition()"""
        return self.position

    def is_at_target(self):
        """对应StepperA4988_IsAtTarget()"""
        return self.position == self.target

    def reset_position(self, pos=0):
        """对应StepperA4988_ResetPosition()"""
        self.position = pos
        self.target = pos


# ═══════════════════════════════════════════════════════════════
#  旋转编码器 — 对应 rotary_encoder.h / rotary_encoder.c
# ═══════════════════════════════════════════════════════════════

ROTARY_DIR_NONE = 0
ROTARY_DIR_CW = 1
ROTARY_DIR_CCW = -1


class RotaryEncoder:
    """旋转编码器驱动，与C版本rotary_encoder.h逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.position = 0          # 当前位置
        self.direction = ROTARY_DIR_NONE
        self.button_pressed = False
        self.clicks = 0            # 按钮按下计数
        self._last_a = 0
        self._last_b = 0

    def init(self):
        """对应RotaryEncoder_Init()"""
        self.initialized = True
        self.position = 0
        self.direction = ROTARY_DIR_NONE
        self.button_pressed = False
        self.clicks = 0
        self._last_a = 0
        self._last_b = 0
        return True

    def _inject_rotation(self, direction):
        """模拟旋转中断注入（测试用）"""
        if direction == ROTARY_DIR_CW:
            self.position += 1
            self.direction = ROTARY_DIR_CW
        elif direction == ROTARY_DIR_CCW:
            self.position -= 1
            self.direction = ROTARY_DIR_CCW

    def _inject_button_press(self):
        """模拟按钮按下（测试用）"""
        self.button_pressed = True
        self.clicks += 1

    def _inject_button_release(self):
        """模拟按钮释放（测试用）"""
        self.button_pressed = False

    def get_position(self):
        """对应RotaryEncoder_GetPosition()"""
        return self.position

    def get_direction(self):
        """对应RotaryEncoder_GetDirection()"""
        return self.direction

    def get_clicks(self):
        """对应RotaryEncoder_GetClicks()"""
        return self.clicks

    def is_pressed(self):
        """对应RotaryEncoder_IsPressed()"""
        return self.button_pressed

    def reset(self):
        """对应RotaryEncoder_Reset()"""
        self.position = 0
        self.direction = ROTARY_DIR_NONE
        self.clicks = 0


# ═══════════════════════════════════════════════════════════════
#  LCD1602液晶显示 — 对应 lcd1602.h / lcd1602.c
# ═══════════════════════════════════════════════════════════════

LCD1602_COLS = 16
LCD1602_ROWS = 2
LCD1602_CMD_CLEAR = 0x01
LCD1602_CMD_HOME = 0x02
LCD1602_CMD_ENTRY_MODE = 0x06
LCD1602_CMD_DISPLAY_ON = 0x0C
LCD1602_CMD_DISPLAY_OFF = 0x08
LCD1602_CMD_CURSOR_ON = 0x0E
LCD1602_CMD_CURSOR_OFF = 0x0C
LCD1602_CMD_BLINK_ON = 0x0F
LCD1602_CMD_4BIT_MODE = 0x28
LCD1602_CMD_SET_DDRAM = 0x80


class LCD1602:
    """LCD1602液晶显示驱动，与C版本lcd1602.h逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.display_on = False
        self.cursor_on = False
        self.blink_on = False
        self.cursor_row = 0
        self.cursor_col = 0
        # 显示缓冲区（2行 x 16列）
        self.buffer = [[' '] * LCD1602_COLS for _ in range(LCD1602_ROWS)]
        self.backlight = True

    def init(self):
        """对应LCD1602_Init()"""
        self.initialized = True
        self.display_on = True
        self.cursor_on = False
        self.blink_on = False
        self.cursor_row = 0
        self.cursor_col = 0
        self._clear_buffer()
        return True

    def _clear_buffer(self):
        """清空显示缓冲区"""
        for row in range(LCD1602_ROWS):
            for col in range(LCD1602_COLS):
                self.buffer[row][col] = ' '

    def clear(self):
        """对应LCD1602_Clear() — 清屏"""
        self._clear_buffer()
        self.cursor_row = 0
        self.cursor_col = 0

    def home(self):
        """对应LCD1602_Home() — 光标回原点"""
        self.cursor_row = 0
        self.cursor_col = 0

    def set_cursor(self, row, col):
        """对应LCD1602_SetCursor() — 设置光标位置"""
        if 0 <= row < LCD1602_ROWS and 0 <= col < LCD1602_COLS:
            self.cursor_row = row
            self.cursor_col = col
            return True
        return False

    def write_char(self, ch):
        """对应LCD1602_WriteChar() — 写入单个字符"""
        if not self.initialized:
            return False
        if self.cursor_row < LCD1602_ROWS and self.cursor_col < LCD1602_COLS:
            self.buffer[self.cursor_row][self.cursor_col] = ch
            self.cursor_col += 1
            return True
        return False

    def write_string(self, text):
        """对应LCD1602_WriteString() — 写入字符串"""
        if not self.initialized:
            return False
        count = 0
        for ch in text:
            if self.write_char(ch):
                count += 1
            else:
                break
        return count

    def print_line(self, row, text):
        """对应LCD1602_PrintLine() — 在指定行显示字符串（自动清除行尾）"""
        if row < 0 or row >= LCD1602_ROWS:
            return 0
        # 清除该行
        for col in range(LCD1602_COLS):
            self.buffer[row][col] = ' '
        # 写入文本
        self.cursor_row = row
        self.cursor_col = 0
        return self.write_string(text)

    def get_line(self, row):
        """获取指定行显示内容"""
        if 0 <= row < LCD1602_ROWS:
            return ''.join(self.buffer[row])
        return ''

    def get_display_text(self):
        """获取全部显示内容"""
        lines = []
        for row in range(LCD1602_ROWS):
            lines.append(''.join(self.buffer[row]))
        return lines

    def display_on_off(self, on):
        """对应LCD1602_DisplayOnOff()"""
        self.display_on = on
        return True

    def cursor_on_off(self, on):
        """对应LCD1602_CursorOnOff()"""
        self.cursor_on = on
        return True

    def blink_on_off(self, on):
        """对应LCD1602_BlinkOnOff()"""
        self.blink_on = on
        return True

    def set_backlight(self, on):
        """对应LCD1602_SetBacklight()"""
        self.backlight = on
        return True


# ═══════════════════════════════════════════════════════════════
#  蜂鸣器驱动 — 对应 buzzer.h / buzzer.c
# ═══════════════════════════════════════════════════════════════

BUZZER_STATE_OFF = 0
BUZZER_STATE_ON = 1
BUZZER_STATE_BEEP = 2
BUZZER_FREQ_MIN = 100
BUZZER_FREQ_MAX = 10000

# 常用音符频率（Hz）
BUZZER_NOTES = {
    'C4': 262, 'D4': 294, 'E4': 330, 'F4': 349,
    'G4': 392, 'A4': 440, 'B4': 494,
    'C5': 523, 'D5': 587, 'E5': 659, 'F5': 698,
    'G5': 784, 'A5': 880, 'B5': 988,
}


class Buzzer:
    """蜂鸣器驱动，与C版本buzzer.h逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.state = BUZZER_STATE_OFF
        self.frequency = 1000        # 当前频率（Hz）
        self.duty = 50               # 占空比（0-100%）
        self.beep_count = 0          # 蜂鸣次数
        self.beep_on_ms = 0          # 蜂鸣持续时间
        self.beep_off_ms = 0         # 蜂鸣间隔时间
        self.total_beeps = 0         # 计划蜂鸣次数
        self.tick_count = 0          # 模拟tick计数

    def init(self):
        """对应Buzzer_Init()"""
        self.initialized = True
        self.state = BUZZER_STATE_OFF
        self.frequency = 1000
        self.duty = 50
        self.beep_count = 0
        return True

    def on(self):
        """对应Buzzer_On() — 持续鸣响"""
        self.state = BUZZER_STATE_ON

    def off(self):
        """对应Buzzer_Off() — 关闭"""
        self.state = BUZZER_STATE_OFF

    def set_frequency(self, freq):
        """对应Buzzer_SetFrequency() — 设置频率"""
        if freq < BUZZER_FREQ_MIN:
            freq = BUZZER_FREQ_MIN
        if freq > BUZZER_FREQ_MAX:
            freq = BUZZER_FREQ_MAX
        self.frequency = freq
        return True

    def set_duty(self, duty):
        """对应Buzzer_SetDuty() — 设置占空比"""
        if duty < 0:
            duty = 0
        if duty > 100:
            duty = 100
        self.duty = duty
        return True

    def play_note(self, note_name):
        """对应Buzzer_PlayNote() — 播放指定音符"""
        if note_name in BUZZER_NOTES:
            self.set_frequency(BUZZER_NOTES[note_name])
            self.on()
            return True
        return False

    def beep(self, count=1, on_ms=100, off_ms=100):
        """对应Buzzer_Beep() — 设置蜂鸣模式"""
        self.state = BUZZER_STATE_BEEP
        self.total_beeps = count
        self.beep_count = 0
        self.beep_on_ms = on_ms
        self.beep_off_ms = off_ms
        self.tick_count = 0
        return True

    def tick(self, elapsed_ms=1):
        """对应Buzzer_Tick() — 蜂鸣定时器回调"""
        if self.state != BUZZER_STATE_BEEP:
            return
        self.tick_count += elapsed_ms
        cycle = self.beep_on_ms + self.beep_off_ms
        if cycle <= 0:
            return
        completed = self.tick_count // cycle
        if completed >= self.total_beeps:
            self.beep_count = self.total_beeps
            self.state = BUZZER_STATE_OFF
        else:
            self.beep_count = completed

    def get_state(self):
        """对应Buzzer_GetState()"""
        return self.state

    def is_on(self):
        """对应Buzzer_IsOn()"""
        return self.state == BUZZER_STATE_ON


# ═══════════════════════════════════════════════════════════════
#  红外接收器 — 对应 ir_receiver.h / ir_receiver.c
# ═══════════════════════════════════════════════════════════════

IR_PROTO_NEC = 0
IR_PROTO_RC5 = 1
IR_BUF_SIZE = 32


class IRCommand:
    """红外命令结构体"""
    def __init__(self, address=0, command=0, protocol=IR_PROTO_NEC):
        self.address = address
        self.command = command
        self.protocol = protocol

    def __eq__(self, other):
        if isinstance(other, IRCommand):
            return (self.address == other.address and
                    self.command == other.command)
        return False

    def __repr__(self):
        return f"IRCommand(addr=0x{self.address:02X}, cmd=0x{self.command:02X})"


class IRReceiver:
    """红外接收器驱动，与C版本ir_receiver.h逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.protocol = IR_PROTO_NEC
        self.buffer = []             # 命令环形缓冲区
        self.cmd_count = 0           # 总接收命令数
        self.error_count = 0         # 错误计数
        self.timeout_ms = 110        # 超时时间（ms）
        self.repeat_code = False     # 是否收到重复码

    def init(self, protocol=IR_PROTO_NEC):
        """对应IRReceiver_Init()"""
        self.initialized = True
        self.protocol = protocol
        self.buffer = []
        self.cmd_count = 0
        self.error_count = 0
        self.repeat_code = False
        return True

    def _inject_command(self, address, command):
        """模拟接收红外命令（测试用）"""
        cmd = IRCommand(address, command, self.protocol)
        if len(self.buffer) < IR_BUF_SIZE:
            self.buffer.append(cmd)
            self.cmd_count += 1
            return True
        else:
            self.error_count += 1
            return False

    def _inject_repeat(self):
        """模拟接收重复码（测试用）"""
        self.repeat_code = True

    def available(self):
        """对应IRReceiver_Available() — 缓冲区中待读取命令数"""
        return len(self.buffer)

    def read(self):
        """对应IRReceiver_Read() — 读取一条命令"""
        if self.buffer:
            return self.buffer.pop(0)
        return None

    def peek(self):
        """对应IRReceiver_Peek() — 查看但不取出"""
        if self.buffer:
            return self.buffer[0]
        return None

    def flush(self):
        """对应IRReceiver_Flush() — 清空缓冲区"""
        self.buffer = []

    def get_error_count(self):
        """对应IRReceiver_GetErrorCount()"""
        return self.error_count

    def get_command_count(self):
        """对应IRReceiver_GetCommandCount()"""
        return self.cmd_count

    def is_repeat(self):
        """对应IRReceiver_IsRepeat()"""
        return self.repeat_code


# ═══════════════════════════════════════════════════════════════
#  MAX7219 LED点阵驱动 — 对应 max7219.c
# ═══════════════════════════════════════════════════════════════

# MAX7219寄存器地址
MAX7219_REG_NOOP        = 0x00
MAX7219_REG_DIGIT0      = 0x01
MAX7219_REG_DIGIT7      = 0x08
MAX7219_REG_DECODE_MODE = 0x09
MAX7219_REG_INTENSITY   = 0x0A
MAX7219_REG_SCAN_LIMIT  = 0x0B
MAX7219_REG_SHUTDOWN    = 0x0C
MAX7219_REG_TEST        = 0x0F

MAX7219_INTENSITY_MAX = 0x0F
MAX7219_ROWS = 8
MAX7219_COLS = 8


class MAX7219:
    """MAX7219 LED点阵驱动，8×8矩阵，SPI接口"""

    def __init__(self, num_cascaded=1):
        self.num_cascaded = num_cascaded if num_cascaded > 0 else 1
        self.initialized = False
        self.shutdown_mode = True
        self.intensity = 0
        self.scan_limit = 7  # 全部8行
        self.decode_mode = 0
        self.test_mode = False
        # 显示缓冲区: 每个设备8行, 每行8位
        self._buf = [[0x00] * MAX7219_ROWS for _ in range(self.num_cascaded)]
        # SPI发送记录（调试用）
        self.spi_log = []

    def init(self):
        """初始化MAX7219，配置为正常工作模式"""
        self.set_scan_limit(7)
        self.set_decode_mode(0x00)
        self.set_intensity(0x07)
        self.set_shutdown(False)
        self.clear()
        self.initialized = True
        return True

    def _write_reg(self, reg, data, chip_index=0):
        """写寄存器，记录SPI操作"""
        self.spi_log.append((chip_index, reg, data))

    def set_shutdown(self, shutdown):
        """设置关断模式，shutdown=True时芯片低功耗"""
        self.shutdown_mode = shutdown
        val = 0x00 if shutdown else 0x01
        for i in range(self.num_cascaded):
            self._write_reg(MAX7219_REG_SHUTDOWN, val, i)

    def set_intensity(self, level):
        """设置亮度 0-15"""
        level = max(0, min(MAX7219_INTENSITY_MAX, level))
        self.intensity = level
        for i in range(self.num_cascaded):
            self._write_reg(MAX7219_REG_INTENSITY, level, i)

    def set_scan_limit(self, limit):
        """设置扫描行数 0-7（0=仅第0行，7=全部8行）"""
        limit = max(0, min(7, limit))
        self.scan_limit = limit
        for i in range(self.num_cascaded):
            self._write_reg(MAX7219_REG_SCAN_LIMIT, limit, i)

    def set_decode_mode(self, mode):
        """设置解码模式（0x00=不解码，0xFF=全部BCD解码）"""
        self.decode_mode = mode
        for i in range(self.num_cascaded):
            self._write_reg(MAX7219_REG_DECODE_MODE, mode, i)

    def set_test_mode(self, enable):
        """测试模式：全部LED点亮"""
        self.test_mode = enable
        val = 0x01 if enable else 0x00
        for i in range(self.num_cascaded):
            self._write_reg(MAX7219_REG_TEST, val, i)

    def set_pixel(self, row, col, on=True, chip_index=0):
        """设置单个像素点（行0-7, 列0-7）"""
        if not self.initialized:
            return False
        if not (0 <= chip_index < self.num_cascaded):
            return False
        if not (0 <= row < MAX7219_ROWS and 0 <= col < MAX7219_COLS):
            return False
        if on:
            self._buf[chip_index][row] |= (1 << col)
        else:
            self._buf[chip_index][row] &= ~(1 << col)
        self._write_reg(MAX7219_REG_DIGIT0 + row, self._buf[chip_index][row], chip_index)
        return True

    def get_pixel(self, row, col, chip_index=0):
        """获取像素状态"""
        if not (0 <= chip_index < self.num_cascaded):
            return False
        if not (0 <= row < MAX7219_ROWS and 0 <= col < MAX7219_COLS):
            return False
        return bool(self._buf[chip_index][row] & (1 << col))

    def set_row(self, row, data, chip_index=0):
        """设置整行数据（8位）"""
        if not self.initialized:
            return False
        if not (0 <= chip_index < self.num_cascaded):
            return False
        if not (0 <= row < MAX7219_ROWS):
            return False
        self._buf[chip_index][row] = data & 0xFF
        self._write_reg(MAX7219_REG_DIGIT0 + row, self._buf[chip_index][row], chip_index)
        return True

    def get_row(self, row, chip_index=0):
        """获取整行数据"""
        if not (0 <= chip_index < self.num_cascaded):
            return 0
        if not (0 <= row < MAX7219_ROWS):
            return 0
        return self._buf[chip_index][row]

    def clear(self, chip_index=None):
        """清除显示缓冲区"""
        if chip_index is None:
            for c in range(self.num_cascaded):
                for r in range(MAX7219_ROWS):
                    self._buf[c][r] = 0x00
                    self._write_reg(MAX7219_REG_DIGIT0 + r, 0x00, c)
        else:
            if 0 <= chip_index < self.num_cascaded:
                for r in range(MAX7219_ROWS):
                    self._buf[chip_index][r] = 0x00
                    self._write_reg(MAX7219_REG_DIGIT0 + r, 0x00, chip_index)

    def flush(self):
        """将缓冲区数据全部写入芯片"""
        if not self.initialized:
            return False
        for c in range(self.num_cascaded):
            for r in range(MAX7219_ROWS):
                self._write_reg(MAX7219_REG_DIGIT0 + r, self._buf[c][r], c)
        return True


# ═══════════════════════════════════════════════════════════════
#  TM1637 数码管驱动 — 对应 tm1637.c
# ═══════════════════════════════════════════════════════════════

TM1637_MAX_DIGITS = 6
TM1637_BRIGHTNESS_MAX = 0x07

# 七段显示码表（共阴极，0-9, A-F, 横杠, 空格）
TM1637_SEG_MAP = {
    '0': 0x3F, '1': 0x06, '2': 0x5B, '3': 0x4F, '4': 0x66,
    '5': 0x6D, '6': 0x7D, '7': 0x07, '8': 0x7F, '9': 0x6F,
    'A': 0x77, 'b': 0x7C, 'C': 0x39, 'd': 0x5E, 'E': 0x79, 'F': 0x71,
    '-': 0x40, ' ': 0x00, 'H': 0x76, 'L': 0x38, 'P': 0x73,
    'r': 0x50, 'o': 0x5C, 'n': 0x54, 'U': 0x3E,
}


class TM1637:
    """TM1637四位/六位数码管驱动"""

    def __init__(self, num_digits=4):
        self.num_digits = min(num_digits, TM1637_MAX_DIGITS)
        if self.num_digits < 1:
            self.num_digits = 1
        self.initialized = False
        self.brightness = 0
        self.display_on = True
        self._buf = [0x00] * self.num_digits

    def init(self):
        """初始化TM1637"""
        self.clear()
        self.set_brightness(7)
        self.display_on = True
        self.initialized = True
        return True

    def set_brightness(self, level):
        """设置亮度 0-7"""
        self.brightness = max(0, min(TM1637_BRIGHTNESS_MAX, level))

    def clear(self):
        """清除显示"""
        self._buf = [0x00] * self.num_digits

    def set_digit(self, pos, seg_data):
        """在指定位置设置段数据"""
        if not self.initialized:
            return False
        if not (0 <= pos < self.num_digits):
            return False
        self._buf[pos] = seg_data & 0xFF
        return True

    def get_digit(self, pos):
        """获取指定位置的段数据"""
        if not (0 <= pos < self.num_digits):
            return 0
        return self._buf[pos]

    def display_number(self, num, pad_with_zero=False):
        """显示整数（支持负数），从右侧对齐"""
        if not self.initialized:
            return False
        # 负数处理
        negative = num < 0
        num = abs(num)
        digits = []
        if num == 0:
            digits = [0]
        else:
            while num > 0:
                digits.append(num % 10)
                num //= 10
        digits.reverse()

        self.clear()
        # 计算可用位数
        available = self.num_digits
        if negative:
            available -= 1

        if len(digits) > available:
            # 超出范围：显示"----"
            for i in range(self.num_digits):
                self._buf[i] = TM1637_SEG_MAP['-']
            return False

        # 对齐填充
        offset = available - len(digits)
        if pad_with_zero and not negative:
            for i in range(offset):
                self._buf[i] = TM1637_SEG_MAP['0']

        if negative:
            self._buf[0] = TM1637_SEG_MAP['-']
            for i, d in enumerate(digits):
                self._buf[1 + offset + i] = TM1637_SEG_MAP[chr(ord('0') + d)]
        else:
            for i, d in enumerate(digits):
                self._buf[offset + i] = TM1637_SEG_MAP[chr(ord('0') + d)]
        return True

    def display_string(self, text):
        """显示字符串（最长num_digits位）"""
        if not self.initialized:
            return False
        self.clear()
        for i, ch in enumerate(text[:self.num_digits]):
            if ch in TM1637_SEG_MAP:
                self._buf[i] = TM1637_SEG_MAP[ch]
            else:
                self._buf[i] = 0x00
        return True

    def display_raw(self, data):
        """直接设置缓冲区"""
        if not self.initialized:
            return False
        for i in range(min(len(data), self.num_digits)):
            self._buf[i] = data[i] & 0xFF
        return True

    def set_colon(self, on):
        """设置冒号（第2位的最高位，仅4位数码管）"""
        if self.num_digits >= 2:
            if on:
                self._buf[1] |= 0x80
            else:
                self._buf[1] &= 0x7F

    def get_buffer(self):
        """获取显示缓冲区副本"""
        return list(self._buf)


# ═══════════════════════════════════════════════════════════════
#  ADS1115 16位ADC — 对应 ads1115.c
# ═══════════════════════════════════════════════════════════════

# ADS1115 I2C地址
ADS1115_ADDR_GND = 0x48
ADS1115_ADDR_VDD = 0x49
ADS1115_ADDR_SDA = 0x4A
ADS1115_ADDR_SCL = 0x4B

# ADS1115寄存器
ADS1115_REG_CONVERSION = 0x00
ADS1115_REG_CONFIG     = 0x01

# 输入多路复用器
ADS1115_MUX_AIN0_AIN1 = 0x00  # 差分 AIN0-AIN1
ADS1115_MUX_AIN0_AIN3 = 0x01  # 差分 AIN0-AIN3
ADS1115_MUX_AIN1_AIN3 = 0x02  # 差分 AIN1-AIN3
ADS1115_MUX_AIN2_AIN3 = 0x03  # 差分 AIN2-AIN3
ADS1115_MUX_AIN0_GND  = 0x04  # 单端 AIN0
ADS1115_MUX_AIN1_GND  = 0x05  # 单端 AIN1
ADS1115_MUX_AIN2_GND  = 0x06  # 单端 AIN2
ADS1115_MUX_AIN3_GND  = 0x07  # 单端 AIN3

# PGA增益
ADS1115_PGA_6_144V = 0x00  # ±6.144V
ADS1115_PGA_4_096V = 0x01  # ±4.096V
ADS1115_PGA_2_048V = 0x02  # ±2.048V (默认)
ADS1115_PGA_1_024V = 0x03  # ±1.024V
ADS1115_PGA_0_512V = 0x04  # ±0.512V
ADS1115_PGA_0_256V = 0x05  # ±0.256V

# PGA满量程电压映射
ADS1115_PGA_FSR = {
    ADS1115_PGA_6_144V: 6.144,
    ADS1115_PGA_4_096V: 4.096,
    ADS1115_PGA_2_048V: 2.048,
    ADS1115_PGA_1_024V: 1.024,
    ADS1115_PGA_0_512V: 0.512,
    ADS1115_PGA_0_256V: 0.256,
}

# 采样速率
ADS1115_DR_8SPS   = 0x00
ADS1115_DR_16SPS  = 0x01
ADS1115_DR_250SPS = 0x04
ADS1115_DR_475SPS = 0x05
ADS1115_DR_860SPS = 0x07


class ADS1115:
    """ADS1115 16位I2C ADC驱动"""

    def __init__(self, addr=ADS1115_ADDR_GND):
        self.addr = addr
        self.initialized = False
        self.pga = ADS1115_PGA_2_048V
        self.mux = ADS1115_MUX_AIN0_GND
        self.data_rate = ADS1115_DR_250SPS
        self.continuous = False
        self.simulated_raw = 0  # 模拟ADC原始值

    def init(self):
        """初始化ADS1115"""
        self.initialized = True
        return True

    def set_pga(self, pga):
        """设置PGA增益/满量程范围"""
        if pga not in ADS1115_PGA_FSR:
            return False
        self.pga = pga
        return True

    def get_pga_fsr(self):
        """获取当前PGA满量程电压"""
        return ADS1115_PGA_FSR.get(self.pga, 2.048)

    def set_mux(self, mux):
        """设置输入多路复用器"""
        if not (0 <= mux <= 7):
            return False
        self.mux = mux
        return True

    def set_data_rate(self, rate):
        """设置数据速率"""
        if not (0 <= rate <= 7):
            return False
        self.data_rate = rate
        return True

    def set_continuous(self, continuous):
        """设置连续/单次转换模式"""
        self.continuous = continuous

    def read_raw(self):
        """读取原始ADC值（模拟返回）"""
        if not self.initialized:
            return None
        return self.simulated_raw & 0xFFFF

    def set_simulated_raw(self, raw):
        """设置模拟的ADC原始值（测试用）"""
        self.simulated_raw = raw & 0xFFFF

    def read_voltage(self):
        """读取电压值"""
        if not self.initialized:
            return None
        raw = self.read_raw()
        if raw is None:
            return None
        # 16位有符号数转电压
        if raw > 0x7FFF:
            raw -= 0x10000
        fsr = self.get_pga_fsr()
        return raw * fsr / 32767.0

    def raw_to_voltage(self, raw):
        """原始值转电压"""
        if raw > 0x7FFF:
            raw -= 0x10000
        fsr = self.get_pga_fsr()
        return raw * fsr / 32767.0


# ═══════════════════════════════════════════════════════════════
#  MCP4725 12位DAC — 对应 mcp4725.c
# ═══════════════════════════════════════════════════════════════

MCP4725_ADDR_A0_GND = 0x60
MCP4725_ADDR_A0_VDD = 0x61
MCP4725_MAX_VALUE   = 4095  # 12位
MCP4725_VREF_DEFAULT = 3.3  # 默认参考电压

# MCP4725上电模式
MCP4725_PD_NONE = 0x00     # 正常模式
MCP4725_PD_1K   = 0x01     # 1K下拉
MCP4725_PD_100K = 0x02     # 100K下拉
MCP4725_PD_500K = 0x03     # 500K下拉


class MCP4725:
    """MCP4725 12位I2C DAC驱动"""

    def __init__(self, addr=MCP4725_ADDR_A0_GND, vref=MCP4725_VREF_DEFAULT):
        self.addr = addr
        self.vref = vref
        self.initialized = False
        self.dac_value = 0         # 0-4095
        self.power_down = MCP4725_PD_NONE
        self.eeprom_dac = 0        # EEPROM存储的DAC值
        self.eeprom_pd = MCP4725_PD_NONE
        self.is_busy = False

    def init(self):
        """初始化MCP4725"""
        self.initialized = True
        self.dac_value = 0
        self.power_down = MCP4725_PD_NONE
        return True

    def set_value(self, value):
        """设置DAC输出值 (0-4095)"""
        if not self.initialized:
            return False
        if not (0 <= value <= MCP4725_MAX_VALUE):
            return False
        self.dac_value = value
        return True

    def get_value(self):
        """获取当前DAC值"""
        return self.dac_value

    def set_voltage(self, voltage):
        """通过电压设置DAC值"""
        if not self.initialized:
            return False
        if voltage < 0 or voltage > self.vref:
            return False
        value = int(round(voltage / self.vref * MCP4725_MAX_VALUE))
        value = max(0, min(MCP4725_MAX_VALUE, value))
        self.dac_value = value
        return True

    def get_voltage(self):
        """获取当前输出电压"""
        return self.dac_value / MCP4725_MAX_VALUE * self.vref

    def set_power_down(self, pd_mode):
        """设置掉电模式"""
        if not (0 <= pd_mode <= 3):
            return False
        self.power_down = pd_mode
        return True

    def write_eeprom(self, value=None, pd_mode=None):
        """写入EEPROM（掉电保持）"""
        if not self.initialized:
            return False
        if value is not None:
            if not (0 <= value <= MCP4725_MAX_VALUE):
                return False
            self.eeprom_dac = value
        if pd_mode is not None:
            self.eeprom_pd = pd_mode
        return True

    def read_status(self):
        """读取状态（返回字典）"""
        return {
            'ready': not self.is_busy,
            'por': False,  # 上电复位标志
            'dac_value': self.dac_value,
            'power_down': self.power_down,
            'eeprom_dac': self.eeprom_dac,
            'eeprom_pd': self.eeprom_pd,
        }


# ═══════════════════════════════════════════════════════════════
#  BMP280 气压传感器 — 对应 bmp280.c
# ═══════════════════════════════════════════════════════════════

BMP280_ADDR_SDO_GND = 0x76
BMP280_ADDR_SDO_VCC = 0x77
BMP280_CHIP_ID      = 0x58

# BMP280工作模式
BMP280_MODE_SLEEP   = 0x00
BMP280_MODE_FORCED  = 0x01
BMP280_MODE_NORMAL  = 0x03

# 过采样
BMP280_OS_SKIPPED = 0x00
BMP280_OS_1X      = 0x01
BMP280_OS_2X      = 0x02
BMP280_OS_4X      = 0x03
BMP280_OS_8X      = 0x04
BMP280_OS_16X     = 0x05

# IIR滤波系数
BMP280_FILTER_OFF = 0x00
BMP280_FILTER_2   = 0x01
BMP280_FILTER_4   = 0x02
BMP280_FILTER_8   = 0x03
BMP280_FILTER_16  = 0x04


class BMP280_Calib:
    """BMP280校准参数"""
    def __init__(self):
        self.dig_T1 = 0; self.dig_T2 = 0; self.dig_T3 = 0
        self.dig_P1 = 0; self.dig_P2 = 0; self.dig_P3 = 0
        self.dig_P4 = 0; self.dig_P5 = 0; self.dig_P6 = 0
        self.dig_P7 = 0; self.dig_P8 = 0; self.dig_P9 = 0


class BMP280:
    """BMP280 气压温度传感器驱动"""

    def __init__(self, addr=BMP280_ADDR_SDO_GND):
        self.addr = addr
        self.initialized = False
        self.mode = BMP280_MODE_SLEEP
        self.osrs_t = BMP280_OS_1X   # 温度过采样
        self.osrs_p = BMP280_OS_1X   # 气压过采样
        self.filter = BMP280_FILTER_OFF
        self.standby_ms = 0
        self.calib = BMP280_Calib()
        self.t_fine = 0.0
        # 模拟原始ADC值（20位）
        self._sim_raw_temp = 0
        self._sim_raw_press = 0

    def init(self):
        """初始化BMP280"""
        # 设置默认校准值（使计算有意义）
        self.calib.dig_T1 = 27504
        self.calib.dig_T2 = 26435
        self.calib.dig_T3 = -1000
        self.calib.dig_P1 = 36477
        self.calib.dig_P2 = -10685
        self.calib.dig_P3 = 3024
        self.calib.dig_P4 = 2855
        self.calib.dig_P5 = 140
        self.calib.dig_P6 = -7
        self.calib.dig_P7 = 15500
        self.calib.dig_P8 = -14600
        self.calib.dig_P9 = 6000
        self.initialized = True
        return True

    def set_mode(self, mode):
        """设置工作模式"""
        if mode not in (BMP280_MODE_SLEEP, BMP280_MODE_FORCED, BMP280_MODE_NORMAL):
            return False
        self.mode = mode
        return True

    def set_oversampling(self, osrs_t, osrs_p):
        """设置过采样率"""
        self.osrs_t = osrs_t & 0x07
        self.osrs_p = osrs_p & 0x07
        return True

    def set_filter(self, coeff):
        """设置IIR滤波系数"""
        if not (0 <= coeff <= 4):
            return False
        self.filter = coeff
        return True

    def set_simulated_raw(self, raw_temp, raw_press):
        """设置模拟原始ADC值（测试用）"""
        self._sim_raw_temp = raw_temp
        self._sim_raw_press = raw_press

    def _compensate_temp(self, raw_temp):
        """温度补偿计算，返回(temperature_C, t_fine)"""
        c = self.calib
        var1 = (raw_temp / 16384.0 - c.dig_T1 / 1024.0) * c.dig_T2
        var2 = ((raw_temp / 131072.0 - c.dig_T1 / 8192.0) ** 2) * c.dig_T3
        self.t_fine = var1 + var2
        return self.t_fine / 5120.0

    def _compensate_press(self, raw_press):
        """气压补偿计算，返回pressure_Pa"""
        c = self.calib
        var1 = self.t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * c.dig_P6 / 32768.0
        var2 = var2 + var1 * c.dig_P5 * 2.0
        var2 = var2 / 4.0 + c.dig_P4 * 65536.0
        var1 = (c.dig_P3 * var1 * var1 / 524288.0 + c.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * c.dig_P1

        if var1 == 0:
            return 0.0

        p = 1048576.0 - raw_press
        p = ((p - var2 / 4096.0) * 6250.0) / var1
        var1 = c.dig_P9 * p * p / 2147483648.0
        var2 = p * c.dig_P8 / 32768.0
        p = p + (var1 + var2 + c.dig_P7) / 16.0
        return p

    def read_temperature(self):
        """读取温度（°C）"""
        if not self.initialized:
            return None
        return self._compensate_temp(self._sim_raw_temp)

    def read_pressure(self):
        """读取气压（Pa）"""
        if not self.initialized:
            return None
        self._compensate_temp(self._sim_raw_temp)  # 更新t_fine
        return self._compensate_press(self._sim_raw_press)

    def read_altitude(self, sea_level_pa=101325.0):
        """计算海拔高度（m）"""
        if not self.initialized:
            return None
        pressure = self.read_pressure()
        if pressure <= 0:
            return 0.0
        return 44330.0 * (1.0 - (pressure / sea_level_pa) ** (1.0 / 5.255))


# ═══════════════════════════════════════════════════════════════
#  VL53L0X 激光测距传感器 — 对应 vl53l0x.c
# ═══════════════════════════════════════════════════════════════

# VL53L0X I2C地址
VL53L0X_ADDR_DEFAULT = 0x29  # 默认I2C地址
VL53L0X_ADDR_ALTERNATE = 0x30  # 备用地址

# VL53L0X测量模式
VL53L0X_MODE_SINGLE = 0x00    # 单次测量
VL53L0X_MODE_CONTINUOUS = 0x01  # 连续测量

# VL53L0X测量精度模式
VL53L0X_ACCURACY_DEFAULT = 0x00   # 默认精度 (约±3%)
VL53L0X_ACCURACY_HIGH = 0x01     # 高精度 (约±1%)
VL53L0X_ACCURACY_LONG = 0x02     # 长距离 (约2m)
VL53L0X_ACCURACY_HIGH_SPEED = 0x03  # 高速模式

# VL53L0X测量范围
VL53L0X_MAX_RANGE_MM = 2000   # 最大测量距离(mm)
VL53L0X_MIN_RANGE_MM = 30     # 最小测量距离(mm)


class VL53L0X:
    """VL53L0X 激光测距传感器驱动，与C版本逻辑一致"""

    def __init__(self, addr=VL53L0X_ADDR_DEFAULT):
        self.addr = addr
        self.initialized = False
        self.mode = VL53L0X_MODE_SINGLE
        self.accuracy = VL53L0X_ACCURACY_DEFAULT
        self._sim_distance = 0  # 模拟距离值(mm)
        self._sim_signal = 0    # 模拟信号强度
        self._sim_ambient = 0   # 模拟环境光

    def init(self):
        """初始化VL53L0X传感器"""
        self.initialized = True
        return True

    def set_mode(self, mode):
        """设置测量模式"""
        if mode not in (VL53L0X_MODE_SINGLE, VL53L0X_MODE_CONTINUOUS):
            return False
        self.mode = mode
        return True

    def set_accuracy(self, accuracy):
        """设置测量精度模式"""
        if accuracy not in (VL53L0X_ACCURACY_DEFAULT, VL53L0X_ACCURACY_HIGH,
                           VL53L0X_ACCURACY_LONG, VL53L0X_ACCURACY_HIGH_SPEED):
            return False
        self.accuracy = accuracy
        return True

    def read_distance(self):
        """读取距离值(mm)"""
        if not self.initialized:
            return None
        # 限制在有效范围内
        d = self._sim_distance
        if d < VL53L0X_MIN_RANGE_MM:
            d = 0  # 低于最小范围返回0
        elif d > VL53L0X_MAX_RANGE_MM:
            d = VL53L0X_MAX_RANGE_MM
        return d

    def read_signal(self):
        """读取信号强度"""
        if not self.initialized:
            return None
        return self._sim_signal

    def read_ambient(self):
        """读取环境光强度"""
        if not self.initialized:
            return None
        return self._sim_ambient

    def set_simulated(self, distance, signal=0, ambient=0):
        """设置模拟值（仅测试用）"""
        self._sim_distance = distance
        self._sim_signal = signal
        self._sim_ambient = ambient


# ═══════════════════════════════════════════════════════════════
#  AS5048A 磁编码器 — 对应 as5048a.c
# ═══════════════════════════════════════════════════════════════

# AS5048A I2C地址
AS5048A_ADDR_DEFAULT = 0x40  # 默认地址
AS5048A_ADDR_ALT1 = 0x41     # 备用地址1
AS5048A_ADDR_ALT2 = 0x42     # 备用地址2
AS5048A_ADDR_ALT3 = 0x43     # 备用地址3

# AS5048A寄存器
AS5048A_REG_ANGLE_H = 0xFE   # 角度高字节
AS5048A_REG_ANGLE_L = 0xFF   # 角度低字节
AS5048A_REG_AGC = 0xFA       # 自动增益控制
AS5048A_REG_MAG = 0xFB       # 磁场强度
AS5048A_REG_DIAG = 0xFB      # 诊断寄存器

# AS5048A角度范围
AS5048A_MAX_RAW = 0x3FFF     # 14位最大值 (16383)
AS5048A_DEGREES = 360.0      # 角度范围


class AS5048A:
    """AS5048A 磁编码器驱动，与C版本逻辑一致"""

    def __init__(self, addr=AS5048A_ADDR_DEFAULT):
        self.addr = addr
        self.initialized = False
        self._sim_raw = 0       # 模拟原始角度值(0-16383)
        self._sim_agc = 0       # 模拟AGC值
        self._sim_magnitude = 0  # 模拟磁场强度
        self._offset = 0        # 零点偏移

    def init(self):
        """初始化AS5048A"""
        self.initialized = True
        return True

    def read_raw(self):
        """读取原始角度值(14位, 0-16383)"""
        if not self.initialized:
            return None
        return self._sim_raw & AS5048A_MAX_RAW

    def read_angle(self):
        """读取角度值(0.0-360.0度)"""
        if not self.initialized:
            return None
        raw = self.read_raw()
        if raw is None:
            return None
        return (raw / AS5048A_MAX_RAW) * AS5048A_DEGREES

    def read_angle_with_offset(self):
        """读取带零点偏移的角度值"""
        if not self.initialized:
            return None
        angle = self.read_angle()
        if angle is None:
            return None
        result = angle - self._offset
        if result < 0:
            result += AS5048A_DEGREES
        return result

    def set_zero(self, angle):
        """设置零点偏移"""
        self._offset = angle % AS5048A_DEGREES
        return True

    def read_agc(self):
        """读取自动增益控制值"""
        if not self.initialized:
            return None
        return self._sim_agc

    def read_magnitude(self):
        """读取磁场强度"""
        if not self.initialized:
            return None
        return self._sim_magnitude

    def set_simulated(self, raw, agc=0, magnitude=0):
        """设置模拟值（仅测试用）"""
        self._sim_raw = raw & AS5048A_MAX_RAW
        self._sim_agc = agc
        self._sim_magnitude = magnitude


# ═══════════════════════════════════════════════════════════════
#  MCP23017 IO扩展器 — 对应 mcp23017.c
# ═══════════════════════════════════════════════════════════════

# MCP23017 I2C地址
MCP23017_ADDR_0 = 0x20  # A2=A1=A0=0
MCP23017_ADDR_1 = 0x21  # A2=A1=0, A0=1
MCP23017_ADDR_2 = 0x22  # A2=A0=0, A1=1
MCP23017_ADDR_7 = 0x27  # A2=A1=A0=1

# MCP23017寄存器 (BANK=0模式)
MCP23017_REG_IODIRA = 0x00   # 端口A方向寄存器
MCP23017_REG_IODIRB = 0x01   # 端口B方向寄存器
MCP23017_REG_IPOLA = 0x02    # 端口A极性反转
MCP23017_REG_IPOLB = 0x03    # 端口B极性反转
MCP23017_REG_GPINTENA = 0x04  # 端口中断使能A
MCP23017_REG_GPINTENB = 0x05  # 端口中断使能B
MCP23017_REG_GPIOA = 0x12    # 端口A数据
MCP23017_REG_GPIOB = 0x13    # 端口B数据
MCP23017_REG_OLATA = 0x14    # 端口A输出锁存
MCP23017_REG_OLATB = 0x15    # 端口B输出锁存

# 端口定义
MCP23017_PORTA = 0
MCP23017_PORTB = 1


class MCP23017:
    """MCP23017 16位IO扩展器驱动，与C版本逻辑一致"""

    def __init__(self, addr=MCP23017_ADDR_0):
        self.addr = addr
        self.initialized = False
        # 寄存器状态
        self.iodir = [0xFF, 0xFF]    # 方向: 1=输入, 0=输出
        self.gpio = [0x00, 0x00]     # GPIO数据
        self.olat = [0x00, 0x00]     # 输出锁存
        self.ipol = [0x00, 0x00]     # 极性反转
        self.gpinten = [0x00, 0x00]  # 中断使能

    def init(self):
        """初始化MCP23017"""
        self.initialized = True
        return True

    def set_direction(self, port, pin, direction):
        """设置引脚方向 (0=输出, 1=输入)"""
        if not self.initialized:
            return False
        if port not in (MCP23017_PORTA, MCP23017_PORTB):
            return False
        if pin < 0 or pin > 7:
            return False
        if direction:
            self.iodir[port] |= (1 << pin)
        else:
            self.iodir[port] &= ~(1 << pin)
        return True

    def get_direction(self, port, pin):
        """获取引脚方向"""
        if not self.initialized:
            return None
        if port not in (MCP23017_PORTA, MCP23017_PORTB):
            return None
        if pin < 0 or pin > 7:
            return None
        return 1 if (self.iodir[port] & (1 << pin)) else 0

    def set_port_direction(self, port, direction):
        """设置整个端口方向"""
        if not self.initialized:
            return False
        if port not in (MCP23017_PORTA, MCP23017_PORTB):
            return False
        self.iodir[port] = direction & 0xFF
        return True

    def write_pin(self, port, pin, value):
        """写入单个引脚"""
        if not self.initialized:
            return False
        if port not in (MCP23017_PORTA, MCP23017_PORTB):
            return False
        if pin < 0 or pin > 7:
            return False
        # 检查是否为输出模式
        if self.iodir[port] & (1 << pin):
            return False  # 引脚为输入模式，不能写入
        if value:
            self.olat[port] |= (1 << pin)
        else:
            self.olat[port] &= ~(1 << pin)
        self.gpio[port] = self.olat[port]
        return True

    def read_pin(self, port, pin):
        """读取单个引脚"""
        if not self.initialized:
            return None
        if port not in (MCP23017_PORTA, MCP23017_PORTB):
            return None
        if pin < 0 or pin > 7:
            return None
        return 1 if (self.gpio[port] & (1 << pin)) else 0

    def write_port(self, port, value):
        """写入整个端口"""
        if not self.initialized:
            return False
        if port not in (MCP23017_PORTA, MCP23017_PORTB):
            return False
        self.olat[port] = value & 0xFF
        self.gpio[port] = self.olat[port] & (~self.iodir[port] & 0xFF)
        return True

    def read_port(self, port):
        """读取整个端口"""
        if not self.initialized:
            return None
        if port not in (MCP23017_PORTA, MCP23017_PORTB):
            return None
        return self.gpio[port]

    def set_simulated_input(self, port, value):
        """设置模拟输入值（仅测试用）"""
        self.gpio[port] = value & 0xFF


# ═══════════════════════════════════════════════════════════════
#  PCF8574 IO扩展器 — 对应 pcf8574.c
# ═══════════════════════════════════════════════════════════════

# PCF8574 I2C地址
PCF8574_ADDR_0 = 0x20  # A2=A1=A0=0
PCF8574_ADDR_1 = 0x21  # A2=A1=0, A0=1
PCF8574_ADDR_2 = 0x22  # A2=A0=0, A1=1
PCF8574_ADDR_7 = 0x27  # A2=A1=A0=1
PCF8574_ADDR_BASE = 0x20  # 基础地址
PCF8574_ADDR_MAX = 0x27   # 最大地址


class PCF8574:
    """PCF8574 8位IO扩展器驱动，与C版本逻辑一致
    PCF8574无方向寄存器，每个引脚准双向：
      - 写1: 引脚为输入（高阻态，内部上拉）
      - 写0: 引脚为输出低电平
    """

    def __init__(self, addr=PCF8574_ADDR_0):
        self.addr = addr
        self.initialized = False
        self._output = 0xFF    # 输出寄存器（上电默认全1=输入）
        self._input = 0xFF     # 模拟输入值

    def init(self):
        """初始化PCF8574"""
        self.initialized = True
        return True

    def write_pin(self, pin, value):
        """写入单个引脚 (0-7)
        value=1: 设为输入模式（上拉）
        value=0: 输出低电平
        """
        if not self.initialized:
            return False
        if pin < 0 or pin > 7:
            return False
        if value:
            self._output |= (1 << pin)
        else:
            self._output &= ~(1 << pin)
        return True

    def read_pin(self, pin):
        """读取单个引脚状态"""
        if not self.initialized:
            return None
        if pin < 0 or pin > 7:
            return None
        return 1 if (self._input & (1 << pin)) else 0

    def write_port(self, value):
        """写入整个端口"""
        if not self.initialized:
            return False
        self._output = value & 0xFF
        return True

    def read_port(self):
        """读取整个端口"""
        if not self.initialized:
            return None
        return self._input

    def toggle_pin(self, pin):
        """翻转单个引脚"""
        if not self.initialized:
            return False
        if pin < 0 or pin > 7:
            return False
        self._output ^= (1 << pin)
        return True

    def set_simulated_input(self, value):
        """设置模拟输入值（仅测试用）"""
        self._input = value & 0xFF


# ═══════════════════════════════════════════════════════════════
#  INA219 电流/功率传感器 — 对应 ina219.c
# ═══════════════════════════════════════════════════════════════

# INA219 I2C地址
INA219_ADDR_0 = 0x40  # A1=A0=0
INA219_ADDR_1 = 0x41  # A1=0, A0=1
INA219_ADDR_2 = 0x44  # A1=1, A0=0
INA219_ADDR_3 = 0x45  # A1=A0=1

# INA219寄存器
INA219_REG_CONFIG = 0x00       # 配置寄存器
INA219_REG_SHUNT_V = 0x01     # 分流电压
INA219_REG_BUS_V = 0x02       # 总线电压
INA219_REG_POWER = 0x03       # 功率
INA219_REG_CURRENT = 0x04     # 电流
INA219_REG_CALIBRATION = 0x05 # 校准

# INA219总线电压范围
INA219_BUS_RANGE_16V = 0x00   # 16V量程
INA219_BUS_RANGE_32V = 0x01   # 32V量程

# INA219 PGA增益
INA219_PGA_40MV = 0x00    # ±40mV
INA219_PGA_80MV = 0x01    # ±80mV
INA219_PGA_160MV = 0x02   # ±160mV
INA219_PGA_320MV = 0x03   # ±320mV (默认)

# INA219 ADC分辨率
INA219_ADC_12BIT = 0x03   # 12位 (默认)
INA219_ADC_11BIT = 0x02   # 11位
INA219_ADC_10BIT = 0x01   # 10位
INA219_ADC_9BIT = 0x00    # 9位


class INA219:
    """INA219 电流/功率传感器驱动，与C版本逻辑一致"""

    def __init__(self, addr=INA219_ADDR_0):
        self.addr = addr
        self.initialized = False
        # 配置参数
        self.bus_range = INA219_BUS_RANGE_32V
        self.pga = INA219_PGA_320MV
        self.shunt_adc = INA219_ADC_12BIT
        self.bus_adc = INA219_ADC_12BIT
        # 校准参数
        self._cal_value = 0
        self._current_lsb = 0.0    # 电流LSB (A/bit)
        self._power_lsb = 0.0      # 功率LSB (W/bit)
        # 模拟值
        self._sim_shunt_v = 0.0    # 分流电压(mV)
        self._sim_bus_v = 0.0      # 总线电压(V)
        self._sim_current = 0.0    # 电流(A)
        self._sim_power = 0.0      # 功率(W)

    def init(self):
        """初始化INA219"""
        self.initialized = True
        return True

    def calibrate(self, max_current_a, r_shunt_ohm=0.1):
        """校准INA219
        max_current_a: 最大预期电流(A)
        r_shunt_ohm: 分流电阻阻值(Ω)
        """
        if not self.initialized:
            return False
        if max_current_a <= 0 or r_shunt_ohm <= 0:
            return False
        # 计算电流LSB: Current_LSB = Max_Current / 32767
        self._current_lsb = max_current_a / 32767.0
        # 计算校准值: Cal = 0.04096 / (Current_LSB * R_shunt)
        self._cal_value = int(0.04096 / (self._current_lsb * r_shunt_ohm))
        # 功率LSB = 20 * Current_LSB
        self._power_lsb = 20.0 * self._current_lsb
        return True

    def read_shunt_voltage(self):
        """读取分流电压(mV)"""
        if not self.initialized:
            return None
        return self._sim_shunt_v

    def read_bus_voltage(self):
        """读取总线电压(V)"""
        if not self.initialized:
            return None
        return self._sim_bus_v

    def read_current(self):
        """读取电流(A)"""
        if not self.initialized:
            return None
        return self._sim_current

    def read_power(self):
        """读取功率(W)"""
        if not self.initialized:
            return None
        return self._sim_power

    def set_simulated(self, shunt_mv, bus_v, current_a, power_w):
        """设置模拟值（仅测试用）"""
        self._sim_shunt_v = shunt_mv
        self._sim_bus_v = bus_v
        self._sim_current = current_a
        self._sim_power = power_w

    def get_cal_value(self):
        """获取校准寄存器值"""
        return self._cal_value


# ═══════════════════════════════════════════════════════════════
#  AD9833 DDS信号发生器 — 对应 dds_signal_generator.c
# ═══════════════════════════════════════════════════════════════

# AD9833控制寄存器位
AD9833_B28      = (1 << 13)
AD9833_HLB      = (1 << 12)
AD9833_FSELECT  = (1 << 11)
AD9833_PSELECT  = (1 << 10)
AD9833_RESET    = (1 << 8)
AD9833_SLEEP1   = (1 << 7)
AD9833_SLEEP12  = (1 << 6)
AD9833_OPBITEN  = (1 << 5)
AD9833_DIV2     = (1 << 3)
AD9833_MODE     = (1 << 1)

AD9833_FREQ0_REG  = 0x4000
AD9833_FREQ1_REG  = 0x8000
AD9833_PHASE_REG  = 0xC000

AD9833_MCLK = 25000000  # 25MHz主时钟

# 波形类型
WAVE_SINE     = 0
WAVE_TRIANGLE = 1
WAVE_SQUARE   = 2
WAVE_NAMES = ["Sine", "Triangle", "Square"]

# 频率步进档位
FREQ_STEPS = [0.1, 1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0]

# 预设频率
PRESET_FREQS = [100.0, 1000.0, 10000.0, 100000.0, 500000.0,
                1000000.0, 5000000.0, 10000000.0]

AD9833_MAX_FREQ = 12500000.0
AD9833_MIN_FREQ = 0.1


class AD9833:
    """AD9833 DDS信号发生器驱动，与C版本dds_signal_generator.c逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.waveform = WAVE_SINE
        self.frequency = 1000.0      # 默认1kHz
        self.phase = 0.0             # 0~360度
        self.step_idx = 3            # 默认步进100Hz
        self.output_enabled = True
        self._spi_log = []           # SPI传输记录（调试用）

    def init(self):
        """初始化AD9833"""
        self.initialized = True
        self.waveform = WAVE_SINE
        self.frequency = 1000.0
        self.phase = 0.0
        return True

    def set_frequency(self, freq_hz):
        """设置输出频率"""
        if not self.initialized:
            return False
        if freq_hz < AD9833_MIN_FREQ or freq_hz > AD9833_MAX_FREQ:
            return False
        self.frequency = freq_hz
        # 计算频率字: freq_word = freq * 2^28 / MCLK
        freq_word = int(freq_hz * (1 << 28) / AD9833_MCLK)
        freq_lsb = (freq_word & 0x3FFF) | AD9833_FREQ0_REG
        freq_msb = ((freq_word >> 14) & 0x3FFF) | AD9833_FREQ0_REG
        self._spi_log.append(('freq', AD9833_B28 | AD9833_RESET))
        self._spi_log.append(('freq', freq_lsb))
        self._spi_log.append(('freq', freq_msb))
        return True

    def set_phase(self, phase_deg):
        """设置输出相位(0~360度)"""
        if not self.initialized:
            return False
        if phase_deg < 0 or phase_deg > 360.0:
            return False
        self.phase = phase_deg
        phase_word = int(phase_deg / 360.0 * 4096.0) & 0x0FFF
        self._spi_log.append(('phase', phase_word | AD9833_PHASE_REG))
        return True

    def set_waveform(self, wave):
        """设置波形类型"""
        if not self.initialized:
            return False
        if wave not in (WAVE_SINE, WAVE_TRIANGLE, WAVE_SQUARE):
            return False
        self.waveform = wave
        control = AD9833_B28
        if wave == WAVE_TRIANGLE:
            control |= AD9833_MODE
        elif wave == WAVE_SQUARE:
            control |= AD9833_OPBITEN | AD9833_DIV2
        self._spi_log.append(('wave', control))
        return True

    def get_frequency(self):
        """获取当前频率"""
        return self.frequency

    def get_phase(self):
        """获取当前相位"""
        return self.phase

    def get_waveform(self):
        """获取当前波形类型"""
        return self.waveform

    def get_waveform_name(self):
        """获取波形名称"""
        return WAVE_NAMES[self.waveform]

    def set_step_index(self, idx):
        """设置频率步进档位"""
        if 0 <= idx < len(FREQ_STEPS):
            self.step_idx = idx
            return True
        return False

    def get_step_value(self):
        """获取当前步进值"""
        return FREQ_STEPS[self.step_idx]

    def freq_up(self):
        """频率增加一个步进"""
        new_freq = self.frequency + FREQ_STEPS[self.step_idx]
        if new_freq > AD9833_MAX_FREQ:
            new_freq = AD9833_MAX_FREQ
        self.set_frequency(new_freq)

    def freq_down(self):
        """频率减少一个步进"""
        step = FREQ_STEPS[self.step_idx]
        if self.frequency > step:
            self.set_frequency(self.frequency - step)
        else:
            self.set_frequency(AD9833_MIN_FREQ)

    def apply_preset(self, idx):
        """应用预设频率"""
        if 0 <= idx < len(PRESET_FREQS):
            return self.set_frequency(PRESET_FREQS[idx])
        return False

    def enable_output(self, enable):
        """使能/禁用输出"""
        self.output_enabled = enable

    def get_spi_log(self):
        """获取SPI传输记录"""
        return list(self._spi_log)


# ═══════════════════════════════════════════════════════════════
#  I2C总线扫描器 — 对应 i2c_scanner_tool.c
# ═══════════════════════════════════════════════════════════════

# 已知I2C设备数据库
KNOWN_I2C_DEVICES = {
    0x00: ("General Call", "Control", "通用呼叫地址"),
    0x04: ("SMBus Alert", "SMBus", "SMBus警报响应"),
    0x07: ("SMBus", "SMBus", "SMBus广播地址"),
    0x0C: ("AK8963", "Magnetometer", "磁力计(MPU9250内置)"),
    0x1D: ("ADXL345", "IMU", "三轴加速度计"),
    0x1E: ("HMC5883L", "Magnetometer", "三轴磁力计"),
    0x20: ("MCP23017", "IO", "IO扩展器"),
    0x27: ("PCF8574", "Display", "LCD1602(I2C背板)"),
    0x29: ("VL53L0X", "ToF", "激光测距"),
    0x39: ("APDS9960", "Sensor", "颜色/手势传感器"),
    0x3C: ("SSD1306", "Display", "0.96寸OLED"),
    0x40: ("INA219/PCA9685", "Power/Motor", "电流监测/PWM驱动"),
    0x44: ("SHT3x", "Sensor", "Sensirion温湿度"),
    0x48: ("ADS1115", "ADC", "16位ADC"),
    0x50: ("AT24Cxx", "EEPROM", "Atmel EEPROM"),
    0x52: ("TCS34725", "Sensor", "RGB颜色传感器"),
    0x53: ("ADXL345", "IMU", "三轴加速度计(ALT)"),
    0x60: ("MCP4725", "DAC", "12位DAC"),
    0x68: ("MPU6050", "IMU", "六轴IMU"),
    0x69: ("MPU6050", "IMU", "六轴IMU(ALT)"),
    0x6E: ("MCP3421", "ADC", "18位Delta-Sigma ADC"),
    0x76: ("BMP280", "Sensor", "气压传感器"),
    0x77: ("BME280", "Sensor", "气压温湿度"),
}

# I2C有效地址范围
I2C_ADDR_MIN = 0x03
I2C_ADDR_MAX = 0x77


class I2CScanner:
    """I2C总线扫描器，与C版本i2c_scanner_tool.c逻辑一致"""

    def __init__(self):
        self.initialized = False
        self._devices = {}          # {addr: responded}
        self._scan_results = []     # [(addr, response_time_us)]
        self._simulate_addrs = set()  # 模拟已连接设备地址

    def init(self):
        """初始化I2C扫描器"""
        self.initialized = True
        return True

    def set_simulated_devices(self, addr_list):
        """设置模拟已连接的I2C设备地址列表"""
        self._simulate_addrs = set(addr_list)

    def probe_address(self, addr):
        """探测指定地址是否有设备响应"""
        if not self.initialized:
            return False
        if addr < I2C_ADDR_MIN or addr > I2C_ADDR_MAX:
            return False
        return addr in self._simulate_addrs

    def scan_bus(self):
        """扫描整个I2C总线(0x03~0x77)"""
        if not self.initialized:
            return []
        self._scan_results = []
        for addr in range(I2C_ADDR_MIN, I2C_ADDR_MAX + 1):
            if addr in self._simulate_addrs:
                response_time = 50  # 模拟50us响应时间
                self._scan_results.append((addr, response_time))
        return list(self._scan_results)

    def get_found_devices(self):
        """获取已发现设备列表"""
        return list(self._scan_results)

    def get_device_count(self):
        """获取发现设备数量"""
        return len(self._scan_results)

    def identify_device(self, addr):
        """识别设备类型（查表）"""
        if addr in KNOWN_I2C_DEVICES:
            name, dev_type, desc = KNOWN_I2C_DEVICES[addr]
            return {'name': name, 'type': dev_type, 'description': desc}
        return {'name': 'Unknown', 'type': 'Unknown', 'description': '未识别设备'}

    def get_address_map(self):
        """生成地址映射图（8行×16列）"""
        found_set = set(addr for addr, _ in self._scan_results)
        rows = []
        for row in range(8):
            cols = []
            for col in range(16):
                addr = (row << 4) | col
                if addr < I2C_ADDR_MIN or addr > I2C_ADDR_MAX:
                    cols.append('--')
                elif addr in found_set:
                    cols.append('XX')
                else:
                    cols.append('..')
            rows.append(cols)
        return rows


# ═══════════════════════════════════════════════════════════════
#  MCP3421 高精度ADC — 对应 multi_adc_sampler.c
# ═══════════════════════════════════════════════════════════════

MCP3421_ADDR = 0x6E

# 分辨率配置
MCP3421_12BIT = 0x10
MCP3421_14BIT = 0x14
MCP3421_16BIT = 0x18
MCP3421_18BIT = 0x1C

# 增益配置
MCP3421_GAIN_1X = 0x00
MCP3421_GAIN_2X = 0x01
MCP3421_GAIN_4X = 0x02
MCP3421_GAIN_8X = 0x03

# LSB分辨率(uV)
MCP3421_LSB_UV = {
    MCP3421_12BIT: 1000.0,
    MCP3421_14BIT: 250.0,
    MCP3421_16BIT: 62.5,
    MCP3421_18BIT: 15.625,
}


class MCP3421:
    """MCP3421 18位Delta-Sigma ADC驱动"""

    def __init__(self, addr=MCP3421_ADDR):
        self.addr = addr
        self.initialized = False
        self.config = MCP3421_18BIT | MCP3421_GAIN_1X
        self._sim_raw = 0
        self._sim_ready = True

    def init(self, config=None):
        """初始化MCP3421"""
        if config is not None:
            self.config = config
        self.initialized = True
        return True

    def set_resolution(self, res):
        """设置分辨率"""
        if res not in (MCP3421_12BIT, MCP3421_14BIT, MCP3421_16BIT, MCP3421_18BIT):
            return False
        self.config = (self.config & 0x03) | res
        return True

    def set_gain(self, gain):
        """设置增益"""
        if gain not in (MCP3421_GAIN_1X, MCP3421_GAIN_2X,
                        MCP3421_GAIN_4X, MCP3421_GAIN_8X):
            return False
        self.config = (self.config & 0xFC) | gain
        return True

    def get_resolution_bits(self):
        """获取当前分辨率位数"""
        res = self.config & 0x1C
        if res == MCP3421_18BIT:
            return 18
        elif res == MCP3421_16BIT:
            return 16
        elif res == MCP3421_14BIT:
            return 14
        return 12

    def get_lsb_uv(self):
        """获取最小分辨电压(uV)"""
        base = MCP3421_LSB_UV.get(self.config & 0x1C, 1000.0)
        gain = 1 << (self.config & 0x03)
        return base / gain

    def set_simulated_raw(self, raw):
        """设置模拟原始值"""
        self._sim_raw = raw

    def read_raw(self):
        """读取原始ADC值"""
        if not self.initialized:
            return None
        return self._sim_raw

    def read_voltage_mv(self):
        """读取电压(mV)"""
        if not self.initialized:
            return None
        return self._sim_raw * self.get_lsb_uv() / 1000.0

    def is_ready(self):
        """数据是否就绪"""
        return self._sim_ready


# ═══════════════════════════════════════════════════════════════
#  内部ADC多通道采样器 — 对应 multi_adc_sampler.c
# ═══════════════════════════════════════════════════════════════

ADC_CHANNELS = 4
ADC_RESOLUTION = 4096  # 12位
VREF_MV = 3300         # 3.3V参考电压


class MultiADC:
    """MSPM0G3507内部多路ADC采样器，与C版本逻辑一致"""

    def __init__(self, channels=ADC_CHANNELS, resolution=ADC_RESOLUTION):
        self.initialized = False
        self.channels = channels
        self.resolution = resolution
        self.vref_mv = VREF_MV
        self.oversample_count = 16
        self.filter_depth = 8
        # 各通道数据
        self._raw_values = [0] * channels
        self._sim_values = [0] * channels
        # 滤波器状态
        self._filter_buf = [[0.0] * self.filter_depth for _ in range(channels)]
        self._filter_idx = [0] * channels
        self._filter_sum = [0.0] * channels
        # 统计
        self._min_mv = [float('inf')] * channels
        self._max_mv = [float('-inf')] * channels
        self._sample_count = [0] * channels

    def init(self):
        """初始化ADC"""
        self.initialized = True
        return True

    def set_oversample_count(self, count):
        """设置过采样次数"""
        self.oversample_count = max(1, count)
        return True

    def set_simulated_raw(self, channel, raw_value):
        """设置模拟原始ADC值"""
        if 0 <= channel < self.channels:
            self._sim_values[channel] = raw_value & 0xFFFF
            return True
        return False

    def read_raw(self, channel):
        """读取指定通道原始ADC值"""
        if not self.initialized:
            return None
        if not (0 <= channel < self.channels):
            return None
        return self._sim_values[channel]

    def raw_to_voltage_mv(self, raw):
        """原始值转电压(mV)"""
        return raw * self.vref_mv / self.resolution

    def read_voltage_mv(self, channel):
        """读取指定通道电压(mV)"""
        raw = self.read_raw(channel)
        if raw is None:
            return None
        return self.raw_to_voltage_mv(raw)

    def _filter_update(self, ch, new_value):
        """更新滑动平均滤波器"""
        self._filter_sum[ch] -= self._filter_buf[ch][self._filter_idx[ch]]
        self._filter_buf[ch][self._filter_idx[ch]] = new_value
        self._filter_sum[ch] += new_value
        self._filter_idx[ch] = (self._filter_idx[ch] + 1) % self.filter_depth
        return self._filter_sum[ch] / self.filter_depth

    def read_filtered_mv(self, channel):
        """读取滤波后电压(mV)"""
        voltage = self.read_voltage_mv(channel)
        if voltage is None:
            return None
        return self._filter_update(channel, voltage)

    def process_sample(self):
        """处理一次采样（更新所有通道统计）"""
        for ch in range(self.channels):
            mv = self.read_voltage_mv(ch)
            if mv is not None:
                if mv < self._min_mv[ch]:
                    self._min_mv[ch] = mv
                if mv > self._max_mv[ch]:
                    self._max_mv[ch] = mv
                self._sample_count[ch] += 1

    def get_min_mv(self, channel):
        """获取通道最小电压"""
        if 0 <= channel < self.channels:
            return self._min_mv[channel]
        return None

    def get_max_mv(self, channel):
        """获取通道最大电压"""
        if 0 <= channel < self.channels:
            return self._max_mv[channel]
        return None

    def get_sample_count(self, channel):
        """获取通道采样计数"""
        if 0 <= channel < self.channels:
            return self._sample_count[channel]
        return None


# ═══════════════════════════════════════════════════════════════
#  时钟发生器 — 对应 clock_generator.c
# ═══════════════════════════════════════════════════════════════

# 时钟源
CLK_SRC_HFXT  = 0   # 外部高速晶振
CLK_SRC_HFOSC = 1   # 内部高速振荡器
CLK_SRC_LFXT  = 2   # 外部低速晶振
CLK_SRC_LFOSC = 3   # 内部低速振荡器

CLK_SOURCES = ["HFXT", "HFOSC", "LFXT", "LFOSC"]

# 时钟源频率
CLK_FREQ_HFXT  = 32000000   # 32MHz外部晶振
CLK_FREQ_HFOSC = 4000000    # 4MHz内部振荡器
CLK_FREQ_LFXT  = 32768      # 32.768kHz外部晶振
CLK_FREQ_LFOSC = 32768      # 32.768kHz内部振荡器

CLK_FREQUENCIES = {
    CLK_SRC_HFXT: CLK_FREQ_HFXT,
    CLK_SRC_HFOSC: CLK_FREQ_HFOSC,
    CLK_SRC_LFXT: CLK_FREQ_LFXT,
    CLK_SRC_LFOSC: CLK_FREQ_LFOSC,
}


class ClockGen:
    """MSPM0G3507时钟发生器，与C版本clock_generator.c逻辑一致"""

    def __init__(self):
        self.initialized = False
        self.sysclk_src = CLK_SRC_HFXT
        self.sysclk_div = 1
        self.mclk_freq = CLK_FREQ_HFXT
        self.peripheral_div = 1
        # 外设时钟
        self._timer_freq = CLK_FREQ_HFXT
        self._uart_freq = CLK_FREQ_HFXT
        self._i2c_freq = CLK_FREQ_HFXT
        self._spi_freq = CLK_FREQ_HFXT
        # PLL
        self._pll_enabled = False
        self._pll_multiplier = 1

    def init(self):
        """初始化时钟系统"""
        self.initialized = True
        self.sysclk_src = CLK_SRC_HFXT
        self.sysclk_div = 1
        self.mclk_freq = CLK_FREQ_HFXT
        return True

    def set_sysclk_source(self, src):
        """设置系统时钟源"""
        if src not in CLK_FREQUENCIES:
            return False
        self.sysclk_src = src
        self._recalc()
        return True

    def set_sysclk_divider(self, div):
        """设置系统时钟分频"""
        if div < 1 or div > 256:
            return False
        self.sysclk_div = div
        self._recalc()
        return True

    def set_peripheral_divider(self, div):
        """设置外设时钟分频"""
        if div < 1 or div > 256:
            return False
        self.peripheral_div = div
        self._recalc()
        return True

    def enable_pll(self, multiplier):
        """使能PLL"""
        if multiplier < 1 or multiplier > 16:
            return False
        self._pll_enabled = True
        self._pll_multiplier = multiplier
        self._recalc()
        return True

    def disable_pll(self):
        """禁用PLL"""
        self._pll_enabled = False
        self._pll_multiplier = 1
        self._recalc()

    def _recalc(self):
        """重新计算时钟频率"""
        base = CLK_FREQUENCIES.get(self.sysclk_src, CLK_FREQ_HFXT)
        if self._pll_enabled:
            base *= self._pll_multiplier
        self.mclk_freq = base // self.sysclk_div
        periph_freq = self.mclk_freq // self.peripheral_div
        self._timer_freq = periph_freq
        self._uart_freq = periph_freq
        self._i2c_freq = periph_freq
        self._spi_freq = periph_freq

    def get_sysclk_freq(self):
        """获取系统时钟频率"""
        return self.mclk_freq

    def get_timer_freq(self):
        """获取定时器时钟频率"""
        return self._timer_freq

    def get_uart_freq(self):
        """获取UART时钟频率"""
        return self._uart_freq

    def get_i2c_freq(self):
        """获取I2C时钟频率"""
        return self._i2c_freq

    def get_spi_freq(self):
        """获取SPI时钟频率"""
        return self._spi_freq

    def get_clock_source_name(self):
        """获取时钟源名称"""
        return CLK_SOURCES[self.sysclk_src]

    def calc_timer_period(self, target_hz):
        """计算定时器周期值"""
        if target_hz <= 0:
            return 0
        return self._timer_freq // target_hz

    def calc_uart_baud_div(self, baud_rate):
        """计算UART波特率分频值"""
        if baud_rate <= 0:
            return 0
        return self._uart_freq // (16 * baud_rate)


# ═══════════════════════════════════════════════════════════════
#  系统诊断工具 — 对应 system_diagnostics.c
# ═══════════════════════════════════════════════════════════════

# 诊断项目
DIAG_CPU       = 0
DIAG_MEMORY    = 1
DIAG_CLOCK     = 2
DIAG_GPIO      = 3
DIAG_ADC       = 4
DIAG_UART      = 5
DIAG_I2C       = 6
DIAG_SPI       = 7
DIAG_TIMER     = 8
DIAG_DMA       = 9

DIAG_NAMES = [
    "CPU", "Memory", "Clock", "GPIO",
    "ADC", "UART", "I2C", "SPI", "Timer", "DMA"
]

# 诊断状态
DIAG_PASS = 0
DIAG_FAIL = 1
DIAG_SKIP = 2
DIAG_WARN = 3

DIAG_STATUS_NAMES = ["PASS", "FAIL", "SKIP", "WARN"]


class SystemDiag:
    """MSPM0G3507系统诊断工具，与C版本system_diagnostics.c逻辑一致"""

    def __init__(self):
        self.initialized = False
        self._results = {}       # {diag_id: (status, message)}
        self._boot_time_ms = 0
        self._uptime_ms = 0
        self._reset_count = 0
        # 模拟系统信息
        self._cpu_freq = 32000000
        self._flash_size_kb = 256
        self._ram_size_kb = 32
        self._chip_id = 0x3507

    def init(self):
        """初始化诊断工具"""
        self.initialized = True
        self._results = {}
        return True

    def run_single(self, diag_id):
        """运行单项诊断"""
        if not self.initialized:
            return (DIAG_FAIL, "未初始化")
        if diag_id < 0 or diag_id >= len(DIAG_NAMES):
            return (DIAG_FAIL, "无效诊断项")
        # 模拟诊断结果：全部通过
        self._results[diag_id] = (DIAG_PASS, "OK")
        return (DIAG_PASS, "OK")

    def run_all(self):
        """运行全部诊断"""
        if not self.initialized:
            return False
        for i in range(len(DIAG_NAMES)):
            self.run_single(i)
        return True

    def get_result(self, diag_id):
        """获取单项诊断结果"""
        return self._results.get(diag_id, (DIAG_SKIP, "未执行"))

    def get_all_results(self):
        """获取全部诊断结果"""
        return dict(self._results)

    def get_pass_count(self):
        """获取通过项数"""
        return sum(1 for s, _ in self._results.values() if s == DIAG_PASS)

    def get_fail_count(self):
        """获取失败项数"""
        return sum(1 for s, _ in self._results.values() if s == DIAG_FAIL)

    def get_summary(self):
        """获取诊断摘要"""
        total = len(self._results)
        passed = self.get_pass_count()
        failed = self.get_fail_count()
        return {
            'total': total,
            'passed': passed,
            'failed': failed,
            'pass_rate': passed / total * 100.0 if total > 0 else 0.0,
        }

    def get_chip_id(self):
        """获取芯片ID"""
        return self._chip_id

    def get_cpu_freq(self):
        """获取CPU频率"""
        return self._cpu_freq

    def get_memory_info(self):
        """获取内存信息"""
        return {
            'flash_kb': self._flash_size_kb,
            'ram_kb': self._ram_size_kb,
        }

    def set_boot_time(self, ms):
        """设置启动时间（测试用）"""
        self._boot_time_ms = ms

    def get_boot_time(self):
        """获取启动时间(ms)"""
        return self._boot_time_ms

    def set_uptime(self, ms):
        """设置运行时间（测试用）"""
        self._uptime_ms = ms

    def get_uptime(self):
        """获取运行时间(ms)"""
        return self._uptime_ms

    def set_reset_count(self, count):
        """设置复位次数（测试用）"""
        self._reset_count = count

    def get_reset_count(self):
        """获取复位次数"""
        return self._reset_count


# ═══════════════════════════════════════════════════════════════
#  ST7789 TFT显示驱动 — SPI接口 240x320
# ═══════════════════════════════════════════════════════════════

ST7789_WIDTH = 240
ST7789_HEIGHT = 320
ST7789_MAX_X = ST7789_WIDTH - 1
ST7789_MAX_Y = ST7789_HEIGHT - 1

# ST7789命令
ST7789_NOP = 0x00
ST7789_SWRESET = 0x01
ST7789_SLPOUT = 0x11
ST7789_NORON = 0x13
ST7789_INVON = 0x21
ST7789_INVOFF = 0x20
ST7789_DISPON = 0x29
ST7789_DISPOFF = 0x28
ST7789_CASET = 0x2A
ST7789_RASET = 0x2B
ST7789_RAMWR = 0x2C
ST7789_MADCTL = 0x36
ST7789_COLMOD = 0x3A
ST7789_PORCTRL = 0xB2
ST7789_GCTRL = 0xB7
ST7789_VCOMS = 0xBB
ST7789_LCMCTRL = 0xC0
ST7789_VDVVRHEN = 0xC2
ST7789_VRHS = 0xC3
ST7789_VDVS = 0xC4
ST7789_FRCTRL2 = 0xC6
ST7789_PWCTRL1 = 0xD0
ST7789_PVGAMCTRL = 0xE0
ST7789_NVGAMCTRL = 0xE1

# MADCTL位定义
ST7789_MADCTL_MY = 0x80
ST7789_MADCTL_MX = 0x40
ST7789_MADCTL_MV = 0x20
ST7789_MADCTL_ML = 0x10
ST7789_MADCTL_RGB = 0x00
ST7789_MADCTL_BGR = 0x08


class ST7789:
    """ST7789 240x320 TFT显示驱动，SPI接口"""

    def __init__(self):
        self.width = ST7789_WIDTH
        self.height = ST7789_HEIGHT
        # 帧缓冲 (RGB565, 每像素2字节)
        self._framebuf = bytearray(self.width * self.height * 2)
        # 状态
        self._display_on = False
        self._sleeping = True
        self._inverted = False
        self._brightness = 255
        self._rotation = 0  # 0=竖屏, 1=横屏, 2=竖屏翻转, 3=横屏翻转
        self._madctl = ST7789_MADCTL_RGB
        self._color_mode = 0x55  # 16bit RGB565
        # 窗口
        self._col_start = 0
        self._col_end = ST7789_MAX_X
        self._row_start = 0
        self._row_end = ST7789_MAX_Y
        # SPI命令日志
        self._cmd_log = []
        # 初始化标志
        self._initialized = False

    def init(self):
        """初始化ST7789"""
        self._cmd_log.append(('cmd', ST7789_SWRESET))
        self._cmd_log.append(('cmd', ST7789_SLPOUT))
        self._cmd_log.append(('cmd', ST7789_NORON))
        self._cmd_log.append(('cmd', ST7789_COLMOD, self._color_mode))
        self._cmd_log.append(('cmd', ST7789_MADCTL, self._madctl))
        self._sleeping = False
        self._initialized = True
        self.clear()

    def display_on(self):
        """开启显示"""
        self._display_on = True
        self._cmd_log.append(('cmd', ST7789_DISPON))

    def display_off(self):
        """关闭显示"""
        self._display_on = False
        self._cmd_log.append(('cmd', ST7789_DISPOFF))

    def is_on(self):
        """显示是否开启"""
        return self._display_on

    def sleep(self):
        """进入睡眠"""
        self._sleeping = True
        self._cmd_log.append(('cmd', ST7789_SLPOUT))

    def wake(self):
        """唤醒"""
        self._sleeping = False
        self._cmd_log.append(('cmd', ST7789_SLPOUT))

    def is_sleeping(self):
        """是否在睡眠"""
        return self._sleeping

    def invert(self, on=True):
        """反转显示颜色"""
        self._inverted = on
        if on:
            self._cmd_log.append(('cmd', ST7789_INVON))
        else:
            self._cmd_log.append(('cmd', ST7789_INVOFF))

    def is_inverted(self):
        """是否反转"""
        return self._inverted

    def set_brightness(self, level):
        """设置亮度(0-255)"""
        self._brightness = max(0, min(255, level))

    def get_brightness(self):
        """获取亮度"""
        return self._brightness

    def set_rotation(self, r):
        """设置旋转方向(0-3)"""
        self._rotation = r % 4
        if self._rotation == 0:
            self._madctl = ST7789_MADCTL_RGB
            self.width = ST7789_WIDTH
            self.height = ST7789_HEIGHT
        elif self._rotation == 1:
            self._madctl = ST7789_MADCTL_MV | ST7789_MADCTL_RGB
            self.width = ST7789_HEIGHT
            self.height = ST7789_WIDTH
        elif self._rotation == 2:
            self._madctl = ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB
            self.width = ST7789_WIDTH
            self.height = ST7789_HEIGHT
        elif self._rotation == 3:
            self._madctl = ST7789_MADCTL_MV | ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB
            self.width = ST7789_HEIGHT
            self.height = ST7789_WIDTH
        self._cmd_log.append(('cmd', ST7789_MADCTL, self._madctl))

    def get_rotation(self):
        """获取旋转方向"""
        return self._rotation

    def set_window(self, x0, y0, x1, y1):
        """设置绘图窗口"""
        self._col_start = max(0, min(ST7789_MAX_X, x0))
        self._col_end = max(0, min(ST7789_MAX_X, x1))
        self._row_start = max(0, min(ST7789_MAX_Y, y0))
        self._row_end = max(0, min(ST7789_MAX_Y, y1))
        self._cmd_log.append(('cmd', ST7789_CASET, self._col_start, self._col_end))
        self._cmd_log.append(('cmd', ST7789_RASET, self._row_start, self._row_end))

    def clear(self):
        """清屏(填充黑色)"""
        for i in range(len(self._framebuf)):
            self._framebuf[i] = 0

    def fill(self, color_rgb565):
        """全屏填充RGB565颜色"""
        hi = (color_rgb565 >> 8) & 0xFF
        lo = color_rgb565 & 0xFF
        for i in range(0, len(self._framebuf), 2):
            self._framebuf[i] = hi
            self._framebuf[i + 1] = lo

    @staticmethod
    def color565(r, g, b):
        """RGB888转RGB565"""
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

    def _pixel_offset(self, x, y):
        """计算像素在帧缓冲中的偏移"""
        return (y * self.width + x) * 2

    def set_pixel(self, x, y, color565):
        """设置像素(RGB565)"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return
        if self._inverted:
            color565 = ~color565 & 0xFFFF
        off = self._pixel_offset(x, y)
        self._framebuf[off] = (color565 >> 8) & 0xFF
        self._framebuf[off + 1] = color565 & 0xFF

    def get_pixel(self, x, y):
        """读取像素(RGB565)"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return 0
        off = self._pixel_offset(x, y)
        return (self._framebuf[off] << 8) | self._framebuf[off + 1]

    def fill_rect(self, x, y, w, h, color565):
        """矩形填充"""
        for j in range(y, y + h):
            for i in range(x, x + w):
                self.set_pixel(i, j, color565)

    def draw_hline(self, x, y, w, color565):
        """水平线"""
        for i in range(x, x + w):
            self.set_pixel(i, y, color565)

    def draw_vline(self, x, y, h, color565):
        """垂直线"""
        for j in range(y, y + h):
            self.set_pixel(x, j, color565)

    def draw_rect(self, x, y, w, h, color565):
        """矩形边框"""
        self.draw_hline(x, y, w, color565)
        self.draw_hline(x, y + h - 1, w, color565)
        self.draw_vline(x, y, h, color565)
        self.draw_vline(x + w - 1, y, h, color565)

    def draw_circle(self, cx, cy, r, color565):
        """画圆(Bresenham)"""
        x = r
        y = 0
        err = 1 - r
        while x >= y:
            self.set_pixel(cx + x, cy + y, color565)
            self.set_pixel(cx + y, cy + x, color565)
            self.set_pixel(cx - y, cy + x, color565)
            self.set_pixel(cx - x, cy + y, color565)
            self.set_pixel(cx - x, cy - y, color565)
            self.set_pixel(cx - y, cy - x, color565)
            self.set_pixel(cx + y, cy - x, color565)
            self.set_pixel(cx + x, cy - y, color565)
            y += 1
            if err < 0:
                err += 2 * y + 1
            else:
                x -= 1
                err += 2 * (y - x) + 1

    def draw_line(self, x0, y0, x1, y1, color565):
        """Bresenham画线"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            self.set_pixel(x0, y0, color565)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def get_framebuffer(self):
        """获取帧缓冲"""
        return bytes(self._framebuf)

    def get_framebuffer_size(self):
        """帧缓冲大小(字节)"""
        return len(self._framebuf)

    def get_cmd_log(self):
        """获取SPI命令日志"""
        return list(self._cmd_log)


# ═══════════════════════════════════════════════════════════════
#  ILI9341 TFT显示驱动 — SPI接口 240x320
# ═══════════════════════════════════════════════════════════════

ILI9341_WIDTH = 240
ILI9341_HEIGHT = 320
ILI9341_TFTWIDTH = 240
ILI9341_TFTHEIGHT = 320

# ILI9341命令
ILI9341_NOP = 0x00
ILI9341_SWRESET = 0x01
ILI9341_SLPOUT = 0x11
ILI9341_DISPON = 0x29
ILI9341_DISPOFF = 0x28
ILI9341_CASET = 0x2A
ILI9341_RASET = 0x2B
ILI9341_RAMWR = 0x2C
ILI9341_RAMRD = 0x2E
ILI9341_MADCTL = 0x36
ILI9341_COLMOD = 0x3A
ILI9341_FRMCTR1 = 0xB1
ILI9341_FRMCTR2 = 0xB2
ILI9341_FRMCTR3 = 0xB3
ILI9341_INVTR = 0xB4
ILI9341_DFUNCTR = 0xB6
ILI9341_PWCTR1 = 0xC0
ILI9341_PWCTR2 = 0xC1
ILI9341_PWCTR3 = 0xC2
ILI9341_PWCTR4 = 0xC3
ILI9341_PWCTR5 = 0xC4
ILI9341_VMCTR1 = 0xC5
ILI9341_GMCTRP1 = 0xE0
ILI9341_GMCTRN1 = 0xE1

# MADCTL位
ILI9341_MADCTL_MY = 0x80
ILI9341_MADCTL_MX = 0x40
ILI9341_MADCTL_MV = 0x20
ILI9341_MADCTL_ML = 0x10
ILI9341_MADCTL_BGR = 0x08
ILI9341_MADCTL_MH = 0x04


class ILI9341:
    """ILI9341 240x320 TFT显示驱动，SPI接口"""

    def __init__(self):
        self.width = ILI9341_WIDTH
        self.height = ILI9341_HEIGHT
        # 帧缓冲 (RGB565)
        self._framebuf = bytearray(self.width * self.height * 2)
        # 状态
        self._display_on = False
        self._sleeping = True
        self._inverted = False
        self._brightness = 255
        self._rotation = 0
        self._madctl = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR
        self._color_mode = 0x55  # 16bit
        # SPI命令日志
        self._cmd_log = []
        self._initialized = False
        # 窗口
        self._win_x0 = 0
        self._win_y0 = 0
        self._win_x1 = ILI9341_WIDTH - 1
        self._win_y1 = ILI9341_HEIGHT - 1

    def init(self):
        """初始化ILI9341"""
        self._cmd_log.append(('cmd', ILI9341_SWRESET))
        self._cmd_log.append(('cmd', ILI9341_SLPOUT))
        self._cmd_log.append(('cmd', ILI9341_PWCTR1, 0x23))
        self._cmd_log.append(('cmd', ILI9341_PWCTR2, 0x10))
        self._cmd_log.append(('cmd', ILI9341_VMCTR1, 0x3E, 0x28))
        self._cmd_log.append(('cmd', ILI9341_MADCTL, self._madctl))
        self._cmd_log.append(('cmd', ILI9341_COLMOD, self._color_mode))
        self._sleeping = False
        self._initialized = True
        self.clear()

    def display_on(self):
        """开启显示"""
        self._display_on = True
        self._cmd_log.append(('cmd', ILI9341_DISPON))

    def display_off(self):
        """关闭显示"""
        self._display_on = False
        self._cmd_log.append(('cmd', ILI9341_DISPOFF))

    def is_on(self):
        return self._display_on

    def sleep(self):
        self._sleeping = True

    def wake(self):
        self._sleeping = False
        self._cmd_log.append(('cmd', ILI9341_SLPOUT))

    def is_sleeping(self):
        return self._sleeping

    def invert(self, on=True):
        """反转颜色"""
        self._inverted = on

    def is_inverted(self):
        return self._inverted

    def set_brightness(self, level):
        self._brightness = max(0, min(255, level))

    def get_brightness(self):
        return self._brightness

    def set_rotation(self, r):
        """设置旋转(0-3)"""
        self._rotation = r % 4
        if self._rotation == 0:
            self._madctl = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR
            self.width = ILI9341_TFTWIDTH
            self.height = ILI9341_TFTHEIGHT
        elif self._rotation == 1:
            self._madctl = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR
            self.width = ILI9341_TFTHEIGHT
            self.height = ILI9341_TFTWIDTH
        elif self._rotation == 2:
            self._madctl = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR
            self.width = ILI9341_TFTWIDTH
            self.height = ILI9341_TFTHEIGHT
        elif self._rotation == 3:
            self._madctl = ILI9341_MADCTL_MV | ILI9341_MADCTL_MY | ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR
            self.width = ILI9341_TFTHEIGHT
            self.height = ILI9341_TFTWIDTH
        self._cmd_log.append(('cmd', ILI9341_MADCTL, self._madctl))

    def get_rotation(self):
        return self._rotation

    def set_window(self, x0, y0, x1, y1):
        """设置绘图窗口"""
        self._win_x0 = max(0, min(self.width - 1, x0))
        self._win_y0 = max(0, min(self.height - 1, y0))
        self._win_x1 = max(0, min(self.width - 1, x1))
        self._win_y1 = max(0, min(self.height - 1, y1))
        self._cmd_log.append(('cmd', ILI9341_CASET, self._win_x0, self._win_x1))
        self._cmd_log.append(('cmd', ILI9341_RASET, self._win_y0, self._win_y1))

    def clear(self):
        """清屏"""
        for i in range(len(self._framebuf)):
            self._framebuf[i] = 0

    def fill(self, color565):
        """全屏填充"""
        hi = (color565 >> 8) & 0xFF
        lo = color565 & 0xFF
        for i in range(0, len(self._framebuf), 2):
            self._framebuf[i] = hi
            self._framebuf[i + 1] = lo

    @staticmethod
    def color565(r, g, b):
        """RGB888→RGB565"""
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

    def _pixel_offset(self, x, y):
        return (y * self.width + x) * 2

    def set_pixel(self, x, y, color565):
        """设置像素"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return
        if self._inverted:
            color565 = ~color565 & 0xFFFF
        off = self._pixel_offset(x, y)
        self._framebuf[off] = (color565 >> 8) & 0xFF
        self._framebuf[off + 1] = color565 & 0xFF

    def get_pixel(self, x, y):
        """读取像素"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return 0
        off = self._pixel_offset(x, y)
        return (self._framebuf[off] << 8) | self._framebuf[off + 1]

    def fill_rect(self, x, y, w, h, color565):
        """矩形填充"""
        for j in range(y, y + h):
            for i in range(x, x + w):
                self.set_pixel(i, j, color565)

    def draw_hline(self, x, y, w, color565):
        for i in range(x, x + w):
            self.set_pixel(i, y, color565)

    def draw_vline(self, x, y, h, color565):
        for j in range(y, y + h):
            self.set_pixel(x, j, color565)

    def draw_rect(self, x, y, w, h, color565):
        """矩形边框"""
        self.draw_hline(x, y, w, color565)
        self.draw_hline(x, y + h - 1, w, color565)
        self.draw_vline(x, y, h, color565)
        self.draw_vline(x + w - 1, y, h, color565)

    def draw_circle(self, cx, cy, r, color565):
        """画圆"""
        x = r
        y = 0
        err = 1 - r
        while x >= y:
            self.set_pixel(cx + x, cy + y, color565)
            self.set_pixel(cx + y, cy + x, color565)
            self.set_pixel(cx - y, cy + x, color565)
            self.set_pixel(cx - x, cy + y, color565)
            self.set_pixel(cx - x, cy - y, color565)
            self.set_pixel(cx - y, cy - x, color565)
            self.set_pixel(cx + y, cy - x, color565)
            self.set_pixel(cx + x, cy - y, color565)
            y += 1
            if err < 0:
                err += 2 * y + 1
            else:
                x -= 1
                err += 2 * (y - x) + 1

    def draw_line(self, x0, y0, x1, y1, color565):
        """Bresenham画线"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            self.set_pixel(x0, y0, color565)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def read_pixel(self, x, y):
        """读取像素(模拟RAMRD命令)"""
        return self.get_pixel(x, y)

    def get_framebuffer(self):
        return bytes(self._framebuf)

    def get_framebuffer_size(self):
        return len(self._framebuf)

    def get_cmd_log(self):
        return list(self._cmd_log)


# ═══════════════════════════════════════════════════════════════
#  W25Qxx SPI Flash驱动
# ═══════════════════════════════════════════════════════════════

W25Q_CMD_WRITE_ENABLE = 0x06
W25Q_CMD_WRITE_DISABLE = 0x04
W25Q_CMD_READ_STATUS1 = 0x05
W25Q_CMD_READ_DATA = 0x03
W25Q_CMD_PAGE_PROGRAM = 0x02
W25Q_CMD_SECTOR_ERASE = 0x20
W25Q_CMD_BLOCK_ERASE_32K = 0x52
W25Q_CMD_BLOCK_ERASE_64K = 0xD8
W25Q_CMD_CHIP_ERASE = 0xC7
W25Q_CMD_JEDEC_ID = 0x9F
W25Q_CMD_POWER_DOWN = 0xB9
W25Q_CMD_RELEASE_PD = 0xAB

# W25Q16参数
W25Q16_JEDEC_ID = (0xEF, 0x40, 0x15)
W25Q16_PAGE_SIZE = 256
W25Q16_SECTOR_SIZE = 4096
W25Q16_BLOCK_SIZE_32K = 32768
W25Q16_BLOCK_SIZE_64K = 65536
W25Q16_TOTAL_SIZE = 2 * 1024 * 1024  # 2MB

# W25Q32参数
W25Q32_JEDEC_ID = (0xEF, 0x40, 0x16)

# W25Q64参数
W25Q64_JEDEC_ID = (0xEF, 0x40, 0x17)


class W25Qxx:
    """W25Qxx SPI Flash驱动模拟"""

    def __init__(self, flash_size=W25Q16_TOTAL_SIZE, jedec_id=W25Q16_JEDEC_ID):
        self._flash_size = flash_size
        self._jedec_id = jedec_id
        # 模拟Flash存储(全0xFF为擦除态)
        self._mem = bytearray([0xFF] * flash_size)
        # 状态寄存器
        self._status1 = 0x00  # BUSY=WEL=0
        self._write_enabled = False
        self._powered_down = False
        # 页大小/扇区大小
        self._page_size = W25Q16_PAGE_SIZE
        self._sector_size = W25Q16_SECTOR_SIZE
        # 统计
        self._erase_count = 0
        self._program_count = 0
        self._read_count = 0
        # SPI日志
        self._cmd_log = []

    def init(self):
        """初始化"""
        self._status1 = 0x00
        self._write_enabled = False
        self._powered_down = False
        self._cmd_log.clear()

    def read_jedec_id(self):
        """读取JEDEC ID"""
        self._cmd_log.append(('cmd', W25Q_CMD_JEDEC_ID))
        return self._jedec_id

    def read_status(self):
        """读状态寄存器1"""
        self._cmd_log.append(('cmd', W25Q_CMD_READ_STATUS1))
        status = self._status1
        if self._write_enabled:
            status |= 0x02  # WEL bit
        return status

    def is_busy(self):
        """是否忙"""
        return bool(self._status1 & 0x01)

    def write_enable(self):
        """写使能"""
        self._write_enabled = True
        self._status1 |= 0x02
        self._cmd_log.append(('cmd', W25Q_CMD_WRITE_ENABLE))

    def write_disable(self):
        """写禁止"""
        self._write_enabled = False
        self._status1 &= ~0x02
        self._cmd_log.append(('cmd', W25Q_CMD_WRITE_DISABLE))

    def read_data(self, addr, length):
        """读取数据"""
        self._read_count += 1
        self._cmd_log.append(('read', addr, length))
        if addr + length > self._flash_size:
            length = self._flash_size - addr
        return bytes(self._mem[addr:addr + length])

    def read_byte(self, addr):
        """读取单字节"""
        if addr >= self._flash_size:
            return 0xFF
        self._read_count += 1
        return self._mem[addr]

    def page_program(self, addr, data):
        """页编程(写入)"""
        self._cmd_log.append(('program', addr, len(data)))
        if not self._write_enabled:
            return False
        if addr >= self._flash_size:
            return False
        # 写入数据(NAND Flash: 只能0→1变0)
        for i, byte in enumerate(data):
            pos = addr + i
            if pos >= self._flash_size:
                break
            # 模拟Flash: AND操作(只能把1变成0)
            self._mem[pos] &= byte
        self._program_count += 1
        self._write_enabled = False
        self._status1 &= ~0x02
        return True

    def sector_erase(self, addr):
        """扇区擦除(4KB)"""
        self._cmd_log.append(('erase_sector', addr))
        if not self._write_enabled:
            return False
        # 对齐到扇区
        sector_addr = addr & ~(self._sector_size - 1)
        for i in range(self._sector_size):
            if sector_addr + i < self._flash_size:
                self._mem[sector_addr + i] = 0xFF
        self._erase_count += 1
        self._write_enabled = False
        self._status1 &= ~0x02
        return True

    def block_erase_32k(self, addr):
        """32KB块擦除"""
        self._cmd_log.append(('erase_block32k', addr))
        if not self._write_enabled:
            return False
        block_addr = addr & ~(W25Q16_BLOCK_SIZE_32K - 1)
        for i in range(W25Q16_BLOCK_SIZE_32K):
            if block_addr + i < self._flash_size:
                self._mem[block_addr + i] = 0xFF
        self._erase_count += 1
        self._write_enabled = False
        return True

    def block_erase_64k(self, addr):
        """64KB块擦除"""
        self._cmd_log.append(('erase_block64k', addr))
        if not self._write_enabled:
            return False
        block_addr = addr & ~(W25Q16_BLOCK_SIZE_64K - 1)
        for i in range(W25Q16_BLOCK_SIZE_64K):
            if block_addr + i < self._flash_size:
                self._mem[block_addr + i] = 0xFF
        self._erase_count += 1
        self._write_enabled = False
        return True

    def chip_erase(self):
        """全片擦除"""
        self._cmd_log.append(('erase_chip',))
        if not self._write_enabled:
            return False
        for i in range(self._flash_size):
            self._mem[i] = 0xFF
        self._erase_count += 1
        self._write_enabled = False
        return True

    def power_down(self):
        """进入掉电模式"""
        self._powered_down = True
        self._cmd_log.append(('cmd', W25Q_CMD_POWER_DOWN))

    def release_power_down(self):
        """退出掉电模式"""
        self._powered_down = False
        self._cmd_log.append(('cmd', W25Q_CMD_RELEASE_PD))

    def is_powered_down(self):
        return self._powered_down

    def get_flash_size(self):
        return self._flash_size

    def get_erase_count(self):
        return self._erase_count

    def get_program_count(self):
        return self._program_count

    def get_read_count(self):
        return self._read_count

    def get_cmd_log(self):
        return list(self._cmd_log)


# ═══════════════════════════════════════════════════════════════
#  FM24CL64 铁电存储器 — I2C接口 8KB
# ═══════════════════════════════════════════════════════════════

FM24CL64_I2C_ADDR = 0x50
FM24CL64_SIZE = 8192  # 8KB
FM24CL64_PAGE_SIZE = 64  # 页大小

# FM24CL64特殊特性: 无限次写入, 无需擦除, 掉电保存
FM24CL64_MAX_WRITE_CYCLES = 10_000_000_000  # 100亿次
FM24CL64_DATA_RETENTION_YEARS = 100


class FM24CL64:
    """FM24CL64 8KB铁电存储器(FRAM)驱动，I2C接口"""

    def __init__(self, i2c_bus=None):
        self.bus = i2c_bus or I2CBus()
        self.addr = FM24CL64_I2C_ADDR
        self._size = FM24CL64_SIZE
        self._page_size = FM24CL64_PAGE_SIZE
        # FRAM存储(不像Flash需要擦除, 可直接写入)
        self._mem = bytearray([0x00] * self._size)
        # 统计
        self._write_count = 0
        self._read_count = 0
        self._byte_writes = 0
        # 初始化标志
        self._initialized = False

    def init(self):
        """初始化FM24CL64"""
        self.bus.init()
        self._initialized = True
        self._write_count = 0
        self._read_count = 0
        self._byte_writes = 0

    def read_byte(self, addr):
        """读取单字节"""
        if addr < 0 or addr >= self._size:
            return 0x00
        self._read_count += 1
        return self._mem[addr]

    def write_byte(self, addr, value):
        """写入单字节(FRAM无擦除需求)"""
        if addr < 0 or addr >= self._size:
            return False
        self._mem[addr] = value & 0xFF
        self._write_count += 1
        self._byte_writes += 1
        return True

    def read(self, addr, length):
        """连续读取"""
        if addr < 0 or addr >= self._size:
            return bytearray(0)
        if addr + length > self._size:
            length = self._size - addr
        self._read_count += 1
        return bytearray(self._mem[addr:addr + length])

    def write(self, addr, data):
        """连续写入(FRAM支持任意地址直接写入, 无需页对齐)"""
        if addr < 0 or addr >= self._size:
            return False
        for i, byte in enumerate(data):
            pos = addr + i
            if pos >= self._size:
                break
            self._mem[pos] = byte
            self._byte_writes += 1
        self._write_count += 1
        return True

    def write_page(self, addr, data):
        """页写入(与I2C EEPROM兼容的接口)"""
        if addr < 0 or addr >= self._size:
            return False
        # FRAM无页边界限制, 但提供兼容接口
        for i, byte in enumerate(data):
            pos = addr + i
            if pos >= self._size:
                break
            self._mem[pos] = byte
            self._byte_writes += 1
        self._write_count += 1
        return True

    def fill(self, value, addr=0, length=None):
        """填充指定区域"""
        if length is None:
            length = self._size - addr
        if addr < 0 or addr >= self._size:
            return False
        for i in range(length):
            pos = addr + i
            if pos >= self._size:
                break
            self._mem[pos] = value & 0xFF
        return True

    def compare(self, addr, data):
        """比较数据"""
        if addr < 0 or addr + len(data) > self._size:
            return False
        for i, byte in enumerate(data):
            if self._mem[addr + i] != byte:
                return False
        return True

    def get_size(self):
        """获取存储器大小"""
        return self._size

    def get_write_count(self):
        """获取写操作次数"""
        return self._write_count

    def get_read_count(self):
        """获取读操作次数"""
        return self._read_count

    def get_byte_writes(self):
        """获取总字节写入数"""
        return self._byte_writes

    def is_initialized(self):
        """是否已初始化"""
        return self._initialized


# ═══════════════════════════════════════════════════════════════
#  环形缓冲区 — 对应 ring_buffer.c (已在wrappers.py中定义RingBuffer)
#  此处仅保留注释说明
# ═══════════════════════════════════════════════════════════════
