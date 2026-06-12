# Orange Pi 5 控制代码库

> 面向全国大学生电子设计竞赛的 Orange Pi 5 (RK3588S) 控制代码 Python 库

## 📁 目录结构

```
02_通用代码库_OrangePi5/
├── control/                    # 控制模块
│   ├── __init__.py
│   ├── pid_controller.py       # PID 控制器
│   ├── motor_controller.py     # 电机控制 (L298N/TB6612FNG)
│   ├── servo_controller.py     # 舵机控制
│   ├── encoder_reader.py       # 编码器读取
│   ├── line_follower.py        # 视觉循线控制器
│   ├── ball_balancer.py        # 滚球平衡控制器
│   ├── inverted_pendulum.py    # 倒立摆控制器
│   └── obstacle_avoider.py     # 避障控制器
├── utils/                      # 工具模块
│   ├── __init__.py
│   ├── gpio_utils.py           # GPIO 操作封装
│   ├── pwm_utils.py            # PWM 操作封装
│   ├── serial_comm.py          # 串口通信协议
│   ├── timer_utils.py          # 高精度定时器
│   └── logging_utils.py        # 日志工具
├── visual_control_bridge.py    # 视觉-控制融合桥接
└── README.md                   # 本文件
```

## 🔧 环境要求

| 依赖 | 用途 | 安装 |
|------|------|------|
| `lgpio` | GPIO 操作 (推荐) | `pip install lgpio` |
| `wiringpi` | GPIO 操作 (备选) | `pip install wiringpi` |
| `pyserial` | 串口通信 | `pip install pyserial` |

## 📖 快速开始

### 1. GPIO 管理器

```python
from utils.gpio_utils import GPIOManager

gpio = GPIOManager()  # 自动选择后端 (lgpio > wiringpi > sysfs)
gpio.setup(18, GPIOManager.OUT)
gpio.output(18, GPIOManager.HIGH)
```

### 2. PID 控制器

```python
from control.pid_controller import PIDController, PIDMode

# 位置式 PID
pid = PIDController(kp=1.0, ki=0.1, kd=0.05, mode=PIDMode.POSITION)

# 增量式 PID (适合电机控制)
pid = PIDController(kp=2.0, ki=0.5, kd=0.1, mode=PIDMode.INCREMENTAL,
                    output_min=-100, output_max=100)

# 计算输出
error = target_speed - actual_speed
output = pid.compute(error)
```

### 3. 电机控制

```python
from utils.gpio_utils import GPIOManager
from utils.pwm_utils import PWMManager
from control.motor_controller import MotorController, DualMotorController

gpio = GPIOManager()
pwm = PWMManager(gpio)

# L298N 双路电机
motor_l = MotorController(gpio, pwm, in1_pin=17, in2_pin=27, pwm_pin=18, pwm_channel=0)
motor_r = MotorController(gpio, pwm, in1_pin=22, in2_pin=23, pwm_pin=24, pwm_channel=1)
car = DualMotorController(motor_l, motor_r)

# 差速驱动
car.drive(linear=50, angular=0)    # 直行
car.drive(linear=0, angular=30)    # 左转
car.stop()
```

### 4. 舵机控制

```python
from utils.gpio_utils import GPIOManager
from control.servo_controller import ServoController

gpio = GPIOManager()
servo = ServoController(gpio, pin=12)

servo.set_angle(90)     # 居中
servo.set_angle(0)      # 最左
servo.set_angle(180)    # 最右
```

### 5. 编码器读取

```python
from utils.gpio_utils import GPIOManager
from control.encoder_reader import EncoderReader

gpio = GPIOManager()
encoder = EncoderReader(gpio, pin_a=5, pin_b=6, ppr=360)

count = encoder.get_count()     # 脉冲计数
speed = encoder.get_speed(0.1)  # RPM
angle = encoder.get_angle()     # 角度
dist = encoder.get_distance(0.065)  # 行驶距离 (米)
```

### 6. 串口通信 (与 STM32)

```python
from utils.serial_comm import SerialProtocol

# 自定义协议: HEAD=0xAA, TAIL=0x55
comm = SerialProtocol(port='/dev/ttyS1', baudrate=115200)
comm.open()

# 发送数据
comm.send(cmd=0x01, data=struct.pack('<ff', speed, angle))

# 注册回调
def on_sensor(cmd, data):
    values = struct.unpack('<fff', data)  # 解析传感器数据
    print(f"传感器: {values}")

comm.register_handler(0x02, on_sensor)
comm.start_receiving()
```

### 7. 视觉-控制融合

```python
from visual_control_bridge import VisualControlBridge, VisualTarget

bridge = VisualControlBridge(image_width=640, image_height=480)

# 从视觉系统获取目标
target = VisualTarget(
    class_id=0, class_name="ball",
    cx=320, cy=240, width=50, height=50,
    confidence=0.95
)

# 跟踪模式
cmd = bridge.track_target(target)
# cmd.linear_x, cmd.angular_z → 电机
# cmd.servo_pan, cmd.servo_tilt → 舵机

# 循线模式
cmd = bridge.follow_line(target)

# 避障模式
cmd = bridge.avoid_obstacle([target])
```

## 🔌 Orange Pi 5 GPIO 引脚对照

| 功能 | WiringPi | BCM | 物理引脚 |
|------|----------|-----|---------|
| GPIO0 | 0 | 150 | 11 |
| GPIO1 | 1 | 152 | 12 |
| GPIO2 | 2 | 155 | 13 |
| GPIO3 | 3 | 154 | 15 |
| GPIO4 | 4 | 159 | 16 |
| GPIO5 | 5 | 156 | 18 |
| GPIO6 | 6 | 157 | 22 |
| PWM1 | 14 | 146 | 33 |

> ⚠️ 具体引脚编号请参考 Orange Pi 5 官方文档，以实物为准。

## ⚡ 常见接线方案

### L298N + 双电机

```
Orange Pi 5          L298N           电机
─────────────────────────────────────────
GPIO17 (IN1)  ──→  IN1 ──→  左电机 A
GPIO27 (IN2)  ──→  IN2 ──→  左电机 B
PWM18         ──→  ENA ──→  (PWM 调速)

GPIO22 (IN3)  ──→  IN3 ──→  右电机 A
GPIO23 (IN4)  ──→  IN4 ──→  右电机 B
PWM24         ──→  ENB ──→  (PWM 调速)

GND           ──→  GND
12V           ──→  12V 电源
```

### 编码器接线

```
编码器 A相 ──→ GPIO5 (上拉)
编码器 B相 ──→ GPIO6 (上拉)
编码器 VCC ──→ 3.3V
编码器 GND ──→ GND
```

## 📝 注意事项

1. **权限**: GPIO 操作需要 root 权限，使用 `sudo` 运行或配置 udev 规则
2. **引脚编号**: 代码使用 BCM 编号，WiringPi 后端会自动转换
3. **软件 PWM 精度**: 受系统调度影响，舵机控制建议使用硬件 PWM
4. **编码器**: 高速编码器 (>10kHz) 建议使用硬件计数器 (如 STM32)
5. **串口**: Orange Pi 5 UART 默认设备为 `/dev/ttyS1`~`/dev/ttyS8`

## 🎯 高级控制模块

### 8. 视觉循线控制器

```python
from control.line_follower import LineFollower, LineType

# 创建循线控制器
follower = LineFollower(
    camera_id=0,
    resolution=(640, 480),
    line_type=LineType.BLACK_LINE,
    pid_params={'kp': 0.8, 'ki': 0.01, 'kd': 0.3},
    base_speed=50.0
)

# 运行（带调试窗口）
follower.run(show_debug=True)

# 动态调整参数
follower.set_parameters(base_speed=60, kp=1.0)
```

### 9. 滚球平衡控制器

```python
from control.ball_balancer import BallBalancer

# 创建滚球平衡控制器
balancer = BallBalancer(
    camera_id=0,
    resolution=(640, 480),
    target_position=(0.5, 0.5),
    pid_x_params={'kp': 0.5, 'ki': 0.01, 'kd': 0.2},
    pid_y_params={'kp': 0.5, 'ki': 0.01, 'kd': 0.2},
    ball_color_lower=(0, 100, 100),  # 红色球
    ball_color_upper=(10, 255, 255)
)

# 运行
balancer.run(show_debug=True)

# 使用卡尔曼滤波版本
from control.ball_balancer import BallBalancerWithKalman
balancer_kf = BallBalancerWithKalman(
    camera_id=0,
    resolution=(640, 480)
)
```

### 10. 倒立摆控制器

```python
from control.inverted_pendulum import InvertedPendulum
import numpy as np

# 创建倒立摆控制器（LQR控制）
pendulum = InvertedPendulum(
    camera_id=0,
    resolution=(640, 480),
    control_frequency=100.0,
    angle_threshold=np.radians(30),
    lqr_params={
        'dt': 0.01,
        'm': 0.1,  # 摆杆质量 (kg)
        'l': 0.5,  # 摆杆长度 (m)
        'g': 9.81
    }
)

# 运行
pendulum.run(show_debug=True)

# 或使用PID版本
from control.inverted_pendulum import InvertedPendulumWithPID
pendulum_pid = InvertedPendulumWithPID(
    pid_params={'kp': 2.0, 'ki': 0.01, 'kd': 0.5},
    camera_id=0,
    resolution=(640, 480)
)
```

### 11. 避障控制器

```python
from control.obstacle_avoider import ObstacleAvoider

# 创建避障控制器
avoider = ObstacleAvoider(
    camera_id=0,
    resolution=(640, 480),
    ultrasonic_pins={'trigger': 23, 'echo': 24},
    detection_method='contour',
    safe_distance=0.5,
    warning_distance=1.0,
    base_speed=50.0
)

# 运行
avoider.run(show_debug=True)

# 多传感器版本
from control.obstacle_avoider import MultiSensorObstacleAvoider
multi_avoider = MultiSensorObstacleAvoider(
    ultrasonic_configs=[
        {'trigger': 23, 'echo': 24, 'name': 'left'},
        {'trigger': 25, 'echo': 8, 'name': 'center'},
        {'trigger': 7, 'echo': 1, 'name': 'right'}
    ],
    camera_id=0,
    resolution=(640, 480)
)
```

### 12. 高精度定时器

```python
from utils.timer_utils import HighPrecisionTimer, RateController, Stopwatch
from utils.timer_utils import TimerMode, PeriodicExecutor

# 高精度定时器
def on_timer(timestamp):
    print(f"定时器触发: {timestamp:.3f}s")

timer = HighPrecisionTimer(
    callback=on_timer,
    interval=0.01,  # 10ms
    mode=TimerMode.PERIODIC
)
timer.start()

# 速率控制器
rate_ctrl = RateController(target_rate=100.0)  # 100Hz
rate_ctrl.start()

for i in range(1000):
    # 执行控制逻辑
    rate_ctrl.sleep()  # 自动控制频率

# 秒表
with Stopwatch("代码块计时") as sw:
    # 执行代码
    pass
print(f"执行时间: {sw.elapsed():.6f}s")

# 周期执行器
def control_loop():
    # 控制逻辑
    pass

executor = PeriodicExecutor(func=control_loop, frequency=100.0)
executor.start()
```

### 13. 日志工具

```python
from utils.logging_utils import create_logger, create_debug_logger
from utils.logging_utils import LogLevel, PerformanceMonitor

# 创建系统日志器
logger = create_logger(
    name="MySystem",
    log_dir="logs",
    console_level=LogLevel.INFO,
    file_level=LogLevel.DEBUG
)

logger.info("系统启动")
logger.warning("警告信息")
logger.error("错误信息")

# 性能监控
logger.log_performance("control_loop", 15.5)  # 15.5ms
print(logger.get_performance_stats())

# 调试日志器
debug_logger = create_debug_logger("Debug", log_dir="logs/debug")

debug_logger.enter_function("my_function", args=(1, 2))
debug_logger.log_variable("x", 42)
debug_logger.log_state("robot_state", {"speed": 50, "angle": 30})
debug_logger.exit_function("my_function", return_value=42)

# 性能监控器
monitor = PerformanceMonitor(logger)
start = monitor.start_timer("operation")
# 执行操作
monitor.stop_timer("operation", start)
monitor.print_summary()

# 使用装饰器
from utils.logging_utils import log_function_call, log_performance

@log_function_call(debug_logger)
@log_performance(logger)
def my_control_function(target, current):
    error = target - current
    return error * 0.5
```

## 📊 模块依赖关系

```
visual_control_bridge.py
    ├── control/pid_controller.py
    ├── control/motor_controller.py
    └── control/servo_controller.py

control/line_follower.py
    ├── control/pid_controller.py
    └── control/motor_controller.py

control/ball_balancer.py
    └── control/pid_controller.py

control/inverted_pendulum.py
    └── control/pid_controller.py

control/obstacle_avoider.py
    └── control/pid_controller.py

utils/timer_utils.py (独立模块)

utils/logging_utils.py (独立模块)
```
