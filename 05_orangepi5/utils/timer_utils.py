#!/usr/bin/env python3
"""
高精度定时器工具
用于控制周期和时间测量
适用于Orange Pi 5实时控制系统
"""
import time
import threading
from typing import Callable, Optional, Dict, List
from enum import Enum
import functools


class TimerMode(Enum):
    """定时器模式"""
    ONE_SHOT = 0      # 单次触发
    PERIODIC = 1      # 周期性触发
    RATE = 2          # 速率控制


class HighPrecisionTimer:
    """
    高精度定时器
    使用系统高精度时钟实现精确定时
    """
    
    def __init__(self, 
                 callback: Callable = None,
                 interval: float = 0.01,
                 mode: TimerMode = TimerMode.PERIODIC,
                 name: str = "HighPrecisionTimer"):
        """
        初始化高精度定时器
        
        Args:
            callback: 回调函数
            interval: 定时间隔 (秒)
            mode: 定时器模式
            name: 定时器名称
        """
        self.callback = callback
        self.interval = interval
        self.mode = mode
        self.name = name
        
        # 状态变量
        self.is_running = False
        self.thread = None
        self.start_time = 0.0
        self.last_tick_time = 0.0
        self.tick_count = 0
        
        # 时间统计
        self.min_interval = float('inf')
        self.max_interval = 0.0
        self.avg_interval = 0.0
        self.jitter = 0.0
        
        # 同步控制
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        
    def start(self):
        """启动定时器"""
        if self.is_running:
            return
        
        self.is_running = True
        self._stop_event.clear()
        self.start_time = self._get_high_precision_time()
        self.last_tick_time = self.start_time
        
        if self.mode == TimerMode.PERIODIC or self.mode == TimerMode.RATE:
            self.thread = threading.Thread(target=self._periodic_loop, daemon=True)
            self.thread.start()
        elif self.mode == TimerMode.ONE_SHOT:
            self.thread = threading.Thread(target=self._one_shot_loop, daemon=True)
            self.thread.start()
    
    def stop(self):
        """停止定时器"""
        self.is_running = False
        self._stop_event.set()
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
    
    def _get_high_precision_time(self) -> float:
        """
        获取高精度时间
        
        Returns:
            高精度时间戳 (秒)
        """
        # 使用time.perf_counter()获得最高精度
        return time.perf_counter()
    
    def _periodic_loop(self):
        """周期性定时器循环"""
        next_tick_time = self.start_time + self.interval
        
        while self.is_running and not self._stop_event.is_set():
            # 高精度等待
            self._high_precision_wait(next_tick_time)
            
            if not self.is_running:
                break
            
            # 执行回调
            current_time = self._get_high_precision_time()
            self._execute_callback(current_time)
            
            # 更新统计
            self._update_statistics(current_time, next_tick_time)
            
            # 计算下一次触发时间
            next_tick_time += self.interval
            
            # 防止时间漂移
            if next_tick_time < current_time:
                next_tick_time = current_time + self.interval
    
    def _one_shot_loop(self):
        """单次定时器循环"""
        target_time = self.start_time + self.interval
        
        # 等待指定时间
        self._high_precision_wait(target_time)
        
        if self.is_running:
            current_time = self._get_high_precision_time()
            self._execute_callback(current_time)
            self._update_statistics(current_time, target_time)
        
        self.is_running = False
    
    def _high_precision_wait(self, target_time: float):
        """
        高精度等待
        
        Args:
            target_time: 目标时间
        """
        # 计算剩余等待时间
        current_time = self._get_high_precision_time()
        remaining = target_time - current_time
        
        if remaining <= 0:
            return
        
        # 对于较长的等待时间，使用sleep
        if remaining > 0.001:  # 1ms以上
            # 使用sleep等待大部分时间
            sleep_time = remaining - 0.0005  # 留出500微秒的余量
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # 忙等待剩余时间
        while self._get_high_precision_time() < target_time:
            if self._stop_event.is_set():
                return
    
    def _execute_callback(self, current_time: float):
        """
        执行回调函数
        
        Args:
            current_time: 当前时间
        """
        if self.callback is None:
            return
        
        try:
            # 计算时间戳和间隔
            timestamp = current_time - self.start_time
            interval = current_time - self.last_tick_time
            
            # 执行回调
            if self.mode == TimerMode.RATE:
                # 速率模式：传递实际间隔
                self.callback(timestamp, interval)
            else:
                # 其他模式：只传递时间戳
                self.callback(timestamp)
            
            self.last_tick_time = current_time
            self.tick_count += 1
            
        except Exception as e:
            print(f"定时器回调执行失败: {e}")
    
    def _update_statistics(self, actual_time: float, expected_time: float):
        """
        更新时间统计
        
        Args:
            actual_time: 实际时间
            expected_time: 期望时间
        """
        interval = actual_time - self.last_tick_time if self.last_tick_time > 0 else 0
        jitter = abs(actual_time - expected_time)
        
        # 更新统计
        if interval > 0:
            self.min_interval = min(self.min_interval, interval)
            self.max_interval = max(self.max_interval, interval)
            
            # 指数移动平均
            alpha = 0.1
            self.avg_interval = alpha * interval + (1 - alpha) * self.avg_interval
        
        self.jitter = jitter
    
    def wait(self, timeout: float = None) -> bool:
        """
        等待定时器完成
        
        Args:
            timeout: 超时时间 (秒)
            
        Returns:
            是否正常完成
        """
        if self.thread is None:
            return True
        
        self.thread.join(timeout=timeout)
        return not self.thread.is_alive()
    
    def reset_statistics(self):
        """重置统计信息"""
        self.min_interval = float('inf')
        self.max_interval = 0.0
        self.avg_interval = 0.0
        self.jitter = 0.0
        self.tick_count = 0
    
    def get_statistics(self) -> Dict:
        """
        获取统计信息
        
        Returns:
            统计信息字典
        """
        return {
            'name': self.name,
            'is_running': self.is_running,
            'tick_count': self.tick_count,
            'interval': self.interval,
            'min_interval': self.min_interval,
            'max_interval': self.max_interval,
            'avg_interval': self.avg_interval,
            'jitter': self.jitter,
            'uptime': self._get_high_precision_time() - self.start_time if self.start_time > 0 else 0
        }


class RateController:
    """
    速率控制器
    用于控制代码执行频率
    """
    
    def __init__(self, target_rate: float = 100.0, name: str = "RateController"):
        """
        初始化速率控制器
        
        Args:
            target_rate: 目标速率 (Hz)
            name: 控制器名称
        """
        self.target_rate = target_rate
        self.target_interval = 1.0 / target_rate
        self.name = name
        
        # 时间变量
        self.last_time = 0.0
        self.start_time = 0.0
        
        # 统计变量
        self.call_count = 0
        self.actual_rate = 0.0
        self.rate_history = []
        self.max_history = 100
        
    def start(self):
        """开始速率控制"""
        self.start_time = time.perf_counter()
        self.last_time = self.start_time
        self.call_count = 0
    
    def sleep(self):
        """
        睡眠到下一个周期
        """
        current_time = time.perf_counter()
        elapsed = current_time - self.last_time
        
        # 计算需要睡眠的时间
        sleep_time = self.target_interval - elapsed
        
        if sleep_time > 0:
            # 使用高精度睡眠
            self._high_precision_sleep(sleep_time)
        
        # 更新时间
        self.last_time = time.perf_counter()
        self.call_count += 1
        
        # 更新速率统计
        self._update_rate()
    
    def _high_precision_sleep(self, duration: float):
        """
        高精度睡眠
        
        Args:
            duration: 睡眠时间 (秒)
        """
        if duration <= 0:
            return
        
        # 对于较长的等待，使用sleep
        if duration > 0.001:
            time.sleep(duration - 0.0005)
        
        # 忙等待剩余时间
        start = time.perf_counter()
        while time.perf_counter() - start < duration:
            pass
    
    def _update_rate(self):
        """更新速率统计"""
        current_time = time.perf_counter()
        total_time = current_time - self.start_time
        
        if total_time > 0:
            self.actual_rate = self.call_count / total_time
            
            # 记录速率历史
            self.rate_history.append(self.actual_rate)
            if len(self.rate_history) > self.max_history:
                self.rate_history.pop(0)
    
    def get_actual_rate(self) -> float:
        """
        获取实际速率
        
        Returns:
            实际速率 (Hz)
        """
        return self.actual_rate
    
    def get_rate_error(self) -> float:
        """
        获取速率误差
        
        Returns:
            速率误差 (Hz)
        """
        return self.actual_rate - self.target_rate
    
    def get_statistics(self) -> Dict:
        """
        获取统计信息
        
        Returns:
            统计信息字典
        """
        return {
            'name': self.name,
            'target_rate': self.target_rate,
            'actual_rate': self.actual_rate,
            'rate_error': self.get_rate_error(),
            'call_count': self.call_count,
            'uptime': time.perf_counter() - self.start_time if self.start_time > 0 else 0
        }


class Stopwatch:
    """
    秒表/计时器
    用于测量代码执行时间
    """
    
    def __init__(self, name: str = "Stopwatch"):
        """
        初始化秒表
        
        Args:
            name: 秒表名称
        """
        self.name = name
        self.start_time = 0.0
        self.stop_time = 0.0
        self.is_running = False
        self.laps = []
        
    def start(self) -> 'Stopwatch':
        """
        开始计时
        
        Returns:
            自身引用，支持链式调用
        """
        self.start_time = time.perf_counter()
        self.stop_time = 0.0
        self.is_running = True
        self.laps = []
        return self
    
    def stop(self) -> float:
        """
        停止计时
        
        Returns:
            经过的时间 (秒)
        """
        if not self.is_running:
            return 0.0
        
        self.stop_time = time.perf_counter()
        self.is_running = False
        return self.stop_time - self.start_time
    
    def lap(self) -> float:
        """
        记录分圈时间
        
        Returns:
            分圈时间 (秒)
        """
        if not self.is_running:
            return 0.0
        
        current_time = time.perf_counter()
        
        if self.laps:
            lap_time = current_time - self.laps[-1][1]
        else:
            lap_time = current_time - self.start_time
        
        self.laps.append((len(self.laps) + 1, current_time, lap_time))
        return lap_time
    
    def elapsed(self) -> float:
        """
        获取已经过的时间
        
        Returns:
            经过的时间 (秒)
        """
        if self.is_running:
            return time.perf_counter() - self.start_time
        elif self.stop_time > 0:
            return self.stop_time - self.start_time
        else:
            return 0.0
    
    def get_laps(self) -> List[Dict]:
        """
        获取所有分圈记录
        
        Returns:
            分圈记录列表
        """
        return [
            {
                'lap': lap[0],
                'time': lap[1] - self.start_time,
                'lap_time': lap[2]
            }
            for lap in self.laps
        ]
    
    def __enter__(self):
        """上下文管理器入口"""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.stop()


class PeriodicExecutor:
    """
    周期执行器
    以固定频率执行函数
    """
    
    def __init__(self, 
                 func: Callable,
                 frequency: float = 100.0,
                 name: str = "PeriodicExecutor"):
        """
        初始化周期执行器
        
        Args:
            func: 要执行的函数
            frequency: 执行频率 (Hz)
            name: 执行器名称
        """
        self.func = func
        self.frequency = frequency
        self.interval = 1.0 / frequency
        self.name = name
        
        # 状态
        self.is_running = False
        self.thread = None
        self._stop_event = threading.Event()
        
        # 统计
        self.execution_count = 0
        self.total_execution_time = 0.0
        self.avg_execution_time = 0.0
        self.max_execution_time = 0.0
        
    def start(self):
        """启动周期执行器"""
        if self.is_running:
            return
        
        self.is_running = True
        self._stop_event.clear()
        self.thread = threading.Thread(target=self._execution_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """停止周期执行器"""
        self.is_running = False
        self._stop_event.set()
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
    
    def _execution_loop(self):
        """执行循环"""
        rate_controller = RateController(self.frequency)
        rate_controller.start()
        
        while self.is_running and not self._stop_event.is_set():
            # 执行函数
            start_time = time.perf_counter()
            
            try:
                self.func()
            except Exception as e:
                print(f"周期执行器 '{self.name}' 执行失败: {e}")
            
            execution_time = time.perf_counter() - start_time
            
            # 更新统计
            self.execution_count += 1
            self.total_execution_time += execution_time
            self.avg_execution_time = self.total_execution_time / self.execution_count
            self.max_execution_time = max(self.max_execution_time, execution_time)
            
            # 控制执行频率
            rate_controller.sleep()
    
    def wait(self, timeout: float = None):
        """
        等待执行器完成
        
        Args:
            timeout: 超时时间 (秒)
        """
        if self.thread:
            self.thread.join(timeout=timeout)
    
    def get_statistics(self) -> Dict:
        """
        获取统计信息
        
        Returns:
            统计信息字典
        """
        return {
            'name': self.name,
            'frequency': self.frequency,
            'is_running': self.is_running,
            'execution_count': self.execution_count,
            'avg_execution_time': self.avg_execution_time,
            'max_execution_time': self.max_execution_time,
            'total_execution_time': self.total_execution_time
        }


class TimerManager:
    """
    定时器管理器
    管理多个定时器
    """
    
    def __init__(self):
        """初始化定时器管理器"""
        self.timers = {}
        self._lock = threading.Lock()
    
    def create_timer(self, 
                     name: str,
                     callback: Callable,
                     interval: float,
                     mode: TimerMode = TimerMode.PERIODIC) -> HighPrecisionTimer:
        """
        创建定时器
        
        Args:
            name: 定时器名称
            callback: 回调函数
            interval: 定时间隔 (秒)
            mode: 定时器模式
            
        Returns:
            定时器实例
        """
        with self._lock:
            if name in self.timers:
                self.timers[name].stop()
            
            timer = HighPrecisionTimer(
                callback=callback,
                interval=interval,
                mode=mode,
                name=name
            )
            self.timers[name] = timer
            return timer
    
    def start_timer(self, name: str):
        """
        启动指定定时器
        
        Args:
            name: 定时器名称
        """
        with self._lock:
            if name in self.timers:
                self.timers[name].start()
    
    def stop_timer(self, name: str):
        """
        停止指定定时器
        
        Args:
            name: 定时器名称
        """
        with self._lock:
            if name in self.timers:
                self.timers[name].stop()
    
    def remove_timer(self, name: str):
        """
        移除指定定时器
        
        Args:
            name: 定时器名称
        """
        with self._lock:
            if name in self.timers:
                self.timers[name].stop()
                del self.timers[name]
    
    def start_all(self):
        """启动所有定时器"""
        with self._lock:
            for timer in self.timers.values():
                timer.start()
    
    def stop_all(self):
        """停止所有定时器"""
        with self._lock:
            for timer in self.timers.values():
                timer.stop()
    
    def get_timer(self, name: str) -> Optional[HighPrecisionTimer]:
        """
        获取指定定时器
        
        Args:
            name: 定时器名称
            
        Returns:
            定时器实例或None
        """
        return self.timers.get(name)
    
    def get_all_statistics(self) -> Dict[str, Dict]:
        """
        获取所有定时器统计信息
        
        Returns:
            统计信息字典
        """
        with self._lock:
            return {name: timer.get_statistics() for name, timer in self.timers.items()}
    
    def cleanup(self):
        """清理所有定时器"""
        self.stop_all()
        with self._lock:
            self.timers.clear()


# 便捷函数
def create_timer(callback: Callable, interval: float, 
                 mode: TimerMode = TimerMode.PERIODIC) -> HighPrecisionTimer:
    """
    创建定时器的便捷函数
    
    Args:
        callback: 回调函数
        interval: 定时间隔 (秒)
        mode: 定时器模式
        
    Returns:
        定时器实例
    """
    return HighPrecisionTimer(callback=callback, interval=interval, mode=mode)


def create_rate_controller(target_rate: float) -> RateController:
    """
    创建速率控制器的便捷函数
    
    Args:
        target_rate: 目标速率 (Hz)
        
    Returns:
        速率控制器实例
    """
    return RateController(target_rate=target_rate)


def create_stopwatch(name: str = "Stopwatch") -> Stopwatch:
    """
    创建秒表的便捷函数
    
    Args:
        name: 秒表名称
        
    Returns:
        秒表实例
    """
    return Stopwatch(name=name)


def measure_time(func: Callable) -> Callable:
    """
    测量函数执行时间的装饰器
    
    Args:
        func: 要测量的函数
        
    Returns:
        装饰后的函数
    """
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        stopwatch = Stopwatch(func.__name__)
        stopwatch.start()
        result = func(*args, **kwargs)
        elapsed = stopwatch.stop()
        print(f"{func.__name__} 执行时间: {elapsed:.6f} 秒")
        return result
    return wrapper


# 测试代码
if __name__ == "__main__":
    # 测试高精度定时器
    print("测试高精度定时器...")
    
    count = 0
    def timer_callback(timestamp):
        global count
        count += 1
        if count % 100 == 0:
            print(f"定时器触发: {count}, 时间: {timestamp:.3f}s")
    
    timer = HighPrecisionTimer(
        callback=timer_callback,
        interval=0.01,  # 10ms
        mode=TimerMode.PERIODIC,
        name="TestTimer"
    )
    
    timer.start()
    time.sleep(1.0)  # 运行1秒
    timer.stop()
    
    stats = timer.get_statistics()
    print(f"统计信息: {stats}")
    
    # 测试速率控制器
    print("\n测试速率控制器...")
    
    rate_controller = RateController(target_rate=100.0)
    rate_controller.start()
    
    for i in range(100):
        # 模拟一些工作
        time.sleep(0.005)
        rate_controller.sleep()
    
    stats = rate_controller.get_statistics()
    print(f"速率控制器统计: {stats}")
    
    # 测试秒表
    print("\n测试秒表...")
    
    with Stopwatch("测试代码块") as sw:
        time.sleep(0.1)
        print(f"分圈1: {sw.lap():.6f}s")
        time.sleep(0.2)
        print(f"分圈2: {sw.lap():.6f}s")
        time.sleep(0.3)
        print(f"分圈3: {sw.lap():.6f}s")
    
    print(f"总时间: {sw.elapsed():.6f}s")
    print(f"分圈记录: {sw.get_laps()}")
    
    # 测试周期执行器
    print("\n测试周期执行器...")
    
    execution_count = 0
    def periodic_function():
        global execution_count
        execution_count += 1
    
    executor = PeriodicExecutor(
        func=periodic_function,
        frequency=50.0,  # 50Hz
        name="TestExecutor"
    )
    
    executor.start()
    time.sleep(0.5)  # 运行0.5秒
    executor.stop()
    
    stats = executor.get_statistics()
    print(f"周期执行器统计: {stats}")
    print(f"执行次数: {execution_count}")
    
    # 测试定时器管理器
    print("\n测试定时器管理器...")
    
    manager = TimerManager()
    
    timer1 = manager.create_timer("timer1", lambda t: None, 0.01)
    timer2 = manager.create_timer("timer2", lambda t: None, 0.02)
    
    manager.start_all()
    time.sleep(0.5)
    manager.stop_all()
    
    stats = manager.get_all_statistics()
    print(f"管理器统计: {stats}")
    
    manager.cleanup()
    
    print("\n所有测试完成!")