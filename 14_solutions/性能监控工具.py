#!/usr/bin/env python3
"""
Orange Pi 5 电赛系统性能监控工具
=================================
功能：CPU/内存/帧率/延迟 实时监控
用法：python3 性能监控工具.py [--log output.csv] [--interval 1.0]
"""

import os
import sys
import time
import csv
import argparse
import threading
from datetime import datetime
from collections import deque

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False


class SystemMonitor:
    """系统资源监控"""

    def __init__(self):
        self.cpu_history = deque(maxlen=60)
        self.mem_history = deque(maxlen=60)
        self.temp_history = deque(maxlen=60)

    def get_cpu_usage(self) -> dict:
        """获取CPU使用率 (per-core)"""
        result = {'total': 0.0, 'cores': []}
        if HAS_PSUTIL:
            result['cores'] = psutil.cpu_percent(interval=0, percpu=True)
            result['total'] = sum(result['cores']) / len(result['cores'])
        else:
            # Fallback: 解析/proc/stat
            try:
                with open('/proc/stat') as f:
                    line = f.readline()
                    values = list(map(int, line.split()[1:]))
                    idle = values[3]
                    total = sum(values)
                    if hasattr(self, '_prev_total'):
                        dtotal = total - self._prev_total
                        didle = idle - self._prev_idle
                        result['total'] = (
                            100.0 * (dtotal - didle) / dtotal if dtotal else 0
                        )
                    self._prev_total = total
                    self._prev_idle = idle
            except Exception:
                pass
        self.cpu_history.append(result['total'])
        return result

    def get_memory(self) -> dict:
        """获取内存使用"""
        result = {'total_mb': 0, 'used_mb': 0, 'percent': 0}
        if HAS_PSUTIL:
            m = psutil.virtual_memory()
            result['total_mb'] = m.total // (1024 * 1024)
            result['used_mb'] = m.used // (1024 * 1024)
            result['percent'] = m.percent
        else:
            try:
                with open('/proc/meminfo') as f:
                    info = {}
                    for line in f:
                        parts = line.split(':')
                        if len(parts) == 2:
                            key = parts[0].strip()
                            val = int(parts[1].strip().split()[0])
                            info[key] = val
                    total = info.get('MemTotal', 0)
                    avail = info.get('MemAvailable', 0)
                    result['total_mb'] = total // 1024
                    result['used_mb'] = (total - avail) // 1024
                    result['percent'] = (
                        100.0 * (total - avail) / total if total else 0
                    )
            except Exception:
                pass
        self.mem_history.append(result['percent'])
        return result

    def get_temperature(self) -> dict:
        """获取温度"""
        result = {'cpu': 0.0, 'gpu': 0.0}
        # CPU温度
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                result['cpu'] = int(f.read().strip()) / 1000.0
        except Exception:
            pass
        # GPU温度
        try:
            with open('/sys/class/thermal/thermal_zone1/temp') as f:
                result['gpu'] = int(f.read().strip()) / 1000.0
        except Exception:
            pass
        self.temp_history.append(result['cpu'])
        return result

    def get_gpu_npu_freq(self) -> dict:
        """获取GPU/NPU频率"""
        result = {'gpu_mhz': 0, 'npu_mhz': 0}
        try:
            with open('/sys/class/devfreq/fb000000.gpu/cur_freq') as f:
                result['gpu_mhz'] = int(f.read().strip()) // 1000000
        except Exception:
            pass
        try:
            with open('/sys/class/devfreq/2c0000000.npu/cur_freq') as f:
                result['npu_mhz'] = int(f.read().strip()) // 1000000
        except Exception:
            pass
        return result

    def get_process_info(self, name_pattern: str) -> dict:
        """获取指定进程信息"""
        result = {'pid': 0, 'cpu': 0, 'mem_mb': 0, 'threads': 0}
        if not HAS_PSUTIL:
            return result
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info.get('cmdline') or [])
                if name_pattern in cmdline:
                    p = psutil.Process(proc.info['pid'])
                    result['pid'] = p.pid
                    result['cpu'] = p.cpu_percent(interval=0)
                    result['mem_mb'] = p.memory_info().rss // (1024 * 1024)
                    result['threads'] = p.num_threads()
                    break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return result


class FrameRateMonitor:
    """帧率监控器"""

    def __init__(self, window_size: int = 30):
        self._timestamps = deque(maxlen=window_size)
        self._lock = threading.Lock()

    def tick(self):
        """记录一帧"""
        with self._lock:
            self._timestamps.append(time.monotonic())

    def get_fps(self) -> float:
        """计算当前帧率"""
        with self._lock:
            if len(self._timestamps) < 2:
                return 0.0
            dt = self._timestamps[-1] - self._timestamps[0]
            if dt <= 0:
                return 0.0
            return (len(self._timestamps) - 1) / dt

    def get_jitter_us(self) -> float:
        """计算帧间隔抖动 (标准差, 微秒)"""
        with self._lock:
            if len(self._timestamps) < 3:
                return 0.0
            intervals = []
            for i in range(1, len(self._timestamps)):
                intervals.append(
                    (self._timestamps[i] - self._timestamps[i-1]) * 1e6)
            mean = sum(intervals) / len(intervals)
            variance = sum((x - mean) ** 2 for x in intervals) / len(intervals)
            return variance ** 0.5


class LatencyMonitor:
    """延迟监控器 (视觉→控制端到端)"""

    def __init__(self):
        self._latencies = deque(maxlen=100)

    def record(self, latency_ms: float):
        self._latencies.append(latency_ms)

    def get_stats(self) -> dict:
        if not self._latencies:
            return {'avg': 0, 'max': 0, 'min': 0, 'p99': 0}
        data = sorted(self._latencies)
        n = len(data)
        return {
            'avg': sum(data) / n,
            'max': data[-1],
            'min': data[0],
            'p99': data[int(n * 0.99)] if n > 1 else data[0],
        }

    def record_from_timestamps(self, send_ns: int, recv_ns: int):
        """根据时间戳计算延迟"""
        latency_ms = (recv_ns - send_ns) / 1e6
        if 0 < latency_ms < 1000:  # 合理性检查
            self.record(latency_ms)


# ============================================================
# 终端显示
# ============================================================

class TerminalDisplay:
    """终端实时显示"""

    # 进度条字符
    BAR_FULL = '█'
    BAR_EMPTY = '░'

    @staticmethod
    def clear():
        os.system('clear' if os.name != 'nt' else 'cls')

    @staticmethod
    def bar(value: float, max_val: float = 100.0,
            width: int = 20, color: bool = True) -> str:
        """绘制进度条"""
        ratio = min(value / max_val, 1.0) if max_val > 0 else 0
        filled = int(ratio * width)
        empty = width - filled

        bar_str = TerminalDisplay.BAR_FULL * filled + \
                  TerminalDisplay.BAR_EMPTY * empty

        if color:
            if ratio > 0.8:
                return f'\033[91m{bar_str}\033[0m'  # 红色
            elif ratio > 0.6:
                return f'\033[93m{bar_str}\033[0m'  # 黄色
            else:
                return f'\033[92m{bar_str}\033[0m'  # 绿色
        return bar_str

    @staticmethod
    def sparkline(data, width: int = 20) -> str:
        """绘制迷你折线图"""
        if not data:
            return ' ' * width
        chars = '▁▂▃▄▅▆▇█'
        mn, mx = min(data), max(data)
        rng = mx - mn if mx != mn else 1
        samples = list(data)[-width:]
        result = ''
        for v in samples:
            idx = int((v - mn) / rng * (len(chars) - 1))
            result += chars[idx]
        return result.ljust(width)


# ============================================================
# 主监控类
# ============================================================

class PerformanceDashboard:
    """性能监控仪表盘"""

    def __init__(self, log_file: str = None, interval: float = 1.0):
        self.sys_monitor = SystemMonitor()
        self.fps_monitor = FrameRateMonitor()
        self.latency_monitor = LatencyMonitor()
        self.display = TerminalDisplay()
        self.interval = interval
        self.log_file = log_file
        self._running = False
        self._csv_writer = None
        self._csv_file = None

        if log_file:
            self._csv_file = open(log_file, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'timestamp', 'cpu_total', 'cpu_core0', 'cpu_core1',
                'cpu_core2', 'cpu_core3', 'cpu_core4', 'cpu_core5',
                'cpu_core6', 'cpu_core7',
                'mem_percent', 'mem_used_mb',
                'cpu_temp', 'gpu_temp',
                'gpu_freq_mhz', 'npu_freq_mhz',
                'vision_fps', 'vision_jitter_us',
                'latency_avg_ms', 'latency_max_ms',
            ])

    def update(self):
        """更新一次监控数据"""
        cpu = self.sys_monitor.get_cpu_usage()
        mem = self.sys_monitor.get_memory()
        temp = self.sys_monitor.get_temperature()
        freq = self.sys_monitor.get_gpu_npu_freq()
        fps = self.fps_monitor.get_fps()
        jitter = self.fps_monitor.get_jitter_us()
        lat = self.latency_monitor.get_stats()

        # 记录到CSV
        if self._csv_writer:
            cores = cpu.get('cores', [0]*8)
            while len(cores) < 8:
                cores.append(0)
            self._csv_writer.writerow([
                datetime.now().isoformat(),
                f"{cpu['total']:.1f}",
                *[f"{c:.1f}" for c in cores[:8]],
                f"{mem['percent']:.1f}",
                f"{mem['used_mb']}",
                f"{temp['cpu']:.1f}",
                f"{temp['gpu']:.1f}",
                freq['gpu_mhz'],
                freq['npu_mhz'],
                f"{fps:.1f}",
                f"{jitter:.1f}",
                f"{lat['avg']:.2f}",
                f"{lat['max']:.2f}",
            ])
            self._csv_file.flush()

        return {
            'cpu': cpu, 'mem': mem, 'temp': temp,
            'freq': freq, 'fps': fps, 'jitter': jitter, 'latency': lat,
        }

    def render(self, data: dict):
        """终端渲染"""
        self.display.clear()
        db = self.display
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        print(f"╔══════════════════════════════════════════════════════════╗")
        print(f"║         Orange Pi 5 电赛系统性能监控                    ║")
        print(f"║         {now}                          ║")
        print(f"╠══════════════════════════════════════════════════════════╣")

        # CPU
        cpu = data['cpu']
        print(f"║  CPU 总使用率: {db.bar(cpu['total'], width=25)}"
              f" {cpu['total']:5.1f}%              ║")
        if cpu.get('cores'):
            cores = cpu['cores']
            for i in range(0, len(cores), 4):
                line = "║  "
                for j in range(4):
                    if i+j < len(cores):
                        core_type = "A76" if (i+j) >= 4 else "A55"
                        line += f"  C{i+j}({core_type}):{cores[i+j]:4.0f}%"
                print(f"{line:<60}║")

        # 内存
        mem = data['mem']
        print(f"║  内存: {db.bar(mem['percent'], width=25)}"
              f" {mem['used_mb']}MB/{mem['total_mb']}MB     ║")

        # 温度
        temp = data['temp']
        temp_color = "🔴" if temp['cpu'] > 75 else "🟡" if temp['cpu'] > 60 else "🟢"
        print(f"║  温度: CPU {temp_color}{temp['cpu']:.0f}°C"
              f"   GPU {temp['gpu']:.0f}°C"
              f"                        ║")

        # 频率
        freq = data['freq']
        print(f"║  频率: GPU {freq['gpu_mhz']}MHz"
              f"   NPU {freq['npu_mhz']}MHz"
              f"                      ║")

        print(f"╠══════════════════════════════════════════════════════════╣")

        # 视觉帧率
        fps = data['fps']
        jitter = data['jitter']
        fps_ok = "✅" if fps >= 25 else "⚠️" if fps >= 15 else "❌"
        print(f"║  视觉帧率: {fps_ok} {fps:.1f} fps"
              f"   抖动: {jitter:.0f} μs"
              f"               ║")

        # 延迟
        lat = data['latency']
        lat_ok = "✅" if lat['avg'] < 1 else "⚠️" if lat['avg'] < 5 else "❌"
        print(f"║  端到端延迟: {lat_ok}"
              f"  avg={lat['avg']:.2f}ms"
              f"  max={lat['max']:.2f}ms"
              f"  p99={lat['p99']:.2f}ms  ║")

        print(f"╠══════════════════════════════════════════════════════════╣")

        # 历史趋势
        if len(self.sys_monitor.cpu_history) > 1:
            cpu_spark = db.sparkline(self.sys_monitor.cpu_history, 30)
            print(f"║  CPU趋势: {cpu_spark}              ║")
        if len(self.sys_monitor.temp_history) > 1:
            temp_spark = db.sparkline(self.sys_monitor.temp_history, 30)
            print(f"║  温度趋势: {temp_spark}              ║")

        print(f"╚══════════════════════════════════════════════════════════╝")
        print(f"  按 Ctrl+C 退出  |  日志: {self.log_file or '未设置'}")

    def run(self):
        """主循环"""
        self._running = True
        print("性能监控启动... (按Ctrl+C退出)")
        try:
            while self._running:
                data = self.update()
                self.render(data)
                time.sleep(self.interval)
        except KeyboardInterrupt:
            print("\n监控已停止")
        finally:
            self.cleanup()

    def cleanup(self):
        if self._csv_file:
            self._csv_file.close()


# ============================================================
# 轻量级版本 (无psutil依赖)
# ============================================================

class LightweightMonitor:
    """
    轻量级监控 (不依赖psutil)
    直接读取 /proc 文件系统
    """

    def __init__(self):
        self._prev_cpu = None

    def get_cpu(self) -> float:
        try:
            with open('/proc/stat') as f:
                parts = f.readline().split()[1:]
                values = [int(x) for x in parts]
                idle = values[3] + values[4]
                total = sum(values)
                if self._prev_cpu:
                    d_idle = idle - self._prev_cpu[0]
                    d_total = total - self._prev_cpu[1]
                    usage = 100.0 * (1 - d_idle / d_total) if d_total else 0
                else:
                    usage = 0
                self._prev_cpu = (idle, total)
                return usage
        except Exception:
            return 0

    def get_mem(self) -> float:
        try:
            with open('/proc/meminfo') as f:
                info = {}
                for line in f:
                    k, v = line.split(':')[:2]
                    info[k.strip()] = int(v.strip().split()[0])
                total = info['MemTotal']
                avail = info['MemAvailable']
                return 100.0 * (total - avail) / total
        except Exception:
            return 0

    def get_temp(self) -> float:
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                return int(f.read()) / 1000.0
        except Exception:
            return 0

    def get_uptime(self) -> str:
        try:
            with open('/proc/uptime') as f:
                secs = float(f.read().split()[0])
                h = int(secs // 3600)
                m = int((secs % 3600) // 60)
                s = int(secs % 60)
                return f"{h:02d}:{m:02d}:{s:02d}"
        except Exception:
            return "??:??:??"


# ============================================================
# 快捷输出模式 (适合日志记录)
# ============================================================

def compact_report(sys_mon: SystemMonitor, fps: float, latency: dict) -> str:
    """单行紧凑报告"""
    cpu = sys_mon.get_cpu_usage()
    mem = sys_mon.get_memory()
    temp = sys_mon.get_temperature()
    return (
        f"[{datetime.now().strftime('%H:%M:%S')}] "
        f"CPU={cpu['total']:.0f}% "
        f"MEM={mem['percent']:.0f}% "
        f"TEMP={temp['cpu']:.0f}°C "
        f"FPS={fps:.0f} "
        f"LAT={latency['avg']:.1f}ms"
    )


# ============================================================
# 主入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(description='Orange Pi 5 性能监控工具')
    parser.add_argument('--log', type=str, default=None,
                        help='CSV日志输出文件路径')
    parser.add_argument('--interval', type=float, default=1.0,
                        help='刷新间隔 (秒)')
    parser.add_argument('--compact', action='store_true',
                        help='紧凑单行输出模式')
    args = parser.parse_args()

    if args.compact:
        # 紧凑模式
        monitor = SystemMonitor()
        try:
            while True:
                cpu = monitor.get_cpu_usage()
                mem = monitor.get_memory()
                temp = monitor.get_temperature()
                print(
                    f"\r[{datetime.now().strftime('%H:%M:%S')}] "
                    f"CPU={cpu['total']:5.1f}% "
                    f"MEM={mem['percent']:5.1f}% "
                    f"TEMP={temp['cpu']:4.1f}°C "
                    f"| {'█' * int(cpu['total'] / 5)}{'░' * (20 - int(cpu['total'] / 5))}",
                    end='', flush=True
                )
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n退出")
    else:
        # 完整仪表盘模式
        dashboard = PerformanceDashboard(
            log_file=args.log,
            interval=args.interval,
        )
        dashboard.run()


if __name__ == '__main__':
    main()
