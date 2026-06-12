#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口实时绘图工具
================
功能：
  - 自动检测可用COM口
  - 多通道实时绘图（支持自定义通道名）
  - 数据实时保存为CSV文件
  - 可配置波特率、采样率、分隔符

依赖：pip install pyserial matplotlib

用法：
  python serial_plotter.py                          # 自动检测串口，9600波特率
  python serial_plotter.py -p COM3 -b 115200        # 指定串口和波特率
  python serial_plotter.py --channels 3 --sep ","   # 3通道，逗号分隔
  python serial_plotter.py --save data.csv          # 同时保存CSV
"""

import argparse
import csv
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path


def detect_serial_ports():
    """自动检测系统可用串口列表"""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [p.device for p in sorted(ports)]
    except ImportError:
        print("[错误] 未安装 pyserial，请执行: pip install pyserial")
        sys.exit(1)


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="串口实时绘图工具 - 多通道数据可视化",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("-p", "--port", type=str, default=None,
                        help="串口名称，如 COM3（默认自动检测）")
    parser.add_argument("-b", "--baudrate", type=int, default=9600,
                        help="波特率（默认 9600）")
    parser.add_argument("-c", "--channels", type=int, default=1,
                        help="数据通道数（默认 1）")
    parser.add_argument("--names", type=str, nargs="*", default=None,
                        help="通道名称列表，如 --names V_in V_out I_out")
    parser.add_argument("--sep", type=str, default=",",
                        help="串口数据分隔符（默认逗号）")
    parser.add_argument("--save", type=str, default=None,
                        help="保存CSV文件路径")
    parser.add_argument("--max-points", type=int, default=500,
                        help="图形窗口最大显示点数（默认 500）")
    parser.add_argument("--title", type=str, default="串口实时数据",
                        help="图表标题")
    return parser.parse_args()


class SerialPlotter:
    """串口实时绘图核心类"""

    def __init__(self, port, baudrate, num_channels, channel_names,
                 separator, max_points, title, save_path):
        self.port = port
        self.baudrate = baudrate
        self.num_channels = num_channels
        self.channel_names = channel_names
        self.separator = separator
        self.max_points = max_points
        self.title = title
        self.save_path = save_path

        # 每个通道用 deque 存储数据，自动丢弃旧数据
        self.data = [deque(maxlen=max_points) for _ in range(num_channels)]
        self.timestamps = deque(maxlen=max_points)
        self.lock = threading.Lock()
        self.running = True
        self.start_time = time.time()
        self.sample_count = 0

        # CSV 写入器
        self.csv_writer = None
        self.csv_file = None

    def open_csv(self):
        """打开CSV文件准备写入"""
        if self.save_path:
            self.csv_file = open(self.save_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            header = ["时间戳", "相对时间(s)"] + self.channel_names
            self.csv_writer.writerow(header)
            print(f"[信息] CSV数据将保存到: {self.save_path}")

    def close_csv(self):
        """关闭CSV文件"""
        if self.csv_file:
            self.csv_file.close()
            print(f"[信息] CSV已保存，共 {self.sample_count} 条记录")

    def serial_reader(self):
        """后台线程：持续读取串口数据"""
        import serial
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"[信息] 已打开串口 {self.port}，波特率 {self.baudrate}")
            print(f"[信息] 等待数据... 按 Ctrl+C 退出")

            while self.running:
                raw_line = ser.readline()
                if not raw_line:
                    continue

                try:
                    line = raw_line.decode("utf-8", errors="ignore").strip()
                except Exception:
                    continue

                if not line:
                    continue

                # 解析多通道数据
                parts = line.split(self.separator)
                values = []
                try:
                    for i in range(self.num_channels):
                        if i < len(parts):
                            values.append(float(parts[i].strip()))
                        else:
                            values.append(0.0)
                except ValueError:
                    continue  # 跳过无法解析的行

                # 线程安全地更新数据
                now = time.time()
                rel_time = now - self.start_time
                with self.lock:
                    self.timestamps.append(rel_time)
                    for i in range(self.num_channels):
                        self.data[i].append(values[i])
                    self.sample_count += 1

                # 写入CSV
                if self.csv_writer:
                    ts_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    row = [ts_str, f"{rel_time:.3f}"] + [f"{v:.6g}" for v in values]
                    self.csv_writer.writerow(row)

            ser.close()
        except Exception as e:
            print(f"[错误] 串口读取异常: {e}")
            self.running = False

    def run_plot(self):
        """主循环：实时绘图"""
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        # 打开CSV
        self.open_csv()

        # 启动串口读取线程
        reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
        reader_thread.start()

        # 创建图表
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.set_title(self.title, fontsize=14)
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("数值")
        lines = []
        for i in range(self.num_channels):
            line, = ax.plot([], [], label=self.channel_names[i])
            lines.append(line)
        ax.legend(loc="upper left")
        ax.grid(True, alpha=0.3)

        def update(frame):
            """动画更新函数"""
            with self.lock:
                if len(self.timestamps) < 2:
                    return lines
                t = list(self.timestamps)
                for i in range(self.num_channels):
                    lines[i].set_data(t, list(self.data[i]))

            # 自动调整坐标范围
            ax.relim()
            ax.autoscale_view()
            return lines

        ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.close_csv()
            print("[信息] 绘图已结束")


def main():
    """主入口"""
    args = parse_args()

    # 检测串口
    if args.port is None:
        ports = detect_serial_ports()
        if not ports:
            print("[错误] 未检测到可用串口，请连接设备后重试")
            sys.exit(1)
        print(f"[信息] 检测到串口: {', '.join(ports)}")
        args.port = ports[0]
        print(f"[信息] 自动选择: {args.port}")

    # 通道名称
    if args.names and len(args.names) >= args.channels:
        channel_names = args.names[:args.channels]
    else:
        channel_names = [f"CH{i+1}" for i in range(args.channels)]

    print(f"[信息] 通道数: {args.channels}, 名称: {channel_names}")

    # 启动绘图
    plotter = SerialPlotter(
        port=args.port,
        baudrate=args.baudrate,
        num_channels=args.channels,
        channel_names=channel_names,
        separator=args.sep,
        max_points=args.max_points,
        title=args.title,
        save_path=args.save,
    )
    plotter.run_plot()


if __name__ == "__main__":
    main()
