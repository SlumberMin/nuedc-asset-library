"""
视频录制器 - 带时间戳 + 性能信息叠加
适用于: 实验记录、调试回放、比赛视频存档
"""

import cv2
import time
import numpy as np
from datetime import datetime
from pathlib import Path


class VideoRecorder:
    """视频录制器，支持时间戳和性能信息叠加"""

    def __init__(self, output_path=None, fps=30, codec='XVID',
                 show_timestamp=True, show_fps=True, show_info=True):
        """
        Args:
            output_path: 输出文件路径(默认自动生成)
            fps: 帧率
            codec: 编码器(XVID, MJPG, mp4v等)
            show_timestamp: 是否叠加时间戳
            show_fps: 是否叠加FPS
            show_info: 是否叠加额外信息行
        """
        self.fps = fps
        self.codec = codec
        self.show_timestamp = show_timestamp
        self.show_fps = show_fps
        self.show_info = show_info

        # 输出路径
        if output_path is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = f"record_{ts}.avi"
        self.output_path = Path(output_path)

        # 性能追踪
        self.writer = None
        self.recording = False
        self.frame_count = 0
        self.start_time = None
        self.fps_buffer = []
        self.last_time = time.time()

        # 叠加信息
        self.info_lines = []  # 额外信息行

    def _create_writer(self, frame):
        """根据第一帧创建VideoWriter"""
        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        self.writer = cv2.VideoWriter(str(self.output_path), fourcc, self.fps, (w, h))
        self.recording = True
        self.start_time = time.time()
        print(f"开始录制: {self.output_path} ({w}x{h}@{self.fps}fps)")

    def add_info(self, text):
        """添加一行叠加信息(调用方动态设置)"""
        self.info_lines = [text] if isinstance(text, str) else list(text)

    def _get_fps(self):
        """计算实时FPS"""
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 0:
            self.fps_buffer.append(1.0 / dt)
        if len(self.fps_buffer) > 30:
            self.fps_buffer.pop(0)
        return np.mean(self.fps_buffer) if self.fps_buffer else 0

    def _overlay_info(self, frame):
        """在帧上叠加信息"""
        display = frame.copy()
        h, w = frame.shape[:2]
        y_offset = 30

        # 时间戳
        if self.show_timestamp:
            ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            cv2.putText(display, ts, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            y_offset += 30

        # FPS
        current_fps = self._get_fps()
        if self.show_fps:
            elapsed = time.time() - self.start_time if self.start_time else 0
            text = f"FPS: {current_fps:.1f} | Frames: {self.frame_count} | Time: {elapsed:.1f}s"
            cv2.putText(display, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 25

        # 额外信息
        if self.show_info and self.info_lines:
            for line in self.info_lines:
                cv2.putText(display, line, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                y_offset += 22

        # 录制指示
        if self.recording:
            cv2.circle(display, (w - 20, 20), 8, (0, 0, 255), -1)
            cv2.putText(display, "REC", (w - 55, 26),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return display

    def write(self, frame):
        """
        写入一帧(自动叠加信息)
        Args:
            frame: BGR图像
        Returns:
            display: 叠加信息后的帧(用于显示)
        """
        if self.writer is None:
            self._create_writer(frame)

        # 叠加信息到显示帧
        display = self._overlay_info(frame)

        # 录制原始帧(不含叠加)或带叠加的帧
        self.writer.write(display)
        self.frame_count += 1

        return display

    def write_raw(self, frame):
        """写入原始帧(不含叠加)"""
        if self.writer is None:
            self._create_writer(frame)
        self.writer.write(frame)
        self.frame_count += 1

    def stop(self):
        """停止录制"""
        if self.writer:
            self.writer.release()
            self.writer = None
            self.recording = False
            elapsed = time.time() - self.start_time if self.start_time else 0
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
            print(f"录制结束: {self.frame_count}帧, {elapsed:.1f}秒, 平均{avg_fps:.1f}fps")
            print(f"保存至: {self.output_path}")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.stop()


def demo():
    """摄像头录制演示"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    recorder = VideoRecorder(
        output_path="record_demo.avi",
        show_timestamp=True,
        show_fps=True
    )

    print("按 'r' 开始/停止录制, 'q' 退出")

    recording = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 模拟一些处理数据
        recorder.add_info([
            f"Resolution: {frame.shape[1]}x{frame.shape[0]}",
            f"Mode: Demo"
        ])

        if recording:
            display = recorder.write(frame)
        else:
            display = recorder._overlay_info(frame)

        cv2.imshow("Video Recorder", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('r'):
            if recording:
                recorder.stop()
                recording = False
            else:
                recorder = VideoRecorder()  # 新文件
                recording = True

    if recording:
        recorder.stop()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo()
