"""
TFT显示仿真 - 帧缓冲/刷新率/DMA传输/图形算法
nuedc-asset-library V3
"""
import numpy as np
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple, Optional
import time
import struct

class PixelFormat(Enum):
    RGB565 = "RGB565"    # 16bit, 2字节
    RGB888 = "RGB888"    # 24bit, 3字节
    ARGB8888 = "ARGB8888" # 32bit, 4字节

@dataclass
class TFTDisplay:
    """TFT显示屏模型"""
    width: int = 320
    height: int = 240
    pixel_format: PixelFormat = PixelFormat.RGB565
    refresh_rate_hz: float = 60.0
    backlight_brightness: float = 1.0  # 0~1

    def __post_init__(self):
        bpp = {PixelFormat.RGB565: 2, PixelFormat.RGB888: 3, PixelFormat.ARGB8888: 4}
        self.bytes_per_pixel = bpp[self.pixel_format]
        self.framebuffer = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.frame_count = 0
        self.frame_time_us = 1e6 / self.refresh_rate_hz

    @property
    def framebuffer_size_bytes(self) -> int:
        return self.width * self.height * self.bytes_per_pixel

    @property
    def pixel_count(self) -> int:
        return self.width * self.height

@dataclass
class DMAConfig:
    """DMA传输配置"""
    bus_width: int = 16          # DMA总线宽度bit
    clock_mhz: float = 100.0    # DMA时钟频率
    burst_size: int = 16         # 突发传输长度
    priority: int = 3            # 优先级 0-7

    @property
    def bandwidth_mbps(self) -> float:
        return self.bus_width * self.clock_mhz / 8  # MB/s

    def transfer_time_us(self, size_bytes: int) -> float:
        """计算DMA传输时间(微秒)"""
        total_transfers = size_bytes / (self.bus_width / 8)
        effective_clock = self.clock_mhz * 1e6
        # 考虑突发传输开销
        burst_overhead_cycles = total_transfers / self.burst_size * 2  # 每突发2周期开销
        return (total_transfers + burst_overhead_cycles) / effective_clock * 1e6

class GraphicsEngine:
    """图形算法引擎"""

    @staticmethod
    def draw_pixel(fb: np.ndarray, x: int, y: int, color: Tuple[int, int, int]):
        h, w = fb.shape[:2]
        if 0 <= x < w and 0 <= y < h:
            fb[y, x] = color

    @staticmethod
    def draw_line(fb: np.ndarray, x0: int, y0: int, x1: int, y1: int, color: Tuple[int, int, int]):
        """Bresenham直线算法"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            GraphicsEngine.draw_pixel(fb, x0, y0, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    @staticmethod
    def draw_rect(fb: np.ndarray, x: int, y: int, w: int, h: int, color: Tuple[int, int, int], filled: bool = False):
        if filled:
            y0, y1 = max(0, y), min(fb.shape[0], y + h)
            x0, x1 = max(0, x), min(fb.shape[1], x + w)
            fb[y0:y1, x0:x1] = color
        else:
            GraphicsEngine.draw_line(fb, x, y, x + w, y, color)
            GraphicsEngine.draw_line(fb, x + w, y, x + w, y + h, color)
            GraphicsEngine.draw_line(fb, x + w, y + h, x, y + h, color)
            GraphicsEngine.draw_line(fb, x, y + h, x, y, color)

    @staticmethod
    def draw_circle(fb: np.ndarray, cx: int, cy: int, r: int, color: Tuple[int, int, int], filled: bool = False):
        """Bresenham圆算法"""
        x, y = r, 0
        err = 1 - r
        points = []
        while x >= y:
            points.extend([(cx+x,cy+y),(cx+y,cy+x),(cx-y,cy+x),(cx-x,cy+y),
                          (cx-x,cy-y),(cx-y,cy-x),(cx+y,cy-x),(cx+x,cy-y)])
            y += 1
            if err < 0:
                err += 2 * y + 1
            else:
                x -= 1
                err += 2 * (y - x) + 1
        if filled:
            for py in range(max(0, cy-r), min(fb.shape[0], cy+r+1)):
                for px in range(max(0, cx-r), min(fb.shape[1], cx+r+1)):
                    if (px-cx)**2 + (py-cy)**2 <= r**2:
                        fb[py, px] = color
        else:
            for px, py in points:
                GraphicsEngine.draw_pixel(fb, px, py, color)

    @staticmethod
    def fill_gradient(fb: np.ndarray, direction: str = "vertical"):
        """渐变填充"""
        h, w = fb.shape[:2]
        if direction == "vertical":
            for y in range(h):
                fb[y, :] = [int(255 * y / h), int(128 * (1 - y / h)), 200]
        else:
            for x in range(w):
                fb[:, x] = [int(200 * x / w), int(255 * (1 - x / w)), 128]

class FramebufferManager:
    """帧缓冲管理器 - 双缓冲/局部刷新"""

    def __init__(self, display: TFTDisplay):
        self.display = display
        self.front_buffer = np.zeros((display.height, display.width, 3), dtype=np.uint8)
        self.back_buffer = np.zeros((display.height, display.width, 3), dtype=np.uint8)
        self.dirty_regions: List[Tuple[int, int, int, int]] = []
        self.tearing = False

    def swap_buffers(self):
        """交换前后缓冲"""
        self.front_buffer, self.back_buffer = self.back_buffer, self.front_buffer
        self.dirty_regions.clear()

    def mark_dirty(self, x: int, y: int, w: int, h: int):
        self.dirty_regions.append((x, y, w, h))

    def get_dirty_transfer_size(self) -> int:
        """计算脏区域需要传输的字节数"""
        total_pixels = 0
        for rx, ry, rw, rh in self.dirty_regions:
            total_pixels += rw * rh
        return total_pixels * self.display.bytes_per_pixel

    def compute_diff(self) -> np.ndarray:
        """计算前后缓冲差异（仅传输变化区域）"""
        diff = np.any(self.front_buffer != self.back_buffer, axis=2)
        return diff

class TFTSimulator:
    """TFT显示完整仿真器"""

    def __init__(self, display: TFTDisplay = None, dma: DMAConfig = None):
        self.display = display or TFTDisplay()
        self.dma = dma or DMAConfig()
        self.fb_mgr = FramebufferManager(self.display)
        self.gfx = GraphicsEngine()
        self.stats = {
            "frames_rendered": 0,
            "total_pixels_drawn": 0,
            "total_dma_bytes": 0,
            "avg_frame_time_us": 0.0,
            "fps_actual": 0.0,
        }
        self._frame_times: List[float] = []

    def render_frame(self, draw_func=None):
        """渲染一帧"""
        t0 = time.perf_counter()

        buf = self.fb_mgr.back_buffer
        buf[:] = 0  # 清屏

        if draw_func:
            draw_func(buf, self.gfx)

        # DMA传输
        fb_size = self.display.framebuffer_size_bytes
        dma_time = self.dma.transfer_time_us(fb_size)

        self.fb_mgr.swap_buffers()

        dt = (time.perf_counter() - t0) * 1e6  # us
        self._frame_times.append(dt + dma_time)
        self.stats["frames_rendered"] += 1
        self.stats["total_dma_bytes"] += fb_size
        self.stats["total_pixels_drawn"] += self.display.pixel_count

        return {
            "frame_render_us": dt,
            "dma_transfer_us": dma_time,
            "total_frame_us": dt + dma_time,
            "meets_refresh": (dt + dma_time) < self.display.frame_time_us,
        }

    def simulate_refresh(self, num_frames: int = 60, draw_func=None) -> dict:
        """模拟多帧刷新"""
        results = []
        for i in range(num_frames):
            r = self.render_frame(draw_func)
            results.append(r)

        times = [r["total_frame_us"] for r in results]
        self.stats["avg_frame_time_us"] = np.mean(times)
        self.stats["fps_actual"] = 1e6 / np.mean(times) if np.mean(times) > 0 else 0
        return {
            "frames": num_frames,
            "avg_frame_us": np.mean(times),
            "max_frame_us": np.max(times),
            "min_frame_us": np.min(times),
            "achieved_fps": self.stats["fps_actual"],
            "target_fps": self.display.refresh_rate_hz,
            "all_meets_refresh": all(r["meets_refresh"] for r in results),
            "stats": self.stats.copy(),
        }

    def simulate_partial_refresh(self, dirty_regions: List[Tuple[int,int,int,int]]) -> dict:
        """局部刷新仿真"""
        for r in dirty_regions:
            self.fb_mgr.mark_dirty(*r)
        dirty_bytes = self.fb_mgr.get_dirty_transfer_size()
        full_bytes = self.display.framebuffer_size_bytes
        dma_time_partial = self.dma.transfer_time_us(dirty_bytes)
        dma_time_full = self.dma.transfer_time_us(full_bytes)
        return {
            "dirty_regions": len(dirty_regions),
            "dirty_bytes": dirty_bytes,
            "full_frame_bytes": full_bytes,
            "compression_ratio": dirty_bytes / full_bytes if full_bytes > 0 else 0,
            "dma_partial_us": dma_time_partial,
            "dma_full_us": dma_time_full,
            "speedup": dma_time_full / dma_time_partial if dma_time_partial > 0 else float('inf'),
        }

    def color_conversion_test(self) -> dict:
        """颜色格式转换性能测试"""
        r, g, b = 200, 100, 50
        rgb565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
        r565 = (rgb565 >> 11) << 3
        g565 = ((rgb565 >> 5) & 0x3F) << 2
        b565 = (rgb565 & 0x1F) << 3
        return {
            "input_rgb": (r, g, b),
            "rgb565_hex": f"0x{rgb565:04X}",
            "roundtrip_rgb": (r565, g565, b565),
            "color_error": abs(r - r565) + abs(g - g565) + abs(b - b565),
        }


# ── 演示 ──
def demo():
    print("=" * 60)
    print("TFT显示仿真器 - Demo")
    print("=" * 60)

    # 1. 基本帧率测试
    tft = TFTSimulator(
        TFTDisplay(320, 240, PixelFormat.RGB565, 60),
        DMAConfig(bus_width=16, clock_mhz=100)
    )

    def demo_scene(buf, gfx):
        gfx.fill_gradient(buf, "vertical")
        gfx.draw_rect(buf, 50, 50, 100, 80, (255, 0, 0), filled=True)
        gfx.draw_circle(buf, 200, 120, 40, (0, 255, 0))
        gfx.draw_line(buf, 0, 0, 319, 239, (255, 255, 255))

    result = tft.simulate_refresh(120, demo_scene)
    print(f"\n[帧率测试] {result['frames']}帧")
    print(f"  平均帧时间: {result['avg_frame_us']:.1f} us")
    print(f"  实际FPS: {result['achieved_fps']:.1f} (目标: {result['target_fps']})")
    print(f"  是否达标: {result['all_meets_refresh']}")

    # 2. 局部刷新
    print(f"\n[局部刷新]")
    partial = tft.simulate_partial_refresh([(0,0,100,100), (200,150,50,50)])
    print(f"  脏区域: {partial['dirty_regions']}个, {partial['dirty_bytes']}字节")
    print(f"  全帧: {partial['full_frame_bytes']}字节")
    print(f"  压缩比: {partial['compression_ratio']:.2%}")
    print(f"  加速比: {partial['speedup']:.2f}x")

    # 3. DMA带宽对比
    print(f"\n[DMA带宽对比]")
    for bw in [8, 16, 32]:
        for clk in [50, 100, 200]:
            d = DMAConfig(bus_width=bw, clock_mhz=clk)
            fb = 320 * 240 * 2  # RGB565
            t = d.transfer_time_us(fb)
            print(f"  {bw}bit @{clk}MHz -> {d.bandwidth_mbps:.0f}MB/s, 帧传输: {t:.1f}us")

    # 4. 颜色转换
    print(f"\n[颜色转换]")
    conv = tft.color_conversion_test()
    print(f"  输入RGB: {conv['input_rgb']}")
    print(f"  RGB565: {conv['rgb565_hex']}")
    print(f"  回转RGB: {conv['roundtrip_rgb']}")
    print(f"  色差: {conv['color_error']}")

    print("\n✅ TFT显示仿真完成")


if __name__ == "__main__":
    demo()
