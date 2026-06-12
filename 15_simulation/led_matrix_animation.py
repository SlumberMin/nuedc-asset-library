#!/usr/bin/env python3
"""
LED点阵动画仿真
功能：滚动文字、图案显示、动画帧生成
适用：电赛LED点阵/显示屏类题目
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.animation import FuncAnimation

# ============== 点阵参数 ==============
MATRIX_ROWS = 16       # 点阵行数
MATRIX_COLS = 32       # 点阵列数
COLOR_ON = '#FF3300'   # LED点亮颜色
COLOR_OFF = '#1a0500'  # LED熄灭颜色
FPS = 15               # 动画帧率

# ============== 5×7 ASCII字模 ==============
FONT_5x7 = {
    ' ': [0x00, 0x00, 0x00, 0x00, 0x00],
    '0': [0x3E, 0x51, 0x49, 0x45, 0x3E],
    '1': [0x00, 0x42, 0x7F, 0x40, 0x00],
    '2': [0x42, 0x61, 0x51, 0x49, 0x46],
    '3': [0x21, 0x41, 0x45, 0x4B, 0x31],
    '4': [0x18, 0x14, 0x12, 0x7F, 0x10],
    '5': [0x27, 0x45, 0x45, 0x45, 0x39],
    '6': [0x3C, 0x4A, 0x49, 0x49, 0x30],
    '7': [0x01, 0x71, 0x09, 0x05, 0x03],
    '8': [0x36, 0x49, 0x49, 0x49, 0x36],
    '9': [0x06, 0x49, 0x49, 0x29, 0x1E],
    'A': [0x7E, 0x11, 0x11, 0x11, 0x7E],
    'B': [0x7F, 0x49, 0x49, 0x49, 0x36],
    'C': [0x3E, 0x41, 0x41, 0x41, 0x22],
    'D': [0x7F, 0x41, 0x41, 0x22, 0x1C],
    'E': [0x7F, 0x49, 0x49, 0x49, 0x41],
    'F': [0x7F, 0x09, 0x09, 0x09, 0x01],
    'G': [0x3E, 0x41, 0x49, 0x49, 0x7A],
    'H': [0x7F, 0x08, 0x08, 0x08, 0x7F],
    'I': [0x00, 0x41, 0x7F, 0x41, 0x00],
    'J': [0x20, 0x40, 0x41, 0x3F, 0x01],
    'K': [0x7F, 0x08, 0x14, 0x22, 0x41],
    'L': [0x7F, 0x40, 0x40, 0x40, 0x40],
    'M': [0x7F, 0x02, 0x0C, 0x02, 0x7F],
    'N': [0x7F, 0x04, 0x08, 0x10, 0x7F],
    'O': [0x3E, 0x41, 0x41, 0x41, 0x3E],
    'P': [0x7F, 0x09, 0x09, 0x09, 0x06],
    'Q': [0x3E, 0x41, 0x51, 0x21, 0x5E],
    'R': [0x7F, 0x09, 0x19, 0x29, 0x46],
    'S': [0x46, 0x49, 0x49, 0x49, 0x31],
    'T': [0x01, 0x01, 0x7F, 0x01, 0x01],
    'U': [0x3F, 0x40, 0x40, 0x40, 0x3F],
    'V': [0x1F, 0x20, 0x40, 0x20, 0x1F],
    'W': [0x3F, 0x40, 0x38, 0x40, 0x3F],
    'X': [0x63, 0x14, 0x08, 0x14, 0x63],
    'Y': [0x07, 0x08, 0x70, 0x08, 0x07],
    'Z': [0x61, 0x51, 0x49, 0x45, 0x43],
    ':': [0x00, 0x36, 0x36, 0x00, 0x00],
    '.': [0x00, 0x40, 0x00, 0x00, 0x00],
    '-': [0x08, 0x08, 0x08, 0x08, 0x08],
    '+': [0x08, 0x08, 0x3E, 0x08, 0x08],
    '!': [0x00, 0x00, 0x6F, 0x00, 0x00],
    'H': [0x7F, 0x08, 0x08, 0x08, 0x7F],
    'e': [0x38, 0x54, 0x54, 0x54, 0x18],
    'l': [0x00, 0x41, 0x7F, 0x40, 0x00],
    'o': [0x38, 0x44, 0x44, 0x44, 0x38],
}

# 常用图案定义
PATTERNS = {
    'heart': np.array([
        [0,0,1,1,0,0,1,1,0,0],
        [0,1,1,1,1,1,1,1,1,0],
        [1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1],
        [0,1,1,1,1,1,1,1,1,0],
        [0,0,1,1,1,1,1,1,0,0],
        [0,0,0,1,1,1,1,0,0,0],
        [0,0,0,0,1,1,0,0,0,0],
    ]),
    'smile': np.array([
        [0,0,1,1,1,1,1,1,0,0],
        [0,1,0,0,0,0,0,0,1,0],
        [1,0,1,0,0,0,0,1,0,1],
        [1,0,0,0,0,0,0,0,0,1],
        [1,0,1,0,0,0,0,1,0,1],
        [1,0,0,1,1,1,1,0,0,1],
        [0,1,0,0,0,0,0,0,1,0],
        [0,0,1,1,1,1,1,1,0,0],
    ]),
    'arrow_right': np.array([
        [0,0,0,0,1,0,0,0],
        [0,0,0,1,1,0,0,0],
        [0,0,1,1,1,0,0,0],
        [1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1],
        [0,0,1,1,1,0,0,0],
        [0,0,0,1,1,0,0,0],
        [0,0,0,0,1,0,0,0],
    ]),
}


def text_to_bitmap(text, rows=7):
    """将文本字符串转为点阵位图（宽=字符数×6）"""
    char_width = 6  # 5列字模 + 1列间距
    total_width = len(text) * char_width
    bitmap = np.zeros((rows, total_width), dtype=int)

    for i, ch in enumerate(text.upper()):
        glyph = FONT_5x7.get(ch, FONT_5x7.get(' ', [0]*5))
        col_start = i * char_width
        for c in range(5):
            byte = glyph[c]
            for r in range(rows):
                if byte & (1 << r):
                    bitmap[r][col_start + c] = 1
    return bitmap


def place_pattern(matrix, pattern, row_offset, col_offset):
    """在矩阵上放置图案"""
    r, c = pattern.shape
    for i in range(r):
        for j in range(c):
            ri = row_offset + i
            ci = col_offset + j
            if 0 <= ri < matrix.shape[0] and 0 <= ci < matrix.shape[1]:
                matrix[ri][ci] = pattern[i][j]
    return matrix


def render_matrix(matrix, ax, title=""):
    """渲染LED点阵到matplotlib axes"""
    rows, cols = matrix.shape
    display = np.zeros((rows, cols, 3))
    for r in range(rows):
        for c in range(cols):
            if matrix[r][c]:
                display[r][c] = mcolors.to_rgb(COLOR_ON)
            else:
                display[r][c] = mcolors.to_rgb(COLOR_OFF)
    ax.clear()
    ax.imshow(display, interpolation='nearest', aspect='equal')
    ax.set_title(title, fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    # 画网格线
    for x in range(cols + 1):
        ax.axhline(x - 0.5, color='#333', linewidth=0.3)
    for y in range(rows + 1):
        ax.axvline(y - 0.5, color='#333', linewidth=0.3)


def animation_scroll_text():
    """滚动文字动画"""
    text = "HELLO WORLD  2026 ELECTRONICS CONTEST"
    bitmap = text_to_bitmap(text)
    text_rows, text_cols = bitmap.shape

    # 创建帧序列
    n_frames = text_cols + MATRIX_COLS
    frames = []
    for f in range(n_frames):
        matrix = np.zeros((MATRIX_ROWS, MATRIX_COLS), dtype=int)
        offset = f - MATRIX_COLS
        row_off = (MATRIX_ROWS - text_rows) // 2
        for r in range(text_rows):
            for c in range(text_cols):
                mc = c - offset
                if 0 <= mc < MATRIX_COLS and 0 <= (row_off + r) < MATRIX_ROWS:
                    matrix[row_off + r][mc] = bitmap[r][c]
        frames.append(matrix.copy())
    return frames


def animation_pattern_demo():
    """图案动画：心形跳动 + 箭头"""
    frames = []
    for f in range(60):
        matrix = np.zeros((MATRIX_ROWS, MATRIX_COLS), dtype=int)
        pattern_name = 'heart' if (f // 15) % 2 == 0 else 'smile'
        pattern = PATTERNS[pattern_name]
        # 居中
        r_off = (MATRIX_ROWS - pattern.shape[0]) // 2
        c_off = (MATRIX_COLS - pattern.shape[1]) // 2
        place_pattern(matrix, pattern, r_off, c_off)
        frames.append(matrix.copy())
    return frames


def animation_marquee_border():
    """边框跑马灯动画"""
    frames = []
    for f in range(64):
        matrix = np.zeros((MATRIX_ROWS, MATRIX_COLS), dtype=int)
        # 顶部边框
        for c in range(MATRIX_COLS):
            if (c + f) % 4 < 2:
                matrix[0][c] = 1
                matrix[1][c] = 1
        # 底部边框
        for c in range(MATRIX_COLS):
            if (c + f) % 4 < 2:
                matrix[MATRIX_ROWS-1][c] = 1
                matrix[MATRIX_ROWS-2][c] = 1
        # 左边框
        for r in range(MATRIX_ROWS):
            if (r + f) % 4 < 2:
                matrix[r][0] = 1
                matrix[r][1] = 1
        # 右边框
        for r in range(MATRIX_ROWS):
            if (r + f) % 4 < 2:
                matrix[r][MATRIX_COLS-1] = 1
                matrix[r][MATRIX_COLS-2] = 1
        # 中心放文字
        text_bitmap = text_to_bitmap("DIANSAI")
        tr, tc = text_bitmap.shape
        r_off = (MATRIX_ROWS - tr) // 2
        c_off = (MATRIX_COLS - tc) // 2
        for r in range(tr):
            for c in range(tc):
                mr = r_off + r
                mc = c_off + c
                if 0 <= mr < MATRIX_ROWS and 0 <= mc < MATRIX_COLS:
                    matrix[mr][mc] = text_bitmap[r][c]
        frames.append(matrix.copy())
    return frames


def animation_rain():
    """雨滴下落动画"""
    np.random.seed(42)
    frames = []
    drops = []  # (col, current_row)

    for f in range(80):
        matrix = np.zeros((MATRIX_ROWS, MATRIX_COLS), dtype=int)
        # 随机新增雨滴
        if np.random.random() < 0.3:
            drops.append([np.random.randint(0, MATRIX_COLS), 0])
        # 更新雨滴位置
        new_drops = []
        for col, row in drops:
            if row < MATRIX_ROWS:
                matrix[row][col] = 1
                if row > 0:
                    matrix[row-1][col] = 1  # 雨滴尾巴
                new_drops.append([col, row + 1])
        drops = new_drops
        frames.append(matrix.copy())
    return frames


def run_simulation():
    print("=" * 60)
    print("LED点阵动画仿真系统")
    print("=" * 60)
    print(f"点阵尺寸: {MATRIX_ROWS}×{MATRIX_COLS}")
    print(f"动画帧率: {FPS} fps")

    # ---- 静态展示 ----
    fig_static, axes = plt.subplots(2, 3, figsize=(16, 10))
    fig_static.suptitle("LED点阵动画仿真", fontsize=16, fontweight='bold')

    # 1. 文字位图
    text = "HELLO"
    bitmap = text_to_bitmap(text)
    matrix = np.zeros((MATRIX_ROWS, MATRIX_COLS), dtype=int)
    r_off = (MATRIX_ROWS - bitmap.shape[0]) // 2
    c_off = (MATRIX_COLS - bitmap.shape[1]) // 2
    place_pattern(matrix, bitmap, r_off, c_off)
    render_matrix(matrix, axes[0][0], f'文字: "{text}"')

    # 2. 心形图案
    matrix = np.zeros((MATRIX_ROWS, MATRIX_COLS), dtype=int)
    place_pattern(matrix, PATTERNS['heart'],
                  (MATRIX_ROWS - 8) // 2, (MATRIX_COLS - 10) // 2)
    render_matrix(matrix, axes[0][1], '心形图案')

    # 3. 笑脸图案
    matrix = np.zeros((MATRIX_ROWS, MATRIX_COLS), dtype=int)
    place_pattern(matrix, PATTERNS['smile'],
                  (MATRIX_ROWS - 8) // 2, (MATRIX_COLS - 10) // 2)
    render_matrix(matrix, axes[0][2], '笑脸图案')

    # 4. 边框跑马灯（静态帧）
    border_frames = animation_marquee_border()
    render_matrix(border_frames[0], axes[1][0], '边框跑马灯 (帧0)')

    # 5. 滚动文字（中间帧）
    scroll_frames = animation_scroll_text()
    mid = len(scroll_frames) // 2
    render_matrix(scroll_frames[mid], axes[1][1], f'滚动文字 (帧{mid}/{len(scroll_frames)})')

    # 6. 雨滴动画（静态帧）
    rain_frames = animation_rain()
    render_matrix(rain_frames[40], axes[1][2], '雨滴动画 (帧40)')

    plt.tight_layout()
    plt.savefig('led_matrix_static.png', dpi=150, bbox_inches='tight')
    print("静态图像已保存: led_matrix_static.png")

    # ---- 生成所有动画帧并保存关键信息 ----
    all_anims = {
        '滚动文字': scroll_frames,
        '边框跑马灯': border_frames,
        '雨滴动画': rain_frames,
    }

    for name, frames in all_anims.items():
        print(f"\n[{name}] 共 {len(frames)} 帧, "
              f"时长 {len(frames)/FPS:.1f}s @ {FPS}fps")
        # 统计LED点亮率
        rates = []
        for f in frames:
            rate = np.sum(f) / f.size * 100
            rates.append(rate)
        print(f"  LED平均点亮率: {np.mean(rates):.1f}%  "
              f"最大: {np.max(rates):.1f}%  最小: {np.min(rates):.1f}%")

    # ---- 生成动画帧序列图 ----
    fig_seq = plt.figure(figsize=(18, 8))
    fig_seq.suptitle("LED点阵动画帧序列", fontsize=14, fontweight='bold')

    show_frames = [0, 10, 20, 30, 40, 50]
    for i, fi in enumerate(show_frames):
        if fi < len(scroll_frames):
            ax = fig_seq.add_subplot(2, 6, i + 1)
            render_matrix(scroll_frames[fi], ax, f'滚动 帧{fi}')

    for i, fi in enumerate(show_frames):
        if fi < len(rain_frames):
            ax = fig_seq.add_subplot(2, 6, 7 + i)
            render_matrix(rain_frames[fi], ax, f'雨滴 帧{fi}')

    plt.tight_layout()
    plt.savefig('led_matrix_frames.png', dpi=150, bbox_inches='tight')
    print("帧序列图已保存: led_matrix_frames.png")

    # ---- 硬件映射说明 ----
    print("\n" + "=" * 60)
    print("硬件映射参考 (STM32 + LED点阵)")
    print("=" * 60)
    print("行扫描: GPIO → 74HC138(3-8译码) → 74HC245(驱动)")
    print("列数据: SPI → 74HC595(移位寄存器) → ULN2803(驱动)")
    print("扫描方式: 逐行扫描, 每行显示 1~2 ms")
    print(f"刷新率: {MATRIX_ROWS}行 × 2ms = {MATRIX_ROWS*2}ms → {1000/(MATRIX_ROWS*2):.0f} Hz")
    print("编程要点: DMA传输 + 定时器中断 + 双缓冲")

    plt.show()


if __name__ == '__main__':
    run_simulation()
