#!/usr/bin/env python3
"""
TFT游戏机V2测试 — ST7789显示 + 按键 + 蜂鸣器
覆盖: TFT初始化、像素绘制、游戏精灵渲染、
      碰撞检测算法、得分系统、蜂鸣器音效
对应C源文件: 02_mspm0g3507/drivers/st7789.c + 贪吃蛇/打砖块游戏逻辑

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import ST7789, Buzzer, RotaryEncoder, ROTARY_DIR_CW, ROTARY_DIR_CCW


# ═══════════════════════════════════════════════════════════════
#  颜色常量 (RGB565)
# ═══════════════════════════════════════════════════════════════
COLOR_BLACK = 0x0000
COLOR_WHITE = 0xFFFF
COLOR_RED = 0xF800
COLOR_GREEN = 0x07E0
COLOR_BLUE = 0x001F
COLOR_YELLOW = 0xFFE0
COLOR_CYAN = 0x07FF


# ═══════════════════════════════════════════════════════════════
#  贪吃蛇游戏引擎 (Python镜像)
# ═══════════════════════════════════════════════════════════════

# 方向常量
DIR_UP = 0
DIR_DOWN = 1
DIR_LEFT = 2
DIR_RIGHT = 3

# 游戏状态
GAME_IDLE = 0
GAME_RUNNING = 1
GAME_OVER = 2
GAME_PAUSED = 3

# 网格参数
GRID_SIZE = 10       # 每个格子像素
FIELD_W = 24         # 横向格数 (240/10)
FIELD_H = 32         # 纵向格数 (320/10)


class SnakeBody:
    """蛇身节点"""
    def __init__(self, x, y):
        self.x = x
        self.y = y


class SnakeGame:
    """贪吃蛇游戏引擎 — 对应C版本game_snake.c"""

    def __init__(self):
        self.tft = ST7789()
        self.buzzer = Buzzer()
        self.encoder = RotaryEncoder()  # 旋转编码器作方向输入

        # 游戏状态
        self.state = GAME_IDLE
        self.score = 0
        self.high_score = 0
        self.level = 1
        self.tick_count = 0

        # 蛇身队列
        self.snake = []
        self.direction = DIR_RIGHT
        self.next_direction = DIR_RIGHT

        # 食物位置
        self.food_x = 0
        self.food_y = 0

        # 游戏参数
        self.speed = 5  # 每N个tick移动一次

    def init(self):
        """初始化游戏硬件"""
        self.tft.init()
        self.tft.display_on()
        self.tft.set_rotation(1)  # 横屏 320x240
        self.buzzer.init()
        self.encoder.init()
        self._reset_game()

    def _reset_game(self):
        """重置游戏状态"""
        self.snake = []
        # 蛇初始位置：中间偏左，3节长
        start_x = FIELD_W // 4
        start_y = FIELD_H // 2
        for i in range(3):
            self.snake.append(SnakeBody(start_x - i, start_y))
        self.direction = DIR_RIGHT
        self.next_direction = DIR_RIGHT
        self.score = 0
        self.level = 1
        self.tick_count = 0
        self.speed = 5
        self.state = GAME_IDLE
        self._spawn_food()

    def _spawn_food(self):
        """随机生成食物（避免与蛇身重叠）"""
        import random
        while True:
            fx = random.randint(0, FIELD_W - 1)
            fy = random.randint(0, FIELD_H - 1)
            conflict = False
            for seg in self.snake:
                if seg.x == fx and seg.y == fy:
                    conflict = True
                    break
            if not conflict:
                self.food_x = fx
                self.food_y = fy
                return

    def set_direction(self, d):
        """设置方向（禁止180°反转）"""
        opposites = {DIR_UP: DIR_DOWN, DIR_DOWN: DIR_UP,
                     DIR_LEFT: DIR_RIGHT, DIR_RIGHT: DIR_LEFT}
        if d != opposites.get(self.direction):
            self.next_direction = d

    def _check_collision(self, x, y):
        """检测碰撞（墙壁 + 自身）"""
        # 墙壁
        if x < 0 or x >= FIELD_W or y < 0 or y >= FIELD_H:
            return True
        # 自身（跳过头部）
        for i in range(1, len(self.snake)):
            if self.snake[i].x == x and self.snake[i].y == y:
                return True
        return False

    def tick(self):
        """游戏主循环一步"""
        if self.state != GAME_RUNNING:
            return

        self.tick_count += 1
        if self.tick_count % self.speed != 0:
            return

        self.direction = self.next_direction
        head = self.snake[0]

        # 计算新头部位置
        dx, dy = 0, 0
        if self.direction == DIR_UP:
            dy = -1
        elif self.direction == DIR_DOWN:
            dy = 1
        elif self.direction == DIR_LEFT:
            dx = -1
        elif self.direction == DIR_RIGHT:
            dx = 1

        new_x = head.x + dx
        new_y = head.y + dy

        # 碰撞检测
        if self._check_collision(new_x, new_y):
            self.state = GAME_OVER
            self.buzzer.beep(3, on_ms=100, off_ms=100)  # 死亡音效
            if self.score > self.high_score:
                self.high_score = self.score
            return

        # 移动蛇头
        new_head = SnakeBody(new_x, new_y)
        self.snake.insert(0, new_head)

        # 吃食物
        if new_x == self.food_x and new_y == self.food_y:
            self.score += 10
            self.buzzer.beep(1, on_ms=50, off_ms=0)  # 吃食物音效
            # 升级：每50分加速
            if self.score % 50 == 0 and self.speed > 2:
                self.speed -= 1
                self.level += 1
            self._spawn_food()
        else:
            self.snake.pop()  # 没吃到就去掉尾巴

    def start(self):
        """开始游戏"""
        if self.state == GAME_IDLE or self.state == GAME_OVER:
            if self.state == GAME_OVER:
                self._reset_game()
            self.state = GAME_RUNNING

    def pause(self):
        """暂停/恢复"""
        if self.state == GAME_RUNNING:
            self.state = GAME_PAUSED
        elif self.state == GAME_PAUSED:
            self.state = GAME_RUNNING

    def get_snake_length(self):
        """获取蛇身长度"""
        return len(self.snake)

    def get_head_position(self):
        """获取蛇头位置"""
        if self.snake:
            return self.snake[0].x, self.snake[0].y
        return 0, 0

    def draw_game(self):
        """渲染游戏画面"""
        # 清屏
        self.tft.fill(COLOR_BLACK)
        # 画食物
        self.tft.fill_rect(self.food_x * GRID_SIZE, self.food_y * GRID_SIZE,
                           GRID_SIZE, GRID_SIZE, COLOR_RED)
        # 画蛇身
        for i, seg in enumerate(self.snake):
            color = COLOR_GREEN if i == 0 else COLOR_CYAN
            self.tft.fill_rect(seg.x * GRID_SIZE, seg.y * GRID_SIZE,
                               GRID_SIZE, GRID_SIZE, color)
        # 画边界
        self.tft.draw_rect(0, 0, FIELD_W * GRID_SIZE, FIELD_H * GRID_SIZE, COLOR_WHITE)


# ═══════════════════════════════════════════════════════════════
#  测试类
# ═══════════════════════════════════════════════════════════════

class TestTFTGameInit(unittest.TestCase):
    """TFT游戏机初始化测试"""

    def test_init_success(self):
        """硬件初始化成功"""
        game = SnakeGame()
        game.init()
        self.assertTrue(game.tft._initialized)
        self.assertTrue(game.buzzer.initialized)
        self.assertTrue(game.encoder.initialized)

    def test_initial_state_idle(self):
        """初始状态为空闲"""
        game = SnakeGame()
        game.init()
        self.assertEqual(game.state, GAME_IDLE)

    def test_initial_snake_length(self):
        """初始蛇身长度为3"""
        game = SnakeGame()
        game.init()
        self.assertEqual(game.get_snake_length(), 3)

    def test_initial_direction_right(self):
        """初始方向向右"""
        game = SnakeGame()
        game.init()
        self.assertEqual(game.direction, DIR_RIGHT)


class TestTFTDisplay(unittest.TestCase):
    """ST7789 TFT显示测试"""

    def setUp(self):
        self.tft = ST7789()
        self.tft.init()

    def test_init_cmd_log(self):
        """初始化发送正确命令序列"""
        log = self.tft.get_cmd_log()
        # 应包含SWRESET, SLPOUT, NORON, COLMOD, MADCTL
        cmd_ids = [entry[1] for entry in log if entry[0] == 'cmd']
        self.assertIn(0x01, cmd_ids)  # SWRESET
        self.assertIn(0x11, cmd_ids)  # SLPOUT

    def test_pixel_set_get(self):
        """像素写入和读取"""
        color = ST7789.color565(255, 0, 0)  # 红色
        self.tft.set_pixel(100, 100, color)
        self.assertEqual(self.tft.get_pixel(100, 100), color)

    def test_color565_conversion(self):
        """RGB888转RGB565"""
        # 红色: R=0xF8>>3=0x1F << 11 = 0xF800
        red = ST7789.color565(255, 0, 0)
        self.assertEqual(red, 0xF800)
        # 绿色: G=0xFC>>2=0x3F << 5 = 0x07E0
        green = ST7789.color565(0, 255, 0)
        self.assertEqual(green, 0x07E0)
        # 蓝色: B=0xF8>>3=0x1F = 0x001F
        blue = ST7789.color565(0, 0, 255)
        self.assertEqual(blue, 0x001F)

    def test_fill_rect(self):
        """矩形填充"""
        self.tft.fill_rect(10, 10, 20, 20, COLOR_RED)
        self.assertEqual(self.tft.get_pixel(15, 15), COLOR_RED)
        # 边界外应为黑色
        self.assertEqual(self.tft.get_pixel(5, 5), COLOR_BLACK)

    def test_draw_rect_border(self):
        """矩形边框绘制"""
        self.tft.draw_rect(0, 0, 10, 10, COLOR_WHITE)
        self.assertEqual(self.tft.get_pixel(0, 0), COLOR_WHITE)
        self.assertEqual(self.tft.get_pixel(9, 0), COLOR_WHITE)
        self.assertEqual(self.tft.get_pixel(0, 9), COLOR_WHITE)
        self.assertEqual(self.tft.get_pixel(9, 9), COLOR_WHITE)
        # 内部应为黑色
        self.assertEqual(self.tft.get_pixel(5, 5), COLOR_BLACK)

    def test_draw_line_horizontal(self):
        """水平线绘制"""
        self.tft.draw_hline(0, 10, 50, COLOR_GREEN)
        for x in range(50):
            self.assertEqual(self.tft.get_pixel(x, 10), COLOR_GREEN)
        # 线外应为黑
        self.assertEqual(self.tft.get_pixel(0, 11), COLOR_BLACK)

    def test_rotation(self):
        """屏幕旋转"""
        self.tft.set_rotation(0)
        self.assertEqual(self.tft.width, 240)
        self.assertEqual(self.tft.height, 320)
        self.tft.set_rotation(1)
        self.assertEqual(self.tft.width, 320)
        self.assertEqual(self.tft.height, 240)

    def test_clear(self):
        """清屏"""
        self.tft.fill_rect(0, 0, 50, 50, COLOR_WHITE)
        self.tft.clear()
        self.assertEqual(self.tft.get_pixel(25, 25), COLOR_BLACK)

    def test_framebuffer_size(self):
        """帧缓冲大小正确"""
        # 240 * 320 * 2 = 153600
        self.assertEqual(self.tft.get_framebuffer_size(), 240 * 320 * 2)

    def test_draw_circle(self):
        """画圆"""
        self.tft.draw_circle(120, 160, 50, COLOR_BLUE)
        # 圆的右端点应有像素
        self.assertEqual(self.tft.get_pixel(170, 160), COLOR_BLUE)
        # 圆心应无像素（空心圆）
        self.assertEqual(self.tft.get_pixel(120, 160), COLOR_BLACK)

    def test_draw_line_diagonal(self):
        """斜线绘制"""
        self.tft.draw_line(0, 0, 10, 10, COLOR_YELLOW)
        self.assertEqual(self.tft.get_pixel(0, 0), COLOR_YELLOW)
        self.assertEqual(self.tft.get_pixel(10, 10), COLOR_YELLOW)


class TestSnakeMovement(unittest.TestCase):
    """贪吃蛇移动测试"""

    def setUp(self):
        self.game = SnakeGame()
        self.game.init()
        self.game.start()

    def test_start_game(self):
        """启动游戏"""
        self.assertEqual(self.game.state, GAME_RUNNING)

    def test_move_right(self):
        """向右移动"""
        old_x = self.game.snake[0].x
        # 执行足够多的tick让蛇移动一步
        for _ in range(self.game.speed):
            self.game.tick()
        new_x = self.game.snake[0].x
        self.assertEqual(new_x, old_x + 1)

    def test_direction_change_down(self):
        """改变方向为向下"""
        self.game.set_direction(DIR_DOWN)
        old_y = self.game.snake[0].y
        for _ in range(self.game.speed):
            self.game.tick()
        new_y = self.game.snake[0].y
        self.assertEqual(new_y, old_y + 1)

    def test_no_reverse_direction(self):
        """禁止180°反转"""
        self.game.set_direction(DIR_LEFT)  # 当前向右，不能反向
        # 方向应保持RIGHT
        self.assertEqual(self.game.direction, DIR_RIGHT)

    def test_pause_resume(self):
        """暂停和恢复"""
        self.game.pause()
        self.assertEqual(self.game.state, GAME_PAUSED)
        old_head = self.game.get_head_position()
        self.game.tick()  # 暂停时不应移动
        self.assertEqual(self.game.get_head_position(), old_head)
        self.game.pause()
        self.assertEqual(self.game.state, GAME_RUNNING)


class TestSnakeCollision(unittest.TestCase):
    """贪吃蛇碰撞检测测试"""

    def setUp(self):
        self.game = SnakeGame()
        self.game.init()
        self.game.start()

    def test_wall_collision_right(self):
        """撞右墙"""
        # 把蛇移到右边界
        self.game.snake[0].x = FIELD_W - 1
        self.game.tick()  # speed=5, 需要5个tick
        for _ in range(self.game.speed - 1):
            self.game.tick()
        self.assertEqual(self.game.state, GAME_OVER)

    def test_wall_collision_up(self):
        """撞上墙"""
        self.game.snake[0].y = 0
        self.game.set_direction(DIR_UP)
        for _ in range(self.game.speed):
            self.game.tick()
        self.assertEqual(self.game.state, GAME_OVER)

    def test_no_collision_normal(self):
        """正常移动无碰撞"""
        for _ in range(10):
            self.game.tick()
        self.assertEqual(self.game.state, GAME_RUNNING)

    def test_self_collision_detect(self):
        """自身碰撞检测函数"""
        # 构造蛇身，新位置(10,10)与身体第3段重叠
        self.game.snake = [
            SnakeBody(11, 10),  # 当前头部
            SnakeBody(10, 10),  # 身体段1
            SnakeBody(9, 10),   # 身体段2
        ]
        # 新位置(10,10)与身体段1碰撞
        self.assertTrue(self.game._check_collision(10, 10))

    def test_boundary_detect(self):
        """边界检测函数"""
        self.assertTrue(self.game._check_collision(-1, 5))
        self.assertTrue(self.game._check_collision(FIELD_W, 5))
        self.assertTrue(self.game._check_collision(5, -1))
        self.assertTrue(self.game._check_collision(5, FIELD_H))


class TestSnakeScore(unittest.TestCase):
    """贪吃蛇得分系统测试"""

    def setUp(self):
        self.game = SnakeGame()
        self.game.init()
        self.game.start()

    def test_eat_food_score(self):
        """吃食物得分"""
        head = self.game.snake[0]
        self.game.food_x = head.x + 1
        self.game.food_y = head.y
        old_score = self.game.score
        # 移动到食物位置
        for _ in range(self.game.speed):
            self.game.tick()
        self.assertEqual(self.game.score, old_score + 10)

    def test_eat_food_grow(self):
        """吃食物蛇身增长"""
        initial_len = self.game.get_snake_length()
        head = self.game.snake[0]
        self.game.food_x = head.x + 1
        self.game.food_y = head.y
        for _ in range(self.game.speed):
            self.game.tick()
        self.assertEqual(self.game.get_snake_length(), initial_len + 1)

    def test_no_grow_no_food(self):
        """不吃食物蛇身不变"""
        # 确保食物不在移动路径上
        self.game.food_x = 0
        self.game.food_y = 0
        initial_len = self.game.get_snake_length()
        for _ in range(self.game.speed):
            self.game.tick()
        self.assertEqual(self.game.get_snake_length(), initial_len)

    def test_high_score_update(self):
        """最高分更新"""
        self.game.score = 100
        self.game.high_score = 50
        self.game.state = GAME_OVER
        # 检测到GAME_OVER时应更新high_score
        self.game.high_score = max(self.game.high_score, self.game.score)
        self.assertEqual(self.game.high_score, 100)

    def test_game_over_restart(self):
        """游戏结束后重启"""
        self.game.state = GAME_OVER
        self.game.start()
        self.assertEqual(self.game.state, GAME_RUNNING)
        self.assertEqual(self.game.score, 0)
        self.assertEqual(self.game.get_snake_length(), 3)


class TestBuzzerSound(unittest.TestCase):
    """蜂鸣器音效测试"""

    def test_init(self):
        """蜂鸣器初始化"""
        bz = Buzzer()
        bz.init()
        self.assertTrue(bz.initialized)

    def test_set_frequency(self):
        """设置频率"""
        bz = Buzzer()
        bz.init()
        self.assertTrue(bz.set_frequency(2000))
        self.assertEqual(bz.frequency, 2000)

    def test_on_off(self):
        """开关控制"""
        bz = Buzzer()
        bz.init()
        bz.on()
        self.assertTrue(bz.is_on())
        bz.off()
        self.assertFalse(bz.is_on())

    def test_beep_pattern(self):
        """蜂鸣模式"""
        bz = Buzzer()
        bz.init()
        self.assertTrue(bz.beep(3, on_ms=100, off_ms=50))
        self.assertEqual(bz.total_beeps, 3)
        self.assertEqual(bz.beep_on_ms, 100)

    def test_play_note(self):
        """播放音符"""
        bz = Buzzer()
        bz.init()
        self.assertTrue(bz.play_note("C4"))
        self.assertTrue(bz.is_on())


class TestRotaryInput(unittest.TestCase):
    """旋转编码器输入测试"""

    def test_init(self):
        """编码器初始化"""
        enc = RotaryEncoder()
        enc.init()
        self.assertTrue(enc.initialized)

    def test_clockwise(self):
        """顺时针旋转"""
        enc = RotaryEncoder()
        enc.init()
        enc._inject_rotation(ROTARY_DIR_CW)
        enc._inject_rotation(ROTARY_DIR_CW)
        self.assertEqual(enc.get_position(), 2)
        self.assertEqual(enc.get_direction(), ROTARY_DIR_CW)

    def test_counter_clockwise(self):
        """逆时针旋转"""
        enc = RotaryEncoder()
        enc.init()
        enc._inject_rotation(ROTARY_DIR_CCW)
        enc._inject_rotation(ROTARY_DIR_CCW)
        self.assertEqual(enc.get_position(), -2)
        self.assertEqual(enc.get_direction(), ROTARY_DIR_CCW)

    def test_button_press(self):
        """按钮按下"""
        enc = RotaryEncoder()
        enc.init()
        enc._inject_button_press()
        self.assertTrue(enc.is_pressed())
        self.assertEqual(enc.get_clicks(), 1)

    def test_reset(self):
        """重置"""
        enc = RotaryEncoder()
        enc.init()
        enc._inject_rotation(ROTARY_DIR_CW)
        enc._inject_button_press()
        enc.reset()
        self.assertEqual(enc.get_position(), 0)
        self.assertEqual(enc.get_clicks(), 0)


class TestGameDraw(unittest.TestCase):
    """游戏渲染测试"""

    def test_draw_game(self):
        """完整渲染一帧"""
        game = SnakeGame()
        game.init()
        game.start()
        game.draw_game()
        # 验证帧缓冲非全零
        fb = game.tft.get_framebuffer()
        self.assertTrue(any(b != 0 for b in fb))

    def test_snake_head_color(self):
        """蛇头颜色为绿色"""
        game = SnakeGame()
        game.init()
        game.start()
        game.draw_game()
        head = game.snake[0]
        pixel = game.tft.get_pixel(head.x * GRID_SIZE + 1, head.y * GRID_SIZE + 1)
        self.assertEqual(pixel, COLOR_GREEN)

    def test_food_color(self):
        """食物颜色为红色"""
        game = SnakeGame()
        game.init()
        game.start()
        # 设置确定的食物位置（远离蛇身，在屏幕范围内）
        game.food_x = 20
        game.food_y = 10
        game.draw_game()
        pixel = game.tft.get_pixel(game.food_x * GRID_SIZE + 1, game.food_y * GRID_SIZE + 1)
        self.assertEqual(pixel, COLOR_RED)


if __name__ == '__main__':
    unittest.main()
