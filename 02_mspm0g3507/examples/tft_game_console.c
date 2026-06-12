/**
 * @file tft_game_console.c
 * @brief TFT游戏机 - ST7789 + 按键 + 贪吃蛇/俄罗斯方块
 * @platform MSPM0G3507
 *
 * 硬件连接：
 *   ST7789 TFT (SPI):
 *     SCK  -> PA10 (SPI0_SCK)
 *     MOSI -> PA8  (SPI0_MOSI)
 *     CS   -> PA12 (GPIO)
 *     DC   -> PA13 (GPIO)
 *     RST  -> PA14 (GPIO)
 *     BL   -> PA15 (GPIO, 背光)
 *     VCC  -> 3.3V, GND -> GND
 *
 *   按键 (上拉输入, 低有效):
 *     UP    -> PB0
 *     DOWN  -> PB1
 *     LEFT  -> PB2
 *     RIGHT -> PB3
 *     A     -> PB4 (确认/旋转)
 *     B     -> PB5 (返回/暂停)
 *
 * 功能：贪吃蛇 / 俄罗斯方块，通过按键操控，TFT实时显示
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/* ===== ST7789 驱动 ===== */
#define ST7789_WIDTH  240
#define ST7789_HEIGHT 320

#define CS_LOW()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_12)
#define CS_HIGH()  DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_12)
#define DC_LOW()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13)
#define DC_HIGH()  DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_13)
#define RST_LOW()  DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14)
#define RST_HIGH() DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14)
#define BL_ON()    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_15)
#define BL_OFF()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_15)

/* 颜色定义 RGB565 */
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF
#define COLOR_ORANGE  0xFD20
#define COLOR_GRAY    0x8410

/* ===== 按键定义 ===== */
#define KEY_UP    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_0)))
#define KEY_DOWN  (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_1)))
#define KEY_LEFT  (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2)))
#define KEY_RIGHT (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3)))
#define KEY_A     (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_4)))
#define KEY_B     (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_5)))

/* 游戏区域 */
#define GAME_X_OFFSET  0
#define GAME_Y_OFFSET  40
#define CELL_SIZE      8   /* 每个格子像素 */
#define GRID_COLS      (ST7789_WIDTH / CELL_SIZE)   /* 30 */
#define GRID_ROWS      ((ST7789_HEIGHT - 80) / CELL_SIZE)  /* 30 */

/* ===== SPI 发送单字节 ===== */
static void SPI_WriteByte(uint8_t dat)
{
    DL_SPI_transmitData8(SPI0, dat);
    while (DL_SPI_isBusy(SPI0)) {}
}

/* ===== ST7789 命令/数据 ===== */
static void ST7789_WriteCmd(uint8_t cmd)
{
    DC_LOW();
    CS_LOW();
    SPI_WriteByte(cmd);
    CS_HIGH();
}

static void ST7789_WriteData(uint8_t dat)
{
    DC_HIGH();
    CS_LOW();
    SPI_WriteByte(dat);
    CS_HIGH();
}

static void ST7789_WriteData16(uint16_t dat)
{
    DC_HIGH();
    CS_LOW();
    SPI_WriteByte(dat >> 8);
    SPI_WriteByte(dat & 0xFF);
    CS_HIGH();
}

static void delay_ms(uint32_t ms)
{
    delay_cycles(ms * (CPUCLK_FREQ / 1000));
}

/* ===== ST7789 初始化 ===== */
static void ST7789_Init(void)
{
    RST_LOW();
    delay_ms(20);
    RST_HIGH();
    delay_ms(120);

    ST7789_WriteCmd(0x11);  /* Sleep Out */
    delay_ms(120);

    ST7789_WriteCmd(0x36);  /* MADCTL: 显示方向 */
    ST7789_WriteData(0x00); /* 竖屏 */

    ST7789_WriteCmd(0x3A);  /* 像素格式 */
    ST7789_WriteData(0x55); /* 16bit RGB565 */

    ST7789_WriteCmd(0x21);  /* Display Inversion On (ST7789需反色) */
    ST7789_WriteCmd(0x29);  /* Display On */
    delay_ms(20);

    BL_ON();
}

/* ===== 设置绘图窗口 ===== */
static void ST7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    ST7789_WriteCmd(0x2A);
    ST7789_WriteData16(x0);
    ST7789_WriteData16(x1);
    ST7789_WriteCmd(0x2B);
    ST7789_WriteData16(y0);
    ST7789_WriteData16(y1);
    ST7789_WriteCmd(0x2C);
}

/* ===== 填充矩形 ===== */
static void ST7789_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ST7789_SetWindow(x, y, x + w - 1, y + h - 1);
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        ST7789_WriteData16(color);
    }
}

/* ===== 清屏 ===== */
static void ST7789_Clear(uint16_t color)
{
    ST7789_FillRect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, color);
}

/* ===== 绘制单个字符 (8x16, 简易ASCII) ===== */
/* 简化：用方块模拟字符显示 */
static void Draw_CharSmall(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg)
{
    /* 简易字体: 仅显示方块标识 */
    ST7789_SetWindow(x, y, x + 7, y + 15);
    for (int i = 0; i < 128; i++) {
        ST7789_WriteData16(bg);
    }
}

/* ===== 绘制数字 (简易版) ===== */
static const uint8_t font3x5[][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111}, /* 0 */
    {0b010, 0b110, 0b010, 0b010, 0b111}, /* 1 */
    {0b111, 0b001, 0b111, 0b100, 0b111}, /* 2 */
    {0b111, 0b001, 0b111, 0b001, 0b111}, /* 3 */
    {0b101, 0b101, 0b111, 0b001, 0b001}, /* 4 */
    {0b111, 0b100, 0b111, 0b001, 0b111}, /* 5 */
    {0b111, 0b100, 0b111, 0b101, 0b111}, /* 6 */
    {0b111, 0b001, 0b001, 0b001, 0b001}, /* 7 */
    {0b111, 0b101, 0b111, 0b101, 0b111}, /* 8 */
    {0b111, 0b101, 0b111, 0b001, 0b111}, /* 9 */
};

static void Draw_Digit(uint16_t x, uint16_t y, uint8_t digit, uint16_t color, uint8_t scale)
{
    if (digit > 9) return;
    for (int row = 0; row < 5; row++) {
        for (int col = 0; col < 3; col++) {
            if (font3x5[digit][row] & (1 << (2 - col))) {
                ST7789_FillRect(x + col * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

static void Draw_Number(uint16_t x, uint16_t y, int num, uint16_t color, uint8_t scale)
{
    char buf[12];
    int len = 0;
    if (num == 0) { buf[0] = '0'; len = 1; }
    else {
        int tmp = num;
        if (tmp < 0) { tmp = -tmp; buf[len++] = '-'; }
        char rev[10]; int rlen = 0;
        while (tmp > 0) { rev[rlen++] = '0' + (tmp % 10); tmp /= 10; }
        for (int i = rlen - 1; i >= 0; i--) buf[len++] = rev[i];
    }
    uint16_t cx = x;
    for (int i = 0; i < len; i++) {
        Draw_Digit(cx, y, buf[i] - '0', color, scale);
        cx += 4 * scale;
    }
}

/* ===== 按键消抖 ===== */
static bool Key_Pressed(void)
{
    return KEY_UP || KEY_DOWN || KEY_LEFT || KEY_RIGHT || KEY_A || KEY_B;
}

static void Wait_KeyRelease(void)
{
    while (Key_Pressed()) { delay_ms(10); }
}

/* ===== 游戏1: 贪吃蛇 ===== */
typedef struct {
    int16_t x, y;
} Point;

typedef enum { DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT } Direction;

static Point snake[GRID_COLS * GRID_ROWS];
static int snake_len;
static Direction snake_dir;
static Point food;
static bool game_over;
static int score;
static uint8_t game_grid[GRID_COLS][GRID_ROWS]; /* 0=空, 1=蛇身, 2=食物 */

static void Snake_PlaceFood(void)
{
    do {
        food.x = rand() % GRID_COLS;
        food.y = rand() % GRID_ROWS;
    } while (game_grid[food.x][food.y] != 0);
    game_grid[food.x][food.y] = 2;
    /* 绘制食物 */
    uint16_t px = GAME_X_OFFSET + food.x * CELL_SIZE;
    uint16_t py = GAME_Y_OFFSET + food.y * CELL_SIZE;
    ST7789_FillRect(px + 1, py + 1, CELL_SIZE - 2, CELL_SIZE - 2, COLOR_RED);
}

static void Snake_Init(void)
{
    memset(game_grid, 0, sizeof(game_grid));
    snake_len = 3;
    snake_dir = DIR_RIGHT;
    score = 0;
    game_over = false;

    /* 蛇初始位置居中 */
    int sx = GRID_COLS / 2;
    int sy = GRID_ROWS / 2;
    for (int i = 0; i < snake_len; i++) {
        snake[i].x = sx - i;
        snake[i].y = sy;
        game_grid[snake[i].x][snake[i].y] = 1;
    }

    /* 绘制背景 */
    ST7789_Clear(COLOR_BLACK);
    /* 标题 */
    ST7789_FillRect(0, 0, ST7789_WIDTH, 38, COLOR_BLUE);
    /* 简单用方块拼 SNAKE */
    /* 分数区 */
    ST7789_FillRect(0, ST7789_HEIGHT - 38, ST7789_WIDTH, 38, COLOR_BLUE);
    Draw_Number(10, ST7789_HEIGHT - 32, score, COLOR_YELLOW, 2);

    /* 绘制蛇 */
    for (int i = 0; i < snake_len; i++) {
        uint16_t px = GAME_X_OFFSET + snake[i].x * CELL_SIZE;
        uint16_t py = GAME_Y_OFFSET + snake[i].y * CELL_SIZE;
        uint16_t c = (i == 0) ? COLOR_GREEN : COLOR_CYAN;
        ST7789_FillRect(px + 1, py + 1, CELL_SIZE - 2, CELL_SIZE - 2, c);
    }

    Snake_PlaceFood();
}

static void Snake_Update(void)
{
    /* 读取按键改变方向 */
    if (KEY_UP    && snake_dir != DIR_DOWN)  snake_dir = DIR_UP;
    if (KEY_DOWN  && snake_dir != DIR_UP)    snake_dir = DIR_DOWN;
    if (KEY_LEFT  && snake_dir != DIR_RIGHT) snake_dir = DIR_LEFT;
    if (KEY_RIGHT && snake_dir != DIR_LEFT)  snake_dir = DIR_RIGHT;

    /* 计算新头部 */
    Point newHead = snake[0];
    switch (snake_dir) {
        case DIR_UP:    newHead.y--; break;
        case DIR_DOWN:  newHead.y++; break;
        case DIR_LEFT:  newHead.x--; break;
        case DIR_RIGHT: newHead.x++; break;
    }

    /* 碰墙检测 */
    if (newHead.x < 0 || newHead.x >= GRID_COLS ||
        newHead.y < 0 || newHead.y >= GRID_ROWS) {
        game_over = true;
        return;
    }

    /* 碰自身检测 */
    if (game_grid[newHead.x][newHead.y] == 1) {
        game_over = true;
        return;
    }

    bool ate = (game_grid[newHead.x][newHead.y] == 2);

    /* 如果没吃到食物, 擦除尾部 */
    if (!ate) {
        Point tail = snake[snake_len - 1];
        game_grid[tail.x][tail.y] = 0;
        uint16_t px = GAME_X_OFFSET + tail.x * CELL_SIZE;
        uint16_t py = GAME_Y_OFFSET + tail.y * CELL_SIZE;
        ST7789_FillRect(px, py, CELL_SIZE, CELL_SIZE, COLOR_BLACK);
    } else {
        snake_len++;
        score += 10;
        /* 更新分数显示 */
        ST7789_FillRect(0, ST7789_HEIGHT - 38, 120, 38, COLOR_BLUE);
        Draw_Number(10, ST7789_HEIGHT - 32, score, COLOR_YELLOW, 2);
    }

    /* 移动蛇身 */
    for (int i = snake_len - 1; i > 0; i--) {
        snake[i] = snake[i - 1];
    }
    snake[0] = newHead;
    game_grid[newHead.x][newHead.y] = 1;

    /* 绘制新头部 */
    uint16_t px = GAME_X_OFFSET + newHead.x * CELL_SIZE;
    uint16_t py = GAME_Y_OFFSET + newHead.y * CELL_SIZE;
    ST7789_FillRect(px + 1, py + 1, CELL_SIZE - 2, CELL_SIZE - 2, COLOR_GREEN);

    if (ate) {
        Snake_PlaceFood();
    }
}

/* ===== 游戏2: 俄罗斯方块 ===== */
#define TETRIS_COLS  10
#define TETRIS_ROWS  20
#define TETRIS_CELL  14
#define TETRIS_X_OFF ((ST7789_WIDTH - TETRIS_COLS * TETRIS_CELL) / 2)
#define TETRIS_Y_OFF 30

static uint8_t tetris_board[TETRIS_COLS][TETRIS_ROWS]; /* 0=空, 1-7=方块颜色ID */
static int tetris_score;
static int tetris_level;
static int tetris_lines;
static bool tetris_over;
static int current_piece;  /* 当前方块类型 0-6 */
static int current_rot;    /* 当前旋转 0-3 */
static int piece_x, piece_y; /* 方块位置 */

/* 7种方块, 4个旋转态, 4x4网格 */
static const uint8_t tetromino[7][4][4][4] = {
    /* I */
    {{{0,0,0,0},{1,1,1,1},{0,0,0,0},{0,0,0,0}},
     {{0,0,1,0},{0,0,1,0},{0,0,1,0},{0,0,1,0}},
     {{0,0,0,0},{0,0,0,0},{1,1,1,1},{0,0,0,0}},
     {{0,1,0,0},{0,1,0,0},{0,1,0,0},{0,1,0,0}}},
    /* O */
    {{{1,1,0,0},{1,1,0,0},{0,0,0,0},{0,0,0,0}},
     {{1,1,0,0},{1,1,0,0},{0,0,0,0},{0,0,0,0}},
     {{1,1,0,0},{1,1,0,0},{0,0,0,0},{0,0,0,0}},
     {{1,1,0,0},{1,1,0,0},{0,0,0,0},{0,0,0,0}}},
    /* T */
    {{{0,1,0,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}},
     {{0,1,0,0},{0,1,1,0},{0,1,0,0},{0,0,0,0}},
     {{0,0,0,0},{1,1,1,0},{0,1,0,0},{0,0,0,0}},
     {{0,1,0,0},{1,1,0,0},{0,1,0,0},{0,0,0,0}}},
    /* S */
    {{{0,1,1,0},{1,1,0,0},{0,0,0,0},{0,0,0,0}},
     {{0,1,0,0},{0,1,1,0},{0,0,1,0},{0,0,0,0}},
     {{0,0,0,0},{0,1,1,0},{1,1,0,0},{0,0,0,0}},
     {{1,0,0,0},{1,1,0,0},{0,1,0,0},{0,0,0,0}}},
    /* Z */
    {{{1,1,0,0},{0,1,1,0},{0,0,0,0},{0,0,0,0}},
     {{0,0,1,0},{0,1,1,0},{0,1,0,0},{0,0,0,0}},
     {{0,0,0,0},{1,1,0,0},{0,1,1,0},{0,0,0,0}},
     {{0,1,0,0},{1,1,0,0},{1,0,0,0},{0,0,0,0}}},
    /* L */
    {{{0,0,1,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}},
     {{0,1,0,0},{0,1,0,0},{0,1,1,0},{0,0,0,0}},
     {{0,0,0,0},{1,1,1,0},{1,0,0,0},{0,0,0,0}},
     {{1,1,0,0},{0,1,0,0},{0,1,0,0},{0,0,0,0}}},
    /* J */
    {{{1,0,0,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}},
     {{0,1,1,0},{0,1,0,0},{0,1,0,0},{0,0,0,0}},
     {{0,0,0,0},{1,1,1,0},{0,0,1,0},{0,0,0,0}},
     {{0,1,0,0},{0,1,0,0},{1,1,0,0},{0,0,0,0}}},
};

static const uint16_t piece_colors[8] = {
    COLOR_BLACK, COLOR_CYAN, COLOR_YELLOW, COLOR_CYAN,
    COLOR_GREEN, COLOR_RED, COLOR_ORANGE, COLOR_BLUE
};

static void Tetris_DrawCell(int col, int row, uint16_t color)
{
    uint16_t px = TETRIS_X_OFF + col * TETRIS_CELL;
    uint16_t py = TETRIS_Y_OFF + row * TETRIS_CELL;
    ST7789_FillRect(px, py, TETRIS_CELL, TETRIS_CELL, color);
    /* 边框效果 */
    ST7789_FillRect(px, py, TETRIS_CELL, 1, COLOR_WHITE);
    ST7789_FillRect(px, py, 1, TETRIS_CELL, COLOR_WHITE);
}

static void Tetris_DrawBoard(void)
{
    for (int c = 0; c < TETRIS_COLS; c++) {
        for (int r = 0; r < TETRIS_ROWS; r++) {
            Tetris_DrawCell(c, r, piece_colors[tetris_board[c][r]]);
        }
    }
}

static bool Tetris_CheckCollision(int piece, int rot, int px, int py)
{
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (!tetromino[piece][rot][r][c]) continue;
            int bx = px + c;
            int by = py + r;
            if (bx < 0 || bx >= TETRIS_COLS || by >= TETRIS_ROWS) return true;
            if (by >= 0 && tetris_board[bx][by]) return true;
        }
    }
    return false;
}

static void Tetris_PlacePiece(void)
{
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (!tetromino[current_piece][current_rot][r][c]) continue;
            int bx = piece_x + c;
            int by = piece_y + r;
            if (by >= 0 && by < TETRIS_ROWS && bx >= 0 && bx < TETRIS_COLS) {
                tetris_board[bx][by] = current_piece + 1;
            }
        }
    }
}

static void Tetris_ClearLines(void)
{
    int lines = 0;
    for (int r = TETRIS_ROWS - 1; r >= 0; r--) {
        bool full = true;
        for (int c = 0; c < TETRIS_COLS; c++) {
            if (!tetris_board[c][r]) { full = false; break; }
        }
        if (full) {
            lines++;
            /* 下移 */
            for (int rr = r; rr > 0; rr--) {
                for (int c = 0; c < TETRIS_COLS; c++) {
                    tetris_board[c][rr] = tetris_board[c][rr - 1];
                }
            }
            for (int c = 0; c < TETRIS_COLS; c++) tetris_board[c][0] = 0;
            r++; /* 重新检查当前行 */
        }
    }
    if (lines > 0) {
        int pts[] = {0, 100, 300, 500, 800};
        tetris_score += pts[lines] * (tetris_level + 1);
        tetris_lines += lines;
        tetris_level = tetris_lines / 10;
    }
}

static void Tetris_SpawnPiece(void)
{
    current_piece = rand() % 7;
    current_rot = 0;
    piece_x = TETRIS_COLS / 2 - 2;
    piece_y = -1;
    if (Tetris_CheckCollision(current_piece, current_rot, piece_x, piece_y)) {
        tetris_over = true;
    }
}

static void Tetris_DrawPiece(uint16_t color)
{
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (!tetromino[current_piece][current_rot][r][c]) continue;
            int bx = piece_x + c;
            int by = piece_y + r;
            if (by >= 0 && by < TETRIS_ROWS && bx >= 0 && bx < TETRIS_COLS) {
                Tetris_DrawCell(bx, by, color);
            }
        }
    }
}

static void Tetris_Init(void)
{
    memset(tetris_board, 0, sizeof(tetris_board));
    tetris_score = 0;
    tetris_level = 0;
    tetris_lines = 0;
    tetris_over = false;

    ST7789_Clear(COLOR_BLACK);
    /* 边框 */
    ST7789_FillRect(TETRIS_X_OFF - 2, TETRIS_Y_OFF - 2,
                    TETRIS_COLS * TETRIS_CELL + 4, TETRIS_ROWS * TETRIS_CELL + 4,
                    COLOR_GRAY);

    Tetris_SpawnPiece();
    Tetris_DrawBoard();
}

static uint32_t tetris_tick = 0;
static uint32_t tetris_speed = 30; /* 每N次循环下落一格 */

static void Tetris_Update(void)
{
    /* 擦除当前方块 */
    Tetris_DrawPiece(COLOR_BLACK);

    /* 按键处理 */
    if (KEY_LEFT) {
        if (!Tetris_CheckCollision(current_piece, current_rot, piece_x - 1, piece_y))
            piece_x--;
    }
    if (KEY_RIGHT) {
        if (!Tetris_CheckCollision(current_piece, current_rot, piece_x + 1, piece_y))
            piece_x++;
    }
    if (KEY_A) {
        int newRot = (current_rot + 1) % 4;
        if (!Tetris_CheckCollision(current_piece, newRot, piece_x, piece_y))
            current_rot = newRot;
    }
    if (KEY_DOWN) {
        if (!Tetris_CheckCollision(current_piece, current_rot, piece_x, piece_y + 1))
            piece_y++;
    }

    /* 自动下落 */
    tetris_tick++;
    if (tetris_tick >= tetris_speed - tetris_level * 2) {
        tetris_tick = 0;
        if (!Tetris_CheckCollision(current_piece, current_rot, piece_x, piece_y + 1)) {
            piece_y++;
        } else {
            /* 锁定 */
            Tetris_PlacePiece();
            Tetris_ClearLines();
            Tetris_DrawBoard();
            Tetris_SpawnPiece();
        }
    }

    /* 绘制当前方块 */
    if (!tetris_over) {
        Tetris_DrawPiece(piece_colors[current_piece + 1]);
        /* 更新分数 */
        ST7789_FillRect(0, 0, TETRIS_X_OFF - 4, 30, COLOR_BLACK);
        Draw_Number(2, 2, tetris_score, COLOR_YELLOW, 2);
    }
}

/* ===== 菜单选择 ===== */
typedef enum { GAME_SNAKE, GAME_TETRIS } GameType;

static GameType Show_Menu(void)
{
    ST7789_Clear(COLOR_BLACK);

    /* 标题 */
    ST7789_FillRect(20, 40, 200, 30, COLOR_BLUE);
    ST7789_FillRect(20, 90, 200, 30, COLOR_GREEN);
    ST7789_FillRect(20, 140, 200, 30, COLOR_RED);

    /* 标识色块: 贪吃蛇图标 */
    ST7789_FillRect(60, 100, 8, 8, COLOR_GREEN);
    ST7789_FillRect(70, 100, 8, 8, COLOR_GREEN);
    ST7789_FillRect(80, 100, 8, 8, COLOR_GREEN);
    /* 俄罗斯方块图标 */
    ST7789_FillRect(60, 150, 12, 12, COLOR_CYAN);
    ST7789_FillRect(74, 150, 12, 12, COLOR_CYAN);
    ST7789_FillRect(60, 164, 12, 12, COLOR_CYAN);
    ST7789_FillRect(74, 164, 12, 12, COLOR_YELLOW);

    /* 选择指示器 */
    int sel = 0;
    while (1) {
        /* 绘制选择箭头 */
        ST7789_FillRect(25, 95, 10, 10, sel == 0 ? COLOR_YELLOW : COLOR_BLACK);
        ST7789_FillRect(25, 145, 10, 10, sel == 1 ? COLOR_YELLOW : COLOR_BLACK);

        if (KEY_UP)   { sel = 0; Wait_KeyRelease(); }
        if (KEY_DOWN)  { sel = 1; Wait_KeyRelease(); }
        if (KEY_A)     { Wait_KeyRelease(); return (GameType)sel; }

        delay_ms(50);
    }
}

/* ===== 主函数 ===== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* 初始化ST7789 */
    ST7789_Init();

    while (1) {
        GameType game = Show_Menu();

        if (game == GAME_SNAKE) {
            Snake_Init();
            while (!game_over) {
                Snake_Update();
                if (KEY_B) break; /* B键退出 */
                delay_ms(120);    /* 游戏速度 */
            }
            /* 游戏结束画面 */
            if (game_over) {
                ST7789_FillRect(40, 100, 160, 60, COLOR_RED);
                Draw_Number(60, 120, score, COLOR_WHITE, 3);
                Wait_KeyRelease();
                while (!Key_Pressed()) { delay_ms(50); }
                Wait_KeyRelease();
            }
        } else {
            Tetris_Init();
            while (!tetris_over) {
                Tetris_Update();
                if (KEY_B) break;
                delay_ms(80);
            }
            if (tetris_over) {
                ST7789_FillRect(40, 100, 160, 60, COLOR_RED);
                Draw_Number(60, 120, tetris_score, COLOR_WHITE, 3);
                Wait_KeyRelease();
                while (!Key_Pressed()) { delay_ms(50); }
                Wait_KeyRelease();
            }
        }
    }
}
