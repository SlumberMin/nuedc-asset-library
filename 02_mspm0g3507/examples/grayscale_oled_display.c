/**
 * @file grayscale_oled_display.c
 * @brief MSPM0G3507 灰度OLED显示示例（SSD1327 + 图形绘制 + 动画效果）
 *
 * 硬件连接：
 *   SSD1327 128x128 灰度OLED（I2C）：
 *     SCL -> PB2 (I2C0_SCL)
 *     SDA -> PB3 (I2C0_SDA)
 *     I2C地址: 0x3C
 *
 *   按键：
 *     模式切换 -> PA11 (切换显示Demo)
 *     参数增加 -> PA12
 *     参数减少 -> PA13
 *
 * 功能：
 *   - SSD1327 16级灰度（4位色深）驱动
 *   - 支持点、线、矩形、圆、填充等图形操作
 *   - 中文字符显示（16x16点阵）
 *   - 多种动画Demo：
 *     1. 渐变色条
 *     2. 旋转正方形
 *     3. 粒子效果
 *     4. 波浪动画
 *     5. 矩阵雨效果
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ========== SSD1327配置 ========== */
#define SSD1327_I2C_ADDR        0x3C
#define SSD1327_WIDTH           128
#define SSD1327_HEIGHT          128
#define SSD1327_PAGES           (SSD1327_HEIGHT / 2)  /* 每字节2像素(4bit灰度) */
#define SSD1327_CMD             0x00
#define SSD1327_DATA            0x40

/* ========== 按键引脚 ========== */
#define BTN_MODE_PORT           GPIOA
#define BTN_MODE_PIN            DL_GPIO_PIN_11
#define BTN_UP_PORT             GPIOA
#define BTN_UP_PIN              DL_GPIO_PIN_12
#define BTN_DOWN_PORT           GPIOA
#define BTN_DOWN_PIN            DL_GPIO_PIN_13

/* ========== 灰度等级定义 ========== */
#define GRAY_0                  0x00   /* 黑色 */
#define GRAY_1                  0x02
#define GRAY_2                  0x04
#define GRAY_3                  0x06
#define GRAY_4                  0x08
#define GRAY_5                  0x0A
#define GRAY_6                  0x0C
#define GRAY_7                  0x0E   /* 中灰 */
#define GRAY_8                  0x11
#define GRAY_9                  0x13
#define GRAY_A                  0x15
#define GRAY_B                  0x17
#define GRAY_C                  0x19
#define GRAY_D                  0x1B
#define GRAY_E                  0x1D
#define GRAY_F                  0x1F   /* 最亮（白色） */

/* ========== 全局变量 ========== */
static volatile uint32_t g_systickCount = 0;
static uint8_t g_demoMode = 0;          /* 当前Demo模式 0-4 */
static uint8_t g_brightness = 15;       /* 全局亮度 0-15 */

/*
 * SSD1327显存缓冲区
 * 每字节包含2个像素（高4位=左像素，低4位=右像素）
 * 大小：128 * 64 = 8192字节
 */
static uint8_t g_frameBuffer[SSD1327_WIDTH * SSD1327_PAGES];

/* ========== 16x16中文字库（常用字子集） ========== */
/* "你好世界"等几个示例汉字，GB2312编码点阵 */
static const uint8_t g_chineseFont[][32] = {
    /* 你 (0xC4E3) */
    {0x00,0x00,0x23,0xF8,0x12,0x08,0x12,0x08,0x83,0xF8,0x42,0x08,0x42,0x08,0x13,0xF8,
     0x10,0x40,0x20,0x40,0xE0,0xA0,0x21,0x10,0x22,0x08,0x24,0x06,0x28,0x04,0x00,0x00},
    /* 好 (0xBAC3) */
    {0x00,0x00,0x20,0x00,0x13,0xF8,0x12,0x08,0x02,0x48,0x02,0x48,0xF2,0x48,0x12,0x48,
     0x12,0x48,0x14,0x48,0x14,0x80,0x18,0x80,0x11,0x00,0x02,0x00,0x04,0x00,0x00,0x00},
    /* 世 (0xCAC0) */
    {0x00,0x00,0x04,0x40,0x04,0x40,0x04,0x40,0x04,0x40,0x7F,0xFC,0x44,0x44,0x44,0x44,
     0x44,0x44,0x44,0x44,0x44,0x44,0x7F,0xFC,0x04,0x40,0x04,0x40,0x04,0x40,0x00,0x00},
    /* 界 (0xBDE7) */
    {0x00,0x00,0x1F,0xF0,0x10,0x10,0x1F,0xF0,0x10,0x10,0x1F,0xF0,0x04,0x00,0x7F,0xFC,
     0x0A,0x20,0x11,0x10,0x20,0x88,0x40,0x44,0x1F,0xF0,0x10,0x10,0x1F,0xF0,0x00,0x00},
};

/* ========== 函数声明 ========== */
void SysTick_Handler(void);
static void Delay_ms(uint32_t ms);
static bool Button_IsPressed(GPIO_Regs *port, uint32_t pin);

/* SSD1327驱动 */
static void SSD1327_WriteCmd(uint8_t cmd);
static void SSD1327_WriteCmd2(uint8_t cmd1, uint8_t cmd2);
static void SSD1327_WriteData(const uint8_t *data, uint16_t len);
static void SSD1327_Init(void);
static void SSD1327_Update(void);
static void SSD1327_SetContrast(uint8_t contrast);

/* 图形函数 */
static void GFX_Clear(void);
static void GFX_SetPixel(int16_t x, int16_t y, uint8_t gray);
static uint8_t GFX_GetPixel(int16_t x, int16_t y);
static void GFX_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t gray);
static void GFX_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t gray);
static void GFX_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t gray);
static void GFX_DrawCircle(int16_t cx, int16_t cy, int16_t r, uint8_t gray);
static void GFX_FillCircle(int16_t cx, int16_t cy, int16_t r, uint8_t gray);
static void GFX_DrawChar(int16_t x, int16_t y, char ch, uint8_t size, uint8_t gray);
static void GFX_DrawString(int16_t x, int16_t y, const char *str, uint8_t size, uint8_t gray);
static void GFX_DrawChinese(int16_t x, int16_t y, uint8_t index, uint8_t gray);
static void GFX_DrawHGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t grayStart, uint8_t grayEnd);
static void GFX_DrawVGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t grayStart, uint8_t grayEnd);

/* 数学辅助 */
static int16_t fastSin(uint16_t angle);  /* 0-359度 */
static int16_t fastCos(uint16_t angle);

/* 动画Demo */
static void Demo_GradientBars(void);
static void Demo_RotatingSquare(void);
static void Demo_ParticleEffect(void);
static void Demo_WaveAnimation(void);
static void Demo_MatrixRain(void);

/* ========== 基本字体 5x8 ========== */
static const uint8_t g_font5x8[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
    {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14},
    {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x01,0x01}, {0x3E,0x41,0x41,0x51,0x32},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03},
    {0x61,0x51,0x49,0x45,0x43},
};

/* ========== 延时 ========== */
void SysTick_Handler(void) { g_systickCount++; }

static void Delay_ms(uint32_t ms) {
    uint32_t start = g_systickCount;
    while ((g_systickCount - start) < ms);
}

static bool Button_IsPressed(GPIO_Regs *port, uint32_t pin) {
    if (DL_GPIO_readPins(port, pin) == 0) {
        Delay_ms(20);
        if (DL_GPIO_readPins(port, pin) == 0) {
            while (DL_GPIO_readPins(port, pin) == 0);
            return true;
        }
    }
    return false;
}

/* ========== 快速三角函数查表 ========== */
/* 预计算sin值表（0-90度，精度1度，放大1000倍） */
static const int16_t g_sinTable[91] = {
      0,  17,  35,  52,  70,  87, 105, 122, 139, 156,
    174, 191, 208, 225, 242, 259, 276, 292, 309, 326,
    342, 358, 375, 391, 407, 423, 438, 454, 469, 485,
    500, 515, 530, 545, 559, 574, 588, 602, 616, 629,
    643, 656, 669, 682, 695, 707, 719, 731, 743, 755,
    766, 777, 788, 799, 809, 819, 829, 839, 848, 857,
    866, 875, 883, 891, 899, 906, 914, 921, 927, 934,
    940, 946, 951, 956, 961, 966, 970, 974, 978, 982,
    985, 988, 990, 993, 995, 996, 998, 999, 999,1000,
   1000
};

static int16_t fastSin(uint16_t angle) {
    angle = angle % 360;
    if (angle <= 90)  return g_sinTable[angle];
    if (angle <= 180) return g_sinTable[180 - angle];
    if (angle <= 270) return -g_sinTable[angle - 180];
    return -g_sinTable[360 - angle];
}

static int16_t fastCos(uint16_t angle) {
    return fastSin(angle + 90);
}

/* ========== SSD1327驱动 ========== */
static void SSD1327_WriteCmd(uint8_t cmd) {
    uint8_t buf[2] = {SSD1327_CMD, cmd};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, SSD1327_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static void SSD1327_WriteCmd2(uint8_t cmd1, uint8_t cmd2) {
    uint8_t buf[3] = {SSD1327_CMD, cmd1, cmd2};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 3);
    DL_I2C_startControllerTransfer(I2C_0_INST, SSD1327_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 3);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static void SSD1327_WriteData(const uint8_t *data, uint16_t len) {
    /* I2C FIFO有限，分段发送 */
    uint16_t sent = 0;
    while (sent < len) {
        uint16_t chunk = len - sent;
        if (chunk > 30) chunk = 30;  /* 留1字节给地址头 */
        uint8_t buf[31];
        buf[0] = SSD1327_DATA;
        memcpy(&buf[1], &data[sent], chunk);
        DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, chunk + 1);
        DL_I2C_startControllerTransfer(I2C_0_INST, SSD1327_I2C_ADDR,
                                       DL_I2C_CONTROLLER_DIRECTION_TX, chunk + 1);
        while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
        sent += chunk;
    }
}

static void SSD1327_Init(void) {
    Delay_ms(100);
    SSD1327_WriteCmd(0xAE);  /* 关闭显示 */
    SSD1327_WriteCmd2(0xA0, 0x51);  /* 设置重映射：启用水平地址递增 */
    SSD1327_WriteCmd2(0xA1, 0x00);  /* 起始行=0 */
    SSD1327_WriteCmd2(0xA2, 0x00);  /* 显示偏移=0 */
    SSD1327_WriteCmd(0xA4);  /* 正常显示（非全亮） */
    SSD1327_WriteCmd2(0xA8, 0x7F);  /* 多路复用：1/128 */
    SSD1327_WriteCmd2(0xAB, 0x01);  /* VDD内部 */
    SSD1327_WriteCmd2(0xB1, 0x51);  /* 相位1/2周期 */
    SSD1327_WriteCmd2(0xB3, 0x01);  /* 前时钟分频 */
    SSD1327_WriteCmd(0xB9);  /* 使用默认灰度表 */
    SSD1327_WriteCmd2(0xBC, 0x08);  /* COM电压 */
    SSD1327_WriteCmd2(0xBE, 0x07);  /* VCOMH */
    SSD1327_WriteCmd2(0x81, 0x80);  /* 对比度=128 */
    SSD1327_WriteCmd(0xAF);  /* 开启显示 */
}

static void SSD1327_SetContrast(uint8_t contrast) {
    SSD1327_WriteCmd2(0x81, contrast);
}

/*
 * 将帧缓冲区写入OLED
 * SSD1327：每字节2像素（高4位=列x，低4位=列x+1）
 */
static void SSD1327_Update(void) {
    /* 设置列地址范围 */
    SSD1327_WriteCmd2(0x15, 0x00);  /* 起始列=0 */
    SSD1327_WriteCmd2(0x7F, 0x00);  /* 结束列=127 (被忽略，用0x7F) */
    /* 设置行地址范围 */
    SSD1327_WriteCmd2(0x75, 0x00);  /* 起始行=0 */
    SSD1327_WriteCmd2(0x7F, 0x00);  /* 结束行=127 */

    /* 发送整帧数据 */
    SSD1327_WriteData(g_frameBuffer, sizeof(g_frameBuffer));
}

/* ========== 图形函数 ========== */
static void GFX_Clear(void) {
    memset(g_frameBuffer, 0, sizeof(g_frameBuffer));
}

/*
 * 设置像素的灰度值
 * x: 0-127, y: 0-127, gray: 0-15(0=黑, 15=最亮)
 * 帧缓冲区布局：每字节2像素，高4位=偶数列，低4位=奇数列
 * 索引 = (y * 64) + (x / 2)
 */
static void GFX_SetPixel(int16_t x, int16_t y, uint8_t gray) {
    if (x < 0 || x >= SSD1327_WIDTH || y < 0 || y >= SSD1327_HEIGHT) return;

    uint16_t idx = (uint16_t)(y * (SSD1327_WIDTH / 2)) + (x / 2);
    uint8_t gray4 = gray & 0x0F;  /* 确保4位 */

    if (x & 1) {
        /* 奇数列：低4位 */
        g_frameBuffer[idx] = (g_frameBuffer[idx] & 0xF0) | gray4;
    } else {
        /* 偶数列：高4位 */
        g_frameBuffer[idx] = (g_frameBuffer[idx] & 0x0F) | (gray4 << 4);
    }
}

static uint8_t GFX_GetPixel(int16_t x, int16_t y) {
    if (x < 0 || x >= SSD1327_WIDTH || y < 0 || y >= SSD1327_HEIGHT) return 0;
    uint16_t idx = (uint16_t)(y * (SSD1327_WIDTH / 2)) + (x / 2);
    if (x & 1) return g_frameBuffer[idx] & 0x0F;
    return (g_frameBuffer[idx] >> 4) & 0x0F;
}

static void GFX_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t gray) {
    int16_t dx = abs(x1-x0), dy = abs(y1-y0);
    int16_t sx = (x0<x1)?1:-1, sy = (y0<y1)?1:-1;
    int16_t err = dx - dy;
    while (1) {
        GFX_SetPixel(x0, y0, gray);
        if (x0==x1 && y0==y1) break;
        int16_t e2 = 2*err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

static void GFX_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t gray) {
    GFX_DrawLine(x, y, x+w-1, y, gray);
    GFX_DrawLine(x+w-1, y, x+w-1, y+h-1, gray);
    GFX_DrawLine(x+w-1, y+h-1, x, y+h-1, gray);
    GFX_DrawLine(x, y+h-1, x, y, gray);
}

static void GFX_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t gray) {
    for (int16_t i = x; i < x + w; i++)
        for (int16_t j = y; j < y + h; j++)
            GFX_SetPixel(i, j, gray);
}

static void GFX_DrawCircle(int16_t cx, int16_t cy, int16_t r, uint8_t gray) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t xi = 0;
    int16_t yi = r;

    GFX_SetPixel(cx, cy + r, gray);
    GFX_SetPixel(cx, cy - r, gray);
    GFX_SetPixel(cx + r, cy, gray);
    GFX_SetPixel(cx - r, cy, gray);

    while (xi < yi) {
        if (f >= 0) { yi--; ddF_y += 2; f += ddF_y; }
        xi++;
        ddF_x += 2;
        f += ddF_x;
        GFX_SetPixel(cx + xi, cy + yi, gray);
        GFX_SetPixel(cx - xi, cy + yi, gray);
        GFX_SetPixel(cx + xi, cy - yi, gray);
        GFX_SetPixel(cx - xi, cy - yi, gray);
        GFX_SetPixel(cx + yi, cy + xi, gray);
        GFX_SetPixel(cx - yi, cy + xi, gray);
        GFX_SetPixel(cx + yi, cy - xi, gray);
        GFX_SetPixel(cx - yi, cy - xi, gray);
    }
}

static void GFX_FillCircle(int16_t cx, int16_t cy, int16_t r, uint8_t gray) {
    for (int16_t y = -r; y <= r; y++) {
        for (int16_t x = -r; x <= r; x++) {
            if (x*x + y*y <= r*r) {
                GFX_SetPixel(cx + x, cy + y, gray);
            }
        }
    }
}

static void GFX_DrawChar(int16_t x, int16_t y, char ch, uint8_t size, uint8_t gray) {
    if (ch < ' ' || ch > 'Z') return;
    uint8_t idx = ch - ' ';
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = g_font5x8[idx][i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                if (size == 1) GFX_SetPixel(x+i, y+j, gray);
                else GFX_FillRect(x+i*size, y+j*size, size, size, gray);
            }
        }
    }
}

static void GFX_DrawString(int16_t x, int16_t y, const char *str, uint8_t size, uint8_t gray) {
    while (*str) {
        GFX_DrawChar(x, y, *str, size, gray);
        x += (size==1) ? 6 : (6*size);
        if (x >= SSD1327_WIDTH - 5) { x = 0; y += (size==1)?8:(8*size); }
        str++;
    }
}

/* 绘制16x16中文字符 */
static void GFX_DrawChinese(int16_t x, int16_t y, uint8_t index, uint8_t gray) {
    if (index >= sizeof(g_chineseFont) / 32) return;
    for (uint8_t row = 0; row < 16; row++) {
        uint8_t b0 = g_chineseFont[index][row * 2];
        uint8_t b1 = g_chineseFont[index][row * 2 + 1];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (b0 & (0x80 >> bit)) GFX_SetPixel(x + bit, y + row, gray);
            if (b1 & (0x80 >> bit)) GFX_SetPixel(x + 8 + bit, y + row, gray);
        }
    }
}

/* 水平渐变填充 */
static void GFX_DrawHGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t grayStart, uint8_t grayEnd) {
    for (int16_t i = 0; i < w; i++) {
        uint8_t gray = grayStart + (uint8_t)((int16_t)(grayEnd - grayStart) * i / w);
        for (int16_t j = y; j < y + h; j++) {
            GFX_SetPixel(x + i, j, gray);
        }
    }
}

/* 垂直渐变填充 */
static void GFX_DrawVGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t grayStart, uint8_t grayEnd) {
    for (int16_t j = 0; j < h; j++) {
        uint8_t gray = grayStart + (uint8_t)((int16_t)(grayEnd - grayStart) * j / h);
        for (int16_t i = x; i < x + w; i++) {
            GFX_SetPixel(i, y + j, gray);
        }
    }
}

/* ========== Demo 1: 渐变色条 ========== */
static void Demo_GradientBars(void) {
    static uint16_t phase = 0;
    GFX_Clear();

    /* 标题 */
    GFX_DrawString(24, 2, "GRAY SCALE", 2, GRAY_F);

    /* 水平灰度条 */
    for (uint8_t i = 0; i < 16; i++) {
        GFX_FillRect(i * 8, 24, 8, 20, i);
    }

    /* 渐变条 */
    GFX_DrawHGradient(0, 50, 128, 16, GRAY_0, GRAY_F);
    GFX_DrawVGradient(0, 70, 128, 16, GRAY_0, GRAY_F);

    /* 脉冲圆环 */
    uint8_t pulse = (uint8_t)((fastSin(phase) + 1000) * 15 / 2000);
    GFX_DrawCircle(64, 104, 20, pulse);
    GFX_FillCircle(64, 104, 14, pulse / 2);

    GFX_DrawString(8, 100, "16-Level", 1, GRAY_B);

    phase = (phase + 3) % 360;
    SSD1327_Update();
}

/* ========== Demo 2: 旋转正方形 ========== */
static void Demo_RotatingSquare(void) {
    static uint16_t angle = 0;
    GFX_Clear();

    GFX_DrawString(16, 2, "ROTATION", 2, GRAY_F);

    int16_t cx = 64, cy = 72;
    int16_t size = 30;

    /* 计算旋转后的正方形顶点 */
    int16_t cosVal = fastCos(angle);
    int16_t sinVal = fastSin(angle);

    int16_t corners[4][2];
    int16_t dx[4] = {-size, size, size, -size};
    int16_t dy[4] = {-size, -size, size, size};

    for (uint8_t i = 0; i < 4; i++) {
        corners[i][0] = cx + (int16_t)((int32_t)dx[i] * cosVal - (int32_t)dy[i] * sinVal) / 1000;
        corners[i][1] = cy + (int16_t)((int32_t)dx[i] * sinVal + (int32_t)dy[i] * cosVal) / 1000;
    }

    /* 绘制旋转正方形 */
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t next = (i + 1) % 4;
        uint8_t gray = (uint8_t)(GRAY_4 + i * 3);
        GFX_DrawLine(corners[i][0], corners[i][1],
                     corners[next][0], corners[next][1], gray);
    }

    /* 中心点 */
    GFX_FillCircle(cx, cy, 3, GRAY_F);

    /* 角度显示 */
    char buf[16];
    int n = 0;
    uint16_t displayAngle = angle % 360;
    buf[n++] = '0' + displayAngle / 100;
    buf[n++] = '0' + (displayAngle / 10) % 10;
    buf[n++] = '0' + displayAngle % 10;
    buf[n++] = 0xB0;  /* 度符号 */
    buf[n] = 0;
    GFX_DrawString(48, 106, buf, 1, GRAY_C);

    angle = (angle + 2) % 360;
    SSD1327_Update();
}

/* ========== Demo 3: 粒子效果 ========== */
#define MAX_PARTICLES  40

typedef struct {
    int16_t x, y;
    int16_t vx, vy;
    uint8_t life;
    uint8_t maxLife;
} Particle_t;

static void Demo_ParticleEffect(void) {
    static Particle_t particles[MAX_PARTICLES];
    static uint8_t frame = 0;

    /* 淡出效果（不清屏，降低所有像素亮度） */
    for (uint16_t i = 0; i < sizeof(g_frameBuffer); i++) {
        uint8_t hi = (g_frameBuffer[i] >> 4);
        uint8_t lo = (g_frameBuffer[i] & 0x0F);
        if (hi > 0) hi--;
        if (lo > 0) lo--;
        g_frameBuffer[i] = (hi << 4) | lo;
    }

    /* 标题（每帧重绘） */
    GFX_DrawString(8, 2, "PARTICLES", 2, GRAY_F);

    /* 更新和绘制粒子 */
    for (uint8_t i = 0; i < MAX_PARTICLES; i++) {
        if (particles[i].life == 0) {
            /* 重生粒子（从中心发射） */
            particles[i].x = 64 * 16;  /* 定点数 *16 */
            particles[i].y = 80 * 16;
            particles[i].vx = (int16_t)(((int32_t)(frame * 7 + i * 13) % 61) - 30);
            particles[i].vy = (int16_t)(((int32_t)(frame * 11 + i * 17) % 41) - 50);
            particles[i].maxLife = 30 + (i * 3) % 30;
            particles[i].life = particles[i].maxLife;
        }

        /* 更新位置 */
        particles[i].x += particles[i].vx;
        particles[i].y += particles[i].vy;
        particles[i].vy += 1;  /* 重力 */
        particles[i].life--;

        /* 绘制粒子 */
        if (particles[i].life > 0) {
            int16_t px = particles[i].x / 16;
            int16_t py = particles[i].y / 16;
            uint8_t gray = (uint8_t)((uint16_t)particles[i].life * 15 / particles[i].maxLife);
            if (gray > 15) gray = 15;
            GFX_SetPixel(px, py, gray);
            if (particles[i].life > particles[i].maxLife / 2) {
                GFX_SetPixel(px + 1, py, gray / 2);
                GFX_SetPixel(px, py + 1, gray / 2);
            }
        }
    }

    frame++;
    SSD1327_Update();
}

/* ========== Demo 4: 波浪动画 ========== */
static void Demo_WaveAnimation(void) {
    static uint16_t phase = 0;
    GFX_Clear();

    GFX_DrawString(16, 2, "WAVE FX", 2, GRAY_F);

    /* 绘制多层波浪 */
    for (uint8_t layer = 0; layer < 4; layer++) {
        uint8_t gray = (uint8_t)(GRAY_4 + layer * 3);
        uint16_t offset = phase + layer * 30;
        for (int16_t x = 0; x < 128; x++) {
            int16_t y = 50 + (int16_t)((int32_t)fastSin((x * 3 + offset) % 360) * (20 - layer * 4) / 1000);
            /* 画竖线段到波浪位置 */
            for (int16_t dy = y; dy < 128; dy++) {
                uint8_t fade = (uint8_t)(gray * (128 - dy) / 128);
                GFX_SetPixel(x, dy, fade);
            }
        }
    }

    /* 顶部波形线 */
    for (int16_t x = 0; x < 127; x++) {
        int16_t y0 = 32 + (int16_t)((int32_t)fastSin((x * 4 + phase * 2) % 360) * 12 / 1000);
        int16_t y1 = 32 + (int16_t)((int32_t)fastSin(((x+1) * 4 + phase * 2) % 360) * 12 / 1000);
        GFX_DrawLine(x, y0, x+1, y1, GRAY_F);
    }

    phase = (phase + 4) % 360;
    SSD1327_Update();
}

/* ========== Demo 5: 矩阵雨 ========== */
#define RAIN_COLS  21

static void Demo_MatrixRain(void) {
    static uint8_t rainPos[RAIN_COLS];
    static uint8_t rainLen[RAIN_COLS];
    static uint8_t rainSpeed[RAIN_COLS];
    static bool rainInit = false;

    if (!rainInit) {
        for (uint8_t i = 0; i < RAIN_COLS; i++) {
            rainPos[i] = (uint8_t)(i * 7 + (i * 3) % 7);
            rainLen[i] = 5 + (i * 2) % 10;
            rainSpeed[i] = 1 + (i % 3);
        }
        rainInit = true;
    }

    /* 半清屏（渐隐效果） */
    for (uint16_t i = 0; i < sizeof(g_frameBuffer); i++) {
        uint8_t hi = (g_frameBuffer[i] >> 4);
        uint8_t lo = (g_frameBuffer[i] & 0x0F);
        if (hi > 1) hi -= 2; else hi = 0;
        if (lo > 1) lo -= 2; else lo = 0;
        g_frameBuffer[i] = (hi << 4) | lo;
    }

    /* 标题 */
    GFX_DrawString(12, 2, "MATRIX RAIN", 1, GRAY_F);

    /* 更新和绘制雨滴 */
    for (uint8_t col = 0; col < RAIN_COLS; col++) {
        int16_t x = col * 6 + 1;

        /* 雨滴头 */
        int16_t headY = rainPos[col];
        GFX_SetPixel(x, headY, GRAY_F);

        /* 雨滴尾部（渐暗） */
        for (uint8_t t = 1; t < rainLen[col]; t++) {
            int16_t tailY = headY - t;
            if (tailY >= 12) {
                uint8_t gray = (uint8_t)(GRAY_F - (uint16_t)t * GRAY_F / rainLen[col]);
                GFX_SetPixel(x, tailY, gray);
            }
        }

        /* 更新位置 */
        rainPos[col] += rainSpeed[col];
        if (rainPos[col] >= 128 + rainLen[col]) {
            rainPos[col] = 12;
            rainLen[col] = 5 + (rainLen[col] * 2) % 15;
            rainSpeed[col] = 1 + (rainSpeed[col] % 3);
        }
    }

    SSD1327_Update();
}

/* ========== 主函数 ========== */
int main(void) {
    /* 系统初始化 */
    DL_SYSCTL_initSYSCTL();
    SysTick_Config(SystemCoreClock / 1000);

    /* 外设初始化 */
    DL_I2C_initController(I2C_0_INST, 400000);

    /* GPIO */
    DL_GPIO_initDigitalInput(BTN_MODE_PORT | BTN_MODE_PIN);
    DL_GPIO_initDigitalInput(BTN_UP_PORT | BTN_UP_PIN);
    DL_GPIO_initDigitalInput(BTN_DOWN_PORT | BTN_DOWN_PIN);

    /* SSD1327初始化 */
    SSD1327_Init();

    /* 开机画面 */
    GFX_Clear();
    GFX_DrawString(8, 24, "SSD1327 128x128", 1, GRAY_F);
    GFX_DrawString(20, 40, "16-Level Gray", 1, GRAY_C);
    GFX_DrawChinese(32, 60, 0, GRAY_F);  /* 你 */
    GFX_DrawChinese(48, 60, 1, GRAY_F);  /* 好 */
    GFX_DrawChinese(64, 60, 2, GRAY_F);  /* 世 */
    GFX_DrawChinese(80, 60, 3, GRAY_F);  /* 界 */
    GFX_DrawString(16, 88, "Press MODE to", 1, GRAY_8);
    GFX_DrawString(24, 96, "switch demo", 1, GRAY_8);
    SSD1327_Update();
    Delay_ms(2000);

    /* Demo模式名称 */
    static const char *demoName[] = {
        "Gradient", "Rotation", "Particles", "Waves", "Matrix"
    };

    while (1) {
        /* 按键：切换Demo */
        if (Button_IsPressed(BTN_MODE_PORT, BTN_MODE_PIN)) {
            g_demoMode = (g_demoMode + 1) % 5;
            GFX_Clear();
        }

        /* 按键：调节亮度 */
        if (Button_IsPressed(BTN_UP_PORT, BTN_UP_PIN)) {
            if (g_brightness < 15) g_brightness++;
            SSD1327_SetContrast((uint8_t)(g_brightness * 17));  /* 0-15 -> 0-255 */
        }
        if (Button_IsPressed(BTN_DOWN_PORT, BTN_DOWN_PIN)) {
            if (g_brightness > 0) g_brightness--;
            SSD1327_SetContrast((uint8_t)(g_brightness * 17));
        }

        /* 执行当前Demo */
        switch (g_demoMode) {
            case 0: Demo_GradientBars();    break;
            case 1: Demo_RotatingSquare();  break;
            case 2: Demo_ParticleEffect();  break;
            case 3: Demo_WaveAnimation();   break;
            case 4: Demo_MatrixRain();      break;
        }

        Delay_ms(30);  /* ~33fps */
    }
}
