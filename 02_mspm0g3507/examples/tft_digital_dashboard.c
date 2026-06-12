/**
 * @file tft_digital_dashboard.c
 * @brief TFT数字仪表盘 - ST7789 + 指针绘制 + 数据图表
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
 *
 *   旋转编码器 (模拟数据源):
 *     A    -> PB0 (增量调节)
 *     B    -> PB1
 *     SW   -> PB2 (切换仪表模式)
 *
 *   ADC模拟输入 (数据源):
 *     PA27 -> ADC0_CH5 (模拟电压输入)
 *
 * 功能：
 *   - 圆形仪表盘（带刻度、指针动画）
 *   - 实时波形图表（滚动显示）
 *   - 多页面切换（旋钮切换：电压表/速度表/温度表/波形图）
 *   - 数据标签和状态栏
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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

/* ===== 颜色定义 RGB565 ===== */
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_ORANGE    0xFD20
#define COLOR_GRAY      0x8410
#define COLOR_DARKGRAY  0x4208
#define COLOR_DARKBLUE  0x0010
#define COLOR_DARKGREEN 0x0400
#define COLOR_LIGHTBLUE 0x051F

/* 仪表盘主题色 */
#define GAUGE_BG_COLOR      0x10A2   /* 深蓝灰背景 */
#define GAUGE_RING_COLOR    0x3186   /* 灰色刻度环 */
#define GAUGE_SCALE_COLOR   0xC618   /* 浅灰刻度线 */
#define GAUGE_DANGER_COLOR  0xFA80   /* 警告橙 */
#define GAUGE_SAFE_COLOR    0x07E0   /* 安全绿 */

/* ===== 按键 ===== */
#define BTN_MODE (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2)))

/* ===== 仪表盘模式 ===== */
typedef enum {
    DASH_VOLTAGE = 0,   /* 电压表 */
    DASH_SPEED,         /* 速度表 */
    DASH_TEMPERATURE,   /* 温度表 */
    DASH_WAVEFORM,      /* 波形图 */
    DASH_MODE_COUNT
} DashboardMode_t;

/* ===== 仪表盘参数 ===== */
typedef struct {
    const char *title;      /* 仪表标题 */
    const char *unit;       /* 单位 */
    float min_val;          /* 最小值 */
    float max_val;          /* 最大值 */
    float warning_val;      /* 警告阈值 */
    float danger_val;       /* 危险阈值 */
    uint16_t scale_count;   /* 主刻度数量 */
    uint16_t color;         /* 主题色 */
} GaugeConfig_t;

/* 仪表配置表 */
static const GaugeConfig_t gauge_cfg[DASH_MODE_COUNT] = {
    { "VOLTAGE",  "V",   0.0f,  30.0f,  24.0f,  27.0f,  6, COLOR_CYAN    },
    { "SPEED",    "RPM", 0.0f, 8000.0f, 6000.0f, 7000.0f, 8, COLOR_GREEN   },
    { "TEMP",     "C", -20.0f, 100.0f,  60.0f,   80.0f,  6, COLOR_ORANGE  },
    { "WAVE",     "",    0.0f,   3.3f,   0.0f,    0.0f,   0, COLOR_MAGENTA },
};

/* ===== SPI 发送 ===== */
static void SPI_WriteByte(uint8_t dat)
{
    DL_SPI_transmitData8(SPI0, dat);
    while (DL_SPI_isBusy(SPI0)) {}
}

static void ST7789_WriteCmd(uint8_t cmd)
{
    DC_LOW(); CS_LOW();
    SPI_WriteByte(cmd);
    CS_HIGH();
}

static void ST7789_WriteData8(uint8_t dat)
{
    DC_HIGH(); CS_LOW();
    SPI_WriteByte(dat);
    CS_HIGH();
}

static void ST7789_WriteData16(uint16_t dat)
{
    DC_HIGH(); CS_LOW();
    SPI_WriteByte(dat >> 8);
    SPI_WriteByte(dat & 0xFF);
    CS_HIGH();
}

/* ===== ST7789 初始化 ===== */
static void ST7789_Init(void)
{
    RST_LOW();
    delay_cycles(1600000);  /* ~100ms @16MHz */
    RST_HIGH();
    delay_cycles(1600000);

    ST7789_WriteCmd(0x11);   /* Sleep Out */
    delay_cycles(800000);

    ST7789_WriteCmd(0x36);   /* MADCTL */
    ST7789_WriteData8(0x00); /* 正常方向 */

    ST7789_WriteCmd(0x3A);   /* 颜色格式 */
    ST7789_WriteData8(0x55); /* 16bit/pixel */

    ST7789_WriteCmd(0xB2);   /* Porch设置 */
    ST7789_WriteData8(0x0C); ST7789_WriteData8(0x0C);
    ST7789_WriteData8(0x00); ST7789_WriteData8(0x33);
    ST7789_WriteData8(0x33);

    ST7789_WriteCmd(0xB7);   /* Gate Control */
    ST7789_WriteData8(0x35);

    ST7789_WriteCmd(0xBB);   /* VCOM */
    ST7789_WriteData8(0x19);

    ST7789_WriteCmd(0xC0);   /* LCM Control */
    ST7789_WriteData8(0x2C);

    ST7789_WriteCmd(0xC2);   /* VDV/VRH */
    ST7789_WriteData8(0x01);

    ST7789_WriteCmd(0xC3);   /* VRH */
    ST7789_WriteData8(0x12);

    ST7789_WriteCmd(0xC4);   /* VDV */
    ST7789_WriteData8(0x20);

    ST7789_WriteCmd(0xC6);   /* Frame Rate */
    ST7789_WriteData8(0x0F); /* 60Hz */

    ST7789_WriteCmd(0xD0);   /* Power Control */
    ST7789_WriteData8(0xA4); ST7789_WriteData8(0xA1);

    ST7789_WriteCmd(0x21);   /* Display Inversion On (ST7789需要) */
    ST7789_WriteCmd(0x29);   /* Display On */
    delay_cycles(160000);
    BL_ON();
}

/* 设置绘图窗口 */
static void ST7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    ST7789_WriteCmd(0x2A);
    ST7789_WriteData16(x0); ST7789_WriteData16(x1);
    ST7789_WriteCmd(0x2B);
    ST7789_WriteData16(y0); ST7789_WriteData16(y1);
    ST7789_WriteCmd(0x2C);
}

/* 填充矩形 */
static void ST7789_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ST7789_SetWindow(x, y, x + w - 1, y + h - 1);
    DC_HIGH(); CS_LOW();
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        SPI_WriteByte(color >> 8);
        SPI_WriteByte(color & 0xFF);
    }
    CS_HIGH();
}

/* 填充全屏 */
static void ST7789_FillScreen(uint16_t color)
{
    ST7789_FillRect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, color);
}

/* 画像素点 */
static void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= ST7789_WIDTH || y >= ST7789_HEIGHT) return;
    ST7789_SetWindow(x, y, x, y);
    ST7789_WriteData16(color);
}

/* ===== Bresenham画线 ===== */
static void ST7789_DrawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0), dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (1) {
        ST7789_DrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* 画粗线段（带宽度） */
static void ST7789_DrawThickLine(int x0, int y0, int x1, int y1, uint8_t thick, uint16_t color)
{
    /* 简易粗线：用多条偏移线模拟 */
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    for (int t = -(int)thick / 2; t <= (int)thick / 2; t++) {
        if (dx > dy)
            ST7789_DrawLine(x0, y0 + t, x1, y1 + t, color);
        else
            ST7789_DrawLine(x0 + t, y0, x1 + t, y1, color);
    }
}

/* 画圆（Bresenham） */
static void ST7789_DrawCircle(int cx, int cy, int r, uint16_t color)
{
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        ST7789_DrawPixel(cx + x, cy + y, color);
        ST7789_DrawPixel(cx + y, cy + x, color);
        ST7789_DrawPixel(cx - y, cy + x, color);
        ST7789_DrawPixel(cx - x, cy + y, color);
        ST7789_DrawPixel(cx - x, cy - y, color);
        ST7789_DrawPixel(cx - y, cy - x, color);
        ST7789_DrawPixel(cx + y, cy - x, color);
        ST7789_DrawPixel(cx + x, cy - y, color);
        y++;
        if (err < 0) { err += 2 * y + 1; }
        else { x--; err += 2 * (y - x) + 1; }
    }
}

/* 填充圆 */
static void ST7789_FillCircle(int cx, int cy, int r, uint16_t color)
{
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)sqrtf((float)(r * r - dy * dy));
        ST7789_DrawLine(cx - dx, cy + dy, cx + dx, cy + dy, color);
    }
}

/* ===== 简易 8x16 ASCII 字体（仅含数字、大写字母和常用符号）===== */
/* 点阵字体表 — 每字符8字节，取上半部分用作8x8显示 */
static const uint8_t font_8x8[][8] = {
    /* 0-9 数字 (ASCII 0x30~0x39) */
    {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00}, /* '0' */
    {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, /* '1' */
    {0x3C,0x66,0x06,0x1C,0x30,0x60,0x7E,0x00}, /* '2' */
    {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00}, /* '3' */
    {0x0C,0x1C,0x3C,0x6C,0x7E,0x0C,0x0C,0x00}, /* '4' */
    {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00}, /* '5' */
    {0x1C,0x30,0x60,0x7C,0x66,0x66,0x3C,0x00}, /* '6' */
    {0x7E,0x06,0x0C,0x18,0x30,0x30,0x30,0x00}, /* '7' */
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, /* '8' */
    {0x3C,0x66,0x66,0x3E,0x06,0x0C,0x38,0x00}, /* '9' */
};

/* 绘制单个字符 (8x8) */
static void DrawChar8x8(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg, uint8_t scale)
{
    const uint8_t *glyph = NULL;
    if (ch >= '0' && ch <= '9') {
        glyph = font_8x8[ch - '0'];
    } else if (ch == '.') {
        static const uint8_t dot[] = {0,0,0,0,0,0x18,0x18,0};
        glyph = dot;
    } else if (ch == ' ') {
        static const uint8_t sp[] = {0,0,0,0,0,0,0,0};
        glyph = sp;
    } else if (ch == '-') {
        static const uint8_t minus[] = {0,0,0,0x7E,0,0,0,0};
        glyph = minus;
    } else if (ch == 'V' || ch == 'v') {
        static const uint8_t v[] = {0xC3,0xC3,0x66,0x66,0x3C,0x3C,0x18,0x00};
        glyph = v;
    } else if (ch == 'R' || ch == 'r') {
        static const uint8_t r[] = {0x7C,0x66,0x66,0x7C,0x6C,0x66,0x66,0x00};
        glyph = r;
    } else if (ch == 'P' || ch == 'p') {
        static const uint8_t p[] = {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00};
        glyph = p;
    } else if (ch == 'M' || ch == 'm') {
        static const uint8_t m[] = {0xC3,0xE7,0xFF,0xDB,0xC3,0xC3,0xC3,0x00};
        glyph = m;
    } else if (ch == 'C' || ch == 'c') {
        static const uint8_t c[] = {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00};
        glyph = c;
    } else if (ch == 'T' || ch == 't') {
        static const uint8_t t[] = {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00};
        glyph = t;
    } else if (ch == 'E' || ch == 'e') {
        static const uint8_t e[] = {0x7E,0x60,0x60,0x7C,0x60,0x60,0x7E,0x00};
        glyph = e;
    } else if (ch == 'S' || ch == 's') {
        static const uint8_t s[] = {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00};
        glyph = s;
    } else if (ch == 'A' || ch == 'a') {
        static const uint8_t a[] = {0x3C,0x66,0x66,0x7E,0x66,0x66,0x66,0x00};
        glyph = a;
    } else if (ch == 'W' || ch == 'w') {
        static const uint8_t w[] = {0xC3,0xC3,0xC3,0xDB,0xFF,0xE7,0xC3,0x00};
        glyph = w;
    } else if (ch == ':') {
        static const uint8_t col[] = {0,0x18,0x18,0,0x18,0x18,0,0};
        glyph = col;
    } else {
        /* 默认空白 */
        static const uint8_t def[] = {0,0,0,0,0,0,0,0};
        glyph = def;
    }

    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            if (glyph[row] & (0x80 >> col)) {
                if (scale == 1)
                    ST7789_DrawPixel(x + col, y + row, fg);
                else
                    ST7789_FillRect(x + col * scale, y + row * scale, scale, scale, fg);
            } else if (bg != COLOR_BLACK) {
                if (scale == 1)
                    ST7789_DrawPixel(x + col, y + row, bg);
                else
                    ST7789_FillRect(x + col * scale, y + row * scale, scale, scale, bg);
            }
        }
    }
}

/* 绘制字符串 */
static void DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale)
{
    while (*str) {
        DrawChar8x8(x, y, *str, fg, bg, scale);
        x += 8 * scale;
        str++;
    }
}

/* 绘制浮点数 */
static void DrawFloat(uint16_t x, uint16_t y, float val, uint8_t decimals, uint16_t fg, uint16_t bg, uint8_t scale)
{
    char buf[16];
    int ipart = (int)val;
    int neg = 0;
    if (val < 0) { neg = 1; ipart = -ipart; val = -val; }

    /* 整数部分 */
    char *p = buf;
    if (neg) *p++ = '-';
    if (ipart == 0) { *p++ = '0'; }
    else {
        char tmp[8]; int i = 0;
        while (ipart > 0) { tmp[i++] = '0' + ipart % 10; ipart /= 10; }
        while (i > 0) *p++ = tmp[--i];
    }

    /* 小数部分 */
    if (decimals > 0) {
        *p++ = '.';
        float frac = val - (float)(int)val;
        for (int i = 0; i < decimals; i++) {
            frac *= 10.0f;
            *p++ = '0' + (int)frac % 10;
        }
    }
    *p = '\0';
    DrawString(x, y, buf, fg, bg, scale);
}

/* ===== ADC 读取 ===== */
static uint16_t ADC_Read(void)
{
    DL_ADC12_startConversion(ADC0);
    while (!DL_ADC12_isConversionComplete(ADC0)) {}
    return DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
}

/* ===== 仪表盘绘制 ===== */

/* 圆形仪表盘中心 */
#define GAUGE_CX   120
#define GAUGE_CY   150
#define GAUGE_R    90

/* 清除仪表区域 */
static void ClearGaugeArea(void)
{
    ST7789_FillRect(0, 30, ST7789_WIDTH, 260, COLOR_BLACK);
}

/* 绘制仪表刻度环 */
static void DrawGaugeRing(const GaugeConfig_t *cfg)
{
    /* 外环 */
    ST7789_DrawCircle(GAUGE_CX, GAUGE_CY, GAUGE_R, GAUGE_RING_COLOR);
    ST7789_DrawCircle(GAUGE_CX, GAUGE_CY, GAUGE_R - 1, GAUGE_RING_COLOR);

    /* 刻度线 — 从225°到-45°（270°扇面） */
    float start_angle = 225.0f * 3.14159f / 180.0f;
    float sweep = 270.0f * 3.14159f / 180.0f;

    for (int i = 0; i <= cfg->scale_count; i++) {
        float angle = start_angle - sweep * i / cfg->scale_count;
        float cos_a = cosf(angle);
        float sin_a = sinf(angle);

        /* 主刻度线 */
        int x0 = GAUGE_CX + (int)(cos_a * (GAUGE_R - 12));
        int y0 = GAUGE_CY - (int)(sin_a * (GAUGE_R - 12));
        int x1 = GAUGE_CX + (int)(cos_a * (GAUGE_R - 2));
        int y1 = GAUGE_CY - (int)(sin_a * (GAUGE_R - 2));
        ST7789_DrawLine(x0, y0, x1, y1, GAUGE_SCALE_COLOR);

        /* 子刻度 */
        if (i < cfg->scale_count) {
            float sub_angle = angle - sweep / cfg->scale_count * 0.5f;
            int sx0 = GAUGE_CX + (int)(cosf(sub_angle) * (GAUGE_R - 7));
            int sy0 = GAUGE_CY - (int)(sinf(sub_angle) * (GAUGE_R - 7));
            int sx1 = GAUGE_CX + (int)(cosf(sub_angle) * (GAUGE_R - 2));
            int sy1 = GAUGE_CY - (int)(sinf(sub_angle) * (GAUGE_R - 2));
            ST7789_DrawLine(sx0, sy0, sx1, sy1, COLOR_DARKGRAY);
        }

        /* 刻度值 */
        float val = cfg->min_val + (cfg->max_val - cfg->min_val) * i / cfg->scale_count;
        int lx = GAUGE_CX + (int)(cos_a * (GAUGE_R - 22));
        int ly = GAUGE_CY - (int)(sin_a * (GAUGE_R - 22));
        DrawChar8x8(lx - 4, ly - 4, '0' + ((int)val / 10) % 10, GAUGE_SCALE_COLOR, COLOR_BLACK, 1);
        DrawChar8x8(lx + 4, ly - 4, '0' + (int)val % 10, GAUGE_SCALE_COLOR, COLOR_BLACK, 1);
    }

    /* 绘制警告区域弧线 */
    /* 危险区域用红色表示 */
    float warn_angle = start_angle - sweep * (cfg->warning_val - cfg->min_val) / (cfg->max_val - cfg->min_val);
    float danger_angle = start_angle - sweep * (cfg->danger_val - cfg->min_val) / (cfg->max_val - cfg->min_val);
    /* 警告区域：橙色 */
    for (float a = warn_angle; a >= danger_angle; a -= 0.02f) {
        int px = GAUGE_CX + (int)(cosf(a) * GAUGE_R);
        int py = GAUGE_CY - (int)(sinf(a) * GAUGE_R);
        ST7789_DrawPixel(px, py, GAUGE_DANGER_COLOR);
    }
    /* 危险区域：红色 */
    float end_angle = start_angle - sweep;
    for (float a = danger_angle; a >= end_angle; a -= 0.02f) {
        int px = GAUGE_CX + (int)(cosf(a) * GAUGE_R);
        int py = GAUGE_CY - (int)(sinf(a) * GAUGE_R);
        ST7789_DrawPixel(px, py, COLOR_RED);
    }
}

/* 绘制指针 */
static void DrawNeedle(float value, const GaugeConfig_t *cfg, uint16_t color)
{
    float start_angle = 225.0f * 3.14159f / 180.0f;
    float sweep = 270.0f * 3.14159f / 180.0f;

    /* 限制范围 */
    if (value < cfg->min_val) value = cfg->min_val;
    if (value > cfg->max_val) value = cfg->max_val;

    float ratio = (value - cfg->min_val) / (cfg->max_val - cfg->min_val);
    float angle = start_angle - sweep * ratio;
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);

    /* 指针尖端 */
    int tip_x = GAUGE_CX + (int)(cos_a * (GAUGE_R - 15));
    int tip_y = GAUGE_CY - (int)(sin_a * (GAUGE_R - 15));
    /* 指针尾部 */
    int tail_x = GAUGE_CX - (int)(cos_a * 12);
    int tail_y = GAUGE_CY + (int)(sin_a * 12);

    ST7789_DrawThickLine(tail_x, tail_y, tip_x, tip_y, 2, color);

    /* 中心圆 */
    ST7789_FillCircle(GAUGE_CX, GAUGE_CY, 6, color);
    ST7789_FillCircle(GAUGE_CX, GAUGE_CY, 3, COLOR_BLACK);
}

/* 绘制仪表盘标题栏 */
static void DrawStatusBar(const char *title, uint16_t color)
{
    ST7789_FillRect(0, 0, ST7789_WIDTH, 28, COLOR_DARKBLUE);
    DrawString(4, 6, title, COLOR_WHITE, COLOR_DARKBLUE, 2);
    /* 分隔线 */
    ST7789_FillRect(0, 28, ST7789_WIDTH, 2, color);
}

/* 绘制数据面板 */
static void DrawDataPanel(float value, const GaugeConfig_t *cfg, uint16_t color)
{
    ST7789_FillRect(20, 250, 200, 50, GAUGE_BG_COLOR);
    ST7789_DrawCircle(GAUGE_CX, 275, 48, GAUGE_RING_COLOR);

    /* 数值显示 */
    DrawFloat(60, 260, value, 1, COLOR_WHITE, GAUGE_BG_COLOR, 3);
    DrawString(170, 270, cfg->unit, color, GAUGE_BG_COLOR, 2);

    /* 状态指示 */
    const char *status = "NORMAL";
    uint16_t st_color = GAUGE_SAFE_COLOR;
    if (value >= cfg->danger_val) {
        status = "DANGER!";
        st_color = COLOR_RED;
    } else if (value >= cfg->warning_val) {
        status = "WARNING";
        st_color = GAUGE_DANGER_COLOR;
    }
    DrawString(70, 290, status, st_color, GAUGE_BG_COLOR, 1);
}

/* ===== 波形图表 ===== */
#define WAVE_X_START   30
#define WAVE_X_END     230
#define WAVE_Y_START   40
#define WAVE_Y_END     270
#define WAVE_WIDTH     (WAVE_X_END - WAVE_X_START)
#define WAVE_HEIGHT    (WAVE_Y_END - WAVE_Y_START)
#define WAVE_POINTS    WAVE_WIDTH

static float wave_buffer[WAVE_POINTS];
static uint16_t wave_idx = 0;

/* 清除波形区域并绘制网格 */
static void DrawWaveGrid(const GaugeConfig_t *cfg)
{
    ST7789_FillRect(0, 30, ST7789_WIDTH, 270, COLOR_BLACK);

    /* 边框 */
    ST7789_DrawLine(WAVE_X_START, WAVE_Y_START, WAVE_X_START, WAVE_Y_END, COLOR_DARKGRAY);
    ST7789_DrawLine(WAVE_X_START, WAVE_Y_END, WAVE_X_END, WAVE_Y_END, COLOR_DARKGRAY);

    /* 水平网格线 */
    for (int i = 0; i <= 4; i++) {
        int y = WAVE_Y_START + WAVE_HEIGHT * i / 4;
        for (int x = WAVE_X_START; x < WAVE_X_END; x += 4)
            ST7789_DrawPixel(x, y, COLOR_DARKGRAY);
        /* Y轴标签 */
        float val = cfg->max_val - (cfg->max_val - cfg->min_val) * i / 4;
        DrawFloat(0, y - 4, val, 1, COLOR_GRAY, COLOR_BLACK, 1);
    }

    /* 垂直网格线 */
    for (int i = 0; i <= 4; i++) {
        int x = WAVE_X_START + WAVE_WIDTH * i / 4;
        for (int y = WAVE_Y_START; y < WAVE_Y_END; y += 4)
            ST7789_DrawPixel(x, y, COLOR_DARKGRAY);
    }
}

/* 添加波形数据点 */
static void WaveAddPoint(float value, const GaugeConfig_t *cfg)
{
    wave_buffer[wave_idx] = value;
    wave_idx = (wave_idx + 1) % WAVE_POINTS;
}

/* 绘制波形 */
static void DrawWaveform(const GaugeConfig_t *cfg)
{
    float range = cfg->max_val - cfg->min_val;

    /* 清除波形区域 */
    ST7789_FillRect(WAVE_X_START + 1, WAVE_Y_START, WAVE_WIDTH - 1, WAVE_HEIGHT, COLOR_BLACK);
    /* 重绘网格 */
    for (int i = 0; i <= 4; i++) {
        int y = WAVE_Y_START + WAVE_HEIGHT * i / 4;
        for (int x = WAVE_X_START; x < WAVE_X_END; x += 4)
            ST7789_DrawPixel(x, y, COLOR_DARKGRAY);
    }

    /* 绘制曲线 */
    int prev_x = WAVE_X_START;
    int prev_y = WAVE_Y_START + (int)(WAVE_HEIGHT * (1.0f - (wave_buffer[0] - cfg->min_val) / range));

    for (int i = 1; i < WAVE_POINTS; i++) {
        int idx = (wave_idx + i) % WAVE_POINTS;
        int x = WAVE_X_START + i;
        float ratio = (wave_buffer[idx] - cfg->min_val) / range;
        if (ratio < 0) ratio = 0; if (ratio > 1) ratio = 1;
        int y = WAVE_Y_START + (int)(WAVE_HEIGHT * (1.0f - ratio));

        ST7789_DrawLine(prev_x, prev_y, x, y, cfg->color);
        prev_x = x; prev_y = y;
    }

    /* 实时值显示 */
    float last_val = wave_buffer[(wave_idx + WAVE_POINTS - 1) % WAVE_POINTS];
    DrawFloat(WAVE_X_END + 2, WAVE_Y_START + 2, last_val, 2, COLOR_WHITE, COLOR_BLACK, 2);
    DrawString(WAVE_X_END + 2, WAVE_Y_START + 22, cfg->unit, cfg->color, COLOR_BLACK, 1);
}

/* ===== 仪表盘主循环 ===== */
static DashboardMode_t current_mode = DASH_VOLTAGE;
static float current_value = 0.0f;
static float target_value = 0.0f;
static float smooth_value = 0.0f;  /* 动画平滑值 */
static uint32_t anim_counter = 0;

/* 值平滑插值（指针动画） */
static float SmoothApproach(float current, float target, float speed)
{
    float diff = target - current;
    if (fabsf(diff) < 0.05f) return target;
    return current + diff * speed;
}

/* 初始化波形缓冲区 */
static void WaveBuffer_Init(void)
{
    memset(wave_buffer, 0, sizeof(wave_buffer));
    wave_idx = 0;
}

/* ===== 延时函数 ===== */
static void delay_ms(uint32_t ms)
{
    delay_cycles(ms * 16000);  /* ~16MHz */
}

/* ===== 模拟数据源（用于演示，实际可替换为传感器读取）===== */
static float SimulateData(DashboardMode_t mode)
{
    static uint32_t tick = 0;
    tick++;

    switch (mode) {
    case DASH_VOLTAGE: {
        /* 读取ADC并转换为电压 (3.3V参考，12位，分压比10) */
        uint16_t adc = ADC_Read();
        return (float)adc / 4095.0f * 3.3f * 10.0f;  /* 0~33V */
    }
    case DASH_SPEED:
        /* 模拟正弦波动的速度 */
        return 4000.0f + 3000.0f * sinf((float)tick * 0.05f);
    case DASH_TEMPERATURE:
        /* 模拟缓慢升温 */
        return 25.0f + 30.0f * (1.0f - expf(-(float)tick * 0.01f))
             + 3.0f * sinf((float)tick * 0.1f);
    default:
        return 0.0f;
    }
}

/* ===== 按键消抖 ===== */
static uint8_t last_btn = 0;
static uint8_t btn_pressed = 0;

static void CheckButton(void)
{
    uint8_t now = BTN_MODE ? 1 : 0;
    if (now && !last_btn) btn_pressed = 1;
    last_btn = now;
}

/* ===== 模式切换重绘 ===== */
static uint8_t need_redraw = 1;

static void SwitchMode(void)
{
    current_mode = (DashboardMode_t)((current_mode + 1) % DASH_MODE_COUNT);
    need_redraw = 1;
    smooth_value = gauge_cfg[current_mode].min_val;
    WaveBuffer_Init();
}

/* ===== 主函数 ===== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* ADC校准 */
    DL_ADC12_startConversion(ADC0);
    DL_ADC12_enableConversions(ADC0);

    /* TFT初始化 */
    ST7789_Init();
    ST7789_FillScreen(COLOR_BLACK);

    /* 显示启动画面 */
    DrawString(40, 80, "DIGITAL", COLOR_CYAN, COLOR_BLACK, 3);
    DrawString(40, 120, "DASHBOARD", COLOR_CYAN, COLOR_BLACK, 3);
    DrawString(50, 170, "V1.0", COLOR_GRAY, COLOR_BLACK, 2);
    delay_ms(1500);

    /* 初始绘制 */
    DrawStatusBar(gauge_cfg[current_mode].title, gauge_cfg[current_mode].color);
    DrawGaugeRing(&gauge_cfg[current_mode]);
    need_redraw = 0;

    /* ===== 主循环 ===== */
    while (1) {
        /* 按键检测 */
        CheckButton();
        if (btn_pressed) {
            btn_pressed = 0;
            SwitchMode();
        }

        /* 重绘模式 */
        if (need_redraw) {
            need_redraw = 0;
            const GaugeConfig_t *cfg = &gauge_cfg[current_mode];
            DrawStatusBar(cfg->title, cfg->color);
            if (current_mode == DASH_WAVEFORM) {
                DrawWaveGrid(cfg);
            } else {
                ClearGaugeArea();
                DrawGaugeRing(cfg);
            }
        }

        /* 读取数据 */
        if (current_mode == DASH_WAVEFORM) {
            target_value = SimulateData(DASH_VOLTAGE); /* 波形图也读ADC */
            WaveAddPoint(target_value, &gauge_cfg[DASH_WAVEFORM]);
            DrawWaveform(&gauge_cfg[DASH_WAVEFORM]);
        } else {
            target_value = SimulateData(current_mode);
            smooth_value = SmoothApproach(smooth_value, target_value, 0.15f);

            /* 重绘指针（擦除旧指针用背景色，再画新指针） */
            DrawNeedle(smooth_value, &gauge_cfg[current_mode], COLOR_BLACK);
            smooth_value = SmoothApproach(smooth_value, target_value, 0.15f);
            DrawNeedle(smooth_value, &gauge_cfg[current_mode],
                       gauge_cfg[current_mode].color);

            /* 更新数据面板 */
            DrawDataPanel(smooth_value, &gauge_cfg[current_mode],
                         gauge_cfg[current_mode].color);
        }

        anim_counter++;
        delay_ms(30);  /* ~33fps */
    }

    return 0;
}
