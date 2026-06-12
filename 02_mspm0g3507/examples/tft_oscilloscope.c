/**
 * @file tft_oscilloscope.c
 * @brief TFT示波器 - ILI9341 + ADC采样 + 波形显示 + 触发
 * @platform MSPM0G3507
 *
 * 硬件连接：
 *   ILI9341 TFT (SPI):
 *     SCK  -> PA10 (SPI0_SCK)
 *     MOSI -> PA8  (SPI0_MOSI)
 *     CS   -> PA12 (GPIO)
 *     DC   -> PA13 (GPIO)
 *     RST  -> PA14 (GPIO)
 *     BL   -> PA15 (GPIO)
 *
 *   ADC输入:
 *     PA25 -> ADC0通道5 (被测信号输入, 0~3.3V)
 *
 *   按键:
 *     PB0 -> 触发电平+
 *     PB1 -> 触发电平-
 *     PB2 -> 时基切换
 *     PB3 -> 触发模式切换
 *
 * 功能：实时波形显示、自动/手动触发、时基可调、电压刻度
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===== 显示参数 ===== */
#define TFT_WIDTH   320
#define TFT_HEIGHT  240

/* 波形显示区域 */
#define WAVE_X_START  40
#define WAVE_X_END    310
#define WAVE_Y_START  10
#define WAVE_Y_END    200
#define WAVE_WIDTH    (WAVE_X_END - WAVE_X_START)
#define WAVE_HEIGHT   (WAVE_Y_END - WAVE_Y_START)

#define CS_LOW()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_12)
#define CS_HIGH()  DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_12)
#define DC_LOW()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13)
#define DC_HIGH()  DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_13)
#define RST_LOW()  DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14)
#define RST_HIGH() DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14)
#define BL_ON()    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_15)

/* 颜色 */
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF
#define COLOR_GRAY    0x8410
#define COLOR_DARKGRAY 0x4208
#define COLOR_BG      0x0000

/* ADC参数 */
#define ADC_MAX_VALUE   4095    /* 12位ADC */
#define ADC_VREF_MV     3300    /* 参考电压3.3V */
#define SAMPLE_BUF_SIZE 300     /* 采样缓冲区大小(匹配波形宽度) */

/* 触发模式 */
typedef enum {
    TRIG_AUTO,      /* 自动触发 */
    TRIG_NORMAL,    /* 普通触发 */
    TRIG_SINGLE     /* 单次触发 */
} TrigMode;

/* 时基设置 */
typedef struct {
    const char *name;
    uint32_t sample_delay_us;  /* 采样间隔(us) */
    uint32_t time_per_div_us;  /* 每格时间 */
} TimeBase;

static const TimeBase timebases[] = {
    {"1ms/div",  40,   1000},
    {"2ms/div",  80,   2000},
    {"5ms/div",  200,  5000},
    {"10ms/div", 400,  10000},
    {"20ms/div", 800,  20000},
};
#define TIMEBASE_COUNT 5

/* 全局变量 */
static uint16_t sample_buffer[SAMPLE_BUF_SIZE];  /* ADC采样缓冲区 */
static uint16_t last_waveform[SAMPLE_BUF_SIZE];   /* 上一次波形(用于擦除) */
static uint16_t trig_level = 2048;                 /* 触发电平 */
static TrigMode trig_mode = TRIG_AUTO;
static int timebase_idx = 0;
static bool waveform_ready = false;
static uint32_t freq_estimate = 0;   /* 频率估计 */
static uint32_t vpp_mv = 0;          /* 峰峰值(mV) */
static uint32_t vrms_mv = 0;         /* 有效值(mV) */

/* ===== SPI + ILI9341 驱动 ===== */
static void SPI_WriteByte(uint8_t dat)
{
    DL_SPI_transmitData8(SPI0, dat);
    while (DL_SPI_isBusy(SPI0)) {}
}

static void ILI9341_WriteCmd(uint8_t cmd)
{
    DC_LOW(); CS_LOW();
    SPI_WriteByte(cmd);
    CS_HIGH();
}

static void ILI9341_WriteData(uint8_t dat)
{
    DC_HIGH(); CS_LOW();
    SPI_WriteByte(dat);
    CS_HIGH();
}

static void ILI9341_WriteData16(uint16_t dat)
{
    DC_HIGH(); CS_LOW();
    SPI_WriteByte(dat >> 8);
    SPI_WriteByte(dat & 0xFF);
    CS_HIGH();
}

static void delay_ms(uint32_t ms)
{
    delay_cycles(ms * (CPUCLK_FREQ / 1000));
}

static void delay_us(uint32_t us)
{
    delay_cycles(us * (CPUCLK_FREQ / 1000000));
}

static void ILI9341_Init(void)
{
    RST_LOW(); delay_ms(20);
    RST_HIGH(); delay_ms(120);

    ILI9341_WriteCmd(0x01);  /* Software Reset */
    delay_ms(120);

    ILI9341_WriteCmd(0xCB); ILI9341_WriteData(0x39);
    ILI9341_WriteData(0x2C); ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x34); ILI9341_WriteData(0x02);

    ILI9341_WriteCmd(0xCF); ILI9341_WriteData(0x00);
    ILI9341_WriteData(0xC1); ILI9341_WriteData(0x30);

    ILI9341_WriteCmd(0xE8); ILI9341_WriteData(0x85);
    ILI9341_WriteData(0x00); ILI9341_WriteData(0x78);

    ILI9341_WriteCmd(0x3A); ILI9341_WriteData(0x55); /* 16bit */

    ILI9341_WriteCmd(0x36); ILI9341_WriteData(0x28); /* 横屏 */

    ILI9341_WriteCmd(0x11); delay_ms(120);
    ILI9341_WriteCmd(0x29); delay_ms(20);
    BL_ON();
}

static void ILI9341_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    ILI9341_WriteCmd(0x2A);
    ILI9341_WriteData16(x0); ILI9341_WriteData16(x1);
    ILI9341_WriteCmd(0x2B);
    ILI9341_WriteData16(y0); ILI9341_WriteData16(y1);
    ILI9341_WriteCmd(0x2C);
}

static void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ILI9341_SetWindow(x, y, x + w - 1, y + h - 1);
    for (uint32_t i = 0; i < (uint32_t)w * h; i++)
        ILI9341_WriteData16(color);
}

static void ILI9341_Clear(uint16_t color)
{
    ILI9341_FillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, color);
}

/* ===== 绘制像素点 ===== */
static void Draw_Pixel(uint16_t x, uint16_t y, uint16_t color)
{
    ILI9341_SetWindow(x, y, x, y);
    ILI9341_WriteData16(color);
}

/* ===== 绘制数字(3x5像素字体) ===== */
static const uint8_t font3x5[][5] = {
    {0b111,0b101,0b101,0b101,0b111}, {0b010,0b110,0b010,0b010,0b111},
    {0b111,0b001,0b111,0b100,0b111}, {0b111,0b001,0b111,0b001,0b111},
    {0b101,0b101,0b111,0b001,0b001}, {0b111,0b100,0b111,0b001,0b111},
    {0b111,0b100,0b111,0b101,0b111}, {0b111,0b001,0b001,0b001,0b001},
    {0b111,0b101,0b111,0b101,0b111}, {0b111,0b101,0b111,0b001,0b111},
};

static void Draw_DigitScaled(uint16_t x, uint16_t y, uint8_t d, uint16_t color, uint8_t s)
{
    if (d > 9) return;
    for (int r = 0; r < 5; r++)
        for (int c = 0; c < 3; c++)
            if (font3x5[d][r] & (1 << (2 - c)))
                ILI9341_FillRect(x + c * s, y + r * s, s, s, color);
}

static void Draw_Number(uint16_t x, uint16_t y, int num, uint16_t color, uint8_t s)
{
    if (num < 0) { x += s; num = -num; }
    char buf[10]; int len = 0;
    if (num == 0) { buf[0] = '0'; len = 1; }
    else { int t = num; char r[10]; int rl = 0;
        while (t > 0) { r[rl++] = '0' + t % 10; t /= 10; }
        for (int i = rl - 1; i >= 0; i--) buf[len++] = r[i];
    }
    for (int i = 0; i < len; i++) {
        Draw_DigitScaled(x, y, buf[i] - '0', color, s);
        x += 4 * s;
    }
}

/* ===== ADC采样 ===== */
static uint16_t ADC_Read(void)
{
    DL_ADC12_startConversion(ADC0);
    while (!DL_ADC12_getStatus(ADC0, DL_ADC12_STATUS_CONVERSION_DONE)) {}
    return (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
}

/* ===== 采集波形 ===== */
static void Capture_Waveform(void)
{
    uint32_t delay_us_val = timebases[timebase_idx].sample_delay_us;
    bool triggered = false;
    uint32_t timeout = 0;

    /* 寻找触发点 */
    if (trig_mode == TRIG_AUTO || trig_mode == TRIG_NORMAL) {
        /* 预采集一小段寻找触发 */
        uint16_t prev = ADC_Read();
        uint16_t curr;
        while (!triggered && timeout < 100000) {
            curr = ADC_Read();
            /* 上升沿触发: 从低于触发电平到高于触发电平 */
            if (prev < trig_level && curr >= trig_level) {
                triggered = true;
            }
            prev = curr;
            timeout++;
            delay_us(5);
        }

        /* 自动模式超时也继续 */
        if (!triggered && trig_mode == TRIG_AUTO) {
            triggered = true;
        }
    } else {
        triggered = true;
    }

    /* 正式采样 */
    if (triggered) {
        for (int i = 0; i < SAMPLE_BUF_SIZE; i++) {
            sample_buffer[i] = ADC_Read();
            delay_us(delay_us_val);
        }
        waveform_ready = true;
    }
}

/* ===== 分析波形参数 ===== */
static void Analyze_Waveform(void)
{
    uint16_t vmax = 0, vmin = 4095;
    uint32_t sum = 0;
    int zero_cross = 0;

    for (int i = 0; i < SAMPLE_BUF_SIZE; i++) {
        uint16_t v = sample_buffer[i];
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
        sum += v;
        /* 简单过零检测(以均值为参考) */
        if (i > 0) {
            uint16_t avg = sum / (i + 1);
            if ((sample_buffer[i - 1] < avg && sample_buffer[i] >= avg) ||
                (sample_buffer[i - 1] >= avg && sample_buffer[i] < avg)) {
                zero_cross++;
            }
        }
    }

    vpp_mv = (uint32_t)(vmax - vmin) * ADC_VREF_MV / ADC_MAX_VALUE;
    vrms_mv = vpp_mv / 2828 * 1000; /* 近似: Vpp/(2*sqrt(2)) */

    /* 频率估计: 过零次数/2 / 采样总时间 */
    uint32_t total_time_us = SAMPLE_BUF_SIZE * timebases[timebase_idx].sample_delay_us;
    if (total_time_us > 0 && zero_cross > 1) {
        freq_estimate = (uint32_t)zero_cross * 500000UL / total_time_us;
    }
}

/* ===== 绘制波形 ===== */
static void Draw_Waveform(void)
{
    /* 擦除旧波形 */
    for (int i = 0; i < SAMPLE_BUF_SIZE - 1; i++) {
        uint16_t x = WAVE_X_START + i;
        uint16_t y1 = last_waveform[i];
        uint16_t y2 = last_waveform[i + 1];
        /* 只擦除波形线附近的像素 */
        uint16_t ymin = (y1 < y2) ? y1 : y2;
        uint16_t ymax = (y1 > y2) ? y1 : y2;
        if (ymin > 0) ymin--;
        if (ymax < WAVE_Y_END) ymax++;
        for (uint16_t y = ymin; y <= ymax; y++) {
            Draw_Pixel(x, y, COLOR_BG);
        }
    }

    /* 绘制网格 */
    for (int i = 0; i <= 5; i++) {
        uint16_t gy = WAVE_Y_START + i * WAVE_HEIGHT / 5;
        for (uint16_t x = WAVE_X_START; x < WAVE_X_END; x += 4) {
            Draw_Pixel(x, gy, COLOR_DARKGRAY);
        }
    }
    for (int i = 0; i <= 10; i++) {
        uint16_t gx = WAVE_X_START + i * WAVE_WIDTH / 10;
        for (uint16_t y = WAVE_Y_START; y < WAVE_Y_END; y += 4) {
            Draw_Pixel(gx, y, COLOR_DARKGRAY);
        }
    }

    /* 绘制触发电平线 */
    uint16_t trig_y = WAVE_Y_END - (uint32_t)trig_level * WAVE_HEIGHT / ADC_MAX_VALUE;
    if (trig_y > WAVE_Y_START && trig_y < WAVE_Y_END) {
        for (uint16_t x = WAVE_X_START; x < WAVE_X_END; x += 2) {
            Draw_Pixel(x, trig_y, COLOR_YELLOW);
        }
    }

    /* 将ADC值映射到屏幕Y坐标 */
    for (int i = 0; i < SAMPLE_BUF_SIZE; i++) {
        uint16_t y = WAVE_Y_END - (uint32_t)sample_buffer[i] * WAVE_HEIGHT / ADC_MAX_VALUE;
        if (y < WAVE_Y_START) y = WAVE_Y_START;
        if (y >= WAVE_Y_END) y = WAVE_Y_END - 1;
        last_waveform[i] = y;
    }

    /* 绘制新波形(连线) */
    for (int i = 0; i < SAMPLE_BUF_SIZE - 1; i++) {
        uint16_t x = WAVE_X_START + i;
        uint16_t y1 = last_waveform[i];
        uint16_t y2 = last_waveform[i + 1];
        /* Bresenham连线 */
        int dy = (y2 > y1) ? 1 : -1;
        int ady = (y2 > y1) ? (y2 - y1) : (y1 - y2);
        if (ady <= 1) {
            Draw_Pixel(x, y1, COLOR_GREEN);
        } else {
            int y = y1;
            for (int step = 0; step < ady; step++) {
                Draw_Pixel(x, y, COLOR_GREEN);
                y += dy;
            }
        }
    }
    /* 最后一个点 */
    Draw_Pixel(WAVE_X_START + SAMPLE_BUF_SIZE - 1, last_waveform[SAMPLE_BUF_SIZE - 1], COLOR_GREEN);
}

/* ===== 绘制信息栏 ===== */
static void Draw_InfoBar(void)
{
    /* 顶部信息栏 */
    ILI9341_FillRect(0, WAVE_Y_END + 2, TFT_WIDTH, 40, COLOR_BG);

    /* 触发模式 */
    const char *mode_str;
    uint16_t mode_color;
    switch (trig_mode) {
        case TRIG_AUTO:   mode_str = "AUTO";   mode_color = COLOR_GREEN; break;
        case TRIG_NORMAL: mode_str = "NORM";   mode_color = COLOR_YELLOW; break;
        case TRIG_SINGLE: mode_str = "SING";   mode_color = COLOR_CYAN; break;
    }

    /* 频率 */
    Draw_Number(5, WAVE_Y_END + 4, (int)freq_estimate, COLOR_CYAN, 2);
    /* 单位Hz标识: 用小方块模拟 */

    /* 时基 */
    Draw_Number(5, WAVE_Y_END + 22, timebases[timebase_idx].time_per_div_us, COLOR_WHITE, 2);

    /* Vpp */
    Draw_Number(120, WAVE_Y_END + 4, (int)vpp_mv, COLOR_YELLOW, 2);

    /* 触发电平 */
    Draw_Number(120, WAVE_Y_END + 22, (int)((uint32_t)trig_level * ADC_VREF_MV / ADC_MAX_VALUE), mode_color, 2);

    /* 左侧电压刻度 */
    ILI9341_FillRect(0, 0, WAVE_X_START - 2, TFT_HEIGHT, COLOR_BG);
    for (int i = 0; i <= 5; i++) {
        uint16_t y = WAVE_Y_START + i * WAVE_HEIGHT / 5;
        int mv = ADC_VREF_MV * (5 - i) / 5;
        Draw_Number(2, y - 3, mv, COLOR_WHITE, 1);
    }
}

/* ===== 主函数 ===== */
int main(void)
{
    SYSCFG_DL_init();
    ILI9341_Init();
    ILI9341_Clear(COLOR_BG);

    memset(last_waveform, 0, sizeof(last_waveform));

    /* 主循环 */
    while (1) {
        /* 读取按键 */
        if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_0))) {
            /* 触发电平+ */
            if (trig_level < ADC_MAX_VALUE - 100) trig_level += 100;
            delay_ms(200);
        }
        if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_1))) {
            /* 触发电平- */
            if (trig_level > 100) trig_level -= 100;
            delay_ms(200);
        }
        if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2))) {
            /* 时基切换 */
            timebase_idx = (timebase_idx + 1) % TIMEBASE_COUNT;
            delay_ms(200);
        }
        if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3))) {
            /* 触发模式切换 */
            trig_mode = (TrigMode)(((int)trig_mode + 1) % 3);
            delay_ms(200);
        }

        /* 采集 */
        Capture_Waveform();

        if (waveform_ready) {
            /* 分析 */
            Analyze_Waveform();
            /* 显示 */
            Draw_Waveform();
            Draw_InfoBar();
        }
    }
}
