/**
 * @file led_matrix_display.c
 * @brief MSPM0G3507 LED点阵显示屏示例
 *
 * 硬件连接（MAX7219 8x8 LED点阵模块）：
 *   DIN  -> PA15 (SPI MOSI)
 *   CS   -> PA14 (GPIO片选)
 *   CLK  -> PA13 (SPI SCK)
 *   VCC  -> 3.3V
 *   GND  -> GND
 *
 * 功能：滚动显示文字 + 多种动画效果
 */

#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdint.h>

/* ========== MAX7219 寄存器定义 ========== */
#define MAX7219_REG_NOOP        0x00
#define MAX7219_REG_DIGIT0      0x01
#define MAX7219_REG_DIGIT1      0x02
#define MAX7219_REG_DIGIT2      0x03
#define MAX7219_REG_DIGIT3      0x04
#define MAX7219_REG_DIGIT4      0x05
#define MAX7219_REG_DIGIT5      0x06
#define MAX7219_REG_DIGIT6      0x07
#define MAX7219_REG_DIGIT7      0x08
#define MAX7219_REG_DECODEMODE  0x09
#define MAX7219_REG_INTENSITY   0x0A
#define MAX7219_REG_SCANLIMIT   0x0B
#define MAX7219_REG_SHUTDOWN    0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F

/* 级联模块数量 */
#define MATRIX_MODULES  4
#define MATRIX_COLS     (8 * MATRIX_MODULES)

/* 片选引脚定义（根据实际修改） */
#define CS_PORT     GPIOA
#define CS_PIN      DL_GPIO_PIN_14

/* ========== 5x7 ASCII字模表（部分常用字符）========== */
/* 每个字符5列，每列8位（实际用7位，MSB在上） */
static const uint8_t font5x7[][5] = {
    /* ' ' */ {0x00, 0x00, 0x00, 0x00, 0x00},
    /* '!' */ {0x00, 0x00, 0x5F, 0x00, 0x00},
    /* '"' */ {0x00, 0x07, 0x00, 0x07, 0x00},
    /* '#' */ {0x14, 0x7F, 0x14, 0x7F, 0x14},
    /* '$' */ {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    /* '%' */ {0x23, 0x13, 0x08, 0x64, 0x62},
    /* '&' */ {0x36, 0x49, 0x55, 0x22, 0x50},
    /* ''' */ {0x00, 0x05, 0x03, 0x00, 0x00},
    /* '(' */ {0x00, 0x1C, 0x22, 0x41, 0x00},
    /* ')' */ {0x00, 0x41, 0x22, 0x1C, 0x00},
    /* '*' */ {0x14, 0x08, 0x3E, 0x08, 0x14},
    /* '+' */ {0x08, 0x08, 0x3E, 0x08, 0x08},
    /* ',' */ {0x00, 0x50, 0x30, 0x00, 0x00},
    /* '-' */ {0x08, 0x08, 0x08, 0x08, 0x08},
    /* '.' */ {0x00, 0x60, 0x60, 0x00, 0x00},
    /* '/' */ {0x20, 0x10, 0x08, 0x04, 0x02},
    /* '0' */ {0x3E, 0x51, 0x49, 0x45, 0x3E},
    /* '1' */ {0x00, 0x42, 0x7F, 0x40, 0x00},
    /* '2' */ {0x42, 0x61, 0x51, 0x49, 0x46},
    /* '3' */ {0x21, 0x41, 0x45, 0x4B, 0x31},
    /* '4' */ {0x18, 0x14, 0x12, 0x7F, 0x10},
    /* '5' */ {0x27, 0x45, 0x45, 0x45, 0x39},
    /* '6' */ {0x3C, 0x4A, 0x49, 0x49, 0x30},
    /* '7' */ {0x01, 0x71, 0x09, 0x05, 0x03},
    /* '8' */ {0x36, 0x49, 0x49, 0x49, 0x36},
    /* '9' */ {0x06, 0x49, 0x49, 0x29, 0x1E},
    /* ':' */ {0x00, 0x36, 0x36, 0x00, 0x00},
    /* ';' */ {0x00, 0x56, 0x36, 0x00, 0x00},
    /* '<' */ {0x08, 0x14, 0x22, 0x41, 0x00},
    /* '=' */ {0x14, 0x14, 0x14, 0x14, 0x14},
    /* '>' */ {0x00, 0x41, 0x22, 0x14, 0x08},
    /* '?' */ {0x02, 0x01, 0x51, 0x09, 0x06},
    /* '@' */ {0x32, 0x49, 0x59, 0x51, 0x3E},
    /* 'A' */ {0x7E, 0x11, 0x11, 0x11, 0x7E},
    /* 'B' */ {0x7F, 0x49, 0x49, 0x49, 0x36},
    /* 'C' */ {0x3E, 0x41, 0x41, 0x41, 0x22},
    /* 'D' */ {0x7F, 0x41, 0x41, 0x22, 0x1C},
    /* 'E' */ {0x7F, 0x49, 0x49, 0x49, 0x41},
    /* 'F' */ {0x7F, 0x09, 0x09, 0x09, 0x01},
    /* 'G' */ {0x3E, 0x41, 0x49, 0x49, 0x7A},
    /* 'H' */ {0x7F, 0x08, 0x08, 0x08, 0x7F},
    /* 'I' */ {0x00, 0x41, 0x7F, 0x41, 0x00},
    /* 'J' */ {0x20, 0x40, 0x41, 0x3F, 0x01},
    /* 'K' */ {0x7F, 0x08, 0x14, 0x22, 0x41},
    /* 'L' */ {0x7F, 0x40, 0x40, 0x40, 0x40},
    /* 'M' */ {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    /* 'N' */ {0x7F, 0x04, 0x08, 0x10, 0x7F},
    /* 'O' */ {0x3E, 0x41, 0x41, 0x41, 0x3E},
    /* 'P' */ {0x7F, 0x09, 0x09, 0x09, 0x06},
    /* 'Q' */ {0x3E, 0x41, 0x51, 0x21, 0x5E},
    /* 'R' */ {0x7F, 0x09, 0x19, 0x29, 0x46},
    /* 'S' */ {0x46, 0x49, 0x49, 0x49, 0x31},
    /* 'T' */ {0x01, 0x01, 0x7F, 0x01, 0x01},
    /* 'U' */ {0x3F, 0x40, 0x40, 0x40, 0x3F},
    /* 'V' */ {0x1F, 0x20, 0x40, 0x20, 0x1F},
    /* 'W' */ {0x3F, 0x40, 0x38, 0x40, 0x3F},
    /* 'X' */ {0x63, 0x14, 0x08, 0x14, 0x63},
    /* 'Y' */ {0x07, 0x08, 0x70, 0x08, 0x07},
    /* 'Z' */ {0x61, 0x51, 0x49, 0x45, 0x43},
};

/* 显示缓冲区：每模块8行，每行1字节 */
static uint8_t displayBuffer[MATRIX_MODULES][8];

/* ========== SPI发送函数（软件模拟） ========== */
static void spi_send_byte(uint8_t data)
{
    for (int i = 7; i >= 0; i--) {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13);  /* CLK低 */
        if (data & (1 << i)) {
            DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_15);    /* DIN高 */
        } else {
            DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_15);  /* DIN低 */
        }
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_13);    /* CLK高 */
    }
}

/* ========== MAX7219发送命令（级联模式）========== */
static void max7219_send(uint8_t module, uint8_t reg, uint8_t data)
{
    DL_GPIO_clearPins(CS_PORT, CS_PIN);  /* CS拉低，开始传输 */

    /* 对于级联模块，先发送目标模块，其余发NOOP */
    for (int i = MATRIX_MODULES - 1; i >= 0; i--) {
        if (i == (int)module) {
            spi_send_byte(reg);
            spi_send_byte(data);
        } else {
            spi_send_byte(MAX7219_REG_NOOP);
            spi_send_byte(0x00);
        }
    }

    DL_GPIO_setPins(CS_PORT, CS_PIN);  /* CS拉高，锁存数据 */
}

/* ========== 向所有模块发送同一命令 ========== */
static void max7219_send_all(uint8_t reg, uint8_t data)
{
    DL_GPIO_clearPins(CS_PORT, CS_PIN);
    for (int i = 0; i < MATRIX_MODULES; i++) {
        spi_send_byte(reg);
        spi_send_byte(data);
    }
    DL_GPIO_setPins(CS_PORT, CS_PIN);
}

/* ========== MAX7219初始化 ========== */
static void max7219_init(void)
{
    /* GPIO初始化 */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_15); /* DIN */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_14); /* CS  */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_13); /* CLK */
    DL_GPIO_setPins(CS_PORT, CS_PIN);

    /* 关闭显示测试 */
    max7219_send_all(MAX7219_REG_DISPLAYTEST, 0x00);
    /* 非解码模式 */
    max7219_send_all(MAX7219_REG_DECODEMODE, 0x00);
    /* 扫描全部8行 */
    max7219_send_all(MAX7219_REG_SCANLIMIT, 0x07);
    /* 亮度（0x00~0x0F） */
    max7219_send_all(MAX7219_REG_INTENSITY, 0x08);
    /* 正常模式 */
    max7219_send_all(MAX7219_REG_SHUTDOWN, 0x01);

    /* 清屏 */
    for (int row = 0; row < 8; row++) {
        max7219_send_all(MAX7219_REG_DIGIT0 + row, 0x00);
    }
}

/* ========== 更新显示缓冲区到硬件 ========== */
static void matrix_update(void)
{
    for (int row = 0; row < 8; row++) {
        DL_GPIO_clearPins(CS_PORT, CS_PIN);
        for (int m = MATRIX_MODULES - 1; m >= 0; m--) {
            spi_send_byte(MAX7219_REG_DIGIT0 + row);
            spi_send_byte(displayBuffer[m][row]);
        }
        DL_GPIO_setPins(CS_PORT, CS_PIN);
    }
}

/* ========== 清屏 ========== */
static void matrix_clear(void)
{
    memset(displayBuffer, 0, sizeof(displayBuffer));
}

/* ========== 获取字符点阵列数据 ========== */
static uint8_t get_char_col(char ch, int col)
{
    if (ch < ' ' || ch > 'Z') ch = ' ';
    return font5x7[ch - ' '][col];
}

/* ========== 滚动文字显示 ========== */
/* text: 要显示的字符串, delay_ms: 每列滚动间隔 */
static void scroll_text(const char *text, uint16_t delay_ms)
{
    int len = strlen(text);
    int totalCols = len * 6;  /* 每字符5列 + 1列间距 */

    /* 从右侧开始滚动到左侧 */
    for (int offset = MATRIX_COLS; offset >= -totalCols; offset--) {
        matrix_clear();

        for (int i = 0; i < len; i++) {
            int charStart = offset + i * 6;
            for (int col = 0; col < 5; col++) {
                int screenCol = charStart + col;
                if (screenCol >= 0 && screenCol < MATRIX_COLS) {
                    /* 计算属于哪个模块和模块内行 */
                    int mod = screenCol / 8;
                    int bit = 7 - (screenCol % 8);
                    uint8_t colData = get_char_col(text[i], col);
                    for (int row = 0; row < 8; row++) {
                        if (colData & (1 << row)) {
                            displayBuffer[mod][row] |= (1 << bit);
                        }
                    }
                }
            }
        }

        matrix_update();
        delay_cycles(delay_ms * 32000);  /* 粗略延时 */
    }
}

/* ========== 动画效果1：心跳 ========== */
static void animation_heart(uint16_t delay_ms)
{
    /* 大心形 */
    static const uint8_t heart_big[8] = {
        0x00, 0x66, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C, 0x18
    };
    /* 小心形 */
    static const uint8_t heart_small[8] = {
        0x00, 0x00, 0x24, 0x7E, 0x7E, 0x3C, 0x18, 0x00
    };

    for (int beat = 0; beat < 3; beat++) {
        /* 大心 */
        matrix_clear();
        for (int mod = 0; mod < MATRIX_MODULES; mod++) {
            for (int row = 0; row < 8; row++) {
                displayBuffer[mod][row] = heart_big[row];
            }
        }
        matrix_update();
        delay_cycles(delay_ms * 32000);

        /* 小心 */
        matrix_clear();
        for (int mod = 0; mod < MATRIX_MODULES; mod++) {
            for (int row = 0; row < 8; row++) {
                displayBuffer[mod][row] = heart_small[row];
            }
        }
        matrix_update();
        delay_cycles(delay_ms * 32000);
    }
}

/* ========== 动画效果2：波浪 ========== */
static void animation_wave(uint16_t delay_ms)
{
    for (int frame = 0; frame < 32; frame++) {
        matrix_clear();
        for (int col = 0; col < MATRIX_COLS; col++) {
            int row = (int)(3.5 + 3.5 * __sin_approx((col + frame) * 0.5));
            if (row >= 0 && row < 8) {
                int mod = col / 8;
                int bit = 7 - (col % 8);
                displayBuffer[mod][row] |= (1 << bit);
            }
        }
        matrix_update();
        delay_cycles(delay_ms * 32000);
    }
}

/* ========== 动画效果3：流星雨 ========== */
static void animation_rain(uint16_t delay_ms)
{
    /* 简单的下落粒子效果 */
    int particles[8][2];  /* [col, row] */
    for (int i = 0; i < 8; i++) {
        particles[i][0] = (i * 7 + 3) % MATRIX_COLS;
        particles[i][1] = -i * 2;  /* 起始位置错开 */
    }

    for (int frame = 0; frame < 40; frame++) {
        matrix_clear();
        for (int i = 0; i < 8; i++) {
            int row = particles[i][1];
            int col = particles[i][0];
            /* 绘制尾迹 */
            for (int tail = 0; tail < 3; tail++) {
                int r = row - tail;
                if (r >= 0 && r < 8 && col >= 0 && col < MATRIX_COLS) {
                    int mod = col / 8;
                    int bit = 7 - (col % 8);
                    displayBuffer[mod][r] |= (1 << bit);
                }
            }
            particles[i][1]++;
            if (particles[i][1] > 10) {
                particles[i][1] = -3;
                particles[i][0] = (particles[i][0] + 11) % MATRIX_COLS;
            }
        }
        matrix_update();
        delay_cycles(delay_ms * 32000);
    }
}

/* ========== 动画效果4：螺旋 ========== */
static void animation_spiral(uint16_t delay_ms)
{
    /* 螺旋填充路径 */
    static const uint8_t spiral[64][2] = {
        {3,3},{4,3},{4,4},{4,5},{3,5},{2,5},{2,4},{2,3},
        {2,2},{3,2},{4,2},{5,2},{5,3},{5,4},{5,5},{5,6},
        {4,6},{3,6},{2,6},{1,6},{1,5},{1,4},{1,3},{1,2},
        {1,1},{2,1},{3,1},{4,1},{5,1},{6,1},{6,2},{6,3},
        {6,4},{6,5},{6,6},{6,7},{5,7},{4,7},{3,7},{2,7},
        {1,7},{0,7},{0,6},{0,5},{0,4},{0,3},{0,2},{0,1},
        {0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{6,0},{7,0},
        {7,1},{7,2},{7,3},{7,4},{7,5},{7,6},{7,7},{7,7}
    };

    matrix_clear();
    for (int i = 0; i < 64; i++) {
        int row = spiral[i][0];
        int col = spiral[i][1];
        int mod = col / 8;
        int bit = 7 - (col % 8);
        displayBuffer[mod][row] |= (1 << bit);
        matrix_update();
        delay_cycles(delay_ms * 32000);
    }

    /* 反向清除 */
    for (int i = 63; i >= 0; i--) {
        int row = spiral[i][0];
        int col = spiral[i][1];
        int mod = col / 8;
        int bit = 7 - (col % 8);
        displayBuffer[mod][row] &= ~(1 << bit);
        matrix_update();
        delay_cycles(delay_ms * 32000);
    }
}

/* ========== sin近似函数（用于波浪动画）========== */
static float __sin_approx(float x)
{
    /* 简化的sin近似，用于嵌入式环境 */
    while (x > 3.14159f) x -= 6.28318f;
    while (x < -3.14159f) x += 6.28318f;
    float x2 = x * x;
    return x * (1.0f - x2 / 6.0f + x2 * x2 / 120.0f);
}

/* ========== 主函数 ========== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* MAX7219初始化 */
    max7219_init();

    /* 主循环：依次演示各种效果 */
    while (1) {
        /* 效果1：滚动文字 */
        scroll_text("MSPM0G3507 LED MATRIX DEMO", 50);

        /* 效果2：心跳动画 */
        animation_heart(200);

        /* 效果3：波浪动画 */
        animation_wave(100);

        /* 效果4：流星雨 */
        animation_rain(80);

        /* 效果5：螺旋 */
        animation_spiral(30);

        /* 演示结束，短暂停留 */
        matrix_clear();
        matrix_update();
        delay_cycles(500 * 32000);
    }
}
