/**
 * @file    oled_ssd1306_stm32.c
 * @brief   SSD1306 OLED显示驱动实现 — STM32 HAL库 I2C
 * @details 基于STM32 HAL库的I2C接口驱动SSD1306 128x64 OLED显示屏。
 *          使用页地址模式, 维护显存缓冲区g_gram, 支持:
 *          - 像素级绘点
 *          - 字符显示(8x16字体)
 *          - 字符串显示
 *          - 整数和浮点数显示
 *          - 显存批量刷新
 */

#include "drivers/oled_ssd1306_stm32.h"
#include "drivers/oledfont.h"
#include <string.h>
#include <math.h>

/* ── 内部变量 ─────────────────────────────────────────────── */
/* I2C句柄指针 */
static I2C_HandleTypeDef *g_hi2c = NULL;
/* 显存缓冲区: [列][页], 每页8行像素 */
static uint8_t g_gram[OLED_WIDTH][OLED_HEIGHT / 8];

/* ── I2C 底层 ─────────────────────────────────────────────── */

/**
 * @brief 向OLED写入一个字节(命令或数据)
 * @param dat 要写入的字节
 * @param mode 0=命令模式, 1=数据模式
 */
static void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t buf[2];
    buf[0] = mode ? 0x40 : 0x00;  /* Co=0, D/C#=mode */
    buf[1] = dat;
    /* 使用有限超时防止I2C总线锁死 */
    HAL_I2C_Master_Transmit(g_hi2c, OLED_ADDR << 1, buf, 2, 50);
}

/* ── OLED 公开API ─────────────────────────────────────────── */

/**
 * @brief 初始化SSD1306 OLED显示屏
 * @param hi2c I2C外设句柄指针
 *
 * @details 发送初始化命令序列:
 *          - 关闭显示
 *          - 设置列/行地址
 *          - 设置对比度、扫描方向
 *          - 配置时钟分频、预充电
 *          - 启用电荷泵
 *          - 清屏后开启显示
 */
void OLED_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return;
    g_hi2c = hi2c;

    HAL_Delay(100); /* 等待OLED上电稳定 */

    /* 基本显示配置命令序列 */
    OLED_WR_Byte(0xAE, 0); /* 关闭显示 */
    OLED_WR_Byte(0x00, 0); /* 低列地址 */
    OLED_WR_Byte(0x10, 0); /* 高列地址 */
    OLED_WR_Byte(0x40, 0); /* 起始行 */
    OLED_WR_Byte(0x81, 0); /* 对比度 */
    OLED_WR_Byte(0xCF, 0); /* 对比度值 */
    OLED_WR_Byte(0xA1, 0); /* 段重映射(左右翻转) */
    OLED_WR_Byte(0xC8, 0); /* COM扫描方向(上下翻转) */
    OLED_WR_Byte(0xA6, 0); /* 正常显示(非反色) */
    OLED_WR_Byte(0xA8, 0); /* 多路复用率 */
    OLED_WR_Byte(0x3F, 0); /* 1/64 duty */
    OLED_WR_Byte(0xD3, 0); /* 显示偏移 */
    OLED_WR_Byte(0x00, 0); /* 无偏移 */
    OLED_WR_Byte(0xD5, 0); /* 时钟分频 */
    OLED_WR_Byte(0x80, 0); /* 默认分频 */
    OLED_WR_Byte(0xD9, 0); /* 预充电周期 */
    OLED_WR_Byte(0xF1, 0);
    OLED_WR_Byte(0xDA, 0); /* COM引脚配置 */
    OLED_WR_Byte(0x12, 0);
    OLED_WR_Byte(0xDB, 0); /* VCOMH电压 */
    OLED_WR_Byte(0x30, 0);
    OLED_WR_Byte(0x20, 0); /* 页地址模式 */
    OLED_WR_Byte(0x02, 0);
    OLED_WR_Byte(0x8D, 0); /* 电荷泵设置 */
    OLED_WR_Byte(0x14, 0); /* 启用电荷泵 */

    OLED_Clear();
    OLED_WR_Byte(0xAF, 0); /* 开显示 */
}

/**
 * @brief 清屏(清零显存并刷新)
 */
void OLED_Clear(void)
{
    memset(g_gram, 0, sizeof(g_gram));
    OLED_Refresh();
}

/**
 * @brief 将显存缓冲区刷新到OLED屏幕
 * @details 按页(0~7)逐列发送显存数据, 每页128字节
 */
void OLED_Refresh(void)
{
    for (uint8_t page = 0; page < 8; page++) {
        /* 设置页地址和列地址 */
        OLED_WR_Byte(0xB0 + page, 0);  /* 页地址 */
        OLED_WR_Byte(0x00, 0);          /* 低列地址 */
        OLED_WR_Byte(0x10, 0);          /* 高列地址 */
        /* 发送该页的128列数据 */
        for (uint8_t col = 0; col < OLED_WIDTH; col++) {
            OLED_WR_Byte(g_gram[col][page], 1);
        }
    }
}

/**
 * @brief 在显存中绘制一个像素点
 * @param x X坐标(0~127)
 * @param y Y坐标(0~63)
 * @param dot 1=点亮, 0=熄灭
 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t dot)
{
    /* 坐标越界保护 */
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    /* 计算页地址和位掩码 */
    uint8_t page = y / 8;
    uint8_t mask = 1 << (y % 8);
    if (dot) g_gram[x][page] |= mask;
    else     g_gram[x][page] &= ~mask;
}

/**
 * @brief 计算m的n次方(用于数字位提取)
 * @param m 底数
 * @param n 指数
 * @return m^n
 */
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t r = 1;
    while (n--) r *= m;
    return r;
}

/**
 * @brief 在指定位置显示一个字符
 * @param x 起始X坐标
 * @param y 起始Y坐标
 * @param chr 要显示的ASCII字符
 * @param size 字体大小(仅支持8)
 * @param mode 1=正常显示, 0=反色显示
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t size, uint8_t mode)
{
    uint8_t c = chr - ' ';
    if (size == 8) {
        /* 8x16字体: 每个字符6列宽, 8行高 */
        for (uint8_t i = 0; i < 6; i++) {
            uint8_t d = asc2_0806[c][i];
            for (uint8_t m = 0; m < 8; m++) {
                OLED_DrawPoint(x, y + m, (d & (1 << m)) ? mode : !mode);
            }
            x++;
        }
    }
}

/**
 * @brief 在指定位置显示字符串
 * @param x 起始X坐标
 * @param y 起始Y坐标
 * @param str 要显示的字符串(ASCII可打印字符)
 * @param size 字体大小(仅支持8)
 * @param mode 1=正常显示, 0=反色显示
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size, uint8_t mode)
{
    if (str == NULL) return;
    /* 逐字符显示, 遇到非可打印字符停止 */
    while (*str >= ' ' && *str <= '~') {
        OLED_ShowChar(x, y, *str, size, mode);
        x += (size == 8) ? 6 : size / 2;
        str++;
    }
}

/**
 * @brief 在指定位置显示无符号整数
 * @param x 起始X坐标
 * @param y 起始Y坐标
 * @param num 要显示的数字
 * @param len 显示位数(不足补零)
 * @param size 字体大小(仅支持8)
 * @param mode 1=正常显示, 0=反色显示
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode)
{
    /* 逐位提取并显示 */
    for (uint8_t t = 0; t < len; t++) {
        uint8_t d = (num / oled_pow(10, len - t - 1)) % 10;
        OLED_ShowChar(x + 6 * t, y, d + '0', size, mode);
    }
}

/**
 * @brief 在指定位置显示浮点数
 * @param x 起始X坐标
 * @param y 起始Y坐标
 * @param num 要显示的浮点数
 * @param len 整数部分位数
 * @param dec 小数部分位数
 * @param size 字体大小(仅支持8)
 * @param mode 1=正常显示, 0=反色显示
 */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t dec, uint8_t size, uint8_t mode)
{
    /* 处理负数 */
    if (num < 0) { OLED_ShowChar(x, y, '-', size, mode); num = -num; x += 6; }
    /* 整数部分 */
    uint32_t ip = (uint32_t)num;
    OLED_ShowNum(x, y, ip, len, size, mode);
    x += 6 * len;
    /* 小数点 */
    OLED_ShowChar(x, y, '.', size, mode); x += 6;
    /* 小数部分逐位提取 */
    float frac = num - ip;
    for (uint8_t i = 0; i < dec; i++) {
        frac *= 10;
        uint8_t d = (uint8_t)frac;
        OLED_ShowChar(x + 6 * i, y, d + '0', size, mode);
        frac -= d;
    }
}
