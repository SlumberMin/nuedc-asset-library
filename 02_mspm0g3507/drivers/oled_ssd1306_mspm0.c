/**
 * @file    oled_ssd1306_mspm0.c
 * @brief   SSD1306 OLED显示驱动实现 — MSPM0G3507
 * @note    使用硬件I2C通信，支持中英文显示
 *          基于天猛星MSPM0G3507模块移植代码优化
 */

#include "oled_ssd1306_mspm0.h"
#include <string.h>
#include <math.h>

/* ── 私有变量 ────────────────────────────────────────────── */
static I2C_Regs *g_oled_i2c;
static uint8_t g_oled_gram[OLED_WIDTH][OLED_HEIGHT / 8]; /* 显存 */

/* ── 字库数据 (ASCII 6x8/8x16/12x24) ─────────────────────── */
/* 这里需要包含字库数据文件，为节省空间仅声明 */
/* 实际使用时需要包含 oledfont.h */
extern const unsigned char asc2_0806[][6];
extern const unsigned char asc2_1608[][16];
extern const unsigned char asc2_2412[][36];

/* ── I2C超时计数 ─────────────────────────────────────────── */
#ifndef I2C_TIMEOUT_COUNT
#define I2C_TIMEOUT_COUNT   100000
#endif

/* ── 内部函数 ────────────────────────────────────────────── */

/**
 * @brief 发送一个字节到OLED
 * @param dat  数据
 * @param mode 0=命令, 1=数据
 */
static void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t buf[2];
    buf[0] = mode ? 0x40 : 0x00;
    buf[1] = dat;
    
    /* 填充TX FIFO */
    DL_I2C_fillControllerTXFIFO(g_oled_i2c, buf, 2);
    
    /* 等待I2C空闲 (带超时) */
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (!(DL_I2C_getControllerStatus(g_oled_i2c) & DL_I2C_CONTROLLER_STATUS_IDLE)
           && --timeout) {}
    if (timeout == 0) {
        DL_I2C_flushControllerTXFIFO(g_oled_i2c);
        return;
    }
    
    /* 启动传输 */
    DL_I2C_startControllerTransfer(g_oled_i2c, OLED_I2C_ADDR, 
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
}

/**
 * @brief m的n次方
 */
static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) {
        result *= m;
    }
    return result;
}

/* ── 公开API ─────────────────────────────────────────────── */

void OLED_Init(I2C_Regs *i2c)
{
    g_oled_i2c = i2c;
    
    /* 延时等待OLED上电稳定 */
    DELAY_MS(100);
    
    /* 初始化序列 */
    OLED_WR_Byte(0xAE, OLED_CMD); /* 关闭显示 */
    OLED_WR_Byte(0x00, OLED_CMD); /* 设置低列地址 */
    OLED_WR_Byte(0x10, OLED_CMD); /* 设置高列地址 */
    OLED_WR_Byte(0x40, OLED_CMD); /* 设置起始行地址 */
    OLED_WR_Byte(0x81, OLED_CMD); /* 设置对比度 */
    OLED_WR_Byte(0xCF, OLED_CMD); /* 对比度值 */
    OLED_WR_Byte(0xA1, OLED_CMD); /* 设置段重映射 */
    OLED_WR_Byte(0xC8, OLED_CMD); /* 设置COM扫描方向 */
    OLED_WR_Byte(0xA6, OLED_CMD); /* 正常显示 */
    OLED_WR_Byte(0xA8, OLED_CMD); /* 设置多路复用率 */
    OLED_WR_Byte(0x3F, OLED_CMD); /* 1/64 duty */
    OLED_WR_Byte(0xD3, OLED_CMD); /* 设置显示偏移 */
    OLED_WR_Byte(0x00, OLED_CMD); /* 无偏移 */
    OLED_WR_Byte(0xD5, OLED_CMD); /* 设置时钟分频 */
    OLED_WR_Byte(0x80, OLED_CMD); /* 分频值 */
    OLED_WR_Byte(0xD9, OLED_CMD); /* 设置预充电周期 */
    OLED_WR_Byte(0xF1, OLED_CMD); /* 预充电值 */
    OLED_WR_Byte(0xDA, OLED_CMD); /* 设置COM引脚配置 */
    OLED_WR_Byte(0x12, OLED_CMD); /* 配置值 */
    OLED_WR_Byte(0xDB, OLED_CMD); /* 设置VCOMH */
    OLED_WR_Byte(0x30, OLED_CMD); /* VCOMH值 */
    OLED_WR_Byte(0x20, OLED_CMD); /* 设置页地址模式 */
    OLED_WR_Byte(0x02, OLED_CMD); /* 页地址模式 */
    OLED_WR_Byte(0x8D, OLED_CMD); /* 设置电荷泵 */
    OLED_WR_Byte(0x14, OLED_CMD); /* 启用电荷泵 */
    
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD); /* 开启显示 */
}

void OLED_Clear(void)
{
    memset(g_oled_gram, 0, sizeof(g_oled_gram));
    OLED_Refresh();
}

void OLED_Refresh(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        OLED_WR_Byte(0xB0 + i, OLED_CMD); /* 设置页地址 */
        OLED_WR_Byte(0x00, OLED_CMD);     /* 设置低列地址 */
        OLED_WR_Byte(0x10, OLED_CMD);     /* 设置高列地址 */
        
        for (uint8_t n = 0; n < OLED_WIDTH; n++) {
            OLED_WR_Byte(g_oled_gram[n][i], OLED_DATA);
        }
    }
}

void OLED_DisplayOn(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD); /* 电荷泵使能 */
    OLED_WR_Byte(0x14, OLED_CMD); /* 开启电荷泵 */
    OLED_WR_Byte(0xAF, OLED_CMD); /* 点亮屏幕 */
}

void OLED_DisplayOff(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD); /* 电荷泵使能 */
    OLED_WR_Byte(0x10, OLED_CMD); /* 关闭电荷泵 */
    OLED_WR_Byte(0xAE, OLED_CMD); /* 关闭屏幕 */
}

void OLED_ColorTurn(uint8_t mode)
{
    if (mode) {
        OLED_WR_Byte(0xA7, OLED_CMD); /* 反色显示 */
    } else {
        OLED_WR_Byte(0xA6, OLED_CMD); /* 正常显示 */
    }
}

void OLED_DisplayTurn(uint8_t mode)
{
    if (mode) {
        OLED_WR_Byte(0xC0, OLED_CMD); /* 反转显示 */
        OLED_WR_Byte(0xA0, OLED_CMD);
    } else {
        OLED_WR_Byte(0xC8, OLED_CMD); /* 正常显示 */
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
}

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t dot)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    
    uint8_t page = y / 8;
    uint8_t bit_pos = y % 8;
    uint8_t mask = 1 << bit_pos;
    
    if (dot) {
        g_oled_gram[x][page] |= mask;
    } else {
        g_oled_gram[x][page] &= ~mask;
    }
}

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
    int16_t dx = abs(x2 - x1);
    int16_t dy = abs(y2 - y1);
    int16_t sx = (x1 < x2) ? 1 : -1;
    int16_t sy = (y1 < y2) ? 1 : -1;
    int16_t err = dx - dy;
    int16_t e2;
    
    while (1) {
        OLED_DrawPoint(x1, y1, mode);
        
        if (x1 == x2 && y1 == y2) break;
        
        e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int16_t a = 0, b = r;
    int16_t d = 3 - 2 * r;
    
    while (a <= b) {
        OLED_DrawPoint(x + a, y + b, 1);
        OLED_DrawPoint(x - a, y + b, 1);
        OLED_DrawPoint(x + a, y - b, 1);
        OLED_DrawPoint(x - a, y - b, 1);
        OLED_DrawPoint(x + b, y + a, 1);
        OLED_DrawPoint(x - b, y + a, 1);
        OLED_DrawPoint(x + b, y - a, 1);
        OLED_DrawPoint(x - b, y - a, 1);
        
        if (d < 0) {
            d += 4 * a + 6;
        } else {
            d += 4 * (a - b) + 10;
            b--;
        }
        a++;
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
    uint8_t temp, size2;
    uint8_t x0 = x, y0 = y;
    
    if (size1 == 8) size2 = 6;
    else size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    
    uint8_t chr1 = chr - ' ';
    
    for (uint8_t i = 0; i < size2; i++) {
        if (size1 == 8) temp = asc2_0806[chr1][i];
        else if (size1 == 16) temp = asc2_1608[chr1][i];
        else if (size1 == 24) temp = asc2_2412[chr1][i];
        else return;
        
        for (uint8_t m = 0; m < 8; m++) {
            if (temp & 0x01) OLED_DrawPoint(x, y, mode);
            else OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        
        if ((size1 != 8) && ((x - x0) == size1 / 2)) {
            x = x0;
            y0 += 8;
        }
        y = y0;
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size1, uint8_t mode)
{
    while (*str >= ' ' && *str <= '~') {
        OLED_ShowChar(x, y, *str, size1, mode);
        if (size1 == 8) x += 6;
        else x += size1 / 2;
        str++;
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
    uint8_t temp, m = 0;
    if (size1 == 8) m = 2;
    
    for (uint8_t t = 0; t < len; t++) {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
    }
}

void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t dec, uint8_t size1, uint8_t mode)
{
    uint8_t m = 0;
    if (size1 == 8) m = 2;
    
    /* 处理负数 */
    if (num < 0) {
        OLED_ShowChar(x, y, '-', size1, mode);
        num = -num;
        x += size1 / 2 + m;
    }
    
    /* 显示整数部分 */
    uint32_t int_part = (uint32_t)num;
    OLED_ShowNum(x, y, int_part, len, size1, mode);
    
    /* 显示小数点 */
    x += (size1 / 2 + m) * len;
    OLED_ShowChar(x, y, '.', size1, mode);
    x += size1 / 2 + m;
    
    /* 显示小数部分 */
    float frac = num - int_part;
    for (uint8_t i = 0; i < dec; i++) {
        frac *= 10;
        uint8_t digit = (uint8_t)frac;
        OLED_ShowChar(x + (size1 / 2 + m) * i, y, digit + '0', size1, mode);
        frac -= digit;
    }
}

void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, const uint8_t bmp[], uint8_t mode)
{
    uint16_t j = 0;
    uint8_t x0 = x, y0 = y;
    uint8_t sizey_page = sizey / 8 + ((sizey % 8) ? 1 : 0);
    
    for (uint8_t n = 0; n < sizey_page; n++) {
        for (uint8_t i = 0; i < sizex; i++) {
            uint8_t temp = bmp[j++];
            for (uint8_t m = 0; m < 8; m++) {
                if (temp & 0x01) OLED_DrawPoint(x, y, mode);
                else OLED_DrawPoint(x, y, !mode);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == sizex) {
                x = x0;
                y0 += 8;
            }
            y = y0;
        }
    }
}