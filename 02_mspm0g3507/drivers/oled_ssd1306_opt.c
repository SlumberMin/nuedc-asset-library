/**
 * @file oled_ssd1306_opt.c
 * @brief SSD1306 OLED 驱动 -- 性能优化版 (MSPM0G3507)
 *
 * 优化策略:
 * 1. 脏页标记: 只刷新被修改的页面，避免全屏刷新
 * 2. 批量字符写入: ShowString 时直接写入帧缓冲，逐字节拷贝而非逐像素
 * 3. 减少边界检查: 在外层检查一次，内层循环去除冗余判断
 * 4. 使用 memset 批量操作: Clear/FillArea 使用 memset
 * 5. 数字显示优化: 避免 snprintf 等重量级函数，使用手动整数转换
 *
 * 预期性能提升:
 * - OLED_Refresh (局部更新): ~4-8x 加速 (仅刷新脏页)
 * - OLED_ShowString: ~1.3x 加速 (减少边界检查)
 */

#include "drivers/oled_ssd1306.h"

#define OLED_I2C    I2C_0_INST

/* 帧缓冲区 */
static uint8_t g_frameBuf[SSD1306_PAGES][SSD1306_WIDTH];

/* 脏页标记 (每位对应一个page) */
static uint8_t g_dirtyPages = 0;

/* 6x8 ASCII字库 */
static const uint8_t FONT_6X8[96][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x5F,0x00,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00,0x00},
    {0x14,0x7F,0x14,0x7F,0x14,0x00},
    {0x24,0x2A,0x7F,0x2A,0x12,0x00},
    {0x23,0x13,0x08,0x64,0x62,0x00},
    {0x36,0x49,0x55,0x22,0x50,0x00},
    {0x00,0x05,0x03,0x00,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00,0x00},
    {0x00,0x41,0x22,0x1C,0x00,0x00},
    {0x08,0x2A,0x1C,0x2A,0x08,0x00},
    {0x08,0x08,0x3E,0x08,0x08,0x00},
    {0x00,0x50,0x30,0x00,0x00,0x00},
    {0x08,0x08,0x08,0x08,0x08,0x00},
    {0x00,0x60,0x60,0x00,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02,0x00},
    {0x3E,0x51,0x49,0x45,0x3E,0x00},
    {0x00,0x42,0x7F,0x40,0x00,0x00},
    {0x42,0x61,0x51,0x49,0x46,0x00},
    {0x21,0x41,0x45,0x4B,0x31,0x00},
    {0x18,0x14,0x12,0x7F,0x10,0x00},
    {0x27,0x45,0x45,0x45,0x39,0x00},
    {0x3C,0x4A,0x49,0x49,0x30,0x00},
    {0x01,0x71,0x09,0x05,0x03,0x00},
    {0x36,0x49,0x49,0x49,0x36,0x00},
    {0x06,0x49,0x49,0x29,0x1E,0x00},
    {0x00,0x36,0x36,0x00,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41,0x00},
    {0x14,0x14,0x14,0x14,0x14,0x00},
    {0x41,0x22,0x14,0x08,0x00,0x00},
    {0x02,0x01,0x51,0x09,0x06,0x00},
    {0x32,0x49,0x79,0x41,0x3E,0x00},
    {0x7E,0x11,0x11,0x11,0x7E,0x00},
    {0x7F,0x49,0x49,0x49,0x36,0x00},
    {0x3E,0x41,0x41,0x41,0x22,0x00},
    {0x7F,0x41,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41,0x00},
    {0x7F,0x09,0x09,0x01,0x01,0x00},
    {0x3E,0x41,0x41,0x51,0x32,0x00},
    {0x7F,0x08,0x08,0x08,0x7F,0x00},
    {0x00,0x41,0x7F,0x41,0x00,0x00},
    {0x20,0x40,0x41,0x3F,0x01,0x00},
    {0x7F,0x08,0x14,0x22,0x41,0x00},
    {0x7F,0x40,0x40,0x40,0x40,0x00},
    {0x7F,0x02,0x04,0x02,0x7F,0x00},
    {0x7F,0x04,0x08,0x10,0x7F,0x00},
    {0x3E,0x41,0x41,0x41,0x3E,0x00},
    {0x7F,0x09,0x09,0x09,0x06,0x00},
    {0x3E,0x41,0x51,0x21,0x5E,0x00},
    {0x7F,0x09,0x19,0x29,0x46,0x00},
    {0x46,0x49,0x49,0x49,0x31,0x00},
    {0x01,0x01,0x7F,0x01,0x01,0x00},
    {0x3F,0x40,0x40,0x40,0x3F,0x00},
    {0x1F,0x20,0x40,0x20,0x1F,0x00},
    {0x7F,0x20,0x18,0x20,0x7F,0x00},
    {0x63,0x14,0x08,0x14,0x63,0x00},
    {0x03,0x04,0x78,0x04,0x03,0x00},
    {0x61,0x51,0x49,0x45,0x43,0x00},
    {0x00,0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20,0x00},
    {0x41,0x41,0x7F,0x00,0x00,0x00},
    {0x04,0x02,0x01,0x02,0x04,0x00},
    {0x40,0x40,0x40,0x40,0x40,0x00},
    {0x00,0x01,0x02,0x04,0x00,0x00},
    {0x20,0x54,0x54,0x54,0x78,0x00},
    {0x7F,0x48,0x44,0x44,0x38,0x00},
    {0x38,0x44,0x44,0x44,0x20,0x00},
    {0x38,0x44,0x44,0x48,0x7F,0x00},
    {0x38,0x54,0x54,0x54,0x18,0x00},
    {0x08,0x7E,0x09,0x01,0x02,0x00},
    {0x08,0x14,0x54,0x54,0x3C,0x00},
    {0x7F,0x08,0x04,0x04,0x78,0x00},
    {0x00,0x44,0x7D,0x40,0x00,0x00},
    {0x20,0x40,0x44,0x3D,0x00,0x00},
    {0x00,0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00,0x00},
    {0x7C,0x04,0x18,0x04,0x78,0x00},
    {0x7C,0x08,0x04,0x04,0x78,0x00},
    {0x38,0x44,0x44,0x44,0x38,0x00},
    {0x7C,0x14,0x14,0x14,0x08,0x00},
    {0x08,0x14,0x14,0x18,0x7C,0x00},
    {0x7C,0x08,0x04,0x04,0x08,0x00},
    {0x48,0x54,0x54,0x54,0x20,0x00},
    {0x04,0x3F,0x44,0x40,0x20,0x00},
    {0x3C,0x40,0x40,0x20,0x7C,0x00},
    {0x1C,0x20,0x40,0x20,0x1C,0x00},
    {0x3C,0x40,0x30,0x40,0x3C,0x00},
    {0x44,0x28,0x10,0x28,0x44,0x00},
    {0x0C,0x50,0x50,0x50,0x3C,0x00},
    {0x44,0x64,0x54,0x4C,0x44,0x00},
    {0x00,0x08,0x36,0x41,0x00,0x00},
    {0x00,0x00,0x7F,0x00,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00,0x00},
    {0x08,0x08,0x2A,0x1C,0x08,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00},
};

#ifndef I2C_TIMEOUT_COUNT
#define I2C_TIMEOUT_COUNT   100000
#endif

static bool OLED_WriteCmd(uint8_t cmd)
{
    uint8_t txBuf[2];
    txBuf[0] = SSD1306_CMD;
    txBuf[1] = cmd;

    DL_I2C_flushControllerTXFIFO(OLED_I2C);
    DL_I2C_fillControllerTXFIFO(OLED_I2C, txBuf, 2);

    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (!(DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)
           && --timeout);
    if (timeout == 0) {
        DL_I2C_flushControllerTXFIFO(OLED_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(OLED_I2C, SSD1306_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    timeout = I2C_TIMEOUT_COUNT;
    while ((DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
           && --timeout);
    if (timeout == 0) {
        DL_I2C_flushControllerTXFIFO(OLED_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(OLED_I2C);
    return !(DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR);
}

static bool OLED_WriteData(const uint8_t *data, uint16_t len)
{
    uint16_t sent = 0;

    while (sent < len) {
        uint16_t chunk = len - sent;
        if (chunk > 15) chunk = 15;

        uint8_t buf[16];
        buf[0] = SSD1306_DATA;
        memcpy(&buf[1], &data[sent], chunk);

        DL_I2C_flushControllerTXFIFO(OLED_I2C);
        DL_I2C_fillControllerTXFIFO(OLED_I2C, buf, (uint8_t)(chunk + 1));

        uint32_t timeout = I2C_TIMEOUT_COUNT;
        while (!(DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)
               && --timeout);
        if (timeout == 0) {
            DL_I2C_flushControllerTXFIFO(OLED_I2C);
            return false;
        }

        DL_I2C_startControllerTransfer(OLED_I2C, SSD1306_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_TX, chunk + 1);

        timeout = I2C_TIMEOUT_COUNT;
        while ((DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
               && --timeout);
        if (timeout == 0) {
            DL_I2C_flushControllerTXFIFO(OLED_I2C);
            return false;
        }

        if (DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
            DL_I2C_flushControllerTXFIFO(OLED_I2C);
            return false;
        }

        DL_I2C_flushControllerTXFIFO(OLED_I2C);
        sent += chunk;
    }

    return true;
}

bool OLED_Init(void)
{
    delay_cycles(320000);

    OLED_WriteCmd(SSD1306_DISPLAYOFF);
    OLED_WriteCmd(SSD1306_SETDISPLAYCLOCKDIV);
    OLED_WriteCmd(0x80);
    OLED_WriteCmd(SSD1306_SETMULTIPLEX);
    OLED_WriteCmd(0x3F);
    OLED_WriteCmd(SSD1306_SETDISPLAYOFFSET);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(SSD1306_SETSTARTLINE | 0x00);
    OLED_WriteCmd(SSD1306_CHARGEPUMP);
    OLED_WriteCmd(0x14);
    OLED_WriteCmd(SSD1306_MEMORYMODE);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(SSD1306_SEGREMAP | 0x01);
    OLED_WriteCmd(SSD1306_COMSCANDEC);
    OLED_WriteCmd(SSD1306_SETCOMPINS);
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(SSD1306_SETCONTRAST);
    OLED_WriteCmd(0xCF);
    OLED_WriteCmd(SSD1306_SETPRECHARGE);
    OLED_WriteCmd(0xF1);
    OLED_WriteCmd(SSD1306_SETVCOMDETECT);
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(SSD1306_DISPLAYALLON_RESUME);
    OLED_WriteCmd(SSD1306_NORMALDISPLAY);
    OLED_WriteCmd(SSD1306_DISPLAYON);

    OLED_Clear();
    OLED_Refresh();

    return true;
}

void OLED_Clear(void)
{
    memset(g_frameBuf, 0, sizeof(g_frameBuf));
    g_dirtyPages = 0xFF;  /* 全部标记为脏 */
}

/**
 * @brief 标记脏页
 */
static inline void mark_dirty(uint8_t page)
{
    g_dirtyPages |= (1 << page);
}

/**
 * @brief 全屏刷新 (兼容原有API)
 */
void OLED_Refresh(void)
{
    OLED_WriteCmd(SSD1306_COLUMNADDR);
    OLED_WriteCmd(0);
    OLED_WriteCmd(SSD1306_WIDTH - 1);
    OLED_WriteCmd(SSD1306_PAGEADDR);
    OLED_WriteCmd(0);
    OLED_WriteCmd(SSD1306_PAGES - 1);

    OLED_WriteData(&g_frameBuf[0][0], SSD1306_WIDTH * SSD1306_PAGES);
    g_dirtyPages = 0;
}

/**
 * @brief 增量刷新 -- 只刷新被修改的页面
 *
 * 这是主要的性能优化点:
 * - 如果只修改了1-2个页面，数据传输量从1024字节降到128-256字节
 * - I2C传输时间与数据量成正比，因此可获得 4-8x 加速
 */
void OLED_RefreshDirty(void)
{
    if (g_dirtyPages == 0) return;  /* 无脏页，跳过 */

    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        if (!(g_dirtyPages & (1 << page))) continue;  /* 跳过干净页 */

        OLED_WriteCmd(SSD1306_COLUMNADDR);
        OLED_WriteCmd(0);
        OLED_WriteCmd(SSD1306_WIDTH - 1);
        OLED_WriteCmd(SSD1306_PAGEADDR);
        OLED_WriteCmd(page);
        OLED_WriteCmd(page);

        OLED_WriteData(g_frameBuf[page], SSD1306_WIDTH);
    }

    g_dirtyPages = 0;
}

void OLED_SetCursor(uint8_t x, uint8_t y)
{
    OLED_WriteCmd(SSD1306_COLUMNADDR);
    OLED_WriteCmd(x);
    OLED_WriteCmd(SSD1306_WIDTH - 1);
    OLED_WriteCmd(SSD1306_PAGEADDR);
    OLED_WriteCmd(y);
    OLED_WriteCmd(SSD1306_PAGES - 1);
}

void OLED_DrawPoint(uint8_t x, uint8_t y, bool on)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    uint8_t page = y / 8;
    uint8_t bit  = y % 8;

    if (on) {
        g_frameBuf[page][x] |= (1 << bit);
    } else {
        g_frameBuf[page][x] &= ~(1 << bit);
    }

    mark_dirty(page);
}

/**
 * @brief 显示字符 -- 优化版
 * 直接写入帧缓冲，标记脏页
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size)
{
    (void)size;

    if (x >= SSD1306_WIDTH || y >= SSD1306_PAGES) return;

    if (ch < 32 || ch > 127) ch = ' ';

    uint8_t idx = (uint8_t)(ch - 32);
    const uint8_t *glyph = FONT_6X8[idx];

    /* 边界检查在外层，内层循环不再检查 */
    uint8_t max_col = (x + 6 <= SSD1306_WIDTH) ? 6 : (SSD1306_WIDTH - x);

    for (uint8_t i = 0; i < max_col; i++) {
        g_frameBuf[y][x + i] = glyph[i];
    }

    mark_dirty(y);
}

/**
 * @brief 显示字符串 -- 优化版
 * 减少函数调用开销，批量处理
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size)
{
    if (y >= SSD1306_PAGES) return;

    while (*str) {
        if (x + 6 > SSD1306_WIDTH) {
            x = 0;
            y++;
            if (y >= SSD1306_PAGES) return;
        }

        /* 内联字符写入 (避免重复边界检查) */
        char ch = *str;
        if (ch < 32 || ch > 127) ch = ' ';

        const uint8_t *glyph = FONT_6X8[(uint8_t)(ch - 32)];
        uint8_t max_col = (x + 6 <= SSD1306_WIDTH) ? 6 : (SSD1306_WIDTH - x);

        for (uint8_t i = 0; i < max_col; i++) {
            g_frameBuf[y][x + i] = glyph[i];
        }
        mark_dirty(y);

        x += 6;
        str++;
    }
}

/**
 * @brief 显示数字 -- 优化版
 * 使用手动整数转换，避免浮点除法和 powf
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    char buf[12];
    uint8_t i;

    for (i = 0; i < len && i < sizeof(buf) - 1; i++) {
        buf[i] = '0';
    }
    buf[len] = '\0';

    i = len;
    while (i > 0) {
        i--;
        buf[i] = '0' + (char)(num % 10);
        num /= 10;
        if (num == 0) break;
    }

    OLED_ShowString(x, y, buf, size);
}

void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size)
{
    if (num < 0) {
        OLED_ShowChar(x, y, '-', size);
        x += 6;
        OLED_ShowNum(x, y, (uint32_t)(-num), len, size);
    } else {
        OLED_ShowChar(x, y, ' ', size);
        x += 6;
        OLED_ShowNum(x, y, (uint32_t)num, len, size);
    }
}

void OLED_FillArea(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t data)
{
    for (uint8_t page = y; page < y + height && page < SSD1306_PAGES; page++) {
        if (x + width <= SSD1306_WIDTH) {
            /* 无越界: 使用 memset 批量填充 */
            memset(&g_frameBuf[page][x], data, width);
        } else {
            /* 有越界: 逐字节填充到边界 */
            memset(&g_frameBuf[page][x], data, SSD1306_WIDTH - x);
        }
        mark_dirty(page);
    }
}

void OLED_InvertDisplay(bool invert)
{
    OLED_WriteCmd(invert ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
}

void OLED_DisplayOn(bool on)
{
    OLED_WriteCmd(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
}
