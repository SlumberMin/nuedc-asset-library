/**
 * @file    oled_ssd1306.c
 * @brief   SSD1306 0.96寸OLED I2C驱动实现 — MSPM0G3507
 *
 * 工作原理:
 *   SSD1306内部有128×64位的GDDRAM，按页组织(8页×128列)
 *   每页8行像素，每字节控制一列8个垂直像素(LSB在上)
 *   I2C通信：控制字节(0x00=命令, 0x40=数据) + 数据
 *   使用内部1024字节帧缓冲区，批量刷新
 *
 * SysConfig生成的宏:
 *   I2C_0_INST
 */

#include "drivers/oled_ssd1306.h"

/* ── I2C实例 (由SysConfig生成) ───────────────────────────── */
#define OLED_I2C    I2C_0_INST

/* ── 帧缓冲区 ────────────────────────────────────────────── */
static uint8_t g_frameBuf[SSD1306_PAGES][SSD1306_WIDTH];

/* ── 6×8 ASCII字库 (字符32~127, 每字符6字节) ─────────────── */
/*  格式: 6列×8行, 每字节表示一列8个像素(LSB=顶部) */
static const uint8_t FONT_6X8[96][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* 32  (space) */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* 33  ! */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* 34  " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* 35  # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* 36  $ */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* 37  % */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* 38  & */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* 39  ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* 40  ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* 41  ) */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* 42  * */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* 43  + */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* 44  , */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* 45  - */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* 46  . */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* 47  / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 48  0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 49  1 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 50  2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 51  3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 52  4 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 53  5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 54  6 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 55  7 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 56  8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 57  9 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* 58  : */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* 59  ; */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* 60  < */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* 61  = */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /* 62  > */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* 63  ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* 64  @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* 65  A */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* 66  B */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* 67  C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* 68  D */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* 69  E */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /* 70  F */
    {0x3E,0x41,0x41,0x51,0x32,0x00}, /* 71  G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* 72  H */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* 73  I */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* 74  J */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* 75  K */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* 76  L */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /* 77  M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* 78  N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* 79  O */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* 80  P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* 81  Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* 82  R */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* 83  S */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* 84  T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* 85  U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* 86  V */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /* 87  W */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* 88  X */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /* 89  Y */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* 90  Z */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /* 91  [ */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* 92  \ */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /* 93  ] */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* 94  ^ */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* 95  _ */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* 96  ` */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* 97  a */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* 98  b */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* 99  c */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* 100 d */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* 101 e */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* 102 f */
    {0x08,0x14,0x54,0x54,0x3C,0x00}, /* 103 g */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* 104 h */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* 105 i */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* 106 j */
    {0x00,0x7F,0x10,0x28,0x44,0x00}, /* 107 k */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* 108 l */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* 109 m */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* 110 n */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* 111 o */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* 112 p */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* 113 q */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* 114 r */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* 115 s */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* 116 t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* 117 u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* 118 v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* 119 w */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* 120 x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* 121 y */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* 122 z */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* 123 { */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* 124 | */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* 125 } */
    {0x08,0x08,0x2A,0x1C,0x08,0x00}, /* 126 ~ */
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* 127 DEL */
};

/* ── I2C超时计数 ─────────────────────────────────────────── */
#ifndef I2C_TIMEOUT_COUNT
#define I2C_TIMEOUT_COUNT   100000
#endif

/* ── 内部: I2C发送命令字节 ───────────────────────────────── */
static bool OLED_WriteCmd(uint8_t cmd)
{
    uint8_t txBuf[2];
    txBuf[0] = SSD1306_CMD;
    txBuf[1] = cmd;

    DL_I2C_flushControllerTXFIFO(OLED_I2C);
    DL_I2C_fillControllerTXFIFO(OLED_I2C, txBuf, 2);

    /* 等待I2C空闲 (带超时) */
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (!(DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)
           && --timeout)
        ;
    if (timeout == 0) {
        DL_I2C_flushControllerTXFIFO(OLED_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(OLED_I2C, SSD1306_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    /* 等待传输完成 (带超时) */
    timeout = I2C_TIMEOUT_COUNT;
    while ((DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
           && --timeout)
        ;
    if (timeout == 0) {
        DL_I2C_flushControllerTXFIFO(OLED_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(OLED_I2C);
    return !(DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR);
}

/* ── 内部: I2C发送数据块(带控制字节) ─────────────────────── */
static bool OLED_WriteData(const uint8_t *data, uint16_t len)
{
    /*
     * SSD1306 I2C协议: 控制字节 + 数据
     * 由于I2C FIFO有限(通常16字节)，分批发送
     * 每批: 1字节控制(0x40) + 最多15字节数据
     */
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
               && --timeout)
            ;
        if (timeout == 0) {
            DL_I2C_flushControllerTXFIFO(OLED_I2C);
            return false;
        }

        DL_I2C_startControllerTransfer(OLED_I2C, SSD1306_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_TX, chunk + 1);

        timeout = I2C_TIMEOUT_COUNT;
        while ((DL_I2C_getControllerStatus(OLED_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
               && --timeout)
            ;
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

/* ── 初始化 ──────────────────────────────────────────────── */
bool OLED_Init(void)
{
    /* 短暂延时等待OLED上电稳定 */
    delay_cycles(320000);  /* ~10ms @32MHz */

    /* SSD1306 初始化序列 (参考数据手册) */
    OLED_WriteCmd(SSD1306_DISPLAYOFF);             /* 0xAE: 关闭显示 */
    OLED_WriteCmd(SSD1306_SETDISPLAYCLOCKDIV);     /* 0xD5 */
    OLED_WriteCmd(0x80);                           /* 分频比=1, 频率=正常 */
    OLED_WriteCmd(SSD1306_SETMULTIPLEX);           /* 0xA8 */
    OLED_WriteCmd(0x3F);                           /* 64行 (0x3F = 63) */
    OLED_WriteCmd(SSD1306_SETDISPLAYOFFSET);       /* 0xD3 */
    OLED_WriteCmd(0x00);                           /* 无偏移 */
    OLED_WriteCmd(SSD1306_SETSTARTLINE | 0x00);    /* 0x40: 起始行=0 */
    OLED_WriteCmd(SSD1306_CHARGEPUMP);             /* 0x8D */
    OLED_WriteCmd(0x14);                           /* 内部VCC供电 */
    OLED_WriteCmd(SSD1306_MEMORYMODE);             /* 0x20 */
    OLED_WriteCmd(0x00);                           /* 水平寻址模式 */
    OLED_WriteCmd(SSD1306_SEGREMAP | 0x01);        /* 0xA1: 列地址127映射到SEG0 */
    OLED_WriteCmd(SSD1306_COMSCANDEC);             /* 0xC8: COM扫描方向反向 */
    OLED_WriteCmd(SSD1306_SETCOMPINS);             /* 0xDA */
    OLED_WriteCmd(0x12);                           /* 顺序COM, 禁用左右反置 */
    OLED_WriteCmd(SSD1306_SETCONTRAST);            /* 0x81 */
    OLED_WriteCmd(0xCF);                           /* 对比度207 */
    OLED_WriteCmd(SSD1306_SETPRECHARGE);           /* 0xD9 */
    OLED_WriteCmd(0xF1);                           /* 预充电周期 */
    OLED_WriteCmd(SSD1306_SETVCOMDETECT);          /* 0xDB */
    OLED_WriteCmd(0x40);                           /* VCOMH deselect level */
    OLED_WriteCmd(SSD1306_DISPLAYALLON_RESUME);    /* 0xA4: 跟随RAM内容 */
    OLED_WriteCmd(SSD1306_NORMALDISPLAY);          /* 0xA6: 正常显示(非反色) */
    OLED_WriteCmd(SSD1306_DISPLAYON);              /* 0xAF: 开启显示 */

    /* 清屏 */
    OLED_Clear();
    OLED_Refresh();

    return true;
}

/* ── 清屏 ────────────────────────────────────────────────── */
void OLED_Clear(void)
{
    memset(g_frameBuf, 0, sizeof(g_frameBuf));
}

/* ── 刷新显存到OLED ──────────────────────────────────────── */
void OLED_Refresh(void)
{
    /* 设置列地址和页地址范围 */
    OLED_WriteCmd(SSD1306_COLUMNADDR); /* 0x21 */
    OLED_WriteCmd(0);                   /* 起始列 */
    OLED_WriteCmd(SSD1306_WIDTH - 1);  /* 结束列127 */
    OLED_WriteCmd(SSD1306_PAGEADDR);   /* 0x22 */
    OLED_WriteCmd(0);                   /* 起始页 */
    OLED_WriteCmd(SSD1306_PAGES - 1);  /* 结束页7 */

    /* 批量发送整个帧缓冲区 (1024字节) */
    OLED_WriteData(&g_frameBuf[0][0], SSD1306_WIDTH * SSD1306_PAGES);
}

/* ── 设置光标位置 ────────────────────────────────────────── */
void OLED_SetCursor(uint8_t x, uint8_t y)
{
    OLED_WriteCmd(SSD1306_COLUMNADDR);
    OLED_WriteCmd(x);
    OLED_WriteCmd(SSD1306_WIDTH - 1);
    OLED_WriteCmd(SSD1306_PAGEADDR);
    OLED_WriteCmd(y);
    OLED_WriteCmd(SSD1306_PAGES - 1);
}

/* ── 画点 ────────────────────────────────────────────────── */
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
}

/* ── 显示一个字符 (6×8) ──────────────────────────────────── */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size)
{
    (void)size;  /* 目前仅支持6×8 */

    if (x >= SSD1306_WIDTH || y >= SSD1306_PAGES) return;

    /* 字符索引: ASCII 32~127 */
    if (ch < 32 || ch > 127) ch = ' ';

    uint8_t idx = (uint8_t)(ch - 32);
    const uint8_t *glyph = FONT_6X8[idx];

    for (uint8_t i = 0; i < 6; i++) {
        if ((x + i) < SSD1306_WIDTH) {
            g_frameBuf[y][x + i] = glyph[i];
        }
    }
}

/* ── 显示字符串 ──────────────────────────────────────────── */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size)
{
    if (y >= SSD1306_PAGES) return;

    while (*str) {
        if (x + 6 > SSD1306_WIDTH) {
            /* 换行 */
            x = 0;
            y++;
            if (y >= SSD1306_PAGES) return;
        }
        OLED_ShowChar(x, y, *str, size);
        x += 6;
        str++;
    }
}

/* ── 显示数字 ────────────────────────────────────────────── */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    char buf[12];
    uint8_t i;

    /* 将数字转换为字符串
     * BugFix: len可能>=12导致buf[len]越界写入'\0' */
    if (len > sizeof(buf) - 1) len = (uint8_t)(sizeof(buf) - 1);

    for (i = 0; i < len; i++) {
        buf[i] = '0';
    }
    buf[len] = '\0';

    /* 从低位开始填充 */
    i = len;
    while (i > 0) {
        i--;
        buf[i] = '0' + (char)(num % 10);
        num /= 10;
        if (num == 0) break;
    }

    OLED_ShowString(x, y, buf, size);
}

/* ── 显示有符号数字 ──────────────────────────────────────── */
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

/* ── 填充区域 ────────────────────────────────────────────── */
void OLED_FillArea(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t data)
{
    /* BugFix: y+height和x+width在uint8_t上可能溢出回绕
     * 例如 y=200, height=100 → y+height=44(溢出), 循环条件永假
     * 改用uint16_t中间变量避免溢出 */
    uint16_t y_end = (uint16_t)y + height;
    uint16_t x_end = (uint16_t)x + width;
    if (y_end > SSD1306_PAGES) y_end = SSD1306_PAGES;
    if (x_end > SSD1306_WIDTH) x_end = SSD1306_WIDTH;

    for (uint8_t page = y; page < y_end; page++) {
        for (uint8_t col = x; col < x_end; col++) {
            g_frameBuf[page][col] = data;
        }
    }
}

/* ── 反色显示 ────────────────────────────────────────────── */
void OLED_InvertDisplay(bool invert)
{
    OLED_WriteCmd(invert ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
}

/* ── 开启/关闭显示 ───────────────────────────────────────── */
void OLED_DisplayOn(bool on)
{
    OLED_WriteCmd(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
}
