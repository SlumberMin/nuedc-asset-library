/**
 * @file    oled_tm4c.c
 * @brief   SSD1306 OLED显示驱动 实现文件 (TM4C123 I2C)
 * @details 128x64 I2C OLED, 基于TivaWare I2C接口
 */

#include "oled_tm4c.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"

/* ========== SSD1306 命令定义 ========== */
#define SSD1306_CMD_DISPLAY_OFF     0xAE
#define SSD1306_CMD_DISPLAY_ON      0xAF
#define SSD1306_CMD_SET_CONTRAST    0x81
#define SSD1306_CMD_NORMAL_DISPLAY  0xA6
#define SSD1306_CMD_INVERT_DISPLAY  0xA7
#define SSD1306_CMD_SET_MULTIPLEX   0xA8
#define SSD1306_CMD_SET_DISP_OFFSET 0xD3
#define SSD1306_CMD_SET_START_LINE  0x40
#define SSD1306_CMD_SEG_REMAP       0xA1
#define SSD1306_CMD_COM_SCAN_DEC    0xC8
#define SSD1306_CMD_SET_COM_PINS    0xDA
#define SSD1306_CMD_SET_PRECHARGE   0xD9
#define SSD1306_CMD_SET_VCOMH       0xDB
#define SSD1306_CMD_CHARGE_PUMP     0x8D
#define SSD1306_CMD_SET_CLOCK       0xD5
#define SSD1306_CMD_MEMORY_MODE     0x20
#define SSD1306_CMD_SET_COL_ADDR    0x21
#define SSD1306_CMD_SET_PAGE_ADDR   0x22
#define SSD1306_CMD_COM_SCAN_INC    0xC0
#define SSD1306_CMD_SEG_REMAP_NORM  0xA0

/* ========== 显存缓冲区 (128x64 = 1024字节) ========== */
static uint8_t g_oled_buffer[OLED_PAGES][OLED_WIDTH];

/* ========== 内部变量 ========== */
static const OLED_Config_t *g_oled_cfg = 0;
static uint8_t g_i2c_addr = OLED_I2C_ADDR;

/* ========== 字库 (6x8 ASCII 32~127) ========== */
static const uint8_t g_font_6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* 空格 */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* $ */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* % */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* & */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* * */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* + */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* - */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* = */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* A */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* B */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* D */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* E */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /* F */
    {0x3E,0x41,0x41,0x51,0x32,0x00}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* H */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* J */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* K */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* L */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* O */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* R */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* S */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /* W */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* X */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /* Y */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* Z */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* \ */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* _ */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* a */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* b */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* c */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* d */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* e */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* f */
    {0x08,0x14,0x54,0x54,0x3C,0x00}, /* g */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* h */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* j */
    {0x00,0x7F,0x10,0x28,0x44,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* m */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* n */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* o */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* p */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* q */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* r */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* s */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* w */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* y */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* z */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* } */
    {0x08,0x08,0x2A,0x1C,0x08,0x00}, /* ~ */
};

/* ========================================================================== */
/*                              I2C底层通信                                     */
/* ========================================================================== */

/**
 * @brief  通过I2C发送命令到OLED
 * @param  cmd  命令字节
 */
static void _OLED_WriteCmd(uint8_t cmd)
{
    /* [修复#6] 添加I2C忙等待超时 */
    uint32_t timeout;
    I2CMasterSlaveAddrSet(g_oled_cfg->i2c_base, g_i2c_addr, false);
    I2CMasterDataPut(g_oled_cfg->i2c_base, 0x00);  /* Co=0, D/C=0 → 命令 */
    I2CMasterControl(g_oled_cfg->i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
    timeout = 100000;
    while (I2CMasterBusy(g_oled_cfg->i2c_base) && --timeout) {}

    I2CMasterDataPut(g_oled_cfg->i2c_base, cmd);
    I2CMasterControl(g_oled_cfg->i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    timeout = 100000;
    while (I2CMasterBusy(g_oled_cfg->i2c_base) && --timeout) {}
}

/**
 * @brief  通过I2C发送数据到OLED (批量)
 * @param  data  数据指针
 * @param  len   数据长度
 */
static void _OLED_WriteData(const uint8_t *data, uint32_t len)
{
    if (len == 0) return;

    /* [修复#6] 添加I2C忙等待超时 */
    uint32_t timeout;
    I2CMasterSlaveAddrSet(g_oled_cfg->i2c_base, g_i2c_addr, false);

    /* 起始 + 控制字节 (Co=0, D/C=1 → 数据) */
    I2CMasterDataPut(g_oled_cfg->i2c_base, 0x40);  /* Co=0, D/C=1 */
    I2CMasterControl(g_oled_cfg->i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
    timeout = 100000;
    while (I2CMasterBusy(g_oled_cfg->i2c_base) && --timeout) {}

    /* 发送数据 */
    uint32_t i;
    for (i = 0; i < len - 1; i++) {
        I2CMasterDataPut(g_oled_cfg->i2c_base, data[i]);
        I2CMasterControl(g_oled_cfg->i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);
        timeout = 100000;
        while (I2CMasterBusy(g_oled_cfg->i2c_base) && --timeout) {}
    }

    /* 最后一个字节 */
    I2CMasterDataPut(g_oled_cfg->i2c_base, data[len - 1]);
    I2CMasterControl(g_oled_cfg->i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    timeout = 100000;
    while (I2CMasterBusy(g_oled_cfg->i2c_base) && --timeout) {}
}

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

void OLED_Init(const OLED_Config_t *cfg)
{
    g_oled_cfg = cfg;
    g_i2c_addr = cfg->i2c_addr ? cfg->i2c_addr : OLED_I2C_ADDR;

    /* ---- 1. 使能外设时钟 ---- */
    SysCtlPeripheralEnable(cfg->i2c_periph);
    while (!SysCtlPeripheralReady(cfg->i2c_periph)) {}

    SysCtlPeripheralEnable(cfg->gpio_periph);
    while (!SysCtlPeripheralReady(cfg->gpio_periph)) {}

    /* ---- 2. 配置I2C引脚 ---- */
    GPIOPinConfigure(cfg->sda_config);
    GPIOPinConfigure(cfg->scl_config);
    GPIOPinTypeI2C(cfg->gpio_base, cfg->sda_pin);
    GPIOPinTypeI2CSCL(cfg->gpio_base, cfg->scl_pin);

    /* ---- 3. 初始化I2C主机 ---- */
    I2CMasterInitExpClk(cfg->i2c_base, SysCtlClockGet(), false);

    /* ---- 4. SSD1306初始化序列 ---- */
    _OLED_WriteCmd(SSD1306_CMD_DISPLAY_OFF);       /* 关显示 */

    _OLED_WriteCmd(SSD1306_CMD_SET_CLOCK);          /* 设置时钟分频 */
    _OLED_WriteCmd(0x80);                           /* 分频因子=1, 频率=100 */

    _OLED_WriteCmd(SSD1306_CMD_SET_MULTIPLEX);      /* 多路复用率 */
    _OLED_WriteCmd(0x3F);                           /* 1/64 duty */

    _OLED_WriteCmd(SSD1306_CMD_SET_DISP_OFFSET);    /* 显示偏移 */
    _OLED_WriteCmd(0x00);                           /* 无偏移 */

    _OLED_WriteCmd(SSD1306_CMD_SET_START_LINE | 0); /* 起始行=0 */

    _OLED_WriteCmd(SSD1306_CMD_CHARGE_PUMP);        /* 电荷泵 */
    _OLED_WriteCmd(0x14);                           /* 使能 */

    _OLED_WriteCmd(SSD1306_CMD_MEMORY_MODE);        /* 寻址模式 */
    _OLED_WriteCmd(0x00);                           /* 水平寻址模式 */

    _OLED_WriteCmd(SSD1306_CMD_SEG_REMAP);          /* 列地址127映射到SEG0 */
    _OLED_WriteCmd(SSD1306_CMD_COM_SCAN_DEC);       /* COM扫描方向: 递减 */

    _OLED_WriteCmd(SSD1306_CMD_SET_COM_PINS);       /* COM引脚配置 */
    _OLED_WriteCmd(0x12);                           /* 128x64配置 */

    _OLED_WriteCmd(SSD1306_CMD_SET_CONTRAST);       /* 对比度 */
    _OLED_WriteCmd(0xCF);                           /* 对比度=207 */

    _OLED_WriteCmd(SSD1306_CMD_SET_PRECHARGE);      /* 预充电周期 */
    _OLED_WriteCmd(0xF1);

    _OLED_WriteCmd(SSD1306_CMD_SET_VCOMH);          /* VCOMH电压 */
    _OLED_WriteCmd(0x30);                           /* 0.83*VCC */

    _OLED_WriteCmd(SSD1306_CMD_NORMAL_DISPLAY);     /* 正常显示 (非反显) */

    _OLED_WriteCmd(SSD1306_CMD_DISPLAY_ON);         /* 开显示 */

    /* ---- 5. 清屏 ---- */
    OLED_Clear();
    OLED_Update();
}

void OLED_Clear(void)
{
    /* 清零显存缓冲区 */
    uint16_t i;
    for (i = 0; i < sizeof(g_oled_buffer); i++) {
        ((uint8_t *)g_oled_buffer)[i] = 0x00;
    }
}

void OLED_Update(void)
{
    /* 设置列地址范围: 0~127 */
    _OLED_WriteCmd(SSD1306_CMD_SET_COL_ADDR);
    _OLED_WriteCmd(0);
    _OLED_WriteCmd(OLED_WIDTH - 1);

    /* 设置页地址范围: 0~7 */
    _OLED_WriteCmd(SSD1306_CMD_SET_PAGE_ADDR);
    _OLED_WriteCmd(0);
    _OLED_WriteCmd(OLED_PAGES - 1);

    /* 批量发送整个显存 */
    _OLED_WriteData((const uint8_t *)g_oled_buffer, sizeof(g_oled_buffer));
}

void OLED_UpdatePage(uint8_t page)
{
    if (page >= OLED_PAGES) return;

    /* 设置列地址范围 */
    _OLED_WriteCmd(SSD1306_CMD_SET_COL_ADDR);
    _OLED_WriteCmd(0);
    _OLED_WriteCmd(OLED_WIDTH - 1);

    /* 设置页地址 */
    _OLED_WriteCmd(SSD1306_CMD_SET_PAGE_ADDR);
    _OLED_WriteCmd(page);
    _OLED_WriteCmd(page);

    /* 发送该页数据 */
    _OLED_WriteData(g_oled_buffer[page], OLED_WIDTH);
}

void OLED_SetPixel(uint8_t x, uint8_t y, bool on)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    uint8_t page = y / 8;
    uint8_t bit  = y % 8;

    if (on) {
        g_oled_buffer[page][x] |= (1 << bit);
    } else {
        g_oled_buffer[page][x] &= ~(1 << bit);
    }
}

void OLED_DrawChar(uint8_t x, uint8_t y, char ch, OLED_Font_t font)
{
    if (ch < 32 || ch > 126) ch = ' ';  /* 非可打印字符用空格替代 */
    uint8_t idx = ch - 32;

    if (font == OLED_FONT_6x8) {
        /* 6x8字体: 每字符6列, 存在g_font_6x8数组中 */
        uint8_t col;
        for (col = 0; col < 6; col++) {
            uint8_t line = g_font_6x8[idx][col];
            uint8_t row;
            for (row = 0; row < 8; row++) {
                OLED_SetPixel(x + col, y + row, (line >> row) & 1);
            }
        }
    }
    /* OLED_FONT_8x16 需要更大的字库, 此处仅实现6x8 */
}

void OLED_DrawString(uint8_t x, uint8_t y, const char *str, OLED_Font_t font)
{
    uint8_t char_width = (font == OLED_FONT_6x8) ? 6 : 8;

    while (*str) {
        OLED_DrawChar(x, y, *str, font);
        x += char_width;
        str++;
    }
}

void OLED_DrawNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, OLED_Font_t font)
{
    /* [修复#7] 增大缓冲区防止溢出: 10位数字+符号+null */
    char buf[16];
    uint8_t i = 0;
    bool neg = false;

    if (num < 0) {
        neg = true;
        num = -num;
    }

    /* 转换为字符串 (逆序) */
    do {
        buf[i++] = '0' + (num % 10);
        num /= 10;
        len--;
    } while (num > 0 || len > 0);

    if (neg) {
        buf[i++] = '-';
    }

    /* 反转 */
    uint8_t j;
    for (j = 0; j < i / 2; j++) {
        char tmp = buf[j];
        buf[j] = buf[i - 1 - j];
        buf[i - 1 - j] = tmp;
    }
    buf[i] = '\0';

    OLED_DrawString(x, y, buf, font);
}

void OLED_DrawFloat(uint8_t x, uint8_t y, float num,
                    uint8_t int_len, uint8_t dec_len, OLED_Font_t font)
{
    char buf[20];
    uint8_t pos = 0;
    bool neg = false;

    if (num < 0.0f) {
        neg = true;
        num = -num;
    }

    int32_t int_part = (int32_t)num;
    float dec_part = num - (float)int_part;

    /* 整数部分 */
    if (neg) buf[pos++] = '-';

    /* 简单转换整数部分 */
    char int_buf[10];
    uint8_t int_pos = 0;
    if (int_part == 0) {
        int_buf[int_pos++] = '0';
    } else {
        while (int_part > 0) {
            int_buf[int_pos++] = '0' + (int_part % 10);
            int_part /= 10;
        }
    }
    /* 反转 */
    uint8_t k;
    for (k = 0; k < int_pos / 2; k++) {
        char tmp = int_buf[k];
        int_buf[k] = int_buf[int_pos - 1 - k];
        int_buf[int_pos - 1 - k] = tmp;
    }
    for (k = 0; k < int_pos; k++) buf[pos++] = int_buf[k];

    /* 小数点 */
    buf[pos++] = '.';

    /* 小数部分 */
    for (k = 0; k < dec_len; k++) {
        dec_part *= 10.0f;
        buf[pos++] = '0' + (int32_t)dec_part % 10;
    }
    buf[pos] = '\0';

    OLED_DrawString(x, y, buf, font);
}

void OLED_DrawHLine(uint8_t x, uint8_t y, uint8_t width)
{
    uint8_t i;
    for (i = 0; i < width; i++) {
        OLED_SetPixel(x + i, y, true);
    }
}

void OLED_DrawVLine(uint8_t x, uint8_t y, uint8_t height)
{
    uint8_t i;
    for (i = 0; i < height; i++) {
        OLED_SetPixel(x, y + i, true);
    }
}

void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    OLED_DrawHLine(x, y, w);
    OLED_DrawHLine(x, y + h - 1, w);
    OLED_DrawVLine(x, y, h);
    OLED_DrawVLine(x + w - 1, y, h);
}

void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    uint8_t i, j;
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            OLED_SetPixel(x + i, y + j, true);
        }
    }
}

void OLED_InvertDisplay(bool invert)
{
    _OLED_WriteCmd(invert ? SSD1306_CMD_INVERT_DISPLAY : SSD1306_CMD_NORMAL_DISPLAY);
}

void OLED_DisplayOn(bool on)
{
    _OLED_WriteCmd(on ? SSD1306_CMD_DISPLAY_ON : SSD1306_CMD_DISPLAY_OFF);
}
