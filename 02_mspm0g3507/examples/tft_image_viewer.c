/**
 * @file tft_image_viewer.c
 * @brief TFT图片查看器 - ILI9341 + SPI Flash图片库 + 幻灯片
 * @platform MSPM0G3507
 *
 * 硬件连接：
 *   ILI9341 TFT (SPI):
 *     SCK  -> PA10 (SPI0_SCK)
 *     MOSI -> PA8  (SPI0_MOSI)
 *     MISO -> PA9  (SPI0_MISO, 可选)
 *     CS   -> PA12 (GPIO)
 *     DC   -> PA13 (GPIO)
 *     RST  -> PA14 (GPIO)
 *     BL   -> PA15 (GPIO, 背光)
 *
 *   W25Q128 SPI Flash (SPI1):
 *     SCK  -> PB8  (SPI1_SCK)
 *     MOSI -> PB6  (SPI1_MOSI)
 *     MISO -> PB7  (SPI1_MISO)
 *     CS   -> PB9  (GPIO)
 *
 *   按键:
 *     PB0 -> 上一张
 *     PB1 -> 下一张
 *     PB2 -> 自动播放/暂停
 *     PB3 -> 缩放切换
 *
 *   LED:
 *     PB14 -> 播放状态指示
 *
 * 功能：
 *   - 从SPI Flash读取图片数据并显示在TFT上
 *   - 支持RGB565格式原始位图（240x320或320x240）
 *   - 幻灯片自动播放（可调间隔）
 *   - 图片切换动画（淡入效果）
 *   - 支持缩放显示（1x/2x/适应屏幕）
 *   - 最多管理64张图片
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===== ILI9341 TFT 驱动 ===== */
#define TFT_WIDTH   240
#define TFT_HEIGHT  320

#define CS_LOW()    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_12)
#define CS_HIGH()   DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_12)
#define DC_LOW()    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13)
#define DC_HIGH()   DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_13)
#define RST_LOW()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14)
#define RST_HIGH()  DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14)
#define BL_ON()     DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_15)
#define BL_OFF()    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_15)

/* W25Q Flash CS */
#define FLASH_CS_LOW()   DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_9)
#define FLASH_CS_HIGH()  DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_9)

/* LED */
#define LED_ON()    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14)
#define LED_OFF()   DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14)
#define LED_TOGGLE() DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14)

/* 按键 */
#define BTN_PREV    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_0)))
#define BTN_NEXT    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_1)))
#define BTN_PLAY    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2)))
#define BTN_ZOOM    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3)))

/* 颜色定义 */
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF
#define COLOR_GRAY    0x8410

/* ===== W25Q Flash 命令 ===== */
#define W25Q_CMD_READ_DATA       0x03
#define W25Q_CMD_READ_SFDP       0x5A
#define W25Q_CMD_WRITE_ENABLE    0x06
#define W25Q_CMD_WRITE_DISABLE   0x04
#define W25Q_CMD_READ_STATUS1    0x05
#define W25Q_CMD_PAGE_PROGRAM    0x02
#define W25Q_CMD_SECTOR_ERASE    0x20  /* 4KB */
#define W25Q_CMD_BLOCK_ERASE_32K 0x52  /* 32KB */
#define W25Q_CMD_BLOCK_ERASE_64K 0xD8  /* 64KB */
#define W25Q_CMD_CHIP_ERASE      0xC7
#define W25Q_CMD_JEDEC_ID        0x9F
#define W25Q_CMD_POWER_DOWN      0xB9
#define W25Q_CMD_RELEASE_PD      0xAB

/* Flash参数 */
#define W25Q_PAGE_SIZE      256
#define W25Q_SECTOR_SIZE    4096
#define W25Q_BLOCK_SIZE     65536

/* 图片存储布局 */
#define IMG_HEADER_MAGIC    0x494D4731  /* "IMG1" */
#define IMG_SLOT_SIZE       (128 * 1024)  /* 128KB/张图片 */
#define IMG_MAX_SLOTS       64           /* 最多64张 */
#define IMG_DATA_START      (W25Q_BLOCK_SIZE) /* 从64KB偏移开始存储图片 */

/* 图片头结构 */
typedef struct {
    uint32_t magic;         /* 魔数标识 */
    uint16_t width;         /* 图片宽度 */
    uint16_t height;        /* 图片高度 */
    uint16_t format;        /* 格式: 0=RGB565 */
    uint16_t reserved;      /* 保留 */
    uint32_t data_size;     /* 数据大小(字节) */
    char     name[16];      /* 图片名称 */
} ImageHeader_t;  /* 32字节 */

/* 缩放模式 */
typedef enum {
    ZOOM_1X = 0,    /* 原始尺寸 */
    ZOOM_2X,        /* 2倍放大 */
    ZOOM_FIT,       /* 适应屏幕 */
    ZOOM_MODE_COUNT
} ZoomMode_t;

/* ===== SPI 发送 ===== */
static void SPI0_WriteByte(uint8_t dat)
{
    DL_SPI_transmitData8(SPI0, dat);
    while (DL_SPI_isBusy(SPI0)) {}
}

static uint8_t SPI0_ReadByte(void)
{
    DL_SPI_transmitData8(SPI0, 0xFF);
    while (DL_SPI_isBusy(SPI0)) {}
    return DL_SPI_receiveData8(SPI0);
}

static void SPI1_WriteByte(uint8_t dat)
{
    DL_SPI_transmitData8(SPI1, dat);
    while (DL_SPI_isBusy(SPI1)) {}
}

static uint8_t SPI1_ReadByte(void)
{
    DL_SPI_transmitData8(SPI1, 0xFF);
    while (DL_SPI_isBusy(SPI1)) {}
    return DL_SPI_receiveData8(SPI1);
}

/* ===== ILI9341 驱动 ===== */
static void ILI9341_WriteCmd(uint8_t cmd)
{
    DC_LOW(); CS_LOW();
    SPI0_WriteByte(cmd);
    CS_HIGH();
}

static void ILI9341_WriteData8(uint8_t dat)
{
    DC_HIGH(); CS_LOW();
    SPI0_WriteByte(dat);
    CS_HIGH();
}

static void ILI9341_WriteData16(uint16_t dat)
{
    DC_HIGH(); CS_LOW();
    SPI0_WriteByte(dat >> 8);
    SPI0_WriteByte(dat & 0xFF);
    CS_HIGH();
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
    DC_HIGH(); CS_LOW();
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        SPI0_WriteByte(color >> 8);
        SPI0_WriteByte(color & 0xFF);
    }
    CS_HIGH();
}

static void ILI9341_FillScreen(uint16_t color)
{
    ILI9341_FillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, color);
}

static void ILI9341_Init(void)
{
    RST_LOW();
    delay_cycles(1600000);
    RST_HIGH();
    delay_cycles(1600000);

    ILI9341_WriteCmd(0x01); /* 软复位 */
    delay_cycles(800000);

    ILI9341_WriteCmd(0xCF); /* Power Control B */
    ILI9341_WriteData8(0x00); ILI9341_WriteData8(0xC1); ILI9341_WriteData8(0x30);

    ILI9341_WriteCmd(0xED); /* Power on Sequence */
    ILI9341_WriteData8(0x64); ILI9341_WriteData8(0x03);
    ILI9341_WriteData8(0x12); ILI9341_WriteData8(0x81);

    ILI9341_WriteCmd(0xE8); /* Driver Timing A */
    ILI9341_WriteData8(0x85); ILI9341_WriteData8(0x00); ILI9341_WriteData8(0x78);

    ILI9341_WriteCmd(0xCB); /* Power Control A */
    ILI9341_WriteData8(0x39); ILI9341_WriteData8(0x2C);
    ILI9341_WriteData8(0x00); ILI9341_WriteData8(0x34); ILI9341_WriteData8(0x02);

    ILI9341_WriteCmd(0xF7); /* Pump Ratio */
    ILI9341_WriteData8(0x20);

    ILI9341_WriteCmd(0xEA); /* Driver Timing B */
    ILI9341_WriteData8(0x00); ILI9341_WriteData8(0x00);

    ILI9341_WriteCmd(0xC0); /* Power Control 1 */
    ILI9341_WriteData8(0x23); /* VRH=4.60V */

    ILI9341_WriteCmd(0xC1); /* Power Control 2 */
    ILI9341_WriteData8(0x10); /* step-up factor */

    ILI9341_WriteCmd(0xC5); /* VCOM Control 1 */
    ILI9341_WriteData8(0x3E); ILI9341_WriteData8(0x28);

    ILI9341_WriteCmd(0xC7); /* VCOM Control 2 */
    ILI9341_WriteData8(0x86);

    ILI9341_WriteCmd(0x36); /* MADCTL */
    ILI9341_WriteData8(0x48); /* 竖屏 */

    ILI9341_WriteCmd(0x3A); /* Pixel Format */
    ILI9341_WriteData8(0x55); /* 16bit */

    ILI9341_WriteCmd(0xB1); /* Frame Control */
    ILI9341_WriteData8(0x00); ILI9341_WriteData8(0x18); /* 79Hz */

    ILI9341_WriteCmd(0xB6); /* Display Function */
    ILI9341_WriteData8(0x08); ILI9341_WriteData8(0x82);
    ILI9341_WriteData8(0x27);

    ILI9341_WriteCmd(0xF2); /* 3Gamma */
    ILI9341_WriteData8(0x00); /* 禁用 */

    ILI9341_WriteCmd(0x26); /* Gamma */
    ILI9341_WriteData8(0x01);

    ILI9341_WriteCmd(0xE0); /* Positive Gamma */
    {
        uint8_t pgamma[] = {0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,
                            0x37,0x07,0x10,0x03,0x0E,0x09,0x00};
        for (int i = 0; i < 15; i++) ILI9341_WriteData8(pgamma[i]);
    }

    ILI9341_WriteCmd(0xE1); /* Negative Gamma */
    {
        uint8_t ngamma[] = {0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,
                            0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F};
        for (int i = 0; i < 15; i++) ILI9341_WriteData8(ngamma[i]);
    }

    ILI9341_WriteCmd(0x11); /* Sleep Out */
    delay_cycles(800000);
    ILI9341_WriteCmd(0x29); /* Display On */
    delay_cycles(160000);
    BL_ON();
}

/* ===== W25Q Flash 驱动 ===== */

/* 等待Flash就绪 */
static void W25Q_WaitBusy(void)
{
    FLASH_CS_LOW();
    SPI1_WriteByte(W25Q_CMD_READ_STATUS1);
    uint32_t timeout = 100000;
    while (timeout--) {
        if (!(SPI1_ReadByte() & 0x01)) break;
    }
    FLASH_CS_HIGH();
}

/* 读取JEDEC ID */
static uint32_t W25Q_ReadID(void)
{
    uint32_t id;
    FLASH_CS_LOW();
    SPI1_WriteByte(W25Q_CMD_JEDEC_ID);
    id  = (uint32_t)SPI1_ReadByte() << 16;
    id |= (uint32_t)SPI1_ReadByte() << 8;
    id |= SPI1_ReadByte();
    FLASH_CS_HIGH();
    return id;
}

/* 写使能 */
static void W25Q_WriteEnable(void)
{
    FLASH_CS_LOW();
    SPI1_WriteByte(W25Q_CMD_WRITE_ENABLE);
    FLASH_CS_HIGH();
}

/* 读取数据 */
static void W25Q_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
    FLASH_CS_LOW();
    SPI1_WriteByte(W25Q_CMD_READ_DATA);
    SPI1_WriteByte((addr >> 16) & 0xFF);
    SPI1_WriteByte((addr >> 8) & 0xFF);
    SPI1_WriteByte(addr & 0xFF);
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = SPI1_ReadByte();
    }
    FLASH_CS_HIGH();
}

/* 扇区擦除 (4KB) */
static void W25Q_EraseSector(uint32_t addr)
{
    W25Q_WriteEnable();
    FLASH_CS_LOW();
    SPI1_WriteByte(W25Q_CMD_SECTOR_ERASE);
    SPI1_WriteByte((addr >> 16) & 0xFF);
    SPI1_WriteByte((addr >> 8) & 0xFF);
    SPI1_WriteByte(addr & 0xFF);
    FLASH_CS_HIGH();
    W25Q_WaitBusy();
}

/* 页编程 (256字节/次) */
static void W25Q_PageProgram(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    if (len > W25Q_PAGE_SIZE) len = W25Q_PAGE_SIZE;
    W25Q_WriteEnable();
    FLASH_CS_LOW();
    SPI1_WriteByte(W25Q_CMD_PAGE_PROGRAM);
    SPI1_WriteByte((addr >> 16) & 0xFF);
    SPI1_WriteByte((addr >> 8) & 0xFF);
    SPI1_WriteByte(addr & 0xFF);
    for (uint16_t i = 0; i < len; i++) {
        SPI1_WriteByte(buf[i]);
    }
    FLASH_CS_HIGH();
    W25Q_WaitBusy();
}

/* 写入任意长度数据（自动跨页） */
static void W25Q_WriteData(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    while (len > 0) {
        uint16_t page_remain = W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE);
        uint16_t chunk = (len < page_remain) ? (uint16_t)len : page_remain;
        W25Q_PageProgram(addr, buf, chunk);
        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
}

/* 读取图片头 */
static bool W25Q_ReadImageHeader(uint8_t slot, ImageHeader_t *hdr)
{
    uint32_t addr = IMG_DATA_START + (uint32_t)slot * IMG_SLOT_SIZE;
    W25Q_ReadData(addr, (uint8_t *)hdr, sizeof(ImageHeader_t));
    return (hdr->magic == IMG_HEADER_MAGIC);
}

/* ===== 简易 8x8 字体 ===== */
static const uint8_t font_8x8[][8] = {
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
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, /* '.' */
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, /* '-' */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x7C,0x66,0x66,0x7C,0x6C,0x66,0x66,0x00}, /* 'R' */
    {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00}, /* 'C' */
    {0x7E,0x60,0x60,0x7C,0x60,0x60,0x7E,0x00}, /* 'E' */
    {0x7E,0x60,0x60,0x7C,0x60,0x60,0x60,0x00}, /* 'F' */
    {0x66,0x66,0x66,0x7E,0x66,0x66,0x3C,0x00}, /* 'H' */
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, /* 'T' */
    {0x66,0x76,0x7E,0x7E,0x6E,0x66,0x66,0x00}, /* 'M' */
    {0x7C,0x66,0x66,0x7C,0x70,0x68,0x64,0x00}, /* 'P' */
    {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, /* 'O' */
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, /* 'I' */
    {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00}, /* 'S' */
    {0x3C,0x66,0x66,0x7E,0x66,0x66,0x66,0x00}, /* 'A' */
    {0x7C,0x66,0x66,0x66,0x66,0x66,0x7C,0x00}, /* 'D' */
    {0x06,0x06,0x06,0x06,0x06,0x06,0x7E,0x00}, /* 'L' */
    {0x3C,0x06,0x06,0x1C,0x06,0x06,0x3C,0x00}, /* '3' dup - use as G */
    {0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00}, /* U */
    {0xC3,0xE7,0xFF,0xDB,0xC3,0xC3,0xC3,0x00}, /* W */
};

static int CharToIdx(char ch)
{
    if (ch >= '0' && ch <= '9') return ch - '0';
    switch(ch) {
    case '.': return 10; case '-': return 11; case ' ': return 12;
    case 'R': case 'r': return 13; case 'C': case 'c': return 14;
    case 'E': case 'e': return 15; case 'F': case 'f': return 16;
    case 'H': case 'h': return 17; case 'T': case 't': return 18;
    case 'M': case 'm': return 19; case 'P': case 'p': return 20;
    case 'O': case 'o': return 21; case 'I': case 'i': return 22;
    case 'S': case 's': return 23; case 'A': case 'a': return 24;
    case 'D': case 'd': return 25; case 'L': case 'l': return 26;
    case 'G': case 'g': return 27; case 'U': case 'u': return 28;
    case 'W': case 'w': return 29;
    default: return -1;
    }
}

static void DrawChar(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg, uint8_t scale)
{
    int idx = CharToIdx(ch);
    const uint8_t *glyph;
    static const uint8_t blank[8] = {0};
    glyph = (idx >= 0) ? font_8x8[idx] : blank;

    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            uint16_t c = (glyph[row] & (0x80 >> col)) ? fg : bg;
            if (scale == 1) {
                DrawPixel(x + col, y + row, c);
            } else {
                ILI9341_FillRect(x + col * scale, y + row * scale, scale, scale, c);
            }
        }
    }
}

static void DrawText(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t sc)
{
    while (*s) { DrawChar(x, y, *s, fg, bg, sc); x += 8 * sc; s++; }
}

/* 注意：上面DrawChar里DrawPixel用的是ST7789宏，这里统一为ILI9341版本 */
/* 修正：重写DrawPixel为ILI9341版本 */
static void DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT) return;
    ILI9341_SetWindow(x, y, x, y);
    ILI9341_WriteData16(color);
}

/* ===== 从Flash直接写入TFT（流式传输，避免大量RAM缓冲）===== */
static void FlashToTFT(uint32_t flash_addr, uint16_t x, uint16_t y,
                        uint16_t w, uint16_t h)
{
    ILI9341_SetWindow(x, y, x + w - 1, y + h - 1);
    DC_HIGH(); CS_LOW();

    FLASH_CS_LOW();
    SPI1_WriteByte(W25Q_CMD_READ_DATA);
    SPI1_WriteByte((flash_addr >> 16) & 0xFF);
    SPI1_WriteByte((flash_addr >> 8) & 0xFF);
    SPI1_WriteByte(flash_addr & 0xFF);

    uint32_t total = (uint32_t)w * h * 2; /* RGB565 = 2 bytes/pixel */
    for (uint32_t i = 0; i < total; i++) {
        uint8_t b = SPI1_ReadByte();
        SPI0_WriteByte(b);  /* 直接转发到TFT */
    }
    FLASH_CS_HIGH();
    CS_HIGH();
}

/* 带缩放的图片显示 */
static void DisplayImageScaled(uint8_t slot, ZoomMode_t zoom)
{
    ImageHeader_t hdr;
    if (!W25Q_ReadImageHeader(slot, &hdr)) {
        ILI9341_FillScreen(COLOR_BLACK);
        DrawText(30, 150, "NO IMAGE", COLOR_RED, COLOR_BLACK, 2);
        return;
    }

    uint32_t img_data_addr = IMG_DATA_START + (uint32_t)slot * IMG_SLOT_SIZE + sizeof(ImageHeader_t);

    switch (zoom) {
    case ZOOM_1X:
        /* 原始尺寸居中显示 */
        {
            int ox = (TFT_WIDTH - hdr.width) / 2;
            int oy = (TFT_HEIGHT - hdr.height) / 2;
            if (ox < 0) ox = 0; if (oy < 0) oy = 0;
            ILI9341_FillScreen(COLOR_BLACK);
            FlashToTFT(img_data_addr, ox, oy, hdr.width, hdr.height);
        }
        break;

    case ZOOM_2X:
        /* 2倍放大 — 逐像素复制 */
        {
            ILI9341_FillScreen(COLOR_BLACK);
            /* 读取并放大显示 (使用小缓冲区) */
            uint16_t line_buf[240]; /* 单行缓冲 */
            for (uint16_t row = 0; row < hdr.height && row < 160; row++) {
                uint32_t line_addr = img_data_addr + (uint32_t)row * hdr.width * 2;
                /* 读取一行 */
                FLASH_CS_LOW();
                SPI1_WriteByte(W25Q_CMD_READ_DATA);
                SPI1_WriteByte((line_addr >> 16) & 0xFF);
                SPI1_WriteByte((line_addr >> 8) & 0xFF);
                SPI1_WriteByte(line_addr & 0xFF);
                for (uint16_t i = 0; i < hdr.width && i < 240; i++) {
                    uint8_t hi = SPI1_ReadByte();
                    uint8_t lo = SPI1_ReadByte();
                    line_buf[i] = (hi << 8) | lo;
                }
                FLASH_CS_HIGH();

                /* 放大2倍写入 */
                for (uint16_t r = 0; r < 2; r++) {
                    int y_out = row * 2 + r;
                    if (y_out >= TFT_HEIGHT) break;
                    ILI9341_SetWindow(0, y_out, hdr.width * 2 - 1, y_out);
                    DC_HIGH(); CS_LOW();
                    for (uint16_t i = 0; i < hdr.width && i * 2 < TFT_WIDTH; i++) {
                        uint16_t c = line_buf[i];
                        SPI0_WriteByte(c >> 8); SPI0_WriteByte(c & 0xFF);
                        SPI0_WriteByte(c >> 8); SPI0_WriteByte(c & 0xFF);
                    }
                    CS_HIGH();
                }
            }
        }
        break;

    case ZOOM_FIT:
        /* 适应屏幕 — 最近邻缩放 */
        {
            ILI9341_FillScreen(COLOR_BLACK);
            uint16_t line_buf[240];
            for (uint16_t row = 0; row < TFT_HEIGHT; row++) {
                uint16_t src_row = (uint16_t)((uint32_t)row * hdr.height / TFT_HEIGHT);
                uint32_t line_addr = img_data_addr + (uint32_t)src_row * hdr.width * 2;

                FLASH_CS_LOW();
                SPI1_WriteByte(W25Q_CMD_READ_DATA);
                SPI1_WriteByte((line_addr >> 16) & 0xFF);
                SPI1_WriteByte((line_addr >> 8) & 0xFF);
                SPI1_WriteByte(line_addr & 0xFF);
                for (uint16_t i = 0; i < hdr.width && i < 240; i++) {
                    uint8_t hi = SPI1_ReadByte();
                    uint8_t lo = SPI1_ReadByte();
                    line_buf[i] = (hi << 8) | lo;
                }
                FLASH_CS_HIGH();

                ILI9341_SetWindow(0, row, TFT_WIDTH - 1, row);
                DC_HIGH(); CS_LOW();
                for (uint16_t col = 0; col < TFT_WIDTH; col++) {
                    uint16_t src_col = (uint16_t)((uint32_t)col * hdr.width / TFT_WIDTH);
                    uint16_t c = line_buf[src_col];
                    SPI0_WriteByte(c >> 8); SPI0_WriteByte(c & 0xFF);
                }
                CS_HIGH();
            }
        }
        break;

    default: break;
    }
}

/* 显示图片信息叠加层 */
static void ShowImageInfo(uint8_t slot, uint8_t total, const ImageHeader_t *hdr)
{
    char info[32];
    /* 底部状态栏 */
    ILI9341_FillRect(0, TFT_HEIGHT - 20, TFT_WIDTH, 20, 0x2104);
    /* 图片序号 */
    DrawText(4, TFT_HEIGHT - 16, "IMAGE", COLOR_WHITE, 0x2104, 1);
    /* 图片名称 */
    DrawText(120, TFT_HEIGHT - 16, hdr->name, COLOR_YELLOW, 0x2104, 1);
}

/* ===== 淡入效果 ===== */
static void FadeInEffect(uint8_t slot, ZoomMode_t zoom)
{
    /* 简易淡入：逐步提高背光亮度 */
    BL_OFF();
    DisplayImageScaled(slot, zoom);

    /* 通过快速开关背光模拟淡入 */
    for (int i = 0; i < 8; i++) {
        BL_ON();
        delay_cycles((i + 1) * 40000);
        BL_OFF();
        delay_cycles((8 - i) * 40000);
    }
    BL_ON();
}

/* ===== 按键消抖 ===== */
typedef struct {
    uint8_t prev;
    uint8_t pressed;
} Button_t;

static Button_t btn_prev = {0, 0};
static Button_t btn_next = {0, 0};
static Button_t btn_play = {0, 0};
static Button_t btn_zoom = {0, 0};

static void UpdateButton(Button_t *btn, uint8_t raw)
{
    btn->pressed = (raw && !btn->prev) ? 1 : 0;
    btn->prev = raw;
}

static void ScanButtons(void)
{
    UpdateButton(&btn_prev, BTN_PREV ? 1 : 0);
    UpdateButton(&btn_next, BTN_NEXT ? 1 : 0);
    UpdateButton(&btn_play, BTN_PLAY ? 1 : 0);
    UpdateButton(&btn_zoom, BTN_ZOOM ? 1 : 0);
}

/* ===== 延时 ===== */
static void delay_ms(uint32_t ms) { delay_cycles(ms * 16000); }

/* ===== 主函数 ===== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* TFT初始化 */
    ILI9341_Init();
    ILI9341_FillScreen(COLOR_BLACK);

    /* 启动画面 */
    DrawText(30, 60, "IMAGE VIEWER", COLOR_CYAN, COLOR_BLACK, 2);
    DrawText(50, 100, "V1.0", COLOR_GRAY, COLOR_BLACK, 2);
    delay_ms(1000);

    /* 检测Flash */
    uint32_t flash_id = W25Q_ReadID();
    /* 显示Flash ID */
    {
        char id_str[16];
        id_str[0] = '0'; id_str[1] = 'x';
        uint32_t id = flash_id;
        for (int i = 9; i >= 2; i--) {
            uint8_t nib = id & 0xF;
            id_str[i] = nib < 10 ? '0' + nib : 'A' + nib - 10;
            id >>= 4;
        }
        id_str[10] = '\0';
        ILI9341_FillScreen(COLOR_BLACK);
        DrawText(20, 100, "FLASH ID:", COLOR_GREEN, COLOR_BLACK, 2);
        DrawText(20, 130, id_str, COLOR_YELLOW, COLOR_BLACK, 2);
    }
    delay_ms(1500);

    /* 扫描已存储图片数量 */
    uint8_t image_count = 0;
    for (uint8_t i = 0; i < IMG_MAX_SLOTS; i++) {
        ImageHeader_t hdr;
        if (W25Q_ReadImageHeader(i, &hdr)) {
            image_count++;
        } else {
            break;  /* 遇到空槽就停止 */
        }
    }

    /* 如果没有图片，显示提示 */
    if (image_count == 0) {
        ILI9341_FillScreen(COLOR_BLACK);
        DrawText(20, 100, "NO IMAGES", COLOR_RED, COLOR_BLACK, 2);
        DrawText(20, 130, "IN FLASH!", COLOR_RED, COLOR_BLACK, 2);
        while (1) {
            LED_TOGGLE();
            delay_ms(500);
        }
    }

    /* 显示第一张图片 */
    uint8_t current_img = 0;
    ZoomMode_t zoom_mode = ZOOM_FIT;
    bool auto_play = false;
    uint32_t auto_timer = 0;
    uint32_t auto_interval = 3000;  /* 3秒/张 */
    uint8_t need_display = 1;

    /* 主循环 */
    while (1) {
        /* 按键扫描 */
        ScanButtons();

        /* 上一张 */
        if (btn_prev.pressed) {
            if (current_img > 0) current_img--;
            else current_img = image_count - 1;
            need_display = 1;
        }

        /* 下一张 */
        if (btn_next.pressed) {
            current_img = (current_img + 1) % image_count;
            need_display = 1;
        }

        /* 播放/暂停 */
        if (btn_play.pressed) {
            auto_play = !auto_play;
            auto_timer = 0;
            if (auto_play) LED_ON(); else LED_OFF();
        }

        /* 缩放切换 */
        if (btn_zoom.pressed) {
            zoom_mode = (ZoomMode_t)((zoom_mode + 1) % ZOOM_MODE_COUNT);
            need_display = 1;
        }

        /* 自动播放定时 */
        if (auto_play) {
            auto_timer += 30;
            if (auto_timer >= auto_interval) {
                auto_timer = 0;
                current_img = (current_img + 1) % image_count;
                need_display = 1;
                LED_TOGGLE();
            }
        }

        /* 显示图片 */
        if (need_display) {
            need_display = 0;
            FadeInEffect(current_img, zoom_mode);

            /* 显示信息 */
            ImageHeader_t hdr;
            if (W25Q_ReadImageHeader(current_img, &hdr)) {
                ShowImageInfo(current_img, image_count, &hdr);
            }
        }

        delay_ms(30);
    }

    return 0;
}
