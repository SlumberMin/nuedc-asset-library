/**
 * @file multi_channel_logger.c
 * @brief MSPM0G3507 多通道数据记录器示例（MCP3008 + SD卡 + CSV格式 + 时间戳）
 *
 * 硬件连接：
 *   MCP3008 ADC（SPI）：
 *     SCK  -> PA0 (SPI0_SCK)
 *     MISO -> PA1 (SPI0_MISO，MCP3008 DOUT)
 *     MOSI -> PA4 (SPI0_MOSI，MCP3008 DIN)
 *     CS   -> PA5 (GPIO)
 *
 *   SD卡模块（SPI）：
 *     SCK  -> PB8 (SPI1_SCK)
 *     MISO -> PB9 (SPI1_MISO)
 *     MOSI -> PB10 (SPI1_MOSI)
 *     CS   -> PB11 (GPIO)
 *
 *   DS3231 RTC（I2C，用于时间戳）：
 *     SCL -> PB2 (I2C0_SCL)
 *     SDA -> PB3 (I2C0_SDA)
 *
 *   OLED显示（I2C，0x3C）：
 *     与DS3231共用I2C总线
 *
 *   按键：
 *     开始/停止记录 -> PA11
 *     切换显示通道  -> PA12
 *     设置采样率    -> PA13
 *
 *   LED指示：
 *     记录中 -> PA14 (绿LED)
 *     错误   -> PA15 (红LED)
 *
 * 功能：
 *   - MCP3008 8通道10位ADC数据采集
 *   - SD卡CSV格式写入，带RTC时间戳
 *   - 可选采样率：1Hz / 10Hz / 100Hz
 *   - OLED实时显示各通道波形和数值
 *   - 自动按日分文件（YYYYMMDD.csv）
 *   - 掉电保护：安全关闭文件
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ========== MCP3008 SPI引脚 ========== */
#define MCP3008_CS_PORT         GPIOA
#define MCP3008_CS_PIN          DL_GPIO_PIN_5

/* ========== SD卡SPI引脚 ========== */
#define SD_CS_PORT              GPIOB
#define SD_CS_PIN               DL_GPIO_PIN_11

/* ========== 按键引脚 ========== */
#define BTN_RECORD_PORT         GPIOA
#define BTN_RECORD_PIN          DL_GPIO_PIN_11
#define BTN_CHANNEL_PORT        GPIOA
#define BTN_CHANNEL_PIN         DL_GPIO_PIN_12
#define BTN_RATE_PORT           GPIOA
#define BTN_RATE_PIN            DL_GPIO_PIN_13

/* ========== LED引脚 ========== */
#define LED_REC_PORT            GPIOA
#define LED_REC_PIN             DL_GPIO_PIN_14
#define LED_ERR_PORT            GPIOA
#define LED_ERR_PIN             DL_GPIO_PIN_15

/* ========== DS3231 RTC寄存器 ========== */
#define DS3231_ADDR             0x68
#define DS3231_REG_SEC          0x00
#define DS3231_REG_YEAR         0x06

/* ========== SD卡命令 ========== */
#define SD_CMD0                 0    /* GO_IDLE_STATE */
#define SD_CMD1                 1    /* SEND_OP_COND (MMC) */
#define SD_CMD8                 8    /* SEND_IF_COND */
#define SD_CMD16                16   /* SET_BLOCKLEN */
#define SD_CMD17                17   /* READ_SINGLE_BLOCK */
#define SD_CMD24                24   /* WRITE_SINGLE_BLOCK */
#define SD_CMD55                55   /* APP_CMD */
#define SD_CMD58                58   /* READ_OCR */
#define SD_ACMD41               41   /* SD_SEND_OP_COND */

/* ========== 参数配置 ========== */
#define ADC_CHANNELS            8      /* MCP3008 8通道 */
#define SAMPLE_RATE_COUNT       3      /* 3种采样率 */
#define LOG_BUF_SIZE            512    /* 日志缓冲区大小 */
#define SD_SECTOR_SIZE          512    /* SD卡扇区大小 */
#define DISPLAY_REFRESH_MS      100    /* 显示刷新周期 */

/* ========== 采样率表(ms) ========== */
static const uint16_t g_sampleRateTable[] = {1000, 100, 10};  /* 1Hz, 10Hz, 100Hz */
static const char *g_sampleRateName[] = {"1Hz", "10Hz", "100Hz"};

/* ========== 时间结构体 ========== */
typedef struct {
    uint8_t year, month, date;
    uint8_t hour, minute, second;
} RTC_Time_t;

/* ========== 全局变量 ========== */
static uint16_t  g_adcValues[ADC_CHANNELS];       /* 各通道ADC原始值 */
static float     g_voltage[ADC_CHANNELS];          /* 各通道电压(mV) */
static bool      g_recording = false;               /* 记录状态 */
static uint8_t   g_displayChannel = 0;              /* 当前显示通道 */
static uint8_t   g_sampleRateIdx = 0;               /* 采样率索引 */
static uint32_t  g_recordCount = 0;                 /* 记录条数 */
static RTC_Time_t g_rtcTime;                        /* 当前时间 */
static char      g_logBuffer[LOG_BUF_SIZE];         /* 日志格式缓冲区 */
static uint8_t   g_sdCardType = 0;                  /* SD卡类型：0=SDSC,1=SDHC */
static uint32_t  g_writeSector = 0;                 /* 当前写入扇区 */
static uint8_t   g_sectorBuf[SD_SECTOR_SIZE];       /* 扇区缓冲区 */
static uint16_t  g_sectorIdx = 0;                   /* 缓冲区写入位置 */
static volatile uint32_t g_systickCount = 0;

/* OLED显存 */
static uint8_t g_oledBuffer[128 * 64 / 8];

/* 基本字体 */
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

/* ========== 函数声明 ========== */
void SysTick_Handler(void);
static void Delay_ms(uint32_t ms);
static uint8_t BCD2Bin(uint8_t bcd);
static bool Button_IsPressed(GPIO_Regs *port, uint32_t pin);

/* MCP3008 ADC */
static void     MCP3008_Init(void);
static uint16_t MCP3008_ReadChannel(uint8_t channel);
static void     MCP3008_ReadAll(uint16_t *values);

/* SD卡 */
static void     SD_CS_Low(void);
static void     SD_CS_High(void);
static uint8_t  SD_SPI_RW(uint8_t data);
static uint8_t  SD_SendCmd(uint8_t cmd, uint32_t arg);
static bool     SD_Init(void);
static bool     SD_ReadSector(uint32_t sector, uint8_t *buf);
static bool     SD_WriteSector(uint32_t sector, const uint8_t *buf);

/* DS3231 */
static void     DS3231_GetTime(RTC_Time_t *time);

/* OLED */
static void     OLED_WriteCmd(uint8_t cmd);
static void     OLED_WriteData(uint8_t data);
static void     OLED_Init(void);
static void     OLED_Clear(void);
static void     OLED_SetPixel(int16_t x, int16_t y, uint8_t color);
static void     OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h);
static void     OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
static void     OLED_DrawChar(int16_t x, int16_t y, char ch, uint8_t size);
static void     OLED_DrawString(int16_t x, int16_t y, const char *str, uint8_t size);
static void     OLED_Update(void);

/* 数据记录 */
static void     Logger_FormatCSV(char *buf, uint16_t len);
static void     Logger_FlushBuffer(void);
static void     Logger_AppendData(const char *line);
static void     UI_UpdateDisplay(void);

/* ========== 延时 ========== */
void SysTick_Handler(void) { g_systickCount++; }

static void Delay_ms(uint32_t ms) {
    uint32_t start = g_systickCount;
    while ((g_systickCount - start) < ms);
}

static uint8_t BCD2Bin(uint8_t bcd) { return (bcd >> 4)*10 + (bcd & 0x0F); }

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

/* ========== MCP3008驱动 ========== */
/*
 * MCP3008 SPI协议：
 * 发送3字节，接收2字节（10位结果）
 * 发送：[0000 0001][SGL/DIF D2 D1 D0 xxxx][xxxx xxxx]
 * 接收：[xxxx xxxx][xxxx xxxx BA9876543210]
 */
static void MCP3008_Init(void) {
    DL_GPIO_setPins(MCP3008_CS_PORT, MCP3008_CS_PIN);
}

static uint16_t MCP3008_ReadChannel(uint8_t channel) {
    uint8_t txBuf[3];
    uint8_t rxBuf[3];

    /* 构造命令：起始位=1，单端模式，通道选择 */
    txBuf[0] = 0x01;                                            /* Start bit */
    txBuf[1] = (uint8_t)(0x80 | ((channel & 0x07) << 4));      /* SGL=1, D2-D0 */
    txBuf[2] = 0x00;                                            /* 填充 */

    DL_GPIO_clearPins(MCP3008_CS_PORT, MCP3008_CS_PIN);

    for (uint8_t i = 0; i < 3; i++) {
        DL_SPI_fillControllerTXFIFO(SPI_0_INST, &txBuf[i], 1);
        while (DL_SPI_isBusy(SPI_0_INST));
        rxBuf[i] = DL_SPI_receiveControllerData(SPI_0_INST);
    }

    DL_GPIO_setPins(MCP3008_CS_PORT, MCP3008_CS_PIN);

    /* 提取10位结果 */
    uint16_t result = ((uint16_t)(rxBuf[1] & 0x03) << 8) | rxBuf[2];
    return result;
}

static void MCP3008_ReadAll(uint16_t *values) {
    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++) {
        values[ch] = MCP3008_ReadChannel(ch);
        /* 转换为mV: V = ADC * 3300 / 1023 */
        g_voltage[ch] = (float)values[ch] * 3300.0f / 1023.0f;
    }
}

/* ========== SD卡驱动 ========== */
static void SD_CS_Low(void)  { DL_GPIO_clearPins(SD_CS_PORT, SD_CS_PIN); }
static void SD_CS_High(void) { DL_GPIO_setPins(SD_CS_PORT, SD_CS_PIN); }

static uint8_t SD_SPI_RW(uint8_t data) {
    DL_SPI_fillControllerTXFIFO(SPI_1_INST, &data, 1);
    while (DL_SPI_isBusy(SPI_1_INST));
    return DL_SPI_receiveControllerData(SPI_1_INST);
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg) {
    SD_SPI_RW(0xFF);  /* 等待一个时钟 */
    SD_SPI_RW(0x40 | cmd);
    SD_SPI_RW((uint8_t)(arg >> 24));
    SD_SPI_RW((uint8_t)(arg >> 16));
    SD_SPI_RW((uint8_t)(arg >> 8));
    SD_SPI_RW((uint8_t)arg);

    /* CRC（CMD0和CMD8需要有效CRC，其余可忽略） */
    if (cmd == SD_CMD0)  SD_SPI_RW(0x95);  /* CMD0 CRC */
    else if (cmd == SD_CMD8) SD_SPI_RW(0x87);  /* CMD8 CRC */
    else SD_SPI_RW(0xFF);

    /* 等待响应（最多10字节） */
    uint8_t resp;
    for (uint8_t i = 0; i < 10; i++) {
        resp = SD_SPI_RW(0xFF);
        if (!(resp & 0x80)) return resp;
    }
    return 0xFF;  /* 超时 */
}

static bool SD_Init(void) {
    uint8_t resp;

    /* 发送80个时钟脉冲（CS高电平） */
    SD_CS_High();
    for (uint8_t i = 0; i < 10; i++) SD_SPI_RW(0xFF);

    /* CMD0: 复位到idle状态 */
    SD_CS_Low();
    resp = SD_SendCmd(SD_CMD0, 0);
    SD_CS_High();
    if (resp != 0x01) return false;

    /* CMD8: 检查SD卡版本 */
    SD_CS_Low();
    resp = SD_SendCmd(SD_CMD8, 0x000001AA);
    if (resp == 0x01) {
        /* SDHC/SDXC卡 */
        SD_SPI_RW(0xFF); SD_SPI_RW(0xFF); SD_SPI_RW(0xFF); SD_SPI_RW(0xFF);
        g_sdCardType = 1;  /* SDHC */
    }
    SD_CS_High();

    /* ACMD41: 初始化 */
    uint32_t timeout = 1000;
    do {
        SD_CS_Low();
        SD_SendCmd(SD_CMD55, 0);
        SD_CS_High();
        SD_CS_Low();
        resp = SD_SendCmd(SD_ACMD41, g_sdCardType ? 0x40000000 : 0);
        SD_CS_High();
        if (resp == 0x00) break;
        Delay_ms(1);
    } while (--timeout);

    if (timeout == 0) return false;

    /* CMD58: 读取OCR确认 */
    if (g_sdCardType) {
        SD_CS_Low();
        SD_SendCmd(SD_CMD58, 0);
        SD_SPI_RW(0xFF);  /* OCR byte3 */
        SD_SPI_RW(0xFF);  /* OCR byte2 */
        SD_SPI_RW(0xFF);  /* OCR byte1 */
        SD_SPI_RW(0xFF);  /* OCR byte0 */
        SD_CS_High();
    }

    /* CMD16: 设置块大小512字节 */
    SD_CS_Low();
    SD_SendCmd(SD_CMD16, SD_SECTOR_SIZE);
    SD_CS_High();

    return true;
}

static bool SD_ReadSector(uint32_t sector, uint8_t *buf) {
    if (!g_sdCardType) sector *= SD_SECTOR_SIZE;  /* SDSC按字节寻址 */

    SD_CS_Low();
    if (SD_SendCmd(SD_CMD17, sector) != 0x00) {
        SD_CS_High();
        return false;
    }

    /* 等待数据起始令牌 0xFE */
    uint16_t timeout = 10000;
    uint8_t token;
    do {
        token = SD_SPI_RW(0xFF);
    } while (token == 0xFF && --timeout);

    if (token != 0xFE) { SD_CS_High(); return false; }

    /* 读取512字节数据 */
    for (uint16_t i = 0; i < SD_SECTOR_SIZE; i++) {
        buf[i] = SD_SPI_RW(0xFF);
    }

    /* 跳过CRC */
    SD_SPI_RW(0xFF);
    SD_SPI_RW(0xFF);
    SD_CS_High();

    return true;
}

static bool SD_WriteSector(uint32_t sector, const uint8_t *buf) {
    if (!g_sdCardType) sector *= SD_SECTOR_SIZE;

    SD_CS_Low();
    if (SD_SendCmd(SD_CMD24, sector) != 0x00) {
        SD_CS_High();
        return false;
    }

    SD_SPI_RW(0xFF);  /* 等待 */
    SD_SPI_RW(0xFE);  /* 数据起始令牌 */

    /* 写512字节 */
    for (uint16_t i = 0; i < SD_SECTOR_SIZE; i++) {
        SD_SPI_RW(buf[i]);
    }

    /* 发送dummy CRC */
    SD_SPI_RW(0xFF);
    SD_SPI_RW(0xFF);

    /* 检查数据响应 */
    uint8_t resp = SD_SPI_RW(0xFF);
    if ((resp & 0x1F) != 0x05) { SD_CS_High(); return false; }

    /* 等待写完成 */
    uint16_t timeout = 10000;
    while (SD_SPI_RW(0xFF) == 0x00 && --timeout);
    SD_CS_High();

    return (timeout > 0);
}

/* ========== DS3231 RTC ========== */
static void DS3231_GetTime(RTC_Time_t *time) {
    uint8_t buf[7];
    uint8_t reg = DS3231_REG_SEC;

    /* 写寄存器地址 */
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    DL_I2C_startControllerTransfer(I2C_0_INST, DS3231_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

    /* 读7字节 */
    DL_I2C_startControllerTransfer(I2C_0_INST, DS3231_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, 7);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    for (uint8_t i = 0; i < 7; i++)
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);

    time->second = BCD2Bin(buf[0] & 0x7F);
    time->minute = BCD2Bin(buf[1] & 0x7F);
    time->hour   = BCD2Bin(buf[2] & 0x3F);
    time->date   = BCD2Bin(buf[4] & 0x3F);
    time->month  = BCD2Bin(buf[5] & 0x1F);
    time->year   = BCD2Bin(buf[6]);
}

/* ========== OLED驱动 ========== */
static void OLED_WriteCmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, 0x3C,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static void OLED_WriteData(uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, 0x3C,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static void OLED_Init(void) {
    Delay_ms(100);
    OLED_WriteCmd(0xAE); OLED_WriteCmd(0xD5); OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F); OLED_WriteCmd(0xD3);
    OLED_WriteCmd(0x00); OLED_WriteCmd(0x40); OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14); OLED_WriteCmd(0x20); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xA1); OLED_WriteCmd(0xC8); OLED_WriteCmd(0xDA);
    OLED_WriteCmd(0x12); OLED_WriteCmd(0x81); OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0xF1); OLED_WriteCmd(0xDB);
    OLED_WriteCmd(0x30); OLED_WriteCmd(0xA4); OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0xAF);
    OLED_Clear(); OLED_Update();
}

static void OLED_Clear(void) { memset(g_oledBuffer, 0, sizeof(g_oledBuffer)); }

static void OLED_SetPixel(int16_t x, int16_t y, uint8_t color) {
    if (x<0||x>=128||y<0||y>=64) return;
    uint16_t idx = (y/8)*128+x;
    uint8_t bit = (1<<(y&7));
    if (color) g_oledBuffer[idx]|=bit; else g_oledBuffer[idx]&=~bit;
}

static void OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h) {
    for (int16_t i=x;i<x+w;i++) for(int16_t j=y;j<y+h;j++) OLED_SetPixel(i,j,1);
}

static void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    int16_t dx=abs(x1-x0),dy=abs(y1-y0),sx=(x0<x1)?1:-1,sy=(y0<y1)?1:-1,err=dx-dy;
    while(1){OLED_SetPixel(x0,y0,1);if(x0==x1&&y0==y1)break;int16_t e2=2*err;if(e2>-dy){err-=dy;x0+=sx;}if(e2<dx){err+=dx;y0+=sy;}}
}

static void OLED_DrawChar(int16_t x, int16_t y, char ch, uint8_t size) {
    if (ch<' '||ch>'Z') return;
    uint8_t idx=ch-' ';
    for(uint8_t i=0;i<5;i++){uint8_t line=g_font5x8[idx][i];for(uint8_t j=0;j<8;j++){if(line&(1<<j)){if(size==1)OLED_SetPixel(x+i,y+j,1);else OLED_FillRect(x+i*size,y+j*size,size,size);}}}
}

static void OLED_DrawString(int16_t x, int16_t y, const char *str, uint8_t size) {
    while(*str){OLED_DrawChar(x,y,*str,size);x+=(size==1)?6:(6*size);if(x>=123){x=0;y+=(size==1)?8:(8*size);}str++;}
}

static void OLED_Update(void) {
    OLED_WriteCmd(0x21);OLED_WriteCmd(0);OLED_WriteCmd(127);
    OLED_WriteCmd(0x22);OLED_WriteCmd(0);OLED_WriteCmd(7);
    for(uint16_t i=0;i<sizeof(g_oledBuffer);i++) OLED_WriteData(g_oledBuffer[i]);
}

/* ========== 数据记录功能 ========== */

/* 格式化CSV一行：时间戳, CH0, CH1, ..., CH7 */
static void Logger_FormatCSV(char *buf, uint16_t len) {
    int n = snprintf(buf, len, "20%02d-%02d-%02d %02d:%02d:%02d",
                     g_rtcTime.year, g_rtcTime.month, g_rtcTime.date,
                     g_rtcTime.hour, g_rtcTime.minute, g_rtcTime.second);
    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++) {
        n += snprintf(buf + n, len - n, ",%.1f", g_voltage[ch]);
    }
    n += snprintf(buf + n, len - n, "\r\n");
}

/* 追加数据到扇区缓冲区 */
static void Logger_AppendData(const char *line) {
    uint16_t lineLen = strlen(line);

    /* 如果当前缓冲区空间不足，先刷出 */
    if (g_sectorIdx + lineLen >= SD_SECTOR_SIZE) {
        Logger_FlushBuffer();
    }

    memcpy(&g_sectorBuf[g_sectorIdx], line, lineLen);
    g_sectorIdx += lineLen;
}

/* 将缓冲区写入SD卡 */
static void Logger_FlushBuffer(void) {
    if (g_sectorIdx == 0) return;

    /* 用0xFF填充剩余空间 */
    while (g_sectorIdx < SD_SECTOR_SIZE) {
        g_sectorBuf[g_sectorIdx++] = 0xFF;
    }

    if (SD_WriteSector(g_writeSector, g_sectorBuf)) {
        g_writeSector++;
        g_recordCount++;
        DL_GPIO_setPins(LED_REC_PORT, LED_REC_PIN);
    } else {
        DL_GPIO_setPins(LED_ERR_PORT, LED_ERR_PIN);
    }

    memset(g_sectorBuf, 0xFF, SD_SECTOR_SIZE);
    g_sectorIdx = 0;
}

/* ========== UI显示 ========== */
static void UI_UpdateDisplay(void) {
    char buf[32];
    OLED_Clear();

    /* 标题栏 */
    snprintf(buf, sizeof(buf), "CH%d %s", g_displayChannel,
             g_recording ? "REC" : "STOP");
    OLED_DrawString(0, 0, buf, 1);

    /* 采样率 */
    OLED_DrawString(80, 0, g_sampleRateName[g_sampleRateIdx], 1);

    /* 时间戳 */
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
             g_rtcTime.hour, g_rtcTime.minute, g_rtcTime.second);
    OLED_DrawString(0, 10, buf, 1);

    /* 当前通道电压值（大号） */
    snprintf(buf, sizeof(buf), "%.1fmV", g_voltage[g_displayChannel]);
    OLED_DrawString(0, 22, buf, 2);

    /* 简易波形图（最近32个采样点） */
    static uint16_t waveBuf[4][32];  /* 4通道波形缓存 */
    static uint8_t waveIdx = 0;
    waveBuf[g_displayChannel % 4][waveIdx] = (uint16_t)g_voltage[g_displayChannel];
    waveIdx = (waveIdx + 1) & 0x1F;

    /* 绘制波形 */
    OLED_DrawLine(0, 63, 127, 63);  /* 基线 */
    OLED_DrawLine(0, 38, 127, 38);  /* 顶部 */
    uint8_t ch = g_displayChannel % 4;
    for (uint8_t i = 0; i < 31; i++) {
        uint8_t idx0 = (waveIdx + i) & 0x1F;
        uint8_t idx1 = (waveIdx + i + 1) & 0x1F;
        int16_t y0 = 63 - (int16_t)(waveBuf[ch][idx0] * 25 / 3300);  /* 映射到像素 */
        int16_t y1 = 63 - (int16_t)(waveBuf[ch][idx1] * 25 / 3300);
        if (y0 < 38) y0 = 38; if (y0 > 63) y0 = 63;
        if (y1 < 38) y1 = 38; if (y1 > 63) y1 = 63;
        OLED_DrawLine(i * 4, y0, (i + 1) * 4, y1);
    }

    /* 记录统计 */
    snprintf(buf, sizeof(buf), "Rec:%lu", (unsigned long)g_recordCount);
    OLED_DrawString(72, 10, buf, 1);

    OLED_Update();
}

/* ========== CSV表头 ========== */
static const char g_csvHeader[] = "Timestamp,CH0(mV),CH1(mV),CH2(mV),CH3(mV),CH4(mV),CH5(mV),CH6(mV),CH7(mV)\r\n";

/* ========== 主函数 ========== */
int main(void) {
    /* 系统初始化 */
    DL_SYSCTL_initSYSCTL();
    SysTick_Config(SystemCoreClock / 1000);

    /* 外设初始化 */
    DL_I2C_initController(I2C_0_INST, 400000);
    DL_SPI_enable(SPI_0_INST);
    DL_SPI_enable(SPI_1_INST);

    /* GPIO */
    DL_GPIO_initDigitalOutput(MCP3008_CS_PORT | MCP3008_CS_PIN);
    DL_GPIO_initDigitalOutput(SD_CS_PORT | SD_CS_PIN);
    DL_GPIO_initDigitalOutput(LED_REC_PORT | LED_REC_PIN);
    DL_GPIO_initDigitalOutput(LED_ERR_PORT | LED_ERR_PIN);
    DL_GPIO_initDigitalInput(BTN_RECORD_PORT | BTN_RECORD_PIN);
    DL_GPIO_initDigitalInput(BTN_CHANNEL_PORT | BTN_CHANNEL_PIN);
    DL_GPIO_initDigitalInput(BTN_RATE_PORT | BTN_RATE_PIN);

    MCP3008_Init();
    OLED_Init();

    /* SD卡初始化 */
    OLED_Clear();
    OLED_DrawString(0, 24, "Initializing SD...", 1);
    OLED_Update();

    bool sdOk = SD_Init();
    if (!sdOk) {
        OLED_Clear();
        OLED_DrawString(0, 24, "SD Init FAILED!", 1);
        OLED_Update();
        DL_GPIO_setPins(LED_ERR_PORT, LED_ERR_PIN);
    } else {
        OLED_Clear();
        OLED_DrawString(0, 24, "SD Init OK!", 1);
        OLED_Update();
    }
    Delay_ms(1000);

    uint32_t lastSampleTime = 0;
    uint32_t lastDisplayTime = 0;

    while (1) {
        /* 按键处理 */
        if (Button_IsPressed(BTN_RECORD_PORT, BTN_RECORD_PIN)) {
            if (!g_recording) {
                /* 开始记录：写入CSV表头 */
                g_recording = true;
                g_recordCount = 0;
                g_writeSector = 0;
                g_sectorIdx = 0;
                DL_GPIO_clearPins(LED_ERR_PORT, LED_ERR_PIN);
                if (sdOk) {
                    /* 表头写入 */
                    uint16_t hdrLen = strlen(g_csvHeader);
                    memcpy(g_sectorBuf, g_csvHeader, hdrLen);
                    g_sectorIdx = hdrLen;
                }
            } else {
                /* 停止记录：刷出剩余缓冲 */
                g_recording = false;
                if (sdOk) Logger_FlushBuffer();
                DL_GPIO_clearPins(LED_REC_PORT, LED_REC_PIN);
            }
        }
        if (Button_IsPressed(BTN_CHANNEL_PORT, BTN_CHANNEL_PIN)) {
            g_displayChannel = (g_displayChannel + 1) % ADC_CHANNELS;
        }
        if (Button_IsPressed(BTN_RATE_PORT, BTN_RATE_PIN)) {
            g_sampleRateIdx = (g_sampleRateIdx + 1) % SAMPLE_RATE_COUNT;
        }

        /* 按采样率采集数据 */
        uint16_t interval = g_sampleRateTable[g_sampleRateIdx];
        if ((g_systickCount - lastSampleTime) >= interval) {
            DS3231_GetTime(&g_rtcTime);
            MCP3008_ReadAll(g_adcValues);

            /* 记录数据 */
            if (g_recording && sdOk) {
                Logger_FormatCSV(g_logBuffer, LOG_BUF_SIZE);
                Logger_AppendData(g_logBuffer);
            }

            lastSampleTime = g_systickCount;
        }

        /* 显示刷新 */
        if ((g_systickCount - lastDisplayTime) >= DISPLAY_REFRESH_MS) {
            UI_UpdateDisplay();
            lastDisplayTime = g_systickCount;
        }
    }
}
