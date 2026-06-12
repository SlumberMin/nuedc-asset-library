/**
 * @file rtc_alarm_clock.c
 * @brief MSPM0G3507 RTC闹钟时钟示例（DS3231 + OLED + 双闹钟 + 温度显示）
 *
 * 硬件连接：
 *   DS3231 RTC模块（I2C）：
 *     SCL -> PB2 (I2C0_SCL)
 *     SDA -> PB3 (I2C0_SDA)
 *     INT/SQW -> PA7 (中断输入，闹钟触发)
 *
 *   OLED显示屏（I2C地址0x3C，128x64）：
 *     SCL -> PB2 (与DS3231共用I2C总线)
 *     SDA -> PB3
 *
 *   按键（接地有效，内部上拉）：
 *     设置键   -> PA11
 *     增加键   -> PA12
 *     减少键   -> PA13
 *     模式键   -> PA14
 *
 *   蜂鸣器：
 *     Buzzer -> PA15 (PWM输出)
 *
 * 功能：
 *   - DS3231精确RTC时钟，显示年-月-日 时:分:秒
 *   - 双闹钟设置（闹钟A/闹钟B），闹钟触发蜂鸣器响铃
 *   - DS3231内置温度传感器读取并显示
 *   - OLED菜单式界面，按键切换设置项
 *   - 掉电后时间保持（DS3231自带电池）
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ========== DS3231 I2C配置 ========== */
#define DS3231_I2C_INST         I2C_0_INST
#define DS3231_I2C_ADDR         0x68  /* DS3231 I2C地址 */
#define DS3231_I2C_TIMEOUT      1000

/* ========== OLED I2C配置 ========== */
#define OLED_I2C_ADDR           0x3C  /* SSD1306 OLED地址 */
#define OLED_WIDTH              128
#define OLED_HEIGHT             64

/* ========== 按键引脚定义 ========== */
#define BTN_SET_PORT            GPIOA
#define BTN_SET_PIN             DL_GPIO_PIN_11
#define BTN_UP_PORT             GPIOA
#define BTN_UP_PIN              DL_GPIO_PIN_12
#define BTN_DOWN_PORT           GPIOA
#define BTN_DOWN_PIN            DL_GPIO_PIN_13
#define BTN_MODE_PORT           GPIOA
#define BTN_MODE_PIN            DL_GPIO_PIN_14

/* ========== 蜂鸣器引脚定义 ========== */
#define BUZZER_PORT             GPIOA
#define BUZZER_PIN              DL_GPIO_PIN_15

/* ========== DS3231寄存器地址 ========== */
#define DS3231_REG_SEC          0x00
#define DS3231_REG_MIN          0x01
#define DS3231_REG_HOUR         0x02
#define DS3231_REG_DAY          0x03
#define DS3231_REG_DATE         0x04
#define DS3231_REG_MONTH        0x05
#define DS3231_REG_YEAR         0x06
#define DS3231_REG_ALM1_SEC     0x07
#define DS3231_REG_ALM1_MIN     0x08
#define DS3231_REG_ALM1_HOUR    0x09
#define DS3231_REG_ALM1_DAY     0x0A
#define DS3231_REG_ALM2_MIN     0x0B
#define DS3231_REG_ALM2_HOUR    0x0C
#define DS3231_REG_ALM2_DAY     0x0D
#define DS3231_REG_CONTROL      0x0E
#define DS3231_REG_STATUS       0x0F
#define DS3231_REG_TEMP_MSB     0x11
#define DS3231_REG_TEMP_LSB     0x12

/* ========== OLED命令 ========== */
#define OLED_CMD                0x00
#define OLED_DATA               0x40

/* ========== 日期时间结构体 ========== */
typedef struct {
    uint8_t year;       /* 00-99 */
    uint8_t month;      /* 01-12 */
    uint8_t date;       /* 01-31 */
    uint8_t day;        /* 1-7 (星期几) */
    uint8_t hour;       /* 00-23 */
    uint8_t minute;     /* 00-59 */
    uint8_t second;     /* 00-59 */
} RTC_Time_t;

/* ========== 闹钟结构体 ========== */
typedef struct {
    uint8_t hour;
    uint8_t minute;
    bool    enabled;    /* 闹钟是否使能 */
    bool    ringing;    /* 闹钟正在响铃 */
} Alarm_t;

/* ========== 全局变量 ========== */
static RTC_Time_t g_rtcTime;                /* 当前时间 */
static Alarm_t    g_alarmA = {7, 0, true, false};   /* 闹钟A：默认7:00 */
static Alarm_t    g_alarmB = {22, 0, false, false};  /* 闹钟B：默认22:00 */
static float      g_temperature = 0.0f;     /* DS3231温度 */
static uint8_t    g_uiMode = 0;             /* UI模式：0=主界面,1-7=设置 */
static uint8_t    g_settingField = 0;       /* 当前设置字段 */
static bool       g_updateDisplay = true;   /* 显示更新标志 */
static uint8_t    g_displayPage = 0;        /* 显示页面：0=时间，1=闹钟，2=温度 */
static uint32_t   g_buzzerCounter = 0;      /* 蜂鸣器计数 */

/* 128x64 OLED显存缓冲区 */
static uint8_t g_oledBuffer[OLED_WIDTH * OLED_HEIGHT / 8];

/* ========== 基本字体 5x8 ASCII ========== */
static const uint8_t g_font5x8[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 空格 */
    {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
    {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x36,0x49,0x55,0x22,0x50}, /* & */
    {0x00,0x05,0x03,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08}, /* * */
    {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x50,0x30,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x60,0x60,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41}, /* < */
    {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x41,0x22,0x14,0x08,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x01,0x01}, /* F */
    {0x3E,0x41,0x41,0x51,0x32}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x04,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x03,0x04,0x78,0x04,0x03}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
};

/* ========== 函数声明 ========== */
static void     I2C_WriteReg(uint8_t devAddr, uint8_t reg, uint8_t data);
static uint8_t  I2C_ReadReg(uint8_t devAddr, uint8_t reg);
static void     I2C_ReadMulti(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint8_t len);
static uint8_t  BCD2Bin(uint8_t bcd);
static uint8_t  Bin2BCD(uint8_t bin);

/* DS3231驱动 */
static void     DS3231_Init(void);
static void     DS3231_GetTime(RTC_Time_t *time);
static void     DS3231_SetTime(const RTC_Time_t *time);
static void     DS3231_SetAlarmA(uint8_t hour, uint8_t minute, uint8_t second);
static void     DS3231_SetAlarmB(uint8_t hour, uint8_t minute);
static void     DS3231_EnableAlarmA(bool enable);
static void     DS3231_EnableAlarmB(bool enable);
static void     DS3231_ClearAlarmFlag(uint8_t flag);
static float    DS3231_GetTemperature(void);

/* OLED驱动 */
static void     OLED_WriteCmd(uint8_t cmd);
static void     OLED_WriteData(uint8_t data);
static void     OLED_Init(void);
static void     OLED_Clear(void);
static void     OLED_SetPixel(int16_t x, int16_t y, uint8_t color);
static void     OLED_DrawChar(int16_t x, int16_t y, char ch, uint8_t size);
static void     OLED_DrawString(int16_t x, int16_t y, const char *str, uint8_t size);
static void     OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
static void     OLED_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h);
static void     OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h);
static void     OLED_DrawBitmap(int16_t x, int16_t y, const uint8_t *bmp, int16_t w, int16_t h);
static void     OLED_Update(void);

/* UI功能 */
static void     UI_DrawMainPage(void);
static void     UI_DrawAlarmPage(void);
static void     UI_DrawTempPage(void);
static void     UI_DrawSettingField(int16_t x, int16_t y, const char *label, uint8_t val, bool active);
static void     Buzzer_Beep(uint32_t duration_ms);
static bool     Button_IsPressed(GPIO_Regs *port, uint32_t pin);
static void     Delay_ms(uint32_t ms);

/* ========== BCD转换 ========== */
static uint8_t BCD2Bin(uint8_t bcd) {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static uint8_t Bin2BCD(uint8_t bin) {
    return ((bin / 10) << 4) | (bin % 10);
}

/* ========== 延时函数 ========== */
static volatile uint32_t g_systickCount = 0;

void SysTick_Handler(void) {
    g_systickCount++;
}

static void Delay_ms(uint32_t ms) {
    uint32_t start = g_systickCount;
    while ((g_systickCount - start) < ms);
}

/* ========== I2C底层操作 ========== */
static void I2C_WriteReg(uint8_t devAddr, uint8_t reg, uint8_t data) {
    DL_I2C_fillControllerTXFIFO(DS3231_I2C_INST, &reg, 1);
    while (!(DL_I2C_getControllerStatus(DS3231_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(DS3231_I2C_INST, devAddr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(DS3231_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_fillControllerTXFIFO(DS3231_I2C_INST, &data, 1);
    while (!(DL_I2C_getControllerStatus(DS3231_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
}

static uint8_t I2C_ReadReg(uint8_t devAddr, uint8_t reg) {
    uint8_t data;
    DL_I2C_fillControllerTXFIFO(DS3231_I2C_INST, &reg, 1);
    DL_I2C_startControllerTransfer(DS3231_I2C_INST, devAddr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(DS3231_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_startControllerTransfer(DS3231_I2C_INST, devAddr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(DS3231_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    data = DL_I2C_receiveControllerData(DS3231_I2C_INST);
    return data;
}

static void I2C_ReadMulti(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint8_t len) {
    DL_I2C_fillControllerTXFIFO(DS3231_I2C_INST, &reg, 1);
    DL_I2C_startControllerTransfer(DS3231_I2C_INST, devAddr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(DS3231_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_startControllerTransfer(DS3231_I2C_INST, devAddr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);
    while (DL_I2C_getControllerStatus(DS3231_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = DL_I2C_receiveControllerData(DS3231_I2C_INST);
    }
}

/* ========== DS3231驱动实现 ========== */
static void DS3231_Init(void) {
    /* 控制寄存器：使能闹钟中断，关闭方波输出 */
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_CONTROL, 0x04);  /* INTCN=1, ALM中断模式 */
    /* 清除所有闹钟标志 */
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_STATUS, 0x00);
}

static void DS3231_GetTime(RTC_Time_t *time) {
    uint8_t buf[7];
    I2C_ReadMulti(DS3231_I2C_ADDR, DS3231_REG_SEC, buf, 7);
    time->second = BCD2Bin(buf[0] & 0x7F);
    time->minute = BCD2Bin(buf[1] & 0x7F);
    /* 24小时制 */
    time->hour   = BCD2Bin(buf[2] & 0x3F);
    time->day    = BCD2Bin(buf[3] & 0x07);
    time->date   = BCD2Bin(buf[4] & 0x3F);
    time->month  = BCD2Bin(buf[5] & 0x1F);
    time->year   = BCD2Bin(buf[6]);
}

static void DS3231_SetTime(const RTC_Time_t *time) {
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_SEC,  Bin2BCD(time->second));
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_MIN,  Bin2BCD(time->minute));
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_HOUR, Bin2BCD(time->hour));  /* 24h */
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_DAY,  Bin2BCD(time->day));
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_DATE, Bin2BCD(time->date));
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_MONTH,Bin2BCD(time->month));
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_YEAR, Bin2BCD(time->year));
}

/* 闹钟A：时:分:秒匹配模式（A1M1-A4=0100） */
static void DS3231_SetAlarmA(uint8_t hour, uint8_t minute, uint8_t second) {
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_ALM1_SEC,  Bin2BCD(second));  /* A1M1=0 */
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_ALM1_MIN,  Bin2BCD(minute));  /* A1M2=0 */
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_ALM1_HOUR, Bin2BCD(hour));    /* A1M3=0 */
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_ALM1_DAY,  0x80);             /* A1M4=1, DY/DT=0 */
}

/* 闹钟B：时:分匹配模式（A2M2-A4=100） */
static void DS3231_SetAlarmB(uint8_t hour, uint8_t minute) {
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_ALM2_MIN,  Bin2BCD(minute));  /* A2M2=0 */
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_ALM2_HOUR, Bin2BCD(hour));    /* A2M3=0 */
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_ALM2_DAY,  0x80);             /* A2M4=1 */
}

static void DS3231_EnableAlarmA(bool enable) {
    uint8_t ctrl = I2C_ReadReg(DS3231_I2C_ADDR, DS3231_REG_CONTROL);
    if (enable) {
        ctrl |= (1 << 0);   /* A1IE=1 */
    } else {
        ctrl &= ~(1 << 0);  /* A1IE=0 */
    }
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_CONTROL, ctrl);
}

static void DS3231_EnableAlarmB(bool enable) {
    uint8_t ctrl = I2C_ReadReg(DS3231_I2C_ADDR, DS3231_REG_CONTROL);
    if (enable) {
        ctrl |= (1 << 1);   /* A2IE=1 */
    } else {
        ctrl &= ~(1 << 1);  /* A2IE=0 */
    }
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_CONTROL, ctrl);
}

static void DS3231_ClearAlarmFlag(uint8_t flag) {
    uint8_t status = I2C_ReadReg(DS3231_I2C_ADDR, DS3231_REG_STATUS);
    status &= ~flag;  /* 清除A1F/A2F标志位 */
    I2C_WriteReg(DS3231_I2C_ADDR, DS3231_REG_STATUS, status);
}

static float DS3231_GetTemperature(void) {
    uint8_t msb = I2C_ReadReg(DS3231_I2C_ADDR, DS3231_REG_TEMP_MSB);
    uint8_t lsb = I2C_ReadReg(DS3231_I2C_ADDR, DS3231_REG_TEMP_LSB);
    /* 高8位为整数（含符号），高2位低6位为小数（0.25°C精度） */
    float temp = (float)(int8_t)msb;
    temp += (float)((lsb >> 6) * 25) / 100.0f;
    return temp;
}

/* ========== OLED驱动实现 ========== */
static void OLED_WriteCmd(uint8_t cmd) {
    uint8_t buf[2] = {OLED_CMD, cmd};
    DL_I2C_fillControllerTXFIFO(DS3231_I2C_INST, buf, 2);
    DL_I2C_startControllerTransfer(DS3231_I2C_INST, OLED_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(DS3231_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static void OLED_WriteData(uint8_t data) {
    uint8_t buf[2] = {OLED_DATA, data};
    DL_I2C_fillControllerTXFIFO(DS3231_I2C_INST, buf, 2);
    DL_I2C_startControllerTransfer(DS3231_I2C_INST, OLED_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(DS3231_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static void OLED_Init(void) {
    Delay_ms(100);
    OLED_WriteCmd(0xAE);  /* 关闭显示 */
    OLED_WriteCmd(0xD5);  /* 时钟分频 */
    OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8);  /* 多路复用 */
    OLED_WriteCmd(0x3F);  /* 1/64 duty */
    OLED_WriteCmd(0xD3);  /* 显示偏移 */
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x40);  /* 起始行=0 */
    OLED_WriteCmd(0x8D);  /* 电荷泵 */
    OLED_WriteCmd(0x14);  /* 使能 */
    OLED_WriteCmd(0x20);  /* 寻址模式 */
    OLED_WriteCmd(0x00);  /* 水平寻址 */
    OLED_WriteCmd(0xA1);  /* 段重映射 */
    OLED_WriteCmd(0xC8);  /* COM扫描方向 */
    OLED_WriteCmd(0xDA);  /* COM引脚配置 */
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0x81);  /* 对比度 */
    OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xD9);  /* 预充电 */
    OLED_WriteCmd(0xF1);
    OLED_WriteCmd(0xDB);  /* VCOMH */
    OLED_WriteCmd(0x30);
    OLED_WriteCmd(0xA4);  /* 全局显示关闭 */
    OLED_WriteCmd(0xA6);  /* 正常显示（非反色） */
    OLED_WriteCmd(0xAF);  /* 开启显示 */
    OLED_Clear();
    OLED_Update();
}

static void OLED_Clear(void) {
    memset(g_oledBuffer, 0, sizeof(g_oledBuffer));
}

static void OLED_SetPixel(int16_t x, int16_t y, uint8_t color) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    uint16_t idx = (y / 8) * OLED_WIDTH + x;
    uint8_t  bit = (1 << (y & 7));
    if (color) {
        g_oledBuffer[idx] |= bit;
    } else {
        g_oledBuffer[idx] &= ~bit;
    }
}

static void OLED_DrawChar(int16_t x, int16_t y, char ch, uint8_t size) {
    if (ch < ' ' || ch > 'Z') return;
    uint8_t idx = ch - ' ';
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = g_font5x8[idx][i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                if (size == 1) {
                    OLED_SetPixel(x + i, y + j, 1);
                } else {
                    OLED_FillRect(x + i * size, y + j * size, size, size);
                }
            }
        }
    }
}

static void OLED_DrawString(int16_t x, int16_t y, const char *str, uint8_t size) {
    while (*str) {
        OLED_DrawChar(x, y, *str, size);
        x += (size == 1) ? 6 : (6 * size);
        if (x >= OLED_WIDTH - 5) {
            x = 0;
            y += (size == 1) ? 8 : (8 * size);
        }
        str++;
    }
}

static void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    int16_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int16_t dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    while (1) {
        OLED_SetPixel(x0, y0, 1);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

static void OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h) {
    for (int16_t i = x; i < x + w; i++) {
        for (int16_t j = y; j < y + h; j++) {
            OLED_SetPixel(i, j, 1);
        }
    }
}

static void OLED_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h) {
    OLED_DrawLine(x, y, x + w - 1, y);
    OLED_DrawLine(x + w - 1, y, x + w - 1, y + h - 1);
    OLED_DrawLine(x + w - 1, y + h - 1, x, y + h - 1);
    OLED_DrawLine(x, y + h - 1, x, y);
}

static void OLED_Update(void) {
    OLED_WriteCmd(0x21); OLED_WriteCmd(0); OLED_WriteCmd(127); /* 列范围 */
    OLED_WriteCmd(0x22); OLED_WriteCmd(0); OLED_WriteCmd(7);   /* 页范围 */
    for (uint16_t i = 0; i < sizeof(g_oledBuffer); i++) {
        OLED_WriteData(g_oledBuffer[i]);
    }
}

/* ========== 蜂鸣器控制 ========== */
static void Buzzer_Beep(uint32_t duration_ms) {
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
    Delay_ms(duration_ms);
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}

/* ========== 按键检测（带消抖） ========== */
static bool Button_IsPressed(GPIO_Regs *port, uint32_t pin) {
    if (DL_GPIO_readPins(port, pin) == 0) {
        Delay_ms(20);
        if (DL_GPIO_readPins(port, pin) == 0) {
            while (DL_GPIO_readPins(port, pin) == 0);  /* 等待释放 */
            return true;
        }
    }
    return false;
}

/* ========== UI页面绘制 ========== */

/* 主页面：时间和日期 */
static void UI_DrawMainPage(void) {
    char buf[32];
    OLED_Clear();

    /* 大号时间显示 HH:MM:SS */
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
             g_rtcTime.hour, g_rtcTime.minute, g_rtcTime.second);
    OLED_DrawString(16, 4, buf, 2);

    /* 日期显示 YYYY-MM-DD */
    snprintf(buf, sizeof(buf), "20%02d-%02d-%02d",
             g_rtcTime.year, g_rtcTime.month, g_rtcTime.date);
    OLED_DrawString(14, 28, buf, 1);

    /* 星期显示 */
    static const char *weekdays[] = {"", "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"};
    if (g_rtcTime.day >= 1 && g_rtcTime.day <= 7) {
        OLED_DrawString(80, 28, weekdays[g_rtcTime.day], 1);
    }

    /* 闹钟状态指示 */
    OLED_DrawString(0, 44, "A:", 1);
    if (g_alarmA.enabled) {
        snprintf(buf, sizeof(buf), "%02d:%02d", g_alarmA.hour, g_alarmA.minute);
        OLED_DrawString(12, 44, buf, 1);
    } else {
        OLED_DrawString(12, 44, "OFF", 1);
    }
    OLED_DrawString(64, 44, "B:", 1);
    if (g_alarmB.enabled) {
        snprintf(buf, sizeof(buf), "%02d:%02d", g_alarmB.hour, g_alarmB.minute);
        OLED_DrawString(76, 44, buf, 1);
    } else {
        OLED_DrawString(76, 44, "OFF", 1);
    }

    /* 温度显示 */
    snprintf(buf, sizeof(buf), "T:%.1fC", g_temperature);
    OLED_DrawString(0, 56, buf, 1);

    /* 闹钟响铃动画 */
    if (g_alarmA.ringing || g_alarmB.ringing) {
        static uint8_t blink = 0;
        blink = !blink;
        if (blink) {
            OLED_FillRect(100, 0, 28, 12);
        }
        OLED_DrawString(104, 2, "AL!", 1);
    }

    OLED_Update();
}

/* 闹钟设置页面 */
static void UI_DrawAlarmPage(void) {
    char buf[32];
    OLED_Clear();

    OLED_DrawString(20, 0, "ALARM SETTINGS", 1);
    OLED_DrawLine(0, 10, 127, 10);

    /* 闹钟A */
    OLED_DrawString(0, 16, "Alarm A:", 1);
    snprintf(buf, sizeof(buf), "%02d:%02d", g_alarmA.hour, g_alarmA.minute);
    OLED_DrawString(70, 16, buf, 1);
    OLED_DrawString(104, 16, g_alarmA.enabled ? "ON" : "OFF", 1);

    /* 闹钟B */
    OLED_DrawString(0, 30, "Alarm B:", 1);
    snprintf(buf, sizeof(buf), "%02d:%02d", g_alarmB.hour, g_alarmB.minute);
    OLED_DrawString(70, 30, buf, 1);
    OLED_DrawString(104, 30, g_alarmB.enabled ? "ON" : "OFF", 1);

    /* 设置模式高亮 */
    if (g_uiMode >= 1 && g_uiMode <= 5) {
        int16_t selY = (g_settingField < 2) ? 16 : 30;
        OLED_DrawRect(0, selY - 2, 126, 14);
    }

    OLED_DrawString(0, 50, "MODE:Switch SET:Edit", 1);
    OLED_Update();
}

/* 温度详细页面 */
static void UI_DrawTempPage(void) {
    char buf[32];
    OLED_Clear();

    OLED_DrawString(16, 0, "TEMPERATURE", 1);
    OLED_DrawLine(0, 10, 127, 10);

    /* 大号温度显示 */
    snprintf(buf, sizeof(buf), "%.2f", g_temperature);
    OLED_DrawString(16, 20, buf, 2);
    OLED_DrawString(96, 20, "C", 2);

    /* 温度条形图 */
    int16_t barLen = (int16_t)((g_temperature + 10.0f) * 2.0f);  /* -10~50°C映射到0~120 */
    if (barLen < 0) barLen = 0;
    if (barLen > 120) barLen = 120;
    OLED_DrawRect(4, 44, 121, 10);
    OLED_FillRect(5, 45, barLen, 8);

    snprintf(buf, sizeof(buf), "-10C      50C");
    OLED_DrawString(4, 56, buf, 1);

    OLED_Update();
}

/* 设置模式的字段高亮显示 */
static void UI_DrawSettingField(int16_t x, int16_t y, const char *label, uint8_t val, bool active) {
    char buf[8];
    OLED_DrawString(x, y, label, 1);
    snprintf(buf, sizeof(buf), "%02d", val);
    OLED_DrawString(x + 40, y, buf, 1);
    if (active) {
        OLED_DrawRect(x + 38, y - 2, 16, 12);
    }
}

/* ========== 主函数 ========== */
int main(void) {
    /* 系统初始化 */
    DL_SYSCTL_initSYSCTL();
    SysTick_Config(SystemCoreClock / 1000);  /* 1ms系统节拍 */

    /* I2C外设初始化 */
    DL_I2C_initController(DS3231_I2C_INST, 400000);  /* 400kHz快速模式 */

    /* GPIO初始化 */
    DL_GPIO_initDigitalInput(BTN_SET_PORT  | BTN_SET_PIN);
    DL_GPIO_initDigitalInput(BTN_UP_PORT   | BTN_UP_PIN);
    DL_GPIO_initDigitalInput(BTN_DOWN_PORT | BTN_DOWN_PIN);
    DL_GPIO_initDigitalInput(BTN_MODE_PORT | BTN_MODE_PIN);
    DL_GPIO_initDigitalOutput(BUZZER_PORT  | BUZZER_PIN);

    /* DS3231和OLED初始化 */
    DS3231_Init();
    OLED_Init();

    /* 开机动画 */
    OLED_Clear();
    OLED_DrawString(12, 24, "RTC Alarm Clock", 1);
    OLED_DrawString(16, 40, "DS3231 + OLED", 1);
    OLED_Update();
    Delay_ms(1500);

    /* 设置初始闹钟 */
    if (g_alarmA.enabled) DS3231_SetAlarmA(g_alarmA.hour, g_alarmA.minute, 0);
    if (g_alarmB.enabled) DS3231_SetAlarmB(g_alarmB.hour, g_alarmB.minute);
    DS3231_EnableAlarmA(g_alarmA.enabled);
    DS3231_EnableAlarmB(g_alarmB.enabled);

    uint32_t lastTempRead = 0;

    while (1) {
        /* 读取当前时间 */
        DS3231_GetTime(&g_rtcTime);

        /* 每5秒读取一次温度（DS3231温度转换周期约64秒，但寄存器缓存值可随时读） */
        if ((g_systickCount - lastTempRead) > 5000) {
            g_temperature = DS3231_GetTemperature();
            lastTempRead = g_systickCount;
        }

        /* 检查闹钟中断标志 */
        uint8_t status = I2C_ReadReg(DS3231_I2C_ADDR, DS3231_REG_STATUS);
        if (status & 0x01) {  /* A1F - 闹钟A触发 */
            g_alarmA.ringing = true;
            DS3231_ClearAlarmFlag(0x01);
        }
        if (status & 0x02) {  /* A2F - 闹钟B触发 */
            g_alarmB.ringing = true;
            DS3231_ClearAlarmFlag(0x02);
        }

        /* 闹钟响铃处理 */
        if (g_alarmA.ringing || g_alarmB.ringing) {
            /* 任意按键关闭闹钟 */
            if (Button_IsPressed(BTN_SET_PORT, BTN_SET_PIN) ||
                Button_IsPressed(BTN_UP_PORT, BTN_UP_PIN) ||
                Button_IsPressed(BTN_DOWN_PORT, BTN_DOWN_PIN) ||
                Button_IsPressed(BTN_MODE_PORT, BTN_MODE_PIN)) {
                g_alarmA.ringing = false;
                g_alarmB.ringing = false;
            } else {
                Buzzer_Beep(100);
                Delay_ms(100);
            }
        }

        /* 主界面模式：按键切换页面 */
        if (g_uiMode == 0) {
            /* 模式键切换显示页面 */
            if (Button_IsPressed(BTN_MODE_PORT, BTN_MODE_PIN)) {
                g_displayPage = (g_displayPage + 1) % 3;
            }
            /* 设置键进入设置模式 */
            if (Button_IsPressed(BTN_SET_PORT, BTN_SET_PIN)) {
                g_uiMode = 1;
                g_settingField = 0;
            }

            /* 绘制当前页面 */
            switch (g_displayPage) {
                case 0: UI_DrawMainPage();   break;
                case 1: UI_DrawAlarmPage();  break;
                case 2: UI_DrawTempPage();   break;
            }
        }
        /* 设置模式 */
        else {
            /* 模式键切换设置项 */
            if (Button_IsPressed(BTN_MODE_PORT, BTN_MODE_PIN)) {
                g_settingField++;
                if (g_settingField > 7) {
                    g_uiMode = 0;  /* 退出设置模式 */
                }
            }
            /* 增加/减少当前值 */
            if (Button_IsPressed(BTN_UP_PORT, BTN_UP_PIN)) {
                switch (g_settingField) {
                    case 0: g_rtcTime.hour   = (g_rtcTime.hour + 1) % 24; break;
                    case 1: g_rtcTime.minute = (g_rtcTime.minute + 1) % 60; break;
                    case 2: g_rtcTime.second = 0; break;
                    case 3: g_rtcTime.year   = (g_rtcTime.year + 1) % 100; break;
                    case 4: g_rtcTime.month  = (g_rtcTime.month % 12) + 1; break;
                    case 5: g_rtcTime.date   = (g_rtcTime.date % 31) + 1; break;
                    case 6: g_alarmA.hour = (g_alarmA.hour + 1) % 24; break;
                    case 7: g_alarmA.minute = (g_alarmA.minute + 1) % 60; break;
                }
            }
            if (Button_IsPressed(BTN_DOWN_PORT, BTN_DOWN_PIN)) {
                switch (g_settingField) {
                    case 0: g_rtcTime.hour   = (g_rtcTime.hour + 23) % 24; break;
                    case 1: g_rtcTime.minute = (g_rtcTime.minute + 59) % 60; break;
                    case 2: g_rtcTime.second = 0; break;
                    case 3: g_rtcTime.year   = (g_rtcTime.year + 99) % 100; break;
                    case 4: g_rtcTime.month  = ((g_rtcTime.month + 10) % 12) + 1; break;
                    case 5: g_rtcTime.date   = ((g_rtcTime.date + 29) % 31) + 1; break;
                    case 6: g_alarmA.hour = (g_alarmA.hour + 23) % 24; break;
                    case 7: g_alarmA.minute = (g_alarmA.minute + 59) % 60; break;
                }
            }
            /* 设置键确认并退出 */
            if (Button_IsPressed(BTN_SET_PORT, BTN_SET_PIN)) {
                /* 写入DS3231 */
                DS3231_SetTime(&g_rtcTime);
                DS3231_SetAlarmA(g_alarmA.hour, g_alarmA.minute, 0);
                g_alarmA.enabled = true;
                DS3231_EnableAlarmA(true);
                g_uiMode = 0;
                Buzzer_Beep(50);  /* 确认提示音 */
            }

            /* 绘制设置界面 */
            OLED_Clear();
            OLED_DrawString(8, 0, "TIME SETTING", 1);
            OLED_DrawLine(0, 10, 127, 10);

            UI_DrawSettingField(0, 16,  "Hour:",  g_rtcTime.hour,   g_settingField == 0);
            UI_DrawSettingField(64, 16, "Min:",   g_rtcTime.minute, g_settingField == 1);
            UI_DrawSettingField(0, 28,  "Sec:",   g_rtcTime.second, g_settingField == 2);
            UI_DrawSettingField(64, 28, "Year:",  g_rtcTime.year,   g_settingField == 3);
            UI_DrawSettingField(0, 38,  "Mon:",   g_rtcTime.month,  g_settingField == 4);
            UI_DrawSettingField(64, 38, "Day:",   g_rtcTime.date,   g_settingField == 5);

            OLED_DrawLine(0, 48, 127, 48);
            UI_DrawSettingField(0, 52,  "A_H:",   g_alarmA.hour,    g_settingField == 6);
            UI_DrawSettingField(64, 52, "A_M:",   g_alarmA.minute,  g_settingField == 7);

            OLED_Update();
        }

        Delay_ms(200);  /* 主循环200ms刷新 */
    }
}
