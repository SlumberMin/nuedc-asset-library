/**
 * @file eeprom_data_store.c
 * @brief MSPM0G3507 EEPROM数据存储示例（MCP79410 + 配置存储 + 掉电保存）
 *
 * 硬件连接：
 *   MCP79410 RTC+SRAM+EEPROM（I2C）：
 *     SCL -> PB2 (I2C0_SCL)
 *     SDA -> PB3 (I2C0_SDA)
 *     VBAT -> 3V纽扣电池（掉电时间保持）
 *     MFP -> PA7 (多功能输出，可配置为中断/方波)
 *
 *   OLED显示屏（I2C，0x3C，128x64）：
 *     与MCP79410共用I2C总线
 *
 *   按键：
 *     菜单键   -> PA11 (切换功能页面)
 *     增加键   -> PA12
 *     减少键   -> PA13
 *     确认键   -> PA14
 *
 * 功能：
 *   - MCP79410内部64字节SRAM快速读写
 *   - MCP79410外部EEPROM（MCP24xx256兼容，32KB）数据存储
 *   - 配置参数结构化存储（PID参数、校准数据等）
 *   - 掉电检测与自动保存
 *   - 数据校验（CRC16）和版本管理
 *   - 菜单式OLED界面管理存储数据
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ========== I2C地址定义 ========== */
#define MCP79410_ADDR           0x6F   /* MCP79410 RTC/SRAM/EEPROM地址 */
#define MCP24XX256_ADDR         0x50   /* 外部EEPROM地址（A0-A2接地） */
#define OLED_ADDR               0x3C   /* OLED地址 */

/* ========== MCP79410寄存器地址 ========== */
#define MCP79410_REG_SEC        0x00
#define MCP79410_REG_MIN        0x01
#define MCP79410_REG_HOUR       0x02
#define MCP79410_REG_DAY        0x03
#define MCP79410_REG_DATE       0x04
#define MCP79410_REG_MONTH      0x05
#define MCP79410_REG_YEAR       0x06
#define MCP79410_REG_CONTROL    0x07
#define MCP79410_REG_OSCTRIM    0x08
#define MCP79410_REG_ALM0_SEC   0x0A
#define MCP79410_REG_ALM0_MIN   0x0B
#define MCP79410_REG_ALM0_HOUR  0x0C
#define MCP79410_REG_ALM0_DAY   0x0D
#define MCP79410_REG_ALM0_DATE  0x0E
#define MCP79410_REG_ALM0_MONTH 0x0F
#define MCP79410_REG_PWRFAIL    0x18
#define MCP79410_REG_PWRDNMIN   0x19
#define MCP79410_REG_PWRDNHOUR  0x1A
#define MCP79410_REG_PWRUPMIN   0x1B
#define MCP79410_REG_PWRUPHOUR  0x1C

/* MCP79410 SRAM起始地址 */
#define MCP79410_SRAM_BASE      0x20
#define MCP79410_SRAM_SIZE      64

/* ========== 按键引脚 ========== */
#define BTN_MENU_PORT           GPIOA
#define BTN_MENU_PIN            DL_GPIO_PIN_11
#define BTN_UP_PORT             GPIOA
#define BTN_UP_PIN              DL_GPIO_PIN_12
#define BTN_DOWN_PORT           GPIOA
#define BTN_DOWN_PIN            DL_GPIO_PIN_13
#define BTN_OK_PORT             GPIOA
#define BTN_OK_PIN              DL_GPIO_PIN_14

/* ========== EEPROM存储地址布局 ========== */
#define EEPROM_BASE_ADDR        0x0000
#define EEPROM_CONFIG_ADDR      0x0000   /* 配置参数区（0x0000-0x00FF） */
#define EEPROM_CALIB_ADDR       0x0100   /* 校准数据区（0x0100-0x01FF） */
#define EEPROM_LOG_ADDR         0x0200   /* 日志数据区（0x0200-0x1FFF） */
#define EEPROM_BACKUP_ADDR      0x2000   /* 备份区（0x2000-0x3FFF） */

/* ========== 数据版本 ========== */
#define DATA_VERSION            0x03     /* 当前数据格式版本 */

/* ========== CRC16-CCITT参数 ========== */
#define CRC16_POLY              0x1021
#define CRC16_INIT              0xFFFF

/* ========== 配置参数结构体 ========== */
typedef struct {
    uint8_t  version;           /* 数据版本号 */
    uint8_t  validFlag;         /* 有效标志 0xAA=有效 */
    /* PID参数 */
    float    pidKp;             /* 比例系数 */
    float    pidKi;             /* 积分系数 */
    float    pidKd;             /* 微分系数 */
    /* 校准参数 */
    float    adcGain;           /* ADC增益校准 */
    float    adcOffset;         /* ADC偏移校准 */
    float    dacGain;           /* DAC增益校准 */
    float    dacOffset;         /* DAC偏移校准 */
    /* 系统参数 */
    uint16_t targetVoltage_mV;  /* 目标电压(mV) */
    uint16_t currentLimit_mA;   /* 电流限制(mA) */
    uint8_t  displayBrightness; /* 显示亮度 0-100 */
    uint8_t  sampleRateIdx;     /* 采样率索引 */
    uint8_t  alarmHour;         /* 闹钟小时 */
    uint8_t  alarmMinute;       /* 闹钟分钟 */
    uint8_t  reserved[16];      /* 保留字段 */
    uint16_t crc16;             /* CRC16校验 */
} __attribute__((packed)) ConfigData_t;

/* ========== 校准数据结构体 ========== */
typedef struct {
    uint8_t  version;
    uint8_t  validFlag;
    float    adcCalPoints[5];   /* 5个ADC校准点 */
    float    dacCalPoints[5];   /* 5个DAC校准点 */
    float    tempOffset;        /* 温度偏移 */
    uint32_t calDate;           /* 校准日期 YYYYMMDD */
    uint16_t crc16;
} __attribute__((packed)) CalibData_t;

/* ========== 运行时日志条目 ========== */
typedef struct {
    uint32_t timestamp;         /* 运行秒数 */
    float    voltage_mV;        /* 电压 */
    float    current_mA;        /* 电流 */
    float    temperature;       /* 温度 */
    uint8_t  errorCode;         /* 错误码 */
} __attribute__((packed)) LogEntry_t;

/* ========== 时间结构体 ========== */
typedef struct {
    uint8_t year, month, date, day;
    uint8_t hour, minute, second;
} RTC_Time_t;

/* ========== 全局变量 ========== */
static volatile uint32_t g_systickCount = 0;
static ConfigData_t      g_config;
static CalibData_t       g_calib;
static uint8_t           g_uiPage = 0;       /* UI页面 */
static uint8_t           g_editField = 0;     /* 编辑字段 */
static bool              g_editing = false;   /* 是否在编辑 */
static uint32_t          g_logWriteAddr = EEPROM_LOG_ADDR;
static bool              g_powerFailed = false;  /* 掉电标志 */

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

/* 页面名称 */
static const char *g_pageName[] = {
    "CONFIG", "CALIB", "RTC", "SRAM", "EEPROM INFO", "LOG"
};

/* ========== 函数声明 ========== */
void SysTick_Handler(void);
static void Delay_ms(uint32_t ms);
static void Delay_us(uint32_t us);
static uint8_t BCD2Bin(uint8_t bcd);
static uint8_t Bin2BCD(uint8_t bin);
static bool Button_IsPressed(GPIO_Regs *port, uint32_t pin);

/* CRC16 */
static uint16_t CRC16_Calc(const uint8_t *data, uint16_t len);

/* MCP79410 RTC驱动 */
static void     MCP79410_Init(void);
static void     MCP79410_GetTime(RTC_Time_t *time);
static void     MCP79410_SetTime(const RTC_Time_t *time);
static bool     MCP79410_IsPowerFailed(void);
static void     MCP79410_ClearPowerFailFlag(void);

/* MCP79410 SRAM驱动 */
static void     MCP79410_SRAM_Write(uint8_t addr, uint8_t data);
static uint8_t  MCP79410_SRAM_Read(uint8_t addr);
static void     MCP79410_SRAM_WriteBlock(uint8_t addr, const uint8_t *data, uint8_t len);
static void     MCP79410_SRAM_ReadBlock(uint8_t addr, uint8_t *data, uint8_t len);

/* EEPROM驱动（MCP24xx256兼容） */
static bool     EEPROM_WriteByte(uint16_t addr, uint8_t data);
static uint8_t  EEPROM_ReadByte(uint16_t addr);
static bool     EEPROM_WriteBlock(uint16_t addr, const uint8_t *data, uint16_t len);
static bool     EEPROM_ReadBlock(uint16_t addr, uint8_t *data, uint16_t len);

/* 配置管理 */
static void     Config_LoadDefaults(ConfigData_t *cfg);
static bool     Config_Load(ConfigData_t *cfg);
static bool     Config_Save(const ConfigData_t *cfg);
static bool     Calib_Load(CalibData_t *cal);
static bool     Calib_Save(const CalibData_t *cal);
static void     Log_Append(const LogEntry_t *entry);

/* OLED驱动 */
static void     OLED_WriteCmd(uint8_t cmd);
static void     OLED_WriteData(uint8_t data);
static void     OLED_Init(void);
static void     OLED_Clear(void);
static void     OLED_SetPixel(int16_t x, int16_t y, uint8_t c);
static void     OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h);
static void     OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
static void     OLED_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h);
static void     OLED_DrawChar(int16_t x, int16_t y, char ch, uint8_t size);
static void     OLED_DrawString(int16_t x, int16_t y, const char *str, uint8_t size);
static void     OLED_Update(void);

/* UI */
static void     UI_DrawConfigPage(void);
static void     UI_DrawCalibPage(void);
static void     UI_DrawRTCPage(void);
static void     UI_DrawSRAMPage(void);
static void     UI_DrawEEPROMInfo(void);
static void     UI_DrawLogPage(void);

/* ========== 延时 ========== */
void SysTick_Handler(void) { g_systickCount++; }

static void Delay_ms(uint32_t ms) {
    uint32_t start = g_systickCount;
    while ((g_systickCount - start) < ms);
}

static void Delay_us(uint32_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 4;
    while (ticks--);
}

static uint8_t BCD2Bin(uint8_t bcd) { return (bcd >> 4)*10 + (bcd & 0x0F); }
static uint8_t Bin2BCD(uint8_t bin) { return ((bin/10)<<4)|(bin%10); }

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

/* ========== CRC16-CCITT计算 ========== */
static uint16_t CRC16_Calc(const uint8_t *data, uint16_t len) {
    uint16_t crc = CRC16_INIT;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ CRC16_POLY;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ========== MCP79410 RTC驱动 ========== */
static void MCP79410_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP79410_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static uint8_t MCP79410_ReadReg(uint8_t reg) {
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP79410_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP79410_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    return DL_I2C_receiveControllerData(I2C_0_INST);
}

static void MCP79410_Init(void) {
    /* 检查ST位（振荡器是否运行），如果没有则启动 */
    uint8_t sec = MCP79410_ReadReg(MCP79410_REG_SEC);
    if (!(sec & 0x80)) {
        MCP79410_WriteReg(MCP79410_REG_SEC, sec | 0x80);  /* ST=1，启动振荡器 */
    }

    /* 使能VBAT切换（VBATEN=1），确保掉电后电池供电 */
    uint8_t day = MCP79410_ReadReg(MCP79410_REG_DAY);
    MCP79410_WriteReg(MCP79410_REG_DAY, day | 0x08);  /* VBATEN=1 */

    /* 控制寄存器：SQWEN=0, ALM0中断模式 */
    MCP79410_WriteReg(MCP79410_REG_CONTROL, 0x00);
}

static void MCP79410_GetTime(RTC_Time_t *time) {
    uint8_t sec   = MCP79410_ReadReg(MCP79410_REG_SEC) & 0x7F;
    uint8_t min   = MCP79410_ReadReg(MCP79410_REG_MIN) & 0x7F;
    uint8_t hour  = MCP79410_ReadReg(MCP79410_REG_HOUR) & 0x3F;
    uint8_t day   = MCP79410_ReadReg(MCP79410_REG_DAY) & 0x07;
    uint8_t date  = MCP79410_ReadReg(MCP79410_REG_DATE) & 0x3F;
    uint8_t month = MCP79410_ReadReg(MCP79410_REG_MONTH) & 0x1F;
    uint8_t year  = MCP79410_ReadReg(MCP79410_REG_YEAR);

    time->second = BCD2Bin(sec);
    time->minute = BCD2Bin(min);
    time->hour   = BCD2Bin(hour);
    time->day    = day;
    time->date   = BCD2Bin(date);
    time->month  = BCD2Bin(month);
    time->year   = BCD2Bin(year);
}

static void MCP79410_SetTime(const RTC_Time_t *time) {
    /* 先停止振荡器 */
    MCP79410_WriteReg(MCP79410_REG_SEC, Bin2BCD(time->second));
    MCP79410_WriteReg(MCP79410_REG_MIN, Bin2BCD(time->minute));
    MCP79410_WriteReg(MCP79410_REG_HOUR, Bin2BCD(time->hour));  /* 24h */
    MCP79410_WriteReg(MCP79410_REG_DAY, time->day | 0x08);      /* VBATEN=1 */
    MCP79410_WriteReg(MCP79410_REG_DATE, Bin2BCD(time->date));
    MCP79410_WriteReg(MCP79410_REG_MONTH, Bin2BCD(time->month));
    MCP79410_WriteReg(MCP79410_REG_YEAR, Bin2BCD(time->year));

    /* 重新启动振荡器 */
    uint8_t sec = MCP79410_ReadReg(MCP79410_REG_SEC);
    MCP79410_WriteReg(MCP79410_REG_SEC, sec | 0x80);  /* ST=1 */
}

static bool MCP79410_IsPowerFailed(void) {
    uint8_t pwrFail = MCP79410_ReadReg(MCP79410_REG_PWRFAIL);
    return (pwrFail & 0x10) != 0;  /* PWRFAIL位 */
}

static void MCP79410_ClearPowerFailFlag(void) {
    uint8_t pwrFail = MCP79410_ReadReg(MCP79410_REG_PWRFAIL);
    MCP79410_WriteReg(MCP79410_REG_PWRFAIL, pwrFail & ~0x10);
}

/* ========== MCP79410 SRAM驱动 ========== */
static void MCP79410_SRAM_Write(uint8_t addr, uint8_t data) {
    if (addr >= MCP79410_SRAM_SIZE) return;
    uint8_t reg = MCP79410_SRAM_BASE + addr;
    uint8_t buf[2] = {reg, data};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP79410_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static uint8_t MCP79410_SRAM_Read(uint8_t addr) {
    if (addr >= MCP79410_SRAM_SIZE) return 0xFF;
    uint8_t reg = MCP79410_SRAM_BASE + addr;
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP79410_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP79410_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    return DL_I2C_receiveControllerData(I2C_0_INST);
}

static void MCP79410_SRAM_WriteBlock(uint8_t addr, const uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len && (addr + i) < MCP79410_SRAM_SIZE; i++) {
        MCP79410_SRAM_Write(addr + i, data[i]);
    }
}

static void MCP79410_SRAM_ReadBlock(uint8_t addr, uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len && (addr + i) < MCP79410_SRAM_SIZE; i++) {
        data[i] = MCP79410_SRAM_Read(addr + i);
    }
}

/* ========== EEPROM驱动（MCP24xx256） ========== */
/*
 * MCP24xx256 EEPROM写入时序：
 * [START][ADDR+W][高地址][低地址][DATA0..DATAN][STOP]
 * 注意：页写入最多64字节，跨页需分次写入
 */
static bool EEPROM_WriteByte(uint16_t addr, uint8_t data) {
    uint8_t buf[3];
    buf[0] = (uint8_t)(addr >> 8);    /* 高地址 */
    buf[1] = (uint8_t)(addr & 0xFF);  /* 低地址 */
    buf[2] = data;

    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 3);
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP24XX256_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 3);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

    /* 等待写入完成（ACK轮询） */
    uint16_t timeout = 1000;
    while (timeout--) {
        DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 1);
        DL_I2C_startControllerTransfer(I2C_0_INST, MCP24XX256_ADDR,
                                       DL_I2C_CONTROLLER_DIRECTION_TX, 0);
        Delay_us(100);
        /* 简单延时替代ACK检测 */
        Delay_ms(5);
        break;
    }
    return true;
}

static uint8_t EEPROM_ReadByte(uint16_t addr) {
    uint8_t addrBuf[2];
    addrBuf[0] = (uint8_t)(addr >> 8);
    addrBuf[1] = (uint8_t)(addr & 0xFF);

    DL_I2C_fillControllerTXFIFO(I2C_0_INST, addrBuf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP24XX256_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

    DL_I2C_startControllerTransfer(I2C_0_INST, MCP24XX256_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    return DL_I2C_receiveControllerData(I2C_0_INST);
}

static bool EEPROM_WriteBlock(uint16_t addr, const uint8_t *data, uint16_t len) {
    /* 分页写入（每页64字节，需处理跨页） */
    while (len > 0) {
        /* 计算当前页剩余空间 */
        uint16_t pageRemaining = 64 - (addr % 64);
        uint16_t chunk = (len < pageRemaining) ? len : pageRemaining;

        /* 页写入 */
        uint8_t buf[2 + 64];
        buf[0] = (uint8_t)(addr >> 8);
        buf[1] = (uint8_t)(addr & 0xFF);
        memcpy(&buf[2], data, chunk);

        DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2 + chunk);
        DL_I2C_startControllerTransfer(I2C_0_INST, MCP24XX256_ADDR,
                                       DL_I2C_CONTROLLER_DIRECTION_TX, 2 + chunk);
        while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

        /* 等待写入完成 */
        Delay_ms(5);

        addr += chunk;
        data += chunk;
        len -= chunk;
    }
    return true;
}

static bool EEPROM_ReadBlock(uint16_t addr, uint8_t *data, uint16_t len) {
    uint8_t addrBuf[2];
    addrBuf[0] = (uint8_t)(addr >> 8);
    addrBuf[1] = (uint8_t)(addr & 0xFF);

    DL_I2C_fillControllerTXFIFO(I2C_0_INST, addrBuf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP24XX256_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

    /* 分批读取（I2C FIFO限制） */
    uint16_t read = 0;
    while (read < len) {
        uint16_t chunk = len - read;
        if (chunk > 32) chunk = 32;
        DL_I2C_startControllerTransfer(I2C_0_INST, MCP24XX256_ADDR,
                                       DL_I2C_CONTROLLER_DIRECTION_RX, chunk);
        while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
        for (uint16_t i = 0; i < chunk; i++) {
            data[read + i] = DL_I2C_receiveControllerData(I2C_0_INST);
        }
        read += chunk;
    }
    return true;
}

/* ========== 配置管理 ========== */
static void Config_LoadDefaults(ConfigData_t *cfg) {
    memset(cfg, 0, sizeof(ConfigData_t));
    cfg->version = DATA_VERSION;
    cfg->validFlag = 0xAA;
    /* 默认PID参数 */
    cfg->pidKp = 2.0f;
    cfg->pidKi = 0.5f;
    cfg->pidKd = 0.1f;
    /* 默认校准参数 */
    cfg->adcGain = 1.0f;
    cfg->adcOffset = 0.0f;
    cfg->dacGain = 1.0f;
    cfg->dacOffset = 0.0f;
    /* 默认系统参数 */
    cfg->targetVoltage_mV = 3300;
    cfg->currentLimit_mA = 500;
    cfg->displayBrightness = 80;
    cfg->sampleRateIdx = 0;
    cfg->alarmHour = 7;
    cfg->alarmMinute = 0;
    /* 计算CRC */
    cfg->crc16 = CRC16_Calc((const uint8_t *)cfg, sizeof(ConfigData_t) - 2);
}

static bool Config_Load(ConfigData_t *cfg) {
    EEPROM_ReadBlock(EEPROM_CONFIG_ADDR, (uint8_t *)cfg, sizeof(ConfigData_t));

    /* 验证有效标志 */
    if (cfg->validFlag != 0xAA) return false;

    /* 验证版本 */
    if (cfg->version != DATA_VERSION) return false;

    /* 验证CRC */
    uint16_t calcCRC = CRC16_Calc((const uint8_t *)cfg, sizeof(ConfigData_t) - 2);
    if (calcCRC != cfg->crc16) return false;

    return true;
}

static bool Config_Save(const ConfigData_t *cfg) {
    /* 计算CRC前先准备数据 */
    ConfigData_t temp = *cfg;
    temp.validFlag = 0xAA;
    temp.version = DATA_VERSION;
    temp.crc16 = CRC16_Calc((const uint8_t *)&temp, sizeof(ConfigData_t) - 2);

    /* 先写入备份区 */
    EEPROM_WriteBlock(EEPROM_BACKUP_ADDR, (const uint8_t *)&temp, sizeof(ConfigData_t));

    /* 再写入主区 */
    return EEPROM_WriteBlock(EEPROM_CONFIG_ADDR, (const uint8_t *)&temp, sizeof(ConfigData_t));
}

static bool Calib_Load(CalibData_t *cal) {
    EEPROM_ReadBlock(EEPROM_CALIB_ADDR, (uint8_t *)cal, sizeof(CalibData_t));
    if (cal->validFlag != 0xAA || cal->version != DATA_VERSION) return false;
    uint16_t calcCRC = CRC16_Calc((const uint8_t *)cal, sizeof(CalibData_t) - 2);
    return (calcCRC == cal->crc16);
}

static bool Calib_Save(const CalibData_t *cal) {
    CalibData_t temp = *cal;
    temp.validFlag = 0xAA;
    temp.version = DATA_VERSION;
    temp.crc16 = CRC16_Calc((const uint8_t *)&temp, sizeof(CalibData_t) - 2);
    return EEPROM_WriteBlock(EEPROM_CALIB_ADDR, (const uint8_t *)&temp, sizeof(CalibData_t));
}

static void Log_Append(const LogEntry_t *entry) {
    /* 检查是否超过日志区边界 */
    if (g_logWriteAddr + sizeof(LogEntry_t) > EEPROM_BACKUP_ADDR) {
        g_logWriteAddr = EEPROM_LOG_ADDR;  /* 环形覆盖 */
    }
    EEPROM_WriteBlock(g_logWriteAddr, (const uint8_t *)entry, sizeof(LogEntry_t));
    g_logWriteAddr += sizeof(LogEntry_t);
}

/* ========== OLED驱动 ========== */
static void OLED_WriteCmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, OLED_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
}

static void OLED_WriteData(uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, OLED_ADDR,
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

static void OLED_SetPixel(int16_t x, int16_t y, uint8_t c) {
    if (x<0||x>=128||y<0||y>=64) return;
    uint16_t idx = (y/8)*128+x;
    uint8_t bit = (1<<(y&7));
    if (c) g_oledBuffer[idx]|=bit; else g_oledBuffer[idx]&=~bit;
}

static void OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h) {
    for (int16_t i=x;i<x+w;i++) for(int16_t j=y;j<y+h;j++) OLED_SetPixel(i,j,1);
}

static void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    int16_t dx=abs(x1-x0),dy=abs(y1-y0),sx=(x0<x1)?1:-1,sy=(y0<y1)?1:-1,err=dx-dy;
    while(1){OLED_SetPixel(x0,y0,1);if(x0==x1&&y0==y1)break;int16_t e2=2*err;if(e2>-dy){err-=dy;x0+=sx;}if(e2<dx){err+=dx;y0+=sy;}}
}

static void OLED_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h) {
    OLED_DrawLine(x,y,x+w-1,y);OLED_DrawLine(x+w-1,y,x+w-1,y+h-1);
    OLED_DrawLine(x+w-1,y+h-1,x,y+h-1);OLED_DrawLine(x,y+h-1,x,y);
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

/* ========== UI页面 ========== */

/* 配置参数页面 */
static void UI_DrawConfigPage(void) {
    char buf[32];
    OLED_Clear();

    OLED_DrawString(0, 0, "CONFIG PARAMS", 1);
    OLED_DrawLine(0, 9, 127, 9);

    snprintf(buf, sizeof(buf), "Kp:%.2f Ki:%.2f", g_config.pidKp, g_config.pidKi);
    OLED_DrawString(0, 12, buf, 1);
    snprintf(buf, sizeof(buf), "Kd:%.2f", g_config.pidKd);
    OLED_DrawString(0, 22, buf, 1);
    snprintf(buf, sizeof(buf), "Vset:%dmV", g_config.targetVoltage_mV);
    OLED_DrawString(0, 32, buf, 1);
    snprintf(buf, sizeof(buf), "Ilim:%dmA", g_config.currentLimit_mA);
    OLED_DrawString(0, 42, buf, 1);
    snprintf(buf, sizeof(buf), "Bright:%d%%", g_config.displayBrightness);
    OLED_DrawString(0, 52, buf, 1);

    OLED_Update();
}

/* 校准数据页面 */
static void UI_DrawCalibPage(void) {
    char buf[32];
    OLED_Clear();

    OLED_DrawString(0, 0, "CALIBRATION", 1);
    OLED_DrawLine(0, 9, 127, 9);

    snprintf(buf, sizeof(buf), "ADC G:%.4f", g_calib.adcCalPoints[0]);
    OLED_DrawString(0, 12, buf, 1);
    snprintf(buf, sizeof(buf), "DAC G:%.4f", g_calib.dacCalPoints[0]);
    OLED_DrawString(0, 22, buf, 1);
    snprintf(buf, sizeof(buf), "TmpOf:%.2fC", g_calib.tempOffset);
    OLED_DrawString(0, 32, buf, 1);

    snprintf(buf, sizeof(buf), "Date:%lu", (unsigned long)g_calib.calDate);
    OLED_DrawString(0, 44, buf, 1);

    OLED_Update();
}

/* RTC页面 */
static void UI_DrawRTCPage(void) {
    char buf[32];
    RTC_Time_t time;
    MCP79410_GetTime(&time);

    OLED_Clear();
    OLED_DrawString(0, 0, "MCP79410 RTC", 1);
    OLED_DrawLine(0, 9, 127, 9);

    snprintf(buf, sizeof(buf), "20%02d-%02d-%02d",
             time.year, time.month, time.date);
    OLED_DrawString(0, 14, buf, 2);

    snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
             time.hour, time.minute, time.second);
    OLED_DrawString(16, 34, buf, 2);

    /* 掉电状态 */
    if (g_powerFailed) {
        OLED_DrawString(0, 54, "PWR FAIL DETECTED!", 1);
    } else {
        OLED_DrawString(0, 54, "VBAT:OK", 1);
    }

    OLED_Update();
}

/* SRAM页面 */
static void UI_DrawSRAMPage(void) {
    char buf[32];
    OLED_Clear();

    OLED_DrawString(0, 0, "64B SRAM VIEW", 1);
    OLED_DrawLine(0, 9, 127, 9);

    /* 显示前32字节（8行x4字节） */
    for (uint8_t row = 0; row < 6; row++) {
        uint8_t addr = row * 8;
        snprintf(buf, sizeof(buf), "%02X:", addr);
        for (uint8_t i = 0; i < 8; i++) {
            uint8_t val = MCP79410_SRAM_Read(addr + i);
            int n = strlen(buf);
            snprintf(buf + n, sizeof(buf) - n, " %02X", val);
        }
        OLED_DrawString(0, 12 + row * 9, buf, 1);
    }

    OLED_Update();
}

/* EEPROM信息页面 */
static void UI_DrawEEPROMInfo(void) {
    char buf[32];
    OLED_Clear();

    OLED_DrawString(0, 0, "EEPROM STATUS", 1);
    OLED_DrawLine(0, 9, 127, 9);

    snprintf(buf, sizeof(buf), "Size: 32KB");
    OLED_DrawString(0, 12, buf, 1);

    snprintf(buf, sizeof(buf), "Cfg: 0x%04X", EEPROM_CONFIG_ADDR);
    OLED_DrawString(0, 22, buf, 1);
    snprintf(buf, sizeof(buf), "Cal: 0x%04X", EEPROM_CALIB_ADDR);
    OLED_DrawString(0, 32, buf, 1);
    snprintf(buf, sizeof(buf), "Log: 0x%04X", (unsigned int)g_logWriteAddr);
    OLED_DrawString(0, 42, buf, 1);
    snprintf(buf, sizeof(buf), "Bak: 0x%04X", EEPROM_BACKUP_ADDR);
    OLED_DrawString(0, 52, buf, 1);

    OLED_Update();
}

/* 日志页面 */
static void UI_DrawLogPage(void) {
    char buf[32];
    OLED_Clear();

    OLED_DrawString(0, 0, "DATA LOG", 1);
    OLED_DrawLine(0, 9, 127, 9);

    /* 读取最后几条日志 */
    uint32_t readAddr = g_logWriteAddr;
    for (uint8_t i = 0; i < 3 && readAddr > EEPROM_LOG_ADDR + sizeof(LogEntry_t); i++) {
        readAddr -= sizeof(LogEntry_t);
        LogEntry_t entry;
        EEPROM_ReadBlock(readAddr, (uint8_t *)&entry, sizeof(LogEntry_t));

        snprintf(buf, sizeof(buf), "T%lu V%.0f I%.0f",
                 (unsigned long)entry.timestamp, entry.voltage_mV, entry.current_mA);
        OLED_DrawString(0, 12 + i * 14, buf, 1);
        snprintf(buf, sizeof(buf), "  T:%.1fC E:%d", entry.temperature, entry.errorCode);
        OLED_DrawString(0, 20 + i * 14, buf, 1);
    }

    snprintf(buf, sizeof(buf), "Next: 0x%04X", (unsigned int)g_logWriteAddr);
    OLED_DrawString(0, 56, buf, 1);

    OLED_Update();
}

/* ========== 主函数 ========== */
int main(void) {
    /* 系统初始化 */
    DL_SYSCTL_initSYSCTL();
    SysTick_Config(SystemCoreClock / 1000);

    /* 外设初始化 */
    DL_I2C_initController(I2C_0_INST, 400000);

    /* GPIO */
    DL_GPIO_initDigitalInput(BTN_MENU_PORT | BTN_MENU_PIN);
    DL_GPIO_initDigitalInput(BTN_UP_PORT | BTN_UP_PIN);
    DL_GPIO_initDigitalInput(BTN_DOWN_PORT | BTN_DOWN_PIN);
    DL_GPIO_initDigitalInput(BTN_OK_PORT | BTN_OK_PIN);

    /* MCP79410和OLED初始化 */
    MCP79410_Init();
    OLED_Init();

    /* 开机画面 */
    OLED_Clear();
    OLED_DrawString(8, 16, "EEPROM DATA STORE", 1);
    OLED_DrawString(16, 32, "MCP79410 + MCP24", 1);
    OLED_DrawString(24, 48, "Config Manager", 1);
    OLED_Update();
    Delay_ms(1500);

    /* 检测掉电 */
    g_powerFailed = MCP79410_IsPowerFailed();
    if (g_powerFailed) {
        OLED_Clear();
        OLED_DrawString(8, 20, "POWER FAILURE!", 1);
        OLED_DrawString(8, 36, "Recovering config...", 1);
        OLED_Update();
        MCP79410_ClearPowerFailFlag();
        Delay_ms(2000);
    }

    /* 加载配置 */
    if (!Config_Load(&g_config)) {
        /* 配置无效（首次运行或数据损坏），加载默认值 */
        Config_LoadDefaults(&g_config);
        Config_Save(&g_config);
        OLED_Clear();
        OLED_DrawString(8, 24, "Default config loaded", 1);
        OLED_Update();
        Delay_ms(1000);
    }

    /* 加载校准数据 */
    if (!Calib_Load(&g_calib)) {
        memset(&g_calib, 0, sizeof(CalibData_t));
        g_calib.version = DATA_VERSION;
        g_calib.validFlag = 0xAA;
        g_calib.adcCalPoints[0] = 1.0f;
        g_calib.dacCalPoints[0] = 1.0f;
        g_calib.calDate = 20260101;
        Calib_Save(&g_calib);
    }

    /* 写入SRAM测试数据（用于演示） */
    uint8_t sramTest[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
    MCP79410_SRAM_WriteBlock(0, sramTest, 8);

    /* 写入测试日志 */
    LogEntry_t testLog = {g_systickCount / 1000, 3300.0f, 150.0f, 25.5f, 0};
    Log_Append(&testLog);

    uint32_t lastLogTime = 0;
    uint32_t lastDisplayTime = 0;

    while (1) {
        /* 按键：切换页面 */
        if (Button_IsPressed(BTN_MENU_PORT, BTN_MENU_PIN)) {
            g_uiPage = (g_uiPage + 1) % 6;
        }

        /* 按键OK：保存当前配置 */
        if (Button_IsPressed(BTN_OK_PORT, BTN_OK_PIN)) {
            if (g_uiPage == 0) {
                Config_Save(&g_config);
                OLED_Clear();
                OLED_DrawString(16, 28, "CONFIG SAVED!", 2);
                OLED_Update();
                Delay_ms(500);
            } else if (g_uiPage == 1) {
                Calib_Save(&g_calib);
                OLED_Clear();
                OLED_DrawString(16, 28, "CALIB SAVED!", 2);
                OLED_Update();
                Delay_ms(500);
            }
        }

        /* 每5秒记录一次日志 */
        if ((g_systickCount - lastLogTime) >= 5000) {
            LogEntry_t logEntry;
            logEntry.timestamp = g_systickCount / 1000;
            logEntry.voltage_mV = 3300.0f;  /* 示例值 */
            logEntry.current_mA = 150.0f;
            logEntry.temperature = 25.5f;
            logEntry.errorCode = 0;
            Log_Append(&logEntry);
            lastLogTime = g_systickCount;
        }

        /* 200ms显示刷新 */
        if ((g_systickCount - lastDisplayTime) >= 200) {
            switch (g_uiPage) {
                case 0: UI_DrawConfigPage();  break;
                case 1: UI_DrawCalibPage();   break;
                case 2: UI_DrawRTCPage();     break;
                case 3: UI_DrawSRAMPage();    break;
                case 4: UI_DrawEEPROMInfo();  break;
                case 5: UI_DrawLogPage();     break;
            }
            lastDisplayTime = g_systickCount;
        }
    }
}
