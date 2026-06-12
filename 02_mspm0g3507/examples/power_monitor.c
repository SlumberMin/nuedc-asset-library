/**
 * @file power_monitor.c
 * @brief 功率监测器 - INA219 + OLED显示 + 过流报警 + 数据记录
 * @platform MSPM0G3507
 * @date 2026-06-12
 *
 * 功能概述：
 *   1. INA219电流/电压/功率传感器实时测量
 *   2. SSD1306 OLED显示实时数据（电压/电流/功率/能量）
 *   3. 过流/过压/欠压报警（蜂鸣器+LED）
 *   4. 简易数据记录（环形缓冲区，可通过串口导出）
 *   5. 峰值检测和平均值统计
 *
 * 硬件连接：
 *   INA219: I2C0 (PA10-SCL, PA11-SDA), 地址0x40
 *   OLED:   I2C0, 地址0x3C
 *   蜂鸣器: PB0
 *   LED:    PB1(正常), PB2(报警)
 *   按键:   PB3(清零), PB4(切换显示页)
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ===== I2C设备地址 ===== */
#define INA219_ADDR             0x40
#define SSD1306_ADDR            0x3C

/* ===== INA219寄存器定义 ===== */
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_V      0x01
#define INA219_REG_BUS_V        0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

/* INA219配置值 */
#define INA219_CONFIG_DEFAULT   0x399F      /* 默认配置 */
#define INA219_CONFIG_32V_2A    0x219F      /* 32V量程, 2A */
#define INA219_CONFIG_16V_1A    0x019F      /* 16V量程, 1A */
#define INA219_CAL_32V_2A       4096        /* 校准值(32V/2A) */
#define INA219_CAL_16V_1A       10240       /* 校准值(16V/1A) */
#define INA219_CURRENT_LSB_UA   100         /* 电流LSB=100uA */

/* ===== 报警阈值 ===== */
#define ALERT_OVERCURRENT_MA    1500        /* 过流阈值(mA) */
#define ALERT_OVERVOLTAGE_MV    26000       /* 过压阈值(mV) */
#define ALERT_UNDERVOLTAGE_MV   4500        /* 欠压阈值(mV) */

/* ===== 数据记录参数 ===== */
#define DATA_LOG_SIZE           256         /* 环形缓冲区大小 */
#define LOG_INTERVAL_MS         1000        /* 记录间隔(ms) */

/* ===== OLED参数 ===== */
#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_PAGES              8

/* ===== 引脚定义 ===== */
#define BUZZER_PORT             GPIOB
#define BUZZER_PIN              DL_GPIO_PIN_0
#define LED_NORMAL_PORT         GPIOB
#define LED_NORMAL_PIN          DL_GPIO_PIN_1
#define LED_ALARM_PORT          GPIOB
#define LED_ALARM_PIN           DL_GPIO_PIN_2
#define KEY_CLEAR_PORT          GPIOB
#define KEY_CLEAR_PIN           DL_GPIO_PIN_3
#define KEY_PAGE_PORT           GPIOB
#define KEY_PAGE_PIN            DL_GPIO_PIN_4

/* ===== 数据结构 ===== */
typedef struct {
    float voltage_v;        /* 电压(V) */
    float current_ma;       /* 电流(mA) */
    float power_mw;         /* 功率(mW) */
    uint32_t timestamp_ms;  /* 时间戳 */
} PowerData_t;

typedef struct {
    PowerData_t buffer[DATA_LOG_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    float peak_voltage;
    float peak_current;
    float peak_power;
    float energy_mwh;       /* 累计能量(mWh) */
    uint32_t samples;
} DataLogger_t;

typedef enum {
    PAGE_REALTIME,          /* 实时数据页 */
    PAGE_PEAK,              /* 峰值统计页 */
    PAGE_ENERGY,            /* 能量统计页 */
    PAGE_MAX
} DisplayPage_t;

/* ===== 全局变量 ===== */
static volatile uint32_t g_systick_count = 0;
static DataLogger_t g_logger;
static DisplayPage_t g_display_page = PAGE_REALTIME;
static bool g_alarm_active = false;
static PowerData_t g_current_data;

/* OLED帧缓冲 */
static uint8_t g_oled_buffer[OLED_PAGES][OLED_WIDTH];
static bool g_oled_dirty = false;

/* 简易6x8字体(ASCII 32-127, 仅部分常用字符) */
static const uint8_t font_6x8[][6] = {
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
};

/* ===== SysTick ===== */
void SysTick_Handler(void) { g_systick_count++; }

static void delay_ms(uint32_t ms) {
    uint32_t start = g_systick_count;
    while ((g_systick_count - start) < ms);
}

static void delay_us(uint32_t us) {
    volatile uint32_t count = us * (SystemCoreClock / 1000000) / 4;
    while (count--);
}

static uint32_t get_tick(void) { return g_systick_count; }

/* ===== I2C通信 ===== */
static void i2c_write_reg8(uint8_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
}

static void i2c_write_reg16(uint8_t dev, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 3);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_TX, 3);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
}

static uint16_t i2c_read_reg16(uint8_t dev, uint8_t reg) {
    uint8_t buf[2];
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);

    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_RX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    buf[0] = DL_I2C_receiveControllerData(I2C_0_INST);
    buf[1] = DL_I2C_receiveControllerData(I2C_0_INST);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static void i2c_write_bytes(uint8_t dev, const uint8_t *data, uint8_t len) {
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, data, len);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_TX, len);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
}

/* ===== INA219驱动 ===== */
static void ina219_init(void) {
    /* 复位 */
    i2c_write_reg16(INA219_ADDR, INA219_REG_CONFIG, 0xFFFF);
    delay_ms(10);

    /* 配置: 总线32V量程, 分流器±320mV, 12位ADC, 连续测量 */
    i2c_write_reg16(INA219_ADDR, INA219_REG_CONFIG, INA219_CONFIG_32V_2A);
    delay_ms(1);

    /* 设置校准值
     * 分流电阻=0.1Ω, 最大电流=2A
     * Cal = 0.04096 / (Current_LSB * Rshunt) = 0.04096 / (0.0001 * 0.1) = 4096
     */
    i2c_write_reg16(INA219_ADDR, INA219_REG_CALIBRATION, INA219_CAL_32V_2A);
    delay_ms(1);
}

/* 读取分流电压(mV) */
static int16_t ina219_read_shunt_voltage(void) {
    int16_t raw = (int16_t)i2c_read_reg16(INA219_ADDR, INA219_REG_SHUNT_V);
    /* LSB = 10uV, 转换为mV */
    return raw / 100;
}

/* 读取总线电压(mV) */
static uint16_t ina219_read_bus_voltage(void) {
    uint16_t raw = i2c_read_reg16(INA219_ADDR, INA219_REG_BUS_V);
    /* 右移3位去掉CNVR和OVF标志位, LSB=4mV */
    return (raw >> 3) * 4;
}

/* 读取电流(mA) */
static int16_t ina219_read_current(void) {
    int16_t raw = (int16_t)i2c_read_reg16(INA219_ADDR, INA219_REG_CURRENT);
    /* Current_LSB = 100uA */
    return raw / 10;
}

/* 读取功率(mW) */
static uint16_t ina219_read_power(void) {
    uint16_t raw = i2c_read_reg16(INA219_ADDR, INA219_REG_POWER);
    /* Power_LSB = 20 * Current_LSB = 2mW */
    return raw * 2;
}

/* 读取所有数据 */
static PowerData_t ina219_read_all(void) {
    PowerData_t data;
    data.voltage_v = ina219_read_bus_voltage() / 1000.0f;
    data.current_ma = ina219_read_current();
    data.power_mw = ina219_read_power();
    data.timestamp_ms = get_tick();
    return data;
}

/* ===== SSD1306 OLED驱动 ===== */
static void oled_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd}; /* Co=0, D/C=0: 命令 */
    i2c_write_bytes(SSD1306_ADDR, buf, 2);
}

static void oled_data(uint8_t data) {
    uint8_t buf[2] = {0x40, data}; /* Co=0, D/C=1: 数据 */
    i2c_write_bytes(SSD1306_ADDR, buf, 2);
}

static void oled_init(void) {
    delay_ms(100);
    oled_cmd(0xAE); /* 关闭显示 */
    oled_cmd(0xD5); /* 设置时钟分频 */
    oled_cmd(0x80);
    oled_cmd(0xA8); /* 设置多路复用率 */
    oled_cmd(0x3F); /* 1/64 */
    oled_cmd(0xD3); /* 设置显示偏移 */
    oled_cmd(0x00);
    oled_cmd(0x40); /* 设置起始行=0 */
    oled_cmd(0x8D); /* 电荷泵 */
    oled_cmd(0x14); /* 使能 */
    oled_cmd(0x20); /* 内存寻址模式 */
    oled_cmd(0x02); /* 页寻址 */
    oled_cmd(0xA1); /* 段重映射(左右翻转) */
    oled_cmd(0xC8); /* COM扫描方向(上下翻转) */
    oled_cmd(0xDA); /* COM引脚配置 */
    oled_cmd(0x12);
    oled_cmd(0x81); /* 对比度 */
    oled_cmd(0xCF);
    oled_cmd(0xD9); /* 预充电周期 */
    oled_cmd(0xF1);
    oled_cmd(0xDB); /* VCOMH取消选择电平 */
    oled_cmd(0x30);
    oled_cmd(0xA4); /* 全局显示开启 */
    oled_cmd(0xA6); /* 正常显示(非反色) */
    oled_cmd(0xAF); /* 开启显示 */

    /* 清屏 */
    memset(g_oled_buffer, 0, sizeof(g_oled_buffer));
}

/* 设置像素 */
static void oled_set_pixel(uint8_t x, uint8_t y, bool on) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint8_t page = y / 8;
    uint8_t bit = y % 8;
    if (on) {
        g_oled_buffer[page][x] |= (1 << bit);
    } else {
        g_oled_buffer[page][x] &= ~(1 << bit);
    }
    g_oled_dirty = true;
}

/* 绘制字符(6x8) */
static void oled_draw_char(uint8_t x, uint8_t y, char c) {
    if (c < 32 || c > 90) return; /* 仅支持ASCII 32-90 */
    uint8_t idx = c - 32;
    uint8_t page = y / 8;
    for (uint8_t i = 0; i < 6; i++) {
        if (x + i < OLED_WIDTH) {
            g_oled_buffer[page][x + i] = font_6x8[idx][i];
        }
    }
    g_oled_dirty = true;
}

/* 绘制字符串 */
static void oled_draw_string(uint8_t x, uint8_t y, const char *str) {
    while (*str) {
        oled_draw_char(x, y, *str);
        x += 6;
        str++;
    }
}

/* 刷新显示(将缓冲区写入OLED) */
static void oled_refresh(void) {
    if (!g_oled_dirty) return;

    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_cmd(0xB0 + page); /* 设置页地址 */
        oled_cmd(0x00);        /* 设置列地址低4位 */
        oled_cmd(0x10);        /* 设置列地址高4位 */

        for (uint8_t col = 0; col < OLED_WIDTH; col++) {
            oled_data(g_oled_buffer[page][col]);
        }
    }
    g_oled_dirty = false;
}

/* 清屏 */
static void oled_clear(void) {
    memset(g_oled_buffer, 0, sizeof(g_oled_buffer));
    g_oled_dirty = true;
}

/* ===== 数据记录器 ===== */
static void logger_init(void) {
    memset(&g_logger, 0, sizeof(DataLogger_t));
    g_logger.peak_voltage = 0;
    g_logger.peak_current = 0;
    g_logger.peak_power = 0;
}

static void logger_add(PowerData_t *data) {
    g_logger.buffer[g_logger.head] = *data;
    g_logger.head = (g_logger.head + 1) % DATA_LOG_SIZE;
    if (g_logger.count < DATA_LOG_SIZE) {
        g_logger.count++;
    } else {
        g_logger.tail = (g_logger.tail + 1) % DATA_LOG_SIZE;
    }

    /* 更新峰值 */
    if (data->voltage_v > g_logger.peak_voltage) {
        g_logger.peak_voltage = data->voltage_v;
    }
    if (fabsf(data->current_ma) > fabsf(g_logger.peak_current)) {
        g_logger.peak_current = data->current_ma;
    }
    if (data->power_mw > g_logger.peak_power) {
        g_logger.peak_power = data->power_mw;
    }

    /* 累计能量: E += P * dt, dt=1s, 转换为mWh */
    g_logger.energy_mwh += data->power_mw / 3600.0f;
    g_logger.samples++;
}

/* ===== 报警处理 ===== */
static void check_alarms(PowerData_t *data) {
    bool alarm = false;

    if (fabsf(data->current_ma) > ALERT_OVERCURRENT_MA) {
        alarm = true;
    }
    if (data->voltage_v * 1000 > ALERT_OVERVOLTAGE_MV) {
        alarm = true;
    }
    if (data->voltage_v * 1000 < ALERT_UNDERVOLTAGE_MV && data->voltage_v > 0.1f) {
        alarm = true;
    }

    g_alarm_active = alarm;

    if (alarm) {
        /* 蜂鸣器报警(间歇) */
        DL_GPIO_togglePins(BUZZER_PORT, BUZZER_PIN);
        DL_GPIO_setPins(LED_ALARM_PORT, LED_ALARM_PIN);
        DL_GPIO_clearPins(LED_NORMAL_PORT, LED_NORMAL_PIN);
    } else {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        DL_GPIO_clearPins(LED_ALARM_PORT, LED_ALARM_PIN);
        DL_GPIO_setPins(LED_NORMAL_PORT, LED_NORMAL_PIN);
    }
}

/* ===== 显示页面 ===== */
static void display_realtime(PowerData_t *data) {
    char buf[22];

    oled_clear();
    oled_draw_string(0, 0, "=== POWER MONITOR ===");

    /* 电压 */
    snprintf(buf, sizeof(buf), "V: %5.3f V", data->voltage_v);
    oled_draw_string(0, 16, buf);

    /* 电流 */
    snprintf(buf, sizeof(buf), "I: %6.1f mA", data->current_ma);
    oled_draw_string(0, 24, buf);

    /* 功率 */
    snprintf(buf, sizeof(buf), "P: %6.1f mW", data->power_mw);
    oled_draw_string(0, 32, buf);

    /* 报警状态 */
    if (g_alarm_active) {
        oled_draw_string(0, 48, "!!! ALARM !!!");
    } else {
        oled_draw_string(0, 48, "Status: NORMAL");
    }

    oled_refresh();
}

static void display_peak(void) {
    char buf[22];

    oled_clear();
    oled_draw_string(0, 0, "=== PEAK VALUES ===");

    snprintf(buf, sizeof(buf), "Vmax: %5.3f V", g_logger.peak_voltage);
    oled_draw_string(0, 16, buf);

    snprintf(buf, sizeof(buf), "Imax: %6.1f mA", g_logger.peak_current);
    oled_draw_string(0, 24, buf);

    snprintf(buf, sizeof(buf), "Pmax: %6.1f mW", g_logger.peak_power);
    oled_draw_string(0, 32, buf);

    snprintf(buf, sizeof(buf), "Samples: %lu", g_logger.samples);
    oled_draw_string(0, 48, buf);

    oled_refresh();
}

static void display_energy(void) {
    char buf[22];

    oled_clear();
    oled_draw_string(0, 0, "=== ENERGY ===");

    snprintf(buf, sizeof(buf), "E: %8.2f mWh", g_logger.energy_mwh);
    oled_draw_string(0, 16, buf);

    float hours = g_logger.samples / 3600.0f;
    snprintf(buf, sizeof(buf), "T: %6.2f h", hours);
    oled_draw_string(0, 24, buf);

    /* 平均功率 */
    float avg_power = (g_logger.samples > 0) ?
                      g_logger.energy_mwh / hours : 0;
    if (hours > 0) {
        snprintf(buf, sizeof(buf), "Pavg: %5.1f mW", avg_power);
        oled_draw_string(0, 32, buf);
    }

    snprintf(buf, sizeof(buf), "Log: %d/%d", g_logger.count, DATA_LOG_SIZE);
    oled_draw_string(0, 48, buf);

    oled_refresh();
}

/* ===== 按键扫描 ===== */
static bool key_pressed(GPIO_Regs *port, uint32_t pin) {
    if (!(port->DIN31_0 & pin)) {
        delay_ms(20);
        if (!(port->DIN31_0 & pin)) {
            while (!(port->DIN31_0 & pin));
            return true;
        }
    }
    return false;
}

/* ===== 主函数 ===== */
int main(void) {
    /* 系统初始化 */
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* GPIO初始化 */
    DL_GPIO_initDigitalOutput(BUZZER_PIN);
    DL_GPIO_enableOutput(BUZZER_PORT, BUZZER_PIN);
    DL_GPIO_initDigitalOutput(LED_NORMAL_PIN);
    DL_GPIO_enableOutput(LED_NORMAL_PORT, LED_NORMAL_PIN);
    DL_GPIO_initDigitalOutput(LED_ALARM_PIN);
    DL_GPIO_enableOutput(LED_ALARM_PORT, LED_ALARM_PIN);
    DL_GPIO_initDigitalInput(KEY_CLEAR_PIN);
    DL_GPIO_initDigitalInput(KEY_PAGE_PIN);

    /* INA219初始化 */
    ina219_init();

    /* OLED初始化 */
    oled_init();

    /* 数据记录器初始化 */
    logger_init();

    /* 显示启动信息 */
    oled_clear();
    oled_draw_string(12, 24, "Power Monitor v1.0");
    oled_draw_string(18, 40, "Initializing...");
    oled_refresh();
    delay_ms(1000);

    uint32_t last_log_tick = 0;
    uint32_t last_display_tick = 0;

    /* 主循环 */
    while (1) {
        uint32_t now = get_tick();

        /* 读取传感器数据 */
        g_current_data = ina219_read_all();

        /* 检查报警 */
        check_alarms(&g_current_data);

        /* 数据记录(每秒) */
        if ((now - last_log_tick) >= LOG_INTERVAL_MS) {
            logger_add(&g_current_data);
            last_log_tick = now;
        }

        /* 显示更新(每200ms) */
        if ((now - last_display_tick) >= 200) {
            switch (g_display_page) {
            case PAGE_REALTIME:
                display_realtime(&g_current_data);
                break;
            case PAGE_PEAK:
                display_peak();
                break;
            case PAGE_ENERGY:
                display_energy();
                break;
            default:
                break;
            }
            last_display_tick = now;
        }

        /* 按键处理 */
        if (key_pressed(KEY_CLEAR_PORT, KEY_CLEAR_PIN)) {
            /* 清零统计 */
            logger_init();
        }

        if (key_pressed(KEY_PAGE_PORT, KEY_PAGE_PIN)) {
            /* 切换显示页 */
            g_display_page = (DisplayPage_t)((g_display_page + 1) % PAGE_MAX);
        }

        delay_ms(10);
    }
}
