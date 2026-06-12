/**
 * @file weather_station.c
 * @brief MSPM0G3507 气象站示例
 *
 * 硬件连接：
 *   BMP280气压传感器（I2C）：
 *     SDA -> PB0 (I2C0 SDA)
 *     SCL -> PB1 (I2C0 SCL)
 *     ADDR -> GND (I2C地址 0x76)
 *
 *   SHT30温湿度传感器（I2C）：
 *     SDA -> PB0 (共享I2C总线)
 *     SCL -> PB1
 *     ADDR -> GND (I2C地址 0x44)
 *
 *   OLED显示屏（I2C，128x64 SSD1306）：
 *     SDA -> PB0 (共享I2C总线)
 *     SCL -> PB1
 *     ADDR -> GND (I2C地址 0x3C)
 *
 *   按键：
 *     模式切换 -> PA3
 *     历史查看 -> PA4
 *     报警设置 -> PA5
 *
 * 功能：
 *   - BMP280气压、温度、海拔测量
 *   - SHT30温湿度精确测量
 *   - OLED多页面显示（实时数据/趋势图/历史记录）
 *   - 气压趋势预测（晴天/阴天/雨天）
 *   - 温湿度报警
 *   - 24小时历史数据记录
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ========== I2C设备地址 ========== */
#define BMP280_ADDR     0x76
#define SHT30_ADDR      0x44
#define OLED_ADDR       0x3C

/* ========== BMP280寄存器 ========== */
#define BMP280_REG_ID        0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB  0xFA

/* BMP280校准数据结构 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibData_t;

/* BMP280测量结果 */
typedef struct {
    float temperature;   /* 温度 °C */
    float pressure;      /* 气压 Pa */
    float altitude;      /* 海拔 m */
} BMP280_Data_t;

/* ========== SHT30命令 ========== */
#define SHT30_CMD_MEAS_HIGH  0x2400  /* 高重复性测量 */
#define SHT30_CMD_MEAS_MED   0x240B  /* 中重复性测量 */
#define SHT30_CMD_MEAS_LOW   0x2416  /* 低重复性测量 */
#define SHT30_CMD_SOFT_RESET 0x30A2  /* 软复位 */

/* SHT30测量结果 */
typedef struct {
    float temperature;   /* 温度 °C */
    float humidity;      /* 相对湿度 %RH */
} SHT30_Data_t;

/* ========== 天气趋势枚举 ========== */
typedef enum {
    TREND_SUNNY = 0,    /* 晴天（气压稳定或上升） */
    TREND_CLOUDY,       /* 多云（气压缓慢下降） */
    TREND_RAINY,        /* 雨天（气压快速下降） */
    TREND_UNKNOWN       /* 数据不足 */
} WeatherTrend_t;

/* 天气趋势名称 */
static const char *trend_names[] = {"Sunny", "Cloudy", "Rainy", "Unknown"};
static const char *trend_icons[] = {"*", "~", "=", "?"};

/* ========== 历史数据结构 ========== */
#define HISTORY_SIZE  288  /* 24小时，每5分钟一个点 */

typedef struct {
    float temp_history[HISTORY_SIZE];
    float humi_history[HISTORY_SIZE];
    float press_history[HISTORY_SIZE];
    uint32_t write_idx;
    uint32_t count;
} HistoryData_t;

/* ========== 报警阈值 ========== */
typedef struct {
    float temp_high;     /* 温度上限 °C */
    float temp_low;      /* 温度下限 °C */
    float humi_high;     /* 湿度上限 %RH */
    float humi_low;      /* 湿度下限 %RH */
    float press_high;    /* 气压上限 hPa */
    float press_low;     /* 气压下限 hPa */
} AlarmThreshold_t;

/* ========== 显示页面枚举 ========== */
typedef enum {
    PAGE_REALTIME = 0,   /* 实时数据 */
    PAGE_TREND,          /* 趋势图 */
    PAGE_HISTORY,        /* 历史记录 */
    PAGE_ALARM,          /* 报警设置 */
    PAGE_COUNT
} DisplayPage_t;

/* ========== 全局变量 ========== */
static BMP280_CalibData_t g_bmp280_calib;
static BMP280_Data_t g_bmp280_data = {0};
static SHT30_Data_t g_sht30_data = {0};
static WeatherTrend_t g_weather_trend = TREND_UNKNOWN;
static HistoryData_t g_history = {0};
static AlarmThreshold_t g_alarm = {40.0f, 0.0f, 90.0f, 20.0f, 1050.0f, 950.0f};
static DisplayPage_t g_current_page = PAGE_REALTIME;
static bool g_alarm_enabled = true;
static bool g_alarm_triggered = false;
static uint32_t g_tick = 0;
static float g_press_trend[12] = {0};  /* 最近1小时气压（每5分钟） */
static uint8_t g_trend_idx = 0;

/* I2C句柄 */
extern I2C_Regs *g_i2c;

/* ========== I2C辅助函数 ========== */
static bool i2c_write_bytes(uint8_t addr, const uint8_t *data, uint8_t len)
{
    DL_I2C_flushControllerTXFIFO(g_i2c);
    DL_I2C_fillControllerTXFIFO(g_i2c, (uint8_t *)data, len);
    DL_I2C_startControllerTransfer(g_i2c, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    while (DL_I2C_getControllerStatus(g_i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    return true;
}

static bool i2c_read_bytes(uint8_t addr, uint8_t *data, uint8_t len)
{
    DL_I2C_startControllerTransfer(g_i2c, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    while (DL_I2C_getControllerStatus(g_i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    for (int i = 0; i < len; i++) {
        data[i] = DL_I2C_receiveControllerData(g_i2c);
    }
    return true;
}

static bool i2c_write_reg8(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_write_bytes(addr, buf, 2);
}

static uint8_t i2c_read_reg8(uint8_t addr, uint8_t reg)
{
    uint8_t val;
    i2c_write_bytes(addr, &reg, 1);
    i2c_read_bytes(addr, &val, 1);
    return val;
}

/* ========== BMP280驱动 ========== */
static void bmp280_read_calibration(void)
{
    uint8_t calib[26];
    uint8_t reg = 0x88;
    i2c_write_bytes(BMP280_ADDR, &reg, 1);
    i2c_read_bytes(BMP280_ADDR, calib, 26);

    g_bmp280_calib.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    g_bmp280_calib.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    g_bmp280_calib.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
    g_bmp280_calib.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    g_bmp280_calib.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    g_bmp280_calib.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    g_bmp280_calib.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    g_bmp280_calib.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    g_bmp280_calib.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    g_bmp280_calib.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    g_bmp280_calib.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    g_bmp280_calib.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
}

static bool bmp280_init(void)
{
    /* 检测芯片ID */
    uint8_t id = i2c_read_reg8(BMP280_ADDR, BMP280_REG_ID);
    if (id != 0x58 && id != 0x60) {
        return false;  /* BMP280 ID应为0x58，BME280为0x60 */
    }

    /* 读取校准数据 */
    bmp280_read_calibration();

    /* 配置：
     * osrs_t = x16 (温度过采样)
     * osrs_p = x16 (气压过采样)
     * mode = normal
     */
    i2c_write_reg8(BMP280_ADDR, BMP280_REG_CTRL_MEAS, 0x57);  /* 16x过采样，Normal模式 */
    i2c_write_reg8(BMP280_ADDR, BMP280_REG_CONFIG, 0xA0);      /* 500ms standby, IIR=16 */

    return true;
}

/* BMP280温压补偿计算（参考Bosch数据手册） */
static void bmp280_read_data(void)
{
    uint8_t data[6];
    uint8_t reg = BMP280_REG_PRESS_MSB;
    i2c_write_bytes(BMP280_ADDR, &reg, 1);
    i2c_read_bytes(BMP280_ADDR, data, 6);

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

    /* 温度补偿 */
    int32_t var1_t = ((((adc_T >> 3) - ((int32_t)g_bmp280_calib.dig_T1 << 1))) *
                      ((int32_t)g_bmp280_calib.dig_T2)) >> 11;
    int32_t var2_t = (((((adc_T >> 4) - ((int32_t)g_bmp280_calib.dig_T1)) *
                        ((adc_T >> 4) - ((int32_t)g_bmp280_calib.dig_T1))) >> 12) *
                      ((int32_t)g_bmp280_calib.dig_T3)) >> 14;
    int32_t t_fine = var1_t + var2_t;
    g_bmp280_data.temperature = (t_fine * 5 + 128) >> 8;
    g_bmp280_data.temperature /= 100.0f;

    /* 气压补偿 */
    int64_t var1_p = ((int64_t)t_fine) - 128000;
    int64_t var2_p = var1_p * var1_p * (int64_t)g_bmp280_calib.dig_P6;
    var2_p = var2_p + ((int64_t)(var1_p * (int64_t)g_bmp280_calib.dig_P5) << 17);
    var2_p = var2_p + (((int64_t)g_bmp280_calib.dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)g_bmp280_calib.dig_P3) >> 8) +
             ((var1_p * (int64_t)g_bmp280_calib.dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)g_bmp280_calib.dig_P1) >> 33;

    if (var1_p == 0) {
        g_bmp280_data.pressure = 0;
        return;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = ((int64_t)g_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13) >> 25;
    var2_p = ((int64_t)g_bmp280_calib.dig_P8) * p >> 19;
    p = ((p + var1_p + var2_p) >> 8) + (((int64_t)g_bmp280_calib.dig_P7) << 4);
    g_bmp280_data.pressure = (float)p / 256.0f;  /* Pa */

    /* 海拔计算（气压公式） */
    g_bmp280_data.altitude = 44330.0f * (1.0f - powf(g_bmp280_data.pressure / 101325.0f, 0.1903f));
}

/* ========== SHT30驱动 ========== */
static bool sht30_init(void)
{
    /* 软复位 */
    uint8_t cmd[2] = {(SHT30_CMD_SOFT_RESET >> 8) & 0xFF, SHT30_CMD_SOFT_RESET & 0xFF};
    i2c_write_bytes(SHT30_ADDR, cmd, 2);
    delay_cycles(10 * 32000);  /* 等待10ms */
    return true;
}

/* CRC-8校验（SHT30使用多项式0x31） */
static uint8_t sht30_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

static void sht30_read_data(void)
{
    /* 发送高精度测量命令 */
    uint8_t cmd[2] = {(SHT30_CMD_MEAS_HIGH >> 8) & 0xFF, SHT30_CMD_MEAS_HIGH & 0xFF};
    i2c_write_bytes(SHT30_ADDR, cmd, 2);

    /* 等待测量完成（高精度约15ms） */
    delay_cycles(20 * 32000);

    /* 读取6字节数据 */
    uint8_t data[6];
    i2c_read_bytes(SHT30_ADDR, data, 6);

    /* CRC校验 */
    if (sht30_crc8(data, 2) != data[2] || sht30_crc8(data + 3, 2) != data[5]) {
        return;  /* CRC错误，保留上次数据 */
    }

    /* 温度计算 */
    uint16_t raw_temp = (data[0] << 8) | data[1];
    g_sht30_data.temperature = -45.0f + 175.0f * (float)raw_temp / 65535.0f;

    /* 湿度计算 */
    uint16_t raw_humi = (data[3] << 8) | data[4];
    g_sht30_data.humidity = 100.0f * (float)raw_humi / 65535.0f;

    /* 湿度限幅 */
    if (g_sht30_data.humidity > 100.0f) g_sht30_data.humidity = 100.0f;
    if (g_sht30_data.humidity < 0.0f) g_sht30_data.humidity = 0.0f;
}

/* ========== 天气趋势预测 ========== */
/* 基于气压变化趋势预测天气：
 * - 气压稳定或上升 -> 晴天
 * - 气压缓慢下降（<1hPa/h）-> 多云
 * - 气压快速下降（>1hPa/h）-> 雨天
 */
static void update_weather_trend(float pressure)
{
    /* 记录气压历史（每5分钟一次） */
    static uint32_t trend_timer = 0;
    trend_timer++;

    /* 每5分钟（300秒）记录一次 */
    if (trend_timer >= 300) {
        trend_timer = 0;
        g_press_trend[g_trend_idx] = pressure / 100.0f;  /* 转为hPa */
        g_trend_idx = (g_trend_idx + 1) % 12;

        /* 至少需要2个数据点才能分析趋势 */
        if (g_history.count < 2) {
            g_weather_trend = TREND_UNKNOWN;
            return;
        }

        /* 计算最近1小时的气压变化 */
        uint8_t prev_idx = (g_trend_idx + 11) % 12;  /* 1小时前的索引 */
        float dp = g_press_trend[g_trend_idx] - g_press_trend[prev_idx];

        /* 趋势判断 */
        if (dp > 0.5f) {
            g_weather_trend = TREND_SUNNY;   /* 气压上升>0.5hPa -> 晴天 */
        } else if (dp > -1.0f) {
            g_weather_trend = TREND_CLOUDY;  /* 气压轻微下降 -> 多云 */
        } else {
            g_weather_trend = TREND_RAINY;   /* 气压快速下降 -> 雨天 */
        }
    }
}

/* ========== 历史数据记录 ========== */
static void record_history(void)
{
    static uint32_t record_timer = 0;
    record_timer++;

    /* 每5分钟记录一次 */
    if (record_timer >= 300) {
        record_timer = 0;

        g_history.temp_history[g_history.write_idx] = g_sht30_data.temperature;
        g_history.humi_history[g_history.write_idx] = g_sht30_data.humidity;
        g_history.press_history[g_history.write_idx] = g_bmp280_data.pressure / 100.0f;

        g_history.write_idx = (g_history.write_idx + 1) % HISTORY_SIZE;
        if (g_history.count < HISTORY_SIZE) {
            g_history.count++;
        }
    }
}

/* ========== 报警检测 ========== */
static void check_alarms(void)
{
    if (!g_alarm_enabled) {
        g_alarm_triggered = false;
        return;
    }

    g_alarm_triggered = false;

    /* 温度报警 */
    if (g_sht30_data.temperature > g_alarm.temp_high ||
        g_sht30_data.temperature < g_alarm.temp_low) {
        g_alarm_triggered = true;
    }

    /* 湿度报警 */
    if (g_sht30_data.humidity > g_alarm.humi_high ||
        g_sht30_data.humidity < g_alarm.humi_low) {
        g_alarm_triggered = true;
    }

    /* 气压报警 */
    float press_hpa = g_bmp280_data.pressure / 100.0f;
    if (press_hpa > g_alarm.press_high || press_hpa < g_alarm.press_low) {
        g_alarm_triggered = true;
    }
}

/* ========== OLED显示驱动（简化）========== */
static void oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_bytes(OLED_ADDR, buf, 2);
}

static void oled_init(void)
{
    oled_cmd(0xAE);
    oled_cmd(0xD5); oled_cmd(0x80);
    oled_cmd(0xA8); oled_cmd(0x3F);
    oled_cmd(0xD3); oled_cmd(0x00);
    oled_cmd(0x40);
    oled_cmd(0x8D); oled_cmd(0x14);
    oled_cmd(0x20); oled_cmd(0x02);
    oled_cmd(0xA1);
    oled_cmd(0xC8);
    oled_cmd(0xDA); oled_cmd(0x12);
    oled_cmd(0x81); oled_cmd(0xCF);
    oled_cmd(0xD9); oled_cmd(0xF1);
    oled_cmd(0xDB); oled_cmd(0x40);
    oled_cmd(0xA4);
    oled_cmd(0xA6);
    oled_cmd(0xAF);
}

static void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_cmd(0xB0 + page);
    oled_cmd(0x00 + (col & 0x0F));
    oled_cmd(0x10 + ((col >> 4) & 0x0F));
}

static void oled_clear(void)
{
    for (int page = 0; page < 8; page++) {
        oled_set_cursor(page, 0);
        for (int col = 0; col < 128; col++) {
            uint8_t buf[2] = {0x40, 0x00};
            i2c_write_bytes(OLED_ADDR, buf, 2);
        }
    }
}

/* 绘制简单柱状图（用于趋势显示） */
static void oled_draw_bar(uint8_t page, uint8_t col, uint8_t height)
{
    if (height > 40) height = 40;
    uint8_t full_pages = height / 8;
    uint8_t partial = height % 8;

    /* 填充完整页 */
    for (uint8_t p = 0; p < full_pages && (page - p) < 8; p++) {
        oled_set_cursor(page - p, col);
        uint8_t buf[2] = {0x40, 0xFF};
        i2c_write_bytes(OLED_ADDR, buf, 2);
    }
    /* 部分页 */
    if (full_pages < 5 && (page - full_pages) < 8) {
        oled_set_cursor(page - full_pages, col);
        uint8_t mask = (1 << partial) - 1;
        uint8_t buf[2] = {0x40, mask};
        i2c_write_bytes(OLED_ADDR, buf, 2);
    }
}

/* ========== 页面显示函数 ========== */
static void display_realtime(void)
{
    oled_clear();

    /* 页面0：实时数据
     * 第0行：BMP280 温度:xx.x°C
     * 第1行：SHT30  温度:xx.x°C
     * 第2行：SHT30  湿度:xx.x%RH
     * 第3行：气压:xxxx.xhPa
     * 第4行：海拔:xxxx.xm
     * 第5行：天气趋势:晴天/阴天/雨天
     * 第6行：报警状态
     * 第7行：采集计数
     */

    /* 实际项目中使用字库显示中文和数字 */
    oled_set_cursor(0, 0);
    /* BMP280温度 */
    (void)g_bmp280_data.temperature;

    oled_set_cursor(1, 0);
    /* SHT30温度 */
    (void)g_sht30_data.temperature;

    oled_set_cursor(2, 0);
    /* 湿度 */
    (void)g_sht30_data.humidity;

    oled_set_cursor(3, 0);
    /* 气压 */
    (void)g_bmp280_data.pressure;

    oled_set_cursor(4, 0);
    /* 海拔 */
    (void)g_bmp280_data.altitude;

    oled_set_cursor(5, 0);
    /* 天气趋势 */
    (void)trend_names[g_weather_trend];
}

static void display_trend(void)
{
    oled_clear();

    /* 页面1：趋势图
     * 第0行：标题
     * 第1-7行：柱状图显示最近12个气压数据点
     */
    oled_set_cursor(0, 0);
    /* "Pressure Trend" */

    /* 绘制气压趋势柱状图 */
    for (int i = 0; i < 12; i++) {
        uint8_t idx = (g_trend_idx + i) % 12;
        float p = g_press_trend[idx];
        /* 归一化到显示高度（假设范围950~1050hPa） */
        uint8_t height = 0;
        if (p > 0) {
            height = (uint8_t)((p - 950.0f) / 100.0f * 40.0f);
        }
        oled_draw_bar(7, i * 10 + 2, height);
    }
}

static void display_history(void)
{
    oled_clear();

    /* 页面2：历史记录（最近的12个数据点） */
    oled_set_cursor(0, 0);
    /* "History (24h)" */

    if (g_history.count == 0) {
        oled_set_cursor(3, 20);
        /* "No data yet" */
        return;
    }

    /* 显示温度历史曲线（简化柱状图） */
    for (int i = 0; i < 12; i++) {
        uint32_t idx = (g_history.write_idx + HISTORY_SIZE - 12 + i) % HISTORY_SIZE;
        if (idx < g_history.count) {
            float t = g_history.temp_history[idx];
            uint8_t height = (uint8_t)((t + 10.0f) / 60.0f * 40.0f);  /* -10~50°C */
            oled_draw_bar(7, i * 10 + 2, height);
        }
    }
}

static void display_alarm(void)
{
    oled_clear();

    /* 页面3：报警设置 */
    oled_set_cursor(0, 0);
    /* "Alarm Settings" */

    oled_set_cursor(1, 0);
    /* "T_H:xx.x T_L:xx.x" */

    oled_set_cursor(2, 0);
    /* "H_H:xx.x H_L:xx.x" */

    oled_set_cursor(3, 0);
    /* "P_H:xxxx P_L:xxxx" */

    oled_set_cursor(4, 0);
    /* "Status: ON/OFF" */

    oled_set_cursor(5, 0);
    /* "Triggered: YES/NO" */
}

/* ========== 按键扫描 ========== */
static uint8_t scan_buttons(void)
{
    static uint8_t last = 0xFF;
    uint8_t cur = 0;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_3)) cur |= 0x01;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_4)) cur |= 0x02;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_5)) cur |= 0x04;
    uint8_t pressed = (~cur) & last;
    last = cur;
    if (pressed & 0x01) return 1;
    if (pressed & 0x02) return 2;
    if (pressed & 0x04) return 3;
    return 0;
}

/* ========== 主函数 ========== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* 按键GPIO初始化 */
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_3,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_4,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_5,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 报警输出LED */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_8);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);

    /* 外设初始化 */
    if (!bmp280_init()) {
        /* BMP280初始化失败，LED快闪提示 */
        while (1) {
            DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_8);
            delay_cycles(100 * 32000);
        }
    }

    sht30_init();
    oled_init();
    oled_clear();

    /* 主循环 */
    while (1) {
        /* 每秒读取传感器数据 */
        static uint32_t sensor_timer = 0;
        sensor_timer++;

        if (sensor_timer >= 1000) {  /* 1秒 */
            sensor_timer = 0;

            /* 读取BMP280 */
            bmp280_read_data();

            /* 读取SHT30 */
            sht30_read_data();

            /* 更新天气趋势 */
            update_weather_trend(g_bmp280_data.pressure);

            /* 记录历史数据 */
            record_history();

            /* 报警检测 */
            check_alarms();

            /* 报警LED */
            if (g_alarm_triggered) {
                DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_8);
            } else {
                DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);
            }
        }

        /* 按键处理 */
        uint8_t btn = scan_buttons();
        switch (btn) {
            case 1:  /* 页面切换 */
                g_current_page = (DisplayPage_t)((g_current_page + 1) % PAGE_COUNT);
                break;

            case 2:  /* 历史查看（在历史页面内切换时间范围） */
                /* 可扩展：切换1h/6h/24h视图 */
                break;

            case 3:  /* 报警开关 */
                g_alarm_enabled = !g_alarm_enabled;
                break;
        }

        /* 显示更新（每200ms） */
        static uint32_t display_timer = 0;
        display_timer++;
        if (display_timer >= 200) {
            display_timer = 0;

            switch (g_current_page) {
                case PAGE_REALTIME:
                    display_realtime();
                    break;
                case PAGE_TREND:
                    display_trend();
                    break;
                case PAGE_HISTORY:
                    display_history();
                    break;
                case PAGE_ALARM:
                    display_alarm();
                    break;
                default:
                    break;
            }
        }

        g_tick++;
        delay_cycles(1000 * 32);  /* 1ms延时 */
    }
}
