/**
 * @file    temperature_monitor.c
 * @brief   温度监控系统完整示例 — MSPM0G3507
 *
 * 功能概述:
 *   1. SHT20温湿度传感器读取 (I2C)
 *   2. OLED实时显示温度/湿度
 *   3. 阈值报警: 超温/低温/高湿LED闪烁+蜂鸣器
 *   4. 温度历史曲线 (OLED简易图表)
 *   5. 蓝牙远程查看数据
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │ SHT20 温湿度传感器 (I2C):                                │
 * │   MSPM0 PB2(SCL) → SHT20 SCL                           │
 * │   MSPM0 PB3(SDA) → SHT20 SDA                           │
 * │   I2C地址: 0x40                                          │
 * │   注意: 与OLED共享I2C总线, 地址不同即可共存               │
 * │                                                          │
 * │ OLED (I2C0):                                             │
 * │   MSPM0 PB2(SCL) → OLED SCL (地址0x3C)                  │
 * │   MSPM0 PB3(SDA) → OLED SDA                             │
 * │                                                          │
 * │ 报警指示:                                                │
 * │   MSPM0 PA22 → LED (高温报警)                            │
 * │   MSPM0 PA23 → 蜂鸣器 (可选)                             │
 * │                                                          │
 * │ HC-05 蓝牙 (可选):                                       │
 * │   MSPM0 PA17(U1TX) → HC-05 RX                          │
 * │   MSPM0 PA18(U1RX) ← HC-05 TX                          │
 * └──────────────────────────────────────────────────────────┘
 *
 * SHT20 I2C协议:
 *   温度测量: 触发命令 0xF3 (Hold Master模式)
 *   湿度测量: 触发命令 0xF5 (Hold Master模式)
 *   软复位:   命令 0xFE
 *   数据格式: 16位原始值, 需转换
 *
 * 依赖驱动: oled_ssd1306_mspm0, i2c_bus, bluetooth_hc05_mspm0, pin_config
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/oled_ssd1306_mspm0.h"
#include "drivers/i2c_bus.h"
#include "drivers/bluetooth_hc05_mspm0.h"
#include "drivers/pin_config.h"

/* ══════════════════════════════════════════════════════════════
 *  SHT20 配置
 * ══════════════════════════════════════════════════════════════ */

#define SHT20_I2C_ADDR      0x40    /* SHT20 I2C地址 */

/* SHT20 命令 */
#define SHT20_CMD_MEAS_TEMP 0xF3    /* 触发温度测量 (Hold Master) */
#define SHT20_CMD_MEAS_HUMI 0xF5    /* 触发湿度测量 (Hold Master) */
#define SHT20_CMD_SOFT_RST  0xFE    /* 软复位 */

/* 测量等待时间 (ms) */
#define SHT20_TEMP_WAIT_MS  85      /* 温度测量最大等待 (14bit) */
#define SHT20_HUMI_WAIT_MS  29      /* 湿度测量最大等待 (12bit) */

/* I2C超时 (循环计数) — 错误经验#6: 必须有超时 */
#define I2C_TIMEOUT         100000U

/* ══════════════════════════════════════════════════════════════
 *  报警阈值
 * ══════════════════════════════════════════════════════════════ */

#define TEMP_HIGH_THRESH    35.0f   /* 高温报警 (°C) */
#define TEMP_LOW_THRESH     5.0f    /* 低温报警 (°C) */
#define HUMI_HIGH_THRESH    80.0f   /* 高湿报警 (%RH) */

/* 温度历史记录数量 (用于图表) */
#define TEMP_HISTORY_SIZE   64

/* 采样间隔 (ms) */
#define SAMPLE_INTERVAL_MS  1000

/* OLED刷新间隔 (ms) */
#define OLED_REFRESH_MS     500

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

/* I2C总线句柄 */
static I2C_Bus g_i2c_bus;

/* 测量数据 */
static volatile float g_temperature = 0.0f;
static volatile float g_humidity = 0.0f;

/* 报警状态 */
static volatile uint8_t g_alarm_temp_high = 0;
static volatile uint8_t g_alarm_temp_low = 0;
static volatile uint8_t g_alarm_humi_high = 0;

/* 温度历史 (用于图表) */
static float g_temp_history[TEMP_HISTORY_SIZE];
static uint8_t g_temp_hist_idx = 0;

/* 计时 */
static volatile uint32_t g_sys_tick = 0;
static uint32_t g_last_sample_tick = 0;
static uint32_t g_last_oled_tick = 0;

/* SHT20 CRC校验多项式: x^8 + x^5 + x^4 + 1 */
/* 错误经验#7: CRC查找表占256字节RAM，使用计算替代 */
static uint8_t SHT20_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ══════════════════════════════════════════════════════════════
 *  SHT20 驱动
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief SHT20 软复位
 * @return 0=成功, 负值=错误
 */
static int SHT20_SoftReset(void)
{
    uint8_t cmd = SHT20_CMD_SOFT_RST;
    int ret = I2C_Bus_Write(&g_i2c_bus, SHT20_I2C_ADDR, &cmd, 1);
    if (ret != I2C_BUS_OK) return ret;
    delay_cycles(32000);    /* 等待复位完成 (~1ms) */
    return 0;
}

/**
 * @brief SHT20 读取温度
 * @param temperature 输出温度值 (°C)
 * @return 0=成功, 负值=错误
 *
 * 数据格式:
 *   16位原始值, 高字节在前
 *   温度 = -46.85 + 175.72 * raw / 65536
 *   低2位为状态位, 需屏蔽
 */
static int SHT20_ReadTemperature(float *temperature)
{
    uint8_t cmd = SHT20_CMD_MEAS_TEMP;
    uint8_t rx[3];      /* 2字节数据 + 1字节CRC */
    int ret;

    /* 发送测量命令 */
    ret = I2C_Bus_Write(&g_i2c_bus, SHT20_I2C_ADDR, &cmd, 1);
    if (ret != I2C_BUS_OK) return ret;

    /* 等待测量完成 */
    delay_cycles(SHT20_TEMP_WAIT_MS * 32000);

    /* 读取数据 */
    ret = I2C_Bus_Read(&g_i2c_bus, SHT20_I2C_ADDR, rx, 3);
    if (ret != I2C_BUS_OK) return ret;

    /* CRC校验 */
    if (SHT20_CRC8(rx, 2) != rx[2]) {
        return -10;     /* CRC错误 */
    }

    /* 解析温度值 */
    uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];
    raw &= 0xFFFC;     /* 屏蔽低2位状态位 */

    /* 错误经验#1: 常量65536.0f无除零风险 */
    *temperature = -46.85f + 175.72f * (float)raw / 65536.0f;

    return 0;
}

/**
 * @brief SHT20 读取湿度
 * @param humidity 输出湿度值 (%RH)
 * @return 0=成功, 负值=错误
 *
 * 数据格式:
 *   湿度 = -6.0 + 125.0 * raw / 65536
 */
static int SHT20_ReadHumidity(float *humidity)
{
    uint8_t cmd = SHT20_CMD_MEAS_HUMI;
    uint8_t rx[3];
    int ret;

    ret = I2C_Bus_Write(&g_i2c_bus, SHT20_I2C_ADDR, &cmd, 1);
    if (ret != I2C_BUS_OK) return ret;

    delay_cycles(SHT20_HUMI_WAIT_MS * 32000);

    ret = I2C_Bus_Read(&g_i2c_bus, SHT20_I2C_ADDR, rx, 3);
    if (ret != I2C_BUS_OK) return ret;

    if (SHT20_CRC8(rx, 2) != rx[2]) {
        return -10;
    }

    uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];
    raw &= 0xFFFC;

    /* 错误经验#1: 常量65536.0f无除零风险 */
    *humidity = -6.0f + 125.0f * (float)raw / 65536.0f;

    /* 湿度限幅 (物理范围0~100%) */
    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity < 0.0f)   *humidity = 0.0f;

    return 0;
}

/**
 * @brief 采样温湿度
 */
static void SHT20_Sample(void)
{
    float temp, humi;
    int ret;

    ret = SHT20_ReadTemperature(&temp);
    if (ret == 0) {
        g_temperature = temp;
    }

    ret = SHT20_ReadHumidity(&humi);
    if (ret == 0) {
        g_humidity = humi;
    }

    /* 记录温度历史 */
    g_temp_history[g_temp_hist_idx] = g_temperature;
    g_temp_hist_idx = (g_temp_hist_idx + 1) % TEMP_HISTORY_SIZE;
}

/* ══════════════════════════════════════════════════════════════
 *  报警处理
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 检查阈值并控制报警
 */
static void Alarm_Check(void)
{
    /* 高温报警 */
    g_alarm_temp_high = (g_temperature > TEMP_HIGH_THRESH) ? 1 : 0;

    /* 低温报警 */
    g_alarm_temp_low = (g_temperature < TEMP_LOW_THRESH) ? 1 : 0;

    /* 高湿报警 */
    g_alarm_humi_high = (g_humidity > HUMI_HIGH_THRESH) ? 1 : 0;

    /* LED指示 (PA22): 任何报警都亮 */
    if (g_alarm_temp_high || g_alarm_temp_low || g_alarm_humi_high) {
        /* 闪烁效果: 每500ms翻转 */
        if ((g_sys_tick / 500) & 1) {
            DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_22);
        } else {
            DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_22);
        }
    } else {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_22);
    }
}

/* ══════════════════════════════════════════════════════════════
 *  OLED显示
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 绘制温度历史曲线 (简易版)
 * @param x      起始X坐标
 * @param y      起始Y坐标
 * @param width  图表宽度
 * @param height 图表高度
 */
static void OLED_DrawTempCurve(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    /* 找温度范围 */
    float temp_min = g_temp_history[0];
    float temp_max = g_temp_history[0];

    for (uint8_t i = 1; i < TEMP_HISTORY_SIZE; i++) {
        if (g_temp_history[i] < temp_min) temp_min = g_temp_history[i];
        if (g_temp_history[i] > temp_max) temp_max = g_temp_history[i];
    }

    /* 错误经验#1: 防止除零 */
    float temp_range = temp_max - temp_min;
    if (temp_range < 0.1f) temp_range = 0.1f;

    /* 绘制曲线 */
    for (uint8_t i = 0; i < width && i < TEMP_HISTORY_SIZE; i++) {
        uint8_t idx = (g_temp_hist_idx + i) % TEMP_HISTORY_SIZE;
        float normalized = (g_temp_history[idx] - temp_min) / temp_range;
        /* 错误经验#7: 归一化值在0~1之间，乘以高度不会越界 */
        uint8_t dot_y = y + height - 1 - (uint8_t)(normalized * (float)(height - 1));

        if (dot_y >= y && dot_y < (y + height)) {
            OLED_DrawPoint(x + i, dot_y, 1);
        }
    }
}

static void OLED_UpdateDisplay(void)
{
    OLED_Clear();

    /* 第1行: 温度 */
    OLED_ShowString(0, 0, (char *)"Temp:", 12, 1);
    OLED_ShowFloat(42, 0, g_temperature, 2, 1, 12, 1);
    OLED_ShowString(84, 0, (char *)"C", 12, 1);
    if (g_alarm_temp_high) {
        OLED_ShowString(96, 0, (char *)"!H!", 12, 1);
    } else if (g_alarm_temp_low) {
        OLED_ShowString(96, 0, (char *)"!L!", 12, 1);
    }

    /* 第2行: 湿度 */
    OLED_ShowString(0, 16, (char *)"Humi:", 12, 1);
    OLED_ShowFloat(42, 16, g_humidity, 2, 1, 12, 1);
    OLED_ShowString(84, 16, (char *)"%RH", 12, 1);
    if (g_alarm_humi_high) {
        OLED_ShowString(110, 16, (char *)"!", 12, 1);
    }

    /* 第3-4行: 温度曲线 */
    OLED_DrawTempCurve(0, 32, 128, 32);

    OLED_Refresh();
}

/* ══════════════════════════════════════════════════════════════
 *  蓝牙数据发送
 * ══════════════════════════════════════════════════════════════ */

static void BT_SendData(void)
{
    if (!BT_HC05_IsConnected()) return;

    char msg[64];
    snprintf(msg, sizeof(msg), "T:%.1f H:%.1f %s\r\n",
             g_temperature, g_humidity,
             (g_alarm_temp_high ? "ALARM_H" :
              g_alarm_temp_low  ? "ALARM_L" :
              g_alarm_humi_high ? "ALARM_HU" : "OK"));
    BT_HC05_SendString(msg);
}

/* ══════════════════════════════════════════════════════════════
 *  系统初始化
 * ══════════════════════════════════════════════════════════════ */

static void System_Init(void)
{
    SYSCFG_DL_init();

    /* 初始化I2C总线管理层 (含超时保护) */
    I2C_Bus_Init(&g_i2c_bus, I2C_0_INST);

    /* 初始化OLED */
    OLED_Init(I2C_0_INST);
    OLED_Clear();
    OLED_ShowString(10, 24, (char *)"Temp Monitor", 16, 1);
    OLED_Refresh();

    /* 初始化蓝牙 (可选) */
    BT_HC05_Init(&(BT_HC05_Config){
        .uart = UART_1_INST,
        .state_port = PIN_BT_EN_PORT,
        .state_pin  = PIN_BT_EN_PIN,
        .baudrate   = 9600
    });

    /* SHT20软复位 */
    SHT20_SoftReset();

    /* LED报警引脚 */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_22);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_22);

    /* 初始化温度历史 */
    for (uint8_t i = 0; i < TEMP_HISTORY_SIZE; i++) {
        g_temp_history[i] = 25.0f;  /* 默认值 */
    }

    delay_cycles(16000000);  /* 等待传感器稳定 */
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */

int main(void)
{
    System_Init();

    while (1) {
        /* 1. 定时采样温湿度 */
        if ((g_sys_tick - g_last_sample_tick) >= SAMPLE_INTERVAL_MS) {
            g_last_sample_tick = g_sys_tick;
            SHT20_Sample();
            Alarm_Check();
            BT_SendData();
        }

        /* 2. 定时刷新OLED */
        if ((g_sys_tick - g_last_oled_tick) >= OLED_REFRESH_MS) {
            g_last_oled_tick = g_sys_tick;
            OLED_UpdateDisplay();
        }

        /* 3. 主循环延时 ~10ms */
        delay_cycles(320000);
        g_sys_tick += 10;
    }
}
