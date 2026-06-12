/**
 * @file air_quality_monitor.c
 * @brief 空气质量监测 — SGP30 + OLED显示 + 蓝牙JSON上报
 * @target MSPM0G3507
 *
 * 硬件连接：
 *   SGP30 TVOC/eCO2传感器 (I2C):
 *     VCC -> 3.3V    GND -> GND
 *     SCL -> PB2 (I2C0_SCL)   SDA -> PB3 (I2C0_SDA)
 *   OLED SSD1306 (I2C, 共用总线):
 *     SCL -> PB2    SDA -> PB3
 *     地址0x3C
 *   蓝牙HC-05 (UART):
 *     TX -> PB7 (UART2_RX)   RX -> PB6 (UART2_TX)
 *   可选：蜂鸣器用于空气质量超标报警
 *     Buzzer -> PA8 (GPIO输出)
 *
 * 功能：
 *   1. SGP30传感器初始化并启动IAQ测量
 *   2. 每秒读取eCO2和TVOC数据
 *   3. OLED显示空气质量等级和数值
 *   4. 蓝牙每2秒上报JSON格式数据
 *   5. 空气质量超标时蜂鸣器报警
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ============ 外设驱动头文件 ============ */
#include "drivers/oled_ssd1306.h"
#include "drivers/i2c_master.h"
#include "drivers/uart_helper.h"

/* ============ 硬件配置 ============ */
#define SGP30_I2C       I2C0
#define OLED_I2C        I2C0
#define BT_UART         UART2

/* SGP30 I2C地址（固定） */
#define SGP30_ADDR      0x58

/* ============ SGP30命令定义 ============ */
#define SGP30_CMD_INIT_AIR_QUALITY      0x2003
#define SGP30_CMD_MEASURE_AIR_QUALITY   0x2008
#define SGP30_CMD_GET_BASELINE          0x2015
#define SGP30_CMD_SET_BASELINE          0x201E
#define SGP30_CMD_SET_HUMIDITY          0x2061
#define SGP30_CMD_MEASURE_RAW           0x2050
#define SGP30_CMD_GET_FEATURE_SET       0x202F
#define SGP30_CMD_MEASURE_TEST          0x2032
#define SGP30_CMD_SOFT_RESET            0x0006

/* ============ 空气质量等级定义 ============ */
typedef enum {
    AIR_EXCELLENT,   /* 优秀: eCO2 < 600 */
    AIR_GOOD,        /* 良好: eCO2 600-1000 */
    AIR_MODERATE,    /* 中等: eCO2 1000-1500 */
    AIR_POOR,        /* 较差: eCO2 1500-2500 */
    AIR_HAZARDOUS    /* 危险: eCO2 > 2500 */
} AirQuality_Level_t;

/* ============ SGP30数据结构 ============ */
typedef struct {
    uint16_t eco2;          /* 等效CO2浓度 (ppm)，范围400-60000 */
    uint16_t tvoc;          /* 总挥发性有机物 (ppb)，范围0-60000 */
    uint16_t raw_h2;        /* 原始H2信号 */
    uint16_t raw_ethanol;   /* 原始乙醇信号 */
    uint16_t feature_set;   /* 固件版本 */
    AirQuality_Level_t level; /* 空气质量等级 */
    bool valid;             /* 数据有效标志 */
} SGP30_Data_t;

/* ============ 全局变量 ============ */
static SGP30_Data_t g_air = {0};

/* ============ CRC-8校验 ============ */

/**
 * @brief SGP30使用的CRC-8校验（多项式0x31，初始值0xFF）
 * @param data 2字节数据
 * @return CRC-8校验值
 */
static uint8_t sgp30_crc8(const uint8_t *data)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/* ============ SGP30底层通信 ============ */

/**
 * @brief 向SGP30发送命令（2字节命令 + CRC校验由硬件完成）
 * @param cmd 16位命令
 */
static void sgp30_send_cmd(uint16_t cmd)
{
    uint8_t buf[2];
    buf[0] = (cmd >> 8) & 0xFF;
    buf[1] = cmd & 0xFF;
    i2c_master_write(SGP30_I2C, SGP30_ADDR, buf, 2);
}

/**
 * @brief 从SGP30读取数据（每组3字节：2字节数据+1字节CRC）
 * @param data 输出数据缓冲区
 * @param words 期望读取的字数（每字2字节+CRC=3字节）
 * @return true=所有CRC校验通过
 */
static bool sgp30_read_data(uint16_t *data, uint8_t words)
{
    uint8_t buf[3 * 3];  /* 最多读3个字 */
    uint8_t read_len = words * 3;

    i2c_master_read(SGP30_I2C, SGP30_ADDR, buf, read_len);

    for (int i = 0; i < words; i++) {
        uint8_t *ptr = &buf[i * 3];
        /* CRC校验 */
        if (sgp30_crc8(ptr) != ptr[2]) {
            return false;  /* CRC校验失败 */
        }
        data[i] = (ptr[0] << 8) | ptr[1];
    }
    return true;
}

/* ============ SGP30高级API ============ */

/**
 * @brief SGP30软件复位
 */
static void sgp30_soft_reset(void)
{
    uint8_t reset_cmd = 0x06;
    i2c_master_write(SGP30_I2C, 0x00, &reset_cmd, 1);  /* 广播地址0x00 */
    delay_ms(50);
}

/**
 * @brief 初始化SGP30空气质量监测
 *
 * 必须在上电后调用一次，启动IAQ（室内空气质量）算法
 * 注意：SGP30需要12小时运行才能达到最佳精度
 */
static bool sgp30_init(void)
{
    sgp30_soft_reset();
    delay_ms(50);

    /* 发送初始化空气品质命令 */
    sgp30_send_cmd(SGP30_CMD_INIT_AIR_QUALITY);
    delay_ms(10);

    /* 读取固件版本 */
    sgp30_send_cmd(SGP30_CMD_GET_FEATURE_SET);
    delay_ms(10);
    uint16_t feat[1];
    if (sgp30_read_data(feat, 1)) {
        g_air.feature_set = feat[0];
    }

    return true;
}

/**
 * @brief 读取SGP30空气质量数据
 *
 * 每秒调用一次（SGP30内部测量周期约1秒）
 * @return true=读取成功
 */
static bool sgp30_read_air_quality(void)
{
    uint16_t data[2];

    sgp30_send_cmd(SGP30_CMD_MEASURE_AIR_QUALITY);
    delay_ms(12);  /* 等待测量完成 */

    if (!sgp30_read_data(data, 2)) {
        return false;
    }

    g_air.eco2 = data[0];
    g_air.tvoc = data[1];
    g_air.valid = true;

    /* 判断空气质量等级 */
    if (g_air.eco2 < 600) {
        g_air.level = AIR_EXCELLENT;
    } else if (g_air.eco2 < 1000) {
        g_air.level = AIR_GOOD;
    } else if (g_air.eco2 < 1500) {
        g_air.level = AIR_MODERATE;
    } else if (g_air.eco2 < 2500) {
        g_air.level = AIR_POOR;
    } else {
        g_air.level = AIR_HAZARDOUS;
    }

    return true;
}

/**
 * @brief 读取SGP30原始信号值（可用于自定义算法）
 */
static bool sgp30_read_raw(void)
{
    uint16_t data[2];

    sgp30_send_cmd(SGP30_CMD_MEASURE_RAW);
    delay_ms(25);

    if (!sgp30_read_data(data, 2)) {
        return false;
    }

    g_air.raw_h2 = data[0];
    g_air.raw_ethanol = data[1];
    return true;
}

/**
 * @brief 获取SGP30基线值（可用于保存/恢复以加速初始化）
 * @param eco2_base 输出eCO2基线
 * @param tvoc_base 输出TVOC基线
 */
static bool sgp30_get_baseline(uint16_t *eco2_base, uint16_t *tvoc_base)
{
    uint16_t data[2];

    sgp30_send_cmd(SGP30_CMD_GET_BASELINE);
    delay_ms(10);

    if (!sgp30_read_data(data, 2)) {
        return false;
    }

    *tvoc_base = data[0];
    *eco2_base = data[1];
    return true;
}

/**
 * @brief 设置SGP30基线值（恢复之前保存的基线）
 * @param eco2_base eCO2基线
 * @param tvoc_base TVOC基线
 */
static void sgp30_set_baseline(uint16_t eco2_base, uint16_t tvoc_base)
{
    uint8_t buf[8];

    buf[0] = (SGP30_CMD_SET_BASELINE >> 8) & 0xFF;
    buf[1] = SGP30_CMD_SET_BASELINE & 0xFF;
    buf[2] = (tvoc_base >> 8) & 0xFF;
    buf[3] = tvoc_base & 0xFF;
    buf[4] = sgp30_crc8(&buf[2]);
    buf[5] = (eco2_base >> 8) & 0xFF;
    buf[6] = eco2_base & 0xFF;
    buf[7] = sgp30_crc8(&buf[5]);

    i2c_master_write(SGP30_I2C, SGP30_ADDR, buf, 8);
    delay_ms(10);
}

/**
 * @brief 设置湿度补偿（提高SGP30测量精度）
 * @param temperature 温度（°C）
 * @param humidity 相对湿度（%RH）
 */
static void sgp30_set_humidity(float temperature, float humidity)
{
    /* 绝对湿度计算（g/m³） */
    float abs_humidity = 216.7f * ((humidity / 100.0f) * 6.112f *
        expf((17.62f * temperature) / (243.12f + temperature)) /
        (273.15f + temperature));

    /* 转换为SGP30格式：8.8定点数 */
    uint16_t hum_raw = (uint16_t)(abs_humidity * 256.0f);

    uint8_t buf[5];
    buf[0] = (SGP30_CMD_SET_HUMIDITY >> 8) & 0xFF;
    buf[1] = SGP30_CMD_SET_HUMIDITY & 0xFF;
    buf[2] = (hum_raw >> 8) & 0xFF;
    buf[3] = hum_raw & 0xFF;
    buf[4] = sgp30_crc8(&buf[2]);

    i2c_master_write(SGP30_I2C, SGP30_ADDR, buf, 5);
    delay_ms(10);
}

/* ============ OLED显示 ============ */

/**
 * @brief 获取空气质量等级的文字描述
 */
static const char* get_air_level_str(AirQuality_Level_t level)
{
    switch (level) {
        case AIR_EXCELLENT: return "Excellent";
        case AIR_GOOD:      return "Good";
        case AIR_MODERATE:  return "Moderate";
        case AIR_POOR:      return "Poor";
        case AIR_HAZARDOUS: return "HAZARDOUS";
        default:            return "Unknown";
    }
}

/**
 * @brief OLED显示空气质量信息
 *
 * 显示布局：
 *   行0: 标题
 *   行1: eCO2数值
 *   行2: TVOC数值
 *   行3: 空气质量等级
 *   行4: 进度条（eCO2范围映射）
 *   行5-6: 建议信息
 */
static void oled_display_air_quality(void)
{
    char line[32];

    oled_clear();

    /* 标题 */
    oled_show_string(0, 0, "Air Quality Mon.", FONT_16);

    /* eCO2 */
    snprintf(line, sizeof(line), "eCO2: %u ppm", g_air.eco2);
    oled_show_string(0, 2, line, FONT_12);

    /* TVOC */
    snprintf(line, sizeof(line), "TVOC: %u ppb", g_air.tvoc);
    oled_show_string(0, 3, line, FONT_12);

    /* 等级 */
    snprintf(line, sizeof(line), "Level: %s", get_air_level_str(g_air.level));
    oled_show_string(0, 4, line, FONT_12);

    /* 进度条：eCO2映射到0-128像素（400-2500ppm） */
    int bar_len = (int)((g_air.eco2 - 400) * 128.0f / 2100.0f);
    if (bar_len < 0) bar_len = 0;
    if (bar_len > 128) bar_len = 128;

    /* 绘制进度条框 */
    oled_draw_rect(0, 48, 127, 55);
    /* 填充进度 */
    oled_fill_rect(1, 49, bar_len - 2, 54);

    /* 建议 */
    if (g_air.level <= AIR_GOOD) {
        oled_show_string(0, 7, "Air is good :)", FONT_12);
    } else if (g_air.level == AIR_MODERATE) {
        oled_show_string(0, 7, "Open window!", FONT_12);
    } else {
        oled_show_string(0, 7, "Ventilate NOW!", FONT_12);
    }

    oled_refresh();
}

/* ============ 蓝牙上报 ============ */

/**
 * @brief 蓝牙上报空气质量数据（JSON格式）
 *
 * 上报格式：
 * {"eco2":850,"tvoc":120,"level":"good","raw_h2":13456,"raw_eth":18234}
 */
static void bt_report_air_quality(void)
{
    char json[256];
    int len;

    len = snprintf(json, sizeof(json),
        "{\"eco2\":%u,"
        "\"tvoc\":%u,"
        "\"level\":\"%s\","
        "\"raw_h2\":%u,"
        "\"raw_eth\":%u}\r\n",
        g_air.eco2,
        g_air.tvoc,
        get_air_level_str(g_air.level),
        g_air.raw_h2,
        g_air.raw_ethanol
    );

    uart_write_string(BT_UART, json, len);
}

/* ============ 报警控制 ============ */

/**
 * @brief 空气质量超标报警
 * @param level 当前空气质量等级
 */
static void air_quality_alarm(AirQuality_Level_t level)
{
    if (level >= AIR_POOR) {
        /* 连续蜂鸣 */
        for (int i = 0; i < 3; i++) {
            DL_GPIO_setPins(GPIO_BUZZER_PORT, GPIO_BUZZER_PIN);
            delay_ms(100);
            DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_PIN);
            delay_ms(100);
        }
    } else if (level == AIR_MODERATE) {
        /* 短促提示一声 */
        DL_GPIO_setPins(GPIO_BUZZER_PORT, GPIO_BUZZER_PIN);
        delay_ms(50);
        DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_PIN);
    }
    /* 优秀/良好不报警 */
}

/* ============ 主函数 ============ */
int main(void)
{
    /* 系统初始化 */
    DL_SYSCTL_init();
    SysTick_Config(DL_SYSCTL_getMCLKFreq() / 1000);

    /* I2C初始化 */
    i2c_master_init(SGP30_I2C);

    /* UART初始化（蓝牙） */
    uart_init(BT_UART, 9600);

    /* GPIO初始化（蜂鸣器） */
    DL_GPIO_initDigitalOutput(GPIO_BUZZER_PIN);
    DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_PIN);

    /* OLED初始化 */
    oled_init();
    oled_clear();
    oled_show_string(0, 0, "Air Quality Mon.", FONT_16);
    oled_show_string(0, 2, "SGP30 Sensor", FONT_12);
    oled_show_string(0, 3, "Initializing...", FONT_12);
    oled_refresh();

    /* SGP30初始化 */
    sgp30_init();
    delay_ms(1000);

    /* 上电后先丢弃前15秒数据（SGP30预热） */
    oled_clear();
    oled_show_string(0, 3, "Warming up...", FONT_12);
    oled_refresh();
    for (int i = 0; i < 15; i++) {
        sgp30_read_air_quality();
        delay_ms(1000);
    }

    /* 定时器变量 */
    uint32_t last_read_ms = 0;
    uint32_t last_report_ms = 0;
    uint8_t raw_read_cnt = 0;

    /* 主循环 */
    while (1) {
        uint32_t now_ms = get_tick();

        /* 每1秒读取一次SGP30 */
        if (now_ms - last_read_ms >= 1000) {
            last_read_ms = now_ms;

            if (sgp30_read_air_quality()) {
                /* 每5次读一次原始数据 */
                raw_read_cnt++;
                if (raw_read_cnt >= 5) {
                    raw_read_cnt = 0;
                    sgp30_read_raw();
                }

                /* 更新显示 */
                oled_display_air_quality();

                /* 空气质量报警 */
                air_quality_alarm(g_air.level);
            }
        }

        /* 每2秒蓝牙上报 */
        if (now_ms - last_report_ms >= 2000) {
            last_report_ms = now_ms;
            if (g_air.valid) {
                bt_report_air_quality();
            }
        }
    }

    return 0;
}
