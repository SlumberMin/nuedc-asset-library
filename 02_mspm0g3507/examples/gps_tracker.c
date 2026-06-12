/**
 * @file gps_tracker.c
 * @brief GPS追踪器 — NEO-6M + OLED显示经纬度 + 蓝牙上报
 * @target MSPM0G3507
 *
 * 硬件连接：
 *   NEO-6M GPS模块：
 *     VCC -> 3.3V    GND -> GND
 *     TX  -> PA1 (UART1_RX)   RX -> PA0 (UART1_TX)
 *   OLED SSD1306 (I2C):
 *     SCL -> PB2 (I2C0_SCL)   SDA -> PB3 (I2C0_SDA)
 *   蓝牙HC-05/HC-06：
 *     TX  -> PB7 (UART2_RX)   RX -> PB6 (UART2_TX)
 *
 * 功能：
 *   1. 解析NMEA语句获取经纬度、速度、卫星数
 *   2. OLED实时显示定位信息
 *   3. 蓝牙每秒上报JSON格式定位数据
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ============ 外设驱动头文件 ============ */
#include "drivers/oled_ssd1306.h"
#include "drivers/i2c_master.h"
#include "drivers/uart_helper.h"

/* ============ 引脚定义 ============ */
#define GPS_UART        UART1
#define BT_UART         UART2
#define OLED_I2C        I2C0

/* ============ 缓冲区大小 ============ */
#define NMEA_BUF_SIZE   256
#define BT_BUF_SIZE     256

/* ============ GPS数据结构 ============ */
typedef struct {
    double latitude;        /* 纬度，单位：度 */
    double longitude;       /* 经度，单位：度 */
    float  altitude;        /* 海拔，单位：米 */
    float  speed_knots;     /* 速度，单位：节 */
    float  speed_kmh;       /* 速度，单位：千米/时 */
    uint8_t fix_quality;    /* 定位质量：0=无效, 1=GPS, 2=DGPS */
    uint8_t num_satellites; /* 可见卫星数 */
    char   lat_dir;         /* N/S */
    char   lon_dir;         /* E/W */
    char   utc_time[12];    /* UTC时间 HH:MM:SS */
    char   utc_date[8];     /* UTC日期 DDMMYY */
    bool   valid;           /* 数据有效标志 */
} GPS_Data_t;

/* ============ 全局变量 ============ */
static GPS_Data_t g_gps = {0};
static char g_nmea_buf[NMEA_BUF_SIZE];
static volatile uint16_t g_nmea_idx = 0;
static volatile bool g_nmea_ready = false;

/* ============ NMEA解析辅助函数 ============ */

/**
 * @brief 将NMEA格式的ddmm.mmmm转换为十进制度数
 * @param nmea_coord NMEA原始坐标值
 * @param dir 方向 N/S/E/W
 * @return 十进制度数
 */
static double nmea_to_decimal(double nmea_coord, char dir)
{
    int degrees = (int)(nmea_coord / 100.0);
    double minutes = nmea_coord - (degrees * 100.0);
    double decimal = degrees + minutes / 60.0;
    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

/**
 * @brief 计算NMEA校验和
 * @param sentence NMEA语句（不含$和*）
 * @return 校验和
 */
static uint8_t nmea_checksum(const char *sentence)
{
    uint8_t checksum = 0;
    while (*sentence && *sentence != '*') {
        checksum ^= (uint8_t)*sentence++;
    }
    return checksum;
}

/**
 * @brief 解析GPGGA语句（定位数据）
 * @param sentence 完整NMEA语句
 *
 * GPGGA格式:
 * $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 *         时间    纬度  N/S  经度   E/W 质量 卫星 精度 海拔     修正
 */
static void parse_gpgga(const char *sentence)
{
    char buf[NMEA_BUF_SIZE];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    /* 跳过 '$' */
    char *p = buf;
    if (*p == '$') p++;

    char *fields[15];
    int field_count = 0;
    fields[field_count++] = p;

    /* 按逗号分割字段 */
    while (*p && field_count < 15) {
        if (*p == ',') {
            *p = '\0';
            fields[field_count++] = p + 1;
        }
        p++;
    }

    /* 至少需要10个字段 */
    if (field_count < 10) return;

    /* 字段1: UTC时间 */
    if (strlen(fields[1]) >= 6) {
        strncpy(g_gps.utc_time, fields[1], 6);
        g_gps.utc_time[6] = '\0';
        /* 格式化为 HH:MM:SS */
        memmove(g_gps.utc_time + 3, g_gps.utc_time + 2, 5);
        g_gps.utc_time[2] = ':';
        memmove(g_gps.utc_time + 6, g_gps.utc_time + 5, 3);
        g_gps.utc_time[5] = ':';
    }

    /* 字段2-3: 纬度 */
    if (strlen(fields[2]) > 0 && strlen(fields[3]) > 0) {
        double raw_lat = atof(fields[2]);
        g_gps.lat_dir = fields[3][0];
        g_gps.latitude = nmea_to_decimal(raw_lat, g_gps.lat_dir);
    }

    /* 字段4-5: 经度 */
    if (strlen(fields[4]) > 0 && strlen(fields[5]) > 0) {
        double raw_lon = atof(fields[4]);
        g_gps.lon_dir = fields[5][0];
        g_gps.longitude = nmea_to_decimal(raw_lon, g_gps.lon_dir);
    }

    /* 字段6: 定位质量 */
    if (strlen(fields[6]) > 0) {
        g_gps.fix_quality = atoi(fields[6]);
    }

    /* 字段7: 卫星数 */
    if (strlen(fields[7]) > 0) {
        g_gps.num_satellites = atoi(fields[7]);
    }

    /* 字段9: 海拔 */
    if (strlen(fields[9]) > 0) {
        g_gps.altitude = atof(fields[9]);
    }

    g_gps.valid = (g_gps.fix_quality > 0);
}

/**
 * @brief 解析GPRMC语句（推荐最小定位信息，获取速度和日期）
 *
 * GPRMC格式:
 * $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmmyy,x.x,a*hh
 *         时间  状态 纬度  N/S 经度  E/W 速度 航向  日期   磁偏
 */
static void parse_gprmc(const char *sentence)
{
    char buf[NMEA_BUF_SIZE];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *p = buf;
    if (*p == '$') p++;

    char *fields[13];
    int field_count = 0;
    fields[field_count++] = p;

    while (*p && field_count < 13) {
        if (*p == ',') {
            *p = '\0';
            fields[field_count++] = p + 1;
        }
        p++;
    }

    if (field_count < 10) return;

    /* 字段2: 状态 A=有效 V=无效 */
    if (strlen(fields[2]) > 0 && fields[2][0] != 'A') {
        g_gps.valid = false;
        return;
    }

    /* 字段7: 地面速度（节） */
    if (strlen(fields[7]) > 0) {
        g_gps.speed_knots = atof(fields[7]);
        g_gps.speed_kmh = g_gps.speed_knots * 1.852f;  /* 节 -> km/h */
    }

    /* 字段9: 日期 DDMMYY */
    if (strlen(fields[9]) >= 6) {
        strncpy(g_gps.utc_date, fields[9], 6);
        g_gps.utc_date[6] = '\0';
    }
}

/**
 * @brief 处理一条完整的NMEA语句
 * @param sentence 以'\0'结尾的NMEA语句
 */
static void process_nmea_sentence(const char *sentence)
{
    /* 验证校验和 */
    const char *star = strchr(sentence + 1, '*');
    if (star && *(star + 1) && *(star + 2)) {
        uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
        uint8_t actual = nmea_checksum(sentence + 1);
        if (expected != actual) return; /* 校验失败，丢弃 */
    }

    /* 根据语句类型分发解析 */
    if (strstr(sentence, "GGA") != NULL) {
        parse_gpgga(sentence);
    } else if (strstr(sentence, "RMC") != NULL) {
        parse_gprmc(sentence);
    }
}

/**
 * @brief 从串口缓冲区中提取NMEA语句
 *        NMEA以'$'开头，以'\r\n'结尾
 */
static void extract_nmea_from_uart(void)
{
    static char line_buf[NMEA_BUF_SIZE];
    static uint16_t line_idx = 0;

    uint8_t ch;
    while (uart_read_byte(GPS_UART, &ch)) {
        if (ch == '$') {
            line_idx = 0;
            line_buf[line_idx++] = (char)ch;
        } else if (line_idx > 0 && line_idx < NMEA_BUF_SIZE - 1) {
            line_buf[line_idx++] = (char)ch;
            if (ch == '\n' || ch == '\r') {
                line_buf[line_idx] = '\0';
                /* 处理完整的NMEA语句 */
                if (line_idx > 6) {
                    process_nmea_sentence(line_buf);
                }
                line_idx = 0;
            }
        }
    }
}

/**
 * @brief 格式化并显示GPS信息到OLED
 */
static void oled_display_gps(void)
{
    char line[32];

    oled_clear();

    /* 第1行: 标题 */
    oled_show_string(0, 0, "GPS Tracker", FONT_16);

    /* 第2行: UTC时间 */
    snprintf(line, sizeof(line), "UTC: %s", g_gps.utc_time);
    oled_show_string(0, 2, line, FONT_12);

    /* 第3行: 纬度 */
    snprintf(line, sizeof(line), "Lat: %.6f %c", fabs(g_gps.latitude), g_gps.lat_dir);
    oled_show_string(0, 3, line, FONT_12);

    /* 第4行: 经度 */
    snprintf(line, sizeof(line), "Lon: %.6f %c", fabs(g_gps.longitude), g_gps.lon_dir);
    oled_show_string(0, 4, line, FONT_12);

    /* 第5行: 海拔和卫星数 */
    snprintf(line, sizeof(line), "Alt:%.1fm Sat:%d", g_gps.altitude, g_gps.num_satellites);
    oled_show_string(0, 5, line, FONT_12);

    /* 第6行: 速度 */
    snprintf(line, sizeof(line), "Spd: %.1f km/h", g_gps.speed_kmh);
    oled_show_string(0, 6, line, FONT_12);

    /* 第7行: 定位状态 */
    if (g_gps.valid) {
        oled_show_string(0, 7, "Status: FIX OK", FONT_12);
    } else {
        oled_show_string(0, 7, "Status: NO FIX", FONT_12);
    }

    oled_refresh();
}

/**
 * @brief 蓝牙上报GPS数据（JSON格式）
 *
 * 上报格式示例：
 * {"t":"12:34:56","lat":31.234567,"lon":121.567890,"alt":45.2,"spd":60.5,"sat":8,"fix":1}
 */
static void bt_report_gps(void)
{
    char json[BT_BUF_SIZE];
    int len;

    len = snprintf(json, sizeof(json),
        "{\"t\":\"%s\","
        "\"lat\":%.6f,"
        "\"lon\":%.6f,"
        "\"alt\":%.1f,"
        "\"spd\":%.1f,"
        "\"sat\":%d,"
        "\"fix\":%d}\r\n",
        g_gps.utc_time,
        g_gps.latitude,
        g_gps.longitude,
        g_gps.altitude,
        g_gps.speed_kmh,
        g_gps.num_satellites,
        g_gps.fix_quality
    );

    uart_write_string(BT_UART, json, len);
}

/* ============ 主函数 ============ */
int main(void)
{
    /* 系统初始化 */
    DL_SYSCTL_init();
    SysTick_Config(DL_SYSCTL_getMCLKFreq() / 1000);  /* 1ms SysTick */

    /* UART初始化 */
    uart_init(GPS_UART, 9600);     /* NEO-6M默认波特率9600 */
    uart_init(BT_UART, 9600);      /* 蓝牙默认波特率9600 */

    /* I2C + OLED初始化 */
    i2c_master_init(OLED_I2C);
    oled_init();
    oled_clear();
    oled_show_string(0, 0, "GPS Tracker", FONT_16);
    oled_show_string(0, 2, "Initializing...", FONT_12);
    oled_refresh();

    /* 等待GPS模块启动 */
    delay_ms(1000);

    /* 定时器变量 */
    uint32_t last_display_ms = 0;
    uint32_t last_report_ms = 0;

    /* 主循环 */
    while (1) {
        /* 持续读取并解析GPS串口数据 */
        extract_nmea_from_uart();

        uint32_t now_ms = get_tick();

        /* 每500ms更新一次OLED显示 */
        if (now_ms - last_display_ms >= 500) {
            last_display_ms = now_ms;
            oled_display_gps();
        }

        /* 每1000ms蓝牙上报一次 */
        if (now_ms - last_report_ms >= 1000) {
            last_report_ms = now_ms;
            if (g_gps.valid) {
                bt_report_gps();
            }
        }
    }

    return 0;
}
