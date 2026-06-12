/**
 * @file gps_neo6m.c
 * @brief GPS NEO-6M 模块驱动实现（NMEA协议解析）
 *
 * 依赖 SysConfig 生成的 UART 宏。
 * 本驱动不依赖中断，通过逐字节喂入实现 NMEA 解析。
 */

#include "gps_neo6m.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ti/driverlib/dl_uart.h>
#include "ti_msp_dl_config.h"

/* ============================================================
 *  UART 实例适配 —— 根据 SysConfig 配置修改
 * ============================================================ */
#define GPS_UART_INST   UART_0_INST

/* ============================================================
 *  内部解析状态机
 * ============================================================ */
typedef enum {
    PARSE_IDLE = 0,      /* 等待 '$' */
    PARSE_HEADER,        /* 解析语句头（如 GPGGA） */
    PARSE_DATA,          /* 解析数据字段 */
    PARSE_CHECKSUM_H,    /* 解析校验和高半字节 */
    PARSE_CHECKSUM_L     /* 解析校验和低半字节 */
} parse_state_t;

/* ============================================================
 *  内部全局变量
 * ============================================================ */
static gps_data_t      s_gps_data;        /* GPS 数据缓存 */
static volatile bool   s_new_data_flag;   /* 新数据标志 */
static parse_state_t   s_parse_state;     /* 解析状态 */
static uint8_t         s_buf[GPS_NMEA_MAX_LEN]; /* 接收缓冲区 */
static uint8_t         s_buf_idx;         /* 缓冲区索引 */
static uint8_t         s_checksum_calc;   /* 计算的校验和 */
static uint8_t         s_checksum_recv;   /* 接收到的校验和 */
static char            s_sentence_type[6]; /* 语句类型（如 "GGA"） */
static uint8_t         s_gsv_count;       /* GSV语句计数 */
static uint8_t         s_gsv_total;       /* GSV总语句数 */

/* ============================================================
 *  字段提取辅助函数
 * ============================================================ */

/**
 * @brief 从NMEA缓冲区中提取第N个字段（逗号分隔）
 * @param buf NMEA语句（不含 $ 和 *）
 * @param field_index 字段索引（从0开始）
 * @param out 输出缓冲区
 * @param out_len 输出缓冲区长度
 * @return true 找到字段
 */
static bool nmea_get_field(const uint8_t *buf, uint8_t field_index,
                           char *out, uint8_t out_len)
{
    uint8_t idx = 0;
    uint8_t field = 0;
    uint8_t out_idx = 0;

    /* 跳过到指定字段 */
    while (buf[idx] != '\0' && buf[idx] != '*') {
        if (field == field_index) {
            /* 复制字段内容 */
            while (buf[idx] != ',' && buf[idx] != '*' && buf[idx] != '\0') {
                if (out_idx < out_len - 1) {
                    out[out_idx++] = buf[idx];
                }
                idx++;
            }
            out[out_idx] = '\0';
            return (out_idx > 0);
        }
        /* 跳过当前字段 */
        while (buf[idx] != ',' && buf[idx] != '*' && buf[idx] != '\0')
            idx++;
        if (buf[idx] == ',') {
            field++;
            idx++;
        }
    }
    out[0] = '\0';
    return false;
}

/**
 * @brief 将NMEA ddmm.mmmm 格式转为十进制度
 */
double gps_neo6m_nmea_to_degrees(double nmea_val, char dir)
{
    /* ddmm.mmmm → 度 = dd + mm.mmmm/60 */
    int degrees = (int)(nmea_val / 100);
    double minutes = nmea_val - (double)degrees * 100.0;
    double decimal = (double)degrees + minutes / 60.0;

    if (dir == 'S' || dir == 'W')
        decimal = -decimal;

    return decimal;
}

/**
 * @brief 将hhmmss.ss 格式解析为时分秒
 */
static void parse_time(double time_val, uint8_t *h, uint8_t *m,
                       uint8_t *s, uint16_t *ms)
{
    int itime = (int)time_val;
    *h = (uint8_t)(itime / 10000);
    *m = (uint8_t)((itime / 100) % 100);
    *s = (uint8_t)(itime % 100);
    double frac = time_val - (double)itime;
    *ms = (uint16_t)(frac * 1000.0);
}

/**
 * @brief 将ddmmyy 格式解析为日月年
 */
static void parse_date(int date_val, uint8_t *d, uint8_t *m, uint16_t *y)
{
    *d = (uint8_t)(date_val / 10000);
    *m = (uint8_t)((date_val / 100) % 100);
    *y = (uint16_t)(2000 + date_val % 100);
}

/* ============================================================
 *  NMEA 语句解析函数
 * ============================================================ */

/**
 * @brief 解析 GGA 语句（定位数据）
 * $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 *  字段:   0      1      2   3     4   5 6  7   8   9  10  11  12  13
 */
static void parse_gga(const uint8_t *buf)
{
    char field[16];

    /* 字段0: UTC时间 */
    if (nmea_get_field(buf, 0, field, sizeof(field))) {
        double t = atof(field);
        parse_time(t, &s_gps_data.hour, &s_gps_data.minute,
                   &s_gps_data.second, &s_gps_data.millisecond);
        s_gps_data.valid_time = true;
    }

    /* 字段1: 纬度 ddmm.mmmm */
    if (nmea_get_field(buf, 1, field, sizeof(field))) {
        double lat_nmea = atof(field);
        char dir[2];
        nmea_get_field(buf, 2, dir, sizeof(dir));
        s_gps_data.latitude = gps_neo6m_nmea_to_degrees(lat_nmea, dir[0]);
    }

    /* 字段3: 经度 dddmm.mmmm */
    if (nmea_get_field(buf, 3, field, sizeof(field))) {
        double lon_nmea = atof(field);
        char dir[2];
        nmea_get_field(buf, 4, dir, sizeof(dir));
        s_gps_data.longitude = gps_neo6m_nmea_to_degrees(lon_nmea, dir[0]);
    }

    /* 字段5: 定位质量 */
    if (nmea_get_field(buf, 5, field, sizeof(field))) {
        s_gps_data.fix_quality = (gps_fix_quality_t)atoi(field);
    }

    /* 字段6: 使用卫星数 */
    if (nmea_get_field(buf, 6, field, sizeof(field))) {
        s_gps_data.satellites_used = (uint8_t)atoi(field);
    }

    /* 字段7: HDOP */
    if (nmea_get_field(buf, 7, field, sizeof(field))) {
        s_gps_data.hdop = (float)atof(field);
    }

    /* 字段8: 海拔 */
    if (nmea_get_field(buf, 8, field, sizeof(field))) {
        s_gps_data.altitude_m = (float)atof(field);
    }

    /* 位置有效判断 */
    s_gps_data.valid_position = (s_gps_data.fix_quality != GPS_FIX_NONE);
    s_new_data_flag = true;
}

/**
 * @brief 解析 RMC 语句（推荐最小定位）
 * $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
 */
static void parse_rmc(const uint8_t *buf)
{
    char field[16];

    /* 字段0: 时间 */
    if (nmea_get_field(buf, 0, field, sizeof(field))) {
        double t = atof(field);
        parse_time(t, &s_gps_data.hour, &s_gps_data.minute,
                   &s_gps_data.second, &s_gps_data.millisecond);
        s_gps_data.valid_time = true;
    }

    /* 字段1: 状态 A=有效, V=无效 */
    if (nmea_get_field(buf, 1, field, sizeof(field))) {
        s_gps_data.valid_position = (field[0] == 'A');
    }

    /* 字段2-5: 经纬度 */
    if (nmea_get_field(buf, 2, field, sizeof(field))) {
        double lat_nmea = atof(field);
        char dir[2];
        nmea_get_field(buf, 3, dir, sizeof(dir));
        s_gps_data.latitude = gps_neo6m_nmea_to_degrees(lat_nmea, dir[0]);
    }
    if (nmea_get_field(buf, 4, field, sizeof(field))) {
        double lon_nmea = atof(field);
        char dir[2];
        nmea_get_field(buf, 5, dir, sizeof(dir));
        s_gps_data.longitude = gps_neo6m_nmea_to_degrees(lon_nmea, dir[0]);
    }

    /* 字段6: 速度（节） */
    if (nmea_get_field(buf, 6, field, sizeof(field))) {
        s_gps_data.speed_knots = (float)atof(field);
        s_gps_data.speed_kmh = s_gps_data.speed_knots * 1.852f;  /* 1节=1.852km/h */
    }

    /* 字段7: 航向 */
    if (nmea_get_field(buf, 7, field, sizeof(field))) {
        s_gps_data.course_deg = (float)atof(field);
    }

    /* 字段8: 日期 ddmmyy */
    if (nmea_get_field(buf, 8, field, sizeof(field))) {
        int date = atoi(field);
        parse_date(date, &s_gps_data.day, &s_gps_data.month, &s_gps_data.year);
        s_gps_data.valid_date = true;
    }
}

/**
 * @brief 解析 GSV 语句（可见卫星）
 * $GPGSV,x,x,xx,...*hh
 */
static void parse_gsv(const uint8_t *buf)
{
    char field[8];

    /* 字段1: 当前语句编号 */
    if (!nmea_get_field(buf, 1, field, sizeof(field))) return;
    uint8_t msg_num = (uint8_t)atoi(field);

    /* 字段2: 总语句数 */
    if (!nmea_get_field(buf, 2, field, sizeof(field))) return;
    s_gsv_total = (uint8_t)atoi(field);

    /* 字段3: 可见卫星总数 */
    if (msg_num == 1) {
        if (nmea_get_field(buf, 3, field, sizeof(field))) {
            s_gps_data.satellites_in_view = (uint8_t)atoi(field);
            s_gsv_count = 0;
        }
    }

    /* 字段4-7, 8-11, 12-15, 16-19: 每组4个字段代表一颗卫星 */
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t base = 4 + i * 4;
        if (s_gsv_count >= GPS_MAX_SATELLITES) break;

        /* 字段4: PRN */
        char prn_s[8];
        if (!nmea_get_field(buf, base, prn_s, sizeof(prn_s))) break;
        if (prn_s[0] == '\0') break;

        gps_satellite_t *sat = &s_gps_data.satellites[s_gsv_count];
        sat->prn = (uint8_t)atoi(prn_s);

        /* 字段5: 仰角 */
        if (nmea_get_field(buf, base + 1, field, sizeof(field)))
            sat->elevation = (uint8_t)atoi(field);

        /* 字段6: 方位角 */
        if (nmea_get_field(buf, base + 2, field, sizeof(field)))
            sat->azimuth = (uint16_t)atoi(field);

        /* 字段7: 信噪比 */
        if (nmea_get_field(buf, base + 3, field, sizeof(field)))
            sat->snr = (uint8_t)atoi(field);

        s_gsv_count++;
    }
}

/**
 * @brief 解析 GSA 语句（精度因子）
 * $GNGSA,A,3,xx,xx,...,x.x,x.x,x.x*hh
 */
static void parse_gsa(const uint8_t *buf)
{
    char field[8];

    /* 字段14: PDOP */
    if (nmea_get_field(buf, 14, field, sizeof(field)))
        s_gps_data.pdop = (float)atof(field);

    /* 字段15: HDOP */
    if (nmea_get_field(buf, 15, field, sizeof(field)))
        s_gps_data.hdop = (float)atof(field);

    /* 字段16: VDOP（注意：最后一个字段，可能后面就是*了） */
    if (nmea_get_field(buf, 16, field, sizeof(field))) {
        /* 去除可能的 * 和校验和 */
        char *star = strchr(field, '*');
        if (star) *star = '\0';
        s_gps_data.vdop = (float)atof(field);
    }
}

/* ============================================================
 *  校验和计算
 * ============================================================ */
static uint8_t nmea_checksum(const uint8_t *buf, uint8_t len)
{
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= buf[i];
    }
    return cs;
}

/* ============================================================
 *  解析一条完整的NMEA语句
 * ============================================================ */
static void parse_sentence(void)
{
    /* 确保字符串以 \0 结尾 */
    if (s_buf_idx >= GPS_NMEA_MAX_LEN) s_buf_idx = GPS_NMEA_MAX_LEN - 1;
    s_buf[s_buf_idx] = '\0';

    /* 校验和验证 */
    uint8_t cs = nmea_checksum(s_buf, s_buf_idx);
    if (cs != s_checksum_recv) {
        return;  /* 校验和不匹配，丢弃 */
    }

    /* 跳过前缀（如 GP、GN、GL、GA） */
    /* s_sentence_type 中存的是 "GGA" 等不带前缀的类型 */
    if (strcmp(s_sentence_type, "GGA") == 0) {
        parse_gga(s_buf);
    } else if (strcmp(s_sentence_type, "RMC") == 0) {
        parse_rmc(s_buf);
    } else if (strcmp(s_sentence_type, "GSV") == 0) {
        parse_gsv(s_buf);
    } else if (strcmp(s_sentence_type, "GSA") == 0) {
        parse_gsa(s_buf);
    }
}

/* ============================================================
 *  公开 API
 * ============================================================ */

bool gps_neo6m_init(void)
{
    memset(&s_gps_data, 0, sizeof(s_gps_data));
    s_new_data_flag = false;
    s_parse_state = PARSE_IDLE;
    s_buf_idx = 0;
    s_checksum_calc = 0;
    s_checksum_recv = 0;
    s_sentence_type[0] = '\0';
    s_gsv_count = 0;
    s_gsv_total = 0;
    return true;
}

bool gps_neo6m_process_byte(uint8_t byte)
{
    switch (s_parse_state) {
    case PARSE_IDLE:
        /* 等待 '$' 起始符 */
        if (byte == '$') {
            s_parse_state = PARSE_HEADER;
            s_buf_idx = 0;
            s_checksum_calc = 0;
            s_sentence_type[0] = '\0';
        }
        break;

    case PARSE_HEADER:
        /* 解析语句头，提取3字符类型（如GGA、RMC） */
        if (byte == ',') {
            /* 头部结束，开始数据字段 */
            s_parse_state = PARSE_DATA;
            s_checksum_calc ^= byte;
        } else if (byte == '*') {
            s_parse_state = PARSE_CHECKSUM_H;
        } else {
            s_checksum_calc ^= byte;
            /* 提取最后3个字符作为类型（跳过前缀如GP/GN） */
            if (s_buf_idx >= 2) {
                /* 检查是否已收集到足够的头部字符 */
                /* 收集前缀+类型 */
                if (s_buf_idx < 5) {
                    s_buf[s_buf_idx++] = byte;
                }
            } else {
                s_buf[s_buf_idx++] = byte;
            }
            /* 当收集到5个字符（如 GPGGA 或 GNRMC），提取后3个 */
            if (s_buf_idx == 5) {
                memcpy(s_sentence_type, &s_buf[2], 3);
                s_sentence_type[3] = '\0';
            }
        }
        break;

    case PARSE_DATA:
        if (byte == '*') {
            s_parse_state = PARSE_CHECKSUM_H;
        } else {
            s_checksum_calc ^= byte;
            if (s_buf_idx < GPS_NMEA_MAX_LEN) {
                s_buf[s_buf_idx++] = byte;
            }
        }
        break;

    case PARSE_CHECKSUM_H:
        /* 校验和高半字节 */
        if (byte >= '0' && byte <= '9')
            s_checksum_recv = (byte - '0') << 4;
        else if (byte >= 'A' && byte <= 'F')
            s_checksum_recv = (byte - 'A' + 10) << 4;
        else if (byte >= 'a' && byte <= 'f')
            s_checksum_recv = (byte - 'a' + 10) << 4;
        else {
            s_parse_state = PARSE_IDLE;
            break;
        }
        s_parse_state = PARSE_CHECKSUM_L;
        break;

    case PARSE_CHECKSUM_L:
        /* 校验和低半字节 */
        if (byte >= '0' && byte <= '9')
            s_checksum_recv |= (byte - '0');
        else if (byte >= 'A' && byte <= 'F')
            s_checksum_recv |= (byte - 'A' + 10);
        else if (byte >= 'a' && byte <= 'f')
            s_checksum_recv |= (byte - 'a' + 10);
        else {
            s_parse_state = PARSE_IDLE;
            break;
        }
        /* 一条完整语句解析完成 */
        s_buf[s_buf_idx] = '\0';
        parse_sentence();
        s_parse_state = PARSE_IDLE;
        break;
    }

    return true;
}

uint32_t gps_neo6m_process_buffer(const uint8_t *data, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        gps_neo6m_process_byte(data[i]);
    }
    return i;
}

bool gps_neo6m_get_data(gps_data_t *data)
{
    if (data == NULL) return false;
    memcpy(data, &s_gps_data, sizeof(gps_data_t));
    s_new_data_flag = false;
    return s_gps_data.valid_position;
}

bool gps_neo6m_has_new_data(void)
{
    return s_new_data_flag;
}
