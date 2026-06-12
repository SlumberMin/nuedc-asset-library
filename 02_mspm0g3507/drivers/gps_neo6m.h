/**
 * @file gps_neo6m.h
 * @brief GPS NEO-6M 模块驱动（UART接口，NMEA协议解析）
 *
 * NEO-6M 是 u-blox 出品的低成本 GPS 模块：
 *   - 50Hz 位置更新率（NEO-6M 默认 1Hz）
 *   - 冷启动约 27s，热启动约 1s
 *   - NMEA 0183 协议输出
 *   - 默认波特率 9600
 *
 * 本驱动解析以下 NMEA 语句：
 *   - $GPGGA / $GNGGA：定位数据（时间、经纬度、高度、卫星数）
 *   - $GPRMC / $GNRMC：推荐最小定位信息（时间、经纬度、速度、航向）
 *   - $GPGSV / $GSV：可见卫星信息
 *   - $GPGSA / $GNGSA：精度因子（DOP）
 *
 * 硬件接线：
 *   NEO-6M VCC → 3.3V 或 5V（视模块版本）
 *   NEO-6M GND → GND
 *   NEO-6M TX  → MSPM0 UART RX
 *   NEO-6M RX  → MSPM0 UART TX（可选，用于发送UBX命令）
 *   NEO-6M PPS → 可选，秒脉冲输出
 */

#ifndef __GPS_NEO6M_H
#define __GPS_NEO6M_H

#include <stdint.h>
#include <stdbool.h>

/* NMEA 语句最大长度（不含 $ 和 *） */
#define GPS_NMEA_MAX_LEN    128

/* 最大可见卫星数 */
#define GPS_MAX_SATELLITES  12

/* ============================================================
 *  NMEA 语句类型标识
 * ============================================================ */
typedef enum {
    GPS_NMEA_NONE = 0,
    GPS_NMEA_GGA,      /* 定位数据 */
    GPS_NMEA_RMC,      /* 推荐最小定位 */
    GPS_NMEA_GSV,      /* 可见卫星 */
    GPS_NMEA_GSA,      /* 精度因子 */
    GPS_NMEA_VTG,      /* 地面速度和航向 */
    GPS_NMEA_UNKNOWN   /* 未知语句 */
} gps_nmea_type_t;

/**
 * @brief GPS 定位质量枚举
 */
typedef enum {
    GPS_FIX_NONE = 0,      /* 无定位 */
    GPS_FIX_GPS = 1,       /* GPS 定位 */
    GPS_FIX_DGPS = 2,      /* 差分GPS */
    GPS_FIX_PPS = 3,       /* PPS */
    GPS_FIX_RTK = 4,       /* RTK固定解 */
    GPS_FIX_FLOAT = 5,     /* RTK浮动解 */
    GPS_FIX_ESTIMATED = 6  /* 估算 */
} gps_fix_quality_t;

/**
 * @brief 卫星信息结构体
 */
typedef struct {
    uint8_t prn;        /* 卫星编号 */
    uint8_t elevation;  /* 仰角（°） */
    uint16_t azimuth;   /* 方位角（°） */
    uint8_t snr;        /* 信噪比（dB），0=未跟踪 */
} gps_satellite_t;

/**
 * @brief GPS 完整数据结构体
 */
typedef struct {
    /* 时间（UTC） */
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint16_t millisecond;

    /* 日期 */
    uint8_t  day;
    uint8_t  month;
    uint16_t year;

    /* 位置 */
    double   latitude;       /* 纬度，单位 度（正=北纬，负=南纬） */
    double   longitude;      /* 经度，单位 度（正=东经，负=西经） */
    float    altitude_m;     /* 海拔高度，单位 米 */
    float    speed_knots;    /* 对地速度，单位 节 */
    float    speed_kmh;      /* 对地速度，单位 km/h */
    float    course_deg;     /* 对地航向，单位 °（真北） */

    /* 精度 */
    float    hdop;           /* 水平精度因子 */
    float    vdop;           /* 垂直精度因子 */
    float    pdop;           /* 位置精度因子 */

    /* 定位状态 */
    gps_fix_quality_t fix_quality;  /* 定位质量 */
    uint8_t  satellites_used;       /* 使用中的卫星数 */

    /* 有效标志 */
    bool     valid_position;   /* 位置有效（GGA/RMC） */
    bool     valid_time;       /* 时间有效 */
    bool     valid_date;       /* 日期有效 */

    /* 卫星信息 */
    uint8_t  satellites_in_view;  /* 可见卫星数 */
    gps_satellite_t satellites[GPS_MAX_SATELLITES];
} gps_data_t;

/**
 * @brief 初始化GPS模块
 *
 * 初始化UART接收缓冲区，清空GPS数据结构体。
 * 注意：GPS模块不需要发送初始化命令，上电即自动输出NMEA语句。
 *
 * @return true 初始化成功
 */
bool gps_neo6m_init(void);

/**
 * @brief 处理一个接收到的字节
 *
 * 此函数应在UART接收中断或主循环中调用，逐字节喂入NMEA解析器。
 * 每当完整的一条NMEA语句解析完成，内部会自动更新GPS数据。
 *
 * @param byte 接收到的字节
 * @return true 解析成功（不一定表示整个语句完成）
 */
bool gps_neo6m_process_byte(uint8_t byte);

/**
 * @brief 从缓冲区批量处理字节
 *
 * @param data 数据指针
 * @param len  数据长度
 * @return 成功处理的字节数
 */
uint32_t gps_neo6m_process_buffer(const uint8_t *data, uint32_t len);

/**
 * @brief 获取最新GPS数据
 *
 * @param data 输出结构体指针
 * @return true 有有效的定位数据
 */
bool gps_neo6m_get_data(gps_data_t *data);

/**
 * @brief 检查是否有新数据（自上次get_data以来是否有更新）
 * @return true 有新数据
 */
bool gps_neo6m_has_new_data(void);

/**
 * @brief 将NMEA经纬度格式转换为十进制度
 *
 * NMEA格式：ddmm.mmmm（纬度）或 dddmm.mmmm（经度）
 *
 * @param nmea_val NMEA原始值
 * @param dir 方向字符（'N'/'S'/'E'/'W'）
 * @return 十进制度数（南纬/西经为负）
 */
double gps_neo6m_nmea_to_degrees(double nmea_val, char dir);

#endif /* __GPS_NEO6M_H */
