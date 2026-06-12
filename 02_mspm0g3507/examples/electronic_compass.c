/**
 * @file electronic_compass.c
 * @brief 电子罗盘 — QMC5883L + OLED显示方向 + 舵机指北
 * @target MSPM0G3507
 *
 * 硬件连接：
 *   QMC5883L 三轴磁力计 (I2C):
 *     VCC -> 3.3V    GND -> GND
 *     SCL -> PB2 (I2C0_SCL)   SDA -> PB3 (I2C0_SDA)
 *     DRDY -> PA4 (可选，数据就绪中断)
 *   OLED SSD1306 (I2C, 共用总线):
 *     SCL -> PB2    SDA -> PB3
 *     地址0x3C
 *   舵机 SG90 (PWM):
 *     信号线 -> PA15 (TIMA0_CH0, PWM输出)
 *     VCC -> 5V    GND -> GND
 *
 * 功能：
 *   1. QMC5883L三轴磁场测量
 *   2. 椭圆校准消除硬铁/软铁误差
 *   3. 计算航向角（Heading），8方向显示
 *   4. OLED绘制指南针动画
 *   5. 舵机实时指向北方
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ============ 外设驱动头文件 ============ */
#include "drivers/oled_ssd1306.h"
#include "drivers/i2c_master.h"
#include "drivers/servo.h"

/* ============ 硬件配置 ============ */
#define QMC5883L_I2C    I2C0
#define QMC5883L_ADDR   0x0D   /* QMC5883L默认I2C地址 */
#define SERVO_CHANNEL   0      /* 舵机PWM通道0 */

/* ============ QMC5883L寄存器地址 ============ */
#define QMC_REG_DATA_X_LSB    0x00
#define QMC_REG_DATA_X_MSB    0x01
#define QMC_REG_DATA_Y_LSB    0x02
#define QMC_REG_DATA_Y_MSB    0x03
#define QMC_REG_DATA_Z_LSB    0x04
#define QMC_REG_DATA_Z_MSB    0x05
#define QMC_REG_STATUS        0x06
#define QMC_REG_TEMP_LSB      0x07
#define QMC_REG_TEMP_MSB      0x08
#define QMC_REG_CTRL1         0x09
#define QMC_REG_CTRL2         0x0A
#define QMC_REG_SET_RESET     0x0B
#define QMC_REG_CHIP_ID       0x0D

/* QMC5883L控制寄存器值 */
#define QMC_CTRL1_MODE_CONT   0x01   /* 连续测量模式 */
#define QMC_CTRL1_ODR_200HZ   0x00   /* 输出数据率200Hz */
#define QMC_CTRL1_ODR_100HZ   0x04   /* 输出数据率100Hz */
#define QMC_CTRL1_ODR_50HZ    0x08   /* 输出数据率50Hz */
#define QMC_CTRL1_OSR_512     0x00   /* 过采样率512 */
#define QMC_CTRL1_OSR_256     0x40   /* 过采样率256 */
#define QMC_CTRL1_RANGE_2G    0x00   /* 量程±2Gauss */
#define QMC_CTRL1_RANGE_8G    0x10   /* 量程±8Gauss */

#define QMC_CTRL2_ROL_PNT     0x40   /* 指针滚动 */
#define QMC_CTRL2_SOFT_RST    0x80   /* 软复位 */

/* QMC5883L芯片ID */
#define QMC_CHIP_ID_VALUE     0xFF

/* ============ 数据结构 ============ */
typedef struct {
    int16_t raw_x, raw_y, raw_z;   /* 原始磁力计值 */
    float mag_x, mag_y, mag_z;     /* 校准后的磁场值（Gauss） */
    float heading;                  /* 航向角（°），0=北, 90=东 */
    int16_t temperature;           /* 芯片温度 */
    bool data_ready;               /* 新数据就绪 */
} QMC5883L_Data_t;

typedef struct {
    float x_offset, y_offset, z_offset;  /* 硬铁偏移 */
    float x_scale, y_scale, z_scale;      /* 软铁缩放 */
} Compass_Calib_t;

/* ============ 全局变量 ============ */
static QMC5883L_Data_t g_compass = {0};
static Compass_Calib_t g_calib = {
    .x_offset = 0, .y_offset = 0, .z_offset = 0,
    .x_scale = 1.0f, .y_scale = 1.0f, .z_scale = 1.0f
};

#define DEG_TO_RAD  0.0174532925f
#define RAD_TO_DEG  57.29577951f

/* ============ QMC5883L底层读写 ============ */

/**
 * @brief 向QMC5883L写入寄存器
 */
static void qmc_write_reg(uint8_t reg, uint8_t data)
{
    i2c_master_write_reg(QMC5883L_I2C, QMC5883L_ADDR, reg, &data, 1);
}

/**
 * @brief 从QMC5883L读取寄存器
 */
static void qmc_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_master_read_reg(QMC5883L_I2C, QMC5883L_ADDR, reg, buf, len);
}

/* ============ QMC5883L初始化 ============ */

/**
 * @brief 初始化QMC5883L磁力计
 *
 * 配置：连续测量模式，200Hz，过采样512，量程±8Gauss
 */
static bool qmc5883l_init(void)
{
    uint8_t id;

    /* 软复位 */
    qmc_write_reg(QMC_REG_CTRL2, QMC_CTRL2_SOFT_RST);
    delay_ms(50);

    /* 读取芯片ID验证 */
    qmc_read_regs(QMC_REG_CHIP_ID, &id, 1);
    if (id != QMC_CHIP_ID_VALUE) {
        return false;  /* 芯片不存在或通信失败 */
    }

    /* 推荐的SET/RESET周期寄存器设置 */
    qmc_write_reg(QMC_REG_SET_RESET, 0x01);

    /* 控制寄存器1：连续模式，200Hz，OSR 512，量程8G */
    qmc_write_reg(QMC_REG_CTRL1,
        QMC_CTRL1_MODE_CONT |
        QMC_CTRL1_ODR_200HZ |
        QMC_CTRL1_OSR_512 |
        QMC_CTRL1_RANGE_8G
    );

    /* 控制寄存器2：正常配置 */
    qmc_write_reg(QMC_REG_CTRL2, 0x00);

    delay_ms(10);
    return true;
}

/* ============ QMC5883L数据读取 ============ */

/**
 * @brief 检查QMC5883L状态寄存器，判断新数据是否就绪
 * @return true=新数据就绪
 */
static bool qmc5883l_data_ready(void)
{
    uint8_t status;
    qmc_read_regs(QMC_REG_STATUS, &status, 1);
    return (status & 0x01) != 0;  /* DRDY位 */
}

/**
 * @brief 读取QMC5883L三轴磁力计数据
 */
static void qmc5883l_read_data(void)
{
    uint8_t buf[6];

    /* 读取6字节原始数据（X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB） */
    qmc_read_regs(QMC_REG_DATA_X_LSB, buf, 6);

    /* 合成16位有符号值（小端序，注意QMC5883L是小端！） */
    g_compass.raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    g_compass.raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    g_compass.raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    /* 校准：去除硬铁偏移，应用软铁缩放 */
    float cal_x = (g_compass.raw_x - g_calib.x_offset) * g_calib.x_scale;
    float cal_y = (g_compass.raw_y - g_calib.y_offset) * g_calib.y_scale;
    float cal_z = (g_compass.raw_z - g_calib.z_offset) * g_calib.z_scale;

    /* 转换为Gauss（±8G量程，灵敏度约3000 LSB/Gauss） */
    g_compass.mag_x = cal_x / 3000.0f;
    g_compass.mag_y = cal_y / 3000.0f;
    g_compass.mag_z = cal_z / 3000.0f;

    /* 读取温度 */
    uint8_t temp_buf[2];
    qmc_read_regs(QMC_REG_TEMP_LSB, temp_buf, 2);
    g_compass.temperature = (int16_t)((temp_buf[1] << 8) | temp_buf[0]);
}

/**
 * @brief 计算航向角（Heading）
 *
 * 使用atan2计算水平面上的磁北方向
 * Heading = atan2(My, Mx)，补偿磁偏角
 *
 * @param declination 磁偏角（°），中国大部分地区约-6°~+2°
 */
static void qmc5883l_calc_heading(float declination)
{
    /* 使用校准后的X和Y计算航向 */
    float heading_rad = atan2f(g_compass.mag_y, g_compass.mag_x);

    /* 弧度转角度 */
    float heading_deg = heading_rad * RAD_TO_DEG;

    /* 补偿磁偏角 */
    heading_deg += declination;

    /* 归一化到0~360° */
    if (heading_deg < 0) heading_deg += 360.0f;
    if (heading_deg >= 360.0f) heading_deg -= 360.0f;

    g_compass.heading = heading_deg;
}

/* ============ 磁力计校准 ============ */

/**
 * @brief 磁力计校准（椭圆拟合）
 *
 * 使用方法：
 *   1. 在水平面上缓慢旋转传感器360°
 *   2. 采集N个数据点
 *   3. 计算X/Y轴的最大最小值
 *   4. 硬铁偏移 = (max + min) / 2
 *   5. 软铁缩放 = range / max_range
 *
 * 注意：校准时请远离铁磁性物体和电机
 */
#define CALIB_POINTS    500
static void qmc5883l_calibrate(void)
{
    int16_t min_x = 32767, max_x = -32768;
    int16_t min_y = 32767, max_y = -32768;
    int16_t min_z = 32767, max_z = -32768;

    oled_clear();
    oled_show_string(0, 0, "Compass Calib.", FONT_16);
    oled_show_string(0, 3, "Rotate 360 deg", FONT_12);
    oled_show_string(0, 4, "on horizontal", FONT_12);
    oled_refresh();

    delay_ms(2000);  /* 给用户准备时间 */

    /* 采集校准数据 */
    for (int i = 0; i < CALIB_POINTS; i++) {
        if (qmc5883l_data_ready()) {
            qmc5883l_read_data();

            if (g_compass.raw_x < min_x) min_x = g_compass.raw_x;
            if (g_compass.raw_x > max_x) max_x = g_compass.raw_x;
            if (g_compass.raw_y < min_y) min_y = g_compass.raw_y;
            if (g_compass.raw_y > max_y) max_y = g_compass.raw_y;
            if (g_compass.raw_z < min_z) min_z = g_compass.raw_z;
            if (g_compass.raw_z > max_z) max_z = g_compass.raw_z;

            /* 显示进度 */
            if (i % 50 == 0) {
                char line[32];
                snprintf(line, sizeof(line), "%d%%", i * 100 / CALIB_POINTS);
                oled_show_string(0, 6, line, FONT_12);
                oled_refresh();
            }
        }
        delay_ms(20);
    }

    /* 计算硬铁偏移（椭圆中心） */
    g_calib.x_offset = (max_x + min_x) / 2.0f;
    g_calib.y_offset = (max_y + min_y) / 2.0f;
    g_calib.z_offset = (max_z + min_z) / 2.0f;

    /* 计算软铁缩放（使各轴范围一致） */
    float range_x = (max_x - min_x) / 2.0f;
    float range_y = (max_y - min_y) / 2.0f;
    float range_z = (max_z - min_z) / 2.0f;

    float max_range = range_x;
    if (range_y > max_range) max_range = range_y;
    if (range_z > max_range) max_range = range_z;

    g_calib.x_scale = max_range / range_x;
    g_calib.y_scale = max_range / range_y;
    g_calib.z_scale = max_range / range_z;

    oled_clear();
    oled_show_string(0, 3, "Calibration OK!", FONT_12);
    oled_refresh();
    delay_ms(1000);
}

/* ============ 方向辅助函数 ============ */

/**
 * @brief 根据航向角获取8方向字符串
 */
static const char* get_direction_str(float heading)
{
    if (heading >= 337.5 || heading < 22.5)  return "N";
    if (heading >= 22.5  && heading < 67.5)  return "NE";
    if (heading >= 67.5  && heading < 112.5) return "E";
    if (heading >= 112.5 && heading < 157.5) return "SE";
    if (heading >= 157.5 && heading < 202.5) return "S";
    if (heading >= 202.5 && heading < 247.5) return "SW";
    if (heading >= 247.5 && heading < 292.5) return "W";
    if (heading >= 292.5 && heading < 337.5) return "NW";
    return "N";
}

/* ============ OLED显示 ============ */

/**
 * @brief 在OLED上绘制简易指南针动画
 *
 * 绘制布局：
 *   顶部：标题 + 航向角
 *   中间：圆形指南针，指针指向磁北
 *   底部：方向名称 + 舵机角度
 */
static void oled_display_compass(void)
{
    char line[32];

    oled_clear();

    /* 标题 */
    oled_show_string(0, 0, "E-Compass", FONT_16);

    /* 航向角 */
    snprintf(line, sizeof(line), "Heading: %.1f deg", g_compass.heading);
    oled_show_string(0, 2, line, FONT_12);

    /* 方向名称 */
    snprintf(line, sizeof(line), "Direction: %s", get_direction_str(g_compass.heading));
    oled_show_string(0, 3, line, FONT_12);

    /* 绘制圆形指南针（中心64, 55, 半径22） */
    int cx = 64, cy = 55, r = 22;
    oled_draw_circle(cx, cy, r);

    /* 绘制方向标记 N/S/E/W */
    oled_show_string(cx - 3, cy - r - 8, "N", FONT_12);
    oled_show_string(cx - 3, cy + r + 1, "S", FONT_12);
    oled_show_string(cx + r + 2, cy - 4, "E", FONT_12);
    oled_show_string(cx - r - 8, cy - 4, "W", FONT_12);

    /* 绘制指针指向磁北（从中心到圆周） */
    float needle_rad = g_compass.heading * DEG_TO_RAD;
    int nx = cx + (int)(r * 0.8f * sinf(needle_rad));
    int ny = cy - (int)(r * 0.8f * cosf(needle_rad));
    oled_draw_line(cx, cy, nx, ny);

    /* 在北端画一个小三角标记 */
    oled_draw_filled_circle(nx, ny, 2);

    /* 磁场强度 */
    float mag_str = sqrtf(g_compass.mag_x * g_compass.mag_x +
                          g_compass.mag_y * g_compass.mag_y +
                          g_compass.mag_z * g_compass.mag_z);
    snprintf(line, sizeof(line), "B:%.2fG T:%dC", mag_str, g_compass.temperature);
    oled_show_string(0, 7, line, FONT_12);

    oled_refresh();
}

/* ============ 舵机控制 ============ */

/**
 * @brief 舵机指向北方
 *
 * 将航向角映射到舵机角度（0-180°）
 * 舵机在90°位置时指针朝前（物理北方假设）
 * 航向角0°=北方，舵机应转到与磁北对齐
 */
static void servo_point_north(float heading)
{
    /*
     * 舵机角度 = 90 + heading（偏移修正）
     * 限制在0~180°范围内
     */
    float servo_angle = 90.0f + heading;
    if (servo_angle > 180.0f) servo_angle -= 360.0f;
    if (servo_angle < 0.0f) servo_angle += 360.0f;

    /* 限幅到舵机有效范围 */
    if (servo_angle < 0.0f) servo_angle = 0.0f;
    if (servo_angle > 180.0f) servo_angle = 180.0f;

    servo_set_angle(SERVO_CHANNEL, (uint8_t)servo_angle);
}

/* ============ 主函数 ============ */

/* 磁偏角配置（根据当地地理位置调整） */
/* 北京约-6°，上海约-5°，深圳约-2°，成都约-2° */
#define MAGNETIC_DECLINATION  -5.0f   /* 默认-5°，请根据实际位置修改 */

int main(void)
{
    /* 系统初始化 */
    DL_SYSCTL_init();
    SysTick_Config(DL_SYSCTL_getMCLKFreq() / 1000);

    /* I2C初始化 */
    i2c_master_init(QMC5883L_I2C);

    /* 舵机PWM初始化 */
    servo_init();

    /* OLED初始化 */
    oled_init();
    oled_clear();
    oled_show_string(0, 0, "E-Compass", FONT_16);
    oled_show_string(0, 2, "QMC5883L", FONT_12);
    oled_show_string(0, 3, "Initializing...", FONT_12);
    oled_refresh();

    /* QMC5883L初始化 */
    if (!qmc5883l_init()) {
        oled_clear();
        oled_show_string(0, 3, "Sensor Error!", FONT_12);
        oled_refresh();
        while (1);  /* 停机 */
    }

    delay_ms(500);

    /* 磁力计校准 */
    qmc5883l_calibrate();

    /* 定时器变量 */
    uint32_t last_update_ms = 0;

    /* 主循环 */
    while (1) {
        /* 检查新数据 */
        if (qmc5883l_data_ready()) {
            qmc5883l_read_data();

            /* 计算航向角 */
            qmc5883l_calc_heading(MAGNETIC_DECLINATION);

            /* 每100ms更新显示和舵机 */
            uint32_t now_ms = get_tick();
            if (now_ms - last_update_ms >= 100) {
                last_update_ms = now_ms;

                /* 更新OLED显示 */
                oled_display_compass();

                /* 舵机指向北方 */
                servo_point_north(g_compass.heading);
            }
        }

        delay_ms(2);  /* 短暂等待 */
    }

    return 0;
}
