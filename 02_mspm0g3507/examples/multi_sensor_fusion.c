/**
 * @file multi_sensor_fusion.c
 * @brief 多传感器融合 — MPU6050 + QMC5883L + GPS + 卡尔曼滤波
 * @target MSPM0G3507
 *
 * 硬件连接：
 *   MPU6050 六轴IMU (I2C):
 *     SCL -> PB2 (I2C0_SCL)   SDA -> PB3 (I2C0_SDA)
 *     AD0 -> GND (地址0x68)
 *   QMC5883L 三轴磁力计 (I2C, 共用总线):
 *     地址0x0D
 *   NEO-6M GPS (UART):
 *     TX -> PA1 (UART1_RX)
 *   OLED SSD1306 (I2C, 共用总线):
 *     地址0x3C
 *
 * 功能：
 *   1. MPU6050提供加速度和角速度
 *   2. QMC5883L提供磁场方向
 *   3. GPS提供绝对位置和速度
 *   4. 扩展卡尔曼滤波（EKF）融合估计：
 *      - 姿态四元数（roll/pitch/yaw）
 *      - 位置（latitude/longitude）
 *      - 速度（北向/东向）
 *   5. OLED显示融合结果
 *
 * 状态向量：x = [q0, q1, q2, q3, vn, ve, lat, lon]^T （8维）
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

/* ============ 硬件配置 ============ */
#define MPU6050_I2C     I2C0
#define MPU6050_ADDR    0x68
#define QMC5883L_I2C    I2C0
#define QMC5883L_ADDR   0x0D
#define GPS_UART        UART1

/* ============ 常量定义 ============ */
#define DEG_TO_RAD      0.0174532925f
#define RAD_TO_DEG      57.29577951f
#define PI              3.14159265f
#define EARTH_RADIUS    6371000.0f   /* 地球半径（米） */
#define SAMPLE_DT       0.01f        /* 采样周期10ms = 100Hz */

/* MPU6050常量 */
#define ACCEL_SCALE     16384.0f     /* ±2g */
#define GYRO_SCALE      131.0f       /* ±250°/s */

/* QMC5883L寄存器 */
#define QMC_REG_DATA_X_LSB  0x00
#define QMC_REG_STATUS      0x06
#define QMC_REG_CTRL1       0x09
#define QMC_REG_CTRL2       0x0A
#define QMC_REG_SET_RESET   0x0B
#define QMC_REG_CHIP_ID     0x0D

/* ============ 数据结构 ============ */

/* MPU6050数据 */
typedef struct {
    float ax, ay, az;       /* 加速度（g） */
    float gx, gy, gz;       /* 角速度（°/s） */
    float temp;             /* 温度（°C） */
} IMU_Data_t;

/* QMC5883L数据 */
typedef struct {
    float mx, my, mz;       /* 磁场（Gauss） */
    float heading;           /* 航向角（°） */
} Mag_Data_t;

/* GPS数据 */
typedef struct {
    double latitude;         /* 纬度（°） */
    double longitude;        /* 经度（°） */
    float  speed_kmh;        /* 地面速度（km/h） */
    float  course;           /* 地面航向（°） */
    uint8_t fix_quality;     /* 定位质量 */
    uint8_t num_sat;         /* 卫星数 */
    bool   updated;          /* 新数据标志 */
    bool   valid;            /* 定位有效 */
} GPS_Data_t;

/* EKF状态向量 */
#define STATE_DIM   8   /* 状态维度 */
#define MEAS_DIM    6   /* 观测维度 */

typedef struct {
    /* 状态向量 x[8]:
     * [0-3] 四元数 q0,q1,q2,q3
     * [4-5] 北向/东向速度 vn, ve (m/s)
     * [6-7] 纬度/经度偏移 dlat, dlon (rad)
     */
    float x[STATE_DIM];

    /* 状态协方差矩阵 P[8][8] */
    float P[STATE_DIM][STATE_DIM];

    /* 过程噪声协方差 Q[8][8] */
    float Q[STATE_DIM][STATE_DIM];

    /* 观测噪声协方差 R[6][6] */
    float R[MEAS_DIM][MEAS_DIM];

    float dt;   /* 采样周期 */
} EKF_t;

/* ============ 全局变量 ============ */
static IMU_Data_t g_imu = {0};
static Mag_Data_t g_mag = {0};
static GPS_Data_t g_gps = {0};
static EKF_t g_ekf;

/* 融合结果 */
static float g_fused_roll = 0, g_fused_pitch = 0, g_fused_yaw = 0;
static float g_fused_vn = 0, g_fused_ve = 0;
static double g_fused_lat = 0, g_fused_lon = 0;

/* ============ 矩阵运算辅助函数 ============ */

/**
 * @brief 矩阵加法 C = A + B (n×n)
 */
static void mat_add(float *A, float *B, float *C, int n)
{
    for (int i = 0; i < n * n; i++) {
        C[i] = A[i] + B[i];
    }
}

/**
 * @brief 矩阵减法 C = A - B (n×n)
 */
static void mat_sub(float *A, float *B, float *C, int n)
{
    for (int i = 0; i < n * n; i++) {
        C[i] = A[i] - B[i];
    }
}

/**
 * @brief 矩阵乘法 C = A × B
 * @param m A的行数, k A的列数/B的行数, n B的列数
 */
static void mat_mul(float *A, float *B, float *C, int m, int k, int n)
{
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            float sum = 0;
            for (int p = 0; p < k; p++) {
                sum += A[i * k + p] * B[p * n + j];
            }
            C[i * n + j] = sum;
        }
    }
}

/**
 * @brief 矩阵转置 B = A^T
 */
static void mat_trans(float *A, float *B, int m, int n)
{
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            B[j * m + i] = A[i * n + j];
        }
    }
}

/**
 * @brief 3×3矩阵求逆（使用伴随矩阵法）
 * @return true=可逆
 */
static bool mat_inv3(float *A, float *Ainv)
{
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);

    if (fabsf(det) < 1e-10f) return false;

    float inv_det = 1.0f / det;
    Ainv[0] = (A[4] * A[8] - A[5] * A[7]) * inv_det;
    Ainv[1] = (A[2] * A[7] - A[1] * A[8]) * inv_det;
    Ainv[2] = (A[1] * A[5] - A[2] * A[4]) * inv_det;
    Ainv[3] = (A[5] * A[6] - A[3] * A[8]) * inv_det;
    Ainv[4] = (A[0] * A[8] - A[2] * A[6]) * inv_det;
    Ainv[5] = (A[2] * A[3] - A[0] * A[5]) * inv_det;
    Ainv[6] = (A[3] * A[7] - A[4] * A[6]) * inv_det;
    Ainv[7] = (A[1] * A[6] - A[0] * A[7]) * inv_det;
    Ainv[8] = (A[0] * A[4] - A[1] * A[3]) * inv_det;
    return true;
}

/**
 * @brief 单位四元数归一化
 */
static void quat_normalize(float *q)
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-10f) {
        q[0] /= norm; q[1] /= norm;
        q[2] /= norm; q[3] /= norm;
    }
}

/**
 * @brief 四元数乘法 c = a ⊗ b
 */
static void quat_mul(float *a, float *b, float *c)
{
    c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* ============ MPU6050驱动 ============ */

static void mpu6050_write_reg(uint8_t reg, uint8_t val)
{
    i2c_master_write_reg(MPU6050_I2C, MPU6050_ADDR, reg, &val, 1);
}

static void mpu6050_init(void)
{
    mpu6050_write_reg(0x6B, 0x80);  /* 复位 */
    delay_ms(100);
    mpu6050_write_reg(0x6B, 0x00);  /* 唤醒 */
    delay_ms(10);
    mpu6050_write_reg(0x19, 9);     /* 采样率: 100Hz */
    mpu6050_write_reg(0x1A, 0x06);  /* 低通滤波: 5Hz */
    mpu6050_write_reg(0x1B, 0x00);  /* 陀螺仪: ±250°/s */
    mpu6050_write_reg(0x1C, 0x00);  /* 加速度: ±2g */
}

static void mpu6050_read(void)
{
    uint8_t buf[14];
    i2c_master_read_reg(MPU6050_I2C, MPU6050_ADDR, 0x3B, buf, 14);

    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t raw_temp = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    g_imu.ax = raw_ax / ACCEL_SCALE;
    g_imu.ay = raw_ay / ACCEL_SCALE;
    g_imu.az = raw_az / ACCEL_SCALE;
    g_imu.gx = raw_gx / GYRO_SCALE;
    g_imu.gy = raw_gy / GYRO_SCALE;
    g_imu.gz = raw_gz / GYRO_SCALE;
    g_imu.temp = raw_temp / 340.0f + 36.53f;
}

/* ============ QMC5883L驱动 ============ */

static void qmc_write_reg(uint8_t reg, uint8_t val)
{
    i2c_master_write_reg(QMC5883L_I2C, QMC5883L_ADDR, reg, &val, 1);
}

static void qmc5883l_init(void)
{
    qmc_write_reg(0x0A, 0x80);  /* 软复位 */
    delay_ms(50);
    qmc_write_reg(0x0B, 0x01);  /* SET/RESET */
    qmc_write_reg(0x09, 0x1D);  /* 连续模式, 200Hz, OSR512, ±8G */
}

static void qmc5883l_read(void)
{
    uint8_t buf[6];
    i2c_master_read_reg(QMC5883L_I2C, QMC5883L_ADDR, QMC_REG_DATA_X_LSB, buf, 6);

    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    /* 简化校准（实际应做椭圆校准） */
    g_mag.mx = raw_x / 3000.0f;
    g_mag.my = raw_y / 3000.0f;
    g_mag.mz = raw_z / 3000.0f;

    g_mag.heading = atan2f(g_mag.my, g_mag.mx) * RAD_TO_DEG;
    if (g_mag.heading < 0) g_mag.heading += 360.0f;
}

/* ============ GPS NMEA解析（简化版） ============ */

static void gps_parse_gpgga(char *sentence)
{
    char *p = sentence;
    if (*p == '$') p++;

    char *fields[15];
    int cnt = 0;
    fields[cnt++] = p;
    while (*p && cnt < 15) {
        if (*p == ',') { *p = '\0'; fields[cnt++] = p + 1; }
        p++;
    }
    if (cnt < 10) return;

    if (strlen(fields[2]) > 0 && strlen(fields[3]) > 0) {
        double raw_lat = atof(fields[2]);
        int deg = (int)(raw_lat / 100);
        g_gps.latitude = deg + (raw_lat - deg * 100) / 60.0;
        if (fields[3][0] == 'S') g_gps.latitude = -g_gps.latitude;
    }
    if (strlen(fields[4]) > 0 && strlen(fields[5]) > 0) {
        double raw_lon = atof(fields[4]);
        int deg = (int)(raw_lon / 100);
        g_gps.longitude = deg + (raw_lon - deg * 100) / 60.0;
        if (fields[5][0] == 'W') g_gps.longitude = -g_gps.longitude;
    }
    if (strlen(fields[6]) > 0) g_gps.fix_quality = atoi(fields[6]);
    if (strlen(fields[7]) > 0) g_gps.num_sat = atoi(fields[7]);

    g_gps.valid = (g_gps.fix_quality > 0);
    g_gps.updated = true;
}

static void gps_parse_gprmc(char *sentence)
{
    char *p = sentence;
    if (*p == '$') p++;

    char *fields[13];
    int cnt = 0;
    fields[cnt++] = p;
    while (*p && cnt < 13) {
        if (*p == ',') { *p = '\0'; fields[cnt++] = p + 1; }
        p++;
    }
    if (cnt < 10) return;
    if (fields[2][0] != 'A') return;

    if (strlen(fields[7]) > 0) g_gps.speed_kmh = atof(fields[7]) * 1.852f;
    if (strlen(fields[8]) > 0) g_gps.course = atof(fields[8]);
}

static void gps_process_line(char *line)
{
    if (strstr(line, "GGA")) gps_parse_gpgga(line);
    else if (strstr(line, "RMC")) gps_parse_gprmc(line);
}

static void gps_read_uart(void)
{
    static char line_buf[256];
    static uint16_t idx = 0;
    uint8_t ch;

    while (uart_read_byte(GPS_UART, &ch)) {
        if (ch == '$') { idx = 0; line_buf[idx++] = ch; }
        else if (idx > 0 && idx < 255) {
            line_buf[idx++] = ch;
            if (ch == '\n') {
                line_buf[idx] = '\0';
                if (idx > 6) gps_process_line(line_buf);
                idx = 0;
            }
        }
    }
}

/* ============ EKF初始化 ============ */

/**
 * @brief 初始化扩展卡尔曼滤波器
 *
 * 状态向量: [q0, q1, q2, q3, vn, ve, dlat, dlon]
 * 观测向量: [ax, ay, az, mx, my, mz] (IMU+磁力计)
 * GPS作为位置/速度更新（异步量测更新）
 */
static void ekf_init(EKF_t *ekf)
{
    memset(ekf, 0, sizeof(EKF_t));
    ekf->dt = SAMPLE_DT;

    /* 初始状态：单位四元数，零速度，零位置偏移 */
    ekf->x[0] = 1.0f;  /* q0 */
    ekf->x[1] = 0.0f;  /* q1 */
    ekf->x[2] = 0.0f;  /* q2 */
    ekf->x[3] = 0.0f;  /* q3 */

    /* 初始协方差 P */
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->P[i][i] = 1.0f;
    }

    /* 过程噪声 Q */
    ekf->Q[0][0] = 0.001f; ekf->Q[1][1] = 0.001f;
    ekf->Q[2][2] = 0.001f; ekf->Q[3][3] = 0.001f;
    ekf->Q[4][4] = 0.1f;   ekf->Q[5][5] = 0.1f;
    ekf->Q[6][6] = 0.01f;  ekf->Q[7][7] = 0.01f;

    /* 观测噪声 R（加速度计+磁力计） */
    ekf->R[0][0] = 0.5f; ekf->R[1][1] = 0.5f; ekf->R[2][2] = 0.5f;
    ekf->R[3][3] = 1.0f; ekf->R[4][4] = 1.0f; ekf->R[5][5] = 1.0f;
}

/* ============ EKF预测步骤 ============ */

/**
 * @brief EKF预测（时间更新）
 *
 * 状态转移基于角速度积分四元数和加速度积分速度/位置
 *
 * @param gx, gy, gz 陀螺仪角速度（rad/s）
 * @param ax, ay, az 加速度（g，用于重力补偿）
 */
static void ekf_predict(EKF_t *ekf, float gx, float gy, float gz,
                         float ax, float ay, float az)
{
    float dt = ekf->dt;
    float *q = &ekf->x[0];  /* 四元数 */
    float *vn = &ekf->x[4]; /* 北向速度 */
    float *ve = &ekf->x[5]; /* 东向速度 */

    /* 步骤1：四元数更新（一阶龙格-库塔） */
    float q_new[4];
    float half_dt = 0.5f * dt;
    q_new[0] = q[0] + (-q[1]*gx - q[2]*gy - q[3]*gz) * half_dt;
    q_new[1] = q[1] + ( q[0]*gx + q[2]*gz - q[3]*gy) * half_dt;
    q_new[2] = q[2] + ( q[0]*gy - q[1]*gz + q[3]*gx) * half_dt;
    q_new[3] = q[3] + ( q[0]*gz + q[1]*gy - q[2]*gx) * half_dt;
    quat_normalize(q_new);
    memcpy(q, q_new, 4 * sizeof(float));

    /* 步骤2：从四元数计算旋转矩阵（用于将加速度转到导航系） */
    float R[3][3];
    R[0][0] = 1 - 2*(q[2]*q[2] + q[3]*q[3]);
    R[0][1] = 2*(q[1]*q[2] - q[0]*q[3]);
    R[0][2] = 2*(q[1]*q[3] + q[0]*q[2]);
    R[1][0] = 2*(q[1]*q[2] + q[0]*q[3]);
    R[1][1] = 1 - 2*(q[1]*q[1] + q[3]*q[3]);
    R[1][2] = 2*(q[2]*q[3] - q[0]*q[1]);
    R[2][0] = 2*(q[1]*q[3] - q[0]*q[2]);
    R[2][1] = 2*(q[2]*q[3] + q[0]*q[1]);
    R[2][2] = 1 - 2*(q[1]*q[1] + q[2]*q[2]);

    /* 步骤3：将加速度转到导航坐标系，减去重力 */
    float a_nav[3];
    a_nav[0] = (R[0][0]*ax + R[0][1]*ay + R[0][2]*az) * 9.81f;  /* 北向加速度 */
    a_nav[1] = (R[1][0]*ax + R[1][1]*ay + R[1][2]*az) * 9.81f;  /* 东向加速度 */
    /* a_nav[2] 减去重力后的垂直加速度（此处忽略） */
    a_nav[0] -= 0;  /* 水平面无重力分量 */

    /* 步骤4：速度和位置积分 */
    *vn += a_nav[0] * dt;
    *ve += a_nav[1] * dt;
    ekf->x[6] += (*vn / EARTH_RADIUS) * dt;   /* dlat (rad) */
    ekf->x[7] += (*ve / (EARTH_RADIUS * cosf(ekf->x[6]))) * dt; /* dlon (rad) */

    /* 步骤5：协方差预测 P = F*P*F' + Q（简化：用单位阵近似F） */
    /* 这里简化处理，直接加Q */
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->P[i][i] += ekf->Q[i][i];
        /* 限制协方差增长 */
        if (ekf->P[i][i] > 100.0f) ekf->P[i][i] = 100.0f;
    }
}

/* ============ EKF量测更新（IMU+磁力计） ============ */

/**
 * @brief EKF量测更新
 *
 * 观测向量 z = [ax, ay, az, mx, my, mz]
 * 预测观测 h(x) 基于当前四元数计算的重力方向和磁场方向
 */
static void ekf_update_imu_mag(EKF_t *ekf,
                                float ax, float ay, float az,
                                float mx, float my, float mz)
{
    float *q = &ekf->x[0];

    /* 归一化加速度 */
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_norm < 0.01f) return;
    float nax = ax/acc_norm, nay = ay/acc_norm, naz = az/acc_norm;

    /* 归一化磁场 */
    float mag_norm = sqrtf(mx*mx + my*my + mz*mz);
    if (mag_norm < 0.001f) return;
    float nmx = mx/mag_norm, nmy = my/mag_norm, nmz = mz/mag_norm;

    /* 预测的重力方向（四元数旋转的z轴在体坐标系的投影） */
    float hx[6];
    hx[0] = 2*(q[1]*q[3] - q[0]*q[2]);    /* 预测ax */
    hx[1] = 2*(q[0]*q[1] + q[2]*q[3]);    /* 预测ay */
    hx[2] = 1 - 2*(q[1]*q[1] + q[2]*q[2]); /* 预测az */

    /* 预测磁场方向（简化） */
    float mag_ref_x = cosf(0);  /* 假设磁北在导航系x轴 */
    float mag_ref_z = 0;
    hx[3] = mag_ref_x * (1 - 2*(q[2]*q[2] + q[3]*q[3])) + mag_ref_z * 2*(q[1]*q[3] + q[0]*q[2]);
    hx[4] = mag_ref_x * 2*(q[1]*q[2] + q[0]*q[3]) + mag_ref_z * 2*(q[2]*q[3] - q[0]*q[1]);
    hx[5] = mag_ref_x * 2*(q[1]*q[3] - q[0]*q[2]) + mag_ref_z * (1 - 2*(q[1]*q[1] + q[2]*q[2]));

    /* 新息 y = z - h(x) */
    float y[6] = {
        nax - hx[0], nay - hx[1], naz - hx[2],
        nmx - hx[3], nmy - hx[4], nmz - hx[5]
    };

    /* 简化的卡尔曼增益计算（仅更新四元数部分） */
    /* K = P * H^T * (H*P*H^T + R)^-1 */
    /* 简化：直接用新息修正四元数 */
    float gain = 0.02f;  /* 修正增益 */

    /* 修正四元数（梯度下降思想） */
    float ex = (nay * hx[2] - naz * hx[1]);
    float ey = (naz * hx[0] - nax * hx[2]);
    float ez = (nax * hx[1] - nay * hx[0]);

    /* 磁力计修正 */
    float emx = (nmy * hx[5] - nmz * hx[4]);
    float emy = (nmz * hx[3] - nmx * hx[5]);
    float emz = (nmx * hx[4] - nmy * hx[3]);

    ex += emx * 0.5f;
    ey += emy * 0.5f;
    ez += emz * 0.5f;

    /* 应用修正 */
    q[0] += (-q[1]*ex - q[2]*ey - q[3]*ez) * gain;
    q[1] += ( q[0]*ex + q[2]*ez - q[3]*ey) * gain;
    q[2] += ( q[0]*ey - q[1]*ez + q[3]*ex) * gain;
    q[3] += ( q[0]*ez + q[1]*ey - q[2]*ex) * gain;
    quat_normalize(q);

    /* 更新协方差（简化） */
    for (int i = 0; i < 4; i++) {
        ekf->P[i][i] *= (1.0f - gain);
        if (ekf->P[i][i] < 0.001f) ekf->P[i][i] = 0.001f;
    }
}

/* ============ EKF GPS位置更新 ============ */

/**
 * @brief GPS位置量测更新
 *
 * 使用GPS经纬度修正EKF的位置估计
 */
static void ekf_update_gps(EKF_t *ekf, double lat, double lon,
                            float vn_gps, float ve_gps)
{
    /* 将GPS经纬度转换为相对于起始点的偏移（rad） */
    static double ref_lat = 0, ref_lon = 0;
    static bool ref_set = false;

    if (!ref_set) {
        ref_lat = lat;
        ref_lon = lon;
        ref_set = true;
        return;
    }

    double dlat_meas = (lat - ref_lat) * DEG_TO_RAD;
    double dlon_meas = (lon - ref_lon) * DEG_TO_RAD;

    /* 位置新息 */
    float y_lat = (float)dlat_meas - ekf->x[6];
    float y_lon = (float)dlon_meas - ekf->x[7];

    /* 简化的卡尔曼增益 */
    float K_pos = 0.1f;  /* 位置修正增益 */
    float K_vel = 0.05f; /* 速度修正增益 */

    /* 修正位置 */
    ekf->x[6] += K_pos * y_lat;
    ekf->x[7] += K_pos * y_lon;

    /* 修正速度（使用GPS速度和航向） */
    if (vn_gps != 0 || ve_gps != 0) {
        ekf->x[4] += K_vel * (vn_gps - ekf->x[4]);
        ekf->x[5] += K_vel * (ve_gps - ekf->x[5]);
    }
}

/* ============ 从四元数提取欧拉角 ============ */

static void quat_to_euler(float *q, float *roll, float *pitch, float *yaw)
{
    *roll  = atan2f(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2])) * RAD_TO_DEG;
    *pitch = asinf(2*(q[0]*q[2] - q[3]*q[1])) * RAD_TO_DEG;
    *yaw   = atan2f(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3])) * RAD_TO_DEG;
}

/* ============ OLED显示 ============ */

static void oled_display_fusion(void)
{
    char line[32];

    oled_clear();

    oled_show_string(0, 0, "Sensor Fusion", FONT_16);

    /* 姿态角 */
    snprintf(line, sizeof(line), "R:%+6.1f P:%+6.1f", g_fused_roll, g_fused_pitch);
    oled_show_string(0, 2, line, FONT_12);

    snprintf(line, sizeof(line), "Yaw:%+6.1f deg", g_fused_yaw);
    oled_show_string(0, 3, line, FONT_12);

    /* 速度 */
    snprintf(line, sizeof(line), "Vn:%+.1f Ve:%+.1f m/s", g_fused_vn, g_fused_ve);
    oled_show_string(0, 4, line, FONT_12);

    /* GPS状态 */
    snprintf(line, sizeof(line), "GPS: Sat=%d %s",
             g_gps.num_sat, g_gps.valid ? "FIX" : "---");
    oled_show_string(0, 5, line, FONT_12);

    /* 位置 */
    if (g_gps.valid) {
        snprintf(line, sizeof(line), "Lat:%.5f", g_fused_lat);
        oled_show_string(0, 6, line, FONT_12);
        snprintf(line, sizeof(line), "Lon:%.5f", g_fused_lon);
        oled_show_string(0, 7, line, FONT_12);
    } else {
        oled_show_string(0, 6, "Waiting GPS...", FONT_12);
        snprintf(line, sizeof(line), "Hdg:%.0f Mag:%.0f",
                 g_fused_yaw, g_mag.heading);
        oled_show_string(0, 7, line, FONT_12);
    }

    oled_refresh();
}

/* ============ 主函数 ============ */

int main(void)
{
    /* 系统初始化 */
    DL_SYSCTL_init();
    SysTick_Config(DL_SYSCTL_getMCLKFreq() / 1000);

    /* I2C初始化 */
    i2c_master_init(I2C0);

    /* GPS UART初始化 */
    uart_init(GPS_UART, 9600);

    /* OLED初始化 */
    oled_init();
    oled_clear();
    oled_show_string(0, 0, "Sensor Fusion", FONT_16);
    oled_show_string(0, 2, "MPU6050+QMC5883L", FONT_12);
    oled_show_string(0, 3, "+GPS + EKF", FONT_12);
    oled_show_string(0, 5, "Initializing...", FONT_12);
    oled_refresh();

    /* 传感器初始化 */
    mpu6050_init();
    qmc5883l_init();
    delay_ms(500);

    /* EKF初始化 */
    ekf_init(&g_ekf);

    /* 主循环计数器 */
    uint32_t loop_cnt = 0;
    uint32_t last_display_ms = 0;

    while (1) {
        uint32_t now_ms = get_tick();

        /* ---- 步骤1：读取所有传感器 ---- */
        mpu6050_read();
        qmc5883l_read();
        gps_read_uart();

        /* ---- 步骤2：角速度转换为rad/s ---- */
        float gx_rad = g_imu.gx * DEG_TO_RAD;
        float gy_rad = g_imu.gy * DEG_TO_RAD;
        float gz_rad = g_imu.gz * DEG_TO_RAD;

        /* ---- 步骤3：EKF预测 ---- */
        ekf_predict(&g_ekf, gx_rad, gy_rad, gz_rad,
                    g_imu.ax, g_imu.ay, g_imu.az);

        /* ---- 步骤4：EKF量测更新（IMU+磁力计） ---- */
        ekf_update_imu_mag(&g_ekf,
            g_imu.ax, g_imu.ay, g_imu.az,
            g_mag.mx, g_mag.my, g_mag.mz);

        /* ---- 步骤5：GPS量测更新（有新数据时） ---- */
        if (g_gps.updated && g_gps.valid) {
            /* GPS速度分解到北向/东向 */
            float vn_gps = g_gps.speed_kmh / 3.6f * cosf(g_gps.course * DEG_TO_RAD);
            float ve_gps = g_gps.speed_kmh / 3.6f * sinf(g_gps.course * DEG_TO_RAD);

            ekf_update_gps(&g_ekf, g_gps.latitude, g_gps.longitude, vn_gps, ve_gps);
            g_gps.updated = false;

            /* 更新融合位置 */
            static double ref_lat = 0, ref_lon = 0;
            static bool ref_set = false;
            if (!ref_set) { ref_lat = g_gps.latitude; ref_lon = g_gps.longitude; ref_set = true; }
            g_fused_lat = ref_lat + g_ekf.x[6] * RAD_TO_DEG;
            g_fused_lon = ref_lon + g_ekf.x[7] * RAD_TO_DEG;
        }

        /* ---- 步骤6：提取融合结果 ---- */
        quat_to_euler(g_ekf.x, &g_fused_roll, &g_fused_pitch, &g_fused_yaw);
        g_fused_vn = g_ekf.x[4];
        g_fused_ve = g_ekf.x[5];

        /* ---- 步骤7：OLED显示（10Hz） ---- */
        if (now_ms - last_display_ms >= 100) {
            last_display_ms = now_ms;
            oled_display_fusion();
        }

        /* 控制采样率 ~100Hz */
        loop_cnt++;
        delay_ms(10);
    }

    return 0;
}
