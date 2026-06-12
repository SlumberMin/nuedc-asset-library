/**
 * @file imu_ahrs_demo.c
 * @brief IMU姿态估计示例 — MPU6050 + 互补滤波/Mahony + OLED显示
 * @target MSPM0G3507
 *
 * 硬件连接：
 *   MPU6050 (I2C):
 *     VCC -> 3.3V    GND -> GND
 *     SCL -> PB2 (I2C0_SCL)   SDA -> PB3 (I2C0_SDA)
 *     AD0 -> GND (地址0x68)
 *   OLED SSD1306 (I2C, 共用I2C总线):
 *     SCL -> PB2    SDA -> PB3
 *     地址0x3C（与MPU6050不冲突）
 *
 * 功能：
 *   1. 读取MPU6050六轴原始数据（加速度+陀螺仪）
 *   2. 互补滤波估计姿态角（Roll/Pitch/Yaw）
 *   3. Mahony互补滤波算法（更优的收敛性）
 *   4. OLED实时显示姿态角和加速度
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ============ 外设驱动头文件 ============ */
#include "drivers/oled_ssd1306.h"
#include "drivers/i2c_master.h"

/* ============ 硬件配置 ============ */
#define MPU6050_I2C     I2C0
#define MPU6050_ADDR    0x68
#define OLED_I2C        I2C0

/* ============ MPU6050寄存器地址 ============ */
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CFG     0x1B
#define MPU6050_REG_ACCEL_CFG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

/* ============ MPU6050配置常量 ============ */
#define ACCEL_SCALE_2G      16384.0f   /* LSB/g @ ±2g */
#define GYRO_SCALE_250DPS   131.0f     /* LSB/(°/s) @ ±250°/s */
#define DEG_TO_RAD          0.0174532925f
#define RAD_TO_DEG          57.29577951f

/* ============ Mahony滤波参数 ============ */
#define MAHONY_KP           2.0f       /* 比例增益 */
#define MAHONY_KI           0.005f     /* 积分增益 */
#define SAMPLE_RATE_HZ      100        /* 采样率100Hz */

/* ============ 数据结构 ============ */
typedef struct {
    int16_t ax, ay, az;     /* 原始加速度值 */
    int16_t gx, gy, gz;     /* 原始陀螺仪值 */
    int16_t temperature;    /* 原始温度值 */

    float ax_g, ay_g, az_g; /* 加速度（单位: g） */
    float gx_dps, gy_dps, gz_dps; /* 角速度（单位: °/s） */
    float temp_c;           /* 温度（单位: °C） */
} MPU6050_Data_t;

typedef struct {
    float roll;             /* 横滚角（°） */
    float pitch;            /* 俯仰角（°） */
    float yaw;              /* 偏航角（°） */
} Attitude_t;

/* ============ 全局变量 ============ */
static MPU6050_Data_t g_imu;
static Attitude_t g_attitude_comp;   /* 互补滤波结果 */
static Attitude_t g_attitude_mahony; /* Mahony滤波结果 */

/* Mahony算法积分误差 */
static float ex_int = 0, ey_int = 0, ez_int = 0;

/* 四元数（Mahony输出） */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* 互补滤波中的偏航角累积 */
static float g_yaw_comp = 0.0f;
static float g_dt = 1.0f / SAMPLE_RATE_HZ;

/* ============ MPU6050底层读写 ============ */

/**
 * @brief 向MPU6050写入一个字节
 */
static void mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    i2c_master_write_reg(MPU6050_I2C, MPU6050_ADDR, reg, &data, 1);
}

/**
 * @brief 从MPU6050读取多个字节
 */
static void mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_master_read_reg(MPU6050_I2C, MPU6050_ADDR, reg, buf, len);
}

/* ============ MPU6050初始化 ============ */

/**
 * @brief 初始化MPU6050传感器
 *
 * 配置：加速度±2g，陀螺仪±250°/s，采样率100Hz
 */
static void mpu6050_init(void)
{
    uint8_t id;

    /* 复位MPU6050 */
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80);
    delay_ms(100);

    /* 唤醒，使用内部8MHz振荡器 */
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    delay_ms(10);

    /* 验证WHO_AM_I */
    mpu6050_read_regs(MPU6050_REG_WHO_AM_I, &id, 1);
    if (id != 0x68) {
        /* 错误处理：设备ID不匹配 */
        while (1) {
            DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN);
            delay_ms(200);
        }
    }

    /* 设置采样率分频：1kHz / (1+9) = 100Hz */
    mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 9);

    /* 低通滤波器配置：带宽5Hz */
    mpu6050_write_reg(MPU6050_REG_CONFIG, 0x06);

    /* 陀螺仪量程：±250°/s */
    mpu6050_write_reg(MPU6050_REG_GYRO_CFG, 0x00);

    /* 加速度量程：±2g */
    mpu6050_write_reg(MPU6050_REG_ACCEL_CFG, 0x00);
}

/* ============ MPU6050数据读取 ============ */

/**
 * @brief 读取MPU6050六轴原始数据并转换为物理单位
 */
static void mpu6050_read_data(void)
{
    uint8_t buf[14];

    /* 一次读取14字节：加速度(6) + 温度(2) + 陀螺仪(6) */
    mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 14);

    /* 合成16位有符号数据（大端序） */
    g_imu.ax = (int16_t)((buf[0] << 8) | buf[1]);
    g_imu.ay = (int16_t)((buf[2] << 8) | buf[3]);
    g_imu.az = (int16_t)((buf[4] << 8) | buf[5]);
    g_imu.temperature = (int16_t)((buf[6] << 8) | buf[7]);
    g_imu.gx = (int16_t)((buf[8] << 8) | buf[9]);
    g_imu.gy = (int16_t)((buf[10] << 8) | buf[11]);
    g_imu.gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* 转换为物理单位 */
    g_imu.ax_g = g_imu.ax / ACCEL_SCALE_2G;
    g_imu.ay_g = g_imu.ay / ACCEL_SCALE_2G;
    g_imu.az_g = g_imu.az / ACCEL_SCALE_2G;

    g_imu.gx_dps = g_imu.gx / GYRO_SCALE_250DPS;
    g_imu.gy_dps = g_imu.gy / GYRO_SCALE_250DPS;
    g_imu.gz_dps = g_imu.gz / GYRO_SCALE_250DPS;

    g_imu.temp_c = g_imu.temperature / 340.0f + 36.53f;
}

/* ============ 互补滤波 ============ */

/**
 * @brief 互补滤波姿态估计
 *
 * 原理：加速度计提供Roll和Pitch的低频参考（长期稳定），
 *       陀螺仪提供高频角速度（短期精确），
 *       用加权平均融合两者。
 *
 * Roll  = α*(Roll + ωx*dt) + (1-α)*atan2(ay, az)
 * Pitch = α*(Pitch + ωy*dt) + (1-α)*atan2(ax, az)
 * Yaw   = 仅靠陀螺仪积分（磁力计才能修正漂移）
 *
 * α = 0.98 典型值
 */
#define COMP_FILTER_ALPHA   0.98f

static void complementary_filter(void)
{
    float roll_acc, pitch_acc;

    /* 从加速度计计算参考角度 */
    roll_acc  = atan2f(g_imu.ay_g, g_imu.az_g) * RAD_TO_DEG;
    pitch_acc = atan2f(-g_imu.ax_g,
                       sqrtf(g_imu.ay_g * g_imu.ay_g + g_imu.az_g * g_imu.az_g)) * RAD_TO_DEG;

    /* 互补滤波融合 */
    g_attitude_comp.roll  = COMP_FILTER_ALPHA * (g_attitude_comp.roll + g_imu.gx_dps * g_dt)
                          + (1.0f - COMP_FILTER_ALPHA) * roll_acc;
    g_attitude_comp.pitch = COMP_FILTER_ALPHA * (g_attitude_comp.pitch + g_imu.gy_dps * g_dt)
                          + (1.0f - COMP_FILTER_ALPHA) * pitch_acc;

    /* Yaw仅靠陀螺仪积分（无磁力计修正） */
    g_yaw_comp += g_imu.gz_dps * g_dt;
    /* 限制Yaw在±180° */
    if (g_yaw_comp > 180.0f) g_yaw_comp -= 360.0f;
    if (g_yaw_comp < -180.0f) g_yaw_comp += 360.0f;
    g_attitude_comp.yaw = g_yaw_comp;
}

/* ============ Mahony互补滤波算法 ============ */

/**
 * @brief Mahony互补滤波算法（等效旋转矩阵的梯度下降法）
 *
 * 比简单互补滤波的优势：
 * 1. 全姿态工作（无万向锁问题）
 * 2. 积分反馈消除稳态误差
 * 3. 可扩展融合磁力计
 *
 * @param gx, gy, gz 陀螺仪（rad/s）
 * @param ax, ay, az 加速度（g）
 * @param dt 采样周期（s）
 */
static void mahony_ahrs_update(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float dt)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float halfT = 0.5f * dt;

    /* 步骤1：归一化加速度计数据 */
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm < 0.001f) return;  /* 加速度过小，跳过 */
    ax /= acc_norm;
    ay /= acc_norm;
    az /= acc_norm;

    /* 步骤2：从当前四元数估计重力方向向量 */
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 步骤3：计算误差向量（叉积） */
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    /* 步骤4：PI控制器 */
    ex_int += ex * MAHONY_KI * dt;
    ey_int += ey * MAHONY_KI * dt;
    ez_int += ez * MAHONY_KI * dt;

    /* 积分限幅防饱和 */
    if (ex_int > 0.5f) ex_int = 0.5f;
    if (ex_int < -0.5f) ex_int = -0.5f;
    if (ey_int > 0.5f) ey_int = 0.5f;
    if (ey_int < -0.5f) ey_int = -0.5f;
    if (ez_int > 0.5f) ez_int = 0.5f;
    if (ez_int < -0.5f) ez_int = -0.5f;

    /* 步骤5：修正陀螺仪数据 */
    gx = gx + MAHONY_KP * ex + ex_int;
    gy = gy + MAHONY_KP * ey + ey_int;
    gz = gz + MAHONY_KP * ez + ez_int;

    /* 步骤6：一阶龙格-库塔法更新四元数 */
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    /* 步骤7：归一化四元数 */
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    /* 步骤8：从四元数转换为欧拉角 */
    g_attitude_mahony.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                     1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    g_attitude_mahony.pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * RAD_TO_DEG;
    g_attitude_mahony.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                     1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

/* ============ OLED显示 ============ */

/**
 * @brief 在OLED上显示姿态信息
 *
 * 显示布局：
 *   行0: 标题
 *   行1: Roll (互补/Mahony)
 *   行2: Pitch (互补/Mahony)
 *   行3: Yaw (互补/Mahony)
 *   行4: 加速度 ax ay az
 *   行5: 角速度 gx gy gz
 *   行6: 温度
 */
static void oled_display_attitude(void)
{
    char line[32];

    oled_clear();

    /* 标题 */
    oled_show_string(0, 0, "IMU AHRS Demo", FONT_16);

    /* Roll */
    snprintf(line, sizeof(line), "R:%+6.1f M:%+6.1f",
             g_attitude_comp.roll, g_attitude_mahony.roll);
    oled_show_string(0, 2, line, FONT_12);

    /* Pitch */
    snprintf(line, sizeof(line), "P:%+6.1f M:%+6.1f",
             g_attitude_comp.pitch, g_attitude_mahony.pitch);
    oled_show_string(0, 3, line, FONT_12);

    /* Yaw */
    snprintf(line, sizeof(line), "Y:%+6.1f M:%+6.1f",
             g_attitude_comp.yaw, g_attitude_mahony.yaw);
    oled_show_string(0, 4, line, FONT_12);

    /* 加速度 */
    snprintf(line, sizeof(line), "A:%+5.2f %+5.2f %+5.2f",
             g_imu.ax_g, g_imu.ay_g, g_imu.az_g);
    oled_show_string(0, 5, line, FONT_12);

    /* 角速度 */
    snprintf(line, sizeof(line), "G:%+5.1f %+5.1f %+5.1f",
             g_imu.gx_dps, g_imu.gy_dps, g_imu.gz_dps);
    oled_show_string(0, 6, line, FONT_12);

    /* 温度 */
    snprintf(line, sizeof(line), "Temp: %.1f C", g_imu.temp_c);
    oled_show_string(0, 7, line, FONT_12);

    oled_refresh();
}

/* ============ 零偏校准 ============ */

/**
 * @brief MPU6050零偏校准
 *
 * 在静止状态下采集N个样本取平均，作为陀螺仪零偏
 */
#define CALIB_SAMPLES  500
static float g_gyro_offset_x = 0, g_gyro_offset_y = 0, g_gyro_offset_z = 0;
static float g_accel_offset_x = 0, g_accel_offset_y = 0, g_accel_offset_z = 0;

static void mpu6050_calibrate(void)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;

    oled_clear();
    oled_show_string(0, 0, "IMU AHRS Demo", FONT_16);
    oled_show_string(0, 3, "Calibrating...", FONT_12);
    oled_show_string(0, 4, "Keep still!", FONT_12);
    oled_refresh();

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        mpu6050_read_data();
        sum_gx += g_imu.gx_dps;
        sum_gy += g_imu.gy_dps;
        sum_gz += g_imu.gz_dps;
        sum_ax += g_imu.ax_g;
        sum_ay += g_imu.ay_g;
        sum_az += g_imu.az_g;
        delay_ms(2);  /* 100Hz采样 */
    }

    g_gyro_offset_x = sum_gx / CALIB_SAMPLES;
    g_gyro_offset_y = sum_gy / CALIB_SAMPLES;
    g_gyro_offset_z = sum_gz / CALIB_SAMPLES;

    /* 加速度z轴偏移：静止时应为1g */
    g_accel_offset_x = sum_ax / CALIB_SAMPLES;
    g_accel_offset_y = sum_ay / CALIB_SAMPLES;
    g_accel_offset_z = sum_az / CALIB_SAMPLES - 1.0f;  /* 减去1g */

    oled_clear();
    oled_show_string(0, 3, "Calibrate Done!", FONT_12);
    oled_refresh();
    delay_ms(500);
}

/* ============ 主函数 ============ */
int main(void)
{
    /* 系统初始化 */
    DL_SYSCTL_init();
    SysTick_Config(DL_SYSCTL_getMCLKFreq() / 1000);

    /* I2C初始化 */
    i2c_master_init(MPU6050_I2C);

    /* OLED初始化 */
    oled_init();
    oled_clear();
    oled_show_string(0, 0, "IMU AHRS Demo", FONT_16);
    oled_show_string(0, 2, "MPU6050 + Mahony", FONT_12);
    oled_refresh();
    delay_ms(500);

    /* MPU6050初始化 */
    mpu6050_init();

    /* 零偏校准 */
    mpu6050_calibrate();

    /* 主循环 */
    uint32_t last_time_ms = get_tick();
    uint32_t display_cnt = 0;

    while (1) {
        /* 计算实际dt */
        uint32_t now_ms = get_tick();
        g_dt = (now_ms - last_time_ms) / 1000.0f;
        if (g_dt <= 0.0f || g_dt > 0.1f) g_dt = 1.0f / SAMPLE_RATE_HZ;
        last_time_ms = now_ms;

        /* 读取IMU数据 */
        mpu6050_read_data();

        /* 零偏补偿 */
        g_imu.gx_dps -= g_gyro_offset_x;
        g_imu.gy_dps -= g_gyro_offset_y;
        g_imu.gz_dps -= g_gyro_offset_z;
        g_imu.ax_g -= g_accel_offset_x;
        g_imu.ay_g -= g_accel_offset_y;
        g_imu.az_g -= g_accel_offset_z;

        /* 互补滤波 */
        complementary_filter();

        /* Mahony滤波（输入为rad/s） */
        mahony_ahrs_update(
            g_imu.gx_dps * DEG_TO_RAD,
            g_imu.gy_dps * DEG_TO_RAD,
            g_imu.gz_dps * DEG_TO_RAD,
            g_imu.ax_g, g_imu.ay_g, g_imu.az_g,
            g_dt
        );

        /* 每10个周期更新一次显示（约10Hz） */
        display_cnt++;
        if (display_cnt >= 10) {
            display_cnt = 0;
            oled_display_attitude();
        }

        /* 控制采样率 ~100Hz */
        delay_ms(10);
    }

    return 0;
}
