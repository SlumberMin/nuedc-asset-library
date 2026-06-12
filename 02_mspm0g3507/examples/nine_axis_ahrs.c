/**
 * @file nine_axis_ahrs.c
 * @brief 九轴AHRS姿态解算 - MSPM0G3507系统集成示例
 *
 * 功能：MPU9250(6轴IMU) + AK8963(磁力计) + Mahony互补滤波 + 四元数姿态 + OLED显示
 * 硬件：MSPM0G3507 + MPU9250(I2C) + SSD1306 OLED(I2C) + 按键
 *
 * 接线：
 *   MPU9250 SDA  -> PA0  (I2C0)
 *   MPU9250 SCL  -> PA1  (I2C0)
 *   MPU9250 AD0  -> GND  (地址0x68)
 *   MPU9250 INT  -> PA11 (GPIO中断，可选)
 *   OLED SDA     -> PA0  (与MPU9250共用I2C总线)
 *   OLED SCL     -> PA1
 *   按键-归零    -> PA12 (GPIO, 低有效)
 *
 * 算法：
 *   Mahony互补滤波器 - 融合陀螺仪、加速度计、磁力计数据
 *   输出四元数 -> 转换为欧拉角(roll, pitch, yaw)
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== MPU9250 I2C地址 ========== */
#define MPU9250_ADDR       0x68    /* AD0=GND */
#define AK8963_ADDR        0x0C    /* 磁力计地址 */

/* ========== MPU9250寄存器 ========== */
#define SMPLRT_DIV         0x19
#define CONFIG             0x1A
#define GYRO_CONFIG        0x1B
#define ACCEL_CONFIG       0x1C
#define ACCEL_CONFIG2      0x1D
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define ACCEL_XOUT_H       0x3B
#define GYRO_XOUT_H        0x43
#define WHO_AM_I           0x75
#define PWR_MGMT_1         0x6B
#define PWR_MGMT_2         0x6C

/* ========== AK8963寄存器 ========== */
#define AK8963_CNTL1       0x0A
#define AK8963_CNTL2       0x0B
#define AK8963_ST1         0x02
#define AK8963_HXL         0x03
#define AK8963_ASAX        0x10

/* ========== 量程配置 ========== */
#define ACCEL_FS_2G        0x00    /* ±2g */
#define ACCEL_FS_4G        0x08
#define ACCEL_FS_8G        0x10
#define ACCEL_FS_16G       0x18
#define GYRO_FS_250DPS     0x00    /* ±250°/s */
#define GYRO_FS_500DPS     0x08
#define GYRO_FS_1000DPS    0x10
#define GYRO_FS_2000DPS    0x18

/* ========== Mahony滤波参数 ========== */
#define TWO_KP             (2.0f * 10.0f)   /* 比例增益（加速度计） */
#define TWO_KI             (2.0f * 0.3f)    /* 积分增益 */
#define SAMPLE_RATE_HZ     200              /* 采样率 */

/* ========== 数据结构 ========== */
typedef struct {
    float x, y, z;
} Vec3f_t;

typedef struct {
    float w, x, y, z;
} Quaternion_t;

typedef struct {
    float roll;     /* 横滚角 ° */
    float pitch;    /* 俯仰角 ° */
    float yaw;      /* 航向角 ° */
} EulerAngles_t;

/* ========== 全局变量 ========== */
static volatile uint32_t gTickMs = 0;

/* 传感器原始数据 */
static Vec3f_t gAccel;      /* 加速度 (g) */
static Vec3f_t gGyro;       /* 角速度 (°/s) */
static Vec3f_t gMag;        /* 磁力计 (uT) */

/* 姿态 */
static Quaternion_t gQuat = {1.0f, 0.0f, 0.0f, 0.0f};
static EulerAngles_t gEuler;
static float gHeadingOffset = 0.0f;  /* 航向归零偏移 */

/* Mahony积分误差 */
static float gIx = 0, gIy = 0, gIz = 0;

/* 采样控制 */
static volatile bool gDataReady = false;

/* =================================================================
 * 基础延时
 * ================================================================= */

void SysTick_Handler(void) { gTickMs++; }

static void delay_ms(uint32_t ms) {
    uint32_t s = gTickMs;
    while ((gTickMs - s) < ms);
}

/* =================================================================
 * I2C驱动
 * ================================================================= */

/**
 * @brief I2C写寄存器
 */
static bool I2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    DL_I2C_setTargetAddress(I2C0, dev_addr);
    while (DL_I2C_isControllerBusy(I2C0));

    DL_I2C_transmitControllerData(I2C0, reg);
    while (DL_I2C_isControllerBusy(I2C0));
    DL_I2C_transmitControllerData(I2C0, val);
    while (DL_I2C_isControllerBusy(I2C0));

    return true;
}

/**
 * @brief I2C读寄存器
 */
static uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg) {
    DL_I2C_setTargetAddress(I2C0, dev_addr);
    while (DL_I2C_isControllerBusy(I2C0));

    /* 写寄存器地址 */
    DL_I2C_transmitControllerData(I2C0, reg);
    while (DL_I2C_isControllerBusy(I2C0));

    /* 读数据 */
    DL_I2C_startControllerTransfer(I2C0, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (!DL_I2C_isControllerRXFIFOEmpty(I2C0));
    return (uint8_t)DL_I2C_receiveControllerData(I2C0);
}

/**
 * @brief I2C读多个字节
 */
static void I2C_ReadBuf(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    DL_I2C_setTargetAddress(I2C0, dev_addr);
    while (DL_I2C_isControllerBusy(I2C0));

    /* 写寄存器地址 */
    DL_I2C_transmitControllerData(I2C0, reg);
    while (DL_I2C_isControllerBusy(I2C0));

    /* 读数据 */
    DL_I2C_startControllerTransfer(I2C0, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    for (uint8_t i = 0; i < len; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C0));
        buf[i] = (uint8_t)DL_I2C_receiveControllerData(I2C0);
    }
}

/* =================================================================
 * MPU9250驱动
 * ================================================================= */

/**
 * @brief 初始化MPU9250
 */
static bool MPU9250_Init(void) {
    uint8_t whoami;

    /* 检测WHO_AM_I */
    whoami = I2C_ReadReg(MPU9250_ADDR, WHO_AM_I);
    if (whoami != 0x71) {
        /* MPU9250返回0x71，MPU6500返回0x70 */
        if (whoami != 0x70) return false;
    }

    /* 复位 */
    I2C_WriteReg(MPU9250_ADDR, PWR_MGMT_1, 0x80);
    delay_ms(100);

    /* 唤醒，使用内部时钟 */
    I2C_WriteReg(MPU9250_ADDR, PWR_MGMT_1, 0x01);
    delay_ms(10);

    /* 采样率分频：1kHz / (1+4) = 200Hz */
    I2C_WriteReg(MPU9250_ADDR, SMPLRT_DIV, 4);

    /* 低通滤波：带宽41Hz */
    I2C_WriteReg(MPU9250_ADDR, CONFIG, 0x03);

    /* 陀螺仪量程 ±500°/s */
    I2C_WriteReg(MPU9250_ADDR, GYRO_CONFIG, GYRO_FS_500DPS);

    /* 加速度计量程 ±4g */
    I2C_WriteReg(MPU9250_ADDR, ACCEL_CONFIG, ACCEL_FS_4G);
    I2C_WriteReg(MPU9250_ADDR, ACCEL_CONFIG2, 0x03);

    /* 使能磁力计旁路（I2C Master） */
    I2C_WriteReg(MPU9250_ADDR, INT_PIN_CFG, 0x02);  /* I2C_BYPASS_EN */

    return true;
}

/**
 * @brief 读取原始加速度和陀螺仪数据
 */
static void MPU9250_ReadMotion(void) {
    uint8_t buf[14];
    int16_t ax, ay, az, gx, gy, gz;

    I2C_ReadBuf(MPU9250_ADDR, ACCEL_XOUT_H, buf, 14);

    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);
    /* buf[6:7] = 温度 */
    gx = (int16_t)((buf[8] << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* 转换为物理单位 */
    /* ±4g -> 8192 LSB/g */
    gAccel.x = ax / 8192.0f;
    gAccel.y = ay / 8192.0f;
    gAccel.z = az / 8192.0f;

    /* ±500°/s -> 65.536 LSB/(°/s) */
    gGyro.x = gx / 65.536f;
    gGyro.y = gy / 65.536f;
    gGyro.z = gz / 65.536f;
}

/* =================================================================
 * AK8963磁力计驱动
 * ================================================================= */

static float gMagCal[3] = {1.0f, 1.0f, 1.0f};  /* 灵敏度校准 */

/**
 * @brief 初始化AK8963磁力计
 */
static bool AK8963_Init(void) {
    uint8_t asa[3];

    /* 复位 */
    I2C_WriteReg(AK8963_ADDR, AK8963_CNTL2, 0x01);
    delay_ms(10);

    /* 进入Fuse ROM访问模式，读取灵敏度校准 */
    I2C_WriteReg(AK8963_ADDR, AK8963_CNTL1, 0x0F);
    delay_ms(10);

    I2C_ReadBuf(AK8963_ADDR, AK8963_ASAX, asa, 3);
    gMagCal[0] = (asa[0] - 128) / 256.0f + 1.0f;
    gMagCal[1] = (asa[1] - 128) / 256.0f + 1.0f;
    gMagCal[2] = (asa[2] - 128) / 256.0f + 1.0f;

    /* 设置为连续测量模式2，16bit输出，100Hz */
    I2C_WriteReg(AK8963_ADDR, AK8963_CNTL1, 0x16);
    delay_ms(10);

    return true;
}

/**
 * @brief 读取磁力计数据
 */
static bool AK8963_ReadMag(void) {
    uint8_t st1;
    uint8_t buf[7];
    int16_t mx, my, mz;
    uint8_t st2;

    /* 检查数据就绪 */
    st1 = I2C_ReadReg(AK8963_ADDR, AK8963_ST1);
    if (!(st1 & 0x01)) return false;

    /* 读取6字节磁力计数据 + ST2 */
    I2C_ReadBuf(AK8963_ADDR, AK8963_HXL, buf, 7);

    mx = (int16_t)((buf[1] << 8) | buf[0]);
    my = (int16_t)((buf[3] << 8) | buf[2]);
    mz = (int16_t)((buf[5] << 8) | buf[4]);
    st2 = buf[6];

    /* 检查溢出 */
    if (st2 & 0x08) return false;

    /* 转换为uT（16bit模式：0.15 uT/LSB），乘以校准因子 */
    gMag.x = mx * 0.15f * gMagCal[0];
    gMag.y = my * 0.15f * gMagCal[1];
    gMag.z = mz * 0.15f * gMagCal[2];

    return true;
}

/* =================================================================
 * Mahony AHRS互补滤波器
 * ================================================================= */

/**
 * @brief 快速平方根倒数
 */
static float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief Mahony AHRS更新
 * @param ax,ay,az 加速度 (任意单位，会被归一化)
 * @param gx,gy,gz 陀螺仪 (rad/s)
 * @param mx,my,mz 磁力计 (任意单位，会被归一化)
 */
static void MahonyAHRS_Update(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               float mx, float my, float mz)
{
    float q0 = gQuat.w, q1 = gQuat.x, q2 = gQuat.y, q3 = gQuat.z;
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* 如果加速度计数据无效则跳过 */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        /* 归一化加速度计 */
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* 归一化磁力计 */
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        /* 预计算四元数乘积 */
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        /* 参考磁场方向 */
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        /* 估计的重力和磁场方向 */
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        /* 误差 = 测量叉积估计 */
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        /* 积分误差 */
        if (TWO_KI > 0.0f) {
            gIx += TWO_KI * halfex * (1.0f / SAMPLE_RATE_HZ);
            gIy += TWO_KI * halfey * (1.0f / SAMPLE_RATE_HZ);
            gIz += TWO_KI * halfez * (1.0f / SAMPLE_RATE_HZ);
            gx += gIx;
            gy += gIy;
            gz += gIz;
        }

        /* 比例误差反馈 */
        gx += TWO_KP * halfex;
        gy += TWO_KP * halfey;
        gz += TWO_KP * halfez;
    }

    /* 积分四元数（一阶龙格库塔） */
    gx *= (0.5f / SAMPLE_RATE_HZ);
    gy *= (0.5f / SAMPLE_RATE_HZ);
    gz *= (0.5f / SAMPLE_RATE_HZ);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    /* 归一化四元数 */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    gQuat.w = q0 * recipNorm;
    gQuat.x = q1 * recipNorm;
    gQuat.y = q2 * recipNorm;
    gQuat.z = q3 * recipNorm;
}

/**
 * @brief 从四元数计算欧拉角
 */
static void Quaternion_ToEuler(void) {
    float q0 = gQuat.w, q1 = gQuat.x, q2 = gQuat.y, q3 = gQuat.z;

    /* Roll (x轴旋转) */
    gEuler.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2958f;

    /* Pitch (y轴旋转) */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        gEuler.pitch = copysignf(90.0f, sinp);
    else
        gEuler.pitch = asinf(sinp) * 57.2958f;

    /* Yaw (z轴旋转) */
    gEuler.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.2958f;

    /* 应用航向偏移 */
    gEuler.yaw -= gHeadingOffset;
    if (gEuler.yaw > 180.0f)  gEuler.yaw -= 360.0f;
    if (gEuler.yaw < -180.0f) gEuler.yaw += 360.0f;
}

/* =================================================================
 * 角度转弧度（陀螺仪输入需要）
 * ================================================================= */
#define DEG_TO_RAD  0.017453293f

/* =================================================================
 * UART输出
 * ================================================================= */

static void UART_Print(const char *str) {
    while (*str) {
        DL_UART_transmitData(UART0, *str++);
        while (!DL_UART_isTXEmpty(UART0));
    }
}

/* =================================================================
 * OLED显示（简化版，与weight_scale.c类似）
 * ================================================================= */

static void OLED_WriteCmd(uint8_t cmd) {
    DL_I2C_setTargetAddress(I2C0, 0x3C);
    while (DL_I2C_isControllerBusy(I2C0));
    DL_I2C_transmitControllerData(I2C0, 0x00);
    while (DL_I2C_isControllerBusy(I2C0));
    DL_I2C_transmitControllerData(I2C0, cmd);
    while (DL_I2C_isControllerBusy(I2C0));
}

static void OLED_WriteDataByte(uint8_t data) {
    DL_I2C_setTargetAddress(I2C0, 0x3C);
    while (DL_I2C_isControllerBusy(I2C0));
    DL_I2C_transmitControllerData(I2C0, 0x40);
    while (DL_I2C_isControllerBusy(I2C0));
    DL_I2C_transmitControllerData(I2C0, data);
    while (DL_I2C_isControllerBusy(I2C0));
}

static void OLED_SetCursor(uint8_t page, uint8_t col) {
    OLED_WriteCmd(0xB0 + page);
    OLED_WriteCmd(0x00 + (col & 0x0F));
    OLED_WriteCmd(0x10 + ((col >> 4) & 0x0F));
}

static void OLED_Clear(void) {
    for (uint8_t p = 0; p < 8; p++) {
        OLED_SetCursor(p, 0);
        for (uint8_t c = 0; c < 128; c++) {
            OLED_WriteDataByte(0x00);
        }
    }
}

static void OLED_Init(void) {
    delay_ms(100);
    OLED_WriteCmd(0xAE);
    OLED_WriteCmd(0x20); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xB0); OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0x00); OLED_WriteCmd(0x10); OLED_WriteCmd(0x40);
    OLED_WriteCmd(0x81); OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xA1); OLED_WriteCmd(0xA6); OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xA4); OLED_WriteCmd(0xD3); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xD5); OLED_WriteCmd(0xF0);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0x22);
    OLED_WriteCmd(0xDA); OLED_WriteCmd(0x12);
    OLED_WriteCmd(0xDB); OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x8D); OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xAF);
}

/* =================================================================
 * 定时器中断（采样触发）
 * ================================================================= */

/**
 * @brief TIMER_A0中断 - 产生200Hz采样触发
 */
void TIMER_A0_IRQHandler(void) {
    DL_TimerA_clearInterruptStatus(TIMER_A0, DL_TIMER_INTERRUPT_ZERO_EVENT);
    gDataReady = true;
}

/* =================================================================
 * 主函数
 * ================================================================= */
int main(void) {
    /* 系统初始化 */
    DL_SYSCFG_init();
    SysTick_Config(32000);  /* 1ms tick */

    /* I2C初始化 */
    DL_I2C_enableController(I2C0);

    /* UART初始化 */
    NVIC_EnableIRQ(UART0_IRQn);

    /* 定时器初始化（200Hz采样中断） */
    NVIC_EnableIRQ(TIMER_A0_IRQn);

    /* GPIO（按键） */
    DL_GPIO_initDigitalInputFeatures(DL_GPIO_PIN_12,
        DL_GPIO_RESISTOR_PULLUP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* OLED初始化 */
    OLED_Init();
    OLED_Clear();

    UART_Print("\r\n=== 9-Axis AHRS System ===\r\n");

    /* MPU9250初始化 */
    if (!MPU9250_Init()) {
        UART_Print("ERROR: MPU9250 not found!\r\n");
        while (1);
    }
    UART_Print("MPU9250 OK\r\n");

    /* AK8963初始化 */
    if (!AK8963_Init()) {
        UART_Print("WARNING: AK8963 init failed\r\n");
    } else {
        UART_Print("AK8963 OK\r\n");
    }

    /* 等待传感器稳定 */
    delay_ms(500);

    /* 预热：读取几次数据使滤波器收敛 */
    for (uint8_t i = 0; i < 100; i++) {
        MPU9250_ReadMotion();
        AK8963_ReadMag();
        MahonyAHRS_Update(
            gAccel.x, gAccel.y, gAccel.z,
            gGyro.x * DEG_TO_RAD, gGyro.y * DEG_TO_RAD, gGyro.z * DEG_TO_RAD,
            gMag.x, gMag.y, gMag.z
        );
        delay_ms(5);
    }
    UART_Print("AHRS converged\r\n");

    /* 显示标题 */
    OLED_Clear();
    /* 实际项目中应在此显示 "R:xx P:xx Y:xx" 等 */

    uint32_t last_output = 0;
    const uint32_t OUTPUT_INTERVAL = 100;  /* 100ms输出一次 */
    char uart_buf[80];

    /* ===== 主循环 ===== */
    while (1) {
        /* 定时读取传感器并更新AHRS */
        if (gDataReady) {
            gDataReady = false;

            /* 读取IMU */
            MPU9250_ReadMotion();
            AK8963_ReadMag();

            /* Mahony滤波更新 */
            MahonyAHRS_Update(
                gAccel.x, gAccel.y, gAccel.z,
                gGyro.x * DEG_TO_RAD, gGyro.y * DEG_TO_RAD, gGyro.z * DEG_TO_RAD,
                gMag.x, gMag.y, gMag.z
            );

            /* 计算欧拉角 */
            Quaternion_ToEuler();
        }

        /* 航向归零按键 */
        if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_12)) {
            static uint32_t last_btn = 0;
            if ((gTickMs - last_btn) > 500) {
                gHeadingOffset = gEuler.yaw + gHeadingOffset;
                UART_Print("Heading zeroed!\r\n");
                last_btn = gTickMs;
            }
        }

        /* 定时输出 */
        if ((gTickMs - last_output) >= OUTPUT_INTERVAL) {
            /* UART输出欧拉角 */
            snprintf(uart_buf, sizeof(uart_buf),
                     "R:%+7.2f P:%+7.2f Y:%+7.2f\r\n",
                     gEuler.roll, gEuler.pitch, gEuler.yaw);
            UART_Print(uart_buf);

            /* 输出四元数 */
            snprintf(uart_buf, sizeof(uart_buf),
                     "Q: %.4f %.4f %.4f %.4f\r\n",
                     gQuat.w, gQuat.x, gQuat.y, gQuat.z);
            UART_Print(uart_buf);

            last_output = gTickMs;
        }

        delay_ms(1);
    }
}
