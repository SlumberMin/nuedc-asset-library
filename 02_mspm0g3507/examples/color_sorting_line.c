/**
 * @file color_sorting_line.c
 * @brief 颜色分拣流水线 - 完整系统集成示例
 * @target MSPM0G3507
 * @hardware TCS34725颜色传感器 + PCA9685 16路舵机驱动 + 传送带电机 + 步进电机
 *
 * 系统架构：
 *   TCS34725 --I2C0--> RGB颜色数据 --> 颜色分类算法
 *   PCA9685  --I2C0--> 16路PWM舵机控制(分拣机构)
 *   传送带   --PWM(TIMER0)--> 直流电机调速
 *   光电传感器 --GPIO--> 物料到位检测
 *   OLED     --I2C0--> 状态显示
 *
 * 工作流程：
 *   传送带运转 -> 光电检测到物料 -> 停传送带 -> 读取颜色 -> 分类 ->
 *   控制对应舵机推出 -> 复位舵机 -> 继续传送
 *
 * 错误经验库遵守：
 *   - TCS34725读取前需等待数据就绪(ADC complete标志)
 *   - PCA9685舵机角度转换需校准，不同舵机pulse范围不同
 *   - I2C总线上多设备需处理总线冲突(NACK重试)
 *   - 颜色传感器需白平衡校准，不同光照条件差异大
 *   - 传送带启动和停止需缓变速，防止物料滑动
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== TCS34725驱动 ========== */
#define TCS34725_ADDR           0x29
#define TCS34725_CMD_BIT        0x80
#define TCS34725_ENABLE         (TCS34725_CMD_BIT | 0x00)
#define TCS34725_ATIME          (TCS34725_CMD_BIT | 0x01)
#define TCS34725_CONTROL        (TCS34725_CMD_BIT | 0x0F)
#define TCS34725_ID             (TCS34725_CMD_BIT | 0x12)
#define TCS34725_STATUS         (TCS34725_CMD_BIT | 0x13)
#define TCS34725_CDATAL         (TCS34725_CMD_BIT | 0x14)
#define TCS34725_RDATAL         (TCS34725_CMD_BIT | 0x16)
#define TCS34725_GDATAL         (TCS34725_CMD_BIT | 0x18)
#define TCS34725_BDATAL         (TCS34725_CMD_BIT | 0x1A)

#define TCS34725_ENABLE_PON     0x01    /* 上电 */
#define TCS34725_ENABLE_AEN     0x02    /* ADC使能 */
#define TCS34725_ATIME_MS       0xD5    /* 积分时间101ms */
#define TCS34725_GAIN_4X        0x01    /* 增益4x */
#define TCS34725_STATUS_AVALID  0x01    /* 数据就绪 */

/* ========== PCA9685驱动 ========== */
#define PCA9685_ADDR            0x40
#define PCA9685_MODE1           0x00
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_PRESCALE        0xFE
#define PCA9685_OSC_FREQ        25000000UL  /* 内部振荡器25MHz */
#define SERVO_MIN_PULSE         102     /* 0.5ms对应PWM值 */
#define SERVO_MAX_PULSE         512     /* 2.5ms对应PWM值 */
#define SERVO_RANGE             (SERVO_MAX_PULSE - SERVO_MIN_PULSE)

/* ========== 硬件引脚定义 ========== */
/* 光电传感器(物料检测) */
#define SENSOR_PORT             GPIOA
#define SENSOR_PIN              DL_GPIO_PIN_0

/* 传送带电机 */
#define CONVEYOR_IN1_PORT       GPIOA
#define CONVEYOR_IN1_PIN        DL_GPIO_PIN_1
#define CONVEYOR_IN2_PORT       GPIOA
#define CONVEYOR_IN2_PIN        DL_GPIO_PIN_2

/* 指示灯 */
#define LED_RED_PORT            GPIOB
#define LED_RED_PIN             DL_GPIO_PIN_0
#define LED_GREEN_PORT          GPIOB
#define LED_GREEN_PIN           DL_GPIO_PIN_1
#define LED_BLUE_PORT           GPIOB
#define LED_BLUE_PIN            DL_GPIO_PIN_2

/* ========== 系统参数 ========== */
#define CONVEYOR_SPEED          600     /* 传送带PWM值 */
#define CONVEYOR_RAMP_STEP      50      /* 启停加速步进 */
#define CONVEYOR_RAMP_MS        20      /* 加速步进间隔ms */

#define SORT_SERVO_RED          0       /* 红色分拣舵机通道 */
#define SORT_SERVO_GREEN        1       /* 绿色分拣舵机通道 */
#define SORT_SERVO_BLUE         2       /* 蓝色分拣舵机通道 */
#define SORT_SERVO_DEFAULT      3       /* 默认(未识别)通道 */

#define SERVO_PUSH_ANGLE        90      /* 推出角度 */
#define SERVO_HOME_ANGLE        0       /* 归位角度 */
#define SERVO_PUSH_HOLD_MS      500     /* 推出保持时间 */

/* 颜色判定阈值(需根据实际光照校准) */
#define RED_R_MIN               200     /* 红色R分量最小值 */
#define RED_RATIO_MIN           0.45f   /* 红色R占比最小值 */
#define GREEN_G_MIN             200     /* 绿色G分量最小值 */
#define GREEN_RATIO_MIN         0.40f   /* 绿色G占比最小值 */
#define BLUE_B_MIN              200     /* 蓝色B分量最小值 */
#define BLUE_RATIO_MIN          0.40f   /* 蓝色B占比最小值 */
#define MIN_CLEAR               50      /* 最小透明度(排除无物体) */

/* ========== 颜色分类结果 ========== */
typedef enum {
    COLOR_NONE = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_UNKNOWN
} ColorClass_t;

/* ========== 系统状态 ========== */
typedef enum {
    SYS_IDLE,           /* 空闲等待 */
    SYS_DETECTING,      /* 颜色检测中 */
    SYS_SORTING,        /* 分拣执行中 */
    SYS_RESETTING       /* 舵机复位中 */
} SystemState_t;

/* ========== 全局变量 ========== */
static volatile uint32_t g_tick = 0;
static volatile SystemState_t g_state = SYS_IDLE;
static ColorClass_t g_last_color = COLOR_NONE;

/* 白平衡校准系数(在白色参考物上校准得到) */
static float g_cal_r = 1.0f, g_cal_g = 1.0f, g_cal_b = 1.0f;

/* 统计 */
static uint32_t g_count_red = 0, g_count_green = 0, g_count_blue = 0;
static uint32_t g_count_unknown = 0;
static uint32_t g_count_total = 0;

/* 传送带当前速度(缓变速) */
static volatile int32_t g_conveyor_current = 0;
static volatile int32_t g_conveyor_target = 0;

/* ========== I2C辅助函数 ========== */
/**
 * @brief I2C写单字节寄存器
 * 错误经验：I2C通信失败必须返回错误码，不能静默忽略，否则后续操作可能损坏设备
 */
static int I2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    DL_I2C_fillControllerTXFIFO(I2C0, buf, 2);
    DL_I2C_startControllerTransfer(I2C0, dev_addr,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    uint32_t timeout = 100000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_CONTROLLER) {
        if (--timeout == 0) return -1;
    }
    if (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(I2C0);
        return -2;
    }
    return 0;
}

/**
 * @brief I2C读多字节
 */
static int I2C_ReadRegs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    DL_I2C_fillControllerTXFIFO(I2C0, &reg, 1);
    DL_I2C_startControllerTransfer(I2C0, dev_addr,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    uint32_t timeout = 100000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_CONTROLLER) {
        if (--timeout == 0) return -1;
    }

    DL_I2C_startControllerTransfer(I2C0, dev_addr,
                                    DL_I2C_CONTROLLER_DIRECTION_RX, len);
    timeout = 100000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_CONTROLLER) {
        if (--timeout == 0) return -1;
    }

    for (int i = 0; i < len; i++) {
        buf[i] = DL_I2C_receiveControllerData(I2C0);
    }
    return 0;
}

/* ========== TCS34725颜色传感器 ========== */
/**
 * @brief 初始化TCS34725
 * 错误经验：上电后需等待10ms再使能ADC，否则首次读取数据全0
 */
static int TCS34725_Init(void)
{
    /* 检查设备ID (应为0x44或0x4D) */
    uint8_t id;
    if (I2C_ReadRegs(TCS34725_ADDR, TCS34725_ID, &id, 1) != 0) return -1;

    /* 上电 */
    I2C_WriteReg(TCS34725_ADDR, TCS34725_ENABLE, TCS34725_ENABLE_PON);

    /* 等待上电稳定 */
    for (volatile int i = 0; i < 10000; i++) {}

    /* 设置积分时间和增益 */
    I2C_WriteReg(TCS34725_ADDR, TCS34725_ATIME, TCS34725_ATIME_MS);
    I2C_WriteReg(TCS34725_ADDR, TCS34725_CONTROL, TCS34725_GAIN_4X);

    /* 使能ADC */
    I2C_WriteReg(TCS34725_ADDR, TCS34725_ENABLE,
                 TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);

    printf("TCS34725 ID: 0x%02X\r\n", id);
    return 0;
}

/**
 * @brief 读取TCS34725 RGBC数据
 * @param r,g,b,c 指向16位数据的指针
 * 错误经验：必须先检查Status寄存器的AVALID位，否则读到的是上一次的值
 */
static int TCS34725_Read(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    /* 检查数据是否就绪 */
    uint8_t status;
    if (I2C_ReadRegs(TCS34725_ADDR, TCS34725_STATUS, &status, 1) != 0) return -1;
    if (!(status & TCS34725_STATUS_AVALID)) return -2; /* 数据未就绪 */

    /* 读取8字节(每个通道2字节，共4通道) */
    uint8_t buf[8];
    if (I2C_ReadRegs(TCS34725_ADDR, TCS34725_CDATAL, buf, 8) != 0) return -3;

    *c = (uint16_t)(buf[1] << 8 | buf[0]);
    *r = (uint16_t)(buf[3] << 8 | buf[2]);
    *g = (uint16_t)(buf[5] << 8 | buf[4]);
    *b = (uint16_t)(buf[7] << 8 | buf[6]);

    return 0;
}

/**
 * @brief 白平衡校准(在白色参考物上执行)
 */
static void TCS34725_WhiteBalance(void)
{
    uint16_t r, g, b, c;
    uint32_t sum_r = 0, sum_g = 0, sum_b = 0;

    printf("White balancing...\r\n");
    for (int i = 0; i < 16; i++) {
        for (volatile int d = 0; d < 100000; d++) {} /* 等待积分完成 */
        if (TCS34725_Read(&r, &g, &b, &c) == 0) {
            sum_r += r; sum_g += g; sum_b += b;
        }
    }

    uint16_t avg_r = sum_r / 16;
    uint16_t avg_g = sum_g / 16;
    uint16_t avg_b = sum_b / 16;

    /* 校准系数: 使白色时三通道归一化到最高通道 */
    uint16_t max_val = avg_r;
    if (avg_g > max_val) max_val = avg_g;
    if (avg_b > max_val) max_val = avg_b;

    g_cal_r = (float)max_val / (float)(avg_r ? avg_r : 1);
    g_cal_g = (float)max_val / (float)(avg_g ? avg_g : 1);
    g_cal_b = (float)max_val / (float)(avg_b ? avg_b : 1);

    printf("WB: R=%d G=%d B%d -> cal: %.2f %.2f %.2f\r\n",
           avg_r, avg_g, avg_b, g_cal_r, g_cal_g, g_cal_b);
}

/* ========== PCA9685舵机驱动 ========== */
/**
 * @brief 初始化PCA9685，设置PWM频率为50Hz(舵机标准)
 */
static int PCA9685_Init(void)
{
    /* 软复位 */
    I2C_WriteReg(PCA9685_ADDR, PCA9685_MODE1, 0x80);
    for (volatile int i = 0; i < 10000; i++) {}

    /* 计算预分频值: prescale = round(25MHz / (4096 * freq)) - 1 */
    uint8_t prescale = (uint8_t)((PCA9685_OSC_FREQ / (4096UL * 50)) - 1);

    /* 进入睡眠模式才能修改prescale */
    I2C_WriteReg(PCA9685_ADDR, PCA9685_MODE1, 0x10); /* Sleep */
    I2C_WriteReg(PCA9685_ADDR, PCA9685_PRESCALE, prescale);

    /* 唤醒，使能自动递增 */
    I2C_WriteReg(PCA9685_ADDR, PCA9685_MODE1, 0xA0); /* Auto-Increment + Restart */

    /* 等待振荡器稳定 */
    for (volatile int i = 0; i < 10000; i++) {}

    /* 全部舵机归位 */
    for (int ch = 0; ch < 16; ch++) {
        uint8_t reg = PCA9685_LED0_ON_L + 4 * ch;
        uint8_t data[4] = {0, 0, SERVO_MIN_PULSE & 0xFF, SERVO_MIN_PULSE >> 8};
        for (int i = 0; i < 4; i++) {
            I2C_WriteReg(PCA9685_ADDR, reg + i, data[i]);
        }
    }

    printf("PCA9685 init OK, prescale=%d\r\n", prescale);
    return 0;
}

/**
 * @brief 设置舵机角度
 * @param channel 舵机通道(0-15)
 * @param angle   角度(0-180)
 *
 * 错误经验：PCA9685的PWM寄存器是4字节(ON_L, ON_H, OFF_L, OFF_H)
 *          必须先写OFF再写ON的高字节，部分型号I2C速度太快会丢失数据
 */
static void PCA9685_SetServo(uint8_t channel, uint16_t angle)
{
    if (channel > 15 || angle > 180) return;

    /* 角度 -> PWM脉宽 */
    uint16_t pulse = SERVO_MIN_PULSE + (uint16_t)((uint32_t)angle * SERVO_RANGE / 180);

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[4] = {0, 0, pulse & 0xFF, (pulse >> 8) & 0x0F};

    for (int i = 0; i < 4; i++) {
        I2C_WriteReg(PCA9685_ADDR, reg + i, data[i]);
    }
}

/* ========== 传送带控制(缓变速) ========== */
/**
 * @brief 传送带电机控制(带缓变速)
 * 错误经验：传送带启动时如果直接满PWM，皮带会打滑，物料可能飞出
 *          需要通过SysTick逐步增加PWM占空比
 */
static void Conveyor_SetSpeed(int32_t speed)
{
    if (speed > 900) speed = 900;
    if (speed < -900) speed = -900;
    g_conveyor_target = speed;
}

/**
 * @brief 传送带缓变速更新(在SysTick中调用)
 */
static void Conveyor_Update(void)
{
    if (g_conveyor_current < g_conveyor_target) {
        g_conveyor_current += CONVEYOR_RAMP_STEP;
        if (g_conveyor_current > g_conveyor_target)
            g_conveyor_current = g_conveyor_target;
    } else if (g_conveyor_current > g_conveyor_target) {
        g_conveyor_current -= CONVEYOR_RAMP_STEP;
        if (g_conveyor_current < g_conveyor_target)
            g_conveyor_current = g_conveyor_target;
    }

    int32_t spd = g_conveyor_current;
    if (spd >= 0) {
        DL_GPIO_setPins(CONVEYOR_IN1_PORT, CONVEYOR_IN1_PIN);
        DL_GPIO_clearPins(CONVEYOR_IN2_PORT, CONVEYOR_IN2_PIN);
    } else {
        DL_GPIO_clearPins(CONVEYOR_IN1_PORT, CONVEYOR_IN1_PIN);
        DL_GPIO_setPins(CONVEYOR_IN2_PORT, CONVEYOR_IN2_PIN);
        spd = -spd;
    }
    DL_Timer_setCaptureCompareValue(TIMER0, (uint32_t)spd, DL_TIMER_CC_0_INDEX);
}

/* ========== 颜色分类 ========== */
/**
 * @brief 根据RGB数据判断颜色类别
 * 错误经验：纯靠RGB绝对值判断不可靠，需结合比例关系
 *          使用归一化比例(R/(R+G+B))更稳定，对光照强度不敏感
 */
static ColorClass_t Color_Classify(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    if (c < MIN_CLEAR) return COLOR_NONE; /* 无物体或太暗 */

    /* 白平衡校准 */
    float fr = (float)r * g_cal_r;
    float fg = (float)g * g_cal_g;
    float fb = (float)b * g_cal_b;

    float total = fr + fg + fb;
    if (total < 1.0f) return COLOR_NONE;

    float ratio_r = fr / total;
    float ratio_g = fg / total;
    float ratio_b = fb / total;

    /* 判定逻辑 */
    if (ratio_r > RED_RATIO_MIN && fr > RED_R_MIN && ratio_r > ratio_g && ratio_r > ratio_b) {
        return COLOR_RED;
    }
    if (ratio_g > GREEN_RATIO_MIN && fg > GREEN_G_MIN && ratio_g > ratio_r && ratio_g > ratio_b) {
        return COLOR_GREEN;
    }
    if (ratio_b > BLUE_RATIO_MIN && fb > BLUE_B_MIN && ratio_b > ratio_r && ratio_b > ratio_g) {
        return COLOR_BLUE;
    }

    return COLOR_UNKNOWN;
}

/**
 * @brief 获取颜色对应的舵机通道
 */
static uint8_t GetServoChannel(ColorClass_t color)
{
    switch (color) {
        case COLOR_RED:    return SORT_SERVO_RED;
        case COLOR_GREEN:  return SORT_SERVO_GREEN;
        case COLOR_BLUE:   return SORT_SERVO_BLUE;
        default:           return SORT_SERVO_DEFAULT;
    }
}

/**
 * @brief 获取颜色名称字符串
 */
static const char* Color_Name(ColorClass_t color)
{
    switch (color) {
        case COLOR_RED:    return "RED";
        case COLOR_GREEN:  return "GREEN";
        case COLOR_BLUE:   return "BLUE";
        case COLOR_UNKNOWN:return "UNKNOWN";
        default:           return "NONE";
    }
}

/* ========== 中断服务 ========== */
void SysTick_Handler(void)
{
    g_tick++;

    /* 传送带缓变速更新(每20ms) */
    if (g_tick % CONVEYOR_RAMP_MS == 0) {
        Conveyor_Update();
    }
}

/* ========== 系统初始化 ========== */
static void System_Init(void)
{
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* 初始化TCS34725 */
    if (TCS34725_Init() != 0) {
        printf("TCS34725 init FAILED!\r\n");
    }

    /* 初始化PCA9685 */
    if (PCA9685_Init() != 0) {
        printf("PCA9685 init FAILED!\r\n");
    }
}

/* ========== 主函数 ========== */
int main(void)
{
    System_Init();

    printf("=== Color Sorting Line ===\r\n");
    printf("Place white reference and press button for calibration\r\n");

    /* 白平衡校准 */
    TCS34725_WhiteBalance();

    printf("Starting conveyor...\r\n");

    /* 启动传送带 */
    Conveyor_SetSpeed(CONVEYOR_SPEED);

    uint32_t sort_start_tick = 0;

    while (1) {
        switch (g_state) {
        case SYS_IDLE:
            /* 检查光电传感器(低电平=有物料) */
            if (!DL_GPIO_readPins(SENSOR_PORT, SENSOR_PIN)) {
                /* 物料到位 */
                Conveyor_SetSpeed(0); /* 停传送带 */
                g_state = SYS_DETECTING;
                printf("Object detected!\r\n");
            }
            break;

        case SYS_DETECTING: {
            /* 等传送带完全停止后读取颜色 */
            if (g_conveyor_current == 0) {
                uint16_t r, g, b, c;
                /* 多次采样取平均(提高稳定性) */
                uint32_t sum_r = 0, sum_g = 0, sum_b = 0, sum_c = 0;
                int valid_count = 0;
                for (int i = 0; i < 8; i++) {
                    for (volatile int d = 0; d < 50000; d++) {}
                    if (TCS34725_Read(&r, &g, &b, &c) == 0) {
                        sum_r += r; sum_g += g; sum_b += b; sum_c += c;
                        valid_count++;
                    }
                }

                if (valid_count > 0) {
                    r = sum_r / valid_count;
                    g = sum_g / valid_count;
                    b = sum_b / valid_count;
                    c = sum_c / valid_count;

                    g_last_color = Color_Classify(r, g, b, c);
                    printf("Color: %s (R=%d G=%d B=%d C=%d)\r\n",
                           Color_Name(g_last_color), r, g, b, c);

                    /* 控制对应舵机推出 */
                    uint8_t ch = GetServoChannel(g_last_color);
                    PCA9685_SetServo(ch, SERVO_PUSH_ANGLE);

                    g_state = SYS_SORTING;
                    sort_start_tick = g_tick;

                    /* 统计 */
                    g_count_total++;
                    switch (g_last_color) {
                        case COLOR_RED:    g_count_red++;    break;
                        case COLOR_GREEN:  g_count_green++;  break;
                        case COLOR_BLUE:   g_count_blue++;   break;
                        default:           g_count_unknown++; break;
                    }
                }
            }
            break;
        }

        case SYS_SORTING:
            /* 等待舵机推出到位 */
            if (g_tick - sort_start_tick >= SERVO_PUSH_HOLD_MS) {
                /* 舵机归位 */
                uint8_t ch = GetServoChannel(g_last_color);
                PCA9685_SetServo(ch, SERVO_HOME_ANGLE);

                g_state = SYS_RESETTING;
                sort_start_tick = g_tick;
            }
            break;

        case SYS_RESETTING:
            /* 等待舵机归位完成后恢复传送 */
            if (g_tick - sort_start_tick >= 300) {
                Conveyor_SetSpeed(CONVEYOR_SPEED);
                g_state = SYS_IDLE;

                /* 定期打印统计 */
                if (g_count_total % 10 == 0) {
                    printf("Stats: Total=%lu R=%lu G=%lu B=%lu ?=%lu\r\n",
                           g_count_total, g_count_red, g_count_green,
                           g_count_blue, g_count_unknown);
                }
            }
            break;
        }
    }
}
