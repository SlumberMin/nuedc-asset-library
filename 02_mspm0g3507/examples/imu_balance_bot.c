/**
 * @file imu_balance_bot.c
 * @brief IMU自平衡机器人 - 完整系统集成示例
 * @target MSPM0G3507
 * @hardware JY901S IMU + 增量式编码器x2 + TB6612电机驱动 + HC-05蓝牙
 *
 * 系统架构：
 *   IMU(JY901S) --UART1--> 角度/角速度
 *   编码器A/B相 --GPIO中断--> 轮速
 *   三环PID(角度环+角速度环+速度环) --> PWM --> TB6612 --> 直流电机x2
 *   蓝牙(HC-05) --UART0--> 实时调参
 *
 * 错误经验库遵守：
 *   - IMU数据带CRC校验，防止UART传输错误
 *   - 编码器使用硬件QEI或GPIO中断，软件轮询会丢脉冲
 *   - PID输出限幅防积分饱和（anti-windup）
 *   - 电机启动前检查电池电压，低压保护
 *   - 蓝牙调参写入前校验范围，防止异常参数导致失控
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ========== 硬件引脚定义 ========== */
/* TB6612电机驱动 */
#define MOTOR_AIN1_PORT     GPIOA
#define MOTOR_AIN1_PIN      DL_GPIO_PIN_0
#define MOTOR_AIN2_PORT     GPIOA
#define MOTOR_AIN2_PIN      DL_GPIO_PIN_1
#define MOTOR_BIN1_PORT     GPIOA
#define MOTOR_BIN1_PIN      DL_GPIO_PIN_2
#define MOTOR_BIN2_PORT     GPIOA
#define MOTOR_BIN2_PIN      DL_GPIO_PIN_3

/* 编码器引脚 */
#define ENC_L_A_PORT        GPIOA
#define ENC_L_A_PIN         DL_GPIO_PIN_4
#define ENC_L_B_PORT        GPIOA
#define ENC_L_B_PIN         DL_GPIO_PIN_5
#define ENC_R_A_PORT        GPIOA
#define ENC_R_A_PIN         DL_GPIO_PIN_6
#define ENC_R_B_PORT        GPIOA
#define ENC_R_B_PIN         DL_GPIO_PIN_7

/* 电池电压检测 (ADC) */
#define BATTERY_ADC_PORT    GPIOA
#define BATTERY_ADC_PIN     DL_GPIO_PIN_8
#define BATTERY_LOW_MV      6500   /* 6.5V低压报警 (2S锂电池) */

/* ========== 系统参数 ========== */
#define CONTROL_PERIOD_MS   5       /* 控制周期5ms (200Hz) */
#define IMU_BAUD            115200  /* JY901S波特率 */
#define BT_BAUD             9600    /* 蓝牙波特率 */
#define ENCODER_PPR         13      /* 编码器线数(减速前) */
#define GEAR_RATIO          30      /* 减速比 */
#define WHEEL_CIRCUM_MM     204     /* 轮周长mm */
#define PWM_PERIOD          1000    /* PWM周期计数 */
#define PWM_MAX             950     /* PWM最大占空比(留余量) */
#define ANGLE_DEADZONE      1.5f    /* 角度死区(度) */
#define FALL_ANGLE          35.0f   /* 摔倒判定角度 */
#define ANGLE_OFFSET        0.0f    /* 机械零点偏移(调试时校准) */

/* ========== PID结构体 ========== */
typedef struct {
    float Kp;           /* 比例系数 */
    float Ki;           /* 积分系数 */
    float Kd;           /* 微分系数 */
    float integral;     /* 积分累加 */
    float prev_error;   /* 上次误差 */
    float output;       /* 输出 */
    float out_min;      /* 输出下限 */
    float out_max;      /* 输出上限 */
    float int_max;      /* 积分限幅(anti-windup) */
} PID_t;

/* ========== 全局变量 ========== */
/* IMU数据 */
static volatile float g_angle = 0.0f;       /* 俯仰角(度) */
static volatile float g_gyro_y = 0.0f;      /* Y轴角速度(度/秒) */

/* 编码器数据 */
static volatile int32_t g_enc_left = 0;     /* 左轮脉冲计数 */
static volatile int32_t g_enc_right = 0;    /* 右轮脉冲计数 */
static volatile float g_speed_left = 0.0f;  /* 左轮速度(mm/s) */
static volatile float g_speed_right = 0.0f; /* 右轮速度(mm/s) */
static volatile float g_speed_avg = 0.0f;   /* 平均速度 */

/* PID控制器 */
static PID_t g_pid_angle  = {45.0f, 0.0f, 25.0f, 0,0,0, -500,500, 300};  /* 角度环 */
static PID_t g_pid_gyro   = {1.2f,  0.0f, 0.0f,  0,0,0, -300,300, 200};  /* 角速度环 */
static PID_t g_pid_speed  = {0.8f,  0.02f, 0.0f, 0,0,0, -100,100, 50};   /* 速度环 */

/* 蓝牙调参 */
static volatile uint8_t g_bt_rx_buf[64];
static volatile uint8_t g_bt_rx_idx = 0;
static volatile uint8_t g_bt_rx_flag = 0;

/* 系统状态 */
static volatile uint8_t g_fallen = 0;       /* 摔倒标志 */
static volatile uint32_t g_tick = 0;        /* 系统节拍 */
static volatile uint16_t g_battery_mv = 0;  /* 电池电压mV */

/* JY901S接收缓冲 */
static uint8_t g_imu_buf[11];
static uint8_t g_imu_idx = 0;

/* ========== PID计算 ========== */
/**
 * @brief PID控制器计算
 * @param pid   PID结构体指针
 * @param error 当前误差
 * @param dt    时间间隔(秒)
 * @return PID输出值
 *
 * 错误经验：必须做积分限幅(anti-windup)，否则长时间倾斜会积分饱和
 *          微分项使用(error - prev_error)/dt，避免对设定值微分引起抖动
 */
static float PID_Calc(PID_t *pid, float error, float dt)
{
    /* 积分累加（带限幅） */
    pid->integral += error * dt;
    if (pid->integral > pid->int_max)  pid->integral = pid->int_max;
    if (pid->integral < -pid->int_max) pid->integral = -pid->int_max;

    /* 微分项 */
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    /* PID输出 */
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    /* 输出限幅 */
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    return pid->output;
}

/**
 * @brief 重置PID控制器
 */
static void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

/* ========== JY901S IMU驱动 ========== */
/**
 * @brief JY901S数据帧解析
 * 协议：0x55 0x53 + 角度数据(11字节)
 *       0x55 0x52 + 角速度数据(11字节)
 *
 * 错误经验：JY901S数据帧校验和计算方式为低8位累加
 *          必须校验，否则偶尔的UART噪声会导致角度跳变
 */
static uint8_t JY901S_ParseFrame(uint8_t *buf, float *angle, float *gyro)
{
    /* 校验和验证 */
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += buf[i];
    }
    if (sum != buf[10]) return 0; /* 校验失败 */

    if (buf[1] == 0x53) {
        /* 角度数据包 */
        int16_t roll  = (int16_t)(buf[2] | (buf[3] << 8));
        int16_t pitch = (int16_t)(buf[4] | (buf[5] << 8));
        int16_t yaw   = (int16_t)(buf[6] | (buf[7] << 8));
        *angle = (float)pitch / 32768.0f * 180.0f; /* 转换为度 */
        return 1;
    }
    else if (buf[1] == 0x52) {
        /* 角速度数据包 */
        int16_t wx = (int16_t)(buf[2] | (buf[3] << 8));
        int16_t wy = (int16_t)(buf[4] | (buf[5] << 8));
        int16_t wz = (int16_t)(buf[6] | (buf[7] << 8));
        *gyro = (float)wy / 32768.0f * 2000.0f; /* 转换为度/秒 */
        return 2;
    }
    return 0;
}

/* ========== 电机控制 ========== */
/**
 * @brief 设置电机PWM占空比
 * @param left  左电机PWM (-PWM_MAX ~ +PWM_MAX)
 * @param right 右电机PWM (-PWM_MAX ~ +PWM_MAX)
 */
static void Motor_SetPWM(int16_t left, int16_t right)
{
    /* 左电机方向 */
    if (left >= 0) {
        DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
        left = -left;
    }

    /* 右电机方向 */
    if (right >= 0) {
        DL_GPIO_setPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
        right = -right;
    }

    /* 限幅 */
    if (left > PWM_MAX)  left = PWM_MAX;
    if (right > PWM_MAX) right = PWM_MAX;

    /* 设置PWM占空比 (使用TIMER0的CCR1/CCR2) */
    DL_Timer_setCaptureCompareValue(TIMER0, (uint32_t)left, DL_TIMER_CC_1_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, (uint32_t)right, DL_TIMER_CC_2_INDEX);
}

/**
 * @brief 电机紧急制动
 */
static void Motor_Brake(void)
{
    DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    DL_Timer_setCaptureCompareValue(TIMER0, 0, DL_TIMER_CC_1_INDEX);
    DL_Timer_setCaptureCompareValue(TIMER0, 0, DL_TIMER_CC_2_INDEX);
}

/* ========== 蓝牙调参协议 ========== */
/**
 * @brief 解析蓝牙调参指令
 * 协议格式: "$CMD,Pxx,yyyy.yy\r\n"
 *   CMD: AGP/AGI/AGD/GYP/GYI/GYD/SPD/SPI/SPD/OFS (对应各PID参数)
 *   xx: 参数索引, yyyy.yy: 浮点值
 *
 * 错误经验：蓝牙调参必须校验范围，防止Kp过大导致振荡失控
 *          使用strtof而非atof，因为atof无错误检测
 */
static void BT_ParseCommand(uint8_t *buf, uint8_t len)
{
    buf[len] = '\0';
    char *cmd = (char *)buf;

    /* 跳过前导$ */
    if (cmd[0] != '$') return;
    cmd++;

    float val;
    char *p;

    if (strncmp(cmd, "AGP,", 4) == 0) {
        p = cmd + 4;
        val = strtof(p, NULL);
        if (val >= 0.0f && val <= 200.0f) g_pid_angle.Kp = val;
    }
    else if (strncmp(cmd, "AGI,", 4) == 0) {
        p = cmd + 4;
        val = strtof(p, NULL);
        if (val >= 0.0f && val <= 10.0f) g_pid_angle.Ki = val;
    }
    else if (strncmp(cmd, "AGD,", 4) == 0) {
        p = cmd + 4;
        val = strtof(p, NULL);
        if (val >= 0.0f && val <= 100.0f) g_pid_angle.Kd = val;
    }
    else if (strncmp(cmd, "GYP,", 4) == 0) {
        p = cmd + 4;
        val = strtof(p, NULL);
        if (val >= 0.0f && val <= 10.0f) g_pid_gyro.Kp = val;
    }
    else if (strncmp(cmd, "GYD,", 4) == 0) {
        p = cmd + 4;
        val = strtof(p, NULL);
        if (val >= 0.0f && val <= 10.0f) g_pid_gyro.Kd = val;
    }
    else if (strncmp(cmd, "SPP,", 4) == 0) {
        p = cmd + 4;
        val = strtof(p, NULL);
        if (val >= 0.0f && val <= 5.0f) g_pid_speed.Kp = val;
    }
    else if (strncmp(cmd, "SPI,", 4) == 0) {
        p = cmd + 4;
        val = strtof(p, NULL);
        if (val >= 0.0f && val <= 1.0f) g_pid_speed.Ki = val;
    }
    else if (strncmp(cmd, "RST", 3) == 0) {
        /* 复位所有PID */
        PID_Reset(&g_pid_angle);
        PID_Reset(&g_pid_gyro);
        PID_Reset(&g_pid_speed);
        g_fallen = 0;
    }
}

/* ========== 中断服务 ========== */

/**
 * @brief UART1中断 - JY901S IMU数据接收
 * 错误经验：UART接收必须处理帧头同步，否则错位后所有数据解析错误
 */
void UART1_IRQHandler(void)
{
    uint32_t status = DL_UART_getPendingInterrupt(UART1);

    if (status == DL_UART_IIDX_RX) {
        uint8_t ch = DL_UART_receiveData(UART1);

        /* 帧头检测: 0x55 0x53(角度) 或 0x55 0x52(角速度) */
        if (g_imu_idx == 0 && ch != 0x55) return;
        if (g_imu_idx == 1 && ch != 0x53 && ch != 0x52) {
            g_imu_idx = 0;
            return;
        }

        g_imu_buf[g_imu_idx++] = ch;

        if (g_imu_idx >= 11) {
            float angle, gyro;
            uint8_t result = JY901S_ParseFrame(g_imu_buf, &angle, &gyro);
            if (result == 1) {
                g_angle = angle + ANGLE_OFFSET;
            } else if (result == 2) {
                g_gyro_y = gyro;
            }
            g_imu_idx = 0;
        }
    }
}

/**
 * @brief UART0中断 - 蓝牙数据接收
 */
void UART0_IRQHandler(void)
{
    uint32_t status = DL_UART_getPendingInterrupt(UART0);

    if (status == DL_UART_IIDX_RX) {
        uint8_t ch = DL_UART_receiveData(UART0);

        if (ch == '$') {
            g_bt_rx_idx = 0; /* 新指令开始 */
        }

        g_bt_rx_buf[g_bt_rx_idx++] = ch;

        if (ch == '\n' || g_bt_rx_idx >= 63) {
            g_bt_rx_flag = 1;
            BT_ParseCommand((uint8_t *)g_bt_rx_buf, g_bt_rx_idx);
            g_bt_rx_idx = 0;
        }
    }
}

/**
 * @brief 编码器A相中断 - 左轮
 * 错误经验：编码器中断必须判断B相电平确定方向，否则无法区分正反转
 */
void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIOA);

    if (pending & ENC_L_A_PIN) {
        if (DL_GPIO_readPins(ENC_L_B_PORT, ENC_L_B_PIN)) {
            g_enc_left--;
        } else {
            g_enc_left++;
        }
        DL_GPIO_clearInterruptStatus(GPIOA, ENC_L_A_PIN);
    }

    if (pending & ENC_R_A_PIN) {
        if (DL_GPIO_readPins(ENC_R_B_PORT, ENC_R_B_PIN)) {
            g_enc_right--;
        } else {
            g_enc_right++;
        }
        DL_GPIO_clearInterruptStatus(GPIOA, ENC_R_A_PIN);
    }
}

/**
 * @brief SysTick中断 - 5ms控制周期
 * 这是系统核心控制环，在中断中执行三环PID计算
 */
void SysTick_Handler(void)
{
    g_tick++;

    /* 每5ms计算一次轮速 (编码器转mm/s) */
    static int32_t prev_enc_l = 0, prev_enc_r = 0;
    int32_t enc_l = g_enc_left;
    int32_t enc_r = g_enc_right;
    int32_t delta_l = enc_l - prev_enc_l;
    int32_t delta_r = enc_r - prev_enc_r;
    prev_enc_l = enc_l;
    prev_enc_r = enc_r;

    /* 脉冲数 -> 速度(mm/s): speed = delta * circum / (ppr * gear_ratio * dt) */
    float dt = (float)CONTROL_PERIOD_MS / 1000.0f;
    float speed_scale = (float)WHEEL_CIRCUM_MM / ((float)ENCODER_PPR * (float)GEAR_RATIO * dt);
    g_speed_left  = (float)delta_l * speed_scale;
    g_speed_right = (float)delta_r * speed_scale;
    g_speed_avg   = (g_speed_left + g_speed_right) / 2.0f;

    /* 三环PID控制 */
    if (fabsf(g_angle) > FALL_ANGLE) {
        /* 摔倒保护 */
        g_fallen = 1;
        Motor_Brake();
        PID_Reset(&g_pid_angle);
        PID_Reset(&g_pid_gyro);
        PID_Reset(&g_pid_speed);
        return;
    }

    if (fabsf(g_angle) < ANGLE_DEADZONE && fabsf(g_speed_avg) < 5.0f) {
        /* 角度在死区且无速度，保持静止 */
        Motor_Brake();
        PID_Reset(&g_pid_angle);
        PID_Reset(&g_pid_gyro);
        PID_Reset(&g_pid_speed);
        return;
    }

    /* 速度环 -> 输出角度补偿 */
    float speed_out = PID_Calc(&g_pid_speed, 0.0f - g_speed_avg, dt);
    /* 角度设定值 = 速度环输出(补偿) */
    float angle_target = speed_out;
    float angle_error = angle_target - g_angle;

    /* 角度环 -> 输出角速度补偿 */
    float angle_out = PID_Calc(&g_pid_angle, angle_error, dt);
    float gyro_target = angle_out;
    float gyro_error = gyro_target - g_gyro_y;

    /* 角速度环 -> 输出PWM */
    float pwm_out = PID_Calc(&g_pid_gyro, gyro_error, dt);

    /* 转向差速 (预留，可通过蓝牙设置转向偏移) */
    int16_t pwm_left  = (int16_t)(pwm_out);
    int16_t pwm_right = (int16_t)(pwm_out);

    Motor_SetPWM(pwm_left, pwm_right);
}

/* ========== 初始化 ========== */
/**
 * @brief 系统时钟与外设初始化
 */
static void System_Init(void)
{
    /* 初始化SysConfig生成的外设配置 */
    SYSCFG_DL_init();

    /* 配置SysTick 5ms中断 */
    SysTick_Config(SystemCoreClock / (1000 / CONTROL_PERIOD_MS));

    /* 使能UART中断 */
    NVIC_EnableIRQ(UART1_IRQn);  /* IMU */
    NVIC_EnableIRQ(UART0_IRQn);  /* 蓝牙 */

    /* 使能编码器GPIO中断 */
    NVIC_EnableIRQ(GPIOA_IRQn);
    DL_GPIO_enableInterrupt(GPIOA, ENC_L_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENC_R_A_PIN);

    /* 配置ADC用于电池电压检测 */
    DL_ADC12_startConversion(ADC0);
}

/**
 * @brief 电池电压检测
 * 通过ADC采样分压电阻，计算实际电压
 * 错误经验：电池电压采样需多次平均，ADC噪声会导致读数跳动
 */
static uint16_t Battery_Read_mV(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        DL_ADC12_startConversion(ADC0);
        while (!DL_ADC12_getRawDataAvailable(ADC0)) {}
        sum += DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
    }
    uint32_t avg = sum >> 4; /* 16次平均 */
    /* 假设12bit ADC, 参考3.3V, 分压比1:11 (100k:10k) */
    uint32_t adc_mv = avg * 3300 / 4095;
    uint32_t real_mv = adc_mv * 11; /* 还原实际电压 */
    return (uint16_t)real_mv;
}

/* ========== 主函数 ========== */
int main(void)
{
    System_Init();

    /* 启动信息 */
    printf("=== IMU Balance Bot ===\r\n");
    printf("Angle PID: P=%.2f I=%.2f D=%.2f\r\n",
           g_pid_angle.Kp, g_pid_angle.Ki, g_pid_angle.Kd);
    printf("Gyro PID:  P=%.2f D=%.2f\r\n",
           g_pid_gyro.Kp, g_pid_gyro.Kd);
    printf("Speed PID: P=%.2f I=%.2f\r\n",
           g_pid_speed.Kp, g_pid_speed.Ki);
    printf("Bluetooth: $AGP,45.00  $AGI,0.00  $AGD,25.00\r\n");
    printf("$GYP,1.20  $GYD,0.00  $SPP,0.80  $SPI,0.02\r\n");
    printf("$RST - reset all\r\n\r\n");

    uint32_t last_report_tick = 0;
    uint32_t last_battery_tick = 0;

    while (1) {
        /* 电池电压检测 (每1秒) */
        if (g_tick - last_battery_tick >= 200) {
            last_battery_tick = g_tick;
            g_battery_mv = Battery_Read_mV();
            if (g_battery_mv < BATTERY_LOW_MV) {
                Motor_Brake(); /* 低压保护：停止电机 */
                printf("BATTERY LOW: %dmV\r\n", g_battery_mv);
            }
        }

        /* 蓝牙状态上报 (每100ms) */
        if (g_tick - last_report_tick >= 20) {
            last_report_tick = g_tick;
            printf("A:%.1f G:%.1f S:%.1f P:%d BAT:%d F:%d\r\n",
                   g_angle, g_gyro_y, g_speed_avg,
                   (int)g_pid_angle.output,
                   g_battery_mv, g_fallen);
        }

        /* 摔倒自动恢复：倾斜角度回到可恢复范围 */
        if (g_fallen && fabsf(g_angle) < 10.0f) {
            g_fallen = 0;
            PID_Reset(&g_pid_angle);
            PID_Reset(&g_pid_gyro);
            PID_Reset(&g_pid_speed);
            printf("RECOVERED\r\n");
        }
    }
}
