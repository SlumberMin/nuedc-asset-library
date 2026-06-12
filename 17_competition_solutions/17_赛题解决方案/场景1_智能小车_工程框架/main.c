/**
 * @file    main.c
 * @brief   场景1: 智能小车 (2024 H题) — STM32工程框架
 * @details 完整的main.c骨架, 可直接复制到工程中编译
 * @date    2026-06-11
 */

/* ========================================================================== */
/*                              头文件包含                                     */
/* ========================================================================== */
#include "platform/hal_stm32.h"

/* 驱动模块 */
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/sensor_ir.h"
#include "drivers/servo.h"
#include "drivers/oled.h"
#include "drivers/key.h"

/* 算法模块 */
#include "algorithm/pid.h"

/* ========================================================================== */
/*                           外设句柄 (CubeMX生成)                             */
/* ========================================================================== */
extern TIM_HandleTypeDef htim2;   /* 编码器 */
extern TIM_HandleTypeDef htim3;   /* 电机PWM */
extern TIM_HandleTypeDef htim4;   /* 舵机PWM(备用) */
extern ADC_HandleTypeDef hadc1;   /* 红外传感器 */
extern I2C_HandleTypeDef hi2c1;   /* OLED */
extern UART_HandleTypeDef huart1; /* 调试串口 */

/* ========================================================================== */
/*                              全局变量                                       */
/* ========================================================================== */

/* --- 电机 --- */
Motor_t motor_left;
Motor_t motor_right;

/* --- 编码器 --- */
Encoder_t encoder_left;
Encoder_t encoder_right;

/* --- 红外循迹 --- */
SensorIR_t ir_sensor;

/* --- PID控制器 --- */
PID_t pid_speed_left;    /* 左轮速度环 */
PID_t pid_speed_right;   /* 右轮速度环 */
PID_t pid_steer;         /* 转向环(循迹) */

/* --- OLED --- */
OLED_t oled;

/* --- 按键 --- */
KeyMgr_t key_mgr;

/* --- 系统状态 --- */
volatile uint32_t tick_10ms  = 0;   /* 10ms周期标志 */
volatile uint32_t tick_100ms = 0;   /* 100ms周期标志 */
volatile uint32_t tick_1s    = 0;   /* 1s周期标志 */

/* --- 目标参数 --- */
float target_speed = 300.0f;        /* 目标速度 cm/s */
float target_position = 0.0f;       /* 目标循迹位置(0=中心) */

/* ========================================================================== */
/*                           CubeMX配置清单                                   */
/* ========================================================================== */
/**
 * 在CubeMX中需要配置以下外设:
 *
 * 1. TIM2: Encoder Mode (TI1 + TI2)
 *    - Channel1: Encoder Mode
 *    - Channel2: Encoder Mode
 *    - Prescaler: 0
 *    - Counter Period: 65535
 *    - 引脚: PA0(TIM2_CH1), PA1(TIM2_CH2) → 左编码器
 *
 * 2. TIM3: PWM Generation CH1 + CH2
 *    - Channel1: PWM Generation
 *    - Channel2: PWM Generation
 *    - Prescaler: 0 (假设72MHz, 72M/(999+1)=72kHz→需调整)
 *    - 72MHz / (0+1) / (3599+1) = 20kHz
 *    - Prescaler: 0, Period: 3599
 *    - 引脚: PA6(TIM3_CH1) → 左电机PWM, PA7(TIM3_CH2) → 右电机PWM
 *
 * 3. ADC1: Scan Mode
 *    - 扫描5个通道: IN0~IN4
 *    - Continuous Conversion
 *    - Sampling Time: 239.5 cycles
 *    - 引脚: PA0~PA4 (注意与编码器引脚冲突, 需重新分配!)
 *
 *    **推荐引脚重分配方案:**
 *    - 编码器用TIM2: PA0/PA1 (左), PA2/PA3 (右)
 *    - ADC红外用: PB0(IN8), PB1(IN9), PC0(IN10), PC1(IN11), PC2(IN12)
 *
 * 4. I2C1: Fast Mode 400kHz
 *    - 引脚: PB6(SCL), PB7(SDA) → OLED
 *
 * 5. USART1: 115200 8N1
 *    - 引脚: PA9(TX), PA10(RX) → 调试串口
 *
 * 6. GPIO: Input Pull-up
 *    - PB12, PB13, PB14 → 按键K1, K2, K3
 *
 * 7. NVIC:
 *    - TIM2全局中断: 优先级1 (编码器更新)
 *    - TIM3全局中断: 优先级2 (PWM定时)
 *    - ADC1全局中断: 优先级3 (红外采样完成)
 *
 * 8. System Core:
 *    - SysTick: 1ms
 *    - DEBUG: Serial Wire
 */

/* ========================================================================== */
/*                           初始化函数                                       */
/* ========================================================================== */

/**
 * @brief 硬件初始化 (CubeMX生成的初始化在HAL_Init/ MX_xxx_Init中)
 *        本函数负责初始化应用层驱动模块
 */
static void App_Init(void)
{
    /* ---- 电机初始化 ---- */
    /* 左电机: TB6612, IN1=PA4, IN2=PA5, PWM=TIM3_CH1 */
    Motor_Init(&motor_left, MOTOR_DRV_TB6612,
               GPIOA, GPIO_PIN_4,    /* IN1 */
               GPIOA, GPIO_PIN_5,    /* IN2 */
               &htim3, TIM_CHANNEL_1 /* PWM */);

    /* 右电机: TB6612, IN1=PB0, IN2=PB1, PWM=TIM3_CH2 */
    Motor_Init(&motor_right, MOTOR_DRV_TB6612,
               GPIOB, GPIO_PIN_0,
               GPIOB, GPIO_PIN_1,
               &htim3, TIM_CHANNEL_2);

    /* ---- 编码器初始化 ---- */
    /* 左编码器: TIM2, 13线编码器, 6.5cm轮径, 30:1减速 */
    Encoder_Init(&encoder_left, &htim2, 13, 6.5f, 30.0f);

    /* 右编码器: 需要额外定时器, 此处省略(根据实际硬件配置) */
    // Encoder_Init(&encoder_right, &htim5, 13, 6.5f, 30.0f);

    /* ---- 红外循迹初始化 ---- */
    /* 5路红外传感器, ADC模式 */
    uint16_t adc_channels[5] = {8, 9, 10, 11, 12}; /* ADC通道号 */
    SensorIR_InitADC(&ir_sensor, &hadc1, adc_channels, 5, 2000);

    /* ---- PID初始化 ---- */
    /* 速度环: 左轮 */
    PID_Init(&pid_speed_left, PID_MODE_POSITION,
             2.0f,    /* Kp */
             0.5f,    /* Ki */
             0.1f,    /* Kd */
             0.01f);  /* dt = 10ms */
    PID_SetOutputLimit(&pid_speed_left, -1000, 1000);
    PID_SetIntegralLimit(&pid_speed_left, 500);
    PID_SetTarget(&pid_speed_left, target_speed);

    /* 速度环: 右轮 */
    PID_Init(&pid_speed_right, PID_MODE_POSITION,
             2.0f, 0.5f, 0.1f, 0.01f);
    PID_SetOutputLimit(&pid_speed_right, -1000, 1000);
    PID_SetIntegralLimit(&pid_speed_right, 500);
    PID_SetTarget(&pid_speed_right, target_speed);

    /* 转向环: 循迹PID */
    PID_Init(&pid_steer, PID_MODE_INCREMENTAL,
             3.0f,    /* Kp - 转向响应较快 */
             0.0f,    /* Ki - 不需要积分 */
             1.5f,    /* Kd - 抑制振荡 */
             0.01f);
    PID_SetOutputLimit(&pid_steer, -500, 500);
    PID_SetTarget(&pid_steer, 0.0f); /* 目标: 居中 */

    /* ---- OLED初始化 ---- */
    OLED_Init(&oled, &hi2c1, OLED_I2C_ADDR);

    /* ---- 按键初始化 ---- */
    KeyMgr_Init(&key_mgr);
    KeyMgr_Add(&key_mgr, GPIOB, GPIO_PIN_12, KEY_ACTIVE_LOW);
    KeyMgr_Add(&key_mgr, GPIOB, GPIO_PIN_13, KEY_ACTIVE_LOW);
    KeyMgr_Add(&key_mgr, GPIOB, GPIO_PIN_14, KEY_ACTIVE_LOW);

    /* ---- 启动PWM ---- */
    PWM_START(&htim3, TIM_CHANNEL_1);
    PWM_START(&htim3, TIM_CHANNEL_2);

    /* ---- 初始状态: 停车 ---- */
    Motor_Brake(&motor_left);
    Motor_Brake(&motor_right);

    /* ---- OLED开机画面 ---- */
    OLED_Clear(&oled);
    OLED_ShowString(&oled, "Smart Car v1.0", 0, 0);
    OLED_ShowString(&oled, "NUEDC 2024 H", 0, 1);
    OLED_ShowString(&oled, "Ready...", 0, 3);
    OLED_Refresh(&oled);

    DBG_PRINTF("App_Init complete\r\n");
}

/* ========================================================================== */
/*                           周期性任务函数                                    */
/* ========================================================================== */

/**
 * @brief 10ms任务: 编码器更新 + 速度环PID + 电机输出
 */
static void Task_10ms(void)
{
    /* 1. 更新编码器数据 */
    Encoder_Update(&encoder_left);
    // Encoder_Update(&encoder_right);

    /* 2. 获取实际速度 */
    float actual_speed_l = Encoder_GetSpeed(&encoder_left);
    // float actual_speed_r = Encoder_GetSpeed(&encoder_right);

    /* 3. 速度环PID计算 */
    float pwm_left  = PID_Calculate(&pid_speed_left, actual_speed_l);
    // float pwm_right = PID_Calculate(&pid_speed_right, actual_speed_r);

    /* 4. 输出到电机 */
    Motor_SetSpeed(&motor_left, (int16_t)pwm_left);
    // Motor_SetSpeed(&motor_right, (int16_t)pwm_right);
}

/**
 * @brief 20ms任务: 红外循迹更新 + 转向环PID
 */
static void Task_20ms(void)
{
    /* 1. 更新红外传感器数据 */
    SensorIR_Update(&ir_sensor);

    /* 2. 检查是否在线 */
    if (!SensorIR_IsOnLine(&ir_sensor)) {
        /* 丢线处理: 低速前进搜索 */
        Motor_SetSpeed(&motor_left,  100);
        Motor_SetSpeed(&motor_right, 100);
        return;
    }

    /* 3. 获取循迹位置 */
    float position = SensorIR_GetPosition(&ir_sensor);

    /* 4. 转向环PID计算 */
    float steer = PID_Calculate(&pid_steer, position);

    /* 5. 差速控制 */
    float speed_l = target_speed + steer;
    float speed_r = target_speed - steer;

    /* 6. 弯道减速 */
    float abs_pos = (position > 0) ? position : -position;
    if (abs_pos > 2.0f) {
        speed_l *= 0.6f;
        speed_r *= 0.6f;
    }

    /* 7. 限幅并输出 */
    speed_l = CLAMP(speed_l, -1000.0f, 1000.0f);
    speed_r = CLAMP(speed_r, -1000.0f, 1000.0f);
    Motor_SetSpeed(&motor_left,  (int16_t)speed_l);
    Motor_SetSpeed(&motor_right, (int16_t)speed_r);
}

/**
 * @brief 100ms任务: OLED显示更新
 */
static void Task_100ms(void)
{
    OLED_Clear(&oled);

    /* 行0: 速度信息 */
    OLED_ShowString(&oled, "L:", 0, 0);
    OLED_ShowFloat(&oled, Encoder_GetSpeed(&encoder_left), 0, 12, 0);
    OLED_ShowString(&oled, "cm/s", 56, 0);

    /* 行1: 循迹位置 */
    OLED_ShowString(&oled, "POS:", 0, 1);
    OLED_ShowFloat(&oled, SensorIR_GetPosition(&ir_sensor), 1, 30, 1);

    /* 行2: PID输出 */
    OLED_ShowString(&oled, "PID:", 0, 2);
    OLED_ShowFloat(&oled, pid_steer.output, 0, 30, 2);

    /* 行3: 目标速度 */
    OLED_ShowString(&oled, "TGT:", 0, 3);
    OLED_ShowFloat(&oled, target_speed, 0, 30, 3);

    OLED_Refresh(&oled);
}

/**
 * @brief 1000ms任务: 调试输出
 */
static void Task_1000ms(void)
{
    DBG_PRINTF("SPD_L=%.1f POS=%.2f PID_OUT=%.1f\r\n",
               Encoder_GetSpeed(&encoder_left),
               SensorIR_GetPosition(&ir_sensor),
               pid_steer.output);
}

/* ========================================================================== */
/*                           按键处理                                         */
/* ========================================================================== */

static void Key_Process(void)
{
    KeyEvent_t ev;

    /* K1: 启动/停止 */
    ev = Key_GetEvent(&key_mgr.keys[0]);
    if (ev == KEY_EVENT_PRESS) {
        static uint8_t running = 0;
        running = !running;
        if (running) {
            PID_Reset(&pid_speed_left);
            PID_Reset(&pid_steer);
            PID_SetTarget(&pid_speed_left, target_speed);
        } else {
            Motor_Brake(&motor_left);
            Motor_Brake(&motor_right);
        }
    }

    /* K2: 加速 */
    ev = Key_GetEvent(&key_mgr.keys[1]);
    if (ev == KEY_EVENT_PRESS) {
        target_speed += 50.0f;
        if (target_speed > 800.0f) target_speed = 800.0f;
        PID_SetTarget(&pid_speed_left, target_speed);
    }

    /* K3: 减速 */
    ev = Key_GetEvent(&key_mgr.keys[2]);
    if (ev == KEY_EVENT_PRESS) {
        target_speed -= 50.0f;
        if (target_speed < 0.0f) target_speed = 0.0f;
        PID_SetTarget(&pid_speed_left, target_speed);
    }
}

/* ========================================================================== */
/*                              主函数                                         */
/* ========================================================================== */

int main(void)
{
    /* 1. HAL初始化 (CubeMX生成) */
    HAL_Init();
    SystemClock_Config();  /* CubeMX生成 */
    MX_GPIO_Init();        /* CubeMX生成 */
    MX_TIM2_Init();        /* 编码器定时器 */
    MX_TIM3_Init();        /* PWM定时器 */
    MX_ADC1_Init();        /* ADC */
    MX_I2C1_Init();        /* OLED I2C */
    MX_USART1_UART_Init(); /* 调试串口 */

    /* 2. 应用层初始化 */
    App_Init();

    /* 3. 主循环 */
    uint32_t last_tick_10ms  = HAL_GetTick();
    uint32_t last_tick_20ms  = HAL_GetTick();
    uint32_t last_tick_100ms = HAL_GetTick();
    uint32_t last_tick_1s    = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 10ms周期: 速度环 */
        if (now - last_tick_10ms >= 10) {
            last_tick_10ms = now;
            Task_10ms();
        }

        /* 20ms周期: 循迹PID */
        if (now - last_tick_20ms >= 20) {
            last_tick_20ms = now;
            Task_20ms();
        }

        /* 50ms周期: 按键扫描 */
        KeyMgr_Scan(&key_mgr);
        Key_Process();

        /* 100ms周期: OLED显示 */
        if (now - last_tick_100ms >= 100) {
            last_tick_100ms = now;
            Task_100ms();
        }

        /* 1000ms周期: 调试输出 */
        if (now - last_tick_1s >= 1000) {
            last_tick_1s = now;
            Task_1000ms();
        }
    }
}

/* ========================================================================== */
/*                        CubeMX生成的回调函数                                 */
/* ========================================================================== */

/**
 * @brief TIM2中断回调 (编码器更新, 如需精确周期)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        /* 可在此处触发编码器更新 */
    }
}

/**
 * @brief ADC转换完成回调
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        /* ADC DMA完成, 可在此处理红外数据 */
    }
}
