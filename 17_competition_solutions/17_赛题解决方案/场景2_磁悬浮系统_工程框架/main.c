/**
 * @file    main.c
 * @brief   场景2: 磁悬浮系统 (2024 F题) — STM32工程框架
 * @details ADC高度检测 + PWM电磁铁驱动 + PID闭环控制
 * @date    2026-06-11
 */

/* ========================================================================== */
/*                              头文件包含                                     */
/* ========================================================================== */
#include "platform/hal_stm32.h"
#include "drivers/oled.h"
#include "drivers/key.h"
#include "algorithm/pid.h"

/* ========================================================================== */
/*                           外设句柄 (CubeMX生成)                             */
/* ========================================================================== */
extern TIM_HandleTypeDef htim1;   /* 电磁铁PWM (高级定时器) */
extern ADC_HandleTypeDef hadc1;   /* 霍尔传感器/位移传感器 */
extern I2C_HandleTypeDef hi2c1;   /* OLED */
extern UART_HandleTypeDef huart1; /* 调试串口 */

/* ========================================================================== */
/*                              全局变量                                       */
/* ========================================================================== */

/* --- PID控制器 --- */
PID_t pid_position;       /* 位置环PID (高度控制) */
PID_t pid_velocity;       /* 速度环PID (可选, 串级控制) */

/* --- OLED --- */
OLED_t oled;

/* --- 按键 --- */
KeyMgr_t key_mgr;

/* --- 传感器数据 --- */
float current_height = 0.0f;     /* 当前高度 (mm) */
float target_height  = 15.0f;    /* 目标高度 (mm) */

/* --- 系统状态 --- */
typedef enum {
    STATE_IDLE = 0,         /* 待机 */
    STATE_RUNNING,          /* 运行中 */
    STATE_CALIBRATING,      /* 校准中 */
} SystemState_t;

volatile SystemState_t sys_state = STATE_IDLE;
volatile uint8_t emergency_stop = 0;  /* 急停标志 */

/* --- ADC校准参数 --- */
float adc_to_mm_factor = 0.01f;   /* ADC值到mm的转换系数 */
float adc_offset = 0.0f;           /* ADC零点偏移 */

/* ========================================================================== */
/*                           CubeMX配置清单                                   */
/* ========================================================================== */
/**
 * 1. TIM1: PWM Generation CH1 (高级定时器, 支持互补输出)
 *    - Channel1: PWM Generation
 *    - Prescaler: 0
 *    - Period: 7999 → 72MHz/(7999+1)=9kHz
 *    - Pulse: 0 (初始占空比0%)
 *    - 引脚: PA8(TIM1_CH1) → 电磁铁驱动MOSFET
 *    - 注意: 高级定时器需额外使能刹车和死区
 *
 * 2. ADC1: Single Channel
 *    - Channel: IN0
 *    - Sampling Time: 239.5 cycles
 *    - 引脚: PA0(ADC1_IN0) → 霍尔传感器/激光位移传感器
 *
 *    推荐传感器方案:
 *    - 方案A: 霍尔传感器 (SS49E线性霍尔) → 模拟输出, 线性度好
 *    - 方案B: 激光位移传感器 (GP2Y0A21) → 范围10~80cm
 *    - 方案C: 电涡流传感器 → 精度最高, 适合金属球
 *
 * 3. I2C1: Fast Mode 400kHz → OLED
 *    - PB6(SCL), PB7(SDA)
 *
 * 4. USART1: 115200 → 调试串口
 *    - PA9(TX), PA10(RX)
 *
 * 5. GPIO:
 *    - PB12: 急停按键 (外部中断, 下降沿)
 *    - PB13, PB14: 功能按键
 *    - PC13: 状态LED
 *
 * 6. NVIC:
 *    - TIM1_UP_IRQn: 优先级0 (最高, PWM中断用于PID)
 *    - ADC1_IRQn: 优先级1
 *    - EXTI15_10_IRQn: 优先级2 (急停按键)
 *
 * 7. DMA (可选):
 *    - ADC1 → 连续采样, 减少CPU开销
 */

/* ========================================================================== */
/*                           初始化函数                                       */
/* ========================================================================== */

/**
 * @brief ADC单次读取并转换为高度
 */
static float Read_Height(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t adc_val = HAL_ADC_GetValue(&hadc1);

    /* 线性转换: height = (adc_val - offset) * factor */
    float height = ((float)adc_val - adc_offset) * adc_to_mm_factor;

    /* 限幅保护 */
    if (height < 0.0f) height = 0.0f;
    if (height > 50.0f) height = 50.0f;  /* 最大50mm */

    return height;
}

/**
 * @brief 设置电磁铁PWM占空比
 * @param duty 占空比 0~1000 (对应0%~100%)
 */
static void Set_Magnet_PWM(int16_t duty)
{
    duty = CLAMP(duty, 0, 1000);

    /* TIM1的CCR1控制占空比 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty);
}

/**
 * @brief ADC校准 (在无铁球时调用)
 */
static void Calibrate_ADC(void)
{
    sys_state = STATE_CALIBRATING;
    OLED_Clear(&oled);
    OLED_ShowString(&oled, "Calibrating...", 0, 2);
    OLED_Refresh(&oled);

    /* 多次采样取平均 */
    uint32_t sum = 0;
    for (int i = 0; i < 100; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        sum += HAL_ADC_GetValue(&hadc1);
        DELAY_MS(5);
    }
    adc_offset = (float)sum / 100.0f;

    OLED_Clear(&oled);
    OLED_ShowString(&oled, "Cal Done!", 0, 2);
    OLED_ShowString(&oled, "Offset:", 0, 3);
    OLED_ShowFloat(&oled, adc_offset, 0, 50, 3);
    OLED_Refresh(&oled);
    DELAY_MS(1000);

    sys_state = STATE_IDLE;
}

/**
 * @brief 应用层初始化
 */
static void App_Init(void)
{
    /* ---- PID初始化 ---- */
    /* 位置环: 磁悬浮核心控制 */
    PID_Init(&pid_position, PID_MODE_POSITION,
             50.0f,    /* Kp - 磁悬浮需要较高比例增益 */
             80.0f,    /* Ki - 消除稳态误差(重力补偿) */
             5.0f,     /* Kd - 抑制振荡 */
             0.002f);  /* dt = 2ms (500Hz控制频率) */
    PID_SetOutputLimit(&pid_position, 0, 1000);  /* 输出范围: 0~100% PWM */
    PID_SetIntegralLimit(&pid_position, 800);
    PID_SetTarget(&pid_position, target_height);

    /* ---- OLED初始化 ---- */
    OLED_Init(&oled, &hi2c1, OLED_I2C_ADDR);

    /* ---- 按键初始化 ---- */
    KeyMgr_Init(&key_mgr);
    KeyMgr_Add(&key_mgr, GPIOB, GPIO_PIN_12, KEY_ACTIVE_LOW);  /* 急停 */
    KeyMgr_Add(&key_mgr, GPIOB, GPIO_PIN_13, KEY_ACTIVE_LOW);  /* 启动/停止 */
    KeyMgr_Add(&key_mgr, GPIOB, GPIO_PIN_14, KEY_ACTIVE_LOW);  /* 目标高度+ */

    /* ---- 启动PWM ---- */
    PWM_START(&htim1, TIM_CHANNEL_1);
    Set_Magnet_PWM(0);  /* 初始占空比0%, 安全起见 */

    /* ---- 开机画面 ---- */
    OLED_Clear(&oled);
    OLED_ShowString(&oled, "MagLev v1.0", 0, 0);
    OLED_ShowString(&oled, "NUEDC 2024 F", 0, 1);
    OLED_ShowString(&oled, "Press K2:Cal", 0, 3);
    OLED_Refresh(&oled);

    DBG_PRINTF("MagLev App_Init complete\r\n");
}

/* ========================================================================== */
/*                           控制任务                                          */
/* ========================================================================== */

/**
 * @brief 500Hz控制任务 (2ms周期)
 *        由TIM1更新中断触发, 或在主循环中轮询
 */
static void Task_Control_500Hz(void)
{
    if (sys_state != STATE_RUNNING) {
        Set_Magnet_PWM(0);
        return;
    }

    if (emergency_stop) {
        Set_Magnet_PWM(0);
        sys_state = STATE_IDLE;
        return;
    }

    /* 1. 读取当前高度 */
    current_height = Read_Height();

    /* 2. PID计算 */
    float pwm_out = PID_Calculate(&pid_position, current_height);

    /* 3. 输出限幅保护 */
    if (pwm_out < 0.0f) pwm_out = 0.0f;
    if (pwm_out > 1000.0f) pwm_out = 1000.0f;

    /* 4. 安全保护: 高度异常时切断 */
    if (current_height > 40.0f || current_height < 1.0f) {
        /* 高度异常, 可能铁球已掉落或卡住 */
        Set_Magnet_PWM(0);
        return;
    }

    /* 5. 输出PWM */
    Set_Magnet_PWM((int16_t)pwm_out);
}

/**
 * @brief 50ms显示更新任务
 */
static void Task_Display_50ms(void)
{
    OLED_Clear(&oled);

    /* 行0: 状态 */
    if (sys_state == STATE_RUNNING) {
        OLED_ShowString(&oled, "STATE: RUN", 0, 0);
    } else if (sys_state == STATE_IDLE) {
        OLED_ShowString(&oled, "STATE: IDLE", 0, 0);
    } else {
        OLED_ShowString(&oled, "STATE: CAL", 0, 0);
    }

    /* 行1: 当前高度 */
    OLED_ShowString(&oled, "H:", 0, 1);
    OLED_ShowFloat(&oled, current_height, 1, 20, 1);
    OLED_ShowString(&oled, "mm", 80, 1);

    /* 行2: 目标高度 */
    OLED_ShowString(&oled, "T:", 0, 2);
    OLED_ShowFloat(&oled, target_height, 1, 20, 2);
    OLED_ShowString(&oled, "mm", 80, 2);

    /* 行3: PWM输出 */
    OLED_ShowString(&oled, "PWM:", 0, 3);
    OLED_ShowFloat(&oled, pid_position.output, 0, 30, 3);
    OLED_ShowString(&oled, "/1000", 80, 3);

    OLED_Refresh(&oled);
}

/**
 * @brief 按键处理
 */
static void Key_Process(void)
{
    KeyEvent_t ev;

    /* K1(急停): 长按触发 */
    ev = Key_GetEvent(&key_mgr.keys[0]);
    if (ev == KEY_EVENT_LONG_PRESS) {
        emergency_stop = 1;
        Set_Magnet_PWM(0);
        sys_state = STATE_IDLE;
        DBG_PRINTF("EMERGENCY STOP!\r\n");
    }

    /* K2: 短按=启动/停止, 长按=校准 */
    ev = Key_GetEvent(&key_mgr.keys[1]);
    if (ev == KEY_EVENT_LONG_PRESS) {
        Calibrate_ADC();
    } else if (ev == KEY_EVENT_PRESS) {
        if (sys_state == STATE_IDLE) {
            PID_Reset(&pid_position);
            PID_SetTarget(&pid_position, target_height);
            sys_state = STATE_RUNNING;
            emergency_stop = 0;
        } else {
            Set_Magnet_PWM(0);
            sys_state = STATE_IDLE;
        }
    }

    /* K3: 目标高度+1mm */
    ev = Key_GetEvent(&key_mgr.keys[2]);
    if (ev == KEY_EVENT_PRESS) {
        target_height += 1.0f;
        if (target_height > 30.0f) target_height = 30.0f;
        PID_SetTarget(&pid_position, target_height);
    }
}

/* ========================================================================== */
/*                              主函数                                         */
/* ========================================================================== */

int main(void)
{
    /* 1. HAL/外设初始化 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    /* 2. 应用层初始化 */
    App_Init();

    /* 3. 主循环 */
    uint32_t last_ctrl   = HAL_GetTick();
    uint32_t last_disp   = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 500Hz控制 (2ms) */
        if (now - last_ctrl >= 2) {
            last_ctrl = now;
            Task_Control_500Hz();
        }

        /* 按键扫描 (10ms) */
        KeyMgr_Scan(&key_mgr);
        Key_Process();

        /* 显示更新 (50ms) */
        if (now - last_disp >= 50) {
            last_disp = now;
            Task_Display_50ms();
        }
    }
}
