/**
 * @file    motor.c
 * @brief   电机驱动模块 - TB6612FNG双H桥驱动器
 * @author  电赛团队
 * @date    2024
 * @note    使用TIM4的CH1和CH2输出PWM，控制左右两个直流减速电机
 *
 * 接线说明：
 *   左电机(通道A): PB13(AIN1), PB14(AIN2), PB6(PWMA/TIM4_CH1)
 *   右电机(通道B): PB15(BIN1), PA8(BIN2),  PB7(PWMB/TIM4_CH2)
 *   STBY(使能):    PA9
 *
 * TB6612FNG控制逻辑：
 *   IN1=1, IN2=0  → 正转
 *   IN1=0, IN2=1  → 反转
 *   IN1=0, IN2=0  → 停止(滑行)
 *   IN1=1, IN2=1  → 制动(短路制动)
 */

#include "motor.h"

/* ========================================================================== */
/*                              私有变量                                       */
/* ========================================================================== */

static TIM_HandleTypeDef htim4;         /* TIM4句柄 */

/* ========================================================================== */
/*                              私有函数                                       */
/* ========================================================================== */

/**
 * @brief  配置电机控制GPIO引脚
 */
static void Motor_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIO时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置PA8(R_BIN2), PA9(STBY)为推挽输出 */
    GPIO_InitStruct.Pin = MOTOR_R_IN2_PIN | MOTOR_STBY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 配置PB13(AIN1), PB14(AIN2), PB15(BIN1)为推挽输出 */
    GPIO_InitStruct.Pin = MOTOR_L_IN1_PIN | MOTOR_L_IN2_PIN | MOTOR_R_IN1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 初始状态：所有引脚拉低，电机停止 */
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  配置TIM4输出PWM信号
 * @note   TIM4挂载在APB1总线，时钟72MHz
 *         PWM频率 = 72MHz / (预分频+1) / (周期+1)
 *                = 72MHz / 1 / 1000 = 72kHz (实际可调)
 *                设定为: 72MHz / 72 / 100 = 10kHz
 */
static void Motor_TIM4_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* 使能TIM4时钟 */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* TIM4基本配置 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 72 - 1;         /* 预分频: 72 → 1MHz计数频率 */
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = PWM_MAX;            /* 周期: 1000 → PWM频率=1kHz */
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim4);

    /* 通道1配置 (左电机PWMA) */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                    /* 初始占空比: 0 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

    /* 通道2配置 (右电机PWMB) */
    sConfigOC.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

/**
 * @brief  HAL PWM MSP初始化回调（配置PWM对应的GPIO为复用推挽输出）
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM4)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB6 -> TIM4_CH1 (左电机PWM)
         * PB7 -> TIM4_CH2 (右电机PWM) */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* ========================================================================== */
/*                              公有函数                                       */
/* ========================================================================== */

/**
 * @brief  电机模块初始化
 */
void Motor_Init(void)
{
    Motor_GPIO_Init();      /* 初始化方向控制GPIO */
    Motor_TIM4_PWM_Init();  /* 初始化PWM定时器 */
    Motor_Stop();           /* 确保电机初始停止 */
    Motor_Enable();         /* 使能TB6612FNG */
}

/**
 * @brief  设置左电机速度和方向
 * @param  speed: PWM值 (0 ~ PWM_MAX)
 * @param  forward: 1=正转(前进), 0=反转(后退)
 */
void Motor_SetLeft(uint16_t speed, uint8_t forward)
{
    /* 限制PWM范围 */
    if (speed > PWM_MAX) speed = PWM_MAX;

    /* 设置方向 */
    if (forward)
    {
        /* 正转: AIN1=1, AIN2=0 */
        HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    }
    else
    {
        /* 反转: AIN1=0, AIN2=1 */
        HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
    }

    /* 设置PWM占空比 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
}

/**
 * @brief  设置右电机速度和方向
 * @param  speed: PWM值 (0 ~ PWM_MAX)
 * @param  forward: 1=正转(前进), 0=反转(后退)
 */
void Motor_SetRight(uint16_t speed, uint8_t forward)
{
    /* 限制PWM范围 */
    if (speed > PWM_MAX) speed = PWM_MAX;

    /* 设置方向 */
    if (forward)
    {
        /* 正转: BIN1=1, BIN2=0 */
        HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);
    }
    else
    {
        /* 反转: BIN1=0, BIN2=1 */
        HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET);
    }

    /* 设置PWM占空比 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, speed);
}

/**
 * @brief  设置左右电机速度（带符号表示方向）
 * @param  left_speed:  左轮速度 (正值=前进, 负值=后退)
 * @param  right_speed: 右轮速度 (正值=前进, 负值=后退)
 * @note   题目要求禁止后退！负值自动限制为0
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    /* 禁止后退：将负值限制为0 */
    if (left_speed < 0)  left_speed = 0;
    if (right_speed < 0) right_speed = 0;

    /* 限制最大速度 */
    if (left_speed > SPEED_MAX)  left_speed = SPEED_MAX;
    if (right_speed > SPEED_MAX) right_speed = SPEED_MAX;

    /* 确保最小速度不为0时电机能转动 */
    if (left_speed > 0 && left_speed < SPEED_MIN)  left_speed = SPEED_MIN;
    if (right_speed > 0 && right_speed < SPEED_MIN) right_speed = SPEED_MIN;

    /* 设置左电机 */
    Motor_SetLeft((uint16_t)left_speed, 1);

    /* 设置右电机 */
    Motor_SetRight((uint16_t)right_speed, 1);
}

/**
 * @brief  停止所有电机（滑行停止）
 */
void Motor_Stop(void)
{
    /* IN1=0, IN2=0 → 滑行停止 */
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
}

/**
 * @brief  制动（短路制动，快速停车）
 */
void Motor_Brake(void)
{
    /* IN1=1, IN2=1 → 短路制动 */
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
}

/**
 * @brief  使能电机驱动（STBY引脚拉高）
 */
void Motor_Enable(void)
{
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}

/**
 * @brief  禁用电机驱动（STBY引脚拉低）
 */
void Motor_Disable(void)
{
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
}
