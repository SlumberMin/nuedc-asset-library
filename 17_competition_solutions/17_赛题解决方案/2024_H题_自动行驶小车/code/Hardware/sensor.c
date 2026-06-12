/**
 * @file    sensor.c
 * @brief   红外循迹传感器模块 - 7路TCRT5000
 * @author  电赛团队
 * @date    2024
 * @note    传感器排列: S1(左) ... S4(中) ... S7(右)
 *          检测到黑线=1，白底=0
 *          权重: S1=-3, S2=-2, S3=-1, S4=0, S5=+1, S6=+2, S7=+3
 *
 * 偏差计算公式：
 *   error = Σ(sensor[i] × weight[i]) / Σ(sensor[i])
 *   结果为负=偏左（需右转修正），结果为正=偏右（需左转修正）
 */

#include "sensor.h"

/* ========================================================================== */
/*                              私有变量                                       */
/* ========================================================================== */

static SensorData_t sensor_data;                /* 传感器数据 */
static const int8_t weight[7] = {-3, -2, -1, 0, 1, 2, 3};  /* 各传感器权重 */

/* 传感器引脚端口数组（方便循环读取） */
static GPIO_TypeDef* sensor_port[7] = {
    SENSOR_S1_PORT, SENSOR_S2_PORT, SENSOR_S3_PORT, SENSOR_S4_PORT,
    SENSOR_S5_PORT, SENSOR_S6_PORT, SENSOR_S7_PORT
};

/* 传感器引脚号数组 */
static uint16_t sensor_pin[7] = {
    SENSOR_S1_PIN, SENSOR_S2_PIN, SENSOR_S3_PIN, SENSOR_S4_PIN,
    SENSOR_S5_PIN, SENSOR_S6_PIN, SENSOR_S7_PIN
};

/* ========================================================================== */
/*                              私有函数                                       */
/* ========================================================================== */

/**
 * @brief  读取单路传感器状态
 * @param  index: 传感器编号 (0~6)
 * @retval uint8_t 1=检测到黑线, 0=白底
 */
static uint8_t Sensor_ReadSingle(uint8_t index)
{
    if (index >= 7) return 0;

    /*
     * TCRT5000模块输出逻辑：
     *   检测到黑线 → 比较器输出低电平 → GPIO读到0
     *   白底(反射强) → 比较器输出高电平 → GPIO读到1
     *
     * 但有些模块逻辑相反，这里取反：
     *   读到0(黑线) → 返回1(表示检测到黑线)
     *   读到1(白底) → 返回0(表示白底)
     */
    uint8_t pin_state = HAL_GPIO_ReadPin(sensor_port[index], sensor_pin[index]);

    /* 注意：根据实际传感器模块调整逻辑 */
    return (pin_state == GPIO_PIN_RESET) ? 1 : 0;
}

/* ========================================================================== */
/*                              公有函数                                       */
/* ========================================================================== */

/**
 * @brief  传感器模块初始化
 * @note   配置7路传感器引脚为上拉输入
 */
void Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIO时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置PA2(S1), PA3(S2) */
    GPIO_InitStruct.Pin = SENSOR_S1_PIN | SENSOR_S2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 配置PB0(S3), PB1(S4), PB10(S5), PB11(S6), PB12(S7) */
    GPIO_InitStruct.Pin = SENSOR_S3_PIN | SENSOR_S4_PIN |
                          SENSOR_S5_PIN | SENSOR_S6_PIN | SENSOR_S7_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 初始化数据 */
    for (int i = 0; i < 7; i++) {
        sensor_data.s[i] = 0;
    }
    sensor_data.count = 0;
    sensor_data.error = 0;
    sensor_data.all_white = 1;
    sensor_data.all_black = 0;
}

/**
 * @brief  读取所有传感器状态并计算偏差
 * @retval SensorData_t* 指向传感器数据结构体
 */
SensorData_t* Sensor_Read(void)
{
    int32_t weighted_sum = 0;
    int32_t active_count = 0;

    /* 读取7路传感器 */
    for (int i = 0; i < 7; i++) {
        sensor_data.s[i] = Sensor_ReadSingle(i);
    }

    /* 统计检测到黑线的传感器数量 */
    sensor_data.count = 0;
    for (int i = 0; i < 7; i++) {
        if (sensor_data.s[i]) {
            sensor_data.count++;
        }
    }

    /* 计算加权偏差 */
    weighted_sum = 0;
    active_count = 0;
    for (int i = 0; i < 7; i++) {
        if (sensor_data.s[i]) {
            weighted_sum += weight[i] * 10;     /* 乘以10提高精度 */
            active_count++;
        }
    }

    /* 计算偏差值 */
    if (active_count > 0) {
        sensor_data.error = (int8_t)(weighted_sum / active_count);
    } else {
        /* 脱轨情况：保持上次偏差，方向修正 */
        /* error不变，使用上一次的值 */
    }

    /* 脱轨检测 */
    sensor_data.all_white = (sensor_data.count == 0) ? 1 : 0;
    sensor_data.all_black = (sensor_data.count == 7) ? 1 : 0;

    return &sensor_data;
}

/**
 * @brief  获取循迹偏差值
 * @retval int8_t 偏差值 (-30 ~ +30)
 *         负值=偏左（车在黑线左侧，需右转）
 *         正值=偏右（车在黑线右侧，需左转）
 *         0=居中
 */
int8_t Sensor_GetError(void)
{
    return sensor_data.error;
}

/**
 * @brief  判断是否脱轨
 * @retval uint8_t 1=脱轨（所有传感器都在白底上）, 0=正常
 */
uint8_t Sensor_IsOffTrack(void)
{
    return sensor_data.all_white;
}

/**
 * @brief  获取原始传感器位图
 * @retval uint8_t 位图（bit0=S1, bit1=S2, ..., bit6=S7）
 */
uint8_t Sensor_GetBitmap(void)
{
    uint8_t bitmap = 0;
    for (int i = 0; i < 7; i++) {
        if (sensor_data.s[i]) {
            bitmap |= (1 << i);
        }
    }
    return bitmap;
}
