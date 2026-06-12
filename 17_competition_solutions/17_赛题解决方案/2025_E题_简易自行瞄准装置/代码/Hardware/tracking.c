/**
 * @file    tracking.c
 * @brief   红外循迹传感器模块实现
 * 
 * 硬件连接：
 * 8路TCRT5000传感器 → LM393比较器 → MSPM0 GPIO输入
 * 传感器间距1.5cm，覆盖约12cm宽度
 * 黑线输出低电平(0)，白底输出高电平(1)
 */

#include "tracking.h"
#include "msp.h"

/* 传感器GPIO引脚定义 */
#define SENSOR_PORT     GPIOA
#define SENSOR_PINS     (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7)  // PA0-PA7

/* 传感器位置权重表 */
static const int16_t sensor_weights[TRACK_SENSOR_NUM] = {
    -35, -25, -15, -5, 5, 15, 25, 35
};

/* 跨线检测相关变量 */
static uint8_t last_sensor_state = 0x00;
static uint8_t cross_line_flag = 0;

/**
 * @brief  循迹传感器初始化
 * @param  无
 * @retval 无
 */
void Tracking_Init(void)
{
    /* 配置PA0-PA7为输入模式 */
    SENSOR_PORT->DIR &= ~SENSOR_PINS;
    /* 使能上拉电阻 */
    SENSOR_PORT->REN |= SENSOR_PINS;
    SENSOR_PORT->OUT |= SENSOR_PINS;
    
    last_sensor_state = 0x00;
    cross_line_flag = 0;
}

/**
 * @brief  读取循迹传感器原始数据
 * @param  无
 * @retval uint8_t: 8位传感器状态，每位对应一个传感器
 *         bit=0: 检测到黑线
 *         bit=1: 白底
 */
uint8_t Tracking_GetRawData(void)
{
    return (uint8_t)(SENSOR_PORT->IN & SENSOR_PINS);
}

/**
 * @brief  计算小车相对轨迹中心的位置偏差
 * @param  无
 * @retval int16_t: 位置偏差值(-100~+100)
 *         负值表示小车偏左，正值表示偏右，0表示居中
 * 
 * 算法：加权平均法
 * position = Σ(weight[i] * sensor[i]) / Σ(sensor[i])
 */
int16_t Tracking_GetPosition(void)
{
    uint8_t sensor_data = Tracking_GetRawData();
    int32_t weighted_sum = 0;
    int32_t weight_sum = 0;
    int16_t position;
    
    for(uint8_t i = 0; i < TRACK_SENSOR_NUM; i++)
    {
        /* 黑线时bit为0，取反后为1 */
        if(!(sensor_data & (1 << i)))
        {
            weighted_sum += sensor_weights[i];
            weight_sum += 1;
        }
    }
    
    if(weight_sum == 0)
    {
        /* 所有传感器都没检测到黑线，保持上次偏差方向 */
        return (last_sensor_state & 0x80) ? 100 : -100;
    }
    
    position = (int16_t)(weighted_sum / weight_sum);
    
    /* 保存状态 */
    last_sensor_state = (position >= 0) ? 0x80 : 0x00;
    
    return position;
}

/**
 * @brief  检测是否经过十字交叉线
 * @param  无
 * @retval uint8_t: 1=检测到跨线事件，0=未检测到
 * 
 * 判断逻辑：所有传感器同时检测到黑线，视为十字交叉线
 */
uint8_t Tracking_CheckCrossLine(void)
{
    uint8_t sensor_data = Tracking_GetRawData();
    
    /* 所有传感器都检测到黑线（全0） */
    if((sensor_data & SENSOR_PINS) == 0)
    {
        if(!cross_line_flag)
        {
            cross_line_flag = 1;
            return 1;   // 上升沿触发
        }
    }
    else
    {
        cross_line_flag = 0;
    }
    
    return 0;
}

/**
 * @brief  检测是否在轨迹线上
 * @param  无
 * @retval uint8_t: 1=在线上，0=脱线
 */
uint8_t Tracking_IsOnLine(void)
{
    uint8_t sensor_data = Tracking_GetRawData();
    /* 至少有1个传感器检测到黑线 */
    return ((sensor_data & SENSOR_PINS) != SENSOR_PINS) ? 1 : 0;
}
