/**
 * @file    encoder_tm4c.h
 * @brief   增量式编码器驱动 头文件 (TM4C123 QEI硬件模式)
 * @details 使用TM4C123的QEI正交解码接口，自动完成A/B相解码和计数
 *
 * 硬件接线示意 (以QEI0为例):
 *   编码器          TM4C123
 *   ------          --------
 *   A相  ---------->  PD6 (PhA0)
 *   B相  ---------->  PD7 (PhB0)
 *   (Index可选) --->  PD5 (IDX0)
 *
 *   QEI1引脚:
 *   A相  ---------->  PC5 (PhA1)
 *   B相  ---------->  PC4 (PhB1)
 *   (Index可选) --->  PC6 (IDX1)
 *
 * @note    最大计数速率 = 系统时钟 / 2 (硬件4倍频后)
 */

#ifndef ENCODER_TM4C_H
#define ENCODER_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== QEI模块选择 ========== */
typedef enum {
    ENCODER_QEI0 = 0,   /* QEI0模块 (PD6/PD7) */
    ENCODER_QEI1 = 1    /* QEI1模块 (PC4/PC5) */
} Encoder_QEI_t;

/* ========== 编码器配置结构体 ========== */
typedef struct {
    Encoder_QEI_t   qei_module;     /* QEI模块选择 */
    uint32_t        max_position;   /* 最大位置计数 (0=不使用位置模式) */
    uint32_t        ppr;            /* 编码器线数 (每转脉冲数, 4倍频前) */
    uint32_t        sys_clock_hz;   /* 系统时钟频率 */
} Encoder_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化编码器 (QEI硬件模式)
 * @param  cfg  配置结构体指针
 */
void Encoder_Init(const Encoder_Config_t *cfg);

/**
 * @brief  获取当前位置计数
 * @param  module  QEI模块
 * @return 当前位置值 (原始计数)
 */
int32_t Encoder_GetPosition(Encoder_QEI_t module);

/**
 * @brief  获取速度 (RPM)
 * @param  module  QEI模块
 * @return 转速 (RPM), 负值表示反转
 * @note   依赖QEI速度捕获定时器, 需在Init中配置
 */
int32_t Encoder_GetSpeed(Encoder_QEI_t module);

/**
 * @brief  重置位置计数为0
 * @param  module  QEI模块
 */
void Encoder_ResetPosition(Encoder_QEI_t module);

/**
 * @brief  获取原始方向
 * @param  module  QEI模块
 * @return 1=正转, 0=反转
 */
uint32_t Encoder_GetDirection(Encoder_QEI_t module);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_TM4C_H */
