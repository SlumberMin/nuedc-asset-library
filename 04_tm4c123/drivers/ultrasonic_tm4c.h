/**
 * @file    ultrasonic_tm4c.h
 * @brief   HC-SR04超声波测距驱动 头文件 (TM4C123)
 * @details 使用Timer捕获回波信号的高电平持续时间来计算距离
 *
 * 工作原理:
 *   1. 向Trig引脚发送≥10μs的高电平脉冲
 *   2. 超声波模块自动发出8个40kHz脉冲
 *   3. Echo引脚输出高电平, 持续时间=往返时间
 *   4. 距离(cm) = 高电平时间(μs) / 58
 *
 * 硬件接线:
 *   HC-SR04        TM4C123
 *   -------        --------
 *   Trig --------->  PB0  (GPIO输出)
 *   Echo --------->  PB1  (Timer捕获输入, T0CCP0)
 *   VCC  --------->  5V
 *   GND  --------->  GND
 *
 * @note    测量范围: 2cm~400cm, 精度约3mm
 */

#ifndef ULTRASONIC_TM4C_H
#define ULTRASONIC_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 配置结构体 ========== */
typedef struct {
    /* Trig引脚 (GPIO输出) */
    uint32_t trig_gpio_periph;   /* GPIO外设时钟 */
    uint32_t trig_gpio_base;     /* GPIO端口基地址 */
    uint32_t trig_pin;           /* GPIO_PIN_x */

    /* Echo捕获 (Timer捕获模式) */
    uint32_t echo_timer_periph;  /* Timer外设时钟 SYSCTL_PERIPH_TIMERx */
    uint32_t echo_timer_base;    /* Timer基地址 TIMERx_BASE */
    uint32_t echo_timer_cfg;     /* Timer配置 TIMER_CFG_x */
    uint32_t echo_timer_type;    /* 捕获类型 TIMER_A / TIMER_B */
    uint32_t echo_timer_cap_mode; /* 捕获模式 TIMER_EVENT_POS_EDGE */
    uint32_t echo_gpio_periph;   /* Echo GPIO外设时钟 */
    uint32_t echo_gpio_base;     /* Echo GPIO端口 */
    uint32_t echo_pin;           /* Echo GPIO引脚 */
    uint32_t echo_pin_config;    /* 引脚复用配置 GPIO_Pxx_TnCCPm */

    uint32_t sys_clock_hz;       /* 系统时钟频率 */
} Ultrasonic_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化超声波模块
 * @param  cfg  配置结构体指针
 */
void Ultrasonic_Init(const Ultrasonic_Config_t *cfg);

/**
 * @brief  触发一次测量并获取距离
 * @return 距离 (cm), 0表示超时/无效
 * @note   此函数内部阻塞约30ms (等待回波)
 */
float Ultrasonic_GetDistance_cm(void);

/**
 * @brief  触发一次测量并获取距离 (非阻塞版, 仅触发)
 */
void Ultrasonic_Trigger(void);

/**
 * @brief  获取回波状态 (非阻塞版)
 * @return true=测量完成, false=等待中
 */
bool Ultrasonic_IsDone(void);

/**
 * @brief  获取上次测量结果 (非阻塞版)
 * @return 距离 (cm)
 */
float Ultrasonic_GetLastDistance_cm(void);

/**
 * @brief  Timer捕获中断服务例程 (需在启动文件中绑定到对应Timer中断)
 * @note   处理Echo信号的上升/下降沿, 自动计算距离
 */
void Ultrasonic_CaptureISR(void);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_TM4C_H */
