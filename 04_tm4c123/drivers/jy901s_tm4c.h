/**
 * @file    jy901s_tm4c.h
 * @brief   JY901S IMU模块驱动 头文件 (TM4C123 UART中断接收)
 * @details 九轴姿态传感器，通过UART输出加速度/角速度/角度数据
 *
 * 硬件接线:
 *   JY901S         TM4C123
 *   ------         --------
 *   TX  ---------->  PC4 (U4RX)  或  PE4 (U5RX)
 *   RX  ---------->  PC5 (U4TX)  或  PE5 (U5TX)
 *   VCC ---------->  3.3V 或 5V
 *   GND ---------->  GND
 *
 * @note    JY901S默认波特率: 9600/115200 (可通过上位机配置)
 * @note    数据帧格式: 0x55 + TYPE + DATA(8) + SUM
 */

#ifndef JY901S_TM4C_H
#define JY901S_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== JY901S帧定义 ========== */
#define JY901S_FRAME_HEAD       0x55    /* 帧头 */
#define JY901S_FRAME_LEN        11      /* 每帧长度 */
#define JY901S_TYPE_ACCEL       0x51    /* 加速度 */
#define JY901S_TYPE_GYRO        0x52    /* 角速度 */
#define JY901S_TYPE_ANGLE       0x53    /* 角度 */
#define JY901S_TYPE_MAG         0x54    /* 磁场 */

/* ========== UART模块选择 ========== */
typedef enum {
    JY901S_UART4 = 4,   /* UART4: PC4(RX)/PC5(TX) */
    JY901S_UART5 = 5    /* UART5: PE4(RX)/PE5(TX) */
} JY901S_UART_t;

/* ========== IMU数据结构 ========== */
typedef struct {
    /* 加速度 (单位: g) */
    float ax, ay, az;
    /* 角速度 (单位: °/s) */
    float wx, wy, wz;
    /* 姿态角 (单位: °) */
    float roll, pitch, yaw;
    /* 磁场 (原始值) */
    int16_t mx, my, mz;
    /* 原始寄存器值 (用于调试) */
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t angle_raw[3];
    /* 更新标志 */
    volatile bool angle_updated;
} JY901S_Data_t;

/* ========== 配置结构体 ========== */
typedef struct {
    JY901S_UART_t   uart_module;    /* UART模块选择 */
    uint32_t        baudrate;       /* 波特率 (默认9600) */
    uint32_t        sys_clock_hz;   /* 系统时钟频率 */
} JY901S_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化JY901S (UART + 中断)
 * @param  cfg  配置结构体指针
 */
void JY901S_Init(const JY901S_Config_t *cfg);

/**
 * @brief  获取最新IMU数据
 * @param  data  输出数据结构体指针
 */
void JY901S_GetData(JY901S_Data_t *data);

/**
 * @brief  获取姿态角 (roll, pitch, yaw)
 * @param  roll   横滚角 (°)
 * @param  pitch  俯仰角 (°)
 * @param  yaw    偏航角 (°)
 */
void JY901S_GetAngle(float *roll, float *pitch, float *yaw);

/**
 * @brief  检查角度数据是否已更新
 * @return true=有新数据
 */
bool JY901S_IsUpdated(void);

/**
 * @brief  清除更新标志
 */
void JY901S_ClearUpdateFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* JY901S_TM4C_H */
