/**
 * @file    jy901s.h
 * @brief   JY901S 九轴IMU UART驱动 — MSPM0G3507
 *
 * JY901S通信协议:
 *   - UART 9600 8N1
 *   - 数据帧: 帧头0x55 + 类型字节 + 8字节数据 + 校验和 = 11字节
 *   - 0x51: 加速度(ax, ay, az)
 *   - 0x52: 角速度(wx, wy, wz)
 *   - 0x53: 角度(roll, pitch, yaw), 单位0.01度
 *   - 0x54: 磁场(mx, my, mz)
 *
 * 硬件连接:
 *   MSPM0 PA17(UART1_TX) → JY901S RX
 *   MSPM0 PA18(UART1_RX) → JY901S TX
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成, 需配置UART_1)
 */

#ifndef __JY901S_H
#define __JY901S_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ── JY901S数据结构 ────────────────────────────────────────── */

/**
 * @brief JY901S IMU完整数据
 */
typedef struct {
    /* 加速度 (单位: g) */
    float acc_x;
    float acc_y;
    float acc_z;

    /* 角速度 (单位: 度/秒) */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    /* 角度 (单位: 度) */
    float roll;
    float pitch;
    float yaw;

    /* 磁场 (原始值) */
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;

    /* 数据更新标志 */
    bool acc_updated;
    bool gyro_updated;
    bool angle_updated;
} JY901S_Data;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化JY901S驱动
 *        配置UART1外设接收中断，开始接收JY901S数据
 */
void JY901S_Init(void);

/**
 * @brief 获取JY901S最新数据
 * @return 指向内部数据结构的指针（只读）
 */
const volatile JY901S_Data* JY901S_GetData(void);

/**
 * @brief 获取Roll角 (横滚角)
 * @return 角度值，单位: 度
 */
float JY901S_GetRoll(void);

/**
 * @brief 获取Pitch角 (俯仰角)
 * @return 角度值，单位: 度
 */
float JY901S_GetPitch(void);

/**
 * @brief 获取Yaw角 (偏航角)
 * @return 角度值，单位: 度
 */
float JY901S_GetYaw(void);

/**
 * @brief 检查角度数据是否已更新
 * @return true=新数据可用, false=无新数据（读取后自动清除）
 */
bool JY901S_IsAngleUpdated(void);

/**
 * @brief UART1中断处理函数
 *        在UART1_IRQHandler中调用，处理JY901S数据接收
 */
void JY901S_UART_IRQHandler(void);

#endif /* __JY901S_H */
