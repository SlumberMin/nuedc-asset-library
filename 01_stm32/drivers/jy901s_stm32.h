/**
 * @file    jy901s_stm32.h
 * @brief   JY901S IMU 驱动模块 — STM32 HAL库版本（UART中断接收）
 * @details 通过UART中断接收JY901S IMU数据包（9600-8-N-1）。
 *          支持角度(roll/pitch/yaw)、角速度、加速度数据解析。
 * @author  nuedc-asset-library
 * @version 1.0
 * @date    2026-06
 */

#ifndef __JY901S_STM32_H
#define __JY901S_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                              常量定义                                       */
/* ========================================================================== */

#define JY901S_FRAME_LEN     11     /**< JY901S 数据帧长度 */
#define JY901S_HEADER        0x55   /**< 帧头 */
#define JY901S_ID_ANGLE      0x53   /**< 角度数据ID */
#define JY901S_ID_GYRO       0x52   /**< 角速度数据ID */
#define JY901S_ID_ACCEL      0x51   /**< 加速度数据ID */

#define JY901S_BAUDRATE      9600   /**< 默认波特率 */

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/** @brief JY901S 数据结构体 */
typedef struct {
    /* 角度 (度) */
    float roll;                 /**< 横滚角 X (度) */
    float pitch;                /**< 俯仰角 Y (度) */
    float yaw;                  /**< 偏航角 Z (度) */

    /* 角速度 (度/秒) */
    float gyro_x;               /**< X轴角速度 (°/s) */
    float gyro_y;               /**< Y轴角速度 (°/s) */
    float gyro_z;               /**< Z轴角速度 (°/s) */

    /* 加速度 (g) */
    float accel_x;              /**< X轴加速度 (g) */
    float accel_y;              /**< Y轴加速度 (g) */
    float accel_z;              /**< Z轴加速度 (g) */

    /* 状态 */
    uint32_t update_tick;       /**< 最近一次更新时间戳 */
    uint32_t frame_count;       /**< 已接收的完整帧计数 */
    bool     new_data;          /**< 是否有新数据待处理 */
} JY901S_Data_t;

/** @brief JY901S 驱动实例 */
typedef struct {
    UART_HandleTypeDef *huart;          /**< UART句柄指针 */
    volatile uint8_t   rx_byte;         /**< [volatile] 单字节接收缓冲, ISR写主循环读 */
    volatile uint8_t   rx_buf[JY901S_FRAME_LEN]; /**< [volatile] 帧接收缓冲区 */
    volatile uint8_t   rx_index;        /**< [volatile] 当前接收字节索引, ISR中修改 */
    volatile uint8_t   rx_state;        /**< [volatile] 接收状态机, ISR中修改 */
    volatile JY901S_Data_t data;        /**< [volatile] 解析后的IMU数据, ISR写主循环读 */
    bool      initialized;
} JY901S_Dev_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化JY901S驱动
 * @param dev   JY901S设备结构体指针
 * @param huart UART句柄指针（需已初始化为9600-8-N-1）
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef JY901S_Init(JY901S_Dev_t *dev, UART_HandleTypeDef *huart);

/**
 * @brief 启动UART中断接收（调用一次即可）
 * @param dev   JY901S设备结构体指针
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef JY901S_StartReceive(JY901S_Dev_t *dev);

/**
 * @brief UART接收完成回调（在HAL_UART_RxCpltCallback中调用）
 * @param dev   JY901S设备结构体指针
 * @details 自动解析帧数据，更新data结构体
 */
void JY901S_RxCallback(JY901S_Dev_t *dev);

/**
 * @brief 获取IMU数据（线程安全，可通过new_data标志判断是否有新数据）
 * @param dev   JY901S设备结构体指针
 * @param data  输出数据结构体指针
 * @return true=有新数据, false=无新数据
 */
bool JY901S_GetData(JY901S_Dev_t *dev, JY901S_Data_t *data);

/**
 * @brief 获取偏航角（常用，快捷接口）
 * @param dev  JY901S设备结构体指针
 * @return 偏航角(度)
 */
float JY901S_GetYaw(JY901S_Dev_t *dev);

#endif /* __JY901S_STM32_H */
