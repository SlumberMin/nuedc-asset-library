/**
 * @file    jy901s_mspm0.h
 * @brief   JY901S 九轴IMU驱动 — MSPM0G3507
 * @note    适用于JY901S九轴姿态传感器
 *          UART通信，芯片内置卡尔曼滤波，动态精度±0.1°
 *          输出角度(Pitch/Roll/Yaw)、加速度、角速度
 *
 * 接线示例:
 *   MSPM0 PA9 → JY901S TX (MCU的RX)
 *   MSPM0 PA8 → JY901S RX (MCU的TX)
 *   注意：JY901S TX接MCU的RX，JY901S RX接MCU的TX
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#ifndef __JY901S_MSPM0_H
#define __JY901S_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

/* ── JY901S数据结构 ──────────────────────────────────────── */
typedef struct {
    /* 角度 (单位：度) */
    float pitch;        /* 俯仰角 (-180° ~ +180°) */
    float roll;         /* 横滚角 (-180° ° ~ +180°) */
    float yaw;          /* 航向角 (-180° ~ +180°) */
    
    /* 加速度 (单位：g) */
    float acc_x;        /* X轴加速度 */
    float acc_y;        /* Y轴加速度 */
    float acc_z;        /* Z轴加速度 */
    
    /* 角速度 (单位：°/s) */
    float gyro_x;       /* X轴角速度 */
    float gyro_y;       /* Y轴角速度 */
    float gyro_z;       /* Z轴角速度 */
    
    /* 磁场 (单位：uT) */
    float mag_x;        /* X轴磁场 */
    float mag_y;        /* Y轴磁场 */
    float mag_z;        /* Z轴磁场 */
    
    /* 温度 (单位：℃) */
    float temperature;
    
    /* 原始数据 (用于调试) */
    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    int16_t angle_raw[3];
    
    /* 状态标志 */
    uint8_t data_ready;     /* 数据就绪标志 */
    uint32_t update_count;  /* 数据更新计数 */
} JY901S_Data;

/* ── JY901S配置结构 ──────────────────────────────────────── */
typedef struct {
    UART_Regs *uart;        /* UART实例 (如 UART0) */
    uint32_t   baudrate;    /* 波特率 (默认9600) */
    uint8_t    auto_calib;  /* 自动校准 (1=启用, 0=禁用) */
} JY901S_Config;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化JY901S
 * @param cfg  指向JY901S_Config结构
 * @note  自动配置UART，启用数据接收中断
 */
void JY901S_Init(const JY901S_Config *cfg);

/**
 * @brief 获取JY901S数据
 * @return 指向JY901S_Data结构的指针
 */
JY901S_Data* JY901S_GetData(void);

/**
 * @brief 获取角度数据
 * @param pitch  俯仰角指针
 * @param roll   横滚角指针
 * @param yaw    航向角指针
 */
void JY901S_GetAngle(float *pitch, float *roll, float *yaw);

/**
 * @brief 获取加速度数据
 * @param acc_x  X轴加速度指针
 * @param acc_y  Y轴加速度指针
 * @param acc_z  Z轴加速度指针
 */
void JY901S_GetAccel(float *acc_x, float *acc_y, float *acc_z);

/**
 * @brief 获取角速度数据
 * @param gyro_x  X轴角速度指针
 * @param gyro_y  Y轴角速度指针
 * @param gyro_z  Z轴角速度指针
 */
void JY901S_GetGyro(float *gyro_x, float *gyro_y, float *gyro_z);

/**
 * @brief 检查数据是否就绪
 * @return 1=数据就绪, 0=数据未就绪
 */
uint8_t JY901S_IsDataReady(void);

/**
 * @brief 清除数据就绪标志
 */
void JY901S_ClearDataReady(void);

/**
 * @brief 发送命令到JY901S
 * @param cmd  命令字节
 * @param data 数据指针
 * @param len  数据长度
 * @note  用于配置JY901S参数
 */
void JY901S_SendCommand(uint8_t cmd, const uint8_t *data, uint8_t len);

/**
 * @brief 解锁JY901S (用于保存配置)
 */
void JY901S_Unlock(void);

/**
 * @brief 保存JY901S配置
 */
void JY901S_Save(void);

/**
 * @brief 重置JY901S
 */
void JY901S_Reset(void);

/**
 * @brief 设置JY901S校准模式
 * @param mode  0=正常, 1=加速度校准, 2=磁场校准
 */
void JY901S_SetCalibrationMode(uint8_t mode);

#endif /* __JY901S_MSPM0_H */