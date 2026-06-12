/**
 * @file    encoder_fixed.h
 * @brief   编码器模块头文件（修复版）
 * 
 * 修复：API支持分轮查询
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/* 编码器参数 */
#define ENCODER_PPR         13
#define ENCODER_GEAR_RATIO  34
#define WHEEL_DIAMETER_MM   65
#define WHEEL_TRACK_MM      150     // 轮距(mm)

/* 速度结构体 */
typedef struct {
    float speed;        // 速度(cm/s)
    float distance;     // 累计距离(cm)
    int32_t count;      // 脉冲计数
} EncoderData_t;

/* 函数声明 */
void Encoder_Init(void);
void Encoder_Update(void);

/* 修复：支持分轮查询 */
float Encoder_GetLeftSpeed(void);       // 左轮速度(cm/s)
float Encoder_GetRightSpeed(void);      // 右轮速度(cm/s)
float Encoder_GetAvgSpeed(void);        // 平均速度(cm/s)
float Encoder_GetLeftDistance(void);    // 左轮累计距离(cm)
float Encoder_GetRightDistance(void);   // 右轮累计距离(cm)
float Encoder_GetAvgDistance(void);     // 平均累计距离(cm)
void Encoder_ResetDistance(void);

/* 修复：兼容旧API */
float Encoder_GetSpeed(void);           // 返回平均速度
float Encoder_GetDistance(void);        // 返回平均距离

#endif
