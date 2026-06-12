/**
 * @file    gimbal.h
 * @brief   二维舵机云台模块头文件
 */
#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stdint.h>

void Gimbal_Init(void);
void Gimbal_SetAngle(float angle_h, float angle_v);
void Gimbal_Center(void);
void Laser_SetState(uint8_t state);

#endif /* __GIMBAL_H */
