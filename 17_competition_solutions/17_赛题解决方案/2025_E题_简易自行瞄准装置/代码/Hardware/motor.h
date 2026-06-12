/**
 * @file    motor.h
 * @brief   电机驱动模块头文件
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);
void Motor_Stop(void);

#endif /* __MOTOR_H */
