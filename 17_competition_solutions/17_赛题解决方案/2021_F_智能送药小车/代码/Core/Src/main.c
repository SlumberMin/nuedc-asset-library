/**
 * @file    main.c
 * @brief   2021年F题 智能送药小车 - 主程序
 * 
 * 功能：
 * 1. 识别病房号(OpenMV数字识别)
 * 2. 沿红线自动寻径
 * 3. 到达指定病房送药
 * 4. 自动返回药房
 * 5. 双车协同(发挥部分)
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

typedef struct { float Kp, Ki, Kd, integral, prev_error; } PID_t;
PID_t g_track_pid;

/* 药房和病房位置 */
typedef struct {
    uint8_t id;         // 病房号
    uint16_t x, y;      // 坐标
} Room_t;

volatile uint8_t g_target_room = 0;    // 目标病房号
volatile uint8_t g_state = 0;          // 0=药房, 1=运送中, 2=病房, 3=返回中
volatile uint8_t g_has_medicine = 0;   // 是否装载药品

float PID_Calc(PID_t *p, float t, float a) {
    float e = t - a; p->integral += e;
    float d = e - p->prev_error; p->prev_error = e;
    return p->Kp*e + p->Ki*p->integral + p->Kd*d;
}

int main(void) {
    HAL_Init(); SystemClock_Config();
    GPIO_Init(); TIM_Init(); UART_Init();
    OLED_Init(); Motor_Init(); Sensor_Init();
    g_track_pid = (PID_t){2.0f, 0.3f, 1.0f, 0, 0};
    
    OLED_Clear(); OLED_ShowString(0,0,"Smart Medicine Car");
    
    while(1) {
        /* OpenMV读取病房号 */
        uint8_t room = Vision_GetRoomNumber();
        
        /* 按键确认 */
        uint8_t key = Key_Scan();
        if(key == KEY1 && room > 0) {
            g_target_room = room;
            g_has_medicine = 1;
            g_state = 1;    // 开始运送
            Alert_Beep(200);
        }
        
        /* 状态机 */
        switch(g_state) {
            case 0: // 药房等待
                Motor_SetSpeed(0, 0);
                if(g_has_medicine == 0) {
                    /* 等待装载药品 */
                    if(Sensor_DetectLoad()) {
                        g_has_medicine = 1;
                        Alert_Beep(100);
                    }
                }
                break;
                
            case 1: // 运送到病房
            {
                /* 红线循迹 */
                int16_t pos = Sensor_GetRedLinePos();
                float pid_out = PID_Calc(&g_track_pid, 0, pos);
                Motor_SetSpeed(400 + pid_out, 400 - pid_out);
                
                /* 检测到达病房 */
                if(Vision_CheckRoom(g_target_room)) {
                    Motor_SetSpeed(0, 0);
                    g_state = 2;
                    LED_On(1);  // 红灯亮
                    Alert_Beep(300);
                }
                break;
            }
                
            case 2: // 病房等待卸载
                Motor_SetSpeed(0, 0);
                if(Sensor_DetectUnload()) {
                    g_has_medicine = 0;
                    LED_Off(1);
                    g_state = 3;    // 开始返回
                    Alert_Beep(100);
                }
                break;
                
            case 3: // 返回药房
            {
                int16_t pos = Sensor_GetRedLinePos();
                float pid_out = PID_Calc(&g_track_pid, 0, pos);
                Motor_SetSpeed(400 + pid_out, 400 - pid_out);
                
                /* 检测到达药房 */
                if(Vision_CheckRoom(0)) {   // 药房
                    Motor_SetSpeed(0, 0);
                    g_state = 0;
                    LED_On(2);  // 绿灯亮
                    Alert_Beep(500);
                }
                break;
            }
        }
        
        /* 显示 */
        char buf[32];
        sprintf(buf, "Room:%d State:%d", g_target_room, g_state);
        OLED_ShowString(0, 2, buf);
        
        HAL_Delay(10);
    }
}
