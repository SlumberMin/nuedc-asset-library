/**
 * @file    test_jy901s.c
 * @brief   JY901S IMU驱动测试 — MSPM0G3507
 * @note    测试JY901S九轴姿态传感器的数据读取
 *
 * 硬件连接:
 *   MSPM0 PA9 → JY901S TX (MCU的RX)
 *   MSPM0 PA8 → JY901S RX (MCU的TX)
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/jy901s_mspm0.h"

/* ── 主函数 ──────────────────────────────────────────────── */
int main(void)
{
    /* 系统初始化 */
    System_Init();
    
    /* JY901S配置 */
    JY901S_Config jy901s_cfg = {
        .uart = UART_0_INST,  /* 根据实际接线修改 */
        .baudrate = 9600,
        .auto_calib = 1
    };
    
    /* 初始化JY901S */
    JY901S_Init(&jy901s_cfg);
    
    printf("JY901S IMU驱动测试\n");
    printf("请等待数据稳定...\n");
    
    /* 主循环 */
    uint32_t print_count = 0;
    while (1) {
        /* 检查数据是否就绪 */
        if (JY901S_IsDataReady()) {
            JY901S_ClearDataReady();
            
            /* 获取数据 */
            float pitch, roll, yaw;
            float acc_x, acc_y, acc_z;
            float gyro_x, gyro_y, gyro_z;
            
            JY901S_GetAngle(&pitch, &roll, &yaw);
            JY901S_GetAccel(&acc_x, &acc_y, &acc_z);
            JY901S_GetGyro(&gyro_x, &gyro_y, &gyro_z);
            
            /* 每10次数据打印一次 */
            if (++print_count >= 10) {
                print_count = 0;
                
                printf("角度: P=%.1f° R=%.1f° Y=%.1f° | ", pitch, roll, yaw);
                printf("加速度: X=%.2fg Y=%.2fg Z=%.2fg | ", acc_x, acc_y, acc_z);
                printf("角速度: X=%.1f°/s Y=%.1f°/s Z=%.1f°/s\n", 
                       gyro_x, gyro_y, gyro_z);
            }
        }
        
        /* 延时1ms */
        DELAY_MS(1);
    }
    
    return 0;
}