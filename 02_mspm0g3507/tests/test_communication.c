/**
 * @file    test_communication.c
 * @brief   通信模块驱动测试 — MSPM0G3507
 * @note    测试HC-05蓝牙模块和OLED显示模块
 *
 * 硬件连接:
 *   HC-05蓝牙:
 *     MSPM0 PA9 → HC-05 TX (MCU的RX)
 *     MSPM0 PA8 → HC-05 RX (MCU的TX)
 *     MSPM0 PA7 → HC-05 STATE
 *   OLED显示:
 *     MSPM0 PA9 → OLED SCL
 *     MSPM0 PA8 → OLED SDA
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include <string.h>
#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/bluetooth_hc05_mspm0.h"
#include "drivers/oled_ssd1306_mspm0.h"

/* ── 主函数 ──────────────────────────────────────────────── */
int main(void)
{
    /* 系统初始化 */
    System_Init();
    
    /* HC-05蓝牙配置 */
    BT_HC05_Config bt_cfg = {
        .uart = UART_2_INST,
        .state_port = GPIOA,
        .state_pin = DL_GPIO_PIN_7,
        .baudrate = 9600
    };
    
    /* 初始化HC-05蓝牙 */
    BT_HC05_Init(&bt_cfg);
    
    /* 初始化OLED */
    OLED_Init(I2C_0_INST);
    
    /* 显示欢迎信息 */
    OLED_Clear();
    OLED_ShowString(0, 0, "MSPM0G3507", 16, 1);
    OLED_ShowString(0, 16, "BT+OLED Test", 16, 1);
    OLED_Refresh();
    
    printf("通信模块驱动测试\n");
    printf("HC-05蓝牙 + OLED显示\n");
    
    /* 主循环 */
    uint32_t count = 0;
    uint32_t bt_rx_count = 0;
    char display_buf[32];
    
    while (1) {
        /* 检查蓝牙连接状态 */
        uint8_t connected = BT_HC05_IsConnected();
        
        /* 检查是否收到蓝牙数据 */
        if (BT_HC05_IsDataReceived()) {
            uint8_t rx_buf[64];
            uint16_t len = BT_HC05_GetReceivedData(rx_buf, sizeof(rx_buf));
            
            if (len > 0) {
                /* 发送回蓝牙 */
                BT_HC05_SendString((char*)rx_buf);
                
                /* 在OLED上显示 */
                OLED_Clear();
                OLED_ShowString(0, 0, "BT Received:", 16, 1);
                OLED_ShowString(0, 16, (char*)rx_buf, 16, 1);
                OLED_Refresh();
                
                /* 清除接收缓冲区 */
                BT_HC05_ClearRxBuffer();
                bt_rx_count++;
            }
        }
        
        /* 每1000ms更新显示 */
        if (++count >= 1000) {
            count = 0;
            
            /* 更新OLED显示 */
            OLED_Clear();
            OLED_ShowString(0, 0, "MSPM0G3507", 16, 1);
            
            /* 显示连接状态 */
            if (connected) {
                OLED_ShowString(0, 16, "BT: Connected", 16, 1);
            } else {
                OLED_ShowString(0, 16, "BT: Disconnected", 16, 1);
            }
            
            /* 显示接收计数 */
            sprintf(display_buf, "RX Count: %lu", bt_rx_count);
            OLED_ShowString(0, 32, display_buf, 16, 1);
            
            OLED_Refresh();
            
            /* 发送心跳 */
            if (connected) {
                BT_HC05_SendString("Heartbeat\n");
            }
        }
        
        /* 延时1ms */
        DELAY_MS(1);
    }
    
    return 0;
}