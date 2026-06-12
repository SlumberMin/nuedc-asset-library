/**
 * @file    jy901s_mspm0.c
 * @brief   JY901S 九轴IMU驱动实现 — MSPM0G3507
 * @note    使用UART中断接收数据，解析JY901S协议
 *          基于JY901使用说明书V4.4
 */

#include "jy901s_mspm0.h"
#include <string.h>

/* ── JY901S协议定义 ──────────────────────────────────────── */
#define JY901S_HEADER           0x55    /* 帧头 */
#define JY901S_CMD_TIME         0x50    /* 时间 */
#define JY901S_CMD_ACCEL        0x51    /* 加速度 */
#define JY901S_CMD_GYRO         0x52    /* 角速度 */
#define JY901S_CMD_ANGLE        0x53    /* 角度 */
#define JY901S_CMD_MAG          0x54    /* 磁场 */
#define JY901S_CMD_PORT         0x55    /* 端口 */
#define JY901S_CMD_PRESS        0x56    /* 气压 */
#define JY901S_CMD_GPS          0x57    /* GPS */
#define JY901S_CMD_VELOCITY     0x58    /* 速度 */
#define JY901S_CMD_QUAT         0x59    /* 四元数 */
#define JY901S_CMD_GSA          0x5A    /* 卫星 */

#define JY901S_UNLOCK_CMD       0x88    /* 解锁命令 */
#define JY901S_SAVE_CMD         0x00    /* 保存命令 */
#define JY901S_RESET_CMD        0xFF    /* 重置命令 */

/* ── 私有变量 ────────────────────────────────────────────── */
static JY901S_Config g_jy901s_cfg;
static volatile JY901S_Data   g_jy901s_data;
static volatile uint8_t       g_rx_buf[128];      /* 接收缓冲区 */
static volatile uint8_t g_rx_index = 0;  /* 接收索引 */
static volatile uint8_t g_rx_state = 0;  /* 接收状态机 */
static volatile uint8_t g_data_ready = 0; /* 数据就绪标志 */

/* ── 内部函数 ────────────────────────────────────────────── */

/**
 * @brief 解析JY901S数据帧
 * @param buf  数据缓冲区 (11字节)
 */
static void JY901S_ParseFrame(const volatile uint8_t *buf)
{
    /* 检查帧头和校验和 */
    if (buf[0] != JY901S_HEADER) return;
    
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += buf[i];
    }
    if (sum != buf[10]) return;  /* 校验和错误 */
    
    /* 根据命令类型解析数据 */
    uint8_t cmd = buf[1];
    
    switch (cmd) {
    case JY901S_CMD_ACCEL:
        /* 加速度数据 */
        g_jy901s_data.acc_raw[0] = (int16_t)((buf[3] << 8) | buf[2]);
        g_jy901s_data.acc_raw[1] = (int16_t)((buf[5] << 8) | buf[4]);
        g_jy901s_data.acc_raw[2] = (int16_t)((buf[7] << 8) | buf[6]);
        g_jy901s_data.acc_x = (float)g_jy901s_data.acc_raw[0] / 32768.0f * 16.0f;  /* ±16g */
        g_jy901s_data.acc_y = (float)g_jy901s_data.acc_raw[1] / 32768.0f * 16.0f;
        g_jy901s_data.acc_z = (float)g_jy901s_data.acc_raw[2] / 32768.0f * 16.0f;
        break;
        
    case JY901S_CMD_GYRO:
        /* 角速度数据 */
        g_jy901s_data.gyro_raw[0] = (int16_t)((buf[3] << 8) | buf[2]);
        g_jy901s_data.gyro_raw[1] = (int16_t)((buf[5] << 8) | buf[4]);
        g_jy901s_data.gyro_raw[2] = (int16_t)((buf[7] << 8) | buf[6]);
        g_jy901s_data.gyro_x = (float)g_jy901s_data.gyro_raw[0] / 32768.0f * 2000.0f;  /* ±2000°/s */
        g_jy901s_data.gyro_y = (float)g_jy901s_data.gyro_raw[1] / 32768.0f * 2000.0f;
        g_jy901s_data.gyro_z = (float)g_jy901s_data.gyro_raw[2] / 32768.0f * 2000.0f;
        break;
        
    case JY901S_CMD_ANGLE:
        /* 角度数据 */
        g_jy901s_data.angle_raw[0] = (int16_t)((buf[3] << 8) | buf[2]);
        g_jy901s_data.angle_raw[1] = (int16_t)((buf[5] << 8) | buf[4]);
        g_jy901s_data.angle_raw[2] = (int16_t)((buf[7] << 8) | buf[6]);
        g_jy901s_data.roll = (float)g_jy901s_data.angle_raw[0] / 100.0f;
        g_jy901s_data.pitch = (float)g_jy901s_data.angle_raw[1] / 100.0f;
        g_jy901s_data.yaw = (float)g_jy901s_data.angle_raw[2] / 100.0f;
        break;
        
    case JY901S_CMD_MAG:
        /* 磁场数据 */
        g_jy901s_data.mag_raw[0] = (int16_t)((buf[3] << 8) | buf[2]);
        g_jy901s_data.mag_raw[1] = (int16_t)((buf[5] << 8) | buf[4]);
        g_jy901s_data.mag_raw[2] = (int16_t)((buf[7] << 8) | buf[6]);
        g_jy901s_data.mag_x = (float)g_jy901s_data.mag_raw[0];  /* 单位：uT */
        g_jy901s_data.mag_y = (float)g_jy901s_data.mag_raw[1];
        g_jy901s_data.mag_z = (float)g_jy901s_data.mag_raw[2];
        break;
        
    case JY901S_CMD_PORT:
        /* 温度数据 (端口数据帧包含温度) */
        g_jy901s_data.temperature = (float)((int16_t)((buf[3] << 8) | buf[2])) / 100.0f;
        break;
        
    default:
        break;
    }
    
    /* 更新状态 */
    g_jy901s_data.data_ready = 1;
    g_jy901s_data.update_count++;
}

/* ── UART中断处理 ────────────────────────────────────────── */
/* 注意：此函数需要在实际工程中根据具体UART配置进行调整 */
void UART_0_INST_IRQHandler(void)
{
    volatile uint32_t status = DL_UART_getRawInterruptStatus(g_jy901s_cfg.uart);
    
    if (status & DL_UART_INTERRUPT_RX) {
        /* 接收数据 */
        volatile uint8_t data = DL_UART_receiveData(g_jy901s_cfg.uart);
        
        /* 状态机解析 */
        switch (g_rx_state) {
        case 0:  /* 等待帧头 */
            if (data == JY901S_HEADER) {
                g_rx_buf[0] = data;
                g_rx_index = 1;
                g_rx_state = 1;
            }
            break;
            
        case 1:  /* 接收命令和数据 */
            g_rx_buf[g_rx_index++] = data;
            if (g_rx_index >= 11) {
                /* 接收完成，解析数据 */
                JY901S_ParseFrame(g_rx_buf);
                g_rx_state = 0;
                g_rx_index = 0;
            }
            break;
            
        default:
            g_rx_state = 0;
            g_rx_index = 0;
            break;
        }
        
        /* 清除中断标志 */
        DL_UART_clearInterruptStatus(g_jy901s_cfg.uart, DL_UART_INTERRUPT_RX);
    }
}

/* ── 公开API ─────────────────────────────────────────────── */

void JY901S_Init(const JY901S_Config *cfg)
{
    /* 保存配置 */
    g_jy901s_cfg = *cfg;
    
    /* 清空数据 */
    memset(&g_jy901s_data, 0, sizeof(JY901S_Data));
    g_rx_state = 0;
    g_rx_index = 0;
    g_data_ready = 0;
    
    /* 配置UART中断 */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    
    /* 启用UART接收中断 */
    DL_UART_enableInterrupt(g_jy901s_cfg.uart, DL_UART_INTERRUPT_RX);
}

JY901S_Data* JY901S_GetData(void)
{
    return &g_jy901s_data;
}

void JY901S_GetAngle(float *pitch, float *roll, float *yaw)
{
    if (pitch) *pitch = g_jy901s_data.pitch;
    if (roll)  *roll = g_jy901s_data.roll;
    if (yaw)   *yaw = g_jy901s_data.yaw;
}

void JY901S_GetAccel(float *acc_x, float *acc_y, float *acc_z)
{
    if (acc_x) *acc_x = g_jy901s_data.acc_x;
    if (acc_y) *acc_y = g_jy901s_data.acc_y;
    if (acc_z) *acc_z = g_jy901s_data.acc_z;
}

void JY901S_GetGyro(float *gyro_x, float *gyro_y, float *gyro_z)
{
    if (gyro_x) *gyro_x = g_jy901s_data.gyro_x;
    if (gyro_y) *gyro_y = g_jy901s_data.gyro_y;
    if (gyro_z) *gyro_z = g_jy901s_data.gyro_z;
}

uint8_t JY901S_IsDataReady(void)
{
    return g_jy901s_data.data_ready;
}

void JY901S_ClearDataReady(void)
{
    g_jy901s_data.data_ready = 0;
}

void JY901S_SendCommand(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    /* 发送帧头 */
    UART_TX_BYTE(g_jy901s_cfg.uart, 0xFF);
    UART_TX_BYTE(g_jy901s_cfg.uart, 0xAA);
    UART_TX_BYTE(g_jy901s_cfg.uart, cmd);
    
    /* 发送数据 */
    for (uint8_t i = 0; i < len; i++) {
        UART_TX_BYTE(g_jy901s_cfg.uart, data[i]);
    }
}

void JY901S_Unlock(void)
{
    uint8_t data[2] = {0x88, 0xB5};
    JY901S_SendCommand(0x00, data, 2);
}

void JY901S_Save(void)
{
    JY901S_Unlock();
    uint8_t data[2] = {0x00, 0x00};
    JY901S_SendCommand(0x00, data, 2);
}

void JY901S_Reset(void)
{
    JY901S_Unlock();
    uint8_t data[2] = {0xFF, 0xFF};
    JY901S_SendCommand(0x00, data, 2);
}

void JY901S_SetCalibrationMode(uint8_t mode)
{
    JY901S_Unlock();
    uint8_t data[2] = {mode, 0x00};
    JY901S_SendCommand(0x00, data, 2);
}