/**
 * @file    bluetooth_hc05_mspm0.c
 * @brief   HC-05 蓝牙模块驱动实现 — MSPM0G3507
 * @note    使用UART中断接收数据，支持连接状态检测
 *          基于天猛星MSPM0G3507模块移植代码优化
 */

#include "bluetooth_hc05_mspm0.h"
#include <string.h>

/* ── 私有变量 ────────────────────────────────────────────── */
static BT_HC05_Config g_bt_cfg;
static BT_HC05_Data   g_bt_data;

/* ── 内部函数 ────────────────────────────────────────────── */

/**
 * @brief 更新连接状态
 */
static void BT_HC05_UpdateConnectState(void)
{
    /* 读取STATE引脚 */
    uint8_t state = GPIO_READ(g_bt_cfg.state_port, g_bt_cfg.state_pin) ? 1 : 0;
    
    /* 更新连接状态 */
    g_bt_data.connect_state = state ? BT_CONNECTED : BT_DISCONNECTED;
}

/* ── UART中断处理 ────────────────────────────────────────── */
void UART_2_INST_IRQHandler(void)
{
    volatile uint32_t status = DL_UART_getRawInterruptStatus(g_bt_cfg.uart);
    
    if (status & DL_UART_INTERRUPT_RX) {
        /* 接收数据 */
        volatile uint8_t data = DL_UART_receiveData(g_bt_cfg.uart);
        
        /* 检查缓冲区空间 */
        if (g_bt_data.rx_len < BT_RX_BUF_SIZE - 1) {
            g_bt_data.rx_buf[g_bt_data.rx_len++] = data;
            g_bt_data.rx_buf[g_bt_data.rx_len] = '\0';
            g_bt_data.rx_flag = 1;
            g_bt_data.rx_count++;
        }
        
        /* 清除中断标志 */
        DL_UART_clearInterruptStatus(g_bt_cfg.uart, DL_UART_INTERRUPT_RX);
    }
}

/* ── 公开API ─────────────────────────────────────────────── */

void BT_HC05_Init(const BT_HC05_Config *cfg)
{
    /* 保存配置 */
    g_bt_cfg = *cfg;
    
    /* 清空数据 */
    memset(&g_bt_data, 0, sizeof(BT_HC05_Data));
    
    /* 配置UART中断 */
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
    
    /* 启用UART接收中断 */
    DL_UART_enableInterrupt(g_bt_cfg.uart, DL_UART_INTERRUPT_RX);
}

void BT_HC05_SendByte(uint8_t ch)
{
    /* 等待UART空闲 */
    while (DL_UART_isBusy(g_bt_cfg.uart)) {}
    
    /* 发送字节 */
    DL_UART_transmitData(g_bt_cfg.uart, ch);
    g_bt_data.tx_count++;
}

void BT_HC05_SendString(const char *str)
{
    while (str && *str) {
        BT_HC05_SendByte(*str++);
    }
}

void BT_HC05_SendData(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        BT_HC05_SendByte(data[i]);
    }
}

uint8_t BT_HC05_IsDataReceived(void)
{
    return g_bt_data.rx_flag;
}

uint16_t BT_HC05_GetReceivedData(uint8_t *buf, uint16_t max_len)
{
    if (!g_bt_data.rx_flag) return 0;
    
    /* 计算可拷贝的数据长度 */
    uint16_t copy_len = g_bt_data.rx_len;
    if (copy_len > max_len - 1) {
        copy_len = max_len - 1;
    }
    
    /* 拷贝数据 */
    memcpy(buf, g_bt_data.rx_buf, copy_len);
    buf[copy_len] = '\0';
    
    return copy_len;
}

void BT_HC05_ClearRxBuffer(void)
{
    g_bt_data.rx_len = 0;
    g_bt_data.rx_flag = 0;
    memset(g_bt_data.rx_buf, 0, BT_RX_BUF_SIZE);
}

BT_ConnectState BT_HC05_GetConnectState(void)
{
    /* 更新连接状态 */
    BT_HC05_UpdateConnectState();
    return g_bt_data.connect_state;
}

uint8_t BT_HC05_IsConnected(void)
{
    return (BT_HC05_GetConnectState() == BT_CONNECTED);
}

uint8_t BT_HC05_SendIfConnected(const uint8_t *data, uint16_t len)
{
    if (BT_HC05_IsConnected()) {
        BT_HC05_SendData(data, len);
        return 1;
    }
    return 0;
}

uint32_t BT_HC05_GetRxCount(void)
{
    return g_bt_data.rx_count;
}

uint32_t BT_HC05_GetTxCount(void)
{
    return g_bt_data.tx_count;
}