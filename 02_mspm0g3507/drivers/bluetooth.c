/**
 * @file    bluetooth.c
 * @brief   HC-05 蓝牙模块 UART 驱动实现 — MSPM0G3507
 *
 * SysConfig生成的宏:
 *   UART_1_INST, UART_1_INST_IRQHandler, UART_1_INST_INT_IRQN
 *   BT_EN (GPIO输出)
 */

#include "drivers/bluetooth.h"

/* ── 内部变量 ──────────────────────────────────────────────── */
static BT_Handle g_bt;

/* ── 内部辅助函数 ──────────────────────────────────────────── */

static inline uint16_t bt_rx_count(void)
{
    uint16_t h = g_bt.rx_head;
    uint16_t t = g_bt.rx_tail;
    return (uint16_t)((h - t) & (BT_RX_BUF_SIZE - 1));
}

static inline void bt_rx_put(uint8_t byte)
{
    uint16_t next = (g_bt.rx_head + 1) & (BT_RX_BUF_SIZE - 1);
    if (next != g_bt.rx_tail) {
        g_bt.rx_buf[g_bt.rx_head] = byte;
        g_bt.rx_head = next;
        g_bt.rx_count++;
    } else {
        /* 缓冲区满, 丢弃 */
        g_bt.overflow_count++;
    }
}

static inline int16_t bt_rx_get(void)
{
    if (g_bt.rx_head == g_bt.rx_tail) {
        return -1;
    }
    uint8_t byte = g_bt.rx_buf[g_bt.rx_tail];
    g_bt.rx_tail = (g_bt.rx_tail + 1) & (BT_RX_BUF_SIZE - 1);
    return (int16_t)byte;
}

/* 简单延时 (粗略, 基于循环) */
static void bt_delay_ms(uint32_t ms)
{
    volatile uint32_t count = ms * 10000;
    while (count--) {
        __NOP();
    }
}

/**
 * @brief 动态切换UART1波特率
 * @param baudRate 目标波特率
 */
static void bt_set_baudrate(uint32_t baudRate)
{
    /* 使用DriverLib API: 禁用→重配置→使能 */
    DL_UART_disable(UART_1_INST);
    DL_UART_configBaudRate(UART_1_INST, 32000000, baudRate);
    DL_UART_enable(UART_1_INST);
}

/* ── 公开API ──────────────────────────────────────────────── */

void BT_Init(void)
{
    /* 清零句柄 */
    memset((void *)&g_bt, 0, sizeof(g_bt));
    g_bt.state = BT_STATE_IDLE;
    g_bt.mode  = BT_MODE_TRANSPARENT;

    /* 默认透传模式: EN引脚拉低 */
    DL_GPIO_clearPins(BT_PORT, BT_EN_PIN);

    /* 使能UART1接收中断 */
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    DL_UART_enableInterrupt(UART_1_INST, DL_UART_INTERRUPT_RX);
}

void BT_EnterATMode(void)
{
    /* AT指令模式: EN拉高, 波特率38400 */
    DL_GPIO_setPins(BT_PORT, BT_EN_PIN);
    g_bt.mode = BT_MODE_AT;
    g_bt.state = BT_STATE_AT_PENDING;

    /* 等待HC-05进入AT模式 */
    bt_delay_ms(200);

    /* 切换波特率到38400 (HC-05 AT模式默认波特率) */
    bt_set_baudrate(38400);
}

void BT_EnterTransparentMode(void)
{
    /* 透传模式: EN拉低, 波特率9600 */
    DL_GPIO_clearPins(BT_PORT, BT_EN_PIN);
    g_bt.mode = BT_MODE_TRANSPARENT;
    g_bt.state = BT_STATE_IDLE;

    /* 恢复波特率到9600 */
    bt_set_baudrate(9600);

    bt_delay_ms(100);
}

bool BT_SendATCommand(const char *cmd, char *response, uint16_t resp_size, uint32_t timeout_ms)
{
    /* 确保在AT模式 */
    if (g_bt.mode != BT_MODE_AT) {
        BT_EnterATMode();
    }

    /* 清空接收缓冲区 */
    BT_Flush();

    /* 发送AT指令 */
    BT_SendString(cmd);
    BT_SendString("\r\n");

    /* 等待响应 */
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        bt_delay_ms(10);
        elapsed += 10;

        /* 检查是否有数据到达 */
        if (bt_rx_count() > 0) {
            /* 读取响应 */
            uint16_t idx = 0;
            while (bt_rx_count() > 0 && idx < resp_size - 1) {
                int16_t ch = bt_rx_get();
                if (ch >= 0) {
                    response[idx++] = (char)ch;
                }
            }
            response[idx] = '\0';
            return true;
        }
    }

    response[0] = '\0';
    return false;
}

BT_Mode BT_GetMode(void)
{
    return g_bt.mode;
}

void BT_SendByte(uint8_t data)
{
    while (!DL_UART_isTXFIFOEmpty(UART_1_INST));
    DL_UART_Main_transmitData(UART_1_INST, data);
    g_bt.tx_count++;
}

void BT_SendData(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        BT_SendByte(data[i]);
    }
}

void BT_SendString(const char *str)
{
    while (*str) {
        BT_SendByte((uint8_t)*str++);
    }
}

uint16_t BT_Available(void)
{
    return bt_rx_count();
}

int16_t BT_Read(void)
{
    return bt_rx_get();
}

uint16_t BT_ReadData(uint8_t *buf, uint16_t len)
{
    uint16_t count = 0;
    while (count < len && bt_rx_count() > 0) {
        int16_t ch = bt_rx_get();
        if (ch >= 0) {
            buf[count++] = (uint8_t)ch;
        }
    }
    return count;
}

uint16_t BT_ReadLine(char *buf, uint16_t max_len)
{
    static char line_buf[256];
    static uint16_t line_idx = 0;

    while (bt_rx_count() > 0) {
        int16_t ch = bt_rx_get();
        if (ch < 0) break;

        if (ch == '\n' || ch == '\r') {
            if (line_idx > 0) {
                /* 有完整行 */
                uint16_t copy_len = (line_idx < max_len - 1) ? line_idx : (max_len - 1);
                memcpy(buf, line_buf, copy_len);
                buf[copy_len] = '\0';
                line_idx = 0;
                return copy_len;
            }
            /* 忽略多余的换行 */
        } else {
            if (line_idx < sizeof(line_buf) - 1) {
                line_buf[line_idx++] = (char)ch;
            }
        }
    }
    return 0;
}

void BT_Flush(void)
{
    g_bt.rx_head = 0;
    g_bt.rx_tail = 0;
}

BT_State BT_GetState(void)
{
    return g_bt.state;
}

void BT_UART_IRQHandler(void)
{
    uint8_t data;

    switch (DL_UART_getPendingInterrupt(UART_1_INST)) {
    case DL_UART_IIDX_RX:
        data = DL_UART_receiveData(UART_1_INST);
        bt_rx_put(data);
        break;
    default:
        break;
    }
}
