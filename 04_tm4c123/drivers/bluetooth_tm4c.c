/**
 * @file    bluetooth_tm4c.c
 * @brief   HC-05蓝牙模块驱动 实现文件 (TM4C123 UART)
 */

#include "bluetooth_tm4c.h"
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"

/* ========== 内部环形缓冲区 ========== */
static volatile uint8_t  g_rx_buf[BT_RX_BUF_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;

static uint32_t g_uart_base = 0;
static int      g_uart_int  = 0;
static BT_State_t g_state = BT_STATE_IDLE;

/* ========== UART中断服务函数 ========== */
static void Bluetooth_UART_ISR(void)
{
    uint32_t status = UARTIntStatus(g_uart_base, true);
    UARTIntClear(g_uart_base, status);

    while (UARTCharsAvail(g_uart_base)) {
        uint8_t ch = (uint8_t)UARTCharGetNonBlocking(g_uart_base);

        /* 写入环形缓冲区 */
        uint16_t next = (g_rx_head + 1) % BT_RX_BUF_SIZE;
        if (next != g_rx_tail) {
            g_rx_buf[g_rx_head] = ch;
            g_rx_head = next;
        }
        /* 缓冲区满则丢弃 */
    }
}

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

void Bluetooth_Init(const Bluetooth_Config_t *cfg)
{
    uint32_t uart_periph;
    uint32_t gpio_periph, gpio_base;
    uint32_t rx_pin, tx_pin, rx_cfg, tx_cfg;

    g_rx_head = 0;
    g_rx_tail = 0;
    g_state = BT_STATE_IDLE;

    /* ---- 根据模块选择外设 ---- */
    if (cfg->uart_module == BT_UART3) {
        uart_periph = SYSCTL_PERIPH_UART3;
        gpio_periph = SYSCTL_PERIPH_GPIOC;
        gpio_base   = GPIO_PORTC_BASE;
        rx_pin      = GPIO_PIN_6;
        tx_pin      = GPIO_PIN_7;
        rx_cfg      = GPIO_PC6_U3RX;
        tx_cfg      = GPIO_PC7_U3TX;
        g_uart_base = UART3_BASE;
        g_uart_int  = INT_UART3;
    } else {
        uart_periph = SYSCTL_PERIPH_UART5;
        gpio_periph = SYSCTL_PERIPH_GPIOE;
        gpio_base   = GPIO_PORTE_BASE;
        rx_pin      = GPIO_PIN_4;
        tx_pin      = GPIO_PIN_5;
        rx_cfg      = GPIO_PE4_U5RX;
        tx_cfg      = GPIO_PE5_U5TX;
        g_uart_base = UART5_BASE;
        g_uart_int  = INT_UART5;
    }

    /* 1. 使能外设时钟 */
    SysCtlPeripheralEnable(gpio_periph);
    while (!SysCtlPeripheralReady(gpio_periph)) {}
    SysCtlPeripheralEnable(uart_periph);
    while (!SysCtlPeripheralReady(uart_periph)) {}

    /* 2. 配置GPIO引脚 */
    GPIOPinTypeUART(gpio_base, rx_pin | tx_pin);
    GPIOPinConfigure(rx_cfg);
    GPIOPinConfigure(tx_cfg);

    /* 3. 配置UART */
    UARTConfigSetExpClk(g_uart_base, cfg->sys_clock_hz, cfg->baudrate,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    /* 4. 配置中断 - RX中断 */
    UARTIntDisable(g_uart_base, 0xFFFFFFFF);
    UARTIntEnable(g_uart_base, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(g_uart_base, Bluetooth_UART_ISR);
    IntEnable(g_uart_int);

    /* 使能FIFO */
    UARTFIFOLevelSet(g_uart_base, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
    UARTFIFOEnable(g_uart_base);

    /* 5. 使能UART */
    UARTEnable(g_uart_base);
}

void Bluetooth_SendByte(uint8_t ch)
{
    UARTCharPut(g_uart_base, ch);
}

void Bluetooth_SendString(const char *str)
{
    while (*str) {
        UARTCharPut(g_uart_base, (uint8_t)*str);
        str++;
    }
}

void Bluetooth_SendData(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        UARTCharPut(g_uart_base, data[i]);
    }
}

uint16_t Bluetooth_Available(void)
{
    return (g_rx_head - g_rx_tail + BT_RX_BUF_SIZE) % BT_RX_BUF_SIZE;
}

int16_t Bluetooth_Read(void)
{
    if (g_rx_head == g_rx_tail) {
        return -1;
    }

    uint8_t ch = g_rx_buf[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1) % BT_RX_BUF_SIZE;
    return (int16_t)ch;
}

uint16_t Bluetooth_ReadLine(char *buf, uint16_t max_len)
{
    /* 查找缓冲区中的换行符 */
    uint16_t count = Bluetooth_Available();
    uint16_t i;
    bool found = false;

    for (i = 0; i < count && i < max_len - 1; i++) {
        uint16_t idx = (g_rx_tail + i) % BT_RX_BUF_SIZE;
        if (g_rx_buf[idx] == '\n') {
            found = true;
            break;
        }
    }

    if (!found) return 0;

    /* 读取到换行符 */
    uint16_t len = i + 1;
    for (i = 0; i < len; i++) {
        buf[i] = (char)Bluetooth_Read();
    }
    buf[len] = '\0';

    /* 去除末尾\r\n */
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
        buf[--len] = '\0';
    }

    return len;
}

BT_State_t Bluetooth_GetState(void)
{
    return g_state;
}

void Bluetooth_Flush(void)
{
    IntDisable(g_uart_int);
    g_rx_head = 0;
    g_rx_tail = 0;
    IntEnable(g_uart_int);
}
