/**
 * @file    jy901s_tm4c.c
 * @brief   JY901S IMU模块驱动 实现文件 (TM4C123 UART中断接收)
 */

#include "jy901s_tm4c.h"
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"

/* ========== 内部缓冲区 ========== */
static volatile uint8_t  g_rx_buf[JY901S_FRAME_LEN];
static volatile uint8_t  g_rx_idx = 0;
static volatile bool     g_frame_ready = false;

static uint32_t g_uart_base = 0;
static int      g_uart_int  = 0;

static JY901S_Data_t g_imu_data;

/* ========== 内部: 解析一帧数据 ========== */
static void JY901S_ParseFrame(const uint8_t *frame)
{
    uint8_t type = frame[1];
    const int16_t *raw = (const int16_t *)&frame[2];

    /* 校验和 */
    uint8_t sum = 0;
    int i;
    for (i = 0; i < 10; i++) {
        sum += frame[i];
    }
    if (sum != frame[10]) return;

    switch (type) {
    case JY901S_TYPE_ACCEL:
        g_imu_data.accel_raw[0] = raw[0];
        g_imu_data.accel_raw[1] = raw[1];
        g_imu_data.accel_raw[2] = raw[2];
        g_imu_data.ax = (float)raw[0] / 32768.0f * 16.0f;
        g_imu_data.ay = (float)raw[1] / 32768.0f * 16.0f;
        g_imu_data.az = (float)raw[2] / 32768.0f * 16.0f;
        break;

    case JY901S_TYPE_GYRO:
        g_imu_data.gyro_raw[0] = raw[0];
        g_imu_data.gyro_raw[1] = raw[1];
        g_imu_data.gyro_raw[2] = raw[2];
        g_imu_data.wx = (float)raw[0] / 32768.0f * 2000.0f;
        g_imu_data.wy = (float)raw[1] / 32768.0f * 2000.0f;
        g_imu_data.wz = (float)raw[2] / 32768.0f * 2000.0f;
        break;

    case JY901S_TYPE_ANGLE:
        g_imu_data.angle_raw[0] = raw[0];
        g_imu_data.angle_raw[1] = raw[1];
        g_imu_data.angle_raw[2] = raw[2];
        g_imu_data.roll  = (float)raw[0] / 32768.0f * 180.0f;
        g_imu_data.pitch = (float)raw[1] / 32768.0f * 180.0f;
        g_imu_data.yaw   = (float)raw[2] / 32768.0f * 180.0f;
        g_imu_data.angle_updated = true;
        break;

    case JY901S_TYPE_MAG:
        g_imu_data.mx = raw[0];
        g_imu_data.my = raw[1];
        g_imu_data.mz = raw[2];
        break;

    default:
        break;
    }
}

/* ========== UART中断服务函数 ========== */
static void JY901S_UART_ISR(void)
{
    uint32_t status = UARTIntStatus(g_uart_base, true);
    UARTIntClear(g_uart_base, status);

    while (UARTCharsAvail(g_uart_base)) {
        uint8_t ch = (uint8_t)UARTCharGetNonBlocking(g_uart_base);

        /* 等待帧头0x55 */
        if (g_rx_idx == 0 && ch != JY901S_FRAME_HEAD) {
            continue;
        }

        g_rx_buf[g_rx_idx++] = ch;

        if (g_rx_idx >= JY901S_FRAME_LEN) {
            /* 完整帧接收完毕 */
            if (g_rx_buf[0] == JY901S_FRAME_HEAD) {
                JY901S_ParseFrame((const uint8_t *)g_rx_buf);
            }
            g_rx_idx = 0;
        }
    }
}

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

void JY901S_Init(const JY901S_Config_t *cfg)
{
    uint32_t uart_periph;
    uint32_t gpio_periph, gpio_base;
    uint32_t rx_pin, tx_pin, rx_cfg, tx_cfg;

    memset(&g_imu_data, 0, sizeof(g_imu_data));

    /* ---- 根据模块选择外设 ---- */
    if (cfg->uart_module == JY901S_UART4) {
        uart_periph = SYSCTL_PERIPH_UART4;
        gpio_periph = SYSCTL_PERIPH_GPIOC;
        gpio_base   = GPIO_PORTC_BASE;
        rx_pin      = GPIO_PIN_4;
        tx_pin      = GPIO_PIN_5;
        rx_cfg      = GPIO_PC4_U4RX;
        tx_cfg      = GPIO_PC5_U4TX;
        g_uart_base = UART4_BASE;
        g_uart_int  = INT_UART4;
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

    /* 4. 配置中断 - 仅RX中断 */
    UARTIntDisable(g_uart_base, 0xFFFFFFFF);
    UARTIntEnable(g_uart_base, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(g_uart_base, JY901S_UART_ISR);
    IntEnable(g_uart_int);

    /* 使能FIFO */
    UARTFIFOLevelSet(g_uart_base, UART_FIFO_TX4_8, UART_FIFO_RX1_8);
    UARTFIFOEnable(g_uart_base);

    /* 使能UART */
    UARTEnable(g_uart_base);
}

void JY901S_GetData(JY901S_Data_t *data)
{
    IntDisable(g_uart_int);
    *data = g_imu_data;
    IntEnable(g_uart_int);
}

void JY901S_GetAngle(float *roll, float *pitch, float *yaw)
{
    IntDisable(g_uart_int);
    *roll  = g_imu_data.roll;
    *pitch = g_imu_data.pitch;
    *yaw   = g_imu_data.yaw;
    IntEnable(g_uart_int);
}

bool JY901S_IsUpdated(void)
{
    return g_imu_data.angle_updated;
}

void JY901S_ClearUpdateFlag(void)
{
    g_imu_data.angle_updated = false;
}
