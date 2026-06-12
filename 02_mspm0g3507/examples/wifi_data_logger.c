/**
 * @file wifi_data_logger.c
 * @brief WiFi数据记录器 - MSPM0G3507系统集成示例
 *
 * 功能：ESP8266 WiFi模块 + ADC传感器数据 + TCP上传服务器 + SD卡本地备份
 * 硬件：MSPM0G3507 + ESP8266(UART) + ADC传感器(PA25) + SD卡(SPI)
 *
 * 接线：
 *   ESP8266 TX  -> PA8  (UART0_RX)
 *   ESP8266 RX  -> PA9  (UART0_TX)
 *   ESP8266 VCC -> 3.3V (需独立供电)
 *   SD卡 MOSI   -> PA5  (SPI0)
 *   SD卡 MISO   -> PA6  (SPI0)
 *   SD卡 SCK    -> PA4  (SPI0)
 *   SD卡 CS     -> PA3  (GPIO)
 *   传感器      -> PA25 (ADC0_CH4)
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ========== WiFi配置 ========== */
#define WIFI_SSID       "YourSSID"
#define WIFI_PASS       "YourPassword"
#define TCP_SERVER_IP   "192.168.1.100"
#define TCP_SERVER_PORT 8080

/* ========== 引脚定义 ========== */
#define SD_CS_PORT      GPIOA
#define SD_CS_PIN       DL_GPIO_PIN_3
#define LED_PORT        GPIOB
#define LED_PIN         DL_GPIO_PIN_1

/* ========== 缓冲区 ========== */
#define UART_RX_BUF_SIZE    512
#define AT_CMD_BUF_SIZE     256

static volatile uint8_t  gUartRxBuffer[UART_RX_BUF_SIZE];
static volatile uint16_t gUartRxIndex = 0;
static volatile bool     gUartRxComplete = false;
static char gAtCmdBuf[AT_CMD_BUF_SIZE];

/* ========== 时间戳（软件计数） ========== */
static volatile uint32_t gTickMs = 0;

/* =================================================================
 * 基础延时与滴答
 * ================================================================= */

/**
 * @brief SysTick中断处理 - 1ms定时
 */
void SysTick_Handler(void)
{
    gTickMs++;
}

/**
 * @brief 毫秒延时
 */
static void delay_ms(uint32_t ms)
{
    uint32_t start = gTickMs;
    while ((gTickMs - start) < ms);
}

/* =================================================================
 * UART驱动（ESP8266通信）
 * ================================================================= */

/**
 * @brief UART0中断 - 接收ESP8266数据
 */
void UART0_IRQHandler(void)
{
    uint8_t ch;
    if (DL_UART_getEnabledInterruptStatus(UART0, DL_UART_INTERRUPT_RX) &
        DL_UART_INTERRUPT_RX) {
        ch = DL_UART_receiveData(UART0);
        if (gUartRxIndex < UART_RX_BUF_SIZE - 1) {
            gUartRxBuffer[gUartRxIndex++] = ch;
            gUartRxBuffer[gUartRxIndex] = '\0';
        }
        /* 检测到 "OK" 或 "ERROR" 视为响应完成 */
        if (gUartRxIndex >= 2) {
            if (strstr((const char *)gUartRxBuffer, "OK") ||
                strstr((const char *)gUartRxBuffer, "ERROR") ||
                strstr((const char *)gUartRxBuffer, "FAIL") ||
                strstr((const char *)gUartRxBuffer, ">")) {
                gUartRxComplete = true;
            }
        }
        DL_UART_clearInterruptStatus(UART0, DL_UART_INTERRUPT_RX);
    }
}

/**
 * @brief 发送AT命令并等待响应
 * @param cmd AT命令字符串
 * @param timeout_ms 超时时间
 * @return true=收到OK, false=超时或ERROR
 */
static bool ESP_SendCmd(const char *cmd, uint32_t timeout_ms)
{
    gUartRxIndex = 0;
    gUartRxComplete = false;
    memset((void *)gUartRxBuffer, 0, UART_RX_BUF_SIZE);

    /* 发送命令 */
    while (*cmd) {
        DL_UART_transmitData(UART0, *cmd);
        while (!DL_UART_isTXEmpty(UART0));
        cmd++;
    }
    /* 发送回车换行 */
    DL_UART_transmitData(UART0, '\r');
    while (!DL_UART_isTXEmpty(UART0));
    DL_UART_transmitData(UART0, '\n');
    while (!DL_UART_isTXEmpty(UART0));

    /* 等待响应 */
    uint32_t start = gTickMs;
    while (!gUartRxComplete && (gTickMs - start) < timeout_ms);

    if (strstr((const char *)gUartRxBuffer, "OK") ||
        strstr((const char *)gUartRxBuffer, ">")) {
        return true;
    }
    return false;
}

/**
 * @brief 发送透传数据
 */
static void ESP_SendData(const char *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        DL_UART_transmitData(UART0, data[i]);
        while (!DL_UART_isTXEmpty(UART0));
    }
}

/* =================================================================
 * ADC驱动（传感器读取）
 * ================================================================= */

/**
 * @brief 读取ADC通道值（模拟传感器数据）
 * @return ADC原始值 0-4095
 */
static uint16_t ADC_ReadSensor(void)
{
    DL_ADC12_startConversion(ADC0);
    while (!DL_ADC12_getStatus(ADC0) & DL_ADC12_STATUS_CONVERSION_DONE);
    return (uint16_t)DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
}

/**
 * @brief 将ADC值转换为温度（NTC示例）
 */
static float ADC_ToTemperature(uint16_t adc_val)
{
    /* 简化线性映射，实际需根据传感器校准 */
    if (adc_val == 0) adc_val = 1;
    float voltage = (float)adc_val / 4095.0f * 3.3f;
    /* 假设10k NTC, 10k分压, 25°C时2.5V */
    float temp = (voltage - 0.5f) * 100.0f;
    return temp;
}

/* =================================================================
 * SD卡SPI驱动（简化版）
 * ================================================================= */

/**
 * @brief SPI发送接收一个字节
 */
static uint8_t SPI_Transfer(uint8_t data)
{
    DL_SPI_transmitData8(SPI0, data);
    while (DL_SPI_isBusy(SPI0));
    return (uint8_t)DL_SPI_receiveData8(SPI0);
}

/**
 * @brief SD卡片选控制
 */
static void SD_CS_Low(void)  { DL_GPIO_clearPins(SD_CS_PORT, SD_CS_PIN); }
static void SD_CS_High(void) { DL_GPIO_setPins(SD_CS_PORT, SD_CS_PIN); }

/**
 * @brief SD卡写入数据（简化实现，仅示例框架）
 * @param data 要写入的字符串数据
 */
static void SD_WriteData(const char *data)
{
    SD_CS_Low();
    /* 实际需实现SD卡初始化、FAT文件系统等 */
    /* 这里仅演示SPI传输框架 */
    const char *p = data;
    while (*p) {
        SPI_Transfer(*p++);
    }
    SD_CS_High();
}

/* =================================================================
 * ESP8266初始化
 * ================================================================= */

/**
 * @brief 初始化ESP8266并连接WiFi和TCP服务器
 * @return true=成功
 */
static bool ESP8266_Init(void)
{
    /* 复位模块 */
    if (!ESP_SendCmd("AT+RST", 3000)) return false;
    delay_ms(2000);

    /* 测试AT指令 */
    if (!ESP_SendCmd("AT", 1000)) return false;

    /* 设置WiFi模式为Station */
    if (!ESP_SendCmd("AT+CWMODE=1", 1000)) return false;

    /* 连接WiFi热点 */
    snprintf(gAtCmdBuf, AT_CMD_BUF_SIZE,
             "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASS);
    if (!ESP_SendCmd(gAtCmdBuf, 15000)) return false;

    /* 设置单连接模式 */
    if (!ESP_SendCmd("AT+CIPMUX=0", 1000)) return false;

    /* 建立TCP连接 */
    snprintf(gAtCmdBuf, AT_CMD_BUF_SIZE,
             "AT+CIPSTART=\"TCP\",\"%s\",%d", TCP_SERVER_IP, TCP_SERVER_PORT);
    if (!ESP_SendCmd(gAtCmdBuf, 10000)) return false;

    return true;
}

/**
 * @brief 通过TCP发送传感器数据
 * @param json_data JSON格式数据
 * @return true=发送成功
 */
static bool ESP_SendSensorData(const char *json_data)
{
    uint16_t len = strlen(json_data);

    /* 发送数据长度 */
    snprintf(gAtCmdBuf, AT_CMD_BUF_SIZE, "AT+CIPSEND=%d", len);
    if (!ESP_SendCmd(gAtCmdBuf, 3000)) return false;

    /* 发送实际数据 */
    ESP_SendData(json_data, len);
    delay_ms(500);

    return true;
}

/* =================================================================
 * LED指示
 * ================================================================= */
static void LED_Toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
}

/* =================================================================
 * 主函数
 * ================================================================= */
int main(void)
{
    /* 系统初始化 */
    DL_SYSCFG_init();
    SysTick_Config(32000);  /* 32MHz / 32000 = 1ms */

    /* GPIO初始化 */
    DL_GPIO_initDigitalOutput(SD_CS_PIN);
    DL_GPIO_setPins(SD_CS_PORT, SD_CS_PIN);
    DL_GPIO_initDigitalOutput(LED_PIN);

    /* UART初始化（ESP8266） */
    NVIC_EnableIRQ(UART0_IRQn);

    /* ADC初始化 */
    DL_ADC12_initSingleSample(ADC0, DL_ADC12_MEM_IDX_0, DL_ADC12_INPUT_CHAN_4);

    /* SPI初始化（SD卡） */
    DL_SPI_enable(SPI0);

    /* 数据记录缓冲 */
    char json_buf[128];
    uint32_t sample_count = 0;
    uint32_t last_upload_time = 0;
    const uint32_t UPLOAD_INTERVAL_MS = 5000;  /* 每5秒上传一次 */

    /* 初始化WiFi */
    if (!ESP8266_Init()) {
        /* 初始化失败，LED快闪指示 */
        while (1) {
            LED_Toggle();
            delay_ms(100);
        }
    }

    /* 连接成功，LED常亮 */
    DL_GPIO_setPins(LED_PORT, LED_PIN);

    /* ===== 主循环 ===== */
    while (1) {
        /* 读取传感器 */
        uint16_t adc_raw = ADC_ReadSensor();
        float temperature = ADC_ToTemperature(adc_raw);
        sample_count++;

        /* 构造JSON数据包 */
        snprintf(json_buf, sizeof(json_buf),
                 "{\"id\":%lu,\"temp\":%.1f,\"adc\":%u,\"uptime\":%lu}\r\n",
                 (unsigned long)sample_count, temperature,
                 adc_raw, (unsigned long)(gTickMs / 1000));

        /* 本地SD卡备份 */
        SD_WriteData(json_buf);

        /* 定时TCP上传 */
        if ((gTickMs - last_upload_time) >= UPLOAD_INTERVAL_MS) {
            LED_Toggle();
            if (ESP_SendSensorData(json_buf)) {
                /* 上传成功，LED闪烁确认 */
                delay_ms(50);
                LED_Toggle();
            }
            last_upload_time = gTickMs;
        }

        /* 采样间隔1秒 */
        delay_ms(1000);
    }
}
