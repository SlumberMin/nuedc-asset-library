#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
代码生成器 - 电赛备赛实用工具
===================================
功能：根据配置自动生成 STM32 驱动模板代码
用法：python code_generator.py --config peripherals.json --output ./generated
依赖：无额外依赖（纯Python实现）

支持生成的驱动模板：
  gpio      - GPIO初始化与控制代码
  uart      - UART串口通信代码
  pwm       - PWM输出代码
  adc       - ADC采集代码
  encoder   - 编码器测速代码
  spi       - SPI通信代码
  i2c       - I2C通信代码
  timer     - 定时器配置代码
  pid       - PID控制器代码
  hbridge   - H桥电机驱动代码
  oled      - OLED显示驱动代码
  mpu6050   - MPU6050传感器驱动代码

配置文件格式 (JSON):
{
  "project_name": "MyProject",
  "mcu": "STM32F103C8T6",
  "peripherals": [
    {"type": "uart", "instance": "USART1", "baudrate": 115200, ...},
    {"type": "pwm", "instance": "TIM3", "channel": 1, "frequency": 1000, ...}
  ]
}
"""

import argparse
import json
import os
import sys
from datetime import datetime

# ============================================================
# 代码模板库
# ============================================================

# ---- 头文件模板 ----
HEADER_TEMPLATE = """/**
 * @file    {filename}
 * @brief   {description}
 * @note    自动生成于 {timestamp}
 *          电赛备赛代码生成器
 *          项目: {project_name}
 *          MCU:  {mcu}
 */

#ifndef {guard}
#define {guard}

#ifdef __cplusplus
extern "C" {{
#endif

{includes}

{declarations}

#ifdef __cplusplus
}}
#endif

#endif /* {guard} */
"""

# ---- 源文件模板 ----
SOURCE_TEMPLATE = """/**
 * @file    {filename}
 * @brief   {description}
 * @note    自动生成于 {timestamp}
 *          电赛备赛代码生成器
 */

{includes}

{definitions}
"""


# ============================================================
# GPIO 代码生成
# ============================================================
def generate_gpio(config):
    """生成 GPIO 初始化与控制代码"""
    port = config.get('port', 'GPIOA')
    pin = config.get('pin', 0)
    mode = config.get('mode', 'output_push_pull')  # output_push_pull, output_open_drain, input, input_pullup, input_pulldown
    speed = config.get('speed', 'high')  # low, medium, high

    mode_map = {
        'output_push_pull': 'GPIO_Mode_Out_PP',
        'output_open_drain': 'GPIO_Mode_Out_OD',
        'input': 'GPIO_Mode_IN_FLOATING',
        'input_pullup': 'GPIO_Mode_IPU',
        'input_pulldown': 'GPIO_Mode_IPD',
    }
    speed_map = {
        'low': 'GPIO_Speed_2MHz',
        'medium': 'GPIO_Speed_10MHz',
        'high': 'GPIO_Speed_50MHz',
    }

    gpio_mode = mode_map.get(mode, 'GPIO_Mode_Out_PP')
    gpio_speed = speed_map.get(speed, 'GPIO_Speed_50MHz')
    pin_macro = f"GPIO_Pin_{pin}"

    header = f"""/**
 * @brief 初始化 {port}_{pin} 为 {mode} 模式
 */
void {port}_{pin}_Init(void);

/**
 * @brief 设置 {port}_{pin} 输出高电平
 */
void {port}_{pin}_Set(void);

/**
 * @brief 设置 {port}_{pin} 输出低电平
 */
void {port}_{pin}_Reset(void);

/**
 * @brief 翻转 {port}_{pin} 输出
 */
void {port}_{pin}_Toggle(void);

/**
 * @brief 读取 {port}_{pin} 输入状态
 * @return 0 或 1
 */
uint8_t {port}_{pin}_Read(void);
"""

    source = f"""/**
 * @brief 初始化 {port}_{pin}
 */
void {port}_{pin}_Init(void)
{{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_{port}, ENABLE);

    /* 配置GPIO */
    GPIO_InitStruct.GPIO_Pin = {pin_macro};
    GPIO_InitStruct.GPIO_Mode = {gpio_mode};
    GPIO_InitStruct.GPIO_Speed = {gpio_speed};
    GPIO_Init({port}, &GPIO_InitStruct);
}}

void {port}_{pin}_Set(void)
{{
    GPIO_SetBits({port}, {pin_macro});
}}

void {port}_{pin}_Reset(void)
{{
    GPIO_ResetBits({port}, {pin_macro});
}}

void {port}_{pin}_Toggle(void)
{{
    if (GPIO_ReadOutputDataBit({port}, {pin_macro}))
        GPIO_ResetBits({port}, {pin_macro});
    else
        GPIO_SetBits({port}, {pin_macro});
}}

uint8_t {port}_{pin}_Read(void)
{{
    return GPIO_ReadInputDataBit({port}, {pin_macro});
}}
"""
    return header, source


# ============================================================
# UART 代码生成
# ============================================================
def generate_uart(config):
    """生成 UART 串口通信代码"""
    instance = config.get('instance', 'USART1')
    baudrate = config.get('baudrate', 115200)
    rx_interrupt = config.get('rx_interrupt', True)  # 是否启用接收中断
    dma_tx = config.get('dma_tx', False)  # 是否使用DMA发送

    # 引脚映射 (STM32F103)
    pin_map = {
        'USART1': ('GPIOA', 9, 'GPIOA', 10, 'RCC_APB2Periph_USART1'),
        'USART2': ('GPIOA', 2, 'GPIOA', 3, 'RCC_APB1Periph_USART2'),
        'USART3': ('GPIOB', 10, 'GPIOB', 11, 'RCC_APB1Periph_USART3'),
    }

    info = pin_map.get(instance, ('GPIOA', 9, 'GPIOA', 10, 'RCC_APB2Periph_USART1'))
    tx_port, tx_pin, rx_port, rx_pin, clock = info
    ap_type = 'APB2' if 'APB2' in clock else 'APB1'

    header = f"""#include <stdint.h>

#define {instance}_RX_BUF_SIZE    256

/* 接收缓冲区 */
extern uint8_t {instance}_rx_buf[];
extern volatile uint16_t {instance}_rx_cnt;
extern volatile uint8_t {instance}_rx_flag;

/**
 * @brief 初始化 {instance}，波特率 {baudrate}
 */
void {instance}_Init(void);

/**
 * @brief 发送单个字节
 */
void {instance}_SendByte(uint8_t data);

/**
 * @brief 发送字符串
 */
void {instance}_SendString(const char *str);

/**
 * @brief 发送数据块
 */
void {instance}_SendData(uint8_t *data, uint16_t len);

/**
 * @brief 格式化打印 (类似printf)
 */
void {instance}_Printf(const char *fmt, ...);

/**
 * @brief 获取接收到的一行数据
 * @return 0=无数据, 1=有数据已拷贝到buf
 */
uint8_t {instance}_GetLine(char *buf, uint16_t max_len);
"""

    irq_handler = ""
    if rx_interrupt:
        irq_n = instance.replace('USART', 'USART')
        irq_handler = f"""
/**
 * @brief {instance} 中断处理函数 (需在 stm32f10x_it.c 中调用)
 */
void {instance}_IRQHandler(void)
{{
    if (USART_GetITStatus({instance}, USART_IT_RXNE) != RESET)
    {{
        uint8_t ch = USART_ReceiveData({instance});
        if ({instance}_rx_cnt < {instance}_RX_BUF_SIZE - 1)
        {{
            {instance}_rx_buf[{instance}_rx_cnt++] = ch;
            if (ch == '\\n' || ch == '\\r')
            {{
                {instance}_rx_buf[{instance}_rx_cnt] = '\\0';
                {instance}_rx_flag = 1;
            }}
        }}
        USART_ClearITPendingBit({instance}, USART_IT_RXNE);
    }}
}}
"""

    source = f"""#include "{instance.lower()}.h"
#include "{instance.lower()}_cfg.h"
#include "stm32f10x.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* 接收缓冲区 */
uint8_t {instance}_rx_buf[{instance}_RX_BUF_SIZE];
volatile uint16_t {instance}_rx_cnt = 0;
volatile uint8_t {instance}_rx_flag = 0;

/**
 * @brief 初始化 {instance}
 * @note  TX: {tx_port}.{tx_pin}  RX: {rx_port}.{rx_pin}
 *        波特率: {baudrate}
 */
void {instance}_Init(void)
{{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* 使能时钟 */
    RCC_{ap_type}PeriphClockCmd({clock}, ENABLE);
    {"RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);" if ap_type == "APB1" else "RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);"}

    /* TX引脚: 复用推挽输出 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_{tx_pin};
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init({tx_port}, &GPIO_InitStruct);

    /* RX引脚: 浮空输入 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_{rx_pin};
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init({rx_port}, &GPIO_InitStruct);

    /* USART配置 */
    USART_InitStruct.USART_BaudRate = {baudrate};
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init({instance}, &USART_InitStruct);

    USART_Cmd({instance}, ENABLE);

    /* 使能接收中断 */
    USART_ITConfig({instance}, USART_IT_RXNE, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = {instance.replace('USART', 'USART')}_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}}

void {instance}_SendByte(uint8_t data)
{{
    USART_SendData({instance}, data);
    while (USART_GetFlagStatus({instance}, USART_FLAG_TXE) == RESET);
}}

void {instance}_SendString(const char *str)
{{
    while (*str)
    {{
        {instance}_SendByte(*str++);
    }}
}}

void {instance}_SendData(uint8_t *data, uint16_t len)
{{
    for (uint16_t i = 0; i < len; i++)
    {{
        {instance}_SendByte(data[i]);
    }}
}}

void {instance}_Printf(const char *fmt, ...)
{{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    {instance}_SendString(buf);
}}

uint8_t {instance}_GetLine(char *buf, uint16_t max_len)
{{
    if ({instance}_rx_flag)
    {{
        uint16_t len = {instance}_rx_cnt;
        if (len >= max_len) len = max_len - 1;
        memcpy(buf, {instance}_rx_buf, len);
        buf[len] = '\\0';
        {instance}_rx_cnt = 0;
        {instance}_rx_flag = 0;
        return 1;
    }}
    return 0;
}}
{irq_handler}
"""
    return header, source


# ============================================================
# PWM 代码生成
# ============================================================
def generate_pwm(config):
    """生成 PWM 输出代码"""
    instance = config.get('instance', 'TIM3')
    channel = config.get('channel', 1)
    frequency = config.get('frequency', 1000)  # Hz
    resolution = config.get('resolution', 1000)  # 占空比分辨率
    arr_val = config.get('arr', None)  # 自动重装值（自动计算或手动指定）

    # TIM 引脚映射
    pin_map = {
        'TIM3_CH1': ('GPIOA', 6, 'GPIO_Pin_6'),
        'TIM3_CH2': ('GPIOA', 7, 'GPIO_Pin_7'),
        'TIM3_CH3': ('GPIOB', 0, 'GPIO_Pin_0'),
        'TIM3_CH4': ('GPIOB', 1, 'GPIO_Pin_1'),
        'TIM2_CH1': ('GPIOA', 0, 'GPIO_Pin_0'),
        'TIM2_CH2': ('GPIOA', 1, 'GPIO_Pin_1'),
        'TIM4_CH1': ('GPIOB', 6, 'GPIO_Pin_6'),
        'TIM4_CH2': ('GPIOB', 7, 'GPIO_Pin_7'),
        'TIM4_CH3': ('GPIOB', 8, 'GPIO_Pin_8'),
        'TIM4_CH4': ('GPIOB', 9, 'GPIO_Pin_9'),
    }

    key = f'{instance}_CH{channel}'
    port, pin_num, pin_macro = pin_map.get(key, ('GPIOA', 6, 'GPIO_Pin_6'))

    header = f"""#include <stdint.h>

/**
 * @brief 初始化 {instance} CH{channel} PWM输出
 * @note  频率: {frequency}Hz  分辨率: {resolution}级
 *        引脚: {port}.{pin_num}
 */
void {instance}_CH{channel}_PWM_Init(void);

/**
 * @brief 设置PWM占空比
 * @param duty 0 ~ {resolution - 1}
 */
void {instance}_CH{channel}_SetDuty(uint16_t duty);

/**
 * @brief 设置PWM占空比百分比
 * @param percent 0.0 ~ 100.0
 */
void {instance}_CH{channel}_SetPercent(float percent);

/**
 * @brief 获取当前最大占空比值
 */
uint16_t {instance}_CH{channel}_GetMaxDuty(void);
"""

    source = f"""#include "pwm_{instance.lower()}_ch{channel}.h"
#include "stm32f10x.h"

/* PWM 配置参数 */
#define PWM_TIM             {instance}
#define PWM_TIM_CHANNEL     TIM_Channel_{channel}
#define PWM_FREQUENCY       {frequency}
#define PWM_RESOLUTION      {resolution}
#define PWM_TIM_PERIOD      (SystemCoreClock / 72 / PWM_FREQUENCY - 1)

/**
 * @brief 初始化 {instance} CH{channel} PWM
 * @note  时钟: 72MHz (APB1 x2)
 *        预分频: 72 (1MHz计数频率)
 *        ARR: 自动计算
 */
void {instance}_CH{channel}_PWM_Init(void)
{{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_{instance}, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_{port}, ENABLE);

    /* 配置PWM输出引脚 */
    GPIO_InitStruct.GPIO_Pin = {pin_macro};
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init({port}, &GPIO_InitStruct);

    /* 时基配置 */
    TIM_TimeBaseStruct.TIM_Prescaler = 72 - 1;          /* 1MHz */
    TIM_TimeBaseStruct.TIM_Period = PWM_TIM_PERIOD;      /* PWM频率 */
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStruct);

    /* PWM模式配置 */
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;

    /* 使能对应通道 */
    TIM_OC{channel}Init(PWM_TIM, &TIM_OCInitStruct);
    TIM_OC{channel}PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(PWM_TIM, ENABLE);
    TIM_Cmd(PWM_TIM, ENABLE);
}}

void {instance}_CH{channel}_SetDuty(uint16_t duty)
{{
    if (duty > PWM_TIM_PERIOD) duty = PWM_TIM_PERIOD;
    switch (PWM_TIM_CHANNEL)
    {{
        case TIM_Channel_1: TIM_SetCompare1(PWM_TIM, duty); break;
        case TIM_Channel_2: TIM_SetCompare2(PWM_TIM, duty); break;
        case TIM_Channel_3: TIM_SetCompare3(PWM_TIM, duty); break;
        case TIM_Channel_4: TIM_SetCompare4(PWM_TIM, duty); break;
    }}
}}

void {instance}_CH{channel}_SetPercent(float percent)
{{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    uint16_t duty = (uint16_t)(percent / 100.0f * PWM_TIM_PERIOD);
    {instance}_CH{channel}_SetDuty(duty);
}}

uint16_t {instance}_CH{channel}_GetMaxDuty(void)
{{
    return PWM_TIM_PERIOD;
}}
"""
    return header, source


# ============================================================
# ADC 代码生成
# ============================================================
def generate_adc(config):
    """生成 ADC 采集代码"""
    instance = config.get('instance', 'ADC1')
    channels = config.get('channels', [0])  # ADC通道列表
    trigger = config.get('trigger', 'software')  # software, tim2_trgo, tim3_trgo
    continuous = config.get('continuous', True)  # 连续转换模式
    dma = config.get('dma', True)  # 是否使用DMA

    channel_names = ','.join(f'ADC_Channel_{ch}' for ch in channels)
    num_channels = len(channels)

    header = f"""#include <stdint.h>

#define ADC_NUM_CHANNELS    {num_channels}

extern volatile uint16_t adc_values[ADC_NUM_CHANNELS];

/**
 * @brief 初始化 {instance}，多通道采集
 * @note  通道: {channels}
 *        {'DMA模式' if dma else '轮询模式'}
 */
void {instance}_Init(void);

/**
 * @brief 启动ADC采集
 */
void {instance}_Start(void);

/**
 * @brief 停止ADC采集
 */
void {instance}_Stop(void);

/**
 * @brief 获取ADC原始值
 * @param ch 通道索引 (0 ~ {num_channels - 1})
 * @return 12位ADC值
 */
uint16_t {instance}_GetValue(uint8_t ch);

/**
 * @brief 获取ADC电压值
 * @param ch 通道索引
 * @return 电压值 (V)
 */
float {instance}_GetVoltage(uint8_t ch);

/**
 * @brief 获取ADC均值滤波后的电压
 * @param ch 通道索引
 * @param samples 滤波采样数
 * @return 电压值 (V)
 */
float {instance}_GetVoltageAvg(uint8_t ch, uint8_t samples);
"""

    dma_config = ""
    if dma:
        dma_config = f"""
    /* DMA配置 */
    DMA_InitTypeDef DMA_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&({instance}->DR);
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)adc_values;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize = {num_channels};
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_DMACmd({instance}, ENABLE);
"""

    source = f"""#include "adc_{instance.lower()}.h"
#include "stm32f10x.h"

volatile uint16_t adc_values[ADC_NUM_CHANNELS] = {{0}};

void {instance}_Init(void)
{{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    /* 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_{instance} | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);  /* ADC时钟 = 72/6 = 12MHz */

    /* 配置ADC输入引脚 (PA0-PA7) */
    /* 根据实际使用的通道配置对应的GPIO */
    GPIO_InitStruct.GPIO_Pin = 0;
    {' | '.join(f'GPIO_Pin_{ch}' for ch in channels if ch < 8)};
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC配置 */
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStruct.ADC_ScanConvMode = {'ENABLE' if num_channels > 1 else 'DISABLE'};
    ADC_InitStruct.ADC_ContinuousConvMode = {'ENABLE' if continuous else 'DISABLE'};
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfChannel = {num_channels};
    ADC_Init({instance}, &ADC_InitStruct);
{dma_config}
    /* 配置采样通道和顺序 */
    /* 示例：每个通道采样239.5周期 */
    {''.join(f'ADC_RegularChannelConfig({instance}, ADC_Channel_{ch}, {i+1}, ADC_SampleTime_239Cycles5);' + chr(10) + '    ' for i, ch in enumerate(channels))}

    ADC_Cmd({instance}, ENABLE);

    /* ADC校准 */
    ADC_ResetCalibration({instance});
    while (ADC_GetResetCalibrationStatus({instance}));
    ADC_StartCalibration({instance});
    while (ADC_GetCalibrationStatus({instance}));
}}

void {instance}_Start(void)
{{
    ADC_SoftwareStartConvCmd({instance}, ENABLE);
}}

void {instance}_Stop(void)
{{
    ADC_SoftwareStartConvCmd({instance}, DISABLE);
}}

uint16_t {instance}_GetValue(uint8_t ch)
{{
    if (ch >= ADC_NUM_CHANNELS) return 0;
    return adc_values[ch];
}}

float {instance}_GetVoltage(uint8_t ch)
{{
    if (ch >= ADC_NUM_CHANNELS) return 0.0f;
    return (float)adc_values[ch] * 3.3f / 4096.0f;
}}

float {instance}_GetVoltageAvg(uint8_t ch, uint8_t samples)
{{
    if (ch >= ADC_NUM_CHANNELS || samples == 0) return 0.0f;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++)
    {{
        ADC_SoftwareStartConvCmd({instance}, ENABLE);
        while (!ADC_GetFlagStatus({instance}, ADC_FLAG_EOC));
        sum += ADC_GetConversionValue({instance});
    }}
    return (float)sum / samples * 3.3f / 4096.0f;
}}
"""
    return header, source


# ============================================================
# 编码器代码生成
# ============================================================
def generate_encoder(config):
    """生成编码器测速代码"""
    instance = config.get('instance', 'TIM2')
    ppr = config.get('ppr', 13)  # 编码器线数
    gear_ratio = config.get('gear_ratio', 30)  # 减速比
    sample_time_ms = config.get('sample_time_ms', 10)  # 采样周期

    effective_ppr = ppr * gear_ratio * 4  # 四倍频后的总脉冲数/转

    header = f"""#include <stdint.h>

/**
 * @brief 初始化 {instance} 编码器模式
 * @note  编码器: {ppr}线, 减速比: {gear_ratio}:1, 四倍频
 *        有效分辨率: {effective_ppr} 脉冲/转
 */
void Encoder_{instance}_Init(void);

/**
 * @brief 获取编码器累计计数值
 * @return 有符号32位计数值
 */
int32_t Encoder_{instance}_GetCount(void);

/**
 * @brief 清零编码器计数
 */
void Encoder_{instance}_Reset(void);

/**
 * @brief 获取当前转速 (RPM)
 * @note  需周期调用，内部自动计算
 */
float Encoder_{instance}_GetRPM(void);

/**
 * @brief 获取当前转速 (弧度/秒)
 */
float Encoder_{instance}_GetRadPerSec(void);

/**
 * @brief 编码器定时更新（需在定时器中断中调用）
 */
void Encoder_{instance}_Update(void);
"""

    source = f"""#include "encoder_{instance.lower()}.h"
#include "stm32f10x.h"

/* 配置参数 */
#define ENCODER_TIM         {instance}
#define ENCODER_PPR         {ppr}
#define ENCODER_GEAR_RATIO  {gear_ratio}
#define ENCODER_PPR_TOTAL   ({ppr} * {gear_ratio} * 4)  /* 四倍频 */
#define SAMPLE_TIME_MS      {sample_time_ms}

/* 内部变量 */
static volatile int32_t encoder_count = 0;
static volatile int32_t encoder_speed = 0;      /* 脉冲/采样周期 */
static volatile float encoder_rpm = 0.0f;
static int32_t last_count = 0;

void Encoder_{instance}_Init(void)
{{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_{instance}, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* 编码器输入引脚 (上拉输入) */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 时基配置: 不分频，向上计数 */
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF;  /* 最大计数值 */
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENCODER_TIM, &TIM_TimeBaseStruct);

    /* 编码器模式3: TI1和TI2双边沿都计数 (四倍频) */
    TIM_EncoderInterfaceConfig(ENCODER_TIM,
        TIM_EncoderMode_TI12,
        TIM_ICPolarity_Rising,
        TIM_ICPolarity_Rising);

    /* 输入捕获滤波 */
    TIM_ICInitStruct.TIM_ICFilter = 6;
    TIM_ICInit(ENCODER_TIM, &TIM_ICInitStruct);

    TIM_SetCounter(ENCODER_TIM, 0);
    TIM_Cmd(ENCODER_TIM, ENABLE);
}}

int32_t Encoder_{instance}_GetCount(void)
{{
    return (int16_t)TIM_GetCounter(ENCODER_TIM);
}}

void Encoder_{instance}_Reset(void)
{{
    TIM_SetCounter(ENCODER_TIM, 0);
    last_count = 0;
    encoder_count = 0;
    encoder_speed = 0;
    encoder_rpm = 0.0f;
}}

float Encoder_{instance}_GetRPM(void)
{{
    return encoder_rpm;
}}

float Encoder_{instance}_GetRadPerSec(void)
{{
    return encoder_rpm * 2.0f * 3.1415926f / 60.0f;
}}

/**
 * @brief 编码器速度更新
 * @note  需在固定周期的定时器中断中调用（如 {sample_time_ms}ms）
 *        计算公式: RPM = (脉冲增量 / 采样时间) * (60 / PPR_TOTAL)
 */
void Encoder_{instance}_Update(void)
{{
    int16_t current = (int16_t)TIM_GetCounter(ENCODER_TIM);
    int16_t delta = current - (int16_t)(last_count & 0xFFFF);
    last_count = current;
    encoder_count += delta;

    /* 计算RPM */
    /* delta: 采样周期内的脉冲数 */
    /* RPM = delta / PPR_TOTAL / (sample_time_ms / 1000 / 60) */
    encoder_rpm = (float)delta / ENCODER_PPR_TOTAL * (60000.0f / SAMPLE_TIME_MS);

    /* 简单低通滤波 */
    static float filtered_rpm = 0.0f;
    filtered_rpm = filtered_rpm * 0.7f + encoder_rpm * 0.3f;
    encoder_rpm = filtered_rpm;
}}
"""
    return header, source


# ============================================================
# PID 控制器代码生成
# ============================================================
def generate_pid(config):
    """生成 PID 控制器代码"""
    instance_name = config.get('name', 'motor')
    kp = config.get('kp', 1.0)
    ki = config.get('ki', 0.0)
    kd = config.get('kd', 0.0)
    output_min = config.get('output_min', -1000)
    output_max = config.get('output_max', 1000)
    integral_max = config.get('integral_max', 500)
    sample_time_ms = config.get('sample_time_ms', 10)

    header = f"""#include <stdint.h>

typedef struct {{
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float kd;           /* 微分系数 */
    float target;       /* 目标值 */
    float output_min;   /* 输出下限 */
    float output_max;   /* 输出上限 */
    float integral_max; /* 积分限幅 */
    float error;        /* 当前误差 */
    float integral;     /* 积分累加 */
    float prev_error;   /* 上次误差 */
    float output;       /* 输出值 */
}} PID_{instance_name}_t;

extern PID_{instance_name}_t pid_{instance_name};

/**
 * @brief 初始化 {instance_name} PID控制器
 */
void PID_{instance_name}_Init(void);

/**
 * @brief 设置PID参数
 */
void PID_{instance_name}_SetParams(float kp, float ki, float kd);

/**
 * @brief 设置目标值
 */
void PID_{instance_name}_SetTarget(float target);

/**
 * @brief PID计算（周期调用）
 * @param measured 当前测量值
 * @return 控制输出
 */
float PID_{instance_name}_Compute(float measured);

/**
 * @brief 重置PID积分和历史
 */
void PID_{instance_name}_Reset(void);
"""

    source = f"""#include "pid_{instance_name}.h"

/* PID 控制器实例 */
PID_{instance_name}_t pid_{instance_name};

void PID_{instance_name}_Init(void)
{{
    pid_{instance_name}.kp = {kp}f;
    pid_{instance_name}.ki = {ki}f;
    pid_{instance_name}.kd = {kd}f;
    pid_{instance_name}.target = 0.0f;
    pid_{instance_name}.output_min = {output_min}f;
    pid_{instance_name}.output_max = {output_max}f;
    pid_{instance_name}.integral_max = {integral_max}f;
    pid_{instance_name}.error = 0.0f;
    pid_{instance_name}.integral = 0.0f;
    pid_{instance_name}.prev_error = 0.0f;
    pid_{instance_name}.output = 0.0f;
}}

void PID_{instance_name}_SetParams(float kp, float ki, float kd)
{{
    pid_{instance_name}.kp = kp;
    pid_{instance_name}.ki = ki;
    pid_{instance_name}.kd = kd;
}}

void PID_{instance_name}_SetTarget(float target)
{{
    pid_{instance_name}.target = target;
}}

/**
 * @brief 增量式PID / 位置式PID (带抗积分饱和)
 *
 * 位置式PID:
 *   output = Kp*e + Ki*Σe + Kd*(e-e_prev)
 *
 * 抗积分饱和: 当输出达到限幅值时停止积分累加
 */
float PID_{instance_name}_Compute(float measured)
{{
    PID_{instance_name}_t *p = &pid_{instance_name};

    /* 计算误差 */
    p->error = p->target - measured;

    /* 积分累加（带限幅） */
    p->integral += p->error;
    if (p->integral > p->integral_max)
        p->integral = p->integral_max;
    if (p->integral < -p->integral_max)
        p->integral = -p->integral_max;

    /* 计算微分项（使用误差微分，可改为测量值微分以避免阶跃抖动） */
    float derivative = p->error - p->prev_error;

    /* PID 输出 */
    float output = p->kp * p->error
                 + p->ki * p->integral
                 + p->kd * derivative;

    /* 输出限幅 */
    if (output > p->output_max)
        output = p->output_max;
    if (output < p->output_min)
        output = p->output_min;

    /* 抗积分饱和：输出饱和时回退积分 */
    if (output >= p->output_max || output <= p->output_min)
    {{
        p->integral -= p->error;  /* 撤销本次积分 */
    }}

    p->prev_error = p->error;
    p->output = output;

    return output;
}}

void PID_{instance_name}_Reset(void)
{{
    pid_{instance_name}.error = 0.0f;
    pid_{instance_name}.integral = 0.0f;
    pid_{instance_name}.prev_error = 0.0f;
    pid_{instance_name}.output = 0.0f;
}}
"""
    return header, source


# ============================================================
# H桥电机驱动代码生成
# ============================================================
def generate_hbridge(config):
    """生成 H 桥电机驱动代码"""
    instance = config.get('name', 'motor1')
    pwm_timer = config.get('pwm_timer', 'TIM3')
    pwm_channel = config.get('pwm_channel', 1)
    dir_port = config.get('dir_port', 'GPIOB')
    dir_pin_a = config.get('dir_pin_a', 12)
    dir_pin_b = config.get('dir_pin_b', 13)
    pwm_max = config.get('pwm_max', 1000)

    header = f"""#include <stdint.h>

/**
 * @brief 初始化 {instance} H桥驱动
 * @note  PWM: {pwm_timer}_CH{pwm_channel}
 *        方向: {dir_port}.{dir_pin_a}, {dir_port}.{dir_pin_b}
 */
void {instance}_Init(void);

/**
 * @brief 设置电机速度和方向
 * @param speed -{pwm_max} ~ +{pwm_max}，负值反转
 */
void {instance}_SetSpeed(int16_t speed);

/**
 * @brief 电机刹车（能耗制动）
 */
void {instance}_Brake(void);

/**
 * @brief 电机滑行（停止输出）
 */
void {instance}_Coast(void);
"""

    source = f"""#include "hbridge_{instance}.h"
#include "stm32f10x.h"

/* H桥引脚定义 */
#define DIR_PORT    {dir_port}
#define PIN_A       GPIO_Pin_{dir_pin_a}
#define PIN_B       GPIO_Pin_{dir_pin_b}
#define PWM_MAX     {pwm_max}

void {instance}_Init(void)
{{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 方向控制引脚 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_{dir_port}, ENABLE);
    GPIO_InitStruct.GPIO_Pin = PIN_A | PIN_B;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR_PORT, &GPIO_InitStruct);

    /* 初始化PWM (调用PWM模块) */
    /* {pwm_timer}_CH{pwm_channel}_PWM_Init(); */

    {instance}_Brake();
}}

void {instance}_SetSpeed(int16_t speed)
{{
    /* 限幅 */
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    if (speed > 0)
    {{
        /* 正转 */
        GPIO_SetBits(DIR_PORT, PIN_A);
        GPIO_ResetBits(DIR_PORT, PIN_B);
        /* 设置PWM占空比 */
        /* {pwm_timer}_CH{pwm_channel}_SetDuty(speed); */
    }}
    else if (speed < 0)
    {{
        /* 反转 */
        GPIO_ResetBits(DIR_PORT, PIN_A);
        GPIO_SetBits(DIR_PORT, PIN_B);
        /* 设置PWM占空比 */
        /* {pwm_timer}_CH{pwm_channel}_SetDuty(-speed); */
    }}
    else
    {{
        {instance}_Brake();
    }}
}}

void {instance}_Brake(void)
{{
    /* 两个方向引脚都置高，H桥短路制动 */
    GPIO_SetBits(DIR_PORT, PIN_A);
    GPIO_SetBits(DIR_PORT, PIN_B);
    /* {pwm_timer}_CH{pwm_channel}_SetDuty(0); */
}}

void {instance}_Coast(void)
{{
    /* 两个方向引脚都置低，电机自由转动 */
    GPIO_ResetBits(DIR_PORT, PIN_A);
    GPIO_ResetBits(DIR_PORT, PIN_B);
    /* {pwm_timer}_CH{pwm_channel}_SetDuty(0); */
}}
"""
    return header, source


# ============================================================
# 代码生成器注册表
# ============================================================
GENERATORS = {
    'gpio': generate_gpio,
    'uart': generate_uart,
    'pwm': generate_pwm,
    'adc': generate_adc,
    'encoder': generate_encoder,
    'pid': generate_pid,
    'hbridge': generate_hbridge,
}


# ============================================================
# 文件生成器
# ============================================================
def generate_files(config, output_dir):
    """根据配置生成所有驱动文件"""
    project_name = config.get('project_name', 'MyProject')
    mcu = config.get('mcu', 'STM32F103C8T6')
    peripherals = config.get('peripherals', [])

    os.makedirs(output_dir, exist_ok=True)
    generated_files = []

    for idx, periph in enumerate(peripherals):
        ptype = periph.get('type', '')
        if ptype not in GENERATORS:
            print(f"  [跳过] 不支持的外设类型: {ptype}")
            continue

        print(f"  [{idx+1}] 生成 {ptype} 驱动...", end=' ')
        header, source = GENERATORS[ptype](periph)

        # 确定文件名
        instance = periph.get('instance', periph.get('name', f'{ptype}{idx}')).lower()
        if ptype == 'gpio':
            port = periph.get('port', 'GPIOA').lower()
            pin = periph.get('pin', 0)
            basename = f'gpio_{port}_{pin}'
        elif ptype == 'uart':
            basename = instance.lower()
        elif ptype == 'pwm':
            ch = periph.get('channel', 1)
            basename = f'pwm_{instance}_ch{ch}'
        elif ptype == 'adc':
            basename = f'adc_{instance}'
        elif ptype == 'encoder':
            basename = f'encoder_{instance}'
        elif ptype == 'pid':
            name = periph.get('name', 'motor')
            basename = f'pid_{name}'
        elif ptype == 'hbridge':
            name = periph.get('name', 'motor1')
            basename = f'hbridge_{name}'
        else:
            basename = f'{ptype}_{instance}'

        # 写入头文件
        h_filename = f'{basename}.h'
        h_path = os.path.join(output_dir, h_filename)
        guard = h_filename.upper().replace('.', '_').replace('-', '_')
        h_content = HEADER_TEMPLATE.format(
            filename=h_filename,
            description=f'{ptype.upper()} 驱动 - {instance}',
            timestamp=datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            project_name=project_name,
            mcu=mcu,
            guard=guard,
            includes=f'#include <stdint.h>\n#include "stm32f10x.h"',
            declarations=header,
        )
        with open(h_path, 'w', encoding='utf-8') as f:
            f.write(h_content)

        # 写入源文件
        c_filename = f'{basename}.c'
        c_path = os.path.join(output_dir, c_filename)
        c_content = SOURCE_TEMPLATE.format(
            filename=c_filename,
            description=f'{ptype.upper()} 驱动实现 - {instance}',
            timestamp=datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            includes=f'#include "{h_filename}"\n#include "stm32f10x.h"',
            definitions=source,
        )
        with open(c_path, 'w', encoding='utf-8') as f:
            f.write(c_content)

        generated_files.extend([h_path, c_path])
        print(f"OK -> {h_filename}, {c_filename}")

    return generated_files


# ============================================================
# 示例配置生成
# ============================================================
def generate_example_config():
    """生成示例配置文件"""
    config = {
        "project_name": "电赛Demo",
        "mcu": "STM32F103C8T6",
        "peripherals": [
            {
                "type": "uart",
                "instance": "USART1",
                "baudrate": 115200,
                "rx_interrupt": True
            },
            {
                "type": "uart",
                "instance": "USART2",
                "baudrate": 115200,
                "rx_interrupt": True
            },
            {
                "type": "pwm",
                "instance": "TIM3",
                "channel": 1,
                "frequency": 10000
            },
            {
                "type": "pwm",
                "instance": "TIM3",
                "channel": 2,
                "frequency": 10000
            },
            {
                "type": "encoder",
                "instance": "TIM2",
                "ppr": 13,
                "gear_ratio": 30,
                "sample_time_ms": 10
            },
            {
                "type": "encoder",
                "instance": "TIM4",
                "ppr": 13,
                "gear_ratio": 30,
                "sample_time_ms": 10
            },
            {
                "type": "adc",
                "instance": "ADC1",
                "channels": [0, 1, 4, 5],
                "dma": True,
                "continuous": True
            },
            {
                "type": "pid",
                "name": "motor_left",
                "kp": 5.0,
                "ki": 0.5,
                "kd": 0.1,
                "output_min": -999,
                "output_max": 999,
                "sample_time_ms": 10
            },
            {
                "type": "pid",
                "name": "motor_right",
                "kp": 5.0,
                "ki": 0.5,
                "kd": 0.1,
                "output_min": -999,
                "output_max": 999,
                "sample_time_ms": 10
            },
            {
                "type": "hbridge",
                "name": "motor_left",
                "pwm_timer": "TIM3",
                "pwm_channel": 1,
                "dir_port": "GPIOB",
                "dir_pin_a": 12,
                "dir_pin_b": 13
            },
            {
                "type": "hbridge",
                "name": "motor_right",
                "pwm_timer": "TIM3",
                "pwm_channel": 2,
                "dir_port": "GPIOB",
                "dir_pin_a": 14,
                "dir_pin_b": 15
            },
            {
                "type": "gpio",
                "port": "GPIOC",
                "pin": 13,
                "mode": "output_push_pull",
                "speed": "high"
            },
            {
                "type": "gpio",
                "port": "GPIOA",
                "pin": 8,
                "mode": "input_pullup",
                "speed": "medium"
            }
        ]
    }
    return config


# ============================================================
# 主函数
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description='代码生成器 - 电赛备赛',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  python code_generator.py --example                      生成示例配置
  python code_generator.py --config config.json           根据配置生成代码
  python code_generator.py --config config.json --output ./src/drivers
  python code_generator.py --list                         列出支持的外设类型
        """
    )
    parser.add_argument('--config', '-c', type=str, help='配置文件路径 (JSON)')
    parser.add_argument('--output', '-o', type=str, default='./generated',
                        help='输出目录 (默认: ./generated)')
    parser.add_argument('--example', '-e', action='store_true',
                        help='生成示例配置文件')
    parser.add_argument('--list', '-l', action='store_true',
                        help='列出支持的外设类型')

    args = parser.parse_args()

    if args.list:
        print("支持的外设类型:")
        print("  gpio     - GPIO 初始化与控制")
        print("  uart     - UART 串口通信")
        print("  pwm      - PWM 输出")
        print("  adc      - ADC 多通道采集")
        print("  encoder  - 编码器测速")
        print("  pid      - PID 控制器")
        print("  hbridge  - H桥电机驱动")
        return

    if args.example:
        config = generate_example_config()
        example_path = os.path.join(os.path.dirname(__file__), 'peripheral_config_example.json')
        with open(example_path, 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        print(f"[示例] 配置文件已生成: {example_path}")
        print(f"[示例] 使用方法: python code_generator.py --config {example_path}")
        return

    if not args.config:
        parser.print_help()
        print("\n请指定配置文件，或使用 --example 生成示例配置")
        return

    if not os.path.exists(args.config):
        print(f"[错误] 配置文件不存在: {args.config}")
        return

    with open(args.config, 'r', encoding='utf-8') as f:
        config = json.load(f)

    print("=" * 60)
    print(f"  代码生成器 - 电赛备赛工具")
    print(f"  项目: {config.get('project_name', 'N/A')}")
    print(f"  MCU:  {config.get('mcu', 'N/A')}")
    print(f"  外设: {len(config.get('peripherals', []))} 个")
    print(f"  输出: {args.output}")
    print("=" * 60)

    files = generate_files(config, args.output)

    print(f"\n[完成] 共生成 {len(files)} 个文件:")
    for f in files:
        print(f"  -> {f}")

    print(f"\n[提示] 请将生成的 .h 和 .c 文件添加到你的 Keil 工程中")
    print("[提示] 部分函数调用需要根据实际工程调整（已用注释标注）")


if __name__ == '__main__':
    main()
