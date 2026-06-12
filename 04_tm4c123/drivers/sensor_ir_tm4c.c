/**
 * @file sensor_ir_tm4c.c
 * @brief TM4C123 红外循迹传感器实现
 */
#include "platform/tivaware.h"
#include "drivers/sensor_ir_tm4c.h"

/* ======================== 内部变量 ======================== */
static const uint8_t ir_adc_channels[IR_SENSOR_COUNT] = IR_ADC_CHANNELS;

/* 校准数据 */
static uint16_t cal_min[IR_SENSOR_COUNT];
static uint16_t cal_max[IR_SENSOR_COUNT];
static bool     calibrated = false;

/* 缓存最新数据 */
static ir_data_t latest_data;

/* ======================== 实现 ======================== */

void ir_sensor_init(void)
{
    /* 使能外设 */
    PERIPH_ENABLE(SYSCTL_PERIPH_ADC0);
    PERIPH_ENABLE(SYSCTL_PERIPH_GPIOE);
    PERIPH_ENABLE(SYSCTL_PERIPH_GPIOD);
    periph_wait_ready(SYSCTL_PERIPH_ADC0);
    periph_wait_ready(SYSCTL_PERIPH_GPIOE);
    periph_wait_ready(SYSCTL_PERIPH_GPIOD);

    /* 配置GPIO为模拟输入 */
    /* PE0=AIN3, PE1=AIN2, PE2=AIN1, PE3=AIN0 */
    MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
                                         GPIO_PIN_2 | GPIO_PIN_3);
    /* PD0=AIN7, PD1=AIN6, PD2=AIN5, PD3=AIN4 */
    MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
                                         GPIO_PIN_2 | GPIO_PIN_3);

    /* 配置ADC序列3: 8通道扫描, 中断触发 */
    MAP_ADCSequenceDisable(IR_ADC_BASE, IR_ADC_SEQUENCE);

    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        uint32_t ctl = ir_adc_channels[i] | ADC_CTL_IE;
        if (i == IR_SENSOR_COUNT - 1) {
            ctl |= ADC_CTL_END;  /* 最后一个采样结束 */
        }
        MAP_ADCSequenceStepConfigure(IR_ADC_BASE, IR_ADC_SEQUENCE, i, ctl);
    }

    MAP_ADCSequenceEnable(IR_ADC_BASE, IR_ADC_SEQUENCE);
    MAP_ADCIntClear(IR_ADC_BASE, IR_ADC_SEQUENCE);

    /* 校准默认值 */
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        cal_min[i] = 0;
        cal_max[i] = 4095;
    }
}

static bool ir_sensor_is_calibrated(void)
{
    return calibrated;
}

uint16_t ir_sensor_read_single(uint8_t index)
{
    if (index >= IR_SENSOR_COUNT) return 0;
    /* 使用ADC0 SS0进行单通道采样 */
    MAP_ADCSequenceDisable(ADC0_BASE, 0);
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 0, 0,
                                 ir_adc_channels[index] | ADC_CTL_END);
    MAP_ADCSequenceEnable(ADC0_BASE, 0);

    uint32_t val = adc_read_blocking(ADC0_BASE, 0);
    return (uint16_t)val;
}

void ir_sensor_read(ir_data_t *data)
{
    /* 读取所有通道 */
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        data->raw[i] = ir_sensor_read_single(i);
    }

    /* 归一化 */
    data->active_mask = 0;
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        int32_t range = (int32_t)cal_max[i] - (int32_t)cal_min[i];
        if (range <= 0) range = 1;
        int32_t val = (int32_t)data->raw[i] - (int32_t)cal_min[i];
        data->norm[i] = CLAMP((float)val / (float)range, 0.0f, 1.0f);

        if (data->norm[i] > 0.3f) {
            data->active_mask |= (1 << i);
        }
    }

    /* 计算加权位置 */
    float num = 0.0f, den = 0.0f;
    /* 权重: -3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5 */
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        float weight = (float)(i - (IR_SENSOR_COUNT - 1) / 2) - 0.5f;
        /* 映射到 -3500 ~ +3500 */
        weight *= 1000.0f;
        num += weight * data->norm[i];
        den += data->norm[i];
    }

    if (den > 0.01f) {
        data->position = (int16_t)(num / den);
    } else {
        data->position = 0;
    }

    data->position = CLAMP(data->position, IR_POS_LEFT_MAX, IR_POS_RIGHT_MAX);

    /* 缓存 */
    latest_data = *data;
}

int16_t ir_sensor_get_position(void)
{
    return latest_data.position;
}

void ir_sensor_calibrate(uint16_t samples)
{
    /* 初始化极值 */
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        cal_min[i] = 4095;
        cal_max[i] = 0;
    }

    /* 采集多组样本 */
    for (uint16_t s = 0; s < samples; s++) {
        for (int i = 0; i < IR_SENSOR_COUNT; i++) {
            uint16_t val = ir_sensor_read_single(i);
            if (val < cal_min[i]) cal_min[i] = val;
            if (val > cal_max[i]) cal_max[i] = val;
        }
        /* 简单延时 */
        for (volatile uint32_t d = 0; d < 100000; d++) {}
    }

    calibrated = true;
}

bool ir_sensor_line_lost(const ir_data_t *data)
{
    if (data->active_mask == 0) return true;        /* 全白 */
    if (data->active_mask == 0xFF) return true;     /* 全黑 */
    return false;
}
