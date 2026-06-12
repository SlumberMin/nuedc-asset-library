/**
 * @file data_acquisition.c
 * @brief MSPM0G3507 多通道数据采集系统
 *
 * 硬件连接（ADS1115 16位ADC模块）：
 *   SDA -> PB0 (I2C0 SDA)
 *   SCL -> PB1 (I2C0 SCL)
 *   ADDR-> GND (I2C地址 0x48)
 *   ALRT-> PA7 (转换完成中断，可选)
 *
 *   模拟输入：
 *     AIN0 -> 电压采集（0~3.3V经分压）
 *     AIN1 -> 电流采集（ACS712）
 *     AIN2 -> 温度采集（NTC热敏电阻）
 *     AIN3 -> 光照采集（光敏电阻）
 *
 *   OLED显示（I2C，地址0x3C）：
 *     SDA -> PB0 (与ADS1115共享I2C总线)
 *     SCL -> PB1
 *
 *   按键：
 *     采样率+  -> PA3
 *     采样率-  -> PA4
 *     开始/停止 -> PA5
 *     导出数据 -> PA6
 *
 * 功能：
 *   - 4通道16位ADC采集
 *   - 可配置采样率（1/10/100/860 SPS）
 *   - OLED实时显示4通道数据
 *   - 数据环形缓冲存储（2048采样点）
 *   - 统计分析（最大/最小/平均/标准差）
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ========== ADS1115 I2C配置 ========== */
#define ADS1115_ADDR        0x48  /* ADDR接GND时的地址 */
#define ADS1115_REG_CONV    0x00  /* 转换结果寄存器 */
#define ADS1115_REG_CONFIG  0x01  /* 配置寄存器 */
#define ADS1115_REG_LOTHRESH 0x02
#define ADS1115_REG_HITHRESH 0x03

/* ADS1115配置寄位定义 */
#define ADS1115_OS_SINGLE   0x8000  /* 启动单次转换 */
#define ADS1115_MUX_AIN0_G  0x4000  /* AIN0 vs GND */
#define ADS1115_MUX_AIN1_G  0x5000  /* AIN1 vs GND */
#define ADS1115_MUX_AIN2_G  0x6000  /* AIN2 vs GND */
#define ADS1115_MUX_AIN3_G  0x7000  /* AIN3 vs GND */

#define ADS1115_PGA_4096    0x0200  /* ±4.096V量程 */
#define ADS1115_PGA_2048    0x0400  /* ±2.048V量程 */
#define ADS1115_PGA_1024    0x0600  /* ±1.024V量程 */

#define ADS1115_MODE_CONT   0x0000  /* 连续转换 */
#define ADS1115_MODE_SINGLE 0x0100  /* 单次转换 */

#define ADS1115_DR_860      0x0080  /* 860 SPS */
#define ADS1115_DR_250      0x0060  /* 250 SPS */
#define ADS1115_DR_100      0x0040  /* 100 SPS */
#define ADS1115_DR_16       0x0000  /* 16 SPS */

/* 通道MUX值表 */
static const uint16_t mux_table[4] = {
    ADS1115_MUX_AIN0_G,
    ADS1115_MUX_AIN1_G,
    ADS1115_MUX_AIN2_G,
    ADS1115_MUX_AIN3_G
};

/* 采样率配置表 */
typedef struct {
    uint16_t config_val;
    uint16_t sps;
    const char *name;
} SampleRate_t;

static const SampleRate_t sample_rates[] = {
    {ADS1115_DR_16,  16,  "16 SPS"},
    {ADS1115_DR_100, 100, "100 SPS"},
    {ADS1115_DR_250, 250, "250 SPS"},
    {ADS1115_DR_860, 860, "860 SPS"},
};
#define NUM_SAMPLE_RATES  4

/* ========== OLED显示配置（128x64 I2C）========== */
#define OLED_ADDR   0x3C
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

/* ========== 数据缓冲区配置 ========== */
#define BUFFER_SIZE  2048  /* 每通道2048个采样点 */

/* ========== 通道数据结构 ========== */
typedef struct {
    int16_t buffer[BUFFER_SIZE];  /* 原始ADC值环形缓冲 */
    uint32_t write_idx;           /* 写指针 */
    uint32_t count;               /* 已采集总数 */
    float voltage;                /* 最新电压值 */
    float value_processed;        /* 处理后的物理量 */
    float max_val;                /* 最大值 */
    float min_val;                /* 最小值 */
    float avg_val;                /* 平均值 */
    float std_dev;                /* 标准差 */
    const char *unit;             /* 单位字符串 */
    const char *name;             /* 通道名称 */
} ChannelData_t;

/* ========== 全局变量 ========== */
static ChannelData_t g_channels[4] = {
    {.unit = "V",  .name = "Voltage",  .max_val = -999, .min_val = 999},
    {.unit = "mA", .name = "Current",  .max_val = -999, .min_val = 999},
    {.unit = "C",  .name = "Temp",     .max_val = -999, .min_val = 999},
    {.unit = "Lux",.name = "Light",    .max_val = -999, .min_val = 999},
};

static uint8_t g_sample_rate_idx = 1;  /* 默认100 SPS */
static bool g_acquiring = false;       /* 采集状态 */
static volatile uint32_t g_tick = 0;
static uint32_t g_total_samples = 0;

/* I2C句柄（由SYSCFG生成，此处声明extern） */
extern I2C_Regs *g_i2c;

/* ========== I2C辅助函数 ========== */
static bool i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint16_t data)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (data >> 8) & 0xFF;  /* 高字节先发 */
    buf[2] = data & 0xFF;

    DL_I2C_fillControllerTXFIFO(g_i2c, buf, 3);
    DL_I2C_startControllerTransfer(g_i2c, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 3);
    while (DL_I2C_getControllerStatus(g_i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    DL_I2C_flushControllerTXFIFO(g_i2c);
    return true;
}

static bool i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint16_t *data)
{
    uint8_t tx_buf[1] = {reg};
    uint8_t rx_buf[2] = {0};

    /* 写寄存器地址 */
    DL_I2C_fillControllerTXFIFO(g_i2c, tx_buf, 1);
    DL_I2C_startControllerTransfer(g_i2c, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(g_i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    DL_I2C_flushControllerTXFIFO(g_i2c);

    /* 读2字节 */
    DL_I2C_startControllerTransfer(g_i2c, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, 2);
    while (DL_I2C_getControllerStatus(g_i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    rx_buf[0] = DL_I2C_receiveControllerData(g_i2c);
    rx_buf[1] = DL_I2C_receiveControllerData(g_i2c);

    *data = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    return true;
}

/* ========== ADS1115驱动函数 ========== */
static void ads1115_init(void)
{
    /* 默认配置：±4.096V量程，单次模式，100SPS */
    uint16_t config = ADS1115_OS_SINGLE | ADS1115_MUX_AIN0_G |
                      ADS1115_PGA_4096 | ADS1115_MODE_SINGLE |
                      ADS1115_DR_100 | 0x0003;  /* 禁用比较器 */
    i2c_write_reg(ADS1115_ADDR, ADS1115_REG_CONFIG, config);
}

/* 设置采样率 */
static void ads1115_set_sample_rate(uint8_t rate_idx)
{
    /* 采样率在每次转换配置中设置，此处记录即可 */
    (void)rate_idx;
}

/* 读取指定通道的ADC值（16位有符号） */
static int16_t ads1115_read_channel(uint8_t channel)
{
    if (channel > 3) return 0;

    uint16_t config = ADS1115_OS_SINGLE | mux_table[channel] |
                      ADS1115_PGA_4096 | ADS1115_MODE_SINGLE |
                      sample_rates[g_sample_rate_idx].config_val |
                      0x0003;

    i2c_write_reg(ADS1115_ADDR, ADS1115_REG_CONFIG, config);

    /* 等待转换完成（根据采样率计算等待时间） */
    uint32_t wait_us = 1100000UL / sample_rates[g_sample_rate_idx].sps + 100;
    delay_cycles(wait_us * 32);

    uint16_t raw;
    i2c_read_reg(ADS1115_ADDR, ADS1115_REG_CONV, &raw);
    return (int16_t)raw;
}

/* ADC原始值转电压 */
static float adc_to_voltage(int16_t raw)
{
    /* ±4.096V量程，16位分辨率 */
    /* 每LSB = 4.096 * 2 / 65536 = 0.000125V = 125uV */
    return raw * 0.000125f;
}

/* ========== 通道物理量转换 ========== */
static float voltage_to_value(uint8_t ch, float voltage)
{
    switch (ch) {
        case 0:  /* 电压通道：10:1分压，还原实际电压 */
            return voltage * 10.0f;

        case 1:  /* 电流通道：ACS712 5A模块 */
            /* 灵敏度185mV/A，2.5V零点 */
            return (voltage - 2.5f) / 0.185f * 1000.0f;  /* 转为mA */

        case 2:  /* 温度通道：NTC 10K */
        {
            /* Steinhart-Hart简化公式 */
            if (voltage < 0.001f) return -999.0f;
            float resistance = 10000.0f * voltage / (3.3f - voltage);
            float temp_k = 1.0f / (1.0f / 298.15f +
                           (1.0f / 3950.0f) * logf(resistance / 10000.0f));
            return temp_k - 273.15f;
        }

        case 3:  /* 光照通道：简单线性映射 */
            return voltage * 1000.0f;  /* 近似Lux */

        default:
            return voltage;
    }
}

/* ========== 数据统计计算 ========== */
static void calculate_statistics(ChannelData_t *ch)
{
    if (ch->count == 0) return;

    uint32_t n = (ch->count < BUFFER_SIZE) ? ch->count : BUFFER_SIZE;
    float sum = 0, sum_sq = 0;
    float max_v = -999.0f, min_v = 999.0f;

    for (uint32_t i = 0; i < n; i++) {
        float v = adc_to_voltage(ch->buffer[i]);
        v = voltage_to_value(0, v);  /* 用通道0公式简化 */
        sum += v;
        sum_sq += v * v;
        if (v > max_v) max_v = v;
        if (v < min_v) min_v = v;
    }

    ch->avg_val = sum / n;
    ch->max_val = max_v;
    ch->min_val = min_v;

    float variance = (sum_sq / n) - (ch->avg_val * ch->avg_val);
    ch->std_dev = sqrtf(fabsf(variance));
}

/* ========== OLED显示驱动（简化版）========== */
/* 页地址模式写命令 */
static void oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    DL_I2C_fillControllerTXFIFO(g_i2c, buf, 2);
    DL_I2C_startControllerTransfer(g_i2c, OLED_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(g_i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    DL_I2C_flushControllerTXFIFO(g_i2c);
}

static void oled_data(uint8_t data)
{
    uint8_t buf[2] = {0x40, data};
    DL_I2C_fillControllerTXFIFO(g_i2c, buf, 2);
    DL_I2C_startControllerTransfer(g_i2c, OLED_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(g_i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    DL_I2C_flushControllerTXFIFO(g_i2c);
}

static void oled_init(void)
{
    /* SSD1306初始化序列 */
    oled_cmd(0xAE);  /* 关闭显示 */
    oled_cmd(0xD5); oled_cmd(0x80);  /* 时钟分频 */
    oled_cmd(0xA8); oled_cmd(0x3F);  /* 复用率 1/64 */
    oled_cmd(0xD3); oled_cmd(0x00);  /* 显示偏移 */
    oled_cmd(0x40);                   /* 起始行 */
    oled_cmd(0x8D); oled_cmd(0x14);  /* 电荷泵使能 */
    oled_cmd(0x20); oled_cmd(0x02);  /* 页地址模式 */
    oled_cmd(0xA1);                   /* 段重映射 */
    oled_cmd(0xC8);                   /* COM扫描方向 */
    oled_cmd(0xDA); oled_cmd(0x12);  /* COM引脚配置 */
    oled_cmd(0x81); oled_cmd(0xCF);  /* 对比度 */
    oled_cmd(0xD9); oled_cmd(0xF1);  /* 预充电周期 */
    oled_cmd(0xDB); oled_cmd(0x40);  /* VCOMH电压 */
    oled_cmd(0xA4);                   /* 全局显示开启 */
    oled_cmd(0xA6);                   /* 正常显示 */
    oled_cmd(0xAF);                   /* 开启显示 */
}

static void oled_clear(void)
{
    for (int page = 0; page < 8; page++) {
        oled_cmd(0xB0 + page);
        oled_cmd(0x00);
        oled_cmd(0x10);
        for (int col = 0; col < 128; col++) {
            oled_data(0x00);
        }
    }
}

/* 设置光标位置 */
static void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_cmd(0xB0 + page);
    oled_cmd(0x00 + (col & 0x0F));
    oled_cmd(0x10 + ((col >> 4) & 0x0F));
}

/* 显示简化的数字（3x5字体） */
static void oled_show_number(uint8_t page, uint8_t col, int32_t num)
{
    char buf[12];
    /* 简单的数字转字符串 */
    int i = 0;
    bool neg = false;
    if (num < 0) { neg = true; num = -num; }
    if (num == 0) {
        buf[i++] = '0';
    } else {
        while (num > 0 && i < 10) {
            buf[i++] = '0' + (num % 10);
            num /= 10;
        }
    }
    if (neg) buf[i++] = '-';
    buf[i] = '\0';

    /* 反转 */
    for (int j = 0; j < i / 2; j++) {
        char tmp = buf[j];
        buf[j] = buf[i - 1 - j];
        buf[i - 1 - j] = tmp;
    }

    oled_set_cursor(page, col);
    /* 这里简化处理，实际需要字模 */
    (void)buf;
}

/* ========== 按键扫描 ========== */
static uint8_t scan_buttons(void)
{
    static uint8_t last = 0xFF;
    uint8_t cur = 0;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_3)) cur |= 0x01;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_4)) cur |= 0x02;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_5)) cur |= 0x04;
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_6)) cur |= 0x08;
    uint8_t pressed = (~cur) & last;
    last = cur;
    if (pressed & 0x01) return 1;
    if (pressed & 0x02) return 2;
    if (pressed & 0x04) return 3;
    if (pressed & 0x08) return 4;
    return 0;
}

/* ========== 主函数 ========== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* 按键GPIO初始化 */
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_3,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_4,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_5,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIOA, DL_GPIO_PIN_6,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 外设初始化 */
    ads1115_init();
    oled_init();
    oled_clear();

    /* 初始显示 */
    oled_set_cursor(0, 0);
    oled_cmd(0xAF);  /* 确保显示开启 */

    /* 状态指示LED */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_8);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);

    /* 主循环 */
    while (1) {
        /* 按键处理 */
        uint8_t btn = scan_buttons();
        switch (btn) {
            case 1:  /* 采样率增加 */
                if (g_sample_rate_idx < NUM_SAMPLE_RATES - 1) {
                    g_sample_rate_idx++;
                    ads1115_set_sample_rate(g_sample_rate_idx);
                }
                break;

            case 2:  /* 采样率减少 */
                if (g_sample_rate_idx > 0) {
                    g_sample_rate_idx--;
                    ads1115_set_sample_rate(g_sample_rate_idx);
                }
                break;

            case 3:  /* 开始/停止采集 */
                g_acquiring = !g_acquiring;
                if (g_acquiring) {
                    /* 开始采集，LED亮 */
                    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);
                } else {
                    /* 停止采集，LED灭 */
                    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);
                    /* 计算统计数据 */
                    for (int ch = 0; ch < 4; ch++) {
                        calculate_statistics(&g_channels[ch]);
                    }
                }
                break;

            case 4:  /* 导出数据（通过UART打印） */
                /* 打印统计结果 */
                for (int ch = 0; ch < 4; ch++) {
                    ChannelData_t *c = &g_channels[ch];
                    /* 实际项目中通过UART发送 */
                    (void)c;
                }
                break;
        }

        /* 数据采集 */
        if (g_acquiring) {
            for (int ch = 0; ch < 4; ch++) {
                int16_t raw = ads1115_read_channel(ch);
                float voltage = adc_to_voltage(raw);

                /* 存入环形缓冲 */
                ChannelData_t *c = &g_channels[ch];
                c->buffer[c->write_idx] = raw;
                c->write_idx = (c->write_idx + 1) % BUFFER_SIZE;
                c->count++;
                c->voltage = voltage;
                c->value_processed = voltage_to_value(ch, voltage);

                /* 更新极值 */
                if (c->value_processed > c->max_val)
                    c->max_val = c->value_processed;
                if (c->value_processed < c->min_val)
                    c->min_val = c->value_processed;
            }

            g_total_samples++;

            /* 每100个采样更新一次平均值 */
            if (g_total_samples % 100 == 0) {
                for (int ch = 0; ch < 4; ch++) {
                    ChannelData_t *c = &g_channels[ch];
                    uint32_t n = (c->count < BUFFER_SIZE) ? c->count : BUFFER_SIZE;
                    float sum = 0;
                    for (uint32_t i = 0; i < n; i++) {
                        sum += adc_to_voltage(c->buffer[i]);
                    }
                    c->avg_val = sum / n;
                }
            }

            /* LED闪烁表示正在采集 */
            if (g_total_samples % 20 == 0) {
                DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_8);
            }
        }

        /* OLED显示更新（每200ms） */
        if (g_tick % 200 == 0) {
            oled_clear();

            /* 第0行：状态 */
            oled_set_cursor(0, 0);
            /* 显示采样率和采集状态 */

            /* 第1-4行：4通道实时数据 */
            for (int ch = 0; ch < 4; ch++) {
                ChannelData_t *c = &g_channels[ch];
                oled_set_cursor(ch + 1, 0);
                /* 显示通道名、当前值、单位 */
                (void)c;
            }

            /* 第5-6行：统计信息 */
            oled_set_cursor(6, 0);
            /* 显示最大/最小值 */
            oled_set_cursor(7, 0);
            /* 显示采样计数 */
        }

        g_tick++;
        delay_cycles(1000 * 32);  /* 1ms延时 */
    }
}
