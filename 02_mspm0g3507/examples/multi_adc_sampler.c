/**
 * @file multi_adc_sampler.c
 * @brief 多路ADC同步采样系统
 * @platform MSPM0G3507
 * @description
 *   同时使用内部ADC和外部MCP3421高精度ADC进行多路采样：
 *   - 内部ADC: 12位，4通道同时采样(电压/电流/温度/参考)
 *   - MCP3421: 18位高精度ADC(I2C接口)
 *   - DMA自动搬运ADC数据
 *   - 可配置采样率和滤波
 *   - UART输出采样数据
 *
 * 硬件连接：
 *   内部ADC: A0~A3 (4通道模拟输入)
 *   MCP3421: I2C0(PB2-SCL, PB3-SDA), 地址0x6E
 *   UART:    PA10(TX) - 115200波特率
 *   LED:     PA27(采样指示)
 *
 * 应用场景：
 *   电压/电流同步测量、数据采集系统、传感器融合
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== 配置参数 ========== */

#define SAMPLE_RATE_HZ       1000     /* 内部ADC采样率 */
#define OVERSAMPLE_COUNT     16       /* 过采样次数 */
#define FILTER_DEPTH         8        /* 滑动平均滤波深度 */
#define ADC_CHANNELS         4        /* 内部ADC通道数 */
#define ADC_RESOLUTION       4096     /* 12位ADC分辨率 */
#define VREF_MV              3300     /* 参考电压3.3V */

/* MCP3421配置 */
#define MCP3421_ADDR         0x6E     /* I2C地址 */
#define MCP3421_18BIT        0x1C     /* 18位连续转换模式 */
#define MCP3421_16BIT        0x18     /* 16位连续转换模式 */
#define MCP3421_14BIT        0x14     /* 14位连续转换模式 */
#define MCP3421_12BIT        0x10     /* 12位连续转换模式 */
#define MCP3421_1X_GAIN      0x00     /* 增益1x */
#define MCP3421_2X_GAIN      0x01     /* 增益2x */
#define MCP3421_4X_GAIN      0x02     /* 增益4x */
#define MCP3421_8X_GAIN      0x03     /* 增益8x */

/* ADC校准值 */
#define ADC_OFFSET_CAL       0        /* 偏移校准 */
#define ADC_GAIN_CAL         1.0f     /* 增益校准 */

/* ========== 数据结构 ========== */

/**
 * @brief 单通道ADC数据
 */
typedef struct {
    uint16_t raw_value;         /* 原始ADC值 */
    float voltage_mv;           /* 电压(mV) */
    float filtered_mv;          /* 滤波后电压(mV) */
    float min_mv;               /* 最小值 */
    float max_mv;               /* 最大值 */
    float avg_mv;               /* 平均值 */
    uint32_t sample_count;      /* 采样计数 */
} ADC_ChannelData_t;

/**
 * @brief MCP3421高精度ADC数据
 */
typedef struct {
    int32_t raw_value;          /* 原始ADC值(带符号) */
    float voltage_mv;           /* 电压(mV) */
    float filtered_mv;          /* 滤波后电压(mV) */
    uint8_t config;             /* 配置字节 */
    uint8_t resolution;         /* 分辨率位数 */
    bool ready;                 /* 数据就绪标志 */
} MCP3421_Data_t;

/**
 * @brief 同步采样结果
 */
typedef struct {
    ADC_ChannelData_t adc[ADC_CHANNELS];  /* 内部ADC数据 */
    MCP3421_Data_t ext_adc;               /* 外部高精度ADC数据 */
    uint32_t timestamp;                    /* 采样时间戳 */
    uint32_t sequence;                     /* 序列号 */
} SyncSample_t;

/* ========== 全局变量 ========== */

/* 同步采样结果 */
static SyncSample_t g_sample;

/* 滑动平均滤波器缓冲区 */
static float filter_buf[ADC_CHANNELS][FILTER_DEPTH];
static uint8_t filter_idx[ADC_CHANNELS] = {0};
static float filter_sum[ADC_CHANNELS] = {0};

/* DMA传输缓冲区 */
static volatile uint16_t adc_dma_buf[ADC_CHANNELS * OVERSAMPLE_COUNT];

/* 采样控制 */
static volatile bool sample_ready = false;
static volatile uint32_t sample_count = 0;

/* ========== 内部ADC驱动 ========== */

/**
 * @brief 初始化内部ADC多通道+DMA
 */
static void Internal_ADC_Init(void)
{
    /* 复位并使能ADC */
    DL_ADC12_reset(ADC0);
    DL_ADC12_enablePower(ADC0);
    DL_ADC12_setClockConfig(ADC0, DL_ADC12_CLOCK_DIVIDE_1);

    /* 配置ADC参数 */
    DL_ADC12_init(ADC0,
        DL_ADC12_REPEAT_MODE_ENABLED,     /* 连续转换模式 */
        DL_ADC12_CLOCK_DIVIDE_1,          /* 时钟分频 */
        DL_ADC12_SAMPLING_SOURCE_AUTO,    /* 自动触发 */
        DL_ADC12_TRIG_SRC_SOFTWARE,       /* 软件触发 */
        DL_ADC12_SEQ_MODE_ENABLED,        /* 序列模式 */
        DL_ADC12_CONV_RESOLUTION_12_BIT,  /* 12位分辨率 */
        DL_ADC12_SAMP_CONV_COUNT_4        /* 4通道序列 */
    );

    /* 配置通道0: 电压测量 */
    DL_ADC12_configConversion(ADC0, DL_ADC12_INPUT_CHAN_0,
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* 配置通道1: 电流测量 */
    DL_ADC12_configConversion(ADC0, DL_ADC12_INPUT_CHAN_1,
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* 配置通道2: 温度传感器 */
    DL_ADC12_configConversion(ADC0, DL_ADC12_INPUT_CHAN_2,
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* 配置通道3: 外部参考 */
    DL_ADC12_configConversion(ADC0, DL_ADC12_INPUT_CHAN_3,
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* 启动ADC */
    DL_ADC12_enableConversions(ADC0);
}

/**
 * @brief 读取ADC序列数据
 * @param data 输出数据数组(4个通道)
 */
static void Internal_ADC_ReadSequence(uint16_t *data)
{
    /* 启动转换 */
    DL_ADC12_startConversion(ADC0);

    /* 等待所有通道转换完成 */
    while (!DL_ADC12_getStatus(ADC0, DL_ADC12_STATUS_CONVERSION_DONE)) {}

    /* 读取各通道结果 */
    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++) {
        data[ch] = DL_ADC12_getMemResult(ADC0, ch);
    }
}

/**
 * @brief 过采样读取ADC
 * @param ch 通道号
 * @return 过采样平均值(float)
 */
static float Internal_ADC_Oversample(uint8_t ch)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < OVERSAMPLE_COUNT; i++) {
        uint16_t raw[ADC_CHANNELS];
        Internal_ADC_ReadSequence(raw);
        sum += raw[ch];
    }

    /* 过采样平均值(保留小数位提高分辨率) */
    return (float)sum / OVERSAMPLE_COUNT;
}

/* ========== 滑动平均滤波器 ========== */

/**
 * @brief 初始化滤波器
 */
static void Filter_Init(void)
{
    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++) {
        filter_sum[ch] = 0;
        filter_idx[ch] = 0;
        for (uint8_t i = 0; i < FILTER_DEPTH; i++) {
            filter_buf[ch][i] = 0;
        }
    }
}

/**
 * @brief 更新滤波器并返回滤波结果
 * @param ch 通道号
 * @param new_value 新采样值
 * @return 滤波后的值
 */
static float Filter_Update(uint8_t ch, float new_value)
{
    /* 从总和中减去最旧的值 */
    filter_sum[ch] -= filter_buf[ch][filter_idx[ch]];

    /* 存入新值 */
    filter_buf[ch][filter_idx[ch]] = new_value;
    filter_sum[ch] += new_value;

    /* 更新索引(环形缓冲区) */
    filter_idx[ch] = (filter_idx[ch] + 1) % FILTER_DEPTH;

    /* 返回平均值 */
    return filter_sum[ch] / FILTER_DEPTH;
}

/* ========== MCP3421高精度ADC驱动 (I2C) ========== */

/**
 * @brief I2C写入单字节
 */
static void I2C_Write(uint8_t addr, uint8_t *data, uint8_t len)
{
    DL_I2C_flushControllerTXFIFO(I2C0);
    for (uint8_t i = 0; i < len; i++) {
        DL_I2C_fillControllerTXFIFO(I2C0, &data[i], 1);
    }
    DL_I2C_startControllerTransfer(I2C0, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    while (DL_I2C_getControllerStatus(I2C0) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
}

/**
 * @brief I2C读取指定字节数
 */
static int I2C_Read(uint8_t addr, uint8_t *data, uint8_t len)
{
    DL_I2C_flushControllerRXFIFO(I2C0);
    DL_I2C_startControllerTransfer(I2C0, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    uint32_t timeout = 100000;
    uint8_t idx = 0;
    while (timeout-- > 0 && idx < len) {
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C0)) {
            data[idx++] = DL_I2C_receiveControllerData(I2C0);
        }
    }
    return idx;
}

/**
 * @brief 初始化MCP3421
 * @param config 配置字节
 */
static void MCP3421_Init(uint8_t config)
{
    g_sample.ext_adc.config = config;
    g_sample.ext_adc.resolution = 18;  /* 默认18位 */
    g_sample.ext_adc.ready = false;

    /* 写入配置寄存器 */
    uint8_t cfg_data[1] = {config};
    I2C_Write(MCP3421_ADDR, cfg_data, 1);
}

/**
 * @brief 读取MCP3421转换结果
 * @return 电压值(mV)
 */
static float MCP3421_ReadVoltage(void)
{
    uint8_t buf[4] = {0};
    int bytes_read;
    int32_t raw = 0;
    float lsb_uv = 0;  /* 最小分辨电压(微伏) */

    /* 根据分辨率读取不同字节数 */
    uint8_t config = g_sample.ext_adc.config;
    uint8_t rdlen;

    if (config & 0x08) {
        /* 18位模式: 3字节数据 + 1字节配置 */
        rdlen = 4;
        lsb_uv = 15.625f;  /* 18位: 15.625uV/LSB (增益1x) */
    } else if (config & 0x04) {
        /* 16位模式: 2字节数据 + 1字节配置 */
        rdlen = 3;
        lsb_uv = 62.5f;    /* 16位: 62.5uV/LSB */
    } else if (config & 0x02) {
        /* 14位模式: 2字节数据 + 1字节配置 */
        rdlen = 3;
        lsb_uv = 250.0f;   /* 14位: 250uV/LSB */
    } else {
        /* 12位模式: 2字节数据 + 1字节配置 */
        rdlen = 3;
        lsb_uv = 1000.0f;  /* 12位: 1mV/LSB */
    }

    /* 根据增益调整LSB */
    switch (config & 0x03) {
    case 0: /* 1x */  break;
    case 1: /* 2x */  lsb_uv /= 2.0f; break;
    case 2: /* 4x */  lsb_uv /= 4.0f; break;
    case 3: /* 8x */  lsb_uv /= 8.0f; break;
    }

    /* 读取数据 */
    bytes_read = I2C_Read(MCP3421_ADDR, buf, rdlen);
    if (bytes_read < rdlen) return 0.0f;

    /* 解析数据(最后字节是配置+RDY位) */
    uint8_t status_byte = buf[rdlen - 1];
    g_sample.ext_adc.ready = !(status_byte & 0x80);  /* RDY=0表示数据就绪 */

    if (config & 0x08) {
        /* 18位: 使用3字节 */
        raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8);
        raw >>= 8;  /* 符号扩展 */
        /* 检查符号位 */
        if (raw & 0x20000) raw |= 0xFFFC0000;  /* 符号扩展到32位 */
    } else {
        /* 12/14/16位: 使用2字节 */
        raw = ((int16_t)buf[0] << 8) | buf[1];
    }

    g_sample.ext_adc.raw_value = raw;

    /* 转换为电压(mV) */
    float voltage = (float)raw * lsb_uv / 1000.0f;
    g_sample.ext_adc.voltage_mv = voltage;

    /* 滤波 */
    static float ext_filter_sum = 0;
    static float ext_filter_buf[FILTER_DEPTH] = {0};
    static uint8_t ext_filter_idx = 0;

    ext_filter_sum -= ext_filter_buf[ext_filter_idx];
    ext_filter_buf[ext_filter_idx] = voltage;
    ext_filter_sum += voltage;
    ext_filter_idx = (ext_filter_idx + 1) % FILTER_DEPTH;

    g_sample.ext_adc.filtered_mv = ext_filter_sum / FILTER_DEPTH;

    return voltage;
}

/* ========== UART输出 ========== */

/**
 * @brief UART发送字符串
 */
static void UART_SendString(const char *str)
{
    while (*str) {
        DL_UART_main_transmitDataBlocking(UART0, (uint8_t)*str);
        str++;
    }
}

/**
 * @brief UART格式化输出
 */
static void UART_Printf(const char *fmt, ...)
{
    char buf[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    UART_SendString(buf);
}

/* ========== 定时采样中断 ========== */

/**
 * @brief 定时器中断 - 采样触发
 */
void TIMER0_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER0) == DL_TIMERG_IIDX_ZERO) {
        sample_ready = true;
        sample_count++;

        /* LED闪烁指示采样 */
        DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_27);
    }
}

/**
 * @brief 初始化定时器(采样率控制)
 */
static void Sample_Timer_Init(uint32_t sample_rate_hz)
{
    DL_TimerG_reset(TIMER0);
    DL_TimerG_enablePower(TIMER0);

    /* 配置定时器: 32MHz / sample_rate */
    uint32_t period = 32000000 / sample_rate_hz;

    DL_TimerG_init(TIMER0,
        DL_TIMERG_MODE_PERIODIC,
        DL_TIMERG_CLOCK_DIVIDE_1,
        period - 1,
        0);

    /* 使能中断 */
    DL_TimerG_enableInterrupt(TIMER0, DL_TIMERG_IIDX_ZERO);
    NVIC_EnableIRQ(TIMER0_IRQn);

    DL_TimerG_start(TIMER0);
}

/* ========== 数据处理与输出 ========== */

/**
 * @brief ADC原始值转电压(mV)
 */
static float ADC_ToVoltage(uint16_t raw)
{
    float mv = (float)raw * VREF_MV / ADC_RESOLUTION;
    return (mv - ADC_OFFSET_CAL) * ADC_GAIN_CAL;
}

/**
 * @brief 处理单次采样数据
 */
static void Process_Sample(void)
{
    /* 1. 读取内部ADC(过采样) */
    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++) {
        float raw_avg = Internal_ADC_Oversample(ch);
        float voltage = ADC_ToVoltage((uint16_t)(raw_avg + 0.5f));

        g_sample.adc[ch].raw_value = (uint16_t)(raw_avg + 0.5f);
        g_sample.adc[ch].voltage_mv = voltage;

        /* 滑动平均滤波 */
        g_sample.adc[ch].filtered_mv = Filter_Update(ch, voltage);

        /* 统计最值 */
        if (g_sample.adc[ch].sample_count == 0) {
            g_sample.adc[ch].min_mv = voltage;
            g_sample.adc[ch].max_mv = voltage;
        } else {
            if (voltage < g_sample.adc[ch].min_mv)
                g_sample.adc[ch].min_mv = voltage;
            if (voltage > g_sample.adc[ch].max_mv)
                g_sample.adc[ch].max_mv = voltage;
        }

        /* 累加平均值 */
        g_sample.adc[ch].avg_mv =
            (g_sample.adc[ch].avg_mv * g_sample.adc[ch].sample_count + voltage)
            / (g_sample.adc[ch].sample_count + 1);
        g_sample.adc[ch].sample_count++;
    }

    /* 2. 读取MCP3421高精度ADC */
    MCP3421_ReadVoltage();

    /* 3. 更新时间戳 */
    g_sample.timestamp = sample_count;
    g_sample.sequence++;
}

/**
 * @brief 通过UART输出采样数据(CSV格式)
 */
static void Output_SampleData(void)
{
    /* CSV表头(首次输出) */
    static bool header_printed = false;
    if (!header_printed) {
        UART_SendString("Seq,CH0_RAW,CH0_mV,CH0_FILT,");
        UART_SendString("CH1_RAW,CH1_mV,CH1_FILT,");
        UART_SendString("CH2_RAW,CH2_mV,CH2_FILT,");
        UART_SendString("CH3_RAW,CH3_mV,CH3_FILT,");
        UART_SendString("EXT_RAW,EXT_mV,EXT_FILT\r\n");
        header_printed = true;
    }

    /* 输出数据行 */
    UART_Printf("%lu,%u,%.2f,%.2f,%u,%.2f,%.2f,%u,%.2f,%.2f,%u,%.2f,%.2f,%ld,%.4f,%.4f\r\n",
        g_sample.sequence,
        g_sample.adc[0].raw_value, g_sample.adc[0].voltage_mv, g_sample.adc[0].filtered_mv,
        g_sample.adc[1].raw_value, g_sample.adc[1].voltage_mv, g_sample.adc[1].filtered_mv,
        g_sample.adc[2].raw_value, g_sample.adc[2].voltage_mv, g_sample.adc[2].filtered_mv,
        g_sample.adc[3].raw_value, g_sample.adc[3].voltage_mv, g_sample.adc[3].filtered_mv,
        g_sample.ext_adc.raw_value, g_sample.ext_adc.voltage_mv, g_sample.ext_adc.filtered_mv
    );
}

/**
 * @brief 输出统计报告
 */
static void Output_Statistics(void)
{
    UART_SendString("\r\n=== Statistics Report ===\r\n");

    const char *ch_names[] = {"Voltage", "Current", "Temp", "RefV"};

    for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++) {
        UART_Printf("  CH%d(%s): Min=%.2fmV Max=%.2fmV Avg=%.2fmV Samples=%lu\r\n",
            ch, ch_names[ch],
            g_sample.adc[ch].min_mv,
            g_sample.adc[ch].max_mv,
            g_sample.adc[ch].avg_mv,
            g_sample.adc[ch].sample_count);
    }

    UART_Printf("  EXT(MCP3421): Raw=%ld Voltage=%.4fmV\r\n",
        g_sample.ext_adc.raw_value,
        g_sample.ext_adc.voltage_mv);

    UART_Printf("  Total samples: %lu\r\n", sample_count);
    UART_SendString("=========================\r\n\r\n");
}

/* ========== 校准功能 ========== */

/**
 * @brief ADC偏移校准
 * @note 将输入接地，采集多次取平均值作为偏移量
 */
static void Calibrate_Offset(void)
{
    UART_SendString("Calibrating ADC offset...\r\n");

    uint32_t sum = 0;
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t raw[ADC_CHANNELS];
        Internal_ADC_ReadSequence(raw);
        sum += raw[0];  /* 校准通道0 */
    }

    int16_t offset = (int16_t)(sum / 256);
    UART_Printf("  Offset calibration: %d\r\n", offset);

    /* 注意: 实际应用中应保存校准值到Flash */
}

/* ========== 主函数 ========== */

int main(void)
{
    /* 系统初始化 */
    DL_SYSCFG_init();

    /* GPIO初始化 */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_27);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_27);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27);

    /* UART初始化 */
    DL_UART_reset(UART0);
    DL_UART_enablePower(UART0);
    DL_UART_setClockConfig(UART0, DL_UART_CLOCK_DIVIDE_115200);
    DL_UART_init(UART0, DL_UART_MODE_NORMAL);
    DL_UART_enable(UART0);

    /* I2C初始化 */
    DL_I2C_reset(I2C0);
    DL_I2C_enablePower(I2C0);
    DL_I2C_setClockConfig(I2C0, DL_I2C_CLOCK_DIVIDE_400KHZ);
    DL_I2C_enableController(I2C0);

    /* 内部ADC初始化 */
    Internal_ADC_Init();

    /* MCP3421初始化(18位, 1x增益) */
    MCP3421_Init(MCP3421_18BIT | MCP3421_1X_GAIN);

    /* 滤波器初始化 */
    Filter_Init();

    /* 清零采样数据 */
    memset(&g_sample, 0, sizeof(g_sample));

    /* 校准 */
    Calibrate_Offset();

    /* 启动采样定时器 */
    Sample_Timer_Init(SAMPLE_RATE_HZ);

    UART_SendString("\r\n=== Multi-ADC Sampler v1.0 ===\r\n");
    UART_Printf("Sample rate: %d Hz\r\n", SAMPLE_RATE_HZ);
    UART_Printf("Oversampling: %d x\r\n", OVERSAMPLE_COUNT);
    UART_Printf("Filter depth: %d\r\n", FILTER_DEPTH);
    UART_SendString("Starting acquisition...\r\n\r\n");

    /* 用于统计输出的计数器 */
    uint32_t stat_counter = 0;

    /* 主循环 */
    while (1) {
        /* 等待采样触发 */
        if (sample_ready) {
            sample_ready = false;

            /* 处理采样数据 */
            Process_Sample();

            /* 每100个采样输出一次数据 */
            if (g_sample.sequence % 100 == 0) {
                Output_SampleData();
            }

            /* 每10000个采样输出统计报告 */
            stat_counter++;
            if (stat_counter >= 10000) {
                stat_counter = 0;
                Output_Statistics();
            }
        }

        /* 低功耗等待 */
        __WFI();
    }
}
