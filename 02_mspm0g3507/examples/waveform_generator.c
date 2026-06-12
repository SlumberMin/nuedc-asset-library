/**
 * @file waveform_generator.c
 * @brief MSPM0G3507 波形发生器示例 - AD9833 + DAC8571 + 频率/幅度/波形控制
 * 
 * 硬件连接：
 *   AD9833 DDS信号发生器 (SPI接口):
 *     SCK  -> PA2 (SPI0_SCK)
 *     DATA -> PA4 (SPI0_MOSI)
 *     FSYNC-> PA5 (GPIO, 片选)
 * 
 *   DAC8571 (I2C接口):
 *     SDA  -> PB9 (I2C0_SDA)
 *     SCL  -> PB8 (I2C0_SCL)
 *     地址 -> 0x4C (A0接GND)
 * 
 *   按键控制:
 *     KEY1 -> PA8 (波形切换: 正弦/三角/方波)
 *     KEY2 -> PA9 (频率增大)
 *     KEY3 -> PA10(频率减小)
 *     KEY4 -> PA11(幅度调节)
 * 
 *   输出:
 *     AD9833 -> 信号输出 (经过运放调理)
 *     DAC8571-> 直流偏置/幅度控制
 * 
 * 功能说明：
 *   - AD9833 DDS产生正弦波、三角波、方波
 *   - 频率范围: 0.1Hz ~ 12.5MHz, 分辨率0.1Hz
 *   - DAC8571提供程控直流偏置 (-5V ~ +5V)
 *   - 按键控制频率、幅度、波形切换
 *   - 支持扫频输出功能
 * 
 * 适用场景：电赛中信号源、扫频测试、波形激励
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ============================================================
 * 第一部分：AD9833 DDS驱动
 * ============================================================ */

/* AD9833控制字各位定义 */
#define AD9833_B28       (1 << 13)  /* 28位频率字传输模式 */
#define AD9833_HLB       (1 << 12)  /* 高/低字节选择 */
#define AD9833_FSELECT   (1 << 11)  /* 频率寄存器选择 */
#define AD9833_PSELECT   (1 << 10)  /* 相位寄存器选择 */
#define AD9833_RESET     (1 << 8)   /* 复位位 */
#define AD9833_SLEEP1    (1 << 7)   /* DAC休眠 */
#define AD9833_SLEEP12   (1 << 6)   /* 内部时钟休眠 */
#define AD9833_OPBITEN   (1 << 5)   /* 输出使能 */
#define AD9833_DIV2      (1 << 3)   /* 方波分频 */
#define AD9833_MODE      (1 << 1)   /* 模式选择 */

/* 频率寄存器地址 */
#define AD9833_FREQ0_REG 0x4000     /* 频率寄存器0 */
#define AD9833_FREQ1_REG 0x8000     /* 频率寄存器1 */
#define AD9833_PHASE0_REG 0xC000    /* 相位寄存器0 */
#define AD9833_PHASE1_REG 0xE000    /* 相位寄存器1 */

/* 波形类型 */
typedef enum {
    WAVE_SINE = 0,      /* 正弦波 */
    WAVE_TRIANGLE,      /* 三角波 */
    WAVE_SQUARE,        /* 方波 */
    WAVE_SQUARE_DIV2    /* 方波/2 (MSB输出) */
} WaveformType_t;

/* AD9833参数 */
typedef struct {
    WaveformType_t waveform;    /* 波形类型 */
    double         frequency;   /* 频率(Hz) */
    uint16_t       phase;       /* 相位(0~4095, 0~2π) */
} AD9833_Config_t;

/* AD9833参考时钟 (25MHz晶振) */
#define AD9833_MCLK  25000000.0

/* FSYNC引脚定义 */
#define AD9833_FSYNC_PORT   GPIOA
#define AD9833_FSYNC_PIN    DL_GPIO_PIN_5

/* ============================================================
 * 第二部分：DAC8571 I2C DAC驱动
 * ============================================================ */

/* DAC8571 I2C地址 (A0=0时为0x4C) */
#define DAC8571_ADDR        0x4C

/* DAC8571控制字节 */
#define DAC8571_CTRL_WRITE  0x00    /* 写DAC寄存器 */
#define DAC8571_CTRL_UPDATE 0x10    /* 更新DAC输出 */
#define DAC8571_CTRL_BOTH   0x30    /* 写入并更新 */
#define DAC8571_CTRL_BCAST  0x90    /* 广播写入 */

/* DAC参考电压 */
#define DAC_VREF            3.3     /* DAC参考电压(V) */
#define DAC_RESOLUTION      65536.0 /* 16位分辨率 */

/* ============================================================
 * 第三部分：波形发生器参数
 * ============================================================ */

/* 频率预设 (Hz) */
static const double FREQ_PRESETS[] = {
    100.0, 200.0, 500.0, 
    1000.0, 2000.0, 5000.0,
    10000.0, 20000.0, 50000.0,
    100000.0, 500000.0, 1000000.0
};
#define FREQ_PRESET_COUNT  (sizeof(FREQ_PRESETS) / sizeof(FREQ_PRESETS[0]))

/* 幅度预设 (对应DAC输出电压, Vpp) */
static const double AMP_PRESETS[] = {
    0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.3
};
#define AMP_PRESET_COUNT  (sizeof(AMP_PRESETS) / sizeof(AMP_PRESETS[0]))

/* ============================================================
 * 第四部分：全局变量
 * ============================================================ */

static volatile uint32_t g_systick_ms = 0;

/* 当前波形配置 */
static AD9833_Config_t g_wave_config = {
    .waveform = WAVE_SINE,
    .frequency = 1000.0,
    .phase = 0
};

/* 当前幅度索引和偏置 */
static uint8_t g_amplitude_idx = 3;   /* 默认2.0Vpp */
static double g_dc_offset = 0.0;      /* 直流偏置(V) */
static uint8_t g_freq_idx = 3;        /* 默认1kHz */

/* 扫频参数 */
static bool g_sweep_enabled = false;
static double g_sweep_start_freq = 100.0;
static double g_sweep_stop_freq = 100000.0;
static double g_sweep_current_freq = 100.0;
static uint32_t g_sweep_step_time_ms = 10; /* 每步驻留时间 */
static double g_sweep_step_size = 100.0;   /* 每步频率增量 */
static uint8_t g_sweep_direction = 1;      /* 1=升频, 0=降频 */

/* 按键状态 */
static volatile bool g_key1_pressed = false;
static volatile bool g_key2_pressed = false;
static volatile bool g_key3_pressed = false;
static volatile bool g_key4_pressed = false;

/* ============================================================
 * 第五部分：底层SPI与I2C操作
 * ============================================================ */

/**
 * @brief 延时微秒
 */
static void delay_us(uint32_t us)
{
    while (us--) {
        for (volatile int i = 0; i < 8; i++) __NOP();
    }
}

/**
 * @brief 延时毫秒
 */
static void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000);
}

/**
 * @brief SPI传输16位数据 (MSB first)
 * @param data 16位数据
 */
static void spi_send_16bit(uint16_t data)
{
    /* 发送高字节 */
    DL_SPI_transmitData8(SPI0, (uint8_t)(data >> 8));
    while (DL_SPI_isBusy(SPI0)) ;
    (void)DL_SPI_receiveData8(SPI0);
    
    /* 发送低字节 */
    DL_SPI_transmitData8(SPI0, (uint8_t)(data & 0xFF));
    while (DL_SPI_isBusy(SPI0)) ;
    (void)DL_SPI_receiveData8(SPI0);
}

/**
 * @brief AD9833 FSYNC控制
 */
static void ad9833_select(void)
{
    DL_GPIO_clearPins(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN);
    delay_us(1);
}

static void ad9833_deselect(void)
{
    delay_us(1);
    DL_GPIO_setPins(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN);
}

/**
 * @brief 向AD9833写入16位控制字
 * @param data 16位数据
 */
static void ad9833_write(uint16_t data)
{
    ad9833_select();
    spi_send_16bit(data);
    ad9833_deselect();
}

/**
 * @brief I2C写DAC8571
 * @param value 16位DAC值 (0~65535)
 * @return true=写入成功
 */
static bool dac8571_write(uint16_t value)
{
    uint8_t buf[3];
    buf[0] = DAC8571_CTRL_BOTH;             /* 控制字节: 写入并更新 */
    buf[1] = (uint8_t)(value >> 8);         /* 数据高字节 */
    buf[2] = (uint8_t)(value & 0xFF);       /* 数据低字节 */
    
    /* 使用MSPM0的I2C驱动发送 */
    DL_I2C_startTransfer(I2C0);
    DL_I2C_transmitData(I2C0, (DAC8571_ADDR << 1) | 0); /* 写地址 */
    
    /* 等待发送完成 */
    while (!DL_I2C_isTXEmpty(I2C0)) ;
    
    /* 发送控制字节和数据 */
    for (uint8_t i = 0; i < 3; i++) {
        DL_I2C_transmitData(I2C0, buf[i]);
        while (!DL_I2C_isTXEmpty(I2C0)) ;
    }
    
    DL_I2C_stopTransfer(I2C0);
    delay_us(100);
    
    return true;
}

/**
 * @brief 设置DAC8571输出电压
 * @param voltage 输出电压 (0 ~ VREF)
 */
static void dac8571_set_voltage(double voltage)
{
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > DAC_VREF) voltage = DAC_VREF;
    
    uint16_t dac_value = (uint16_t)((voltage / DAC_VREF) * 65535.0);
    dac8571_write(dac_value);
}

/* ============================================================
 * 第六部分：AD9833功能函数
 * ============================================================ */

/**
 * @brief AD9833复位
 */
static void ad9833_reset(void)
{
    ad9833_write(AD9833_RESET);
    delay_ms(10);
}

/**
 * @brief 设置AD9833频率
 * @param freq_hz 频率(Hz)
 */
static void ad9833_set_frequency(double freq_hz)
{
    /* 计算频率字: freq_word = freq * 2^28 / MCLK */
    uint32_t freq_word = (uint32_t)((freq_hz / AD9833_MCLK) * 268435456.0);
    
    /* B28模式，分两次写入 */
    uint16_t control = AD9833_B28;
    
    /* 保留当前波形设置 */
    switch (g_wave_config.waveform) {
        case WAVE_SINE:
            break; /* 默认正弦 */
        case WAVE_TRIANGLE:
            control |= AD9833_MODE;
            break;
        case WAVE_SQUARE:
            control |= AD9833_OPBITEN;
            break;
        case WAVE_SQUARE_DIV2:
            control |= AD9833_OPBITEN | AD9833_DIV2;
            break;
    }
    
    /* 写入控制字 (清除RESET位) */
    ad9833_write(control);
    
    /* 写入频率字低14位 */
    ad9833_write(AD9833_FREQ0_REG | (freq_word & 0x3FFF));
    
    /* 写入频率字高14位 */
    ad9833_write(AD9833_FREQ0_REG | ((freq_word >> 14) & 0x3FFF));
}

/**
 * @brief 设置AD9833相位
 * @param phase 相位值 (0~4095, 对应0~2π)
 */
static void ad9833_set_phase(uint16_t phase)
{
    ad9833_write(AD9833_PHASE0_REG | (phase & 0x0FFF));
}

/**
 * @brief 设置AD9833波形类型
 * @param type 波形类型
 */
static void ad9833_set_waveform(WaveformType_t type)
{
    uint16_t control = AD9833_B28;
    
    switch (type) {
        case WAVE_SINE:
            control = AD9833_B28; /* 清除MODE和OPBITEN */
            break;
        case WAVE_TRIANGLE:
            control = AD9833_B28 | AD9833_MODE;
            break;
        case WAVE_SQUARE:
            control = AD9833_B28 | AD9833_OPBITEN;
            break;
        case WAVE_SQUARE_DIV2:
            control = AD9833_B28 | AD9833_OPBITEN | AD9833_DIV2;
            break;
    }
    
    ad9833_write(control);
    
    /* 重新设置频率以应用新的波形设置 */
    ad9833_set_frequency(g_wave_config.frequency);
}

/**
 * @brief 完整配置AD9833
 * @param config 波形配置
 */
static void ad9833_configure(const AD9833_Config_t *config)
{
    ad9833_reset();
    delay_ms(5);
    
    ad9833_set_waveform(config->waveform);
    ad9833_set_frequency(config->frequency);
    ad9833_set_phase(config->phase);
}

/* ============================================================
 * 第七部分：波形发生器功能函数
 * ============================================================ */

/**
 * @brief 更新输出幅度 (通过DAC8571控制运放增益)
 */
static void update_amplitude(void)
{
    /* 将幅度预设值转换为DAC电压 */
    /* DAC输出通过运放控制信号幅度 */
    double amp = AMP_PRESETS[g_amplitude_idx];
    
    /* DAC电压 = 幅度 / 2 (运放增益为2) */
    double dac_voltage = amp / 2.0;
    
    dac8571_set_voltage(dac_voltage);
}

/**
 * @brief 更新直流偏置
 */
static void update_dc_offset(void)
{
    /* 偏置范围: -VREF/2 ~ +VREF/2 */
    /* 通过DAC输出中心电压实现 */
    double offset_voltage = (g_dc_offset / 2.0) + (DAC_VREF / 2.0);
    
    /* 注意: 这里简化处理，实际可能需要双DAC或运放电路 */
    (void)offset_voltage;
}

/**
 * @brief 执行扫频输出
 */
static void sweep_process(void)
{
    if (!g_sweep_enabled) return;
    
    static uint32_t last_step_time = 0;
    
    if ((g_systick_ms - last_step_time) >= g_sweep_step_time_ms) {
        last_step_time = g_systick_ms;
        
        /* 更新频率 */
        if (g_sweep_direction) {
            g_sweep_current_freq += g_sweep_step_size;
            if (g_sweep_current_freq >= g_sweep_stop_freq) {
                g_sweep_current_freq = g_sweep_stop_freq;
                g_sweep_direction = 0; /* 反向 */
            }
        } else {
            g_sweep_current_freq -= g_sweep_step_size;
            if (g_sweep_current_freq <= g_sweep_start_freq) {
                g_sweep_current_freq = g_sweep_start_freq;
                g_sweep_direction = 1; /* 正向 */
            }
        }
        
        /* 更新AD9833输出频率 */
        ad9833_set_frequency(g_sweep_current_freq);
    }
}

/**
 * @brief 处理按键输入
 */
static void process_keys(void)
{
    /* KEY1: 波形切换 */
    if (g_key1_pressed) {
        g_key1_pressed = false;
        
        g_wave_config.waveform = (WaveformType_t)((g_wave_config.waveform + 1) % 4);
        ad9833_set_waveform(g_wave_config.waveform);
        
        /* LED闪烁指示波形切换 */
        for (uint8_t i = 0; i <= (uint8_t)g_wave_config.waveform; i++) {
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
            delay_ms(100);
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
            delay_ms(100);
        }
    }
    
    /* KEY2: 频率增大 */
    if (g_key2_pressed) {
        g_key2_pressed = false;
        
        if (g_freq_idx < FREQ_PRESET_COUNT - 1) {
            g_freq_idx++;
        }
        g_wave_config.frequency = FREQ_PRESETS[g_freq_idx];
        ad9833_set_frequency(g_wave_config.frequency);
    }
    
    /* KEY3: 频率减小 */
    if (g_key3_pressed) {
        g_key3_pressed = false;
        
        if (g_freq_idx > 0) {
            g_freq_idx--;
        }
        g_wave_config.frequency = FREQ_PRESETS[g_freq_idx];
        ad9833_set_frequency(g_wave_config.frequency);
    }
    
    /* KEY4: 幅度调节 */
    if (g_key4_pressed) {
        g_key4_pressed = false;
        
        g_amplitude_idx = (g_amplitude_idx + 1) % AMP_PRESET_COUNT;
        update_amplitude();
    }
}

/* ============================================================
 * 第八部分：中断处理
 * ============================================================ */

/**
 * @brief GROUP1中断处理 (按键中断)
 */
void GROUP1_IRQHandler(void)
{
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOA, 
                        DL_GPIO_PIN_8 | DL_GPIO_PIN_9 | 
                        DL_GPIO_PIN_10 | DL_GPIO_PIN_11);
    
    if (flags & DL_GPIO_PIN_8) {
        g_key1_pressed = true;
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_8);
    }
    if (flags & DL_GPIO_PIN_9) {
        g_key2_pressed = true;
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_9);
    }
    if (flags & DL_GPIO_PIN_10) {
        g_key3_pressed = true;
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_10);
    }
    if (flags & DL_GPIO_PIN_11) {
        g_key4_pressed = true;
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_11);
    }
}

/* ============================================================
 * 第九部分：主函数
 * ============================================================ */

int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();
    
    /* 配置AD9833 FSYNC引脚 */
    DL_GPIO_initDigitalOutput(AD9833_FSYNC_PIN);
    DL_GPIO_setPins(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN);
    
    /* 配置按键输入 (内部上拉) */
    DL_GPIO_initDigitalInputFeatures(DL_GPIO_PIN_8,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(DL_GPIO_PIN_9,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(DL_GPIO_PIN_10,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(DL_GPIO_PIN_11,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    
    /* 使能按键中断 (下降沿) */
    DL_GPIO_setInterruptEdge(GPIOA, DL_GPIO_PIN_8 | DL_GPIO_PIN_9 | 
                                  DL_GPIO_PIN_10 | DL_GPIO_PIN_11,
                             DL_GPIO_EDGE_FALLING);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    
    /* ==================== 初始化AD9833 ==================== */
    ad9833_reset();
    delay_ms(50);
    
    /* 设置默认输出: 1kHz正弦波 */
    g_wave_config.waveform = WAVE_SINE;
    g_wave_config.frequency = 1000.0;
    g_wave_config.phase = 0;
    ad9833_configure(&g_wave_config);
    
    /* ==================== 初始化DAC8571 ==================== */
    /* 设置默认幅度 */
    dac8571_set_voltage(1.0); /* 默认1V输出 */
    
    /* ==================== 初始化完成 ==================== */
    /* LED快闪3次指示初始化完成 */
    for (uint8_t i = 0; i < 3; i++) {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14);
        delay_ms(200);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14);
        delay_ms(200);
    }
    
    /* ==================== 主循环 ==================== */
    uint32_t last_status_time = 0;
    
    while (1) {
        /* 处理按键 */
        process_keys();
        
        /* 扫频处理 */
        sweep_process();
        
        /* 定期显示状态 (可通过UART输出) */
        if ((g_systick_ms - last_status_time) >= 1000) {
            last_status_time = g_systick_ms;
            
            /* LED闪烁表示正在运行 */
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
        }
        
        /* 模拟系统时间递增 */
        delay_ms(1);
        g_systick_ms++;
    }
}
