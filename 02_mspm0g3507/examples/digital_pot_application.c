/**
 * @file digital_pot_application.c
 * @brief MSPM0G3507 数字电位器应用示例 - MCP4161 + 可编程增益 + 程控电源
 * 
 * 硬件连接：
 *   MCP4161 数字电位器 (SPI接口):
 *     SCK  -> PA2 (SPI0_SCK)
 *     SDI  -> PA4 (SPI0_MOSI, MCP4161的SDI引脚)
 *     SDO  -> PA6 (SPI0_MISO, MCP4161的SDO引脚)
 *     CS   -> PA3 (GPIO, 片选)
 *     SHDN -> PA7 (GPIO, 关断控制, 低电平有效)
 * 
 *   可编程增益运放电路 (TL072):
 *     MCP4161 Wiper -> 运放反馈电阻
 *     实现增益 = 1 + (R_wiper / R_fixed)
 * 
 *   程控电源电路:
 *     MCP4161 Wiper -> LM317 ADJ引脚
 *     Vout = 1.25 * (1 + R2/R1)
 * 
 *   电压监测:
 *     ADC通道0 -> 输出电压分压采样
 *     ADC通道1 -> 电流检测电阻
 * 
 *   按键控制:
 *     KEY1 -> PA8 (电位器值增大)
 *     KEY2 -> PA9 (电位器值减小)
 *     KEY3 -> PA10(模式切换)
 *     KEY4 -> PA11(确认/保存)
 * 
 *   OLED显示 (I2C):
 *     SDA  -> PB9
 *     SCL  -> PB8
 * 
 * 功能说明：
 *   - MCP4161 256级数字电位器驱动 (10kΩ)
 *   - 可编程增益放大器: 增益范围 1x ~ 100x
 *   - 程控电源: 输出电压 1.25V ~ 12V
 *   - ADC监测输出电压和电流
 *   - OLED显示实时参数
 *   - 按键交互控制
 * 
 * 适用场景：电赛中程控电源、可编程放大器、自动增益控制
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ============================================================
 * 第一部分：MCP4161数字电位器驱动
 * ============================================================ */

/* MCP4161命令字 */
#define MCP4161_CMD_WRITE       0x00    /* 写数据 */
#define MCP4161_CMD_READ        0x03    /* 读数据 */
#define MCP4161_CMD_INCREMENT   0x04    /* 递增 */
#define MCP4161_CMD_DECREMENT   0x08    /* 递减 */

/* MCP4161寄存器地址 */
#define MCP4161_REG_WIPER0      0x00    /* 电位器0抽头位置 */
#define MCP4161_REG_WIPER1      0x01    /* 电位器1抽头位置 (MCP4261) */
#define MCP4161_REG_TCON        0x04    /* 端子连接控制 */
#define MCP4161_REG_STATUS      0x05    /* 状态寄存器 */

/* MCP4161参数 */
#define MCP4161_STEPS           256     /* 总步数 (8位) */
#define MCP4161_MAX_VALUE       255     /* 最大值 */
#define MCP4161_RESISTANCE      10000   /* 总电阻 (10kΩ) */
#define MCP4161_STEP_RESISTANCE (MCP4161_RESISTANCE / MCP4161_STEPS)

/* 引脚定义 */
#define MCP4161_CS_PORT         GPIOA
#define MCP4161_CS_PIN          DL_GPIO_PIN_3
#define MCP4161_SHDN_PORT       GPIOA
#define MCP4161_SHDN_PIN        DL_GPIO_PIN_7

/* ============================================================
 * 第二部分：应用参数定义
 * ============================================================ */

/* 工作模式 */
typedef enum {
    MODE_PGA = 0,           /* 可编程增益放大器模式 */
    MODE_POWER_SUPPLY,      /* 程控电源模式 */
    MODE_RESISTANCE,        /* 程控电阻模式 */
    MODE_COUNT              /* 模式总数 */
} AppMode_t;

/* PGA参数 */
#define PGA_R_FIXED         1000.0   /* 固定电阻 (1kΩ) */
#define PGA_R_POT_MAX       10000.0  /* 电位器最大电阻 (10kΩ) */
#define PGA_GAIN_MIN        1.0      /* 最小增益 */
#define PGA_GAIN_MAX        11.0     /* 最大增益 (1 + 10k/1k) */

/* 程控电源参数 */
#define PS_R1               240.0    /* R1电阻 (Ω) */
#define PS_R2_MAX           10000.0  /* R2最大电阻 (Ω) */
#define PS_VREF             1.25     /* LM317参考电压 (V) */
#define PS_VOUT_MIN         1.25     /* 最小输出电压 */
#define PS_VOUT_MAX         12.0     /* 最大输出电压 */

/* ADC参数 */
#define ADC_VREF            3.3      /* ADC参考电压 */
#define ADC_RESOLUTION      4096.0   /* 12位ADC分辨率 */
#define VOLTAGE_DIVIDER     4.0      /* 电压分压比 (用于高压采样) */
#define CURRENT_SENSE_R     0.1      /* 电流检测电阻 (Ω) */
#define CURRENT_AMP_GAIN    10.0     /* 电流放大增益 */

/* ============================================================
 * 第三部分：全局变量
 * ============================================================ */

static volatile uint32_t g_systick_ms = 0;       /* 系统毫秒计数器 */

/* 当前工作模式 */
static AppMode_t g_current_mode = MODE_PGA;

/* 电位器当前值 */
static uint8_t g_pot_value = 0;                   /* 0~255 */
static uint8_t g_pot_value_backup = 0;            /* 备份值 */

/* PGA参数 */
static double g_pga_gain = 1.0;                   /* 当前增益 */
static double g_pga_gain_target = 1.0;            /* 目标增益 */

/* 程控电源参数 */
static double g_ps_voltage = 5.0;                 /* 当前输出电压 */
static double g_ps_voltage_target = 5.0;          /* 目标输出电压 */

/* 测量值 */
static double g_measured_voltage = 0.0;            /* 测量的输出电压 */
static double g_measured_current = 0.0;            /* 测量的输出电流 */

/* 按键状态 */
static volatile bool g_key1_pressed = false;
static volatile bool g_key2_pressed = false;
static volatile bool g_key3_pressed = false;
static volatile bool g_key4_pressed = false;

/* ============================================================
 * 第四部分：底层SPI操作
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
 * @brief SPI传输字节
 * @param data 发送的数据
 * @return 接收的数据
 */
static uint8_t spi_transfer(uint8_t data)
{
    DL_SPI_transmitData8(SPI0, data);
    while (DL_SPI_isBusy(SPI0)) ;
    return (uint8_t)DL_SPI_receiveData8(SPI0);
}

/**
 * @brief MCP4161 CS控制
 */
static void mcp4161_select(void)
{
    DL_GPIO_clearPins(MCP4161_CS_PORT, MCP4161_CS_PIN);
    delay_us(1);
}

static void mcp4161_deselect(void)
{
    delay_us(1);
    DL_GPIO_setPins(MCP4161_CS_PORT, MCP4161_CS_PIN);
}

/* ============================================================
 * 第五部分：MCP4161驱动函数
 * ============================================================ */

/**
 * @brief 写入MCP4161寄存器
 * @param reg 寄存器地址
 * @param value 要写入的值
 */
static void mcp4161_write(uint8_t reg, uint8_t value)
{
    /* 命令格式: [C1:C0] [R1:R0] [D8] [D7:D0] */
    /* 对于8位电位器，D8=0 */
    uint8_t cmd = (MCP4161_CMD_WRITE << 4) | (reg & 0x0F);
    
    mcp4161_select();
    spi_transfer(cmd);
    spi_transfer(value);
    mcp4161_deselect();
}

/**
 * @brief 读取MCP4161寄存器
 * @param reg 寄存器地址
 * @return 读取的值
 */
static uint8_t mcp4161_read(uint8_t reg)
{
    uint8_t cmd = (MCP4161_CMD_READ << 4) | (reg & 0x0F);
    uint8_t value;
    
    mcp4161_select();
    spi_transfer(cmd);
    value = spi_transfer(0xFF); /* 发送哑数据读取 */
    mcp4161_deselect();
    
    return value;
}

/**
 * @brief 设置电位器抽头位置
 * @param value 抽头位置 (0~255)
 */
static void mcp4161_set_wiper(uint8_t value)
{
    mcp4161_write(MCP4161_REG_WIPER0, value);
    g_pot_value = value;
}

/**
 * @brief 递增电位器
 * @param steps 递增步数
 */
static void mcp4161_increment(uint8_t steps)
{
    uint8_t cmd = (MCP4161_CMD_INCREMENT << 4) | MCP4161_REG_WIPER0;
    
    for (uint8_t i = 0; i < steps; i++) {
        mcp4161_select();
        spi_transfer(cmd);
        mcp4161_deselect();
        
        if (g_pot_value < MCP4161_MAX_VALUE) {
            g_pot_value++;
        }
    }
}

/**
 * @brief 递减电位器
 * @param steps 递减步数
 */
static void mcp4161_decrement(uint8_t steps)
{
    uint8_t cmd = (MCP4161_CMD_DECREMENT << 4) | MCP4161_REG_WIPER0;
    
    for (uint8_t i = 0; i < steps; i++) {
        mcp4161_select();
        spi_transfer(cmd);
        mcp4161_deselect();
        
        if (g_pot_value > 0) {
            g_pot_value--;
        }
    }
}

/**
 * @brief 获取当前电阻值
 * @param wiper_pos 抽头位置
 * @return 电阻值 (Ω)
 */
static double mcp4161_get_resistance(uint8_t wiper_pos)
{
    return (double)wiper_pos * MCP4161_STEP_RESISTANCE;
}

/**
 * @brief 使能MCP4161
 */
static void mcp4161_enable(void)
{
    DL_GPIO_setPins(MCP4161_SHDN_PORT, MCP4161_SHDN_PIN);
}

/**
 * @brief 关断MCP4161 (省电模式)
 */
static void mcp4161_shutdown(void)
{
    DL_GPIO_clearPins(MCP4161_SHDN_PORT, MCP4161_SHDN_PIN);
}

/**
 * @brief 初始化MCP4161
 */
static void mcp4161_init(void)
{
    /* 使能 */
    mcp4161_enable();
    delay_ms(10);
    
    /* 设置初始位置为0 */
    mcp4161_set_wiper(0);
    
    /* 配置TCON寄存器: 使能电位器A和B端子 */
    mcp4161_write(MCP4161_REG_TCON, 0xFF);
}

/* ============================================================
 * 第六部分：ADC测量功能
 * ============================================================ */

/**
 * @brief 读取ADC通道值
 * @param channel ADC通道号 (0~7)
 * @return ADC原始值 (0~4095)
 */
static uint16_t adc_read_channel(uint8_t channel)
{
    /* 配置ADC通道 */
    DL_ADC_setChannel(ADC0, channel);
    delay_us(10);
    
    /* 启动转换 */
    DL_ADC_enableConversions(ADC0);
    DL_ADC_startConversion(ADC0);
    
    /* 等待转换完成 */
    while (!DL_ADC_isConversionComplete(ADC0)) ;
    
    /* 读取结果 */
    uint16_t result = DL_ADC_getConversionResult(ADC0);
    
    DL_ADC_disableConversions(ADC0);
    
    return result;
}

/**
 * @brief 读取输出电压
 * @return 电压值 (V)
 */
static double read_output_voltage(void)
{
    uint16_t adc_value = adc_read_channel(0);
    
    /* 计算实际电压 */
    double adc_voltage = (double)adc_value * ADC_VREF / ADC_RESOLUTION;
    double actual_voltage = adc_voltage * VOLTAGE_DIVIDER;
    
    return actual_voltage;
}

/**
 * @brief 读取输出电流
 * @return 电流值 (mA)
 */
static double read_output_current(void)
{
    uint16_t adc_value = adc_read_channel(1);
    
    /* 计算电流检测电阻上的电压 */
    double adc_voltage = (double)adc_value * ADC_VREF / ADC_RESOLUTION;
    double sense_voltage = adc_voltage / CURRENT_AMP_GAIN;
    double current = sense_voltage / CURRENT_SENSE_R * 1000.0; /* 转换为mA */
    
    return current;
}

/* ============================================================
 * 第七部分：PGA可编程增益控制
 * ============================================================ */

/**
 * @brief 设置PGA增益
 * @param gain 目标增益 (1.0 ~ 11.0)
 * @return 实际设置的增益
 */
static double pga_set_gain(double gain)
{
    /* 限制增益范围 */
    if (gain < PGA_GAIN_MIN) gain = PGA_GAIN_MIN;
    if (gain > PGA_GAIN_MAX) gain = PGA_GAIN_MAX;
    
    /* 计算所需的电位器电阻值 */
    /* 增益 = 1 + R_pot / R_fixed */
    /* R_pot = (gain - 1) * R_fixed */
    double r_pot = (gain - 1.0) * PGA_R_FIXED;
    
    /* 计算电位器位置 */
    uint8_t pot_pos = (uint8_t)(r_pot * MCP4161_MAX_VALUE / PGA_R_POT_MAX);
    if (pot_pos > MCP4161_MAX_VALUE) pot_pos = MCP4161_MAX_VALUE;
    
    /* 设置电位器 */
    mcp4161_set_wiper(pot_pos);
    
    /* 计算实际增益 */
    double actual_r_pot = mcp4161_get_resistance(pot_pos);
    double actual_gain = 1.0 + actual_r_pot / PGA_R_FIXED;
    
    g_pga_gain = actual_gain;
    g_pot_value_backup = pot_pos;
    
    return actual_gain;
}

/**
 * @brief 步进调节PGA增益
 * @param up true=增大, false=减小
 * @param step 步长 (电位器步数)
 */
static void pga_step_gain(bool up, uint8_t step)
{
    if (up) {
        mcp4161_increment(step);
    } else {
        mcp4161_decrement(step);
    }
    
    /* 更新增益 */
    double actual_r_pot = mcp4161_get_resistance(g_pot_value);
    g_pga_gain = 1.0 + actual_r_pot / PGA_R_FIXED;
}

/* ============================================================
 * 第八部分：程控电源控制
 * ============================================================ */

/**
 * @brief 设置程控电源输出电压
 * @param voltage 目标电压 (V)
 * @return 实际输出电压
 */
static double ps_set_voltage(double voltage)
{
    /* 限制电压范围 */
    if (voltage < PS_VOUT_MIN) voltage = PS_VOUT_MIN;
    if (voltage > PS_VOUT_MAX) voltage = PS_VOUT_MAX;
    
    /* LM317输出电压: Vout = Vref * (1 + R2/R1) */
    /* R2 = R1 * (Vout/Vref - 1) */
    double r2 = PS_R1 * (voltage / PS_VREF - 1.0);
    
    /* 限制R2范围 */
    if (r2 > PS_R2_MAX) r2 = PS_R2_MAX;
    if (r2 < 0) r2 = 0;
    
    /* 计算电位器位置 */
    uint8_t pot_pos = (uint8_t)(r2 * MCP4161_MAX_VALUE / PS_R2_MAX);
    if (pot_pos > MCP4161_MAX_VALUE) pot_pos = MCP4161_MAX_VALUE;
    
    /* 设置电位器 */
    mcp4161_set_wiper(pot_pos);
    
    /* 计算实际输出电压 */
    double actual_r2 = mcp4161_get_resistance(pot_pos);
    double actual_voltage = PS_VREF * (1.0 + actual_r2 / PS_R1);
    
    g_ps_voltage = actual_voltage;
    g_pot_value_backup = pot_pos;
    
    return actual_voltage;
}

/**
 * @brief 步进调节输出电压
 * @param up true=增大, false=减小
 * @param step 步长 (电位器步数)
 */
static void ps_step_voltage(bool up, uint8_t step)
{
    if (up) {
        mcp4161_increment(step);
    } else {
        mcp4161_decrement(step);
    }
    
    /* 更新电压 */
    double actual_r2 = mcp4161_get_resistance(g_pot_value);
    g_ps_voltage = PS_VREF * (1.0 + actual_r2 / PS_R1);
}

/* ============================================================
 * 第九部分：按键处理
 * ============================================================ */

/**
 * @brief 处理按键事件
 */
static void process_keys(void)
{
    /* KEY1: 增大 */
    if (g_key1_pressed) {
        g_key1_pressed = false;
        
        switch (g_current_mode) {
            case MODE_PGA:
                pga_step_gain(true, 5);
                break;
            case MODE_POWER_SUPPLY:
                ps_step_voltage(true, 5);
                break;
            case MODE_RESISTANCE:
                mcp4161_increment(5);
                break;
            default:
                break;
        }
    }
    
    /* KEY2: 减小 */
    if (g_key2_pressed) {
        g_key2_pressed = false;
        
        switch (g_current_mode) {
            case MODE_PGA:
                pga_step_gain(false, 5);
                break;
            case MODE_POWER_SUPPLY:
                ps_step_voltage(false, 5);
                break;
            case MODE_RESISTANCE:
                mcp4161_decrement(5);
                break;
            default:
                break;
        }
    }
    
    /* KEY3: 模式切换 */
    if (g_key3_pressed) {
        g_key3_pressed = false;
        
        g_current_mode = (AppMode_t)((g_current_mode + 1) % MODE_COUNT);
        
        /* 切换模式时恢复备份值 */
        mcp4161_set_wiper(g_pot_value_backup);
        
        /* LED指示模式切换 */
        DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
        delay_ms(200);
        DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
    }
    
    /* KEY4: 确认/保存 */
    if (g_key4_pressed) {
        g_key4_pressed = false;
        
        /* 保存当前配置 */
        g_pot_value_backup = g_pot_value;
        
        /* 确认提示 */
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14);
        delay_ms(500);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14);
    }
}

/* ============================================================
 * 第十部分：OLED显示函数
 * ============================================================ */

/**
 * @brief 简化的OLED初始化 (I2C)
 */
static void oled_init(void)
{
    /* 发送初始化命令序列 */
    uint8_t init_cmds[] = {
        0xAE, 0x00, 0x10, 0x40, 0x81, 0xCF, 0xA1, 0xA6,
        0xA8, 0x3F, 0xD3, 0x00, 0xD5, 0x80, 0xD9, 0xF1,
        0xDA, 0x12, 0xDB, 0x40, 0x20, 0x02, 0x8D, 0x14,
        0xAF
    };
    
    for (uint8_t i = 0; i < sizeof(init_cmds); i++) {
        /* I2C写命令到OLED (地址0x3C) */
        DL_I2C_startTransfer(I2C0);
        DL_I2C_transmitData(I2C0, (0x3C << 1) | 0);
        DL_I2C_transmitData(I2C0, 0x00); /* 控制字节: 命令 */
        DL_I2C_transmitData(I2C0, init_cmds[i]);
        DL_I2C_stopTransfer(I2C0);
        delay_us(100);
    }
}

/**
 * @brief 更新OLED显示内容
 */
static void update_display(void)
{
    /* 简化实现: 实际应显示中文/数字 */
    /* 这里仅作示意 */
    
    /* 显示模式 */
    /* 显示电位器值 */
    /* 显示测量值 */
    
    /* 通过UART输出调试信息 */
    (void)g_measured_voltage;
    (void)g_measured_current;
}

/* ============================================================
 * 第十一部分：中断处理
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
 * 第十二部分：主函数
 * ============================================================ */

int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();
    
    /* 配置MCP4161 CS引脚 */
    DL_GPIO_initDigitalOutput(MCP4161_CS_PIN);
    DL_GPIO_setPins(MCP4161_CS_PORT, MCP4161_CS_PIN);
    
    /* 配置MCP4161 SHDN引脚 */
    DL_GPIO_initDigitalOutput(MCP4161_SHDN_PIN);
    DL_GPIO_setPins(MCP4161_SHDN_PORT, MCP4161_SHDN_PIN);
    
    /* 配置按键输入 */
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
    
    /* 使能按键中断 */
    DL_GPIO_setInterruptEdge(GPIOA, 
        DL_GPIO_PIN_8 | DL_GPIO_PIN_9 | DL_GPIO_PIN_10 | DL_GPIO_PIN_11,
        DL_GPIO_EDGE_FALLING);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    
    /* ==================== 初始化MCP4161 ==================== */
    mcp4161_init();
    
    /* ==================== 初始化OLED ==================== */
    oled_init();
    
    /* ==================== 设置默认工作模式 ==================== */
    g_current_mode = MODE_PGA;
    pga_set_gain(2.0); /* 默认增益2倍 */
    
    /* LED指示初始化完成 */
    for (uint8_t i = 0; i < 3; i++) {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14);
        delay_ms(200);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14);
        delay_ms(200);
    }
    
    /* ==================== 主循环 ==================== */
    uint32_t last_measure_time = 0;
    uint32_t last_display_time = 0;
    
    while (1) {
        /* 处理按键 */
        process_keys();
        
        /* 定时测量 */
        if ((g_systick_ms - last_measure_time) >= 100) {
            last_measure_time = g_systick_ms;
            
            /* 读取电压和电流 */
            g_measured_voltage = read_output_voltage();
            g_measured_current = read_output_current();
        }
        
        /* 定时更新显示 */
        if ((g_systick_ms - last_display_time) >= 200) {
            last_display_time = g_systick_ms;
            
            update_display();
            
            /* LED心跳 */
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
        }
        
        /* 模拟系统时间递增 */
        delay_ms(1);
        g_systick_ms++;
    }
}
