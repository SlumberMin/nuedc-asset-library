/**
 * @file    battery_manager.c
 * @brief   电池管理系统完整示例 — MSPM0G3507
 *
 * 功能概述:
 *   1. ADC采集电池电压 (分压电阻)
 *   2. SOC (State of Charge) 电量估算 (查表法+库仑计数)
 *   3. 低电压保护: 自动切断负载
 *   4. OLED显示电压/电量/电流/功率
 *   5. 蓝牙远程监控
 *   6. 充电状态检测
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │                    硬件接线说明                           │
 * ├──────────────────────────────────────────────────────────┤
 * │ 电池电压检测 (ADC):                                      │
 * │   电池+ → R1(10K) → MSPM0 PA25(ADC) → R2(10K) → GND   │
 * │   分压比: 1:2, 检测范围 0~6.6V (适配2S锂电7.4V需调整)    │
 * │                                                          │
 * │ 电流检测 (可选, INA219 I2C):                              │
 * │   MSPM0 PB2(SCL) → INA219 SCL                          │
 * │   MSPM0 PB3(SDA) → INA219 SDA                          │
 * │   I2C地址: 0x44                                          │
 * │                                                          │
 * │ OLED (I2C0, 地址0x3C):                                   │
 * │   MSPM0 PB2(SCL) → OLED SCL                             │
 * │   MSPM0 PB3(SDA) → OLED SDA                             │
 * │                                                          │
 * │ 负载控制 (MOSFET/继电器):                                 │
 * │   MSPM0 PA22 → MOSFET Gate (高电平导通)                  │
 * │                                                          │
 * │ 充电检测:                                                 │
 * │   MSPM0 PA23 ← 充电器状态 (高=充电中)                    │
 * │                                                          │
 * │ LED指示:                                                 │
 * │   MSPM0 PA24 → 绿LED (电量OK)                           │
 * │   MSPM0 PA26 → 红LED (低电量)                            │
 * └──────────────────────────────────────────────────────────┘
 *
 * 锂电池SOC估算方法:
 *   1. 开路电压法 (OCV): 根据电压-SOC查表
 *   2. 库仑计数法: 对电流积分 (需电流传感器)
 *   3. 混合法: OCV校准 + 库仑计数跟踪
 *
 * 电压-SOC对照表 (单节3.7V锂电池, 4.2V满电, 3.0V截止):
 *   4.20V=100%, 4.03V=80%, 3.86V=60%, 3.78V=40%,
 *   3.70V=20%, 3.60V=10%, 3.30V=5%, 3.00V=0%
 *
 * 依赖驱动: oled_ssd1306_mspm0, i2c_bus, bluetooth_hc05_mspm0, pin_config
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/oled_ssd1306_mspm0.h"
#include "drivers/i2c_bus.h"
#include "drivers/bluetooth_hc05_mspm0.h"
#include "drivers/pin_config.h"

/* ══════════════════════════════════════════════════════════════
 *  硬件配置
 * ══════════════════════════════════════════════════════════════ */

/* ADC配置 */
#define ADC_INST            ADC12_0_INST    /* ADC实例 */
#define ADC_CHANNEL         DL_ADC12_MEM_IDX_0  /* ADC通道 */
#define ADC_RESOLUTION      4096            /* 12位ADC */

/* 分压电阻 */
/* 错误经验#1: 分压比计算 — R1/(R1+R2)不会为0 (R1,R2>0) */
#define R_DIVIDER_R1        10000.0f        /* 上臂电阻 (Ω) */
#define R_DIVIDER_R2        10000.0f        /* 下臂电阻 (Ω) */
#define DIVIDER_RATIO       (R_DIVIDER_R2 / (R_DIVIDER_R1 + R_DIVIDER_R2))
                                            /* 分压比 = R2/(R1+R2) */
#define ADC_VREF            3.3f            /* ADC参考电压 */

/* 电池参数 */
#define BATTERY_CELLS       1               /* 电池节数 (1S=3.7V, 2S=7.4V) */
#define BATTERY_CAPACITY_MAH 2000.0f        /* 电池容量 (mAh) */

/* 低电压保护阈值 */
#define VOLTAGE_LOW_WARN    3.40f           /* 低电量警告 (V/节) */
#define VOLTAGE_LOW_CUTOFF  3.10f           /* 低电压切断 (V/节) */
#define VOLTAGE_CRITICAL    3.00f           /* 危险电压 (V/节) */

/* 采样和显示间隔 */
#define SAMPLE_INTERVAL_MS  500             /* 电压采样间隔 */
#define OLED_REFRESH_MS     300             /* OLED刷新间隔 */
#define BT_SEND_INTERVAL_MS 1000            /* 蓝牙发送间隔 */

/* 滑动平均窗口大小 */
#define VOLTAGE_FILTER_SIZE 8

/* ══════════════════════════════════════════════════════════════
 *  SOC查找表 (电压→电量百分比)
 *  单节锂电池放电曲线
 * ══════════════════════════════════════════════════════════════ */

typedef struct {
    float voltage;      /* 电压 (V/节) */
    float soc;          /* 电量百分比 (%) */
} OCV_TableEntry;

/* 电压-SOC对照表 (降序排列) */
static const OCV_TableEntry g_ocv_table[] = {
    {4.20f, 100.0f},
    {4.03f,  80.0f},
    {3.86f,  60.0f},
    {3.78f,  40.0f},
    {3.70f,  20.0f},
    {3.60f,  10.0f},
    {3.30f,   5.0f},
    {3.00f,   0.0f}
};

#define OCV_TABLE_SIZE  (sizeof(g_ocv_table) / sizeof(g_ocv_table[0]))

/* ══════════════════════════════════════════════════════════════
 *  全局变量
 * ══════════════════════════════════════════════════════════════ */

/* I2C总线 */
static I2C_Bus g_i2c_bus;

/* 电压滤波 */
static float g_voltage_buf[VOLTAGE_FILTER_SIZE];
static uint8_t g_voltage_idx = 0;

/* 电池状态 */
static volatile float g_batt_voltage = 0.0f;       /* 电池电压 (V) */
static volatile float g_cell_voltage = 0.0f;        /* 单节电压 (V) */
static volatile float g_soc_percent = 100.0f;       /* 电量百分比 (%) */
static volatile float g_current_mA = 0.0f;          /* 电流 (mA) */
static volatile float g_power_mW = 0.0f;            /* 功率 (mW) */

/* 保护状态 */
static volatile uint8_t g_load_enabled = 1;         /* 负载使能 */
static volatile uint8_t g_is_charging = 0;          /* 充电状态 */
static volatile uint8_t g_low_warn = 0;             /* 低电量警告 */
static volatile uint8_t g_low_cutoff = 0;           /* 低电压切断 */

/* 累计电量 (mAh) — 简化版 */
static volatile float g_consumed_mah = 0.0f;

/* 计时 */
static volatile uint32_t g_sys_tick = 0;
static uint32_t g_last_sample_tick = 0;
static uint32_t g_last_oled_tick = 0;
static uint32_t g_last_bt_tick = 0;

/* ══════════════════════════════════════════════════════════════
 *  ADC读取
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 读取ADC原始值
 * @return 12位ADC值 (0~4095)
 *
 * 错误经验#18: 运算符优先级 — !(x & flag) 必须加括号
 */
static uint16_t ADC_ReadRaw(void)
{
    /* 启动转换 */
    DL_ADC12_startConversion(ADC_INST);

    /* 等待转换完成 — 错误经验#18: 必须写成 !(status & flag) */
    uint32_t timeout = I2C_BUS_TIMEOUT;
    while (!(DL_ADC12_getStatus(ADC_INST) & DL_ADC12_STATUS_CONVERSION_DONE)) {
        if (--timeout == 0) return 0;   /* 超时返回0 */
    }

    /* 读取结果 */
    return (uint16_t)DL_ADC12_getMemResult(ADC_INST, ADC_CHANNEL);
}

/**
 * @brief ADC原始值转换为电压 (V)
 */
static float ADC_ToVoltage(uint16_t raw)
{
    /* 错误经验#1: ADC_RESOLUTION为常量4096，无除零风险 */
    return (float)raw * ADC_VREF / (float)ADC_RESOLUTION;
}

/* ══════════════════════════════════════════════════════════════
 *  电压滤波 (滑动平均)
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 滑动平均滤波器
 * @param new_sample 新采样值
 * @return 滤波后的值
 */
static float Voltage_Filter(float new_sample)
{
    g_voltage_buf[g_voltage_idx] = new_sample;
    g_voltage_idx = (g_voltage_idx + 1) % VOLTAGE_FILTER_SIZE;

    float sum = 0.0f;
    for (uint8_t i = 0; i < VOLTAGE_FILTER_SIZE; i++) {
        sum += g_voltage_buf[i];
    }
    /* 错误经验#1: VOLTAGE_FILTER_SIZE为常量8，无除零风险 */
    return sum / (float)VOLTAGE_FILTER_SIZE;
}

/* ══════════════════════════════════════════════════════════════
 *  SOC估算 (查表+线性插值)
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 根据电压查SOC (线性插值)
 * @param cell_voltage 单节电压 (V)
 * @return SOC百分比 (0~100)
 */
static float SOC_FromVoltage(float cell_voltage)
{
    /* 边界检查 */
    if (cell_voltage >= g_ocv_table[0].voltage) {
        return 100.0f;
    }
    if (cell_voltage <= g_ocv_table[OCV_TABLE_SIZE - 1].voltage) {
        return 0.0f;
    }

    /* 查找插值区间 */
    for (uint8_t i = 0; i < OCV_TABLE_SIZE - 1; i++) {
        if (cell_voltage >= g_ocv_table[i + 1].voltage) {
            float v_high = g_ocv_table[i].voltage;
            float v_low  = g_ocv_table[i + 1].voltage;
            float soc_high = g_ocv_table[i].soc;
            float soc_low  = g_ocv_table[i + 1].soc;

            /* 线性插值 */
            /* 错误经验#25: 防止两电压点相同导致除零 */
            float v_range = v_high - v_low;
            if (fabsf(v_range) < 1e-6f) {
                return soc_high;
            }
            float ratio = (cell_voltage - v_low) / v_range;
            return soc_low + ratio * (soc_high - soc_low);
        }
    }

    return 0.0f;
}

/* ══════════════════════════════════════════════════════════════
 *  充电检测
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 检测充电状态 (PA23引脚)
 */
static void Charging_Detect(void)
{
    g_is_charging = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_23) ? 1 : 0;
}

/* ══════════════════════════════════════════════════════════════
 *  低电压保护
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 检查电压并执行保护动作
 */
static void Protection_Check(void)
{
    /* 充电时不执行放电保护 */
    if (g_is_charging) {
        g_low_warn = 0;
        g_low_cutoff = 0;
        return;
    }

    /* 低电压切断 */
    if (g_cell_voltage <= VOLTAGE_LOW_CUTOFF) {
        g_low_cutoff = 1;
        g_load_enabled = 0;
        /* 关闭负载 (PA22低电平) */
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_22);
    }
    /* 低电量警告 */
    else if (g_cell_voltage <= VOLTAGE_LOW_WARN) {
        g_low_warn = 1;
        g_low_cutoff = 0;
    }
    /* 电压正常 */
    else {
        g_low_warn = 0;
        g_low_cutoff = 0;
        g_load_enabled = 1;
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_22);
    }

    /* LED指示 */
    if (g_low_cutoff) {
        /* 红灯闪烁 */
        if ((g_sys_tick / 250) & 1) {
            DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_26);
        } else {
            DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_26);
        }
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_24);
    } else if (g_low_warn) {
        /* 红灯常亮 */
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_26);
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_24);
    } else {
        /* 绿灯常亮 */
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_26);
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_24);
    }
}

/* ══════════════════════════════════════════════════════════════
 *  电池采样主函数
 * ══════════════════════════════════════════════════════════════ */

/**
 * @brief 执行一次电池参数采样
 */
static void Battery_Sample(void)
{
    /* 1. 读取ADC */
    uint16_t raw = ADC_ReadRaw();
    float v_adc = ADC_ToVoltage(raw);

    /* 2. 反算电池电压 (考虑分压比) */
    /* 错误经验#1: DIVIDER_RATIO为常量(0.5)，无除零风险 */
    float v_batt = v_adc / DIVIDER_RATIO;

    /* 3. 滤波 */
    g_batt_voltage = Voltage_Filter(v_batt);

    /* 4. 计算单节电压 */
    g_cell_voltage = g_batt_voltage / (float)BATTERY_CELLS;

    /* 5. 估算SOC */
    g_soc_percent = SOC_FromVoltage(g_cell_voltage);

    /* 6. 估算功率 (简化: 无电流传感器时用估计值) */
    /* 实际应用中应使用INA219读取电流 */
    g_current_mA = 0.0f;   /* 需接入电流传感器 */
    g_power_mW = g_batt_voltage * g_current_mA;

    /* 7. 累计消耗电量 (库仑计数简化版) */
    /* consumed_mAh += current_mA * (sample_interval_ms / 3600000) */
    if (g_current_mA > 0.0f) {
        g_consumed_mah += g_current_mA * (float)SAMPLE_INTERVAL_MS / 3600000.0f;
    }

    /* 8. 充电检测 */
    Charging_Detect();

    /* 9. 保护检查 */
    Protection_Check();
}

/* ══════════════════════════════════════════════════════════════
 *  OLED显示
 * ══════════════════════════════════════════════════════════════ */

static void OLED_UpdateDisplay(void)
{
    OLED_Clear();

    /* 第1行: 电压和电量 */
    OLED_ShowString(0, 0, (char *)"V:", 12, 1);
    OLED_ShowFloat(14, 0, g_batt_voltage, 1, 2, 12, 1);
    OLED_ShowString(54, 0, (char *)"V", 12, 1);
    OLED_ShowString(66, 0, (char *)"SOC:", 12, 1);
    OLED_ShowNum(90, 0, (uint32_t)g_soc_percent, 3, 12, 1);
    OLED_ShowString(114, 0, (char *)"%", 12, 1);

    /* 第2行: 单节电压 */
    OLED_ShowString(0, 16, (char *)"Cell:", 12, 1);
    OLED_ShowFloat(42, 16, g_cell_voltage, 1, 3, 12, 1);
    OLED_ShowString(84, 16, (char *)"V", 12, 1);

    /* 第3行: 状态 */
    OLED_ShowString(0, 32, (char *)"St:", 12, 1);
    if (g_is_charging) {
        OLED_ShowString(24, 32, (char *)"CHARGING  ", 12, 1);
    } else if (g_low_cutoff) {
        OLED_ShowString(24, 32, (char *)"CUTOFF!   ", 12, 1);
    } else if (g_low_warn) {
        OLED_ShowString(24, 32, (char *)"LOW WARN  ", 12, 1);
    } else {
        OLED_ShowString(24, 32, (char *)"OK        ", 12, 1);
    }

    /* 第4行: 电量条 */
    OLED_ShowString(0, 48, (char *)"[", 12, 1);
    /* 简易进度条 (12像素宽, 100像素长) */
    uint8_t bar_width = (uint8_t)(g_soc_percent);   /* 0~100 */
    for (uint8_t i = 0; i < bar_width; i++) {
        OLED_DrawPoint(8 + i, 52, 1);
        OLED_DrawPoint(8 + i, 54, 1);
        OLED_DrawPoint(8 + i, 56, 1);
    }
    OLED_DrawLine(8, 50, 108, 50, 1);   /* 上边框 */
    OLED_DrawLine(8, 58, 108, 58, 1);   /* 下边框 */
    OLED_DrawLine(108, 50, 108, 58, 1); /* 右边框 */
    OLED_ShowString(112, 48, (char *)"]", 12, 1);

    OLED_Refresh();
}

/* ══════════════════════════════════════════════════════════════
 *  蓝牙数据发送
 * ══════════════════════════════════════════════════════════════ */

static void BT_SendData(void)
{
    if (!BT_HC05_IsConnected()) return;

    char msg[80];
    snprintf(msg, sizeof(msg), "V:%.2f SOC:%.0f%% Cell:%.3f %s\r\n",
             g_batt_voltage, g_soc_percent, g_cell_voltage,
             g_is_charging ? "CHG" :
             g_low_cutoff ? "CUT" :
             g_low_warn ? "LOW" : "OK");
    BT_HC05_SendString(msg);
}

/* ══════════════════════════════════════════════════════════════
 *  系统初始化
 * ══════════════════════════════════════════════════════════════ */

static void System_Init(void)
{
    SYSCFG_DL_init();

    /* 初始化I2C总线 */
    I2C_Bus_Init(&g_i2c_bus, I2C_0_INST);

    /* 初始化OLED */
    OLED_Init(I2C_0_INST);
    OLED_Clear();
    OLED_ShowString(10, 24, (char *)"Battery Mgr", 16, 1);
    OLED_Refresh();

    /* 初始化蓝牙 */
    BT_HC05_Init(&(BT_HC05_Config){
        .uart = UART_1_INST,
        .state_port = PIN_BT_EN_PORT,
        .state_pin  = PIN_BT_EN_PIN,
        .baudrate   = 9600
    });

    /* 配置ADC引脚 (PA25) — 模拟输入 */
    /* 注意: ADC引脚配置由SysConfig完成，此处仅为说明 */

    /* 配置GPIO */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_22);  /* 负载控制 */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_24);  /* 绿LED */
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_26);  /* 红LED */
    DL_GPIO_initDigitalInput(GPIOA, DL_GPIO_PIN_23);   /* 充电检测 */

    /* 默认打开负载 */
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_22);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_24);    /* 绿灯亮 */

    /* 初始化电压滤波缓冲 */
    for (uint8_t i = 0; i < VOLTAGE_FILTER_SIZE; i++) {
        g_voltage_buf[i] = 4.0f;   /* 默认值 */
    }

    delay_cycles(8000000);  /* 等待稳定 (~250ms) */
}

/* ══════════════════════════════════════════════════════════════
 *  主函数
 * ══════════════════════════════════════════════════════════════ */

int main(void)
{
    System_Init();

    while (1) {
        /* 1. 定时采样电池参数 */
        if ((g_sys_tick - g_last_sample_tick) >= SAMPLE_INTERVAL_MS) {
            g_last_sample_tick = g_sys_tick;
            Battery_Sample();
        }

        /* 2. 定时刷新OLED */
        if ((g_sys_tick - g_last_oled_tick) >= OLED_REFRESH_MS) {
            g_last_oled_tick = g_sys_tick;
            OLED_UpdateDisplay();
        }

        /* 3. 定时蓝牙发送 */
        if ((g_sys_tick - g_last_bt_tick) >= BT_SEND_INTERVAL_MS) {
            g_last_bt_tick = g_sys_tick;
            BT_SendData();
        }

        /* 4. 主循环延时 ~10ms */
        delay_cycles(320000);
        g_sys_tick += 10;
    }
}
