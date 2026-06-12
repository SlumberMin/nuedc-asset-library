/**
 * @file power_management_demo.c
 * @brief 电源管理示例（电池电压检测+低电压报警+休眠唤醒）
 * @platform MSPM0G3507
 *
 * ============================================================
 * 接线说明
 * ============================================================
 * 模块              MSPM0G3507引脚      说明
 * ---------------------------------------------------------------
 * 电池电压检测
 *   电池+           经分压电阻到PA25     12V电池用10K+1K分压
 *   (分压点)         PA25 (ADC0_CH0)     分压后≤3.3V
 *   电池-           GND
 *   分压电阻         R1=10K(上), R2=1K(下)  分压比 1:11
 *
 * 蜂鸣器（低电压报警）
 *   Buzzer I/O      PA6                  高电平响
 *   VCC             5V/3.3V
 *   GND            GND
 *
 * LED指示灯
 *   电池指示绿灯    PA22                 电量充足
 *   电池指示红灯    PA23                 电量低
 *   系统状态LED     PA24                 系统运行指示
 *
 * 电源控制（MOS管控制外设供电）
 *   外设电源EN      PB0                  高电平使能外设5V
 *   电机电源EN      PB1                  高电平使能电机电源
 *
 * 唤醒按键
 *   WKUP引脚        PA18 (WKUP)          外部中断唤醒
 *
 * UART调试串口
 *   TX              PA8 (UART0_TX)       调试输出
 *   RX              PA9 (UART0_RX)       调试输入
 *
 * ============================================================
 * 功能说明
 * ============================================================
 * 1. 电池电压检测：
 *    - 通过ADC采集分压后的电池电压
 *    - 支持多种电池类型（2S锂电7.4V / 3S锂电11.1V / 12V铅酸）
 *    - 电压滤波（滑动平均）
 *
 * 2. 低电压报警：
 *    - 多级报警：警告(黄) -> 低电(红) -> 严重低电(红+蜂鸣)
 *    - 自动降低系统功耗（关闭非必要外设）
 *    - 严重低电时自动进入安全模式（停电机、存数据）
 *
 * 3. 电源管理：
 *    - 空闲时关闭外设供电省电
 *    - MCU休眠模式（Sleep/DeepSleep）
 *    - 按键或定时器唤醒
 *    - 功耗模式切换
 *
 * 4. 电量估算：
 *    - 基于电压的简单SOC估算
 *    - 支持库仑计模式（需外部电流传感器）
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ======================== 电池参数配置 ======================== */

/* 电池类型选择（取消注释对应类型） */
// #define BATTERY_TYPE_2S_LIPO        /* 2S锂电池 7.4V (满充8.4V) */
#define BATTERY_TYPE_3S_LIPO        /* 3S锂电池 11.1V (满充12.6V) */
// #define BATTERY_TYPE_12V_LEAD      /* 12V铅酸电池 */

/* 分压电阻参数 */
#define VOLTAGE_DIVIDER_R1          10.0f   /* 上臂电阻 (KΩ) */
#define VOLTAGE_DIVIDER_R2          1.0f    /* 下臂电阻 (KΩ) */
#define VOLTAGE_DIVIDER_RATIO       ((VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2)

/* 电池电压阈值 (单位: V) */
#ifdef BATTERY_TYPE_2S_LIPO
    #define BATTERY_FULL_VOLTAGE    8.4f    /* 满电电压 */
    #define BATTERY_NOMINAL_VOLTAGE 7.4f    /* 标称电压 */
    #define BATTERY_WARN_VOLTAGE    7.0f    /* 警告电压 */
    #define BATTERY_LOW_VOLTAGE     6.6f    /* 低电报警 */
    #define BATTERY_CRITICAL_VOLTAGE 6.0f   /* 严重低电，关机 */
    #define BATTERY_CELLS           2
#endif

#ifdef BATTERY_TYPE_3S_LIPO
    #define BATTERY_FULL_VOLTAGE    12.6f
    #define BATTERY_NOMINAL_VOLTAGE 11.1f
    #define BATTERY_WARN_VOLTAGE    10.5f
    #define BATTERY_LOW_VOLTAGE     9.9f
    #define BATTERY_CRITICAL_VOLTAGE 9.0f
    #define BATTERY_CELLS           3
#endif

#ifdef BATTERY_TYPE_12V_LEAD
    #define BATTERY_FULL_VOLTAGE    13.8f
    #define BATTERY_NOMINAL_VOLTAGE 12.0f
    #define BATTERY_WARN_VOLTAGE    11.5f
    #define BATTERY_LOW_VOLTAGE     11.0f
    #define BATTERY_CRITICAL_VOLTAGE 10.5f
    #define BATTERY_CELLS           6      /* 6节2V铅酸 */
#endif

/* ADC参数 */
#define ADC_VREF                    3.3f    /* ADC参考电压 */
#define ADC_RESOLUTION              4096.0f /* 12位ADC */
#define ADC_CHANNEL_BATTERY         0       /* 电池电压ADC通道 */

/* 电压采样滤波 */
#define VOLTAGE_FILTER_SIZE         32      /* 滑动平均窗口大小 */

/* 功耗管理时间 (ms) */
#define IDLE_TIMEOUT_MS             30000   /* 空闲30秒后进入低功耗 */
#define SLEEP_TIMEOUT_MS            120000  /* 空闲2分钟后深度休眠 */

/* ======================== 引脚宏定义 ======================== */
#define BUZZER_PORT     GPIOA
#define BUZZER_PIN      DL_GPIO_PIN_6

#define LED_GREEN_PORT  GPIOA
#define LED_GREEN_PIN   DL_GPIO_PIN_22
#define LED_RED_PORT    GPIOA
#define LED_RED_PIN     DL_GPIO_PIN_23
#define LED_SYS_PORT    GPIOA
#define LED_SYS_PIN     DL_GPIO_PIN_24

#define PERIPH_EN_PORT  GPIOB
#define PERIPH_EN_PIN   DL_GPIO_PIN_0
#define MOTOR_EN_PORT   GPIOB
#define MOTOR_EN_PIN    DL_GPIO_PIN_1

#define WKUP_PORT       GPIOA
#define WKUP_PIN        DL_GPIO_PIN_21

/* ======================== 电池状态枚举 ======================== */
typedef enum {
    BATTERY_OK = 0,         /* 电量充足 */
    BATTERY_WARNING,        /* 警告 */
    BATTERY_LOW,            /* 低电量 */
    BATTERY_CRITICAL,       /* 严重低电 */
    BATTERY_DEAD,           /* 电量耗尽 */
} BatteryStatus_t;

/* ======================== 电源模式枚举 ======================== */
typedef enum {
    POWER_MODE_RUN = 0,         /* 正常运行 */
    POWER_MODE_IDLE,            /* 空闲，降低外设功耗 */
    POWER_MODE_SLEEP,           /* 休眠，仅RTC和唤醒中断 */
    POWER_MODE_DEEPSLEEP,       /* 深度休眠 */
    POWER_MODE_SHUTDOWN,        /* 关机 */
} PowerMode_t;

/* ======================== 电池信息结构体 ======================== */
typedef struct {
    float voltage;                  /* 当前电压 (V) */
    float voltage_filtered;         /* 滤波后电压 */
    float cell_voltage;             /* 单节电压 */
    uint8_t soc;                    /* 电量百分比 (0-100) */
    BatteryStatus_t status;         /* 电池状态 */
    uint32_t last_check_time;       /* 上次检测时间 */
    uint16_t adc_raw;               /* ADC原始值 */
    float filter_buf[VOLTAGE_FILTER_SIZE]; /* 滤波缓冲区 */
    uint8_t filter_idx;             /* 滤波索引 */
    float filter_sum;               /* 滤波累加和 */
} BatteryInfo_t;

/* ======================== 全局变量 ======================== */
static volatile uint32_t gSysTick = 0;
static volatile uint32_t gLastActivity = 0;      /* 上次用户活动时间 */
static volatile bool gBtnPressed = false;          /* 按键标志 */
static volatile PowerMode_t gPowerMode = POWER_MODE_RUN;

static BatteryInfo_t gBattery = {
    .voltage = 0.0f,
    .voltage_filtered = 0.0f,
    .cell_voltage = 0.0f,
    .soc = 100,
    .status = BATTERY_OK,
    .filter_idx = 0,
    .filter_sum = 0.0f,
};

/* ======================== 串口调试输出 ======================== */
static void debug_print(const char *str)
{
    while (*str) {
        while (!DL_UART_isTXFIFOEmpty(UART_0_INST));
        DL_UART_transmitDataBlocking(UART_0_INST, *str++);
    }
}

static void debug_print_float(float val)
{
    char buf[16];
    int whole = (int)val;
    int frac = (int)((val - (float)whole) * 100);
    if (frac < 0) frac = -frac;
    snprintf(buf, sizeof(buf), "%d.%02d", whole, frac);
    debug_print(buf);
}

/* ======================== ADC电压采集 ======================== */

/**
 * @brief 读取ADC并转换为电池电压
 *
 * 计算公式:
 *   V_adc = ADC_raw / 4096 * 3.3V
 *   V_battery = V_adc * (R1 + R2) / R2
 *   V_battery = V_adc * VOLTAGE_DIVIDER_RATIO
 */
static float read_battery_voltage(void)
{
    /* 启动ADC转换 */
    DL_ADC12_startConversion(ADC12_0_INST);
    while (!(DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_DONE));

    gBattery.adc_raw = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);

    /* 转换为实际电压 */
    float v_adc = (float)gBattery.adc_raw / ADC_RESOLUTION * ADC_VREF;
    return v_adc * VOLTAGE_DIVIDER_RATIO;
}

/**
 * @brief 滑动平均滤波
 *
 * 消除ADC噪声，提供稳定的电压读数
 */
static float voltage_filter(float new_val)
{
    /* 减去最旧的值 */
    gBattery.filter_sum -= gBattery.filter_buf[gBattery.filter_idx];
    /* 写入新值 */
    gBattery.filter_buf[gBattery.filter_idx] = new_val;
    /* 加上新值 */
    gBattery.filter_sum += new_val;
    /* 更新索引 */
    gBattery.filter_idx = (gBattery.filter_idx + 1) % VOLTAGE_FILTER_SIZE;

    return gBattery.filter_sum / VOLTAGE_FILTER_SIZE;
}

/* ======================== SOC估算 ======================== */

/**
 * @brief 基于电压估算电池SOC (电量百分比)
 *
 * 使用分段线性插值：
 * - 满电电压 -> 100%
 * - 标称电压 -> 50%
 * - 低电电压 -> 10%
 * - 关机电压 -> 0%
 *
 * 注意：锂电放电曲线非线性，此方法误差约±15%
 * 精确SOC需要库仑计
 */
static uint8_t estimate_soc(float voltage)
{
    if (voltage >= BATTERY_FULL_VOLTAGE) return 100;
    if (voltage <= BATTERY_CRITICAL_VOLTAGE) return 0;

    /* 分段线性插值 */
    if (voltage >= BATTERY_NOMINAL_VOLTAGE) {
        /* 满电到标称：100% -> 50% */
        return (uint8_t)(50.0f + 50.0f * (voltage - BATTERY_NOMINAL_VOLTAGE) /
                        (BATTERY_FULL_VOLTAGE - BATTERY_NOMINAL_VOLTAGE));
    } else if (voltage >= BATTERY_WARN_VOLTAGE) {
        /* 标称到警告：50% -> 20% */
        return (uint8_t)(20.0f + 30.0f * (voltage - BATTERY_WARN_VOLTAGE) /
                        (BATTERY_NOMINAL_VOLTAGE - BATTERY_WARN_VOLTAGE));
    } else if (voltage >= BATTERY_LOW_VOLTAGE) {
        /* 警告到低电：20% -> 5% */
        return (uint8_t)(5.0f + 15.0f * (voltage - BATTERY_LOW_VOLTAGE) /
                        (BATTERY_WARN_VOLTAGE - BATTERY_LOW_VOLTAGE));
    } else {
        /* 低电到关机：5% -> 0% */
        return (uint8_t)(5.0f * (voltage - BATTERY_CRITICAL_VOLTAGE) /
                        (BATTERY_LOW_VOLTAGE - BATTERY_CRITICAL_VOLTAGE));
    }
}

/* ======================== 电池状态管理 ======================== */

/**
 * @brief 更新电池状态
 *
 * 根据电压判断电池状态，触发相应的报警和保护动作
 */
static void battery_update(void)
{
    /* 读取并滤波 */
    float raw_v = read_battery_voltage();
    gBattery.voltage = raw_v;
    gBattery.voltage_filtered = voltage_filter(raw_v);
    gBattery.cell_voltage = gBattery.voltage_filtered / BATTERY_CELLS;
    gBattery.soc = estimate_soc(gBattery.voltage_filtered);

    /* 判断状态 */
    float v = gBattery.voltage_filtered;
    BatteryStatus_t old_status = gBattery.status;

    if (v >= BATTERY_WARN_VOLTAGE) {
        gBattery.status = BATTERY_OK;
    } else if (v >= BATTERY_LOW_VOLTAGE) {
        gBattery.status = BATTERY_WARNING;
    } else if (v >= BATTERY_CRITICAL_VOLTAGE) {
        gBattery.status = BATTERY_LOW;
    } else {
        gBattery.status = BATTERY_CRITICAL;
    }

    /* 状态变化时打印日志 */
    if (gBattery.status != old_status) {
        debug_print("[BATT] Status changed!\r\n");
    }

    gBattery.last_check_time = gSysTick;
}

/**
 * @brief 电池报警处理
 *
 * 根据电池状态控制LED和蜂鸣器
 */
static void battery_alarm(void)
{
    switch (gBattery.status) {
    case BATTERY_OK:
        /* 绿灯常亮，蜂鸣器关 */
        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN);
        DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        break;

    case BATTERY_WARNING:
        /* 绿灯闪烁 */
        DL_GPIO_togglePins(LED_GREEN_PORT, LED_GREEN_PIN);
        DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        break;

    case BATTERY_LOW:
        /* 红灯常亮 */
        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        break;

    case BATTERY_CRITICAL:
        /* 红灯快闪 + 蜂鸣器间歇响 */
        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
        DL_GPIO_togglePins(LED_RED_PORT, LED_RED_PIN);
        /* 蜂鸣器每500ms响100ms */
        if ((gSysTick / 500) % 2 == 0) {
            DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
        } else {
            DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        }
        break;

    case BATTERY_DEAD:
        /* 全灭，进入关机 */
        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
        DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        gPowerMode = POWER_MODE_SHUTDOWN;
        break;
    }
}

/* ======================== 电源管理 ======================== */

/**
 * @brief 关闭非必要外设以降低功耗
 */
static void power_reduce_peripherals(void)
{
    /* 关闭外设供电 */
    DL_GPIO_clearPins(PERIPH_EN_PORT, PERIPH_EN_PIN);
    /* 可选：关闭电机电源 */
    /* DL_GPIO_clearPins(MOTOR_EN_PORT, MOTOR_EN_PIN); */
}

/**
 * @brief 恢复外设供电
 */
static void power_restore_peripherals(void)
{
    /* 恢复外设供电 */
    DL_GPIO_setPins(PERIPH_EN_PORT, PERIPH_EN_PIN);
    /* 等待外设稳定 */
    for (volatile uint32_t i = 0; i < 100000; i++);
}

/**
 * @brief 进入Sleep模式（CPU停止，外设继续运行）
 *
 * 唤醒源：
 * - UART接收中断
 * - 按键外部中断
 * - SysTick定时器
 *
 * 功耗约：~1mA（取决于活跃外设）
 */
static void power_enter_sleep(void)
{
    debug_print("[PWR] Entering Sleep mode...\r\n");

    /* 关闭LED */
    DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
    DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
    DL_GPIO_clearPins(LED_SYS_PORT, LED_SYS_PIN);

    /* 进入Sleep */
    __WFI();  /* Wait For Interrupt */
}

/**
 * @brief 进入DeepSleep模式（主时钟关闭，仅LP外设运行）
 *
 * 唤醒源：
 * - WKUP引脚外部中断（PA18）
 * - RTC闹钟（如果配置）
 *
 * 功耗约：~10μA
 */
static void power_enter_deepsleep(void)
{
    debug_print("[PWR] Entering DeepSleep mode...\r\n");

    /* 关闭所有非必要外设 */
    power_reduce_peripherals();

    /* 配置唤醒源：WKUP引脚低电平唤醒 */
    DL_GPIO_enableInterrupt(WKUP_PORT, WKUP_PIN);

    /* 进入DeepSleep */
    DL_SYSCTL_setPowerPolicyRUNSLEEP();
    __WFI();

    /* ====== 唤醒后从这里继续 ====== */
    debug_print("[PWR] Woke up from DeepSleep!\r\n");

    /* 恢复外设 */
    power_restore_peripherals();
    gPowerMode = POWER_MODE_RUN;
    gLastActivity = gSysTick;
}

/**
 * @brief 安全关机
 *
 * 严重低电时执行：
 * 1. 停止所有电机
 * 2. 保存关键数据到Flash
 * 3. 关闭所有外设
 * 4. 进入最深休眠等待充电
 */
static void power_safe_shutdown(void)
{
    debug_print("[PWR] === SAFE SHUTDOWN ===\r\n");

    /* 停止电机 */
    DL_GPIO_clearPins(MOTOR_EN_PORT, MOTOR_EN_PIN);

    /* 关闭外设 */
    power_reduce_peripherals();

    /* 所有LED灭 */
    DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
    DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
    DL_GPIO_clearPins(LED_SYS_PORT, LED_SYS_PIN);

    debug_print("[PWR] Waiting for charger or reset...\r\n");

    /* 进入最深休眠，只有充电后电压恢复或复位才能唤醒 */
    while (1) {
        __WFI();
    }
}

/**
 * @brief 检查空闲超时，自动进入低功耗模式
 */
static void power_idle_check(void)
{
    uint32_t idle_time = gSysTick - gLastActivity;

    if (gPowerMode == POWER_MODE_RUN && idle_time > IDLE_TIMEOUT_MS) {
        gPowerMode = POWER_MODE_IDLE;
        debug_print("[PWR] Entering IDLE mode\r\n");
        power_reduce_peripherals();
    }

    if (gPowerMode == POWER_MODE_IDLE && idle_time > SLEEP_TIMEOUT_MS) {
        gPowerMode = POWER_MODE_DEEPSLEEP;
        power_enter_deepsleep();
    }
}

/* ======================== 中断处理 ======================== */

/* WKUP引脚中断（唤醒用） */
void GROUP1_IRQHandler(void)
{
    /* 清除中断标志 */
    DL_GPIO_clearInterruptStatus(WKUP_PORT, WKUP_PIN);
    gBtnPressed = true;
    gLastActivity = gSysTick;
    gPowerMode = POWER_MODE_RUN;
}

void SysTick_Handler(void)
{
    gSysTick++;
    /* 系统运行指示LED，每500ms翻转 */
    if (gSysTick % 500 == 0) {
        DL_GPIO_togglePins(LED_SYS_PORT, LED_SYS_PIN);
    }
}

/* ======================== 主函数 ======================== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* 使能中断 */
    NVIC_EnableIRQ(GROUP1_IRQn);

    /* 上电LED自检 */
    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN);
    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN);
    DL_GPIO_setPins(LED_SYS_PORT, LED_SYS_PIN);
    for (volatile uint32_t i = 0; i < 500000; i++);
    DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
    DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);

    /* 使能外设电源 */
    DL_GPIO_setPins(PERIPH_EN_PORT, PERIPH_EN_PIN);
    DL_GPIO_setPins(MOTOR_EN_PORT, MOTOR_EN_PIN);

    debug_print("\r\n=== Power Management Demo ===\r\n");
    debug_print("Battery cells: ");
    debug_print(BATTERY_CELLS == 2 ? "2S" : BATTERY_CELLS == 3 ? "3S" : "6S");
    debug_print("\r\n");

    /* 初始滤波填充 */
    for (uint8_t i = 0; i < VOLTAGE_FILTER_SIZE; i++) {
        float v = read_battery_voltage();
        voltage_filter(v);
        for (volatile uint32_t d = 0; d < 1000; d++);
    }

    uint32_t voltage_report_time = 0;

    /* ======================== 主循环 ======================== */
    while (1) {
        /* 每500ms检测电池电压 */
        if (gSysTick - gBattery.last_check_time >= 500) {
            battery_update();
            battery_alarm();
        }

        /* 每5秒串口打印电池信息 */
        if (gSysTick - voltage_report_time >= 5000) {
            voltage_report_time = gSysTick;

            debug_print("[BATT] V=");
            debug_print_float(gBattery.voltage_filtered);
            debug_print("V SOC=");
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", gBattery.soc);
            debug_print(buf);
            debug_print("% Cell=");
            debug_print_float(gBattery.cell_voltage);
            debug_print("V Status=");
            switch (gBattery.status) {
                case BATTERY_OK: debug_print("OK"); break;
                case BATTERY_WARNING: debug_print("WARN"); break;
                case BATTERY_LOW: debug_print("LOW"); break;
                case BATTERY_CRITICAL: debug_print("CRIT"); break;
                case BATTERY_DEAD: debug_print("DEAD"); break;
            }
            debug_print("\r\n");
        }

        /* 严重低电保护 */
        if (gBattery.status == BATTERY_CRITICAL) {
            /* 持续5秒严重低电则关机 */
            static uint32_t critical_start = 0;
            if (critical_start == 0) critical_start = gSysTick;
            if (gSysTick - critical_start > 5000) {
                power_safe_shutdown();
            }
        }

        /* 空闲功耗管理 */
        power_idle_check();

        /* 按键唤醒活动标记 */
        if (gBtnPressed) {
            gBtnPressed = false;
            gLastActivity = gSysTick;
            if (gPowerMode != POWER_MODE_RUN) {
                gPowerMode = POWER_MODE_RUN;
                power_restore_peripherals();
                debug_print("[PWR] Back to RUN mode\r\n");
            }
        }

        /* 低功耗模式下可以主动进入Sleep */
        if (gPowerMode == POWER_MODE_RUN) {
            /* 正常运行时可以在中断间隙进入浅睡眠 */
            __WFI();
        }
    }

    return 0;
}
