/**
 * @file temperature_alarm.c
 * @brief 温度报警器 - MSPM0G3507系统集成示例
 *
 * 功能：DS18B20温度传感器 + 可设置阈值 + 蜂鸣器报警 + LED指示 + UART输出
 * 硬件：MSPM0G3507 + DS18B20(单总线) + 蜂鸣器(PB15) + LED×3 + 按键×2 + OLED
 *
 * 接线：
 *   DS18B20 DQ   -> PA10 (GPIO, 需4.7K上拉)
 *   LED-Green     -> PB14 (正常指示)
 *   LED-Red       -> PB15 (报警指示)
 *   蜂鸣器        -> PA27 (PWM/TIM输出)
 *   按键-温度升   -> PA12 (低有效)
 *   按键-温度降   -> PA13 (低有效)
 *   OLED SDA      -> PA0  (I2C0)
 *   OLED SCL      -> PA1  (I2C0)
 *
 * 特性：
 *   - 高低温阈值可独立设置
 *   - 支持°C / °F 切换
 *   - 蜂鸣器频率随温度变化（温度越高鸣叫越急）
 *   - 历史最高/最低温度记录
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== 引脚定义 ========== */
#define DS18B20_PORT    GPIOA
#define DS18B20_PIN     DL_GPIO_PIN_10
#define LED_GREEN_PORT  GPIOB
#define LED_GREEN_PIN   DL_GPIO_PIN_14
#define LED_RED_PORT    GPIOB
#define LED_RED_PIN     DL_GPIO_PIN_15
#define BUZZER_PORT     GPIOA
#define BUZZER_PIN      DL_GPIO_PIN_27
#define BTN_UP_PORT     GPIOA
#define BTN_UP_PIN      DL_GPIO_PIN_12
#define BTN_DOWN_PORT   GPIOA
#define BTN_DOWN_PIN    DL_GPIO_PIN_13

/* ========== 报警阈值（默认） ========== */
#define DEFAULT_HIGH_THRESH  40.0f   /* 高温阈值 °C */
#define DEFAULT_LOW_THRESH   5.0f    /* 低温阈值 °C */
#define TEMP_HYSTERESIS      1.0f    /* 回差温度 °C */

/* ========== 全局变量 ========== */
static volatile uint32_t gTickMs = 0;
static float gTemperature   = 0.0f;   /* 当前温度 */
static float gHighThreshold = DEFAULT_HIGH_THRESH;
static float gLowThreshold  = DEFAULT_LOW_THRESH;
static float gTempMax       = -100.0f;
static float gTempMin       = 200.0f;
static bool  gUseFahrenheit = false;   /* false=°C, true=°F */
static uint8_t gAlarmState  = 0;       /* 0=正常, 1=高温, 2=低温 */

/* =================================================================
 * 基础延时（DS18B20需要精确微秒延时）
 * ================================================================= */

void SysTick_Handler(void) { gTickMs++; }

/**
 * @brief 微秒延时（32MHz时钟）
 * @note 使用循环计数，精度约±1us
 */
static void delay_us(uint32_t us) {
    /* 32MHz: 1us ≈ 32 cycles, 减去开销取28 */
    volatile uint32_t cnt = us * 28;
    while (cnt--);
}

static void delay_ms(uint32_t ms) {
    uint32_t s = gTickMs;
    while ((gTickMs - s) < ms);
}

/* =================================================================
 * DS18B20单总线驱动
 * ================================================================= */

/**
 * @brief 设置DQ引脚为输出模式
 */
static void DS18B20_SetOutput(void) {
    DL_GPIO_initDigitalOutput(DS18B20_PIN);
}

/**
 * @brief 设置DQ引脚为输入模式
 */
static void DS18B20_SetInput(void) {
    DL_GPIO_initDigitalInputFeatures(DS18B20_PIN,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/**
 * @brief 拉低DQ
 */
static void DS18B20_Low(void) {
    DL_GPIO_clearPins(DS18B20_PORT, DS18B20_PIN);
}

/**
 * @brief 释放DQ（由上拉电阻拉高）
 */
static void DS18B20_High(void) {
    DS18B20_SetInput();
}

/**
 * @brief 读取DQ电平
 */
static uint8_t DS18B20_Read(void) {
    return DL_GPIO_readPins(DS18B20_PORT, DS18B20_PIN) ? 1 : 0;
}

/**
 * @brief DS18B20初始化（发送复位脉冲并检测存在脉冲）
 * @return true=传感器存在
 */
static bool DS18B20_Reset(void) {
    uint8_t presence;

    DS18B20_SetOutput();
    DS18B20_Low();
    delay_us(480);      /* 拉低480us */
    DS18B20_High();     /* 释放 */
    delay_us(60);       /* 等待60us */
    presence = DS18B20_Read();  /* 读取存在脉冲（低有效） */
    delay_us(420);      /* 等待时隙结束 */

    return (presence == 0);  /* 0=有设备应答 */
}

/**
 * @brief 写一个位
 */
static void DS18B20_WriteBit(uint8_t bit) {
    DS18B20_SetOutput();
    DS18B20_Low();
    delay_us(2);

    if (bit) {
        DS18B20_High();
        delay_us(60);
    } else {
        delay_us(60);
        DS18B20_High();
        delay_us(2);
    }
}

/**
 * @brief 读一个位
 */
static uint8_t DS18B20_ReadBit(void) {
    uint8_t bit;

    DS18B20_SetOutput();
    DS18B20_Low();
    delay_us(2);
    DS18B20_High();
    delay_us(8);
    bit = DS18B20_Read();
    delay_us(50);

    return bit;
}

/**
 * @brief 写一个字节
 */
static void DS18B20_WriteByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        DS18B20_WriteBit(data & 0x01);
        data >>= 1;
    }
}

/**
 * @brief 读一个字节
 */
static uint8_t DS18B20_ReadByte(void) {
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (DS18B20_ReadBit()) {
            data |= (1 << i);
        }
    }
    return data;
}

/**
 * @brief 启动温度转换
 */
static void DS18B20_StartConversion(void) {
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC);  /* Skip ROM（单设备） */
    DS18B20_WriteByte(0x44);  /* 启动转换 */
}

/**
 * @brief 读取温度值
 * @return 温度值（°C），错误返回 -999.0
 */
static float DS18B20_ReadTemperature(void) {
    uint8_t lsb, msb;
    int16_t raw;
    float temp;

    if (!DS18B20_Reset()) {
        return -999.0f;  /* 传感器未响应 */
    }

    DS18B20_WriteByte(0xCC);  /* Skip ROM */
    DS18B20_WriteByte(0xBE);  /* 读暂存器 */

    lsb = DS18B20_ReadByte();  /* 温度低位 */
    msb = DS18B20_ReadByte();  /* 温度高位 */

    raw = (msb << 8) | lsb;

    /* 12位分辨率：0.0625°C/LSB */
    temp = raw * 0.0625f;

    return temp;
}

/* =================================================================
 * 温度单位转换
 * ================================================================= */

static float CtoF(float c) {
    return c * 9.0f / 5.0f + 32.0f;
}

/* =================================================================
 * 蜂鸣器控制（通过GPIO模拟频率）
 * ================================================================= */

/**
 * @brief 蜂鸣器报警
 * @param freq_hz 频率（0=关闭）
 * @param duration_ms 持续时间
 */
static void Buzzer_Beep(uint16_t freq_hz, uint16_t duration_ms) {
    if (freq_hz == 0) {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        return;
    }

    uint32_t half_period_us = 500000 / freq_hz;
    uint32_t end_time = gTickMs + duration_ms;

    while (gTickMs < end_time) {
        DL_GPIO_togglePins(BUZZER_PORT, BUZZER_PIN);
        delay_us(half_period_us);
    }
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}

/* =================================================================
 * 报警判断
 * ================================================================= */

/**
 * @brief 检查温度并更新报警状态
 */
static void CheckAlarm(void) {
    if (gTemperature >= gHighThreshold) {
        gAlarmState = 1;  /* 高温报警 */
    } else if (gTemperature <= gLowThreshold) {
        gAlarmState = 2;  /* 低温报警 */
    } else if (gAlarmState == 1 && gTemperature < (gHighThreshold - TEMP_HYSTERESIS)) {
        gAlarmState = 0;  /* 高温恢复（回差） */
    } else if (gAlarmState == 2 && gTemperature > (gLowThreshold + TEMP_HYSTERESIS)) {
        gAlarmState = 0;  /* 低温恢复（回差） */
    }
}

/* =================================================================
 * 按键处理
 * ================================================================= */

/**
 * @brief 处理阈值调节按键
 */
static void HandleButtons(void) {
    static uint32_t last_btn = 0;
    if ((gTickMs - last_btn) < 200) return;  /* 消抖 */

    if (!DL_GPIO_readPins(BTN_UP_PORT, BTN_UP_PIN)) {
        gHighThreshold += 1.0f;
        if (gHighThreshold > 125.0f) gHighThreshold = 125.0f;
        last_btn = gTickMs;
        Buzzer_Beep(2000, 50);  /* 按键音 */
    }

    if (!DL_GPIO_readPins(BTN_DOWN_PORT, BTN_DOWN_PIN)) {
        gLowThreshold -= 1.0f;
        if (gLowThreshold < -55.0f) gLowThreshold = -55.0f;
        last_btn = gTickMs;
        Buzzer_Beep(1500, 50);
    }
}

/* =================================================================
 * UART输出
 * ================================================================= */

static void UART_Print(const char *str) {
    while (*str) {
        DL_UART_transmitData(UART0, *str++);
        while (!DL_UART_isTXEmpty(UART0));
    }
}

static void UART_PrintFloat(float val) {
    char buf[16];
    int whole = (int)val;
    int frac = (int)(fabsf(val - whole) * 100);
    snprintf(buf, sizeof(buf), "%d.%02d", whole, frac);
    UART_Print(buf);
}

/* =================================================================
 * 主函数
 * ================================================================= */
int main(void) {
    /* 系统初始化 */
    DL_SYSCFG_init();
    SysTick_Config(32000);  /* 1ms tick */

    /* GPIO */
    DL_GPIO_initDigitalInputFeatures(DS18B20_PIN,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalOutput(LED_GREEN_PIN);
    DL_GPIO_initDigitalOutput(LED_RED_PIN);
    DL_GPIO_initDigitalOutput(BUZZER_PIN);
    DL_GPIO_initDigitalInputFeatures(BTN_UP_PIN,
        DL_GPIO_RESISTOR_PULLUP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BTN_DOWN_PIN,
        DL_GPIO_RESISTOR_PULLUP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* UART */
    NVIC_EnableIRQ(UART0_IRQn);

    /* 开机提示 */
    UART_Print("\r\n=== Temperature Alarm System ===\r\n");
    UART_Print("DS18B20 + Buzzer + LED Alarm\r\n");

    /* 检测DS18B20 */
    if (!DS18B20_Reset()) {
        UART_Print("ERROR: DS18B20 not found!\r\n");
        /* LED全亮指示故障 */
        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN);
        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN);
        while (1);
    }
    UART_Print("DS18B20 detected OK\r\n");

    /* 启动首次转换 */
    DS18B20_StartConversion();

    uint32_t last_sample = 0;
    uint32_t last_report = 0;
    const uint32_t SAMPLE_INTERVAL = 1000;  /* 1秒采样 */
    const uint32_t REPORT_INTERVAL = 5000;  /* 5秒上报 */

    char uart_buf[64];

    /* ===== 主循环 ===== */
    while (1) {
        /* 按键处理 */
        HandleButtons();

        /* 定时采样温度 */
        if ((gTickMs - last_sample) >= SAMPLE_INTERVAL) {
            gTemperature = DS18B20_ReadTemperature();

            if (gTemperature > -900.0f) {
                /* 更新极值 */
                if (gTemperature > gTempMax) gTempMax = gTemperature;
                if (gTemperature < gTempMin) gTempMin = gTemperature;

                /* 报警检测 */
                CheckAlarm();

                /* LED指示 */
                switch (gAlarmState) {
                    case 0:  /* 正常 */
                        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN);
                        DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
                        break;
                    case 1:  /* 高温 */
                        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
                        DL_GPIO_togglePins(LED_RED_PORT, LED_RED_PIN);
                        break;
                    case 2:  /* 低温 */
                        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
                        DL_GPIO_togglePins(LED_RED_PORT, LED_RED_PIN);
                        break;
                }

                /* 蜂鸣器报警 */
                if (gAlarmState == 1) {
                    /* 高温：频率随温升增加 */
                    uint16_t freq = 1000 + (uint16_t)(gTemperature - gHighThreshold) * 100;
                    if (freq > 4000) freq = 4000;
                    Buzzer_Beep(freq, 100);
                } else if (gAlarmState == 2) {
                    /* 低温：低频间歇 */
                    Buzzer_Beep(500, 200);
                }
            }

            /* 启动下次转换 */
            DS18B20_StartConversion();
            last_sample = gTickMs;
        }

        /* UART定时上报 */
        if ((gTickMs - last_report) >= REPORT_INTERVAL) {
            float disp_temp = gUseFahrenheit ? CtoF(gTemperature) : gTemperature;
            const char *unit = gUseFahrenheit ? "F" : "C";

            UART_Print("Temp=");
            UART_PrintFloat(disp_temp);
            UART_Print(unit);
            UART_Print(" H=");
            UART_PrintFloat(gHighThreshold);
            UART_Print(" L=");
            UART_PrintFloat(gLowThreshold);

            switch (gAlarmState) {
                case 0: UART_Print(" [NORMAL]"); break;
                case 1: UART_Print(" [HIGH!]");  break;
                case 2: UART_Print(" [LOW!]");   break;
            }

            UART_Print(" Max=");
            UART_PrintFloat(gTempMax);
            UART_Print(" Min=");
            UART_PrintFloat(gTempMin);
            UART_Print("\r\n");

            last_report = gTickMs;
        }

        delay_ms(10);
    }
}
