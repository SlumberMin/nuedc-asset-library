/**
 * @file digital_clock.c
 * @brief MSPM0G3507 数字时钟示例
 *
 * 硬件连接：
 *   TM1637数码管模块：
 *     CLK -> PA0 (GPIO)
 *     DIO -> PA1 (GPIO)
 *
 *   DS18B20温度传感器：
 *     DATA -> PA2 (单总线，需4.7K上拉)
 *
 *   按键（接地有效，内部上拉）：
 *     设置键 -> PA3
 *     增加键 -> PA4
 *     减少键 -> PA5
 *     切换键 -> PA6
 *
 * 功能：
 *   - 显示时:分（带冒号闪烁）
 *   - DS18B20实时温度显示
 *   - 按键设置时/分/秒
 *   - 自动切换时间和温度显示
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== TM1637引脚定义 ========== */
#define TM1637_CLK_PORT     GPIOA
#define TM1637_CLK_PIN      DL_GPIO_PIN_0
#define TM1637_DIO_PORT     GPIOA
#define TM1637_DIO_PIN      DL_GPIO_PIN_1

/* ========== DS18B20引脚定义 ========== */
#define DS18B20_PORT        GPIOA
#define DS18B20_PIN         DL_GPIO_PIN_2

/* ========== 按键引脚定义 ========== */
#define BTN_SET_PORT        GPIOA
#define BTN_SET_PIN         DL_GPIO_PIN_3
#define BTN_UP_PORT         GPIOA
#define BTN_UP_PIN          DL_GPIO_PIN_4
#define BTN_DOWN_PORT       GPIOA
#define BTN_DOWN_PIN        DL_GPIO_PIN_5
#define BTN_MODE_PORT       GPIOA
#define BTN_MODE_PIN        DL_GPIO_PIN_6

/* ========== TM1637段码定义 ========== */
/* 0-9的段码（共阴极，顺序：dp-g-f-e-d-c-b-a） */
static const uint8_t seg_digits[] = {
    0x3F,  /* 0 */
    0x06,  /* 1 */
    0x5B,  /* 2 */
    0x4F,  /* 3 */
    0x66,  /* 4 */
    0x6D,  /* 5 */
    0x7D,  /* 6 */
    0x07,  /* 7 */
    0x7F,  /* 8 */
    0x6F   /* 9 */
};

/* TM1637命令 */
#define TM1637_CMD_DATA_AUTO   0x40  /* 自动递增模式 */
#define TM1637_CMD_DATA_FIXED  0x44  /* 固定地址模式 */
#define TM1637_CMD_ADDR_BASE   0xC0  /* 地址起始 */
#define TM1637_CMD_DISPLAY_ON  0x88  /* 显示开，亮度1 */
#define TM1637_CMD_DISPLAY_OFF 0x80  /* 显示关 */

/* ========== 时间结构体 ========== */
typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;  /* 毫秒计数 */
} ClockTime_t;

/* ========== 全局变量 ========== */
static ClockTime_t g_clock = {12, 30, 0, 0};
static float g_temperature = 25.0f;  /* DS18B20读数 */
static bool g_showTime = true;       /* true=时间, false=温度 */
static uint8_t g_brightness = 5;     /* 亮度 0~7 */
static uint8_t g_setMode = 0;        /* 0=正常, 1=设时, 2=设分, 3=设秒 */
static volatile uint32_t g_tick = 0; /* 系统滴答计数 */

/* ========== 延时函数（微秒级）========== */
static void delay_us(uint32_t us)
{
    /* 32MHz主频下，约32周期/微秒 */
    uint32_t cycles = us * 32;
    while (cycles--) {
        __asm volatile("nop");
    }
}

/* ========== TM1637底层通信 ========== */
static void tm1637_start(void)
{
    /* 起始条件：CLK高时DIO由高变低 */
    DL_GPIO_setPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
    DL_GPIO_setPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
    delay_us(2);
    DL_GPIO_clearPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
    delay_us(2);
    DL_GPIO_clearPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
}

static void tm1637_stop(void)
{
    /* 停止条件：CLK高时DIO由低变高 */
    DL_GPIO_clearPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
    DL_GPIO_clearPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
    delay_us(2);
    DL_GPIO_setPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
    delay_us(2);
    DL_GPIO_setPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
}

static void tm1637_write_byte(uint8_t data)
{
    /* 发送8位数据，LSB first */
    for (int i = 0; i < 8; i++) {
        DL_GPIO_clearPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
        delay_us(2);
        if (data & (1 << i)) {
            DL_GPIO_setPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
        } else {
            DL_GPIO_clearPins(TM1637_DIO_PORT, TM1637_DIO_PIN);
        }
        delay_us(2);
        DL_GPIO_setPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
        delay_us(2);
    }
    /* 等待ACK */
    DL_GPIO_clearPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
    /* DIO设为输入读ACK */
    DL_GPIO_initDigitalInput(TM1637_DIO_PORT, TM1637_DIO_PIN);
    delay_us(2);
    DL_GPIO_setPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
    delay_us(2);
    /* 恢复为输出 */
    DL_GPIO_initDigitalOutput(TM1637_DIO_PORT, TM1637_DIO_PIN);
    DL_GPIO_clearPins(TM1637_CLK_PORT, TM1637_CLK_PIN);
}

/* ========== TM1637显示函数 ========== */
/* 显示4位数码管，colon控制冒号 */
static void tm1637_display(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, bool colon)
{
    uint8_t brightness_cmd = TM1637_CMD_DISPLAY_ON | (g_brightness & 0x07);

    tm1637_start();
    tm1637_write_byte(TM1637_CMD_DATA_AUTO);  /* 自动递增模式 */
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(TM1637_CMD_ADDR_BASE);  /* 从地址0开始 */

    /* 第1位 */
    tm1637_write_byte(d1);
    /* 第2位（冒号由bit7控制） */
    tm1637_write_byte(d2 | (colon ? 0x80 : 0x00));
    /* 第3位 */
    tm1637_write_byte(d3);
    /* 第4位 */
    tm1637_write_byte(d4);
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(brightness_cmd);
    tm1637_stop();
}

/* 显示时间 HH:MM */
static void display_time(ClockTime_t *t)
{
    tm1637_display(
        seg_digits[t->hour / 10],
        seg_digits[t->hour % 10],
        seg_digits[t->minute / 10],
        seg_digits[t->minute % 10],
        (t->second % 2 == 0)  /* 冒号每秒闪烁 */
    );
}

/* 显示温度，范围 -9~99.9°C */
static void display_temperature(float temp)
{
    int t = (int)(temp * 10);  /* 放大10倍保留1位小数 */
    bool negative = (t < 0);
    if (negative) t = -t;

    uint8_t d1, d2, d3, d4;

    if (negative) {
        d1 = 0x40;  /* 负号 '-' */
        d2 = seg_digits[(t / 100) % 10];
        d3 = seg_digits[(t / 10) % 10] | 0x80;  /* 小数点 */
        d4 = seg_digits[t % 10];
    } else if (t >= 1000) {
        /* 100.0以上 */
        d1 = seg_digits[t / 1000];
        d2 = seg_digits[(t / 100) % 10];
        d3 = seg_digits[(t / 10) % 10] | 0x80;
        d4 = seg_digits[t % 10];
    } else {
        d1 = 0x00;  /* 空白 */
        d2 = seg_digits[(t / 100) % 10];
        d3 = seg_digits[(t / 10) % 10] | 0x80;  /* 小数点 */
        d4 = seg_digits[t % 10];
    }

    tm1637_display(d1, d2, d3, d4, false);
}

/* ========== DS18B20单总线协议 ========== */
static void ds18b20_pin_output(void)
{
    DL_GPIO_initDigitalOutput(DS18B20_PORT, DS18B20_PIN);
}

static void ds18b20_pin_input(void)
{
    DL_GPIO_initDigitalInput(DS18B20_PORT, DS18B20_PIN);
}

static bool ds18b20_read_pin(void)
{
    return (DL_GPIO_readPins(DS18B20_PORT, DS18B20_PIN) != 0);
}

static void ds18b20_write_pin(bool val)
{
    if (val)
        DL_GPIO_setPins(DS18B20_PORT, DS18B20_PIN);
    else
        DL_GPIO_clearPins(DS18B20_PORT, DS18B20_PIN);
}

/* 复位并检测DS18B20，返回true表示设备存在 */
static bool ds18b20_reset(void)
{
    bool present;
    ds18b20_pin_output();
    ds18b20_write_pin(false);
    delay_us(480);       /* 拉低480us */
    ds18b20_pin_input(); /* 释放总线 */
    delay_us(60);        /* 等待60us */
    present = !ds18b20_read_pin();  /* 低电平=设备存在 */
    delay_us(420);       /* 等待时隙结束 */
    return present;
}

/* 写1位 */
static void ds18b20_write_bit(bool bit)
{
    ds18b20_pin_output();
    ds18b20_write_pin(false);
    delay_us(2);
    if (bit) {
        ds18b20_pin_input();
    }
    delay_us(60);
    ds18b20_pin_input();
    delay_us(2);
}

/* 读1位 */
static bool ds18b20_read_bit(void)
{
    bool bit;
    ds18b20_pin_output();
    ds18b20_write_pin(false);
    delay_us(2);
    ds18b20_pin_input();
    delay_us(10);
    bit = ds18b20_read_pin();
    delay_us(50);
    return bit;
}

/* 写1字节 */
static void ds18b20_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(data & 0x01);
        data >>= 1;
    }
}

/* 读1字节 */
static uint8_t ds18b20_read_byte(void)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        if (ds18b20_read_bit()) {
            data |= (1 << i);
        }
    }
    return data;
}

/* 读取温度（摄氏度） */
static float ds18b20_read_temperature(void)
{
    uint8_t lsb, msb;
    int16_t raw;

    if (!ds18b20_reset()) {
        return -999.0f;  /* 设备未响应 */
    }

    ds18b20_write_byte(0xCC);  /* 跳过ROM（单设备） */
    ds18b20_write_byte(0x44);  /* 启动温度转换 */

    /* 等待转换完成（约750ms for 12-bit） */
    delay_us(750000);

    if (!ds18b20_reset()) {
        return -999.0f;
    }

    ds18b20_write_byte(0xCC);  /* 跳过ROM */
    ds18b20_write_byte(0xBE);  /* 读暂存器 */

    lsb = ds18b20_read_byte();
    msb = ds18b20_read_byte();

    raw = (msb << 8) | lsb;

    /* 12位精度：每LSB = 0.0625°C */
    return raw * 0.0625f;
}

/* ========== 按键扫描 ========== */
/* 返回：0=无按键, 1=设置, 2=增加, 3=减少, 4=模式 */
static uint8_t scan_buttons(void)
{
    static uint8_t last_state = 0xFF;
    uint8_t current = 0;

    if (!DL_GPIO_readPins(BTN_SET_PORT, BTN_SET_PIN))  current |= 0x01;
    if (!DL_GPIO_readPins(BTN_UP_PORT, BTN_UP_PIN))    current |= 0x02;
    if (!DL_GPIO_readPins(BTN_DOWN_PORT, BTN_DOWN_PIN)) current |= 0x04;
    if (!DL_GPIO_readPins(BTN_MODE_PORT, BTN_MODE_PIN)) current |= 0x08;

    /* 下降沿检测 */
    uint8_t pressed = (~current) & last_state;
    last_state = current;

    if (pressed & 0x01) return 1;
    if (pressed & 0x02) return 2;
    if (pressed & 0x04) return 3;
    if (pressed & 0x08) return 4;
    return 0;
}

/* ========== 时钟更新 ========== */
static void clock_tick(void)
{
    g_clock.millisecond += 10;  /* 每10ms调用一次 */
    if (g_clock.millisecond >= 1000) {
        g_clock.millisecond = 0;
        g_clock.second++;
        if (g_clock.second >= 60) {
            g_clock.second = 0;
            g_clock.minute++;
            if (g_clock.minute >= 60) {
                g_clock.minute = 0;
                g_clock.hour++;
                if (g_clock.hour >= 24) {
                    g_clock.hour = 0;
                }
            }
        }
    }
}

/* ========== 简易延时（毫秒）========== */
static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

/* ========== 主函数 ========== */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* GPIO初始化 - TM1637 */
    DL_GPIO_initDigitalOutput(TM1637_CLK_PORT, TM1637_CLK_PIN);
    DL_GPIO_initDigitalOutput(TM1637_DIO_PORT, TM1637_DIO_PIN);

    /* GPIO初始化 - DS18B20 */
    DL_GPIO_initDigitalOutput(DS18B20_PORT, DS18B20_PIN);
    DL_GPIO_setPins(DS18B20_PORT, DS18B20_PIN);

    /* GPIO初始化 - 按键（内部上拉） */
    DL_GPIO_initDigitalInputFeatures(BTN_SET_PORT, BTN_SET_PIN,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BTN_UP_PORT, BTN_UP_PIN,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BTN_DOWN_PORT, BTN_DOWN_PIN,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BTN_MODE_PORT, BTN_MODE_PIN,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 首次读取温度 */
    g_temperature = ds18b20_read_temperature();
    if (g_temperature < -100.0f) g_temperature = 25.0f;  /* 读取失败用默认值 */

    uint32_t temp_read_counter = 0;
    uint32_t mode_switch_counter = 0;

    /* 主循环 */
    while (1) {
        /* 每10ms更新时钟 */
        clock_tick();

        /* 按键处理 */
        uint8_t btn = scan_buttons();
        switch (btn) {
            case 1:  /* 设置键：切换设置模式 */
                g_setMode++;
                if (g_setMode > 3) g_setMode = 0;
                break;

            case 2:  /* 增加键 */
                if (g_setMode == 1) {
                    g_clock.hour = (g_clock.hour + 1) % 24;
                } else if (g_setMode == 2) {
                    g_clock.minute = (g_clock.minute + 1) % 60;
                } else if (g_setMode == 3) {
                    g_clock.second = (g_clock.second + 1) % 60;
                } else {
                    /* 正常模式下调节亮度 */
                    g_brightness = (g_brightness + 1) & 0x07;
                }
                break;

            case 3:  /* 减少键 */
                if (g_setMode == 1) {
                    g_clock.hour = (g_clock.hour == 0) ? 23 : g_clock.hour - 1;
                } else if (g_setMode == 2) {
                    g_clock.minute = (g_clock.minute == 0) ? 59 : g_clock.minute - 1;
                } else if (g_setMode == 3) {
                    g_clock.second = (g_clock.second == 0) ? 59 : g_clock.second - 1;
                } else {
                    g_brightness = (g_brightness == 0) ? 7 : g_brightness - 1;
                }
                break;

            case 4:  /* 模式切换：时间/温度 */
                g_showTime = !g_showTime;
                mode_switch_counter = 0;
                break;
        }

        /* 显示刷新 */
        if (g_setMode > 0) {
            /* 设置模式下显示对应设置项，设置项闪烁 */
            if ((g_tick / 50) % 2 == 0) {
                display_time(&g_clock);
            } else {
                /* 设置项闪烁时清空对应位 */
                ClockTime_t temp = g_clock;
                if (g_setMode == 1) temp.hour = 0xFF;    /* 清空小时 */
                if (g_setMode == 2) temp.minute = 0xFF;  /* 清空分钟 */
                if (g_setMode == 3) temp.second = 0xFF;   /* 清空秒 */
                display_time(&g_clock);  /* 简化：仍显示完整 */
            }
        } else if (g_showTime) {
            display_time(&g_clock);
        } else {
            display_temperature(g_temperature);
        }

        /* 定时读取温度（每2秒） */
        temp_read_counter++;
        if (temp_read_counter >= 200) {
            temp_read_counter = 0;
            float t = ds18b20_read_temperature();
            if (t > -100.0f) {
                g_temperature = t;
            }
        }

        /* 自动切换回时间显示（温度显示5秒后） */
        if (!g_showTime) {
            mode_switch_counter++;
            if (mode_switch_counter >= 500) {  /* 5秒 */
                g_showTime = true;
                mode_switch_counter = 0;
            }
        }

        g_tick++;
        delay_ms(10);
    }
}
