/**
 * @file weight_scale.c
 * @brief 电子秤 - MSPM0G3507系统集成示例
 *
 * 功能：HX711 ADC模块 + OLED显示 + 去皮/校准 + 单位切换(g/kg/oz) + 按键操作
 * 硬件：MSPM0G3507 + HX711 + SSD1306 OLED(I2C) + 3个按键
 *
 * 接线：
 *   HX711 DOUT  -> PB2  (GPIO输入)
 *   HX711 SCK   -> PB3  (GPIO输出)
 *   OLED SDA    -> PA0  (I2C0)
 *   OLED SCL    -> PA1  (I2C0)
 *   按键-去皮   -> PA12 (GPIO, 低有效)
 *   按键-校准   -> PA13 (GPIO, 低有效)
 *   按键-单位   -> PA14 (GPIO, 低有效)
 *   LED指示     -> PB14 (GPIO)
 *   蜂鸣器      -> PB15 (GPIO)
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== 引脚定义 ========== */
#define HX711_DOUT_PORT  GPIOB
#define HX711_DOUT_PIN   DL_GPIO_PIN_2
#define HX711_SCK_PORT   GPIOB
#define HX711_SCK_PIN    DL_GPIO_PIN_3
#define BTN_TARE_PORT    GPIOA
#define BTN_TARE_PIN     DL_GPIO_PIN_12
#define BTN_CAL_PORT     GPIOA
#define BTN_CAL_PIN      DL_GPIO_PIN_13
#define BTN_UNIT_PORT    GPIOA
#define BTN_UNIT_PIN     DL_GPIO_PIN_14
#define LED_PORT         GPIOB
#define LED_PIN          DL_GPIO_PIN_14
#define BUZZER_PORT      GPIOB
#define BUZZER_PIN       DL_GPIO_PIN_15

/* ========== 重量单位 ========== */
typedef enum {
    UNIT_GRAM = 0,
    UNIT_KG,
    UNIT_OZ,
    UNIT_COUNT
} WeightUnit_t;

static const char *UNIT_NAMES[] = {"g", "kg", "oz"};
static const float UNIT_FACTORS[] = {1.0f, 0.001f, 0.035274f};

/* ========== 全局变量 ========== */
static volatile uint32_t gTickMs = 0;
static int32_t  gTareOffset = 0;       /* 去皮偏移 */
static float    gScaleFactor = 412.0f; /* 校准因子（默认值） */
static WeightUnit_t gUnit = UNIT_GRAM;
static float    gWeight = 0.0f;
static bool     gOverload = false;     /* 超量程标志 */

/* =================================================================
 * 基础延时
 * ================================================================= */

void SysTick_Handler(void) { gTickMs++; }

static void delay_us(uint32_t us) {
    volatile uint32_t cnt = us * 8;
    while (cnt--);
}

static void delay_ms(uint32_t ms) {
    uint32_t s = gTickMs;
    while ((gTickMs - s) < ms);
}

/* =================================================================
 * I2C驱动（OLED）
 * ================================================================= */

static void I2C_Start(void) {
    DL_I2C_resetController(I2C0);
    while (DL_I2C_isControllerBusy(I2C0));
}

static void I2C_Stop(void) { /* handled by HW */ }

/**
 * @brief 通过I2C发送OLED命令/数据
 * @param addr 从机地址
 * @param ctrl 控制字节 (0x00=命令, 0x40=数据)
 * @param data 要发送的数据
 * @param len 数据长度
 */
static void OLED_WriteBytes(uint8_t addr, uint8_t ctrl, const uint8_t *data, uint16_t len) {
    /* 使用TI DriverLib I2C发送 */
    DL_I2C_setTargetAddress(I2C0, addr);
    while (DL_I2C_isControllerBusy(I2C0));

    /* 发送控制字节 + 数据 */
    for (uint16_t i = 0; i < len; i++) {
        if (i == 0) {
            DL_I2C_transmitControllerData(I2C0, ctrl);
        }
        DL_I2C_transmitControllerData(I2C0, data[i]);
        while (DL_I2C_isControllerBusy(I2C0));
    }
}

static void OLED_WriteCmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    /* 简化：直接通过I2C字节发送 */
    DL_I2C_setTargetAddress(I2C0, 0x3C);
    while (DL_I2C_isControllerBusy(I2C0));
    DL_I2C_startControllerTransfer(I2C0, 0x3C, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (!DL_I2C_isControllerTXFIFOEmpty(I2C0));
    DL_I2C_transmitControllerData(I2C0, 0x00);
    while (DL_I2C_isControllerBusy(I2C0));
    DL_I2C_transmitControllerData(I2C0, cmd);
    while (DL_I2C_isControllerBusy(I2C0));
}

static void OLED_WriteData(uint8_t data) {
    DL_I2C_setTargetAddress(I2C0, 0x3C);
    while (DL_I2C_isControllerBusy(I2C0));
    DL_I2C_transmitControllerData(I2C0, 0x40);
    while (DL_I2C_isControllerBusy(I2C0));
    DL_I2C_transmitControllerData(I2C0, data);
    while (DL_I2C_isControllerBusy(I2C0));
}

/**
 * @brief OLED初始化（SSD1306 128x64）
 */
static void OLED_Init(void) {
    delay_ms(100);
    OLED_WriteCmd(0xAE); /* 关闭显示 */
    OLED_WriteCmd(0x20); /* 设置寻址模式 */
    OLED_WriteCmd(0x00); /* 水平寻址 */
    OLED_WriteCmd(0xB0); /* 设置页地址 */
    OLED_WriteCmd(0xC8); /* COM扫描方向 */
    OLED_WriteCmd(0x00); /* 列低位 */
    OLED_WriteCmd(0x10); /* 列高位 */
    OLED_WriteCmd(0x40); /* 起始行 */
    OLED_WriteCmd(0x81); /* 对比度 */
    OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xA1); /* 段重映射 */
    OLED_WriteCmd(0xA6); /* 正常显示 */
    OLED_WriteCmd(0xA8); /* 多路复用率 */
    OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xA4); /* 全显示关闭 */
    OLED_WriteCmd(0xD3); /* 显示偏移 */
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xD5); /* 时钟分频 */
    OLED_WriteCmd(0xF0);
    OLED_WriteCmd(0xD9); /* 预充电周期 */
    OLED_WriteCmd(0x22);
    OLED_WriteCmd(0xDA); /* COM引脚配置 */
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0xDB); /* VCOMH */
    OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x8D); /* 电荷泵 */
    OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xAF); /* 开启显示 */
}

/**
 * @brief 设置OLED光标位置
 */
static void OLED_SetCursor(uint8_t page, uint8_t col) {
    OLED_WriteCmd(0xB0 + page);
    OLED_WriteCmd(0x00 + (col & 0x0F));
    OLED_WriteCmd(0x10 + ((col >> 4) & 0x0F));
}

/**
 * @brief 清屏
 */
static void OLED_Clear(void) {
    for (uint8_t page = 0; page < 8; page++) {
        OLED_SetCursor(page, 0);
        for (uint8_t col = 0; col < 128; col++) {
            OLED_WriteData(0x00);
        }
    }
}

/**
 * @brief 显示ASCII字符（6x8字模，简化版）
 * 注意：实际项目需提供完整字模表，这里仅示例框架
 */
static void OLED_PrintString(uint8_t page, uint8_t col, const char *str) {
    OLED_SetCursor(page, col);
    while (*str) {
        /* 简化：实际需要字模库 */
        /* 这里写入空白占位，实际替换为字模查表 */
        for (uint8_t i = 0; i < 6; i++) {
            /* 简化处理 - 实际应查字模表 */
            OLED_WriteData((*str == ' ') ? 0x00 : 0x7E);
        }
        str++;
        col += 6;
        if (col >= 128) {
            col = 0;
            page++;
            OLED_SetCursor(page, col);
        }
    }
}

/* =================================================================
 * HX711驱动
 * ================================================================= */

static void HX711_SCK_Low(void)  { DL_GPIO_clearPins(HX711_SCK_PORT, HX711_SCK_PIN); }
static void HX711_SCK_High(void) { DL_GPIO_setPins(HX711_SCK_PORT, HX711_SCK_PIN); }
static bool HX711_DOUT_Read(void) { return DL_GPIO_readPins(HX711_DOUT_PORT, HX711_DOUT_PIN) ? true : false; }

/**
 * @brief 读取HX711原始ADC值（24位）
 * @return 24位有符号值，-8388608 ~ 8388607
 * @note 增益128倍（通道A）
 */
static int32_t HX711_ReadRaw(void) {
    int32_t value = 0;

    /* 等待DOUT变低（数据就绪） */
    uint32_t timeout = gTickMs + 100;
    while (HX711_DOUT_Read()) {
        if (gTickMs > timeout) {
            /* 超时，HX711未响应 */
            return 0;
        }
    }

    /* 读取24位数据（MSB first） */
    for (uint8_t i = 0; i < 24; i++) {
        HX711_SCK_High();
        delay_us(1);
        value = (value << 1) | (HX711_DOUT_Read() ? 1 : 0);
        HX711_SCK_Low();
        delay_us(1);
    }

    /* 第25个脉冲：设置增益128 */
    HX711_SCK_High();
    delay_us(1);
    HX711_SCK_Low();
    delay_us(1);

    /* 符号扩展：24位转32位有符号 */
    if (value & 0x800000) {
        value |= 0xFF000000;  /* 负数补码扩展 */
    }

    return value;
}

/**
 * @brief 读取多次平均值（消除噪声）
 * @param times 采样次数
 * @return 平均ADC值
 */
static int32_t HX711_ReadAverage(uint8_t times) {
    int64_t sum = 0;
    int32_t max_val = -8388608, min_val = 8388607;

    for (uint8_t i = 0; i < times + 2; i++) {
        int32_t val = HX711_ReadRaw();
        if (i == 0 || i == times + 1) continue; /* 去掉首尾 */
        if (val > max_val) max_val = val;
        if (val < min_val) min_val = val;
        sum += val;
    }
    sum -= max_val + min_val;  /* 去掉最大最小 */
    return (int32_t)(sum / times);
}

/* =================================================================
 * 去皮与校准
 * ================================================================= */

/**
 * @brief 去皮操作：记录当前空载偏移
 */
static void DoTare(void) {
    gTareOffset = HX711_ReadAverage(10);

    /* 蜂鸣器短响确认 */
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
    delay_ms(100);
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}

/**
 * @brief 校准操作：使用已知砝码校准
 * @note 程序中使用100g校准砝码
 */
static void DoCalibrate(void) {
    const float CALIBRATION_WEIGHT = 100.0f;  /* 校准砝码100g */

    /* 蜂鸣器提示放砝码 */
    for (uint8_t i = 0; i < 3; i++) {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
        delay_ms(100);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        delay_ms(100);
    }

    /* 等待稳定 */
    delay_ms(3000);

    /* 读取校准值 */
    int32_t cal_raw = HX711_ReadAverage(20);
    int32_t diff = cal_raw - gTareOffset;

    if (diff > 100) {
        /* 计算校准因子 */
        gScaleFactor = (float)diff / CALIBRATION_WEIGHT;

        /* 长响确认 */
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
        delay_ms(500);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
    }
}

/* =================================================================
 * 重量计算与单位切换
 * ================================================================= */

/**
 * @brief 获取当前重量（已去皮和校准）
 * @return 重量（克）
 */
static float GetWeightGrams(void) {
    int32_t raw = HX711_ReadAverage(5);
    int32_t diff = raw - gTareOffset;

    if (fabsf((float)diff) < 50) {
        return 0.0f;  /* 零点附近归零 */
    }

    return (float)diff / gScaleFactor;
}

/**
 * @brief 切换单位
 */
static void SwitchUnit(void) {
    gUnit = (WeightUnit_t)((gUnit + 1) % UNIT_COUNT);
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
}

/* =================================================================
 * 按键消抖
 * ================================================================= */

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    uint32_t last_press;
    bool pressed;
} Button_t;

static Button_t btnTare  = {BTN_TARE_PORT, BTN_TARE_PIN, 0, false};
static Button_t btnCal   = {BTN_CAL_PORT,  BTN_CAL_PIN,  0, false};
static Button_t btnUnit  = {BTN_UNIT_PORT, BTN_UNIT_PIN, 0, false};

/**
 * @brief 检测按键按下（消抖）
 */
static bool Button_Check(Button_t *btn) {
    if (!DL_GPIO_readPins(btn->port, btn->pin)) {  /* 低有效 */
        if (!btn->pressed && (gTickMs - btn->last_press) > 200) {
            btn->pressed = true;
            btn->last_press = gTickMs;
            return true;
        }
    } else {
        btn->pressed = false;
    }
    return false;
}

/* =================================================================
 * 主函数
 * ================================================================= */
int main(void) {
    /* 系统初始化 */
    DL_SYSCFG_init();
    SysTick_Config(32000);  /* 1ms */

    /* GPIO初始化 */
    DL_GPIO_initDigitalOutput(HX711_SCK_PIN);
    DL_GPIO_initDigitalInputFeatures(HX711_DOUT_PIN,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BTN_TARE_PIN,
        DL_GPIO_RESISTOR_PULLUP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BTN_CAL_PIN,
        DL_GPIO_RESISTOR_PULLUP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BTN_UNIT_PIN,
        DL_GPIO_RESISTOR_PULLUP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalOutput(LED_PIN);
    DL_GPIO_initDigitalOutput(BUZZER_PIN);

    /* I2C初始化 */
    DL_I2C_enableController(I2C0);

    /* OLED初始化 */
    OLED_Init();
    OLED_Clear();

    /* 初始去皮 */
    DoTare();

    /* 显示标题 */
    OLED_PrintString(0, 0, "Weight Scale");
    OLED_PrintString(1, 0, "Ready");

    char display_buf[32];
    uint32_t last_update = 0;
    const uint32_t UPDATE_INTERVAL = 100;  /* 100ms刷新 */

    /* ===== 主循环 ===== */
    while (1) {
        /* 按键检测 */
        if (Button_Check(&btnTare)) {
            DoTare();
        }
        if (Button_Check(&btnCal)) {
            OLED_Clear();
            OLED_PrintString(2, 0, "Put 100g...");
            DoCalibrate();
            OLED_Clear();
        }
        if (Button_Check(&btnUnit)) {
            SwitchUnit();
        }

        /* 定时更新显示 */
        if ((gTickMs - last_update) >= UPDATE_INTERVAL) {
            /* 获取重量 */
            float weight_g = GetWeightGrams();

            /* 超量程检测 */
            if (weight_g > 5000.0f || weight_g < -100.0f) {
                gOverload = true;
            } else {
                gOverload = false;
            }

            /* 单位转换 */
            float display_weight = weight_g * UNIT_FACTORS[gUnit];

            /* 格式化显示 */
            snprintf(display_buf, sizeof(display_buf), "%.2f %s",
                     display_weight, UNIT_NAMES[gUnit]);

            /* 更新OLED */
            OLED_SetCursor(3, 0);
            /* 清除该行 */
            for (uint8_t i = 0; i < 21; i++) OLED_PrintString(3, i * 6, " ");
            /* 显示重量 */
            OLED_PrintString(3, 0, display_buf);

            /* 超量程警告 */
            if (gOverload) {
                OLED_PrintString(5, 0, "OVERLOAD!");
                DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
            } else {
                OLED_PrintString(5, 0, "         ");
                DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
            }

            last_update = gTickMs;
        }

        delay_ms(10);
    }
}
