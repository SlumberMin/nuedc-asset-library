/**
 * @file io_expander_keypad.c
 * @brief IO扩展键盘 - MCP23017 + 4x4矩阵键盘 + LCD1602显示
 * @platform MSPM0G3507
 * @date 2026-06-12
 *
 * 功能概述：
 *   1. MCP23017 I2C IO扩展器驱动
 *   2. 4x4矩阵键盘扫描（行扫描+列读取）
 *   3. LCD1602字符显示（4位数据模式）
 *   4. 按键事件队列（短按/长按检测）
 *   5. 简易计算器功能（加减乘除）
 *
 * 硬件连接：
 *   MCP23017: I2C0 (PA10-SCL, PA11-SDA), 地址0x20
 *     GPA0-GPA3: 键盘行输出
 *     GPB0-GPB3: 键盘列输入(上拉)
 *     GPB4-GPB7: LCD数据线D4-D7
 *   LCD1602: MCP23017 GPB4-7 (数据), PA0-RS, PA1-RW, PA2-EN
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ===== MCP23017 I2C地址 ===== */
#define MCP23017_ADDR           0x20

/* ===== MCP23017寄存器定义 ===== */
#define MCP23017_IODIRA         0x00    /* A口方向寄存器 */
#define MCP23017_IODIRB         0x01    /* B口方向寄存器 */
#define MCP23017_IPOLA          0x02    /* A口极性 */
#define MCP23017_IPOLB          0x03    /* B口极性 */
#define MCP23017_GPINTENA       0x04    /* A口中断使能 */
#define MCP23017_GPINTENB       0x05    /* B口中断使能 */
#define MCP23017_IOCON          0x0A    /* 配置寄存器 */
#define MCP23017_GPPUA          0x0C    /* A口上拉电阻 */
#define MCP23017_GPPUB          0x0D    /* B口上拉电阻 */
#define MCP23017_GPIOA          0x12    /* A口数据 */
#define MCP23017_GPIOB          0x13    /* B口数据 */
#define MCP23017_OLATA          0x14    /* A口输出锁存 */
#define MCP23017_OLATB          0x15    /* B口输出锁存 */

/* ===== LCD1602控制引脚(MSPM0直连) ===== */
#define LCD_RS_PORT             GPIOA
#define LCD_RS_PIN              DL_GPIO_PIN_0
#define LCD_RW_PORT             GPIOA
#define LCD_RW_PIN              DL_GPIO_PIN_1
#define LCD_EN_PORT             GPIOA
#define LCD_EN_PIN              DL_GPIO_PIN_2

/* ===== LCD1602命令定义 ===== */
#define LCD_CMD_CLEAR           0x01
#define LCD_CMD_HOME            0x02
#define LCD_CMD_ENTRY_MODE      0x06
#define LCD_CMD_DISPLAY_ON      0x0C
#define LCD_CMD_DISPLAY_OFF     0x08
#define LCD_CMD_FUNCTION_4BIT   0x28    /* 4位数据，2行，5x8字体 */
#define LCD_CMD_SET_DDRAM       0x80

/* ===== 键盘参数 ===== */
#define KEYPAD_ROWS             4
#define KEYPAD_COLS             4
#define KEY_LONG_PRESS_MS       1000    /* 长按判定时间 */
#define KEY_DEBOUNCE_MS         20      /* 消抖时间 */

/* ===== 按键事件类型 ===== */
typedef enum {
    KEY_EVENT_NONE,     /* 无事件 */
    KEY_EVENT_PRESS,    /* 短按 */
    KEY_EVENT_LONG      /* 长按 */
} KeyEvent_t;

/* ===== 按键状态 ===== */
typedef struct {
    char key_char;              /* 按键字符 */
    KeyEvent_t event;           /* 事件类型 */
    uint32_t press_time;        /* 按下时间 */
    bool is_pressed;            /* 当前是否按下 */
} KeyInfo_t;

/* ===== 4x4键盘映射 ===== */
static const char key_map[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

/* ===== 全局变量 ===== */
static volatile uint32_t g_systick_count = 0;
static KeyInfo_t g_current_key = {0, KEY_EVENT_NONE, 0, false};

/* 计算器相关 */
static char g_lcd_line1[17] = "";           /* LCD第1行 */
static char g_lcd_line2[17] = "";           /* LCD第2行 */
static char g_input_buf[16] = "";           /* 输入缓冲 */
static uint8_t g_input_len = 0;
static float g_operand1 = 0;               /* 操作数1 */
static float g_result = 0;                 /* 计算结果 */
static char g_operator = 0;                /* 运算符 */
static bool g_new_input = true;            /* 新输入标志 */

/* ===== SysTick ===== */
void SysTick_Handler(void) {
    g_systick_count++;
}

static void delay_ms(uint32_t ms) {
    uint32_t start = g_systick_count;
    while ((g_systick_count - start) < ms);
}

static void delay_us(uint32_t us) {
    volatile uint32_t count = us * (SystemCoreClock / 1000000) / 4;
    while (count--);
}

static uint32_t get_tick(void) { return g_systick_count; }

/* ===== I2C通信 ===== */
static void mcp23017_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP23017_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
}

static uint8_t mcp23017_read_reg(uint8_t reg) {
    uint8_t val;
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, MCP23017_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);

    DL_I2C_startControllerTransfer(I2C_0_INST, MCP23017_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    val = DL_I2C_receiveControllerData(I2C_0_INST);
    return val;
}

/* 同时设置A口和B口 */
static void mcp23017_write_gpio(uint8_t gpio_a, uint8_t gpio_b) {
    mcp23017_write_reg(MCP23017_GPIOA, gpio_a);
    mcp23017_write_reg(MCP23017_GPIOB, gpio_b);
}

/* ===== MCP23017初始化 ===== */
static void mcp23017_init(void) {
    /* IOCON配置: BANK=0, SEQOP=1(顺序地址自增关闭) */
    mcp23017_write_reg(MCP23017_IOCON, 0x20);

    /* A口GPA0-3为输出(行驱动), 高4位输入(未用) */
    mcp23017_write_reg(MCP23017_IODIRA, 0xF0);  /* 高4位输入,低4位输出 */

    /* B口GPB0-3为输入(列读取), GPB4-7为输出(LCD数据) */
    mcp23017_write_reg(MCP23017_IODIRB, 0x0F);  /* 低4位输入,高4位输出 */

    /* B口低4位使能上拉(列线) */
    mcp23017_write_reg(MCP23017_GPPUB, 0x0F);

    /* A口全部输出低 */
    mcp23017_write_reg(MCP23017_GPIOA, 0x00);
    mcp23017_write_reg(MCP23017_GPIOB, 0x00);
}

/* ===== LCD1602驱动 ===== */
static void lcd_pulse_en(void) {
    DL_GPIO_setPins(LCD_EN_PORT, LCD_EN_PIN);
    delay_us(1);
    DL_GPIO_clearPins(LCD_EN_PORT, LCD_EN_PIN);
    delay_us(100);
}

/* 通过MCP23017的GPB4-7发送4位数据 */
static void lcd_send_nibble(uint8_t nibble) {
    /* 读取当前B口值，保留低4位，设置高4位 */
    uint8_t gpio_b = (nibble << 4) & 0xF0;
    mcp23017_write_reg(MCP23017_GPIOB, gpio_b);
    lcd_pulse_en();
}

static void lcd_send_byte(uint8_t byte, bool is_data) {
    /* RS: 0=命令, 1=数据 */
    if (is_data) {
        DL_GPIO_setPins(LCD_RS_PORT, LCD_RS_PIN);
    } else {
        DL_GPIO_clearPins(LCD_RS_PORT, LCD_RS_PIN);
    }
    /* RW: 写 */
    DL_GPIO_clearPins(LCD_RW_PORT, LCD_RW_PIN);

    /* 先发高4位，再发低4位 */
    lcd_send_nibble(byte >> 4);
    lcd_send_nibble(byte & 0x0F);
}

static void lcd_command(uint8_t cmd) {
    lcd_send_byte(cmd, false);
    if (cmd == LCD_CMD_CLEAR || cmd == LCD_CMD_HOME) {
        delay_ms(2);
    }
}

static void lcd_data(uint8_t data) {
    lcd_send_byte(data, true);
}

static void lcd_init(void) {
    /* LCD初始化序列(4位模式) */
    DL_GPIO_initDigitalOutput(LCD_RS_PIN);
    DL_GPIO_enableOutput(LCD_RS_PORT, LCD_RS_PIN);
    DL_GPIO_initDigitalOutput(LCD_RW_PIN);
    DL_GPIO_enableOutput(LCD_RW_PORT, LCD_RW_PIN);
    DL_GPIO_initDigitalOutput(LCD_EN_PIN);
    DL_GPIO_enableOutput(LCD_EN_PORT, LCD_EN_PIN);

    delay_ms(50); /* 等待LCD上电稳定 */

    /* 发送3次0x30(8位模式)用于初始化同步 */
    DL_GPIO_clearPins(LCD_RS_PORT, LCD_RS_PIN);
    DL_GPIO_clearPins(LCD_RW_PORT, LCD_RW_PIN);
    lcd_send_nibble(0x03);
    delay_ms(5);
    lcd_send_nibble(0x03);
    delay_ms(1);
    lcd_send_nibble(0x03);
    delay_ms(1);
    lcd_send_nibble(0x02); /* 切换到4位模式 */
    delay_ms(1);

    lcd_command(LCD_CMD_FUNCTION_4BIT); /* 4位, 2行, 5x8 */
    lcd_command(LCD_CMD_DISPLAY_ON);    /* 显示开, 光标关 */
    lcd_command(LCD_CMD_CLEAR);         /* 清屏 */
    lcd_command(LCD_CMD_ENTRY_MODE);    /* 右移光标 */
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = col;
    if (row == 1) addr += 0x40;
    lcd_command(LCD_CMD_SET_DDRAM | addr);
}

static void lcd_print(const char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

static void lcd_clear(void) {
    lcd_command(LCD_CMD_CLEAR);
}

/* ===== 4x4键盘扫描 ===== */
static char keypad_scan(void) {
    for (uint8_t row = 0; row < KEYPAD_ROWS; row++) {
        /* 设置当前行为低电平，其他行为高电平 */
        uint8_t row_data = ~(1 << row) & 0x0F;
        mcp23017_write_reg(MCP23017_GPIOA, row_data);

        delay_us(50); /* 等待信号稳定 */

        /* 读取列值 */
        uint8_t col_data = mcp23017_read_reg(MCP23017_GPIOB) & 0x0F;

        /* 检查哪一列被拉低 */
        for (uint8_t col = 0; col < KEYPAD_COLS; col++) {
            if (!(col_data & (1 << col))) {
                /* 恢复A口 */
                mcp23017_write_reg(MCP23017_GPIOA, 0x00);
                return key_map[row][col];
            }
        }
    }
    /* 恢复A口 */
    mcp23017_write_reg(MCP23017_GPIOA, 0x00);
    return 0; /* 无按键 */
}

/* 按键事件处理(消抖+长按检测) */
static KeyEvent_t keypad_get_event(char *key_out) {
    static char last_key = 0;
    static uint32_t press_start = 0;
    static bool key_is_down = false;

    char key = keypad_scan();

    if (key != 0) {
        /* 有按键按下 */
        if (!key_is_down) {
            /* 新按下 */
            key_is_down = true;
            last_key = key;
            press_start = get_tick();
        } else if (key == last_key) {
            /* 持续按住 */
            if ((get_tick() - press_start) >= KEY_LONG_PRESS_MS) {
                *key_out = last_key;
                key_is_down = false; /* 防止重复触发 */
                return KEY_EVENT_LONG;
            }
        }
    } else {
        /* 无按键 */
        if (key_is_down) {
            /* 刚释放 */
            key_is_down = false;
            if ((get_tick() - press_start) >= KEY_DEBOUNCE_MS) {
                *key_out = last_key;
                return KEY_EVENT_PRESS;
            }
        }
    }
    return KEY_EVENT_NONE;
}

/* ===== 计算器逻辑 ===== */
static void calc_clear(void) {
    g_input_buf[0] = '\0';
    g_input_len = 0;
    g_operand1 = 0;
    g_result = 0;
    g_operator = 0;
    g_new_input = true;
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("MSPM0 Calculator");
    lcd_set_cursor(1, 0);
    lcd_print("0");
}

static void calc_input_digit(char digit) {
    if (g_new_input) {
        g_input_buf[0] = '\0';
        g_input_len = 0;
        g_new_input = false;
    }

    if (g_input_len < 15) {
        g_input_buf[g_input_len++] = digit;
        g_input_buf[g_input_len] = '\0';

        lcd_set_cursor(1, 0);
        /* 清除该行后显示 */
        lcd_print("                ");
        lcd_set_cursor(1, 0);
        lcd_print(g_input_buf);
    }
}

static void calc_set_operator(char op) {
    if (!g_new_input && g_input_len > 0) {
        /* 先计算之前的运算 */
        float val = 0;
        for (uint8_t i = 0; i < g_input_len; i++) {
            val = val * 10.0f + (g_input_buf[i] - '0');
        }

        if (g_operator == 0) {
            g_operand1 = val;
        } else {
            /* 执行上次运算 */
            switch (g_operator) {
            case '+': g_operand1 += val; break;
            case '-': g_operand1 -= val; break;
            case 'x': g_operand1 *= val; break;
            case '/':
                if (val != 0) g_operand1 /= val;
                else g_operand1 = 0;
                break;
            }
        }
    }
    g_operator = op;
    g_new_input = true;

    /* 显示 */
    lcd_set_cursor(0, 0);
    lcd_print("                ");
    lcd_set_cursor(0, 0);
    snprintf(g_lcd_line1, 17, "%.4g %c", g_operand1, op);
    lcd_print(g_lcd_line1);
}

static void calc_execute(void) {
    if (g_input_len > 0) {
        float val = 0;
        for (uint8_t i = 0; i < g_input_len; i++) {
            val = val * 10.0f + (g_input_buf[i] - '0');
        }

        switch (g_operator) {
        case '+': g_result = g_operand1 + val; break;
        case '-': g_result = g_operand1 - val; break;
        case 'x': g_result = g_operand1 * val; break;
        case '/':
            if (val != 0) g_result = g_operand1 / val;
            else g_result = 0;
            break;
        default: g_result = val; break;
        }

        /* 显示结果 */
        lcd_set_cursor(1, 0);
        lcd_print("                ");
        lcd_set_cursor(1, 0);
        snprintf(g_lcd_line2, 17, "= %.6g", g_result);
        lcd_print(g_lcd_line2);

        g_operand1 = g_result;
        g_operator = 0;
        g_new_input = true;
    }
}

/* ===== 主函数 ===== */
int main(void) {
    /* 系统初始化 */
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* MCP23017初始化 */
    mcp23017_init();

    /* LCD初始化 */
    lcd_init();

    /* 显示欢迎信息 */
    lcd_set_cursor(0, 0);
    lcd_print("MSPM0 Calculator");
    lcd_set_cursor(1, 0);
    lcd_print("0");

    /* 主循环 */
    while (1) {
        char key;
        KeyEvent_t event = keypad_get_event(&key);

        if (event != KEY_EVENT_NONE) {
            if (key >= '0' && key <= '9') {
                /* 数字键 */
                calc_input_digit(key);
            } else if (key == 'A') {
                /* 加法 */
                calc_set_operator('+');
            } else if (key == 'B') {
                /* 减法 */
                calc_set_operator('-');
            } else if (key == 'C') {
                /* 乘法 */
                calc_set_operator('x');
            } else if (key == 'D') {
                /* 除法 */
                calc_set_operator('/');
            } else if (key == '#') {
                /* 等号/执行 */
                calc_execute();
            } else if (key == '*') {
                /* 清除 */
                calc_clear();
            }

            /* 长按#键: 清除并复位 */
            if (event == KEY_EVENT_LONG && key == '#') {
                calc_clear();
            }
        }

        delay_ms(10);
    }
}
