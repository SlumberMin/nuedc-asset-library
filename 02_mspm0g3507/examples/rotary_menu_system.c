/**
 * @file rotary_menu_system.c
 * @brief MSPM0G3507 旋转编码器多级菜单系统
 *
 * 功能：旋转编码器导航 + 按键确认/返回 + LCD1602显示多级菜单
 * 应用：电赛项目参数设置、仪器菜单界面
 *
 * 硬件连接：
 *   LCD1602 (4位并行):
 *     RS=PB0, EN=PB1, D4~D7=PB2~PB5
 *   旋转编码器:
 *     CLK=PA8, DT=PA9, SW=PA10 (按键)
 *
 * @author 电赛资产库
 * @date 2026
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>

/* ========== LCD1602驱动 ========== */

/* LCD引脚定义 */
#define LCD_RS_PORT     GPIOB
#define LCD_RS_PIN      DL_GPIO_PIN_0
#define LCD_EN_PORT     GPIOB
#define LCD_EN_PIN      DL_GPIO_PIN_1
#define LCD_D4_PORT     GPIOB
#define LCD_D4_PIN      DL_GPIO_PIN_2
#define LCD_D5_PORT     GPIOB
#define LCD_D5_PIN      DL_GPIO_PIN_3
#define LCD_D6_PORT     GPIOB
#define LCD_D6_PIN      DL_GPIO_PIN_4
#define LCD_D7_PORT     GPIOB
#define LCD_D7_PIN      DL_GPIO_PIN_5

static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++)
        for (volatile uint32_t j = 0; j < 8000; j++);
}

static void delay_us(uint32_t us) {
    volatile uint32_t count = us * 8;
    while (count--);
}

static void lcd_pulse_en(void) {
    DL_GPIO_setPins(LCD_EN_PORT, LCD_EN_PIN);
    delay_us(1);
    DL_GPIO_clearPins(LCD_EN_PORT, LCD_EN_PIN);
    delay_us(100);
}

static void lcd_send_nibble(uint8_t nibble) {
    /* 发送4位数据 */
    DL_GPIO_writePins(LCD_D4_PORT, LCD_D4_PIN, (nibble & 0x01) ? LCD_D4_PIN : 0);
    DL_GPIO_writePins(LCD_D5_PORT, LCD_D5_PIN, (nibble & 0x02) ? LCD_D5_PIN : 0);
    DL_GPIO_writePins(LCD_D6_PORT, LCD_D6_PIN, (nibble & 0x04) ? LCD_D6_PIN : 0);
    DL_GPIO_writePins(LCD_D7_PORT, LCD_D7_PIN, (nibble & 0x08) ? LCD_D7_PIN : 0);
    lcd_pulse_en();
}

static void lcd_send_byte(uint8_t byte, uint8_t mode) {
    DL_GPIO_writePins(LCD_RS_PORT, LCD_RS_PIN, mode ? LCD_RS_PIN : 0);
    lcd_send_nibble(byte >> 4);    /* 高4位 */
    lcd_send_nibble(byte & 0x0F);  /* 低4位 */
    delay_us(50);
}

static void lcd_command(uint8_t cmd) {
    lcd_send_byte(cmd, 0);
}

static void lcd_data(uint8_t data) {
    lcd_send_byte(data, 1);
}

static void lcd_init(void) {
    delay_ms(50);  /* 等待LCD上电稳定 */

    /* 初始化4位模式 */
    lcd_send_nibble(0x03); delay_ms(5);
    lcd_send_nibble(0x03); delay_ms(1);
    lcd_send_nibble(0x03); delay_ms(1);
    lcd_send_nibble(0x02); delay_ms(1);  /* 切换到4位模式 */

    lcd_command(0x28);  /* 4位, 2行, 5x8 */
    lcd_command(0x0C);  /* 显示开, 光标关 */
    lcd_command(0x06);  /* 光标右移 */
    lcd_command(0x01);  /* 清屏 */
    delay_ms(2);
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    lcd_command(addr);
}

static void lcd_print(const char *str) {
    while (*str) lcd_data(*str++);
}

static void lcd_clear(void) {
    lcd_command(0x01);
    delay_ms(2);
}

static void lcd_print_padded(const char *str, uint8_t width) {
    /* 打印字符串并用空格补齐宽度 */
    uint8_t len = 0;
    while (*str && len < width) {
        lcd_data(*str++);
        len++;
    }
    while (len < width) {
        lcd_data(' ');
        len++;
    }
}

/* ========== 旋转编码器驱动 ========== */

/* 引脚 */
#define ENC_CLK_PORT    GPIOA
#define ENC_CLK_PIN     DL_GPIO_PIN_8
#define ENC_DT_PORT     GPIOA
#define ENC_DT_PIN      DL_GPIO_PIN_9
#define ENC_SW_PORT     GPIOA
#define ENC_SW_PIN      DL_GPIO_PIN_10

/* 编码器状态 */
typedef enum {
    ENC_NONE = 0,
    ENC_CW,       /* 顺时针 */
    ENC_CCW,      /* 逆时针 */
    ENC_PRESS,    /* 短按 */
    ENC_LONG_PRESS /* 长按 */
} EncoderEvent_t;

static volatile int8_t  g_encCount = 0;
static volatile bool    g_encClkLast = 0;
static volatile uint16_t g_btnPressTime = 0;
static volatile bool    g_btnPressed = false;
static volatile bool    g_btnHandled = true;

/**
 * @brief GPIO中断处理 - 编码器旋转和按键
 */
void GROUP1_IRQHandler(void) {
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOA);

    /* 旋转检测 */
    if (flags & ENC_CLK_PIN) {
        bool clk = DL_GPIO_readPins(ENC_CLK_PORT, ENC_CLK_PIN) ? true : false;
        bool dt  = DL_GPIO_readPins(ENC_DT_PORT, ENC_DT_PIN) ? true : false;

        if (clk && !g_encClkLast) {  /* CLK上升沿 */
            g_encCount += dt ? -1 : 1;
        }
        g_encClkLast = clk;
        DL_GPIO_clearInterruptStatus(GPIOA, ENC_CLK_PIN);
    }

    /* 按键检测 */
    if (flags & ENC_SW_PIN) {
        bool sw = DL_GPIO_readPins(ENC_SW_PORT, ENC_SW_PIN) ? true : false;
        if (!sw && !g_btnPressed) {  /* 按下 (低电平有效) */
            g_btnPressed = true;
            g_btnPressTime = 0;
            g_btnHandled = false;
        } else if (sw && g_btnPressed) {  /* 松开 */
            g_btnPressed = false;
        }
        DL_GPIO_clearInterruptStatus(GPIOA, ENC_SW_PIN);
    }
}

/**
 * @brief 读取编码器事件
 */
static EncoderEvent_t encoder_read(void) {
    if (!g_btnHandled) {
        if (!g_btnPressed) {
            g_btnHandled = true;
            if (g_btnPressTime > 1000) return ENC_LONG_PRESS;  /* >1s为长按 */
            return ENC_PRESS;
        }
        return ENC_NONE;
    }

    if (g_encCount > 0) {
        g_encCount--;
        return ENC_CW;
    }
    if (g_encCount < 0) {
        g_encCount++;
        return ENC_CCW;
    }

    return ENC_NONE;
}

/* ========== 菜单系统 ========== */

/* 菜单项类型 */
typedef enum {
    MENU_SUB,       /* 子菜单 */
    MENU_VALUE,     /* 可调数值 */
    MENU_TOGGLE,    /* 开关 */
    MENU_ACTION,    /* 执行动作 */
    MENU_BACK       /* 返回上级 */
} MenuItemType_t;

/* 前向声明 */
typedef struct MenuItem MenuItem_t;

/* 数值参数 */
typedef struct {
    float   *value;       /* 值指针 */
    float    min;         /* 最小值 */
    float    max;         /* 最大值 */
    float    step;        /* 步长 */
    const char *fmt;      /* 显示格式 */
} ValueParam_t;

/* 菜单项 */
struct MenuItem {
    const char    *name;        /* 显示名称 */
    MenuItemType_t type;        /* 类型 */
    union {
        MenuItem_t  *submenu;   /* 子菜单指针 */
        ValueParam_t valueParam;/* 数值参数 */
        bool        *toggle;    /* 开关指针 */
        void       (*action)(void); /* 动作函数 */
    };
};

/* ========== 菜单数据定义 ========== */

/* 可调参数 */
static float g_kp = 2.0f;
static float g_ki = 0.5f;
static float g_kd = 1.0f;
static float g_targetTemp = 25.0f;
static float g_threshold = 3.3f;
static bool  g_autoMode = false;
static bool  g_logging = true;
static float g_brightness = 80.0f;

/* 动作函数 */
static void action_calibrate(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Calibrating...");
    /* 模拟校准过程 */
    delay_ms(1500);
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Calibrate Done!");
    delay_ms(1000);
}

static void action_reset(void) {
    g_kp = 2.0f; g_ki = 0.5f; g_kd = 1.0f;
    g_targetTemp = 25.0f; g_threshold = 3.3f;
    g_autoMode = false; g_logging = true;
    g_brightness = 80.0f;

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Reset to default");
    delay_ms(1000);
}

static void action_show_info(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("MSPM0G3507 v1.0");
    lcd_set_cursor(1, 0);
    lcd_print("2026 Contest Lib");
    delay_ms(2000);
}

/* PID参数子菜单 */
static MenuItem_t pidMenu[] = {
    { "Kp",    MENU_VALUE, .valueParam = { &g_kp, 0.0f, 100.0f, 0.1f, "%.2f" } },
    { "Ki",    MENU_VALUE, .valueParam = { &g_ki, 0.0f, 50.0f,  0.01f, "%.3f" } },
    { "Kd",    MENU_VALUE, .valueParam = { &g_kd, 0.0f, 200.0f, 0.5f, "%.2f" } },
    { "< Back", MENU_BACK },
    { NULL, 0 }
};

/* 温控子菜单 */
static MenuItem_t tempMenu[] = {
    { "Target",  MENU_VALUE, .valueParam = { &g_targetTemp, 0.0f, 300.0f, 1.0f, "%.1fC" } },
    { "Auto",    MENU_TOGGLE, .toggle = &g_autoMode },
    { "< Back",  MENU_BACK },
    { NULL, 0 }
};

/* 系统子菜单 */
static MenuItem_t sysMenu[] = {
    { "Brightness", MENU_VALUE, .valueParam = { &g_brightness, 0.0f, 100.0f, 5.0f, "%.0f%%" } },
    { "Logging",    MENU_TOGGLE, .toggle = &g_logging },
    { "Reset",      MENU_ACTION, .action = action_reset },
    { "Info",       MENU_ACTION, .action = action_show_info },
    { "< Back",     MENU_BACK },
    { NULL, 0 }
};

/* 主菜单 */
static MenuItem_t mainMenu[] = {
    { "PID Params",   MENU_SUB, .submenu = pidMenu },
    { "Temperature",  MENU_SUB, .submenu = tempMenu },
    { "Threshold",    MENU_VALUE, .valueParam = { &g_threshold, 0.0f, 5.0f, 0.1f, "%.2fV" } },
    { "Calibrate",    MENU_ACTION, .action = action_calibrate },
    { "System",       MENU_SUB, .submenu = sysMenu },
    { NULL, 0 }
};

/* ========== 菜单状态机 ========== */

#define MENU_MAX_DEPTH  8

typedef struct {
    MenuItem_t *menu;         /* 当前菜单 */
    uint8_t     cursor;       /* 光标位置(0~1) */
    uint8_t     scroll;       /* 滚动偏移 */
    bool        editing;      /* 是否在编辑数值 */
} MenuState_t;

static MenuState_t g_menuStack[MENU_MAX_DEPTH];
static int8_t g_menuDepth = 0;
static MenuState_t *g_cur = &g_menuStack[0];

/**
 * @brief 获取菜单项数量
 */
static uint8_t menu_count(MenuItem_t *menu) {
    uint8_t count = 0;
    while (menu[count].name) count++;
    return count;
}

/**
 * @brief 刷新LCD显示
 */
static void menu_refresh(void) {
    MenuItem_t *menu = g_cur->menu;

    for (uint8_t row = 0; row < 2; row++) {
        uint8_t idx = g_cur->scroll + row;
        lcd_set_cursor(row, 0);

        if (menu[idx].name == NULL) {
            lcd_print_padded("", 16);
            continue;
        }

        /* 光标指示 */
        lcd_data((g_cur->cursor == row) ? '>' : ' ');

        /* 显示菜单项名 */
        char buf[17];
        uint8_t nameLen = strlen(menu[idx].name);

        switch (menu[idx].type) {
            case MENU_VALUE: {
                /* "Name: 12.34" 格式 */
                char valBuf[10];
                snprintf(valBuf, sizeof(valBuf), menu[idx].valueParam.fmt,
                         *menu[idx].valueParam.value);
                snprintf(buf, sizeof(buf), "%-7s%8s", menu[idx].name, valBuf);

                if (g_cur->editing && g_cur->cursor == row) {
                    /* 编辑模式用方括号标记 */
                    lcd_data('[');
                    lcd_print_padded(valBuf, 7);
                    lcd_data(']');
                    /* 补齐剩余 */
                    for (int i = strlen(valBuf) + 2; i < 8; i++) lcd_data(' ');
                } else {
                    lcd_print_padded(buf, 15);
                }
                break;
            }

            case MENU_TOGGLE: {
                bool on = *menu[idx].toggle;
                snprintf(buf, sizeof(buf), "%-8s %s", menu[idx].name, on ? "ON " : "OFF");
                lcd_print_padded(buf, 15);
                break;
            }

            case MENU_SUB: {
                snprintf(buf, sizeof(buf), "%-12s >", menu[idx].name);
                lcd_print_padded(buf, 15);
                break;
            }

            default:
                lcd_print_padded(menu[idx].name, 15);
                break;
        }
    }
}

/**
 * @brief 进入子菜单
 */
static void menu_enter_submenu(MenuItem_t *submenu) {
    if (g_menuDepth >= MENU_MAX_DEPTH - 1) return;
    g_menuDepth++;
    g_menuStack[g_menuDepth].menu = submenu;
    g_menuStack[g_menuDepth].cursor = 0;
    g_menuStack[g_menuDepth].scroll = 0;
    g_menuStack[g_menuDepth].editing = false;
    g_cur = &g_menuStack[g_menuDepth];
    lcd_clear();
    menu_refresh();
}

/**
 * @brief 返回上级菜单
 */
static void menu_back(void) {
    if (g_menuDepth > 0) {
        g_menuDepth--;
        g_cur = &g_menuStack[g_menuDepth];
        g_cur->editing = false;
        lcd_clear();
        menu_refresh();
    }
}

/**
 * @brief 菜单事件处理
 */
static void menu_handle_event(EncoderEvent_t evt) {
    MenuItem_t *curItem = &g_cur->menu[g_cur->scroll + g_cur->cursor];
    uint8_t totalItems = menu_count(g_cur->menu);

    switch (evt) {
        case ENC_CW:  /* 顺时针: 下移/增加值 */
            if (g_cur->editing && curItem->type == MENU_VALUE) {
                ValueParam_t *vp = &curItem->valueParam;
                *vp->value += vp->step;
                if (*vp->value > vp->max) *vp->value = vp->max;
            } else {
                if (g_cur->cursor < 1 && g_cur->scroll + g_cur->cursor + 1 < totalItems)
                    g_cur->cursor++;
                else if (g_cur->scroll + 2 < totalItems)
                    g_cur->scroll++;
            }
            menu_refresh();
            break;

        case ENC_CCW:  /* 逆时针: 上移/减小值 */
            if (g_cur->editing && curItem->type == MENU_VALUE) {
                ValueParam_t *vp = &curItem->valueParam;
                *vp->value -= vp->step;
                if (*vp->value < vp->min) *vp->value = vp->min;
            } else {
                if (g_cur->cursor > 0)
                    g_cur->cursor--;
                else if (g_cur->scroll > 0)
                    g_cur->scroll--;
            }
            menu_refresh();
            break;

        case ENC_PRESS:  /* 短按: 确认/进入 */
            switch (curItem->type) {
                case MENU_SUB:
                    menu_enter_submenu(curItem->submenu);
                    break;
                case MENU_VALUE:
                    g_cur->editing = !g_cur->editing;  /* 切换编辑模式 */
                    menu_refresh();
                    break;
                case MENU_TOGGLE:
                    *curItem->toggle = !(*curItem->toggle);
                    menu_refresh();
                    break;
                case MENU_ACTION:
                    if (curItem->action) curItem->action();
                    lcd_clear();
                    menu_refresh();
                    break;
                case MENU_BACK:
                    menu_back();
                    break;
            }
            break;

        case ENC_LONG_PRESS:  /* 长按: 返回上级 */
            if (g_cur->editing) {
                g_cur->editing = false;  /* 退出编辑模式 */
                menu_refresh();
            } else {
                menu_back();
            }
            break;

        default:
            break;
    }
}

/* ========== 定时器中断: 按键计时 ========== */
void TIMG0_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(TIMG0) == DL_TIMER_IIDX_ZERO) {
        if (g_btnPressed) {
            g_btnPressTime++;
        }
    }
}

/* ========== 主函数 ========== */
int main(void) {
    /* 初始化系统 */
    SYSCFG_DL_init();

    /* 初始化LCD */
    lcd_init();
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Menu System v1.0");
    lcd_set_cursor(1, 0);
    lcd_print("Initializing...");
    delay_ms(1000);

    /* 配置编码器GPIO中断 */
    DL_GPIO_enableInterrupt(GPIOA, ENC_CLK_PIN | ENC_SW_PIN);
    NVIC_EnableIRQ(GROUP1_IRQn);

    /* 启动按键计时定时器 */
    NVIC_EnableIRQ(TIMG0_IRQn);
    DL_TimerG_startCounter(TIMG0);

    /* 初始化菜单 */
    g_menuStack[0].menu = mainMenu;
    g_menuStack[0].cursor = 0;
    g_menuStack[0].scroll = 0;
    g_menuStack[0].editing = false;
    g_cur = &g_menuStack[0];

    lcd_clear();
    menu_refresh();

    /* 主循环 */
    while (1) {
        EncoderEvent_t evt = encoder_read();
        if (evt != ENC_NONE) {
            menu_handle_event(evt);
        }

        delay_ms(10);  /* 消抖间隔 */
    }
}
