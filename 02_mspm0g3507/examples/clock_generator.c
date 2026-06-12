/**
 * @file clock_generator.c
 * @brief 时钟发生器 - Si5351多路输出
 * @platform MSPM0G3507
 * @description
 *   基于Si5351A时钟发生器实现精密多路时钟输出：
 *   - 3路独立时钟输出(CLK0~CLK2)
 *   - 频率范围 8kHz ~ 160MHz
 *   - 频率分辨率 < 1Hz
 *   - 相位可调(0~360度，步进取决于频率)
 *   - 输出使能独立控制
 *   - OLED显示当前配置
 *   - 按键交互式频率调整
 *
 * 硬件连接：
 *   Si5351A: I2C0(PB2-SCL, PB3-SDA), 地址0x60
 *   OLED:    I2C0(与Si5351共享), 地址0x3C
 *   按键:    PA0(通道切换), PA1(频率+), PA2(频率-), PA3(相位+)
 *   LED:     PA27(输出状态指示)
 *
 * Si5351A关键寄存器：
 *   0x02: 输出使能控制
 *   0x10-0x12: CLK0~CLK2控制
 *   0x1A-0x1B: PLL_A参数
 *   0x2A-0x2B: PLL_B参数
 *   0x40-0x47: Multisynth0参数
 *   0x50-0x57: Multisynth1参数
 *   0x60-0x67: Multisynth2参数
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== Si5351寄存器定义 ========== */

#define SI5351_ADDR          0x60     /* I2C地址 */

/* 设备状态 */
#define SI5351_REG_STATUS    0x00
#define SI5351_REG_IRQ       0x01
#define SI5351_REG_OE        0x03     /* 输出使能 */

/* PLL参数 */
#define SI5351_REG_PLLA      0x1A     /* PLL_A控制 */
#define SI5351_REG_PLLB      0x1B     /* PLL_B控制 */
#define SI5351_REG_MSNA_P1   0x1C     /* PLL_A P1 */
#define SI5351_REG_MSNA_P2   0x1E     /* PLL_A P2 */
#define SI5351_REG_MSNA_P3   0x20     /* PLL_A P3 */
#define SI5351_REG_MSNB_P1   0x2C     /* PLL_B P1 */
#define SI5351_REG_MSNB_P2   0x2E     /* PLL_B P2 */
#define SI5351_REG_MSNB_P3   0x30     /* PLL_B P3 */

/* Multisynth参数 */
#define SI5351_REG_MS0_P1    0x2C     /* MS0 P1 */
#define SI5351_REG_MS0_P2    0x2F     /* MS0 P2 */
#define SI5351_REG_MS0_P3    0x2D     /* MS0 P3 */
#define SI5351_REG_MS0_DIV   0x4C     /* MS0分频器 */
#define SI5351_REG_MS1_P1    0x36     /* MS1 P1 */
#define SI5351_REG_MS1_P2    0x39     /* MS1 P2 */
#define SI5351_REG_MS1_P3    0x37     /* MS1 P3 */
#define SI5351_REG_MS1_DIV   0x4D     /* MS1分频器 */
#define SI5351_REG_MS2_P1    0x40     /* MS2 P1 */
#define SI5351_REG_MS2_P2    0x43     /* MS2 P2 */
#define SI5351_REG_MS2_P3    0x41     /* MS2 P3 */
#define SI5351_REG_MS2_DIV   0x4E     /* MS2分频器 */

/* CLK控制寄存器 */
#define SI5351_REG_CLK0_CTRL 0x10
#define SI5351_REG_CLK1_CTRL 0x11
#define SI5351_REG_CLK2_CTRL 0x12

/* 分频器寄存器偏移 */
#define SI5351_REG_MS_BASE   0x2C
#define SI5351_REG_MS_SPAN   0x0A     /* 每个MS寄存器组跨度 */

/* CLK控制位 */
#define SI5351_CLK_POWERDOWN (1 << 7) /* 输出关断 */
#define SI5351_CLK_INTMODE   (1 << 6) /* 整数模式 */
#define SI5351_CLK_PLL_SEL   (1 << 5) /* PLL选择(0=PLLA,1=PLLB) */
#define SI5351_CLK_INV       (1 << 4) /* 输出反相 */
#define SI5351_CLK_SRC_XTAL  (0 << 2) /* 源=晶振 */
#define SI5351_CLK_SRC_MS    (3 << 2) /* 源=Multisynth */

/* 晶振频率 */
#define SI5351_XTAL_FREQ     25000000UL  /* 25MHz晶振 */
#define SI5351_VCO_MIN       600000000UL /* VCO最小600MHz */
#define SI5351_VCO_MAX       900000000UL /* VCO最大900MHz */

/* PLL参数结构体 */
typedef struct {
    uint8_t a;      /* 整数分频比 (15..90) */
    uint32_t b;     /* 分子 */
    uint32_t c;     /* 分母 */
} PLL_Params_t;

/* Multisynth参数结构体 */
typedef struct {
    uint8_t a;      /* 整数分频比 (4..900) */
    uint32_t b;     /* 分子 */
    uint32_t c;     /* 分母 */
    uint8_t div;    /* R分频器 (1,2,4,8,16,32,64,128) */
} MS_Params_t;

/* 时钟通道结构体 */
typedef struct {
    uint32_t freq_hz;       /* 输出频率 */
    uint16_t phase_deg;     /* 相位(度) */
    bool enabled;           /* 输出使能 */
    uint8_t pll_sel;        /* PLL选择(0=A, 1=B) */
    PLL_Params_t pll;       /* PLL参数 */
    MS_Params_t ms;         /* Multisynth参数 */
} ClockChannel_t;

/* ========== 全局变量 ========== */

#define NUM_CHANNELS  3

static ClockChannel_t channels[NUM_CHANNELS] = {
    { .freq_hz = 10000000, .phase_deg = 0,   .enabled = true, .pll_sel = 0 },
    { .freq_hz = 10000000, .phase_deg = 90,  .enabled = true, .pll_sel = 0 },
    { .freq_hz = 1000000,  .phase_deg = 0,   .enabled = true, .pll_sel = 1 },
};

static uint8_t current_ch = 0;     /* 当前选中通道 */

/* 频率步进 */
static const uint32_t freq_steps[] = {
    1, 10, 100, 1000, 10000, 100000, 1000000, 10000000
};
static uint8_t step_idx = 4;       /* 默认步进10kHz */

/* ========== I2C通信函数 ========== */

/**
 * @brief 向Si5351写入单个寄存器
 */
static void Si5351_WriteReg(uint8_t reg, uint8_t value)
{
    DL_I2C_flushControllerTXFIFO(I2C0);
    uint8_t data[2] = {reg, value};
    DL_I2C_fillControllerTXFIFO(I2C0, data, 2);
    DL_I2C_startControllerTransfer(I2C0, SI5351_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C0) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
}

/**
 * @brief 从Si5351读取单个寄存器
 */
static uint8_t Si5351_ReadReg(uint8_t reg)
{
    DL_I2C_flushControllerTXFIFO(I2C0);
    DL_I2C_flushControllerRXFIFO(I2C0);
    DL_I2C_fillControllerTXFIFO(I2C0, &reg, 1);
    DL_I2C_startControllerTransfer(I2C0, SI5351_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C0) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}

    DL_I2C_startControllerTransfer(I2C0, SI5351_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_isControllerRXFIFOEmpty(I2C0)) {}
    return DL_I2C_receiveControllerData(I2C0);
}

/**
 * @brief 向Si5351连续写入多个寄存器
 */
static void Si5351_WriteBurst(uint8_t start_reg, const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        Si5351_WriteReg(start_reg + i, data[i]);
    }
}

/* ========== Si5351核心驱动 ========== */

/**
 * @brief 初始化Si5351
 */
static void Si5351_Init(void)
{
    /* 停止所有输出 */
    Si5351_WriteReg(SI5351_REG_OE, 0xFF);  /* 禁止所有输出 */

    /* 软复位PLL */
    Si5351_WriteReg(SI5351_REG_PLLA, 0xA0);
    Si5351_WriteReg(SI5351_REG_PLLB, 0xA0);
    Si5351_WriteReg(SI5351_REG_PLLA, 0x20);
    Si5351_WriteReg(SI5351_REG_PLLB, 0x20);

    /* 配置晶振负载电容 */
    Si5351_WriteReg(0xB7, 0xD2);  /* XTAL内部负载电容10pF */

    /* 禁用Fanout */
    Si5351_WriteReg(0xBB, 0x00);  /* 禁用CLKIN/XTAL/MS0-2 Fanout */
    Si5351_WriteReg(0xBC, 0x00);

    /* 设置所有CLK为PowerDown */
    for (uint8_t i = 0; i < 8; i++) {
        Si5351_WriteReg(SI5351_REG_CLK0_CTRL + i, SI5351_CLK_POWERDOWN);
    }
}

/**
 * @brief 计算PLL参数 (a + b/c)
 * @param freq_hz 目标VCO频率
 * @param params 输出PLL参数
 */
static void Si5351_CalcPLL(uint32_t vco_freq, PLL_Params_t *params)
{
    /* a = vco_freq / xtal_freq */
    params->a = vco_freq / SI5351_XTAL_FREQ;

    /* 余数 = vco_freq - a * xtal_freq */
    uint32_t remainder = vco_freq - params->a * SI5351_XTAL_FREQ;

    /* 使用简化的b/c(实际可用最大精度) */
    params->b = remainder;
    params->c = SI5351_XTAL_FREQ;

    /* 约分简化b/c */
    if (params->c > 0xFFFFF) {
        /* 如果分母过大，进行近似 */
        double ratio = (double)params->b / (double)params->c;
        params->c = 1000000;
        params->b = (uint32_t)(ratio * params->c);
    }
}

/**
 * @brief 计算Multisynth参数 (a + b/c) / R
 * @param vco_freq VCO频率
 * @param out_freq 输出频率
 * @param params 输出MS参数
 */
static void Si5351_CalcMS(uint32_t vco_freq, uint32_t out_freq, MS_Params_t *params)
{
    /* 确定R分频器 */
    params->div = 1;
    uint32_t ms_div = vco_freq / out_freq;

    while (ms_div > 900 && params->div < 128) {
        params->div *= 2;
        ms_div = vco_freq / (out_freq * params->div);
    }

    /* a = 分频比整数部分 */
    params->a = (uint8_t)ms_div;

    /* 余数 */
    uint32_t remainder = vco_freq - params->a * out_freq * params->div;

    params->b = remainder;
    params->c = out_freq * params->div;

    /* 约分 */
    if (params->c > 0xFFFFF) {
        double ratio = (double)params->b / (double)params->c;
        params->c = 1000000;
        params->b = (uint32_t)(ratio * params->c);
    }

    /* 防止b=0且c=0 */
    if (params->b == 0) params->c = 1;
}

/**
 * @brief 写入PLL参数到Si5351
 * @param pll_sel PLL选择(0=A, 1=B)
 * @param params PLL参数
 */
static void Si5351_WritePLL(uint8_t pll_sel, const PLL_Params_t *params)
{
    uint8_t base = pll_sel ? SI5351_REG_MSNB_P3 : SI5351_REG_MSNA_P3;

    /* P3 (2字节) */
    Si5351_WriteReg(base,     (params->c >> 8) & 0xFF);
    Si5351_WriteReg(base + 1, params->c & 0xFF);

    /* P1 (3字节) - P1 = 128*a + floor(128*b/c) - 512 */
    uint32_t p1 = 128UL * params->a +
                  (128UL * params->b / params->c) - 512;
    Si5351_WriteReg(base + 2, (p1 >> 16) & 0x03);
    Si5351_WriteReg(base + 3, (p1 >> 8) & 0xFF);
    Si5351_WriteReg(base + 4, p1 & 0xFF);

    /* P2 (3字节) - P2 = 128*b - c*floor(128*b/c) */
    uint32_t p2 = 128UL * params->b - params->c * (128UL * params->b / params->c);
    Si5351_WriteReg(base + 5, (p2 >> 16) & 0x03);
    Si5351_WriteReg(base + 6, (p2 >> 8) & 0xFF);
    Si5351_WriteReg(base + 7, p2 & 0xFF);
}

/**
 * @brief 写入Multisynth参数到Si5351
 * @param ch 通道号(0~2)
 * @param params MS参数
 */
static void Si5351_WriteMS(uint8_t ch, const MS_Params_t *params)
{
    uint8_t base = SI5351_REG_MS_BASE + ch * SI5351_REG_MS_SPAN;

    /* P3 (2字节) */
    Si5351_WriteReg(base,     (params->c >> 8) & 0xFF);
    Si5351_WriteReg(base + 1, params->c & 0xFF);

    /* P1 (3字节) */
    uint32_t p1 = 128UL * params->a +
                  (128UL * params->b / params->c) - 512;
    Si5351_WriteReg(base + 2, (p1 >> 16) & 0x03);
    Si5351_WriteReg(base + 3, (p1 >> 8) & 0xFF);
    Si5351_WriteReg(base + 4, p1 & 0xFF);

    /* P2 (3字节) */
    uint32_t p2 = 128UL * params->b - params->c * (128UL * params->b / params->c);
    Si5351_WriteReg(base + 5, (p2 >> 16) & 0x03);
    Si5351_WriteReg(base + 6, (p2 >> 8) & 0xFF);
    Si5351_WriteReg(base + 7, p2 & 0xFF);

    /* R分频器 */
    uint8_t r_val = 0;
    switch (params->div) {
    case 1:   r_val = 0; break;
    case 2:   r_val = 1; break;
    case 4:   r_val = 2; break;
    case 8:   r_val = 3; break;
    case 16:  r_val = 4; break;
    case 32:  r_val = 5; break;
    case 64:  r_val = 6; break;
    case 128: r_val = 7; break;
    }
    Si5351_WriteReg(SI5351_REG_MS0_DIV + ch, r_val << 4);
}

/**
 * @brief 设置通道频率
 * @param ch 通道号(0~2)
 * @param freq_hz 目标频率
 */
static void Si5351_SetFrequency(uint8_t ch, uint32_t freq_hz)
{
    if (ch >= NUM_CHANNELS) return;

    ClockChannel_t *c = &channels[ch];
    c->freq_hz = freq_hz;

    /* 选择PLL: 需要相位控制的通道使用同一PLL */
    /* CLK0和CLK1使用PLLA(实现相位差), CLK2使用PLLB */
    c->pll_sel = (ch < 2) ? 0 : 1;

    /* 计算VCO频率(目标输出频率的整数倍，在600~900MHz范围) */
    uint32_t vco_freq;
    uint32_t ms_div = SI5351_VCO_MIN / freq_hz;
    if (ms_div < 4) ms_div = 4;
    vco_freq = freq_hz * ms_div;

    /* 确保VCO在有效范围 */
    while (vco_freq < SI5351_VCO_MIN) {
        vco_freq += freq_hz;
    }
    while (vco_freq > SI5351_VCO_MAX) {
        vco_freq -= freq_hz;
    }

    /* 计算PLL和Multisynth参数 */
    Si5351_CalcPLL(vco_freq, &c->pll);
    Si5351_CalcMS(vco_freq, freq_hz, &c->ms);

    /* 写入PLL参数 */
    Si5351_WritePLL(c->pll_sel, &c->pll);

    /* 写入Multisynth参数 */
    Si5351_WriteMS(ch, &c->ms);

    /* 配置CLK控制寄存器 */
    uint8_t clk_ctrl = SI5351_CLK_SRC_MS;  /* 源=Multisynth */
    if (c->pll_sel) clk_ctrl |= SI5351_CLK_PLL_SEL;  /* PLLB */
    Si5351_WriteReg(SI5351_REG_CLK0_CTRL + ch, clk_ctrl);

    /* 处理相位(仅限共享PLL的通道) */
    if (ch < 2 && c->phase_deg > 0) {
        /* 相位偏移 = phase_deg / 90 * (VCO_freq / out_freq) / 4 */
        /* 简化: 1个相位寄存器单位 = 1/4 VCO周期 */
        uint8_t phase_val = (uint8_t)((c->phase_deg / 90.0) *
            (vco_freq / freq_hz) / 4.0);
        Si5351_WriteReg(0xA5 + ch, phase_val);  /* CLK0_PHOFF=0xA5, CLK1_PHOFF=0xA6 */
    }

    /* 重置PLL */
    Si5351_WriteReg(0xB1, 0xA0);  /* PLL复位 */
}

/**
 * @brief 设置通道相位
 * @param ch 通道号(0~1，仅CLK0和CLK1支持)
 * @param phase_deg 相位角度(0~359)
 */
static void Si5351_SetPhase(uint8_t ch, uint16_t phase_deg)
{
    if (ch >= 2) return;  /* 仅CLK0/CLK1 */

    channels[ch].phase_deg = phase_deg;

    /* 重新计算并应用 */
    Si5351_SetFrequency(ch, channels[ch].freq_hz);
}

/**
 * @brief 使能/禁止输出
 * @param ch 通道号
 * @param enable 使能标志
 */
static void Si5351_EnableOutput(uint8_t ch, bool enable)
{
    if (ch >= NUM_CHANNELS) return;

    channels[ch].enabled = enable;

    /* 读取当前OE寄存器 */
    uint8_t oe = Si5351_ReadReg(SI5351_REG_OE);
    if (enable) {
        oe &= ~(1 << ch);  /* 清除对应位使能输出 */
    } else {
        oe |= (1 << ch);   /* 置位对应位禁止输出 */
    }
    Si5351_WriteReg(SI5351_REG_OE, oe);
}

/**
 * @brief 更新所有输出使能状态
 */
static void Si5351_UpdateOE(void)
{
    uint8_t oe = 0;
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        if (!channels[ch].enabled) {
            oe |= (1 << ch);
        }
    }
    Si5351_WriteReg(SI5351_REG_OE, oe);
}

/* ========== OLED显示 ========== */

/**
 * @brief 更新OLED显示
 */
static void Display_Update(void)
{
    char buf[32];
    char lines[4][32];

    /* 通道标题 */
    snprintf(lines[0], 32, "CH%d: %s  %s",
        current_ch,
        channels[current_ch].enabled ? "ON" : "OFF",
        channels[current_ch].pll_sel ? "PLLB" : "PLLA");

    /* 频率显示 */
    uint32_t f = channels[current_ch].freq_hz;
    if (f >= 1000000) {
        snprintf(lines[1], 32, "Freq: %lu.%03lu MHz",
            f / 1000000, (f % 1000000) / 1000);
    } else if (f >= 1000) {
        snprintf(lines[1], 32, "Freq: %lu.%03lu kHz",
            f / 1000, f % 1000);
    } else {
        snprintf(lines[1], 32, "Freq: %lu Hz", f);
    }

    /* 相位显示 */
    snprintf(lines[2], 32, "Phase: %u deg", channels[current_ch].phase_deg);

    /* 步进显示 */
    uint32_t step = freq_steps[step_idx];
    if (step >= 1000000) {
        snprintf(lines[3], 32, "Step: %lu MHz", step / 1000000);
    } else if (step >= 1000) {
        snprintf(lines[3], 32, "Step: %lu kHz", step / 1000);
    } else {
        snprintf(lines[3], 32, "Step: %lu Hz", step);
    }

    /* 调用OLED驱动刷新 */
    /* OLED_ShowLines(lines, 4); */
}

/* ========== 按键处理 ========== */

static volatile uint32_t sys_tick = 0;

void SysTick_Handler(void) { sys_tick++; }

/**
 * @brief 按键扫描
 */
static void Key_Process(void)
{
    static uint32_t last_key[4] = {0};
    uint32_t debounce = 200;  /* ms */

    /* PA0: 通道切换 */
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_0) == 0) {
        if (sys_tick - last_key[0] > debounce) {
            last_key[0] = sys_tick;
            current_ch = (current_ch + 1) % NUM_CHANNELS;
            Display_Update();
        }
    }

    /* PA1: 频率增加 */
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_1) == 0) {
        if (sys_tick - last_key[1] > debounce) {
            last_key[1] = sys_tick;
            uint32_t new_freq = channels[current_ch].freq_hz + freq_steps[step_idx];
            if (new_freq <= 160000000) {
                Si5351_SetFrequency(current_ch, new_freq);
                Si5351_UpdateOE();
                Display_Update();
            }
        }
    }

    /* PA3: 频率减少 */
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_3) == 0) {
        if (sys_tick - last_key[2] > debounce) {
            last_key[2] = sys_tick;
            uint32_t f = channels[current_ch].freq_hz;
            if (f > freq_steps[step_idx]) {
                Si5351_SetFrequency(current_ch, f - freq_steps[step_idx]);
            } else {
                Si5351_SetFrequency(current_ch, 8000);  /* 最小8kHz */
            }
            Si5351_UpdateOE();
            Display_Update();
        }
    }

    /* PA4: 步进/相位切换(长按切相位) */
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_4) == 0) {
        if (sys_tick - last_key[3] > debounce) {
            last_key[3] = sys_tick;

            /* 短按: 切换步进 */
            step_idx = (step_idx + 1) % 8;
            Display_Update();
        }
    }

    /* PA5: 相位增加(90度步进) */
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_5) == 0) {
        static uint32_t last_ph = 0;
        if (sys_tick - last_ph > debounce) {
            last_ph = sys_tick;
            uint16_t new_phase = (channels[current_ch].phase_deg + 90) % 360;
            Si5351_SetPhase(current_ch, new_phase);
            Display_Update();
        }
    }
}

/* ========== 预设频率方案 ========== */

/* 预设方案结构体 */
typedef struct {
    const char *name;
    uint32_t freqs[NUM_CHANNELS];
    uint16_t phases[NUM_CHANNELS];
} PresetScheme_t;

static const PresetScheme_t presets[] = {
    /* 标准正交时钟 */
    {"Quadrature", {10000000, 10000000, 1000000}, {0, 90, 0}},
    /* 三相时钟 */
    {"3-Phase", {10000000, 10000000, 10000000}, {0, 120, 240}},
    /* 通信时钟 */
    {"UART CLK", {1843200, 11059200, 24000000}, {0, 0, 0}},
    /* I2S音频时钟 */
    {"I2S Audio", {12288000, 12288000, 24576000}, {0, 0, 0}},
    /* USB时钟 */
    {"USB CLK", {48000000, 48000000, 48000000}, {0, 0, 0}},
};

#define PRESET_COUNT  5

/**
 * @brief 应用预设方案
 */
static void Apply_Preset(uint8_t idx)
{
    if (idx >= PRESET_COUNT) return;

    const PresetScheme_t *p = &presets[idx];
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        channels[ch].freq_hz = p->freqs[ch];
        channels[ch].phase_deg = p->phases[ch];
        channels[ch].enabled = true;
    }

    /* 逐通道配置 */
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        Si5351_SetFrequency(ch, channels[ch].freq_hz);
    }
    Si5351_UpdateOE();
    Display_Update();
}

/* ========== 主函数 ========== */

int main(void)
{
    /* 系统初始化 */
    DL_SYSCFG_init();
    SysTick_Config(32000000 / 1000);  /* 1ms SysTick */

    /* GPIO初始化(按键) */
    uint32_t key_pins[] = {
        DL_GPIO_PIN_0, DL_GPIO_PIN_1, DL_GPIO_PIN_3,
        DL_GPIO_PIN_4, DL_GPIO_PIN_5
    };
    for (uint8_t i = 0; i < 5; i++) {
        DL_GPIO_initDigitalInputFeatures(key_pins[i],
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
            DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    }

    /* LED */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_27);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_27);

    /* I2C初始化 */
    DL_I2C_reset(I2C0);
    DL_I2C_enablePower(I2C0);
    DL_I2C_setClockConfig(I2C0, DL_I2C_CLOCK_DIVIDE_400KHZ);
    DL_I2C_enableController(I2C0);

    /* Si5351初始化 */
    Si5351_Init();

    /* 应用默认配置(CLK0=10MHz, CLK1=10MHz/90°, CLK2=1MHz) */
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        Si5351_SetFrequency(ch, channels[ch].freq_hz);
    }
    Si5351_UpdateOE();

    /* LED指示就绪 */
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27);

    /* 更新显示 */
    Display_Update();

    /* 主循环 */
    while (1) {
        Key_Process();
        __WFI();
    }
}
