/**
 * @file    encoder_gpio_mspm0.c
 * @brief   GPIO中断方式编码器驱动实现 — MSPM0G3507
 * @note    使用GPIO外部中断实现正交解码，适用于N20等带霍尔编码器的电机
 *          基于天猛星MSPM0G3507模块移植代码优化
 */

#include "encoder_gpio_mspm0.h"

/* ── 私有变量 ────────────────────────────────────────────── */
static EncoderGpioConfig g_enc_cfg[ENCODER_GPIO_MAX];
static EncoderGpioData   g_enc_data[ENCODER_GPIO_MAX];
static volatile uint8_t  g_enc_initialized = 0;

/* ── 内部函数 ────────────────────────────────────────────── */

/**
 * @brief 处理单个编码器的中断
 * @param id  编码器编号
 */
static void EncoderGpio_ProcessInterrupt(EncoderGpioId id)
{
    EncoderGpioConfig *cfg = &g_enc_cfg[id];
    EncoderGpioData   *data = &g_enc_data[id];
    
    /* 读取A相和B相当前状态 */
    uint8_t a_state = GPIO_READ(cfg->port, cfg->pin_a) ? 1 : 0;
    uint8_t b_state = GPIO_READ(cfg->port, cfg->pin_b) ? 1 : 0;
    
    /* 正交解码逻辑 */
    if (a_state) {
        /* A相上升沿 */
        if (b_state) {
            data->count--;  /* B相高，反转 */
            data->dir = 1;
        } else {
            data->count++;  /* B相低，正转 */
            data->dir = 0;
        }
    } else {
        /* A相下降沿 */
        if (b_state) {
            data->count++;  /* B相高，正转 */
            data->dir = 0;
        } else {
            data->count--;  /* B相低，反转 */
            data->dir = 1;
        }
    }
    
    /* 应用方向修正 */
    if (cfg->inverted) {
        data->dir = !data->dir;
    }
}

/* ── GROUP1中断处理函数 ──────────────────────────────────── */
/* 注意：此函数需要在实际工程中根据具体引脚配置进行调整 */
void GROUP1_IRQHandler(void)
{
    volatile uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOA, 0xFFFFFFFF);
    
    /* 处理左轮编码器中断 */
    if (flags & (g_enc_cfg[ENCODER_GPIO_LEFT].pin_a | g_enc_cfg[ENCODER_GPIO_LEFT].pin_b)) {
        EncoderGpio_ProcessInterrupt(ENCODER_GPIO_LEFT);
    }
    
    /* 处理右轮编码器中断 */
    if (flags & (g_enc_cfg[ENCODER_GPIO_RIGHT].pin_a | g_enc_cfg[ENCODER_GPIO_RIGHT].pin_b)) {
        EncoderGpio_ProcessInterrupt(ENCODER_GPIO_RIGHT);
    }
    
    /* 清除中断标志 */
    DL_GPIO_clearInterruptStatus(GPIOA, flags);
}

/* ── 公开API ─────────────────────────────────────────────── */

void EncoderGpio_Init(const EncoderGpioConfig *cfg)
{
    /* 保存配置 */
    for (int i = 0; i < ENCODER_GPIO_MAX; i++) {
        g_enc_cfg[i] = cfg[i];
        g_enc_data[i].count = 0;
        g_enc_data[i].last_count = 0;
        g_enc_data[i].speed = 0;
        g_enc_data[i].dir = 0;
    }
    
    /* 配置GPIO中断 */
    for (int i = 0; i < ENCODER_GPIO_MAX; i++) {
        /* 启用A相和B相的中断 */
        DL_GPIO_enableInterrupt(g_enc_cfg[i].port, g_enc_cfg[i].pin_a | g_enc_cfg[i].pin_b);
    }
    
    /* 启用GROUP1中断 */
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    
    g_enc_initialized = 1;
}

int32_t EncoderGpio_Read(EncoderGpioId id)
{
    if (id >= ENCODER_GPIO_MAX || !g_enc_initialized) return 0;
    
    int32_t count = g_enc_data[id].count;
    
    /* 应用方向修正 */
    if (g_enc_cfg[id].inverted) {
        count = -count;
    }
    
    return count;
}

int32_t EncoderGpio_GetSpeed(EncoderGpioId id)
{
    if (id >= ENCODER_GPIO_MAX || !g_enc_initialized) return 0;
    
    int32_t speed = g_enc_data[id].speed;
    
    /* 应用方向修正 */
    if (g_enc_cfg[id].inverted) {
        speed = -speed;
    }
    
    return speed;
}

void EncoderGpio_Reset(EncoderGpioId id)
{
    if (id >= ENCODER_GPIO_MAX || !g_enc_initialized) return;
    
    g_enc_data[id].count = 0;
    g_enc_data[id].last_count = 0;
    g_enc_data[id].speed = 0;
}

uint8_t EncoderGpio_GetDirection(EncoderGpioId id)
{
    if (id >= ENCODER_GPIO_MAX || !g_enc_initialized) return 0;
    return g_enc_data[id].dir;
}

void EncoderGpio_SetInverted(EncoderGpioId id, uint8_t inv)
{
    if (id >= ENCODER_GPIO_MAX) return;
    g_enc_cfg[id].inverted = inv;
}

void EncoderGpio_Update(void)
{
    if (!g_enc_initialized) return;
    
    /* 计算速度 = 当前计数 - 上次计数 */
    for (int i = 0; i < ENCODER_GPIO_MAX; i++) {
        int32_t current = g_enc_data[i].count;
        g_enc_data[i].speed = current - g_enc_data[i].last_count;
        g_enc_data[i].last_count = current;
    }
}