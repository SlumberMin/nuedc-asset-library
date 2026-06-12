/**
 * @file    jy901s.c
 * @brief   JY901S 九轴IMU UART驱动实现 — MSPM0G3507
 *
 * SysConfig生成的宏:
 *   UART_1_INST, UART_1_INST_IRQHandler, UART_1_INST_INT_IRQN
 *
 * JY901S数据帧格式 (11字节):
 *   Byte 0:   0x55 (帧头)
 *   Byte 1:   类型 (0x51=加速度, 0x52=角速度, 0x53=角度, 0x54=磁场)
 *   Byte 2-9: 数据 (8字节, 小端序)
 *   Byte 10:  校验和 = Byte0 + Byte1 + ... + Byte9 的低8位
 */

#include "drivers/jy901s.h"
#include <string.h>

/* ── 帧定义 ────────────────────────────────────────────────── */
#define JY901S_FRAME_HEADER     0x55
#define JY901S_FRAME_LEN        11

#define JY901S_TYPE_ACC         0x51    /* 加速度 */
#define JY901S_TYPE_GYRO        0x52    /* 角速度 */
#define JY901S_TYPE_ANGLE       0x53    /* 角度 */
#define JY901S_TYPE_MAG         0x54    /* 磁场 */

/* ── 解析状态机 ────────────────────────────────────────────── */
typedef enum {
    STATE_WAIT_HEADER = 0,  /* 等待帧头 0x55 */
    STATE_WAIT_TYPE,        /* 等待类型字节 */
    STATE_RECV_DATA,        /* 接收8字节数据 */
    STATE_RECV_CHECKSUM,    /* 接收校验和 */
} ParseState;

/* ── 内部变量 (ISR共享, 需volatile) ─────────────────────── */
static volatile JY901S_Data  g_jy901s_data;
static volatile ParseState   g_state         = STATE_WAIT_HEADER;
static volatile uint8_t      g_frame[JY901S_FRAME_LEN];
static volatile uint8_t      g_frame_idx     = 0;
static volatile uint8_t      g_data_count    = 0;

/* ── 内部函数 ──────────────────────────────────────────────── */

/**
 * @brief 处理一帧完整的JY901S数据
 */
static void JY901S_ProcessFrame(const volatile uint8_t *frame)
{
    uint8_t type = frame[1];

    /* 校验和验证 */
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += frame[i];
    }
    if (sum != frame[10]) {
        return;  /* 校验和错误，丢弃 */
    }

    /* 解析数据 (小端序) */
    switch (type) {
    case JY901S_TYPE_ACC:
        /* 加速度, 单位: 32768 = 16g, 即 1g = 32768/16 = 2048 */
        g_jy901s_data.acc_x = (int16_t)(frame[3] << 8 | frame[2]) / 32768.0f * 16.0f;
        g_jy901s_data.acc_y = (int16_t)(frame[5] << 8 | frame[4]) / 32768.0f * 16.0f;
        g_jy901s_data.acc_z = (int16_t)(frame[7] << 8 | frame[6]) / 32768.0f * 16.0f;
        g_jy901s_data.acc_updated = true;
        break;

    case JY901S_TYPE_GYRO:
        /* 角速度, 单位: 32768 = 2000°/s */
        g_jy901s_data.gyro_x = (int16_t)(frame[3] << 8 | frame[2]) / 32768.0f * 2000.0f;
        g_jy901s_data.gyro_y = (int16_t)(frame[5] << 8 | frame[4]) / 32768.0f * 2000.0f;
        g_jy901s_data.gyro_z = (int16_t)(frame[7] << 8 | frame[6]) / 32768.0f * 2000.0f;
        g_jy901s_data.gyro_updated = true;
        break;

    case JY901S_TYPE_ANGLE:
        /* 角度, 单位: 0.01度 → 除以100得到度 */
        g_jy901s_data.roll  = (int16_t)(frame[3] << 8 | frame[2]) / 100.0f;
        g_jy901s_data.pitch = (int16_t)(frame[5] << 8 | frame[4]) / 100.0f;
        g_jy901s_data.yaw   = (int16_t)(frame[7] << 8 | frame[6]) / 100.0f;
        g_jy901s_data.angle_updated = true;
        break;

    case JY901S_TYPE_MAG:
        /* 磁场, 原始值 */
        g_jy901s_data.mag_x = (int16_t)(frame[3] << 8 | frame[2]);
        g_jy901s_data.mag_y = (int16_t)(frame[5] << 8 | frame[4]);
        g_jy901s_data.mag_z = (int16_t)(frame[7] << 8 | frame[6]);
        break;

    default:
        break;
    }
}

/**
 * @brief 解析接收字节（状态机）
 */
static void JY901S_ParseByte(uint8_t byte)
{
    switch (g_state) {
    case STATE_WAIT_HEADER:
        if (byte == JY901S_FRAME_HEADER) {
            g_frame[0] = byte;
            g_frame_idx = 1;
            g_state = STATE_WAIT_TYPE;
        }
        break;

    case STATE_WAIT_TYPE:
        g_frame[1] = byte;
        g_frame_idx = 2;
        g_data_count = 0;
        g_state = STATE_RECV_DATA;
        break;

    case STATE_RECV_DATA:
        g_frame[g_frame_idx++] = byte;
        g_data_count++;
        if (g_data_count >= 8) {
            g_state = STATE_RECV_CHECKSUM;
        }
        break;

    case STATE_RECV_CHECKSUM:
        g_frame[10] = byte;
        JY901S_ProcessFrame(g_frame);
        g_state = STATE_WAIT_HEADER;
        break;

    default:
        g_state = STATE_WAIT_HEADER;
        break;
    }
}

/* ── 公开API ──────────────────────────────────────────────── */

void JY901S_Init(void)
{
    memset(&g_jy901s_data, 0, sizeof(g_jy901s_data));
    g_state = STATE_WAIT_HEADER;
    g_frame_idx = 0;
    g_data_count = 0;

    /* 使能UART1接收中断 */
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    DL_UART_enableInterrupt(UART_1_INST, DL_UART_INTERRUPT_RX);
}

const volatile JY901S_Data* JY901S_GetData(void)
{
    return &g_jy901s_data;
}

float JY901S_GetRoll(void)
{
    return g_jy901s_data.roll;
}

float JY901S_GetPitch(void)
{
    return g_jy901s_data.pitch;
}

float JY901S_GetYaw(void)
{
    return g_jy901s_data.yaw;
}

bool JY901S_IsAngleUpdated(void)
{
    /* 临界区: 原子性地读取并清除标志, 防止ISR竞争 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool updated = g_jy901s_data.angle_updated;
    if (updated) {
        g_jy901s_data.angle_updated = false;
    }
    __set_PRIMASK(primask);
    return updated;
}

void JY901S_UART_IRQHandler(void)
{
    volatile uint8_t data;

    /* 检查是否有接收数据 */
    switch (DL_UART_getPendingInterrupt(UART_1_INST)) {
    case DL_UART_IIDX_RX:
        data = DL_UART_receiveData(UART_1_INST);
        JY901S_ParseByte(data);
        break;
    default:
        break;
    }
}
