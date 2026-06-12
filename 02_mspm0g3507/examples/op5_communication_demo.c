/**
 * @file op5_communication_demo.c
 * @brief OPi5通信完整示例（协议解析+命令处理+传感器上报）
 * @platform MSPM0G3507
 *
 * ============================================================
 * 接线说明（MSPM0G3507 <-> Orange Pi 5）
 * ============================================================
 * MSPM0G3507        OPi5                说明
 * PA8  (UART0_TX) -> GPIO1 (UART4_RX)  MSP发送 -> OPi5接收
 * PA9  (UART0_RX) <- GPIO2 (UART4_TX)  OPi5发送 -> MSP接收
 * GND              -> GND               共地
 *
 * 注意：OPi5为3.3V/1.8V逻辑电平，若MSPM0为5V需电平转换
 *
 * ============================================================
 * 通信协议格式（自定义二进制协议）
 * ============================================================
 * 帧结构:
 * | 帧头(2B) | 长度(1B) | 命令字(1B) | 数据(N B) | CRC16(2B) |
 * 帧头: 0xAA 0x55
 * 长度: 命令字+数据的字节数
 * CRC16: 对长度+命令字+数据部分计算
 *
 * 命令字定义:
 * 0x01 - 握手请求       数据: 空
 * 0x02 - 握手应答       数据: 版本号(1B)
 * 0x10 - 查询传感器数据 数据: 传感器ID(1B)
 * 0x11 - 传感器数据上报 数据: 传感器ID(1B) + 数据(4B float)
 * 0x20 - 设置电机速度   数据: 通道(1B) + 速度(2B, signed)
 * 0x21 - 设置电机应答   数据: 状态(1B)
 * 0x30 - 设置PID参数    数据: 通道(1B) + Kp(4B) + Ki(4B) + Kd(4B)
 * 0x31 - PID参数应答    数据: 状态(1B)
 * 0xF0 - 错误报告       数据: 错误码(1B)
 * 0xFF - 心跳包         数据: 空
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ======================== 协议常量定义 ======================== */
#define PROTO_HEADER_0          0xAA    /* 帧头第一字节 */
#define PROTO_HEADER_1          0x55    /* 帧头第二字节 */
#define PROTO_MAX_DATA_LEN      64      /* 最大数据长度 */
#define PROTO_MAX_FRAME_LEN     72      /* 最大帧长度 */

/* 命令字定义 */
#define CMD_HANDSHAKE_REQ       0x01    /* 握手请求 */
#define CMD_HANDSHAKE_ACK       0x02    /* 握手应答 */
#define CMD_QUERY_SENSOR        0x10    /* 查询传感器 */
#define CMD_SENSOR_REPORT       0x11    /* 传感器上报 */
#define CMD_SET_MOTOR           0x20    /* 设置电机 */
#define CMD_MOTOR_ACK           0x21    /* 电机应答 */
#define CMD_SET_PID             0x30    /* 设置PID参数 */
#define CMD_PID_ACK             0x31    /* PID应答 */
#define CMD_ERROR               0xF0    /* 错误报告 */
#define CMD_HEARTBEAT           0xFF    /* 心跳包 */

/* 传感器ID定义 */
#define SENSOR_ADC_0            0x00    /* ADC通道0 */
#define SENSOR_ADC_1            0x01    /* ADC通道1 */
#define SENSOR_IMU_ACCEL_X      0x10    /* 加速度计X */
#define SENSOR_IMU_ACCEL_Y      0x11    /* 加速度计Y */
#define SENSOR_IMU_GYRO_Z       0x12    /* 陀螺仪Z */
#define SENSOR_GRAY_LINE        0x20    /* 灰度循迹值 */
#define SENSOR_BATTERY          0x30    /* 电池电压 */

/* 错误码定义 */
#define ERR_NONE                0x00
#define ERR_CRC_FAIL            0x01    /* CRC校验失败 */
#define ERR_INVALID_CMD         0x02    /* 无效命令 */
#define ERR_INVALID_DATA        0x03    /* 数据错误 */
#define ERR_TIMEOUT             0x04    /* 超时 */

/* 状态常量 */
#define HANDSHAKE_TIMEOUT_MS    3000    /* 握手超时3秒 */
#define HEARTBEAT_INTERVAL_MS   1000    /* 心跳间隔1秒 */
#define HEARTBEAT_TIMEOUT_MS    5000    /* 心跳超时5秒 */
#define SENSOR_REPORT_INTERVAL  100     /* 传感器上报间隔100ms */

/* ======================== 接收状态机枚举 ======================== */
typedef enum {
    RX_STATE_IDLE,      /* 等待帧头字节0 */
    RX_STATE_HEADER1,   /* 等待帧头字节1 */
    RX_STATE_LENGTH,    /* 接收长度 */
    RX_STATE_CMD,       /* 接收命令字 */
    RX_STATE_DATA,      /* 接收数据 */
    RX_STATE_CRC_H,     /* 接收CRC高字节 */
    RX_STATE_CRC_L,     /* 接收CRC低字节 */
} RxState_t;

/* ======================== 帧结构体 ======================== */
typedef struct {
    uint8_t cmd;                        /* 命令字 */
    uint8_t length;                     /* 数据长度 */
    uint8_t data[PROTO_MAX_DATA_LEN];  /* 数据缓冲区 */
    uint16_t crc;                       /* CRC16校验 */
} Frame_t;

/* ======================== 全局变量 ======================== */
/* 接收状态机变量 */
static volatile RxState_t gRxState = RX_STATE_IDLE;
static volatile uint8_t gRxBuf[PROTO_MAX_DATA_LEN];  /* 接收数据缓冲 */
static volatile uint8_t gRxIndex = 0;                 /* 接收索引 */
static volatile uint8_t gRxLength = 0;                /* 接收数据长度 */
static volatile uint8_t gRxCmd = 0;                   /* 接收命令字 */
static volatile uint16_t gRxCrc = 0;                  /* 接收的CRC */

/* 接收完成标志和帧 */
static volatile bool gFrameReady = false;              /* 帧接收完成标志 */
static Frame_t gRxFrame;                              /* 接收完成的帧 */

/* 连接状态 */
static volatile bool gConnected = false;               /* 是否已握手 */
static volatile uint32_t gLastHeartbeat = 0;           /* 上次心跳时间 */
static volatile uint32_t gSysTick = 0;                 /* 系统滴答计数 */

/* 发送缓冲区 */
static uint8_t gTxBuf[PROTO_MAX_FRAME_LEN];

/* ======================== CRC16-CCITT查表法 ======================== */
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC1AC, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x4864, 0x5845, 0x6826, 0x7807, 0x08E0, 0x18C1, 0x28A2, 0x38A3,
    0xC94C, 0xD96D, 0xE90E, 0xF92F, 0x89C8, 0x99E9, 0xA98A, 0xB9AB,
    0x5A75, 0x4A54, 0x7A37, 0x6A16, 0x1AF1, 0x0AD0, 0x3AB3, 0x2A92,
    0xDB7D, 0xCB5C, 0xFB3F, 0xEB1E, 0x9BF9, 0x8BD8, 0xBBBB, 0xAB9A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x85A9, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

/**
 * @brief 计算CRC16-CCITT校验值
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC16校验值
 */
static uint16_t crc16_calc(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

/* ======================== UART发送函数 ======================== */

/**
 * @brief 通过UART发送单个字节（阻塞）
 */
static void uart_send_byte(uint8_t byte)
{
    while (!DL_UART_isTXFIFOEmpty(UART_0_INST))
        ;
    DL_UART_transmitDataBlocking(UART_0_INST, byte);
}

/**
 * @brief 通过UART发送多个字节（阻塞）
 */
static void uart_send_bytes(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        uart_send_byte(data[i]);
    }
}

/* ======================== 帧发送函数 ======================== */

/**
 * @brief 构建并发送一帧数据
 * @param cmd 命令字
 * @param data 数据指针
 * @param len 数据长度
 *
 * 发送格式: [0xAA][0x55][长度][命令][数据...][CRC_H][CRC_L]
 */
static void proto_send_frame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t idx = 0;

    if (len > PROTO_MAX_DATA_LEN) return;  /* 防止溢出 */

    /* 组装帧头 */
    gTxBuf[idx++] = PROTO_HEADER_0;
    gTxBuf[idx++] = PROTO_HEADER_1;
    gTxBuf[idx++] = len;                   /* 长度 = 命令字 + 数据 */
    gTxBuf[idx++] = cmd;

    /* 拷贝数据 */
    if (data && len > 0) {
        memcpy(&gTxBuf[idx], data, len);
        idx += len;
    }

    /* 计算CRC: 对长度+命令+数据部分 */
    uint16_t crc = crc16_calc(&gTxBuf[2], 2 + len);
    gTxBuf[idx++] = (crc >> 8) & 0xFF;    /* CRC高字节 */
    gTxBuf[idx++] = crc & 0xFF;           /* CRC低字节 */

    /* 发送 */
    uart_send_bytes(gTxBuf, idx);
}

/* ======================== 帧接收处理 ======================== */

/**
 * @brief 处理收到的UART字节（在中断中调用）
 * @param byte 收到的字节
 *
 * 实现简单的帧同步状态机：
 * - 先找到帧头 0xAA 0x55
 * - 再依次读取长度、命令、数据、CRC
 * - 完成后设置帧就绪标志
 */
static void proto_rx_byte(uint8_t byte)
{
    switch (gRxState) {
    case RX_STATE_IDLE:
        if (byte == PROTO_HEADER_0) {
            gRxState = RX_STATE_HEADER1;  /* 找到帧头第1字节 */
        }
        break;

    case RX_STATE_HEADER1:
        if (byte == PROTO_HEADER_1) {
            gRxState = RX_STATE_LENGTH;   /* 帧头匹配 */
        } else if (byte == PROTO_HEADER_0) {
            /* 保持在当前状态，可能是新的帧头 */
        } else {
            gRxState = RX_STATE_IDLE;
        }
        break;

    case RX_STATE_LENGTH:
        if (byte > 0 && byte <= PROTO_MAX_DATA_LEN) {
            gRxLength = byte;
            gRxState = RX_STATE_CMD;
        } else {
            gRxState = RX_STATE_IDLE;     /* 长度无效，重置 */
        }
        break;

    case RX_STATE_CMD:
        gRxCmd = byte;
        gRxIndex = 0;
        if (gRxLength > 0) {
            gRxState = RX_STATE_DATA;     /* 有数据要接收 */
        } else {
            gRxState = RX_STATE_CRC_H;    /* 无数据，直接收CRC */
        }
        break;

    case RX_STATE_DATA:
        if (gRxIndex < gRxLength) {
            gRxBuf[gRxIndex++] = byte;
        }
        if (gRxIndex >= gRxLength) {
            gRxState = RX_STATE_CRC_H;    /* 数据接收完毕 */
        }
        break;

    case RX_STATE_CRC_H:
        gRxCrc = (uint16_t)byte << 8;
        gRxState = RX_STATE_CRC_L;
        break;

    case RX_STATE_CRC_L:
        gRxCrc |= byte;
        /* CRC校验: 对长度+命令+数据部分 */
        {
            uint8_t check_buf[2 + PROTO_MAX_DATA_LEN];
            check_buf[0] = gRxLength;
            check_buf[1] = gRxCmd;
            memcpy(&check_buf[2], (const uint8_t *)gRxBuf, gRxLength);
            uint16_t calc_crc = crc16_calc(check_buf, 2 + gRxLength);

            if (calc_crc == gRxCrc && !gFrameReady) {
                /* CRC正确，组装帧 */
                gRxFrame.cmd = gRxCmd;
                gRxFrame.length = gRxLength;
                memcpy(gRxFrame.data, (const uint8_t *)gRxBuf, gRxLength);
                gRxFrame.crc = gRxCrc;
                gFrameReady = true;       /* 通知主循环 */
            }
        }
        gRxState = RX_STATE_IDLE;
        break;

    default:
        gRxState = RX_STATE_IDLE;
        break;
    }
}

/* ======================== UART中断服务程序 ======================== */
void UART_0_INST_IRQHandler(void)
{
    volatile uint8_t rxByte;

    /* 检查是否有数据接收中断 */
    if (DL_UART_getPendingInterrupt(UART_0_INST) == DL_UART_IIDX_RX) {
        rxByte = DL_UART_receiveDataBlocking(UART_0_INST);
        proto_rx_byte(rxByte);
    }
}

/* ======================== 模拟传感器读取 ======================== */

/**
 * @brief 读取模拟传感器数据
 * @param sensor_id 传感器ID
 * @return 传感器浮点值
 */
static float read_sensor(uint8_t sensor_id)
{
    switch (sensor_id) {
    case SENSOR_ADC_0: {
        /* 读取ADC通道0 - 模拟电位器或红外传感器 */
        DL_ADC12_startConversion(ADC12_0_INST);
        while (!(DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_DONE))
            ;
        uint16_t adc_val = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
        return (float)adc_val / 4096.0f * 3.3f;  /* 转换为电压 */
    }
    case SENSOR_ADC_1: {
        /* 读取ADC通道1 */
        DL_ADC12_startConversion(ADC12_0_INST);
        while (!(DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_DONE))
            ;
        uint16_t adc_val = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
        return (float)adc_val / 4096.0f * 3.3f;
    }
    case SENSOR_BATTERY: {
        /* 读取电池电压（经分压电阻） */
        DL_ADC12_startConversion(ADC12_0_INST);
        while (!(DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_DONE))
            ;
        uint16_t adc_val = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
        float v_adc = (float)adc_val / 4096.0f * 3.3f;
        return v_adc * 11.0f;  /* 假设10:1分压电阻 */
    }
    default:
        return 0.0f;
    }
}

/* ======================== 命令处理函数 ======================== */

/**
 * @brief 处理握手请求
 *
 * OPi5发来握手包后，返回握手应答，包含固件版本号
 */
static void cmd_handle_handshake(void)
{
    uint8_t version = 0x10;  /* 版本1.0 */
    proto_send_frame(CMD_HANDSHAKE_ACK, &version, 1);
    gConnected = true;
    gLastHeartbeat = gSysTick;
}

/**
 * @brief 处理传感器查询请求
 * @param data 命令数据（第1字节为传感器ID）
 *
 * 读取对应传感器并上报数据
 */
static void cmd_handle_query_sensor(const uint8_t *data)
{
    uint8_t sensor_id = data[0];
    float value = read_sensor(sensor_id);

    /* 组装应答数据: 传感器ID(1B) + 浮点值(4B) */
    uint8_t buf[5];
    buf[0] = sensor_id;
    memcpy(&buf[1], &value, sizeof(float));
    proto_send_frame(CMD_SENSOR_REPORT, buf, 5);
}

/**
 * @brief 处理电机速度设置命令
 * @param data 命令数据（通道1B + 速度2B signed）
 *
 * 解析速度值并设置PWM输出
 */
static void cmd_handle_set_motor(const uint8_t *data)
{
    uint8_t channel = data[0];
    int16_t speed = (int16_t)((data[1] << 8) | data[2]);

    /* 速度限幅 -1000 ~ +1000 */
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    /* TODO: 根据实际硬件设置PWM和方向引脚 */
    /* 示例: 假设通道0使用PA0作为PWM输出 */
    if (channel == 0) {
        uint32_t pwm_val = (uint32_t)((speed >= 0 ? speed : -speed) * 100 / 1000);
        /* DL_TimerG_setCaptureCompareValue(PWM_0_INST, pwm_val, DL_TIMER_CC_0_INDEX); */
    }

    uint8_t ack = 0x01;  /* 成功 */
    proto_send_frame(CMD_MOTOR_ACK, &ack, 1);
}

/**
 * @brief 处理PID参数设置命令
 * @param data 命令数据（通道1B + Kp 4B + Ki 4B + Kd 4B）
 */
static void cmd_handle_set_pid(const uint8_t *data)
{
    uint8_t channel = data[0];
    float kp, ki, kd;
    memcpy(&kp, &data[1], sizeof(float));
    memcpy(&ki, &data[5], sizeof(float));
    memcpy(&kd, &data[9], sizeof(float));

    /* TODO: 将PID参数写入对应的PID控制器实例 */
    /* pid_set_params(channel, kp, ki, kd); */

    uint8_t ack = 0x01;
    proto_send_frame(CMD_PID_ACK, &ack, 1);
}

/**
 * @brief 处理心跳包
 *
 * 更新最后心跳时间，标记连接正常
 */
static void cmd_handle_heartbeat(void)
{
    gLastHeartbeat = gSysTick;
    gConnected = true;
    /* 可选：回复心跳 */
    proto_send_frame(CMD_HEARTBEAT, NULL, 0);
}

/**
 * @brief 主帧分发处理函数
 *
 * 根据命令字分发到具体的处理函数
 */
static void proto_process_frame(const Frame_t *frame)
{
    switch (frame->cmd) {
    case CMD_HANDSHAKE_REQ:
        cmd_handle_handshake();
        break;
    case CMD_QUERY_SENSOR:
        if (frame->length >= 1)
            cmd_handle_query_sensor(frame->data);
        break;
    case CMD_SET_MOTOR:
        if (frame->length >= 3)
            cmd_handle_set_motor(frame->data);
        break;
    case CMD_SET_PID:
        if (frame->length >= 13)
            cmd_handle_set_pid(frame->data);
        break;
    case CMD_HEARTBEAT:
        cmd_handle_heartbeat();
        break;
    default:
        /* 未知命令，发送错误报告 */
        {
            uint8_t err = ERR_INVALID_CMD;
            proto_send_frame(CMD_ERROR, &err, 1);
        }
        break;
    }
}

/* ======================== 定时上报函数 ======================== */

/**
 * @brief 定时上报关键传感器数据
 *
 * 每隔 SENSOR_REPORT_INTERVAL 毫秒主动上报电池电压
 */
static void sensor_periodic_report(void)
{
    static uint32_t last_report = 0;

    if (gSysTick - last_report >= SENSOR_REPORT_INTERVAL) {
        last_report = gSysTick;

        if (gConnected) {
            /* 上报电池电压 */
            float battery = read_sensor(SENSOR_BATTERY);
            uint8_t buf[5];
            buf[0] = SENSOR_BATTERY;
            memcpy(&buf[1], &battery, sizeof(float));
            proto_send_frame(CMD_SENSOR_REPORT, buf, 5);
        }
    }
}

/* ======================== 连接监控 ======================== */

/**
 * @brief 检查连接状态
 *
 * 如果超过 HEARTBEAT_TIMEOUT_MS 未收到心跳，标记断连
 */
static void connection_monitor(void)
{
    if (gConnected &&
        (gSysTick - gLastHeartbeat > HEARTBEAT_TIMEOUT_MS)) {
        gConnected = false;
        /* 可选: LED指示断连状态 */
        DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    }
}

/* ======================== SysTick中断（1ms滴答） ======================== */
void SysTick_Handler(void)
{
    gSysTick++;
}

/* ======================== 主函数 ======================== */
int main(void)
{
    /* 系统初始化 - 由SysConfig生成 */
    SYSCFG_DL_init();

    /* 配置SysTick为1ms中断 */
    SysTick_Config(SystemCoreClock / 1000);

    /* 使能UART接收中断 */
    NVIC_EnableIRQ(UART_0_INST_IRQ);

    /* 初始化LED */
    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN |
                                       GPIO_LEDS_USER_LED_2_PIN);

    /* 发送握手请求（主动连接OPi5） */
    proto_send_frame(CMD_HANDSHAKE_REQ, NULL, 0);

    /* 主循环 */
    while (1) {
        /* 处理收到的帧 */
        if (gFrameReady) {
            /* 关中断保护：复制帧后清除标志 */
            __disable_irq();
            Frame_t frame = gRxFrame;
            gFrameReady = false;
            __enable_irq();

            proto_process_frame(&frame);

            /* 收到数据后闪烁LED */
            DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_2_PIN);
        }

        /* 定时传感器上报 */
        sensor_periodic_report();

        /* 连接状态监控 */
        connection_monitor();

        /* LED指示连接状态 */
        if (gConnected) {
            DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
        }
    }

    return 0;
}
