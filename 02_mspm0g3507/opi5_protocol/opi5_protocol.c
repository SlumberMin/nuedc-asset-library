/**
 * @file    opi5_protocol.c
 * @brief   OrangePi5 <-> MSPM0G3507 UART Communication Protocol
 *
 * Frame: [0xAA][CMD][LEN][SEQ][DATA...][CRC8]
 * CRC8: polynomial 0x07, init 0x00
 */

#include "opi5_protocol.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/* ── RX Ring Buffer ──────────────────────────────────────── */
static volatile uint8_t  rx_buf[OPI5_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* ── Frame Parse State Machine ───────────────────────────── */
static OPI5_RxState parse_state = OPI5_STATE_HEAD;
static uint8_t  frame_cmd;
static uint8_t  frame_len;
static uint8_t  frame_seq;
static uint8_t  frame_data[OPI5_MAX_DATA_LEN];
static uint8_t  frame_idx;

/* ── Parsed Frame Queue (max 4 frames) ───────────────────── */
#define FRAME_QUEUE_SIZE 4
static OPI5_Frame frame_queue[FRAME_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;
static volatile uint8_t queue_tail = 0;

/* ── Communication State ─────────────────────────────────── */
static OPI5_CommState comm_state = OPI5_COMM_DISCONNECTED;
static volatile uint32_t heartbeat_timer = 0;
static volatile uint32_t last_rx_tick = 0;
static uint8_t seq_counter = 0;

/* ── External: 1ms tick counter from main.c ──────────────── */
extern volatile uint32_t g_ms;

/* ── External callbacks (implemented in main.c) ──────────── */
extern void OPI5_HandleHeartbeat(uint8_t seq);
extern void OPI5_HandleVersion(uint8_t seq);
extern void OPI5_HandleMotorSet(const uint8_t *data, uint8_t len, uint8_t seq);
extern void OPI5_HandleMotorGet(const uint8_t *data, uint8_t len, uint8_t seq);
extern void OPI5_HandleMotorStop(const uint8_t *data, uint8_t len, uint8_t seq);
extern void OPI5_HandleServoSet(const uint8_t *data, uint8_t len, uint8_t seq);
extern void OPI5_HandleServoGet(const uint8_t *data, uint8_t len, uint8_t seq);
extern void OPI5_HandleAdcRead(const uint8_t *data, uint8_t len, uint8_t seq);
extern void OPI5_HandleAdcMulti(const uint8_t *data, uint8_t len, uint8_t seq);
extern void OPI5_HandleGpioSet(const uint8_t *data, uint8_t len, uint8_t seq);
extern void OPI5_HandleGpioGet(const uint8_t *data, uint8_t len, uint8_t seq);
extern void OPI5_HandleQuerySensor(const uint8_t *data, uint8_t len, uint8_t seq);

/* ── CRC8 (polynomial 0x07, init 0x00) ───────────────────── */
uint8_t OPI5_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    uint8_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ── Initialize ──────────────────────────────────────────── */
void OPI5_Init(void)
{
    rx_head = 0;
    rx_tail = 0;
    queue_head = 0;
    queue_tail = 0;
    parse_state = OPI5_STATE_HEAD;
    comm_state = OPI5_COMM_DISCONNECTED;
    seq_counter = 0;
    heartbeat_timer = 0;
    last_rx_tick = 0;
}

/* ── UART ISR: feed one byte ─────────────────────────────── */
void OPI5_RxByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((rx_head + 1) % OPI5_RX_BUF_SIZE);
    if (next != rx_tail) {
        rx_buf[rx_head] = byte;
        rx_head = next;
    }
}

/* ── Enqueue a parsed frame ──────────────────────────────── */
static void enqueue_frame(uint8_t cmd, uint8_t len, uint8_t seq, const uint8_t *data)
{
    uint8_t next = (uint8_t)((queue_head + 1) % FRAME_QUEUE_SIZE);
    if (next == queue_tail) return; /* queue full, discard */
    frame_queue[queue_head].cmd = cmd;
    frame_queue[queue_head].len = len;
    frame_queue[queue_head].seq = seq;
    if (len > 0 && len <= OPI5_MAX_DATA_LEN) {
        memcpy(frame_queue[queue_head].data, data, len);
    }
    queue_head = next;
}

/* ── Parse bytes from ring buffer into frames ────────────── */
static void parse_rx_data(void)
{
    while (rx_tail != rx_head) {
        uint8_t byte = rx_buf[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1) % OPI5_RX_BUF_SIZE);

        switch (parse_state) {
        case OPI5_STATE_HEAD:
            if (byte == OPI5_FRAME_HEAD) {
                parse_state = OPI5_STATE_CMD;
            }
            break;

        case OPI5_STATE_CMD:
            frame_cmd = byte;
            parse_state = OPI5_STATE_LEN;
            break;

        case OPI5_STATE_LEN:
            if (byte <= OPI5_MAX_DATA_LEN) {
                frame_len = byte;
                parse_state = OPI5_STATE_SEQ;
            } else {
                parse_state = OPI5_STATE_HEAD; /* invalid length */
            }
            break;

        case OPI5_STATE_SEQ:
            frame_seq = byte;
            frame_idx = 0;
            if (frame_len == 0) {
                parse_state = OPI5_STATE_CRC;
            } else {
                parse_state = OPI5_STATE_DATA;
            }
            break;

        case OPI5_STATE_DATA:
            frame_data[frame_idx++] = byte;
            if (frame_idx >= frame_len) {
                parse_state = OPI5_STATE_CRC;
            }
            break;

        case OPI5_STATE_CRC: {
            /* CRC8 over CMD + LEN + SEQ + DATA */
            uint8_t crc_buf[OPI5_MAX_DATA_LEN + 3];
            crc_buf[0] = frame_cmd;
            crc_buf[1] = frame_len;
            crc_buf[2] = frame_seq;
            if (frame_len > 0) {
                memcpy(&crc_buf[3], frame_data, frame_len);
            }
            uint8_t crc_calc = OPI5_CRC8(crc_buf, (uint8_t)(frame_len + 3));
            if (crc_calc == byte) {
                enqueue_frame(frame_cmd, frame_len, frame_seq, frame_data);
                last_rx_tick = g_ms;
            } else {
                /* CRC error: send ERR_CRC */
                OPI5_SendError(frame_seq, ERR_CRC, 0);
            }
            parse_state = OPI5_STATE_HEAD;
            break;
        }
        default:
            parse_state = OPI5_STATE_HEAD;
            break;
        }
    }
}

/* ── Check if frame available ────────────────────────────── */
bool OPI5_HasFrame(void)
{
    parse_rx_data();
    return (queue_head != queue_tail);
}

/* ── Dequeue one frame ───────────────────────────────────── */
bool OPI5_GetFrame(OPI5_Frame *frame)
{
    parse_rx_data();
    if (queue_head == queue_tail) return false;
    *frame = frame_queue[queue_tail];
    queue_tail = (uint8_t)((queue_tail + 1) % FRAME_QUEUE_SIZE);
    return true;
}

/* ── Send frame: [0xAA][CMD][LEN][SEQ][DATA...][CRC8] ───── */
void OPI5_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *data, uint8_t len)
{
    if (len > OPI5_MAX_DATA_LEN) len = OPI5_MAX_DATA_LEN;

    uint8_t tx_buf[OPI5_MAX_DATA_LEN + 5];
    uint8_t crc_buf[OPI5_MAX_DATA_LEN + 3];
    uint8_t idx = 0;

    /* Build frame */
    tx_buf[idx++] = OPI5_FRAME_HEAD;
    tx_buf[idx++] = cmd;
    tx_buf[idx++] = len;
    tx_buf[idx++] = seq;
    if (len > 0 && data != NULL) {
        memcpy(&tx_buf[idx], data, len);
        idx += len;
    }

    /* CRC8 over CMD + LEN + SEQ + DATA */
    crc_buf[0] = cmd;
    crc_buf[1] = len;
    crc_buf[2] = seq;
    if (len > 0 && data != NULL) {
        memcpy(&crc_buf[3], data, len);
    }
    tx_buf[idx++] = OPI5_CRC8(crc_buf, (uint8_t)(len + 3));

    /* Send via UART1 */
    uint8_t i;
    for (i = 0; i < idx; i++) {
        while (!DL_UART_isTXFIFOEmpty(UART_1_INST)) {}
        DL_UART_Main_transmitData(UART_1_INST, tx_buf[i]);
    }
}

/* ── Send ACK ────────────────────────────────────────────── */
void OPI5_SendAck(uint8_t seq, uint8_t orig_cmd, uint8_t status)
{
    uint8_t data[2] = { orig_cmd, status };
    OPI5_SendFrame(CMD_ACK, seq, data, 2);
}

/* ── Send ERROR ──────────────────────────────────────────── */
void OPI5_SendError(uint8_t seq, uint8_t err_code, uint16_t info)
{
    uint8_t data[3];
    data[0] = err_code;
    data[1] = (uint8_t)(info >> 8);
    data[2] = (uint8_t)(info & 0xFF);
    OPI5_SendFrame(CMD_ERROR, seq, data, 3);
}

/* ── Periodic tick (call from 1ms timer ISR) ─────────────── */
void OPI5_Tick(void)
{
    /* Send heartbeat periodically */
    heartbeat_timer++;
    if (heartbeat_timer >= OPI5_HEARTBEAT_MS) {
        heartbeat_timer = 0;
        OPI5_SendFrame(CMD_HEARTBEAT, seq_counter++, NULL, 0);
    }

    /* Check offline timeout */
    if (comm_state == OPI5_COMM_CONNECTED) {
        if ((g_ms - last_rx_tick) > OPI5_OFFLINE_TIMEOUT_MS) {
            comm_state = OPI5_COMM_DISCONNECTED;
        }
    }
}

/* ── Get communication state ─────────────────────────────── */
OPI5_CommState OPI5_GetCommState(void)
{
    return comm_state;
}

/* ── Main loop: process all frames ───────────────────────── */
void OPI5_Process(void)
{
    OPI5_Frame frame;
    while (OPI5_GetFrame(&frame)) {
        /* Any valid frame from OPi5 means it's alive */
        if (comm_state == OPI5_COMM_DISCONNECTED) {
            comm_state = OPI5_COMM_CONNECTED;
        }

        switch (frame.cmd) {
        case CMD_HEARTBEAT:
            OPI5_HandleHeartbeat(frame.seq);
            break;
        case CMD_VERSION:
            OPI5_HandleVersion(frame.seq);
            break;
        case CMD_MOTOR_SET:
            OPI5_HandleMotorSet(frame.data, frame.len, frame.seq);
            break;
        case CMD_MOTOR_GET:
            OPI5_HandleMotorGet(frame.data, frame.len, frame.seq);
            break;
        case CMD_MOTOR_STOP:
            OPI5_HandleMotorStop(frame.data, frame.len, frame.seq);
            break;
        case CMD_SERVO_SET:
            OPI5_HandleServoSet(frame.data, frame.len, frame.seq);
            break;
        case CMD_SERVO_GET:
            OPI5_HandleServoGet(frame.data, frame.len, frame.seq);
            break;
        case CMD_ADC_READ:
            OPI5_HandleAdcRead(frame.data, frame.len, frame.seq);
            break;
        case CMD_ADC_MULTI:
            OPI5_HandleAdcMulti(frame.data, frame.len, frame.seq);
            break;
        case CMD_GPIO_SET:
            OPI5_HandleGpioSet(frame.data, frame.len, frame.seq);
            break;
        case CMD_GPIO_GET:
            OPI5_HandleGpioGet(frame.data, frame.len, frame.seq);
            break;
        case CMD_QUERY_SENSOR:
            OPI5_HandleQuerySensor(frame.data, frame.len, frame.seq);
            break;
        default:
            OPI5_SendError(frame.seq, ERR_CMD, frame.cmd);
            break;
        }
    }
}
