/**
 * @file    opi5_protocol.h
 * @brief   OrangePi5 <-> MSPM0G3507 UART Communication Protocol
 *
 * Frame format:
 *   [HEAD 0xAA][CMD_ID][LEN][SEQ][DATA(0~255B)][CRC8]
 *
 * CMD_ID (OPi5 -> MSPM0):
 *   0x01  HEARTBEAT       Heartbeat / online detection
 *   0x02  VERSION         Query firmware version
 *   0x10  MOTOR_SET       Motor speed control: [ch, dir, speed_H, speed_L]
 *   0x11  MOTOR_GET       Read motor status: [ch]
 *   0x12  MOTOR_STOP      Emergency stop: [ch] (0xFF=all)
 *   0x20  SERVO_SET       Servo angle: [id, angle_H, angle_L] (0.1 deg)
 *   0x21  SERVO_GET       Read servo angle: [id]
 *   0x30  ADC_READ        Single ADC channel: [ch]
 *   0x31  ADC_MULTI       Multi ADC channels: [N, ch0, ch1, ...]
 *   0x40  GPIO_SET        GPIO output: [pin, val]
 *   0x41  GPIO_GET        GPIO input: [pin]
 *   0x50  QUERY_SENSOR    Query sensor data: [sensor_id]
 *
 * Response (MSPM0 -> OPi5):
 *   0xE0  ACK             Generic ACK: [orig_cmd, status]
 *   0xFE  ERROR           Error report: [err_code, info_H, info_L]
 *   0x50  SENSOR_DATA     Sensor data push: [type, data...]
 *
 * CRC8: polynomial 0x07, init 0x00 (over CMD+LEN+SEQ+DATA)
 */

#ifndef __OPI5_PROTOCOL_H
#define __OPI5_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ── Frame Constants ─────────────────────────────────────── */
#define OPI5_FRAME_HEAD         0xAA
#define OPI5_MAX_DATA_LEN       64
#define OPI5_RX_BUF_SIZE        256

/* ── CMD_ID (OPi5 -> MSPM0) ─────────────────────────────── */
#define CMD_HEARTBEAT           0x01
#define CMD_VERSION             0x02
#define CMD_MOTOR_SET           0x10
#define CMD_MOTOR_GET           0x11
#define CMD_MOTOR_STOP          0x12
#define CMD_SERVO_SET           0x20
#define CMD_SERVO_GET           0x21
#define CMD_ADC_READ            0x30
#define CMD_ADC_MULTI           0x31
#define CMD_GPIO_SET            0x40
#define CMD_GPIO_GET            0x41
#define CMD_QUERY_SENSOR        0x50

/* ── Response CMD_ID (MSPM0 -> OPi5) ────────────────────── */
#define CMD_ACK                 0xE0
#define CMD_ERROR               0xFE
#define CMD_SENSOR_DATA         0x50   /* same as QUERY_SENSOR, bidirectional */

/* ── Error Codes ─────────────────────────────────────────── */
#define ERR_CRC                 0x01
#define ERR_CMD                 0x02
#define ERR_PARAM               0x03
#define ERR_BUSY                0x04
#define ERR_TIMEOUT             0x05
#define ERR_RANGE               0x06
#define ERR_STATE               0x07

/* ── Sensor IDs ──────────────────────────────────────────── */
#define SENSOR_ENCODER          0x01
#define SENSOR_IMU              0x02
#define SENSOR_GRAY             0x03
#define SENSOR_ALL              0xFF

/* ── Timeout Parameters (ms) ─────────────────────────────── */
#define OPI5_ACK_TIMEOUT_MS     100
#define OPI5_RETRY_COUNT        1
#define OPI5_RETRY_INTERVAL_MS  10
#define OPI5_HEARTBEAT_MS       1000
#define OPI5_OFFLINE_TIMEOUT_MS 3000

/* ── RX State Machine ────────────────────────────────────── */
typedef enum {
    OPI5_STATE_HEAD = 0,
    OPI5_STATE_CMD,
    OPI5_STATE_LEN,
    OPI5_STATE_SEQ,
    OPI5_STATE_DATA,
    OPI5_STATE_CRC,
} OPI5_RxState;

/* ── Parsed Frame ────────────────────────────────────────── */
typedef struct {
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  seq;
    uint8_t  data[OPI5_MAX_DATA_LEN];
} OPI5_Frame;

/* ── Communication State ─────────────────────────────────── */
typedef enum {
    OPI5_COMM_DISCONNECTED = 0,
    OPI5_COMM_CONNECTED,
    OPI5_COMM_ERROR,
} OPI5_CommState;

/* ── API ─────────────────────────────────────────────────── */

/** Initialize protocol driver */
void OPI5_Init(void);

/** Call from UART ISR to feed received byte */
void OPI5_RxByte(uint8_t byte);

/** Check if a complete frame is available */
bool OPI5_HasFrame(void);

/** Get and dequeue one parsed frame */
bool OPI5_GetFrame(OPI5_Frame *frame);

/** Send a response frame (CMD + LEN + SEQ + DATA + CRC8) */
void OPI5_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *data, uint8_t len);

/** Send ACK for a command */
void OPI5_SendAck(uint8_t seq, uint8_t orig_cmd, uint8_t status);

/** Send ERROR report */
void OPI5_SendError(uint8_t seq, uint8_t err_code, uint16_t info);

/** CRC8 calculation (polynomial 0x07, init 0x00) */
uint8_t OPI5_CRC8(const uint8_t *data, uint8_t len);

/** Main loop: process all received frames */
void OPI5_Process(void);

/** Periodic tick call (from timer ISR, 1ms) for heartbeat & timeout */
void OPI5_Tick(void);

/** Get communication state */
OPI5_CommState OPI5_GetCommState(void);

#endif /* __OPI5_PROTOCOL_H */
