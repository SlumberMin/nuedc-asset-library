/**
 * @file    i2c_bus.h
 * @brief   I2C 总线管理层 — MSPM0G3507
 *
 * 功能:
 *   1. 互斥锁: 防止多驱动同时访问I2C总线
 *   2. 超时保护: 每次操作带超时，避免死锁
 *   3. 自动重试: 传输失败时自动重试
 *   4. 多从机支持: 不同地址的设备共享同一总线
 *
 * 使用方法:
 *   I2C_Bus bus;
 *   I2C_Bus_Init(&bus, I2C_0);
 *
 *   // 写操作
 *   uint8_t tx[] = {reg, val};
 *   I2C_Bus_Write(&bus, 0x3C, tx, 2);
 *
 *   // 读操作
 *   uint8_t rx[2];
 *   I2C_Bus_WriteRead(&bus, 0x3C, &reg, 1, rx, 2);
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成)
 */

#ifndef __I2C_BUS_H
#define __I2C_BUS_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ── 配置常量 ──────────────────────────────────────────────── */

/** I2C总线操作超时 (循环计数, ~10ms @32MHz) */
#ifndef I2C_BUS_TIMEOUT
#define I2C_BUS_TIMEOUT         100000U
#endif

/** 传输失败最大重试次数 */
#ifndef I2C_BUS_MAX_RETRY
#define I2C_BUS_MAX_RETRY       3U
#endif

/** 重试间隔 (delay_cycles数, ~100µs @32MHz) */
#ifndef I2C_BUS_RETRY_DELAY
#define I2C_BUS_RETRY_DELAY     3200U
#endif

/* ── 错误码 ────────────────────────────────────────────────── */

typedef enum {
    I2C_BUS_OK          = 0,    /* 操作成功 */
    I2C_BUS_ERR_BUSY    = -1,   /* 总线忙 (互斥锁) */
    I2C_BUS_ERR_TIMEOUT = -2,   /* 超时 */
    I2C_BUS_ERR_NACK    = -3,   /* 从机无应答 */
    I2C_BUS_ERR_RETRY   = -4,   /* 重试次数耗尽 */
} I2C_Bus_Error;

/* ── 总线句柄 ──────────────────────────────────────────────── */

typedef struct {
    I2C_Regs   *inst;           /* I2C外设实例 (I2C_0 或 I2C_1) */
    volatile bool locked;       /* 互斥锁标志 */
    uint32_t    timeout;        /* 超时计数 */
    uint8_t     max_retry;      /* 最大重试次数 */
    /* 统计 */
    uint32_t    tx_count;       /* 发送次数 */
    uint32_t    rx_count;       /* 接收次数 */
    uint32_t    err_count;      /* 错误次数 */
    uint32_t    retry_count;    /* 重试次数 */
} I2C_Bus;

/* ── API ───────────────────────────────────────────────────── */

/**
 * @brief 初始化I2C总线管理层
 * @param bus   总线句柄指针
 * @param inst  I2C外设实例 (I2C_0 / I2C_1)
 */
void I2C_Bus_Init(I2C_Bus *bus, I2C_Regs *inst);

/**
 * @brief 获取总线锁 (互斥)
 * @param bus   总线句柄
 * @return true=获取成功, false=总线已被占用
 *
 * @note 建议使用 I2C_Bus_Write/WriteRead 等封装函数，
 *       它们会自动加锁/解锁。
 *       如需手动加锁，务必配对调用 I2C_Bus_Unlock。
 */
bool I2C_Bus_Lock(I2C_Bus *bus);

/**
 * @brief 释放总线锁
 * @param bus   总线句柄
 */
void I2C_Bus_Unlock(I2C_Bus *bus);

/**
 * @brief I2C写操作 (带重试+超时)
 * @param bus       总线句柄
 * @param addr      7位从机地址
 * @param tx_buf    发送数据缓冲区
 * @param tx_len    发送数据长度
 * @return I2C_BUS_OK=成功, 负值=错误码
 */
I2C_Bus_Error I2C_Bus_Write(I2C_Bus *bus, uint8_t addr,
                             const uint8_t *tx_buf, uint8_t tx_len);

/**
 * @brief I2C读操作 (带重试+超时)
 * @param bus       总线句柄
 * @param addr      7位从机地址
 * @param rx_buf    接收数据缓冲区
 * @param rx_len    接收数据长度
 * @return I2C_BUS_OK=成功, 负值=错误码
 */
I2C_Bus_Error I2C_Bus_Read(I2C_Bus *bus, uint8_t addr,
                            uint8_t *rx_buf, uint8_t rx_len);

/**
 * @brief I2C 写后读操作 (寄存器读取, 带重试+超时)
 * @param bus       总线句柄
 * @param addr      7位从机地址
 * @param tx_buf    写入数据 (如寄存器地址)
 * @param tx_len    写入数据长度
 * @param rx_buf    读取数据缓冲区
 * @param rx_len    读取数据长度
 * @return I2C_BUS_OK=成功, 负值=错误码
 */
I2C_Bus_Error I2C_Bus_WriteRead(I2C_Bus *bus, uint8_t addr,
                                 const uint8_t *tx_buf, uint8_t tx_len,
                                 uint8_t *rx_buf, uint8_t rx_len);

/**
 * @brief I2C写单个寄存器
 * @param bus       总线句柄
 * @param addr      7位从机地址
 * @param reg       寄存器地址
 * @param val       写入值
 * @return I2C_BUS_OK=成功, 负值=错误码
 */
I2C_Bus_Error I2C_Bus_WriteReg(I2C_Bus *bus, uint8_t addr,
                                uint8_t reg, uint8_t val);

/**
 * @brief I2C读单个寄存器
 * @param bus       总线句柄
 * @param addr      7位从机地址
 * @param reg       寄存器地址
 * @param val       输出: 读取值
 * @return I2C_BUS_OK=成功, 负值=错误码
 */
I2C_Bus_Error I2C_Bus_ReadReg(I2C_Bus *bus, uint8_t addr,
                               uint8_t reg, uint8_t *val);

/**
 * @brief I2C读多个寄存器 (自动递增)
 * @param bus       总线句柄
 * @param addr      7位从机地址
 * @param reg       起始寄存器地址
 * @param buf       输出缓冲区
 * @param len       读取长度
 * @return I2C_BUS_OK=成功, 负值=错误码
 */
I2C_Bus_Error I2C_Bus_ReadMulti(I2C_Bus *bus, uint8_t addr,
                                 uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief 获取总线统计信息
 * @param bus       总线句柄
 * @param tx        输出: 发送次数 (可为NULL)
 * @param rx        输出: 接收次数 (可为NULL)
 * @param err       输出: 错误次数 (可为NULL)
 * @param retry     输出: 重试次数 (可为NULL)
 */
void I2C_Bus_GetStats(const I2C_Bus *bus,
                       uint32_t *tx, uint32_t *rx,
                       uint32_t *err, uint32_t *retry);

/**
 * @brief 错误码转字符串
 * @param err  错误码
 * @return 错误描述字符串
 */
const char *I2C_Bus_ErrorStr(I2C_Bus_Error err);

#endif /* __I2C_BUS_H */
