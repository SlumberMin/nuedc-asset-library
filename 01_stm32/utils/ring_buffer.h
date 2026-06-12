/**
 * @file    ring_buffer.h
 * @brief   环形缓冲区模块 — STM32电赛通用代码库
 * @details 无锁单生产者单消费者环形缓冲区。
 *          使用读写指针和缓冲区大小（2的幂）实现无锁操作。
 *          适用于中断与主循环之间的数据传递。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 */

#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include "platform/hal_stm32.h"

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/**
 * @brief 环形缓冲区结构体
 * @note  容量必须为2的幂，自动向上取整
 */
typedef struct {
    uint8_t  *buffer;       /**< 数据缓冲区指针 */
    uint32_t  size;         /**< 缓冲区大小（必须为2的幂） */
    uint32_t  mask;         /**< size - 1，用于快速取模 */
    volatile uint32_t head; /**< 写指针（生产者使用） */
    volatile uint32_t tail; /**< 读指针（消费者使用） */
    bool      is_static;    /**< 是否使用静态分配（true=不需free） */
} RingBuffer_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化环形缓冲区（动态分配内存）
 * @param rb       环形缓冲区结构体指针
 * @param capacity 期望容量，会自动向上取整到2的幂
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   使用malloc分配内存，如果内存紧张请使用RingBuffer_InitStatic
 */
ErrorCode_t RingBuffer_Init(RingBuffer_t *rb, uint32_t capacity);

/**
 * @brief 初始化环形缓冲区（使用静态缓冲区）
 * @param rb       环形缓冲区结构体指针
 * @param buf      预分配的缓冲区指针
 * @param buf_size 缓冲区大小（必须为2的幂）
 * @return ErrorCode_t
 * @note   推荐方式，避免动态内存分配
 *         例: static uint8_t my_buf[256]; RingBuffer_InitStatic(&rb, my_buf, 256);
 */
ErrorCode_t RingBuffer_InitStatic(RingBuffer_t *rb, uint8_t *buf, uint32_t buf_size);

/**
 * @brief 向缓冲区写入一个字节
 * @param rb    环形缓冲区结构体指针
 * @param data  要写入的字节
 * @return ErrorCode_t: HAL_OK_CODE=成功, HAL_ERR_FULL=缓冲区满
 */
ErrorCode_t RingBuffer_Put(RingBuffer_t *rb, uint8_t data);

/**
 * @brief 从缓冲区读取一个字节
 * @param rb    环形缓冲区结构体指针
 * @param data  读取数据存放指针
 * @return ErrorCode_t: HAL_OK_CODE=成功, HAL_ERR_EMPTY=缓冲区空
 */
ErrorCode_t RingBuffer_Get(RingBuffer_t *rb, uint8_t *data);

/**
 * @brief 向缓冲区写入多个字节
 * @param rb    环形缓冲区结构体指针
 * @param data  数据指针
 * @param len   数据长度
 * @return uint32_t: 实际写入的字节数（可能小于len，如果空间不足）
 */
uint32_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint32_t len);

/**
 * @brief 从缓冲区读取多个字节
 * @param rb    环形缓冲区结构体指针
 * @param buf   接收缓冲区指针
 * @param len   期望读取长度
 * @return uint32_t: 实际读取的字节数（可能小于len，如果数据不足）
 */
uint32_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *buf, uint32_t len);

/**
 * @brief 查看缓冲区中已存数据量
 * @param rb  环形缓冲区结构体指针
 * @return uint32_t: 已存字节数
 */
uint32_t RingBuffer_Available(const RingBuffer_t *rb);

/**
 * @brief 查看缓冲区剩余空间
 * @param rb  环形缓冲区结构体指针
 * @return uint32_t: 可写入字节数
 */
uint32_t RingBuffer_Free(const RingBuffer_t *rb);

/**
 * @brief 缓冲区是否为空
 * @param rb  环形缓冲区结构体指针
 * @return bool: true=空
 */
bool RingBuffer_IsEmpty(const RingBuffer_t *rb);

/**
 * @brief 缓冲区是否已满
 * @param rb  环形缓冲区结构体指针
 * @return bool: true=满
 */
bool RingBuffer_IsFull(const RingBuffer_t *rb);

/**
 * @brief 清空缓冲区
 * @param rb  环形缓冲区结构体指针
 * @return ErrorCode_t
 */
ErrorCode_t RingBuffer_Flush(RingBuffer_t *rb);

/**
 * @brief 销毁缓冲区（释放动态分配的内存）
 * @param rb  环形缓冲区结构体指针
 * @note   静态分配的缓冲区不会被释放
 */
void RingBuffer_DeInit(RingBuffer_t *rb);

#endif /* __RING_BUFFER_H */
