/**
 * @file    ring_buffer.h
 * @brief   环形缓冲区（无锁单生产者单消费者）
 *
 * 参考: GitHub高星项目 lwrb / ringbuf / kfifo 优秀实现
 *
 * 特点:
 *   - 纯C实现，无动态内存分配
 *   - 编译时确定缓冲区大小（必须为2的幂）
 *   - 单生产者单消费者无需加锁（ISR安全）
 *   - 支持批量读写操作
 *
 * 典型应用:
 *   - 串口接收缓冲区
 *   - 传感器数据缓存
 *   - 命令队列
 */
#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * 环形缓冲区控制块
 *
 * 使用 2的幂 大小 + 掩码取代取模运算，提升性能
 * head: 写入位置（生产者维护）
 * tail: 读取位置（消费者维护）
 */
typedef struct {
    uint8_t *buffer;        /**< 数据缓冲区指针 */
    uint32_t size;          /**< 缓冲区大小（必须为2的幂） */
    uint32_t mask;          /**< 掩码 = size - 1 */
    volatile uint32_t head; /**< 写入索引 */
    volatile uint32_t tail; /**< 读取索引 */
} RingBuffer_t;

/**
 * @brief 初始化环形缓冲区
 * @param rb      缓冲区控制块指针
 * @param buf     用户提供的存储空间
 * @param buf_size 缓冲区大小（必须为2的幂，如64, 128, 256）
 */
void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buf, uint32_t buf_size);

/**
 * @brief 重置缓冲区（清空所有数据）
 */
static inline void RingBuffer_Reset(RingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

/**
 * @brief 获取缓冲区中可读数据量
 * @return 已存入的字节数
 */
static inline uint32_t RingBuffer_Used(const RingBuffer_t *rb)
{
    return rb->head - rb->tail;
}

/**
 * @brief 获取缓冲区剩余可用空间
 * @return 可写入的字节数
 */
static inline uint32_t RingBuffer_Free(const RingBuffer_t *rb)
{
    return rb->size - (rb->head - rb->tail);
}

/**
 * @brief 检查缓冲区是否为空
 */
static inline bool RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    return rb->head == rb->tail;
}

/**
 * @brief 检查缓冲区是否已满
 */
static inline bool RingBuffer_IsFull(const RingBuffer_t *rb)
{
    return RingBuffer_Used(rb) >= rb->size;
}

/**
 * @brief 写入单个字节
 * @param rb   缓冲区控制块
 * @param data 要写入的字节
 * @return true=成功, false=缓冲区满
 */
bool RingBuffer_PutByte(RingBuffer_t *rb, uint8_t data);

/**
 * @brief 读取单个字节
 * @param rb   缓冲区控制块
 * @param data 读出数据存放地址
 * @return true=成功, false=缓冲区空
 */
bool RingBuffer_GetByte(RingBuffer_t *rb, uint8_t *data);

/**
 * @brief 批量写入
 * @param rb    缓冲区控制块
 * @param data  数据源
 * @param len   数据长度
 * @return 实际写入的字节数（缓冲区满时可能小于请求量）
 */
uint32_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint32_t len);

/**
 * @brief 批量读取
 * @param rb    缓冲区控制块
 * @param data  目标缓冲区
 * @param len   请求读取长度
 * @return 实际读取的字节数（缓冲区空时可能小于请求量）
 */
uint32_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data, uint32_t len);

/**
 * @brief 查看缓冲区中第一个字节（不移除）
 * @param rb   缓冲区控制块
 * @param data 读出数据存放地址
 * @return true=成功, false=缓冲区空
 */
bool RingBuffer_Peek(const RingBuffer_t *rb, uint8_t *data);

#endif /* __RING_BUFFER_H */
