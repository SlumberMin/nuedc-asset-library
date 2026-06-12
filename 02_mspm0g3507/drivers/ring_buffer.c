/**
 * @file    ring_buffer.c
 * @brief   环形缓冲区实现（无锁SPSC，ISR安全）
 *
 * 算法说明:
 *   - 使用 head/tail 两个索引，head由写入方维护，tail由读取方维护
 *   - 缓冲区大小必须为2的幂，用位掩码替代取模运算
 *   - head/tail 使用uint32_t无符号类型，差值自然处理回绕
 *   - 单字节场景下只修改一个索引，天然无竞争
 */
#include "drivers/ring_buffer.h"

/* ================================================================
 * 辅助函数: 判断是否为2的幂
 * ================================================================ */
static inline bool is_power_of_two(uint32_t n)
{
    return (n != 0) && ((n & (n - 1)) == 0);
}

/* ================================================================
 * 初始化
 * ================================================================ */
void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buf, uint32_t buf_size)
{
    /* 参数校验: 大小必须为2的幂 */
    if (!is_power_of_two(buf_size) || buf == (void *)0) {
        /* 错误: 设置为无效状态 */
        if (rb) {
            rb->buffer = (void *)0;
            rb->size   = 0;
            rb->mask   = 0;
            rb->head   = 0;
            rb->tail   = 0;
        }
        return;
    }

    rb->buffer = buf;
    rb->size   = buf_size;
    rb->mask   = buf_size - 1;
    rb->head   = 0;
    rb->tail   = 0;
}

/* ================================================================
 * 写入单字节
 * ================================================================ */
bool RingBuffer_PutByte(RingBuffer_t *rb, uint8_t data)
{
    if (RingBuffer_IsFull(rb)) {
        return false;
    }

    rb->buffer[rb->head & rb->mask] = data;

    /* 内存屏障: 确保数据写入在索引更新之前完成
     * 对于Cortex-M0+，使用编译器屏障即可（单核无乱序） */
    __asm volatile("" ::: "memory");

    rb->head++;
    return true;
}

/* ================================================================
 * 读取单字节
 * ================================================================ */
bool RingBuffer_GetByte(RingBuffer_t *rb, uint8_t *data)
{
    if (RingBuffer_IsEmpty(rb)) {
        return false;
    }

    *data = rb->buffer[rb->tail & rb->mask];

    __asm volatile("" ::: "memory");

    rb->tail++;
    return true;
}

/* ================================================================
 * 批量写入
 * ================================================================ */
uint32_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint32_t len)
{
    uint32_t free_space = RingBuffer_Free(rb);

    /* 限制写入量不超过可用空间 */
    if (len > free_space) {
        len = free_space;
    }

    if (len == 0) {
        return 0;
    }

    /*
     * 写入分为两段处理（可能跨越缓冲区边界）:
     *   段1: head到缓冲区末尾
     *   段2: 缓冲区头部（回绕部分）
     */
    uint32_t head_idx = rb->head & rb->mask;
    uint32_t first_chunk = rb->size - head_idx;

    if (len <= first_chunk) {
        /* 不需要回绕，一次拷贝 */
        memcpy(&rb->buffer[head_idx], data, len);
    } else {
        /* 需要分两段拷贝 */
        memcpy(&rb->buffer[head_idx], data, first_chunk);
        memcpy(&rb->buffer[0], data + first_chunk, len - first_chunk);
    }

    __asm volatile("" ::: "memory");

    rb->head += len;
    return len;
}

/* ================================================================
 * 批量读取
 * ================================================================ */
uint32_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data, uint32_t len)
{
    uint32_t used = RingBuffer_Used(rb);

    /* 限制读取量不超过可用数据 */
    if (len > used) {
        len = used;
    }

    if (len == 0) {
        return 0;
    }

    /*
     * 读取分为两段处理（可能跨越缓冲区边界）:
     *   段1: tail到缓冲区末尾
     *   段2: 缓冲区头部（回绕部分）
     */
    uint32_t tail_idx = rb->tail & rb->mask;
    uint32_t first_chunk = rb->size - tail_idx;

    if (len <= first_chunk) {
        memcpy(data, &rb->buffer[tail_idx], len);
    } else {
        memcpy(data, &rb->buffer[tail_idx], first_chunk);
        memcpy(data + first_chunk, &rb->buffer[0], len - first_chunk);
    }

    __asm volatile("" ::: "memory");

    rb->tail += len;
    return len;
}

/* ================================================================
 * 查看（不消费）
 * ================================================================ */
bool RingBuffer_Peek(const RingBuffer_t *rb, uint8_t *data)
{
    if (RingBuffer_IsEmpty(rb)) {
        return false;
    }

    *data = rb->buffer[rb->tail & rb->mask];
    return true;
}
