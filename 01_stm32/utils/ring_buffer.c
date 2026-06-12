/**
 * @file    ring_buffer.c
 * @brief   环形缓冲区模块实现
 * @details 无锁单生产者单消费者(SPSC)实现。
 *          head由生产者写入，tail由消费者写入，
 *          volatile保证可见性，在单核MCU上无需额外内存屏障。
 */

#include "utils/ring_buffer.h"
#include <string.h>
#include <stdlib.h>

/* ========================================================================== */
/*                              辅助宏                                        */
/* ========================================================================== */

/** @brief 判断是否为2的幂 */
#define IS_POWER_OF_2(n)    ((n) > 0 && ((n) & ((n) - 1)) == 0)

/** @brief 向上取整到最近的2的幂 */
static uint32_t next_power_of_2(uint32_t v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

ErrorCode_t RingBuffer_Init(RingBuffer_t *rb, uint32_t capacity)
{
    if (rb == NULL || capacity == 0) return HAL_ERR_PARAM;

    /* 向上取整到2的幂 */
    uint32_t size = next_power_of_2(capacity);

    uint8_t *buf = (uint8_t *)malloc(size);
    if (buf == NULL) return HAL_ERR_NOMEM;

    rb->buffer    = buf;
    rb->size      = size;
    rb->mask      = size - 1;
    rb->head      = 0;
    rb->tail      = 0;
    rb->is_static = false;

    return HAL_OK_CODE;
}

ErrorCode_t RingBuffer_InitStatic(RingBuffer_t *rb, uint8_t *buf, uint32_t buf_size)
{
    if (rb == NULL || buf == NULL || buf_size == 0) return HAL_ERR_PARAM;
    if (!IS_POWER_OF_2(buf_size)) return HAL_ERR_PARAM;

    rb->buffer    = buf;
    rb->size      = buf_size;
    rb->mask      = buf_size - 1;
    rb->head      = 0;
    rb->tail      = 0;
    rb->is_static = true;

    return HAL_OK_CODE;
}

ErrorCode_t RingBuffer_Put(RingBuffer_t *rb, uint8_t data)
{
    if (rb == NULL || rb->buffer == NULL) return HAL_ERR_NOT_INIT;

    /* 检查是否满 */
    if ((rb->head - rb->tail) == rb->size) {
        return HAL_ERR_FULL;
    }

    rb->buffer[rb->head & rb->mask] = data;
    rb->head++;

    return HAL_OK_CODE;
}

ErrorCode_t RingBuffer_Get(RingBuffer_t *rb, uint8_t *data)
{
    if (rb == NULL || rb->buffer == NULL || data == NULL) return HAL_ERR_NOT_INIT;

    /* 检查是否空 */
    if (rb->head == rb->tail) {
        return HAL_ERR_EMPTY;
    }

    *data = rb->buffer[rb->tail & rb->mask];
    rb->tail++;

    return HAL_OK_CODE;
}

uint32_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint32_t len)
{
    if (rb == NULL || rb->buffer == NULL || data == NULL) return 0;

    uint32_t free_space = rb->size - (rb->head - rb->tail);
    uint32_t to_write   = (len < free_space) ? len : free_space;

    for (uint32_t i = 0; i < to_write; i++) {
        rb->buffer[rb->head & rb->mask] = data[i];
        rb->head++;
    }

    return to_write;
}

uint32_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *buf, uint32_t len)
{
    if (rb == NULL || rb->buffer == NULL || buf == NULL) return 0;

    uint32_t available = rb->head - rb->tail;
    uint32_t to_read   = (len < available) ? len : available;

    for (uint32_t i = 0; i < to_read; i++) {
        buf[i] = rb->buffer[rb->tail & rb->mask];
        rb->tail++;
    }

    return to_read;
}

uint32_t RingBuffer_Available(const RingBuffer_t *rb)
{
    if (rb == NULL) return 0;
    return rb->head - rb->tail;
}

uint32_t RingBuffer_Free(const RingBuffer_t *rb)
{
    if (rb == NULL) return 0;
    return rb->size - (rb->head - rb->tail);
}

bool RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    if (rb == NULL) return true;
    return (rb->head == rb->tail);
}

bool RingBuffer_IsFull(const RingBuffer_t *rb)
{
    if (rb == NULL) return false;
    return ((rb->head - rb->tail) == rb->size);
}

ErrorCode_t RingBuffer_Flush(RingBuffer_t *rb)
{
    if (rb == NULL) return HAL_ERR_PARAM;

    rb->tail = rb->head;

    return HAL_OK_CODE;
}

void RingBuffer_DeInit(RingBuffer_t *rb)
{
    if (rb == NULL) return;

    if (!rb->is_static && rb->buffer != NULL) {
        free(rb->buffer);
    }

    rb->buffer    = NULL;
    rb->size      = 0;
    rb->mask      = 0;
    rb->head      = 0;
    rb->tail      = 0;
}
