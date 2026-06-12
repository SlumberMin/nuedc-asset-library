/**
 * @file    math_utils.c
 * @brief   数学工具模块实现
 * @details 限幅、映射、死区、滑动平均、一阶低通滤波器的具体实现。
 */

#include "utils/math_utils.h"
#include <string.h>

/* ========================================================================== */
/*                              独立函数实现                                   */
/* ========================================================================== */

float Math_ClampF(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

int32_t Math_ClampI(int32_t value, int32_t min, int32_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float Math_MapF(float value, float in_min, float in_max, float out_min, float out_max)
{
    if (in_min == in_max) return out_min; /* 防止除零 */
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

int32_t Math_MapI(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    if (in_min == in_max) return out_min;
    /* 使用int64防止中间溢出 */
    int64_t result = (int64_t)out_min +
                     ((int64_t)(value - in_min) * (int64_t)(out_max - out_min)) /
                     (int64_t)(in_max - in_min);
    return (int32_t)result;
}

float Math_DeadZoneF(float value, float dead_zone)
{
    if (value > dead_zone)  return value - dead_zone;
    if (value < -dead_zone) return value + dead_zone;
    return 0.0f;
}

int32_t Math_DeadZoneI(int32_t value, int32_t dead_zone)
{
    if (value > dead_zone)  return value - dead_zone;
    if (value < -dead_zone) return value + dead_zone;
    return 0;
}

float Math_DeadZoneCompensateF(float value, float dead_zone)
{
    if (value > 0) {
        return (value > dead_zone) ? (value - dead_zone) : 0.0f;
    } else {
        return (value < -dead_zone) ? (value + dead_zone) : 0.0f;
    }
}

/* ========================================================================== */
/*                              滑动平均滤波器实现                             */
/* ========================================================================== */

ErrorCode_t MovingAvg_Init(MovingAvg_t *avg, uint8_t size)
{
    if (avg == NULL) return HAL_ERR_PARAM;
    if (size == 0 || size > MOVING_AVG_MAX_SIZE) return HAL_ERR_PARAM;

    avg->size  = size;
    avg->index = 0;
    avg->count = 0;
    avg->sum   = 0.0f;

    memset(avg->buffer, 0, sizeof(avg->buffer[0]) * size);

    return HAL_OK_CODE;
}

float MovingAvg_Update(MovingAvg_t *avg, float value)
{
    if (avg == NULL) return 0.0f;

    /* 如果缓冲区已满，减去最旧的值 */
    if (avg->count >= avg->size) {
        avg->sum -= avg->buffer[avg->index];
    }

    /* 写入新值 */
    avg->buffer[avg->index] = value;
    avg->sum += value;

    /* 更新索引（循环） */
    avg->index = (avg->index + 1) % avg->size;

    /* 更新计数 */
    if (avg->count < avg->size) {
        avg->count++;
    }

    return avg->sum / (float)avg->count;
}

float MovingAvg_GetValue(const MovingAvg_t *avg)
{
    if (avg == NULL || avg->count == 0) return 0.0f;
    return avg->sum / (float)avg->count;
}

ErrorCode_t MovingAvg_Reset(MovingAvg_t *avg)
{
    if (avg == NULL) return HAL_ERR_PARAM;

    avg->index = 0;
    avg->count = 0;
    avg->sum   = 0.0f;
    memset(avg->buffer, 0, sizeof(avg->buffer[0]) * avg->size);

    return HAL_OK_CODE;
}

/* ========================================================================== */
/*                              一阶低通滤波器实现                             */
/* ========================================================================== */

ErrorCode_t LowPassFilter_Init(LowPassFilter_t *lpf, float alpha)
{
    if (lpf == NULL) return HAL_ERR_PARAM;

    lpf->alpha      = CLAMP(alpha, 0.0f, 1.0f);
    lpf->filtered   = 0.0f;
    lpf->initialized = false;

    return HAL_OK_CODE;
}

float LowPassFilter_Update(LowPassFilter_t *lpf, float value)
{
    if (lpf == NULL) return 0.0f;

    if (!lpf->initialized) {
        /* 首次调用，直接赋值 */
        lpf->filtered = value;
        lpf->initialized = true;
    } else {
        /*
         * 一阶低通滤波：
         * y(k) = α * x(k) + (1 - α) * y(k-1)
         * α越大跟踪越快，α越小滤波越强
         */
        lpf->filtered = lpf->alpha * value + (1.0f - lpf->alpha) * lpf->filtered;
    }

    return lpf->filtered;
}

float LowPassFilter_GetValue(const LowPassFilter_t *lpf)
{
    if (lpf == NULL) return 0.0f;
    return lpf->filtered;
}

ErrorCode_t LowPassFilter_Reset(LowPassFilter_t *lpf)
{
    if (lpf == NULL) return HAL_ERR_PARAM;

    lpf->filtered    = 0.0f;
    lpf->initialized = false;

    return HAL_OK_CODE;
}
