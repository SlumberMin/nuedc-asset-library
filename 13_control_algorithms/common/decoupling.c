/**
 * @file decoupling.c
 * @brief 解耦控制算法实现
 * @details 提供多通道解耦控制, 用于消除多变量系统中各通道间的耦合干扰。
 *          支持三种解耦模式:
 *          - 前馈解耦: 基于参考输入的解耦
 *          - 反馈解耦: 基于反馈测量的解耦
 *          - 混合解耦: 前馈+反馈组合解耦
 *          所有解耦输出经过可配置的低通滤波器平滑处理。
 */

#include "decoupling.h"
#include <string.h>

/**
 * @brief 初始化解耦控制器
 * @param dec 解耦结构体指针
 * @param n 通道数(最大DECOUPLE_MAX_CHANNELS)
 * @param type 解耦类型
 */
void Decoupling_Init(Decoupling_t *dec, uint8_t n, DecoupleType_t type)
{
    if (dec == NULL) return;
    /* 通道数限幅 */
    if (n > DECOUPLE_MAX_CHANNELS) n = DECOUPLE_MAX_CHANNELS;
    if (n < 2) n = 2;

    memset(dec, 0, sizeof(Decoupling_t));
    dec->n_channels = n;
    dec->type = type;
    dec->filter_alpha = 0.9f;

    /* 单位矩阵初始化K, D, Kf */
    for (uint8_t i = 0; i < n; i++) {
        dec->K[i][i] = 1.0f;
        dec->D[i][i] = 0.0f;
        dec->Kf[i][i] = 1.0f;
    }
}

/**
 * @brief 设置耦合矩阵K(前馈耦合系数)
 * @param dec 解耦结构体指针
 * @param K_data 耦合矩阵数据(行优先 n×n)
 */
void Decoupling_SetCouplingMatrix(Decoupling_t *dec, const float *K_data)
{
    if (dec == NULL || K_data == NULL) return;
    uint8_t n = dec->n_channels;
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            dec->K[i][j] = K_data[i * n + j];
}

/**
 * @brief 设置反馈解耦矩阵D
 * @param dec 解耦结构体指针
 * @param D_data 反馈解耦矩阵数据(行优先 n×n)
 */
void Decoupling_SetFeedbackMatrix(Decoupling_t *dec, const float *D_data)
{
    if (dec == NULL || D_data == NULL) return;
    uint8_t n = dec->n_channels;
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            dec->D[i][j] = D_data[i * n + j];
}

/**
 * @brief 设置前馈解耦矩阵Kf
 * @param dec 解耦结构体指针
 * @param Kf_data 前馈解耦矩阵数据(行优先 n×n)
 */
void Decoupling_SetFeedforwardMatrix(Decoupling_t *dec, const float *Kf_data)
{
    if (dec == NULL || Kf_data == NULL) return;
    uint8_t n = dec->n_channels;
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            dec->Kf[i][j] = Kf_data[i * n + j];
}

/**
 * @brief 设置输出低通滤波系数
 * @param dec 解耦结构体指针
 * @param alpha 滤波系数(0~1), 越大滤波越强
 */
void Decoupling_SetFilter(Decoupling_t *dec, float alpha)
{
    if (dec == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    dec->filter_alpha = alpha;
}

/**
 * @brief 一阶低通滤波器
 * @param x 当前输入
 * @param y_prev 上一次输出
 * @param alpha 滤波系数(0~1)
 * @return 滤波后的输出
 */
static float low_pass_filter(float x, float y_prev, float alpha)
{
    return alpha * y_prev + (1.0f - alpha) * x;
}

/**
 * @brief 前馈解耦更新
 * @param dec 解耦结构体指针
 * @param ref 参考输入数组(长度n)
 * @param out 解耦输出数组(长度n), 可为NULL
 *
 * @details 前馈解耦: u_dec[i] = -Σ_{j≠i}(Kf[i][j] * ref[j])
 *          对角线元素不参与解耦(自身通道)
 */
void Decoupling_FeedforwardUpdate(Decoupling_t *dec, const float *ref, float *out)
{
    if (dec == NULL || ref == NULL) return;

    uint8_t n = dec->n_channels;

    for (uint8_t i = 0; i < n; i++) {
        float sum = 0.0f;
        /* 累加其他通道的耦合影响 */
        for (uint8_t j = 0; j < n; j++) {
            if (i != j) {
                sum += dec->Kf[i][j] * ref[j];
            }
        }
        float raw = -sum;
        /* 低通滤波平滑 */
        dec->decouple_out[i] = low_pass_filter(raw, dec->prev_decouple_out[i],
                                                dec->filter_alpha);
        dec->prev_decouple_out[i] = dec->decouple_out[i];
    }

    /* 输出到调用者 */
    if (out) {
        for (uint8_t i = 0; i < n; i++)
            out[i] = dec->decouple_out[i];
    }
}

/**
 * @brief 反馈解耦更新
 * @param dec 解耦结构体指针
 * @param feedback 反馈测量数组(长度n)
 * @param out 解耦输出数组(长度n), 可为NULL
 *
 * @details 反馈解耦: u_dec[i] = -Σ_{j≠i}(D[i][j] * y[j])
 */
void Decoupling_FeedbackUpdate(Decoupling_t *dec, const float *feedback, float *out)
{
    if (dec == NULL || feedback == NULL) return;

    uint8_t n = dec->n_channels;

    for (uint8_t i = 0; i < n; i++) {
        float sum = 0.0f;
        for (uint8_t j = 0; j < n; j++) {
            if (i != j) {
                sum += dec->D[i][j] * feedback[j];
            }
        }
        float raw = -sum;
        dec->decouple_out[i] = low_pass_filter(raw, dec->prev_decouple_out[i],
                                                dec->filter_alpha);
        dec->prev_decouple_out[i] = dec->decouple_out[i];
    }

    if (out) {
        for (uint8_t i = 0; i < n; i++)
            out[i] = dec->decouple_out[i];
    }
}

/**
 * @brief 混合解耦更新(前馈+反馈)
 * @param dec 解耦结构体指针
 * @param ref 参考输入数组(长度n)
 * @param feedback 反馈测量数组(长度n)
 * @param out 解耦输出数组(长度n), 可为NULL
 *
 * @details 混合解耦: u_dec[i] = -(Σ_{j≠i}(Kf[i][j]*ref[j]) + Σ_{j≠i}(D[i][j]*y[j]))
 */
void Decoupling_HybridUpdate(Decoupling_t *dec, const float *ref,
                              const float *feedback, float *out)
{
    if (dec == NULL || ref == NULL || feedback == NULL) return;

    uint8_t n = dec->n_channels;

    for (uint8_t i = 0; i < n; i++) {
        float ff_sum = 0.0f, fb_sum = 0.0f;
        /* 前馈耦合 + 反馈耦合 */
        for (uint8_t j = 0; j < n; j++) {
            if (i != j) {
                ff_sum += dec->Kf[i][j] * ref[j];
                fb_sum += dec->D[i][j] * feedback[j];
            }
        }
        float raw = -(ff_sum + fb_sum);
        dec->decouple_out[i] = low_pass_filter(raw, dec->prev_decouple_out[i],
                                                dec->filter_alpha);
        dec->prev_decouple_out[i] = dec->decouple_out[i];
    }

    if (out) {
        for (uint8_t i = 0; i < n; i++)
            out[i] = dec->decouple_out[i];
    }
}

/**
 * @brief 获取指定通道的解耦输出
 * @param dec 解耦结构体指针
 * @param channel 通道索引
 * @return 解耦输出值, 越界返回0
 */
float Decoupling_GetOutput(Decoupling_t *dec, uint8_t channel)
{
    if (dec == NULL || channel >= dec->n_channels) return 0.0f;
    return dec->decouple_out[channel];
}

/**
 * @brief 重置解耦控制器状态
 * @param dec 解耦结构体指针
 */
void Decoupling_Reset(Decoupling_t *dec)
{
    if (dec == NULL) return;
    for (uint8_t i = 0; i < dec->n_channels; i++) {
        dec->decouple_out[i] = 0.0f;
        dec->prev_decouple_out[i] = 0.0f;
    }
}
