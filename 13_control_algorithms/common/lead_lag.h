/**
 * @file lead_lag.h
 * @brief 超前滞后补偿器
 *
 * 传递函数: H(s) = K * (s + z) / (s + p)
 *   - 超前(lead): z < p  -> 增加相位裕度,改善瞬态响应
 *   - 滞后(lag):  z > p  -> 提高低频增益,改善稳态精度
 *
 * 双线性变换离散化:
 *   s = (2/T) * (1 - z^-1) / (1 + z^-1)
 */

#ifndef LEAD_LAG_H
#define LEAD_LAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LEAD_COMPENSATOR,   /* 超前补偿 */
    LAG_COMPENSATOR,    /* 滞后补偿 */
    LEAD_LAG_COMPENSATOR /* 超前-滞后复合补偿 */
} LeadLagType_e;

typedef struct {
    float b0, b1, b2;   /* 分子系数 */
    float a0, a1, a2;   /* 分母系数 */
    float x1, x2;       /* 输入历史 */
    float y1, y2;       /* 输出历史 */
    float gain;          /* 前向增益 K */
} LeadLag_t;

/**
 * @brief 初始化超前补偿器
 * @param fc   超前补偿中心频率 (Hz)
 * @param alpha 超前比 (alpha > 1 为超前, alpha < 1 为滞后)
 * @param gain  增益 K
 * @param fs    采样频率 (Hz)
 */
void LeadLag_InitLead(LeadLag_t *comp, float fc, float alpha, float gain, float fs);

/**
 * @brief 初始化滞后补偿器
 * @param fc    滞后补偿中心频率 (Hz)
 * @param alpha 滞后比 (0 < alpha < 1)
 * @param gain  增益 K
 * @param fs    采样频率 (Hz)
 */
void LeadLag_InitLag(LeadLag_t *comp, float fc, float alpha, float gain, float fs);

/**
 * @brief 初始化超前-滞后复合补偿器(二阶)
 * @param fc_lead   超前部分中心频率
 * @param alpha_lead 超前比
 * @param fc_lag    滞后部分中心频率
 * @param alpha_lag 滞后比
 * @param gain      增益
 * @param fs        采样频率
 */
void LeadLag_InitLeadLag(LeadLag_t *comp,
                         float fc_lead, float alpha_lead,
                         float fc_lag,  float alpha_lag,
                         float gain, float fs);

/**
 * @brief 处理一个采样点
 */
float LeadLag_Update(LeadLag_t *comp, float input);

/**
 * @brief 重置补偿器状态
 */
void LeadLag_Reset(LeadLag_t *comp);

#ifdef __cplusplus
}
#endif

#endif /* LEAD_LAG_H */
