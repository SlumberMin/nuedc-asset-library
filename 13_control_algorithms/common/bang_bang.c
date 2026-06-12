/**
 * @file bang_bang.c
 * @brief Bang-Bang控制器实现
 */
#include "bang_bang.h"
#include <math.h>
#include <string.h>

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

void BangBang_Init(BangBang_t *bb, const BangBang_Config_t *config)
{
    if (bb == NULL || config == NULL) return;
    memset(bb, 0, sizeof(BangBang_t));
    memcpy(&bb->config, config, sizeof(BangBang_Config_t));
    bb->initialized = 1;
}

float BangBang_Calc(BangBang_t *bb, float setpoint, float feedback, float dt)
{
    if (bb == NULL || !bb->initialized) return 0.0f;

    float error = setpoint - feedback;
    float abs_error = fabsf(error);

    float output = 0.0f;

    switch (bb->config.mode) {
        case BB_MODE_SIMPLE: {
            /* 简单Bang-Bang */
            if (error > 0.0f) {
                output = bb->config.pos_output;
            } else if (error < 0.0f) {
                output = bb->config.neg_output;
            } else {
                output = 0.0f;
            }
            break;
        }

        case BB_MODE_HYSTERESIS: {
            /* 带滞回的Bang-Bang，防止在目标附近抖振 */
            float h = bb->config.hysteresis;
            if (error > h) {
                output = bb->config.pos_output;
            } else if (error < -h) {
                output = bb->config.neg_output;
            } else {
                /* 滞回区内保持上一次输出 */
                output = bb->output;
            }
            break;
        }

        case BB_MODE_PD_SWITCH: {
            /* 误差大时Bang-Bang，接近目标时切换PD精细控制 */
            if (abs_error > bb->config.switch_threshold) {
                /* Bang-Bang模式 */
                output = (error > 0.0f) ? bb->config.pos_output : bb->config.neg_output;
                bb->in_pd_mode = 0;
            } else {
                /* PD精细模式 */
                float d_error = 0.0f;
                if (dt > 0.0f) {
                    d_error = (error - bb->prev_error) / dt;
                }
                output = bb->config.switch_kp * error + bb->config.switch_kd * d_error;
                bb->in_pd_mode = 1;
            }
            break;
        }

        default:
            output = 0.0f;
            break;
    }

    /* 限幅 */
    output = CLAMP(output, bb->config.output_min, bb->config.output_max);

    bb->prev_error = error;
    bb->output = output;

    return output;
}

uint8_t BangBang_IsInPDMode(const BangBang_t *bb)
{
    return bb->in_pd_mode;
}

void BangBang_Reset(BangBang_t *bb)
{
    if (bb == NULL) return;
    bb->prev_error = 0.0f;
    bb->output = 0.0f;
    bb->in_pd_mode = 0;
}
