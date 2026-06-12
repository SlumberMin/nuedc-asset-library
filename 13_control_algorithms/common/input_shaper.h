/**
 * @file input_shaper.h
 * @brief 输入整形器 (Input Shaping)
 *
 * 用于抑制柔性系统的残余振动。
 * 通过将一系列脉冲与参考输入卷积, 在指定时间消除振荡。
 *
 * 支持:
 *   1. ZV (Zero Vibration) 整形器 - 最短时滞, 对参数敏感
 *   2. ZVD (Zero Vibration and Derivative) 整形器 - 鲁棒性更好
 *   3. ZVDD (ZV Double Derivative) 整形器 - 高鲁棒性
 *   4. EI (Extra Insensitive) 整形器 - 允许少量残余振动换取更短时滞
 *
 * 使用流程:
 *   1. 初始化: InputShaper_Init() 设置固有频率和阻尼比
 *   2. 每个采样周期: 用 InputShaper_Update() 对参考值整形
 */

#ifndef INPUT_SHAPER_H
#define INPUT_SHAPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 整形器类型 */
typedef enum {
    IS_TYPE_ZV,
    IS_TYPE_ZVD,
    IS_TYPE_ZVDD,
    IS_TYPE_EI
} InputShaperType_e;

/* 脉冲序列 (最多支持4个脉冲) */
#define IS_MAX_IMPULSES 4

typedef struct {
    float amplitude;        /* 脉冲幅值 */
    float time;             /* 脉冲时刻 */
} Impulse_t;

/* 整形器实例 */
typedef struct {
    InputShaperType_e type;
    float freq;             /* 固有频率 (Hz) */
    float zeta;             /* 阻尼比 */
    float dt;               /* 采样周期 */

    /* 脉冲序列 */
    Impulse_t impulses[IS_MAX_IMPULSES];
    uint8_t num_impulses;

    /* 延迟缓冲区 */
    float *buffer;          /* 延迟线缓冲区 */
    uint16_t buffer_size;   /* 缓冲区大小 */
    uint16_t write_idx;     /* 写指针 */
    uint16_t delay_samples; /* 最大延迟对应的采样数 */

    /* 整形后的输出 */
    float output;
    uint8_t initialized;
} InputShaper_t;

/* ========== 初始化与销毁 ========== */
void InputShaper_Init(InputShaper_t *shaper, InputShaperType_e type,
                      float freq_hz, float zeta, float dt,
                      float *buffer_mem, uint16_t buffer_size);
void InputShaper_Reset(InputShaper_t *shaper);

/* ========== 整形计算 ========== */
float InputShaper_Update(InputShaper_t *shaper, float reference);
float InputShaper_GetOutput(InputShaper_t *shaper);
float InputShaper_GetDelay(InputShaper_t *shaper);

/* ========== 工具函数 ========== */
void InputShaper_ComputeZV(float freq_hz, float zeta, Impulse_t *impulses, uint8_t *count);
void InputShaper_ComputeZVD(float freq_hz, float zeta, Impulse_t *impulses, uint8_t *count);
void InputShaper_ComputeZVDD(float freq_hz, float zeta, Impulse_t *impulses, uint8_t *count);
void InputShaper_ComputeEI(float freq_hz, float zeta, float allowed_vib,
                           Impulse_t *impulses, uint8_t *count);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_SHAPER_H */
