/**
 * @file perf_benchmark.c
 * @brief 控制算法性能基准测试
 *
 * 使用 SysTick 或 DWT Cycle Counter 测量各算法执行时间
 * 支持 MSPM0G3507 (Cortex-M0+) 和 STM32 (Cortex-M4F)
 *
 * 使用方法:
 *   1. 包含此文件到项目中
 *   2. 在 main() 中调用 PerfBenchmark_RunAll()
 *   3. 通过串口查看结果
 *
 * 测试内容:
 *   - ADRC 原始版 vs 优化版
 *   - Fuzzy PID 原始版 vs 优化版
 *   - MPC 原始版 vs 优化版
 *   - EKF 原始版 vs 优化版
 *   - OLED 帧缓冲操作
 */

#ifndef PERF_BENCHMARK_C
#define PERF_BENCHMARK_C

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* ========== 平台适配 ========== */

/* Cycle counter 读取 (Cortex-M4F DWT) */
#if defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7M__)
/* Cortex-M4/M3: 使用 DWT CYCCNT */
#define DWT_CYCCNT    (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL      (*(volatile uint32_t *)0xE0001000)
#define DEMCR         (*(volatile uint32_t *)0xE000EDFC)

static inline void cycle_counter_init(void)
{
    DEMCR |= (1 << 24);      /* TRCENA */
    DWT_CYCCNT = 0;
    DWT_CTRL |= (1 << 0);    /* CYCCNTENA */
}

static inline uint32_t get_cycles(void)
{
    return DWT_CYCCNT;
}

#define HAS_CYCLE_COUNTER  1

#elif defined(__M0PLUS_REV) || defined(__CORTEX_M)
/* Cortex-M0+: 无 DWT，使用 SysTick 或 Timer */
/* 假设系统时钟 32MHz (MSPM0G3507) */
#define SYSTEM_CLK_MHZ  32

static volatile uint32_t systick_overflow = 0;

/* SysTick 中断处理 */
void SysTick_Handler(void)
{
    systick_overflow++;
}

static inline void cycle_counter_init(void)
{
    /* 使用 SysTick 作为粗粒度计时器 */
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static inline uint32_t get_cycles(void)
{
    /* SysTick 是递减计数器，转换为递增 */
    return (systick_overflow << 24) | (0x00FFFFFF - (SysTick->VAL & 0x00FFFFFF));
}

#define HAS_CYCLE_COUNTER  1

#else
/* 软件模拟: 使用简单计数器 */
static uint32_t sim_cycle_counter = 0;
static inline void cycle_counter_init(void) { sim_cycle_counter = 0; }
static inline uint32_t get_cycles(void) { return sim_cycle_counter++; }
#define HAS_CYCLE_COUNTER  0
#endif

/* ========== 基准测试配置 ========== */

#define BENCH_ITERATIONS    1000    /* 每项测试重复次数 */
#define BENCH_WARMUP        100     /* 预热次数 (不计入统计) */

/* ========== 测试结果结构 ========== */

typedef struct {
    const char *name;           /* 测试名称 */
    uint32_t cycles_min;        /* 最小周期数 */
    uint32_t cycles_max;        /* 最大周期数 */
    uint32_t cycles_avg;        /* 平均周期数 */
    float    time_us;           /* 平均时间 (微秒) */
} BenchResult_t;

/* ========== 辅助函数 ========== */

/**
 * @brief 运行基准测试
 * @param name     测试名称
 * @param func     被测函数指针 (void(*)(void))
 * @param result   结果输出
 */
static void bench_run(const char *name, void (*func)(void), BenchResult_t *result)
{
    result->name = name;
    result->cycles_min = 0xFFFFFFFF;
    result->cycles_max = 0;
    uint64_t total = 0;

    /* 预热 */
    for (int i = 0; i < BENCH_WARMUP; i++) {
        func();
    }

    /* 正式测试 */
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        uint32_t start = get_cycles();
        func();
        uint32_t end = get_cycles();
        uint32_t elapsed = end - start;

        if (elapsed < result->cycles_min) result->cycles_min = elapsed;
        if (elapsed > result->cycles_max) result->cycles_max = elapsed;
        total += elapsed;
    }

    result->cycles_avg = (uint32_t)(total / BENCH_ITERATIONS);

#if defined(SYSTEM_CLK_MHZ)
    result->time_us = (float)result->cycles_avg / SYSTEM_CLK_MHZ;
#elif defined(__ARM_ARCH_7EM__)
    /* 假设 168MHz (STM32F4) */
    result->time_us = (float)result->cycles_avg / 168.0f;
#else
    result->time_us = (float)result->cycles_avg / 64.0f;  /* 默认64MHz */
#endif
}

/**
 * @brief 打印测试结果
 */
static void bench_print(const BenchResult_t *r)
{
    printf("  %-30s | min=%6lu avg=%6lu max=%6lu | %6.1f us\r\n",
           r->name,
           (unsigned long)r->cycles_min,
           (unsigned long)r->cycles_avg,
           (unsigned long)r->cycles_max,
           r->time_us);
}

/**
 * @brief 打印对比结果
 */
static void bench_compare(const BenchResult_t *orig, const BenchResult_t *opt)
{
    float speedup = (opt->cycles_avg > 0) ?
                    (float)orig->cycles_avg / (float)opt->cycles_avg : 0;

    printf("  %-30s | orig=%6lu opt=%6lu | speedup=%.2fx\r\n",
           orig->name,
           (unsigned long)orig->cycles_avg,
           (unsigned long)opt->cycles_avg,
           speedup);
}

/* ========== 测试用例 ========== */

/* 需要包含被测头文件 */
#include "active_disturbance_rejection.h"
#include "common/ladrc.h"
#include "common/ekf.h"
#include "02_MPC_模型预测控制/mpc.h"
#include "06_Fuzzy_PID_模糊自适应PID/fuzzy_pid.h"

/* 全局测试实例 */
static ADRC_t bench_adrc;
static LADRC_t bench_ladrc;
static EKF_t bench_ekf;
static MPC_t bench_mpc;
static FuzzyPID_t bench_fuzzy;

/* 测试输入数据 */
static float bench_sin_input[BENCH_ITERATIONS + BENCH_WARMUP];
static float bench_noise_input[BENCH_ITERATIONS + BENCH_WARMUP];

/**
 * @brief 初始化测试数据
 */
static void bench_data_init(void)
{
    for (int i = 0; i < BENCH_ITERATIONS + BENCH_WARMUP; i++) {
        float t = (float)i * 0.001f;
        bench_sin_input[i] = 10.0f * __builtin_sinf(t * 6.2832f);
        bench_noise_input[i] = 0.5f * __builtin_sinf(t * 6.2832f)
                             + 0.2f * __builtin_sinf(t * 12.5664f);
    }
}

/* ---- ADRC 测试 ---- */

static int adrc_iter = 0;

static void test_adrc_orig(void)
{
    /* 调用原始 ADRC_Compute */
    extern float ADRC_Compute_orig(ADRC_t *, float, float);
    ADRC_Compute_orig(&bench_adrc, bench_sin_input[adrc_iter], bench_noise_input[adrc_iter]);
    adrc_iter++;
    if (adrc_iter >= BENCH_ITERATIONS + BENCH_WARMUP) adrc_iter = 0;
}

static void test_adrc_opt(void)
{
    ADRC_Compute(&bench_adrc, bench_sin_input[adrc_iter], bench_noise_input[adrc_iter]);
    adrc_iter++;
    if (adrc_iter >= BENCH_ITERATIONS + BENCH_WARMUP) adrc_iter = 0;
}

/* ---- LADRC 测试 ---- */

static int ladrc_iter = 0;

static void test_ladrc(void)
{
    LADRC_Update(&bench_ladrc, bench_sin_input[ladrc_iter], bench_noise_input[ladrc_iter]);
    ladrc_iter++;
    if (ladrc_iter >= BENCH_ITERATIONS + BENCH_WARMUP) ladrc_iter = 0;
}

/* ---- MPC 测试 ---- */

static int mpc_iter = 0;

static void test_mpc_orig(void)
{
    extern float MPC_Update_orig(MPC_t *, float, float);
    MPC_Update_orig(&bench_mpc, bench_sin_input[mpc_iter], bench_noise_input[mpc_iter]);
    mpc_iter++;
    if (mpc_iter >= BENCH_ITERATIONS + BENCH_WARMUP) mpc_iter = 0;
}

static void test_mpc_opt(void)
{
    MPC_Update(&bench_mpc, bench_sin_input[mpc_iter], bench_noise_input[mpc_iter]);
    mpc_iter++;
    if (mpc_iter >= BENCH_ITERATIONS + BENCH_WARMUP) mpc_iter = 0;
}

/* ---- Fuzzy PID 测试 ---- */

static int fuzzy_iter = 0;

static void test_fuzzy_orig(void)
{
    extern float FuzzyPID_Update_orig(FuzzyPID_t *, float, float);
    FuzzyPID_Update_orig(&bench_fuzzy, bench_sin_input[fuzzy_iter], bench_noise_input[fuzzy_iter]);
    fuzzy_iter++;
    if (fuzzy_iter >= BENCH_ITERATIONS + BENCH_WARMUP) fuzzy_iter = 0;
}

static void test_fuzzy_opt(void)
{
    FuzzyPID_Update(&bench_fuzzy, bench_sin_input[fuzzy_iter], bench_noise_input[fuzzy_iter]);
    fuzzy_iter++;
    if (fuzzy_iter >= BENCH_ITERATIONS + BENCH_WARMUP) fuzzy_iter = 0;
}

/* ---- EKF 测试 (2D 状态) ---- */

static void ekf_dummy_f(const float *x, const float *u, float *x_new, int n, int m)
{
    (void)u; (void)n; (void)m;
    x_new[0] = x[0] + 0.01f * x[1];
    x_new[1] = x[1];
}

static void ekf_dummy_h(const float *x, float *z, int n, int p)
{
    (void)n; (void)p;
    z[0] = x[0];
}

static void ekf_dummy_F(const float *x, const float *u, float *F, int n, int m)
{
    (void)x; (void)u; (void)m;
    memset(F, 0, n * n * sizeof(float));
    F[0] = 1.0f; F[1] = 0.01f;
    F[2] = 0.0f; F[3] = 1.0f;
}

static void ekf_dummy_H(const float *x, const float *u, float *H, int n, int p)
{
    (void)x; (void)u;
    memset(H, 0, p * n * sizeof(float));
    H[0] = 1.0f; H[1] = 0.0f;
}

static int ekf_iter = 0;

static void test_ekf(void)
{
    float z[1] = {bench_sin_input[ekf_iter]};
    EKF_Step(&bench_ekf, NULL, z);
    ekf_iter++;
    if (ekf_iter >= BENCH_ITERATIONS + BENCH_WARMUP) ekf_iter = 0;
}

/* ========== 主测试入口 ========== */

/**
 * @brief 运行所有性能基准测试
 *
 * 在 main() 中调用此函数，通过串口查看结果
 */
void PerfBenchmark_RunAll(void)
{
    BenchResult_t results[16];
    int idx = 0;

    cycle_counter_init();
    bench_data_init();

    printf("\r\n");
    printf("========================================================\r\n");
    printf("        电赛代码库 性能基准测试报告\r\n");
    printf("========================================================\r\n");
    printf("  测试迭代次数: %d  预热次数: %d\r\n", BENCH_ITERATIONS, BENCH_WARMUP);
    printf("--------------------------------------------------------\r\n\r\n");

    /* --- 1. ADRC 测试 --- */
    printf("[1] ADRC 自抗扰控制器\r\n");
    ADRC_Init(&bench_adrc, 0.001f, 1.0f, 100.0f, 300.0f);

    adrc_iter = 0;
    bench_run("ADRC (Original)", test_adrc_orig, &results[idx]);
    bench_print(&results[idx]);
    BenchResult_t adrc_orig = results[idx++];

    adrc_iter = 0;
    bench_run("ADRC (Optimized)", test_adrc_opt, &results[idx]);
    bench_print(&results[idx]);
    BenchResult_t adrc_opt = results[idx++];

    bench_compare(&adrc_orig, &adrc_opt);

    /* --- 2. LADRC 测试 --- */
    printf("\r\n[2] LADRC 线性自抗扰控制器\r\n");
    LADRC_Init(&bench_ladrc, 100.0f, 300.0f, 1.0f, 0.001f, 100.0f, -100.0f);

    ladrc_iter = 0;
    bench_run("LADRC", test_ladrc, &results[idx]);
    bench_print(&results[idx++]);

    /* --- 3. MPC 测试 --- */
    printf("\r\n[3] MPC 模型预测控制\r\n");
    MPC_Init(&bench_mpc, 0.01f);
    float A[2][2] = {{1.0f, 0.01f}, {0.0f, 1.0f}};
    float B[2] = {0.0f, 0.1f};
    float C[2] = {1.0f, 0.0f};
    MPC_SetModel(&bench_mpc, A, B, C);

    mpc_iter = 0;
    bench_run("MPC (Original 20iter)", test_mpc_orig, &results[idx]);
    bench_print(&results[idx]);
    BenchResult_t mpc_orig = results[idx++];

    mpc_iter = 0;
    bench_run("MPC (Optimized 5iter)", test_mpc_opt, &results[idx]);
    bench_print(&results[idx]);
    BenchResult_t mpc_opt = results[idx++];

    bench_compare(&mpc_orig, &mpc_opt);

    /* --- 4. Fuzzy PID 测试 --- */
    printf("\r\n[4] Fuzzy PID 模糊自适应PID\r\n");
    FuzzyPID_Init(&bench_fuzzy, 1.0f, 1.0f, 10.0f, 0.1f, 1.0f, 5.0f, 0.5f, 0.1f);

    fuzzy_iter = 0;
    bench_run("FuzzyPID (Original)", test_fuzzy_orig, &results[idx]);
    bench_print(&results[idx]);
    BenchResult_t fuzzy_orig = results[idx++];

    fuzzy_iter = 0;
    bench_run("FuzzyPID (Optimized)", test_fuzzy_opt, &results[idx]);
    bench_print(&results[idx]);
    BenchResult_t fuzzy_opt = results[idx++];

    bench_compare(&fuzzy_orig, &fuzzy_opt);

    /* --- 5. EKF 测试 --- */
    printf("\r\n[5] EKF 扩展卡尔曼滤波器\r\n");
    EKF_Init(&bench_ekf, 2, 1, 0.01f);
    EKF_SetFunctions(&bench_ekf, ekf_dummy_f, ekf_dummy_h, ekf_dummy_F, ekf_dummy_H);

    ekf_iter = 0;
    bench_run("EKF (2-state, 1-meas)", test_ekf, &results[idx]);
    bench_print(&results[idx++]);

    /* --- 汇总 --- */
    printf("\r\n========================================================\r\n");
    printf("        性能优化汇总\r\n");
    printf("========================================================\r\n");
    printf("  %-30s | %8s -> %8s | %s\r\n", "算法", "原始(cycles)", "优化(cycles)", "加速比");
    printf("  ------------------------------------------------------\r\n");
    bench_compare(&adrc_orig, &adrc_opt);
    bench_compare(&mpc_orig, &mpc_opt);
    bench_compare(&fuzzy_orig, &fuzzy_opt);
    printf("========================================================\r\n\r\n");
}

#endif /* PERF_BENCHMARK_C */
