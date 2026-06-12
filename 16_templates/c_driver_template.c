/**
 * @file c_driver_template.c
 * @brief C驱动代码模板 — 基于47个已知错误模式的标准化防护
 * @version 2.0
 * @date 2026-06-12
 *
 * 使用说明:
 *   1. 搜索替换 YOUR_MODULE 为实际模块名 (如 Motor, Servo, Encoder)
 *   2. 根据外设类型填写具体寄存器操作
 *   3. 删除不适用的防护段落，但不要删除任何存在的防护
 *
 * 本模板覆盖的错误模式 (来自错误经验库):
 *   #1  除零风险         → 所有除法前检查除数
 *   #5  ISR共享变量缺volatile → 全局共享变量加volatile
 *   #6  I2C忙等待无超时   → 所有while循环加超时计数器
 *   #7  数组/缓冲区溢出   → 长度参数校验
 *   #8  死代码           → 避免赋值后立即覆盖
 *   #13 PWM定时器选择    → 使用TIMA0(高级定时器)
 *   #16 参数未校验       → 函数入口校验
 *   #22 static变量冲突   → 单实例警告或多实例方案
 *   #23 系统增益b=0      → ADRC/LADRC b参数保护
 *   #24 滤波器Q参数      → Q<=0保护
 *   #26 dt未校验         → dt<=0保护
 *   #31-32 I2C超时       → 50ms超时或计数器超时
 *   #34 舵机角度除零     → range检查
 *   #41 浮点函数f后缀    → fabsf/sinf/cosf
 *   #45 除零系统防护     → 入口校验
 *   #46 volatile批量修复 → ISR共享变量
 *   #48 运算符优先级     → !(x & flag) 而非 !x & flag
 */

/* =========================================================================
 *  文件头
 * ========================================================================= */
#include "your_driver.h"    /* 对应的头文件 */
#include <math.h>           /* fabsf, sinf, cosf 等 */
#include <string.h>         /* memset, memcpy */

/* =========================================================================
 *  宏定义 — 防护常量
 * ========================================================================= */

/** 除零保护最小值 (错误经验 #1, #45) */
#define DIV_SAFE_MIN        1e-6f

/** I2C/UART超时计数器 (错误经验 #6, #31, #32) */
#define HW_TIMEOUT_COUNT    100000U

/** I2C HAL超时 (STM32平台, 错误经验 #31) */
#define I2C_HAL_TIMEOUT_MS  50U

/** 数组/缓冲区安全上限 (错误经验 #7, #35) */
#define MAX_BUF_SIZE        256U

/** 角度↔弧度转换 */
#ifndef DEG2RAD
#define DEG2RAD(d)  ((d) * 0.0174532925f)
#endif
#ifndef RAD2DEG
#define RAD2DEG(r)  ((r) * 57.29577951f)
#endif

/** 安全钳位宏 (错误经验 #42 — 方案代码缺宏定义) */
#ifndef CLAMP
#define CLAMP(x, lo, hi)    ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

/* =========================================================================
 *  ISR共享变量 — 必须volatile (错误经验 #5, #46)
 *
 *  说明: 中断服务函数(ISR)写入、主循环读取的变量必须声明为volatile，
 *  否则编译器优化可能导致主循环读到陈旧值。
 * ========================================================================= */

/** ISR: UART接收字节 (错误经验 #33) */
static volatile uint8_t  s_rx_byte = 0;

/** ISR: 数据就绪标志 */
static volatile uint8_t  s_data_ready = 0;

/** ISR: 系统节拍计数 */
static volatile uint32_t s_tick_count = 0;

/** ISR: 接收缓冲区 (共享给主循环) */
static volatile uint8_t  s_rx_buf[MAX_BUF_SIZE];

/** ISR: 接收计数 */
static volatile uint16_t s_rx_count = 0;

/* =========================================================================
 *  模块实例 — 警告static单实例限制 (错误经验 #22)
 *
 *  说明: 若使用static局部变量存储状态，多个实例会共享同一变量。
 *  对于需要多实例的场景，请改用结构体指针传入。
 * ========================================================================= */

/** 模块状态结构体 */
typedef struct {
    float    kp, ki, kd;       /**< PID参数 */
    float    integral;         /**< 积分累积 */
    float    prev_error;       /**< 上次误差 (用于微分) */
    float    output;           /**< 输出值 */
    float    dt;               /**< 控制周期(秒) */
    uint8_t  initialized;      /**< 初始化标志 */
} YOUR_Module_t;

/** @warning 单实例设计 — 多实例请使用动态分配或传入指针 (错误经验 #22) */
static YOUR_Module_t s_module = {0};

/* =========================================================================
 *  初始化函数
 * ========================================================================= */

/**
 * @brief 初始化模块
 *
 * @param[in] kp    比例增益 (不能为0)
 * @param[in] ki    积分增益 (不能为0)
 * @param[in] kd    微分增益
 * @param[in] dt    控制周期(秒), 必须>0
 *
 * @return 0=成功, -1=参数非法
 *
 * @note 防护措施:
 *   - dt<=0 保护 (错误经验 #26)
 *   - kp/ki为0保护 (错误经验 #1, #45)
 *   - 参数NaN/Inf检查
 */
int YOUR_Init(float kp, float ki, float kd, float dt)
{
    /* --- 参数校验: NaN/Inf --- */
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || !isfinite(dt)) {
        return -1;
    }

    /* --- dt校验 (错误经验 #26: dt=0导致除零) --- */
    if (dt <= 0.0f) {
        dt = 0.001f;  /* 默认1ms */
    }

    /* --- 增益校验 (错误经验 #1, #45: 除零防护) --- */
    if (fabsf(kp) < DIV_SAFE_MIN) {
        kp = DIV_SAFE_MIN;
    }
    if (fabsf(ki) < DIV_SAFE_MIN) {
        ki = DIV_SAFE_MIN;
    }

    /* --- 填充实例 --- */
    memset(&s_module, 0, sizeof(s_module));
    s_module.kp          = kp;
    s_module.ki          = ki;
    s_module.kd          = kd;
    s_module.dt          = dt;
    s_module.integral    = 0.0f;
    s_module.prev_error  = 0.0f;
    s_module.output      = 0.0f;
    s_module.initialized = 1;

    return 0;
}

/* =========================================================================
 *  核心计算函数
 * ========================================================================= */

/**
 * @brief 执行一次PID计算
 *
 * @param[in] setpoint  目标值
 * @param[in] feedback  反馈值
 *
 * @return 控制输出 (已钳位)
 *
 * @note 防护措施:
 *   - 除零: dt已在Init校验, 但再次确认 (防御性编程)
 *   - 积分饱和: 带钳位的积分累积
 *   - 输出钳位: ±OUTPUT_MAX
 *   - 数值微分: 使用prev_error而非0 (错误经验 #21)
 */
float YOUR_Calculate(float setpoint, float feedback)
{
    /* --- 初始化检查 --- */
    if (!s_module.initialized) {
        return 0.0f;
    }

    /* --- 除零防护 (错误经验 #1) --- */
    float dt = s_module.dt;
    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    /* --- 误差计算 --- */
    float error = setpoint - feedback;

    /* --- 比例项 --- */
    float p_term = s_module.kp * error;

    /* --- 积分项 (带饱和钳位) --- */
    s_module.integral += error * dt;
    #define INTEGRAL_MAX  1000.0f
    s_module.integral = CLAMP(s_module.integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    float i_term = s_module.ki * s_module.integral;

    /* --- 微分项 (使用prev_error, 错误经验 #21: 不要用0替代历史值) --- */
    float d_term = 0.0f;
    if (dt > 0.0f) {
        float d_error = (error - s_module.prev_error) / dt;
        d_term = s_module.kd * d_error;
    }
    s_module.prev_error = error;  /* 保存本次误差供下次微分使用 */

    /* --- 输出计算与钳位 --- */
    #define OUTPUT_MAX  100.0f
    s_module.output = CLAMP(p_term + i_term + d_term, -OUTPUT_MAX, OUTPUT_MAX);

    return s_module.output;
}

/* =========================================================================
 *  I2C通信函数 — 带超时保护 (错误经验 #6, #31, #32)
 * ========================================================================= */

/**
 * @brief I2C写入字节 (带超时)
 *
 * @param[in] dev_addr  设备地址
 * @param[in] reg_addr  寄存器地址
 * @param[in] data      写入数据
 *
 * @return 0=成功, -1=超时
 *
 * @note 超时保护防止I2C总线异常时MCU卡死 (错误经验 #6, #31, #32)
 */
int YOUR_I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    /* 方案A: 计数器超时 (适用于MSPM0/TM4C等裸机平台) */
    volatile uint32_t timeout = HW_TIMEOUT_COUNT;

    /* TODO: 替换为实际I2C启动+发送序列 */
    /* I2C_Start(); */
    /* I2C_SendAddr(dev_addr, I2C_WRITE); */
    /* I2C_SendData(reg_addr); */
    /* I2C_SendData(data); */

    /* 等待传输完成 (错误经验 #6: 必须有超时) */
    /* while (I2C_IsBusy() && --timeout) {}  // TODO: 替换为实际忙检测 */

    if (timeout == 0) {
        /* I2C_Stop();  // 超时后强制停止 */
        return -1;  /* 超时错误 */
    }

    /* I2C_Stop(); */
    return 0;
}

/**
 * @brief I2C读取字节 (带超时)
 *
 * @param[in]  dev_addr  设备地址
 * @param[in]  reg_addr  寄存器地址
 * @param[out] data      读取缓冲区
 *
 * @return 0=成功, -1=超时
 */
int YOUR_I2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    if (data == NULL) {
        return -1;  /* 空指针检查 */
    }

    volatile uint32_t timeout = HW_TIMEOUT_COUNT;

    /* TODO: 替换为实际I2C启动+重启动+接收序列 */

    /* 等待接收完成 */
    /* while (I2C_IsBusy() && --timeout) {} */

    if (timeout == 0) {
        /* I2C_Stop(); */
        return -1;
    }

    /* *data = I2C_GetData(); */
    /* I2C_Stop(); */
    return 0;
}

/* =========================================================================
 *  STM32 HAL平台I2C示例 (错误经验 #31)
 * ========================================================================= */
#if defined(USE_HAL_DRIVER)  /* 仅在STM32 HAL环境下编译 */

#include "stm32f4xx_hal.h"  /* 或对应系列 */

/**
 * @brief STM32 HAL I2C写入 (不使用HAL_MAX_DELAY)
 *
 * @note 错误经验 #31: HAL_MAX_DELAY(0xFFFFFFFF)会导致I2C总线异常时
 *       MCU永久阻塞。必须使用有限超时值。
 */
int YOUR_I2C_HAL_Write(I2C_HandleTypeDef *hi2c, uint8_t dev_addr,
                        uint8_t reg_addr, uint8_t data)
{
    /* !! 错误写法: HAL_I2C_Master_Transmit(hi2c, addr, &data, 1, HAL_MAX_DELAY); */
    /* !! 正确写法: 使用有限超时 */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        hi2c,
        (uint16_t)(dev_addr << 1),  /* 7位地址左移1位 */
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        I2C_HAL_TIMEOUT_MS          /* 错误经验 #31: 50ms而非HAL_MAX_DELAY */
    );

    return (status == HAL_OK) ? 0 : -1;
}

#endif /* USE_HAL_DRIVER */

/* =========================================================================
 *  数组操作 — 带边界检查 (错误经验 #7, #35)
 * ========================================================================= */

/**
 * @brief 安全写入缓冲区
 *
 * @param[in] buf   缓冲区指针
 * @param[in] size  缓冲区大小
 * @param[in] idx   写入索引
 * @param[in] val   写入值
 *
 * @return 0=成功, -1=越界
 *
 * @note 错误经验 #7, #35: 数组/缓冲区溢出防护
 */
int YOUR_BufWrite(uint8_t *buf, uint16_t size, uint16_t idx, uint8_t val)
{
    if (buf == NULL || idx >= size) {
        return -1;  /* 越界保护 */
    }
    buf[idx] = val;
    return 0;
}

/**
 * @brief 整数转字符串 (带缓冲区保护, 错误经验 #35)
 *
 * @param[out] buf   输出缓冲区
 * @param[in]  size  缓冲区大小
 * @param[in]  num   要转换的整数
 *
 * @return 写入的字符数, -1=缓冲区不足
 */
int YOUR_IntToStr(char *buf, uint16_t size, int32_t num)
{
    if (buf == NULL || size < 2) {
        return -1;
    }

    /* int32_t最多10位数字 + 符号 + null = 12字节 */
    /* 错误经验 #35: 缓冲区至少需要16字节 */
    if (size < 16) {
        return -1;
    }

    /* 安全转换 */
    int len = snprintf(buf, size, "%ld", (long)num);
    return (len > 0 && (uint16_t)len < size) ? len : -1;
}

/* =========================================================================
 *  角度/脉宽转换 — 带除零保护 (错误经验 #34)
 * ========================================================================= */

/**
 * @brief 角度转脉宽(微秒)
 *
 * @param[in] angle_deg    角度(度)
 * @param[in] min_pulse_us 最小脉宽(微秒)
 * @param[in] max_pulse_us 最大脉宽(微秒)
 * @param[in] min_angle    最小角度(度)
 * @param[in] max_angle    最大角度(度)
 *
 * @return 脉宽(微秒)
 *
 * @note 错误经验 #34: (max_angle-min_angle)可能为0导致除零
 */
uint16_t YOUR_AngleToPulse(float angle_deg, uint16_t min_pulse_us,
                            uint16_t max_pulse_us, float min_angle,
                            float max_angle)
{
    /* 错误经验 #34: range除零防护 */
    float angle_range = max_angle - min_angle;
    if (fabsf(angle_range) < 1e-3f) {
        return (uint16_t)((min_pulse_us + max_pulse_us) / 2);  /* 返回中间值 */
    }

    float pulse_range = (float)(max_pulse_us - min_pulse_us);
    float pulse = (float)min_pulse_us +
                  (angle_deg - min_angle) / angle_range * pulse_range;

    return (uint16_t)CLAMP(pulse, (float)min_pulse_us, (float)max_pulse_us);
}

/* =========================================================================
 *  UART中断服务函数示例 (错误经验 #5, #46)
 * ========================================================================= */

/**
 * @brief UART接收中断回调
 *
 * @note ISR中写入的变量必须声明为volatile (错误经验 #5, #46)
 * @note 接收缓冲区有边界保护 (错误经验 #7)
 */
void YOUR_UART_RxCallback(uint8_t byte)
{
    /* 错误经验 #5, #46: s_rx_count在ISR中写入，必须volatile */
    if (s_rx_count < MAX_BUF_SIZE - 1) {
        s_rx_buf[s_rx_count++] = byte;  /* 错误经验 #7: 边界检查 */
    }
    s_rx_byte   = byte;
    s_data_ready = 1;
}

/**
 * @brief 读取接收缓冲区 (主循环调用)
 *
 * @param[out] dst  目标缓冲区
 * @param[in]  max_len 最大读取长度
 *
 * @return 实际读取字节数
 *
 * @note 读取后清除缓冲区和标志
 */
uint16_t YOUR_UART_Read(uint8_t *dst, uint16_t max_len)
{
    if (dst == NULL || max_len == 0) {
        return 0;
    }

    /* 关中断保护 (临界区) */
    /* __disable_irq(); 或 ENTER_CRITICAL(); */

    uint16_t count = s_rx_count;
    if (count > max_len) {
        count = max_len;
    }

    /* 从volatile缓冲区复制 */
    for (uint16_t i = 0; i < count; i++) {
        dst[i] = s_rx_buf[i];
    }

    s_rx_count   = 0;
    s_data_ready = 0;

    /* __enable_irq(); 或 EXIT_CRITICAL(); */

    return count;
}

/* =========================================================================
 *  PWM输出示例 (错误经验 #13: 使用TIMA0而非TIMG0)
 * ========================================================================= */

/**
 * @brief 设置PWM占空比
 *
 * @param[in] duty_pct  占空比百分比 (0~100)
 *
 * @note 错误经验 #13: 电机PWM必须使用TIMA0(高级定时器),
 *       不能使用TIMG0(通用定时器)。引脚分配参照pin_config.h。
 */
void YOUR_SetPWM(float duty_pct)
{
    duty_pct = CLAMP(duty_pct, 0.0f, 100.0f);

    /* TODO: 替换为实际定时器操作 */
    /* uint32_t period = DL_Timer_getLoadValue(TIMA0); */
    /* uint32_t compare = (uint32_t)((duty_pct / 100.0f) * period); */
    /* DL_Timer_setCaptureCompareValue(TIMA0, compare, CC_0_INDEX); */
    (void)duty_pct;  /* 占位: 避免未使用变量警告 */
}

/* =========================================================================
 *  运算符优先级防护示例 (错误经验 #18)
 * ========================================================================= */

/**
 * @brief 安全的ADC状态检测示例
 *
 * @note 错误经验 #18: !优先于&，必须加括号
 *   - 错误: while (!DL_ADC12_getStatus(inst) & DL_ADC12_STATUS_CONVERSION_DONE)
 *   - 正确: while (!(DL_ADC12_getStatus(inst) & DL_ADC12_STATUS_CONVERSION_DONE))
 */
void YOUR_ADC_WaitConversion(void)
{
    volatile uint32_t timeout = HW_TIMEOUT_COUNT;

    /* !! 错误写法: while (!getStatus(inst) & FLAG) — !优先于& */
    /* 正确写法: */
    /* while (!(DL_ADC12_getStatus(ADC_INST) & DL_ADC12_STATUS_CONVERSION_DONE) */
    /*        && --timeout) {} */

    if (timeout == 0) {
        /* 超时处理 */
        return;
    }
}

/* =========================================================================
 *  反馈控制符号检查 (错误经验 #18: 控制仿真反馈符号)
 * ========================================================================= */

/**
 * @brief 简单一阶系统仿真步进
 *
 * @param[in]  output   控制器输出
 * @param[in]  dt       时间步长(秒)
 * @param[out] state    系统状态(输入输出)
 *
 * @note 错误经验 #18: 控制律u=-k*sign(s)已含负号时，
 *       plant模型必须取反闭合负反馈: state -= output * dt
 *       而非 state += output * dt (会变成正反馈导致发散)
 */
void YOUR_PlantStep(float output, float dt, float *state)
{
    if (state == NULL || dt <= 0.0f) {
        return;
    }
    /* 负反馈: 确保闭环稳定 */
    *state -= output * dt;  /* !! 注意符号: +=会导致正反馈发散 */
}

/* =========================================================================
 *  模块信息函数
 * ========================================================================= */

/**
 * @brief 获取模块版本
 *
 * @return 版本字符串
 */
const char* YOUR_GetVersion(void)
{
    return "YOUR_Module v2.0 (error-proof template)";
}
