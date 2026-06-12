/**
 * @file c_driver_template.h
 * @brief C驱动头文件模板 — 标准化API声明与类型定义
 * @version 2.0
 * @date 2026-06-12
 *
 * 使用说明:
 *   1. 搜索替换 YOUR_MODULE 为实际模块名
 *   2. 根据外设类型调整API声明
 *   3. 保留 include guard / extern "C" / Doxygen 注释结构
 *
 * 本模板覆盖的错误模式:
 *   #14 示例代码与驱动API不匹配 → 所有API在此声明
 *   #40 README函数名与驱动API不匹配 → 统一API命名规范
 *   #42 缺少宏定义 → 提供必要宏
 *   #43 伪代码→可编译代码 → 提供真实类型定义
 */

#ifndef YOUR_MODULE_H    /* include guard — 防止重复包含 */
#define YOUR_MODULE_H

#ifdef __cplusplus
extern "C" {             /* C++兼容: 确保C链接约定 */
#endif

/* =========================================================================
 *  头文件包含
 * ========================================================================= */

#include <stdint.h>      /* uint8_t, int32_t 等标准整型 */
#include <stdbool.h>     /* bool, true, false */
#include <stddef.h>      /* NULL, size_t */

/* 条件包含: 根据平台选择 */
/* #include "ti_msp_dl_config.h" */   /* MSPM0 SysConfig */
/* #include "stm32f4xx_hal.h" */      /* STM32 HAL */
/* #include "platform/driverlib_mspm0.h" */  /* 自定义驱动层 */

/* =========================================================================
 *  版本信息
 * ========================================================================= */

#define YOUR_MODULE_VERSION_MAJOR   2
#define YOUR_MODULE_VERSION_MINOR   0
#define YOUR_MODULE_VERSION_PATCH   0
#define YOUR_MODULE_VERSION_STRING  "2.0.0"

/* =========================================================================
 *  硬件引脚分配 (参照 pin_config.h, 错误经验 #13, #17)
 *
 *  !! 重要: 引脚分配必须与pin_config.h一致，不能凭记忆分配
 *  !! 电机PWM必须使用TIMA0(高级定时器)，不能用TIMG0
 *  !! 编码器引脚注意不要与超声波/灰度传感器冲突 (错误经验 #17)
 * ========================================================================= */

/* 示例: 电机A (TIMA0, CC_0_INDEX, PA12) */
/* #define MOTOR_A_PWM_TIM      TIMA0 */
/* #define MOTOR_A_PWM_CC       CC_0_INDEX */
/* #define MOTOR_A_PWM_PIN      DL_GPIO_PIN_12 */
/* #define MOTOR_A_PWM_PORT     GPIOA */

/* 示例: 编码器 (错误经验 #17: 不要使用PB6/PB7, 与超声波冲突) */
/* #define ENCODER_A_GPIO_PORT  GPIOB */
/* #define ENCODER_A_PIN_A      DL_GPIO_PIN_4  // PB4 */
/* #define ENCODER_A_PIN_B      DL_GPIO_PIN_5  // PB5 */

/* =========================================================================
 *  宏定义 — 错误经验 #42 (方案代码缺宏定义)
 * ========================================================================= */

/** 安全钳位 */
#ifndef CLAMP
#define CLAMP(x, lo, hi)    ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

/** 角度↔弧度 (错误经验 #11: 单位转换) */
#ifndef DEG2RAD
#define DEG2RAD(d)          ((d) * 0.0174532925f)
#endif
#ifndef RAD2DEG
#define RAD2DEG(r)          ((r) * 57.29577951f)
#endif

/** PI (使用float精度) */
#ifndef M_PI_F
#define M_PI_F              3.14159265f
#endif

/** 重力加速度 */
#ifndef GRAVITY
#define GRAVITY             9.80665f
#endif

/** 除零保护最小值 (错误经验 #1) */
#ifndef DIV_SAFE_MIN
#define DIV_SAFE_MIN        1e-6f
#endif

/** 输出上下限 (可按需调整) */
#ifndef OUTPUT_MAX
#define OUTPUT_MAX          100.0f
#endif
#ifndef OUTPUT_MIN
#define OUTPUT_MIN          (-100.0f)
#endif

/** 积分饱和限幅 */
#ifndef INTEGRAL_MAX
#define INTEGRAL_MAX        1000.0f
#endif

/** 错误码定义 */
#define YOUR_OK             0
#define YOUR_ERR_PARAM      (-1)   /**< 参数非法 */
#define YOUR_ERR_TIMEOUT    (-2)   /**< 通信超时 */
#define YOUR_ERR_NOT_INIT   (-3)   /**< 未初始化 */
#define YOUR_ERR_OVERFLOW   (-4)   /**< 缓冲区溢出 */
#define YOUR_ERR_BUSY       (-5)   /**< 忙 */

/* =========================================================================
 *  类型定义 — 错误经验 #14, #40, #43
 *  (示例代码与驱动API类型匹配)
 * ========================================================================= */

/**
 * @brief 模块配置结构体
 *
 * @note 初始化时所有参数都会被校验 (错误经验 #16, #45)
 */
typedef struct {
    float    kp;             /**< 比例增益 (必须>0) */
    float    ki;             /**< 积分增益 (必须>0) */
    float    kd;             /**< 微分增益 */
    float    dt;             /**< 控制周期(秒), 必须>0 */
    float    output_max;     /**< 输出上限 */
    float    output_min;     /**< 输出下限 */
    float    integral_max;   /**< 积分饱和限幅 */
} YOUR_Config_t;

/**
 * @brief 模块句柄 (不透明类型, 外部不应直接访问)
 */
typedef struct YOUR_Handle YOUR_Handle_t;

/**
 * @brief 模块状态枚举
 */
typedef enum {
    YOUR_STATE_IDLE       = 0,  /**< 空闲 */
    YOUR_STATE_RUNNING    = 1,  /**< 运行中 */
    YOUR_STATE_ERROR      = 2,  /**< 错误状态 */
    YOUR_STATE_CALIBRATING = 3  /**< 校准中 */
} YOUR_State_t;

/**
 * @brief 传感器数据结构体 (ISR共享字段加volatile, 错误经验 #33)
 *
 * @note rx_byte/rx_buf/rx_index/rx_state/data在UART ISR中写入,
 *       主循环读取, 必须声明为volatile
 */
typedef struct {
    volatile uint8_t   rx_byte;      /**< ISR: 最近接收的字节 */
    volatile uint8_t   rx_buf[256];  /**< ISR: 接收缓冲区 */
    volatile uint16_t  rx_index;     /**< ISR: 缓冲区写入位置 */
    volatile uint8_t   rx_state;     /**< ISR: 接收状态机 */
    volatile uint8_t   data_ready;   /**< ISR: 数据就绪标志 */
    float              temperature;  /**< 温度值 */
    float              humidity;     /**< 湿度值 */
    uint32_t           timestamp;    /**< 数据时间戳 */
} YOUR_SensorData_t;

/**
 * @brief 回调函数类型
 *
 * @param[in] event  事件类型
 * @param[in] data   事件数据指针
 * @param[in] ctx    用户上下文指针
 */
typedef void (*YOUR_Callback_t)(uint32_t event, const void *data, void *ctx);

/* =========================================================================
 *  API函数声明 — 错误经验 #14 (所有API在此声明)
 *  (方案README中的函数名必须与这些声明匹配, 错误经验 #40)
 * ========================================================================= */

/**
 * @brief 初始化模块
 *
 * @param[in] config  配置参数 (参数会被校验, 错误经验 #16)
 *
 * @return 模块句柄, NULL=失败
 *
 * @note 所有参数会在内部校验: dt<=0会修正为0.001f, b=0会修正为1e-6f
 */
YOUR_Handle_t* YOUR_Init(const YOUR_Config_t *config);

/**
 * @brief 释放模块
 *
 * @param[in] handle  模块句柄
 */
void YOUR_Deinit(YOUR_Handle_t *handle);

/**
 * @brief 重置模块状态
 *
 * @param[in] handle  模块句柄
 */
void YOUR_Reset(YOUR_Handle_t *handle);

/**
 * @brief 执行一次计算
 *
 * @param[in]  handle    模块句柄
 * @param[in]  setpoint  目标值
 * @param[in]  feedback  反馈值
 * @param[out] output    输出指针
 *
 * @return YOUR_OK=成功, <0=错误码
 */
int YOUR_Calculate(YOUR_Handle_t *handle, float setpoint,
                   float feedback, float *output);

/**
 * @brief 获取模块状态
 *
 * @param[in] handle  模块句柄
 *
 * @return 当前状态枚举值
 */
YOUR_State_t YOUR_GetState(const YOUR_Handle_t *handle);

/**
 * @brief 获取版本字符串
 *
 * @return 版本字符串指针 (静态存储)
 */
const char* YOUR_GetVersion(void);

/* =========================================================================
 *  I2C/UART通信API
 * ========================================================================= */

/**
 * @brief I2C写入 (带超时保护, 错误经验 #6, #31, #32)
 *
 * @param[in] dev_addr  7位设备地址
 * @param[in] reg_addr  寄存器地址
 * @param[in] data      数据缓冲区
 * @param[in] len       数据长度
 *
 * @return YOUR_OK=成功, YOUR_ERR_TIMEOUT=超时
 */
int YOUR_I2C_Write(uint8_t dev_addr, uint8_t reg_addr,
                    const uint8_t *data, uint16_t len);

/**
 * @brief I2C读取 (带超时保护)
 *
 * @param[in]  dev_addr  7位设备地址
 * @param[in]  reg_addr  寄存器地址
 * @param[out] data      接收缓冲区
 * @param[in]  len       读取长度
 *
 * @return YOUR_OK=成功, YOUR_ERR_TIMEOUT=超时
 */
int YOUR_I2C_Read(uint8_t dev_addr, uint8_t reg_addr,
                   uint8_t *data, uint16_t len);

/**
 * @brief UART接收中断处理 (ISR中调用)
 *
 * @param[in] byte  接收到的字节
 *
 * @note 内部使用volatile变量存储 (错误经验 #5, #46)
 */
void YOUR_UART_RxIRQHandler(uint8_t byte);

/* =========================================================================
 *  PWM/舵机API
 * ========================================================================= */

/**
 * @brief 设置PWM占空比 (错误经验 #13: 必须使用TIMA0)
 *
 * @param[in] channel   PWM通道号
 * @param[in] duty_pct  占空比百分比 (0~100)
 *
 * @return YOUR_OK=成功
 */
int YOUR_SetPWM(uint8_t channel, float duty_pct);

/**
 * @brief 设置舵机角度 (错误经验 #34: 带除零保护)
 *
 * @param[in] channel    舵机通道号
 * @param[in] angle_deg  目标角度(度)
 *
 * @return YOUR_OK=成功
 */
int YOUR_SetServoAngle(uint8_t channel, float angle_deg);

/* =========================================================================
 *  回调注册
 * ========================================================================= */

/**
 * @brief 注册事件回调
 *
 * @param[in] handle   模块句柄
 * @param[in] callback 回调函数
 * @param[in] ctx      用户上下文
 *
 * @return YOUR_OK=成功
 */
int YOUR_RegisterCallback(YOUR_Handle_t *handle, YOUR_Callback_t callback,
                           void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* YOUR_MODULE_H */
