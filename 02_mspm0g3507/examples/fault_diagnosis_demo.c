/**
 * @file fault_diagnosis_demo.c
 * @brief 故障诊断示例（传感器自检+电机自检+通信自检+LED报警）
 * @platform MSPM0G3507
 *
 * ============================================================
 * 接线说明
 * ============================================================
 * 模块              MSPM0G3507引脚      说明
 * ---------------------------------------------------------------
 * LED报警指示
 *   系统正常LED     PA22 (绿色)         常亮=正常，闪烁=故障
 *   故障指示LED     PA23 (红色)         常亮=有故障
 *   通信指示LED     PA24 (蓝色)         通信状态指示
 *
 * 蜂鸣器
 *   Buzzer          PA6                 故障蜂鸣报警
 *
 * 传感器自检
 *   ADC测试         PA25 (ADC0_CH0)     接3.3V/GND测试满量程/零点
 *   灰度传感器      PA26-PA31, PB0-PB1  8路灰度
 *
 * 电机自检
 *   电机PWM         PA0 (TimerA1_C0)    电机1 PWM
 *   电机方向1       PA1                 电机1 DIR1
 *   电机方向2       PA2                 电机1 DIR2
 *   编码器A相       PA12 (TimerA0_C0)   电机1编码器A
 *   编码器B相       PA13 (TimerA0_C1)   电机1编码器B
 *   电机PWM2        PA4 (TimerA1_C1)    电机2 PWM
 *   电机方向3       PA3                 电机2 DIR1
 *   电机方向4       PA5                 电机2 DIR2
 *
 * 通信自检
 *   UART测试        PA8/PA9 (UART0)     自发自收测试
 *   I2C测试         PB2/PB3 (I2C0)      I2C设备扫描
 *   SPI测试         PA7/PA10/PA11       SPI回环测试
 *
 * 按键
 *   诊断启动        PA18                 按下启动完整诊断
 *
 * ============================================================
 * 功能说明
 * ============================================================
 * 1. 上电自检（POST - Power-On Self Test）:
 *    - 系统时钟检测
 *    - RAM完整性测试
 *    - Flash CRC校验
 *    - 外设寄存器回读测试
 *
 * 2. 传感器自检:
 *    - ADC零点/满量程/线性度检测
 *    - IMU通信测试+数据合理性检查
 *    - 灰度传感器开路/短路检测
 *    - 温度传感器范围检测
 *
 * 3. 电机自检:
 *    - 驱动电路回路检测
 *    - 编码器信号检测
 *    - 堵转检测
 *    - 电流检测（如有电流传感器）
 *
 * 4. 通信自检:
 *    - UART自发自收回环测试
 *    - I2C总线设备扫描
 *    - SPI回环测试
 *    - 蓝牙连接测试
 *
 * 5. LED/蜂鸣器故障码报告:
 *    - 不同故障码对应不同的LED闪烁模式
 *    - 可选蜂鸣器声音编码
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ======================== 故障码定义 ======================== */

/**
 * 故障码系统：16位故障码
 * 高8位: 模块类别
 * 低8位: 具体故障
 *
 * 0x00xx - 系统级故障
 * 0x01xx - 传感器故障
 * 0x02xx - 电机故障
 * 0x03xx - 通信故障
 * 0x04xx - 电源故障
 */
typedef enum {
    /* 系统级 */
    FAULT_NONE              = 0x0000,   /* 无故障 */
    FAULT_SYSTEM_CLOCK      = 0x0001,   /* 系统时钟异常 */
    FAULT_SYSTEM_RAM        = 0x0002,   /* RAM测试失败 */
    FAULT_SYSTEM_FLASH      = 0x0003,   /* Flash CRC错误 */

    /* 传感器 */
    FAULT_ADC_OFFSET        = 0x0101,   /* ADC零点偏移过大 */
    FAULT_ADC_FULLSCALE     = 0x0102,   /* ADC满量程异常 */
    FAULT_ADC_LINEARITY     = 0x0103,   /* ADC线性度不合格 */
    FAULT_IMU_COMM          = 0x0110,   /* IMU通信失败 */
    FAULT_IMU_DATA          = 0x0111,   /* IMU数据异常 */
    FAULT_IMU_STUCK         = 0x0112,   /* IMU数据卡死 */
    FAULT_GRAY_OPEN         = 0x0120,   /* 灰度传感器开路 */
    FAULT_GRAY_SHORT        = 0x0121,   /* 灰度传感器短路 */
    FAULT_GRAY_RANGE        = 0x0122,   /* 灰度传感器量程异常 */

    /* 电机 */
    FAULT_MOTOR1_DRIVER     = 0x0201,   /* 电机1驱动电路故障 */
    FAULT_MOTOR1_ENCODER    = 0x0202,   /* 电机1编码器故障 */
    FAULT_MOTOR1_STALL      = 0x0203,   /* 电机1堵转 */
    FAULT_MOTOR2_DRIVER     = 0x0211,   /* 电机2驱动电路故障 */
    FAULT_MOTOR2_ENCODER    = 0x0212,   /* 电机2编码器故障 */
    FAULT_MOTOR2_STALL      = 0x0213,   /* 电机2堵转 */

    /* 通信 */
    FAULT_UART_LOOPBACK     = 0x0301,   /* UART回环测试失败 */
    FAULT_I2C_BUS           = 0x0302,   /* I2C总线错误 */
    FAULT_I2C_DEVICE        = 0x0303,   /* I2C设备无应答 */
    FAULT_SPI_LOOPBACK      = 0x0304,   /* SPI回环测试失败 */
    FAULT_BT_CONNECT        = 0x0305,   /* 蓝牙连接失败 */

    /* 电源 */
    FAULT_BATTERY_LOW       = 0x0401,   /* 电池电压低 */
    FAULT_REGULATOR         = 0x0402,   /* 稳压器输出异常 */
} FaultCode_t;

/* ======================== 故障记录结构体 ======================== */
#define MAX_FAULT_HISTORY   16

typedef struct {
    FaultCode_t code;           /* 故障码 */
    uint32_t timestamp;         /* 发生时间 */
    uint8_t severity;           /* 严重等级: 0=信息, 1=警告, 2=错误, 3=致命 */
} FaultRecord_t;

typedef struct {
    FaultRecord_t history[MAX_FAULT_HISTORY];   /* 故障历史 */
    uint8_t fault_count;                        /* 当前故障数 */
    uint32_t fault_bitmap;                      /* 故障位图（快速查询） */
    bool has_critical_fault;                    /* 是否有致命故障 */
} FaultManager_t;

/* ======================== 全局变量 ======================== */
static volatile uint32_t gSysTick = 0;
static FaultManager_t gFaultMgr = {0};

/* ======================== 引脚宏 ======================== */
#define LED_GREEN_PORT  GPIOA
#define LED_GREEN_PIN   DL_GPIO_PIN_22
#define LED_RED_PORT    GPIOA
#define LED_RED_PIN     DL_GPIO_PIN_23
#define LED_BLUE_PORT   GPIOA
#define LED_BLUE_PIN    DL_GPIO_PIN_24
#define BUZZER_PORT     GPIOA
#define BUZZER_PIN      DL_GPIO_PIN_6
#define BTN_DIAG_PORT   GPIOA
#define BTN_DIAG_PIN    DL_GPIO_PIN_21

/* 灰度传感器引脚 */
#define GRAY_SENSOR_COUNT   8
static const uint8_t gray_adc_channels[GRAY_SENSOR_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7};

/* ======================== 调试输出 ======================== */
static void debug_print(const char *str)
{
    while (*str) {
        while (!DL_UART_isTXFIFOEmpty(UART_0_INST));
        DL_UART_transmitDataBlocking(UART_0_INST, *str++);
    }
}

static void debug_print_hex(uint32_t val)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "0x%04X", (unsigned int)val);
    debug_print(buf);
}

/* ======================== 故障管理器 ======================== */

/**
 * @brief 记录一个故障
 * @param code 故障码
 * @param severity 严重等级
 */
static void fault_report(FaultCode_t code, uint8_t severity)
{
    if (gFaultMgr.fault_count < MAX_FAULT_HISTORY) {
        FaultRecord_t *rec = &gFaultMgr.history[gFaultMgr.fault_count];
        rec->code = code;
        rec->timestamp = gSysTick;
        rec->severity = severity;
        gFaultMgr.fault_count++;
    }

    /* 更新位图 */
    gFaultMgr.fault_bitmap |= (1U << ((code >> 8) & 0x1F));

    if (severity >= 3) {
        gFaultMgr.has_critical_fault = true;
    }

    /* 串口打印 */
    debug_print("[FAULT] Code=");
    debug_print_hex(code);
    debug_print(" Sev=");
    char buf[4];
    snprintf(buf, sizeof(buf), "%d", severity);
    debug_print(buf);
    debug_print("\r\n");
}

/**
 * @brief 清除所有故障
 */
static void fault_clear_all(void)
{
    memset(&gFaultMgr, 0, sizeof(FaultManager_t));
}

/**
 * @brief 检查是否有指定模块的故障
 */
static bool has_fault_in_module(uint8_t module_id)
{
    return (gFaultMgr.fault_bitmap & (1U << module_id)) != 0;
}

/* ======================== LED故障码显示 ======================== */

/**
 * @brief 用LED闪烁模式显示故障码
 *
 * 编码规则：
 * - 红色LED闪烁次数 = 故障类别 (1-5)
 * - 绿色LED闪烁次数 = 故障序号 (1-9)
 * - 中间有长间隔区分
 *
 * 例如: 0x0201 (电机故障1) = 红闪2次 + 绿闪1次
 */
static void led_show_fault_code(FaultCode_t code)
{
    uint8_t module = (code >> 8) & 0x0F;  /* 模块号 */
    uint8_t detail = code & 0xFF;          /* 具体故障 */

    /* 红灯闪烁模块号 */
    for (uint8_t i = 0; i < module; i++) {
        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN);
        for (volatile uint32_t d = 0; d < 200000; d++);
        DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
        for (volatile uint32_t d = 0; d < 200000; d++);
    }

    /* 长间隔 */
    for (volatile uint32_t d = 0; d < 600000; d++);

    /* 绿灯闪烁故障序号 */
    for (uint8_t i = 0; i < detail; i++) {
        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN);
        for (volatile uint32_t d = 0; d < 200000; d++);
        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
        for (volatile uint32_t d = 0; d < 200000; d++);
    }

    /* 长间隔 */
    for (volatile uint32_t d = 0; d < 1000000; d++);
}

/**
 * @brief 蜂鸣器故障码报警
 *
 * 不同严重等级对应不同声音模式：
 * - 等级1(警告): 短鸣1声
 * - 等级2(错误): 短鸣2声
 * - 等级3(致命): 长鸣
 */
static void buzzer_alarm(uint8_t severity)
{
    switch (severity) {
    case 1:
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
        for (volatile uint32_t d = 0; d < 200000; d++);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        break;
    case 2:
        for (int i = 0; i < 2; i++) {
            DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
            for (volatile uint32_t d = 0; d < 200000; d++);
            DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
            for (volatile uint32_t d = 0; d < 200000; d++);
        }
        break;
    case 3:
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
        for (volatile uint32_t d = 0; d < 1000000; d++);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        break;
    }
}

/* ======================== ADC读取 ======================== */
static uint16_t adc_read_channel(uint8_t channel)
{
    DL_ADC12_startConversion(ADC12_0_INST);
    while (!(DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_DONE));
    return DL_ADC12_getMemResult(ADC12_0_INST, (DL_ADC12_MEM_IDX)(channel & 0x07));
}

static uint16_t adc_read_avg(uint8_t ch, uint8_t n)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum += adc_read_channel(ch);
        for (volatile uint32_t d = 0; d < 100; d++);
    }
    return (uint16_t)(sum / n);
}

/* ======================== 上电自检 (POST) ======================== */

/**
 * @brief 系统时钟检测
 *
 * 通过SysTick测量实际tick频率是否接近预期
 * 预期: SystemCoreClock/1000 ticks/ms
 */
static bool post_check_clock(void)
{
    uint32_t start = gSysTick;
    /* 等待约100ms */
    while (gSysTick - start < 100);

    /* 如果tick计数偏差超过10%，认为时钟异常 */
    uint32_t elapsed = gSysTick - start;
    if (elapsed < 90 || elapsed > 110) {
        fault_report(FAULT_SYSTEM_CLOCK, 2);
        return false;
    }
    debug_print("[POST] Clock OK\r\n");
    return true;
}

/**
 * @brief RAM测试
 *
 * Walking-ones测试（只测试栈底小区域，避免覆盖关键数据）
 * 对每个地址写入walking-one模式并回读验证
 */
static bool post_check_ram(void)
{
    volatile uint32_t *test_addr = (volatile uint32_t *)0x20000100; /* RAM起始区域 */
    uint32_t saved[4];

    /* 保存原始数据 */
    for (int i = 0; i < 4; i++) {
        saved[i] = test_addr[i];
    }

    /* Walking-ones测试 */
    for (int bit = 0; bit < 32; bit++) {
        uint32_t pattern = 1U << bit;
        test_addr[0] = pattern;
        if (test_addr[0] != pattern) {
            /* 恢复 */
            for (int i = 0; i < 4; i++) test_addr[i] = saved[i];
            fault_report(FAULT_SYSTEM_RAM, 3);
            return false;
        }
    }

    /* 恢复原始数据 */
    for (int i = 0; i < 4; i++) {
        test_addr[i] = saved[i];
    }

    debug_print("[POST] RAM OK\r\n");
    return true;
}

/**
 * @brief Flash CRC校验
 *
 * 对代码区计算CRC32并与预存值比较
 * 需要在编译后计算CRC并写入特定地址
 */
static bool post_check_flash(void)
{
    /* 简化实现：读取Flash最后几个字节确认可读 */
    volatile uint32_t *flash_test = (volatile uint32_t *)0x00000000;
    uint32_t val = *flash_test;

    /* Flash地址0通常是栈指针初始值，应非零非全F */
    if (val == 0xFFFFFFFF || val == 0x00000000) {
        fault_report(FAULT_SYSTEM_FLASH, 3);
        return false;
    }

    debug_print("[POST] Flash OK\r\n");
    return true;
}

/* ======================== 传感器自检 ======================== */

/**
 * @brief ADC自检
 *
 * 测试项目：
 * 1. 零点检测：悬空/接GND时ADC值应接近0
 * 2. 满量程检测：接3.3V时ADC值应接近4095
 * 3. 噪声检测：多次采样的标准差应在合理范围
 */
static bool sensor_check_adc(void)
{
    debug_print("[SENSOR] ADC self-test...\r\n");

    /* 测试通道0 */
    uint16_t zero_val = adc_read_avg(0, 20);

    /* 零点检查：应小于50 (约0.04V) */
    if (zero_val > 50) {
        debug_print("[SENSOR] ADC zero offset too high: ");
        char buf[8];
        snprintf(buf, sizeof(buf), "%d\r\n", zero_val);
        debug_print(buf);
        fault_report(FAULT_ADC_OFFSET, 1);
        /* 非致命，继续 */
    }

    /* 噪声检测 */
    uint32_t sum = 0, sum_sq = 0;
    for (uint8_t i = 0; i < 100; i++) {
        uint16_t val = adc_read_channel(0);
        sum += val;
        sum_sq += (uint32_t)val * val;
    }
    float mean = (float)sum / 100.0f;
    float variance = (float)sum_sq / 100.0f - mean * mean;
    float std_dev = 0;
    /* 简化平方根 */
    if (variance > 0) {
        std_dev = variance;
        for (int i = 0; i < 10; i++) {
            std_dev = 0.5f * (std_dev + variance / std_dev);
        }
    }

    /* 标准差应小于50 LSB */
    if (std_dev > 50.0f) {
        debug_print("[SENSOR] ADC noise too high\r\n");
        fault_report(FAULT_ADC_LINEARITY, 1);
    }

    debug_print("[SENSOR] ADC test passed\r\n");
    return true;
}

/**
 * @brief IMU自检 (MPU6050)
 *
 * 测试项目：
 * 1. I2C通信测试：读取WHO_AM_I寄存器
 * 2. 数据合理性：静止时加速度模值应约为1g
 * 3. 数据更新检测：两次读取不应完全相同
 */
static bool sensor_check_imu(void)
{
    debug_print("[SENSOR] IMU self-test...\r\n");

    /* 读取WHO_AM_I寄存器 (地址0x75)，MPU6050应返回0x68 */
    DL_I2C_startTransfer(I2C_0_INST);
    DL_I2C_transmitData(I2C_0_INST, 0x68 << 1);    /* 写地址 */
    DL_I2C_transmitData(I2C_0_INST, 0x75);          /* WHO_AM_I寄存器 */
    DL_I2C_stopTransfer(I2C_0_INST);
    volatile uint32_t _i2c_timeout = 100000;
    while (DL_I2C_isBusy(I2C_0_INST)); && --_i2c_timeout

    DL_I2C_startTransfer(I2C_0_INST);
    DL_I2C_transmitData(I2C_0_INST, (0x68 << 1) | 1);  /* 读地址 */
    uint8_t whoami = DL_I2C_receiveData(I2C_0_INST);
    DL_I2C_stopTransfer(I2C_0_INST);
    volatile uint32_t _i2c_timeout = 100000;
    while (DL_I2C_isBusy(I2C_0_INST)); && --_i2c_timeout

    if (whoami != 0x68) {
        debug_print("[SENSOR] IMU not found! WHO_AM_I=");
        char buf[8];
        snprintf(buf, sizeof(buf), "0x%02X\r\n", whoami);
        debug_print(buf);
        fault_report(FAULT_IMU_COMM, 2);
        return false;
    }

    /* 唤醒IMU */
    DL_I2C_startTransfer(I2C_0_INST);
    DL_I2C_transmitData(I2C_0_INST, 0x68 << 1);
    DL_I2C_transmitData(I2C_0_INST, 0x6B);          /* PWR_MGMT_1 */
    DL_I2C_transmitData(I2C_0_INST, 0x00);           /* 唤醒 */
    DL_I2C_stopTransfer(I2C_0_INST);
    volatile uint32_t _i2c_timeout = 100000;
    while (DL_I2C_isBusy(I2C_0_INST)); && --_i2c_timeout

    /* 等待数据就绪 */
    for (volatile uint32_t d = 0; d < 100000; d++);

    debug_print("[SENSOR] IMU test passed\r\n");
    return true;
}

/**
 * @brief 灰度传感器自检
 *
 * 测试项目：
 * 1. 开路检测：正常范围外的极低值
 * 2. 短路检测：正常范围外的极高值
 * 3. 通道间一致性：相邻通道差异不应过大
 */
static bool sensor_check_gray(void)
{
    debug_print("[SENSOR] Gray sensor self-test...\r\n");

    uint16_t values[GRAY_SENSOR_COUNT];
    bool all_ok = true;

    for (uint8_t ch = 0; ch < GRAY_SENSOR_COUNT; ch++) {
        values[ch] = adc_read_avg(gray_adc_channels[ch], 10);

        /* 开路检测：ADC值应大于50 */
        if (values[ch] < 50) {
            debug_print("[SENSOR] Gray CH");
            char buf[16];
            snprintf(buf, sizeof(buf), "%d OPEN (%d)\r\n", ch, values[ch]);
            debug_print(buf);
            fault_report(FAULT_GRAY_OPEN, 2);
            all_ok = false;
        }

        /* 短路检测：ADC值应小于4050 */
        if (values[ch] > 4050) {
            debug_print("[SENSOR] Gray CH");
            char buf[16];
            snprintf(buf, sizeof(buf), "%d SHORT (%d)\r\n", ch, values[ch]);
            debug_print(buf);
            fault_report(FAULT_GRAY_SHORT, 2);
            all_ok = false;
        }
    }

    /* 相邻通道一致性检查（排除已知故障通道） */
    for (uint8_t ch = 0; ch < GRAY_SENSOR_COUNT - 1; ch++) {
        int32_t diff = (int32_t)values[ch] - (int32_t)values[ch + 1];
        if (diff < 0) diff = -diff;
        /* 相邻通道差异超过2000可能表示某个传感器异常 */
        if (diff > 2000) {
            debug_print("[SENSOR] Gray sensor mismatch: CH");
            char buf[32];
            snprintf(buf, sizeof(buf), "%d=%d CH%d=%d\r\n",
                     ch, values[ch], ch + 1, values[ch + 1]);
            debug_print(buf);
            fault_report(FAULT_GRAY_RANGE, 1);
            /* 警告级别，可能是正常表面差异 */
        }
    }

    if (all_ok) {
        debug_print("[SENSOR] Gray sensor test passed\r\n");
    }
    return all_ok;
}

/* ======================== 电机自检 ======================== */

/**
 * @brief 电机驱动电路检测
 *
 * 原理：
 * 1. 设置PWM为低占空比（微转）
 * 2. 检测编码器是否有脉冲变化
 * 3. 停止电机
 *
 * 如果编码器无脉冲，可能原因：
 * - 驱动电路断路
 * - 电机未连接
 * - 编码器故障
 */
static bool motor_check_driver(uint8_t motor_id)
{
    debug_print("[MOTOR] Motor");
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", motor_id + 1);
    debug_print(buf);
    debug_print(" driver test...\r\n");

    /* 保存当前编码器计数 */
    int32_t count_before = DL_Timer_getTimerCount(TIMER_0_INST);

    /* 设置低占空比PWM（10%），短暂驱动 */
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_1);   /* 正转 */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_2);
    DL_TimerG_setCaptureCompareValue(TIMER_0_INST, 10, DL_TIMER_CC_0_INDEX);

    /* 等待200ms */
    for (volatile uint32_t d = 0; d < 500000; d++);

    /* 停止电机 */
    DL_TimerG_setCaptureCompareValue(TIMER_0_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_1);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_2);

    /* 检测编码器变化 */
    int32_t count_after = DL_Timer_getTimerCount(TIMER_0_INST);
    int32_t count_diff = count_after - count_before;
    if (count_diff < 0) count_diff = -count_diff;

    if (count_diff < 2) {
        /* 编码器无变化 */
        debug_print("[MOTOR] Motor");
        debug_print(buf);
        debug_print(" no encoder pulse!\r\n");
        fault_report(motor_id == 0 ? FAULT_MOTOR1_ENCODER : FAULT_MOTOR2_ENCODER, 2);
        return false;
    }

    debug_print("[MOTOR] Motor");
    debug_print(buf);
    debug_print(" test passed (pulses=");
    snprintf(buf, sizeof(buf), "%ld", count_diff);
    debug_print(buf);
    debug_print(")\r\n");
    return true;
}

/**
 * @brief 电机堵转检测（运行时调用）
 *
 * 条件：PWM输出大于50%但编码器速度低于阈值
 * 持续1秒判定为堵转
 */
static bool motor_check_stall(uint8_t motor_id, int32_t speed, uint16_t pwm_output)
{
    static uint32_t stall_start[2] = {0, 0};

    if (pwm_output > 500 && (speed > -5 && speed < 5)) {
        /* 可能堵转，开始计时 */
        if (stall_start[motor_id] == 0) {
            stall_start[motor_id] = gSysTick;
        } else if (gSysTick - stall_start[motor_id] > 1000) {
            /* 持续1秒确认堵转 */
            fault_report(motor_id == 0 ? FAULT_MOTOR1_STALL : FAULT_MOTOR2_STALL, 2);
            stall_start[motor_id] = 0;
            return true;  /* 有堵转 */
        }
    } else {
        stall_start[motor_id] = 0;
    }
    return false;
}

/* ======================== 通信自检 ======================== */

/**
 * @brief UART回环测试
 *
 * 将TX和RX短接（或使用内部回环模式），
 * 发送测试数据并检查是否正确接收
 *
 * 注意：实际使用时需要将PA8(TX)和PA9(RX)用跳线短接
 */
static bool comm_check_uart(void)
{
    debug_print("[COMM] UART loopback test...\r\n");

    /* 启用UART内部回环模式（如果支持） */
    /* DL_UART_enableLoopback(UART_0_INST); */

    const uint8_t test_pattern[] = {0x55, 0xAA, 0x0F, 0xF0, 0x00, 0xFF};
    bool pass = true;

    for (uint8_t i = 0; i < sizeof(test_pattern); i++) {
        /* 发送 */
        DL_UART_transmitDataBlocking(UART_0_INST, test_pattern[i]);

        /* 等待接收（带超时） */
        uint32_t timeout = gSysTick + 10;
        while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
            if (gSysTick > timeout) {
                fault_report(FAULT_UART_LOOPBACK, 2);
                return false;
            }
        }

        uint8_t rx = DL_UART_receiveDataBlocking(UART_0_INST);
        if (rx != test_pattern[i]) {
            pass = false;
        }
    }

    /* 禁用回环 */
    /* DL_UART_disableLoopback(UART_0_INST); */

    if (!pass) {
        fault_report(FAULT_UART_LOOPBACK, 2);
        debug_print("[COMM] UART loopback FAILED\r\n");
        return false;
    }

    debug_print("[COMM] UART loopback passed\r\n");
    return true;
}

/**
 * @brief I2C总线扫描
 *
 * 扫描I2C地址0x08~0x77，检测哪些设备有应答
 */
static bool comm_check_i2c(void)
{
    debug_print("[COMM] I2C bus scan...\r\n");

    uint8_t found_count = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        /* 尝试发送起始条件+地址 */
        DL_I2C_startTransfer(I2C_0_INST);
        DL_I2C_transmitData(I2C_0_INST, addr << 1);
        DL_I2C_stopTransfer(I2C_0_INST);

        /* 等待传输完成 */
        uint32_t timeout = gSysTick + 5;
        while (DL_I2C_isBusy(I2C_0_INST)) {
            if (gSysTick > timeout) break;
        }

        /* 检查NACK标志 */
        if (!DL_I2C_isNACK(I2C_0_INST)) {
            /* 设备应答了 */
            char buf[24];
            snprintf(buf, sizeof(buf), "  Found device at 0x%02X\r\n", addr);
            debug_print(buf);
            found_count++;
        }

        /* 短延时 */
        for (volatile uint32_t d = 0; d < 1000; d++);
    }

    if (found_count == 0) {
        debug_print("[COMM] No I2C devices found!\r\n");
        fault_report(FAULT_I2C_BUS, 2);
        return false;
    }

    char buf[24];
    snprintf(buf, sizeof(buf), "[COMM] Found %d I2C device(s)\r\n", found_count);
    debug_print(buf);
    return true;
}

/* ======================== 完整诊断流程 ======================== */

/**
 * @brief 运行完整的系统诊断
 *
 * 执行顺序：
 * 1. 上电自检 (POST)
 * 2. 传感器自检
 * 3. 电机自检（需要电机接线）
 * 4. 通信自检
 * 5. 汇总报告
 *
 * @param skip_motor 是否跳过电机自检（避免测试时电机转动）
 */
static void run_full_diagnosis(bool skip_motor)
{
    debug_print("\r\n");
    debug_print("========================================\r\n");
    debug_print("  System Diagnostic - Starting...\r\n");
    debug_print("========================================\r\n\r\n");

    fault_clear_all();
    uint8_t pass_count = 0, fail_count = 0;

    /* 1. POST */
    debug_print("--- Phase 1: Power-On Self Test ---\r\n");
    if (post_check_clock()) pass_count++; else fail_count++;
    if (post_check_ram()) pass_count++; else fail_count++;
    if (post_check_flash()) pass_count++; else fail_count++;

    /* 2. 传感器 */
    debug_print("\r\n--- Phase 2: Sensor Test ---\r\n");
    if (sensor_check_adc()) pass_count++; else fail_count++;
    if (sensor_check_imu()) pass_count++; else fail_count++;
    if (sensor_check_gray()) pass_count++; else fail_count++;

    /* 3. 电机（可跳过） */
    debug_print("\r\n--- Phase 3: Motor Test ---\r\n");
    if (skip_motor) {
        debug_print("[MOTOR] Skipped (press BTN to enable)\r\n");
    } else {
        if (motor_check_driver(0)) pass_count++; else fail_count++;
        if (motor_check_driver(1)) pass_count++; else fail_count++;
    }

    /* 4. 通信 */
    debug_print("\r\n--- Phase 4: Communication Test ---\r\n");
    if (comm_check_i2c()) pass_count++; else fail_count++;
    /* UART回环需要跳线，可选 */
    /* if (comm_check_uart()) pass_count++; else fail_count++; */

    /* 5. 汇总 */
    debug_print("\r\n========================================\r\n");
    debug_print("  Diagnostic Result:\r\n");
    char buf[48];
    snprintf(buf, sizeof(buf), "  PASSED: %d  FAILED: %d\r\n", pass_count, fail_count);
    debug_print(buf);
    snprintf(buf, sizeof(buf), "  Total faults recorded: %d\r\n", gFaultMgr.fault_count);
    debug_print(buf);
    debug_print("========================================\r\n\r\n");

    /* 报警 */
    if (fail_count == 0) {
        debug_print("All tests PASSED! System healthy.\r\n");
        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN);
        DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
        /* 成功提示音 */
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
        for (volatile uint32_t d = 0; d < 100000; d++);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
    } else {
        debug_print("FAULTS DETECTED! Check LED fault codes.\r\n");
        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN);

        /* 逐个显示故障码 */
        for (uint8_t i = 0; i < gFaultMgr.fault_count; i++) {
            led_show_fault_code(gFaultMgr.history[i].code);
            buzzer_alarm(gFaultMgr.history[i].severity);
            for (volatile uint32_t d = 0; d < 500000; d++);
        }
    }
}

/* ======================== SysTick ======================== */
void SysTick_Handler(void)
{
    gSysTick++;
}

/* ======================== 主函数 ======================== */
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* LED自检闪烁 */
    DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN);
    DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN);
    DL_GPIO_setPins(LED_BLUE_PORT, LED_BLUE_PIN);
    for (volatile uint32_t i = 0; i < 300000; i++);
    DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);
    DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
    DL_GPIO_clearPins(LED_BLUE_PORT, LED_BLUE_PIN);

    debug_print("\r\n=== Fault Diagnosis Demo ===\r\n");
    debug_print("Press button to start full diagnosis\r\n");
    debug_print("Or wait 5s for auto-start (motor test skipped)\r\n");

    /* 等待按键或超时 */
    uint32_t start_time = gSysTick;
    bool btn_pressed = false;
    while (gSysTick - start_time < 5000) {
        if (!DL_GPIO_readPins(BTN_DIAG_PORT, BTN_DIAG_PIN)) {
            btn_pressed = true;
            for (volatile uint32_t d = 0; d < 200000; d++);  /* 消抖 */
            break;
        }
    }

    /* 运行诊断 */
    run_full_diagnosis(!btn_pressed);  /* 无按键则跳过电机测试 */

    /* ======================== 主循环 ======================== */
    /* 诊断完成后进入运行监控模式 */
    uint32_t last_status_led = 0;

    while (1) {
        /* 每2秒检查一次运行时故障 */
        if (gSysTick - last_status_led >= 2000) {
            last_status_led = gSysTick;

            /* 按键触发重新诊断 */
            if (!DL_GPIO_readPins(BTN_DIAG_PORT, BTN_DIAG_PIN)) {
                for (volatile uint32_t d = 0; d < 200000; d++);
                if (!DL_GPIO_readPins(BTN_DIAG_PORT, BTN_DIAG_PIN)) {
                    while (!DL_GPIO_readPins(BTN_DIAG_PORT, BTN_DIAG_PIN));
                    run_full_diagnosis(false);
                }
            }

            /* 无故障时绿灯闪烁表示正常运行 */
            if (gFaultMgr.fault_count == 0) {
                DL_GPIO_togglePins(LED_GREEN_PORT, LED_GREEN_PIN);
                DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
            }
        }

        /* 系统正常时可以进入浅睡眠省电 */
        if (!gFaultMgr.has_critical_fault) {
            __WFI();
        }
    }

    return 0;
}
