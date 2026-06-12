/**
 * @file    pin_config.h
 * @brief   MSPM0G3507 统一引脚分配表
 *
 * 本文件集中定义所有外设的引脚分配，便于：
 *   1. 快速查阅引脚占用情况
 *   2. 检测引脚冲突
 *   3. 切换引脚方案时只需修改此文件
 *
 * 使用方法:
 *   在 SysConfig 中按此文件的分配配置引脚，
 *   或在自定义版本驱动中引用此文件的宏。
 *
 * @note    引脚共存/互斥关系见末尾的共存矩阵
 */

#ifndef __PIN_CONFIG_H
#define __PIN_CONFIG_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ═══════════════════════════════════════════════════════════════
 *  第一部分: I2C 总线配置
 *  所有I2C设备共享 I2C0 (PB2=SCL, PB3=SDA)
 *  通过不同从机地址区分设备
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_I2C0_SCL_PORT       GPIOB
#define PIN_I2C0_SCL_PIN        DL_GPIO_PIN_2
#define PIN_I2C0_SDA_PORT       GPIOB
#define PIN_I2C0_SDA_PIN        DL_GPIO_PIN_3

/* I2C从机地址 */
#define PIN_I2C_ADDR_OLED       0x3C    /* SSD1306 OLED */
#define PIN_I2C_ADDR_TCS34725   0x29    /* TCS34725 颜色传感器 */
#define PIN_I2C_ADDR_PCA9685    0x40    /* PCA9685 舵机驱动板 (可调) */
#define PIN_I2C_ADDR_AT24C02    0x50    /* AT24C02 EEPROM */

/* ═══════════════════════════════════════════════════════════════
 *  第二部分: UART 配置
 *  UART1: PA17=TX, PA18=RX (JY901S / Bluetooth 互斥)
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_UART1_TX_PORT       GPIOA
#define PIN_UART1_TX_PIN        DL_GPIO_PIN_17
#define PIN_UART1_RX_PORT       GPIOA
#define PIN_UART1_RX_PIN        DL_GPIO_PIN_18

/* 蓝牙 EN 引脚 (PA16, 高=AT模式, 低=透传) */
#define PIN_BT_EN_PORT          GPIOA
#define PIN_BT_EN_PIN           DL_GPIO_PIN_16

/* ═══════════════════════════════════════════════════════════════
 *  第三部分: TB6612 电机驱动
 *  方向: PA0=AIN1, PA1=AIN2, PA2=BIN1, PA3=BIN2
 *  PWM:  PA12=PWMA (TIMA0 CH0), PA13=PWMB (TIMA0 CH3)
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_TB6612_PORT         GPIOA
#define PIN_TB6612_AIN1         DL_GPIO_PIN_0
#define PIN_TB6612_AIN2         DL_GPIO_PIN_1
#define PIN_TB6612_BIN1         DL_GPIO_PIN_2
#define PIN_TB6612_BIN2         DL_GPIO_PIN_3
#define PIN_TB6612_PWM_TIM      TIMA0
#define PIN_TB6612_PWM_C0_IDX   GPIO_PWM_0_C0_IDX   /* PA12 CH0 */
#define PIN_TB6612_PWM_C3_IDX   GPIO_PWM_0_C3_IDX   /* PA13 CH3 */

/* ═══════════════════════════════════════════════════════════════
 *  第四部分: L298N 电机驱动
 *  方向: PA4=IN1, PA5=IN2, PA6=IN3, PA7=IN4
 *  PWM:  PA8=ENA (TIMA0 CH0), PA9=ENB (TIMA0 CH1)
 *
 *  @warning 与 TB6612 互斥（同类驱动）
 *  @warning PA8 与 Servo 互斥
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_L298N_PORT          GPIOA
#define PIN_L298N_IN1           DL_GPIO_PIN_4
#define PIN_L298N_IN2           DL_GPIO_PIN_5
#define PIN_L298N_IN3           DL_GPIO_PIN_6
#define PIN_L298N_IN4           DL_GPIO_PIN_7
#define PIN_L298N_PWM_TIM       TIMA0
#define PIN_L298N_PWM_C0_IDX    GPIO_PWM_0_C0_IDX   /* PA8 CH0 */
#define PIN_L298N_PWM_C1_IDX    GPIO_PWM_0_C1_IDX   /* PA9 CH1 */

/* ═══════════════════════════════════════════════════════════════
 *  第五部分: SG90 舵机
 *  信号: PA8 (TIMA0 CH0 PWM)
 *
 *  @warning 与 L298N ENA 互斥（共用 PA8）
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_SERVO_PORT          GPIOA
#define PIN_SERVO_PIN           DL_GPIO_PIN_8
#define PIN_SERVO_TIM           TIMA0
#define PIN_SERVO_C0_IDX        GPIO_SERVO_C0_IDX    /* PA8 CH0 */

/* ═══════════════════════════════════════════════════════════════
 *  第六部分: N20 霍尔编码器
 *  左轮: PB0=E1A (中断), PB1=E1B (输入)
 *  右轮: PB4=E2A (中断), PB5=E2B (输入)
 *
 *  @warning 与 Grayscale 互斥（共用 PB0~PB5）
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_ENC_PORT            GPIOB
#define PIN_ENC_E1A             DL_GPIO_PIN_0
#define PIN_ENC_E1B             DL_GPIO_PIN_1
#define PIN_ENC_E2A             DL_GPIO_PIN_4
#define PIN_ENC_E2B             DL_GPIO_PIN_5

/* ═══════════════════════════════════════════════════════════════
 *  第七部分: SR04 超声波
 *  Trig: PB6, Echo: PB7 (GPIO中断)
 *
 *  @warning 与 Grayscale 互斥（共用 PB6, PB7）
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_ULTRA_PORT          GPIOB
#define PIN_ULTRA_TRIG          DL_GPIO_PIN_6
#define PIN_ULTRA_ECHO          DL_GPIO_PIN_7

/* ═══════════════════════════════════════════════════════════════
 *  第八部分: 感为8路灰度传感器
 *  数字输出: PB0~PB7 = G0~G7
 *
 *  @warning 与 Encoder 互斥（共用 PB0~PB5）
 *  @warning 与 Ultrasonic 互斥（共用 PB6~PB7）
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_GRAY_PORT           GPIOB
#define PIN_GRAY_G0             DL_GPIO_PIN_0
#define PIN_GRAY_G1             DL_GPIO_PIN_1
#define PIN_GRAY_G2             DL_GPIO_PIN_2
#define PIN_GRAY_G3             DL_GPIO_PIN_3
#define PIN_GRAY_G4             DL_GPIO_PIN_4
#define PIN_GRAY_G5             DL_GPIO_PIN_5
#define PIN_GRAY_G6             DL_GPIO_PIN_6
#define PIN_GRAY_G7             DL_GPIO_PIN_7

/* ═══════════════════════════════════════════════════════════════
 *  第九部分: 红外循迹传感器 (自定义版本)
 *  引脚由 IRConfig 结构体运行时指定
 *  默认方案: PA10, PA11, PA24, PA25, PA26, PA27, PA28, PA31
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_IR_DEFAULT_PORT     GPIOA
#define PIN_IR_CH0              DL_GPIO_PIN_10
#define PIN_IR_CH1              DL_GPIO_PIN_11
#define PIN_IR_CH2              DL_GPIO_PIN_24
#define PIN_IR_CH3              DL_GPIO_PIN_25
#define PIN_IR_CH4              DL_GPIO_PIN_26
#define PIN_IR_CH5              DL_GPIO_PIN_27
#define PIN_IR_CH6              DL_GPIO_PIN_28
#define PIN_IR_CH7              DL_GPIO_PIN_31

/* ═══════════════════════════════════════════════════════════════
 *  第十部分: 定时器资源分配
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_TIM_MOTOR           TIMA0       /* 电机PWM (TB6612/L298N/Servo) */
#define PIN_TIM_SYSTEM          TIMG6       /* 系统计时 (1ms中断) */
#define PIN_TIM_ENCODER         TIMG6       /* 编码器采样 (与系统计时共用) */

/* ═══════════════════════════════════════════════════════════════
 *  共存/互斥矩阵速查
 *
 *  [可共存]
 *    I2C设备 (OLED + TCS34725 + PCA9685 + AT24C02) ← 地址不同
 *    TB6612 + Encoder + Bluetooth
 *    TB6612 + Servo + Bluetooth
 *    PCA9685 + 任意GPIO设备 (舵机通过I2C板控制)
 *
 *  [互斥]
 *    TB6612 ↔ L298N          (同类电机驱动)
 *    L298N  ↔ Servo          (共用 PA8)
 *    JY901S ↔ Bluetooth      (共用 UART1)
 *    Grayscale ↔ Encoder     (共用 PB0~PB5)
 *    Grayscale ↔ Ultrasonic  (共用 PB6~PB7)
 * ═══════════════════════════════════════════════════════════════ */

#endif /* __PIN_CONFIG_H */
