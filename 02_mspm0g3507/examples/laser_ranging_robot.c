/**
 * @file laser_ranging_robot.c
 * @brief 激光测距机器人 - VL53L0X + 避障 + 路径规划
 * @platform MSPM0G3507
 * @date 2026-06-12
 *
 * 功能概述：
 *   1. VL53L0X激光测距传感器获取前方距离
 *   2. 基于阈值的避障决策（左转/右转/后退）
 *   3. 简单路径规划：沿墙行走 + 避障模式切换
 *   4. 双电机差速驱动（PWM控制）
 *   5. 状态机管理机器人行为
 *
 * 硬件连接：
 *   VL53L0X: I2C0 (PA10-SCL, PA11-SDA, XSHUT=PA15)
 *   左电机:  PWM0_CH0 (PA0-EN, PA1-INA, PA2-INB)
 *   右电机:  PWM0_CH1 (PA3-EN, PA4-INA, PA5-INB)
 *   前方LED: PB0 (指示灯)
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== 硬件引脚定义 ===== */
#define VL53L0X_XSHUT_PORT     GPIOA
#define VL53L0X_XSHUT_PIN      DL_GPIO_PIN_15

#define LED_FRONT_PORT          GPIOB
#define LED_FRONT_PIN           DL_GPIO_PIN_0

#define MOTOR_L_EN_PORT         GPIOA
#define MOTOR_L_EN_PIN          DL_GPIO_PIN_0
#define MOTOR_L_INA_PORT        GPIOA
#define MOTOR_L_INA_PIN         DL_GPIO_PIN_1
#define MOTOR_L_INB_PORT        GPIOA
#define MOTOR_L_INB_PIN         DL_GPIO_PIN_2

#define MOTOR_R_EN_PORT         GPIOA
#define MOTOR_R_EN_PIN          DL_GPIO_PIN_3
#define MOTOR_R_INA_PORT        GPIOA
#define MOTOR_R_INA_PIN         DL_GPIO_PIN_4
#define MOTOR_R_INB_PORT        GPIOA
#define MOTOR_R_INB_PIN         DL_GPIO_PIN_5

/* ===== 参数配置 ===== */
#define OBSTACLE_DIST_MM        200     /* 避障距离阈值(mm) */
#define WALL_FOLLOW_DIST_MM     150     /* 沿墙行走目标距离(mm) */
#define DIST_TOO_CLOSE_MM       100     /* 太近阈值 */
#define SPEED_CRUISE            60      /* 巡航速度(0-100) */
#define SPEED_TURN              40      /* 转弯速度 */
#define SPEED_SLOW              30      /* 慢速 */
#define LOOP_INTERVAL_MS        50      /* 主循环间隔(ms) */

/* ===== VL53L0X寄存器定义 ===== */
#define VL53L0X_ADDR            0x29
#define VL53L0X_REG_WHO_AM_I    0xC0
#define VL53L0X_WHO_AM_I_VAL    0xEE

/* ===== 机器人状态枚举 ===== */
typedef enum {
    ROBOT_IDLE,             /* 待机 */
    ROBOT_CRUISE,           /* 巡航前进 */
    ROBOT_OBSTACLE_AVOID,   /* 避障模式 */
    ROBOT_WALL_FOLLOW,      /* 沿墙行走 */
    ROBOT_TURN_LEFT,        /* 左转 */
    ROBOT_TURN_RIGHT,       /* 右转 */
    ROBOT_BACKUP,           /* 后退 */
    ROBOT_STOP              /* 停止 */
} RobotState_t;

/* ===== 全局变量 ===== */
static volatile uint32_t g_systick_count = 0;
static RobotState_t g_robot_state = ROBOT_IDLE;
static uint16_t g_distance_mm = 0;          /* 当前测距值 */
static uint16_t g_last_distances[4] = {0};  /* 历史距离缓存 */
static uint8_t g_dist_index = 0;
static bool g_obstacle_detected = false;
static uint32_t g_turn_timer = 0;           /* 转弯计时 */
static uint32_t g_backup_timer = 0;         /* 后退计时 */
static int8_t g_last_turn_dir = 1;          /* 上次转弯方向(1=右,-1=左) */

/* ===== SysTick延时 ===== */
void SysTick_Handler(void) {
    g_systick_count++;
}

static void delay_ms(uint32_t ms) {
    uint32_t start = g_systick_count;
    while ((g_systick_count - start) < ms);
}

static uint32_t get_tick(void) {
    return g_systick_count;
}

/* ===== I2C基础通信 ===== */
static void i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
}

static uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg) {
    uint8_t val;
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);

    DL_I2C_startControllerTransfer(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    val = DL_I2C_receiveControllerData(I2C_0_INST);
    return val;
}

static void i2c_read_multi(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);

    DL_I2C_startControllerTransfer(I2C_0_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
}

/* ===== VL53L0X驱动 ===== */
static void vl53l0x_reset(void) {
    /* XSHUT拉低复位 */
    DL_GPIO_clearPins(VL53L0X_XSHUT_PORT, VL53L0X_XSHUT_PIN);
    delay_ms(20);
    /* XSHUT拉高释放 */
    DL_GPIO_setPins(VL53L0X_XSHUT_PORT, VL53L0X_XSHUT_PIN);
    delay_ms(20);
}

static bool vl53l0x_init(void) {
    uint8_t id;

    vl53l0x_reset();

    /* 读取设备ID */
    id = i2c_read_reg(VL53L0X_ADDR, VL53L0X_REG_WHO_AM_I);
    if (id != VL53L0X_WHO_AM_I_VAL) {
        return false;
    }

    /* VL53L0X初始化序列(简化版，完整版需加载校准数据) */
    /* 设置I2C标准模式 */
    i2c_write_reg(VL53L0X_ADDR, 0x88, 0x00);
    i2c_write_reg(VL53L0X_ADDR, 0x80, 0x01);
    i2c_write_reg(VL53L0X_ADDR, 0xFF, 0x01);
    i2c_write_reg(VL53L0X_ADDR, 0x00, 0x00);

    /* 停止已有的测量 */
    i2c_write_reg(VL53L0X_ADDR, 0x91, 0x00);

    /* 配置测量模式：标准模式 */
    i2c_write_reg(VL53L0X_ADDR, 0xFF, 0x01);
    i2c_write_reg(VL53L0X_ADDR, 0x00, 0x00);
    i2c_write_reg(VL53L0X_ADDR, 0x09, 0x00);
    i2c_write_reg(VL53L0X_ADDR, 0x01, 0x00);
    i2c_write_reg(VL53L0X_ADDR, 0xFF, 0x00);
    i2c_write_reg(VL53L0X_ADDR, 0x80, 0x00);

    /* 设置测量时间预算(33ms标准模式) */
    i2c_write_reg(VL53L0X_ADDR, 0x01, 0x02); /* SYSRANGE_START: 单次测量 */

    delay_ms(10);
    return true;
}

static uint16_t vl53l0x_read_distance(void) {
    uint8_t buf[12];
    uint16_t distance;

    /* 启动单次测量 */
    i2c_write_reg(VL53L0X_ADDR, 0x00, 0x01);

    /* 等待测量完成(超时50ms) */
    uint32_t timeout = get_tick() + 50;
    while (get_tick() < timeout) {
        uint8_t status = i2c_read_reg(VL53L0X_ADDR, 0x13);
        if (status & 0x01) break;
    }

    /* 读取结果寄存器 */
    i2c_read_multi(VL53L0X_ADDR, 0x14, buf, 12);

    /* 距离在结果寄存器的第10-11字节 */
    distance = ((uint16_t)buf[10] << 8) | buf[11];

    /* 读取Range Status */
    uint8_t range_status = buf[0];
    if (range_status != 0x00) {
        distance = 8190; /* 无效读取返回最大值 */
    }

    /* 清除中断 */
    i2c_write_reg(VL53L0X_ADDR, 0x0B, 0x01);

    return distance;
}

/* 更新历史距离缓存并计算滤波值 */
static uint16_t filter_distance(uint16_t new_dist) {
    g_last_distances[g_dist_index] = new_dist;
    g_dist_index = (g_dist_index + 1) & 0x03;

    /* 中值滤波(取4个样本的中值) */
    uint16_t sorted[4];
    for (int i = 0; i < 4; i++) sorted[i] = g_last_distances[i];
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (sorted[i] > sorted[j]) {
                uint16_t tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }
    return (sorted[1] + sorted[2]) / 2;
}

/* ===== 电机控制 ===== */
static void motor_set_left(int8_t speed) {
    /* speed: -100 ~ +100, 正=前进 */
    if (speed > 0) {
        DL_GPIO_setPins(MOTOR_L_INA_PORT, MOTOR_L_INA_PIN);
        DL_GPIO_clearPins(MOTOR_L_INB_PORT, MOTOR_L_INB_PIN);
    } else if (speed < 0) {
        DL_GPIO_clearPins(MOTOR_L_INA_PORT, MOTOR_L_INA_PIN);
        DL_GPIO_setPins(MOTOR_L_INB_PORT, MOTOR_L_INB_PIN);
        speed = -speed;
    } else {
        DL_GPIO_clearPins(MOTOR_L_INA_PORT, MOTOR_L_INA_PIN);
        DL_GPIO_clearPins(MOTOR_L_INB_PORT, MOTOR_L_INB_PIN);
    }
    /* 设置PWM占空比 (假定Timer_0, CC0) */
    uint32_t duty = (uint32_t)speed * DL_TimerG_getLoadValue(TIMER_0_INST) / 100;
    DL_TimerG_setCaptureCompareValue(TIMER_0_INST, duty, DL_TIMER_CC_0_INDEX);
}

static void motor_set_right(int8_t speed) {
    if (speed > 0) {
        DL_GPIO_setPins(MOTOR_R_INA_PORT, MOTOR_R_INA_PIN);
        DL_GPIO_clearPins(MOTOR_R_INB_PORT, MOTOR_R_INB_PIN);
    } else if (speed < 0) {
        DL_GPIO_clearPins(MOTOR_R_INA_PORT, MOTOR_R_INA_PIN);
        DL_GPIO_setPins(MOTOR_R_INB_PORT, MOTOR_R_INB_PIN);
        speed = -speed;
    } else {
        DL_GPIO_clearPins(MOTOR_R_INA_PORT, MOTOR_R_INA_PIN);
        DL_GPIO_clearPins(MOTOR_R_INB_PORT, MOTOR_R_INB_PIN);
    }
    uint32_t duty = (uint32_t)speed * DL_TimerG_getLoadValue(TIMER_0_INST) / 100;
    DL_TimerG_setCaptureCompareValue(TIMER_0_INST, duty, DL_TIMER_CC_1_INDEX);
}

/* 组合运动控制 */
static void robot_forward(uint8_t speed) {
    motor_set_left(speed);
    motor_set_right(speed);
}

static void robot_backward(uint8_t speed) {
    motor_set_left(-speed);
    motor_set_right(-speed);
}

static void robot_turn_left(uint8_t speed) {
    motor_set_left(-speed);
    motor_set_right(speed);
}

static void robot_turn_right(uint8_t speed) {
    motor_set_left(speed);
    motor_set_right(-speed);
}

static void robot_stop(void) {
    motor_set_left(0);
    motor_set_right(0);
}

/* ===== 状态机处理 ===== */
static void robot_update_state(void) {
    /* 读取距离 */
    uint16_t raw_dist = vl53l0x_read_distance();
    g_distance_mm = filter_distance(raw_dist);

    /* 避障检测 */
    g_obstacle_detected = (g_distance_mm < OBSTACLE_DIST_MM);

    /* 前方LED指示 */
    if (g_obstacle_detected) {
        DL_GPIO_setPins(LED_FRONT_PORT, LED_FRONT_PIN);
    } else {
        DL_GPIO_clearPins(LED_FRONT_PORT, LED_FRONT_PIN);
    }

    switch (g_robot_state) {
    case ROBOT_IDLE:
        /* 等待启动命令 */
        robot_stop();
        break;

    case ROBOT_CRUISE:
        if (g_obstacle_detected) {
            /* 检测到障碍物，切换到避障模式 */
            g_robot_state = ROBOT_OBSTACLE_AVOID;
            g_last_turn_dir = (g_distance_mm % 2 == 0) ? 1 : -1;
        } else {
            robot_forward(SPEED_CRUISE);
        }
        break;

    case ROBOT_OBSTACLE_AVOID:
        if (g_distance_mm < DIST_TOO_CLOSE_MM) {
            /* 太近，先后退 */
            g_robot_state = ROBOT_BACKUP;
            g_backup_timer = get_tick() + 500;
        } else {
            /* 根据上次方向转弯 */
            if (g_last_turn_dir > 0) {
                g_robot_state = ROBOT_TURN_RIGHT;
            } else {
                g_robot_state = ROBOT_TURN_LEFT;
            }
            g_turn_timer = get_tick() + 300;
        }
        break;

    case ROBOT_TURN_LEFT:
        robot_turn_left(SPEED_TURN);
        if (get_tick() >= g_turn_timer && !g_obstacle_detected) {
            g_robot_state = ROBOT_WALL_FOLLOW;
            g_last_turn_dir = -1;
        }
        break;

    case ROBOT_TURN_RIGHT:
        robot_turn_right(SPEED_TURN);
        if (get_tick() >= g_turn_timer && !g_obstacle_detected) {
            g_robot_state = ROBOT_WALL_FOLLOW;
            g_last_turn_dir = 1;
        }
        break;

    case ROBOT_BACKUP:
        robot_backward(SPEED_SLOW);
        if (get_tick() >= g_backup_timer) {
            g_robot_state = ROBOT_OBSTACLE_AVOID;
        }
        break;

    case ROBOT_WALL_FOLLOW:
        /* 沿墙行走模式 */
        if (g_obstacle_detected) {
            g_robot_state = ROBOT_OBSTACLE_AVOID;
        } else if (g_distance_mm < WALL_FOLLOW_DIST_MM) {
            /* 距墙太近，微调远离 */
            if (g_last_turn_dir > 0) {
                motor_set_left(SPEED_CRUISE);
                motor_set_right(SPEED_SLOW);
            } else {
                motor_set_left(SPEED_SLOW);
                motor_set_right(SPEED_CRUISE);
            }
        } else {
            /* 正常巡航 */
            robot_forward(SPEED_CRUISE);
        }
        break;

    case ROBOT_STOP:
        robot_stop();
        break;
    }
}

/* ===== 主函数 ===== */
int main(void) {
    /* 系统初始化 */
    SYSCFG_DL_init();

    /* SysTick 1ms */
    SysTick_Config(SystemCoreClock / 1000);

    /* 外设初始化 */
    DL_GPIO_initDigitalInput(DL_GPIO_PIN_10); /* I2C SCL上拉 */
    DL_GPIO_initDigitalInput(DL_GPIO_PIN_11); /* I2C SDA上拉 */

    /* XSHUT引脚初始化(输出) */
    DL_GPIO_initDigitalOutput(VL53L0X_XSHUT_PIN);
    DL_GPIO_enableOutput(VL53L0X_XSHUT_PORT, VL53L0X_XSHUT_PIN);

    /* LED初始化 */
    DL_GPIO_initDigitalOutput(LED_FRONT_PIN);
    DL_GPIO_enableOutput(LED_FRONT_PORT, LED_FRONT_PIN);

    /* 启动PWM定时器 */
    DL_TimerG_startCounter(TIMER_0_INST);

    /* 初始化VL53L0X */
    if (!vl53l0x_init()) {
        /* 初始化失败，LED快闪报警 */
        while (1) {
            DL_GPIO_togglePins(LED_FRONT_PORT, LED_FRONT_PIN);
            delay_ms(100);
        }
    }

    /* 启动巡航 */
    g_robot_state = ROBOT_CRUISE;

    /* 主循环 */
    while (1) {
        robot_update_state();
        delay_ms(LOOP_INTERVAL_MS);
    }
}
