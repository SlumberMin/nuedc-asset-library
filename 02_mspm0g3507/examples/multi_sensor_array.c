/**
 * @file multi_sensor_array.c
 * @brief 多传感器阵列 - VL53L0X×3 + 超声波×3 + 融合避障
 * @platform MSPM0G3507
 * @date 2026-06-12
 *
 * 功能概述：
 *   1. 3路VL53L0X激光测距(左/中/右) - I2C地址切换
 *   2. 3路HC-SR04超声波测距(左/中/右) - 定时器捕获
 *   3. 多传感器数据融合(加权平均+置信度)
 *   4. 障碍物地图构建(简单栅格)
 *   5. 融合避障决策输出(方向+速度)
 *
 * 硬件连接：
 *   VL53L0X×3: I2C0, XSHUT分别接PA0/PA1/PA2(用于地址分配)
 *   超声波HC-SR04×3:
 *     左: TRIG=PB0, ECHO=PB1
 *     中: TRIG=PB2, ECHO=PB3
 *     右: TRIG=PB4, ECHO=PB5
 *   方向指示: LED×3 (PA8-左, PA9-中, PA10-右)
 *   串口调试: UART0 (PA16-TX, PA17-RX)
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ===== VL53L0X引脚定义 ===== */
#define VL53L0X_XSHUT_LEFT_PORT     GPIOA
#define VL53L0X_XSHUT_LEFT_PIN      DL_GPIO_PIN_0
#define VL53L0X_XSHUT_CENTER_PORT   GPIOA
#define VL53L0X_XSHUT_CENTER_PIN    DL_GPIO_PIN_1
#define VL53L0X_XSHUT_RIGHT_PORT    GPIOA
#define VL53L0X_XSHUT_RIGHT_PIN     DL_GPIO_PIN_2

/* 超声波引脚 */
#define US_LEFT_TRIG_PORT           GPIOB
#define US_LEFT_TRIG_PIN            DL_GPIO_PIN_0
#define US_LEFT_ECHO_PORT           GPIOB
#define US_LEFT_ECHO_PIN            DL_GPIO_PIN_1

#define US_CENTER_TRIG_PORT         GPIOB
#define US_CENTER_TRIG_PIN          DL_GPIO_PIN_2
#define US_CENTER_ECHO_PORT         GPIOB
#define US_CENTER_ECHO_PIN          DL_GPIO_PIN_3

#define US_RIGHT_TRIG_PORT          GPIOB
#define US_RIGHT_TRIG_PIN           DL_GPIO_PIN_4
#define US_RIGHT_ECHO_PORT          GPIOB
#define US_RIGHT_ECHO_PIN           DL_GPIO_PIN_5

/* LED指示 */
#define LED_LEFT_PORT               GPIOA
#define LED_LEFT_PIN                DL_GPIO_PIN_8
#define LED_CENTER_PORT             GPIOA
#define LED_CENTER_PIN              DL_GPIO_PIN_9
#define LED_RIGHT_PORT              GPIOA
#define LED_RIGHT_PIN               DL_GPIO_PIN_10

/* ===== VL53L0X参数 ===== */
#define VL53L0X_BASE_ADDR           0x29    /* 默认地址 */
#define VL53L0X_ADDR_LEFT           0x30    /* 左传感器新地址 */
#define VL53L0X_ADDR_CENTER         0x31    /* 中传感器新地址 */
#define VL53L0X_ADDR_RIGHT          0x32    /* 右传感器新地址 */
#define VL53L0X_REG_WHO_AM_I        0xC0
#define VL53L0X_WHO_AM_I_VAL        0xEE
#define VL53L0X_REG_I2C_ADDR        0x8A
#define VL53L0X_REG_SYSRANGE_START  0x00
#define VL53L0X_REG_RESULT_STATUS   0x14

/* ===== 传感器数量 ===== */
#define NUM_LASER_SENSORS           3
#define NUM_ULTRASONIC_SENSORS      3
#define NUM_TOTAL_SENSORS           6

/* ===== 融合参数 ===== */
#define LASER_WEIGHT                0.7f        /* 激光传感器权重 */
#define ULTRASONIC_WEIGHT           0.3f        /* 超声波传感器权重 */
#define FUSION_OBSTACLE_THRESH_MM   300         /* 融合后障碍物阈值 */
#define GRID_SIZE                   10          /* 栅格地图大小 */
#define GRID_RESOLUTION_MM          100         /* 每格100mm */

/* ===== 超声波参数 ===== */
#define US_TIMEOUT_US               30000       /* 超时30ms(约5m) */
#define US_SOUND_SPEED_DIV2         0.1715f     /* 声速/2 (mm/us) */
#define US_MAX_MM                   4000        /* 最大测量距离 */

/* ===== 避障决策输出 ===== */
typedef enum {
    DIRECTION_FORWARD,          /* 前进 */
    DIRECTION_FORWARD_LEFT,     /* 前方偏左 */
    DIRECTION_FORWARD_RIGHT,    /* 前方偏右 */
    DIRECTION_TURN_LEFT,        /* 左转 */
    DIRECTION_TURN_RIGHT,       /* 右转 */
    DIRECTION_BACKWARD,         /* 后退 */
    DIRECTION_STOP              /* 停止 */
} Direction_t;

/* ===== 传感器数据结构 ===== */
typedef struct {
    uint16_t distance_mm;       /* 距离(mm) */
    uint8_t confidence;         /* 置信度(0-100) */
    bool valid;                 /* 数据是否有效 */
    uint32_t last_update_ms;    /* 最后更新时间 */
} SensorData_t;

/* ===== 传感器位置索引 ===== */
typedef enum {
    POS_LEFT = 0,
    POS_CENTER = 1,
    POS_RIGHT = 2
} SensorPosition_t;

/* ===== 全局变量 ===== */
static volatile uint32_t g_systick_count = 0;

/* 传感器数据 */
static SensorData_t g_laser[NUM_LASER_SENSORS];
static SensorData_t g_ultrasonic[NUM_ULTRASONIC_SENSORS];
static float g_fused_distance[NUM_LASER_SENSORS]; /* 融合后的距离 */

/* 障碍物栅格地图(简单:每格记录最小距离) */
static uint16_t g_grid_map[GRID_SIZE][GRID_SIZE];

/* 当前避障决策 */
static Direction_t g_direction = DIRECTION_STOP;
static uint8_t g_speed = 0;                    /* 输出速度(0-100) */

/* 串口输出缓冲 */
static char g_uart_buf[128];

/* ===== SysTick ===== */
void SysTick_Handler(void) { g_systick_count++; }

static void delay_ms(uint32_t ms) {
    uint32_t start = g_systick_count;
    while ((g_systick_count - start) < ms);
}

static void delay_us(uint32_t us) {
    SysTick->LOAD = us * (SystemCoreClock / 1000000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    SysTick->CTRL = 0;
}

static uint32_t get_tick(void) { return g_systick_count; }

/* ===== 串口输出 ===== */
static void uart_send_string(const char *str) {
    while (*str) {
        DL_UART_main_transmitData(UART_0_INST, *str++);
        while (!DL_UART_main_isTXFIFOEmpty(UART_0_INST));
    }
}

/* ===== I2C通信 ===== */
static void i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
}

static uint8_t i2c_read_reg(uint8_t dev, uint8_t reg) {
    uint8_t val;
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);

    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    val = DL_I2C_receiveControllerData(I2C_0_INST);
    return val;
}

static void i2c_read_multi(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len) {
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);

    DL_I2C_startControllerTransfer(I2C_0_INST, dev, DL_I2C_CONTROLLER_DIRECTION_RX, len);
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
}

/* ===== VL53L0X XSHUT控制 ===== */
static void vl53l0x_xshut_all_low(void) {
    DL_GPIO_clearPins(VL53L0X_XSHUT_LEFT_PORT, VL53L0X_XSHUT_LEFT_PIN);
    DL_GPIO_clearPins(VL53L0X_XSHUT_CENTER_PORT, VL53L0X_XSHUT_CENTER_PIN);
    DL_GPIO_clearPins(VL53L0X_XSHUT_RIGHT_PORT, VL53L0X_XSHUT_RIGHT_PIN);
}

static void vl53l0x_xshut_set(SensorPosition_t pos, bool state) {
    GPIO_Regs *port;
    uint32_t pin;

    switch (pos) {
    case POS_LEFT:
        port = VL53L0X_XSHUT_LEFT_PORT;
        pin = VL53L0X_XSHUT_LEFT_PIN;
        break;
    case POS_CENTER:
        port = VL53L0X_XSHUT_CENTER_PORT;
        pin = VL53L0X_XSHUT_CENTER_PIN;
        break;
    case POS_RIGHT:
        port = VL53L0X_XSHUT_RIGHT_PORT;
        pin = VL53L0X_XSHUT_RIGHT_PIN;
        break;
    }

    if (state) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static void vl53l0x_set_addr(uint8_t new_addr) {
    /* VL53L0X的I2C地址写入寄存器0x8A */
    i2c_write_reg(VL53L0X_BASE_ADDR, VL53L0X_REG_I2C_ADDR, new_addr);
}

static void vl53l0x_basic_init(uint8_t addr) {
    /* 简化的VL53L0X初始化 */
    i2c_write_reg(addr, 0x88, 0x00);
    i2c_write_reg(addr, 0x80, 0x01);
    i2c_write_reg(addr, 0xFF, 0x01);
    i2c_write_reg(addr, 0x00, 0x00);
    i2c_write_reg(addr, 0x91, 0x00);
    i2c_write_reg(addr, 0xFF, 0x00);
    i2c_write_reg(addr, 0x80, 0x00);
}

/* ===== 多VL53L0X初始化(逐个分配地址) ===== */
static bool multi_vl53l0x_init(void) {
    uint8_t addrs[NUM_LASER_SENSORS] = {
        VL53L0X_ADDR_LEFT, VL53L0X_ADDR_CENTER, VL53L0X_ADDR_RIGHT
    };
    uint8_t id;

    /* 第1步: 全部复位(XSHUT拉低) */
    vl53l0x_xshut_all_low();
    delay_ms(20);

    /* 第2步: 逐个唤醒并分配地址 */
    for (uint8_t i = 0; i < NUM_LASER_SENSORS; i++) {
        /* 唤醒当前传感器 */
        vl53l0x_xshut_set((SensorPosition_t)i, true);
        delay_ms(20);

        /* 验证通信 */
        id = i2c_read_reg(VL53L0X_BASE_ADDR, VL53L0X_REG_WHO_AM_I);
        if (id != VL53L0X_WHO_AM_I_VAL) {
            return false;
        }

        /* 分配新I2C地址 */
        vl53l0x_set_addr(addrs[i]);
        delay_ms(5);

        /* 初始化传感器 */
        vl53l0x_basic_init(addrs[i]);

        g_laser[i].valid = false;
        g_laser[i].confidence = 0;
    }

    return true;
}

/* 读取指定VL53L0X的距离 */
static uint16_t vl53l0x_read_distance(uint8_t addr) {
    uint8_t buf[12];
    uint16_t distance;

    /* 启动单次测量 */
    i2c_write_reg(addr, VL53L0X_REG_SYSRANGE_START, 0x01);

    /* 等待测量完成 */
    uint32_t timeout = get_tick() + 50;
    while (get_tick() < timeout) {
        uint8_t status = i2c_read_reg(addr, 0x13);
        if (status & 0x01) break;
    }

    /* 读取结果 */
    i2c_read_multi(addr, VL53L0X_REG_RESULT_STATUS, buf, 12);
    distance = ((uint16_t)buf[10] << 8) | buf[11];

    /* 验证状态 */
    if (buf[0] != 0x00) {
        distance = 8190;
    }

    /* 清除中断 */
    i2c_write_reg(addr, 0x0B, 0x01);

    return distance;
}

/* ===== HC-SR04超声波驱动 ===== */
/* 触发超声波测量 */
static void ultrasonic_trigger(GPIO_Regs *trig_port, uint32_t trig_pin) {
    DL_GPIO_clearPins(trig_port, trig_pin);
    delay_us(2);
    DL_GPIO_setPins(trig_port, trig_pin);
    delay_us(10);
    DL_GPIO_clearPins(trig_port, trig_pin);
}

/* 读取回波时间(us) */
static uint32_t ultrasonic_read_echo(GPIO_Regs *echo_port, uint32_t echo_pin) {
    /* 等待ECHO变高(超时) */
    uint32_t timeout = get_tick() + 50;
    while (!(echo_port->DIN31_0 & echo_pin)) {
        if (get_tick() >= timeout) return 0;
    }

    /* 记录开始时间 */
    uint32_t start = SysTick->VAL;
    uint32_t start_ms = g_systick_count;

    /* 等待ECHO变低(超时) */
    timeout = get_tick() + 50;
    while (echo_port->DIN31_0 & echo_pin) {
        if (get_tick() >= timeout) return US_TIMEOUT_US;
    }

    /* 计算时间差 */
    uint32_t end = SysTick->VAL;
    uint32_t elapsed_ms = g_systick_count - start_ms;
    uint32_t elapsed_us;

    if (elapsed_ms > 0) {
        elapsed_us = elapsed_ms * 1000;
        elapsed_us += (start - end) / (SystemCoreClock / 1000000);
    } else {
        elapsed_us = (start - end) / (SystemCoreClock / 1000000);
    }

    return elapsed_us;
}

/* 超声波测距(mm) */
static uint16_t ultrasonic_measure(GPIO_Regs *trig_port, uint32_t trig_pin,
                                    GPIO_Regs *echo_port, uint32_t echo_pin) {
    ultrasonic_trigger(trig_port, trig_pin);
    uint32_t echo_us = ultrasonic_read_echo(echo_port, echo_pin);

    if (echo_us == 0 || echo_us >= US_TIMEOUT_US) {
        return 0; /* 无效 */
    }

    uint16_t dist_mm = (uint16_t)(echo_us * US_SOUND_SPEED_DIV2);
    if (dist_mm > US_MAX_MM) dist_mm = US_MAX_MM;
    return dist_mm;
}

/* ===== 传感器数据融合 ===== */
static void fuse_sensor_data(void) {
    for (uint8_t i = 0; i < NUM_LASER_SENSORS; i++) {
        float laser_d = g_laser[i].valid ? g_laser[i].distance_mm : 8190;
        float us_d = g_ultrasonic[i].valid ? g_ultrasonic[i].distance_mm : US_MAX_MM;

        /* 加权平均融合 */
        float w_laser = LASER_WEIGHT;
        float w_us = ULTRASONIC_WEIGHT;

        /* 根据置信度调整权重 */
        if (!g_laser[i].valid) {
            w_laser = 0;
            w_us = 1.0f;
        }
        if (!g_ultrasonic[i].valid) {
            w_us = 0;
            w_laser = 1.0f;
        }

        float total_weight = w_laser + w_us;
        if (total_weight > 0) {
            g_fused_distance[i] = (laser_d * w_laser + us_d * w_us) / total_weight;
        } else {
            g_fused_distance[i] = 8190; /* 都无效时返回最大值 */
        }
    }
}

/* ===== 障碍物地图更新 ===== */
static void update_grid_map(void) {
    /* 简单实现: 前方3列分别对应左/中/右传感器 */
    for (uint8_t col = 0; col < GRID_SIZE; col++) {
        for (uint8_t row = 0; row < GRID_SIZE; row++) {
            g_grid_map[row][col] = 0xFFFF; /* 初始化为未知 */
        }
    }

    /* 根据传感器距离填充栅格 */
    uint16_t dists[3];
    for (uint8_t i = 0; i < 3; i++) {
        dists[i] = (uint16_t)g_fused_distance[i];
    }

    /* 将距离映射到栅格行 */
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t grid_col = 3 + i * 2; /* 左=3, 中=5, 右=7 */
        uint8_t grid_row = dists[i] / GRID_RESOLUTION_MM;
        if (grid_row >= GRID_SIZE) grid_row = GRID_SIZE - 1;

        /* 标记障碍物 */
        g_grid_map[grid_row][grid_col] = dists[i];

        /* 标记障碍物附近区域 */
        if (grid_row > 0) {
            g_grid_map[grid_row - 1][grid_col] = dists[i];
        }
        if (grid_col > 0) {
            g_grid_map[grid_row][grid_col - 1] = dists[i];
        }
        if (grid_col < GRID_SIZE - 1) {
            g_grid_map[grid_row][grid_col + 1] = dists[i];
        }
    }
}

/* ===== 避障决策 ===== */
static void obstacle_decision(void) {
    float left = g_fused_distance[POS_LEFT];
    float center = g_fused_distance[POS_CENTER];
    float right = g_fused_distance[POS_RIGHT];

    bool left_blocked = (left < FUSION_OBSTACLE_THRESH_MM);
    bool center_blocked = (center < FUSION_OBSTACLE_THRESH_MM);
    bool right_blocked = (right < FUSION_OBSTACLE_THRESH_MM);

    /* 决策逻辑 */
    if (!left_blocked && !center_blocked && !right_blocked) {
        /* 全部畅通 */
        g_direction = DIRECTION_FORWARD;
        g_speed = 80;
    } else if (!center_blocked) {
        /* 中间畅通 */
        if (left_blocked && !right_blocked) {
            g_direction = DIRECTION_FORWARD_RIGHT;
            g_speed = 60;
        } else if (!left_blocked && right_blocked) {
            g_direction = DIRECTION_FORWARD_LEFT;
            g_speed = 60;
        } else {
            g_direction = DIRECTION_FORWARD;
            g_speed = 50;
        }
    } else if (center_blocked) {
        /* 中间有障碍 */
        if (!left_blocked && right_blocked) {
            g_direction = DIRECTION_TURN_LEFT;
            g_speed = 40;
        } else if (left_blocked && !right_blocked) {
            g_direction = DIRECTION_TURN_RIGHT;
            g_speed = 40;
        } else if (!left_blocked) {
            /* 左边更宽 */
            g_direction = DIRECTION_TURN_LEFT;
            g_speed = 30;
        } else if (!right_blocked) {
            /* 右边更宽 */
            g_direction = DIRECTION_TURN_RIGHT;
            g_speed = 30;
        } else {
            /* 全部被堵 */
            if (left > right) {
                g_direction = DIRECTION_TURN_LEFT;
                g_speed = 30;
            } else {
                g_direction = DIRECTION_TURN_RIGHT;
                g_speed = 30;
            }
        }
    }

    /* 严重障碍(全部<150mm) */
    if (left < 150 && center < 150 && right < 150) {
        g_direction = DIRECTION_BACKWARD;
        g_speed = 30;
    }

    /* 极近(全部<80mm) */
    if (left < 80 && center < 80 && right < 80) {
        g_direction = DIRECTION_STOP;
        g_speed = 0;
    }
}

/* ===== LED指示更新 ===== */
static void update_leds(void) {
    /* 左LED: 左传感器障碍指示 */
    if (g_fused_distance[POS_LEFT] < FUSION_OBSTACLE_THRESH_MM) {
        DL_GPIO_setPins(LED_LEFT_PORT, LED_LEFT_PIN);
    } else {
        DL_GPIO_clearPins(LED_LEFT_PORT, LED_LEFT_PIN);
    }

    /* 中LED */
    if (g_fused_distance[POS_CENTER] < FUSION_OBSTACLE_THRESH_MM) {
        DL_GPIO_setPins(LED_CENTER_PORT, LED_CENTER_PIN);
    } else {
        DL_GPIO_clearPins(LED_CENTER_PORT, LED_CENTER_PIN);
    }

    /* 右LED */
    if (g_fused_distance[POS_RIGHT] < FUSION_OBSTACLE_THRESH_MM) {
        DL_GPIO_setPins(LED_RIGHT_PORT, LED_RIGHT_PIN);
    } else {
        DL_GPIO_clearPins(LED_RIGHT_PORT, LED_RIGHT_PIN);
    }
}

/* ===== 串口调试输出 ===== */
static void print_debug(void) {
    const char *dir_str[] = {
        "FWD", "FWD_L", "FWD_R", "LEFT", "RIGHT", "BACK", "STOP"
    };

    snprintf(g_uart_buf, sizeof(g_uart_buf),
             "L:%4d C:%4d R:%4d | fL:%4.0f fC:%4.0f fR:%4.0f | %s %d%%\r\n",
             g_laser[POS_LEFT].distance_mm,
             g_laser[POS_CENTER].distance_mm,
             g_laser[POS_RIGHT].distance_mm,
             g_fused_distance[POS_LEFT],
             g_fused_distance[POS_CENTER],
             g_fused_distance[POS_RIGHT],
             dir_str[g_direction],
             g_speed);
    uart_send_string(g_uart_buf);

    snprintf(g_uart_buf, sizeof(g_uart_buf),
             "US L:%4d C:%4d R:%4d\r\n",
             g_ultrasonic[POS_LEFT].distance_mm,
             g_ultrasonic[POS_CENTER].distance_mm,
             g_ultrasonic[POS_RIGHT].distance_mm);
    uart_send_string(g_uart_buf);
}

/* ===== 主函数 ===== */
int main(void) {
    /* 系统初始化 */
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* GPIO初始化 - VL53L0X XSHUT */
    DL_GPIO_initDigitalOutput(VL53L0X_XSHUT_LEFT_PIN);
    DL_GPIO_enableOutput(VL53L0X_XSHUT_LEFT_PORT, VL53L0X_XSHUT_LEFT_PIN);
    DL_GPIO_initDigitalOutput(VL53L0X_XSHUT_CENTER_PIN);
    DL_GPIO_enableOutput(VL53L0X_XSHUT_CENTER_PORT, VL53L0X_XSHUT_CENTER_PIN);
    DL_GPIO_initDigitalOutput(VL53L0X_XSHUT_RIGHT_PIN);
    DL_GPIO_enableOutput(VL53L0X_XSHUT_RIGHT_PORT, VL53L0X_XSHUT_RIGHT_PIN);

    /* GPIO初始化 - 超声波 */
    DL_GPIO_initDigitalOutput(US_LEFT_TRIG_PIN);
    DL_GPIO_enableOutput(US_LEFT_TRIG_PORT, US_LEFT_TRIG_PIN);
    DL_GPIO_initDigitalInput(US_LEFT_ECHO_PIN);
    DL_GPIO_initDigitalOutput(US_CENTER_TRIG_PIN);
    DL_GPIO_enableOutput(US_CENTER_TRIG_PORT, US_CENTER_TRIG_PIN);
    DL_GPIO_initDigitalInput(US_CENTER_ECHO_PIN);
    DL_GPIO_initDigitalOutput(US_RIGHT_TRIG_PIN);
    DL_GPIO_enableOutput(US_RIGHT_TRIG_PORT, US_RIGHT_TRIG_PIN);
    DL_GPIO_initDigitalInput(US_RIGHT_ECHO_PIN);

    /* GPIO初始化 - LED */
    DL_GPIO_initDigitalOutput(LED_LEFT_PIN);
    DL_GPIO_enableOutput(LED_LEFT_PORT, LED_LEFT_PIN);
    DL_GPIO_initDigitalOutput(LED_CENTER_PIN);
    DL_GPIO_enableOutput(LED_CENTER_PORT, LED_CENTER_PIN);
    DL_GPIO_initDigitalOutput(LED_RIGHT_PIN);
    DL_GPIO_enableOutput(LED_RIGHT_PORT, LED_RIGHT_PIN);

    /* 初始化传感器数据 */
    memset(g_laser, 0, sizeof(g_laser));
    memset(g_ultrasonic, 0, sizeof(g_ultrasonic));
    for (uint8_t i = 0; i < NUM_LASER_SENSORS; i++) {
        g_fused_distance[i] = 8190;
    }

    /* 初始化栅格地图 */
    for (uint8_t r = 0; r < GRID_SIZE; r++) {
        for (uint8_t c = 0; c < GRID_SIZE; c++) {
            g_grid_map[r][c] = 0xFFFF;
        }
    }

    /* 多VL53L0X初始化 */
    uart_send_string("Initializing VL53L0X sensors...\r\n");
    if (!multi_vl53l0x_init()) {
        uart_send_string("ERROR: VL53L0X init failed!\r\n");
        /* 错误指示: 全部LED快闪 */
        while (1) {
            DL_GPIO_togglePins(LED_LEFT_PORT, LED_LEFT_PIN);
            DL_GPIO_togglePins(LED_CENTER_PORT, LED_CENTER_PIN);
            DL_GPIO_togglePins(LED_RIGHT_PORT, LED_RIGHT_PIN);
            delay_ms(100);
        }
    }
    uart_send_string("VL53L0X init OK\r\n");

    /* 主循环 */
    uint8_t sensor_index = 0;  /* 轮询传感器索引 */
    uint32_t last_debug_tick = 0;

    while (1) {
        uint32_t now = get_tick();

        /* ===== 传感器轮询读取 ===== */
        /* 读取激光传感器(每次循环读一个，避免阻塞) */
        uint8_t laser_addrs[3] = {
            VL53L0X_ADDR_LEFT, VL53L0X_ADDR_CENTER, VL53L0X_ADDR_RIGHT
        };

        uint16_t laser_dist = vl53l0x_read_distance(laser_addrs[sensor_index]);
        g_laser[sensor_index].distance_mm = laser_dist;
        g_laser[sensor_index].valid = (laser_dist < 8000);
        g_laser[sensor_index].confidence = g_laser[sensor_index].valid ? 90 : 0;
        g_laser[sensor_index].last_update_ms = now;

        /* 读取对应的超声波传感器 */
        uint16_t us_dist;
        switch (sensor_index) {
        case POS_LEFT:
            us_dist = ultrasonic_measure(
                US_LEFT_TRIG_PORT, US_LEFT_TRIG_PIN,
                US_LEFT_ECHO_PORT, US_LEFT_ECHO_PIN);
            break;
        case POS_CENTER:
            us_dist = ultrasonic_measure(
                US_CENTER_TRIG_PORT, US_CENTER_TRIG_PIN,
                US_CENTER_ECHO_PORT, US_CENTER_ECHO_PIN);
            break;
        case POS_RIGHT:
            us_dist = ultrasonic_measure(
                US_RIGHT_TRIG_PORT, US_RIGHT_TRIG_PIN,
                US_RIGHT_ECHO_PORT, US_RIGHT_ECHO_PIN);
            break;
        default:
            us_dist = 0;
            break;
        }

        g_ultrasonic[sensor_index].distance_mm = us_dist;
        g_ultrasonic[sensor_index].valid = (us_dist > 20 && us_dist < US_MAX_MM);
        g_ultrasonic[sensor_index].confidence = g_ultrasonic[sensor_index].valid ? 70 : 0;
        g_ultrasonic[sensor_index].last_update_ms = now;

        /* 轮询下一个传感器 */
        sensor_index = (sensor_index + 1) % NUM_LASER_SENSORS;

        /* ===== 数据融合 ===== */
        fuse_sensor_data();

        /* ===== 障碍物地图更新 ===== */
        update_grid_map();

        /* ===== 避障决策 ===== */
        obstacle_decision();

        /* ===== LED更新 ===== */
        update_leds();

        /* ===== 调试输出(每500ms) ===== */
        if ((now - last_debug_tick) >= 500) {
            print_debug();
            last_debug_tick = now;
        }

        delay_ms(50); /* 总循环约200ms(3个传感器×50ms+开销) */
    }
}
