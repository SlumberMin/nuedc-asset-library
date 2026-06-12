/**
 * @file 16ch_servo_controller.c
 * @brief 16路舵机控制器 - SG90多路 + 预设动作组 + 串口控制
 * @platform MSPM0G3507
 *
 * 硬件连接：
 *   PCA9685舵机驱动板 (I2C):
 *     SDA  -> PA0 (I2C0_SDA)
 *     SCL  -> PA1 (I2C0_SCL)
 *     VCC  -> 5V (舵机电源)
 *     V+   -> 外部5~6V (舵机大电流供电)
 *     OE   -> PA15 (输出使能, 低有效)
 *
 *   SG90舵机 x16:
 *     CH0~CH15 -> PCA9685输出通道
 *     信号线(橙) -> PCA9685 PWM输出
 *     电源(红)   -> 5V
 *     地(棕)     -> GND
 *
 *   UART串口控制:
 *     PA8  -> UART0_TX
 *     PA9  -> UART0_RX
 *     波特率: 115200
 *
 *   LED指示:
 *     PB0 -> 运行指示
 *     PB1 -> 串口活动指示
 *
 * 串口协议 (ASCII):
 *   #<ch>P<us>T<ms>!   设置单通道角度
 *     ch: 通道0-15, us: 脉宽500-2500, ms: 运动时间
 *   $<group>!           执行预设动作组 (0-9)
 *   G<ch>!              查询通道角度
 *   M<group>!           存储当前姿态为动作组
 *   H!                  全部归中
 *   S<speed>!           设置默认速度(1-10)
 *   A<state>!           使能/禁用输出 (0=禁用, 1=使能)
 *
 * 功能：16路舵机独立控制、平滑运动、动作组存储回放
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/* ===== PCA9685 驱动 ===== */
#define PCA9685_ADDR    0x40   /* I2C默认地址(A5~A0=0) */

/* PCA9685寄存器 */
#define PCA9685_MODE1      0x00
#define PCA9685_MODE2      0x01
#define PCA9685_LED0_ON_L  0x06
#define PCA9685_LED0_ON_H  0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09
#define PCA9685_ALL_LED_ON_L  0xFA
#define PCA9685_ALL_LED_ON_H  0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_PRESCALE   0xFE

#define PCA9685_OSC_FREQ   25000000UL  /* 内部振荡器25MHz */
#define PCA9685_PWM_FREQ   50          /* 舵机标准50Hz */
#define PCA9685_PWM_STEPS  4096        /* 12位分辨率 */

/* SG90舵机参数 */
#define SERVO_MIN_PULSE_US  500    /* 最小脉宽(us) -> -90° */
#define SERVO_MAX_PULSE_US  2500   /* 最大脉宽(us) -> +90° */
#define SERVO_CENTER_US     1500   /* 中位脉宽(us) -> 0° */
#define SERVO_MIN_ANGLE     -90
#define SERVO_MAX_ANGLE     90

#define NUM_CHANNELS  16
#define NUM_PRESETS   10   /* 预设动作组数 */

/* EEPROM存储地址(使用FM24CL64或片内Flash模拟) */
/* 这里使用片内SRAM模拟, 实际可接铁电存储器 */
#define PRESET_MAGIC  0x50524553  /* "PRES" */

/* ===== 动作组结构 ===== */
#pragma pack(push, 1)
typedef struct {
    uint16_t pulse_width[NUM_CHANNELS]; /* 16通道脉宽(us) */
    uint16_t move_time_ms;              /* 运动时间(ms) */
} ServoPose;
#pragma pack(pop)

typedef struct {
    uint32_t magic;
    uint8_t  pose_count;                /* 动作组包含的姿势数 */
    ServoPose poses[20];                /* 每组最多20个姿势 */
    uint16_t loop_count;                /* 循环次数(0=不循环) */
} ActionPreset;

/* ===== 全局变量 ===== */
static uint16_t current_pulse[NUM_CHANNELS];   /* 当前脉宽 */
static uint16_t target_pulse[NUM_CHANNELS];    /* 目标脉宽 */
static uint16_t start_pulse[NUM_CHANNELS];     /* 运动起始脉宽 */
static uint32_t move_start_tick = 0;
static uint32_t move_duration_ms = 0;
static bool is_moving = false;
static bool output_enabled = true;
static uint8_t default_speed = 5;   /* 默认速度1-10 */

static ActionPreset presets[NUM_PRESETS];

/* 串口接收缓冲 */
#define UART_BUF_SIZE 64
static uint8_t uart_buf[UART_BUF_SIZE];
static uint8_t uart_idx = 0;
static bool cmd_ready = false;

static uint32_t system_tick = 0;

/* ===== I2C通信 ===== */
static bool PCA9685_WriteReg(uint8_t reg, uint8_t val)
{
    DL_I2C_clearInterruptStatus(I2C0, DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
                                      DL_I2C_INTERRUPT_CONTROLLER_NACK);
    DL_I2C_startControllerTransfer(I2C0, PCA9685_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    DL_I2C_transmitControllerData(I2C0, reg);
    uint32_t timeout = 10000;
    while (DL_I2C_isControllerTXFIFOFull(I2C0) && timeout--) {}
    DL_I2C_transmitControllerData(I2C0, val);
    timeout = 10000;
    while (DL_I2C_isControllerBusy(I2C0) && timeout--) {}
    return true;
}

static uint8_t PCA9685_ReadReg(uint8_t reg)
{
    DL_I2C_clearInterruptStatus(I2C0, DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
                                      DL_I2C_INTERRUPT_CONTROLLER_NACK);
    /* 写寄存器地址 */
    DL_I2C_startControllerTransfer(I2C0, PCA9685_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    DL_I2C_transmitControllerData(I2C0, reg);
    uint32_t timeout = 10000;
    while (DL_I2C_isControllerBusy(I2C0) && timeout--) {}

    /* 读数据 */
    DL_I2C_startControllerTransfer(I2C0, PCA9685_ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    timeout = 10000;
    while (DL_I2C_isControllerRXFIFOEmpty(I2C0) && timeout--) {}
    uint8_t val = DL_I2C_receiveControllerData(I2C0);
    DL_I2C_stopControllerTransfer(I2C0);
    return val;
}

static void delay_ms(uint32_t ms)
{
    delay_cycles(ms * (CPUCLK_FREQ / 1000));
}

/* ===== PCA9685 初始化 ===== */
static void PCA9685_Init(void)
{
    /* 硬件复位 */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_15);  /* OE拉低使能 */
    delay_ms(10);

    /* 软复位 */
    PCA9685_WriteReg(PCA9685_MODE1, 0x80);  /* RESTART位 */
    delay_ms(10);

    /* 设置PWM频率50Hz */
    /* prescale = round(25MHz / (4096 * freq)) - 1 */
    uint8_t prescale = (PCA9685_OSC_FREQ / (PCA9685_PWM_STEPS * PCA9685_PWM_FREQ)) - 1;
    /* prescale = 25000000/(4096*50)-1 = 121.1 -> 122 */

    uint8_t old_mode = PCA9685_ReadReg(PCA9685_MODE1);
    PCA9685_WriteReg(PCA9685_MODE1, (old_mode & 0x7F) | 0x10);  /* Sleep */
    PCA9685_WriteReg(PCA9685_PRESCALE, prescale);
    PCA9685_WriteReg(PCA9685_MODE1, old_mode);
    delay_ms(5);
    PCA9685_WriteReg(PCA9685_MODE1, old_mode | 0xA0);  /* Auto-Increment + Restart */

    PCA9685_WriteReg(PCA9685_MODE2, 0x04);  /* 输出驱动模式: totem-pole */
}

/* ===== 设置单通道PWM ===== */
/* pulse_us: 脉宽微秒 (500~2500) */
static void PCA9685_SetPulse(uint8_t ch, uint16_t pulse_us)
{
    if (ch >= NUM_CHANNELS) return;

    /* 将脉宽(us)转换为PWM计数值 */
    /* 一个PWM周期 = 1000000/50 = 20000us = 4096计数 */
    /* 计数值 = pulse_us * 4096 / 20000 */
    uint32_t off_count = (uint32_t)pulse_us * PCA9685_PWM_STEPS / 20000;
    if (off_count > PCA9685_PWM_STEPS - 1) off_count = PCA9685_PWM_STEPS - 1;

    uint8_t reg = PCA9685_LED0_ON_L + ch * 4;

    /* 从0开始, 在off_count结束 */
    uint8_t data[4] = {
        0x00, 0x00,                    /* ON_L, ON_H: 从0开始 */
        off_count & 0xFF,              /* OFF_L */
        (off_count >> 8) & 0x0F        /* OFF_H */
    };

    /* 写入4个寄存器 */
    for (int i = 0; i < 4; i++) {
        PCA9685_WriteReg(reg + i, data[i]);
    }
}

/* ===== 全部通道设置 ===== */
static void PCA9685_SetAllPulse(uint16_t *pulse_array)
{
    for (int i = 0; i < NUM_CHANNELS; i++) {
        PCA9685_SetPulse(i, pulse_array[i]);
        current_pulse[i] = pulse_array[i];
    }
}

/* ===== 输出使能控制 ===== */
static void Servo_OutputEnable(bool en)
{
    output_enabled = en;
    if (en) {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_15);  /* OE低使能 */
    } else {
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_15);    /* OE高禁用 */
    }
}

/* ===== 角度与脉宽转换 ===== */
static int16_t PulseToAngle(uint16_t pulse_us)
{
    if (pulse_us <= SERVO_MIN_PULSE_US) return SERVO_MIN_ANGLE;
    if (pulse_us >= SERVO_MAX_PULSE_US) return SERVO_MAX_ANGLE;
    return (int16_t)((int32_t)(pulse_us - SERVO_CENTER_US) * 90 /
                     ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 2));
}

static uint16_t AngleToPulse(int16_t angle)
{
    if (angle <= SERVO_MIN_ANGLE) return SERVO_MIN_PULSE_US;
    if (angle >= SERVO_MAX_ANGLE) return SERVO_MAX_PULSE_US;
    return (uint16_t)((int32_t)angle * ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 2) /
                      90 + SERVO_CENTER_US);
}

/* ===== 平滑运动控制 ===== */
static void Servo_MoveTo(uint8_t ch, uint16_t target_us, uint16_t time_ms)
{
    target_pulse[ch] = target_us;
    if (time_ms == 0) {
        /* 立即移动 */
        start_pulse[ch] = target_us;
        current_pulse[ch] = target_us;
        PCA9685_SetPulse(ch, target_us);
    } else {
        start_pulse[ch] = current_pulse[ch];
        is_moving = true;
        move_duration_ms = time_ms;
        move_start_tick = system_tick;
    }
}

/* 多通道同步移动 */
static void Servo_MoveAll(uint16_t *targets, uint16_t time_ms)
{
    for (int i = 0; i < NUM_CHANNELS; i++) {
        Servo_MoveTo(i, targets[i], time_ms);
    }
}

/* 运动更新(在主循环中调用) */
static void Servo_UpdateMovement(void)
{
    if (!is_moving) return;

    uint32_t elapsed = system_tick - move_start_tick;
    if (elapsed >= move_duration_ms) {
        /* 运动完成 */
        for (int i = 0; i < NUM_CHANNELS; i++) {
            current_pulse[i] = target_pulse[i];
            PCA9685_SetPulse(i, current_pulse[i]);
        }
        is_moving = false;
    } else {
        /* 线性插值 */
        uint32_t progress = (uint32_t)elapsed * 256 / move_duration_ms;
        for (int i = 0; i < NUM_CHANNELS; i++) {
            int32_t diff = (int32_t)target_pulse[i] - start_pulse[i];
            current_pulse[i] = start_pulse[i] + (diff * progress / 256);
            PCA9685_SetPulse(i, current_pulse[i]);
        }
    }
}

/* ===== 全部归中 ===== */
static void Servo_CenterAll(void)
{
    uint16_t targets[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        targets[i] = SERVO_CENTER_US;
    }
    Servo_MoveAll(targets, 500);
}

/* ===== 动作组管理 ===== */
static void Preset_SaveCurrentAs(uint8_t group)
{
    if (group >= NUM_PRESETS) return;
    presets[group].magic = PRESET_MAGIC;
    presets[group].pose_count = 1;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        presets[group].poses[0].pulse_width[i] = current_pulse[i];
    }
    presets[group].poses[0].move_time_ms = 500;
    presets[group].loop_count = 0;

    /* 实际应保存到Flash/EEPROM */
}

static void Preset_Execute(uint8_t group)
{
    if (group >= NUM_PRESETS) return;
    if (presets[group].magic != PRESET_MAGIC) return;

    ActionPreset *p = &presets[group];
    uint16_t loop = p->loop_count > 0 ? p->loop_count : 1;

    for (uint16_t l = 0; l < loop; l++) {
        for (uint8_t pose = 0; pose < p->pose_count; pose++) {
            Servo_MoveAll(p->poses[pose].pulse_width, p->poses[pose].move_time_ms);
            /* 等待运动完成 */
            while (is_moving) {
                Servo_UpdateMovement();
                delay_ms(20);
            }
            delay_ms(50);  /* 姿势间隔 */
        }
    }
}

/* ===== 预设动作组示例(机械臂常见动作) ===== */
static void Preset_InitDemo(void)
{
    /* 动作组0: 全部归中 */
    presets[0].magic = PRESET_MAGIC;
    presets[0].pose_count = 1;
    for (int i = 0; i < NUM_CHANNELS; i++)
        presets[0].poses[0].pulse_width[i] = SERVO_CENTER_US;
    presets[0].poses[0].move_time_ms = 800;
    presets[0].loop_count = 1;

    /* 动作组1: 挥手 */
    presets[1].magic = PRESET_MAGIC;
    presets[1].pose_count = 4;
    uint16_t wave_poses[4][16] = {
        {1500, 1500, 1500, 1500, 2000, 1500, 1500, 1500,
         1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500},
        {1500, 1500, 1500, 1500, 2000, 1200, 1500, 1500,
         1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500},
        {1500, 1500, 1500, 1500, 2000, 1800, 1500, 1500,
         1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500},
        {1500, 1500, 1500, 1500, 2000, 1200, 1500, 1500,
         1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500},
    };
    for (int p = 0; p < 4; p++) {
        memcpy(presets[1].poses[p].pulse_width, wave_poses[p], sizeof(uint16_t) * NUM_CHANNELS);
        presets[1].poses[p].move_time_ms = 400;
    }
    presets[1].loop_count = 3;
}

/* ===== 串口命令解析 ===== */
static void UART_SendStr(const char *str)
{
    while (*str) {
        DL_UART_transmitData(UART0, *str++);
        while (DL_UART_isBusy(UART0)) {}
    }
}

static void UART_SendNum(int num)
{
    char buf[12];
    int pos = 0;
    if (num < 0) { buf[pos++] = '-'; num = -num; }
    if (num == 0) { buf[pos++] = '0'; }
    else {
        char rev[10]; int rl = 0;
        while (num > 0) { rev[rl++] = '0' + num % 10; num /= 10; }
        for (int i = rl - 1; i >= 0; i--) buf[pos++] = rev[i];
    }
    buf[pos] = '\0';
    UART_SendStr(buf);
}

/* 解析数字 */
static int Parse_Number(const char *s, int len)
{
    int val = 0;
    bool neg = false;
    for (int i = 0; i < len; i++) {
        if (s[i] == '-') { neg = true; continue; }
        if (s[i] >= '0' && s[i] <= '9') {
            val = val * 10 + (s[i] - '0');
        }
    }
    return neg ? -val : val;
}

static void Process_Command(void)
{
    if (!cmd_ready) return;
    cmd_ready = false;

    if (uart_buf[0] == '#') {
        /* #<ch>P<us>T<ms>! 设置单通道 */
        /* 例: #5P1500T500!  通道5, 1500us, 500ms */
        int ch = -1, pw = -1, tm = 0;
        char *pP = NULL, *pT = NULL;

        for (int i = 1; i < uart_idx; i++) {
            if (uart_buf[i] == 'P') { pP = (char *)&uart_buf[i + 1]; ch = Parse_Number((char *)&uart_buf[1], i - 1); }
            if (uart_buf[i] == 'T') { pT = (char *)&uart_buf[i + 1]; }
        }
        if (pP) pw = Parse_Number(pP, (pT ? (int)(pT - pP - 1) : (int)(uart_idx - (int)(pP - (char *)uart_buf) - 1)));
        if (pT) tm = Parse_Number(pT, uart_idx - (int)(pT - (char *)uart_buf) - 1);

        if (ch >= 0 && ch < NUM_CHANNELS && pw >= SERVO_MIN_PULSE_US && pw <= SERVO_MAX_PULSE_US) {
            Servo_MoveTo(ch, pw, tm);
            UART_SendStr("OK\r\n");
        } else {
            UART_SendStr("ERR\r\n");
        }
    }
    else if (uart_buf[0] == '$') {
        /* $<group>! 执行动作组 */
        uint8_t group = uart_buf[1] - '0';
        if (group < NUM_PRESETS) {
            Preset_Execute(group);
            UART_SendStr("OK\r\n");
        }
    }
    else if (uart_buf[0] == 'G') {
        /* G<ch>! 查询通道 */
        uint8_t ch = uart_buf[1] - '0';
        if (ch < NUM_CHANNELS) {
            UART_SendStr("CH");
            UART_SendNum(ch);
            UART_SendStr(":");
            UART_SendNum(current_pulse[ch]);
            UART_SendStr("us (");
            UART_SendNum(PulseToAngle(current_pulse[ch]));
            UART_SendStr(" deg)\r\n");
        }
    }
    else if (uart_buf[0] == 'M') {
        /* M<group>! 存储动作组 */
        uint8_t group = uart_buf[1] - '0';
        if (group < NUM_PRESETS) {
            Preset_SaveCurrentAs(group);
            UART_SendStr("SAVED\r\n");
        }
    }
    else if (uart_buf[0] == 'H') {
        /* H! 归中 */
        Servo_CenterAll();
        UART_SendStr("CENTER\r\n");
    }
    else if (uart_buf[0] == 'S') {
        /* S<speed>! 设置速度 */
        uint8_t spd = uart_buf[1] - '0';
        if (spd >= 1 && spd <= 9) {
            default_speed = spd;
            UART_SendStr("OK\r\n");
        }
    }
    else if (uart_buf[0] == 'A') {
        /* A<state>! 使能/禁用 */
        Servo_OutputEnable(uart_buf[1] == '1');
        UART_SendStr("OK\r\n");
    }

    uart_idx = 0;
}

/* ===== UART中断接收 ===== */
void UART0_IRQHandler(void)
{
    if (DL_UART_isRXFIFOEmpty(UART0) == false) {
        uint8_t ch = DL_UART_receiveData8(UART0);

        if (ch == '!' || ch == '\n' || ch == '\r') {
            if (uart_idx > 0) {
                cmd_ready = true;
            }
        } else {
            if (uart_idx < UART_BUF_SIZE - 1) {
                uart_buf[uart_idx++] = ch;
            }
        }

        DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_1);
    }
}

/* ===== SysTick中断(1ms) ===== */
void SysTick_Handler(void)
{
    system_tick++;
}

/* ===== 主函数 ===== */
int main(void)
{
    SYSCFG_DL_init();

    /* 配置SysTick 1ms中断 */
    SysTick_Config(SystemCoreClock / 1000);

    /* 初始化PCA9685 */
    PCA9685_Init();

    /* 初始化预设动作组 */
    Preset_InitDemo();

    /* 全部归中 */
    Servo_CenterAll();

    /* 发送启动信息 */
    UART_SendStr("=== 16CH Servo Controller ===\r\n");
    UART_SendStr("Commands:\r\n");
    UART_SendStr("  #<ch>P<us>T<ms>!  Move servo\r\n");
    UART_SendStr("  $<group>!         Execute preset\r\n");
    UART_SendStr("  G<ch>!            Query servo\r\n");
    UART_SendStr("  M<group>!         Save preset\r\n");
    UART_SendStr("  H!                Center all\r\n");
    UART_SendStr("  A<0|1>!           Enable/Disable\r\n");

    while (1) {
        /* 处理串口命令 */
        Process_Command();

        /* 更新运动 */
        Servo_UpdateMovement();

        /* 运行指示LED */
        static uint32_t led_tick = 0;
        if (system_tick - led_tick >= 500) {
            led_tick = system_tick;
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_0);
        }

        delay_ms(20);
    }
}
