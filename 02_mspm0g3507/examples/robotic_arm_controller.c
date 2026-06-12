/**
 * @file robotic_arm_controller.c
 * @brief 机械臂控制器 - 6路舵机 + 逆运动学 + 示教再现
 * @platform MSPM0G3507
 *
 * 硬件连接：
 *   PCA9685 16路PWM舵机驱动板 (I2C0):
 *     SCL  -> PA1  (I2C0_SCL)
 *     SDA  -> PA0  (I2C0_SDA)
 *     VCC  -> 5V (舵机电源)
 *     V+   -> 外部电源 (舵机供电, 6~8.4V)
 *
 *   6路舵机连接 (PCA9685通道):
 *     CH0  -> 底座旋转 (Base)       - MG996R
 *     CH1  -> 大臂 (Shoulder)       - MG996R
 *     CH2  -> 小臂 (Elbow)          - MG996R
 *     CH3  -> 腕部俯仰 (Wrist Pitch) - SG90
 *     CH4  -> 腕部旋转 (Wrist Roll)  - SG90
 *     CH5  -> 夹爪 (Gripper)         - SG90
 *
 *   电位器示教输入 (ADC0):
 *     PA25 -> ADC0_CH0 (Base电位器)
 *     PA26 -> ADC0_CH1 (Shoulder电位器)
 *     PA27 -> ADC0_CH2 (Elbow电位器)
 *     PA28 -> ADC0_CH3 (Wrist Pitch电位器)
 *     PA29 -> ADC0_CH4 (Wrist Roll电位器)
 *     PA30 -> ADC0_CH5 (Gripper电位器)
 *
 *   按键:
 *     PB0 -> 示教模式/运行模式切换
 *     PB1 -> 记录当前姿态点
 *     PB2 -> 执行示教轨迹回放
 *     PB3 -> 夹爪开/关
 *
 *   OLED SSD1306 (I2C1):
 *     SCL  -> PB2  (I2C1_SCL)
 *     SDA  -> PB3  (I2C1_SDA)
 *
 *   LED:
 *     PB14 -> 示教模式指示
 *     PB15 -> 运行中指示
 *
 * 功能：
 *   - 6自由度机械臂控制
 *   - 基于PCA9685的精确PWM输出
 *   - 逆运动学算法（3连杆平面IK）
 *   - 示教模式：通过电位器控制各关节角度
 *   - 姿态记录：存储最多32个示教点到FM24CL64
 *   - 轨迹回放：可变速率平滑插值
 *   - 运动约束：关节角度限位保护
 *
 * 机械臂尺寸（mm）:
 *   L1(大臂)=100, L2(小臂)=80, L3(腕部)=60
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ===== PCA9685 驱动 ===== */
#define PCA9685_ADDR        0x40
#define PCA9685_REG_MODE1   0x00
#define PCA9685_REG_LED0_ON_L  0x06
#define PCA9685_REG_PRE_SCALE  0xFE

/* PCA9685参数 */
#define PCA9685_OSC_FREQ    25000000U
#define PCA9685_PWM_STEPS   4096

/* ===== 舵机参数 ===== */
#define SERVO_COUNT     6
#define SERVO_FREQ      50      /* 50Hz */
#define SERVO_MIN_US    500     /* 0°对应脉宽 (us) */
#define SERVO_MAX_US    2500    /* 180°对应脉宽 (us) */

/* 舵机通道映射 */
#define CH_BASE         0
#define CH_SHOULDER     1
#define CH_ELBOW        2
#define CH_WRIST_PITCH  3
#define CH_WRIST_ROLL   4
#define CH_GRIPPER      5

/* 关节角度限位 (度) */
typedef struct {
    float min_angle;
    float max_angle;
    float init_angle;   /* 初始角度 */
    const char *name;
} JointLimit_t;

static const JointLimit_t joint_limits[SERVO_COUNT] = {
    {   0.0f,  180.0f,  90.0f, "BASE"    },  /* 底座旋转 */
    {  10.0f,  170.0f,  90.0f, "SHLDR"   },  /* 大臂 */
    {   0.0f,  160.0f,  90.0f, "ELBOW"   },  /* 小臂 */
    {   0.0f,  180.0f,  90.0f, "WPITCH"  },  /* 腕部俯仰 */
    {   0.0f,  180.0f,  90.0f, "WROLL"   },  /* 腕部旋转 */
    {  20.0f,  150.0f,  90.0f, "GRIP"    },  /* 夹爪 */
};

/* 机械臂物理尺寸 (mm) */
#define ARM_L1  100.0f  /* 大臂长度 */
#define ARM_L2  80.0f   /* 小臂长度 */
#define ARM_L3  60.0f   /* 腕部长度 */

/* ===== 示教参数 ===== */
#define MAX_TEACH_POINTS    32
#define TEACH_STORAGE_ADDR  0x0000   /* FM24CL64存储起始地址 */

/* 姿态点 */
typedef struct {
    float angles[SERVO_COUNT];  /* 6个关节角度 */
    uint16_t duration_ms;       /* 运动持续时间 */
    uint8_t valid;              /* 有效性标记 */
} PosePoint_t;

/* 运动模式 */
typedef enum {
    MODE_IDLE = 0,      /* 空闲 */
    MODE_TEACH,         /* 示教模式 */
    MODE_PLAYBACK,      /* 回放模式 */
    MODE_IK_CONTROL,    /* 逆运动学控制 */
    MODE_COUNT
} ControlMode_t;

/* ===== I2C PCA9685 操作 ===== */
static bool PCA9685_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    DL_I2C_fillControllerTXFIFO(I2C0, buf, 2);
    DL_I2C_startControllerTransfer(I2C0, PCA9685_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    uint32_t t = 10000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--t == 0) return false;
    }
    DL_I2C_flushControllerTXFIFO(I2C0);
    return true;
}

static bool PCA9685_Init(void)
{
    delay_cycles(160000);

    /* 软复位 */
    if (!PCA9685_WriteReg(PCA9685_REG_MODE1, 0x80)) return false;
    delay_cycles(16000);

    /* 设置PWM频率 50Hz */
    uint8_t prescale = (uint8_t)(PCA9685_OSC_FREQ / (PCA9685_PWM_STEPS * SERVO_FREQ) - 1);
    PCA9685_WriteReg(PCA9685_REG_MODE1, 0x10);  /* 进入睡眠 */
    delay_cycles(8000);
    PCA9685_WriteReg(PCA9685_REG_PRE_SCALE, prescale);
    PCA9685_WriteReg(PCA9685_REG_MODE1, 0xA0);  /* 自动递增+重启 */
    delay_cycles(8000);

    return true;
}

/* 设置指定通道的PWM脉宽 */
static void PCA9685_SetPulse(uint8_t channel, uint16_t pulse_us)
{
    /* 计算PWM值: pulse_us * 4096 / 20000 (20ms周期) */
    uint32_t ticks = (uint32_t)pulse_us * PCA9685_PWM_STEPS / 20000;
    if (ticks > PCA9685_PWM_STEPS - 1) ticks = PCA9685_PWM_STEPS - 1;

    uint8_t reg = PCA9685_REG_LED0_ON_L + 4 * channel;
    uint8_t data[5];
    data[0] = reg;
    data[1] = 0x00;          /* ON_L = 0 */
    data[2] = 0x00;          /* ON_H = 0 */
    data[3] = ticks & 0xFF;  /* OFF_L */
    data[4] = (ticks >> 8);  /* OFF_H */

    DL_I2C_fillControllerTXFIFO(I2C0, data, 5);
    DL_I2C_startControllerTransfer(I2C0, PCA9685_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 5);
    uint32_t t = 10000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--t == 0) break;
    }
    DL_I2C_flushControllerTXFIFO(I2C0);
}

/* 设置关节角度 */
static float current_angles[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};

static void SetJointAngle(uint8_t joint, float angle)
{
    /* 限位保护 */
    if (angle < joint_limits[joint].min_angle) angle = joint_limits[joint].min_angle;
    if (angle > joint_limits[joint].max_angle) angle = joint_limits[joint].max_angle;

    current_angles[joint] = angle;

    /* 转换为脉宽 */
    float ratio = (angle - joint_limits[joint].min_angle) /
                  (joint_limits[joint].max_angle - joint_limits[joint].min_angle);
    uint16_t pulse = SERVO_MIN_US + (uint16_t)(ratio * (SERVO_MAX_US - SERVO_MIN_US));

    PCA9685_SetPulse(joint, pulse);
}

/* 设置所有关节 */
static void SetAllJoints(const float angles[SERVO_COUNT])
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        SetJointAngle(i, angles[i]);
    }
}

/* ===== ADC 多通道读取 ===== */
/* 注意：MSPM0G3507 ADC单次转换一个通道，这里通过轮询实现 */
static uint16_t ADC_ReadChannel(uint8_t ch)
{
    /* 选择通道并转换 */
    DL_ADC12_startConversion(ADC0);
    while (!DL_ADC12_isConversionComplete(ADC0)) {}
    return DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
}

/* 从电位器读取角度 */
static float ReadPotAngle(uint8_t joint)
{
    uint16_t adc = ADC_ReadChannel(joint);
    float ratio = (float)adc / 4095.0f;
    float angle = joint_limits[joint].min_angle +
                  ratio * (joint_limits[joint].max_angle - joint_limits[joint].min_angle);
    return angle;
}

/* ===== 逆运动学 (3连杆平面IK) ===== */
/* 输入：目标末端位置 (x, y, z)，输出：base_angle, theta1, theta2 */
typedef struct {
    float base_angle;   /* 底座旋转角 (度) */
    float theta1;       /* 大臂仰角 (度) */
    float theta2;       /* 小臂相对角 (度) */
    bool valid;         /* 解是否有效 */
} IKResult_t;

static IKResult_t InverseKinematics(float x, float y, float z)
{
    IKResult_t result;
    result.valid = false;

    /* 底座角度 = atan2(y, x) */
    result.base_angle = atan2f(y, x) * 180.0f / 3.14159265f;
    if (result.base_angle < 0) result.base_angle += 360.0f;

    /* 水平距离 */
    float r = sqrtf(x * x + y * y);
    /* 垂直距离（减去底座高度偏移） */
    float z_eff = z;

    /* 考虑腕部末端偏移 */
    /* 简化：2连杆IK，求解大臂和小臂角度 */
    /* 目标距离 */
    float D = sqrtf(r * r + z_eff * z_eff);
    float L_sum = ARM_L1 + ARM_L2;

    /* 检查是否可达 */
    if (D > L_sum || D < fabsf(ARM_L1 - ARM_L2)) {
        return result;  /* 不可达 */
    }

    /* 余弦定理求小臂角度 */
    float cos_theta2 = (D * D - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2) / (2.0f * ARM_L1 * ARM_L2);
    if (cos_theta2 < -1.0f || cos_theta2 > 1.0f) return result;
    float theta2_rad = acosf(cos_theta2);

    /* 大臂角度 */
    float alpha = atan2f(z_eff, r);
    float beta = acosf((D * D + ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2) / (2.0f * D * ARM_L1));
    float theta1_rad = alpha + beta;

    result.theta1 = theta1_rad * 180.0f / 3.14159265f;
    result.theta2 = theta2_rad * 180.0f / 3.14159265f;
    result.valid = true;

    return result;
}

/* 正运动学 — 根据关节角度计算末端位置 */
typedef struct {
    float x, y, z;
} Vec3_t;

static Vec3_t ForwardKinematics(const float angles[SERVO_COUNT])
{
    Vec3_t pos;
    float base_rad = angles[0] * 3.14159265f / 180.0f;
    float theta1_rad = angles[1] * 3.14159265f / 180.0f;
    float theta2_rad = angles[2] * 3.14159265f / 180.0f;

    /* 2连杆FK */
    float r = ARM_L1 * cosf(theta1_rad) + ARM_L2 * cosf(theta1_rad + theta2_rad);
    float z = ARM_L1 * sinf(theta1_rad) + ARM_L2 * sinf(theta1_rad + theta2_rad);

    pos.x = r * cosf(base_rad);
    pos.y = r * sinf(base_rad);
    pos.z = z;

    return pos;
}

/* ===== FM24CL64 存储示教数据 ===== */
#define FM24CL64_ADDR   0x50

static bool FM24_Write(uint16_t addr, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t buf[3];
        buf[0] = (uint8_t)((addr + i) >> 8);
        buf[1] = (uint8_t)((addr + i) & 0xFF);
        buf[2] = data[i];
        DL_I2C_flushControllerTXFIFO(I2C0);
        DL_I2C_fillControllerTXFIFO(I2C0, buf, 3);
        DL_I2C_startControllerTransfer(I2C0, FM24CL64_ADDR,
                                        DL_I2C_CONTROLLER_DIRECTION_TX, 3);
        uint32_t t = 10000;
        while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
            if (--t == 0) return false;
        }
    }
    return true;
}

static bool FM24_Read(uint16_t addr, uint8_t *data, uint16_t len)
{
    uint8_t ab[2] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF)};
    DL_I2C_flushControllerTXFIFO(I2C0);
    DL_I2C_fillControllerTXFIFO(I2C0, ab, 2);
    DL_I2C_startControllerTransfer(I2C0, FM24CL64_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    uint32_t t = 10000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--t == 0) return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C0);
    DL_I2C_startControllerTransfer(I2C0, FM24CL64_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_RX, len);
    t = 10000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--t == 0) return false;
    }
    for (uint16_t i = 0; i < len; i++) data[i] = DL_I2C_receiveControllerData(I2C0);
    return true;
}

/* 保存示教轨迹到FM24CL64 */
static uint8_t teach_count = 0;
static PosePoint_t teach_points[MAX_TEACH_POINTS];

static bool SaveTeachData(void)
{
    /* 写入点数 */
    uint8_t cnt = teach_count;
    FM24_Write(TEACH_STORAGE_ADDR, &cnt, 1);

    /* 写入所有点 */
    for (uint8_t i = 0; i < teach_count; i++) {
        uint16_t addr = TEACH_STORAGE_ADDR + 1 + i * sizeof(PosePoint_t);
        FM24_Write(addr, (uint8_t *)&teach_points[i], sizeof(PosePoint_t));
    }
    return true;
}

/* 加载示教轨迹 */
static bool LoadTeachData(void)
{
    uint8_t cnt;
    FM24_Read(TEACH_STORAGE_ADDR, &cnt, 1);
    if (cnt > MAX_TEACH_POINTS) { teach_count = 0; return false; }

    teach_count = cnt;
    for (uint8_t i = 0; i < teach_count; i++) {
        uint16_t addr = TEACH_STORAGE_ADDR + 1 + i * sizeof(PosePoint_t);
        FM24_Read(addr, (uint8_t *)&teach_points[i], sizeof(PosePoint_t));
    }
    return true;
}

/* ===== 平滑运动插值 ===== */
static float target_angles[SERVO_COUNT];

/* 线性插值移动到目标 */
static void MoveToTarget(uint16_t duration_ms)
{
    float start_angles[SERVO_COUNT];
    memcpy(start_angles, current_angles, sizeof(start_angles));

    uint16_t steps = duration_ms / 20; /* 20ms/步 */
    if (steps < 1) steps = 1;

    for (uint16_t step = 1; step <= steps; step++) {
        float t = (float)step / (float)steps;
        /* S曲线插值（缓入缓出） */
        float st = t * t * (3.0f - 2.0f * t);

        for (uint8_t i = 0; i < SERVO_COUNT; i++) {
            float angle = start_angles[i] + (target_angles[i] - start_angles[i]) * st;
            SetJointAngle(i, angle);
        }
        delay_ms(20);
    }
}

/* ===== OLED 显示 ===== */
#define OLED_ADDR_DISP  0x3C

static void OLED_WriteCmd2(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    DL_I2C_flushControllerTXFIFO(I2C0, buf, 2);
    DL_I2C_startControllerTransfer(I2C0, OLED_ADDR_DISP,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    uint32_t t = 10000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--t == 0) break;
    }
    DL_I2C_flushControllerTXFIFO(I2C0);
}

/* ===== UART 调试 ===== */
static void UART_Print(const char *s)
{
    while (*s) {
        DL_UART_main_transmitData(UART0, *s);
        while (DL_UART_isBusy(UART0)) {}
        s++;
    }
}

static void UART_PrintNum(int32_t val)
{
    char buf[12]; int p = 0;
    if (val < 0) { DL_UART_main_transmitData(UART0, '-'); while (DL_UART_isBusy(UART0)){} val = -val; }
    char tmp[10]; int t = 0;
    if (val == 0) { tmp[t++] = '0'; }
    else { while (val > 0) { tmp[t++] = '0' + val % 10; val /= 10; } }
    while (t > 0) { DL_UART_main_transmitData(UART0, tmp[--t]); while (DL_UART_isBusy(UART0)){} }
}

static void UART_Println(const char *s) { UART_Print(s); UART_Print("\r\n"); }

/* ===== LED ===== */
#define TEACH_LED_ON()   DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14)
#define TEACH_LED_OFF()  DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14)
#define RUN_LED_ON()     DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_15)
#define RUN_LED_OFF()    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_15)

/* ===== 按键 ===== */
#define BTN_MODE    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_0)))
#define BTN_RECORD  (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_1)))
#define BTN_PLAY    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2)))
#define BTN_GRIP    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3)))

static void delay_ms(uint32_t ms) { delay_cycles(ms * 16000); }

typedef struct { uint8_t prev, pressed; } Btn_t;
static void ScanBtn(Btn_t *b, uint8_t raw) { b->pressed = raw && !b->prev; b->prev = raw; }

/* ===== 关节角度到字符串 ===== */
static void PrintAngles(const char *prefix, const float angles[SERVO_COUNT])
{
    UART_Print(prefix);
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        UART_Print(joint_limits[i].name);
        UART_Print("=");
        UART_PrintNum((int32_t)angles[i]);
        if (i < SERVO_COUNT - 1) UART_Print(" ");
    }
    UART_Print("\r\n");
}

/* ===== 主函数 ===== */
int main(void)
{
    SYSCFG_DL_init();

    UART_Println("\r\n=== Robotic Arm Controller v1.0 ===");
    UART_Println("6DOF + IK + Teach & Playback");

    /* 初始化PCA9685 */
    if (!PCA9685_Init()) {
        UART_Println("[ERR] PCA9685 init failed!");
        while (1) { DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_15); delay_ms(200); }
    }
    UART_Println("[OK] PCA9685 initialized");

    /* 设置初始姿态 */
    float init_angles[SERVO_COUNT];
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        init_angles[i] = joint_limits[i].init_angle;
    }
    SetAllJoints(init_angles);
    memcpy(target_angles, init_angles, sizeof(target_angles));
    delay_ms(1000);

    UART_Println("[OK] Initial pose set");

    /* 加载已有示教数据 */
    if (LoadTeachData() && teach_count > 0) {
        UART_Print("[OK] Loaded ");
        UART_PrintNum(teach_count);
        UART_Println(" teach points");
    } else {
        UART_Println("[INFO] No teach data found");
        teach_count = 0;
    }

    /* 主循环 */
    ControlMode_t mode = MODE_IDLE;
    Btn_t btn_mode = {0}, btn_record = {0}, btn_play = {0}, btn_grip = {0};
    uint8_t gripper_closed = 0;
    uint32_t tick = 0;
    uint8_t playback_idx = 0;
    uint8_t playback_running = 0;

    /* IK目标位置 */
    float ik_x = 100.0f, ik_y = 0.0f, ik_z = 100.0f;

    UART_Println("\r\nCommands: T=Teach, R=Record, P=Play, G=Grip, I=IK");
    UART_Println("Or use buttons: PB0=Mode PB1=Rec PB2=Play PB3=Grip");

    while (1) {
        /* 按键扫描 */
        ScanBtn(&btn_mode, BTN_MODE);
        ScanBtn(&btn_record, BTN_RECORD);
        ScanBtn(&btn_play, BTN_PLAY);
        ScanBtn(&btn_grip, BTN_GRIP);

        /* 模式切换 */
        if (btn_mode.pressed) {
            mode = (ControlMode_t)((mode + 1) % MODE_COUNT);
            switch (mode) {
            case MODE_IDLE:
                UART_Println("\r\n[MODE] IDLE");
                TEACH_LED_OFF();
                break;
            case MODE_TEACH:
                UART_Println("\r\n[MODE] TEACH");
                TEACH_LED_ON();
                break;
            case MODE_PLAYBACK:
                UART_Println("\r\n[MODE] PLAYBACK");
                TEACH_LED_OFF();
                if (teach_count == 0) {
                    UART_Println("[WARN] No teach points!");
                    mode = MODE_IDLE;
                }
                break;
            case MODE_IK_CONTROL:
                UART_Println("\r\n[MODE] IK CONTROL");
                TEACH_LED_ON();
                /* 显示当前末端位置 */
                {
                    Vec3_t pos = ForwardKinematics(current_angles);
                    UART_Print("Current pos: X=");
                    UART_PrintNum((int32_t)pos.x);
                    UART_Print(" Y=");
                    UART_PrintNum((int32_t)pos.y);
                    UART_Print(" Z=");
                    UART_PrintNum((int32_t)pos.z);
                    UART_Print("\r\n");
                }
                break;
            default: break;
            }
        }

        /* ===== 示教模式 ===== */
        if (mode == MODE_TEACH) {
            /* 从电位器读取各关节角度 */
            for (uint8_t i = 0; i < SERVO_COUNT; i++) {
                float angle = ReadPotAngle(i);
                SetJointAngle(i, angle);
            }

            /* 记录示教点 */
            if (btn_record.pressed) {
                if (teach_count < MAX_TEACH_POINTS) {
                    memcpy(teach_points[teach_count].angles, current_angles, sizeof(current_angles));
                    teach_points[teach_count].duration_ms = 1000; /* 默认1秒 */
                    teach_points[teach_count].valid = 1;
                    teach_count++;

                    UART_Print("[REC] Point #");
                    UART_PrintNum(teach_count);
                    UART_Print(" ");
                    PrintAngles("", current_angles);

                    /* 保存到铁电RAM */
                    SaveTeachData();

                    /* LED闪烁确认 */
                    TEACH_LED_OFF();
                    delay_ms(100);
                    TEACH_LED_ON();
                } else {
                    UART_Println("[WARN] Teach buffer full!");
                }
            }

            /* 夹爪控制 */
            if (btn_grip.pressed) {
                gripper_closed = !gripper_closed;
                SetJointAngle(CH_GRIPPER, gripper_closed ? 30.0f : 120.0f);
                UART_Print(gripper_closed ? "[GRIP] Closed\r\n" : "[GRIP] Opened\r\n");
            }
        }

        /* ===== 回放模式 ===== */
        if (mode == MODE_PLAYBACK) {
            if (btn_play.pressed) {
                if (!playback_running) {
                    playback_running = 1;
                    playback_idx = 0;
                    RUN_LED_ON();
                    UART_Println("[PLAY] Starting playback...");
                } else {
                    playback_running = 0;
                    RUN_LED_OFF();
                    UART_Println("[PLAY] Stopped");
                }
            }

            if (playback_running && teach_count > 0) {
                /* 移动到当前示教点 */
                memcpy(target_angles, teach_points[playback_idx].angles, sizeof(target_angles));
                uint16_t duration = teach_points[playback_idx].duration_ms;
                if (duration < 100) duration = 100;

                UART_Print("[PLAY] Moving to point #");
                UART_PrintNum(playback_idx + 1);
                UART_Print("/");
                UART_PrintNum(teach_count);
                UART_Print("\r\n");

                /* 平滑移动 */
                MoveToTarget(duration);
                PrintAngles("  -> ", current_angles);

                /* 下一个点 */
                playback_idx++;
                if (playback_idx >= teach_count) {
                    playback_idx = 0;  /* 循环播放 */
                    UART_Println("[PLAY] Loop restart");
                    delay_ms(500);  /* 循环间隔 */
                }

                /* LED闪烁 */
                RUN_LED_TOGGLE();
            }
        }

        /* ===== 逆运动学控制模式 ===== */
        if (mode == MODE_IK_CONTROL) {
            /* 简化：通过电位器0/1/2控制XYZ目标 */
            float raw_x = ReadPotAngle(0);  /* PA25 -> X */
            float raw_y = ReadPotAngle(1);  /* PA26 -> Y */
            float raw_z = ReadPotAngle(2);  /* PA27 -> Z */

            /* 映射到工作空间 */
            ik_x = raw_x / 180.0f * 200.0f - 50.0f;  /* -50 ~ 150mm */
            ik_y = raw_y / 180.0f * 200.0f - 100.0f;  /* -100 ~ 100mm */
            ik_z = raw_z / 180.0f * 200.0f;            /* 0 ~ 200mm */

            /* 求解IK */
            IKResult_t ik = InverseKinematics(ik_x, ik_y, ik_z);
            if (ik.valid) {
                SetJointAngle(CH_BASE, ik.base_angle);
                SetJointAngle(CH_SHOULDER, ik.theta1);
                SetJointAngle(CH_ELBOW, ik.theta2);
            }

            /* 定期打印位置 */
            if (tick % 50 == 0) {
                Vec3_t pos = ForwardKinematics(current_angles);
                UART_Print("IK: target(");
                UART_PrintNum((int32_t)ik_x); UART_Print(",");
                UART_PrintNum((int32_t)ik_y); UART_Print(",");
                UART_PrintNum((int32_t)ik_z);
                UART_Print(") actual(");
                UART_PrintNum((int32_t)pos.x); UART_Print(",");
                UART_PrintNum((int32_t)pos.y); UART_Print(",");
                UART_PrintNum((int32_t)pos.z);
                UART_Print(")\r\n");
            }

            /* 记录IK位姿点 */
            if (btn_record.pressed && teach_count < MAX_TEACH_POINTS) {
                memcpy(teach_points[teach_count].angles, current_angles, sizeof(current_angles));
                teach_points[teach_count].duration_ms = 1000;
                teach_points[teach_count].valid = 1;
                teach_count++;
                SaveTeachData();
                UART_Print("[REC] IK pose saved #");
                UART_PrintNum(teach_count);
                UART_Print("\r\n");
            }
        }

        /* ===== 空闲模式 ===== */
        if (mode == MODE_IDLE) {
            /* 夹爪控制在任何模式都可用 */
            if (btn_grip.pressed) {
                gripper_closed = !gripper_closed;
                SetJointAngle(CH_GRIPPER, gripper_closed ? 30.0f : 120.0f);
            }

            /* 检查UART命令 */
            if (DL_UART_getEnabledInterruptStatus(UART0) & DL_UART_INTERRUPT_RX) {
                uint8_t cmd = DL_UART_receiveData8(UART0);
                switch (cmd) {
                case 'T': case 't':
                    mode = MODE_TEACH;
                    TEACH_LED_ON();
                    UART_Println("[CMD] -> TEACH mode");
                    break;
                case 'P': case 'p':
                    if (teach_count > 0) {
                        mode = MODE_PLAYBACK;
                        UART_Println("[CMD] -> PLAYBACK mode");
                    }
                    break;
                case 'I': case 'i':
                    mode = MODE_IK_CONTROL;
                    TEACH_LED_ON();
                    UART_Println("[CMD] -> IK mode");
                    break;
                case 'h': case 'H':
                    UART_Println("\r\n=== Help ===");
                    UART_Println("T - Teach mode");
                    UART_Println("P - Playback");
                    UART_Println("I - IK control");
                    UART_Println("R - Record point");
                    UART_Println("G - Gripper toggle");
                    UART_Println("S - Show status");
                    UART_Println("C - Clear teach data");
                    break;
                case 's': case 'S':
                    PrintAngles("Angles: ", current_angles);
                    {
                        Vec3_t pos = ForwardKinematics(current_angles);
                        UART_Print("Position: X=");
                        UART_PrintNum((int32_t)pos.x);
                        UART_Print(" Y=");
                        UART_PrintNum((int32_t)pos.y);
                        UART_Print(" Z=");
                        UART_PrintNum((int32_t)pos.z);
                        UART_Print("\r\n");
                    }
                    UART_Print("Teach points: ");
                    UART_PrintNum(teach_count);
                    UART_Print("\r\n");
                    break;
                case 'c': case 'C':
                    teach_count = 0;
                    SaveTeachData();
                    UART_Println("[CMD] Teach data cleared");
                    break;
                default: break;
                }
            }
        }

        /* 低速主循环 */
        delay_ms(20);
        tick++;
    }

    return 0;
}
