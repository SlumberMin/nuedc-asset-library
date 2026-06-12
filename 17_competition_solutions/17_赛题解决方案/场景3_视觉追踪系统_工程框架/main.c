/**
 * @file    main.c
 * @brief   场景3: 视觉追踪系统 (2025 E题) — STM32工程框架
 * @details 舵机云台 + 视觉通信UART + 串级PID + OLED显示
 * @date    2026-06-11
 */

/* ========================================================================== */
/*                              头文件包含                                     */
/* ========================================================================== */
#include "platform/hal_stm32.h"
#include "drivers/servo.h"
#include "drivers/oled.h"
#include "drivers/key.h"
#include "algorithm/pid.h"

/* ========================================================================== */
/*                           外设句柄 (CubeMX生成)                             */
/* ========================================================================== */
extern TIM_HandleTypeDef htim2;   /* 舵机PWM (50Hz) */
extern TIM_HandleTypeDef htim3;   /* 备用PWM */
extern I2C_HandleTypeDef hi2c1;   /* OLED */
extern UART_HandleTypeDef huart1; /* 与视觉模块通信 */
extern UART_HandleTypeDef huart2; /* 调试串口 */

/* ========================================================================== */
/*                              全局变量                                       */
/* ========================================================================== */

/* --- 舵机 --- */
Servo_t servo_pan;          /* 水平旋转舵机 (Pan) */
Servo_t servo_tilt;         /* 垂直俯仰舵机 (Tilt) */

/* --- PID控制器 (串级控制) --- */
/* 水平方向 (Pan) */
PID_t pid_pan_outer;        /* 位置环 */
PID_t pid_pan_inner;        /* 速度环 */

/* 垂直方向 (Tilt) */
PID_t pid_tilt_outer;       /* 位置环 */
PID_t pid_tilt_inner;       /* 速度环 */

/* --- OLED --- */
OLED_t oled;

/* --- 按键 --- */
KeyMgr_t key_mgr;

/* --- 视觉数据 --- */
typedef struct {
    int16_t target_x;        /* 目标X坐标 (像素, 画面中心=0) */
    int16_t target_y;        /* 目标Y坐标 (像素, 画面中心=0) */
    uint8_t target_found;    /* 是否检测到目标 */
    uint32_t timestamp;      /* 数据时间戳 */
} VisionData_t;

volatile VisionData_t vision = {0, 0, 0, 0};

/* --- 系统状态 --- */
typedef enum {
    TRACK_IDLE = 0,
    TRACK_AUTO,              /* 自动追踪 */
    TRACK_MANUAL,            /* 手动控制 */
    TRACK_CALIBRATE,         /* 校准中心 */
} TrackState_t;

volatile TrackState_t track_state = TRACK_IDLE;

/* --- 通信协议 --- */
#define FRAME_HEADER  0xAA
#define FRAME_TAIL    0x55
typedef struct __packed {
    uint8_t  header;          /* 0xAA */
    int16_t  target_x;        /* -320~+320 (640宽画面) */
    int16_t  target_y;        /* -240~+240 (480高画面) */
    uint8_t  confidence;      /* 0~100 置信度 */
    uint8_t  tail;            /* 0x55 */
} VisionFrame_t;

static uint8_t rx_buf[32];   /* UART接收缓冲 */
static uint8_t rx_idx = 0;

/* ========================================================================== */
/*                           CubeMX配置清单                                   */
/* ========================================================================== */
/**
 * 1. TIM2: PWM Generation CH1 + CH2 (50Hz舵机PWM)
 *    - Channel1: PWM Generation → Pan舵机
 *    - Channel2: PWM Generation → Tilt舵机
 *    - Prescaler: 71 (72MHz/72=1MHz)
 *    - Period: 19999 (1MHz/20000=50Hz)
 *    - Pulse: 1500 (初始1500us=90度居中)
 *    - 引脚: PA0(TIM2_CH1) → Pan, PA1(TIM2_CH2) → Tilt
 *
 * 2. USART1: 115200 8N1 → 与视觉模块通信
 *    - PA9(TX), PA10(RX)
 *    - 使能RXNE中断 (接收视觉数据)
 *
 * 3. USART2: 115200 8N1 → 调试串口
 *    - PA2(TX), PA3(RX)
 *
 * 4. I2C1: Fast Mode 400kHz → OLED
 *    - PB6(SCL), PB7(SDA)
 *
 * 5. GPIO:
 *    - PB12~PB14: 按键
 *    - PC13: 状态LED
 *
 * 6. NVIC:
 *    - USART1_IRQn: 优先级1 (视觉数据接收, 最高)
 *    - TIM2_IRQn: 优先级2
 *    - EXTI15_10_IRQn: 优先级3 (按键)
 */

/* ========================================================================== */
/*                           通信协议解析                                      */
/* ========================================================================== */

/**
 * @brief 解析视觉模块发来的数据帧
 *        协议: [0xAA][X_H][X_L][Y_H][Y_L][Conf][0x55]
 */
static void Vision_ParseByte(uint8_t byte)
{
    static VisionFrame_t frame;
    static uint8_t parse_state = 0;

    switch (parse_state) {
    case 0: /* 等待帧头 */
        if (byte == FRAME_HEADER) {
            frame.header = byte;
            parse_state = 1;
            rx_idx = 0;
        }
        break;
    case 1: /* 接收数据 */
        rx_buf[rx_idx++] = byte;
        if (rx_idx >= 5) {
            parse_state = 2;
        }
        break;
    case 2: /* 验证帧尾 */
        if (byte == FRAME_TAIL) {
            frame.target_x = (int16_t)((rx_buf[0] << 8) | rx_buf[1]);
            frame.target_y = (int16_t)((rx_buf[2] << 8) | rx_buf[3]);
            frame.confidence = rx_buf[4];
            frame.timestamp = HAL_GetTick();

            /* 更新全局数据 */
            vision.target_x = frame.target_x;
            vision.target_y = frame.target_y;
            vision.target_found = (frame.confidence > 30);
            vision.timestamp = frame.timestamp;
        }
        parse_state = 0;
        break;
    }
}

/**
 * @brief UART接收中断回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint8_t byte;
        HAL_UART_Receive_IT(&huart1, &byte, 1);
        Vision_ParseByte(byte);
    }
}

/* ========================================================================== */
/*                           初始化函数                                       */
/* ========================================================================== */

static void App_Init(void)
{
    /* ---- 舵机初始化 ---- */
    Servo_Init(&servo_pan, &htim2, TIM_CHANNEL_1, SERVO_TYPE_SG90);
    Servo_Init(&servo_tilt, &htim2, TIM_CHANNEL_2, SERVO_TYPE_SG90);
    Servo_Center(&servo_pan);
    Servo_Center(&servo_tilt);

    /* ---- 水平方向串级PID ---- */
    /* 外环: 位置环 (像素误差 → 速度指令) */
    PID_Init(&pid_pan_outer, PID_MODE_POSITION,
             0.05f,    /* Kp: 像素误差到速度指令的增益 */
             0.001f,   /* Ki: 消除稳态偏差 */
             0.01f,    /* Kd: 抑制过冲 */
             0.033f);  /* dt = 33ms (30Hz视觉帧率) */
    PID_SetOutputLimit(&pid_pan_outer, -50.0f, 50.0f);  /* 速度指令限幅 */
    PID_SetTarget(&pid_pan_outer, 0.0f);  /* 目标: 画面中心 */

    /* 内环: 速度环 (速度指令 → 舵机角度增量) */
    PID_Init(&pid_pan_inner, PID_MODE_INCREMENTAL,
             2.0f,     /* Kp */
             0.0f,     /* Ki */
             0.5f,     /* Kd */
             0.010f);  /* dt = 10ms (100Hz控制) */
    PID_SetOutputLimit(&pid_pan_inner, -5.0f, 5.0f);  /* 角度增量限幅/周期 */
    PID_SetTarget(&pid_pan_inner, 0.0f);

    /* ---- 垂直方向串级PID ---- */
    PID_Init(&pid_tilt_outer, PID_MODE_POSITION,
             0.05f, 0.001f, 0.01f, 0.033f);
    PID_SetOutputLimit(&pid_tilt_outer, -50.0f, 50.0f);
    PID_SetTarget(&pid_tilt_outer, 0.0f);

    PID_Init(&pid_tilt_inner, PID_MODE_INCREMENTAL,
             2.0f, 0.0f, 0.5f, 0.010f);
    PID_SetOutputLimit(&pid_tilt_inner, -5.0f, 5.0f);
    PID_SetTarget(&pid_tilt_inner, 0.0f);

    /* ---- OLED初始化 ---- */
    OLED_Init(&oled, &hi2c1, OLED_I2C_ADDR);

    /* ---- 按键初始化 ---- */
    KeyMgr_Init(&key_mgr);
    KeyMgr_Add(&key_mgr, GPIOB, GPIO_PIN_12, KEY_ACTIVE_LOW);
    KeyMgr_Add(&key_mgr, GPIOB, GPIO_PIN_13, KEY_ACTIVE_LOW);
    KeyMgr_Add(&key_mgr, GPIOB, GPIO_PIN_14, KEY_ACTIVE_LOW);

    /* ---- 启动舵机PWM ---- */
    PWM_START(&htim2, TIM_CHANNEL_1);
    PWM_START(&htim2, TIM_CHANNEL_2);

    /* ---- 启动UART接收中断 ---- */
    uint8_t dummy;
    HAL_UART_Receive_IT(&huart1, &dummy, 1);

    /* ---- 开机画面 ---- */
    OLED_Clear(&oled);
    OLED_ShowString(&oled, "VisionTrack v1", 0, 0);
    OLED_ShowString(&oled, "NUEDC 2025 E", 0, 1);
    OLED_ShowString(&oled, "Waiting...", 0, 3);
    OLED_Refresh(&oled);

    DBG_PRINTF("VisionTrack App_Init complete\r\n");
}

/* ========================================================================== */
/*                           控制任务                                          */
/* ========================================================================== */

/**
 * @brief 100Hz控制任务: 串级PID (内环)
 */
static void Task_Control_100Hz(void)
{
    if (track_state != TRACK_AUTO) return;

    /* ---- 水平方向 (Pan) ---- */
    /* 内环输入 = 外环输出 (速度指令) */
    float pan_velocity_cmd = PID_Calculate(&pid_pan_outer, (float)vision.target_x);
    float pan_angle_delta  = PID_Calculate(&pid_pan_inner, pan_velocity_cmd);

    /* 累加到当前角度 */
    float pan_angle = Servo_GetAngle(&servo_pan) + pan_angle_delta;
    pan_angle = CLAMP(pan_angle, 0.0f, 180.0f);
    Servo_SetAngle(&servo_pan, pan_angle);

    /* ---- 垂直方向 (Tilt) ---- */
    float tilt_velocity_cmd = PID_Calculate(&pid_tilt_outer, (float)vision.target_y);
    float tilt_angle_delta  = PID_Calculate(&pid_tilt_inner, tilt_velocity_cmd);

    float tilt_angle = Servo_GetAngle(&servo_tilt) + tilt_angle_delta;
    tilt_angle = CLAMP(tilt_angle, 0.0f, 180.0f);
    Servo_SetAngle(&servo_tilt, tilt_angle);
}

/**
 * @brief 50ms显示更新
 */
static void Task_Display_50ms(void)
{
    OLED_Clear(&oled);

    /* 行0: 状态 */
    switch (track_state) {
    case TRACK_IDLE:     OLED_ShowString(&oled, "IDLE", 0, 0); break;
    case TRACK_AUTO:     OLED_ShowString(&oled, "AUTO", 0, 0); break;
    case TRACK_MANUAL:   OLED_ShowString(&oled, "MANUAL", 0, 0); break;
    case TRACK_CALIBRATE:OLED_ShowString(&oled, "CAL", 0, 0); break;
    }

    OLED_ShowString(&oled, vision.target_found ? "FOUND" : "LOST", 60, 0);

    /* 行1: 目标坐标 */
    OLED_ShowString(&oled, "X:", 0, 1);
    OLED_ShowInt(&oled, vision.target_x, 16, 1);
    OLED_ShowString(&oled, " Y:", 64, 1);
    OLED_ShowInt(&oled, vision.target_y, 80, 1);

    /* 行2: 舵机角度 */
    OLED_ShowString(&oled, "P:", 0, 2);
    OLED_ShowFloat(&oled, Servo_GetAngle(&servo_pan), 0, 16, 2);
    OLED_ShowString(&oled, "T:", 64, 2);
    OLED_ShowFloat(&oled, Servo_GetAngle(&servo_tilt), 0, 80, 2);

    /* 行3: 数据年龄 */
    uint32_t age = HAL_GetTick() - vision.timestamp;
    OLED_ShowString(&oled, "Age:", 0, 3);
    OLED_ShowInt(&oled, (int32_t)age, 30, 3);
    OLED_ShowString(&oled, "ms", 60, 3);

    OLED_Refresh(&oled);
}

/**
 * @brief 按键处理
 */
static void Key_Process(void)
{
    KeyEvent_t ev;

    /* K1: 切换自动/手动 */
    ev = Key_GetEvent(&key_mgr.keys[0]);
    if (ev == KEY_EVENT_PRESS) {
        if (track_state == TRACK_AUTO) {
            track_state = TRACK_IDLE;
        } else {
            PID_Reset(&pid_pan_outer);
            PID_Reset(&pid_pan_inner);
            PID_Reset(&pid_tilt_outer);
            PID_Reset(&pid_tilt_inner);
            track_state = TRACK_AUTO;
        }
    }

    /* K2: 校准中心 */
    ev = Key_GetEvent(&key_mgr.keys[1]);
    if (ev == KEY_EVENT_LONG_PRESS) {
        Servo_Center(&servo_pan);
        Servo_Center(&servo_tilt);
    }

    /* K3: 切换手动模式 */
    ev = Key_GetEvent(&key_mgr.keys[2]);
    if (ev == KEY_EVENT_PRESS) {
        if (track_state == TRACK_MANUAL) {
            track_state = TRACK_IDLE;
        } else {
            track_state = TRACK_MANUAL;
        }
    }
}

/* ========================================================================== */
/*                              主函数                                         */
/* ========================================================================== */

int main(void)
{
    /* 1. HAL/外设初始化 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();          /* 舵机PWM */
    MX_I2C1_Init();          /* OLED */
    MX_USART1_UART_Init();   /* 视觉通信 */
    MX_USART2_UART_Init();   /* 调试 */

    /* 2. 应用初始化 */
    App_Init();

    /* 3. 主循环 */
    uint32_t last_ctrl   = HAL_GetTick();
    uint32_t last_disp   = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 100Hz: 串级PID控制 */
        if (now - last_ctrl >= 10) {
            last_ctrl = now;
            Task_Control_100Hz();
        }

        /* 按键扫描 (10ms) */
        KeyMgr_Scan(&key_mgr);
        Key_Process();

        /* 50ms: 显示更新 */
        if (now - last_disp >= 50) {
            last_disp = now;
            Task_Display_50ms();
        }

        /* 超时保护: 500ms无视觉数据 → 停止追踪 */
        if (track_state == TRACK_AUTO) {
            if (now - vision.timestamp > 500) {
                vision.target_found = 0;
                /* 可选择保持最后位置或归中 */
            }
        }
    }
}
