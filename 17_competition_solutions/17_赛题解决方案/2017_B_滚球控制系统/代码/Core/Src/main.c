/**
 * @file    main.c
 * @brief   2017年B题 滚球控制系统 - 主程序
 * 
 * 功能：
 * 1. 电阻触摸屏检测小球位置(X,Y)
 * 2. 双轴级联PID控制平板倾斜
 * 3. 4舵机支撑平台，控制小球运动
 * 4. 支持多种路径模式和键盘设定
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>

/* 区域坐标定义（触摸屏坐标系，单位：mm） */
typedef struct {
    int16_t x;
    int16_t y;
} Point_t;

/* 9个区域的中心坐标 */
static const Point_t zone_pos[10] = {
    {0, 0},         // 占位
    {65, 65},       // 区域1
    {65, 260},      // 区域2
    {65, 455},      // 区域3
    {260, 65},      // 区域4
    {260, 260},     // 区域5（中心）
    {260, 455},     // 区域6
    {455, 65},      // 区域7
    {455, 260},     // 区域8
    {455, 455},     // 区域9
};

/* 系统状态 */
typedef enum {
    MODE_IDLE = 0,
    MODE_MOVE_TO_ZONE,      // 移动到指定区域
    MODE_PATH_SEQUENCE,     // 按序列移动
    MODE_CIRCLE_AROUND,     // 环绕运动
    MODE_KEYBOARD_PATH      // 键盘设定路径
} Mode_t;

/* 全局变量 */
volatile Mode_t g_mode = MODE_IDLE;
volatile int16_t g_ball_x = 0, g_ball_y = 0;    // 小球当前位置
volatile int16_t g_target_x = 260, g_target_y = 260;  // 目标位置
volatile uint8_t g_path_seq[8] = {0};           // 路径序列
volatile uint8_t g_path_len = 0;
volatile uint8_t g_path_index = 0;
volatile uint32_t g_timer_ms = 0;
volatile uint8_t g_stay_timer = 0;              // 停留计时

/* PID控制器 */
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
    float out_min, out_max;
} PID_t;

PID_t pid_x, pid_y;        // 位置PID
PID_t pid_angle_x, pid_angle_y;  // 角度PID

/* 舵机角度 */
volatile float g_servo_x1 = 90, g_servo_x2 = 90;
volatile float g_servo_y1 = 90, g_servo_y2 = 90;

/* 函数声明 */
void System_Init(void);
float PID_Calc(PID_t *pid, float target, float actual);
void Ball_GetPosition(int16_t *x, int16_t *y);
void Platform_SetAngle(float angle_x, float angle_y);
void Servo_Set(uint8_t ch, float angle);
uint8_t IsInZone(int16_t ball_x, int16_t ball_y, uint8_t zone);
void Control_Loop(void);

/**
 * @brief  主函数
 */
int main(void)
{
    System_Init();
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Ball Control v1.0");
    OLED_ShowString(0, 2, "Zone: Center(5)");
    OLED_ShowString(0, 4, "Ball: (0, 0)");
    OLED_ShowString(0, 6, "Mode: IDLE");
    
    while(1)
    {
        /* 键盘输入处理 */
        uint8_t key = Key_Scan();
        
        /* 模式选择 */
        if(key >= '1' && key <= '9')
        {
            g_mode = MODE_MOVE_TO_ZONE;
            g_target_x = zone_pos[key-'0'].x;
            g_target_y = zone_pos[key-'0'].y;
            g_stay_timer = 0;
        }
        
        /* 控制循环 */
        Control_Loop();
        
        /* 显示 */
        char buf[32];
        sprintf(buf, "Ball: (%d,%d)", g_ball_x, g_ball_y);
        OLED_ShowString(0, 4, buf);
        
        HAL_Delay(10);  // 10ms控制周期
    }
}

/**
 * @brief  控制主循环
 */
void Control_Loop(void)
{
    /* 1. 读取小球位置 */
    Ball_GetPosition((int16_t*)&g_ball_x, (int16_t*)&g_ball_y);
    
    /* 2. 位置PID计算目标角度 */
    float angle_x = PID_Calc(&pid_x, (float)g_target_x, (float)g_ball_x);
    float angle_y = PID_Calc(&pid_y, (float)g_target_y, (float)g_ball_y);
    
    /* 3. 角度限幅 */
    if(angle_x > 15.0f) angle_x = 15.0f;
    if(angle_x < -15.0f) angle_x = -15.0f;
    if(angle_y > 15.0f) angle_y = 15.0f;
    if(angle_y < -15.0f) angle_y = -15.0f;
    
    /* 4. 设置平台角度 */
    Platform_SetAngle(angle_x, angle_y);
    
    /* 5. 到达目标区域检测 */
    if(g_mode == MODE_MOVE_TO_ZONE)
    {
        /* 检查是否在目标区域 */
        uint8_t target_zone = 0;
        for(uint8_t i = 1; i <= 9; i++)
        {
            if(g_target_x == zone_pos[i].x && g_target_y == zone_pos[i].y)
            {
                target_zone = i;
                break;
            }
        }
        
        if(target_zone && IsInZone(g_ball_x, g_ball_y, target_zone))
        {
            g_stay_timer++;
            if(g_stay_timer > 200)  // 停留2s
            {
                Alert_Beep(100);    // 提示音
                g_mode = MODE_IDLE;
            }
        }
        else
        {
            g_stay_timer = 0;
        }
    }
}

/**
 * @brief  PID计算
 */
float PID_Calc(PID_t *pid, float target, float actual)
{
    float error = target - actual;
    pid->integral += error;
    
    /* 抗积分饱和 */
    if(pid->integral > pid->out_max) pid->integral = pid->out_max;
    if(pid->integral < pid->out_min) pid->integral = pid->out_min;
    
    float derivative = error - pid->prev_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    /* 输出限幅 */
    if(output > pid->out_max) output = pid->out_max;
    if(output < pid->out_min) output = pid->out_min;
    
    pid->prev_error = error;
    return output;
}

/**
 * @brief  设置平台倾斜角度
 * @param  angle_x: X轴倾斜角(-15°~+15°)
 * @param  angle_y: Y轴倾斜角(-15°~+15°)
 */
void Platform_SetAngle(float angle_x, float angle_y)
{
    /* 四舵机布局：
     *  Y1 ---- Y2
     *  |        |
     *  X1 ---- X2
     */
    float x1 = 90.0f - angle_x;    // 左侧
    float x2 = 90.0f + angle_x;    // 右侧
    float y1 = 90.0f - angle_y;    // 前侧
    float y2 = 90.0f + angle_y;    // 后侧
    
    Servo_Set(0, x1);
    Servo_Set(1, x2);
    Servo_Set(2, y1);
    Servo_Set(3, y2);
}

/**
 * @brief  判断小球是否在指定区域
 */
uint8_t IsInZone(int16_t bx, int16_t by, uint8_t zone)
{
    if(zone < 1 || zone > 9) return 0;
    int16_t dx = bx - zone_pos[zone].x;
    int16_t dy = by - zone_pos[zone].y;
    return (dx*dx + dy*dy < 225) ? 1 : 0;  // 半径15mm内
}

/**
 * @brief  读取触摸屏获取小球位置
 */
void Ball_GetPosition(int16_t *x, int16_t *y)
{
    /* 电阻触摸屏读取 */
    /* X方向：Y+接VCC，Y-接GND，读X+的ADC值 */
    /* Y方向：X+接VCC，X-接GND，读Y+的ADC值 */
    *x = (int16_t)ADC_ReadX();
    *y = (int16_t)ADC_ReadY();
}
