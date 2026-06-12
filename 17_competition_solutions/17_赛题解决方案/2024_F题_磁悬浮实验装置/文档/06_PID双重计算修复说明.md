# 2024年F题 磁悬浮实验装置 - PID双重计算修复说明

## 问题描述

main.c中存在两处PID计算：
1. **StateMachine_Run() → STATE_SUSPENDING分支**（第212行）：在主循环(20Hz)中调用PID_Calculate
2. **HAL_TIM_PeriodElapsedCallback()**（第325行）：在TIM2中断(1kHz)中调用PID_Calculate

两处使用同一个`pid_controller`实例，并发修改`integral`/`prev_error`/`output`，导致：
- 积分项被双重累加
- 输出被互相覆盖
- 控制行为完全不可预测

## 修复方案

**原则：PID只在中断中执行，主循环只做状态管理和显示**

### 修复后的main.c关键代码：

```c
// ===== HAL_TIM_PeriodElapsedCallback（保留） =====
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)  // 1kHz
    {
        // 读取传感器
        uint16_t adc_values[4];
        ADC_ReadAll(adc_values);
        g_actual_height = Height_Calculate(adc_values);
        
        // 只在悬浮状态下执行PID
        if (g_state == STATE_SUSPENDING)
        {
            float pid_output = PID_Calculate(&pid_controller, 
                                              g_set_height, 
                                              g_actual_height);
            if (pid_output < 0) pid_output = 0;
            if (pid_output > PWM_MAX_DUTY) pid_output = PWM_MAX_DUTY;
            g_pwm_output = (uint16_t)pid_output;
            PWM_SetDuty(g_pwm_output);
        }
    }
}

// ===== StateMachine_Run（修复后） =====
// STATE_SUSPENDING分支：只做状态监控，不执行PID
case STATE_SUSPENDING:
    g_suspend_time++;
    // 检查是否掉落
    if (g_actual_height < 0.5f)
    {
        g_state = STATE_IDLE;
        PWM_SetDuty(0);
    }
    break;
```

## 验证方法

1. 编译后烧录，启动悬浮
2. 观察串口输出的PID输出值是否稳定
3. 加载20g重物，观察恢复时间
4. 检查悬浮盘是否出现异常振荡
