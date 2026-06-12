# 有限状态机（FSM）使用文档

## 1. 设计原理

有限状态机是一种计算模型，由一组有限的状态、一组输入事件和一组状态转换规则组成。

### 1.1 核心概念

- **状态（State）**: 系统在特定时刻的状况
- **事件（Event）**: 触发状态转换的输入
- **转换（Transition）**: 从一个状态到另一个状态的变化
- **动作（Action）**: 状态转换时执行的操作

### 1.2 状态转换图

```
     ┌─────────┐
     │  IDLE   │
     └────┬────┘
          │ START
          ▼
     ┌─────────┐
     │  INIT   │
     └────┬────┘
          │ INIT_COMPLETE
          ▼
     ┌─────────┐
     │ RUNNING │◄────┐
     └────┬────┘     │
          │          │ RESUME
          ▼          │
     ┌─────────┐     │
     │ STOPPED │─────┘
     └────┬────┘
          │ ERROR
          ▼
     ┌─────────┐
     │  ERROR  │
     └─────────┘
```

### 1.3 状态转换表

| 当前状态 | 事件 | 下一状态 | 动作 |
|---------|------|---------|------|
| IDLE | START | INIT | 初始化硬件 |
| INIT | INIT_COMPLETE | RUNNING | 开始运行 |
| RUNNING | STOP | STOPPED | 停止运动 |
| RUNNING | ERROR | ERROR | 错误处理 |
| STOPPED | RESUME | RUNNING | 恢复运行 |
| STOPPED | ERROR | ERROR | 错误处理 |
| ERROR | RESET | IDLE | 复位系统 |

## 2. 使用步骤

### 2.1 定义状态和事件
```c
// 根据实际需求修改状态枚举
typedef enum {
    STATE_IDLE = 0,
    STATE_INIT,
    STATE_RUNNING,
    STATE_STOPPED,
    STATE_ERROR,
    STATE_COUNT
} SystemState;

// 根据实际需求修改事件枚举
typedef enum {
    EVENT_NONE = 0,
    EVENT_START,
    EVENT_STOP,
    EVENT_ERROR,
    EVENT_RESET,
    EVENT_COUNT
} SystemEvent;
```

### 2.2 实现状态处理函数
```c
void IdleState_Handler(void)
{
    // 空闲状态处理
    // 例如：等待启动命令
}

void InitState_Handler(void)
{
    // 初始化状态处理
    // 例如：初始化硬件、校准传感器
    static bool initialized = false;
    
    if (!initialized) {
        // 执行初始化
        Hardware_Init();
        Sensor_Calibrate();
        initialized = true;
    }
    
    // 初始化完成后触发事件
    if (initialized) {
        StateMachine_SetEvent(&hfsm, EVENT_INIT_COMPLETE);
    }
}

void RunningState_Handler(void)
{
    // 运行状态处理
    // 例如：执行主任务
    MainTask_Execute();
}

void StoppedState_Handler(void)
{
    // 停止状态处理
    // 例如：停止所有运动
    Motor_Stop();
}

void ErrorState_Handler(void)
{
    // 错误状态处理
    // 例如：显示错误信息、安全处理
    LED_Blink(ERROR_LED);
    Motor_Stop();
}
```

### 2.3 实现事件处理函数
```c
void OnStartEvent(void)
{
    // 启动事件处理
    printf("System starting...\n");
}

void OnStopEvent(void)
{
    // 停止事件处理
    printf("System stopping...\n");
    Motor_Stop();
}

void OnErrorEvent(void)
{
    // 错误事件处理
    printf("Error occurred!\n");
    Motor_Stop();
    LED_On(ERROR_LED);
}

void OnResetEvent(void)
{
    // 复位事件处理
    printf("System resetting...\n");
    System_Reset();
}
```

### 2.4 初始化状态机
```c
StateMachine_HandleTypeDef hfsm;

void main()
{
    // 初始化状态机
    StateMachine_Init(&hfsm);
    
    // 注册状态处理函数
    StateMachine_RegisterState(&hfsm, STATE_IDLE, IdleState_Handler);
    StateMachine_RegisterState(&hfsm, STATE_INIT, InitState_Handler);
    StateMachine_RegisterState(&hfsm, STATE_RUNNING, RunningState_Handler);
    StateMachine_RegisterState(&hfsm, STATE_STOPPED, StoppedState_Handler);
    StateMachine_RegisterState(&hfsm, STATE_ERROR, ErrorState_Handler);
    
    // 注册状态转换
    StateMachine_RegisterTransition(&hfsm, STATE_IDLE, EVENT_START, STATE_INIT, OnStartEvent);
    StateMachine_RegisterTransition(&hfsm, STATE_INIT, EVENT_INIT_COMPLETE, STATE_RUNNING, NULL);
    StateMachine_RegisterTransition(&hfsm, STATE_RUNNING, EVENT_STOP, STATE_STOPPED, OnStopEvent);
    StateMachine_RegisterTransition(&hfsm, STATE_RUNNING, EVENT_ERROR, STATE_ERROR, OnErrorEvent);
    StateMachine_RegisterTransition(&hfsm, STATE_STOPPED, EVENT_RESUME, STATE_RUNNING, NULL);
    StateMachine_RegisterTransition(&hfsm, STATE_STOPPED, EVENT_ERROR, STATE_ERROR, OnErrorEvent);
    StateMachine_RegisterTransition(&hfsm, STATE_ERROR, EVENT_RESET, STATE_IDLE, OnResetEvent);
    
    // 主循环
    while (1) {
        // 检查输入事件
        CheckInputEvents();
        
        // 运行状态机
        StateMachine_Run(&hfsm);
    }
}
```

## 3. 实际应用示例

### 3.1 RoboMaster比赛策略
```c
// 定义比赛状态
typedef enum {
    MATCH_IDLE,         // 比赛空闲
    MATCH_PREPARING,    // 比赛准备
    MATCH_RUNNING,      // 比赛进行中
    MATCH_PAUSED,       // 比赛暂停
    MATCH_ENDED         // 比赛结束
} MatchState;

// 定义比赛事件
typedef enum {
    MATCH_EVENT_START,      // 比赛开始
    MATCH_EVENT_PAUSE,      // 比赛暂停
    MATCH_EVENT_RESUME,     // 比赛恢复
    MATCH_EVENT_END,        // 比赛结束
    MATCH_EVENT_RESET       // 比赛复位
} MatchEvent;

// 比赛运行状态处理
void MatchRunning_Handler(void)
{
    // 执行比赛策略
    Strategy_Execute();
    
    // 检查比赛时间
    if (GetMatchTime() >= MATCH_DURATION) {
        StateMachine_SetEvent(&hmatch, MATCH_EVENT_END);
    }
}
```

### 3.2 飞思卡尔智能车状态机
```c
// 定义小车状态
typedef enum {
    CAR_IDLE,           // 空闲
    CAR_CALIBRATING,    // 校准中
    CAR_RUNNING,        // 运行中
    CAR_OBSTACLE,       // 避障中
    CAR_PARKING         // 停车中
} CarState;

// 小车运行状态处理
void CarRunning_Handler(void)
{
    // 车道线检测
    LaneDetectionResult *lane = LaneDetector_GetResult(&hlane);
    
    // 转向控制
    if (lane->valid) {
        float steering = PID_Compute(&hpid, 0, lane->offset);
        Motor_SetSteering(steering);
    }
    
    // 速度控制
    Motor_SetSpeed(CRUISE_SPEED);
    
    // 检查障碍物
    if (Ultrasonic_GetDistance() < OBSTACLE_DISTANCE) {
        StateMachine_SetEvent(&hcar, CAR_EVENT_OBSTACLE);
    }
}
```

## 4. 高级功能

### 4.1 超时检测
```c
void CheckTimeout(StateMachine_HandleTypeDef *hfsm, uint32_t timeout_ms)
{
    if (StateMachine_GetStateDuration(hfsm) > timeout_ms) {
        StateMachine_SetEvent(hfsm, EVENT_TIMEOUT);
    }
}
```

### 4.2 状态历史记录
```c
#define HISTORY_SIZE 10

typedef struct {
    SystemState states[HISTORY_SIZE];
    uint8_t count;
} StateHistory;

void RecordState(StateHistory *history, SystemState state)
{
    if (history->count < HISTORY_SIZE) {
        history->states[history->count++] = state;
    } else {
        // 移动历史记录
        for (uint8_t i = 0; i < HISTORY_SIZE - 1; i++) {
            history->states[i] = history->states[i + 1];
        }
        history->states[HISTORY_SIZE - 1] = state;
    }
}
```

### 4.3 并行状态机
```c
// 使用多个状态机处理不同任务
StateMachine_HandleTypeDef hmotion;    // 运动控制状态机
StateMachine_HandleTypeDef hweapon;    // 武器控制状态机
StateMachine_HandleTypeDef hcomm;      // 通信状态机

void main()
{
    // 初始化各个状态机
    StateMachine_Init(&hmotion);
    StateMachine_Init(&hweapon);
    StateMachine_Init(&hcomm);
    
    while (1) {
        StateMachine_Run(&hmotion);
        StateMachine_Run(&hweapon);
        StateMachine_Run(&hcomm);
    }
}
```

## 5. 调试技巧

### 5.1 状态转换日志
```c
void LogStateTransition(SystemState from, SystemState to, SystemEvent event)
{
    printf("[%lu] State: %d -> %d, Event: %d\n", HAL_GetTick(), from, to, event);
}
```

### 5.2 状态可视化
```c
void DisplayState(SystemState state)
{
    switch (state) {
        case STATE_IDLE:    LED_SetColor(GREEN); break;
        case STATE_INIT:    LED_SetColor(YELLOW); break;
        case STATE_RUNNING: LED_SetColor(BLUE); break;
        case STATE_STOPPED: LED_SetColor(RED); break;
        case STATE_ERROR:   LED_Blink(RED); break;
    }
}
```

## 6. 常见问题

### 6.1 状态机卡死
- 检查是否有未处理的事件
- 检查状态处理函数是否有阻塞操作
- 添加超时机制

### 6.2 状态转换混乱
- 检查状态转换表是否正确
- 确保事件处理函数不会触发新的事件
- 添加状态转换日志

### 6.3 内存不足
- 减少状态数量
- 使用位域存储状态
- 优化事件处理函数
