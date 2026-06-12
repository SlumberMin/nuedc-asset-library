/**
 * @file keypad_lock.c
 * @brief MSPM0G3507 密码锁示例 - CH455G键盘 + EEPROM密码存储 + 继电器控制
 * 
 * 硬件连接：
 *   CH455G I2C键盘控制器:
 *     SDA  -> PB9 (I2C0_SDA)
 *     SCL  -> PB8 (I2C0_SCL)
 *     INT  -> PB2 (外部中断, 按键按下时拉低)
 *     地址 -> 0x24 (CH455G默认地址)
 * 
 *   EEPROM (AT24C16):
 *     SDA  -> PB9 (与CH455G共用I2C总线)
 *     SCL  -> PB8
 *     地址 -> 0x50
 * 
 *   电磁锁/继电器:
 *     LOCK -> PA15 (GPIO输出, 高电平=开锁)
 *     BUZZER-> PA14 (GPIO输出, 蜂鸣器)
 * 
 *   LED指示:
 *     LED_GREEN -> PB14 (绿色LED, 开锁指示)
 *     LED_RED   -> PB15 (红色LED, 错误指示)
 * 
 * 功能说明：
 *   - 4x4矩阵键盘通过CH455G I2C扩展器读取
 *   - 支持6位数字密码输入，EEPROM持久存储
 *   - 3次错误密码锁定30秒
 *   - 支持密码修改 (需输入旧密码验证)
 *   - 蜂鸣器+LED声音/灯光提示
 * 
 * 适用场景：电赛中电子密码锁、门禁控制系统
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================
 * 第一部分：CH455G I2C键盘控制器驱动
 * ============================================================ */

/* CH455G I2C地址 */
#define CH455G_ADDR         0x24

/* CH455G命令字 */
#define CH455G_CMD_READ_KEY 0x0F    /* 读取按键值 */
#define CH455G_CMD_SET_SEG  0x01    /* 设置数码管段码 */

/* CH455G按键键值映射 (4x4键盘) */
/* 
 * 键盘布局:
 *   [1] [2] [3] [A]
 *   [4] [5] [6] [B]
 *   [7] [8] [9] [C]
 *   [*] [0] [#] [D]
 */
static const char KEY_MAP[16] = {
    '1', '4', '7', '*',    /* 第1列 */
    '2', '5', '8', '0',    /* 第2列 */
    '3', '6', '9', '#',    /* 第3列 */
    'A', 'B', 'C', 'D'     /* 第4列 */
};

/* ============================================================
 * 第二部分：EEPROM AT24C16驱动
 * ============================================================ */

/* AT24C16 I2C地址 */
#define EEPROM_ADDR         0x50

/* EEPROM存储地址 */
#define EEPROM_ADDR_PASSWORD    0x0000  /* 密码存储起始地址 (6字节) */
#define EEPROM_ADDR_ERROR_COUNT 0x0010  /* 错误计数存储地址 */
#define EEPROM_ADDR_MAGIC       0x0020  /* 魔数验证地址 */

/* 魔数 (用于判断EEPROM是否已初始化) */
#define EEPROM_MAGIC_VALUE  0xA5

/* ============================================================
 * 第三部分：密码锁参数
 * ============================================================ */

#define PASSWORD_LENGTH     6       /* 密码长度 */
#define MAX_ERROR_COUNT     3       /* 最大错误次数 */
#define LOCKOUT_TIME_MS     30000   /* 锁定时间(ms) = 30秒 */
#define UNLOCK_TIME_MS      5000    /* 开锁时间(ms) = 5秒 */
#define KEY_BEEP_TIME_MS    100     /* 按键蜂鸣时间 */

/* 默认密码 */
static const char DEFAULT_PASSWORD[PASSWORD_LENGTH] = {'1', '2', '3', '4', '5', '6'};

/* 锁状态 */
typedef enum {
    LOCK_IDLE,          /* 空闲等待输入 */
    LOCK_INPUTTING,     /* 正在输入密码 */
    LOCK_VERIFYING,     /* 验证密码中 */
    LOCK_UNLOCKED,      /* 已开锁 */
    LOCK_LOCKED_OUT,    /* 错误次数过多，已锁定 */
    LOCK_CHANGE_PWD,    /* 修改密码模式 */
    LOCK_CHANGE_PWD_NEW /* 输入新密码 */
} LockState_t;

/* ============================================================
 * 第四部分：全局变量
 * ============================================================ */

static volatile uint32_t g_systick_ms = 0;       /* 系统毫秒计数器 */
static volatile bool g_key_interrupt = false;     /* 按键中断标志 */

/* 锁状态 */
static LockState_t g_lock_state = LOCK_IDLE;
static char g_input_buffer[PASSWORD_LENGTH + 1];  /* 输入缓冲区 */
static uint8_t g_input_count = 0;                 /* 当前输入位数 */
static char g_stored_password[PASSWORD_LENGTH + 1]; /* 存储的密码 */
static uint8_t g_error_count = 0;                 /* 连续错误次数 */
static uint32_t g_lockout_start_time = 0;         /* 锁定开始时间 */
static uint32_t g_unlock_start_time = 0;          /* 开锁开始时间 */

/* 修改密码临时存储 */
static char g_old_password[PASSWORD_LENGTH + 1];  /* 旧密码 */
static char g_new_password[PASSWORD_LENGTH + 1];  /* 新密码 */
static uint8_t g_change_step = 0;                 /* 修改步骤: 0=输入旧密码, 1=输入新密码, 2=确认新密码 */

/* ============================================================
 * 第五部分：底层I2C操作
 * ============================================================ */

/**
 * @brief 延时毫秒
 */
static void delay_ms(uint32_t ms)
{
    while (ms--) {
        for (volatile int i = 0; i < 8000; i++) __NOP();
    }
}

/**
 * @brief 延时微秒
 */
static void delay_us(uint32_t us)
{
    while (us--) {
        for (volatile int i = 0; i < 8; i++) __NOP();
    }
}

/**
 * @brief I2C写一个字节
 * @param addr 设备地址
 * @param data 要写入的字节
 * @return true=成功
 */
static bool i2c_write_byte(uint8_t addr, uint8_t data)
{
    DL_I2C_startTransfer(I2C0);
    DL_I2C_transmitData(I2C0, (addr << 1) | 0); /* 写地址 */
    while (!DL_I2C_isTXEmpty(I2C0)) ;
    
    DL_I2C_transmitData(I2C0, data);
    while (!DL_I2C_isTXEmpty(I2C0)) ;
    
    DL_I2C_stopTransfer(I2C0);
    delay_us(100);
    return true;
}

/**
 * @brief I2C读一个字节
 * @param addr 设备地址
 * @param data 读取的数据
 * @return true=成功
 */
static bool i2c_read_byte(uint8_t addr, uint8_t *data)
{
    DL_I2C_startTransfer(I2C0);
    DL_I2C_transmitData(I2C0, (addr << 1) | 1); /* 读地址 */
    while (!DL_I2C_isRXFull(I2C0)) ;
    
    *data = DL_I2C_receiveData(I2C0);
    DL_I2C_stopTransfer(I2C0);
    
    return true;
}

/* ============================================================
 * 第六部分：CH455G键盘驱动
 * ============================================================ */

/**
 * @brief 读取CH455G按键值
 * @param key_value 按键键值
 * @return true=有按键按下
 */
static bool ch455g_read_key(uint8_t *key_value)
{
    uint8_t data;
    
    if (!i2c_read_byte(CH455G_ADDR, &data)) {
        return false;
    }
    
    /* CH455G返回的键值格式: 位7=按下标志, 位3~0=键索引 */
    if (data & 0x80) {
        *key_value = data & 0x0F;
        return true;
    }
    
    return false;
}

/**
 * @brief 将键值转换为字符
 * @param key_idx 键索引
 * @return 对应的字符
 */
static char key_to_char(uint8_t key_idx)
{
    if (key_idx < 16) {
        return KEY_MAP[key_idx];
    }
    return '\0';
}

/* ============================================================
 * 第七部分：EEPROM驱动
 * ============================================================ */

/**
 * @brief 写入EEPROM一个字节
 * @param mem_addr 内存地址 (0~2047)
 * @param data 要写入的数据
 * @return true=写入成功
 */
static bool eeprom_write_byte(uint16_t mem_addr, uint8_t data)
{
    /* AT24C16地址格式: 设备地址 = 0x50 | (A10:A8) */
    uint8_t dev_addr = EEPROM_ADDR | ((mem_addr >> 8) & 0x07);
    uint8_t word_addr = (uint8_t)(mem_addr & 0xFF);
    
    DL_I2C_startTransfer(I2C0);
    DL_I2C_transmitData(I2C0, (dev_addr << 1) | 0);
    while (!DL_I2C_isTXEmpty(I2C0)) ;
    
    DL_I2C_transmitData(I2C0, word_addr);
    while (!DL_I2C_isTXEmpty(I2C0)) ;
    
    DL_I2C_transmitData(I2C0, data);
    while (!DL_I2C_isTXEmpty(I2C0)) ;
    
    DL_I2C_stopTransfer(I2C0);
    
    /* AT24C16写入周期: 最大5ms */
    delay_ms(5);
    
    return true;
}

/**
 * @brief 读取EEPROM一个字节
 * @param mem_addr 内存地址
 * @param data 读取的数据
 * @return true=读取成功
 */
static bool eeprom_read_byte(uint16_t mem_addr, uint8_t *data)
{
    uint8_t dev_addr = EEPROM_ADDR | ((mem_addr >> 8) & 0x07);
    uint8_t word_addr = (uint8_t)(mem_addr & 0xFF);
    
    /* 写入地址 */
    DL_I2C_startTransfer(I2C0);
    DL_I2C_transmitData(I2C0, (dev_addr << 1) | 0);
    while (!DL_I2C_isTXEmpty(I2C0)) ;
    
    DL_I2C_transmitData(I2C0, word_addr);
    while (!DL_I2C_isTXEmpty(I2C0)) ;
    
    /* 重复开始，读取数据 */
    DL_I2C_startTransfer(I2C0);
    DL_I2C_transmitData(I2C0, (dev_addr << 1) | 1);
    while (!DL_I2C_isRXFull(I2C0)) ;
    
    *data = DL_I2C_receiveData(I2C0);
    DL_I2C_stopTransfer(I2C0);
    
    return true;
}

/**
 * @brief 写入EEPROM多个字节
 * @param mem_addr 起始地址
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return true=写入成功
 */
static bool eeprom_write(uint16_t mem_addr, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (!eeprom_write_byte(mem_addr + i, data[i])) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 读取EEPROM多个字节
 * @param mem_addr 起始地址
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return true=读取成功
 */
static bool eeprom_read(uint16_t mem_addr, uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (!eeprom_read_byte(mem_addr + i, &data[i])) {
            return false;
        }
    }
    return true;
}

/* ============================================================
 * 第八部分：密码管理功能
 * ============================================================ */

/**
 * @brief 从EEPROM加载密码
 * @return true=加载成功 (EEPROM已初始化)
 */
static bool load_password(void)
{
    uint8_t magic;
    
    /* 读取魔数 */
    if (!eeprom_read_byte(EEPROM_ADDR_MAGIC, &magic)) {
        return false;
    }
    
    if (magic != EEPROM_MAGIC_VALUE) {
        /* EEPROM未初始化，使用默认密码 */
        memcpy(g_stored_password, DEFAULT_PASSWORD, PASSWORD_LENGTH);
        g_stored_password[PASSWORD_LENGTH] = '\0';
        
        /* 保存默认密码到EEPROM */
        save_password();
        
        /* 写入魔数 */
        eeprom_write_byte(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_VALUE);
        
        /* 初始化错误计数 */
        g_error_count = 0;
        eeprom_write_byte(EEPROM_ADDR_ERROR_COUNT, 0);
        
        return true;
    }
    
    /* 读取存储的密码 */
    if (!eeprom_read(EEPROM_ADDR_PASSWORD, (uint8_t *)g_stored_password, PASSWORD_LENGTH)) {
        return false;
    }
    g_stored_password[PASSWORD_LENGTH] = '\0';
    
    /* 读取错误计数 */
    eeprom_read_byte(EEPROM_ADDR_ERROR_COUNT, &g_error_count);
    
    return true;
}

/**
 * @brief 保存密码到EEPROM
 * @return true=保存成功
 */
static bool save_password(void)
{
    return eeprom_write(EEPROM_ADDR_PASSWORD, (const uint8_t *)g_stored_password, PASSWORD_LENGTH);
}

/**
 * @brief 保存错误计数到EEPROM
 */
static void save_error_count(void)
{
    eeprom_write_byte(EEPROM_ADDR_ERROR_COUNT, g_error_count);
}

/**
 * @brief 验证密码
 * @param input 输入的密码
 * @return true=密码正确
 */
static bool verify_password(const char *input)
{
    for (uint8_t i = 0; i < PASSWORD_LENGTH; i++) {
        if (input[i] != g_stored_password[i]) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 修改密码
 * @param new_pwd 新密码
 * @return true=修改成功
 */
static bool change_password(const char *new_pwd)
{
    memcpy(g_stored_password, new_pwd, PASSWORD_LENGTH);
    g_stored_password[PASSWORD_LENGTH] = '\0';
    return save_password();
}

/* ============================================================
 * 第九部分：硬件控制功能
 * ============================================================ */

/**
 * @brief 蜂鸣器响
 * @param duration_ms 持续时间(ms)
 */
static void buzzer_beep(uint32_t duration_ms)
{
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14);
    delay_ms(duration_ms);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14);
}

/**
 * @brief 开锁
 */
static void unlock(void)
{
    /* 继电器开锁 */
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_15);
    
    /* 绿色LED亮 */
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14);
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_15);
    
    /* 蜂鸣器提示 */
    buzzer_beep(200);
    delay_ms(100);
    buzzer_beep(200);
    
    g_lock_state = LOCK_UNLOCKED;
    g_unlock_start_time = g_systick_ms;
}

/**
 * @brief 关锁
 */
static void lock(void)
{
    /* 继电器关锁 */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_15);
    
    /* 绿色LED灭 */
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14);
    
    g_lock_state = LOCK_IDLE;
}

/**
 * @brief 错误提示
 */
static void error_indicate(void)
{
    /* 红色LED闪烁 */
    for (uint8_t i = 0; i < 3; i++) {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_15);
        buzzer_beep(100);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_15);
        delay_ms(100);
    }
}

/**
 * @brief 锁定提示 (30秒锁定)
 */
static void lockout_indicate(void)
{
    /* 长鸣1秒 */
    buzzer_beep(1000);
    
    /* 红色LED持续亮 */
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_15);
}

/* ============================================================
 * 第十部分：按键处理状态机
 * ============================================================ */

/**
 * @brief 处理数字键 (0~9)
 * @param key 输入的字符
 */
static void handle_digit_key(char key)
{
    if (g_lock_state == LOCK_INPUTTING || 
        g_lock_state == LOCK_CHANGE_PWD ||
        g_lock_state == LOCK_CHANGE_PWD_NEW) {
        
        if (g_input_count < PASSWORD_LENGTH) {
            g_input_buffer[g_input_count] = key;
            g_input_count++;
            g_input_buffer[g_input_count] = '\0';
            
            /* 按键蜂鸣 */
            buzzer_beep(KEY_BEEP_TIME_MS);
        }
    }
}

/**
 * @brief 处理确认键 (#)
 */
static void handle_confirm_key(void)
{
    switch (g_lock_state) {
        case LOCK_INPUTTING:
            /* 验证密码 */
            if (verify_password(g_input_buffer)) {
                /* 密码正确 */
                g_error_count = 0;
                save_error_count();
                unlock();
            } else {
                /* 密码错误 */
                g_error_count++;
                save_error_count();
                error_indicate();
                
                if (g_error_count >= MAX_ERROR_COUNT) {
                    /* 达到最大错误次数，锁定 */
                    g_lockout_start_time = g_systick_ms;
                    g_lock_state = LOCK_LOCKED_OUT;
                    lockout_indicate();
                } else {
                    /* 重新输入 */
                    g_lock_state = LOCK_IDLE;
                }
            }
            break;
            
        case LOCK_CHANGE_PWD:
            /* 验证旧密码 */
            if (verify_password(g_input_buffer)) {
                /* 旧密码正确，进入新密码输入 */
                g_change_step = 1;
                g_lock_state = LOCK_CHANGE_PWD_NEW;
                g_input_count = 0;
                memset(g_input_buffer, 0, sizeof(g_input_buffer));
                
                /* 提示输入新密码 */
                buzzer_beep(300);
            } else {
                /* 旧密码错误 */
                error_indicate();
                g_lock_state = LOCK_IDLE;
            }
            break;
            
        case LOCK_CHANGE_PWD_NEW:
            if (g_change_step == 1) {
                /* 保存新密码 */
                memcpy(g_new_password, g_input_buffer, PASSWORD_LENGTH);
                g_new_password[PASSWORD_LENGTH] = '\0';
                
                /* 再次输入新密码确认 */
                g_change_step = 2;
                g_input_count = 0;
                memset(g_input_buffer, 0, sizeof(g_input_buffer));
                
                buzzer_beep(200);
                delay_ms(50);
                buzzer_beep(200);
            } else if (g_change_step == 2) {
                /* 确认新密码 */
                if (memcmp(g_input_buffer, g_new_password, PASSWORD_LENGTH) == 0) {
                    /* 密码匹配，保存新密码 */
                    change_password(g_new_password);
                    
                    /* 成功提示 */
                    buzzer_beep(500);
                    
                    g_lock_state = LOCK_IDLE;
                } else {
                    /* 两次输入不一致 */
                    error_indicate();
                    g_lock_state = LOCK_IDLE;
                }
            }
            break;
            
        default:
            break;
    }
    
    /* 清空输入缓冲区 */
    g_input_count = 0;
    memset(g_input_buffer, 0, sizeof(g_input_buffer));
}

/**
 * @brief 处理取消键 (*)
 */
static void handle_cancel_key(void)
{
    /* 清空输入 */
    g_input_count = 0;
    memset(g_input_buffer, 0, sizeof(g_input_buffer));
    
    /* 返回空闲状态 */
    g_lock_state = LOCK_IDLE;
    
    buzzer_beep(50);
}

/**
 * @brief 处理功能键 (A, B, C, D)
 * @param key 按键字符
 */
static void handle_function_key(char key)
{
    switch (key) {
        case 'A':
            /* A键: 进入密码输入模式 */
            if (g_lock_state == LOCK_IDLE) {
                g_lock_state = LOCK_INPUTTING;
                g_input_count = 0;
                memset(g_input_buffer, 0, sizeof(g_input_buffer));
                buzzer_beep(200);
            }
            break;
            
        case 'B':
            /* B键: 修改密码 (需要先开锁) */
            if (g_lock_state == LOCK_UNLOCKED) {
                g_lock_state = LOCK_CHANGE_PWD;
                g_change_step = 0;
                g_input_count = 0;
                memset(g_input_buffer, 0, sizeof(g_input_buffer));
                buzzer_beep(200);
                delay_ms(50);
                buzzer_beep(200);
            }
            break;
            
        case 'C':
            /* C键: 手动关锁 */
            if (g_lock_state == LOCK_UNLOCKED) {
                lock();
                buzzer_beep(100);
            }
            break;
            
        case 'D':
            /* D键: 重置 (测试用) */
            /* 生产环境中应禁用此功能 */
            g_error_count = 0;
            save_error_count();
            buzzer_beep(500);
            break;
            
        default:
            break;
    }
}

/**
 * @brief 处理按键事件
 * @param key_char 按键字符
 */
static void process_key_event(char key_char)
{
    /* 检查是否在锁定状态 */
    if (g_lock_state == LOCK_LOCKED_OUT) {
        /* 锁定期间不处理按键 */
        buzzer_beep(200);
        return;
    }
    
    /* 数字键 (0~9) */
    if (key_char >= '0' && key_char <= '9') {
        handle_digit_key(key_char);
        return;
    }
    
    /* 确认键 */
    if (key_char == '#') {
        handle_confirm_key();
        return;
    }
    
    /* 取消键 */
    if (key_char == '*') {
        handle_cancel_key();
        return;
    }
    
    /* 功能键 */
    handle_function_key(key_char);
}

/* ============================================================
 * 第十一部分：中断处理
 * ============================================================ */

/**
 * @brief GROUP1中断处理 (PB2 - CH455G INT)
 */
void GROUP1_IRQHandler(void)
{
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOB, DL_GPIO_PIN_2);
    
    if (flags & DL_GPIO_PIN_2) {
        g_key_interrupt = true;
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_2);
    }
}

/* ============================================================
 * 第十二部分：主函数
 * ============================================================ */

int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();
    
    /* 配置输出引脚 */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_15);  /* LOCK */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_14);  /* BUZZER */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_14);  /* LED_GREEN (PB14) */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_15);  /* LED_RED (PB15) */
    
    /* 初始状态: 关锁 */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_15);   /* 继电器关 */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14);   /* 蜂鸣器关 */
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14);   /* 绿LED灭 */
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_15);   /* 红LED灭 */
    
    /* ==================== 初始化EEPROM和密码 ==================== */
    delay_ms(100); /* 等待EEPROM就绪 */
    
    if (!load_password()) {
        /* EEPROM读取失败，使用默认密码 */
        memcpy(g_stored_password, DEFAULT_PASSWORD, PASSWORD_LENGTH);
        g_stored_password[PASSWORD_LENGTH] = '\0';
        g_error_count = 0;
        
        /* 错误指示 */
        while (1) {
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_15);
            delay_ms(200);
        }
    }
    
    /* 检查是否有未清除的错误计数 */
    if (g_error_count >= MAX_ERROR_COUNT) {
        /* 上次关机时处于锁定状态 */
        g_lockout_start_time = g_systick_ms;
        g_lock_state = LOCK_LOCKED_OUT;
        lockout_indicate();
    }
    
    /* ==================== 初始化完成 ==================== */
    /* LED快闪指示初始化成功 */
    for (uint8_t i = 0; i < 3; i++) {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14);
        delay_ms(100);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14);
        delay_ms(100);
    }
    
    /* ==================== 主循环 ==================== */
    uint32_t last_state_check = 0;
    
    while (1) {
        /* 处理按键中断 */
        if (g_key_interrupt) {
            g_key_interrupt = false;
            
            uint8_t key_value;
            if (ch455g_read_key(&key_value)) {
                char key_char = key_to_char(key_value);
                if (key_char != '\0') {
                    process_key_event(key_char);
                }
            }
        }
        
        /* 定时检查状态 */
        if ((g_systick_ms - last_state_check) >= 100) {
            last_state_check = g_systick_ms;
            
            /* 检查锁定超时 */
            if (g_lock_state == LOCK_LOCKED_OUT) {
                if ((g_systick_ms - g_lockout_start_time) >= LOCKOUT_TIME_MS) {
                    /* 锁定时间到，解除锁定 */
                    g_lock_state = LOCK_IDLE;
                    g_error_count = 0;
                    save_error_count();
                    
                    /* 恢复正常指示 */
                    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_15);
                    
                    /* 提示音 */
                    buzzer_beep(200);
                    delay_ms(100);
                    buzzer_beep(200);
                }
            }
            
            /* 检查开锁超时 */
            if (g_lock_state == LOCK_UNLOCKED) {
                if ((g_systick_ms - g_unlock_start_time) >= UNLOCK_TIME_MS) {
                    /* 开锁时间到，自动关锁 */
                    lock();
                    buzzer_beep(100);
                }
            }
        }
        
        /* 模拟系统时间递增 */
        delay_ms(1);
        g_systick_ms++;
    }
}
