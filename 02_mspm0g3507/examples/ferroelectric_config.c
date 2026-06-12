/**
 * @file ferroelectric_config.c
 * @brief 铁电存储配置管理 - FM24CL64 参数存储 + 版本控制
 * @platform MSPM0G3507
 *
 * 硬件连接：
 *   FM24CL64 (I2C):
 *     SDA  -> PA0 (I2C0_SDA)
 *     SCL  -> PA1 (I2C0_SCL)
 *     A0   -> GND (地址位0)
 *     A1   -> GND (地址位1)
 *     A2   -> GND (地址位2)
 *     WP   -> GND (写保护禁用)
 *     VCC  -> 3.3V
 *
 *   LED指示:
 *     PB0 -> 读写操作指示
 *     PB1 -> 错误指示
 *
 *   按键:
 *     PB2 -> 保存当前参数
 *     PB3 -> 恢复出厂默认
 *     PB4 -> 切换参数组
 *
 * 功能：铁电RAM存储系统参数，支持多组配置、版本控制、校验保护、掉电保存
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===== FM24CL64 驱动 ===== */
#define FM24CL64_ADDR       0x50   /* I2C基础地址 (A2=A1=A0=0) */
#define FM24CL64_SIZE       8192   /* 8KB = 64Kbit */
#define FM24CL64_PAGE_SIZE  64     /* 页大小 */

/* I2C超时 */
#define I2C_TIMEOUT  10000

/* ===== 配置存储布局 ===== */
#define CFG_MAGIC           0xCF640001  /* 配置魔数 */
#define CFG_VERSION         0x0001      /* 配置格式版本 */

/* 存储地址分配:
 * 0x0000 - 0x003F : 配置头(64字节)
 * 0x0040 - 0x01FF : 参数组0 (主配置, 448字节)
 * 0x0200 - 0x03FF : 参数组1 (备用配置, 512字节)
 * 0x0400 - 0x05FF : 参数组2 (校准数据, 512字节)
 * 0x0600 - 0x07FF : 参数组3 (用户设置, 512字节)
 * 0x0800 - 0x0FFF : 参数组4-7 (扩展, 2KB)
 * 0x1000 - 0x1FFF : 操作日志区(4KB)
 */

#define ADDR_HEADER     0x0000
#define ADDR_PARAM_BASE 0x0040
#define PARAM_GROUP_SIZE 0x01C0  /* 448字节/组 */
#define ADDR_LOG_BASE   0x1000
#define LOG_ENTRY_SIZE  16
#define MAX_LOG_ENTRIES  ((FM24CL64_SIZE - ADDR_LOG_BASE) / LOG_ENTRY_SIZE)  /* 256条 */

/* ===== 配置头结构 ===== */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;             /* 魔数 */
    uint16_t format_version;    /* 格式版本 */
    uint16_t data_version;      /* 数据版本(每次保存+1) */
    uint32_t param_count;       /* 参数总数 */
    uint32_t active_group;      /* 当前激活组 */
    uint32_t param_checksum;    /* 参数CRC校验 */
    uint8_t  device_id[16];     /* 设备标识 */
    uint8_t  reserved[24];      /* 保留 */
} ConfigHeader;  /* 64字节 */
#pragma pack(pop)

/* ===== 系统参数结构 ===== */
#pragma pack(push, 1)
typedef struct {
    /* 基本参数 */
    uint16_t pwm_frequency;       /* PWM频率 (Hz) */
    uint16_t pwm_duty_cycle;      /* PWM占空比 (0-10000 = 0.00%-100.00%) */
    uint16_t target_temperature;  /* 目标温度 (0.1°C) */
    uint16_t temp_hysteresis;     /* 温度回差 (0.1°C) */

    /* PID参数 (定点Q8.8) */
    int16_t pid_kp;               /* 比例系数 */
    int16_t pid_ki;               /* 积分系数 */
    int16_t pid_kd;               /* 微分系数 */
    int16_t pid_output_min;       /* 输出下限 */
    int16_t pid_output_max;       /* 输出上限 */

    /* ADC校准 */
    uint16_t adc_offset[4];       /* 4通道零偏 */
    uint16_t adc_gain[4];         /* 4通道增益 (Q1.15, 32768=1.0) */

    /* DAC校准 */
    uint16_t dac_offset[2];       /* 2通道零偏 */
    uint16_t dac_fullscale[2];    /* 2通道满量程 */

    /* 通信参数 */
    uint32_t uart_baudrate;       /* 串口波特率 */
    uint8_t  device_address;      /* 设备地址 */
    uint8_t  comm_protocol;       /* 通信协议 (0=自定义, 1=Modbus) */

    /* 系统设置 */
    uint8_t  display_brightness;  /* 显示亮度 (0-100) */
    uint8_t  backlight_timeout;   /* 背光超时 (秒, 0=常亮) */
    uint16_t alarm_threshold;     /* 报警阈值 */
    uint8_t  alarm_enable;        /* 报警使能位 */
    uint8_t  reserved[29];        /* 保留到448字节 */
} SystemParams;
#pragma pack(pop)

#define PARAM_MAGIC  0x5041524D  /* "PARM" */

/* ===== 操作日志结构 ===== */
#pragma pack(push, 1)
typedef struct {
    uint32_t timestamp;      /* 系统tick */
    uint8_t  operation;      /* 操作类型: 0=读, 1=写, 2=恢复默认, 3=校验错误 */
    uint8_t  param_group;    /* 参数组 */
    uint16_t data_version;   /* 操作时的版本号 */
    uint32_t checksum;       /* 操作后校验 */
    uint8_t  reserved[4];
} LogEntry;  /* 16字节 */
#pragma pack(pop)

/* ===== 全局变量 ===== */
static ConfigHeader cfg_header;
static SystemParams sys_params;
static uint32_t system_tick = 0;
static uint32_t log_write_index = 0;  /* 日志写入索引 */

/* ===== I2C通信 ===== */
static bool I2C_WriteBytes(uint8_t dev_addr, uint16_t mem_addr, const uint8_t *data, uint16_t len)
{
    uint32_t timeout;

    for (uint16_t i = 0; i < len; i++) {
        /* 发送起始 + 设备地址 + 写 */
        DL_I2C_clearInterruptStatus(I2C0, DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
                                          DL_I2C_INTERRUPT_CONTROLLER_NACK);
        DL_I2C_startControllerTransfer(I2C0, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 2 + 1);

        /* 发送内存地址(高字节先) */
        DL_I2C_transmitControllerData(I2C0, (mem_addr >> 8) & 0xFF);
        timeout = I2C_TIMEOUT;
        while (DL_I2C_isControllerTXFIFOFull(I2C0) && timeout--) { if (!timeout) return false; }

        DL_I2C_transmitControllerData(I2C0, mem_addr & 0xFF);
        timeout = I2C_TIMEOUT;
        while (DL_I2C_isControllerTXFIFOFull(I2C0) && timeout--) { if (!timeout) return false; }

        DL_I2C_transmitControllerData(I2C0, data[i]);
        timeout = I2C_TIMEOUT;
        while (DL_I2C_isControllerBusy(I2C0) && timeout--) { if (!timeout) return false; }

        mem_addr++;
        delay_cycles(CPUCLK_FREQ / 100000);  /* ~10us */
    }
    return true;
}

static bool I2C_ReadBytes(uint8_t dev_addr, uint16_t mem_addr, uint8_t *data, uint16_t len)
{
    uint32_t timeout;

    /* 发送内存地址(伪写) */
    DL_I2C_clearInterruptStatus(I2C0, DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
                                      DL_I2C_INTERRUPT_CONTROLLER_NACK);
    DL_I2C_startControllerTransfer(I2C0, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    DL_I2C_transmitControllerData(I2C0, (mem_addr >> 8) & 0xFF);
    timeout = I2C_TIMEOUT;
    while (DL_I2C_isControllerTXFIFOFull(I2C0) && timeout--) { if (!timeout) return false; }

    DL_I2C_transmitControllerData(I2C0, mem_addr & 0xFF);
    timeout = I2C_TIMEOUT;
    while (DL_I2C_isControllerBusy(I2C0) && timeout--) { if (!timeout) return false; }

    /* 读取数据 */
    for (uint16_t i = 0; i < len; i++) {
        DL_I2C_startControllerTransfer(I2C0, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
        timeout = I2C_TIMEOUT;
        while (!DL_I2C_isControllerRXFIFOEmpty(I2C0) == 0 && timeout--) { if (!timeout) return false; }
        data[i] = DL_I2C_receiveControllerData(I2C0);
    }

    DL_I2C_stopControllerTransfer(I2C0);
    return true;
}

/* ===== 简化封装 ===== */
static bool FRAM_Write(uint16_t addr, const void *buf, uint16_t len)
{
    return I2C_WriteBytes(FM24CL64_ADDR, addr, (const uint8_t *)buf, len);
}

static bool FRAM_Read(uint16_t addr, void *buf, uint16_t len)
{
    return I2C_ReadBytes(FM24CL64_ADDR, addr, (uint8_t *)buf, len);
}

static void delay_ms(uint32_t ms)
{
    delay_cycles(ms * (CPUCLK_FREQ / 1000));
}

/* ===== CRC32校验 ===== */
static uint32_t CRC32_Calc(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

/* ===== 操作日志 ===== */
static void Write_Log(uint8_t operation, uint8_t group, uint16_t version, uint32_t checksum)
{
    if (log_write_index >= MAX_LOG_ENTRIES) {
        /* 日志已满, 覆盖最旧的 */
        log_write_index = 0;
    }
    LogEntry entry;
    entry.timestamp = system_tick;
    entry.operation = operation;
    entry.param_group = group;
    entry.data_version = version;
    entry.checksum = checksum;
    memset(entry.reserved, 0, sizeof(entry.reserved));

    uint16_t addr = ADDR_LOG_BASE + log_write_index * LOG_ENTRY_SIZE;
    FRAM_Write(addr, &entry, sizeof(LogEntry));
    log_write_index++;
}

/* ===== 参数组管理 ===== */
static uint16_t Get_GroupAddr(uint8_t group)
{
    return ADDR_PARAM_BASE + group * PARAM_GROUP_SIZE;
}

/* 保存参数到指定组 */
static bool Param_SaveGroup(uint8_t group)
{
    if (group > 7) return false;

    uint16_t addr = Get_GroupAddr(group);

    /* 写入魔数 + 参数 */
    uint32_t magic = PARAM_MAGIC;
    FRAM_Write(addr, &magic, 4);
    FRAM_Write(addr + 4, &sys_params, sizeof(SystemParams));

    /* 更新头信息 */
    cfg_header.data_version++;
    cfg_header.active_group = group;
    cfg_header.param_count = sizeof(SystemParams) / 2;
    cfg_header.param_checksum = CRC32_Calc((uint8_t *)&sys_params, sizeof(SystemParams));

    FRAM_Write(ADDR_HEADER, &cfg_header, sizeof(ConfigHeader));

    /* 写日志 */
    Write_Log(1, group, cfg_header.data_version, cfg_header.param_checksum);

    /* LED指示 */
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_0);
    delay_ms(50);
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);

    return true;
}

/* 从指定组加载参数 */
static bool Param_LoadGroup(uint8_t group)
{
    if (group > 7) return false;

    uint16_t addr = Get_GroupAddr(group);
    uint32_t magic;

    FRAM_Read(addr, &magic, 4);
    if (magic != PARAM_MAGIC) {
        return false;  /* 该组无有效数据 */
    }

    SystemParams tmp;
    FRAM_Read(addr + 4, &tmp, sizeof(SystemParams));

    /* CRC校验 */
    uint32_t crc = CRC32_Calc((uint8_t *)&tmp, sizeof(SystemParams));
    if (crc != cfg_header.param_checksum) {
        Write_Log(3, group, cfg_header.data_version, crc);  /* 校验错误日志 */
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1);  /* 错误LED */
        return false;
    }

    sys_params = tmp;
    Write_Log(0, group, cfg_header.data_version, crc);  /* 读取日志 */
    return true;
}

/* ===== 出厂默认参数 ===== */
static void Param_LoadDefaults(void)
{
    memset(&sys_params, 0, sizeof(SystemParams));

    sys_params.pwm_frequency = 1000;        /* 1kHz */
    sys_params.pwm_duty_cycle = 5000;       /* 50.00% */
    sys_params.target_temperature = 2500;    /* 25.0°C */
    sys_params.temp_hysteresis = 50;         /* 0.5°C */

    /* PID默认参数(Q8.8: 256=1.0) */
    sys_params.pid_kp = 256;    /* Kp = 1.0 */
    sys_params.pid_ki = 64;     /* Ki = 0.25 */
    sys_params.pid_kd = 128;    /* Kd = 0.5 */
    sys_params.pid_output_min = 0;
    sys_params.pid_output_max = 10000;

    /* ADC零偏 */
    for (int i = 0; i < 4; i++) {
        sys_params.adc_offset[i] = 0;
        sys_params.adc_gain[i] = 32768;  /* 1.0倍 */
    }

    /* DAC默认 */
    for (int i = 0; i < 2; i++) {
        sys_params.dac_offset[i] = 0;
        sys_params.dac_fullscale[i] = 4095;
    }

    sys_params.uart_baudrate = 115200;
    sys_params.device_address = 0x01;
    sys_params.comm_protocol = 0;
    sys_params.display_brightness = 80;
    sys_params.backlight_timeout = 60;
    sys_params.alarm_threshold = 3000;
    sys_params.alarm_enable = 0xFF;

    /* 保存默认参数到组0 */
    cfg_header.magic = CFG_MAGIC;
    cfg_header.format_version = CFG_VERSION;
    cfg_header.data_version = 0;
    cfg_header.active_group = 0;
    strncpy((char *)cfg_header.device_id, "MSPM0G3507-v1", 16);

    Param_SaveGroup(0);

    Write_Log(2, 0, cfg_header.data_version, cfg_header.param_checksum);
}

/* ===== 参数验证 ===== */
static bool Param_Validate(const SystemParams *p)
{
    if (p->pwm_frequency < 100 || p->pwm_frequency > 100000) return false;
    if (p->pwm_duty_cycle > 10000) return false;
    if (p->target_temperature > 10000) return false;  /* <100°C */
    if (p->pid_kp < 0 || p->pid_kp > 25600) return false;
    if (p->uart_baudrate < 9600 || p->uart_baudrate > 921600) return false;
    if (p->display_brightness > 100) return false;
    return true;
}

/* ===== 参数复制(组间) ===== */
static bool Param_CopyGroup(uint8_t src_group, uint8_t dst_group)
{
    uint8_t buf[PARAM_GROUP_SIZE];
    uint16_t src_addr = Get_GroupAddr(src_group);
    FRAM_Read(src_addr, buf, PARAM_GROUP_SIZE);

    uint16_t dst_addr = Get_GroupAddr(dst_group);
    FRAM_Write(dst_addr, buf, PARAM_GROUP_SIZE);

    return true;
}

/* ===== 按键读取 ===== */
static bool Key_ReadDebounce(uint8_t pin)
{
    if (!(DL_GPIO_readPins(GPIOB, 1 << pin))) {
        delay_ms(20);
        if (!(DL_GPIO_readPins(GPIOB, 1 << pin))) {
            while (!(DL_GPIO_readPins(GPIOB, 1 << pin))) {}
            return true;
        }
    }
    return false;
}

/* ===== 串口命令解析(简易) ===== */
static void Process_UART_Command(void)
{
    if (!DL_UART_isRXFIFOEmpty(UART0)) {
        uint8_t cmd = DL_UART_receiveData8(UART0);

        switch (cmd) {
        case 'R':  /* 读取当前参数 */
            {
                uint8_t *p = (uint8_t *)&sys_params;
                for (uint32_t i = 0; i < sizeof(SystemParams); i++) {
                    DL_UART_transmitData(UART0, p[i]);
                    while (DL_UART_isBusy(UART0)) {}
                }
            }
            break;

        case 'W':  /* 写入参数(等待后续数据) */
            {
                SystemParams tmp;
                uint8_t *p = (uint8_t *)&tmp;
                for (uint32_t i = 0; i < sizeof(SystemParams); i++) {
                    uint32_t timeout = 100000;
                    while (DL_UART_isRXFIFOEmpty(UART0) && timeout--) {
                        if (!timeout) return;
                    }
                    p[i] = DL_UART_receiveData8(UART0);
                }
                if (Param_Validate(&tmp)) {
                    sys_params = tmp;
                    Param_SaveGroup(cfg_header.active_group);
                    DL_UART_transmitData(UART0, 'K');
                } else {
                    DL_UART_transmitData(UART0, 'E');
                }
                while (DL_UART_isBusy(UART0)) {}
            }
            break;

        case 'F':  /* 恢复出厂 */
            Param_LoadDefaults();
            DL_UART_transmitData(UART0, 'K');
            while (DL_UART_isBusy(UART0)) {}
            break;

        case 'S':  /* 切换参数组 */
            {
                uint32_t timeout = 100000;
                while (DL_UART_isRXFIFOEmpty(UART0) && timeout--) {}
                if (timeout) {
                    uint8_t grp = DL_UART_receiveData8(UART0);
                    if (Param_LoadGroup(grp)) {
                        DL_UART_transmitData(UART0, 'K');
                    } else {
                        DL_UART_transmitData(UART0, 'E');
                    }
                    while (DL_UART_isBusy(UART0)) {}
                }
            }
            break;

        case 'V':  /* 读取版本信息 */
            {
                uint8_t *p = (uint8_t *)&cfg_header;
                for (uint32_t i = 0; i < sizeof(ConfigHeader); i++) {
                    DL_UART_transmitData(UART0, p[i]);
                    while (DL_UART_isBusy(UART0)) {}
                }
            }
            break;
        }
    }
}

/* ===== 主函数 ===== */
int main(void)
{
    SYSCFG_DL_init();

    /* 读取配置头 */
    FRAM_Read(ADDR_HEADER, &cfg_header, sizeof(ConfigHeader));

    /* 检查是否首次使用 */
    if (cfg_header.magic != CFG_MAGIC) {
        /* 首次使用, 加载出厂默认 */
        Param_LoadDefaults();
    } else {
        /* 加载当前激活组 */
        if (!Param_LoadGroup(cfg_header.active_group)) {
            /* 加载失败, 尝试其他组 */
            bool loaded = false;
            for (uint8_t g = 0; g < 8; g++) {
                if (g == cfg_header.active_group) continue;
                if (Param_LoadGroup(g)) {
                    cfg_header.active_group = g;
                    loaded = true;
                    break;
                }
            }
            if (!loaded) {
                /* 所有组都损坏, 恢复默认 */
                Param_LoadDefaults();
            }
        }
    }

    /* 找到日志写入位置 */
    log_write_index = 0;
    for (uint32_t i = 0; i < MAX_LOG_ENTRIES; i++) {
        LogEntry entry;
        FRAM_Read(ADDR_LOG_BASE + i * LOG_ENTRY_SIZE, &entry, sizeof(LogEntry));
        if (entry.timestamp == 0xFFFFFFFF) {
            log_write_index = i;
            break;
        }
        log_write_index = i + 1;
    }

    /* LED确认初始化完成 */
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_0);
    delay_ms(200);
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);

    /* 主循环 */
    while (1) {
        /* PB2: 保存当前参数 */
        if (Key_ReadDebounce(2)) {
            Param_SaveGroup(cfg_header.active_group);
        }

        /* PB3: 恢复出厂默认 */
        if (Key_ReadDebounce(3)) {
            Param_LoadDefaults();
            /* 闪灯确认 */
            for (int i = 0; i < 3; i++) {
                DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_1);
                delay_ms(200);
            }
        }

        /* PB4: 切换参数组 */
        if (Key_ReadDebounce(4)) {
            uint8_t next_group = (cfg_header.active_group + 1) % 4;
            if (Param_LoadGroup(next_group)) {
                cfg_header.active_group = next_group;
                /* 快闪确认 */
                for (int i = 0; i < next_group + 1; i++) {
                    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_0);
                    delay_ms(100);
                    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);
                    delay_ms(100);
                }
            }
        }

        /* 处理串口命令 */
        Process_UART_Command();

        system_tick++;
        delay_ms(10);
    }
}
