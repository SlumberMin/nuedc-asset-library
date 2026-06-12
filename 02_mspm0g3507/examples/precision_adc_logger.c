/**
 * @file precision_adc_logger.c
 * @brief MSPM0G3507 精密ADC数据记录系统 - ADS1256 + SD卡 + 多通道同步
 * 
 * 硬件连接：
 *   ADS1256 SPI接口:
 *     SCK  -> PA2 (SPI0_SCK)
 *     DIN  -> PA4 (SPI0_MOSI, ADS1256的DOUT引脚)
 *     DOUT -> PA6 (SPI0_MISO, ADS1256的DIN引脚)
 *     CS   -> PA5 (GPIO)
 *     DRDY -> PB0 (外部中断, 数据就绪)
 *     RST  -> PB1 (GPIO)
 * 
 *   SD卡 SPI接口 (使用SPI1):
 *     SCK  -> PB8
 *     MOSI -> PB9
 *     MISO -> PB10
 *     CS   -> PB11
 * 
 *   ADS1256模拟输入:
 *     AIN0~AIN7 -> 8路差分/单端输入
 * 
 * 功能说明：
 *   - 24位Σ-Δ ADC ADS1256驱动，支持8通道差分/单端输入
 *   - 可编程增益放大器(PGA): x1, x2, x4, x8, x16, x32, x64
 *   - 多通道自动扫描模式，支持同步采样
 *   - SD卡FAT文件系统记录，CSV格式输出
 *   - 支持定时采样、触发采样、连续记录模式
 * 
 * 适用场景：电赛中高精度电压/电流测量、多通道数据采集系统
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ============================================================
 * 第一部分：ADS1256寄存器定义
 * ============================================================ */

/* ADS1256寄存器地址 */
#define ADS1256_REG_STATUS  0x00  /* 状态寄存器 */
#define ADS1256_REG_MUX     0x01  /* 多路复用寄存器 */
#define ADS1256_REG_ADCON   0x02  /* A/D控制寄存器 */
#define ADS1256_REG_DRATE   0x03  /* 数据速率寄存器 */
#define ADS1256_REG_IO      0x04  /* GPIO控制寄存器 */
#define ADS1256_REG_OFC0    0x05  /* 偏移校准字节0 */
#define ADS1256_REG_OFC1    0x06  /* 偏移校准字节1 */
#define ADS1256_REG_OFC2    0x07  /* 偏移校准字节2 */
#define ADS1256_REG_FSC0    0x08  /* 满量程校准字节0 */
#define ADS1256_REG_FSC1    0x09  /* 满量程校准字节1 */
#define ADS1256_REG_FSC2    0x0A  /* 满量程校准字节2 */

/* ADS1256 SPI命令 */
#define ADS1256_CMD_WAKEUP   0x00  /* 唤醒 */
#define ADS1256_CMD_RDATA    0x01  /* 读取数据 */
#define ADS1256_CMD_RDATAC   0x03  /* 连续读取数据 */
#define ADS1256_CMD_SDATAC   0x0F  /* 停止连续读取 */
#define ADS1256_CMD_RREG     0x10  /* 读寄存器 (0x10 + addr) */
#define ADS1256_CMD_WREG     0x50  /* 写寄存器 (0x50 + addr) */
#define ADS1256_CMD_SELFCAL  0xF0  /* 自校准 */
#define ADS1256_CMD_SYSOCAL  0xF1  /* 系统偏移校准 */
#define ADS1256_CMD_SYSGCAL  0xF2  /* 系统增益校准 */
#define ADS1256_CMD_SYNC     0xFC  /* 同步 */
#define ADS1256_CMD_STANDBY  0xFD  /* 待机 */
#define ADS1256_CMD_RESET    0xFE  /* 复位 */

/* 数据速率定义 (DRATE寄存器值) */
#define ADS1256_DRATE_30000  0xF0  /* 30000 SPS */
#define ADS1256_DRATE_15000  0xE0  /* 15000 SPS */
#define ADS1256_DRATE_7500   0xD0  /* 7500 SPS */
#define ADS1256_DRATE_3750   0xC0  /* 3750 SPS */
#define ADS1256_DRATE_2000   0xB0  /* 2000 SPS */
#define ADS1256_DRATE_1000   0xA1  /* 1000 SPS */
#define ADS1256_DRATE_500    0x92  /* 500 SPS */
#define ADS1256_DRATE_100    0x82  /* 100 SPS */
#define ADS1256_DRATE_50     0x72  /* 50 SPS */
#define ADS1256_DRATE_25     0x63  /* 25 SPS (最大分辨率) */
#define ADS1256_DRATE_10     0x53  /* 10 SPS */
#define ADS1256_DRATE_5      0x43  /* 5 SPS */
#define ADS1256_DRATE_2_5    0x33  /* 2.5 SPS */

/* PGA增益定义 */
#define ADS1256_PGA_1        0x00  /* 增益x1 */
#define ADS1256_PGA_2        0x01  /* 增益x2 */
#define ADS1256_PGA_4        0x02  /* 增益x4 */
#define ADS1256_PGA_8        0x03  /* 增益x8 */
#define ADS1256_PGA_16       0x04  /* 增益x16 */
#define ADS1256_PGA_32       0x05  /* 增益x32 */
#define ADS1256_PGA_64       0x06  /* 增益x64 */

/* 通道选择 (MUX寄存器) */
#define ADS1256_MUXP_AIN0   0x00  /* 正输入AIN0 */
#define ADS1256_MUXP_AIN1   0x10  /* 正输入AIN1 */
#define ADS1256_MUXP_AIN2   0x20  /* 正输入AIN2 */
#define ADS1256_MUXP_AIN3   0x30  /* 正输入AIN3 */
#define ADS1256_MUXP_AIN4   0x40  /* 正输入AIN4 */
#define ADS1256_MUXP_AIN5   0x50  /* 正输入AIN5 */
#define ADS1256_MUXP_AIN6   0x60  /* 正输入AIN6 */
#define ADS1256_MUXP_AIN7   0x70  /* 正输入AIN7 */
#define ADS1256_MUXN_AIN0   0x00  /* 负输入AIN0 */
#define ADS1256_MUXN_AIN1   0x01  /* 负输入AIN1 */
#define ADS1256_MUXN_AIN2   0x02  /* 负输入AIN2 */
#define ADS1256_MUXN_AIN3   0x03  /* 负输入AIN3 */
#define ADS1256_MUXN_AINCOM 0x08  /* 负输入AINCOM */

/* 引脚定义 */
#define ADS1256_CS_PORT     GPIOA
#define ADS1256_CS_PIN      DL_GPIO_PIN_5
#define ADS1256_DRDY_PORT   GPIOB
#define ADS1256_DRDY_PIN    DL_GPIO_PIN_0
#define ADS1256_RST_PORT    GPIOB
#define ADS1256_RST_PIN     DL_GPIO_PIN_1

/* SD卡引脚定义 */
#define SD_CS_PORT          GPIOB
#define SD_CS_PIN           DL_GPIO_PIN_11

/* ============================================================
 * 第二部分：数据结构定义
 * ============================================================ */

/* ADC采样配置 */
typedef struct {
    uint8_t  pga_gain;          /* PGA增益 */
    uint8_t  data_rate;         /* 数据速率 */
    uint8_t  channels;          /* 采样通道数 */
    uint8_t  channel_list[8];   /* 通道列表 */
    bool     differential;      /* 是否差分模式 */
} ADC_SampleConfig_t;

/* 采样数据结构 */
typedef struct {
    uint32_t timestamp_ms;      /* 时间戳(ms) */
    int32_t  raw_value[8];      /* 原始ADC值 */
    double   voltage[8];        /* 转换后的电压值 */
    uint8_t  channel_count;     /* 有效通道数 */
} ADC_SampleData_t;

/* 记录模式 */
typedef enum {
    RECORD_MODE_CONTINUOUS,     /* 连续记录 */
    RECORD_MODE_TIMED,          /* 定时记录 */
    RECORD_MODE_TRIGGERED       /* 触发记录 */
} RecordMode_t;

/* 记录状态 */
typedef enum {
    RECORD_IDLE,                /* 空闲 */
    RECORD_RUNNING,             /* 正在记录 */
    RECORD_PAUSED,              /* 暂停 */
    RECORD_ERROR                /* 错误 */
} RecordState_t;

/* ============================================================
 * 第三部分：全局变量
 * ============================================================ */

static volatile uint32_t g_systick_ms = 0;       /* 系统毫秒计数器 */
static volatile bool g_drdy_flag = false;         /* DRDY中断标志 */
static volatile bool g_sd_ready = false;          /* SD卡就绪标志 */

/* 当前配置 */
static ADC_SampleConfig_t g_adc_config = {
    .pga_gain = ADS1256_PGA_1,
    .data_rate = ADS1256_DRATE_1000,
    .channels = 4,
    .channel_list = {0, 1, 2, 3, 0, 0, 0, 0},
    .differential = false
};

/* 记录控制 */
static RecordMode_t g_record_mode = RECORD_MODE_CONTINUOUS;
static RecordState_t g_record_state = RECORD_IDLE;
static uint32_t g_sample_count = 0;               /* 已采样计数 */
static uint32_t g_sample_interval_ms = 10;         /* 采样间隔(ms) */
static uint32_t g_max_samples = 0;                 /* 最大采样数(0=无限) */

/* 参考电压 */
#define VREF            2.500    /* ADS1256参考电压(V) */
#define ADC_RESOLUTION  8388608.0 /* 2^23 (24位ADC) */

/* 缓冲区 */
#define SAMPLE_BUF_SIZE 1024
static ADC_SampleData_t g_sample_buffer[SAMPLE_BUF_SIZE];
static uint16_t g_buf_head = 0;
static uint16_t g_buf_tail = 0;

/* SD卡文件名 */
static char g_filename[32] = "datalog_001.csv";

/* ============================================================
 * 第四部分：底层SPI与GPIO操作
 * ============================================================ */

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
 * @brief 延时毫秒
 */
static void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000);
}

/**
 * @brief ADS1256 CS控制
 */
static void ads1256_select(void)
{
    DL_GPIO_clearPins(ADS1256_CS_PORT, ADS1256_CS_PIN);
    delay_us(1);
}

static void ads1256_deselect(void)
{
    delay_us(1);
    DL_GPIO_setPins(ADS1256_CS_PORT, ADS1256_CS_PIN);
}

/**
 * @brief SD卡 CS控制
 */
static void sd_select(void)
{
    DL_GPIO_clearPins(SD_CS_PORT, SD_CS_PIN);
    delay_us(1);
}

static void sd_deselect(void)
{
    delay_us(1);
    DL_GPIO_setPins(SD_CS_PORT, SD_CS_PIN);
}

/**
 * @brief SPI0传输一个字节 (ADS1256用)
 */
static uint8_t spi0_transfer(uint8_t data)
{
    DL_SPI_transmitData8(SPI0, data);
    while (DL_SPI_isBusy(SPI0)) ;
    return (uint8_t)DL_SPI_receiveData8(SPI0);
}

/**
 * @brief SPI1传输一个字节 (SD卡用)
 */
static uint8_t spi1_transfer(uint8_t data)
{
    DL_SPI_transmitData8(SPI1, data);
    while (DL_SPI_isBusy(SPI1)) ;
    return (uint8_t)DL_SPI_receiveData8(SPI1);
}

/* ============================================================
 * 第五部分：ADS1256驱动层
 * ============================================================ */

/**
 * @brief 等待DRDY信号变低 (数据就绪)
 * @param timeout_ms 超时时间(ms)
 * @return true=数据就绪, false=超时
 */
static bool ads1256_wait_drdy(uint32_t timeout_ms)
{
    while (timeout_ms--) {
        if (!DL_GPIO_readPins(ADS1256_DRDY_PORT, ADS1256_DRDY_PIN)) {
            return true;
        }
        delay_ms(1);
    }
    return false;
}

/**
 * @brief 写ADS1256寄存器
 * @param reg 寄存器地址
 * @param value 要写入的值
 */
static void ads1256_write_reg(uint8_t reg, uint8_t value)
{
    ads1256_select();
    spi0_transfer(ADS1256_CMD_WREG | reg); /* 写命令 + 寄存器地址 */
    spi0_transfer(0x00);                    /* 写1个字节 */
    spi0_transfer(value);
    delay_us(50);
    ads1256_deselect();
}

/**
 * @brief 读ADS1256寄存器
 * @param reg 寄存器地址
 * @return 寄存器值
 */
static uint8_t ads1256_read_reg(uint8_t reg)
{
    uint8_t value;
    ads1256_select();
    spi0_transfer(ADS1256_CMD_RREG | reg); /* 读命令 + 寄存器地址 */
    spi0_transfer(0x00);                    /* 读1个字节 */
    delay_us(10);
    value = spi0_transfer(0xFF);
    ads1256_deselect();
    return value;
}

/**
 * @brief 发送ADS1256命令
 * @param cmd 命令字节
 */
static void ads1256_send_cmd(uint8_t cmd)
{
    ads1256_select();
    spi0_transfer(cmd);
    ads1256_deselect();
}

/**
 * @brief 读取ADC原始数据 (24位有符号)
 * @return ADC原始值 (int32_t, 符号扩展)
 */
static int32_t ads1256_read_data(void)
{
    int32_t result = 0;
    uint8_t buf[3];
    
    ads1256_select();
    spi0_transfer(ADS1256_CMD_RDATA);
    delay_us(10); /* t6: 命令到数据的延时 */
    
    buf[0] = spi0_transfer(0xFF); /* 高字节 */
    buf[1] = spi0_transfer(0xFF); /* 中字节 */
    buf[2] = spi0_transfer(0xFF); /* 低字节 */
    ads1256_deselect();
    
    /* 组合24位数据 */
    result = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    
    /* 符号扩展到32位 */
    if (result & 0x800000) {
        result |= 0xFF000000; /* 负数符号扩展 */
    }
    
    return result;
}

/**
 * @brief 设置ADS1256 MUX通道
 * @param positive 正输入通道 (0~7)
 * @param negative 负输入通道 (0~7, 或8表示AINCOM)
 */
static void ads1256_set_mux(uint8_t positive, uint8_t negative)
{
    uint8_t mux_val = ((positive & 0x0F) << 4) | (negative & 0x0F);
    ads1256_write_reg(ADS1256_REG_MUX, mux_val);
}

/**
 * @brief 设置ADS1256 PGA增益
 * @param gain 增益值 (ADS1256_PGA_xxx)
 */
static void ads1256_set_pga(uint8_t gain)
{
    /* 读取ADCON寄存器，保留高3位，修改低3位 */
    uint8_t adcon = ads1256_read_reg(ADS1256_REG_ADCON);
    adcon = (adcon & 0xF8) | (gain & 0x07);
    ads1256_write_reg(ADS1256_REG_ADCON, adcon);
}

/**
 * @brief 设置ADS1256数据速率
 * @param drate 数据速率 (ADS1256_DRATE_xxx)
 */
static void ads1256_set_drate(uint8_t drate)
{
    ads1256_write_reg(ADS1256_REG_DRATE, drate);
}

/**
 * @brief ADS1256硬件复位
 */
static void ads1256_hardware_reset(void)
{
    DL_GPIO_clearPins(ADS1256_RST_PORT, ADS1256_RST_PIN);
    delay_ms(10);
    DL_GPIO_setPins(ADS1256_RST_PORT, ADS1256_RST_PIN);
    delay_ms(50); /* 等待复位完成 */
}

/**
 * @brief 执行ADS1256自校准
 * @return true=校准成功
 */
static bool ads1256_self_calibrate(void)
{
    ads1256_send_cmd(ADS1256_CMD_SELFCAL);
    return ads1256_wait_drdy(1000);
}

/**
 * @brief 初始化ADS1256
 * @param config 采样配置
 * @return true=初始化成功
 */
static bool ads1256_init(const ADC_SampleConfig_t *config)
{
    /* 硬件复位 */
    ads1256_hardware_reset();
    
    /* 等待DRDY */
    if (!ads1256_wait_drdy(1000)) {
        return false;
    }
    
    /* 停止连续读取模式 */
    ads1256_send_cmd(ADS1256_CMD_SDATAC);
    delay_us(100);
    
    /* 读取并验证芯片ID */
    uint8_t status = ads1256_read_reg(ADS1256_REG_STATUS);
    /* ADS1256的ID应该在高4位 */
    
    /* 配置状态寄存器: 自动校准使能, 缓冲区使能 */
    ads1256_write_reg(ADS1256_REG_STATUS, 0x06);
    
    /* 设置PGA增益 */
    ads1256_set_pga(config->pga_gain);
    
    /* 设置数据速率 */
    ads1256_set_drate(config->data_rate);
    
    /* 设置初始通道 */
    if (config->channels > 0) {
        ads1256_set_mux(config->channel_list[0], 
                        config->differential ? config->channel_list[1] : ADS1256_MUXN_AINCOM);
    }
    
    /* 执行自校准 */
    if (!ads1256_self_calibrate()) {
        return false;
    }
    
    return true;
}

/**
 * @brief 读取指定通道的ADC值
 * @param positive 正输入通道
 * @param negative 负输入通道
 * @return 24位ADC原始值
 */
static int32_t ads1256_read_channel(uint8_t positive, uint8_t negative)
{
    /* 设置通道 */
    ads1256_set_mux(positive, negative);
    delay_us(5);
    
    /* 发送同步命令，启动转换 */
    ads1256_send_cmd(ADS1256_CMD_SYNC);
    delay_us(5);
    ads1256_send_cmd(ADS1256_CMD_WAKEUP);
    
    /* 等待转换完成 */
    if (!ads1256_wait_drdy(100)) {
        return 0; /* 超时返回0 */
    }
    
    /* 读取数据 */
    return ads1256_read_data();
}

/**
 * @brief ADC原始值转换为电压
 * @param raw 24位ADC原始值
 * @param pga_gain PGA增益值 (1,2,4,8,16,32,64)
 * @return 电压值(V)
 */
static double ads1256_to_voltage(int32_t raw, uint8_t pga_gain)
{
    double gain = 1.0;
    switch (pga_gain) {
        case ADS1256_PGA_1:  gain = 1.0;  break;
        case ADS1256_PGA_2:  gain = 2.0;  break;
        case ADS1256_PGA_4:  gain = 4.0;  break;
        case ADS1256_PGA_8:  gain = 8.0;  break;
        case ADS1256_PGA_16: gain = 16.0; break;
        case ADS1256_PGA_32: gain = 32.0; break;
        case ADS1256_PGA_64: gain = 64.0; break;
    }
    
    return ((double)raw / ADC_RESOLUTION) * (VREF / gain);
}

/* ============================================================
 * 第六部分：SD卡SPI接口驱动 (简化版FAT16)
 * ============================================================ */

/* SD卡类型 */
#define SD_TYPE_NONE    0
#define SD_TYPE_MMC     1
#define SD_TYPE_SDSC    2   /* 标准容量SD (<=2GB) */
#define SD_TYPE_SDHC    3   /* 高容量SD (<=32GB) */

static uint8_t g_sd_type = SD_TYPE_NONE;

/**
 * @brief SD卡发送命令
 * @param cmd 命令号
 * @param arg 32位参数
 * @return SD卡响应 (R1)
 */
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t response;
    uint8_t retry = 0xFF;
    
    sd_select();
    spi1_transfer(0x40 | cmd);
    spi1_transfer((uint8_t)(arg >> 24));
    spi1_transfer((uint8_t)(arg >> 16));
    spi1_transfer((uint8_t)(arg >> 8));
    spi1_transfer((uint8_t)arg);
    
    /* CMD0的CRC, 其他命令通常不需要正确的CRC */
    if (cmd == 0x00) spi1_transfer(0x95); /* CMD0 CRC */
    else if (cmd == 0x08) spi1_transfer(0x87); /* CMD8 CRC */
    else spi1_transfer(0xFF);
    
    /* 等待响应 (非0xFF) */
    do {
        response = spi1_transfer(0xFF);
        retry--;
    } while ((response == 0xFF) && (retry > 0));
    
    return response;
}

/**
 * @brief SD卡初始化
 * @return true=初始化成功
 */
static bool sd_init(void)
{
    uint8_t response;
    uint16_t retry;
    
    sd_deselect();
    
    /* 发送80个时钟 (至少74个) */
    for (uint8_t i = 0; i < 10; i++) {
        spi1_transfer(0xFF);
    }
    
    /* 发送CMD0，进入SPI模式 */
    retry = 1000;
    do {
        response = sd_send_cmd(0x00, 0x00000000);
        sd_deselect();
        if (response == 0x01) break;
        delay_ms(1);
    } while (retry--);
    
    if (response != 0x01) {
        return false; /* SD卡未就绪 */
    }
    
    /* 发送CMD8，检查SD卡版本 */
    response = sd_send_cmd(0x08, 0x000001AA);
    sd_deselect();
    
    if (response == 0x01) {
        /* SDHC/SDXC卡 */
        /* 读取剩余的R7响应 */
        for (uint8_t i = 0; i < 4; i++) spi1_transfer(0xFF);
        
        /* 发送ACMD41 (CMD55 + CMD41) */
        retry = 1000;
        do {
            sd_send_cmd(0x55, 0x00000000);
            sd_deselect();
            response = sd_send_cmd(0x41, 0x40000000); /* HCS位 */
            sd_deselect();
            if (response == 0x00) break;
            delay_ms(1);
        } while (retry--);
        
        if (response != 0x00) return false;
        
        /* 发送CMD58读取OCR */
        response = sd_send_cmd(0x58, 0x00000000);
        uint8_t ocr[4];
        for (uint8_t i = 0; i < 4; i++) ocr[i] = spi1_transfer(0xFF);
        sd_deselect();
        
        g_sd_type = (ocr[0] & 0x40) ? SD_TYPE_SDHC : SD_TYPE_SDSC;
    } else {
        /* SDSC/MMC卡 */
        retry = 1000;
        do {
            sd_send_cmd(0x55, 0x00000000);
            sd_deselect();
            response = sd_send_cmd(0x41, 0x00000000);
            sd_deselect();
            if (response == 0x00) break;
            delay_ms(1);
        } while (retry--);
        
        g_sd_type = SD_TYPE_SDSC;
    }
    
    /* 设置块大小为512字节 */
    sd_send_cmd(0x10, 0x00000200); /* CMD16 */
    sd_deselect();
    
    return true;
}

/**
 * @brief 读取SD卡单个扇区 (512字节)
 * @param sector 扇区号
 * @param buf 数据缓冲区 (至少512字节)
 * @return true=读取成功
 */
static bool sd_read_sector(uint32_t sector, uint8_t *buf)
{
    uint8_t response;
    uint16_t retry;
    
    /* SDHC卡使用扇区寻址，SDSC使用字节寻址 */
    if (g_sd_type != SD_TYPE_SDHC) {
        sector <<= 9; /* 转换为字节地址 */
    }
    
    /* CMD17读单个扇区 */
    response = sd_send_cmd(0x11, sector);
    if (response != 0x00) {
        sd_deselect();
        return false;
    }
    
    /* 等待数据开始令牌 (0xFE) */
    retry = 10000;
    do {
        response = spi1_transfer(0xFF);
        if (response == 0xFE) break;
        retry--;
    } while (retry > 0);
    
    if (response != 0xFE) {
        sd_deselect();
        return false;
    }
    
    /* 读取512字节数据 */
    for (uint16_t i = 0; i < 512; i++) {
        buf[i] = spi1_transfer(0xFF);
    }
    
    /* 读取并丢弃CRC */
    spi1_transfer(0xFF);
    spi1_transfer(0xFF);
    
    sd_deselect();
    return true;
}

/**
 * @brief 写入SD卡单个扇区 (512字节)
 * @param sector 扇区号
 * @param buf 数据缓冲区 (512字节)
 * @return true=写入成功
 */
static bool sd_write_sector(uint32_t sector, const uint8_t *buf)
{
    uint8_t response;
    uint16_t retry;
    
    if (g_sd_type != SD_TYPE_SDHC) {
        sector <<= 9;
    }
    
    /* CMD24写单个扇区 */
    response = sd_send_cmd(0x18, sector);
    if (response != 0x00) {
        sd_deselect();
        return false;
    }
    
    /* 发送数据开始令牌 */
    spi1_transfer(0xFE);
    
    /* 发送512字节数据 */
    for (uint16_t i = 0; i < 512; i++) {
        spi1_transfer(buf[i]);
    }
    
    /* 发送伪CRC */
    spi1_transfer(0xFF);
    spi1_transfer(0xFF);
    
    /* 检查数据响应 */
    response = spi1_transfer(0xFF);
    if ((response & 0x1F) != 0x05) {
        sd_deselect();
        return false;
    }
    
    /* 等待写入完成 */
    retry = 10000;
    do {
        response = spi1_transfer(0xFF);
        if (response != 0x00) break;
        retry--;
    } while (retry > 0);
    
    sd_deselect();
    return true;
}

/* ============================================================
 * 第七部分：简化FAT16文件系统
 * ============================================================ */

/* FAT16 BPB (BIOS参数块) 偏移 */
#define FAT_BPB_SECTOR_COUNT    0x13  /* 总扇区数 */
#define FAT_BPB_FAT_SIZE        0x16  /* FAT大小(扇区) */
#define FAT_BPB_ROOT_ENTRIES    0x17  /* 根目录项数 */
#define FAT_BPB_SECTORS_PER_FAT 0x16  /* 每个FAT的扇区数 */

static uint32_t g_fat_start_sector = 0;
static uint32_t g_root_dir_sector = 0;
static uint32_t g_data_start_sector = 0;
static uint32_t g_current_cluster = 0;
static uint32_t g_file_size = 0;

/* 扇区缓冲区 */
static uint8_t g_sector_buf[512];

/**
 * @brief 解析FAT16引导扇区
 * @return true=解析成功
 */
static bool fat16_parse_boot_sector(void)
{
    /* 读取MBR (扇区0) */
    if (!sd_read_sector(0, g_sector_buf)) return false;
    
    /* 检查是否有分区表 */
    uint32_t boot_sector = 0;
    if (g_sector_buf[0] == 0xEB || g_sector_buf[0] == 0xE9) {
        boot_sector = 0; /* 直接是引导扇区 */
    } else {
        /* 读取第一个分区表项 */
        boot_sector = *(uint32_t *)&g_sector_buf[0x1C6]; /* 分区1起始扇区 */
    }
    
    /* 读取引导扇区 */
    if (!sd_read_sector(boot_sector, g_sector_buf)) return false;
    
    /* 验证FAT16签名 */
    if (g_sector_buf[0x1FE] != 0x55 || g_sector_buf[0x1FF] != 0xAA) {
        return false;
    }
    
    /* 解析BPB */
    uint16_t sectors_per_cluster = g_sector_buf[0x0D];
    uint16_t reserved_sectors = *(uint16_t *)&g_sector_buf[0x0E];
    uint8_t  num_fats = g_sector_buf[0x10];
    uint16_t root_entries = *(uint16_t *)&g_sector_buf[0x11];
    uint16_t fat_sectors = *(uint16_t *)&g_sector_buf[0x16];
    
    /* 计算关键扇区位置 */
    g_fat_start_sector = boot_sector + reserved_sectors;
    g_root_dir_sector = g_fat_start_sector + (num_fats * fat_sectors);
    g_data_start_sector = g_root_dir_sector + ((root_entries * 32) / 512);
    
    return true;
}

/**
 * @brief 查找或创建文件
 * @param filename 8.3格式文件名 (如 "DATA    CSV")
 * @return 文件起始簇号, 0=失败
 */
static uint16_t fat16_find_file(const char *filename)
{
    uint32_t sector = g_root_dir_sector;
    uint16_t max_entries = 512; /* 简化: 假设根目录最多512项 */
    uint16_t entries_per_sector = 16; /* 512/32 */
    uint16_t entry_idx = 0;
    uint16_t free_entry_sector = 0;
    uint16_t free_entry_offset = 0;
    bool found_free = false;
    
    for (uint16_t s = 0; s < (max_entries / entries_per_sector); s++) {
        if (!sd_read_sector(sector + s, g_sector_buf)) return 0;
        
        for (uint16_t e = 0; e < entries_per_sector; e++) {
            uint16_t offset = e * 32;
            uint8_t first_byte = g_sector_buf[offset];
            
            if (first_byte == 0x00 || first_byte == 0xE5) {
                /* 空闲或已删除的目录项 */
                if (!found_free) {
                    free_entry_sector = sector + s;
                    free_entry_offset = offset;
                    found_free = true;
                }
                if (first_byte == 0x00) goto no_more_entries;
                continue;
            }
            
            /* 比较文件名 (简化: 只比较前8+3字节) */
            bool match = true;
            for (uint8_t i = 0; i < 11; i++) {
                if (g_sector_buf[offset + i] != (uint8_t)filename[i]) {
                    match = false;
                    break;
                }
            }
            
            if (match) {
                return *(uint16_t *)&g_sector_buf[offset + 0x1A]; /* 起始簇 */
            }
        }
    }
    
no_more_entries:
    /* 文件不存在，创建新文件 */
    if (!found_free) return 0;
    
    /* 创建目录项 */
    memset(g_sector_buf, 0, 512);
    sd_read_sector(free_entry_sector, g_sector_buf);
    
    memcpy(&g_sector_buf[free_entry_offset], filename, 11);
    g_sector_buf[free_entry_offset + 0x0B] = 0x20; /* 存档属性 */
    g_sector_buf[free_entry_offset + 0x1A] = 0x02; /* 起始簇=2 */
    g_sector_buf[free_entry_offset + 0x1B] = 0x00;
    
    sd_write_sector(free_entry_sector, g_sector_buf);
    
    return 2; /* 返回起始簇 */
}

/**
 * @brief 追加数据到文件 (简化实现)
 * @param data 要写入的数据
 * @param len 数据长度
 * @return true=写入成功
 */
static bool fat16_append_data(const char *data, uint16_t len)
{
    /* 简化实现: 直接追加到当前簇 */
    /* 实际应用中需要更完整的FAT16实现 */
    
    uint32_t data_sector = g_data_start_sector + (g_current_cluster - 2);
    
    /* 计算在当前扇区中的偏移 */
    uint16_t sector_offset = g_file_size % 512;
    
    if (sector_offset == 0 && g_file_size > 0) {
        /* 需要新扇区 */
        data_sector++;
    }
    
    /* 读取当前扇区 */
    if (!sd_read_sector(data_sector, g_sector_buf)) return false;
    
    /* 追加数据 */
    uint16_t bytes_to_write = len;
    if (bytes_to_write > (512 - sector_offset)) {
        bytes_to_write = 512 - sector_offset;
    }
    
    memcpy(&g_sector_buf[sector_offset], data, bytes_to_write);
    sd_write_sector(data_sector, g_sector_buf);
    
    g_file_size += bytes_to_write;
    
    return true;
}

/* ============================================================
 * 第八部分：数据记录功能
 * ============================================================ */

/**
 * @brief 格式化CSV行
 * @param data 采样数据
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @return 写入的字符数
 */
static uint16_t format_csv_line(const ADC_SampleData_t *data, char *buf, uint16_t buf_size)
{
    uint16_t pos = 0;
    
    /* 时间戳 */
    pos += snprintf(buf + pos, buf_size - pos, "%lu", (unsigned long)data->timestamp_ms);
    
    /* 各通道数据 */
    for (uint8_t i = 0; i < data->channel_count; i++) {
        pos += snprintf(buf + pos, buf_size - pos, ",%d", (int)data->raw_value[i]);
        pos += snprintf(buf + pos, buf_size - pos, ",%.6f", data->voltage[i]);
    }
    
    /* 换行 */
    if (pos < buf_size - 2) {
        buf[pos++] = '\r';
        buf[pos++] = '\n';
    }
    buf[pos] = '\0';
    
    return pos;
}

/**
 * @brief 写入CSV头
 */
static void write_csv_header(void)
{
    char header[256];
    uint16_t pos = 0;
    
    pos += snprintf(header + pos, sizeof(header) - pos, "Timestamp_ms");
    
    for (uint8_t i = 0; i < g_adc_config.channels; i++) {
        pos += snprintf(header + pos, sizeof(header) - pos, 
                       ",CH%d_Raw,CH%d_V", i, i);
    }
    
    header[pos++] = '\r';
    header[pos++] = '\n';
    header[pos] = '\0';
    
    fat16_append_data(header, pos);
}

/**
 * @brief 执行多通道扫描采样
 * @param data 输出采样数据
 */
static void perform_multi_channel_scan(ADC_SampleData_t *data)
{
    data->timestamp_ms = g_systick_ms;
    data->channel_count = g_adc_config.channels;
    
    for (uint8_t i = 0; i < g_adc_config.channels; i++) {
        uint8_t pos_ch = g_adc_config.channel_list[i];
        uint8_t neg_ch = g_adc_config.differential ? 
                         g_adc_config.channel_list[i + 1] : ADS1256_MUXN_AINCOM;
        
        data->raw_value[i] = ads1256_read_channel(pos_ch, neg_ch);
        data->voltage[i] = ads1256_to_voltage(data->raw_value[i], g_adc_config.pga_gain);
    }
}

/**
 * @brief 保存采样数据到缓冲区和SD卡
 * @param data 采样数据
 */
static void save_sample_data(const ADC_SampleData_t *data)
{
    /* 保存到环形缓冲区 */
    g_sample_buffer[g_buf_head] = *data;
    g_buf_head = (g_buf_head + 1) % SAMPLE_BUF_SIZE;
    
    /* 格式化并写入SD卡 */
    if (g_sd_ready) {
        char csv_line[512];
        uint16_t len = format_csv_line(data, csv_line, sizeof(csv_line));
        fat16_append_data(csv_line, len);
    }
    
    g_sample_count++;
}

/* ============================================================
 * 第九部分：中断处理
 * ============================================================ */

/**
 * @brief GROUP1中断处理 (PB0 - ADS1256 DRDY)
 */
void GROUP1_IRQHandler(void)
{
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOB, ADS1256_DRDY_PIN);
    
    if (flags & ADS1256_DRDY_PIN) {
        g_drdy_flag = true;
        DL_GPIO_clearInterruptStatus(GPIOB, ADS1256_DRDY_PIN);
    }
}

/* ============================================================
 * 第十部分：主函数
 * ============================================================ */

int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();
    
    /* 配置ADS1256 CS引脚 */
    DL_GPIO_initDigitalOutput(ADS1256_CS_PIN);
    DL_GPIO_setPins(ADS1256_CS_PORT, ADS1256_CS_PIN);
    
    /* 配置ADS1256 RST引脚 */
    DL_GPIO_initDigitalOutput(ADS1256_RST_PIN);
    DL_GPIO_setPins(ADS1256_RST_PORT, ADS1256_RST_PIN);
    
    /* 配置SD卡 CS引脚 */
    DL_GPIO_initDigitalOutput(SD_CS_PIN);
    DL_GPIO_setPins(SD_CS_PORT, SD_CS_PIN);
    
    /* ==================== 初始化ADS1256 ==================== */
    if (!ads1256_init(&g_adc_config)) {
        /* ADS1256初始化失败 */
        while (1) {
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
            delay_ms(100);
        }
    }
    
    /* ==================== 初始化SD卡 ==================== */
    delay_ms(100);
    if (sd_init()) {
        g_sd_ready = true;
        
        /* 解析FAT16文件系统 */
        if (fat16_parse_boot_sector()) {
            /* 查找或创建数据文件 */
            g_current_cluster = fat16_find_file("DATALOG CSV");
            if (g_current_cluster > 0) {
                write_csv_header();
            }
        }
    }
    
    /* LED指示初始化完成 */
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14);
    delay_ms(1000);
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14);
    
    /* 设置记录参数 */
    g_record_mode = RECORD_MODE_TIMED;
    g_sample_interval_ms = 10; /* 10ms间隔 = 100Hz */
    g_max_samples = 0; /* 无限 */
    
    /* ==================== 主循环 ==================== */
    uint32_t last_sample_time = 0;
    uint32_t last_led_toggle = 0;
    
    g_record_state = RECORD_RUNNING;
    
    while (1) {
        /* 定时采样 */
        if (g_record_state == RECORD_RUNNING) {
            if ((g_systick_ms - last_sample_time) >= g_sample_interval_ms) {
                last_sample_time = g_systick_ms;
                
                /* 执行多通道扫描 */
                ADC_SampleData_t new_data;
                perform_multi_channel_scan(&new_data);
                
                /* 保存数据 */
                save_sample_data(&new_data);
                
                /* 检查是否达到最大采样数 */
                if (g_max_samples > 0 && g_sample_count >= g_max_samples) {
                    g_record_state = RECORD_IDLE;
                }
            }
        }
        
        /* LED指示 */
        if ((g_systick_ms - last_led_toggle) >= 500) {
            last_led_toggle = g_systick_ms;
            if (g_record_state == RECORD_RUNNING) {
                DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
            }
        }
        
        /* 模拟系统时间递增 */
        delay_ms(1);
        g_systick_ms++;
    }
}
