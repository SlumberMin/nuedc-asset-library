/**
 * @file spi_flash_datalogger.c
 * @brief SPI Flash数据记录器 - W25Qxx多通道ADC + 时间戳
 * @platform MSPM0G3507
 *
 * 硬件连接：
 *   W25Qxx Flash (SPI):
 *     SCK  -> PA10 (SPI0_SCK)
 *     MOSI -> PA8  (SPI0_MOSI)
 *     MISO -> PA9  (SPI0_MISO)
 *     CS   -> PA12 (GPIO)
 *
 *   ADC通道:
 *     PA25 -> ADC0_CH5 (通道1)
 *     PA26 -> ADC0_CH6 (通道2)
 *     PA27 -> ADC0_CH7 (通道3)
 *     PA22 -> ADC0_CH4 (通道4)
 *
 *   LED指示:
 *     PB0 -> 记录中指示灯
 *     PB1 -> 数据满指示灯
 *
 *   按键:
 *     PB2 -> 开始/停止记录
 *     PB3 -> 回放/查看
 *     PB4 -> 擦除数据
 *
 * 功能：4通道ADC数据采集，带时间戳存储到Flash，支持回放和导出
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===== W25Qxx Flash驱动 ===== */
#define W25Q_CS_LOW()   DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_12)
#define W25Q_CS_HIGH()  DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_12)

#define W25Q_CMD_WRITE_ENABLE    0x06
#define W25Q_CMD_WRITE_DISABLE   0x04
#define W25Q_CMD_READ_STATUS     0x05
#define W25Q_CMD_READ_DATA       0x03
#define W25Q_CMD_PAGE_PROGRAM    0x02
#define W25Q_CMD_SECTOR_ERASE    0x20   /* 4KB擦除 */
#define W25Q_CMD_BLOCK_ERASE     0xD8   /* 64KB擦除 */
#define W25Q_CMD_CHIP_ERASE      0xC7
#define W25Q_CMD_JEDEC_ID        0x9F
#define W25Q_CMD_POWER_DOWN      0xB9
#define W25Q_CMD_RELEASE_PD      0xAB

#define W25Q_PAGE_SIZE     256
#define W25Q_SECTOR_SIZE   4096
#define W25Q_BLOCK_SIZE    65536

/* Flash大小(根据型号): W25Q16=2MB, W25Q32=4MB, W25Q64=8MB, W25Q128=16MB */
#define W25Q_TOTAL_SIZE    (4 * 1024 * 1024)  /* 假设W25Q32 */

/* ===== 数据记录格式 ===== */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;         /* 魔数 0xDA7A0001 */
    uint32_t timestamp_ms;  /* 系统运行时间(ms) */
    uint16_t channel[4];    /* 4通道ADC值 */
    uint16_t checksum;      /* 校验和 */
} DataRecord;
#pragma pack(pop)

#define RECORD_SIZE     sizeof(DataRecord)   /* 20字节 */
#define RECORDS_PER_PAGE (W25Q_PAGE_SIZE / RECORD_SIZE)  /* 12条/页 */
#define DATA_START_ADDR  0x1000              /* 数据起始地址(跳过前4KB保留区) */
#define CONFIG_ADDR      0x0000              /* 配置信息地址 */

#define MAX_RECORDS ((W25Q_TOTAL_SIZE - DATA_START_ADDR) / RECORD_SIZE)

/* 配置区结构 */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;           /* 0xC0F10001 */
    uint32_t record_count;    /* 当前记录数 */
    uint32_t write_addr;      /* 下一个写入地址 */
    uint32_t sample_interval_ms; /* 采样间隔 */
    uint8_t  channel_mask;    /* 使能通道掩存 */
    uint8_t  reserved[7];
} FlashConfig;
#pragma pack(pop)

/* ===== 全局变量 ===== */
static FlashConfig config;
static uint32_t system_tick_ms = 0;
static bool is_recording = false;
static bool flash_ready = false;
static uint32_t tick_counter = 0;

/* ===== SPI通信 ===== */
static uint8_t SPI_Transfer(uint8_t dat)
{
    DL_SPI_transmitData8(SPI0, dat);
    while (DL_SPI_isBusy(SPI0)) {}
    return DL_SPI_receiveData8(SPI0);
}

static void delay_ms(uint32_t ms)
{
    delay_cycles(ms * (CPUCLK_FREQ / 1000));
}

/* ===== W25Qxx 底层操作 ===== */
static void W25Q_WriteEnable(void)
{
    W25Q_CS_LOW();
    SPI_Transfer(W25Q_CMD_WRITE_ENABLE);
    W25Q_CS_HIGH();
}

static uint8_t W25Q_ReadStatus(void)
{
    W25Q_CS_LOW();
    SPI_Transfer(W25Q_CMD_READ_STATUS);
    uint8_t status = SPI_Transfer(0xFF);
    W25Q_CS_HIGH();
    return status;
}

static void W25Q_WaitBusy(void)
{
    while (W25Q_ReadStatus() & 0x01) {
        delay_ms(1);
    }
}

static uint32_t W25Q_ReadJEDEC(void)
{
    W25Q_CS_LOW();
    SPI_Transfer(W25Q_CMD_JEDEC_ID);
    uint32_t id = 0;
    id |= (uint32_t)SPI_Transfer(0xFF) << 16;
    id |= (uint32_t)SPI_Transfer(0xFF) << 8;
    id |= (uint32_t)SPI_Transfer(0xFF);
    W25Q_CS_HIGH();
    return id;
}

/* 读取数据 */
static void W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    W25Q_CS_LOW();
    SPI_Transfer(W25Q_CMD_READ_DATA);
    SPI_Transfer((addr >> 16) & 0xFF);
    SPI_Transfer((addr >> 8) & 0xFF);
    SPI_Transfer(addr & 0xFF);
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = SPI_Transfer(0xFF);
    }
    W25Q_CS_HIGH();
}

/* 页编程(地址需页对齐, 长度<=256) */
static void W25Q_PageProgram(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    W25Q_WriteEnable();
    W25Q_CS_LOW();
    SPI_Transfer(W25Q_CMD_PAGE_PROGRAM);
    SPI_Transfer((addr >> 16) & 0xFF);
    SPI_Transfer((addr >> 8) & 0xFF);
    SPI_Transfer(addr & 0xFF);
    for (uint32_t i = 0; i < len; i++) {
        SPI_Transfer(buf[i]);
    }
    W25Q_CS_HIGH();
    W25Q_WaitBusy();
}

/* 扇区擦除(4KB) */
static void W25Q_EraseSector(uint32_t addr)
{
    W25Q_WriteEnable();
    W25Q_CS_LOW();
    SPI_Transfer(W25Q_CMD_SECTOR_ERASE);
    SPI_Transfer((addr >> 16) & 0xFF);
    SPI_Transfer((addr >> 8) & 0xFF);
    SPI_Transfer(addr & 0xFF);
    W25Q_CS_HIGH();
    W25Q_WaitBusy();
}

/* 块擦除(64KB) */
static void W25Q_EraseBlock(uint32_t addr)
{
    W25Q_WriteEnable();
    W25Q_CS_LOW();
    SPI_Transfer(W25Q_CMD_BLOCK_ERASE);
    SPI_Transfer((addr >> 16) & 0xFF);
    SPI_Transfer((addr >> 8) & 0xFF);
    SPI_Transfer(addr & 0xFF);
    W25Q_CS_HIGH();
    W25Q_WaitBusy();
}

/* 全片擦除 */
static void W25Q_EraseChip(void)
{
    W25Q_WriteEnable();
    W25Q_CS_LOW();
    SPI_Transfer(W25Q_CMD_CHIP_ERASE);
    W25Q_CS_HIGH();
    W25Q_WaitBusy();
}

/* ===== 配置管理 ===== */
static uint16_t Calc_Checksum(const DataRecord *rec)
{
    uint16_t sum = 0;
    const uint8_t *p = (const uint8_t *)rec;
    /* 校验除checksum字段外的所有数据 */
    for (uint32_t i = 0; i < RECORD_SIZE - 2; i++) {
        sum += p[i];
    }
    return sum;
}

static void Config_Save(void)
{
    W25Q_EraseSector(CONFIG_ADDR);
    uint8_t buf[W25Q_PAGE_SIZE];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &config, sizeof(config));
    W25Q_PageProgram(CONFIG_ADDR, buf, sizeof(config));
}

static void Config_Load(void)
{
    FlashConfig tmp;
    W25Q_Read(CONFIG_ADDR, (uint8_t *)&tmp, sizeof(tmp));
    if (tmp.magic == 0xC0F10001) {
        config = tmp;
    } else {
        /* 首次使用, 初始化默认配置 */
        memset(&config, 0, sizeof(config));
        config.magic = 0xC0F10001;
        config.record_count = 0;
        config.write_addr = DATA_START_ADDR;
        config.sample_interval_ms = 100;  /* 默认100ms */
        config.channel_mask = 0x0F;       /* 4通道全开 */
        Config_Save();
    }
}

/* ===== ADC多通道读取 ===== */
static uint16_t ADC_ReadChannel(DL_ADC12_MEM_IDX mem_idx)
{
    /* 配置序列器选择通道(简化实现, 实际需根据TRM配置) */
    DL_ADC12_startConversion(ADC0);
    while (!DL_ADC12_getStatus(ADC0, DL_ADC12_STATUS_CONVERSION_DONE)) {}
    return (uint16_t)DL_ADC12_getMemResult(ADC0, mem_idx);
}

static void Read_AllChannels(DataRecord *rec)
{
    memset(rec, 0, sizeof(DataRecord));
    rec->magic = 0xDA7A0001;
    rec->timestamp_ms = system_tick_ms;
    rec->channel[0] = (config.channel_mask & 0x01) ? ADC_ReadChannel(DL_ADC12_MEM_IDX_0) : 0;
    rec->channel[1] = (config.channel_mask & 0x02) ? ADC_ReadChannel(DL_ADC12_MEM_IDX_1) : 0;
    rec->channel[2] = (config.channel_mask & 0x04) ? ADC_ReadChannel(DL_ADC12_MEM_IDX_2) : 0;
    rec->channel[3] = (config.channel_mask & 0x08) ? ADC_ReadChannel(DL_ADC12_MEM_IDX_3) : 0;
    rec->checksum = Calc_Checksum(rec);
}

/* ===== 记录管理 ===== */
static void Logger_Start(void)
{
    if (config.record_count >= MAX_RECORDS) {
        /* 数据区满 */
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1);
        return;
    }
    is_recording = true;
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_0);
}

static void Logger_Stop(void)
{
    is_recording = false;
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);
    Config_Save(); /* 保存当前记录状态 */
}

static void Logger_WriteRecord(void)
{
    if (config.record_count >= MAX_RECORDS) {
        Logger_Stop();
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1);
        return;
    }

    DataRecord rec;
    Read_AllChannels(&rec);

    /* 如果写入地址跨扇区边界, 先擦除 */
    uint32_t sector_start = config.write_addr & ~(W25Q_SECTOR_SIZE - 1);
    uint32_t offset_in_sector = config.write_addr - sector_start;
    if (offset_in_sector == 0 && config.write_addr >= DATA_START_ADDR) {
        W25Q_EraseSector(config.write_addr);
    }

    /* 写入记录 */
    uint8_t buf[W25Q_PAGE_SIZE];
    uint32_t page_offset = config.write_addr % W25Q_PAGE_SIZE;
    uint32_t page_start = config.write_addr - page_offset;

    /* 读取当前页内容(可能有其他记录) */
    W25Q_Read(page_start, buf, W25Q_PAGE_SIZE);
    /* 覆盖目标位置 */
    memcpy(&buf[page_offset], &rec, RECORD_SIZE);
    /* 页编程 */
    W25Q_PageProgram(page_start, buf, W25Q_PAGE_SIZE);

    config.write_addr += RECORD_SIZE;
    config.record_count++;
}

/* ===== 数据回放 ===== */
static bool Logger_ReadRecord(uint32_t index, DataRecord *rec)
{
    if (index >= config.record_count) return false;

    uint32_t addr = DATA_START_ADDR + index * RECORD_SIZE;
    W25Q_Read(addr, (uint8_t *)rec, RECORD_SIZE);

    /* 验证 */
    if (rec->magic != 0xDA7A0001) return false;
    if (rec->checksum != Calc_Checksum(rec)) return false;
    return true;
}

/* ===== 数据导出(通过串口) ===== */
static void Logger_ExportAll(void)
{
    /* 通过UART输出CSV格式数据 */
    /* 表头 */
    const char *header = "Index,Time_ms,CH0,CH1,CH2,CH3\r\n";
    /* 通过DL_UART_transmitData逐字节发送 */
    for (int i = 0; header[i]; i++) {
        DL_UART_transmitData(UART0, header[i]);
        while (DL_UART_isBusy(UART0)) {}
    }

    for (uint32_t idx = 0; idx < config.record_count; idx++) {
        DataRecord rec;
        if (!Logger_ReadRecord(idx, &rec)) continue;

        /* 输出CSV行(简易整数转字符串) */
        char line[80];
        int pos = 0;

        /* 简易itoa */
        #define APPEND_NUM(n) do { \
            char tmp[12]; int tl = 0; \
            if ((n) == 0) { tmp[0] = '0'; tl = 1; } \
            else { uint32_t t = (n); while (t > 0) { tmp[tl++] = '0' + t % 10; t /= 10; } } \
            for (int t = tl - 1; t >= 0; t--) line[pos++] = tmp[t]; \
        } while(0)

        APPEND_NUM(idx);
        line[pos++] = ',';
        APPEND_NUM(rec.timestamp_ms);
        line[pos++] = ',';
        APPEND_NUM(rec.channel[0]);
        line[pos++] = ',';
        APPEND_NUM(rec.channel[1]);
        line[pos++] = ',';
        APPEND_NUM(rec.channel[2]);
        line[pos++] = ',';
        APPEND_NUM(rec.channel[3]);
        line[pos++] = '\r';
        line[pos++] = '\n';
        line[pos] = '\0';

        for (int i = 0; i < pos; i++) {
            DL_UART_transmitData(UART0, line[i]);
            while (DL_UART_isBusy(UART0)) {}
        }
    }
}

/* ===== LED指示 ===== */
static void LED_Blink(uint8_t pin, int times, uint32_t interval_ms)
{
    for (int i = 0; i < times; i++) {
        DL_GPIO_setPins(GPIOB, 1 << pin);
        delay_ms(interval_ms);
        DL_GPIO_clearPins(GPIOB, 1 << pin);
        delay_ms(interval_ms);
    }
}

/* ===== 按键消抖 ===== */
static bool Key_Read(uint8_t pin)
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

/* ===== 主函数 ===== */
int main(void)
{
    SYSCFG_DL_init();

    /* 初始化W25Qxx */
    uint32_t jedec = W25Q_ReadJEDEC();
    flash_ready = (jedec != 0 && jedec != 0xFFFFFF);

    if (flash_ready) {
        /* 闪灯确认Flash已识别 */
        LED_Blink(0, 3, 100);
    }

    /* 加载配置 */
    Config_Load();

    /* 初始化LED状态 */
    if (config.record_count >= MAX_RECORDS) {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1); /* 数据满 */
    }

    while (1) {
        /* PB2: 开始/停止记录 */
        if (Key_Read(2)) {
            if (is_recording) {
                Logger_Stop();
            } else {
                Logger_Start();
            }
        }

        /* PB3: 回放导出 */
        if (Key_Read(3)) {
            if (!is_recording) {
                LED_Blink(0, 5, 50); /* 导出指示 */
                Logger_ExportAll();
            }
        }

        /* PB4: 擦除数据 */
        if (Key_Read(4)) {
            if (!is_recording) {
                /* 长按确认: 闪3次后擦除 */
                LED_Blink(1, 3, 200);
                /* 擦除数据区 */
                for (uint32_t addr = 0; addr < W25Q_TOTAL_SIZE; addr += W25Q_BLOCK_SIZE) {
                    W25Q_EraseBlock(addr);
                    /* LED进度 */
                    DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_0);
                }
                /* 重置配置 */
                config.record_count = 0;
                config.write_addr = DATA_START_ADDR;
                Config_Save();
                DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0 | DL_GPIO_PIN_1);
                LED_Blink(0, 3, 100); /* 完成 */
            }
        }

        /* 记录数据 */
        if (is_recording) {
            Logger_WriteRecord();
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_0); /* 记录指示灯闪烁 */
        }

        delay_ms(config.sample_interval_ms);
        system_tick_ms += config.sample_interval_ms;
    }
}
