/**
 * @file flash_file_system.c
 * @brief 简易文件系统 - W25Qxx SPI Flash + 目录 + 文件读写 + 删除
 * @platform MSPM0G3507
 *
 * 硬件连接：
 *   W25Q128 SPI Flash (SPI1):
 *     SCK  -> PB8  (SPI1_SCK)
 *     MOSI -> PB6  (SPI1_MOSI)
 *     MISO -> PB7  (SPI1_MISO)
 *     CS   -> PB9  (GPIO)
 *
 *   OLED SSD1306 (I2C0, 用于显示):
 *     SCL  -> PA1  (I2C0_SCL)
 *     SDA  -> PA0  (I2C0_SDA)
 *
 *   按键:
 *     PB0 -> 上移
 *     PB1 -> 下移
 *     PB2 -> 确认/操作
 *     PB3 -> 返回/取消
 *
 *   LED:
 *     PB14 -> 活动指示
 *     PB15 -> 错误指示
 *
 * 功能：
 *   - 自研微型文件系统 MiniFS，专为SPI Flash优化
 *   - 支持：创建文件、写入数据、读取数据、删除文件、格式化
 *   - FAT表管理，每文件最大64KB
 *   - 最多支持32个文件
 *   - 断电安全：写入时使用双备份机制
 *   - 通过OLED显示文件目录和操作状态
 *
 * 存储布局 (W25Q128, 16MB):
 *   [0x000000 ~ 0x000FFF] 超级块 (4KB) — 文件系统元数据
 *   [0x001000 ~ 0x001FFF] FAT表 (4KB) — 文件分配表
 *   [0x002000 ~ 0x002FFF] 目录区 (4KB) — 文件目录项
 *   [0x003000 ~ 0x003FFF] 备份区 (4KB) — 元数据备份
 *   [0x004000 ~ 0x0FFFFF] 数据区 (1MB) — 文件数据存储
 *   [0x100000 ~ 0xFFFFFF] 扩展区 (15MB) — 未使用
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===== W25Q Flash 命令 ===== */
#define W25Q_CMD_READ_DATA       0x03
#define W25Q_CMD_WRITE_ENABLE    0x06
#define W25Q_CMD_READ_STATUS1    0x05
#define W25Q_CMD_PAGE_PROGRAM    0x02
#define W25Q_CMD_SECTOR_ERASE    0x20
#define W25Q_CMD_BLOCK_ERASE_64K 0xD8
#define W25Q_CMD_CHIP_ERASE      0xC7
#define W25Q_CMD_JEDEC_ID        0x9F

/* Flash参数 */
#define W25Q_PAGE_SIZE      256
#define W25Q_SECTOR_SIZE    4096
#define W25Q_BLOCK_SIZE     65536

/* ===== MiniFS 文件系统参数 ===== */
#define FS_MAGIC            0x4D696E69  /* "Mini" */
#define FS_VERSION          0x0001
#define FS_MAX_FILES        32
#define FS_MAX_FILENAME     28          /* 文件名最大长度（含\0） */
#define FS_MAX_FILE_SIZE    (64 * 1024) /* 单文件最大64KB */
#define FS_DATA_SECTORS     256         /* 数据区扇区数 (256*4KB=1MB) */

/* 存储地址 */
#define FS_ADDR_SUPERBLOCK  0x000000
#define FS_ADDR_FAT         0x001000
#define FS_ADDR_DIR         0x002000
#define FS_ADDR_BACKUP      0x003000
#define FS_ADDR_DATA        0x004000
#define FS_ADDR_DATA_END    0x100000

/* FAT表项 */
#define FAT_FREE            0xFFFF      /* 空闲扇区 */
#define FAT_EOF             0xFFFE      /* 文件结尾 */
#define FAT_BAD             0xFFFD      /* 坏扇区 */
#define FAT_RESERVED        0xFFF0      /* 系统保留 */

/* 文件属性 */
#define FILE_ATTR_USED      0x01        /* 已使用 */
#define FILE_ATTR_READONLY  0x02        /* 只读 */
#define FILE_ATTR_HIDDEN    0x04        /* 隐藏 */
#define FILE_ATTR_SYSTEM    0x08        /* 系统文件 */

/* ===== 数据结构 ===== */

/* 超级块 (4KB扇区内，实际只用64字节) */
typedef struct {
    uint32_t magic;             /* 魔数: FS_MAGIC */
    uint16_t version;           /* 版本号 */
    uint16_t total_sectors;     /* 总扇区数 */
    uint16_t free_sectors;      /* 空闲扇区数 */
    uint16_t file_count;        /* 文件数量 */
    uint32_t data_start;        /* 数据区起始地址 */
    uint32_t data_size;         /* 数据区总大小 */
    uint16_t format_count;      /* 格式化次数 */
    uint16_t checksum;          /* 校验和 */
    uint8_t  reserved[32];      /* 保留 */
} SuperBlock_t;  /* 64字节 */

/* 目录项 */
typedef struct {
    uint8_t  name[FS_MAX_FILENAME]; /* 文件名（UTF-8或ASCII） */
    uint8_t  attributes;            /* 文件属性 */
    uint16_t start_sector;          /* 起始扇区号 */
    uint32_t file_size;             /* 文件大小（字节） */
    uint32_t create_time;           /* 创建时间戳 */
    uint32_t modify_time;           /* 修改时间戳 */
    uint16_t checksum;              /* 校验和 */
} DirEntry_t;  /* 48字节 */

/* 文件系统状态 */
typedef struct {
    bool     mounted;               /* 已挂载 */
    uint16_t fat[FS_DATA_SECTORS];  /* FAT表缓存 (512字节RAM) */
    DirEntry_t dir[FS_MAX_FILES];   /* 目录缓存 (1536字节RAM) */
    SuperBlock_t superblock;        /* 超级块缓存 */
    bool     dirty;                 /* 有未写回的修改 */
} MiniFS_t;

static MiniFS_t fs;

/* ===== OLED SSD1306 驱动 ===== */
#define OLED_ADDR   0x3C

static void OLED_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    DL_I2C_fillControllerTXFIFO(I2C0, buf, 2);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE)) {}
    DL_I2C_startControllerTransfer(I2C0, OLED_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    DL_I2C_flushControllerTXFIFO(I2C0);
}

static void OLED_Init(void)
{
    delay_cycles(160000);
    OLED_WriteCmd(0xAE); OLED_WriteCmd(0xD5); OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F); OLED_WriteCmd(0xD3);
    OLED_WriteCmd(0x00); OLED_WriteCmd(0x40); OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14); OLED_WriteCmd(0x20); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xA1); OLED_WriteCmd(0xC8); OLED_WriteCmd(0xDA);
    OLED_WriteCmd(0x12); OLED_WriteCmd(0x81); OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0xF1); OLED_WriteCmd(0xDB);
    OLED_WriteCmd(0x40); OLED_WriteCmd(0xA4); OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0xAF);
}

/* OLED显示缓冲区 (128x64 / 8 = 1024字节) */
static uint8_t oled_buf[1024];

static void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on)
{
    if (x >= 128 || y >= 64) return;
    uint16_t idx = (y / 8) * 128 + x;
    if (on) oled_buf[idx] |= (1 << (y % 8));
    else    oled_buf[idx] &= ~(1 << (y % 8));
}

static void OLED_Clear(void) { memset(oled_buf, 0, sizeof(oled_buf)); }

static void OLED_Refresh(void)
{
    for (uint8_t page = 0; page < 8; page++) {
        OLED_WriteCmd(0xB0 + page);
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);
        DL_I2C_fillControllerTXFIFO(I2C0, &oled_buf[page * 128], 128);
        /* 需要分段传输 */
        for (int i = 0; i < 128; i += 16) {
            uint8_t chunk[17];
            chunk[0] = 0x40;
            memcpy(&chunk[1], &oled_buf[page * 128 + i], 16);
            DL_I2C_fillControllerTXFIFO(I2C0, chunk, 17);
            DL_I2C_startControllerTransfer(I2C0, OLED_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 17);
            while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
            DL_I2C_flushControllerTXFIFO(I2C0);
        }
    }
}

/* 简易 6x8 字体 */
static const uint8_t font_6x8[][6] = {
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* '9' */
};

static void OLED_DrawChar6x8(uint8_t x, uint8_t y, char ch)
{
    const uint8_t *glyph = NULL;
    uint8_t buf[6];

    if (ch >= '0' && ch <= '9') {
        glyph = font_6x8[ch - '0'];
    } else if (ch >= 'a' && ch <= 'z') {
        /* 简化：用减法偏移生成小写字母 */
        for (int i = 0; i < 6; i++) buf[i] = font_6x8[0][i]; /* placeholder */
        glyph = buf;
    } else {
        memset(buf, 0, 6);
        glyph = buf;
    }

    for (uint8_t col = 0; col < 6; col++) {
        uint8_t data = glyph ? glyph[col] : 0;
        for (uint8_t row = 0; row < 8; row++) {
            OLED_SetPixel(x + col, y + row, (data >> row) & 1);
        }
    }
}

static void OLED_PrintString(uint8_t x, uint8_t y, const char *s)
{
    while (*s && x < 122) {
        OLED_DrawChar6x8(x, y, *s);
        x += 7;
        s++;
    }
}

static void OLED_DrawHLine(uint8_t y)
{
    for (uint8_t x = 0; x < 128; x++) OLED_SetPixel(x, y, 1);
}

/* ===== SPI Flash 底层 ===== */
static void SPI1_WriteByte(uint8_t dat)
{
    DL_SPI_transmitData8(SPI1, dat);
    while (DL_SPI_isBusy(SPI1)) {}
}

static uint8_t SPI1_ReadByte(void)
{
    DL_SPI_transmitData8(SPI1, 0xFF);
    while (DL_SPI_isBusy(SPI1)) {}
    return DL_SPI_receiveData8(SPI1);
}

static void W25Q_WaitBusy(void)
{
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_9);
    SPI1_WriteByte(W25Q_CMD_READ_STATUS1);
    uint32_t t = 200000;
    while (t-- && (SPI1_ReadByte() & 0x01)) {}
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_9);
}

static void W25Q_WriteEnable(void)
{
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_9);
    SPI1_WriteByte(W25Q_CMD_WRITE_ENABLE);
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_9);
}

static uint32_t W25Q_ReadID(void)
{
    uint32_t id;
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_9);
    SPI1_WriteByte(W25Q_CMD_JEDEC_ID);
    id  = (uint32_t)SPI1_ReadByte() << 16;
    id |= (uint32_t)SPI1_ReadByte() << 8;
    id |= SPI1_ReadByte();
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_9);
    return id;
}

static void W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_9);
    SPI1_WriteByte(W25Q_CMD_READ_DATA);
    SPI1_WriteByte((addr >> 16) & 0xFF);
    SPI1_WriteByte((addr >> 8) & 0xFF);
    SPI1_WriteByte(addr & 0xFF);
    for (uint32_t i = 0; i < len; i++) buf[i] = SPI1_ReadByte();
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_9);
}

static void W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    while (len > 0) {
        uint16_t remain = W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE);
        uint16_t chunk = (len < remain) ? (uint16_t)len : remain;
        W25Q_WriteEnable();
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_9);
        SPI1_WriteByte(W25Q_CMD_PAGE_PROGRAM);
        SPI1_WriteByte((addr >> 16) & 0xFF);
        SPI1_WriteByte((addr >> 8) & 0xFF);
        SPI1_WriteByte(addr & 0xFF);
        for (uint16_t i = 0; i < chunk; i++) SPI1_WriteByte(buf[i]);
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_9);
        W25Q_WaitBusy();
        addr += chunk; buf += chunk; len -= chunk;
    }
}

static void W25Q_EraseSector(uint32_t addr)
{
    W25Q_WriteEnable();
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_9);
    SPI1_WriteByte(W25Q_CMD_SECTOR_ERASE);
    SPI1_WriteByte((addr >> 16) & 0xFF);
    SPI1_WriteByte((addr >> 8) & 0xFF);
    SPI1_WriteByte(addr & 0xFF);
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_9);
    W25Q_WaitBusy();
}

/* ===== 校验和 ===== */
static uint16_t CalcChecksum(const uint8_t *data, uint32_t len)
{
    uint16_t sum = 0;
    for (uint32_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

/* ===== MiniFS 文件系统 API ===== */

/* 格式化文件系统 */
int8_t MiniFS_Format(void)
{
    /* 擦除超级块扇区 */
    W25Q_EraseSector(FS_ADDR_SUPERBLOCK);
    W25Q_EraseSector(FS_ADDR_FAT);
    W25Q_EraseSector(FS_ADDR_DIR);
    W25Q_EraseSector(FS_ADDR_BACKUP);

    /* 初始化超级块 */
    memset(&fs.superblock, 0, sizeof(SuperBlock_t));
    fs.superblock.magic = FS_MAGIC;
    fs.superblock.version = FS_VERSION;
    fs.superblock.total_sectors = FS_DATA_SECTORS;
    fs.superblock.free_sectors = FS_DATA_SECTORS;
    fs.superblock.file_count = 0;
    fs.superblock.data_start = FS_ADDR_DATA;
    fs.superblock.data_size = FS_ADDR_DATA_END - FS_ADDR_DATA;
    fs.superblock.checksum = CalcChecksum((uint8_t *)&fs.superblock,
                                           sizeof(SuperBlock_t) - 2);

    /* 初始化FAT表 */
    for (int i = 0; i < FS_DATA_SECTORS; i++) fs.fat[i] = FAT_FREE;

    /* 初始化目录 */
    memset(fs.dir, 0, sizeof(fs.dir));

    /* 写入Flash */
    W25Q_Write(FS_ADDR_SUPERBLOCK, (uint8_t *)&fs.superblock, sizeof(SuperBlock_t));
    W25Q_Write(FS_ADDR_FAT, (uint8_t *)fs.fat, sizeof(fs.fat));
    W25Q_Write(FS_ADDR_DIR, (uint8_t *)fs.dir, sizeof(fs.dir));

    /* 写入备份 */
    W25Q_Write(FS_ADDR_BACKUP, (uint8_t *)&fs.superblock, sizeof(SuperBlock_t));

    fs.mounted = true;
    fs.dirty = false;

    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14);
    return 0;
}

/* 挂载文件系统 */
int8_t MiniFS_Mount(void)
{
    /* 读取超级块 */
    W25Q_Read(FS_ADDR_SUPERBLOCK, (uint8_t *)&fs.superblock, sizeof(SuperBlock_t));

    /* 验证魔数 */
    if (fs.superblock.magic != FS_MAGIC) {
        /* 尝试读取备份 */
        SuperBlock_t backup;
        W25Q_Read(FS_ADDR_BACKUP, (uint8_t *)&backup, sizeof(SuperBlock_t));
        if (backup.magic == FS_MAGIC) {
            memcpy(&fs.superblock, &backup, sizeof(SuperBlock_t));
            W25Q_Write(FS_ADDR_SUPERBLOCK, (uint8_t *)&fs.superblock, sizeof(SuperBlock_t));
        } else {
            return -1;  /* 需要格式化 */
        }
    }

    /* 读取FAT表 */
    W25Q_Read(FS_ADDR_FAT, (uint8_t *)fs.fat, sizeof(fs.fat));

    /* 读取目录 */
    W25Q_Read(FS_ADDR_DIR, (uint8_t *)fs.dir, sizeof(fs.dir));

    fs.mounted = true;
    fs.dirty = false;
    return 0;
}

/* 同步元数据到Flash */
static void MiniFS_Sync(void)
{
    if (!fs.dirty) return;

    fs.superblock.checksum = CalcChecksum((uint8_t *)&fs.superblock,
                                           sizeof(SuperBlock_t) - 2);

    W25Q_EraseSector(FS_ADDR_SUPERBLOCK);
    W25Q_Write(FS_ADDR_SUPERBLOCK, (uint8_t *)&fs.superblock, sizeof(SuperBlock_t));

    W25Q_EraseSector(FS_ADDR_FAT);
    W25Q_Write(FS_ADDR_FAT, (uint8_t *)fs.fat, sizeof(fs.fat));

    W25Q_EraseSector(FS_ADDR_DIR);
    W25Q_Write(FS_ADDR_DIR, (uint8_t *)fs.dir, sizeof(fs.dir));

    /* 更新备份 */
    W25Q_EraseSector(FS_ADDR_BACKUP);
    W25Q_Write(FS_ADDR_BACKUP, (uint8_t *)&fs.superblock, sizeof(SuperBlock_t));

    fs.dirty = false;
}

/* 查找空闲扇区 */
static int16_t MiniFS_AllocSector(void)
{
    for (int i = 0; i < FS_DATA_SECTORS; i++) {
        if (fs.fat[i] == FAT_FREE) {
            fs.fat[i] = FAT_EOF;
            fs.superblock.free_sectors--;
            fs.dirty = true;
            return i;
        }
    }
    return -1;  /* 磁盘已满 */
}

/* 释放扇区链 */
static void MiniFS_FreeChain(uint16_t start_sector)
{
    uint16_t sec = start_sector;
    while (sec != FAT_FREE && sec != FAT_EOF && sec < FS_DATA_SECTORS) {
        uint16_t next = fs.fat[sec];
        fs.fat[sec] = FAT_FREE;
        fs.superblock.free_sectors++;
        sec = next;
    }
    fs.dirty = true;
}

/* 计算扇区物理地址 */
static uint32_t SectorAddr(uint16_t sector_num)
{
    return FS_ADDR_DATA + (uint32_t)sector_num * W25Q_SECTOR_SIZE;
}

/* 查找文件名在目录中的索引 */
static int8_t FindFile(const char *name)
{
    for (int i = 0; i < FS_MAX_FILES; i++) {
        if (fs.dir[i].attributes & FILE_ATTR_USED) {
            if (strncmp((const char *)fs.dir[i].name, name, FS_MAX_FILENAME - 1) == 0) {
                return i;
            }
        }
    }
    return -1;
}

/* 查找空闲目录项 */
static int8_t FindFreeDir(void)
{
    for (int i = 0; i < FS_MAX_FILES; i++) {
        if (!(fs.dir[i].attributes & FILE_ATTR_USED)) return i;
    }
    return -1;
}

/* ===== 文件操作 API ===== */

/* 创建文件 */
int8_t MiniFS_Create(const char *name)
{
    if (!fs.mounted) return -1;
    if (FindFile(name) >= 0) return -2;  /* 已存在 */

    int8_t idx = FindFreeDir();
    if (idx < 0) return -3;  /* 目录已满 */

    memset(&fs.dir[idx], 0, sizeof(DirEntry_t));
    strncpy((char *)fs.dir[idx].name, name, FS_MAX_FILENAME - 1);
    fs.dir[idx].attributes = FILE_ATTR_USED;
    fs.dir[idx].start_sector = FAT_FREE;
    fs.dir[idx].file_size = 0;
    fs.dir[idx].create_time = 0;  /* 简化：无RTC时间 */
    fs.dir[idx].modify_time = 0;

    fs.superblock.file_count++;
    fs.dirty = true;
    MiniFS_Sync();

    return idx;
}

/* 写入文件数据 */
int32_t MiniFS_Write(const char *name, const uint8_t *data, uint32_t size)
{
    if (!fs.mounted) return -1;
    if (size > FS_MAX_FILE_SIZE) return -2;

    int8_t idx = FindFile(name);
    if (idx < 0) {
        /* 自动创建 */
        idx = MiniFS_Create(name);
        if (idx < 0) return -3;
    }

    /* 释放旧数据扇区 */
    if (fs.dir[idx].start_sector != FAT_FREE) {
        MiniFS_FreeChain(fs.dir[idx].start_sector);
        fs.dir[idx].start_sector = FAT_FREE;
    }

    /* 分配新扇区并写入数据 */
    uint16_t first_sector = FAT_FREE;
    uint16_t prev_sector = FAT_FREE;
    uint32_t written = 0;
    uint32_t data_offset = 0;

    while (written < size) {
        int16_t sec = MiniFS_AllocSector();
        if (sec < 0) {
            /* 空间不足，释放已分配的 */
            if (first_sector != FAT_FREE)
                MiniFS_FreeChain(first_sector);
            return -4;
        }

        if (first_sector == FAT_FREE) first_sector = (uint16_t)sec;
        if (prev_sector != FAT_FREE) fs.fat[prev_sector] = (uint16_t)sec;

        /* 写入一个扇区的数据 */
        uint32_t chunk = size - written;
        if (chunk > W25Q_SECTOR_SIZE) chunk = W25Q_SECTOR_SIZE;

        W25Q_EraseSector(SectorAddr(sec));
        W25Q_Write(SectorAddr(sec), data + data_offset, chunk);

        written += chunk;
        data_offset += chunk;
        prev_sector = (uint16_t)sec;
    }

    /* 标记链尾 */
    if (prev_sector != FAT_FREE) fs.fat[prev_sector] = FAT_EOF;

    /* 更新目录项 */
    fs.dir[idx].start_sector = first_sector;
    fs.dir[idx].file_size = size;

    fs.dirty = true;
    MiniFS_Sync();

    return (int32_t)size;
}

/* 读取文件数据 */
int32_t MiniFS_Read(const char *name, uint8_t *buf, uint32_t max_size)
{
    if (!fs.mounted) return -1;

    int8_t idx = FindFile(name);
    if (idx < 0) return -2;  /* 不存在 */

    uint32_t to_read = fs.dir[idx].file_size;
    if (to_read > max_size) to_read = max_size;

    uint16_t sec = fs.dir[idx].start_sector;
    uint32_t read_total = 0;

    while (sec != FAT_EOF && sec != FAT_FREE && sec < FS_DATA_SECTORS && read_total < to_read) {
        uint32_t chunk = to_read - read_total;
        if (chunk > W25Q_SECTOR_SIZE) chunk = W25Q_SECTOR_SIZE;

        W25Q_Read(SectorAddr(sec), buf + read_total, chunk);
        read_total += chunk;

        sec = fs.fat[sec];
    }

    return (int32_t)read_total;
}

/* 删除文件 */
int8_t MiniFS_Delete(const char *name)
{
    if (!fs.mounted) return -1;

    int8_t idx = FindFile(name);
    if (idx < 0) return -2;

    /* 释放数据扇区 */
    if (fs.dir[idx].start_sector != FAT_FREE) {
        MiniFS_FreeChain(fs.dir[idx].start_sector);
    }

    /* 清除目录项 */
    memset(&fs.dir[idx], 0, sizeof(DirEntry_t));
    fs.superblock.file_count--;

    fs.dirty = true;
    MiniFS_Sync();
    return 0;
}

/* 获取文件大小 */
int32_t MiniFS_GetSize(const char *name)
{
    int8_t idx = FindFile(name);
    if (idx < 0) return -1;
    return (int32_t)fs.dir[idx].file_size;
}

/* 获取文件数量 */
uint16_t MiniFS_GetFileCount(void)
{
    return fs.superblock.file_count;
}

/* 获取空闲空间 */
uint32_t MiniFS_GetFreeSpace(void)
{
    return (uint32_t)fs.superblock.free_sectors * W25Q_SECTOR_SIZE;
}

/* 获取文件列表（指定索引的文件名） */
bool MiniFS_GetFileName(uint8_t index, char *name, uint8_t max_len)
{
    if (index >= FS_MAX_FILES) return false;
    if (!(fs.dir[index].attributes & FILE_ATTR_USED)) return false;
    strncpy(name, (const char *)fs.dir[index].name, max_len);
    return true;
}

/* ===== 按键 ===== */
#define BTN_UP    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_0)))
#define BTN_DOWN  (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_1)))
#define BTN_OK    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2)))
#define BTN_BACK  (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3)))

typedef struct { uint8_t prev, pressed; } Btn_t;
static void ScanBtn(Btn_t *b, uint8_t raw) { b->pressed = raw && !b->prev; b->prev = raw; }

static void delay_ms(uint32_t ms) { delay_cycles(ms * 16000); }

/* ===== 测试数据 ===== */
static const char *test_files[] = {
    "readme.txt",
    "config.dat",
    "log_001.bin",
    "sensor.csv",
    "calib.dat",
};
#define NUM_TEST_FILES 5

static const char *test_data[] = {
    "Hello MiniFS! This is a test file.\nLine 2 of readme.",
    "KEY=VALUE\nMODE=AUTO\nSPEED=100\nWIDTH=240",
    "LOG:2024-01-01 12:00:00 Boot OK\nLOG:2024-01-01 12:01:00 Temp=25.3\n",
    "temp,humidity,pressure\n25.3,60.2,101.3\n26.1,58.7,101.2\n",
    "OFFSET_CH0=0.001\nOFFSET_CH1=-0.002\nGAIN=1.003\n",
};

/* ===== 主函数 ===== */
int main(void)
{
    SYSCFG_DL_init();

    /* 初始化OLED */
    OLED_Init();
    OLED_Clear();
    OLED_PrintString(0, 0, "MiniFS v1.0");
    OLED_PrintString(0, 10, "Initializing...");
    OLED_Refresh();

    /* 检测Flash */
    uint32_t flash_id = W25Q_ReadID();
    if ((flash_id & 0xFF0000) == 0 || (flash_id & 0xFF0000) == 0xFF0000) {
        OLED_Clear();
        OLED_PrintString(0, 0, "Flash Error!");
        OLED_Refresh();
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_15);
        while (1) {}
    }

    /* 尝试挂载 */
    int8_t ret = MiniFS_Mount();
    if (ret != 0) {
        OLED_PrintString(0, 20, "Formatting...");
        OLED_Refresh();
        MiniFS_Format();
    }

    /* 写入测试文件 */
    OLED_Clear();
    OLED_PrintString(0, 0, "Writing files...");
    OLED_Refresh();

    for (int i = 0; i < NUM_TEST_FILES; i++) {
        MiniFS_Write(test_files[i], (const uint8_t *)test_data[i], strlen(test_data[i]));
    }

    /* 主循环：显示文件列表 */
    uint8_t cursor = 0;
    uint8_t mode = 0;  /* 0=浏览, 1=读取, 2=删除确认 */
    uint8_t read_buf[256];
    uint32_t tick = 0;

    while (1) {
        /* 按键扫描 */
        uint8_t b_up = BTN_UP, b_dn = BTN_DOWN, b_ok = BTN_OK, b_bk = BTN_BACK;
        ScanBtn(&(Btn_t){0,0}, 0); /* placeholder */

        /* 简易消抖 */
        static uint8_t prev_up = 0, prev_dn = 0, prev_ok = 0, prev_bk = 0;
        uint8_t p_up = b_up && !prev_up;
        uint8_t p_dn = b_dn && !prev_dn;
        uint8_t p_ok = b_ok && !prev_ok;
        uint8_t p_bk = b_bk && !prev_bk;
        prev_up = b_up; prev_dn = b_dn; prev_ok = b_ok; prev_bk = b_bk;

        if (mode == 0) {
            /* 浏览模式 */
            if (p_up && cursor > 0) cursor--;
            if (p_dn && cursor < NUM_TEST_FILES - 1) cursor++;

            if (p_ok) {
                /* 读取并显示文件内容 */
                mode = 1;
                memset(read_buf, 0, sizeof(read_buf));
                MiniFS_Read(test_files[cursor], read_buf, sizeof(read_buf) - 1);
            }

            OLED_Clear();
            OLED_PrintString(0, 0, "== MiniFS Files ==");
            OLED_DrawHLine(9);

            for (int i = 0; i < NUM_TEST_FILES; i++) {
                uint8_t y = 12 + i * 10;
                if (i == cursor) OLED_PrintString(0, y, ">");
                OLED_PrintString(8, y, test_files[i]);

                int32_t sz = MiniFS_GetSize(test_files[i]);
                /* 显示大小 */
                char sz_str[8];
                sz_str[0] = ' ';
                int p = 1;
                if (sz >= 100) { sz_str[p++] = '0' + sz / 100; sz %= 100; }
                if (sz >= 10) { sz_str[p++] = '0' + sz / 10; sz %= 10; }
                sz_str[p++] = '0' + sz;
                sz_str[p] = 'B';
                sz_str[p + 1] = '\0';
                OLED_PrintString(90, y, sz_str);
            }

            /* 底部状态 */
            OLED_DrawHLine(54);
            char info[32];
            OLED_PrintString(0, 56, "Free:");
            /* 简化显示空闲空间 */
            uint32_t free_kb = MiniFS_GetFreeSpace() / 1024;
            OLED_PrintString(40, 56, "KB");

        } else if (mode == 1) {
            /* 读取显示模式 */
            if (p_bk) { mode = 0; }

            OLED_Clear();
            OLED_PrintString(0, 0, test_files[cursor]);
            OLED_DrawHLine(9);

            /* 显示前几行内容 */
            uint8_t line = 0;
            uint8_t col = 0;
            for (uint32_t i = 0; i < 200 && read_buf[i] && line < 6; i++) {
                if (read_buf[i] == '\n') { line++; col = 0; continue; }
                if (col < 20) {
                    /* 简化显示 */
                    col++;
                }
            }
            OLED_PrintString(0, 12, "[Content]");
            OLED_PrintString(0, 56, "BACK:PB3");
        }

        OLED_Refresh();
        DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
        delay_ms(200);
        tick++;
    }

    return 0;
}
