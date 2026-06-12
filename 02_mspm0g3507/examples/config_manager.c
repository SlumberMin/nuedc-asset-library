/**
 * @file config_manager.c
 * @brief 配置管理器 - FM24CL64 + JSON格式 + 版本控制 + 备份恢复
 * @platform MSPM0G3507
 *
 * 硬件连接：
 *   FM24CL64 铁电RAM (I2C0):
 *     SDA  -> PA0  (I2C0_SDA)
 *     SCL  -> PA1  (I2C0_SCL)
 *     A0   -> GND
 *     A1   -> GND
 *     A2   -> GND
 *     WP   -> GND (写保护禁用)
 *     VCC  -> 3.3V
 *
 *   UART调试 (UART0):
 *     TX   -> PA11 (UART0_TX)
 *     RX   -> PA10 (UART0_RX)
 *
 *   按键:
 *     PB0 -> 保存配置
 *     PB1 -> 恢复默认
 *     PB2 -> 备份配置
 *     PB3 -> 导出JSON (串口)
 *
 *   LED:
 *     PB14 -> 操作成功指示
 *     PB15 -> 错误指示
 *
 * 功能：
 *   - 在FM24CL64中存储JSON格式的系统配置
 *   - 支持：读取/写入键值对、类型安全、版本控制
 *   - 自动备份/恢复（双区域冗余存储）
 *   - 配置变更日志
 *   - 通过串口导出/导入JSON配置
 *   - 校验和保护，防止配置损坏
 *
 * 存储布局 (FM24CL64, 8KB):
 *   [0x0000 ~ 0x03FF] 配置区A (1KB) — 主配置JSON
 *   [0x0400 ~ 0x07FF] 配置区B (1KB) — 备份配置JSON
 *   [0x0800 ~ 0x0FFF] 日志区 (2KB) — 变更日志
 *   [0x1000 ~ 0x1FFF] 扩展区 (4KB) — 保留
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/* ===== FM24CL64 驱动 ===== */
#define FM24CL64_ADDR       0x50
#define FM24CL64_SIZE       8192

#define I2C_TIMEOUT         10000

/* 存储地址 */
#define CFG_ADDR_PRIMARY    0x0000
#define CFG_ADDR_BACKUP     0x0400
#define CFG_ADDR_LOG        0x0800
#define CFG_ADDR_EXT        0x1000
#define CFG_SECTOR_SIZE     1024

/* 配置头 */
#define CFG_MAGIC           0x4A534F4E  /* "JSON" */
#define CFG_MAX_JSON_SIZE   1000        /* JSON数据最大长度 */

/* 日志 */
#define LOG_ENTRY_SIZE      64
#define LOG_MAX_ENTRIES     32          /* 2KB / 64 = 32条 */

/* 配置头部结构 */
typedef struct {
    uint32_t magic;         /* 魔数 */
    uint16_t version;       /* 配置版本号 */
    uint16_t json_len;      /* JSON数据长度 */
    uint16_t checksum;      /* JSON数据校验和 */
    uint16_t seq_number;    /* 序列号(每次写入递增) */
    uint8_t  reserved[4];   /* 保留 */
} ConfigHeader_t;  /* 16字节 */

/* 日志条目 */
typedef struct {
    uint32_t timestamp;     /* 时间戳(简化：运行tick) */
    uint16_t version;       /* 配置版本 */
    uint8_t  action;        /* 操作类型: 0=创建, 1=修改, 2=删除, 3=恢复 */
    uint8_t  key_len;       /* 键名长度 */
    char     key[24];       /* 键名 */
    char     old_value[16]; /* 旧值 */
    char     new_value[16]; /* 新值 */
} LogEntry_t;  /* 64字节 */

/* ===== I2C 读写 ===== */
static bool FM24_WriteBytes(uint16_t addr, const uint8_t *data, uint16_t len)
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
        uint32_t timeout = I2C_TIMEOUT;
        while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
            if (--timeout == 0) return false;
        }
    }
    return true;
}

static bool FM24_ReadBytes(uint16_t addr, uint8_t *data, uint16_t len)
{
    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)(addr >> 8);
    addr_buf[1] = (uint8_t)(addr & 0xFF);

    DL_I2C_flushControllerTXFIFO(I2C0);
    DL_I2C_fillControllerTXFIFO(I2C0, addr_buf, 2);
    DL_I2C_startControllerTransfer(I2C0, FM24CL64_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    uint32_t timeout = I2C_TIMEOUT;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0) return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C0);
    DL_I2C_startControllerTransfer(I2C0, FM24CL64_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_RX, len);
    timeout = I2C_TIMEOUT;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0) return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        data[i] = DL_I2C_receiveControllerData(I2C0);
    }
    return true;
}

/* ===== 校验和 ===== */
static uint16_t CalcChecksum(const uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

/* ===== UART 发送 ===== */
static void UART_Print(const char *str)
{
    while (*str) {
        DL_UART_main_transmitData(UART0, *str);
        while (DL_UART_isBusy(UART0)) {}
        str++;
    }
}

static void UART_Println(const char *str)
{
    UART_Print(str);
    UART_Print("\r\n");
}

static void UART_PrintHex(uint8_t val)
{
    const char hex[] = "0123456789ABCDEF";
    DL_UART_main_transmitData(UART0, hex[(val >> 4) & 0xF]);
    while (DL_UART_isBusy(UART0)) {}
    DL_UART_main_transmitData(UART0, hex[val & 0xF]);
    while (DL_UART_isBusy(UART0)) {}
}

/* ===== LED ===== */
#define LED_OK_ON()    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14)
#define LED_OK_OFF()   DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14)
#define LED_ERR_ON()   DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_15)
#define LED_ERR_OFF()  DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_15)

/* ===== 简易JSON构建器 ===== */
/* 由于MCU资源有限，使用简化JSON格式，避免完整的解析器 */

/* JSON缓冲区 */
static char json_buf[CFG_MAX_JSON_SIZE];
static uint16_t json_len = 0;

/* 初始化JSON */
static void JSON_Begin(void)
{
    json_buf[0] = '{';
    json_len = 1;
}

/* 添加字符串键值对 */
static bool JSON_AddString(const char *key, const char *value)
{
    /* 需要: "key":"value", 最长3+strlen(key)+strlen(value) */
    uint16_t needed = strlen(key) + strlen(value) + 8;
    if (json_len + needed >= CFG_MAX_JSON_SIZE - 2) return false;

    if (json_len > 1) json_buf[json_len++] = ',';
    json_buf[json_len++] = '"';
    strcpy(&json_buf[json_len], key); json_len += strlen(key);
    json_buf[json_len++] = '"';
    json_buf[json_len++] = ':';
    json_buf[json_len++] = '"';
    strcpy(&json_buf[json_len], value); json_len += strlen(value);
    json_buf[json_len++] = '"';
    return true;
}

/* 添加整数键值对 */
static bool JSON_AddInt(const char *key, int32_t value)
{
    char val_str[12];
    int p = 0;
    if (value < 0) { val_str[p++] = '-'; value = -value; }
    char tmp[10]; int t = 0;
    if (value == 0) { tmp[t++] = '0'; }
    else { while (value > 0) { tmp[t++] = '0' + value % 10; value /= 10; } }
    while (t > 0) val_str[p++] = tmp[--t];
    val_str[p] = '\0';

    uint16_t needed = strlen(key) + strlen(val_str) + 6;
    if (json_len + needed >= CFG_MAX_JSON_SIZE - 2) return false;

    if (json_len > 1) json_buf[json_len++] = ',';
    json_buf[json_len++] = '"';
    strcpy(&json_buf[json_len], key); json_len += strlen(key);
    json_buf[json_len++] = '"';
    json_buf[json_len++] = ':';
    strcpy(&json_buf[json_len], val_str); json_len += strlen(val_str);
    return true;
}

/* 添加浮点键值对（x1000整数存储） */
static bool JSON_AddFloat(const char *key, float value)
{
    /* 整数部分 */
    int32_t ipart = (int32_t)value;
    int32_t fpart = (int32_t)((value - (float)ipart) * 1000);
    if (fpart < 0) fpart = -fpart;

    char val_str[16];
    int p = 0;
    if (value < 0 && ipart == 0) val_str[p++] = '-';
    /* 整数部分 */
    char tmp[8]; int t = 0;
    int32_t abs_ip = ipart < 0 ? -ipart : ipart;
    if (abs_ip == 0) { tmp[t++] = '0'; }
    else { while (abs_ip > 0) { tmp[t++] = '0' + abs_ip % 10; abs_ip /= 10; } }
    if (ipart < 0) val_str[p++] = '-';
    while (t > 0) val_str[p++] = tmp[--t];
    val_str[p++] = '.';
    /* 小数部分 */
    val_str[p++] = '0' + (fpart / 100) % 10;
    val_str[p++] = '0' + (fpart / 10) % 10;
    val_str[p++] = '0' + fpart % 10;
    val_str[p] = '\0';

    return JSON_AddString(key, val_str);
}

/* 完成JSON */
static void JSON_End(void)
{
    json_buf[json_len++] = '}';
    json_buf[json_len] = '\0';
}

/* ===== 简易JSON解析器 ===== */
/* 在JSON中查找key对应的字符串值 */
static bool JSON_GetString(const char *json, const char *key, char *value, uint16_t max_len)
{
    /* 查找 "key":" */
    char pattern[64];
    pattern[0] = '"';
    strcpy(&pattern[1], key);
    strcat(pattern, "\":\"");

    const char *found = strstr(json, pattern);
    if (!found) return false;

    found += strlen(pattern);
    const char *end = strchr(found, '"');
    if (!end) return false;

    uint16_t len = (uint16_t)(end - found);
    if (len >= max_len) len = max_len - 1;
    memcpy(value, found, len);
    value[len] = '\0';
    return true;
}

/* 在JSON中查找key对应的整数值 */
static bool JSON_GetInt(const char *json, const char *key, int32_t *value)
{
    char pattern[64];
    pattern[0] = '"';
    strcpy(&pattern[1], key);
    strcat(pattern, "\":");

    const char *found = strstr(json, pattern);
    if (!found) return false;

    found += strlen(pattern);
    *value = 0;
    bool neg = false;
    if (*found == '-') { neg = true; found++; }
    while (*found >= '0' && *found <= '9') {
        *value = *value * 10 + (*found - '0');
        found++;
    }
    if (neg) *value = -*value;
    return true;
}

/* 在JSON中查找key对应的浮点值 */
static bool JSON_GetFloat(const char *json, const char *key, float *value)
{
    char str[20];
    if (!JSON_GetString(json, key, str, sizeof(str))) return false;
    /* 简易atof */
    float result = 0;
    bool neg = false;
    const char *p = str;
    if (*p == '-') { neg = true; p++; }
    while (*p >= '0' && *p <= '9') { result = result * 10 + (*p - '0'); p++; }
    if (*p == '.') {
        p++;
        float frac = 0, div = 10;
        while (*p >= '0' && *p <= '9') { frac += (*p - '0') / div; div *= 10; p++; }
        result += frac;
    }
    *value = neg ? -result : result;
    return true;
}

/* ===== 配置管理器 API ===== */

static uint16_t config_version = 0;
static uint32_t config_seq = 0;
static uint32_t sys_tick = 0;
static uint8_t log_write_idx = 0;

/* 写入配置区域 */
static bool WriteConfigZone(uint16_t addr, const char *json, uint16_t json_len,
                             uint16_t version, uint32_t seq)
{
    ConfigHeader_t hdr;
    hdr.magic = CFG_MAGIC;
    hdr.version = version;
    hdr.json_len = json_len;
    hdr.checksum = CalcChecksum((const uint8_t *)json, json_len);
    hdr.seq_number = (uint16_t)seq;

    /* 写头部 */
    if (!FM24_WriteBytes(addr, (uint8_t *)&hdr, sizeof(ConfigHeader_t))) return false;

    /* 写JSON数据 */
    if (!FM24_WriteBytes(addr + sizeof(ConfigHeader_t), (const uint8_t *)json, json_len))
        return false;

    return true;
}

/* 读取配置区域 */
static bool ReadConfigZone(uint16_t addr, char *json, uint16_t max_len,
                            ConfigHeader_t *hdr)
{
    if (!FM24_ReadBytes(addr, (uint8_t *)hdr, sizeof(ConfigHeader_t))) return false;

    if (hdr->magic != CFG_MAGIC) return false;
    if (hdr->json_len > max_len) return false;

    if (!FM24_ReadBytes(addr + sizeof(ConfigHeader_t), (uint8_t *)json, hdr->json_len))
        return false;

    json[hdr->json_len] = '\0';

    /* 校验 */
    uint16_t cs = CalcChecksum((const uint8_t *)json, hdr->json_len);
    if (cs != hdr->checksum) return false;

    return true;
}

/* 写入日志 */
static void WriteLog(uint8_t action, const char *key, const char *old_val, const char *new_val)
{
    LogEntry_t entry;
    entry.timestamp = sys_tick;
    entry.version = config_version;
    entry.action = action;
    entry.key_len = strlen(key);
    strncpy(entry.key, key, 23); entry.key[23] = '\0';
    strncpy(entry.old_value, old_val ? old_val : "", 15); entry.old_value[15] = '\0';
    strncpy(entry.new_value, new_val ? new_val : "", 15); entry.new_value[15] = '\0';

    uint16_t addr = CFG_ADDR_LOG + log_write_idx * LOG_ENTRY_SIZE;
    FM24_WriteBytes(addr, (uint8_t *)&entry, sizeof(LogEntry_t));
    log_write_idx = (log_write_idx + 1) % LOG_MAX_ENTRIES;
}

/* ===== 公共 API ===== */

/* 初始化配置管理器 */
int8_t Config_Init(void)
{
    /* 尝试读取主配置 */
    ConfigHeader_t hdr;
    bool primary_ok = ReadConfigZone(CFG_ADDR_PRIMARY, json_buf, CFG_MAX_JSON_SIZE, &hdr);

    if (primary_ok) {
        config_version = hdr.version;
        config_seq = hdr.seq_number;
        UART_Println("[CONFIG] Primary config loaded OK");
        return 0;
    }

    /* 主配置损坏，尝试备份 */
    UART_Println("[CONFIG] Primary corrupted, trying backup...");
    bool backup_ok = ReadConfigZone(CFG_ADDR_BACKUP, json_buf, CFG_MAX_JSON_SIZE, &hdr);

    if (backup_ok) {
        config_version = hdr.version;
        config_seq = hdr.seq_number;
        /* 恢复主配置 */
        WriteConfigZone(CFG_ADDR_PRIMARY, json_buf, hdr.json_len, hdr.version, hdr.seq_number);
        UART_Println("[CONFIG] Backup restored to primary");
        WriteLog(3, "SYSTEM", "CORRUPT", "RESTORED");
        return 0;
    }

    /* 两者都损坏，加载默认配置 */
    UART_Println("[CONFIG] No valid config, loading defaults...");
    Config_LoadDefaults();
    return 1;
}

/* 加载默认配置 */
void Config_LoadDefaults(void)
{
    JSON_Begin();
    JSON_AddString("device_name", "MSPM0G3507");
    JSON_AddString("firmware", "v1.0.0");
    JSON_AddInt("mode", 0);
    JSON_AddInt("baudrate", 115200);
    JSON_AddFloat("voltage_ref", 3.3f);
    JSON_AddFloat("adc_gain", 1.0f);
    JSON_AddFloat("temp_offset", 0.0f);
    JSON_AddInt("log_level", 2);
    JSON_AddInt("auto_save", 1);
    JSON_AddInt("timeout_ms", 5000);
    JSON_AddString("wifi_ssid", "");
    JSON_AddString("wifi_pass", "");
    JSON_AddInt("brightness", 80);
    JSON_End();

    config_version = 1;
    config_seq = 0;

    /* 写入两个区域 */
    WriteConfigZone(CFG_ADDR_PRIMARY, json_buf, json_len, config_version, config_seq);
    WriteConfigZone(CFG_ADDR_BACKUP, json_buf, json_len, config_version, config_seq);

    WriteLog(0, "SYSTEM", NULL, "DEFAULTS");
    LED_OK_ON();
}

/* 保存配置（主+备份双写） */
int8_t Config_Save(void)
{
    config_version++;
    config_seq++;

    if (!WriteConfigZone(CFG_ADDR_PRIMARY, json_buf, json_len, config_version, config_seq)) {
        LED_ERR_ON();
        UART_Println("[CONFIG] Save FAILED (primary)");
        return -1;
    }

    if (!WriteConfigZone(CFG_ADDR_BACKUP, json_buf, json_len, config_version, config_seq)) {
        UART_Println("[CONFIG] Warning: backup write failed");
    }

    WriteLog(1, "SYSTEM", NULL, "SAVED");
    LED_OK_ON();
    UART_Println("[CONFIG] Config saved OK");
    return 0;
}

/* 恢复备份 */
int8_t Config_RestoreBackup(void)
{
    ConfigHeader_t hdr;
    if (!ReadConfigZone(CFG_ADDR_BACKUP, json_buf, CFG_MAX_JSON_SIZE, &hdr)) {
        UART_Println("[CONFIG] Backup invalid");
        return -1;
    }

    config_version = hdr.version;
    config_seq = hdr.seq_number;
    json_len = hdr.json_len;

    WriteConfigZone(CFG_ADDR_PRIMARY, json_buf, json_len, config_version, config_seq);
    WriteLog(3, "SYSTEM", "BACKUP", "RESTORED");
    UART_Println("[CONFIG] Backup restored");
    return 0;
}

/* 获取字符串配置项 */
bool Config_GetString(const char *key, char *value, uint16_t max_len)
{
    return JSON_GetString(json_buf, key, value, max_len);
}

/* 设置字符串配置项 */
bool Config_SetString(const char *key, const char *value)
{
    /* 读取旧值用于日志 */
    char old_val[20];
    JSON_GetString(json_buf, key, old_val, sizeof(old_val));

    /* 重建JSON（简化方法：在末尾追加新键值，旧的会被覆盖优先级低） */
    /* 更好的方法：删除旧键并重写，但为简化，直接重写整个配置 */
    /* 这里用简单方法：在JSON结束符前插入 */
    /* 由于简化JSON不支持修改，这里重新构建 */

    /* 保存当前JSON到临时缓冲 */
    char temp[CFG_MAX_JSON_SIZE];
    memcpy(temp, json_buf, json_len + 1);

    /* 简化实现：在现有JSON的 '}' 前查找并替换 */
    /* 查找 "key":"xxx" 模式 */
    char pattern[64];
    pattern[0] = '"';
    strcpy(&pattern[1], key);
    strcat(pattern, "\":\"");

    char *found = strstr(json_buf, pattern);
    if (found) {
        /* 找到旧值，替换 */
        char *val_start = found + strlen(pattern);
        char *val_end = strchr(val_start, '"');
        if (val_end) {
            uint16_t old_len = (uint16_t)(val_end - val_start);
            uint16_t new_len = strlen(value);
            if (new_len != old_len) {
                /* 需要移位 */
                int16_t diff = (int16_t)new_len - (int16_t)old_len;
                if (json_len + diff >= CFG_MAX_JSON_SIZE) return false;
                memmove(val_start + new_len, val_start + old_len,
                        json_len - (val_start + old_len - json_buf));
                json_len += diff;
            }
            memcpy(val_start, value, new_len);
            json_buf[json_len] = '\0';
        }
    } else {
        /* 新键，插入到'}'前 */
        if (json_len + strlen(key) + strlen(value) + 8 >= CFG_MAX_JSON_SIZE) return false;
        char *closing = &json_buf[json_len - 1];
        if (*closing == '}') {
            /* 插入: ,"key":"value" */
            uint16_t insert_len = strlen(key) + strlen(value) + 7;
            char insert_buf[80];
            insert_buf[0] = ',';
            insert_buf[1] = '"';
            strcpy(&insert_buf[2], key);
            uint16_t p = 2 + strlen(key);
            insert_buf[p++] = '"';
            insert_buf[p++] = ':';
            insert_buf[p++] = '"';
            strcpy(&insert_buf[p], value);
            p += strlen(value);
            insert_buf[p++] = '"';
            insert_buf[p] = '\0';

            memmove(closing + insert_len, closing, json_len - (closing - json_buf) + 1);
            memcpy(closing, insert_buf, insert_len);
            json_len += insert_len;
        }
    }

    WriteLog(1, key, old_val, value);
    return true;
}

/* 获取整数配置项 */
bool Config_GetInt(const char *key, int32_t *value)
{
    return JSON_GetInt(json_buf, key, value);
}

/* 设置整数配置项 */
bool Config_SetInt(const char *key, int32_t value)
{
    char val_str[12];
    int p = 0;
    if (value < 0) { val_str[p++] = '-'; value = -value; }
    char tmp[10]; int t = 0;
    if (value == 0) { tmp[t++] = '0'; }
    else { while (value > 0) { tmp[t++] = '0' + value % 10; value /= 10; } }
    while (t > 0) val_str[p++] = tmp[--t];
    val_str[p] = '\0';

    char old_val[20];
    JSON_GetString(json_buf, key, old_val, sizeof(old_val));

    return Config_SetString(key, val_str);
}

/* 获取浮点配置项 */
bool Config_GetFloat(const char *key, float *value)
{
    return JSON_GetFloat(json_buf, key, value);
}

/* 设置浮点配置项 */
bool Config_SetFloat(const char *key, float value)
{
    char val_str[20];
    int32_t ipart = (int32_t)value;
    int32_t fpart = (int32_t)((value - (float)ipart) * 1000);
    if (fpart < 0) fpart = -fpart;
    int p = 0;
    if (value < 0 && ipart == 0) val_str[p++] = '-';
    char tmp[8]; int t = 0;
    int32_t abs_ip = ipart < 0 ? -ipart : ipart;
    if (abs_ip == 0) { tmp[t++] = '0'; }
    else { while (abs_ip > 0) { tmp[t++] = '0' + abs_ip % 10; abs_ip /= 10; } }
    if (ipart < 0) val_str[p++] = '-';
    while (t > 0) val_str[p++] = tmp[--t];
    val_str[p++] = '.';
    val_str[p++] = '0' + (fpart / 100) % 10;
    val_str[p++] = '0' + (fpart / 10) % 10;
    val_str[p++] = '0' + fpart % 10;
    val_str[p] = '\0';

    return Config_SetString(key, val_str);
}

/* 删除配置项 */
bool Config_Delete(const char *key)
{
    char pattern[64];
    pattern[0] = '"';
    strcpy(&pattern[1], key);
    strcat(pattern, "\":\"");

    char *found = strstr(json_buf, pattern);
    if (!found) return false;

    char *val_start = found + strlen(pattern);
    char *val_end = strchr(val_start, '"');
    if (!val_end) return false;
    val_end++; /* 跳过结束引号 */

    /* 如果前一个是逗号，也删除逗号 */
    char *start = found;
    if (start > json_buf && *(start - 1) == ',') start--;

    uint16_t del_len = (uint16_t)(val_end - start);
    memmove(start, val_end, json_len - (val_end - json_buf) + 1);
    json_len -= del_len;

    WriteLog(2, key, NULL, "DELETED");
    return true;
}

/* 获取配置版本 */
uint16_t Config_GetVersion(void) { return config_version; }

/* 导出JSON到串口 */
void Config_ExportUART(void)
{
    UART_Println("=== CONFIG JSON ===");
    UART_Println(json_buf);
    UART_Println("=== END ===");
}

/* 打印日志到串口 */
void Config_PrintLog(void)
{
    UART_Println("=== CONFIG LOG ===");
    for (uint8_t i = 0; i < LOG_MAX_ENTRIES; i++) {
        LogEntry_t entry;
        uint16_t addr = CFG_ADDR_LOG + i * LOG_ENTRY_SIZE;
        if (!FM24_ReadBytes(addr, (uint8_t *)&entry, sizeof(LogEntry_t))) continue;
        if (entry.timestamp == 0 && entry.version == 0) continue;

        UART_Print("T=");
        /* 简化时间输出 */
        char tick_str[10];
        uint32_t t = entry.timestamp;
        int p = 0; char tmp[10]; int ti = 0;
        if (t == 0) { tmp[ti++] = '0'; }
        else { while (t > 0) { tmp[ti++] = '0' + t % 10; t /= 10; } }
        while (ti > 0) tick_str[p++] = tmp[--ti];
        tick_str[p] = '\0';
        UART_Print(tick_str);

        UART_Print(" V=");
        char v_str[6];
        v_str[0] = '0' + entry.version / 10; v_str[1] = '0' + entry.version % 10;
        v_str[2] = '\0';
        UART_Print(v_str);

        UART_Print(" ACT=");
        DL_UART_main_transmitData(UART0, '0' + entry.action);
        while (DL_UART_isBusy(UART0)) {}

        UART_Print(" KEY=");
        UART_Print(entry.key);

        UART_Print(" -> ");
        UART_Println(entry.new_value);
    }
    UART_Println("=== END ===");
}

/* ===== 按键 ===== */
#define BTN_SAVE    (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_0)))
#define BTN_RESET   (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_1)))
#define BTN_BACKUP  (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2)))
#define BTN_EXPORT  (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3)))

static void delay_ms(uint32_t ms) { delay_cycles(ms * 16000); }

/* ===== 按键消抖 ===== */
typedef struct { uint8_t prev, pressed; } Btn_t;
static void ScanBtn(Btn_t *b, uint8_t raw) { b->pressed = raw && !b->prev; b->prev = raw; }

/* ===== 主函数 ===== */
int main(void)
{
    SYSCFG_DL_init();

    /* UART初始化消息 */
    UART_Println("\r\n=== Config Manager v1.0 ===");
    UART_Println("FM24CL64 + JSON + Version Control");

    /* 初始化配置管理器 */
    int8_t ret = Config_Init();
    if (ret == 1) {
        UART_Println("[MAIN] Default config loaded");
    } else if (ret == 0) {
        UART_Println("[MAIN] Config loaded from FM24CL64");
    }

    /* 打印当前配置 */
    UART_Println("\r\nCurrent config:");
    Config_ExportUART();

    /* 主循环 */
    Btn_t btn_save = {0}, btn_reset = {0}, btn_backup = {0}, btn_export = {0};
    uint32_t led_timer = 0;
    uint32_t tick = 0;

    while (1) {
        /* 按键扫描 */
        ScanBtn(&btn_save, BTN_SAVE);
        ScanBtn(&btn_reset, BTN_RESET);
        ScanBtn(&btn_backup, BTN_BACKUP);
        ScanBtn(&btn_export, BTN_EXPORT);

        /* 保存配置 */
        if (btn_save.pressed) {
            UART_Println("\r\n[BTN] Saving config...");
            /* 演示：修改一些值 */
            Config_SetInt("mode", (tick / 10) % 5);
            Config_SetFloat("temp_offset", (float)(tick % 100) * 0.1f);
            Config_SetInt("brightness", 50 + (tick % 50));
            Config_Save();
            LED_OK_ON();
            led_timer = 50;
        }

        /* 恢复默认 */
        if (btn_reset.pressed) {
            UART_Println("\r\n[BTN] Reset to defaults...");
            Config_LoadDefaults();
            Config_Save();
            UART_Println("Defaults saved:");
            Config_ExportUART();
            LED_OK_ON();
            led_timer = 50;
        }

        /* 备份 */
        if (btn_backup.pressed) {
            UART_Println("\r\n[BTN] Creating backup...");
            /* 备份就是当前配置已自动双写，这里显示确认 */
            ConfigHeader_t hdr;
            if (ReadConfigZone(CFG_ADDR_BACKUP, json_buf, CFG_MAX_JSON_SIZE, &hdr)) {
                UART_Println("Backup verified OK");
                LED_OK_ON();
            } else {
                UART_Println("Backup verify FAILED");
                LED_ERR_ON();
            }
            led_timer = 50;
        }

        /* 导出到串口 */
        if (btn_export.pressed) {
            UART_Println("\r\n[BTN] Exporting config...");
            Config_ExportUART();
            UART_Println("\r\nConfig log:");
            Config_PrintLog();
        }

        /* LED闪烁控制 */
        if (led_timer > 0) {
            led_timer--;
            if (led_timer == 0) {
                LED_OK_OFF();
                LED_ERR_OFF();
            }
        }

        /* 定期状态输出 */
        if (tick % 500 == 0) {
            UART_Print("[STATUS] v=");
            char v[4];
            v[0] = '0' + Config_GetVersion() / 10;
            v[1] = '0' + Config_GetVersion() % 10;
            v[2] = '\0';
            UART_Println(v);
        }

        delay_ms(20);
        tick++;
        sys_tick = tick;
    }

    return 0;
}
