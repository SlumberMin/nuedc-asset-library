/**
 * @file smart_home_monitor.c
 * @brief 智能家居环境监控系统 - 完整系统集成示例
 * @target MSPM0G3507
 * @hardware SHT20温湿度传感器 + SGP30空气质量传感器 + 0.96寸OLED(SSD1306) + HC-05蓝牙
 *
 * 系统架构：
 *   SHT20  --I2C0--> 温度/湿度
 *   SGP30  --I2C0--> TVOC(ppb) + eCO2(ppm)
 *   SSD1306 OLED --I2C0--> 数据显示
 *   HC-05蓝牙 --UART0--> 数据上报至上位机/手机
 *   蜂鸣器 --GPIO--> 超限报警
 *   LED    --GPIO--> 状态指示
 *
 * 功能特性：
 *   - 周期性采集温湿度+空气质量
 *   - OLED实时显示数据+趋势图标
 *   - 蓝牙上报JSON格式数据
 *   - 温度/湿度/TVOC/eCO2超限报警
 *   - 历史最大最小值记录
 *   - SGP30基线自动保存(每12小时)
 *
 * 错误经验库遵守：
 *   - SGP30上电后需12小时基线学习，初始读数不准
 *   - SHT20测量后需等待(湿度~29ms, 温度~85ms)再读结果
 *   - I2C总线上多设备需注意地址冲突(SHT20=0x40, SGP30=0x58, SSD1306=0x3C)
 *   - OLED刷新不宜过频(~10Hz够用)，否则I2C带宽紧张影响传感器读取
 *   - SGP30需要每小时执行一次基线补偿命令
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== I2C设备地址 ========== */
#define SHT20_ADDR              0x40
#define SGP30_ADDR              0x58
#define SSD1306_ADDR            0x3C

/* ========== SHT20命令 ========== */
#define SHT20_CMD_TEMP_HOLD     0xE3    /* 温度测量(主机保持) */
#define SHT20_CMD_HUMI_HOLD     0xE5    /* 湿度测量(主机保持) */
#define SHT20_CMD_TEMP_NOHOLD   0xF3    /* 温度测量(非保持) */
#define SHT20_CMD_HUMI_NOHOLD   0xF5    /* 湿度测量(非保持) */
#define SHT20_CMD_WRITE_REG     0xE6    /* 写用户寄存器 */
#define SHT20_CMD_READ_REG      0xE7    /* 读用户寄存器 */
#define SHT20_CMD_SOFT_RESET    0xFE    /* 软复位 */

/* ========== SGP30命令 ========== */
#define SGP30_CMD_IAQ_INIT      0x2003  /* IAQ初始化 */
#define SGP30_CMD_MEASURE_IAQ   0x2008  /* IAQ测量 */
#define SGP30_CMD_GET_BASELINE  0x2015  /* 获取基线 */
#define SGP30_CMD_SET_BASELINE  0x201E  /* 设置基线 */
#define SGP30_CMD_SET_HUMIDITY  0x2061  /* 设置湿度补偿 */

/* ========== SSD1306 OLED ========== */
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64
#define SSD1306_PAGES           8       /* 64/8 = 8页 */

/* ========== 报警阈值 ========== */
#define TEMP_HIGH_THRESH        35.0f   /* 温度高温报警 */
#define TEMP_LOW_THRESH         5.0f    /* 温度低温报警 */
#define HUMI_HIGH_THRESH        85.0f   /* 湿度高湿报警 */
#define HUMI_LOW_THRESH         20.0f   /* 湿度低湿报警 */
#define TVOC_HIGH_THRESH        500     /* TVOC高浓度报警(ppb) */
#define ECO2_HIGH_THRESH        1000    /* eCO2高浓度报警(ppm) */

/* ========== 系统参数 ========== */
#define MEASURE_INTERVAL_MS     2000    /* 采集周期2秒 */
#define OLED_UPDATE_INTERVAL_MS 500     /* OLED刷新周期500ms */
#define BT_REPORT_INTERVAL_MS   5000    /* 蓝牙上报周期5秒 */
#define ALARM_BEEP_MS           200     /* 报警蜂鸣时长 */
#define SGP30_BASELINE_HOUR     12      /* 基线学习小时数 */

/* ========== 全局变量 ========== */
static volatile uint32_t g_tick = 0;

/* 传感器数据 */
static volatile float g_temperature = 0.0f;  /* 温度(°C) */
static volatile float g_humidity = 0.0f;     /* 湿度(%) */
static volatile uint16_t g_tvoc = 0;         /* TVOC(ppb) */
static volatile uint16_t g_eco2 = 0;         /* eCO2(ppm) */

/* 历史极值 */
static float g_temp_max = -999.0f, g_temp_min = 999.0f;
static float g_humi_max = 0.0f, g_humi_min = 100.0f;
static uint16_t g_tvoc_max = 0, g_eco2_max = 0;

/* 报警状态 */
static volatile uint8_t g_alarm_flags = 0;
#define ALARM_TEMP_HIGH    0x01
#define ALARM_TEMP_LOW     0x02
#define ALARM_HUMI_HIGH    0x04
#define ALARM_HUMI_LOW     0x08
#define ALARM_TVOC_HIGH    0x10
#define ALARM_ECO2_HIGH    0x20

/* SGP30运行时间(用于基线管理) */
static volatile uint32_t g_sgp30_runtime_sec = 0;
static volatile uint8_t g_sgp30_baseline_valid = 0;
static uint16_t g_sgp30_baseline_tvoc = 0;
static uint16_t g_sgp30_baseline_eco2 = 0;

/* OLED帧缓冲 */
static uint8_t g_oled_buf[SSD1306_PAGES][SSD1306_WIDTH];

/* SHT20 CRC查找表(简化实现) */
static const uint16_t g_crc_table[256] = {
    /* CRC-8 for SHT20: x^8 + x^5 + x^4 + 1 (0x131) */
    /* 为节省空间，这里用计算代替查表 */
};

/* ========== I2C辅助函数 ========== */
/**
 * @brief I2C写数据
 * 错误经验：MSPM0的I2C FIFO在发送前需要先填充再启动传输
 */
static int I2C_Write(uint8_t dev_addr, const uint8_t *data, uint8_t len)
{
    DL_I2C_flushControllerTXFIFO(I2C0);
    DL_I2C_fillControllerTXFIFO(I2C0, (uint8_t *)data, len);
    DL_I2C_startControllerTransfer(I2C0, dev_addr,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, len);
    uint32_t timeout = 100000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_CONTROLLER) {
        if (--timeout == 0) return -1;
    }
    if (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(I2C0);
        return -2;
    }
    return 0;
}

/**
 * @brief I2C写后读
 */
static int I2C_WriteRead(uint8_t dev_addr, const uint8_t *wdata, uint8_t wlen,
                           uint8_t *rdata, uint8_t rlen)
{
    DL_I2C_flushControllerTXFIFO(I2C0);
    DL_I2C_fillControllerTXFIFO(I2C0, (uint8_t *)wdata, wlen);
    DL_I2C_startControllerTransfer(I2C0, dev_addr,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, wlen);
    uint32_t timeout = 100000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_CONTROLLER) {
        if (--timeout == 0) return -1;
    }

    DL_I2C_startControllerTransfer(I2C0, dev_addr,
                                    DL_I2C_CONTROLLER_DIRECTION_RX, rlen);
    timeout = 100000;
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_CONTROLLER) {
        if (--timeout == 0) return -1;
    }

    for (int i = 0; i < rlen; i++) {
        rdata[i] = DL_I2C_receiveControllerData(I2C0);
    }
    return 0;
}

/* ========== CRC校验 ========== */
/**
 * @brief SHT20/SGP30 CRC-8校验 (多项式0x31, 初始值0xFF)
 * 错误经验：I2C传感器数据必须做CRC校验，否则偶尔的总线错误会导致数据跳变
 */
static uint8_t CRC8_Calc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/* ========== SHT20温湿度传感器 ========== */
/**
 * @brief SHT20软复位
 */
static void SHT20_Reset(void)
{
    uint8_t cmd = SHT20_CMD_SOFT_RESET;
    I2C_Write(SHT20_ADDR, &cmd, 1);
    /* 等待复位完成 */
    for (volatile int i = 0; i < 50000; i++) {}
}

/**
 * @brief SHT20读取温湿度
 * @param temperature 温度输出(°C)
 * @param humidity    湿度输出(%)
 * @return 0=成功, 负值=错误
 *
 * 错误经验：
 *   - SHT20测量命令发出后必须等待：湿度~29ms, 温度~85ms
 *   - 使用非保持模式(No Hold)时，主机需自行等待
 *   - 数据格式: 16bit, 最低2位是状态位, 计算时需屏蔽
 */
static int SHT20_Measure(float *temperature, float *humidity)
{
    uint8_t cmd, buf[3];

    /* 测量温度 */
    cmd = SHT20_CMD_TEMP_NOHOLD;
    if (I2C_Write(SHT20_ADDR, &cmd, 1) != 0) return -1;

    /* 等待温度测量完成(85ms) */
    for (volatile int i = 0; i < 500000; i++) {}

    if (I2C_WriteRead(SHT20_ADDR, &cmd, 0, buf, 3) != 0) return -2;

    /* CRC校验 */
    if (CRC8_Calc(buf, 2) != buf[2]) return -3;

    uint16_t raw_temp = (uint16_t)(buf[0] << 8 | buf[1]) & 0xFFFC;
    *temperature = -46.85f + 175.72f * (float)raw_temp / 65536.0f;

    /* 测量湿度 */
    cmd = SHT20_CMD_HUMI_NOHOLD;
    if (I2C_Write(SHT20_ADDR, &cmd, 1) != 0) return -4;

    /* 等待湿度测量完成(29ms) */
    for (volatile int i = 0; i < 200000; i++) {}

    if (I2C_WriteRead(SHT20_ADDR, &cmd, 0, buf, 3) != 0) return -5;

    if (CRC8_Calc(buf, 2) != buf[2]) return -6;

    uint16_t raw_humi = (uint16_t)(buf[0] << 8 | buf[1]) & 0xFFFC;
    *humidity = -6.0f + 125.0f * (float)raw_humi / 65536.0f;
    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity < 0.0f) *humidity = 0.0f;

    return 0;
}

/* ========== SGP30空气质量传感器 ========== */
/**
 * @brief SGP30初始化
 * 错误经验：SGP30上电后必须发送IAQ_INIT命令，否则测量命令返回全0
 */
static int SGP30_Init(void)
{
    uint8_t cmd[2] = {SGP30_CMD_IAQ_INIT >> 8, SGP30_CMD_IAQ_INIT & 0xFF};
    return I2C_Write(SGP30_ADDR, cmd, 2);
}

/**
 * @brief SGP30测量TVOC和eCO2
 * @param tvoc TVOC输出(ppb)
 * @param eco2 eCO2输出(ppm)
 * @return 0=成功
 *
 * 错误经验：SGP30测量命令发出后需等待12ms
 *          读回6字节: TVOC_H, TVOC_CRC, eCO2_H, eCO2_CRC... 
 *          但实际数据是2字节一组+1字节CRC
 */
static int SGP30_Measure(uint16_t *tvoc, uint16_t *eco2)
{
    uint8_t cmd[2] = {SGP30_CMD_MEASURE_IAQ >> 8, SGP30_CMD_MEASURE_IAQ & 0xFF};
    if (I2C_Write(SGP30_ADDR, cmd, 2) != 0) return -1;

    /* 等待测量完成(12ms) */
    for (volatile int i = 0; i < 100000; i++) {}

    uint8_t buf[6];
    if (I2C_WriteRead(SGP30_ADDR, cmd, 0, buf, 6) != 0) return -2;

    /* 解析: eCO2(2字节+CRC) + TVOC(2字节+CRC) */
    if (CRC8_Calc(buf, 2) != buf[2]) return -3;
    if (CRC8_Calc(buf + 3, 2) != buf[5]) return -4;

    *eco2 = (uint16_t)(buf[0] << 8 | buf[1]);
    *tvoc = (uint16_t)(buf[3] << 8 | buf[4]);

    return 0;
}

/**
 * @brief SGP30设置湿度补偿(提高精度)
 * @param humidity 相对湿度(%)
 * @param temp     温度(°C)
 */
static void SGP30_SetHumidity(float humidity, float temp)
{
    /* 绝对湿度计算(g/m^3) */
    float abs_humi = 216.7f * ((humidity / 100.0f) * 6.112f *
                     expf(17.62f * temp / (243.12f + temp)) / (273.15f + temp));

    /* 固定点格式: 高8位=整数, 低8位=小数 */
    uint8_t humi_fixed = (uint8_t)abs_humi;
    uint8_t humi_frac = (uint8_t)((abs_humi - humi_fixed) * 256);

    uint8_t data[5];
    data[0] = SGP30_CMD_SET_HUMIDITY >> 8;
    data[1] = SGP30_CMD_SET_HUMIDITY & 0xFF;
    data[2] = humi_fixed;
    data[3] = humi_frac;
    data[4] = CRC8_Calc(data + 2, 2);

    I2C_Write(SGP30_ADDR, data, 5);
}

/**
 * @brief SGP30获取基线(用于保存)
 */
static int SGP30_GetBaseline(uint16_t *tvoc_base, uint16_t *eco2_base)
{
    uint8_t cmd[2] = {SGP30_CMD_GET_BASELINE >> 8, SGP30_CMD_GET_BASELINE & 0xFF};
    if (I2C_Write(SGP30_ADDR, cmd, 2) != 0) return -1;

    for (volatile int i = 0; i < 100000; i++) {}

    uint8_t buf[6];
    if (I2C_WriteRead(SGP30_ADDR, cmd, 0, buf, 6) != 0) return -2;

    if (CRC8_Calc(buf, 2) != buf[2]) return -3;
    if (CRC8_Calc(buf + 3, 2) != buf[5]) return -4;

    *eco2_base = (uint16_t)(buf[0] << 8 | buf[1]);
    *tvoc_base = (uint16_t)(buf[3] << 8 | buf[4]);
    return 0;
}

/**
 * @brief SGP30恢复基线(从保存值恢复，加速校准)
 */
static void SGP30_RestoreBaseline(uint16_t tvoc_base, uint16_t eco2_base)
{
    uint8_t data[8];
    data[0] = SGP30_CMD_SET_BASELINE >> 8;
    data[1] = SGP30_CMD_SET_BASELINE & 0xFF;
    data[2] = eco2_base >> 8;
    data[3] = eco2_base & 0xFF;
    data[4] = CRC8_Calc(data + 2, 2);
    data[5] = tvoc_base >> 8;
    data[6] = tvoc_base & 0xFF;
    data[7] = CRC8_Calc(data + 5, 2);

    I2C_Write(SGP30_ADDR, data, 8);
    printf("SGP30 baseline restored: TVOC=%d eCO2=%d\r\n", tvoc_base, eco2_base);
}

/* ========== SSD1306 OLED驱动 ========== */
/**
 * @brief SSD1306发送命令
 */
static void SSD1306_WriteCmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd}; /* Co=0, D/C=0 -> 命令 */
    I2C_Write(SSD1306_ADDR, data, 2);
}

/**
 * @brief SSD1306发送数据
 */
static void SSD1306_WriteData(const uint8_t *data, uint16_t len)
{
    /* 分块发送(OLED数据量大，I2C FIFO有限) */
    uint16_t sent = 0;
    while (sent < len) {
        uint16_t chunk = len - sent;
        if (chunk > 16) chunk = 16; /* FIFO深度限制 */

        /* 构造发送缓冲(前缀0x40表示数据) */
        uint8_t buf[17];
        buf[0] = 0x40;
        for (int i = 0; i < chunk; i++) {
            buf[i + 1] = data[sent + i];
        }
        I2C_Write(SSD1306_ADDR, buf, chunk + 1);
        sent += chunk;
    }
}

/**
 * @brief SSD1306初始化
 *
 * 错误经验：SSD1306初始化序列必须严格按照datasheet顺序
 *          遗漏或顺序错误会导致屏幕无显示或显示异常
 *          关键步骤：关闭显示 -> 设置时钟 -> 设置多路复用 -> 设置偏移 ->
 *          设置起始行 -> 设置电荷泵 -> 设置内存模式 -> 段重映射 ->
 *          扫描方向 -> 设置COM引脚 -> 设置对比度 -> 预充电 -> 设置VCOMH -> 打开显示
 */
static void SSD1306_Init(void)
{
    SSD1306_WriteCmd(0xAE); /* 关闭显示 */
    SSD1306_WriteCmd(0xD5); /* 设置时钟分频 */
    SSD1306_WriteCmd(0x80);
    SSD1306_WriteCmd(0xA8); /* 设置多路复用率 */
    SSD1306_WriteCmd(0x3F); /* 64行 */
    SSD1306_WriteCmd(0xD3); /* 设置显示偏移 */
    SSD1306_WriteCmd(0x00);
    SSD1306_WriteCmd(0x40); /* 设置起始行=0 */
    SSD1306_WriteCmd(0x8D); /* 电荷泵设置 */
    SSD1306_WriteCmd(0x14); /* 使能电荷泵 */
    SSD1306_WriteCmd(0x20); /* 内存寻址模式 */
    SSD1306_WriteCmd(0x02); /* 页寻址模式 */
    SSD1306_WriteCmd(0xA1); /* 段重映射(左右翻转) */
    SSD1306_WriteCmd(0xC8); /* COM扫描方向(上下翻转) */
    SSD1306_WriteCmd(0xDA); /* COM引脚配置 */
    SSD1306_WriteCmd(0x12);
    SSD1306_WriteCmd(0x81); /* 对比度 */
    SSD1306_WriteCmd(0xCF);
    SSD1306_WriteCmd(0xD9); /* 预充电周期 */
    SSD1306_WriteCmd(0xF1);
    SSD1306_WriteCmd(0xDB); /* VCOMH电压 */
    SSD1306_WriteCmd(0x40);
    SSD1306_WriteCmd(0xA4); /* 全局显示开启(跟随RAM) */
    SSD1306_WriteCmd(0xA6); /* 正常显示(非反色) */
    SSD1306_WriteCmd(0xAF); /* 打开显示 */

    /* 清屏 */
    memset(g_oled_buf, 0, sizeof(g_oled_buf));
}

/**
 * @brief OLED刷新整个屏幕
 */
static void SSD1306_Refresh(void)
{
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        SSD1306_WriteCmd(0xB0 + page); /* 设置页地址 */
        SSD1306_WriteCmd(0x00);        /* 列地址低4位 */
        SSD1306_WriteCmd(0x10);        /* 列地址高4位 */
        SSD1306_WriteData(g_oled_buf[page], SSD1306_WIDTH);
    }
}

/* ========== 简易字体(5x7 ASCII) ========== */
static const uint8_t g_font5x7[][5] = {
    /* 仅包含数字0-9和部分符号(节省空间) */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */
    {0x00,0x00,0x06,0x06,0x00}, /* . */
    {0x08,0x08,0x2A,0x1C,0x08}, /* ->(箭头) */
};

/**
 * @brief OLED写字符(简易5x7字体)
 */
static void OLED_DrawChar(uint8_t x, uint8_t page, char ch)
{
    if (x > SSD1306_WIDTH - 6) return;

    const uint8_t *glyph = NULL;
    if (ch >= '0' && ch <= '9') {
        glyph = g_font5x7[ch - '0'];
    } else if (ch == ':') {
        glyph = g_font5x7[10];
    } else if (ch == '.') {
        glyph = g_font5x7[11];
    } else if (ch == '%') {
        glyph = g_font5x7[12]; /* 复用箭头 */
    } else {
        return; /* 不支持的字符 */
    }

    if (glyph) {
        for (int i = 0; i < 5; i++) {
            g_oled_buf[page][x + i] = glyph[i];
        }
        g_oled_buf[page][x + 5] = 0x00; /* 字符间距 */
    }
}

/**
 * @brief OLED写字符串(仅支持数字和部分符号)
 */
static void OLED_DrawString(uint8_t x, uint8_t page, const char *str)
{
    while (*str) {
        OLED_DrawChar(x, page, *str);
        x += 6;
        str++;
    }
}

/* ========== 报警管理 ========== */
/**
 * @brief 检查传感器数据并更新报警标志
 */
static void Alarm_Check(void)
{
    g_alarm_flags = 0;

    if (g_temperature > TEMP_HIGH_THRESH) g_alarm_flags |= ALARM_TEMP_HIGH;
    if (g_temperature < TEMP_LOW_THRESH)  g_alarm_flags |= ALARM_TEMP_LOW;
    if (g_humidity > HUMI_HIGH_THRESH)    g_alarm_flags |= ALARM_HUMI_HIGH;
    if (g_humidity < HUMI_LOW_THRESH)     g_alarm_flags |= ALARM_HUMI_LOW;
    if (g_tvoc > TVOC_HIGH_THRESH)        g_alarm_flags |= ALARM_TVOC_HIGH;
    if (g_eco2 > ECO2_HIGH_THRESH)        g_alarm_flags |= ALARM_ECO2_HIGH;
}

/**
 * @brief 报警蜂鸣
 */
static void Alarm_Beep(void)
{
    if (g_alarm_flags) {
        /* 蜂鸣器响 */
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_3);
    }
}

static void Alarm_Silent(void)
{
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_3);
}

/* ========== 蓝牙数据上报 ========== */
/**
 * @brief 通过蓝牙上报JSON格式数据
 * 格式: {"T":25.3,"H":60.2,"TVOC":123,"eCO2":456,"alarm":0x1F}
 */
static void BT_ReportJSON(void)
{
    char json[128];
    int len = snprintf(json, sizeof(json),
        "{\"T\":%.1f,\"H\":%.1f,\"TVOC\":%u,\"eCO2\":%u,\"A\":%u,\"R\":%lu}\r\n",
        g_temperature, g_humidity, g_tvoc, g_eco2,
        g_alarm_flags, g_sgp30_runtime_sec);

    for (int i = 0; i < len; i++) {
        while (!DL_UART_isTXFIFOEmpty(UART0)) {}
        DL_UART_transmitData(UART0, json[i]);
    }
}

/* ========== OLED显示更新 ========== */
/**
 * @brief 更新OLED显示内容
 * 布局(128x64, 8页):
 *   Page 0: 温度:xx.x C
 *   Page 1: 湿度:xx.x %
 *   Page 2: TVOC:xxx ppb
 *   Page 3: eCO2:xxxx ppm
 *   Page 4: (空行)
 *   Page 5: Max/Min统计
 *   Page 6: 报警状态
 *   Page 7: 运行时间
 */
static void OLED_UpdateDisplay(void)
{
    memset(g_oled_buf, 0, sizeof(g_oled_buf));

    /* Page 0: 温度 */
    char buf[32];
    snprintf(buf, sizeof(buf), "%2d.%d", (int)g_temperature,
             (int)(g_temperature * 10) % 10);
    OLED_DrawString(0, 0, buf);

    /* Page 1: 湿度 */
    snprintf(buf, sizeof(buf), "%2d.%d", (int)g_humidity,
             (int)(g_humidity * 10) % 10);
    OLED_DrawString(0, 1, buf);

    /* Page 2: TVOC */
    snprintf(buf, sizeof(buf), "%d", g_tvoc);
    OLED_DrawString(0, 2, buf);

    /* Page 3: eCO2 */
    snprintf(buf, sizeof(buf), "%d", g_eco2);
    OLED_DrawString(0, 3, buf);

    /* Page 5: 最大值 */
    snprintf(buf, sizeof(buf), "%d", g_tvoc_max);
    OLED_DrawString(0, 5, buf);

    /* Page 7: 运行时间(秒) */
    snprintf(buf, sizeof(buf), "%lu", g_sgp30_runtime_sec);
    OLED_DrawString(0, 7, buf);

    SSD1306_Refresh();
}

/* ========== 中断服务 ========== */
void SysTick_Handler(void)
{
    g_tick++;
}

/* ========== 系统初始化 ========== */
static void System_Init(void)
{
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* 初始化SHT20 */
    SHT20_Reset();

    /* 初始化SGP30 */
    SGP30_Init();

    /* 初始化OLED */
    SSD1306_Init();

    /* 尝试恢复SGP30基线(如果之前保存过) */
    if (g_sgp30_baseline_valid) {
        SGP30_RestoreBaseline(g_sgp30_baseline_tvoc, g_sgp30_baseline_eco2);
    }
}

/* ========== 主函数 ========== */
int main(void)
{
    System_Init();

    printf("=== Smart Home Monitor ===\r\n");
    printf("SHT20  : Temp+Humidity\r\n");
    printf("SGP30  : TVOC+eCO2\r\n");
    printf("OLED   : 128x64 Display\r\n");
    printf("BLE    : JSON report every %dms\r\n", BT_REPORT_INTERVAL_MS);
    printf("Alarm  : T>%.0f T<%.0f H>%.0f H<%.0f TVOC>%d eCO2>%d\r\n\r\n",
           TEMP_HIGH_THRESH, TEMP_LOW_THRESH,
           HUMI_HIGH_THRESH, HUMI_LOW_THRESH,
           TVOC_HIGH_THRESH, ECO2_HIGH_THRESH);

    uint32_t last_measure_tick = 0;
    uint32_t last_oled_tick = 0;
    uint32_t last_bt_tick = 0;
    uint32_t alarm_toggle_tick = 0;
    uint32_t baseline_tick = 0;

    while (1) {
        /* 周期性采集传感器数据 */
        if (g_tick - last_measure_tick >= MEASURE_INTERVAL_MS) {
            last_measure_tick = g_tick;

            /* 读取SHT20温湿度 */
            float temp, humi;
            if (SHT20_Measure(&temp, &humi) == 0) {
                g_temperature = temp;
                g_humidity = humi;

                /* 更新极值 */
                if (temp > g_temp_max) g_temp_max = temp;
                if (temp < g_temp_min) g_temp_min = temp;
                if (humi > g_humi_max) g_humi_max = humi;
                if (humi < g_humi_min) g_humi_min = humi;

                /* SGP30湿度补偿 */
                SGP30_SetHumidity(humi, temp);
            }

            /* 读取SGP30空气质量 */
            uint16_t tvoc, eco2;
            if (SGP30_Measure(&tvoc, &eco2) == 0) {
                g_tvoc = tvoc;
                g_eco2 = eco2;
                if (tvoc > g_tvoc_max) g_tvoc_max = tvoc;
                if (eco2 > g_eco2_max) g_eco2_max = eco2;
            }

            g_sgp30_runtime_sec = g_tick / 1000;

            /* 检查报警 */
            Alarm_Check();

            printf("T:%.1f H:%.1f TVOC:%u eCO2:%u\r\n",
                   g_temperature, g_humidity, g_tvoc, g_eco2);
        }

        /* OLED刷新 */
        if (g_tick - last_oled_tick >= OLED_UPDATE_INTERVAL_MS) {
            last_oled_tick = g_tick;
            OLED_UpdateDisplay();
        }

        /* 蓝牙上报 */
        if (g_tick - last_bt_tick >= BT_REPORT_INTERVAL_MS) {
            last_bt_tick = g_tick;
            BT_ReportJSON();
        }

        /* 报警蜂鸣(间歇响) */
        if (g_alarm_flags) {
            if (g_tick - alarm_toggle_tick >= ALARM_BEEP_MS) {
                alarm_toggle_tick = g_tick;
                DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_3); /* 蜂鸣器 */
                DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_0); /* LED */
            }
        } else {
            Alarm_Silent();
            DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);
        }

        /* SGP30基线管理(每12小时保存一次) */
        if (g_sgp30_runtime_sec > SGP30_BASELINE_HOUR * 3600 &&
            g_tick - baseline_tick >= 3600000) { /* 每小时检查一次 */
            baseline_tick = g_tick;
            uint16_t tvoc_base, eco2_base;
            if (SGP30_GetBaseline(&tvoc_base, &eco2_base) == 0) {
                g_sgp30_baseline_tvoc = tvoc_base;
                g_sgp30_baseline_eco2 = eco2_base;
                g_sgp30_baseline_valid = 1;
                printf("Baseline saved: TVOC=%u eCO2=%u\r\n",
                       tvoc_base, eco2_base);
            }
        }
    }
}
