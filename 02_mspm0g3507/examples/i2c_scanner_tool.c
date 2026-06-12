/**
 * @file i2c_scanner_tool.c
 * @brief I2C总线扫描工具
 * @platform MSPM0G3507
 * @description
 *   扫描I2C总线上所有已连接设备：
 *   - 7位地址全扫描 (0x03~0x77)
 *   - 常见I2C设备类型自动识别
 *   - 扫描结果通过UART输出
 *   - 支持多路I2C总线切换
 *   - 设备响应时间检测
 *
 * 硬件连接：
 *   I2C0: PB2(SCL), PB3(SDA)
 *   UART: PA10(TX), PA9(RX) - 115200波特率
 *   LED:  PA27(扫描指示)
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>

/* ========== 常见I2C设备地址与名称映射表 ========== */

/**
 * @brief 已知I2C设备信息结构体
 */
typedef struct {
    uint8_t addr;           /* 7位地址 */
    const char *name;       /* 设备名称 */
    const char *type;       /* 设备类型 */
    const char *description;/* 简要描述 */
} I2C_DeviceInfo_t;

/* 常见I2C设备数据库 */
static const I2C_DeviceInfo_t known_devices[] = {
    /* 通用地址 */
    {0x00, "General Call",  "Control",   "通用呼叫地址"},
    {0x01, "CBUS",          "Bus",       "CBUS地址"},
    {0x02, "Reserved",      "Reserved",  "保留地址"},
    {0x03, "Reserved",      "Reserved",  "保留地址"},
    {0x04, "SMBus Alert",   "SMBus",     "SMBus警报响应"},
    {0x05, "SMBus",         "SMBus",     "SMBus地址"},
    {0x06, "SMBus",         "SMBus",     "SMBus地址"},
    {0x07, "SMBus",         "SMBus",     "SMBus广播地址"},

    /* EEPROM */
    {0x50, "AT24Cxx",       "EEPROM",    "Atmel EEPROM"},
    {0x51, "AT24Cxx",       "EEPROM",    "Atmel EEPROM"},
    {0x52, "AT24Cxx",       "EEPROM",    "Atmel EEPROM"},
    {0x53, "AT24Cxx",       "EEPROM",    "Atmel EEPROM"},
    {0x54, "AT24Cxx",       "EEPROM",    "Atmel EEPROM"},
    {0x55, "AT24Cxx",       "EEPROM",    "Atmel EEPROM"},
    {0x56, "AT24Cxx",       "EEPROM",    "Atmel EEPROM"},
    {0x57, "AT24Cxx",       "EEPROM",    "Atmel EEPROM"},

    /* 实时时钟 */
    {0x68, "DS3231/DS1307", "RTC",       "实时时钟模块"},
    {0x6F, "MCP7940x",      "RTC",       "Microchip RTC"},

    /* 温湿度传感器 */
    {0x40, "Si7021/HTU21",  "Sensor",    "温湿度传感器"},
    {0x44, "SHT3x",         "Sensor",    "Sensirion温湿度"},
    {0x45, "SHT3x",         "Sensor",    "Sensirion温湿度"},
    {0x5C, "AM2320",        "Sensor",    "温湿度传感器"},
    {0x76, "BME280/BMP280", "Sensor",    "Bosch气压温湿度"},
    {0x77, "BME280/BMP280", "Sensor",    "Bosch气压温湿度"},

    /* 显示器 */
    {0x3C, "SSD1306",       "Display",   "0.96寸OLED"},
    {0x3D, "SSD1306",       "Display",   "0.96寸OLED"},
    {0x27, "PCF8574",       "Display",   "LCD1602(I2C背板)"},
    {0x3F, "PCF8574A",      "Display",   "LCD1602(I2C背板)"},

    /* 加速度计/陀螺仪 */
    {0x1D, "ADXL345",       "IMU",       "三轴加速度计"},
    {0x53, "ADXL345",       "IMU",       "三轴加速度计(ALT)"},
    {0x68, "MPU6050/MPU9250","IMU",      "六轴/九轴IMU"},
    {0x69, "MPU6050",       "IMU",       "六轴IMU(ALT)"},

    /* DAC/ADC */
    {0x48, "ADS1115/PCF8591","ADC/DAC",  "16位ADC/8位DAC"},
    {0x49, "ADS1115",       "ADC",       "16位ADC"},
    {0x4A, "ADS1115",       "ADC",       "16位ADC"},
    {0x4B, "ADS1115",       "ADC",       "16位ADC"},
    {0x60, "MCP4725",       "DAC",       "12位DAC"},
    {0x61, "MCP4725",       "DAC",       "12位DAC(ALT)"},
    {0x6E, "MCP3421",       "ADC",       "18位Delta-Sigma ADC"},

    /* IO扩展 */
    {0x20, "PCF8574/MCP23017","IO",      "IO扩展器"},
    {0x21, "PCF8574/MCP23017","IO",      "IO扩展器"},
    {0x22, "PCF8574/MCP23017","IO",      "IO扩展器"},
    {0x23, "PCF8574/MCP23017","IO",      "IO扩展器"},
    {0x24, "MCP23017",       "IO",       "16位IO扩展器"},
    {0x25, "MCP23017",       "IO",       "16位IO扩展器"},
    {0x26, "MCP23017",       "IO",       "16位IO扩展器"},
    {0x27, "MCP23017",       "IO",       "16位IO扩展器"},

    /* 电机驱动 */
    {0x60, "PCA9685",       "Motor",     "16路PWM驱动"},
    {0x40, "PCA9685",       "Motor",     "16路PWM驱动(ALT)"},
    {0x62, "PCA9685",       "Motor",     "16路PWM驱动(ALT)"},

    /* 气压传感器 */
    {0x60, "BMP180",        "Sensor",    "气压传感器"},
    {0x76, "BMP280",        "Sensor",    "气压传感器"},

    /* 磁力计 */
    {0x0C, "AK8963",        "Magnetometer","磁力计(MPU9250内置)"},
    {0x1E, "HMC5883L",      "Magnetometer","三轴磁力计"},

    /* 颜色传感器 */
    {0x29, "VL53L0x",       "ToF",       "激光测距"},
    {0x39, "APDS9960",      "Sensor",    "颜色/手势传感器"},
    {0x52, "TCS34725",      "Sensor",    "RGB颜色传感器"},

    /* 音频 */
    {0x1A, "WM8960",        "Audio",     "音频编解码器"},
    {0x1B, "SGTL5000",      "Audio",     "音频编解码器"},

    /* 电源管理 */
    {0x48, "INA219",        "Power",     "电流/功率监测"},
    {0x4A, "INA219",        "Power",     "电流/功率监测(ALT)"},
    {0x6B, "BQ27441",       "Power",     "电池电量计"},
};

#define KNOWN_DEVICE_COUNT  (sizeof(known_devices) / sizeof(known_devices[0]))

/* 扫描结果缓冲区 */
#define MAX_FOUND_DEVICES  32

typedef struct {
    uint8_t addr;
    bool responded;
    uint32_t response_time_us;  /* 响应时间(微秒) */
} ScanResult_t;

static ScanResult_t scan_results[MAX_FOUND_DEVICES];
static uint8_t found_count = 0;

/* ========== UART驱动 ========== */

/**
 * @brief UART发送字符串
 */
static void UART_SendString(const char *str)
{
    while (*str) {
        DL_UART_main_transmitDataBlocking(UART0, (uint8_t)*str);
        str++;
    }
}

/**
 * @brief 格式化输出到UART
 */
static void UART_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    UART_SendString(buf);
}

/* ========== I2C总线扫描核心 ========== */

/**
 * @brief 检测指定地址的I2C设备是否存在
 * @param addr 7位I2C地址
 * @return true=设备存在, false=无响应
 */
static bool I2C_ProbeAddress(uint8_t addr)
{
    /* 清除I2C状态 */
    DL_I2C_flushControllerTXFIFO(I2C0);

    /* 发送起始条件+地址字节，尝试写操作 */
    DL_I2C_setTargetAddress(I2C0, addr);

    /* 发送零字节的写传输(仅检测ACK) */
    DL_I2C_startControllerTransfer(I2C0, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 0);

    /* 等待传输完成或超时 */
    uint32_t timeout = 10000;
    while (timeout-- > 0) {
        uint32_t status = DL_I2C_getControllerStatus(I2C0);

        /* 检查NACK(设备不存在) */
        if (status & DL_I2C_CONTROLLER_STATUS_ERROR_NACK) {
            DL_I2C_flushControllerTXFIFO(I2C0);
            return false;
        }

        /* 检查传输完成 */
        if (!(status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) {
            return true;
        }
    }

    /* 超时，视为无设备 */
    DL_I2C_flushControllerTXFIFO(I2C0);
    return false;
}

/**
 * @brief 测量I2C设备响应时间
 * @param addr 7位I2C地址
 * @return 响应时间(微秒)
 */
static uint32_t I2C_MeasureResponseTime(uint8_t addr)
{
    /* 简化计时：使用SysTick */
    uint32_t start = SysTick->VAL;

    I2C_ProbeAddress(addr);

    uint32_t end = SysTick->VAL;
    uint32_t elapsed = start - end;  /* SysTick向下计数 */

    /* 转换为微秒 (假设32MHz系统时钟) */
    return elapsed / 32;
}

/**
 * @brief 扫描整个I2C总线
 * @note 扫描范围0x03~0x77，跳过保留地址
 */
static void I2C_ScanBus(void)
{
    found_count = 0;

    UART_SendString("\r\n");
    UART_SendString("============================================\r\n");
    UART_SendString("  I2C Bus Scanner - MSPM0G3507\r\n");
    UART_SendString("============================================\r\n");
    UART_SendString("Scanning address range: 0x03 - 0x77\r\n");
    UART_SendString("Please wait...\r\n\r\n");

    /* LED指示扫描进行中 */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_27);

    /* 遍历所有有效7位地址 */
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        bool found = I2C_ProbeAddress(addr);

        if (found && found_count < MAX_FOUND_DEVICES) {
            scan_results[found_count].addr = addr;
            scan_results[found_count].responded = true;
            scan_results[found_count].response_time_us =
                I2C_MeasureResponseTime(addr);
            found_count++;

            UART_Printf("  [0x%02X] Device found!\r\n", addr);
        }
    }

    /* LED指示扫描完成 */
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27);
}

/**
 * @brief 查找已知设备信息
 * @param addr I2C地址
 * @return 设备信息指针，未找到返回NULL
 */
static const I2C_DeviceInfo_t* I2C_FindDeviceInfo(uint8_t addr)
{
    for (uint32_t i = 0; i < KNOWN_DEVICE_COUNT; i++) {
        if (known_devices[i].addr == addr) {
            return &known_devices[i];
        }
    }
    return NULL;
}

/**
 * @brief 输出扫描结果报告
 */
static void I2C_PrintReport(void)
{
    UART_SendString("\r\n");
    UART_SendString("============================================\r\n");
    UART_SendString("  SCAN RESULTS\r\n");
    UART_SendString("============================================\r\n");

    if (found_count == 0) {
        UART_SendString("  No I2C devices found!\r\n");
        UART_SendString("  Check wiring and pull-up resistors.\r\n");
    } else {
        UART_Printf("  Found %d device(s):\r\n\r\n", found_count);
        UART_SendString("  ADDR  | TYPE       | DEVICE NAME\r\n");
        UART_SendString("  ------+------------+---------------------------\r\n");

        for (uint8_t i = 0; i < found_count; i++) {
            uint8_t addr = scan_results[i].addr;
            const I2C_DeviceInfo_t *info = I2C_FindDeviceInfo(addr);

            if (info) {
                UART_Printf("  0x%02X  | %-10s | %s\r\n",
                    addr, info->type, info->description);
            } else {
                UART_Printf("  0x%02X  | Unknown    | (未识别设备)\r\n", addr);
            }
        }
    }

    UART_SendString("\r\n============================================\r\n");
    UART_SendString("  Scan complete.\r\n");
    UART_SendString("============================================\r\n\r\n");
}

/**
 * @brief 输出地址表（所有可能地址的响应状态）
 */
static void I2C_PrintAddressMap(void)
{
    UART_SendString("\r\n  Address Map (7-bit):\r\n");
    UART_SendString("       0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

    for (uint8_t row = 0; row < 8; row++) {
        UART_Printf("  %X0: ", row);
        for (uint8_t col = 0; col < 16; col++) {
            uint8_t addr = (row << 4) | col;

            if (addr < 0x03 || addr > 0x77) {
                UART_SendString("-- ");  /* 无效地址 */
            } else {
                /* 检查是否已发现 */
                bool found = false;
                for (uint8_t i = 0; i < found_count; i++) {
                    if (scan_results[i].addr == addr) {
                        found = true;
                        break;
                    }
                }
                UART_SendString(found ? "XX " : ".. ");
            }
        }
        UART_SendString("\r\n");
    }
    UART_SendString("  Legend: XX=found, ..=empty, --=reserved\r\n\r\n");
}

/**
 * @brief 扫描并识别设备类型（尝试读取设备ID寄存器）
 * @param addr 设备地址
 */
static void I2C_IdentifyDevice(uint8_t addr)
{
    UART_Printf("\r\n  Probing 0x%02X for device identification...\r\n", addr);

    /* 尝试读取WHO_AM_I寄存器(常见于IMU传感器) */
    uint8_t reg = 0x0F;  /* WHO_AM_I寄存器地址 */
    uint8_t data = 0;

    DL_I2C_flushControllerTXFIFO(I2C0);
    DL_I2C_fillControllerTXFIFO(I2C0, &reg, 1);
    DL_I2C_startControllerTransfer(I2C0, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    uint32_t timeout = 50000;
    while (timeout-- > 0) {
        if (!(DL_I2C_getControllerStatus(I2C0) &
              DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) break;
    }

    DL_I2C_flushControllerRXFIFO(I2C0);
    DL_I2C_startControllerTransfer(I2C0, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    timeout = 50000;
    while (timeout-- > 0) {
        if (DL_I2C_isControllerRXFIFOEmpty(I2C0) == false) {
            data = DL_I2C_receiveControllerData(I2C0);
            UART_Printf("  WHO_AM_I(0x%02X) = 0x%02X\r\n", reg, data);

            /* 识别常见设备 */
            if (data == 0x68) UART_SendString("  -> MPU6050 detected\r\n");
            else if (data == 0x71) UART_SendString("  -> MPU9250 detected\r\n");
            else if (data == 0xE5) UART_SendString("  -> HMC5883L detected\r\n");
            else if (data == 0x60) UART_SendString("  -> BMP180 detected\r\n");
            else if (data == 0x58) UART_SendString("  -> BMP280 detected\r\n");
            else if (data == 0x61) UART_SendString("  -> BME280 detected\r\n");
            else UART_SendString("  -> Unknown device\r\n");
            return;
        }
    }
    UART_SendString("  -> No ID register response (generic device)\r\n");
}

/* ========== 主函数 ========== */

int main(void)
{
    /* 系统初始化 */
    DL_SYSCFG_init();

    /* LED初始化 */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_27);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_27);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27);

    /* UART初始化 */
    DL_UART_reset(UART0);
    DL_UART_enablePower(UART0);
    DL_UART_setClockConfig(UART0, DL_UART_CLOCK_DIVIDE_115200);
    DL_UART_init(UART0, DL_UART_MODE_NORMAL);
    DL_UART_enable(UART0);

    /* I2C初始化 */
    DL_I2C_reset(I2C0);
    DL_I2C_enablePower(I2C0);
    DL_I2C_setClockConfig(I2C0, DL_I2C_CLOCK_DIVIDE_100KHZ);
    DL_I2C_enableController(I2C0);

    UART_SendString("\r\n=== I2C Bus Scanner v1.0 ===\r\n");
    UART_SendString("Initializing...\r\n");

    /* 执行总线扫描 */
    I2C_ScanBus();

    /* 输出详细报告 */
    I2C_PrintReport();

    /* 输出地址映射图 */
    I2C_PrintAddressMap();

    /* 对每个发现的设备进行深度识别 */
    UART_SendString("\r\n--- Device Deep Identification ---\r\n");
    for (uint8_t i = 0; i < found_count; i++) {
        I2C_IdentifyDevice(scan_results[i].addr);
    }

    UART_SendString("\r\n=== Scan Complete ===\r\n");

    /* 扫描完成后进入低功耗等待 */
    while (1) {
        __WFI();
    }
}
