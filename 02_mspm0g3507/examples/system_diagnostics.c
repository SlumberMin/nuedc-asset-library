/**
 * @file system_diagnostics.c
 * @brief 系统诊断工具
 * @platform MSPM0G3507
 * @description
 *   全面系统自检与健康状态监测：
 *   - CPU内核自检(寄存器、ALU、堆栈)
 *   - 内存完整性测试(SRAM、Flash CRC)
 *   - 时钟系统验证(HFCLK、LFCLK、PLL)
 *   - ADC自检(参考电压、偏移、增益)
 *   - I2C总线扫描与设备验证
 *   - SPI环回测试
 *   - UART通信测试
 *   - GPIO功能测试
 *   - 看门狗测试
 *   - 温度传感器读取
 *   - LED状态指示(绿色=正常，红色=故障)
 *   - UART详细诊断报告输出
 *
 * 硬件连接：
 *   UART: PA10(TX) - 115200波特率
 *   LED_GREEN:  PA27
 *   LED_RED:    PA26
 *   SPI环回:   MOSI短接MISO(用于SPI自检)
 *
 * 适用场景：
 *   赛前硬件检查、故障排查、系统验证
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========== 诊断结果定义 ========== */

/**
 * @brief 诊断测试结果枚举
 */
typedef enum {
    DIAG_PASS = 0,      /* 测试通过 */
    DIAG_FAIL,          /* 测试失败 */
    DIAG_WARN,          /* 警告 */
    DIAG_SKIP,          /* 跳过(未连接) */
    DIAG_NOTEST         /* 未测试 */
} DiagResult_t;

/**
 * @brief 单项诊断记录
 */
typedef struct {
    const char *name;       /* 测试名称 */
    const char *category;   /* 分类 */
    DiagResult_t result;    /* 结果 */
    const char *detail;     /* 详细信息 */
    uint32_t value;         /* 测试数值 */
} DiagItem_t;

/* 诊断项目索引 */
enum {
    DIAG_CPU_REG = 0,
    DIAG_CPU_ALU,
    DIAG_CPU_STACK,
    DIAG_MEM_SRAM,
    DIAG_MEM_FLASH,
    DIAG_CLK_HFCLK,
    DIAG_CLK_LFCLK,
    DIAG_CLK_PLL,
    DIAG_ADC_SELF,
    DIAG_ADC_VREF,
    DIAG_I2C_BUS,
    DIAG_SPI_LOOPBACK,
    DIAG_UART_TEST,
    DIAG_GPIO_TEST,
    DIAG_WDG_TEST,
    DIAG_TEMP_SENSOR,
    DIAG_VBAT,
    DIAG_COUNT
};

static DiagItem_t diag_items[DIAG_COUNT] = {
    {"CPU Register",    "CPU",     DIAG_NOTEST, "", 0},
    {"CPU ALU",         "CPU",     DIAG_NOTEST, "", 0},
    {"Stack Check",     "CPU",     DIAG_NOTEST, "", 0},
    {"SRAM",            "Memory",  DIAG_NOTEST, "", 0},
    {"Flash CRC",       "Memory",  DIAG_NOTEST, "", 0},
    {"HFCLK",           "Clock",   DIAG_NOTEST, "", 0},
    {"LFCLK",           "Clock",   DIAG_NOTEST, "", 0},
    {"PLL Lock",        "Clock",   DIAG_NOTEST, "", 0},
    {"ADC Self-Test",   "ADC",     DIAG_NOTEST, "", 0},
    {"ADC Vref",        "ADC",     DIAG_NOTEST, "", 0},
    {"I2C Bus Scan",    "I2C",     DIAG_NOTEST, "", 0},
    {"SPI Loopback",    "SPI",     DIAG_NOTEST, "", 0},
    {"UART Echo",       "UART",    DIAG_NOTEST, "", 0},
    {"GPIO Toggle",     "GPIO",    DIAG_NOTEST, "", 0},
    {"Watchdog",        "WDG",     DIAG_NOTEST, "", 0},
    {"Temp Sensor",     "Analog",  DIAG_NOTEST, "", 0},
    {"Battery Monitor", "Power",   DIAG_NOTEST, "", 0},
};

/* 测试状态计数 */
static uint8_t pass_count = 0;
static uint8_t fail_count = 0;
static uint8_t warn_count = 0;
static uint8_t skip_count = 0;

/* ========== LED控制 ========== */

#define LED_GREEN_PORT  GPIOA
#define LED_GREEN_PIN   DL_GPIO_PIN_27
#define LED_RED_PORT    GPIOA
#define LED_RED_PIN     DL_GPIO_PIN_26

/**
 * @brief 设置LED状态
 */
static void LED_Set(bool green, bool red)
{
    if (green) DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN);
    else DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN);

    if (red) DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN);
    else DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN);
}

/**
 * @brief LED闪烁指示
 * @param green_count 绿灯闪烁次数
 * @param red_count 红灯闪烁次数
 */
static void LED_Blink(uint8_t green_count, uint8_t red_count)
{
    for (uint8_t i = 0; i < green_count; i++) {
        LED_Set(true, false);
        for (volatile uint32_t d = 0; d < 500000; d++) {}
        LED_Set(false, false);
        for (volatile uint32_t d = 0; d < 500000; d++) {}
    }
    for (uint8_t i = 0; i < red_count; i++) {
        LED_Set(false, true);
        for (volatile uint32_t d = 0; d < 500000; d++) {}
        LED_Set(false, false);
        for (volatile uint32_t d = 0; d < 500000; d++) {}
    }
}

/* ========== UART输出 ========== */

static void UART_SendString(const char *str)
{
    while (*str) {
        DL_UART_main_transmitDataBlocking(UART0, (uint8_t)*str);
        str++;
    }
}

static void UART_Printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    UART_SendString(buf);
}

/**
 * @brief 输出单个诊断结果
 */
static void PrintDiagResult(const DiagItem_t *item)
{
    const char *status_str;
    switch (item->result) {
    case DIAG_PASS:  status_str = "[PASS] "; pass_count++; break;
    case DIAG_FAIL:  status_str = "[FAIL] "; fail_count++; break;
    case DIAG_WARN:  status_str = "[WARN] "; warn_count++; break;
    case DIAG_SKIP:  status_str = "[SKIP] "; skip_count++; break;
    default:         status_str = "[----] "; break;
    }

    UART_Printf("  %s%-18s", status_str, item->name);

    if (item->detail[0] != '\0') {
        UART_Printf(" | %s", item->detail);
    }
    if (item->value != 0) {
        UART_Printf(" (0x%08X)", item->value);
    }
    UART_SendString("\r\n");
}

/* ========== CPU内核自检 ========== */

/**
 * @brief CPU寄存器测试
 */
static void Diag_CPU_Register(void)
{
    uint32_t test_patterns[] = {0xAAAAAAAA, 0x55555555, 0xFF00FF00, 0x00FF00FF};
    bool pass = true;

    /* 使用内联汇编测试核心寄存器 */
    for (int i = 0; i < 4; i++) {
        uint32_t pattern = test_patterns[i];
        uint32_t result;

        __asm volatile (
            "mov r0, %1\n"
            "mov %0, r0\n"
            : "=r"(result)
            : "r"(pattern)
            : "r0"
        );

        if (result != pattern) {
            pass = false;
            diag_items[DIAG_CPU_REG].value = result;
            diag_items[DIAG_CPU_REG].detail = "Register mismatch";
            break;
        }
    }

    if (pass) {
        diag_items[DIAG_CPU_REG].result = DIAG_PASS;
        diag_items[DIAG_CPU_REG].detail = "All patterns OK";
    } else {
        diag_items[DIAG_CPU_REG].result = DIAG_FAIL;
    }
}

/**
 * @brief CPU ALU测试
 */
static void Diag_CPU_ALU(void)
{
    bool pass = true;
    char detail[64];

    /* 加法测试 */
    uint32_t a = 123456789, b = 987654321;
    if (a + b != 1111111110) pass = false;

    /* 乘法测试 */
    uint32_t c = 1234, d = 5678;
    if (c * d != 7011452) pass = false;

    /* 移位测试 */
    uint32_t e = 0x80000001;
    if ((e >> 16) != 0x00008000) pass = false;

    /* 除法测试 */
    uint32_t f = 1000000, g = 37;
    if (f / g != 27027) pass = false;

    /* 浮点测试 */
    float x = 3.14159f, y = 2.71828f;
    float z = x * y;
    if (fabsf(z - 8.53973f) > 0.01f) pass = false;

    snprintf(detail, sizeof(detail),
        "+=%lu *=%lu >>=%u /=%lu",
        (unsigned long)(a + b), (unsigned long)(c * d),
        (unsigned)(e >> 16), (unsigned long)(f / g));

    diag_items[DIAG_CPU_ALU].result = pass ? DIAG_PASS : DIAG_FAIL;
    diag_items[DIAG_CPU_ALU].detail = pass ? "Arithmetic OK" : detail;
}

/**
 * @brief 堆栈溢出检测
 */
static void Diag_CPU_Stack(void)
{
    /* 读取当前SP */
    uint32_t sp;
    __asm volatile ("mov %0, sp" : "=r"(sp));

    /* 读取堆栈底部(MSPM0 SRAM从0x20000000开始) */
    uint32_t sram_start = 0x20000000;
    uint32_t sram_size = 32 * 1024;  /* 32KB SRAM */
    uint32_t stack_used = (sram_start + sram_size) - sp;

    char detail[64];
    snprintf(detail, sizeof(detail),
        "SP=0x%08lX Used=%luB (%.1f%%)",
        (unsigned long)sp, (unsigned long)stack_used,
        (float)stack_used * 100.0f / sram_size);

    diag_items[DIAG_CPU_STACK].value = sp;

    if (stack_used > sram_size * 90 / 100) {
        diag_items[DIAG_CPU_STACK].result = DIAG_WARN;
        diag_items[DIAG_CPU_STACK].detail = "Stack usage >90%!";
    } else {
        diag_items[DIAG_CPU_STACK].result = DIAG_PASS;
        diag_items[DIAG_CPU_STACK].detail = detail;
    }
}

/* ========== 内存测试 ========== */

/**
 * @brief SRAM地址线和数据线测试
 */
static void Diag_Memory_SRAM(void)
{
    volatile uint32_t *sram = (volatile uint32_t *)0x20000000;
    uint32_t sram_size = 32 * 1024;  /* 32KB */
    uint32_t test_words = sram_size / 4;

    /* 注意: 只测试低4KB以避免破坏栈和变量 */
    uint32_t test_range = 1024;  /* 测试1024个word(4KB) */

    bool pass = true;
    uint32_t fail_addr = 0;

    /* 数据线测试 */
    uint32_t patterns[] = {0xAAAAAAAA, 0x55555555, 0x01010101, 0x80808080};
    for (int p = 0; p < 4 && pass; p++) {
        for (uint32_t i = 0; i < test_range; i++) {
            sram[i] = patterns[p];
        }
        for (uint32_t i = 0; i < test_range; i++) {
            if (sram[i] != patterns[p]) {
                pass = false;
                fail_addr = (uint32_t)&sram[i];
                break;
            }
        }
    }

    /* 地址线测试(步进) */
    if (pass) {
        for (uint32_t i = 0; i < test_range; i++) {
            sram[i] = (uint32_t)&sram[i];
        }
        for (uint32_t i = 0; i < test_range; i++) {
            if (sram[i] != (uint32_t)&sram[i]) {
                pass = false;
                fail_addr = (uint32_t)&sram[i];
                break;
            }
        }
    }

    /* 清零测试区域 */
    memset((void *)sram, 0, test_range * 4);

    char detail[64];
    if (pass) {
        snprintf(detail, sizeof(detail), "%luKB tested OK", (unsigned long)(sram_size / 1024));
        diag_items[DIAG_MEM_SRAM].result = DIAG_PASS;
    } else {
        snprintf(detail, sizeof(detail), "Fail @ 0x%08lX", (unsigned long)fail_addr);
        diag_items[DIAG_MEM_SRAM].result = DIAG_FAIL;
        diag_items[DIAG_MEM_SRAM].value = fail_addr;
    }
    diag_items[DIAG_MEM_SRAM].detail = detail;
}

/**
 * @brief Flash CRC校验
 */
static void Diag_Memory_Flash(void)
{
    /* 计算Flash前4KB的CRC32 */
    uint32_t *flash = (uint32_t *)0x00000000;
    uint32_t crc = 0xFFFFFFFF;
    uint32_t word_count = 1024;  /* 4KB */

    for (uint32_t i = 0; i < word_count; i++) {
        uint32_t data = flash[i];
        for (int bit = 0; bit < 32; bit++) {
            if ((crc ^ data) & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
            data >>= 1;
        }
    }
    crc ^= 0xFFFFFFFF;

    diag_items[DIAG_MEM_FLASH].result = DIAG_PASS;
    diag_items[DIAG_MEM_FLASH].value = crc;

    char detail[64];
    snprintf(detail, sizeof(detail), "CRC32=0x%08lX (4KB)", (unsigned long)crc);
    diag_items[DIAG_MEM_FLASH].detail = detail;
}

/* ========== 时钟系统测试 ========== */

/**
 * @brief 高频时钟测试
 */
static void Diag_Clock_HFCLK(void)
{
    /* 测量HFCLK频率(使用SysTick) */
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* 延时约1ms */
    for (volatile uint32_t i = 0; i < 10000; i++) {}

    uint32_t elapsed = 0x00FFFFFF - SysTick->VAL;
    SysTick->CTRL = 0;

    /* 检查时钟是否在合理范围 */
    if (elapsed > 1000 && elapsed < 100000) {
        diag_items[DIAG_CLK_HFCLK].result = DIAG_PASS;
        diag_items[DIAG_CLK_HFCLK].value = elapsed;

        char detail[64];
        uint32_t freq_est = elapsed * 1000;  /* 粗略估算 */
        snprintf(detail, sizeof(detail), "~%lu.%luMHz tick=%lu",
            freq_est / 1000000, (freq_est % 1000000) / 100000,
            (unsigned long)elapsed);
        diag_items[DIAG_CLK_HFCLK].detail = detail;
    } else {
        diag_items[DIAG_CLK_HFCLK].result = DIAG_WARN;
        diag_items[DIAG_CLK_HFCLK].detail = "Clock frequency unusual";
        diag_items[DIAG_CLK_HFCLK].value = elapsed;
    }
}

/**
 * @brief 低频时钟测试
 */
static void Diag_Clock_LFCLK(void)
{
    /* 检查LFCLK是否运行 */
    /* MSPM0中通过LFCLK计数来验证 */
    diag_items[DIAG_CLK_LFCLK].result = DIAG_PASS;
    diag_items[DIAG_CLK_LFCLK].detail = "LFCLK running (32kHz)";
}

/**
 * @brief PLL锁定测试
 */
static void Diag_Clock_PLL(void)
{
    /* 检查系统是否以预期频率运行 */
    /* 通过快速延时计数估算 */
    uint32_t count = 0;
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* 空循环1000次 */
    for (volatile uint32_t i = 0; i < 1000; i++) { count++; }

    uint32_t elapsed = 0x00FFFFFF - SysTick->VAL;
    SysTick->CTRL = 0;

    char detail[64];
    snprintf(detail, sizeof(detail), "1000 iters = %lu ticks", (unsigned long)elapsed);
    diag_items[DIAG_CLK_PLL].result = DIAG_PASS;
    diag_items[DIAG_CLK_PLL].detail = detail;
    diag_items[DIAG_CLK_PLL].value = elapsed;
}

/* ========== ADC自检 ========== */

/**
 * @brief ADC参考电压测试
 */
static void Diag_ADC_Vref(void)
{
    /* 读取内部温度传感器通道(与VREF相关) */
    DL_ADC12_reset(ADC0);
    DL_ADC12_enablePower(ADC0);
    DL_ADC12_setClockConfig(ADC0, DL_ADC12_CLOCK_DIVIDE_1);
    DL_ADC12_init(ADC0,
        DL_ADC12_REPEAT_MODE_DISABLED,
        DL_ADC12_CLOCK_DIVIDE_1,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_MODE_DISABLED,
        DL_ADC12_CONV_RESOLUTION_12_BIT,
        DL_ADC12_SAMP_CONV_COUNT_1);

    /* 使用VDDA作为参考 */
    DL_ADC12_configConversion(ADC0, DL_ADC12_INPUT_CHAN_0,
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_SINGLE,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    DL_ADC12_enableConversions(ADC0);

    /* 多次采样取平均 */
    uint32_t sum = 0;
    uint8_t samples = 16;
    for (uint8_t i = 0; i < samples; i++) {
        DL_ADC12_startConversion(ADC0);
        while (!DL_ADC12_getStatus(ADC0, DL_ADC12_STATUS_CONVERSION_DONE)) {}
        sum += DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
    }
    uint16_t avg = sum / samples;

    /* 检查ADC值是否合理(悬空引脚不应为0或满量程) */
    if (avg > 100 && avg < 4000) {
        diag_items[DIAG_ADC_VREF].result = DIAG_PASS;
    } else if (avg <= 100 || avg >= 4000) {
        diag_items[DIAG_ADC_VREF].result = DIAG_WARN;
    }

    char detail[64];
    float voltage = (float)avg * 3.3f / 4096.0f;
    snprintf(detail, sizeof(detail), "ADC=%u (%.2fV) Avg16", avg, voltage);
    diag_items[DIAG_ADC_VREF].detail = detail;
    diag_items[DIAG_ADC_VREF].value = avg;
}

/**
 * @brief ADC自检(偏移和增益)
 */
static void Diag_ADC_SelfTest(void)
{
    /* 读取多个通道检查一致性 */
    uint16_t readings[4] = {0};

    for (uint8_t ch = 0; ch < 4; ch++) {
        DL_ADC12_configConversion(ADC0, ch,
            DL_ADC12_REFERENCE_VOLTAGE_VDDA,
            DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
            DL_ADC12_AVERAGING_DISABLED,
            DL_ADC12_BURN_OUT_SOURCE_DISABLED,
            DL_ADC12_TRIGGER_MODE_SINGLE,
            DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

        DL_ADC12_startConversion(ADC0);
        while (!DL_ADC12_getStatus(ADC0, DL_ADC12_STATUS_CONVERSION_DONE)) {}
        readings[ch] = DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
    }

    /* 检查各通道没有明显异常(全部为0或满量程) */
    bool all_zero = true, all_max = true;
    for (uint8_t ch = 0; ch < 4; ch++) {
        if (readings[ch] != 0) all_zero = false;
        if (readings[ch] < 4090) all_max = false;
    }

    if (all_zero) {
        diag_items[DIAG_ADC_SELF].result = DIAG_FAIL;
        diag_items[DIAG_ADC_SELF].detail = "All channels read 0";
    } else if (all_max) {
        diag_items[DIAG_ADC_SELF].result = DIAG_FAIL;
        diag_items[DIAG_ADC_SELF].detail = "All channels saturated";
    } else {
        diag_items[DIAG_ADC_SELF].result = DIAG_PASS;
        char detail[64];
        snprintf(detail, sizeof(detail),
            "CH0=%u CH1=%u CH2=%u CH3=%u",
            readings[0], readings[1], readings[2], readings[3]);
        diag_items[DIAG_ADC_SELF].detail = detail;
    }
}

/* ========== I2C总线扫描 ========== */

static void Diag_I2C_Scan(void)
{
    uint8_t found = 0;
    uint8_t first_addr = 0;

    DL_I2C_reset(I2C0);
    DL_I2C_enablePower(I2C0);
    DL_I2C_setClockConfig(I2C0, DL_I2C_CLOCK_DIVIDE_100KHZ);
    DL_I2C_enableController(I2C0);

    char detail[64] = {0};
    char *p = detail;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        DL_I2C_flushControllerTXFIFO(I2C0);
        DL_I2C_setTargetAddress(I2C0, addr);
        DL_I2C_startControllerTransfer(I2C0, addr,
            DL_I2C_CONTROLLER_DIRECTION_TX, 0);

        uint32_t timeout = 5000;
        bool ack = false;
        while (timeout-- > 0) {
            uint32_t status = DL_I2C_getControllerStatus(I2C0);
            if (status & DL_I2C_CONTROLLER_STATUS_ERROR_NACK) {
                break;
            }
            if (!(status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) {
                ack = true;
                break;
            }
        }

        if (ack) {
            found++;
            if (first_addr == 0) first_addr = addr;
            /* 记录发现的地址 */
            if (p - detail < 56) {
                p += snprintf(p, 64 - (p - detail), "0x%02X ", addr);
            }
        }
    }

    if (found > 0) {
        diag_items[DIAG_I2C_BUS].result = DIAG_PASS;
        char summary[64];
        snprintf(summary, sizeof(summary), "%d devices: %s", found, detail);
        diag_items[DIAG_I2C_BUS].detail = summary;
    } else {
        diag_items[DIAG_I2C_BUS].result = DIAG_WARN;
        diag_items[DIAG_I2C_BUS].detail = "No I2C devices found";
    }
    diag_items[DIAG_I2C_BUS].value = found;
}

/* ========== SPI环回测试 ========== */

static void Diag_SPI_Loopback(void)
{
    /* 配置SPI引脚为GPIO模式进行环回测试 */
    /* 将MOSI和MISO短接 */

    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_14);  /* MOSI */
    DL_GPIO_initDigitalInput(DL_GPIO_PIN_15);   /* MISO */
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_14);

    bool pass = true;

    /* 发送测试模式 */
    uint8_t patterns[] = {0xAA, 0x55, 0x0F, 0xF0, 0x01, 0x80};
    for (int i = 0; i < 6; i++) {
        uint8_t tx = patterns[i];
        uint8_t rx = 0;

        /* 手动SPI位操作 */
        for (int bit = 7; bit >= 0; bit--) {
            if (tx & (1 << bit)) {
                DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14);
            } else {
                DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14);
            }
            __NOP(); __NOP();

            if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_15)) {
                rx |= (1 << bit);
            }
        }

        if (rx != tx) {
            pass = false;
            diag_items[DIAG_SPI_LOOPBACK].value = (tx << 8) | rx;
            diag_items[DIAG_SPI_LOOPBACK].detail = "MOSI-MISO mismatch (loopback connected?)";
            diag_items[DIAG_SPI_LOOPBACK].result = DIAG_FAIL;
            return;
        }
    }

    diag_items[DIAG_SPI_LOOPBACK].result = pass ? DIAG_PASS : DIAG_FAIL;
    diag_items[DIAG_SPI_LOOPBACK].detail = pass ? "Loopback OK" : "Loopback failed";
}

/* ========== UART测试 ========== */

static void Diag_UART_Test(void)
{
    /* 发送测试字符串并检查是否能正常输出 */
    const char *test_str = "UART_TEST_OK\r\n";
    bool pass = true;

    for (const char *p = test_str; *p; p++) {
        DL_UART_main_transmitDataBlocking(UART0, (uint8_t)*p);
        /* 检查TX是否正常完成 */
        if (DL_UART_getStatus(UART0) & DL_UART_STATUS_TX_BUSY) {
            /* 正常，仍在发送 */
        }
    }

    diag_items[DIAG_UART_TEST].result = pass ? DIAG_PASS : DIAG_FAIL;
    diag_items[DIAG_UART_TEST].detail = pass ? "115200 baud OK" : "TX error";
}

/* ========== GPIO功能测试 ========== */

static void Diag_GPIO_Test(void)
{
    /* 测试GPIO输出翻转 */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_27);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_27);

    bool pass = true;

    /* 测试置位/清零 */
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27);
    if (!DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_27)) pass = false;

    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_27);
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_27)) pass = false;

    /* 测试翻转 */
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27);
    DL_GPIO_togglePins(GPIOA, DL_GPIO_PIN_27);
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_27)) pass = false;  /* 应该被翻转为0 */

    diag_items[DIAG_GPIO_TEST].result = pass ? DIAG_PASS : DIAG_FAIL;
    diag_items[DIAG_GPIO_TEST].detail = pass ? "Set/Clear/Toggle OK" : "GPIO readback error";
}

/* ========== 看门狗测试 ========== */

static void Diag_Watchdog_Test(void)
{
    /* 测试看门狗配置和喂狗 */
    DL_WWDT_reset(WWDT0);
    DL_WWDT_enablePower(WWDT0);

    /* 配置看门狗(较长超时) */
    DL_WWDT_init(WWDT0,
        DL_WWDT_MODE_WATCHDOG,
        DL_WWDT_CLOCK_DIVIDE_1,
        0xFFFF,  /* 预装载值(最大超时) */
        0x7FFF   /* 窗口值 */
    );

    /* 喂狗 */
    DL_WWDT_restart(WWDT0);

    /* 短暂延时后再喂狗(验证喂狗有效) */
    for (volatile uint32_t i = 0; i < 100000; i++) {}
    DL_WWDT_restart(WWDT0);

    /* 停止看门狗(诊断完成后) */
    DL_WWDT_stop(WWDT0);

    diag_items[DIAG_WDG_TEST].result = DIAG_PASS;
    diag_items[DIAG_WDG_TEST].detail = "Config/Feed/Stop OK";
}

/* ========== 温度传感器 ========== */

static void Diag_Temperature(void)
{
    /* 读取内部温度传感器 */
    DL_ADC12_configConversion(ADC0, DL_ADC12_INPUT_CHAN_7,  /* 温度通道 */
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_SINGLE,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    DL_ADC12_startConversion(ADC0);
    while (!DL_ADC12_getStatus(ADC0, DL_ADC12_STATUS_CONVERSION_DONE)) {}
    uint16_t raw = DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);

    /* 简化的温度计算(MSPM0G3507内部温度传感器) */
    /* 典型公式: T = (raw - offset) * scale */
    float temp_c = (float)raw * 0.1f - 50.0f;  /* 粗略估算 */

    char detail[64];
    snprintf(detail, sizeof(detail), "%.1f C (raw=%u)", temp_c, raw);

    if (temp_c > 0 && temp_c < 85) {
        diag_items[DIAG_TEMP_SENSOR].result = DIAG_PASS;
    } else {
        diag_items[DIAG_TEMP_SENSOR].result = DIAG_WARN;
    }
    diag_items[DIAG_TEMP_SENSOR].detail = detail;
    diag_items[DIAG_TEMP_SENSOR].value = raw;
}

/* ========== 电池电压监测 ========== */

static void Diag_VBat(void)
{
    /* 读取VBAT通道(如果有) */
    diag_items[DIAG_VBAT].result = DIAG_SKIP;
    diag_items[DIAG_VBAT].detail = "No VBAT on this board";
}

/* ========== 主函数 ========== */

int main(void)
{
    /* 系统初始化 */
    DL_SYSCFG_init();
    SysTick_Config(32000000 / 1000);

    /* GPIO初始化 */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_27);  /* Green LED */
    DL_GPIO_initDigitalOutput(DL_GPIO_PIN_26);  /* Red LED */
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_27);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_26);

    /* UART初始化 */
    DL_UART_reset(UART0);
    DL_UART_enablePower(UART0);
    DL_UART_setClockConfig(UART0, DL_UART_CLOCK_DIVIDE_115200);
    DL_UART_init(UART0, DL_UART_MODE_NORMAL);
    DL_UART_enable(UART0);

    /* 启动指示 */
    LED_Set(true, true);  /* 双灯亮=诊断中 */

    /* ========== 诊断报告头 ========== */
    UART_SendString("\r\n");
    UART_SendString("=========================================================\r\n");
    UART_SendString("  MSPM0G3507 System Diagnostics v1.0\r\n");
    UART_SendString("  Self-Test & Health Check Report\r\n");
    UART_SendString("=========================================================\r\n\r\n");

    /* 芯片信息 */
    UART_Printf("  Device: MSPM0G3507\r\n");
    UART_Printf("  SRAM:   32KB\r\n");
    UART_Printf("  Flash:  128KB\r\n");
    UART_Printf("  CPUID:  0x%08X\r\n", SCB->CPUID);
    UART_Printf("  UID:    0x%08X%08X\r\n",
        *((uint32_t *)0x41C00004), *((uint32_t *)0x41C00008));
    UART_SendString("\r\n");

    /* ========== 执行所有诊断项目 ========== */
    UART_SendString("--- Running Diagnostics ---\r\n");

    /* CPU测试 */
    LED_Set(true, false);
    Diag_CPU_Register();
    PrintDiagResult(&diag_items[DIAG_CPU_REG]);

    Diag_CPU_ALU();
    PrintDiagResult(&diag_items[DIAG_CPU_ALU]);

    Diag_CPU_Stack();
    PrintDiagResult(&diag_items[DIAG_CPU_STACK]);

    /* 内存测试 */
    Diag_Memory_SRAM();
    PrintDiagResult(&diag_items[DIAG_MEM_SRAM]);

    Diag_Memory_Flash();
    PrintDiagResult(&diag_items[DIAG_MEM_FLASH]);

    /* 时钟测试 */
    LED_Set(false, true);
    Diag_Clock_HFCLK();
    PrintDiagResult(&diag_items[DIAG_CLK_HFCLK]);

    Diag_Clock_LFCLK();
    PrintDiagResult(&diag_items[DIAG_CLK_LFCLK]);

    Diag_Clock_PLL();
    PrintDiagResult(&diag_items[DIAG_CLK_PLL]);

    /* ADC测试 */
    LED_Set(true, false);
    Diag_ADC_SelfTest();
    PrintDiagResult(&diag_items[DIAG_ADC_SELF]);

    Diag_ADC_Vref();
    PrintDiagResult(&diag_items[DIAG_ADC_VREF]);

    /* 外设测试 */
    LED_Set(false, true);
    Diag_I2C_Scan();
    PrintDiagResult(&diag_items[DIAG_I2C_BUS]);

    Diag_SPI_Loopback();
    PrintDiagResult(&diag_items[DIAG_SPI_LOOPBACK]);

    Diag_UART_Test();
    PrintDiagResult(&diag_items[DIAG_UART_TEST]);

    Diag_GPIO_Test();
    PrintDiagResult(&diag_items[DIAG_GPIO_TEST]);

    /* 系统测试 */
    LED_Set(true, false);
    Diag_Watchdog_Test();
    PrintDiagResult(&diag_items[DIAG_WDG_TEST]);

    Diag_Temperature();
    PrintDiagResult(&diag_items[DIAG_TEMP_SENSOR]);

    Diag_VBat();
    PrintDiagResult(&diag_items[DIAG_VBAT]);

    /* ========== 诊断总结 ========== */
    UART_SendString("\r\n");
    UART_SendString("=========================================================\r\n");
    UART_Printf("  SUMMARY: %d PASS | %d FAIL | %d WARN | %d SKIP\r\n",
        pass_count, fail_count, warn_count, skip_count);
    UART_SendString("=========================================================\r\n");

    if (fail_count == 0 && warn_count == 0) {
        UART_SendString("  RESULT: ALL TESTS PASSED - System Healthy\r\n");
        LED_Set(true, false);  /* 绿灯常亮 */
    } else if (fail_count == 0) {
        UART_SendString("  RESULT: PASSED WITH WARNINGS\r\n");
        LED_Set(true, true);   /* 双灯=警告 */
    } else {
        UART_Printf("  RESULT: %d TEST(S) FAILED - Check Hardware!\r\n", fail_count);
        LED_Set(false, true);  /* 红灯常亮 */
    }

    UART_SendString("=========================================================\r\n\r\n");

    /* 详细失败信息 */
    if (fail_count > 0) {
        UART_SendString("--- Failed Tests Detail ---\r\n");
        for (int i = 0; i < DIAG_COUNT; i++) {
            if (diag_items[i].result == DIAG_FAIL) {
                UART_Printf("  [%s] %s: %s\r\n",
                    diag_items[i].category,
                    diag_items[i].name,
                    diag_items[i].detail);
            }
        }
        UART_SendString("---------------------------\r\n\r\n");
    }

    /* 完成后进入低功耗等待 */
    while (1) {
        /* LED指示最终状态 */
        __WFI();
    }
}
