/**
 * @file wireless_sensor_network.c
 * @brief 无线传感器网络 - MSPM0G3507系统集成示例
 *
 * 功能：NRF24L01无线模块 + 多节点星型拓扑 + 数据汇聚 + UART上报
 * 硬件：MSPM0G3507 + NRF24L01(SPI) + DS18B20温度传感器 + LED指示
 *
 * 接线：
 *   NRF24L01 CE   -> PB0  (GPIO)
 *   NRF24L01 CSN  -> PB1  (GPIO)
 *   NRF24L01 MOSI -> PA5  (SPI0)
 *   NRF24L01 MISO -> PA6  (SPI0)
 *   NRF24L01 SCK  -> PA4  (SPI0)
 *   NRF24L01 IRQ  -> PA11 (GPIO中断)
 *   LED0          -> PB14 (GPIO)
 *   LED1          -> PB15 (GPIO)
 *
 * 协议：
 *   - 汇聚节点（本机）地址：0xA0A0A0A0A0
 *   - 子节点地址：0xA1A1A1A1A1 ~ 0xA5A5A5A5A5（最多5个子节点）
 *   - 数据帧格式：[节点ID(1B)][温度(2B)][电压(2B)][序列号(2B)][校验(1B)]
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>

/* ========== NRF24L01寄存器定义 ========== */
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_SETUP_RETR  0x04
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_RX_ADDR_P0  0x0A
#define NRF_TX_ADDR     0x10
#define NRF_RX_PW_P0    0x11
#define NRF_FIFO_STATUS 0x17
#define NRF_W_TX_PAYLOAD 0xA0
#define NRF_W_REGISTER  0x20
#define NRF_R_REGISTER  0x00
#define NRF_R_RX_PAYLOAD 0x61
#define NRF_FLUSH_TX    0xE1
#define NRF_FLUSH_RX    0xE2
#define NRF_NOP         0xFF

/* ========== 引脚定义 ========== */
#define NRF_CE_PORT     GPIOB
#define NRF_CE_PIN      DL_GPIO_PIN_0
#define NRF_CSN_PORT    GPIOB
#define NRF_CSN_PIN     DL_GPIO_PIN_1
#define LED0_PORT       GPIOB
#define LED0_PIN        DL_GPIO_PIN_14
#define LED1_PORT       GPIOB
#define LED1_PIN        DL_GPIO_PIN_15

/* ========== 网络配置 ========== */
#define MAX_CHILD_NODES    5
#define PAYLOAD_SIZE       8
#define CHANNEL            40
#define RETRY_DELAY_US     250
#define RETRY_COUNT        10

/* ========== 数据帧结构 ========== */
typedef struct {
    uint8_t  node_id;       /* 节点编号 1~5 */
    int16_t  temperature;   /* 温度 x100 */
    uint16_t voltage_mv;    /* 电池电压 mV */
    uint16_t seq_num;       /* 序列号 */
    uint8_t  checksum;      /* 校验和 */
} __attribute__((packed)) SensorFrame_t;

/* ========== 节点状态 ========== */
typedef struct {
    uint32_t last_rx_time;    /* 最后接收时间 */
    uint16_t last_seq;        /* 最后序列号 */
    uint16_t lost_count;      /* 丢包计数 */
    bool     online;          /* 在线状态 */
    SensorFrame_t data;       /* 最新数据 */
} NodeStatus_t;

static NodeStatus_t gNodes[MAX_CHILD_NODES];
static volatile uint32_t gTickMs = 0;
static volatile bool gNrfIrqFlag = false;

/* 汇聚节点地址 */
static const uint8_t SINK_ADDR[5] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0};

/* =================================================================
 * 基础驱动
 * ================================================================= */

void SysTick_Handler(void) { gTickMs++; }

static void delay_ms(uint32_t ms) {
    uint32_t s = gTickMs;
    while ((gTickMs - s) < ms);
}

static void delay_us(uint32_t us) {
    /* 粗略微秒延时 @32MHz */
    volatile uint32_t cnt = us * 8;
    while (cnt--);
}

/* =================================================================
 * SPI驱动
 * ================================================================= */

static uint8_t SPI_RW(uint8_t data) {
    DL_SPI_transmitData8(SPI0, data);
    while (DL_SPI_isBusy(SPI0));
    return (uint8_t)DL_SPI_receiveData8(SPI0);
}

/* =================================================================
 * NRF24L01驱动
 * ================================================================= */

static void NRF_CSN_Low(void)  { DL_GPIO_clearPins(NRF_CSN_PORT, NRF_CSN_PIN); }
static void NRF_CSN_High(void) { DL_GPIO_setPins(NRF_CSN_PORT, NRF_CSN_PIN); }
static void NRF_CE_Low(void)   { DL_GPIO_clearPins(NRF_CE_PORT, NRF_CE_PIN); }
static void NRF_CE_High(void)  { DL_GPIO_setPins(NRF_CE_PORT, NRF_CE_PIN); }

/**
 * @brief 写NRF24L01寄存器
 */
static uint8_t NRF_WriteReg(uint8_t reg, uint8_t val) {
    uint8_t status;
    NRF_CSN_Low();
    status = SPI_RW(NRF_W_REGISTER | reg);
    SPI_RW(val);
    NRF_CSN_High();
    return status;
}

/**
 * @brief 读NRF24L01寄存器
 */
static uint8_t NRF_ReadReg(uint8_t reg) {
    uint8_t val;
    NRF_CSN_Low();
    SPI_RW(NRF_R_REGISTER | reg);
    val = SPI_RW(NRF_NOP);
    NRF_CSN_High();
    return val;
}

/**
 * @brief 写多字节寄存器（如地址）
 */
static void NRF_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len) {
    NRF_CSN_Low();
    SPI_RW(NRF_W_REGISTER | reg);
    for (uint8_t i = 0; i < len; i++) {
        SPI_RW(buf[i]);
    }
    NRF_CSN_High();
}

/**
 * @brief 读多字节数据
 */
static void NRF_ReadBuf(uint8_t reg, uint8_t *buf, uint8_t len) {
    NRF_CSN_Low();
    SPI_RW(NRF_R_REGISTER | reg);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI_RW(NRF_NOP);
    }
    NRF_CSN_High();
}

/**
 * @brief 写TX Payload
 */
static void NRF_WritePayload(const uint8_t *data, uint8_t len) {
    NRF_CSN_Low();
    SPI_RW(NRF_W_TX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++) {
        SPI_RW(data[i]);
    }
    NRF_CSN_High();
}

/**
 * @brief 读RX Payload
 */
static void NRF_ReadPayload(uint8_t *data, uint8_t len) {
    NRF_CSN_Low();
    SPI_RW(NRF_R_RX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++) {
        data[i] = SPI_RW(NRF_NOP);
    }
    NRF_CSN_High();
}

/**
 * @brief NRF24L01初始化为接收模式（汇聚节点）
 */
static void NRF_InitAsSink(void) {
    NRF_CE_Low();
    delay_ms(100);

    NRF_WriteReg(NRF_CONFIG, 0x0F);        /* 上电, CRC 2字节, 接收模式 */
    NRF_WriteReg(NRF_EN_AA, 0x01);          /* 通道0自动应答 */
    NRF_WriteReg(NRF_EN_RXADDR, 0x01);      /* 使能通道0 */
    NRF_WriteReg(NRF_SETUP_AW, 0x03);       /* 地址宽度5字节 */
    NRF_WriteReg(NRF_SETUP_RETR, 
        (RETRY_DELAY_US / 250 - 1) << 4 | RETRY_COUNT);
    NRF_WriteReg(NRF_RF_CH, CHANNEL);       /* 信道 */
    NRF_WriteReg(NRF_RF_SETUP, 0x0E);       /* 2Mbps, 0dBm */
    NRF_WriteReg(NRF_RX_PW_P0, PAYLOAD_SIZE);

    /* 设置接收地址（汇聚节点地址） */
    NRF_WriteBuf(NRF_RX_ADDR_P0, SINK_ADDR, 5);
    NRF_WriteBuf(NRF_TX_ADDR, SINK_ADDR, 5);

    /* 清空FIFO */
    NRF_CSN_Low();
    SPI_RW(NRF_FLUSH_TX);
    NRF_CSN_High();
    NRF_CSN_Low();
    SPI_RW(NRF_FLUSH_RX);
    NRF_CSN_High();

    /* 清除中断标志 */
    NRF_WriteReg(NRF_STATUS, 0x70);

    NRF_CE_High();
    delay_ms(5);
}

/**
 * @brief 检查是否有数据可读
 * @return true=有数据
 */
static bool NRF_DataReady(void) {
    /* 通过STATUS寄存器的RX_DR位判断 */
    return (NRF_ReadReg(NRF_STATUS) & 0x40) ? true : false;
}

/**
 * @brief 接收数据帧
 * @param buf 输出缓冲区
 * @return true=接收成功
 */
static bool NRF_Receive(uint8_t *buf) {
    if (!NRF_DataReady()) return false;

    NRF_ReadPayload(buf, PAYLOAD_SIZE);

    /* 清除RX_DR中断 */
    NRF_WriteReg(NRF_STATUS, 0x40);

    return true;
}

/* =================================================================
 * 校验和计算
 * ================================================================= */

static uint8_t CalcChecksum(const SensorFrame_t *f) {
    const uint8_t *p = (const uint8_t *)f;
    uint8_t sum = 0;
    for (uint8_t i = 0; i < sizeof(SensorFrame_t) - 1; i++) {
        sum ^= p[i];
    }
    return sum;
}

/* =================================================================
 * 数据处理
 * ================================================================= */

/**
 * @brief 处理接收到的传感器帧
 */
static bool ProcessFrame(const uint8_t *raw) {
    SensorFrame_t frame;
    memcpy(&frame, raw, sizeof(SensorFrame_t));

    /* 校验 */
    if (CalcChecksum(&frame) != frame.checksum) return false;
    if (frame.node_id < 1 || frame.node_id > MAX_CHILD_NODES) return false;

    /* 更新节点状态 */
    NodeStatus_t *node = &gNodes[frame.node_id - 1];
    node->data = frame;
    node->last_rx_time = gTickMs;
    node->online = true;

    /* 丢包检测 */
    if (node->last_seq != 0 && frame.seq_num > node->last_seq + 1) {
        node->lost_count += frame.seq_num - node->last_seq - 1;
    }
    node->last_seq = frame.seq_num;

    return true;
}

/**
 * @brief 通过UART上报所有节点数据
 */
static void ReportToPC(void) {
    char buf[128];
    snprintf(buf, sizeof(buf), "=== Network Status @%lus ===\r\n",
             (unsigned long)(gTickMs / 1000));
    /* UART输出 */
    for (const char *p = buf; *p; p++) {
        DL_UART_transmitData(UART0, *p);
        while (!DL_UART_isTXEmpty(UART0));
    }

    for (uint8_t i = 0; i < MAX_CHILD_NODES; i++) {
        NodeStatus_t *n = &gNodes[i];
        if (n->online) {
            float temp = n->data.temperature / 100.0f;
            snprintf(buf, sizeof(buf),
                     "Node%d: T=%.2fC V=%dmV Seq=%u Lost=%u %s\r\n",
                     n->data.node_id, temp, n->data.voltage_mv,
                     n->data.seq_num, n->lost_count,
                     ((gTickMs - n->last_rx_time) > 30000) ? "TIMEOUT" : "OK");
            for (const char *p = buf; *p; p++) {
                DL_UART_transmitData(UART0, *p);
                while (!DL_UART_isTXEmpty(UART0));
            }
        }
    }
}

/* =================================================================
 * GPIO中断（NRF24L01 IRQ）
 * ================================================================= */

void GROUP1_IRQHandler(void) {
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOA, DL_GPIO_PIN_11);
    if (flags & DL_GPIO_PIN_11) {
        gNrfIrqFlag = true;
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_11);
    }
}

/* =================================================================
 * 主函数
 * ================================================================= */
int main(void) {
    /* 系统初始化 */
    DL_SYSCFG_init();
    SysTick_Config(32000);  /* 1ms tick */

    /* GPIO */
    DL_GPIO_initDigitalOutput(NRF_CE_PIN);
    DL_GPIO_initDigitalOutput(NRF_CSN_PIN);
    DL_GPIO_initDigitalOutput(LED0_PIN);
    DL_GPIO_initDigitalOutput(LED1_PIN);
    NRF_CSN_High();
    NRF_CE_Low();

    /* SPI */
    DL_SPI_enable(SPI0);

    /* UART */
    NVIC_EnableIRQ(UART0_IRQn);

    /* NRF24L01 IRQ引脚 */
    DL_GPIO_initDigitalInputFeatures(DL_GPIO_PIN_11,
        DL_GPIO_RESISTOR_PULLUP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_setInterruptEdge(GPIOA, DL_GPIO_PIN_11, DL_GPIO_EDGE_FALLING);
    DL_GPIO_enableInterrupt(GPIOA, DL_GPIO_PIN_11);
    NVIC_EnableIRQ(GROUP1_IRQn);

    /* 初始化节点状态 */
    memset(gNodes, 0, sizeof(gNodes));

    /* 初始化NRF24L01 */
    NRF_InitAsSink();

    /* LED指示就绪 */
    DL_GPIO_setPins(LED0_PORT, LED0_PIN);

    uint32_t last_report = 0;
    const uint32_t REPORT_INTERVAL = 5000;  /* 每5秒上报 */

    /* ===== 主循环 ===== */
    while (1) {
        /* 接收数据 */
        uint8_t rx_buf[PAYLOAD_SIZE];
        if (NRF_Receive(rx_buf)) {
            if (ProcessFrame(rx_buf)) {
                /* 收到有效数据，LED1闪烁 */
                DL_GPIO_togglePins(LED1_PORT, LED1_PIN);
            }
        }

        /* 超时检测：30秒无数据则标记离线 */
        for (uint8_t i = 0; i < MAX_CHILD_NODES; i++) {
            if (gNodes[i].online &&
                (gTickMs - gNodes[i].last_rx_time) > 30000) {
                gNodes[i].online = false;
            }
        }

        /* 定时上报 */
        if ((gTickMs - last_report) >= REPORT_INTERVAL) {
            ReportToPC();
            last_report = gTickMs;
        }

        delay_ms(10);
    }
}
