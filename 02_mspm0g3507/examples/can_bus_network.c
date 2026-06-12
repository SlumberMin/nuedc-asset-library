/**
 * @file can_bus_network.c
 * @brief MSPM0G3507 CAN总线网络示例 - MCP2515多节点通信与数据路由
 * 
 * 硬件连接：
 *   MCP2515 SPI接口:
 *     SCK  -> PA2 (SPI0_SCK)
 *     MOSI -> PA4 (SPI0_MOSI)
 *     MISO -> PA6 (SPI0_MISO)
 *     CS   -> PA3 (GPIO)
 *     INT  -> PB2 (外部中断)
 *     RST  -> PB3 (GPIO)
 * 
 *   本节点CAN总线连接:
 *     CANH / CANL -> CAN收发器(TJA1050) -> CAN总线
 * 
 * 功能说明：
 *   - 通过MCP2515 SPI转CAN控制器实现CAN总线通信
 *   - 支持多节点网络，每个节点有唯一ID
 *   - 实现消息路由：根据CAN ID将数据转发到不同处理函数
 *   - 支持标准帧(11bit)和扩展帧(29bit)
 *   - 实现节点心跳检测和在线状态管理
 * 
 * 适用场景：电赛中多板间CAN通信、分布式控制系统
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================
 * 第一部分：MCP2515寄存器定义与底层驱动
 * ============================================================ */

/* MCP2515 SPI命令 */
#define MCP2515_CMD_RESET       0xC0    /* 复位命令 */
#define MCP2515_CMD_READ        0x03    /* 读寄存器 */
#define MCP2515_CMD_WRITE       0x02    /* 写寄存器 */
#define MCP2515_CMD_RTS_TXB0    0x81    /* 请求发送TX缓冲区0 */
#define MCP2515_CMD_RTS_TXB1    0x82    /* 请求发送TX缓冲区1 */
#define MCP2515_CMD_RTS_TXB2    0x84    /* 请求发送TX缓冲区2 */
#define MCP2515_CMD_READ_STATUS 0xA0    /* 读状态 */
#define MCP2515_CMD_BIT_MODIFY  0x05    /* 位修改 */

/* MCP2515寄存器地址 */
#define MCP2515_REG_CANSTAT     0x0E    /* CAN状态寄存器 */
#define MCP2515_REG_CANCTRL     0x0F    /* CAN控制寄存器 */
#define MCP2515_REG_CNF3        0x28    /* 配置寄存器3 */
#define MCP2515_REG_CNF2        0x29    /* 配置寄存器2 */
#define MCP2515_REG_CNF1        0x2A    /* 配置寄存器1 */
#define MCP2515_REG_CANINTE     0x2B    /* 中断使能寄存器 */
#define MCP2515_REG_CANINTF     0x2C    /* 中断标志寄存器 */
#define MCP2515_REG_TXB0CTRL    0x30    /* TX缓冲区0控制 */
#define MCP2515_REG_TXB0SIDH    0x31    /* TX缓冲区0标准ID高位 */
#define MCP2515_REG_TXB0SIDL    0x32    /* TX缓冲区0标准ID低位 */
#define MCP2515_REG_TXB0DLC     0x35    /* TX缓冲区0数据长度 */
#define MCP2515_REG_TXB0D0      0x36    /* TX缓冲区0数据字节0 */
#define MCP2515_REG_RXB0CTRL    0x60    /* RX缓冲区0控制 */
#define MCP2515_REG_RXB0SIDH    0x61    /* RX缓冲区0标准ID高位 */
#define MCP2515_REG_RXB0SIDL    0x62    /* RX缓冲区0标准ID低位 */
#define MCP2515_REG_RXB0DLC     0x65    /* RX缓冲区0数据长度 */
#define MCP2515_REG_RXB0D0      0x66    /* RX缓冲区0数据字节0 */
#define MCP2515_REG_RXB1CTRL    0x70    /* RX缓冲区1控制 */
#define MCP2515_REG_RXB1SIDH    0x71    /* RX缓冲区1标准ID高位 */
#define MCP2515_REG_RXB1SIDL    0x72    /* RX缓冲区1标准ID低位 */
#define MCP2515_REG_RXB1DLC     0x75    /* RX缓冲区1数据长度 */
#define MCP2515_REG_RXB1D0      0x76    /* RX缓冲区1数据字节0 */

/* MCP2515工作模式 */
#define MCP2515_MODE_NORMAL     0x00    /* 正常模式 */
#define MCP2515_MODE_SLEEP      0x20    /* 睡眠模式 */
#define MCP2515_MODE_LOOPBACK   0x40    /* 回环模式(测试用) */
#define MCP2515_MODE_LISTEN     0x60    /* 只听模式 */
#define MCP2515_MODE_CONFIG     0x80    /* 配置模式 */

/* 中断标志位 */
#define MCP2515_INT_RX0IF       0x01    /* RX缓冲区0满中断 */
#define MCP2515_INT_RX1IF       0x02    /* RX缓冲区1满中断 */
#define MCP2515_INT_TX0IF       0x04    /* TX缓冲区0空中断 */
#define MCP2515_INT_TX1IF       0x08    /* TX缓冲区1空中断 */
#define MCP2515_INT_TX2IF       0x10    /* TX缓冲区2空中断 */
#define MCP2515_INT_ERRIF       0x20    /* 错误中断 */
#define MCP2515_INT_WAKIF       0x40    /* 唤醒中断 */
#define MCP2515_INT_MERRF       0x80    /* 总线错误中断 */

/* 引脚定义 */
#define MCP2515_CS_PORT         GPIOA
#define MCP2515_CS_PIN          DL_GPIO_PIN_3
#define MCP2515_RST_PORT        GPIOB
#define MCP2515_RST_PIN         DL_GPIO_PIN_3

/* CAN消息结构体 */
typedef struct {
    uint32_t id;            /* CAN ID (11bit标准 或 29bit扩展) */
    bool     extended;      /* 是否为扩展帧 */
    bool     remote;        /* 是否为远程帧 */
    uint8_t  dlc;           /* 数据长度 (0~8) */
    uint8_t  data[8];       /* 数据内容 */
} CAN_Message_t;

/* 节点状态结构体 */
typedef struct {
    uint8_t  node_id;       /* 节点ID */
    bool     online;        /* 是否在线 */
    uint32_t last_heartbeat;/* 最后心跳时间(ms) */
    uint32_t rx_count;      /* 接收计数 */
    uint32_t tx_count;      /* 发送计数 */
    uint32_t error_count;   /* 错误计数 */
} CAN_NodeInfo_t;

/* ============================================================
 * 第二部分：系统时间与全局变量
 * ============================================================ */

static volatile uint32_t g_systick_ms = 0;     /* 系统毫秒计数器 */
static volatile bool g_mcp2515_int_flag = false; /* MCP2515中断标志 */

/* 本节点配置 */
#define LOCAL_NODE_ID       0x01    /* 本节点CAN ID */
#define HEARTBEAT_INTERVAL  1000    /* 心跳间隔(ms) */
#define MAX_REMOTE_NODES    8       /* 最大远程节点数 */
#define CAN_BAUDRATE_500K   1       /* 500kbps波特率 */

/* 远程节点表 */
static CAN_NodeInfo_t g_remote_nodes[MAX_REMOTE_NODES];
static uint8_t g_remote_node_count = 0;

/* 本地统计 */
static uint32_t g_local_tx_count = 0;
static uint32_t g_local_rx_count = 0;
static uint32_t g_local_err_count = 0;

/* ============================================================
 * 第三部分：SPI与GPIO底层操作
 * ============================================================ */

/**
 * @brief 延时微秒 (软件延时)
 * @param us 微秒数
 */
static void delay_us(uint32_t us)
{
    /* 32MHz主频下，约32个周期/us */
    while (us--) {
        __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP();
    }
}

/**
 * @brief 延时毫秒
 * @param ms 毫秒数
 */
static void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

/**
 * @brief 选择MCP2515 (CS拉低)
 */
static void mcp2515_select(void)
{
    DL_GPIO_clearPins(MCP2515_CS_PORT, MCP2515_CS_PIN);
    delay_us(1);
}

/**
 * @brief 释放MCP2515 (CS拉高)
 */
static void mcp2515_deselect(void)
{
    delay_us(1);
    DL_GPIO_setPins(MCP2515_CS_PORT, MCP2515_CS_PIN);
}

/**
 * @brief SPI发送接收一个字节
 * @param data 要发送的字节
 * @return 接收到的字节
 */
static uint8_t spi_transfer(uint8_t data)
{
    /* 等待TX FIFO有空间 */
    while (DL_SPI_isBusy(SPI0)) ;
    
    /* 发送数据 */
    DL_SPI_transmitData8(SPI0, data);
    
    /* 等待传输完成并读取接收数据 */
    while (!DL_SPI_isRXFIFOEmpty(SPI0)) ;
    return (uint8_t)DL_SPI_receiveData8(SPI0);
}

/* ============================================================
 * 第四部分：MCP2515驱动层
 * ============================================================ */

/**
 * @brief 复位MCP2515
 */
static void mcp2515_reset(void)
{
    /* 硬件复位 */
    DL_GPIO_clearPins(MCP2515_RST_PORT, MCP2515_RST_PIN);
    delay_ms(10);
    DL_GPIO_setPins(MCP2515_RST_PORT, MCP2515_RST_PIN);
    delay_ms(10);
    
    /* 软件复位 */
    mcp2515_select();
    spi_transfer(MCP2515_CMD_RESET);
    mcp2515_deselect();
    delay_ms(10);
}

/**
 * @brief 读取MCP2515寄存器
 * @param addr 寄存器地址
 * @return 寄存器值
 */
static uint8_t mcp2515_read_reg(uint8_t addr)
{
    uint8_t value;
    mcp2515_select();
    spi_transfer(MCP2515_CMD_READ);
    spi_transfer(addr);
    value = spi_transfer(0xFF);
    mcp2515_deselect();
    return value;
}

/**
 * @brief 写入MCP2515寄存器
 * @param addr 寄存器地址
 * @param value 要写入的值
 */
static void mcp2515_write_reg(uint8_t addr, uint8_t value)
{
    mcp2515_select();
    spi_transfer(MCP2515_CMD_WRITE);
    spi_transfer(addr);
    spi_transfer(value);
    mcp2515_deselect();
}

/**
 * @brief 位修改寄存器 (只修改特定位)
 * @param addr 寄存器地址
 * @param mask 位掩码 (1的位会被修改)
 * @param data 新值
 */
static void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    mcp2515_select();
    spi_transfer(MCP2515_CMD_BIT_MODIFY);
    spi_transfer(addr);
    spi_transfer(mask);
    spi_transfer(data);
    mcp2515_deselect();
}

/**
 * @brief 读取MCP2515状态
 * @return 状态字节
 */
static uint8_t mcp2515_read_status(void)
{
    uint8_t status;
    mcp2515_select();
    spi_transfer(MCP2515_CMD_READ_STATUS);
    status = spi_transfer(0xFF);
    mcp2515_deselect();
    return status;
}

/**
 * @brief 设置MCP2515工作模式
 * @param mode 目标模式
 * @return true=设置成功, false=超时
 */
static bool mcp2515_set_mode(uint8_t mode)
{
    uint32_t timeout = 1000;
    
    mcp2515_bit_modify(MCP2515_REG_CANCTRL, 0xE0, mode);
    
    /* 等待模式切换完成 */
    while (timeout--) {
        if ((mcp2515_read_reg(MCP2515_REG_CANSTAT) & 0xE0) == mode) {
            return true;
        }
        delay_us(100);
    }
    return false;
}

/**
 * @brief 初始化MCP2515 CAN控制器
 * @param baudrate_setting 波特率配置 (1=500kbps@8MHz晶振)
 * @return true=初始化成功
 */
static bool mcp2515_init(uint8_t baudrate_setting)
{
    /* 复位MCP2515 */
    mcp2515_reset();
    
    /* 进入配置模式 */
    if (!mcp2515_set_mode(MCP2515_MODE_CONFIG)) {
        return false;
    }
    
    /* 配置波特率 (使用8MHz晶振) */
    switch (baudrate_setting) {
        case 1: /* 500kbps: TQ=250ns, 16TQ, 采样点87.5% */
            mcp2515_write_reg(MCP2515_REG_CNF1, 0x00); /* BRP=0, SJW=1 */
            mcp2515_write_reg(MCP2515_REG_CNF2, 0xF0); /* BLT=1, SAM=0, PS1=6TQ, PRSEG=7TQ */
            mcp2515_write_reg(MCP2515_REG_CNF3, 0x02); /* PS2=3TQ (实际需+1) */
            break;
        case 2: /* 250kbps */
            mcp2515_write_reg(MCP2515_REG_CNF1, 0x01); /* BRP=1 */
            mcp2515_write_reg(MCP2515_REG_CNF2, 0xF0);
            mcp2515_write_reg(MCP2515_REG_CNF3, 0x02);
            break;
        case 3: /* 125kbps */
            mcp2515_write_reg(MCP2515_REG_CNF1, 0x03); /* BRP=3 */
            mcp2515_write_reg(MCP2515_REG_CNF2, 0xF0);
            mcp2515_write_reg(MCP2515_REG_CNF3, 0x02);
            break;
        default:
            return false;
    }
    
    /* 配置RX缓冲区0：接收所有消息 */
    mcp2515_write_reg(MCP2515_REG_RXB0CTRL, 0x60); /* 接收所有消息 */
    
    /* 配置RX缓冲区1：接收所有消息 */
    mcp2515_write_reg(MCP2515_REG_RXB1CTRL, 0x60); /* 接收所有消息 */
    
    /* 使能RX和错误中断 */
    mcp2515_write_reg(MCP2515_REG_CANINTE, 
                      MCP2515_INT_RX0IF | MCP2515_INT_RX1IF | MCP2515_INT_ERRIF);
    
    /* 清除中断标志 */
    mcp2515_write_reg(MCP2515_REG_CANINTF, 0x00);
    
    /* 切换到正常模式 */
    return mcp2515_set_mode(MCP2515_MODE_NORMAL);
}

/**
 * @brief 发送CAN消息
 * @param msg 指向CAN消息结构体的指针
 * @return true=发送成功, false=发送缓冲区满
 */
static bool mcp2515_send_message(const CAN_Message_t *msg)
{
    uint8_t tx_buf_addr;
    uint8_t status;
    
    /* 检查哪个TX缓冲区空闲 */
    status = mcp2515_read_status();
    if (!(status & 0x04)) {          /* TXB0是否空闲 */
        tx_buf_addr = MCP2515_REG_TXB0CTRL;
    } else if (!(status & 0x10)) {   /* TXB1是否空闲 */
        tx_buf_addr = MCP2515_REG_TXB0CTRL + 0x10; /* TXB1起始地址 */
    } else if (!(status & 0x40)) {   /* TXB2是否空闲 */
        tx_buf_addr = MCP2515_REG_TXB0CTRL + 0x20; /* TXB2起始地址 */
    } else {
        return false; /* 所有TX缓冲区都满 */
    }
    
    /* 写入标准ID (高8位) */
    mcp2515_write_reg(tx_buf_addr + 1, (uint8_t)(msg->id >> 3));
    
    /* 写入标准ID (低3位) + 扩展标志 */
    uint8_t sidl = (uint8_t)((msg->id & 0x07) << 5);
    if (msg->extended) sidl |= 0x08; /* EXIDE位 */
    mcp2515_write_reg(tx_buf_addr + 2, sidl);
    
    /* 扩展ID部分 */
    if (msg->extended) {
        mcp2515_write_reg(tx_buf_addr + 3, (uint8_t)(msg->id >> 16));
        mcp2515_write_reg(tx_buf_addr + 4, (uint8_t)(msg->id >> 8));
    }
    
    /* 写入DLC和数据 */
    uint8_t dlc = msg->dlc;
    if (dlc > 8) dlc = 8;
    if (msg->remote) dlc |= 0x40; /* 远程帧标志 */
    mcp2515_write_reg(tx_buf_addr + 5, dlc);
    
    for (uint8_t i = 0; i < (dlc & 0x0F); i++) {
        mcp2515_write_reg(tx_buf_addr + 6 + i, msg->data[i]);
    }
    
    /* 请求发送 */
    uint8_t rts_cmd;
    if (tx_buf_addr == MCP2515_REG_TXB0CTRL) {
        rts_cmd = MCP2515_CMD_RTS_TXB0;
    } else if (tx_buf_addr == MCP2515_REG_TXB0CTRL + 0x10) {
        rts_cmd = MCP2515_CMD_RTS_TXB1;
    } else {
        rts_cmd = MCP2515_CMD_RTS_TXB2;
    }
    
    mcp2515_select();
    spi_transfer(rts_cmd);
    mcp2515_deselect();
    
    return true;
}

/**
 * @brief 接收CAN消息
 * @param buf_idx 接收缓冲区索引 (0或1)
 * @param msg 输出的CAN消息
 * @return true=接收成功
 */
static bool mcp2515_receive_message(uint8_t buf_idx, CAN_Message_t *msg)
{
    uint8_t rx_buf_addr;
    uint8_t sidl, dlc;
    
    if (buf_idx == 0) {
        rx_buf_addr = MCP2515_REG_RXB0SIDH;
    } else {
        rx_buf_addr = MCP2515_REG_RXB1SIDH;
    }
    
    /* 读取标准ID */
    uint8_t sidh = mcp2515_read_reg(rx_buf_addr);
    sidl = mcp2515_read_reg(rx_buf_addr + 1);
    
    msg->extended = (sidl & 0x08) ? true : false;
    msg->id = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
    
    if (msg->extended) {
        /* 读取扩展ID */
        uint8_t eid8 = mcp2515_read_reg(rx_buf_addr + 2);
        uint8_t eid0 = mcp2515_read_reg(rx_buf_addr + 3);
        msg->id = (msg->id << 16) | ((uint32_t)eid8 << 8) | eid0;
    }
    
    /* 读取DLC */
    dlc = mcp2515_read_reg(rx_buf_addr + 4);
    msg->remote = (dlc & 0x40) ? true : false;
    msg->dlc = dlc & 0x0F;
    
    /* 读取数据 */
    for (uint8_t i = 0; i < msg->dlc && i < 8; i++) {
        msg->data[i] = mcp2515_read_reg(rx_buf_addr + 5 + i);
    }
    
    return true;
}

/**
 * @brief 检查是否有接收到的消息
 * @return 接收缓冲区标志位
 */
static uint8_t mcp2515_check_receive(void)
{
    return mcp2515_read_reg(MCP2515_REG_CANINTF) & (MCP2515_INT_RX0IF | MCP2515_INT_RX1IF);
}

/**
 * @brief 清除接收中断标志
 * @param flags 要清除的标志位
 */
static void mcp2515_clear_rx_flag(uint8_t flags)
{
    mcp2515_bit_modify(MCP2515_REG_CANINTF, flags, 0x00);
}

/* ============================================================
 * 第五部分：CAN网络管理层 - 多节点通信与路由
 * ============================================================ */

/* CAN ID定义 (按功能分组) */
#define CAN_ID_HEARTBEAT    0x100   /* 心跳消息基址: 0x100 + node_id */
#define CAN_ID_SENSOR_DATA  0x200   /* 传感器数据基址: 0x200 + node_id */
#define CAN_ID_CMD          0x300   /* 命令基址: 0x300 + target_id */
#define CAN_ID_BROADCAST    0x0FF   /* 广播地址 */
#define CAN_ID_ACK          0x0FE   /* 应答地址 */

/**
 * @brief 路由处理函数类型
 */
typedef void (*CAN_RouteHandler_t)(uint8_t source_node, const CAN_Message_t *msg);

/* 路由表条目 */
typedef struct {
    uint16_t         id_mask;    /* ID匹配掩码 */
    uint16_t         id_match;   /* ID匹配值 */
    CAN_RouteHandler_t handler; /* 处理函数 */
} CAN_RouteEntry_t;

/* 路由表 */
#define MAX_ROUTE_ENTRIES   8
static CAN_RouteEntry_t g_route_table[MAX_ROUTE_ENTRIES];
static uint8_t g_route_count = 0;

/**
 * @brief 注册路由处理函数
 * @param id_mask 匹配掩码
 * @param id_match 匹配值
 * @param handler 处理函数
 * @return true=注册成功
 */
static bool can_register_route(uint16_t id_mask, uint16_t id_match, 
                                CAN_RouteHandler_t handler)
{
    if (g_route_count >= MAX_ROUTE_ENTRIES) return false;
    
    g_route_table[g_route_count].id_mask = id_mask;
    g_route_table[g_route_count].id_match = id_match;
    g_route_table[g_route_count].handler = handler;
    g_route_count++;
    return true;
}

/**
 * @brief 处理心跳消息
 * @param source_node 源节点ID
 * @param msg CAN消息
 */
static void handle_heartbeat(uint8_t source_node, const CAN_Message_t *msg)
{
    /* 查找或创建节点记录 */
    for (uint8_t i = 0; i < g_remote_node_count; i++) {
        if (g_remote_nodes[i].node_id == source_node) {
            g_remote_nodes[i].online = true;
            g_remote_nodes[i].last_heartbeat = g_systick_ms;
            return;
        }
    }
    
    /* 新节点，添加到表中 */
    if (g_remote_node_count < MAX_REMOTE_NODES) {
        g_remote_nodes[g_remote_node_count].node_id = source_node;
        g_remote_nodes[g_remote_node_count].online = true;
        g_remote_nodes[g_remote_node_count].last_heartbeat = g_systick_ms;
        g_remote_nodes[g_remote_node_count].rx_count = 0;
        g_remote_nodes[g_remote_node_count].tx_count = 0;
        g_remote_nodes[g_remote_node_count].error_count = 0;
        g_remote_node_count++;
    }
}

/**
 * @brief 处理传感器数据
 * @param source_node 源节点ID
 * @param msg CAN消息
 */
static void handle_sensor_data(uint8_t source_node, const CAN_Message_t *msg)
{
    /* msg->data[0] = 传感器类型, data[1:2] = 数据值 */
    /* 这里可扩展为实际的传感器数据处理 */
    (void)source_node;
    (void)msg;
    
    /* 更新节点统计 */
    for (uint8_t i = 0; i < g_remote_node_count; i++) {
        if (g_remote_nodes[i].node_id == source_node) {
            g_remote_nodes[i].rx_count++;
            break;
        }
    }
}

/**
 * @brief 处理命令消息
 * @param source_node 源节点ID
 * @param msg CAN消息
 */
static void handle_command(uint8_t source_node, const CAN_Message_t *msg)
{
    uint8_t cmd = msg->data[0];
    
    switch (cmd) {
        case 0x01: /* 查询状态命令 */
            /* 回复本节点状态 */
            {
                CAN_Message_t ack;
                ack.id = CAN_ID_SENSOR_DATA + LOCAL_NODE_ID;
                ack.extended = false;
                ack.remote = false;
                ack.dlc = 4;
                ack.data[0] = 0x01; /* 状态回复 */
                ack.data[1] = g_remote_node_count;
                ack.data[2] = (uint8_t)(g_local_tx_count & 0xFF);
                ack.data[3] = (uint8_t)(g_local_rx_count & 0xFF);
                mcp2515_send_message(&ack);
            }
            break;
            
        case 0x02: /* 重置统计 */
            g_local_tx_count = 0;
            g_local_rx_count = 0;
            g_local_err_count = 0;
            break;
            
        default:
            break;
    }
}

/**
 * @brief 发送心跳消息
 */
static void can_send_heartbeat(void)
{
    CAN_Message_t msg;
    msg.id = CAN_ID_HEARTBEAT + LOCAL_NODE_ID;
    msg.extended = false;
    msg.remote = false;
    msg.dlc = 2;
    msg.data[0] = LOCAL_NODE_ID;
    msg.data[1] = g_remote_node_count; /* 报告已知节点数 */
    
    if (mcp2515_send_message(&msg)) {
        g_local_tx_count++;
    }
}

/**
 * @brief 发送传感器数据
 * @param sensor_type 传感器类型
 * @param value 数据值
 */
static void can_send_sensor_data(uint8_t sensor_type, uint16_t value)
{
    CAN_Message_t msg;
    msg.id = CAN_ID_SENSOR_DATA + LOCAL_NODE_ID;
    msg.extended = false;
    msg.remote = false;
    msg.dlc = 3;
    msg.data[0] = sensor_type;
    msg.data[1] = (uint8_t)(value >> 8);
    msg.data[2] = (uint8_t)(value & 0xFF);
    
    if (mcp2515_send_message(&msg)) {
        g_local_tx_count++;
    }
}

/**
 * @brief 向指定节点发送命令
 * @param target_id 目标节点ID
 * @param cmd 命令字
 * @param param 命令参数
 * @param param_len 参数长度
 */
static void can_send_command(uint8_t target_id, uint8_t cmd, 
                              const uint8_t *param, uint8_t param_len)
{
    CAN_Message_t msg;
    msg.id = CAN_ID_CMD + target_id;
    msg.extended = false;
    msg.remote = false;
    msg.dlc = 1 + param_len;
    if (msg.dlc > 8) msg.dlc = 8;
    msg.data[0] = cmd;
    for (uint8_t i = 0; i < param_len && i < 7; i++) {
        msg.data[1 + i] = param[i];
    }
    
    if (mcp2515_send_message(&msg)) {
        g_local_tx_count++;
    }
}

/**
 * @brief 检查节点在线状态 (心跳超时检测)
 */
static void can_check_node_status(void)
{
    for (uint8_t i = 0; i < g_remote_node_count; i++) {
        if (g_remote_nodes[i].online) {
            /* 超过3个心跳周期未收到心跳，标记为离线 */
            if ((g_systick_ms - g_remote_nodes[i].last_heartbeat) > 
                (HEARTBEAT_INTERVAL * 3)) {
                g_remote_nodes[i].online = false;
            }
        }
    }
}

/**
 * @brief CAN接收消息处理主循环
 */
static void can_process_rx(void)
{
    uint8_t int_flags = mcp2515_check_receive();
    
    if (int_flags & MCP2515_INT_RX0IF) {
        CAN_Message_t msg;
        if (mcp2515_receive_message(0, &msg)) {
            g_local_rx_count++;
            
            /* 提取源节点ID */
            uint8_t source_node = 0;
            if (msg.id >= CAN_ID_HEARTBEAT && msg.id < CAN_ID_HEARTBEAT + 0x100) {
                source_node = msg.id - CAN_ID_HEARTBEAT;
            } else if (msg.id >= CAN_ID_SENSOR_DATA && msg.id < CAN_ID_SENSOR_DATA + 0x100) {
                source_node = msg.id - CAN_ID_SENSOR_DATA;
            }
            
            /* 路由分发 */
            for (uint8_t i = 0; i < g_route_count; i++) {
                if ((msg.id & g_route_table[i].id_mask) == g_route_table[i].id_match) {
                    if (g_route_table[i].handler) {
                        g_route_table[i].handler(source_node, &msg);
                    }
                    break;
                }
            }
        }
        mcp2515_clear_rx_flag(MCP2515_INT_RX0IF);
    }
    
    if (int_flags & MCP2515_INT_RX1IF) {
        CAN_Message_t msg;
        if (mcp2515_receive_message(1, &msg)) {
            g_local_rx_count++;
            
            uint8_t source_node = 0;
            if (msg.id >= CAN_ID_HEARTBEAT && msg.id < CAN_ID_HEARTBEAT + 0x100) {
                source_node = msg.id - CAN_ID_HEARTBEAT;
            }
            
            for (uint8_t i = 0; i < g_route_count; i++) {
                if ((msg.id & g_route_table[i].id_mask) == g_route_table[i].id_match) {
                    if (g_route_table[i].handler) {
                        g_route_table[i].handler(source_node, &msg);
                    }
                    break;
                }
            }
        }
        mcp2515_clear_rx_flag(MCP2515_INT_RX1IF);
    }
    
    /* 检查错误中断 */
    uint8_t err_flag = mcp2515_read_reg(MCP2515_REG_CANINTF) & MCP2515_INT_ERRIF;
    if (err_flag) {
        g_local_err_count++;
        mcp2515_bit_modify(MCP2515_REG_CANINTF, MCP2515_INT_ERRIF, 0x00);
    }
}

/* ============================================================
 * 第六部分：MCP2515外部中断处理
 * ============================================================ */

/**
 * @brief GROUP1中断处理 (PB2 - MCP2515 INT引脚)
 *        当MCP2515收到消息时触发
 */
void GROUP1_IRQHandler(void)
{
    uint32_t gpio_flags = DL_GPIO_getEnabledInterruptStatus(GPIOB, MCP2515_RST_PIN);
    
    if (gpio_flags & MCP2515_RST_PIN) {
        g_mcp2515_int_flag = true;
        DL_GPIO_clearInterruptStatus(GPIOB, MCP2515_RST_PIN);
    }
}

/* ============================================================
 * 第七部分：主函数
 * ============================================================ */

int main(void)
{
    /* 系统初始化 (由SysConfig生成) */
    SYSCFG_DL_init();
    
    /* 配置MCP2515 CS引脚为输出，初始高电平 */
    DL_GPIO_initDigitalOutput(MCP2515_CS_PIN);
    DL_GPIO_setPins(MCP2515_CS_PORT, MCP2515_CS_PIN);
    
    /* 配置MCP2515 RST引脚为输出 */
    DL_GPIO_initDigitalOutput(MCP2515_RST_PIN);
    DL_GPIO_setPins(MCP2515_RST_PORT, MCP2515_RST_PIN);
    
    /* 初始化MCP2515 */
    if (!mcp2515_init(CAN_BAUDRATE_500K)) {
        /* MCP2515初始化失败，LED快闪指示 */
        while (1) {
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14); /* 板载LED */
            delay_ms(100);
        }
    }
    
    /* 注册路由处理函数 */
    /* 心跳消息路由 */
    can_register_route(0xFF00, CAN_ID_HEARTBEAT, handle_heartbeat);
    /* 传感器数据路由 */
    can_register_route(0xFF00, CAN_ID_SENSOR_DATA, handle_sensor_data);
    /* 命令消息路由 (只处理发给本节点的) */
    can_register_route(0xFF00, CAN_ID_CMD + LOCAL_NODE_ID, handle_command);
    /* 广播命令路由 */
    can_register_route(0xFFFF, CAN_ID_CMD + CAN_ID_BROADCAST, handle_command);
    
    /* 变量初始化 */
    uint32_t last_heartbeat_time = 0;
    uint32_t last_status_check = 0;
    
    /* LED指示初始化完成 */
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14);
    delay_ms(500);
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14);
    
    /* ==================== 主循环 ==================== */
    while (1) {
        /* 1. 处理接收到的CAN消息 */
        can_process_rx();
        
        /* 2. 定时发送心跳 */
        if ((g_systick_ms - last_heartbeat_time) >= HEARTBEAT_INTERVAL) {
            last_heartbeat_time = g_systick_ms;
            can_send_heartbeat();
            
            /* 心跳时LED闪烁 */
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_14);
        }
        
        /* 3. 定时检查节点状态 */
        if ((g_systick_ms - last_status_check) >= 2000) {
            last_status_check = g_systick_ms;
            can_check_node_status();
        }
        
        /* 4. 示例：定期发送传感器数据 (模拟温度传感器) */
        static uint32_t last_sensor_time = 0;
        if ((g_systick_ms - last_sensor_time) >= 500) {
            last_sensor_time = g_systick_ms;
            uint16_t temperature = 2500; /* 25.00°C，单位0.01°C */
            can_send_sensor_data(0x01, temperature);
        }
        
        /* 5. 模拟系统时间递增 (实际应使用SysTick定时器) */
        delay_ms(1);
        g_systick_ms++;
    }
}
