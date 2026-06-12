/**
 * @file    vision.c
 * @brief   OpenMV视觉通信模块实现
 * 
 * 通信协议：
 * OpenMV通过UART发送目标坐标数据
 * 格式："$X,Y\n" 或 "$X,Y,AREA\n"
 * 其中X为目标水平偏移量，Y为垂直偏移量
 * 坐标原点为画面中心
 * 
 * OpenMV端运行颜色追踪或AprilTag识别程序
 */

#include "vision.h"
#include "msp.h"
#include <string.h>
#include <stdlib.h>

/* 接收缓冲区 */
#define RX_BUF_SIZE     64
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_index = 0;
static uint8_t rx_complete = 0;

/* 解析后的数据 */
static VisionData_t vision_data = {0, 0, 0, 0};

/**
 * @brief  视觉模块初始化
 * @param  无
 * @retval 无
 */
void Vision_Init(void)
{
    rx_index = 0;
    rx_complete = 0;
    vision_data.valid = 0;
    memset(rx_buf, 0, RX_BUF_SIZE);
    
    /* UART1已在main.c中初始化 */
}

/**
 * @brief  UART接收回调函数（在中断中调用）
 * @param  ch: 接收到的字节
 * @retval 无
 * 
 * 协议解析：
 * 1. 等待'$'起始字符
 * 2. 接收数据直到'\n'
 * 3. 解析X,Y坐标
 */
void Vision_RxCallback(uint8_t ch)
{
    if(ch == '$')
    {
        /* 起始字符，重置缓冲区 */
        rx_index = 0;
        rx_buf[0] = '\0';
    }
    else if(ch == '\n')
    {
        /* 结束字符，标记接收完成 */
        rx_buf[rx_index] = '\0';
        rx_complete = 1;
    }
    else
    {
        /* 数据字符 */
        if(rx_index < RX_BUF_SIZE - 1)
        {
            rx_buf[rx_index++] = ch;
        }
    }
}

/**
 * @brief  获取视觉数据
 * @param  data: 输出数据结构指针
 * @retval uint8_t: 1=有新数据，0=无新数据
 * 
 * 解析逻辑：
 * 1. 检查接收完成标志
 * 2. 查找逗号分隔符
 * 3. 解析X和Y坐标
 * 4. 清除完成标志
 */
uint8_t Vision_GetData(VisionData_t *data)
{
    if(!rx_complete)
    {
        data->valid = 0;
        return 0;
    }
    
    /* 查找逗号位置 */
    char *comma1 = strchr(rx_buf, ',');
    if(comma1 != NULL)
    {
        /* 解析X坐标 */
        *comma1 = '\0';
        vision_data.dx = (float)atof(rx_buf);
        
        /* 解析Y坐标 */
        char *comma2 = strchr(comma1 + 1, ',');
        if(comma2 != NULL)
        {
            /* 有面积数据 */
            *comma2 = '\0';
            vision_data.dy = (float)atof(comma1 + 1);
            vision_data.area = (uint16_t)atoi(comma2 + 1);
        }
        else
        {
            /* 无面积数据 */
            vision_data.dy = (float)atof(comma1 + 1);
            vision_data.area = 0;
        }
        
        vision_data.valid = 1;
    }
    
    /* 清除完成标志 */
    rx_complete = 0;
    
    /* 复制数据 */
    *data = vision_data;
    
    return data->valid;
}
