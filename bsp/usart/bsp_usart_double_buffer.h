#ifndef __BSP_USART_DOUBLE_BUFFER_H
#define __BSP_USART_DOUBLE_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "stdint.h"

#define DEVICE_USART_CNT 8     // MC02串口数量8个，其中
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里

typedef void (*usart_module_callback)(); // 模块回调函数,用于解析协议

typedef struct
{
    uint8_t buff[2][USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USARTInstance;

typedef struct
{
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USART_Init_Config_s;	   		 

#endif // !__BSP_USART_DOUBLE_BUFFER_H