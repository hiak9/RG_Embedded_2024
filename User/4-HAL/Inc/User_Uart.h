/**
 * @file    User_Uart.h
 * @brief   Uart外设进一步封装
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

#ifndef __HAL_USER_UART_H
#define __HAL_USER_UART_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usart.h"

#include "Chassis.h"

/* 宏定义 -------------------------------------------------------------------------------------------------------------*/
#define UART_TX_BUFFER_SIZE            256         // 串口TX缓冲区字节长度
#define UART_RX_BUFFER_SIZE            256         // 串口RX缓冲区字节长度

/* 结构体定义 ----------------------------------------------------------------------------------------------------------*/
/**
 * @brief UART处理结构体
 */
struct Struct_UART_Manage_Object
{
    UART_HandleTypeDef * huart;
    uint8_t Tx_Buffer[UART_TX_BUFFER_SIZE];
    uint8_t Rx_Buffer[UART_RX_BUFFER_SIZE];
    uint16_t Rx_Data_Size;
};

/* 变量声明 ------------------------------------------------------------------------------------------------------------*/
extern Struct_UART_Manage_Object UART1_Manage_Object;
extern Struct_UART_Manage_Object UART3_Manage_Object;

/* 函数声明 ------------------------------------------------------------------------------------------------------------*/
void UART_Init(Struct_UART_Manage_Object * UART_Mangae_Obj, uint16_t Rx_Data_Size);
HAL_StatusTypeDef UART_Send(Struct_UART_Manage_Object * UART_Mangae_Obj, uint8_t * Data, uint16_t Length);
HAL_StatusTypeDef UART_ReceiveToIdle_DMA(Struct_UART_Manage_Object * UART_Mangae_Obj);

#endif  /* HAL_User_Uart.h */
