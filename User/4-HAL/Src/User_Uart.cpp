/**
 * @file    User_Uart.cpp
 * @brief   Uart外设进一步封装
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

/* 头文件引用 ---------------------------------------------------------------------------------------------------------*/
#include "User_Uart.h"

/* 全局变量 -----------------------------------------------------------------------------------------------------------*/
Struct_UART_Manage_Object UART1_Manage_Object = {&huart1};
Struct_UART_Manage_Object UART3_Manage_Object = {&huart3};

/* 函数定义 -----------------------------------------------------------------------------------------------------------*/
/***********************************************************************************************************************
 * @brief UART初始化
 *
 * @param UART_Mangae_Obj      UART处理结构体指针
 **********************************************************************************************************************/
void UART_Init(Struct_UART_Manage_Object * UART_Mangae_Obj, uint16_t Rx_Data_Size)
{
    /* RX接收数据长度赋值 */
    UART_Mangae_Obj->Rx_Data_Size = Rx_Data_Size;
    
    /* 开启串口接收（DMA-IDLE） */
    HAL_UARTEx_ReceiveToIdle_DMA(UART_Mangae_Obj->huart, UART_Mangae_Obj->Rx_Buffer, UART_Mangae_Obj->Rx_Data_Size);
	/* 失能DMA半满中断 */
    __HAL_DMA_DISABLE_IT(UART_Mangae_Obj->huart->hdmarx, DMA_IT_HT);
}

/***********************************************************************************************************************
 * @brief   UART发送数据
 *
 * @param   UART_Manage_Obj     UART处理结构体指针
 * @param   Data                数据
 * @param   Length              数据长度
 * @return  uint8_t             执行结果
 **********************************************************************************************************************/
HAL_StatusTypeDef UART_Send(Struct_UART_Manage_Object * UART_Mangae_Obj, uint8_t * Data, uint16_t Length)
{
    return (HAL_UART_Transmit_DMA(UART_Mangae_Obj->huart, Data, Length));
}

/***********************************************************************************************************************
 * @brief   UART开启一次DMA-IDLE接收
 *
 * @param   UART_Manage_Obj     UART处理结构体指针
 **********************************************************************************************************************/
HAL_StatusTypeDef UART_ReceiveToIdle_DMA(Struct_UART_Manage_Object * UART_Mangae_Obj)
{	
	/* 开启一次DMA-IDLE接收 */
    auto hal_status = HAL_UARTEx_ReceiveToIdle_DMA(UART_Mangae_Obj->huart, UART_Mangae_Obj->Rx_Buffer, UART_Mangae_Obj->Rx_Data_Size);
	/* 失能DMA半满中断 */
    __HAL_DMA_DISABLE_IT(UART_Mangae_Obj->huart->hdmarx, DMA_IT_HT);
	
	return hal_status;
}
