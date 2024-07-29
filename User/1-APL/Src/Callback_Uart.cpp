/**
 * @file    Callback_Uart.cpp
 * @brief   UART回调函数重写
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Callback_Uart.h"

/* 函数定义 -----------------------------------------------------------------------------------------------------------*/
/***********************************************************************************************************************
 * @brief   UART-RX事件中断回调函数重写
 *
 * @param   huart   UART外设句柄
 * @param   Size    当前接收长度
 **********************************************************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if (huart->Instance == huart3.Instance)
    {
        /* 鲁班猫上位机串口数据处理 */
        COM_LuBanCat.DataProcess(Size);

        /* 开启新一次串口接收（DMA-IDLE） */
        UART_ReceiveToIdle_DMA(&UART3_Manage_Object);
    }
}

/***********************************************************************************************************************
 * @brief   UART错误回调函数重写
 *
 * @param   huart   UART外设句柄
 **********************************************************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if (HAL_UART_GetError(huart) & HAL_UART_ERROR_PE)
    {
        // 奇偶校验错误
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
    else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_NE)
    {
        // 噪声错误
        __HAL_UART_CLEAR_NEFLAG(huart);
    }
    else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_FE)
    {
        // 帧格式错误
        __HAL_UART_CLEAR_FEFLAG(huart);
    }
    else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
    {
        // 数据太多串口来不及接收错
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

    // 当这个串口发生了错误，一定要在重新使能接收中断
    if (huart->Instance == huart3.Instance)
    {
        /* 开启新一次串口接收（DMA-IDLE） */
        UART_ReceiveToIdle_DMA(&UART3_Manage_Object);
    }
}
