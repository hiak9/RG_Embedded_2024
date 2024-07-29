/**
 * @file    Callback_Can.cpp
 * @brief   CAN回调函数重写
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Callback_Can.h"

/* 函数定义 ------------------------------------------------------------------------------------------------------------*/
/************************************************************************************************************************
 * @brief   CAN-RX0中断回调函数重写
 *
 * @param   hcan    CAN外设句柄
 ***********************************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
//    if(hcan->Instance == CAN1)
//    {
//        CAN_Receive_Data(&CAN1_Manage_Object, CAN_FILTER_FIFO0);

//        switch(CAN1_Manage_Object.Rx_Buffer.Header.StdId)
//        {
//            case (DJI_Motor_ID_0x201):
//            {
//                frictiongear[0].DataGet();
//                break;
//            }
//            case (DJI_Motor_ID_0x205):
//            {
//                frictiongear[1].DataGet();
//                break;
//            }
//        }
//    }
}

/************************************************************************************************************************
 * @brief   CAN-RX1中断回调函数重写
 *
 * @param   hcan    CAN外设句柄
 ***********************************************************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
//    if(hcan->Instance == CAN1)
//    {
//        CAN_Receive_Data(&CAN1_Manage_Object, CAN_FILTER_FIFO1);

//        switch(CAN1_Manage_Object.Rx_Buffer.Header.StdId)
//        {
//            case (DJI_Motor_ID_0x201):
//            {
//                frictiongear[0].DataGet();
//                break;
//            }
//            case (DJI_Motor_ID_0x205):
//            {
//                frictiongear[1].DataGet();
//                break;
//            }
//        }
//    }
}
