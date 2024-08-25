/**
 * @file    Callback_Tim.cpp
 * @brief   TIM回调函数重写
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Callback_Tim.h"

/************************************************************************************************************************
 * @brief   TIM更新中断回调函数重写
 *
 * @param   htim    TIM外设句柄
 ***********************************************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
    if (htim->Instance == htim6.Instance)
    {
        static uint16_t count;

        /* 串口离线检测，10Hz */
        COM_LuBanCat.AliveCheck(100);

        /* 摩擦轮离线检测，10Hz */
//        frictiongear[0].AliveCheck(100);
//        frictiongear[1].AliveCheck(100);
        friction_gear_up[0].Control();
        friction_gear_up[1].Control();
        
        friction_gear_down[0].Control();
        friction_gear_down[1].Control();

        /* 底盘控制 */
        Committee_Chariot.Control();

        /* 电机闭环控制 */
       /*  Committee_Chariot.Motor_Wheel[0].Control();
        Committee_Chariot.Motor_Wheel[1].Control();
        Committee_Chariot.Motor_Wheel[2].Control();
        Committee_Chariot.Motor_Wheel[3].Control(); */

        Committee_Chariot.Motor_Wheel[0].Control_test();
        Committee_Chariot.Motor_Wheel[1].Control_test();
        Committee_Chariot.Motor_Wheel[2].Control_test();
        Committee_Chariot.Motor_Wheel[3].Control_test();


        /* 摩擦轮闭环控制 */
//        frictiongear[0].Control();
//        frictiongear[1].Control();

        /* 大疆电机CAN数据发送 */
        //DJI_CAN_SendData();

        /* 数据上传，50Hz */
        if (count < 19U)
        {
            count += 1;
        }
        else
        {
            count = 0;
            COM_LuBanCat.DataSend(0x00);
        }
    }
}
