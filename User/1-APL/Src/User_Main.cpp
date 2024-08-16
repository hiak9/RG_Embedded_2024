/**
 * @file    User_Main.cpp
 * @brief   初始化，主循环函数C语言接口
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "User_Main.h"
#include "Chassis.h"
#include "Communication.h"
#include "Board.h"
#include "Motor.h"
//#include "Motor_DJI.h"
#include "User_Can.h"
#include "User_Delay.h"
#include "tim.h"
#include "User_Math.h"

/************************************************************************************************************************
 * @brief   初始化函数封装
 ***********************************************************************************************************************/
void User_setup(void)
{
    /* 用户延时初始化 */
    Delay_Init(168U);

    /* 使能CAN外设 */
    CAN_Init(&CAN1_Manage_Object);
    
    /* 测试Servo电机 */
    Motor_Test_Servo.Init(&htim9, TIM_CHANNEL_1, 180.0f);

    /* 摩擦轮初始化 */
//    frictiongear[0].PID_Omega.Init(2.4f, 6.1f, 0.0f, 0.0f, 12.0f, 20.0f);
//    frictiongear[1].PID_Omega.Init(2.4f, 6.1f, 0.0f, 0.0f, 12.0f, 20.0f);

//    frictiongear[0].Init(&CAN1_Manage_Object, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);
//    frictiongear[1].Init(&CAN1_Manage_Object, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_OMEGA);

    /* 麦轮底盘初始化 */
    Committee_Chariot.Init();

    /* 串口初始化 */
    COM_LuBanCat.Init(22U, 19U);

    /* 使能系统心跳定时器 */
    HAL_TIM_Base_Start_IT(&htim6);
}

/************************************************************************************************************************
 * @brief   主循环函数封装
 ***********************************************************************************************************************/
void User_loop(void)
{
    // frictiongear[0].Set_Target_Omega(-20.0f);
    // frictiongear[1].Set_Target_Omega(20.0f);
    Committee_Chariot.Enable();
    Committee_Chariot.Set_Motion(0.1, 0, 0);
    HAL_Delay(1000);
    Committee_Chariot.Set_Motion(0, 0.1, 0);
    HAL_Delay(1000);
    Committee_Chariot.Set_Motion(-0.1, 0, 0);
    HAL_Delay(1000);
    Committee_Chariot.Set_Motion(0, -0.1, 0);
    HAL_Delay(1000);
    Committee_Chariot.Set_Motion(0, 0, 0.1f * PI);
    HAL_Delay(1000);
}
