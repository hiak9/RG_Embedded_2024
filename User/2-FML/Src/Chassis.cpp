/**
 * @file    Chassis.cpp
 * @brief   底盘功能模块
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Chassis.h"

/* 全局变量创建 --------------------------------------------------------------------------------------------------------*/
Class_Chassis_Macnum Committee_Chariot;

/* 函数定义 ------------------------------------------------------------------------------------------------------------*/
/************************************************************************************************************************
 * @brief   麦轮底盘初始化函数
 *
 * @param   __Control_Cycle     底盘控制周期 (控制周期 = __Control_Cycle * 系统心跳周期)
 ***********************************************************************************************************************/
void Class_Chassis_Macnum::Init(float __Wheel_Omega_MAX, uint16_t __Control_Cycle)
{
    /* 参数赋值 */
    this->Wheel_Omega_MAX = __Wheel_Omega_MAX;
    this->Control_Cycle = __Control_Cycle;

    /* 电机初始化 */
    this->Motor_Wheel[0].Init(&htim2, &htim8, TIM_CHANNEL_1, GPIOC, GPIOA, GPIO_PIN_12, GPIO_PIN_8 , 20.0f, 14.0f);
    this->Motor_Wheel[1].Init(&htim3, &htim8, TIM_CHANNEL_2, GPIOD, GPIOD, GPIO_PIN_10, GPIO_PIN_12, 20.0f, 14.0f);
    this->Motor_Wheel[2].Init(&htim4, &htim8, TIM_CHANNEL_3, GPIOC, GPIOB, GPIO_PIN_0 , GPIO_PIN_14, 20.0f, 14.0f);
    this->Motor_Wheel[3].Init(&htim5, &htim8, TIM_CHANNEL_4, GPIOB, GPIOB, GPIO_PIN_13, GPIO_PIN_12, 20.0f, 14.0f);

    for (uint8_t i = 0; i < 4; i++)
    {
        this->Motor_Wheel[i].PID_Omega.Init(0.1f, 5.0f, 0.0f, 0.0f, 10.0f, 20.0f, 0.05f);
        this->Motor_Wheel[i].Gear_Slope.Init(5.0f);
    }

    /* 初始化完成，底盘使能 */
    this->Enable();
}

/************************************************************************************************************************
 * @brief   麦轮底盘控制函数（需在系统心跳定时器更新中断中执行）
 ***********************************************************************************************************************/
void Class_Chassis_Macnum::Control()
{
    /* 判断是否到达控制周期 */
    if (this->Cycle_Counter < this->Control_Cycle - 1)
    {
        /* 未到达控制周期 */
        this->Cycle_Counter += 1;
    }
    else
    {
        /* 到达控制周期，进行底盘控制 */
        this->Cycle_Counter = 0;

        if (this->Chassis_State == Chassis_Disable || this->Chassis_State == Chassis_Suspend)
        {
            /* 速度清零 */
            this->Velocity_X = 0.0f;
            this->Velocity_Y = 0.0f;
            this->Omega = 0.0f;

            /* 电机设置为悬空 */
            for (uint8_t i = 0; i < 4; i++)
            {
                this->Motor_Wheel[i].StopSet(Motor_Suspend);
            }
        }
        else if (this->Chassis_State == Chassis_Brake)
        {
            /* 速度清零 */
            this->Velocity_X = 0.0f;
            this->Velocity_Y = 0.0f;
            this->Omega = 0.0f;

            /* 电机设置为刹车 */
            for (uint8_t i = 0; i < 4; i++)
            {
                this->Motor_Wheel[i].StopSet(Motor_Brake);
            }
        }
        else if (this->Chassis_State == Chassis_Run)
        {
            float Wheel_Omega_Preliminary[4];
            float Wheel_Omega_Max = 0.0f;
            float Limit_Factor = 1.0f;

            /* 四轮角速度初步解算 (rad/s) */
            Wheel_Omega_Preliminary[0] =
            (this->Velocity_X - this->Velocity_Y - this->Omega * (this->Wheel_Spacing + this->Wheel_Base) / 2) / this->Wheel_Radius;
            Wheel_Omega_Preliminary[1] =
            -(this->Velocity_X + this->Velocity_Y + this->Omega * (this->Wheel_Spacing + this->Wheel_Base) / 2) / this->Wheel_Radius;
            Wheel_Omega_Preliminary[2] =
            (this->Velocity_X + this->Velocity_Y - this->Omega * (this->Wheel_Spacing + this->Wheel_Base) / 2) / this->Wheel_Radius;
            Wheel_Omega_Preliminary[3] =
            -(this->Velocity_X - this->Velocity_Y + this->Omega * (this->Wheel_Spacing + this->Wheel_Base) / 2) / this->Wheel_Radius;

            /* 轮速限幅因子计算 */
            for (uint8_t i = 0; i < 4; i++)
            {
                if (Wheel_Omega_Max < Math_Abs(Wheel_Omega_Preliminary[i]))
                {
                    Wheel_Omega_Max = Math_Abs(Wheel_Omega_Preliminary[i]);
                }
            }
            if (Wheel_Omega_Max > this->Wheel_Omega_MAX)
            {
                Limit_Factor = this->Wheel_Omega_MAX / Wheel_Omega_Max;
            }

            /* 电机转速赋值 */
            for (uint8_t i = 0; i < 4; i++)
            {
                this->Motor_Wheel[i].MotionSet(Wheel_Omega_Preliminary[i] * Limit_Factor);
            }
        }
    }
}
