/**
 * @file    Motor.cpp
 * @brief   摩擦轮电机（穿越机电机）驱动
 *
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Motor_Fir.h"

/* 全局变量创建 --------------------------------------------------------------------------------------------------------*/


/* 函数定义 ------------------------------------------------------------------------------------------------------------*/
/************************************************************************************************************************
 * @brief   摩擦轮电机初始化函数
 *
 * @param   __TIM_PWM           电机绑定 TIM-PWM 句柄指针
 * @param   __PWM_Channel       电机绑定 PWM输出通道
 * @param   __Omega_MAX         电机输出轴速度最大值 (rad/s)
 * @param   __Duty_Circle_MAX   电调最大占空比
 * @param   __Duty_Circle_MIN   电调最小占空比
 * @param   __Control_Cycle     电机控制周期 (控制周期 = __Control_Cycle * 系统心跳周期)
 ***********************************************************************************************************************/
void Class_Motor_Fir::Init(TIM_HandleTypeDef *__TIM_PWM, uint32_t __PWM_Channel, uint32_t __PWM_CCR,
                                uint16_t __Duty_Circle_MAX, uint16_t __Duty_Circle_MIN, uint16_t __Control_Cycle)
{
    /*参数赋值*/
    this->TIM_PWM = __TIM_PWM;
    this->PWM_Channel = __PWM_Channel;
    this->Duty_Circle_MAX = __Duty_Circle_MAX;
    this->Duty_Circle_MIN = __Duty_Circle_MIN;
    this->Control_Cycle = __Control_Cycle;

    /*定时器启动*/
    HAL_TIM_PWM_Start(this->TIM_PWM, this->PWM_Channel);
}

