/**
 * @file    Motor_Fir.h
 * @brief   摩擦轮电机（穿越机电机）控制
 *
 */

#ifndef __HDL_MOTOR_FIR_H
#define __HDL_MOTOR_FIR_H

/* 头文件包含 ----------------------------------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 枚举变量定义 --------------------------------------------------------------------------------------------------------*/
/**
 * @brief   穿越机电机枚举变量
 */
enum Enum_Motor_Fir_Status
{
	Motor_Fir_DISABLE = 0,
	Motor_Fir_ENABLE,
};


/* 类定义 -------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   穿越机类定义
 */
class Class_Motor_Fir
{
public:
	void Init(TIM_HandleTypeDef *__TIM_PWM, uint32_t __PWM_Channel, uint32_t __PWM_CCR = 0,
				uint16_t __Duty_Circle_MAX = 1900U, uint16_t __Duty_Circle_MIN = 1100U, 
				uint16_t __Control_Cycle = 50U);
	void Control();

	inline void Set_Speed(uint16_t __PWM_CCR);
	inline void test_Set_Speed(uint16_t __PWM_CCR);
protected:

	float Speed_MAX = 43512;									/*!< 电机最大速度（rad/s）*/
	float Speed_MIN = 0;										/*!< 电机最小速度（rad/s）*/
	uint16_t Duty_Circle_MAX;									/*!< 电调最大占空比*/
	uint16_t Duty_Circle_MIN;									/*!< 电调最小占空比*/
	uint32_t PWM_CCR;											/*!< 输出比较寄存器的值*/

	TIM_HandleTypeDef *TIM_PWM;									/*!< TIM_PWM句柄*/
	uint32_t PWM_Channel;										/*!< TIM_PWM通道*/
	uint16_t Control_Cycle;    									/*!< 电机控制周期 (控制周期 = Control_Cycle * 系统心跳周期) */
	uint16_t Cycle_Counter = 0U;								/*!< 电机控制周期计数器 */
	Enum_Motor_Fir_Status Motor_Fir_Status 
										= Motor_Fir_DISABLE;	/*!< 电机当前状态*/

	
};

/* 变量声明 ------------------------------------------------------------------------------------------------------------*/
extern Class_Motor_Fir friction_gear_up[2];
extern Class_Motor_Fir friction_gear_down[2];


/* 函数声明 ------------------------------------------------------------------------------------------------------------*/

/************************************************************************************************************************
 * @brief   摩擦轮电机设置速度
 *
 * @param   __PWM_CCR           PWM波占空比
 ***********************************************************************************************************************/
void Class_Motor_Fir::Set_Speed(uint16_t __PWM_CCR)
{
	if(this->Motor_Fir_Status != Motor_Fir_ENABLE)
	{
		this->Motor_Fir_Status = Motor_Fir_ENABLE;
	}
	/*!< 判断占空比大小，避免超出范围*/
	if(__PWM_CCR < this->Duty_Circle_MIN){
		this->PWM_CCR = this->Duty_Circle_MIN;
	}
	else if(__PWM_CCR > this->Duty_Circle_MAX){
		this->PWM_CCR = this->Duty_Circle_MAX;
	}
	else{
		this->PWM_CCR = __PWM_CCR;
	}
	__HAL_TIM_SET_COMPARE(this->TIM_PWM, this->PWM_Channel, this->PWM_CCR);
}
#endif  /* HDL_Motor_Fir.h */
