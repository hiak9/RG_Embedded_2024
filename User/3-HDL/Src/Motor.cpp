/**
 * @file    Motor.cpp
 * @brief   电机驱动
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Motor.h"

/* 全局变量创建 --------------------------------------------------------------------------------------------------------*/
Class_Motor_Servo Motor_Test_Servo;

/* 函数定义 ------------------------------------------------------------------------------------------------------------*/
/************************************************************************************************************************
 * @brief   BDC电机初始化函数
 *
 * @param   __TIM_Encoder       电机绑定 TIM-编码器 句柄指针
 * @param   __TIM_PWM           电机绑定 TIM-PWM 句柄指针
 * @param   __PWM_Channel       电机绑定 PWM输出通道
 * @param   __GPIO_Direction_A  电机绑定方向引脚A
 * @param   __GPIO_Direction_B  电机绑定方向引脚B
 * @param   __Omega_MAX         电机输出轴速度最大值 (rad/s)
 * @param   __Reduction_Ratio   电机减速比
 * @param   __Encoder_Lines     电机编码器线数
 * @param   __Control_Cycle     电机控制周期 (控制周期 = __Control_Cycle * 系统心跳周期)
 ***********************************************************************************************************************/
void Class_Motor_BDC::Init(TIM_HandleTypeDef * __TIM_Encoder, TIM_HandleTypeDef * __TIM_PWM, uint32_t __PWM_Channel,
                           GPIO_TypeDef * __GPIOx_Dir_A, GPIO_TypeDef * __GPIOx_Dir_B,
                           uint32_t __GPIO_Pin_Dir_A, uint32_t __GPIO_Pin_Dir_B,
                           float __Omega_MAX, float __Reduction_Ratio, uint16_t __Encoder_Lines, uint16_t __Control_Cycle)
{
    /* 参数赋值 */
    this->TIM_Encoder = __TIM_Encoder;
    this->TIM_PWM = __TIM_PWM;
    this->PWM_Channel = __PWM_Channel;
    this->GPIOx_Dir[0] = __GPIOx_Dir_A;
    this->GPIOx_Dir[1] = __GPIOx_Dir_B;
    this->GPIO_Pin_Dir[0] = __GPIO_Pin_Dir_A;
    this->GPIO_Pin_Dir[1] = __GPIO_Pin_Dir_B;
    this->Omega_MAX = __Omega_MAX;
    this->Reduction_Ratio = __Reduction_Ratio;
    this->Encoder_Lines = __Encoder_Lines;
    this->Control_Cycle = __Control_Cycle;

    /* PID 输出限幅 */
    this->PID_Omega.Set_Out_Max(this->Omega_MAX);

    /* 底层初始化 */
    this->Msp_Init();

    /* 编码器，PWM 定时器启动 */
    HAL_TIM_Encoder_Start(this->TIM_Encoder, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(this->TIM_PWM, this->PWM_Channel);
}

/************************************************************************************************************************
 * @brief   BDC电机底层初始化函数
 ***********************************************************************************************************************/
void Class_Motor_BDC::Msp_Init()
{
    GPIO_InitTypeDef gpio_init = {0};
	
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
	
	/* 电机方向控制引脚初始化 */
	for (uint8_t i = 0; i < 2; i++)
	{
		if (this->GPIOx_Dir[i] == GPIOA)
		{
			__HAL_RCC_GPIOA_CLK_ENABLE();
		}
		else if (this->GPIOx_Dir[i] == GPIOB)
		{
			__HAL_RCC_GPIOB_CLK_ENABLE();
		}
		else if (this->GPIOx_Dir[i] == GPIOC)
		{
			__HAL_RCC_GPIOC_CLK_ENABLE();
		}
		else if (this->GPIOx_Dir[i] == GPIOD)
		{
			__HAL_RCC_GPIOD_CLK_ENABLE();
		}
		else if (this->GPIOx_Dir[i] == GPIOE)
		{
			__HAL_RCC_GPIOE_CLK_ENABLE();
		}
		else if (this->GPIOx_Dir[i] == GPIOF)
		{
			__HAL_RCC_GPIOF_CLK_ENABLE();
		}
		else if (this->GPIOx_Dir[i] == GPIOG)
		{
			__HAL_RCC_GPIOG_CLK_ENABLE();
		}
		
		gpio_init.Pin = this->GPIO_Pin_Dir[i];
		HAL_GPIO_Init(this->GPIOx_Dir[i], &gpio_init);
		
		/* 设置为悬空模式 */
		HAL_GPIO_WritePin(this->GPIOx_Dir[i], this->GPIO_Pin_Dir[i], GPIO_PIN_SET);
	}
}

/************************************************************************************************************************
 * @brief   BDC电机控制函数（需在系统心跳定时器更新中断中执行）
 ***********************************************************************************************************************/
void Class_Motor_BDC::Control()
{
    /* 判断是否到达控制周期 */
    if (this->Cycle_Counter < this->Control_Cycle - 1)
    {
        /* 未到达控制周期 */
        this->Cycle_Counter += 1;
    }
    else
    {
        /* 到达控制周期，进行电机控制 */
        this->Cycle_Counter = 0;

        /* 获取电机实际速度 */
        int16_t Actual_Omega_Raw = __HAL_TIM_GetCounter(this->TIM_Encoder);
        __HAL_TIM_SetCounter(this->TIM_Encoder, 0);
        this->Actual_Omega = (float)Actual_Omega_Raw / 4.0f / this->Encoder_Lines / this->Reduction_Ratio /
                             (this->Control_Cycle * this->Heartbeat_Period) * 1000 * 2 * PI;

        /* 判断电机当前状态 */
        if (this->Motor_State == Motor_Suspend)
        {   
            /* 速度清零 */
            this->Set_Omega = 0.0f;
            this->Target_Omega = 0.0f;
            this->Out_Omega = 0.0f;

            /* PID积分项归零 */
            this->PID_Omega.Set_Integral_Error(0.0f);

            /* 方向引脚输出悬空信号 */
            HAL_GPIO_WritePin(this->GPIOx_Dir[0], this->GPIO_Pin_Dir[0], GPIO_PIN_SET);
            HAL_GPIO_WritePin(this->GPIOx_Dir[1], this->GPIO_Pin_Dir[1], GPIO_PIN_SET);
        }
        else if (this->Motor_State == Motor_Brake)
        {
            /* 速度清零 */
            this->Set_Omega = 0.0f;
            this->Target_Omega = 0.0f;
            this->Out_Omega = 0.0f;

            /* PID积分项归零 */
            this->PID_Omega.Set_Integral_Error(0.0f);

            /* 方向引脚输出刹车信号 */
            HAL_GPIO_WritePin(this->GPIOx_Dir[0], this->GPIO_Pin_Dir[0], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(this->GPIOx_Dir[1], this->GPIO_Pin_Dir[1], GPIO_PIN_RESET);
        }
        else if (this->Motor_State == Motor_Run)
        {   
            /* 梯形变速获得当前目标速度 */
            this->Gear_Slope.Set_Value(this->Set_Omega);
            this->Gear_Slope.Calculate();
            this->Target_Omega = this->Gear_Slope.Get_Out();

            /* PID 计算得到输出值 */
            this->PID_Omega.Set_Target(this->Target_Omega);
            this->PID_Omega.Set_Actual(this->Actual_Omega);
            this->PID_Omega.Calculate();
            this->Out_Omega = this->PID_Omega.Get_Out();

            /* 正反转控制，将输出值映射到PWM输出比较寄存器上 */
            if (this->Out_Omega > 0)
            {
                /* 输出比较寄存器赋值 */
                __HAL_TIM_SET_COMPARE(this->TIM_PWM, this->PWM_Channel,
                                      (uint32_t)(this->Out_Omega / this->Omega_MAX * this->TIM_PWM->Init.Period));
                /* 方向引脚输出正转信号 */
                HAL_GPIO_WritePin(this->GPIOx_Dir[0], this->GPIO_Pin_Dir[0], GPIO_PIN_RESET);
                HAL_GPIO_WritePin(this->GPIOx_Dir[1], this->GPIO_Pin_Dir[1], GPIO_PIN_SET);
            }
            else
            {
                /* 输出比较寄存器赋值 */
                __HAL_TIM_SET_COMPARE(this->TIM_PWM, this->PWM_Channel,
                                      (uint32_t)(-this->Out_Omega / this->Omega_MAX * this->TIM_PWM->Init.Period));
                /* 方向引脚输出反转信号 */
                HAL_GPIO_WritePin(this->GPIOx_Dir[0], this->GPIO_Pin_Dir[0], GPIO_PIN_SET);
                HAL_GPIO_WritePin(this->GPIOx_Dir[1], this->GPIO_Pin_Dir[1], GPIO_PIN_RESET);
            }
        }
    }
}

/*****************************************************************************************************
 * @brief   步进电机初始化函数
*****************************************************************************************************/
void Class_Motor_Step::Init()
{
    /* 参数赋值 */

    /* 底层初始化 */
    this->Msp_Init();
}

/*****************************************************************************************************
 * @brief   步进电机底层初始化函数
*****************************************************************************************************/
void Class_Motor_Step::Msp_Init()
{
	GPIO_InitTypeDef gpio_init = {0};
	
	gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
	gpio_init.Pull = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
	
	/* 电机控制引脚初始化 */
	for (uint8_t i = 0; i < 2; i++)
	{
		if (this->GPIOx[i] == GPIOA)
		{
			__HAL_RCC_GPIOA_CLK_ENABLE();
		}
		else if (this->GPIOx[i] == GPIOB)
		{
			__HAL_RCC_GPIOB_CLK_ENABLE();
		}
		else if (this->GPIOx[i] == GPIOC)
		{
			__HAL_RCC_GPIOC_CLK_ENABLE();
		}
		else if (this->GPIOx[i] == GPIOD)
		{
			__HAL_RCC_GPIOD_CLK_ENABLE();
		}
		else if (this->GPIOx[i] == GPIOE)
		{
			__HAL_RCC_GPIOE_CLK_ENABLE();
		}
		else if (this->GPIOx[i] == GPIOF)
		{
			__HAL_RCC_GPIOF_CLK_ENABLE();
		}
		else if (this->GPIOx[i] == GPIOG)
		{
			__HAL_RCC_GPIOG_CLK_ENABLE();
		}
		
		gpio_init.Pin = this->GPIO_Pin[i];
		HAL_GPIO_Init(this->GPIOx[i], &gpio_init);
		
		/* 引脚默认值设置 */
		HAL_GPIO_WritePin(this->GPIOx[i], this->GPIO_Pin[i], GPIO_PIN_SET);
	}
}

/*****************************************************************************************************
 * @brief   步进电机角速度控制函数
 * 
 * @param   Motor_Omega     设定的电机角速度
*****************************************************************************************************/
void Class_Motor_Step::OmegaControl(float Motor_Omega)
{
    uint16_t Arr_Val, Arr_Last_Val;
	uint8_t flag = 0;
    float Omega_Abs;
	
	/* 电机角速度限幅（等于0代表电机停止） */
	if (Motor_Omega > 0)
	{
		if (Motor_Omega > this->Speed_MAX)
		{
			Motor_Omega = this->Speed_MAX;
		}
		else if (Motor_Omega < this->Speed_MIN)
		{
			Motor_Omega = this->Speed_MIN;
		}
		
		Omega_Abs = Motor_Omega;
	}
	else if (Motor_Omega < 0)
	{
		if (Motor_Omega < -this->Speed_MAX)
		{
			Motor_Omega = -this->Speed_MAX;
		}
		else if (Motor_Omega > -this->Speed_MIN)
		{
			Motor_Omega = -this->Speed_MIN;
		}
		
		Omega_Abs = -Motor_Omega;
	}
	
	/* 角速度值更新 */
	this->Target_Omega_Last = this->Target_Omega_Now;
	this->Target_Omega_Now = Motor_Omega;
	
	/* 判断是否需要进行角速度控制 */
	if (this->Target_Omega_Now == this->Target_Omega_Last)
	{
        return;
	}

    /* 判断角速度是否为零 */
    if (this->Target_Omega_Now == 0.0f)
    {
        HAL_TIM_PWM_Stop(this->TIM_PWM, this->PWM_Channel);
    }
    else
    {
        Arr_Last_Val = __HAL_TIM_GET_AUTORELOAD(this->TIM_PWM);
    
        /* 判断是否需要换向 */
        if (this->Target_Omega_Now > 0)
        {
            if (this->Target_Omega_Last <= 0)
            {
                /* 电机换向标志置位 */
                flag = 1;
                
                /* 进行电机方向切换 */
                __HAL_TIM_SET_COMPARE(this->TIM_PWM, this->PWM_Channel, Arr_Last_Val + 1);
                HAL_TIM_PWM_Start(this->TIM_PWM, this->PWM_Channel);
                Delay_us(2);
                HAL_GPIO_WritePin(this->GPIOx[0], this->GPIO_Pin[0], GPIO_PIN_SET);
                Delay_us(2);
            }
        }
        else
        {
            if (this->Target_Omega_Last >= 0)
            {
                /* 电机换向标志置位 */
                flag = 1;
                
                /* 进行电机方向切换 */
                __HAL_TIM_SET_COMPARE(this->TIM_PWM, this->PWM_Channel, Arr_Last_Val + 1);
                HAL_TIM_PWM_Start(this->TIM_PWM, this->PWM_Channel);
                Delay_us(2);
                HAL_GPIO_WritePin(this->GPIOx[0], this->GPIO_Pin[0], GPIO_PIN_RESET);
                Delay_us(2);
            }
        }
        
        /* 设置PWM输出频率，控制电机转速 */
        Arr_Val = this->TIM_Frequence / (Omega_Abs / (2.0f * PI) * this->Subdivision) - 1;
        
        __HAL_TIM_SET_AUTORELOAD(this->TIM_PWM, Arr_Val);
        
        if (flag == 1)
        {
            __HAL_TIM_SET_COMPARE(this->TIM_PWM, this->PWM_Channel, this->PWM_CCR);
        }
    }
}

/*****************************************************************************************************
 * @brief   步进电机控制函数（需在系统心跳定时器更新中断中执行）
*****************************************************************************************************/
void Class_Motor_Step::Control()
{	
	/* 判断是否到达控制周期 */
    if (this->Cycle_Counter < this->Control_Cycle - 1)
    {
        /* 未到达控制周期 */
        this->Cycle_Counter += 1;
    }
    else
    {
        int16_t complete_steps, target_steps;

        /* 到达控制周期，进行电机控制 */
        this->Cycle_Counter = 0;

        /* 判断电机当前控制模式 */
        if (this->control_mode == Motor_Speed)
        {
            /* 梯形变速速度控制 */
            this->Gear_Slope.Set_Value(this->Set_Omega);
            this->Gear_Slope.Calculate();
            this->OmegaControl(this->Gear_Slope.Get_Out());
        }
        else if ((this->control_mode == Motor_RelativeAngle) || (this->control_mode == Motor_AbsoluteAngle))
        {
            complete_steps = __HAL_TIM_GET_COUNTER(this->TIM_Slave);
            target_steps = __HAL_TIM_GET_AUTORELOAD(this->TIM_Slave);
            
            if (((complete_steps * this->Step_Len / this->Cycle_Step) <= this->Speed_MIN) || 
                (((target_steps - complete_steps) * this->Step_Len / this->Cycle_Step) <= this->Speed_MIN))
            {
                return;
            }
            
            if (this->Set_Omega > 0)
            {
                if (complete_steps < (target_steps / 2))
                {
                    if (complete_steps < (this->Set_Omega * this->Cycle_Step / this->Step_Len))
                    {
                        this->OmegaControl(complete_steps * this->Step_Len / this->Cycle_Step);
                    }
                    else
                    {
                        this->OmegaControl(this->Set_Omega);
                    }
                }
                else if (complete_steps > (target_steps / 2))
                {
                    if ((target_steps - complete_steps) < (this->Set_Omega * this->Cycle_Step / this->Step_Len))
                    {
                        this->OmegaControl((target_steps - complete_steps) * this->Step_Len / this->Cycle_Step);
                    }
                    else
                    {
                        this->OmegaControl(this->Set_Omega);
                    }
                }
            }
            else if (this->Set_Omega < 0)
            {
                if (complete_steps < (target_steps / 2))
                {
                    if (complete_steps < -(this->Set_Omega * this->Cycle_Step / this->Step_Len))
                    {
                        this->OmegaControl(-complete_steps * this->Step_Len / this->Cycle_Step);
                    }
                    else
                    {
                        this->OmegaControl(this->Set_Omega);
                    }
                }
                else if (complete_steps > (target_steps / 2))
                {
                    if ((target_steps - complete_steps) < -(this->Set_Omega * this->Cycle_Step / this->Step_Len))
                    {
                        this->OmegaControl(-(target_steps - complete_steps) * this->Step_Len / this->Cycle_Step);
                    }
                    else
                    {
                        this->OmegaControl(this->Set_Omega);
                    }
                }
            }
        }
    }
}

/*****************************************************************************************************
 * @brief   步进电机电机角度控制中断处理函数（需在脉冲计数定时器更新中断中执行）
*****************************************************************************************************/
void Class_Motor_Step::AngleITProc()
{
    /* 停止电机 */
    this->OmegaControl(0);
    this->Set_Omega = 0.0f;
    
    /* 电机角度记录 */
    if (this->Target_Omega_Last > 0)
    {
        this->Total_Step += __HAL_TIM_GET_AUTORELOAD(this->TIM_Slave) + 1;
        this->angle += (__HAL_TIM_GET_AUTORELOAD(this->TIM_Slave) + 1) / (this->Subdivision / 360.0);
    }
    else if (this->Target_Omega_Last < 0)
    {
        this->Total_Step -= __HAL_TIM_GET_AUTORELOAD(this->TIM_Slave) + 1;
        this->angle -= (__HAL_TIM_GET_AUTORELOAD(this->TIM_Slave) + 1) / (this->Subdivision / 360.0);
    }
    
    /* 失能脉冲计数定时器 */
    HAL_TIM_Base_Stop(this->TIM_Slave);
}

/*****************************************************************************************************
 * @brief   步进电机相对角度控制函数
 * 
 * @param   Motor_Speed     设定电机速度
 * @param   Motor_Angle     设定电机角度
*****************************************************************************************************/
void Class_Motor_Step::AngleSet_Relative(float Motor_Speed, float Motor_Angle)
{
	uint16_t motor_steps, complete_steps, remain_steps;
	
	if ((Motor_Speed == 0.0f) || (Motor_Angle == 0.0f))
	{
		return;
	}
	
	if (this->TIM_Slave != nullptr)
	{
		/* 计算应走步数 */
		motor_steps = (uint16_t)(Motor_Angle * this->Subdivision / 360.0f);
		
		if (this->control_mode == Motor_RelativeAngle)
		{
			complete_steps = __HAL_TIM_GET_COUNTER(this->TIM_Slave);
			remain_steps = __HAL_TIM_GET_AUTORELOAD(this->TIM_Slave) - complete_steps;
			
			if ((this->Target_Omega_Now * Motor_Speed) > 0)
			{
				motor_steps += remain_steps;
			}
			else if ((this->Target_Omega_Now * Motor_Speed) < 0)
			{
				if (motor_steps == remain_steps)
				{
					this->OmegaControl(0);
					return;
				}
				else if (motor_steps > remain_steps)
				{
					motor_steps = motor_steps - remain_steps;
				}
				else if (motor_steps < remain_steps)
				{
					motor_steps = remain_steps - motor_steps;
					Motor_Speed *= -1;
				}
			}
		}
		
		if (motor_steps == 0)
		{
			this->OmegaSet(0);
			return;
		}
		
		motor_steps = motor_steps - 1;
		
		/* 设置脉冲计数定时器参数 */
		__HAL_TIM_SET_COUNTER(this->TIM_Slave, 0);
		__HAL_TIM_SET_AUTORELOAD(this->TIM_Slave, motor_steps);
		
		/* 使能定时器，开始脉冲计数 */
		HAL_TIM_Base_Start(this->TIM_Slave);
		
		/* 启动电机 */
        this->Set_Omega = Motor_Speed;
		this->OmegaControl(((Motor_Speed > 0) ? 1 : -1) * this->Speed_MIN);
		this->control_mode = Motor_RelativeAngle;
	}
}

/*****************************************************************************************************
 * @brief   步进电机绝对角度控制函数
 * 
 * @param   Motor_Speed     设定电机速度(>=0)
 * @param   Target_Angle    设定电机目标角度
*****************************************************************************************************/
void Class_Motor_Step::AngleSet_Absolute(float Motor_Speed, float Target_Angle)
{
	int16_t complete_steps, relative_steps, target_steps;
	
	if (Motor_Speed == 0)
	{
		return;
	}
	
	if (this->TIM_Slave != nullptr)
	{
		
		if (this->control_mode == Motor_AbsoluteAngle)
		{
			complete_steps = __HAL_TIM_GET_COUNTER(this->TIM_Slave);
			
			/* 电机角度记录 */
			if (this->Target_Omega_Now > 0)
			{
				this->Total_Step += complete_steps;
				this->angle += complete_steps / (this->Subdivision / 360.0f);
			}
			else if (this->Target_Omega_Now < 0)
			{
				this->Total_Step -= complete_steps;
				this->angle -= complete_steps / (this->Subdivision / 360.0f);
			}
		}
		
		target_steps = (int16_t)(Target_Angle * this->Subdivision / 360.0f);
		
		relative_steps = target_steps - this->Total_Step;
		
		if (relative_steps == 0)
		{
			this->OmegaControl(0);
			return;
		}
		else if (relative_steps < 0)
		{
			relative_steps *= -1;
			Motor_Speed *= -1;
		}
		
		relative_steps -= 1;
		
		/* 设置脉冲计数定时器参数 */
		__HAL_TIM_SET_COUNTER(this->TIM_Slave, 0);
		__HAL_TIM_SET_AUTORELOAD(this->TIM_Slave, (uint16_t)relative_steps);
		
		/* 使能定时器，开始脉冲计数 */
		HAL_TIM_Base_Start(this->TIM_Slave);
		
		/* 启动电机 */
        this->Set_Omega = Motor_Speed;
		this->OmegaControl(((Motor_Speed > 0) ? 1 : -1) * this->Speed_MIN);
		this->control_mode = Motor_AbsoluteAngle;
	}
}

/************************************************************************************************************************
 * @brief   舵机初始化
 *
 * @param   __Angle_MAX     舵机最大角度 (°)
 * @param   __Zero_Offset   舵机安装零偏 (°)
 * @param   __PWM_Period    PWM 周期 (us)
 ***********************************************************************************************************************/
void Class_Motor_Servo::Init(TIM_HandleTypeDef * __TIM_PWM, uint32_t __PWM_Channel,
                             float __Angle_MAX, float __Zero_Offset, float __PWM_Period)
{
    /* 参数赋值 */
    this->TIM_PWM = __TIM_PWM;
    this->PWM_Channel = __PWM_Channel;
    this->Angle_MAX = __Angle_MAX;
    this->Zero_Offset = __Zero_Offset;

    /* 初始位置设置 */
    this->AngleSet(0.0f);

    /* PWM 定时器启动 */
    HAL_TIM_PWM_Start(this->TIM_PWM, this->PWM_Channel);
}

/************************************************************************************************************************
 * @brief   舵机角度设置
 *
 * @param   __Set_Angle     舵机设置角度 (°)（相对于安装 0°）
 * @return  float           实际设置的角度 (°)
 ***********************************************************************************************************************/
float Class_Motor_Servo::AngleSet(float __Set_Angle)
{
    /* 零偏补偿 */
    __Set_Angle = __Set_Angle + this->Zero_Offset;

    /* 角度限定 */
    __Set_Angle = fmodf(__Set_Angle, 360.0f);
    if (__Set_Angle > this->Angle_MAX)
    {
        __Set_Angle = this->Angle_MAX;
    }
    else if (__Set_Angle < 0)
    {
        __Set_Angle = 0;
    }
    this->Set_Angle = __Set_Angle;

    /* 输出比较寄存器赋值，产生对应PWM信号 */
    auto ccr_val = (uint32_t)((this->Set_Angle / this->Angle_MAX * 2.0f + 0.5f) / this->PWM_Period * (this->TIM_PWM->Init.Period + 1U)) - 1U;
    __HAL_TIM_SET_COMPARE(this->TIM_PWM, this->PWM_Channel, ccr_val);
    
    return (this->Set_Angle - this->Zero_Offset);
}
