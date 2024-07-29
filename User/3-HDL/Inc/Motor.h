/**
 * @file    Motor.h
 * @brief   电机驱动
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

#ifndef __HDL_MOTOR_H
#define __HDL_MOTOR_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "tim.h"

#include "Gear.h"
#include "Pid.h"
#include "User_Delay.h"

/* 枚举类型定义 --------------------------------------------------------------------------------------------------------*/
/**
 * @brief   BDC电机状态枚举类型
 */
enum Enum_MotorState_BDC
{
    Motor_Suspend   = 0U,   /*!< 悬空 */
    Motor_Brake     = 1U,   /*!< 刹车 */
    Motor_Run       = 2U,   /*!< 运行 */
};

/**
 * @brief   步进电机控制模式枚举类型
 */
enum Enum_MotorMode_Step
{
    Motor_Speed         = 0U,   /*!< 速度模式 */
    Motor_RelativeAngle = 1U,   /*!< 相对角度模式 */
    Motor_AbsoluteAngle = 2U,   /*!< 绝对角度模式 */
};

/* 类定义 -------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   BDC电机驱动类（直流有刷电机）（规定朝向电机轴方向看，逆时针为正方向）
 */
class Class_Motor_BDC
{
public:
    /* 变量 */
    Class_PID PID_Omega;                    /*!< 角速度 PID 控制器 */
    Class_Gear_Slope Gear_Slope;            /*!< 斜坡变速控制器 */

    TIM_HandleTypeDef * TIM_Encoder;        /*!< TIM-编码器句柄 */
    TIM_HandleTypeDef * TIM_PWM;            /*!< TIM-PWM 句柄 */
    uint32_t PWM_Channel;                   /*!< TIM-PWM 通道 */
    GPIO_TypeDef * GPIOx_Dir[2];            /*!< 方向控制引脚 Port */
    uint32_t GPIO_Pin_Dir[2];               /*!< 方向控制引脚 Pin */

    /* 函数 */
    void Init(TIM_HandleTypeDef * __TIM_Encoder, TIM_HandleTypeDef * __TIM_PWM, uint32_t __PWM_Channel,
              GPIO_TypeDef * __GPIOx_Dir_A, GPIO_TypeDef * __GPIOx_Dir_B, uint32_t __GPIO_Pin_Dir_A, uint32_t __GPIO_Pin_Dir_B,
              float __Omega_MAX = 26.0f, float __Reduction_Ratio = 27.0f, uint16_t __Encoder_Lines = 13U, uint16_t __Control_Cycle = 50U);
    void Control();
    
    inline float MotionSet(float __Set_Omega);
    inline void StopSet(Enum_MotorState_BDC __Stop_State);
    inline float Get_ActualOmega();
    inline float Get_TargetOmega();
private:
    /* 函数 */
    inline void Msp_Init();

    /* 常量 */
    float Omega_MAX;                        /*!< 电机输出轴最大角速度 (rad/s) */
    float Reduction_Ratio;                  /*!< 电机减速比 */
    uint16_t Encoder_Lines;                 /*!< 电机编码器线数 */
    uint16_t Control_Cycle;                 /*!< 电机控制周期 (控制周期 = Control_Cycle * 系统心跳周期) */
    const float Heartbeat_Period = 1.0f;    /*!< 系统心跳定时器周期 (ms) */

    /* 读写变量 */
    float Set_Omega = 0.0f;                 /*!< 电机输出轴设定角速度 (rad/s) */
    float Target_Omega = 0.0f;              /*!< 电机输出轴目标角速度 (rad/s) */
    float Actual_Omega = 0.0f;              /*!< 电机输出轴实际角速度 (rad/s) */
    float Out_Omega = 0.0f;                 /*!< 电机输出轴输出角速度 (rad/s) */

    /* 内部变量 */
    Enum_MotorState_BDC Motor_State =       /*!< 电机当前状态 状态机 */
                        Motor_Suspend;
    uint16_t Cycle_Counter = 0U;            /*!< 电机控制周期计数器 */
};

/**
 * @brief	 步进电机驱动类
 */
class Class_Motor_Step
{
public:
	/* 变量 */
    Class_Gear_Slope Gear_Slope;            /*!< 斜坡变速控制器 */

	TIM_HandleTypeDef * TIM_Slave;		    /*!< TIM-Slave 句柄（用于脉冲计数进行角度控制） */
    TIM_HandleTypeDef * TIM_PWM;            /*!< TIM-PWM 句柄 */
    uint32_t PWM_Channel;                   /*!< TIM-PWM 通道 */
    GPIO_TypeDef * GPIOx[2];                /*!< 控制引脚 Port */
    uint32_t GPIO_Pin[2];                   /*!< 控制引脚 Pin */

    /* 函数 */
    void Init();
    void Control();
    void AngleITProc();
    void AngleSet_Relative(float Motor_Speed, float Motor_Angle);
    void AngleSet_Absolute(float Motor_Speed, float Target_Angle);

    inline float OmegaSet(float __Set_Omega);
    inline float AngleGet();
protected:
    /* 函数 */
    inline void Msp_Init();
    void OmegaControl(float Motor_Omega);

    /* 常量 */
    uint32_t PWM_CCR;                       /*!< PWM输出比较寄存器值 */
	uint16_t Subdivision;				    /*!< 电机细分 */
	float Speed_MAX;					    /*!< 电机速度最大值 (rad/s) */
	float Speed_MIN;					    /*!< 电机速度最小值 (rad/s) */
	uint16_t Step_Len;					    /*!< 电机变速步长 */
	uint16_t Control_Cycle;                 /*!< 电机控制周期 (控制周期 = Control_Cycle * 系统心跳周期) */
	uint16_t Cycle_Step;				    /*!< 电机控制周期（步数） */
    float TIM_Frequence = 1000000.0f;       /*!< 定时时器计数频率 (Hz) */
	
	/* 读写变量 */
    int32_t Total_Step = 0U;		        /*!< 电机总共运行步数（仅用于绝对角度控制） */
	float angle = 0.0f;					    /*!< 电机当前设定角度（°） */
    float Set_Omega = 0.0f;				    /*!< 电机设定速度 (rad/s) */
	float Target_Omega_Now = 0.0f;		    /*!< 电机当前目标速度 (rad/s) */
	float Target_Omega_Last = 0.0f;		    /*!< 电机上次目标速度 (rad/s) */
	
    /* 内部变量 */
    Enum_MotorMode_Step control_mode;	    /*!< 电机控制模式 */
	uint16_t Cycle_Counter = 0U;            /*!< 电机控制周期计数器 */
};

/**
 * @brief   Servo电机驱动类（舵机）
 */
class Class_Motor_Servo
{
public:
    /* 变量 */
    TIM_HandleTypeDef * TIM_PWM;            /*!< TIM-PWM 句柄 */
    uint32_t PWM_Channel;                   /*!< TIM-PWM 通道 */

    /* 函数 */
    void Init(TIM_HandleTypeDef * __TIM_PWM, uint32_t __PWM_Channel,
              float __Angle_MAX = 360.0f, float __Zero_Offset = 0.0f, float __PWM_Period = 20.0f);
    float AngleSet(float __Set_Angle);
private:
    /* 常量 */
    float PWM_Period;                       /*!< PWM 周期 (us) */
    float Angle_MAX;                        /*!< 舵机最大角度 (°) */
    float Zero_Offset;                      /*!< 舵机安装零偏 (°)（定义：处于安装零点时舵机实际角度） */

    /* 读写变量 */
    float Set_Angle;                        /*!< 舵机目标角度 (°) */
};

/* 变量声明 ------------------------------------------------------------------------------------------------------------*/
extern Class_Motor_Servo Motor_Test_Servo;

/* 接口函数定义 --------------------------------------------------------------------------------------------------------*/
/**
 * @brief   BDC电机运动设置函数
 *
 * @param   __Set_Omega     设定的角速度 (rad/s)
 * @return  float           返回实际设定的角速度 (rad/s)
 */
float Class_Motor_BDC::MotionSet(float __Set_Omega)
{
    /* 速度设置 */
    if (__Set_Omega > this->Omega_MAX)
    {
        __Set_Omega = this->Omega_MAX;
    }
    else if (__Set_Omega < -this->Omega_MAX)
    {
        __Set_Omega = -this->Omega_MAX;
    }
    this->Set_Omega = __Set_Omega;

    /* 状态设置 */
    this->Motor_State = Motor_Run;
    return this->Set_Omega;
}

/**
 * @brief   BDC电机停止设置函数
 *
 * @param   __Stop_State    设定的停止模式
 */
void Class_Motor_BDC::StopSet(Enum_MotorState_BDC __Stop_State)
{
    /* 判断状态是否有效 */
    if (__Stop_State == Motor_Suspend || __Stop_State == Motor_Brake)
    {
        /* 状态设置 */
        this->Motor_State = __Stop_State;
    }
}

/**
 * @brief   BDC电机获取目标速度
 */
float Class_Motor_BDC::Get_TargetOmega()
{
    return this->Target_Omega;
}

/**
 * @brief   BDC电机获取实际速度
 */
float Class_Motor_BDC::Get_ActualOmega()
{
    return this->Actual_Omega;
}

/**
 * @brief   步进电机角速度设定函数
 * 
 * @param   __Set_Omega     设定的角速度 (rad/s)
 * @return  float           返回实际设定的角速度 (rad/s)
 */
float Class_Motor_Step::OmegaSet(float __Set_Omega)
{
	this->Set_Omega = __Set_Omega;
	this->control_mode = Motor_Speed;

    return this->Set_Omega;
}

/**
 * @brief   步进电机角度获取函数
 * 
 * @return  float   返回当前角度 (°)
 */
float Class_Motor_Step::AngleGet()
{
    return (this->Total_Step / this->Subdivision * 360.0f);
}

#endif  /* HDL_Motor.h */
