/**
 * @file    Chassis.h
 * @brief   底盘功能模块
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

#ifndef __FML_CHASSIS_H
#define __FML_CHASSIS_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Motor.h"

/* 结构体定义 ----------------------------------------------------------------------------------------------------------*/

/* 枚举类型定义 --------------------------------------------------------------------------------------------------------*/
/**
 * @brief   底盘状态枚举类型
 */
enum Enum_ChassisState : uint8_t
{
    Chassis_Disable = 0U,   /*!< 底盘失能 */
    Chassis_Suspend = 1U,   /*!< 底盘悬空 */
    Chassis_Brake   = 2U,   /*!< 底盘刹车 */
    Chassis_Run     = 3U,   /*!< 底盘运行 */
};

/* 类定义 --------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   麦克纳姆轮底盘功能模块类
 *          规定底盘前向为X轴正方向，左向为Y轴正方向，逆时针为旋转正方向
 *          第一象限电机0，第二象限电机2，第三象限电机3，第四象限电机1
 */
class Class_Chassis_Macnum
{
public:
    /* 变量 */
    Class_Motor_BDC Motor_Wheel[4];         /*!< 四轮驱动电机对象 */

    /* 函数 */
    void Init(float __Wheel_Omega_MAX = 26.0f, uint16_t __Control_Cycle = 50U);
    void Control();

    inline void Enable();
    inline void Disable();
    inline void Set_Motion(float __Velocity_X, float __Velocity_Y, float __Omega);
    inline void Set_Stop(Enum_ChassisState __Stop_State);
protected:
    /* 常量 */
    const float Wheel_Radius = 0.0635f;     /*!< 底盘轮子半径 (m) */
    const float Wheel_Spacing = 0.8f;       /*!< 底盘轮间距（左右）(m) */
    const float Wheel_Base = 0.8f;          /*!< 底盘轴距（前后）(m) */
    float Wheel_Omega_MAX;                  /*!< 轮子最大角速度 (rad/s) */
    uint16_t Control_Cycle;                 /*!< 底盘控制周期 (控制周期 = Control_Cycle * 系统心跳周期) */

    /* 读写变量 */
    float Velocity_X = 0.0f;                /*!< X方向目标速度 (m/s) */
    float Velocity_Y = 0.0f;                /*!< Y方向目标速度 (m/s) */
    float Omega = 0.0f;                     /*!< 旋转目标角速度 (rad/s) */

    /* 内部变量 */
    Enum_ChassisState Chassis_State =       /*!< 底盘状态 状态机 */
                      Chassis_Disable;
    uint16_t Cycle_Counter = 0U;            /*!< 底盘控制周期计数器 */
};

/* 变量声明 ------------------------------------------------------------------------------------------------------------*/
extern Class_Chassis_Macnum Committee_Chariot;

/* 接口函数定义 --------------------------------------------------------------------------------------------------------*/
/**
 * @brief   底盘使能
 */
void Class_Chassis_Macnum::Enable()
{
    this->Chassis_State = Chassis_Suspend;
}

/**
 * @brief   底盘失能
 */
void Class_Chassis_Macnum::Disable()
{
    this->Chassis_State = Chassis_Disable;
}

/**
 * @brief   底盘运动设置
 *
 * @param   __Velocity_X    X方向设定速度 (m/s)
 * @param   __Velocity_Y    Y方向设定速度 (m/s)
 * @param   __Omega         旋转设定角速度 (rad/s)
 */
void Class_Chassis_Macnum::Set_Motion(float __Velocity_X, float __Velocity_Y, float __Omega)
{
    if (this->Chassis_State != Chassis_Disable)
    {
        this->Velocity_X = __Velocity_X;
        this->Velocity_Y = __Velocity_Y;
        this->Omega = __Omega;
        this->Chassis_State = Chassis_Run;
    }
}

/**
 * @brief   底盘停止设置
 * 
 * @param   设定的停止状态
 */
void Class_Chassis_Macnum::Set_Stop(Enum_ChassisState __Stop_State)
{
    if (this->Chassis_State != Chassis_Disable && (__Stop_State == Chassis_Suspend || __Stop_State == Chassis_Brake))
    {
        this->Chassis_State = __Stop_State;
    }
}

#endif  /* FML_Chassis.h */
