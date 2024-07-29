/**
 * @file    Gear.h
 * @brief   变速算法
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-15
 * @version v1.0
 */

#ifndef __MIL_GEAR_H
#define __MIL_GEAR_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "User_Math.h"

/* 枚举类型定义 --------------------------------------------------------------------------------------------------------*/

/* 类定义 --------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   斜坡变速控制器类
 */
class Class_Gear_Slope
{
public:
    /* 函数 */
    void Init(float __Step_Length = 0.0f);
    void Calculate();

    inline float Set_Value(float __Set_Value);
    inline float Get_Out();
protected:
    /* 常量 */
    float Step_Length;      /*!< 变速步长 */

    /* 读写变量 */
    float Set_Val = 0.0f;   /*!< 设定值 */
    float Out_Val = 0.0f;   /*!< 当前输出值 */
};

/* 函数声明 ------------------------------------------------------------------------------------------------------------*/

/* 接口函数定义 --------------------------------------------------------------------------------------------------------*/
/**
 * @brief   变速值设定
 *
 * @return  float   设定的值
 */
float Class_Gear_Slope::Set_Value(float __Set_Value)
{
    this->Set_Val = __Set_Value;
    return this->Set_Val;
}

/**
 * @brief   输出值获取
 *
 * @return  float   当前输出值
 */
float Class_Gear_Slope::Get_Out()
{
    return this->Out_Val;
}

#endif /* MIL_Gear.h */
