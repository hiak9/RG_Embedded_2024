/**
 * @file    User_Math.h
 * @brief   用户数学库
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

#ifndef __MIL_MATH_H
#define __MIL_MATH_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "arm_math.h"
#include "float.h"
#include "limits.h"
#include "stdint.h"

/* 宏定义 -------------------------------------------------------------------------------------------------------------*/
#define RPM_TO_RADPS        (2.0f * PI / 60.0f)     /* RPM换算到rad/s */
#define DEG_TO_RAD          (PI / 180.0f)           /* deg换算到rad */
#define CELSIUS_TO_KELVIN   (273.15f)               /* 摄氏度换算到开氏度 */
#define GRAVITY_ACCELERATE  (9.7933f)               /* 重力加速度 (合肥9.7933) */

/* 联合体定义 ----------------------------------------------------------------------------------------------------------*/
union Union_uint8_float
{
    uint8_t Data_uint8[4];
    float Data_float;
};

/* 函数声明 ------------------------------------------------------------------------------------------------------------*/
void Math_Endian_Reverse_16(void *Address);
void Math_Endian_Reverse_16(void *Source, void *Destination);
void Math_Endian_Reverse_32(void *Address);
void Math_Endian_Reverse_32(void *Source, void *Destination);
uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length);
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length);
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length);
float Math_Sinc(float x);
int32_t Math_Float_To_Int(float x, float Float_Min, float Float_Max, int32_t Int_Min, int32_t Int_Max);
float Math_Int_To_Float(int32_t x, int32_t Int_Min, int32_t Int_Max, float Float_Min, float Float_Max);
void Math_Matrix_Multiply_3_3(float (*_RotationMatrix_1)[3],float (*_RotationMatrix_2)[3],float (*_RotationMatrix_Out)[3]);
void Math_Matrix_Multiply_3_1(float (*_RotationMatrix_1)[3],float (*_RotationMatrix_2),float (*_RotationMatrix_Out));

/**
 * @brief 限幅函数
 *
 * @tparam Type
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
template<typename Type>
void Math_Constrain(Type *x, Type Min, Type Max)
{
    if(*x < Min)
    {
        *x = Min;
    }
    else if(*x > Max)
    {
        *x = Max;
    }
}

/**
 * @brief 求绝对值
 *
 * @tparam Type
 * @param x 传入数据
 * @return Type x的绝对值
 */
template<typename Type>
Type Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}

#endif /* MIL_Math.h */
