/**
 * @file    Crc.h
 * @brief   CRC校验算法
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-16
 * @version v1.0
 */

#ifndef __MIL_CRC_H
#define __MIL_CRC_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "User_Math.h"

/* 函数声明 ------------------------------------------------------------------------------------------------------------*/
uint8_t Calculate_CRC8(uint8_t * Buffer, uint32_t Buffer_Len);

#endif  /* MIL_Crc.h */
