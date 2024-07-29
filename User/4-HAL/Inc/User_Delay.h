#ifndef __HAL_USER_DELAY_H
#define __HAL_USER_DELAY_H

/* ͷ�ļ����� ----------------------------------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* �������� ------------------------------------------------------------------------------------------------------------*/
void Delay_Init(uint16_t __SysClk);
void Delay_us(uint32_t nus);
void Delay_ms(uint16_t nms);
void HAL_Delay(uint32_t Delay);

#endif    /* HAL_User_Delay.h */
