1. 用cube重新生成文件时，要将stm32f407xx.h中#define __FPU_PRESENT  1U屏蔽，因为已在魔法棒中进行了宏定义

2. TIM2 ~ TIM5为编码器接口，TIM6为定时器，TIM9输出为麦克纳姆轮的PWM波，TIM9为上面两个摩擦轮生成PWM波，TIM10、TIM11两个生成下面两个摩擦轮的PWM波

3. 编码器线数是13

4. 

   

