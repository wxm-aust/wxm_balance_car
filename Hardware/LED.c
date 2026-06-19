#include "stm32f10x.h"                  // Device header

void LED_Init(void)
{	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_6|GPIO_Pin_5;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB,&GPIO_InitStruct);
}
