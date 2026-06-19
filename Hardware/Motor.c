#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	PWM_Init();
}


void Motor1_SetSpeed(int Speed)
{
	if(Speed>7200)
		Speed=7200;
	else if(Speed<-7200)
		Speed=-7200;
	
	if(Speed >= 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
		GPIO_ResetBits(GPIOA, GPIO_Pin_3);
		PWM_SetCompare1(Speed);
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
		GPIO_SetBits(GPIOA, GPIO_Pin_3);
		PWM_SetCompare1(-Speed);
	}
}
void Motor2_SetSpeed(int Speed)
{
	if(Speed>7200)
		Speed=7200;
	else if(Speed<-7200)
		Speed=-7200;
	
	if(Speed >= 0)
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare2(Speed);
	}
	else
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare2(-Speed);
	}
}
