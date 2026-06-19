#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1  ;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period= 7199 ;
  TIM_TimeBaseInitStructure.TIM_Prescaler=0 ;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);      
	
	TIM_OCInitTypeDef TIM_OCInitStructure; 
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCPolarity_High ;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable ;
	TIM_OCInitStructure.TIM_Pulse=0;//CCR      
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);//配置通道三，对应A0
			
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCPolarity_High ;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable ;
	TIM_OCInitStructure.TIM_Pulse=0;//CCR      
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);//配置通道二，对应A1
			
			
	TIM_Cmd(TIM2,ENABLE);
	
}

void PWM_SetCompare1(int Compare)
{
	TIM_SetCompare1(TIM2,Compare);
}

void PWM_SetCompare2(int Compare)
{
	TIM_SetCompare2(TIM2,Compare);
}
