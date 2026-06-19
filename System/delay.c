/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "delay.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果需要使用OS,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"
#include "task.h"
#endif

static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数

//systick中断服务函数,使用ucos时用到
extern void xPortSysTickHandler(void);
void SysTick_Handler(void)
{	
//	if(delay_osrunning==1)						//OS开始跑了,才执行正常的调度处理
//	{
//		OSIntEnter();							//进入中断
//		OSTimeTick();       					//调用ucos的时钟服务程序               
//		OSIntExit();       	 					//触发任务切换软中断
//	}
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//FreeROTS系统已经运行
    {
        xPortSysTickHandler();
    }
}

//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init()
{
#if SYSTEM_SUPPORT_OS  							//如果需要支持OS.
	u32 reload;
#endif
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	//fac_us=SystemCoreClock/8000000; //为系统时钟的1/8 
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);	//FreeRTOS选择外部时钟  HCLK					
	fac_us=SystemCoreClock/1000000;	
#if SYSTEM_SUPPORT_OS  							//如果需要支持OS.
	
//ucos
//	reload=SystemCoreClock/8000000;				//每秒钟的计数次数 单位为K	   
//	reload*=1000000/delay_ostickspersec;		//根据delay_ostickspersec设定溢出时间
//												//reload为24位寄存器,最大值:16777216,在72M下,约合1.86s左右	
//	fac_ms=1000/delay_ostickspersec;			//代表OS可以延时的最少单位	   
 
//	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
//	SysTick->LOAD=reload; 						//每1/delay_ostickspersec秒中断一次	
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK  
	
//RTOS
	reload=SystemCoreClock/1000000; //每秒钟的计数次数 单位为M
    reload*=1000000/configTICK_RATE_HZ; //根据configTICK_RATE_HZ 设定溢出
    //时间reload 为24 位寄存器,最大值:
    //16777216,在72M 下,约合0.233s 左右
    fac_ms=1000/configTICK_RATE_HZ; //代表OS 可以延时的最少单位
    SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk; //开启SYSTICK 中断
    SysTick->LOAD=reload; //每1/configTICK_RATE_HZ 秒中断 一次
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开启SYSTICK
 
#else
	fac_ms=(u16)fac_us*1000;					//非OS下,代表每个ms需要的systick时钟数   
#endif
}

//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;					//LOAD的值	    	 
	ticks=nus*fac_us; 							//需要的节拍数	
	//普通延时FreeRTOS无法调度 不用设置阻止调度	
	//delay_osschedlock();						//阻止OS调度，防止打断us延时 属于UCOS us 
	told=SysTick->VAL;        					//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;		//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;				//时间超过/等于要延迟的时间,则退出.
		}  
	};
	//delay_osschedunlock();						//恢复OS调度	 属于UCOS						    
}
//延时nms
//nms:要延时的ms数
void delay_ms(u32 nms)
{	
//	if(delay_osrunning&&delay_osintnesting==0)	//如果OS已经在跑了,并且不是在中断里面(中断里面不能任务调度)	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//Free RTOS系统已经运行    
	{		 
		if(nms>=fac_ms)							//延时的时间大于OS的最少时间周期 
		{ 
//   			delay_ostimedly(nms/fac_ms);		//OS延时
			 vTaskDelay(nms/fac_ms); //FreeRTOS 延时 不会阻止调度
		}
		nms%=fac_ms;							//OS已经无法提供这么小的延时了,采用普通方式延时    
	}
	delay_us((u32)(nms*1000));					//普通方式延时  
}
 
//延时nms,不会引起任务调度
//nms:要延时的ms 数
void delay_xms(u32 nms)
{
    u32 i;
    for(i=0;i<nms;i++) delay_us(1000);
}





































