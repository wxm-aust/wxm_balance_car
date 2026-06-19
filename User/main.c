#include "stm32f10x.h"                  // Device header
/*外设头文件*/
#include "OLED.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "exti.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "Timer.h"
#include <string.h>
#include <cstring>
/*FreeRTOS用到的头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"


/**********************************************************************************/
/* START_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define START_TASK_PRIO         1
#define START_TASK_STACK_SIZE   128
TaskHandle_t    start_task_handler;
void start_task( void * pvParameters );
/**********************************************************************************/
/* TASK1 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define TASK1_PRIO         3
#define TASK1_STACK_SIZE   256
TaskHandle_t    task1_handler;
void task1( void * pvParameters );
/**********************************************************************************/
/* TASK2 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define TASK2_PRIO         1
#define TASK2_STACK_SIZE   256
TaskHandle_t    task2_handler;
void task2( void * pvParameters );

#define TASK3_PRIO         3
#define TASK3_STACK_SIZE   256
TaskHandle_t    task3_handler;
void task3( void * pvParameters );
/**********************************************************************************/
/*
 * 定时器 配置
 */
 TimerHandle_t timer_MPU6050_GET_10ms_Handle = 0;    /* 10ms周期定时器 */
 void timer_MPU6050_GET_10ms_callback( TimerHandle_t pxTimer );
 
 TimerHandle_t timer_PID_Ctrl_10ms_Handle = 0;    /* 10ms周期定时器 */
 void timer_PID_Ctrl_10ms_callback( TimerHandle_t pxTimer );
 
 TimerHandle_t timer_1000ms_handle = 0;    /* 1000ms周期定时器 */
 void timer_1000ms_callback( TimerHandle_t pxTimer );
 
  TimerHandle_t timer_HC_05_GET_10ms_Handle = 0;    /* 10ms周期定时器 */
 void timer_HC_05_GET_10ms_callback( TimerHandle_t pxTimer );
 /**********************************************************************************/
/*
 * 事件标志组 
 */
EventGroupHandle_t  eventgroup_handle;
#define LED_FLASH_EVT       (1 << 0)
#define MPU6050_GET_EVT     (1 << 1)
#define PID_CTRL_EVT				(1 << 2)
#define EVENT_ALL    		(LED_FLASH_EVT | MPU6050_GET_EVT | PID_CTRL_EVT)
/**********************************************************************************/
/*
 * 二值信号量
 */
 QueueHandle_t semphore_handle;
/**********************************************************************************/
/*本地全局函数*/

static void Get_Angle(void);/*获取mpu6050角度值*/
void oled_show(void);
void GET_Data(void);
/**********************************************************************************/

int s1,s2,v1,v2;
float Pitch,Roll,Yaw;						//角度
short gyrox,gyroy,gyroz;				//陀螺仪--角速度
uint8_t RxData;							//接收蓝牙指令
float Balance_Kp=48000,Balance_Kd=150,Velocity_Kp=22000,Velocity_Ki=110,Turn_Kp=4200,Turn_Kd=0;//PID参数（放大100倍）
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		//平衡环PWM变量，速度环PWM变量
uint8_t RxSTA=1;
void pid_ctrl(){
	
	int Motor_1,Motor_2;      //电机PWM变量
	
	v2=GetSpeed2();
	
	v1=GetSpeed1();
	
	
	Balance_Pwm =Balance(Pitch,gyroy);	//平衡PID控制
	Velocity_Pwm=Velocity(v1,v2);		//速度环PID控制
	
	Turn_Pwm=Turn(gyroz);			//转向环PID控制
	
	Motor_1=Balance_Pwm+Velocity_Pwm+Turn_Pwm;
	Motor_2=Balance_Pwm+Velocity_Pwm-Turn_Pwm;
	
	Motor_1=PWM_Limit(Motor_1,6900,-6900);
	Motor_2=PWM_Limit(Motor_2,6900,-6900);	//PWM限幅
	
		Motor1_SetSpeed(Motor_1);
		Motor2_SetSpeed(Motor_2);
}
void freertos_demo(void)		//系统入口
{    
    xTaskCreate((TaskFunction_t         )   start_task,
                (char *                 )   "start_task",
                (uint16_t               )   START_TASK_STACK_SIZE,
                (void *                 )   NULL,
                (UBaseType_t            )   START_TASK_PRIO,
                (TaskHandle_t *         )   &start_task_handler );
	  vTaskStartScheduler();
}

/**
 * @brief       用于创建任务线程、软件定时器、事件标志组、信号量、消息队列等
 * @param       无
 * @retval      无
 */
void start_task( void * pvParameters )	//
{
    taskENTER_CRITICAL();               /* 进入临界区 */   
		semphore_handle = xSemaphoreCreateBinary();                     //创建二值信号量
    if(semphore_handle != NULL)
    {
       // printf("二值信号量创建成功！！！\r\n");
    }	
    /*1MS周期定时器 */
    timer_1000ms_handle = xTimerCreate( "1000ms_timer", 
                                    1000,
                                    pdTRUE,
                                    (void *)1,
                                    timer_1000ms_callback );
		/* MPU6050 5MS周期定时器 */																
		timer_MPU6050_GET_10ms_Handle = xTimerCreate( "timer_MPU6050_GET_10ms", 
														5,
														pdTRUE,
														(void *)2,
														timer_MPU6050_GET_10ms_callback ); 
		/* HC_05 5MS周期定时器 */	
		timer_HC_05_GET_10ms_Handle = xTimerCreate( "timer_HC_05_GET_10ms", 
														5,
														pdTRUE,
														(void *)2,
														timer_HC_05_GET_10ms_callback ); 
														
//		/* PID 10MS周期定时器 */																
//		timer_PID_Ctrl_10ms_Handle = xTimerCreate( "timer_PID_Ctrl_10ms", 
//														10,
//														pdTRUE,
//														(void *)3,
//														timer_PID_Ctrl_10ms_callback ); 												
														
		eventgroup_handle = xEventGroupCreate();
																						
    xTaskCreate((TaskFunction_t         )   task1,
                (char *                 )   "task1",
                (uint16_t							  )   TASK1_STACK_SIZE,
                (void *                 )   NULL,
                (UBaseType_t            )   TASK1_PRIO,
                (TaskHandle_t *         )   &task1_handler );					//主任务
								
		xTaskCreate((TaskFunction_t         )   task2,
                (char *                 )   "task2",
                (uint16_t							  )   TASK2_STACK_SIZE,
                (void *                 )   NULL,
                (UBaseType_t            )   TASK2_PRIO,
                (TaskHandle_t *         )   &task2_handler );					//屏幕任务			
		xTaskCreate((TaskFunction_t         )   task3,
                (char *                 )   "task3",
                (uint16_t							  )   TASK3_STACK_SIZE,
                (void *                 )   NULL,
                (UBaseType_t            )   TASK3_PRIO,
                (TaskHandle_t *         )   &task3_handler );   				//蓝牙任务					
								
    vTaskDelete(NULL);
    taskEXIT_CRITICAL();                /* 退出临界区 */
}

void task1( void * pvParameters )
{
	/*开启软件定时器*/
		xTimerStart(timer_1000ms_handle,portMAX_DELAY);    
		xTimerStart(timer_MPU6050_GET_10ms_Handle,portMAX_DELAY);	
	/*用于获取事件标志位*/
		EventBits_t event_bit = 0;
    while(1) 
    {
			event_bit = xEventGroupWaitBits( eventgroup_handle,           /* 事件标志组句柄 */
										 EVENT_ALL,                 /* 等待事件标志组的bit0和bit1位 */
                                         pdTRUE,                    /* 成功等待到事件标志位后，清除事件标志组中的bit0和bit1位 */
                                         pdFALSE,                   /* pdFALSE 或 ; pdTRUE 与 即所有标志位满足才行*/
                                         0 );           /* 死等 */
			if(event_bit & PID_CTRL_EVT)
			{
				GET_Data();			//获取mpu6050数据
				pid_ctrl();			//pid入口
				xSemaphoreGive(semphore_handle);
				
			}
    }
}

void task2(void * pvParameters)
{
	BaseType_t err;
	while(1)
	{
		err = xSemaphoreTake(semphore_handle,portMAX_DELAY); /* 获取信号量并死等 */
		if(err == pdTRUE)
		{
//				oled_show();
			OLED_ShowFloat(2,1,Pitch);
			
//			OLED_ShowSignedNum(3,1,s1,4);
//			OLED_ShowSignedNum(4,1,s2,4);
		}		
	}
}

void task3(void *pvParameters){
							//蓝牙标志位
	while(1){
		if (Serial_GetRxFlag() == 1)			//检查串口接收数据的标志位
		{
			RxData = Serial_GetRxData();		//获取串口接收的数据
			/*if(RxData == 'X') OLED_ShowString(1,6,"Mode 1");
			if(RxData == 'Y') OLED_ShowString(1,6,"Mode 2");
			if(RxData == 'Z') OLED_ShowString(1,6,"Mode 3");
			if(RxData == 'A') OLED_ShowString(1,6,"forward");
			if(RxData == 'E') OLED_ShowString(1,6,"backward");
			if(RxData == 'G') OLED_ShowString(1,6,"left");
			if(RxData == 'C') OLED_ShowString(1,6,"right");
			if(RxData == '1'||RxData == '2'||RxData == '3'||RxData == '4'||RxData == '5') OLED_ShowChar(1,1,RxData);
			Serial_SendByte(RxData);			//串口将收到的数据回传回去，用于测试
//			OLED_ShowHexNum(4, 2, RxData, 2);	//显示串口接收的数据*/
		}
	}
}

static void Get_Angle(void)
{ 
	mpu_dmp_get_data(&Pitch,&Roll,&Yaw);	
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	   
}

void GET_Data(void)
{
		Get_Angle();
//		v2=my_GetSpeed2();
//		v1=my_GetSpeed1();
		s1+=GetSpeed1();
		s2+=GetSpeed2();
}
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

void oled_show(void)
{
//		xxx++;
//		OLED_ShowNum(1,1,xxx,3);
		

}

/*LED 1000ms超时回调函数 */
void timer_1000ms_callback( TimerHandle_t pxTimer )
{
    xEventGroupSetBits( eventgroup_handle, LED_FLASH_EVT); 
	
}
/* MPU6050获取角度 10ms的超时回调函数 */
void timer_MPU6050_GET_10ms_callback( TimerHandle_t pxTimer )
{
  xEventGroupSetBits( eventgroup_handle, PID_CTRL_EVT); 
}
/* HC_05 10ms的超时回调函数 */
void timer_HC_05_GET_10ms_callback( TimerHandle_t pxTimer )
{
  xEventGroupSetBits( eventgroup_handle, PID_CTRL_EVT); 
}
//void timer_PID_Ctrl_10ms_callback ( TimerHandle_t pxTimer )
//{
//	xEventGroupSetBits( eventgroup_handle,PID_CTRL_EVT); 
//}
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init();
	Serial_Init();
	OLED_Init();
	MPU_Init();
	mpu_dmp_init();
	Motor_Init();
	Encoder1_Init();
	Encoder2_Init();
	OLED_ShowChar(1,1,'A');
	freertos_demo();//freertos入口
	return 0;
}
/*	OLED_ShowNum(1,1,1,1);
	HC05_SendString("success");
	uint8_t RxData;							//接收蓝牙指令
	uint8_t RxSTA=1;							//蓝牙标志位
	while(1){
	RxData = Serial_GetRxData();			
	Serial_SendByte(RxData);
	OLED_ShowNum(2,1,RxData,2);
	}*/
