#include "stm32f10x.h"                  // Device header
/*相关头文件*/
#include "OLED.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "Config.h"
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
#include "queue.h"


/**********************************************************************************/
/* START_TASK 任务 配置 */
#define START_TASK_PRIO         1
#define START_TASK_STACK_SIZE   128
TaskHandle_t    start_task_handler;
void start_task( void * pvParameters );
/**********************************************************************************/
/* TASK1 任务 配置 — 控制任务，独享最高优先级 */
#define TASK1_PRIO         3
#define TASK1_STACK_SIZE   256
TaskHandle_t    task1_handler;
void task1( void * pvParameters );
/**********************************************************************************/
/* TASK2 任务 配置 — 显示任务 */
#define TASK2_PRIO         1
#define TASK2_STACK_SIZE   256
TaskHandle_t    task2_handler;
void task2( void * pvParameters );
/**********************************************************************************/
/* TASK3 任务 配置 — 蓝牙任务 */
#define TASK3_PRIO         1
#define TASK3_STACK_SIZE   256
TaskHandle_t    task3_handler;
void task3( void * pvParameters );
/**********************************************************************************/
/*
 * 显示数据队列 — 替代二值信号量，避免丢帧
 */
QueueHandle_t display_queue_handle;
/**********************************************************************************/
/*全局数据结构体*/
Attitude_t attitude;          /* 姿态数据 */
Motion_t   motion;            /* 运动数据 */
uint8_t RxData;               /* 蓝牙接收数据 */
uint8_t RxSTA=1;              /* 接收标志位 */
/**********************************************************************************/
/*全局函数声明*/
static void Get_Angle(void);  /* 获取mpu6050角度值 */
void GET_Data(void);
void pid_ctrl(void);
/**********************************************************************************/

void pid_ctrl(void)
{
    int Motor_1, Motor_2;      /* 左右PWM输出 */
    int Balance_Pwm, Velocity_Pwm, Turn_Pwm;

    motion.speed_right = GetSpeed2();
    motion.speed_left  = GetSpeed1();

    Balance_Pwm  = Balance(attitude.Pitch, attitude.gyroy);   /* 平衡PID控制 */
    Velocity_Pwm = Velocity(motion.speed_left, motion.speed_right); /* 速度环PID控制 */
    Turn_Pwm     = Turn(attitude.gyroz);                      /* 转向PID控制 */

    Motor_1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
    Motor_2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm;

    Motor_1 = PWM_Limit(Motor_1, PWM_MAX, -PWM_MAX);
    Motor_2 = PWM_Limit(Motor_2, PWM_MAX, -PWM_MAX);          /* PWM限幅 */

    Motor1_SetSpeed(Motor_1);
    Motor2_SetSpeed(Motor_2);
}

void freertos_demo(void)        /* 系统启动 */
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
 * @brief       初始任务，创建所有线程、定时器、队列
 * @param       无
 * @retval      无
 */
void start_task( void * pvParameters )
{
    taskENTER_CRITICAL();               /* 进入临界区 */

    /* 创建显示数据队列，队列项为 float（Pitch），长度 5 */
    display_queue_handle = xQueueCreate(5, sizeof(float));
    if (display_queue_handle == NULL)
    {
        /* 队列创建失败 */
    }

    xTaskCreate((TaskFunction_t         )   task1,
                (char *                 )   "task1",
                (uint16_t               )   TASK1_STACK_SIZE,
                (void *                 )   NULL,
                (UBaseType_t            )   TASK1_PRIO,
                (TaskHandle_t *         )   &task1_handler );                 /* 控制任务 */

    xTaskCreate((TaskFunction_t         )   task2,
                (char *                 )   "task2",
                (uint16_t               )   TASK2_STACK_SIZE,
                (void *                 )   NULL,
                (UBaseType_t            )   TASK2_PRIO,
                (TaskHandle_t *         )   &task2_handler );                 /* 显示任务 */

    xTaskCreate((TaskFunction_t         )   task3,
                (char *                 )   "task3",
                (uint16_t               )   TASK3_STACK_SIZE,
                (void *                 )   NULL,
                (UBaseType_t            )   TASK3_PRIO,
                (TaskHandle_t *         )   &task3_handler );                 /* 蓝牙任务 */

    vTaskDelete(NULL);
    taskEXIT_CRITICAL();                /* 退出临界区 */
}

/**
 * @brief       控制任务 — 阻塞等待 TIM4 中断通知，执行数据采集与PID控制
 */
void task1( void * pvParameters )
{
    float pitch_to_send;
    while (1)
    {
        /* 阻塞等待 TIM4 硬件定时器中断的通知 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        GET_Data();             /* 获取mpu6050数据 */
        pid_ctrl();             /* pid控制 */

        /* 通过队列发送 Pitch 给显示任务 */
        pitch_to_send = attitude.Pitch;
        xQueueSend(display_queue_handle, &pitch_to_send, 0);
    }
}

/**
 * @brief       显示任务 — 阻塞等待队列数据，刷新OLED
 */
void task2(void * pvParameters)
{
    float pitch_received;
    while (1)
    {
        /* 阻塞等待队列中的姿态数据 */
        if (xQueueReceive(display_queue_handle, &pitch_received, portMAX_DELAY) == pdTRUE)
        {
            OLED_ShowFloat(2, 1, pitch_received);
        }
    }
}

/**
 * @brief       蓝牙任务 — 阻塞等待串口中断通知，处理蓝牙指令
 */
void task3(void *pvParameters)
{
    while (1)
    {
        /* 阻塞等待串口接收中断的通知 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 检查并处理接收到的数据 */
        if (Serial_GetRxFlag() == 1)
        {
            RxData = Serial_GetRxData();        /* 获取串口接收到的数据 */
        }
    }
}

static void Get_Angle(void)
{
    mpu_dmp_get_data(&attitude.Pitch, &attitude.Roll, &attitude.Yaw);
    MPU_Get_Gyroscope(&attitude.gyrox, &attitude.gyroy, &attitude.gyroz);
}

void GET_Data(void)
{
    Get_Angle();
    motion.distance_left  += GetSpeed1();
    motion.distance_right += GetSpeed2();
}

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
    Timer4_Control_Init();   /* 启动 TIM4 硬件定时器，5ms 触发控制任务 */
    freertos_demo();         /* freertos启动 */
    return 0;
}
