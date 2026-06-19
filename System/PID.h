#ifndef __PID_H
#define __PID_H
#include "sys.h"
#include "Config.h"

/* PID 控制器结构体 */
typedef struct {
    float Kp, Ki, Kd;          /* 比例、积分、微分系数（×100 放大约定） */
    float error;               /* 当前误差 */
    float last_error;          /* 上一次误差 */
    float integral;            /* 积分累积项 */
    float output;              /* 输出 */
    float output_max;          /* 输出上限 */
    float output_min;          /* 输出下限 */
} PID_Controller_t;

/* 姿态数据结构体 */
typedef struct {
    float Pitch, Roll, Yaw;      /* 角度 */
    short gyrox, gyroy, gyroz;   /* 陀螺仪角速度 */
} Attitude_t;

/* 运动数据结构体 */
typedef struct {
    int speed_left, speed_right;       /* 左右轮当前速度 */
    int distance_left, distance_right; /* 左右轮累计行程 */
} Motion_t;

/* PID 初始化 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max, float min);
/* PID 统一计算接口 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement);

int Balance(float Angle,float Gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
int PWM_Limit(int IN,int max,int min);
#endif
