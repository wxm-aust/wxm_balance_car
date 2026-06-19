#ifndef __PID_H
#define __PID_H
#include "sys.h"
#define Middle_angle 0
int Balance(float Angle,float Gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
int PWM_Limit(int IN,int max,int min);
#endif
