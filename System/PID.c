#include "stm32f10x.h"
#include "PID.h"
#include <math.h>

extern uint8_t RxData;
extern Attitude_t attitude;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max, float min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
    pid->output_max = max;
    pid->output_min = min;
}

float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement)
{
    pid->error = setpoint - measurement;
    pid->integral += pid->error;
    /* 积分限幅 */
    if (pid->integral > INTEGRAL_MAX) pid->integral = INTEGRAL_MAX;
    if (pid->integral < -INTEGRAL_MAX) pid->integral = -INTEGRAL_MAX;
    /* PID 输出（×100 约定） */
    pid->output = pid->Kp / 100.0f * pid->error
                + pid->Ki / 100.0f * pid->integral
                + pid->Kd / 100.0f * (pid->error - pid->last_error);
    pid->last_error = pid->error;
    /* 输出限幅 */
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < pid->output_min) pid->output = pid->output_min;
    return pid->output;
}

int Balance(float Angle, float Gyro)
{
    static PID_Controller_t outer_pid, inner_pid;
    static uint8_t inited = 0;
    if (!inited) {
        PID_Init(&outer_pid, BALANCE_OUTER_KP, 0, 0, 0, 0);  /* 外环纯 P，不限制输出 */
        PID_Init(&inner_pid, BALANCE_INNER_KP, BALANCE_INNER_KI, BALANCE_INNER_KD, PWM_MAX, -PWM_MAX);
        inited = 1;
    }
    /* 外环：角度偏差 → 目标角速度 */
    /* 保持原符号约定：target_rate = -Kp_outer/100 * (MIDDLE_ANGLE - Angle) */
    float target_rate = -outer_pid.Kp / 100.0f * (MIDDLE_ANGLE - Angle);
    /* 内环：角速度偏差 → PWM。使用 -Gyro 作为测量值以匹配原 D 项符号 (+Kd*Gyro) */
    float pwm = PID_Compute(&inner_pid, target_rate, -Gyro);
    return (int)pwm;
}

int Velocity(int encoder_left, int encoder_right)
{
    static float velocity, Encoder_Least, Encoder_bias, Movement;
    static float Encoder_Integral;
    float Ki_factor;

    /* 遥控前向/后向 */
    if (RxData == CMD_FORWARD) Movement = TARGET_VELOCITY;
    else if (RxData == CMD_BACKWARD) Movement = -TARGET_VELOCITY;
    else Movement = 0;

    /* 速度计算 + 一阶低通滤波 */
    Encoder_Least = (encoder_left + encoder_right) - 0;
    Encoder_bias *= 0.7f;
    Encoder_bias += Encoder_Least * 0.3f;

    /* 积分分离：直立环未收敛时清零积分 */
    if (fabsf(attitude.Pitch) > BALANCE_CONVERGE_THRESHOLD) {
        Encoder_Integral = 0;
    } else {
        /* 变速积分：偏差大时减弱积分 */
        if (fabsf(Encoder_bias) > VELOCITY_INTEGRAL_THRESHOLD) {
            Ki_factor = 0;
        } else {
            Ki_factor = 1.0f - fabsf(Encoder_bias) / VELOCITY_INTEGRAL_THRESHOLD;
        }
        Encoder_Integral += Encoder_bias * Ki_factor;
        Encoder_Integral += Movement;
    }

    /* 积分限幅 */
    if (Encoder_Integral > INTEGRAL_MAX) Encoder_Integral = INTEGRAL_MAX;
    if (Encoder_Integral < -INTEGRAL_MAX) Encoder_Integral = -INTEGRAL_MAX;

    velocity = -Encoder_bias * VELOCITY_KP / 100.0f - Encoder_Integral * VELOCITY_KI / 100.0f;
    return (int)velocity;
}

int Turn(float gyro)
{
    static PID_Controller_t outer_pid, inner_pid;
    static uint8_t inited = 0;
    static float target_yaw = 0;
    static uint8_t was_turning = 0;
    float target_rate;
    float pwm;

    if (!inited) {
        PID_Init(&outer_pid, TURN_OUTER_KP, 0, 0, TURN_AMPLITUDE, -TURN_AMPLITUDE);
        PID_Init(&inner_pid, TURN_INNER_KP, TURN_INNER_KI, TURN_INNER_KD, PWM_MAX, -PWM_MAX);
        inited = 1;
    }

    if (RxData == CMD_LEFT) {
        target_rate = TURN_AMPLITUDE;
        was_turning = 1;
    } else if (RxData == CMD_RIGHT) {
        target_rate = -TURN_AMPLITUDE;
        was_turning = 1;
    } else {
        /* 无转向指令：航向锁定 */
        if (was_turning) {
            target_yaw = attitude.Yaw;  /* 释放指令时锁定当前航向 */
            was_turning = 0;
        }
        /* 外环：偏航角偏差 → 目标角速度 */
        target_rate = PID_Compute(&outer_pid, target_yaw, attitude.Yaw);
    }

    /* 内环：Z 轴角速度偏差 → 转向 PWM */
    pwm = PID_Compute(&inner_pid, target_rate, gyro);
    return (int)pwm;
}

int PWM_Limit(int IN, int max, int min)
{
    int OUT = IN;
    if (OUT > max) OUT = max;
    if (OUT < min) OUT = min;
    if (abs((int)attitude.Pitch) >= FALLDOWN_ANGLE) OUT = 0;
    return OUT;
}
