#ifndef __CONFIG_H
#define __CONFIG_H

#include "sys.h"

/* PID 参数 (保留 ×100 放大约定, 取值来自当前 main.c) */
#define BALANCE_KP           48000   // 直立环比例系数
#define BALANCE_KD           150     // 直立环微分系数
#define VELOCITY_KP          22000   // 速度环比例系数
#define VELOCITY_KI          110     // 速度环积分系数
#define TURN_KP              4200    // 转向环比例系数
#define TURN_KD              0       // 转向环微分系数

/* 串级平衡环参数 (角度环外环 P + 角速度环内环 PID) */
#define BALANCE_OUTER_KP     200     // 角度环外环 P 系数, ×100 约定
#define BALANCE_INNER_KP     150     // 角速度环内环 P, ×100 约定
#define BALANCE_INNER_KI     0       // 角速度环内环 I, ×100 约定
#define BALANCE_INNER_KD     150     // 角速度环内环 D, ×100 约定, 复用原 Balance_Kd

/* 串级转向环参数 (偏航角外环 P + Z 轴角速度内环 PID) */
#define TURN_OUTER_KP        200     // 偏航角外环 P, ×100 约定
#define TURN_INNER_KP        4200    // Z 轴角速度内环 P, ×100 约定
#define TURN_INNER_KI        0       // Z 轴角速度内环 I
#define TURN_INNER_KD        0       // Z 轴角速度内环 D

/* 限幅 */
#define PWM_MAX              6900    // PWM 输出最大限幅
#define INTEGRAL_MAX         10000   // 积分项最大限幅

/* 阈值 */
#define MIDDLE_ANGLE                 0       // 机械中值角度
#define VELOCITY_INTEGRAL_THRESHOLD  200     // 速度环变速积分阈值, 编码器计数单位
#define BALANCE_CONVERGE_THRESHOLD   15.0f   // 直立环收敛角度阈值, 度, 超过则关闭速度积分
#define FALLDOWN_ANGLE               50      // 跌倒停机角度阈值, 度

/* 蓝牙指令映射 */
#define CMD_FORWARD          'A'     // 前进指令
#define CMD_BACKWARD         'E'     // 后退指令
#define CMD_LEFT             'G'     // 左转指令
#define CMD_RIGHT            'C'     // 右转指令

/* 运动目标 */
#define TARGET_VELOCITY      500     // 目标速度
#define TURN_AMPLITUDE       54      // 转向幅度

/* 控制周期 */
#define CONTROL_PERIOD_MS    5       // 控制周期, 毫秒

#endif
