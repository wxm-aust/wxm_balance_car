# 基于 FreeRTOS 的 STM32 两轮自平衡小车

## 项目简介

本项目是一个基于 STM32F103C8T6 微控制器和 FreeRTOS 实时操作系统的两轮自平衡小车。通过 MPU6050 六轴传感器获取姿态数据，采用串级 PID 控制算法实现车体的直立平衡、速度控制和转向控制，并支持蓝牙遥控与 OLED 实时显示。
基于江协科技的标准库例程，非HAL库。

## 硬件平台

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控芯片 | STM32F103C8T6 | Cortex-M3, 72MHz, 64KB Flash, 20KB SRAM |
| IMU 传感器 | MPU6050 | 六轴陀螺仪+加速度计，I2C 通信 |
| 电机驱动 | TB6612 | 双路直流电机驱动 |
| 编码器 | 霍尔编码器 | 电机转速与行程测量 |
| 显示屏 | OLED 0.96寸 | I2C 接口，128x64 分辨率 |
| 蓝牙模块 | HC-05 | 串口透传，用于遥控 |
| 其他外设 | 蜂鸣器、光敏传感器、LED | 状态指示与辅助功能 |

## 软件架构

```
version 4.0(2)/
├── FreeRTOS/              # FreeRTOS V9.0.0 内核源码
│   ├── include/           # FreeRTOS 头文件及配置
│   ├── portable/          # 平台移植层（RVDS/ARM_CM3）
│   ├── croutine.c         # 协程
│   ├── event_groups.c     # 事件组
│   ├── list.c             # 链表
│   ├── queue.c            # 队列
│   ├── tasks.c            # 任务调度
│   └── timers.c           # 软件定时器
├── Hardware/              # 硬件驱动层
│   ├── mpu6050.c/h        # MPU6050 驱动
│   ├── inv_mpu.c/h        # MPU6050 DMP 运动库
│   ├── mpuiic.c/h         # MPU6050 I2C 通信
│   ├── Motor.c/h          # 电机驱动
│   ├── Encoder.c/h        # 编码器读取
│   ├── OLED.c/h           # OLED 显示屏驱动
│   ├── HC05.c/h           # 蓝牙模块驱动
│   ├── Buzzer.c/h         # 蜂鸣器驱动
│   ├── LED.c/h            # LED 驱动
│   ├── LightSensor.c/h    # 光敏传感器驱动
│   ├── PWM.c/h            # PWM 输出
│   └── exti.c/h           # 外部中断
├── System/                # 系统功能层
│   ├── Config.h           # 全局参数配置（PID、阈值、指令）
│   ├── PID.c/h            # PID 控制器（直立/速度/转向）
│   ├── Timer.c/h          # 硬件定时器配置
│   ├── delay.c/h          # 延时函数
│   ├── sys.c/h            # 系统基础函数
│   └── usart.c/h          # 串口通信
├── Library/               # STM32 标准外设库
├── Start/                 # 启动文件与 CMSIS
├── User/                  # 用户应用层
│   ├── main.c             # 主程序入口与任务定义
│   ├── stm32f10x_conf.h   # 外设库配置
│   └── stm32f10x_it.c/h   # 中断服务函数
└── Project.uvprojx        # Keil MDK 工程文件
```

## FreeRTOS 任务设计

| 任务 | 优先级 | 栈大小 | 功能描述 |
|------|--------|--------|----------|
| start_task | 1 | 128 | 初始化任务：创建子任务和队列后自删除 |
| task1（控制） | 3 | 256 | 5ms 周期采集 MPU6050 数据并执行 PID 控制 |
| task2（显示） | 1 | 256 | 通过队列接收姿态数据，刷新 OLED 显示 |
| task3（蓝牙） | 1 | 256 | 等待串口中断通知，处理蓝牙遥控指令 |

**任务同步机制**：
- 控制任务由 TIM4 硬件定时器中断通过 `ulTaskNotifyTake` 唤醒，保证严格的 5ms 控制周期
- 显示任务通过 `xQueueReceive` 阻塞等待姿态数据，避免丢帧
- 蓝牙任务通过任务通知阻塞等待串口接收中断

## 控制算法

### 串级 PID 控制

采用三环串级 PID 控制策略：

#### 1. 直立环（Balance）
- **外环**：角度环（P 控制），将 Pitch 角度偏差转换为目标角速度
- **内环**：角速度环（PID 控制），跟踪目标角速度，输出 PWM

#### 2. 速度环（Velocity）
- PI 控制，根据编码器反馈调节速度
- 积分分离：直立环未收敛时清零积分，防止积分饱和
- 变速积分：偏差大时减弱积分作用，提高稳定性

#### 3. 转向环（Turn）
- **外环**：偏航角环（P 控制），将角度偏差转换为目标 Z 轴角速度
- **内环**：Z 轴角速度环（PID 控制），输出转向 PWM

### 安全保护
- 跌倒检测：当 Pitch 角度超过 ±50° 时自动关闭电机输出
- PWM 输出限幅：防止电机过载
- 积分限幅：防止积分饱和

## 蓝牙遥控指令

| 指令 | 功能 |
|------|------|
| `A` | 前进 |
| `E` | 后退 |
| `G` | 左转 |
| `C` | 右转 |

## 关键参数配置

所有可调参数集中在 [Config.h](System/Config.h) 中：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `BALANCE_OUTER_KP` | 200 | 直立环角度外环 P |
| `BALANCE_INNER_KP` | 150 | 直立环角速度内环 P |
| `BALANCE_INNER_KD` | 150 | 直立环角速度内环 D |
| `VELOCITY_KP` | 22000 | 速度环 P |
| `VELOCITY_KI` | 110 | 速度环 I |
| `TURN_OUTER_KP` | 200 | 转向环偏航外环 P |
| `TURN_INNER_KP` | 4200 | 转向环角速度内环 P |
| `PWM_MAX` | 6900 | PWM 最大输出限幅 |
| `CONTROL_PERIOD_MS` | 5 | 控制周期（毫秒） |
| `FALLDOWN_ANGLE` | 50 | 跌倒停机角度阈值（度） |

## 开发环境

- **IDE**：Keil 
- **编译器**：ARM Compiler V5（RVDS）
- **RTOS**：FreeRTOS V9.0.0
- **下载器**：ST-Link 
- **目标芯片**：STM32F103C8

## 编译与烧录

1. 使用 Keil MDK-ARM V5 打开 `Project.uvprojx`
2. 确认目标芯片为 `STM32F103C8`
3. 编译工程（Build）
4. 通过 ST-Link 或串口烧录到目标板

## 使用说明

1. 上电后，小车进入初始化状态
2. 将小车竖直放置，待 MPU6050 DMP 初始化完成
3. 通过蓝牙连接 HC-05 模块，发送遥控指令
4. OLED 屏幕实时显示 Pitch 姿态角度
5. 小车倾斜超过 50° 时自动停机保护
