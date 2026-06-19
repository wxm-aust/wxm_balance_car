#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include "OLED.h"

//直立环
//调用：Vertical(0,Angle,gyro_X)
extern uint8_t RxData;
extern float Pitch,Roll,Yaw,Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;

int Balance(float Angle,float Gyro)
{  
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//求出平衡的角度中值 和机械相关
	 Gyro_bias=0-Gyro; 
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

int Velocity(int encoder_left,int encoder_right)
{  
	  static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral,Target_Velocity;

	  //================遥控前进后退部分====================// 
		Target_Velocity = 500;
		if(RxData == 'A')    	Movement=Target_Velocity;	  //收到前进信号
		else if(RxData == 'E')	Movement=-Target_Velocity;  //收到后退信号
		else  Movement=0;	
		//if(RxData=='A'||RxData == 'E') Velocity_Kp=0;
   //=============速度PI控制器=======================//	
		Encoder_Least =(encoder_left+encoder_right)-0;                    //获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和） 
		Encoder_bias *= 0.7;		                                          //一阶低通滤波器       
		Encoder_bias += Encoder_Least*0.3;	                              //一阶低通滤波器，减缓速度变化 
		Encoder_Integral +=Encoder_bias;                                  //积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //积分限幅
		if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //积分限幅	
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //速度控制	
		return velocity;
}

int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//修改转向速度，请修改Turn_Amplitude即可
	//===================遥控左右旋转部分=================//
	 if(RxData == 'C')	        Turn_Target=-Turn_Amplitude;
	 else if(RxData == 'G')	  Turn_Target=Turn_Amplitude; 
	 else Turn_Target=0;
	 if(RxData == 'A'||RxData == 'E')  Kd=Turn_Kd;        
	 else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  //===================转向PD控制器=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100;//结合Z轴陀螺仪进行PD控制
	 return turn;								 				 //转向环PWM右转为正，左转为负
}

int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	if(abs((int)Pitch)>=50) OUT = 0;
	return OUT;
}
