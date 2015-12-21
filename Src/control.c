/**
 ****************************************************************************************
 *
 * @file control.c
 *
 * @brief 步进电机驱动控制算法.
 *
 * Copyright (C) sunsjw 2015
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>

#include "control.h"

float DstAngle,DstSpeed;		//设定角度、设定速度
float ActualAngle,ActualSpeed;	//实际测量角度、实际测量速度
float AngularVelocity;			//角速度
float Kp_Angle,Ki_Angle,Kd_Angle;		//角度控制PID参数
float Kp_Gyro,Ki_Gyro,Kd_Gyro;	//角速度控制PID参数
float Kp_Speed,Ki_Speed;		//速度控制PID参数
float wp;

/**
****************************************************************************************
* @brief  初始化A3976驱动芯片
* @parameter
* 	none
* @description
*  这个函数用来初始化A3976驱动芯片，设置为1/8步
*****************************************************************************************
*/
void InitControlChip(void)
{
	#ifdef A3976
	STEP_8();
	#endif
}

/**
****************************************************************************************
* @brief  初始化控制参数
* @parameter
* 	none
* @description
*  这个函数用来初始化控制参数
*****************************************************************************************
*/
void InitControlParam(void)
{
	//初始化PID控制参数
	Kp_Angle = 600;
	Ki_Angle = 0;
	Kd_Angle = 0;
	//内环PID
	Kp_Gyro = 60;
	Ki_Gyro = 15;
	Kd_Gyro = 0;
	Kp_Speed = 0;
	Ki_Speed = 0;
	//初始化目标参数
	DstAngle = -2;
	DstSpeed = 0;
	wp = 1;
}

/**
****************************************************************************************
* @brief  角度平衡算法
* @parameter
* 	angle:实际测量的角度
*	angle_veloctity：角速度
* @description
*  这个函数用来计算驱动步进电机角度平衡.直立PD控制
*****************************************************************************************
*/
int AngleBalanceCalc(float angle,float angle_veloctity)
{
	float result;
	float errAngle = angle - DstAngle;
	result = Kp_Angle*tan(wp*errAngle)*errAngle + Kd_Angle * angle_veloctity;
	return result;                 
}

float i_Angle;
float old_Angle;
int i_Gyro;
int old_Gyro;
/**
****************************************************************************************
* @brief  串级PID平衡算法
* @parameter
* 	angle:实际测量的角度
*	gyro：角速度
* @description
*  这个函数用来计算驱动步进电机平衡控制
*****************************************************************************************
*/
int FreqOut(float angle,int gyro)
{
	//角度外环
	float AngleOut;
	float errAngle = DstAngle - angle;
	i_Angle += errAngle;
	float d_Angle = angle - old_Angle;
	AngleOut = Kp_Angle * errAngle + Ki_Angle * i_Angle + Kd_Angle * d_Angle;
	old_Angle = angle;
	//角速度内环
	int GyroOut;
	int errGyro = AngleOut - gyro + 12;
	i_Gyro += errGyro;
	int d_Gyro = gyro - old_Gyro;
	GyroOut = Kp_Gyro * errGyro + Ki_Angle * i_Gyro + Kd_Gyro * d_Gyro;
	return GyroOut;
}


/**
****************************************************************************************
* @brief  速度平衡算法
* @parameter
*	wheel_speed:车轮的速度
* @description
*  这个函数用来计算驱动步进电机速度平衡.速度PI控制
*****************************************************************************************
*/
int SpeedBalanceCalc(int wheel_speed)
{
	int result;
	static float speed_last,speed_filter;
	static int speed_integral;
	speed_last = DstSpeed - wheel_speed;		//获取最新速度偏差 :目标速度-测量速度
	speed_filter *= 0.7;						//一阶低通滤波器 
	speed_filter += speed_last * 0.3;			//一阶低通滤波器
	speed_integral += speed_filter;				//积分出位移 积分时间：10ms
	
	result = Kp_Speed * speed_filter + Ki_Speed * speed_integral;
	return result;
}

/**
****************************************************************************************
* @brief  平衡控制总输出
* @parameter 无
* @description
*  这个函数用来计算驱动步进电机最后需要的频率
*****************************************************************************************
*/
int ControlOut()
{
	return AngleBalanceCalc(ActualAngle,AngularVelocity) + SpeedBalanceCalc(ActualSpeed);
}
