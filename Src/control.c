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

float DstAngle,DstSpeed;		//设定角度、设定速度
float ActualAngle,ActualSpeed;	//实际测量角度、实际测量速度
float AngularVelocity;			//角速度
float Kp_Angle,Kd_Angle;		//角度控制PID参数
float Kp_Speed,Ki_Speed;		//速度控制PID参数

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
	int result;
	result = Kp_Angle*(DstAngle-ActualAngle) + Kd_Angle * angle_veloctity;
	return result;
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