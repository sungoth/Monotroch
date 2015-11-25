/**
 ****************************************************************************************
 *
 * @file control.h
 *
 * @brief 步进电机驱动控制算法头文件.
 *
 * Copyright (C) sunsjw 2015
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef __CONTROL_H
#define __CONTROL_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "gpio.h"

#define DIRECTION_UP	GPIO_PIN_10		//上面方向
#define DIRECTION_DOWN	GPIO_PIN_9		//下面方向
#define STEP			GPIO_PIN_0		//步进驱动
#define MS1				GPIO_PIN_9
#define MS2				GPIO_PIN_8
	 
#define STEP_8()		\
do{	\
	HAL_GPIO_WritePin(GPIOF,MS1,GPIO_PIN_SET);	\
	HAL_GPIO_WritePin(GPIOF,MS2,GPIO_PIN_SET);	\
}while(0)

/** 
* @brief  定义小车的左右上下四个方向
*/	
#define LEFT	HAL_GPIO_WritePin(GPIOF,DIRECTION_UP,GPIO_PIN_RESET)
#define RIGHT	HAL_GPIO_WritePin(GPIOF,DIRECTION_UP,GPIO_PIN_SET)
#define FORWARD	HAL_GPIO_WritePin(GPIOF,DIRECTION_DOWN,GPIO_PIN_RESET)
#define BACKWARD	HAL_GPIO_WritePin(GPIOF,DIRECTION_DOWN,GPIO_PIN_RESET)

void InitA3967(void);
void InitControlParam(void);
int AngleBalanceCalc(float angle,float angle_veloctity);
int SpeedBalanceCalc(int wheel_speed);
int ControlOut(void);

#ifdef __cplusplus
}
#endif
#endif /*__ CONTROL_H */
