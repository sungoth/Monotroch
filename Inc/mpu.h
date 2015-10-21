/**
 ****************************************************************************************
 *
 * @file mpu.h
 *
 * @brief mpu6050 dmp header file.
 *
 * Copyright (C) sunsjw 2015
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 

#ifndef __mpu_H
#define __mpu_H
#ifdef __cplusplus
 extern "C" {
#endif

#define MPU_OK	0
#define MPU_ERR	-1

/** 
  * @brief  mpu6050 data structures definition
  */	 
typedef struct
{
	short gyro[3];
	short accel[3];
	float Pitch;
	float Roll;
	float Yaw;
}MPU_Data;
	 
void mpu_start(void);
int mpu_read(MPU_Data* data);
	 
	 
#ifdef __cplusplus
}
#endif
#endif /*__ mpu_H */

