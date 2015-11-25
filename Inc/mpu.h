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
 

#ifndef __MPU_H
#define __MPU_H
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

void run_self_test(void);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
unsigned short inv_row_2_scale(const signed char *row);
	 
#ifdef __cplusplus
}
#endif
#endif /*__ MPU_H */

