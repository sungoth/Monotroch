/**
 ****************************************************************************************
 *
 * @file mpu.c
 *
 * @brief mpu6050 dmp helper.
 *
 * Copyright (C) sunsjw 2015
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#include <stdlib.h>
#include <stdio.h>
#include <arm_math.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu.h"

static signed char gyro_orientation[9] = {-1, 0, 0,
                                          0,-1, 0,
                                          0, 0, 1
                                         };

/**
****************************************************************************************
* @brief  start mpu6050 dmp
* @description
*  This function is used to initialize mpu6050 dmp.
*****************************************************************************************
*/
void mpu_start()
{
    int result = mpu_init(NULL);
	while(result != MPU_OK)
	{
		result = mpu_init(NULL);
	}
    if(result == MPU_OK)
    {
        printf("mpu initialization complete......\n ");	 	  //mpu_set_sensor
        if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) == MPU_OK)
            printf("mpu_set_sensor complete ......\n");
        else
            printf("mpu_set_sensor come across error ......\n");
        if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) == MPU_OK)	   	  //mpu_configure_fifo
            printf("mpu_configure_fifo complete ......\n");
        else
            printf("mpu_configure_fifo come across error ......\n");
        if(mpu_set_sample_rate(DEFAULT_MPU_HZ) == MPU_OK)	   	  //mpu_set_sample_rate
            printf("mpu_set_sample_rate complete ......\n");
        else
            printf("mpu_set_sample_rate error ......\n");
        if(dmp_load_motion_driver_firmware() == MPU_OK)   	  //dmp_load_motion_driver_firmvare
            printf("dmp_load_motion_driver_firmware complete ......\n");
        else
            printf("dmp_load_motion_driver_firmware come across error ......\n");
        if(dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)) == MPU_OK) 	  //dmp_set_orientation
            printf("dmp_set_orientation complete ......\n");
        else
            printf("dmp_set_orientation come across error ......\n");
        if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                              DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                              DMP_FEATURE_GYRO_CAL) == MPU_OK)		   	  //dmp_enable_feature
            printf("dmp_enable_feature complete ......\n");
        else
            printf("dmp_enable_feature come across error ......\n");
        if(dmp_set_fifo_rate(DEFAULT_MPU_HZ) == MPU_OK)   	  //dmp_set_fifo_rate
            printf("dmp_set_fifo_rate complete ......\n");
        else
            printf("dmp_set_fifo_rate come across error ......\n");
        run_self_test();
        if(mpu_set_dmp_state(1) == MPU_OK)
            printf("mpu_set_dmp_state complete ......\n");
        else
            printf("mpu_set_dmp_state come across error ......\n");
    }
}

/**
****************************************************************************************
* @brief  read data from mpu6050
* @description
*  This function is to use DMP to read data from mpu6050 sensor.
*****************************************************************************************
*/
int mpu_read(MPU_Data* data)
{

#define q30  	1073741824.0f
#define u8		uint8_t

    float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

    unsigned long sensor_timestamp;
    short sensors;
    unsigned char more;
    long quat[4];

    dmp_read_fifo(data->gyro, data->accel, quat, &sensor_timestamp, &sensors,&more);
    /* Gyro and accel data are written to the FIFO by the DMP in chip
    * frame and hardware units. This behavior is convenient because it
    * keeps the gyro and accel outputs of dmp_read_fifo and
    * mpu_read_fifo consistent.
    */

    /* Unlike gyro and accel, quaternions are written to the FIFO in
    * the body frame, q30. The orientation is set by the scalar passed
    * to dmp_set_orientation during initialization.
    */
    if (sensors & INV_WXYZ_QUAT )
    {
        q0=quat[0] / q30;
        q1=quat[1] / q30;
        q2=quat[2] / q30;
        q3=quat[3] / q30;
        data->Pitch  = asin(2 * q1 * q3 - 2 * q0* q2)* 57.3; // pitch
        data->Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
        data->Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
		return MPU_OK;
    }

    return MPU_ERR;
}

extern int dmp_set_gyro_bias(long *bias);
extern int dmp_set_accel_bias(long *bias);

void run_self_test(void)
{
    int result;
//    char test_packet[4] = {0};
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\n");
    }
	else
	{
		printf("bias has not been modified ......\n");
	}
}

 unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
 unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}
