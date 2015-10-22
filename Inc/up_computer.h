/**
 ****************************************************************************************
 *
 * @file up_computer.h
 *
 * @brief Transmission data to the upper computer header file.
 *
 * Copyright (C) sunsjw 2015
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#include <stdint.h>

#ifndef __up_computer_H
#define __up_computer_H
#ifdef __cplusplus
 extern "C" {
#endif

#define u8	uint8_t
	 
void report_to_pc(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);

#ifdef __cplusplus
}
#endif
#endif /*__ up_computer_H */
