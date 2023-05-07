/*
 * compass.h
 *
 *  Created on: 17 Mar 2023
 *      Author: sahin
 */

#ifndef SRC_COMPASS_H_
#define SRC_COMPASS_H_
#include "main.h"
#include "mpu6050.h"




extern MPU6050_t MPU6050_1;
extern double angle_pitch_output, angle_roll_output , angle_yaw_rate_output;
extern char buffer[25];
extern int _channels[12];
extern uint8_t gps_buffer[100];
extern int16_t MagX,MagY,MagZ;
extern uint8_t data_compass[13];
extern uint8_t MAGZ[6];
extern float heading;
extern HAL_StatusTypeDef result_compass;
extern int16_t compass_offset_y;                              //Add the y-offset to the raw value.
extern float compass_scale_y;                               //Scale the y-value so it matches the other axis.
extern int16_t compass_offset_z;                              //Add the z-offset to the raw value.
extern float compass_scale_z;                               //Scale the z-value so it matches the other axis.
extern int16_t compass_offset_x;
extern int16_t compass_cal_values[6];
extern int compass_calibrated;
extern float Xsf,Ysf,Xoff,Yoff;
extern I2C_HandleTypeDef hi2c3;
void i2c_disconnected_compass();
void compass_read_corrected();
void compass_read();
void compas_calibrate();
int compass_connect();


#endif /* SRC_COMPASS_H_ */
