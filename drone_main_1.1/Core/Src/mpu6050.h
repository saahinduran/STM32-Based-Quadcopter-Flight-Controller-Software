/*
 * mpu6050.h
 *
 *  Created on: Nov 22, 2022
 *      Author: sahin
 */

#ifndef SRC_MPU6050_H_
#define SRC_MPU6050_H_


#include <stdint.h>
#include "main.h"
#include <stdbool.h>

// MPU6050 structure
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Accel_X_Raw_Offset,Accel_Y_Raw_Offset,Accel_Z_Raw_Rate;
    float Temperature;

    double  yaw_calibration_value;
    double roll_calibration_value;
    double  pitch_calibration_value;
    float temp_calibration_value;

    double acc_total_vector ,angle_pitch_acc ,  angle_roll_acc ;
    double angle_pitch,angle_roll,angle_yaw;
    float angle_pitch_acc_calib,angle_roll_acc_calib;
    float accel_in_z,velocity_in_z;
    double Accel_Z_Calib;


} MPU6050_t;



//void update_filter(float input);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);


bool calibrate_yaw(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
bool calibrate_ACC(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
uint8_t MPU6050_Init_Benim(I2C_HandleTypeDef *I2Cx);
void MPU6050_Read_All_Benim(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void calibrate_mpu6050();

#endif /* SRC_MPU6050_H_ */
