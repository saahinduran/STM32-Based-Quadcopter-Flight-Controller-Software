///*
// * mpu6050_yedek.h
// *
// *  Created on: Dec 2, 2022
// *      Author: sahin
// */
//
//#ifndef SRC_MPU6050_YEDEK_H_
//#define SRC_MPU6050_YEDEK_H_
//
//
///*
// * mpu6050.h
// *
// *  Created on: Nov 22, 2022
// *      Author: sahin
// */
//
//#ifndef SRC_MPU6050_H_
//#define SRC_MPU6050_H_
//
//
//#include <stdint.h>
//#include "main.h"
//#include <stdbool.h>
//
//// MPU6050 structure
//typedef struct {
//
//    int16_t Accel_X_RAW;
//    int16_t Accel_Y_RAW;
//    int16_t Accel_Z_RAW;
//    double Ax;
//    double Ay;
//    double Az;
//
//    int16_t Gyro_X_RAW;
//    int16_t Gyro_Y_RAW;
//    int16_t Gyro_Z_RAW;
//    double Gx;
//    double Gy;
//    double Gz;
//
//    float Temperature;
//
//    double  yaw_calibration_value;
//    double roll_calibration_value;
//    double  pitch_calibration_value;
//
//    double acc_total_vector ,angle_pitch_acc ,  angle_roll_acc ;
//    double angle_pitch,angle_roll,angle_yaw;
//    float angle_pitch_acc_calib,angle_roll_acc_calib;
//
//
//} MPU6050_t;
//
//
//
//bool final_calibration (I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
//uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
//
//void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
//void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
//void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
//void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
////double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
//void MPU6050_Read_All_DMA(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//void butterworth_2nd(float *x,float *y,float angle);
//bool calibrate_yaw(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//bool calibrate_ACC(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//uint8_t MPU6050_Init_Benim(I2C_HandleTypeDef *I2Cx);
//void MPU6050_Read_All_Benim(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
//
//#endif /* SRC_MPU6050_H_ */
//
//
//
//
//#endif /* SRC_MPU6050_YEDEK_H_ */
