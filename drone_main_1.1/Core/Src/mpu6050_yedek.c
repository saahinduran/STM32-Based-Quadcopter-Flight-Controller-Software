///*
//*************************
//*                       *
//* İbrahim Cahit Özdemir *
//*                       *
//*     October 2021      *
//*                       *
//*************************
//*/
//
//#include <math.h>
//#include "mpu6050.h"
//#include "main.h"
//#include <stdbool.h>
//
//
//#define RAD_TO_DEG 57.295779513082320876798154814105
//#define ACCELERATION_RATE 16384.0f
//#define ACCELERATION_RATE_Z 17500.0f
//#define GYRO_RATE 131.0f
//
//#define WHO_AM_I_REG 0x75
//#define PWR_MGMT_1_REG 0x6B
//#define SMPLRT_DIV_REG 0x19
//#define ACCEL_CONFIG_REG 0x1C
//#define ACCEL_XOUT_H_REG 0x3B
//#define TEMP_OUT_H_REG 0x41
//#define GYRO_CONFIG_REG 0x1B
//#define GYRO_XOUT_H_REG 0x43
//#define MPU6050_TIMEOUT 100
//
//// Setup MPU6050
//#define MPU6050_ADDR 0xD0
//float my_alpha=0.9996;
//bool set_gyro_angles;
//float dF=500;
//
//float y_pitch[10] = {0};
//float y_roll[10] = {0};
//float y_yaw[2]={0};
//
//
//extern double angle_pitch_output, angle_roll_output;
//extern TIM_HandleTypeDef htim1;
//
//
//
//
//uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
//    uint8_t check;
//    uint8_t Data;
//
//    // check device ID WHO_AM_I
//
//    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, MPU6050_TIMEOUT);
//
//    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
//    {
//        // power management register 0X6B we should write all 0's to wake the sensor up
//        Data = 0;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, MPU6050_TIMEOUT);
//
//        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
//        Data = 0x07;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, MPU6050_TIMEOUT);
//
//        // Set accelerometer configuration in ACCEL_CONFIG Register
//        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
//        Data = 0x00;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, MPU6050_TIMEOUT);
//
//        // Set Gyroscopic configuration in GYRO_CONFIG Register
//        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
//        Data = 0x00;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, MPU6050_TIMEOUT);
//
//        // DEĞİŞİKLİK BAŞLIYOR
//        // interrupt config
///*
//        Data = (0x01<<7) | (0x01<<5) ;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x37, 1, &Data, 1, i2c_timeout);
//
//        // data ready interrupt config
//        Data = 1 ;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x38, 1, &Data, 1, i2c_timeout);
//*/
//        // DEĞİŞİKLİK BİTİYOR
//        return 0;
//    }
//    return 1;
//}
//
//
//
//
//void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
//    uint8_t Rec_Data[14];
//    int16_t temp;
//
//    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
//
//    if(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, MPU6050_TIMEOUT)!=HAL_OK)
//    	i2c_disconnected();
//
//
//    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
//    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
//    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
//    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
//    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
//    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
//    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);
//
//    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCELERATION_RATE;
//    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCELERATION_RATE;
//    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCELERATION_RATE_Z; // burayı yapmayı unutma
//    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
//    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_RATE; // buralar 131di
//    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_RATE;
//    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_RATE;
//
//    DataStruct->Gx = DataStruct->Gx - DataStruct->pitch_calibration_value;
//    DataStruct->Gy = DataStruct->Gy - DataStruct->roll_calibration_value;
//    DataStruct->Gz = DataStruct->Gz - DataStruct->yaw_calibration_value;
//
//    DataStruct->acc_total_vector = sqrt((DataStruct->Ax*DataStruct->Ax)+(DataStruct->Ay*DataStruct->Ay)+(DataStruct->Az*DataStruct->Az));
//
//        DataStruct->angle_pitch_acc = asin((float)DataStruct->Ay/DataStruct->acc_total_vector)* RAD_TO_DEG;
//        DataStruct->angle_roll_acc = asin((float)DataStruct->Ax/DataStruct->acc_total_vector)* -RAD_TO_DEG;
//
//
//        DataStruct->angle_pitch_acc-=DataStruct->angle_pitch_acc_calib;
//    	DataStruct->angle_roll_acc-=DataStruct->angle_roll_acc_calib;
//
//
//    //DataStruct->Yaw = DataStruct->Yaw + (DataStruct->Gz)/0.7e3;
///*
//    // Kalman angle solve
//    double dt = (double) (HAL_GetTick() - timer) / 1000;
//    timer = HAL_GetTick();
//    double roll;
//    double roll_sqrt = sqrt(
//            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
//    if (roll_sqrt != 0.0) {
//        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
//    } else {
//        roll = 0.0;
//    }
//    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
//    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
//        KalmanY.angle = pitch;
//        DataStruct->KalmanAngleY = pitch;
//    } else {
//        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
//    }
//    if (fabs(DataStruct->KalmanAngleY) > 90)
//        DataStruct->Gx = -DataStruct->Gx;
//    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);
//
//    // değişiklik başlıyor
//    butterworth_2nd(x_pitch,y_pitch,DataStruct->KalmanAngleX);
//    butterworth_2nd(x_roll,y_roll,DataStruct->KalmanAngleY);
//    butterworth_2nd(x_yaw,y_yaw,DataStruct->Gz);
//    y_yaw[0]=y_yaw[0]-DataStruct->yaw_calibration_value;
//
//    // değişiklik bitiyor
//*/
//}
///*
//double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
//    double rate = newRate - Kalman->bias;
//    Kalman->angle += dt * rate;
//
//    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
//    Kalman->P[0][1] -= dt * Kalman->P[1][1];
//    Kalman->P[1][0] -= dt * Kalman->P[1][1];
//    Kalman->P[1][1] += Kalman->Q_bias * dt;
//
//    double S = Kalman->P[0][0] + Kalman->R_measure;
//    double K[2];
//    K[0] = Kalman->P[0][0] / S;
//    K[1] = Kalman->P[1][0] / S;
//
//    double y = newAngle - Kalman->angle;
//    Kalman->angle += K[0] * y;
//    Kalman->bias += K[1] * y;
//
//    double P00_temp = Kalman->P[0][0];
//    double P01_temp = Kalman->P[0][1];
//
//    Kalman->P[0][0] -= K[0] * P00_temp;
//    Kalman->P[0][1] -= K[0] * P01_temp;
//    Kalman->P[1][0] -= K[1] * P00_temp;
//    Kalman->P[1][1] -= K[1] * P01_temp;
//
//    return Kalman->angle;
//};
//*/
//bool calibrate_yaw(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
//{
//	int i;
//	double summationZ=0;
//	double summationX=0;
//	double summationY=0;
//	for (i=0;i<2000;i++)
//	{
//		MPU6050_Read_All(I2Cx,DataStruct);
//		summationZ+=DataStruct->Gz;
//		summationX+=DataStruct->Gx;
//		summationY+=DataStruct->Gy;
//
//
//	}
//	DataStruct->yaw_calibration_value=summationZ/(double)i;
//	DataStruct->pitch_calibration_value=summationX/(double)i;
//	DataStruct->roll_calibration_value=summationY/(double)i;
//	return 1;
//}
//
//bool calibrate_ACC(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
//{
//	int i;
//	double summationRoll=0;
//	double summationPitch=0;
//	for (i=0;i<2000;i++)
//	{
//		MPU6050_Read_All(I2Cx,DataStruct);
//		summationRoll+=DataStruct->angle_roll_acc;
//		summationPitch+=DataStruct->angle_pitch_acc;
//
//
//	}
//	DataStruct->angle_pitch_acc_calib=round((summationPitch/(double)i)*10)/10;
//	DataStruct->angle_roll_acc_calib=round((summationRoll/(double)i)*10)/10;
//	return 1;
//}
//
//void butterworth_2nd(float *x,float *y,float angle)
//{
//	x[0]=angle;
//	y[0] = (float)1.93372357*y[1] + (float)-0.93585051*y[2] +
//			(float) 0.00053173*x[0] + (float) 0.00106347*x[1] + (float) 0.00053173*x[2];
//
//	for(int i = 1; i >= 0; i--)
//	{
//		 x[i+1] = x[i]; // store xi
//		 y[i+1] = y[i]; // store yi
//	}
//}
//
//uint8_t MPU6050_Init_Benim(I2C_HandleTypeDef *I2Cx) {
//    uint8_t check;
//    uint8_t Data;
//
//    // check device ID WHO_AM_I
//
//    while(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, MPU6050_TIMEOUT)!=HAL_OK)
//    	i2c_disconnected();
//
//    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
//    {
//        // power management register 0X6B we should write all 0's to wake the sensor up
//        Data = 0;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, MPU6050_TIMEOUT);
//
//        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
//        Data = 0x7; // edit var, eski değeri 07di.
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, MPU6050_TIMEOUT);
//
//        // Set accelerometer configuration in ACCEL_CONFIG Register
//        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
//        Data = 0x00;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, MPU6050_TIMEOUT);
//
//        // Set Gyroscopic configuration in GYRO_CONFIG Register
//        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
//        Data = 0x00;
//        // Data = 0x00 | 1<<3;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, MPU6050_TIMEOUT);
//
//        //DLPF AÇ
//
//        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, 0x1A, 1, &Data, 1, MPU6050_TIMEOUT);
//        Data= Data | 0x06;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1A, 1, &Data, 1, MPU6050_TIMEOUT);
//
//
//
//
//
//        return 0;
//    }
//    return 1;
//}
//
//void MPU6050_Read_All_Benim(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
//    uint8_t Rec_Data[14];
//    int16_t temp;
//
//    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
//
//    while(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, MPU6050_TIMEOUT)!=HAL_OK)
//        	i2c_disconnected();
//
//    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
//    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
//    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
//    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
//    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
//    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
//    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);
//
//    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCELERATION_RATE;
//    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCELERATION_RATE;
//    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCELERATION_RATE_Z;
//
//    //Diferansiyel açı değişimi
//    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
//    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_RATE; // buralar 131'di
//    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_RATE;
//    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_RATE;
//
//    //Offset değerlerini çıkar
//    DataStruct->Gx = DataStruct->Gx - DataStruct->pitch_calibration_value;
//    DataStruct->Gy = DataStruct->Gy - DataStruct->roll_calibration_value;
//    DataStruct->Gz = DataStruct->Gz - DataStruct->yaw_calibration_value;
//
//    //Integral işleminin yapıldığı yer   //dF= Frekans
//
//    DataStruct->angle_pitch += (float)DataStruct->Gx/dF;
//    DataStruct->angle_roll += (float)DataStruct->Gy/dF;
//    DataStruct->angle_yaw += (float)DataStruct->Gz/dF;
//
//    // Pitch ve Roll'a YAW Düzeltmesi
//    //DataStruct->angle_roll-=DataStruct->angle_pitch * sin((DataStruct->Gz/dF)*M_PI/180);
//    //DataStruct->angle_pitch+=DataStruct->angle_roll * sin((DataStruct->Gz/dF)*M_PI/180);
//
//    DataStruct->acc_total_vector = sqrt((DataStruct->Ax*DataStruct->Ax)+(DataStruct->Ay*DataStruct->Ay)+(DataStruct->Az*DataStruct->Az));
//
//    DataStruct->angle_pitch_acc = asin((float)DataStruct->Ay/DataStruct->acc_total_vector)* RAD_TO_DEG;
//    DataStruct->angle_roll_acc = asin((float)DataStruct->Ax/DataStruct->acc_total_vector)* -RAD_TO_DEG;
//
//
//    DataStruct->angle_pitch_acc-=DataStruct->angle_pitch_acc_calib;
//	DataStruct->angle_roll_acc-=DataStruct->angle_roll_acc_calib;
//// yedek değerler 0.9992 0.0008
//	//0.0001446875
//
//
//    if(set_gyro_angles){                                                 //If the IMU is already started
//    	DataStruct->angle_pitch = DataStruct->angle_pitch * my_alpha + DataStruct->angle_pitch_acc * (1-my_alpha);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
//    	DataStruct->angle_roll = DataStruct->angle_roll * my_alpha + DataStruct->angle_roll_acc * (1-my_alpha);        //Correct the drift of the gyro roll angle with the accelerometer roll angle
//      }
//      else{                                                                //At first start
//    	  DataStruct->angle_pitch = DataStruct->angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
//    	  DataStruct->angle_roll = DataStruct->angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
//        set_gyro_angles = true;                                            //Set the IMU started flag
//      }
//
//
//
//
//
//    angle_pitch_output = angle_pitch_output * 0.9 + DataStruct->angle_pitch* 0.1 ;
//    angle_roll_output = angle_roll_output * 0.9 + DataStruct->angle_roll* 0.1 ;
//
//    // değişiklik başlıyor
///*
//    butterworth_2nd(x_pitch,y_pitch,angle_pitch_output);
//
//    butterworth_2nd(x_roll,y_roll,DataStruct->KalmanAngleY);
//    butterworth_2nd(x_yaw,y_yaw,DataStruct->Gz);
//    y_yaw[0]=y_yaw[0]-DataStruct->yaw_calibration_value;
//*/
//
//    // sadığın filtresini uygula
///*
//    angle_pitch_output_filtered=movingAvg(10,angle_pitch_output,y_pitch);
//    angle_roll_output_filtered=movingAvg(10,angle_roll_output,y_roll);
//    // değişiklik bitiyor
//    // dt ölç, yüksek doğrulukla
//    // imunun (gyro) konfigürasyonundan emin ol
//    // ivme sensörünün vektör boytunun 1 civarında olduğundan emin ol
//    // acceleronun çıkışına raw dataya low pass koy
//    // 10 örnekli moving average da uygulanabilir
//     * */
//
//    // gyronun ölçüm aralığını kesin artır!!!
//
//}
//
//
////burayı sonradan ekledim
//
//
///*// bu kod çalışmıyor !!!!!!!
//bool final_calibration (I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
//{
//		int i;
//		double summationRoll=0;
//		double summationPitch=0;
//		for (i=0;i<2000;i++)
//		{
//			MPU6050_Read_All_Benim(I2Cx,DataStruct);
//			summationRoll+=angle_roll_output;
//			summationPitch+=angle_pitch_output;
//
//
//		}
//		DataStruct->final_calib_pitch=summationPitch/(double)i;
//		DataStruct->final_calib_roll=summationRoll/(double)i;
//		return 1;
//
//}
//*/
