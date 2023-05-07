/*
 * compass.c
 *
 *  Created on: 17 Mar 2023
 *      Author: sahin
 */
#include "compass.h"
#include "math.h"
#include "receiver.h"
#include "main.h"
#include "mpu6050.h"
#include "log_to_flash.h"
#define MAG_DEC 6       // MAGNETIC DECLINATION IN ANKARA IS 6 DEGREES
#define COMPASS_TIMEOUT 1

double MagX_float,MagY_float,MagZ_float;
float heading_corrected;
extern float calibration_buffer_float[5];
extern float Zsf,Zoff;
float Sf_Toplam=0;


/// this function rescues the hang i2c bus that is connected to compass
void i2c_disconnected_compass(){

	uint32_t data;
	HAL_I2C_DeInit(&hi2c3);
	data=GPIOA->MODER;
	data= data & ~0b11<<16;
	data= data | 0b01<<16;
	GPIOA->MODER=data;


	data=GPIOA->OTYPER;

	data = data & ~0b1<<8;
	GPIOA->OTYPER=data;

	data=GPIOA->AFR[1];
	data=  data & ~0b1111;
	GPIOA->AFR[1]=data;

	for(int i=0;i<14;i++) // either 9 or 14
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	}

	  HAL_I2C_Init(&hi2c3);
}
extern I2C_HandleTypeDef hi2c2;
void compass_read_corrected(){
	HAL_I2C_Mem_Read(&hi2c3, 0x3D,0x03,1,MAGZ, 6, COMPASS_TIMEOUT);
	MagX=MAGZ[0]<<8|MAGZ[1];
	MagZ=MAGZ[2]<<8|MAGZ[3];
	MagY=MAGZ[4]<<8|MAGZ[5];
	MagY=-1*MagY;
	MagX=-1*MagX;
	MagZ=-1*MagZ;

	MagX = Xsf* (MagX - Xoff);

	MagY = Ysf * (MagY - Yoff);

	MagZ = Zsf* (MagZ - Zoff);

	MagX_float = +(double)MagX*cos(angle_roll_output*M_PI/180)
	+(double)MagY*sin(angle_pitch_output*M_PI/180)*sin(angle_roll_output*M_PI/180)
	-(double)MagZ*cos(angle_pitch_output*M_PI/180)*sin(angle_roll_output*M_PI/180);

	MagY_float = (double)MagY*cos(angle_pitch_output*M_PI/180)
	-(double)MagZ*sin(angle_pitch_output*M_PI/180);

	 heading_corrected = atan2(MagX_float,MagY_float ) * 180 / M_PI + MAG_DEC;
	 if(heading_corrected<0) heading_corrected+=360;

}
void compass_read(){

	HAL_I2C_Mem_Read(&hi2c3, 0x3D,0x03,1,MAGZ, 6, COMPASS_TIMEOUT);
	MPU6050_Read_All_Benim(&hi2c2, &MPU6050_1);
	MagX=MAGZ[0]<<8|MAGZ[1];
	MagZ=MAGZ[2]<<8|MAGZ[3];
	MagY=MAGZ[4]<<8|MAGZ[5];
	MagY=-1*MagY;
	MagX=-1*MagX;
	MagZ=-1*MagZ;

}
void compas_calibrate()
{
	int compass_calibrated=0;
	compass_read();
	compass_cal_values[0] = MagX;
	compass_cal_values[1] = MagX;
	compass_cal_values[2] = MagY;
	compass_cal_values[3] = MagY;
	compass_cal_values[4] = MagZ;
	compass_cal_values[5] = MagZ;

	while(compass_calibrated==0){
		compass_read();

		if (MagX < compass_cal_values[0])compass_cal_values[0] = MagX;
		if (MagX > compass_cal_values[1])compass_cal_values[1] = MagX;
		if (MagY < compass_cal_values[2])compass_cal_values[2] = MagY;
		if (MagY > compass_cal_values[3])compass_cal_values[3] = MagY;
		if (MagZ < compass_cal_values[4])compass_cal_values[4] = MagZ;
		if (MagZ > compass_cal_values[5])compass_cal_values[5] = MagZ;

		Xsf =(float)(compass_cal_values[1]-compass_cal_values[0])/2.0;
		Ysf =(float)(compass_cal_values[3]-compass_cal_values[2])/2.0;
		Zsf =(float)(compass_cal_values[5]-compass_cal_values[4])/2.0;
		Xoff =(float)(compass_cal_values[1]+compass_cal_values[0])/2.0;
		Yoff =(float)(compass_cal_values[3]+compass_cal_values[2])/2.0;
		Zoff =(float)(compass_cal_values[4]+compass_cal_values[5])/2.0;
		Sf_Toplam=(float)(Xsf+Ysf+Zsf)/3.0;
		Xsf=Sf_Toplam/Xsf;
		Ysf=Sf_Toplam/Ysf;
		Zsf=Sf_Toplam/Zsf;

		if(!decode_receiver()){
			if(_channels[1]<200 && _channels[0]<200) compass_calibrated=1;
		}
	}

	fill_calibration_buffer_compass();
	flash_write_calib_data_compass(calibration_buffer_float,6);
}
int compass_connect(){
	while(HAL_I2C_Mem_Read(&hi2c3, 0x3D,0x00,1,data_compass, 13, 100)!=HAL_OK){
		i2c_disconnected_compass();
	}
	uint8_t data=0;

	data|=0b110<<3;
	HAL_I2C_Mem_Write(&hi2c3, 0x3C,0x00,1,&data, 1, COMPASS_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c3, 0x3D,0x00,1,data_compass, 13, COMPASS_TIMEOUT);
	return 0;

}
