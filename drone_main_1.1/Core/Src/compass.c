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
#define MAG_DEC 6
extern UART_HandleTypeDef huart7;
double MagX_float,MagY_float,MagZ_float;
extern TIM_HandleTypeDef htim3;
extern uint32_t time1,time2,timex;
float heading_corrected;
extern float calibration_buffer_float[5];
extern float Zsf,Zoff;
uint8_t calibration_buffer[26]={0};
uint8_t raw[4]={"Raw:"};
float Sf_Toplam=0;
/// bu fonksiyon hang kalan i2c hattini kurtarır, pusulaya BAGLI I2C'YI KURTARIR
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
	HAL_I2C_Mem_Read(&hi2c3, 0x3D,0x03,1,MAGZ, 6, 100);
	MagX=MAGZ[0]<<8|MAGZ[1];
	MagZ=MAGZ[2]<<8|MAGZ[3];
	MagY=MAGZ[4]<<8|MAGZ[5];
	MagY=-1*MagY;
	MagX=-1*MagX;
	MagZ=-1*MagZ;

	MagX = Xsf* (MagX - Xoff);

	MagY = Ysf * (MagY - Yoff);

	MagZ = Zsf* (MagZ - Zoff);

	/*
	MagX = Xsf* MagX + Xoff;

		MagY = Ysf * MagY + Yoff;

		//MagZ = MagZ + Zoff;
*/
	MagX_float = +(double)MagX*cos(angle_roll_output*M_PI/180)
	+(double)MagY*sin(angle_pitch_output*M_PI/180)*sin(angle_roll_output*M_PI/180)
	- (double)MagZ*cos(angle_pitch_output*M_PI/180)*sin(angle_roll_output*M_PI/180);

	MagY_float = (double)MagY*cos(angle_pitch_output*M_PI/180) - (double)MagZ*sin(angle_pitch_output*M_PI/180);



	 heading_corrected = atan2(MagX_float,MagY_float ) * 180 / M_PI + MAG_DEC;
	 if(heading_corrected<0) heading_corrected+=360;

}
void compass_read(){

	HAL_I2C_Mem_Read(&hi2c3, 0x3D,0x03,1,MAGZ, 6, 100);
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

		HAL_TIM_Base_Start(&htim3);
	while(compass_calibrated==0){
		htim3.Instance->CNT=0;
		time1=htim3.Instance->CNT;

		compass_read();
/*
		fill_compass_buffer();
		HAL_UART_Transmit(&huart7, raw, sizeof(raw),100);
		HAL_UART_Transmit(&huart7, calibration_buffer, sizeof(calibration_buffer),100);

*/
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

	timex=htim3.Instance->CNT;
	while((htim3.Instance->CNT-time1)<=4000);
	}

	fill_calibration_buffer_compass();
	flash_write_calib_data_compass(calibration_buffer_float,6);



}
int compass_connect(){
	while(HAL_I2C_Mem_Read(&hi2c3, 0x3D,0x00,1,data_compass, 13, 100)!=HAL_OK){
		i2c_disconnected_compass();
	}
	uint8_t data=0;

	//HAL_I2C_Mem_Read(&hi2c3, 0x3D,0x00,1,&data, 1, 100);
	data|=0b110<<3;
	HAL_I2C_Mem_Write(&hi2c3, 0x3C,0x00,1,&data, 1, 100);
	//HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	HAL_I2C_Mem_Read(&hi2c3, 0x3D,0x00,1,data_compass, 13, 100);
	return 0;

}


void fill_compass_buffer(){

	calibration_buffer[0]=(0)&0xFF;
	calibration_buffer[1]=(0)&0xFF;
	calibration_buffer[2]=',';
	calibration_buffer[3]=(0)&0xFF;
	calibration_buffer[4]=(0)&0xFF;
	calibration_buffer[5]=',';
	calibration_buffer[6]=(0)&0xFF;
	calibration_buffer[7]=(0)&0xFF;
	calibration_buffer[8]=',';
	calibration_buffer[9]=(0)&0xFF;
	calibration_buffer[10]=(0)&0xFF;
	calibration_buffer[11]=',';
	calibration_buffer[12]=(0)&0xFF;
	calibration_buffer[13]=(0)&0xFF;
	calibration_buffer[14]=',';
	calibration_buffer[15]=(0)&0xFF;
	calibration_buffer[16]=(0)&0xFF;
	calibration_buffer[17]=',';




	calibration_buffer[19]=(MagX>>8)&0xFF;
	calibration_buffer[18]=(MagX)&0xFF;
	calibration_buffer[20]=',';
	calibration_buffer[22]=(MagY>>8)&0xFF;
	calibration_buffer[21]=(MagY)&0xFF;
	calibration_buffer[23]=',';
	calibration_buffer[25]=(MagZ>>8)&0xFF;
	calibration_buffer[24]=(MagZ)&0xFF;
}


/*
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

		HAL_TIM_Base_Start(&htim3);
	while(compass_calibrated==0){
		htim3.Instance->CNT=0;
		time1=htim3.Instance->CNT;

		compass_read();
		fill_compass_buffer();
		HAL_UART_Transmit(&huart7, raw, sizeof(raw),100);
		HAL_UART_Transmit(&huart7, calibration_buffer, sizeof(calibration_buffer),100);


	if (MagX < compass_cal_values[0])compass_cal_values[0] = MagX;
	if (MagX > compass_cal_values[1])compass_cal_values[1] = MagX;
	if (MagY < compass_cal_values[2])compass_cal_values[2] = MagY;
	if (MagY > compass_cal_values[3])compass_cal_values[3] = MagY;
	if (MagZ < compass_cal_values[4])compass_cal_values[4] = MagZ;
	if (MagZ > compass_cal_values[5])compass_cal_values[5] = MagZ;

	Xsf = (float)(compass_cal_values[3] - compass_cal_values[2]) /
			(compass_cal_values[1] - compass_cal_values[0]);
	if(Xsf<1) Xsf=1;

	Ysf =(float)(compass_cal_values[1] - compass_cal_values[0]) /
			(compass_cal_values[3] - compass_cal_values[2]);
	if(Ysf<1) Ysf=1;
	Xoff = ((compass_cal_values[1] - compass_cal_values[0])/2 - compass_cal_values[1]) * Xsf;
	Yoff = ((compass_cal_values[3] - compass_cal_values[2])/2 - compass_cal_values[3]) * Ysf;
	Zsf = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);
	Zoff = ((float)compass_cal_values[5] - compass_cal_values[4]) / 2;

	if(!decode_receiver()){
		if(_channels[1]<200 && _channels[0]<200) compass_calibrated=1;
	}

	timex=htim3.Instance->CNT;
	while((htim3.Instance->CNT-time1)<=4000);
	}

	fill_calibration_buffer_compass();
	flash_write_calib_data_compass(calibration_buffer_float,6);



}
*/

