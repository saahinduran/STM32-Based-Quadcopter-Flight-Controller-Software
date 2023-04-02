/*
 * log_to_flash.c
 *
 *  Created on: Feb 25, 2023
 *      Author: sahin
 */
#include "log_to_flash.h"
#include "main.h"
#include "mpu6050.h"



extern float calibration_buffer_float[5];
extern float imu_calibration_values[5];
extern MPU6050_t MPU6050_1;
extern float Xsf,Ysf,Xoff,Yoff,Zsf,Zoff;

void flash_write_calib_data_byte(uint8_t* data_ptr,uint16_t data_amount){

	flash_init_for_calibration();

	int i=0;
	for(i=0;i<data_amount*4;i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (Sector15_Address+i), data_ptr[i]);
	}
	HAL_FLASH_Lock();
}

void flash_write_calib_data_four_byte(uint32_t* data_ptr,uint16_t data_amount){

	flash_init_for_calibration();
	int i=0;
	for(i=0;i<data_amount;i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (Sector15_Address+i*4), data_ptr[i]);
	}
	HAL_FLASH_Lock();
}
void flash_write_calib_data_compass(uint32_t* data_ptr,uint16_t data_amount){

	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_14, FLASH_VOLTAGE_RANGE_3);
	int i=0;
	for(i=0;i<data_amount;i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (Sector14_Address+i*4), data_ptr[i]);
	}
	HAL_FLASH_Lock();
}



void read_calib_value(uint32_t Flash_address,uint8_t data_amount,float* calib_buffer){
	int i;
	for(i=0;i<data_amount;i++){
		calib_buffer[i]=*(float*)(Flash_address+i*4);
	}
}
void flash_init_for_log(void){

	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_5, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_6, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_7, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_8, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_9, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_10, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_11, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_12, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_13, FLASH_VOLTAGE_RANGE_3);

}
void flash_init_for_calibration(void){

	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_15, FLASH_VOLTAGE_RANGE_3);

}
uint8_t log_write(uint8_t *data, uint64_t *address_counter){
	int i=0;
	for(i=0;i<PACKET_SIZE;i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (Sector5_Address+ (*address_counter)+i), data[i]);
	}
	*address_counter+=PACKET_SIZE;

	if(*address_counter==Sector15_Address){
		return 1;
	}
	return 0;
}

void fill_calibration_buffer()
{
	//Accel_X_Raw_Offset,Accel_Y_Raw_Offset,Accel_Z_Raw_Rate;

	calibration_buffer_float[0]=(float)MPU6050_1.pitch_calibration_value;
	calibration_buffer_float[1]=(float)MPU6050_1.roll_calibration_value;
	calibration_buffer_float[2]=(float)MPU6050_1.yaw_calibration_value;
	calibration_buffer_float[3]=(float)MPU6050_1.Accel_X_Raw_Offset;
	calibration_buffer_float[4]=(float)MPU6050_1.Accel_Y_Raw_Offset;
	calibration_buffer_float[5]=(float)MPU6050_1.Accel_Z_Raw_Rate;
}

void fill_calibration_buffer_compass()
{
	//Accel_X_Raw_Offset,Accel_Y_Raw_Offset,Accel_Z_Raw_Rate;
	calibration_buffer_float[0]=(float)Xsf;
	calibration_buffer_float[1]=(float)Ysf;
	calibration_buffer_float[2]=(float)Xoff;
	calibration_buffer_float[3]=(float)Yoff;
	calibration_buffer_float[4]=(float)Zsf;
	calibration_buffer_float[5]=(float)Zoff;
}


void fill_calibration_buffer_uint(uint32_t* memory_data_buffer, float* calibration_data_ptr)
{
	int i=0;
	for(i=0;i<8;i++){
		*(memory_data_buffer+i)=calibration_data_ptr[i];
	}

}
