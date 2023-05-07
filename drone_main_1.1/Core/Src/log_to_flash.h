/*
 * log_to_flash.h
 *
 *  Created on: Feb 25, 2023
 *      Author: sahin
 */

#ifndef SRC_LOG_TO_FLASH_H_
#define SRC_LOG_TO_FLASH_H_

#include "main.h"

#define PACKET_SIZE 34
#define Sector5_Address 0x08020000   // - 0x0803 FFFF 128 Kbyte
#define Sector6_Address 0x08040000   // - 0x0805 FFFF 128 Kbyte
#define Sector7_Address 0x08060000   // - 0x0807 FFFF 128 Kbyte
#define Sector8_Address 0x08080000   // - 0x0809 FFFF 128 Kbyte
#define Sector9_Address 0x080A0000   // - 0x080B FFFF 128 Kbyte
#define Sector10_Address 0x080C0000  // - 0x080D FFFF 128 Kbyte
#define Sector11_Address 0x080E0000  // - 0x080F FFFF 128 Kbyte
#define Sector12_Address 0x080100000 // - 0x0811 FFFF 128 Kbyte
#define Sector13_Address 0x08120000  // - 0x0813 FFFF 128 Kbyte
#define Sector14_Address 0x08140000  // - 0x0815 FFFF 128 Kbyte
#define Sector15_Address 0x08160000  // - 0x0817 FFFF 128 Kbyte



void flash_write_calib_data_byte(uint8_t* data_ptr,uint16_t data_amount);
void flash_write_calib_data_four_byte(uint32_t* data_ptr,uint16_t data_amount);
void flash_write_calib_data_compass(uint32_t* data_ptr,uint16_t data_amount);
void read_calib_value(uint32_t Flash_address,uint8_t data_amount,float* calib_buffer);
void flash_init_for_log(void);
void flash_init_for_calibration(void);
void fill_calibration_buffer();
void fill_calibration_buffer_compass();
void fill_calibration_buffer_uint(uint32_t* memory_data_buffer, float* calibration_data_ptr);

uint8_t log_write(uint8_t *data, uint64_t *address_counter);



#endif /* SRC_LOG_TO_FLASH_H_ */
