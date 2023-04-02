/*
 * gy63-i2c.h
 *
 *  Created on: Feb 10, 2023
 *      Author: Administrator
 */

#ifndef SRC_GY63_I2C_H_
#define SRC_GY63_I2C_H_
struct GY63_t {
	uint16_t C[8];
	uint32_t D1,D2;
	int32_t dT;
	int64_t TEMP;
	float float_TEMP;
	int32_t P;
	float float_P;
	int64_t OFF;
	int64_t SENS;
	float ALT;
	float GROUND;
	int64_t T2;
	int64_t OFF2;
	int64_t SENS2;

};


   //Register mapı bulan aasyalı kardeşimize teşekkürler ♥ 定义器件在IIC总线中的从地址


#define MS5611_RST     0x1E

#define MS5611_D1_OSR_256    0x40
#define MS5611_D1_OSR_512    0x42
#define MS5611_D1_OSR_1024   0x44
#define MS5611_D1_OSR_2048   0x46
#define MS5611_D1_OSR_4096   0x48

#define MS5611_D2_OSR_256    0x50
#define MS5611_D2_OSR_512    0x52
#define MS5611_D2_OSR_1024   0x54
#define MS5611_D2_OSR_2048   0x56
#define MS5611_D2_OSR_4096   0x58

#define MS5611_ADC_RD     0x00
#define MS5611_PROM_RD    0xA0
#define MS5611_PROM_CRC   0xAE

void MS5611_Initilize();
void MS5611_Reset();
void MS5611_ReadPROM();
void MS5611_ReadPressure();
void MS5611_ReadTemperature();
void MS5611_ReadAltitude1();
void MS5611_ReadAltitude2();
void MS5611_StartCommunication();
void MS5611_StopCommunication();
void MS5611_Request_Pressure();
void MS5611_Request_Temp();
void MS5611_setMyGround();
void baro_read();


#endif /* SRC_GY63_I2C_H_ */
