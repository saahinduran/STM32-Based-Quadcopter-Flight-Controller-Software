/*
 * gy63-i2c.c
 *
 *  Created on:  feb 10, 2023
 *      Author: Sadık
 *
 *
 *      m   m
 *      m m m
 *      m   m
 */


/* ps high
 * SDA pb9  CN7 sağ üsstten 2.
 * SCL pb6 CN7 sol alttan 11.
 * CSB VDD or GND. that results in slave address (CSB 0 ise slave address 0x77)  --- (CSB 1 ise slave address 0x76)
 * gnd
 * vcc
 * stm324f13zh --> GPIO A4 output for led
 */

/*
 * NASIL ÇALIŞACAK ALGORİTMA? 4ms (250hzde çalıştıgımı varsayıyorum)
 *
 * 1- mainde whiledan önce reset at. sonra PROM değerlerini oku
 * 2- counter pressure değişken tanımla ---> counter_pressure = 0;
 * 3- counter_temp=0; tanımla
 * 4- while a gel. if(counter_pressure%3 == 0) ise request_pressure at ve ife girerse counterı sıfırla.
 * 5- if kosulunun altında başka bir if kosulu olustur. if(counter_temperature%==15) ise request_temperature at
 *
 *
 *
 */
#include "main.h"
#include "math.h"
#include "gy63-i2c.h"
uint8_t tx_buf[3], rx_buf[3];

#define SEA_PRESSURE 1013.25f
struct GY63_t GY63;

extern I2C_HandleTypeDef hi2c1;
#define MS5611_SlaveAddress   0x77
extern int loop_counter_pressure_requested;
extern int temp_requested_flag;
extern int pressure_ready;
extern uint32_t loop_counter;
extern float baro_filter_input[4];extern float baro_filter_output[4];
extern float baro_filter_output_coeff[3];
extern float baro_filter_input_coeff[4];
void MS5611_Initilize(){
	  MS5611_Reset();
	  MS5611_ReadPROM();
	  //MS5611_Request_Temp();
	  //HAL_Delay(10);
	  //MS5611_ReadTemperature();
	  //MS5611_Request_Pressure();
	  //HAL_Delay(10);
	  //MS5611_ReadAltitude2();
	  MS5611_setMyGround();
}

void MS5611_Reset(){

	// Send the command to reset the sensor
	tx_buf[0] = 0x1E;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
	HAL_Delay(3); // min 2.8ms beklemek zorunda reset atarsan

}

void MS5611_StartCommunication(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void MS5611_StopCommunication(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void MS5611_ReadPROM(){
    // Send the command to read the PROM coefficients

	// ne oldugunu bilmediğim
	tx_buf[0] = 0xA0;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 2, 100);
	GY63.C[0]=(rx_buf[0] << 8) | rx_buf[1];
	HAL_Delay(10);

	// PRESSURE SENSİTİVİTY (SENST1)
    tx_buf[0] = 0xA2;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 2, 100);
    GY63.C[1]=(rx_buf[0] << 8) | rx_buf[1];
	HAL_Delay(10);

    // PRESSURE OFFSET (OFFT1)
    tx_buf[0] = 0xA4;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 2, 100);
    GY63.C[2]=(rx_buf[0] << 8) | rx_buf[1];
	HAL_Delay(10);

    // TCS
    tx_buf[0] = 0xA6;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 2, 100);
    GY63.C[3]=(rx_buf[0] << 8) | rx_buf[1];
	HAL_Delay(10);

    // TCO
    tx_buf[0] = 0xA8;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 2, 100);
    GY63.C[4]=(rx_buf[0] << 8) | rx_buf[1];
	HAL_Delay(10);

    // TREF
    tx_buf[0] = 0xAA;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 2, 100);
    GY63.C[5]=(rx_buf[0] << 8) | rx_buf[1];
	HAL_Delay(10);

    // TEMPSENS
    tx_buf[0] = 0xAC;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 2, 100);
    GY63.C[6]=(rx_buf[0] << 8) | rx_buf[1];
	HAL_Delay(10);

    // crc
    tx_buf[0] = 0xAE;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 2, 100);
    GY63.C[7]=(rx_buf[0] << 8) | rx_buf[1];
	HAL_Delay(10);

}
void MS5611_setMyGround(){
	int i=0;
	float total;
	for(i=0;i<5;i++){
		MS5611_Request_Temp();
		HAL_Delay(10);
		MS5611_ReadTemperature();
		MS5611_Request_Pressure();
		HAL_Delay(10);
		MS5611_ReadAltitude2();
		total+=GY63.ALT;
	}
	GY63.GROUND=total/5.0;
}
void MS5611_ReadTemperature(){
    // Send the command to start temperature conversion
    //tx_buf[0] = 0x58;
	//HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress, tx_buf, 1, 100);
    //wait 10ms to make data ready	---> BURASI ALGORİTMA HALİNE GELCEK VE SİLİNECEK YMFC32 ALGORİTMASI KULLANILABİLİR
    //HAL_Delay(10);

    // Send the command to read the temperature
    tx_buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 3, 100);
    GY63.D2 = (rx_buf[0] << 16) | (rx_buf[1] << 8) | rx_buf[2];

    // Calculate the temperature
    GY63.dT = (GY63.D2 - (GY63.C[5] * 256 ));
    GY63.TEMP = (2000 + (GY63.dT * GY63.C[6]) / 8388608);
    GY63.float_TEMP = GY63.TEMP/100.0;
    if(GY63.float_TEMP<20){
    	GY63.T2=pow(GY63.dT,2) /pow(2,31);

    	GY63.TEMP = GY63.TEMP -GY63.T2;

    }
}

void MS5611_ReadPressure(){

    // Send the command to start pressure conversion
    //tx_buf[0] = 0x48 ;
	//HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress, tx_buf, 1, 100);
    //HAL_Delay(10);

    // Send the command to read the pressure
    tx_buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, MS5611_SlaveAddress<<1, rx_buf, 3, 100);
    GY63.D1 = (rx_buf[0] << 16) | (rx_buf[1] << 8) | rx_buf[2];

    // Calculate the pressure
    //MS5611_ReadTemperature();

    GY63.OFF = ((int64_t)GY63.C[2] * 65536) + (  ((int64_t)GY63.C[4] * GY63.dT) / 128);				// OFF
    GY63.SENS = ((int64_t)GY63.C[1] * 32768) + (((int64_t)GY63.C[3] * GY63.dT) / 256);

    if(GY63.float_TEMP<20){
    	GY63.OFF2=5* pow((GY63.TEMP - 2000),2)/2;
    	GY63.SENS2=5 * pow((GY63.TEMP -2000),2)/ 2;
    	GY63.OFF= GY63.OFF -GY63.OFF2;
    	GY63.SENS = GY63.SENS -GY63.SENS2;

        }
    GY63.P = ((GY63.D1 * GY63.SENS / 2097152) - GY63.OFF) / 32768;									// P
    GY63.P= (int32_t)GY63.P;
    GY63.float_P=GY63.P/100.0;

}


void MS5611_ReadAltitude1(){
	MS5611_ReadPressure();
	//GY63.ALT = (uint32_t)((288.15 / 0.0065) * (powf((GY63.P / 1013.25), (1 / 5.255)) - 1));
	GY63.ALT = ((1.0 - powf((GY63.float_P / SEA_PRESSURE), 0.1902226)) * 44307.69396);
}


// altitude with temperature correction
void MS5611_ReadAltitude2(){
	MS5611_ReadPressure();
	//GY63.ALT = (uint32_t)((288.15 / 0.0065) * (powf((GY63.P / 1013.25), (1 / 5.255)) - 1));
	GY63.ALT = (1.0f - powf((GY63.float_P / SEA_PRESSURE), 0.1902226f)) * ((GY63.float_TEMP + 273.15f) / 0.0065f);

}

void MS5611_Request_Pressure(){
	// Send the command to start pressure conversion
	tx_buf[0] = 0x48 ;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
}
void MS5611_Request_Temp(){
    // Send the command to start temperature conversion
    tx_buf[0] = 0x58;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SlaveAddress<<1, tx_buf, 1, 100);
}

void baro_read(){

	if (loop_counter%3==0){
		MS5611_ReadAltitude1();		// ReadAltitude2 sıcaklık ayarı çeker
		baro_filter_input[0]=GY63.ALT;
		baro_filter_output[0] = baro_filter_output_coeff[0]*baro_filter_output[1] + baro_filter_output_coeff[1]*baro_filter_output[2] + baro_filter_output_coeff[2]*baro_filter_output[3] +baro_filter_input_coeff[0]*baro_filter_input[0] + baro_filter_input_coeff[1]*baro_filter_input[1] + baro_filter_input_coeff[2]*baro_filter_input[2]+ baro_filter_input_coeff[3]*baro_filter_input[3];
		for(int i = 2; i >= 0; i--){
			baro_filter_input[i+1] = baro_filter_input[i]; // store xi
			baro_filter_output[i+1] = baro_filter_output[i]; // store yi
		}
		MS5611_Request_Pressure();
	}
}


/*
 * if(loop_counter%7==0 && pressure_ready==0){
		  MS5611_Request_Pressure();
		  loop_counter_pressure_requested=loop_counter;
	  }
	  if(loop_counter-loop_counter_pressure_requested==3){
		  pressure_ready=1;
	  }

	  if (pressure_ready){
		  MS5611_ReadAltitude1();		// ReadAltitude2 sıcaklık ayarı çeker
		  pressure_ready=0;
		  baro_filter_input[0]=GY63.ALT;
		  baro_filter_output[0] = baro_filter_output_coeff[0]*baro_filter_output[1] + baro_filter_output_coeff[1]*baro_filter_output[2] + baro_filter_output_coeff[2]*baro_filter_output[3] +baro_filter_input_coeff[0]*baro_filter_input[0] + baro_filter_input_coeff[1]*baro_filter_input[1] + baro_filter_input_coeff[2]*baro_filter_input[2]+ baro_filter_input_coeff[3]*baro_filter_input[3];
		  for(int i = 2; i >= 0; i--){
			  baro_filter_input[i+1] = baro_filter_input[i]; // store xi
			  baro_filter_output[i+1] = baro_filter_output[i]; // store yi
		  }
	  }
 */
