/*
 * failsafe.c
 *
 *  Created on: Feb 24, 2023
 *      Author: sahin
 */
#include "failsafe.h"
#include "pwm_esc.h"
#include "receiver.h"

extern I2C_HandleTypeDef hi2c2;
/// this function rescues the hang i2c bus that is connected to imu
void i2c_disconnected(){
//PF0 and PF1 are corresponding I2C pins (SCL and SDA)
	uint16_t data;
	HAL_I2C_DeInit(&hi2c2);
	data=GPIOF->MODER;
	data= data & ~0b1111;
	data= data | 0b0101;
	GPIOF->MODER=data;


	data=GPIOF->OTYPER;

	data = data & ~0b11;
	GPIOF->OTYPER=data;

	data=GPIOF->AFR[0];
	data=  data & ~0b11111111;
	GPIOF->AFR[0]=data;

	for(int i=0;i<14;i++) // either 9 or 14
	{
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
	}

	  HAL_I2C_Init(&hi2c2);
}
// worst scenerio, shut the motors and wait for watchdog to reset the mcu
void failsafe_handler()
{
	stop_motors();
	while(1);

}
void red_led_on(){
	HAL_GPIO_WritePin(GPIOB, LED3_Pin, SET);
}
void red_led_off(){
	HAL_GPIO_WritePin(GPIOB, LED3_Pin, RESET);
}
void green_led_on(){
	HAL_GPIO_WritePin(GPIOB, LED1_Pin, SET);
}
void green_led_off(){
	HAL_GPIO_WritePin(GPIOB, LED1_Pin, RESET);
}
void blue_led_on(){
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, SET);
}
void blue_led_off(){
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, RESET);
}

