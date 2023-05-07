/*
 * failsafe.h
 *
 *  Created on: Feb 24, 2023
 *      Author: sahin
 */

#ifndef SRC_FAILSAFE_H_
#define SRC_FAILSAFE_H_

#include "main.h"

void failsafe_handler();
void i2c_disconnected();
void red_led_on();
void red_led_off();
void green_led_on();
void green_led_off();
void blue_led_on();
void blue_led_off();
//HAL_GPIO_WritePin(GPIOB, LED1_Pin, SET);  //  GREEN LED
//HAL_GPIO_WritePin(GPIOB, LED2_Pin, SET);  //   BLUE LED
//HAL_GPIO_WritePin(GPIOB, LED3_Pin, SET);  //   RED  LED
#endif /* SRC_FAILSAFE_H_ */
