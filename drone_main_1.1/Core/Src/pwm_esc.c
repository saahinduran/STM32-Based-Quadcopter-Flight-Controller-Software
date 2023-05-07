/*
 * pwm=esc.c
 *
 *  Created on: Nov 26, 2022
 *      Author: Administrator
 */


#include "pwm_esc.h"
#include "main.h"
#include <stdlib.h>
extern TIM_HandleTypeDef htim1;


// motors won't start if this function is not called
void activate_motors(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

}

void changespeedM1(int *duty_cycle_1,int MIN_Duty_cycle){
	if (*duty_cycle_1<=MIN_Duty_cycle){
		TIM1->CCR1= MIN_Duty_cycle;
	}
	else
	{
		TIM1->CCR1= *duty_cycle_1;
	}



	//ESC_PWM_degisken=*duty_cycle_1;
}


void changespeedM2(int *duty_cycle_2,int MIN_Duty_cycle){
	if(*duty_cycle_2<=MIN_Duty_cycle){
		TIM1->CCR2= MIN_Duty_cycle;
	}
	else{
		TIM1->CCR2= *duty_cycle_2;

	}
}


void changespeedM3(int *duty_cycle_3,int MIN_Duty_cycle){
	if(*duty_cycle_3<=MIN_Duty_cycle){
		TIM1->CCR3= MIN_Duty_cycle;
	}
	else{
		TIM1->CCR3= *duty_cycle_3;

	}
}

void changespeedM4(int *duty_cycle_4,int MIN_Duty_cycle){
	if (*duty_cycle_4<=MIN_Duty_cycle){
		TIM1->CCR4=MIN_Duty_cycle;
	}
	else{
		TIM1->CCR4=*duty_cycle_4;

	}
}
