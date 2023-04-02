/*
 * pwm=esc.c
 *
 *  Created on: Nov 26, 2022
 *      Author: Administrator
 */


//                                 --------------------> KUMANDADAN GELEN KOD 0 İLA 1000 ARASI OLMASI LAZIM <----------------------------------
#include "pwm_esc.h"
#include "main.h"
#include <stdlib.h>
extern TIM_HandleTypeDef htim1;

//extern int MIN_Duty_cycle;			// istersen benzer şekilde max duty cycle da tanımlarsın ama çok fazla uçması drone a
//int ESC_PWM_degisken;										//test için dutycycle_M1 i write variable a ekle ona farklı değer ver bu da değişir aynı olarak

// whileda olmazsa powerupMx(&duty_cycle_x);

// map fonksiyonu kumandadan gelen veriyi 0 ila 1000 arasında bir ölçeğe çevirir.
// derece dönüşümlerindeki formul baz alınarak hesap denklemi oluşturuldu.
float map(int kanal,int kumanda_min, int kumanda_max, int istenen_aralık_alt, int istenen_aralık_üst){
	return (((istenen_aralık_üst-istenen_aralık_alt)*(kanal-kumanda_min)/(kumanda_max-kumanda_min))+istenen_aralık_alt);
}

// bu fonksiyon cagrilmadan motorlar baslamaz.
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
