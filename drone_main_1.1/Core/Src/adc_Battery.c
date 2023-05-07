/*
 * adc_Battery.c
 *
 *  Created on: Feb 24, 2023
 *      Author: sahin
 */

#include "adc_Battery.h"
//Starts the ADC reading with DMA
void start_reading_battery(){
	HAL_ADC_Start_DMA(&hadc1,&battery_raw_data,1);
}


