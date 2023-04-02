/*
 * adc_Battery.h
 *
 *  Created on: Feb 24, 2023
 *      Author: sahin
 */

#ifndef SRC_ADC_BATTERY_H_
#define SRC_ADC_BATTERY_H_

#include "main.h"
extern uint32_t battery_raw_data;
extern ADC_HandleTypeDef hadc1;


void start_reading_battery();
#endif /* SRC_ADC_BATTERY_H_ */
