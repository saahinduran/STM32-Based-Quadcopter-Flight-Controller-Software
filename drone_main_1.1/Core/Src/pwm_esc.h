/*
 * pwm_esc.h
 *
 *  Created on: Nov 26, 2022
 *      Author: Administrator
 */

#ifndef HWDRIVERS_ESC_PWM_H_
#define HWDRIVERS_ESC_PWM_H_
float map(int kanal,int kumanda_min, int kumanda_max, int istenen_aralık_alt, int istenen_aralık_üst);
void changespeedM1(int *duty_cycle_1,int MIN_Duty_cycle);
void changespeedM2(int *duty_cycle_2,int MIN_Duty_cycle);
void changespeedM3(int *duty_cycle_3,int MIN_Duty_cycle);
void changespeedM4(int *duty_cycle_4,int MIN_Duty_cycle);
void activate_motors();

#endif /* HWDRIVERS_ESC_PWM_H_ */
