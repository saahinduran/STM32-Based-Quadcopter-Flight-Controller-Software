/*
 * receiver.h
 *
 *  Created on: Nov 26, 2022
 *      Author: sahin
 */

#ifndef SRC_RECEIVER_H_
#define SRC_RECEIVER_H_
#include "main.h"
struct receiverdata
{

	float rollPulse,yawPulse,pitchPulse;

};

void calculate_motor_powers();
void calculate_pid_altitude();
void reset_alt_pid();
void reset_gps_pid();
void decode_rf();
void altitude_hold();
void gps_hold();
void reset_pid();
int calibrate_esc_and_rc();
void waiting_for_arm();
void stop_motors();
int decode_receiver();
int startup_handler();
int arm_handler();
uint8_t connect_receiver(void);
#endif /* SRC_RECEIVER_H_ */
