/*
 * pid.h
 *
 *  Created on: 24 Kas 2022
 *      Author: sahin
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_


double getControlSignal(double error, double kp, double ki, double kd,
		double* pid_i, double* last_error, double delta_time_in_seconds);
double getControlSignal_roll_pitch(double error, double kp, double ki, double kd, double* pid_i, double* last_error, double delta_time_in_seconds);
#endif /* SRC_PID_H_ */
