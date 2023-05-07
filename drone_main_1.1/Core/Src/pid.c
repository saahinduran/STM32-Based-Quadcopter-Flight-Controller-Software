/*
 * pid.c
 *
 *  Created on: 24 Kas 2022
 *      Author: sahin
 */
#define MAX_CONTROL_SIGNAL 400
#define MIN_CONTROL_SIGNAL -400
#include "pid.h"

double getControlSignal(double error, double kp, double ki, double kd, double* pid_i, double* last_error, double delta_time_in_seconds)
{
  double pid_p = error;
  double pid_d = (error - *last_error);
  *pid_i += error  ;
  if(*pid_i>100)*pid_i=100;  // anti wind-up precaution
  if(*pid_i<-100)*pid_i=-100;  // anti wind-up precaution
  double control_signal = (kp * pid_p) + (ki * (*pid_i)) + (kd * pid_d);
  *last_error = error;


  if(control_signal>MAX_CONTROL_SIGNAL)
	  return MAX_CONTROL_SIGNAL;
  else if (control_signal<MIN_CONTROL_SIGNAL)
	  return MIN_CONTROL_SIGNAL;
  else

	  return control_signal;
}

double getControlSignal_roll_pitch(double error, double kp, double ki, double kd, double* pid_i, double* last_error, double delta_time_in_seconds)
{
  double pid_p = error;
  double pid_d = (error - *last_error);
  *pid_i += error  ;
  if(*pid_i>50/ki)*pid_i=50/ki;  // anti wind-up precaution
  if(*pid_i<-50/ki)*pid_i=-50/ki;  // anti wind-up precaution
  double control_signal = (kp * pid_p) + (ki * (*pid_i)) + (kd * pid_d);
  *last_error = error;


  if(control_signal>MAX_CONTROL_SIGNAL)
	  return MAX_CONTROL_SIGNAL;
  else if (control_signal<MIN_CONTROL_SIGNAL)
	  return MIN_CONTROL_SIGNAL;
  else

	  return control_signal;
}
