/*
 * receiver.c
 *
 *  Created on: Nov 26, 2022
 *      Author: sahin
 */


#include "receiver.h"
#include "pid.h"
#include "main.h"
#include "pwm_esc.h"
#include "mpu6050.h"
#include "log_to_flash.h"
#include "compass.h"
#include <math.h>
#include <string.h>


extern uint8_t receive_buffer[32];
#define MIN_Duty_cycle 500
struct receiverdata My_Receiver;
#define Sector15_Address 0x08160000  // - 0x0817 FFFF 128 Kbyte
#define motorstopValue 1000
int stopMotors=motorstopValue;
extern char buffer[25];
extern int _channels[12];
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern MPU6050_t MPU6050_1;
extern float imu_calibration_values[5];
extern float Xsf,Ysf,Xoff,Yoff,Zsf,Zoff;

double Roll_Error,Yaw_Error,Pitch_Error;
double Roll_Control,Yaw_Control,Pitch_Control;
extern double angle_pitch_output, angle_roll_output , angle_yaw_rate_output;
double delta_time_in_seconds=0.638e-3;
double kp_roll=1.29, ki_roll=0.01714, kd_roll=50,pid_i_roll, last_error_roll; //roll
double kp_pitch=1.29, ki_pitch=0.01714, kd_pitch=50; //pitch
double pid_i_pitch, last_error_pitch;
double kp_yaw=4, ki_yaw=0.04, kd_yaw=0, pid_i_yaw, last_error_yaw;
extern float gyro_roll_input,gyro_pitch_input,gyro_yaw_input;
extern float roll_level_adjust, pitch_level_adjust;
extern float alt_setpoint;
extern float baro_filter_output[4];
float Alt_Error,Alt_Control;
float kp_alt=5,ki_alt=0.0,kd_alt=3.75;
double pid_i_alt,last_error_alt;
extern int Calculated_Throttle1,Calculated_Throttle2,Calculated_Throttle3,Calculated_Throttle4;
extern uint8_t alt_hold_flag;

///GPS HOLD VALUES///
float pid_i_lat,pid_i_lon,last_error_lat,last_error_lon;
extern int gps_hold_flag;
extern uint32_t gps_setpoint_lat,gps_setpoint_lon;
extern float GpsRollControl,GpsPitchControl;
extern uint32_t latitude,longitude;
double lon_error,lat_error;
extern float heading_corrected;
float kp_gps=0.5,ki_gps=0,kd_gps=1.5;
extern float GpsRollControl_temp,GpsPitchControl_temp;
extern uint8_t gps_go_flag;
extern uint8_t heading_lock;
extern float heading_error;
extern uint32_t loop_counter;
void calculate_motor_powers(){

	if(gps_hold_flag){
		_channels[0]+=GpsRollControl;
		_channels[1]-=GpsPitchControl;
	}
	if(gps_go_flag){

		_channels[3]-=heading_error;
		if(abs(heading_error)<2){
			_channels[1]-=155; //5 derece*31=155;
		}
	}
	/*
	kp_roll=1.1+((double)_channels[5]-1000) /2000;   //TO TUNE PID FROM THE TRANSMITTER
	ki_roll=0.001+((double)_channels[6]) /250000;
	kd_roll=20+((double)_channels[7]-1000) /200;
	kp_pitch=kp_roll;
	ki_pitch=ki_roll;
	kd_pitch=kd_roll;

	ki_roll=((double)_channels[5]) /50000;
	ki_pitch=ki_roll;

	*/
	//kp_gps=((double)_channels[7]) /1000;   //TO TUNE PID FROM THE TRANSMITTER

	//kd_gps=((double)_channels[11]) /330;

	kp_alt=((double)_channels[5]) /200;   ////TO TUNE PID FROM THE TRANSMITTER

	kd_alt=((double)_channels[7]) /100;
	ki_alt=((double)_channels[6]) /5000;


	Roll_Error=(((_channels[0])-1000)-(roll_level_adjust))/6     -   gyro_roll_input;

	Pitch_Error=(((_channels[1])-1000)-(pitch_level_adjust))/6 	 -   gyro_pitch_input;

	Yaw_Error=((_channels[3])-1000)*0.166   -   gyro_yaw_input;

	Roll_Control= getControlSignal_roll_pitch(Roll_Error, kp_roll,ki_roll  ,kd_roll,
			&pid_i_roll,&last_error_roll , delta_time_in_seconds);

	Pitch_Control= getControlSignal_roll_pitch(Pitch_Error, kp_pitch,ki_pitch  ,kd_pitch,
				&pid_i_pitch,&last_error_pitch , delta_time_in_seconds);

	Yaw_Control= getControlSignal_roll_pitch(Yaw_Error, kp_yaw, ki_yaw  ,kd_yaw,
				&pid_i_yaw,&last_error_yaw , delta_time_in_seconds);


}
void calculate_pid_altitude(){


	Alt_Error=  alt_setpoint - baro_filter_output[0];

	Alt_Control= getControlSignal(Alt_Error, kp_alt,ki_alt  ,kd_alt,
				&pid_i_alt,&last_error_alt , delta_time_in_seconds);
	if(Alt_Control>300)Alt_Control=300;
	if(Alt_Control<-300)Alt_Control=-300;

}
void calculate_pid_gps(){


	lat_error=(double)gps_setpoint_lat-(double)latitude;
	lon_error=(double)gps_setpoint_lon-(double)longitude;
	GpsRollControl_temp= getControlSignal(lat_error, kp_gps,ki_gps  ,kd_gps,
			&pid_i_lat,&last_error_lat , delta_time_in_seconds);
	GpsPitchControl_temp= getControlSignal(lon_error, kp_gps,ki_gps  ,kd_gps,
				&pid_i_lon,&last_error_lon , delta_time_in_seconds);

	GpsPitchControl=GpsPitchControl_temp*cos(heading_corrected*M_PI/180)+GpsRollControl_temp*cos((heading_corrected-90)*M_PI/180); //heading correction
	GpsRollControl=GpsRollControl_temp*cos(heading_corrected*M_PI/180)+GpsPitchControl_temp*cos((heading_corrected+90)*M_PI/180); //heading correction
	if(GpsPitchControl>500)GpsPitchControl=500;
	else if(GpsPitchControl<-500)GpsPitchControl=-500;
	if(GpsRollControl>500)GpsRollControl=500;
	else if(GpsRollControl<-500)GpsRollControl=-500;
}

void reset_alt_pid(){
	pid_i_alt=0;
	last_error_alt=0;
	Alt_Error=0;
}
void reset_gps_pid(){

	pid_i_lat=0;
	pid_i_lon=0;
	last_error_lat=0;
	last_error_lon=0;
	lat_error=0;
	lon_error=0;
}

// this function is executed before switching the main loop to test the motors
int calibrate_esc_and_rc()
{
	uint8_t Calib=0;
	while(Calib!=1){
		decode_receiver();
		if(_channels[10]<1000){
				  stop_motors();
			  }
		else{
			changespeedM1(&_channels[2],MIN_Duty_cycle);
			changespeedM2(&_channels[2],MIN_Duty_cycle);
			changespeedM3(&_channels[2],MIN_Duty_cycle);
			changespeedM4(&_channels[2],MIN_Duty_cycle);
			if(_channels[2]<1100 && _channels[3]<100){
				Calib=1;
				}
			else
				Calib=0;
		}

	}
	return 0;
}
void stop_motors()
{
	changespeedM1(&stopMotors,MIN_Duty_cycle);
	changespeedM2(&stopMotors,MIN_Duty_cycle);
	changespeedM3(&stopMotors,MIN_Duty_cycle);
	changespeedM4(&stopMotors,MIN_Duty_cycle);
}
void waiting_for_arm()
{
	uint8_t isArmed=0;
	while(!isArmed)
	{
		if (buffer[0]==15){
						_channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
						_channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
						_channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF)/2+1000;
						_channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
						_channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
						_channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
						_channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
						_channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
						_channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
						_channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
						_channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
						_channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
						_channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
						_channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
						_channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
						_channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);
					}
		if(_channels[0]>1900 && _channels[1]>1900) isArmed=1;


	}
}
// since we will use dma, frame synchronization is important
uint8_t connect_receiver(void){
	uint8_t receiver_connected=0;
	while(!receiver_connected){
		HAL_UART_Receive_DMA(&huart2, (uint8_t *)buffer,25);
		if(buffer[0]!=15){
			HAL_UART_DMAStop(&huart2);
			HAL_Delay(10);
			HAL_UART_DeInit(&huart2);
			HAL_Delay(10);
			HAL_UART_Init(&huart2);
			HAL_Delay(10);
			HAL_UART_Receive_DMA(&huart2, (uint8_t *)buffer,25);
			HAL_Delay(100);
			receiver_connected=0;
		}
		else if (buffer[0]==15 && buffer[23]==0){
			receiver_connected=1;
		}
	}
	return 0;
}

int decode_receiver()
{
	if(buffer[0]==15){
		_channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
		_channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
		_channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF)/2+1000;
		_channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
		_channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
		_channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
		_channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
		_channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
		_channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
		_channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
		_channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
		_channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
		_channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
		_channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
		_channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
		_channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);

		if(buffer[23]!=0){
			return 1;
			}
		else
			return 0;
}
	return -1;
}

int startup_handler(){
	uint8_t startup_flag=1;

	while(startup_flag){
		if(!decode_receiver()){
			red_led_off();
			if(_channels[10]<1000){  // right top switch disabled
				if(_channels[3]>1800 && _channels[9]<1000){  // left joystick left-down calibrates mpu
					HAL_GPIO_WritePin(GPIOB, LED2_Pin, SET);
					HAL_Delay(2000);
					calibrate_mpu6050();

					HAL_GPIO_WritePin(GPIOB, LED2_Pin, RESET);
				}
				else if(_channels[3]<100 && _channels[9]<1000){  // left joystick right-down normal startup
					read_calib_value(Sector15_Address,6,imu_calibration_values);
					MPU6050_1.pitch_calibration_value=imu_calibration_values[0];
					MPU6050_1.roll_calibration_value=imu_calibration_values[1];
					MPU6050_1.yaw_calibration_value=imu_calibration_values[2];
					MPU6050_1.Accel_X_Raw_Offset=imu_calibration_values[3];
					MPU6050_1.Accel_Y_Raw_Offset=imu_calibration_values[4];
					MPU6050_1.Accel_Z_Raw_Rate=imu_calibration_values[5];

					read_calib_value(Sector14_Address,6,imu_calibration_values);
					Xsf=imu_calibration_values[0];
					Ysf=imu_calibration_values[1];
					Xoff=imu_calibration_values[2];
					Yoff=imu_calibration_values[3];
					Zsf=imu_calibration_values[4];
					Zoff=imu_calibration_values[5];

					while(compass_connect());
					startup_flag=0;

				}
				else if( _channels[1]>1800 && _channels[0]>1800){ // right joystick right-down calibrates compass
					read_calib_value(Sector15_Address,6,imu_calibration_values);
					MPU6050_1.pitch_calibration_value=imu_calibration_values[0];
					MPU6050_1.roll_calibration_value=imu_calibration_values[1];
					MPU6050_1.yaw_calibration_value=imu_calibration_values[2];
					MPU6050_1.Accel_X_Raw_Offset=imu_calibration_values[3];
					MPU6050_1.Accel_Y_Raw_Offset=imu_calibration_values[4];
					MPU6050_1.Accel_Z_Raw_Rate=imu_calibration_values[5];

					while(compass_connect());
					HAL_GPIO_WritePin(GPIOB, LED2_Pin, SET);
					compas_calibrate();
					HAL_GPIO_WritePin(GPIOB, LED2_Pin, RESET);

				}

			}
		}
		else red_led_on();
	}
	return 0;
}


int arm_handler(){
	uint8_t arm_flag=0;
	while(!arm_flag){
		if(!decode_receiver()){
			red_led_off();
			if(_channels[2]<1100 && _channels[10]>1000){
				activate_motors();
				while(calibrate_esc_and_rc());
				while(1){
					if(!decode_receiver()){
						if(_channels[2]<1100 && _channels[10]>1000 && (_channels[3]<1100 && _channels[3]>900) && _channels[9]<1000){
							arm_flag=1;
							break;
						}
						else
							continue;
					}
				}


			}
			else
				arm_flag=0;

		}
		else red_led_on();
	}
	return 0;
}

void altitude_hold(){

	if(!alt_hold_flag){
		alt_hold_flag=1;
		alt_setpoint=baro_filter_output[0];
	}
	else{
		if(loop_counter%6==0 && loop_counter!=0){ //sync refresh rate of sensor and pid in order to derivative term work properly
			calculate_pid_altitude();
		}

		Calculated_Throttle4-=Alt_Control;
		Calculated_Throttle3-=Alt_Control;
		Calculated_Throttle2-=Alt_Control;
		Calculated_Throttle1-=Alt_Control;
		if(Calculated_Throttle1<1100) Calculated_Throttle1=1100;
		if(Calculated_Throttle1>1800) Calculated_Throttle1=1800;
		if(Calculated_Throttle2<1100) Calculated_Throttle2=1100;
		if(Calculated_Throttle2>1800) Calculated_Throttle2=1800;
		if(Calculated_Throttle3<1100) Calculated_Throttle3=1100;
		if(Calculated_Throttle3>1800) Calculated_Throttle3=1800;
		if(Calculated_Throttle4<1100) Calculated_Throttle4=1100;
		if(Calculated_Throttle4>1800) Calculated_Throttle4=1800;
	}
}

void decode_rf(){
	int i=0;
	for(i=0;i<31;i++){
		if(receive_buffer[i]==35){
			i=i+2;
			i=i%32;
			_channels[2]=receive_buffer[i]+(receive_buffer[i+1]<<8);
			break;
		}
	}


}
void gps_hold(){
	if(!gps_hold_flag){
		gps_hold_flag=1;
		gps_setpoint_lat=latitude;
		gps_setpoint_lon=longitude;
	}
	else{
		if(loop_counter%50==0 && loop_counter!=0){
			calculate_pid_gps();
		}

		if(Calculated_Throttle1<1100) Calculated_Throttle1=1100;
		if(Calculated_Throttle1>1800) Calculated_Throttle1=1800;
		if(Calculated_Throttle2<1100) Calculated_Throttle2=1100;
		if(Calculated_Throttle2>1800) Calculated_Throttle2=1800;
		if(Calculated_Throttle3<1100) Calculated_Throttle3=1100;
		if(Calculated_Throttle3>1800) Calculated_Throttle3=1800;
		if(Calculated_Throttle4<1100) Calculated_Throttle4=1100;
		if(Calculated_Throttle4>1800) Calculated_Throttle4=1800;
	}



}
/////ESC CALIBRATION AND PROGRAMMING//// PUT THIS SECTION IN MAIN FUNCTION RIGHT AFTER CONNECTING TO RECEIVER
// CALIBRATE YOUR ESCS
/*
while(1){
	  decode_receiver();
	  activate_motors();
	  changespeedM1(&_channels[2],MIN_Duty_cycle);
	  changespeedM2(&_channels[2],MIN_Duty_cycle);
	  changespeedM3(&_channels[2],MIN_Duty_cycle);
	  changespeedM4(&_channels[2],MIN_Duty_cycle);
}
*/
void reset_pid(){
	last_error_roll=0;
	last_error_pitch=0;
	last_error_yaw=0;
	pid_i_yaw=0;
	pid_i_roll=0;
	pid_i_pitch=0;
	Roll_Error=0;
	Pitch_Error=0;
	Yaw_Error=0;
	Roll_Control=0;
	Pitch_Control=0;
	Yaw_Control=0;

}


