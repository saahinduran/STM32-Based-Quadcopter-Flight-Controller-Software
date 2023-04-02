
#include <math.h>
#include "mpu6050.h"
#include "main.h"
#include <stdbool.h>
#include "FIRFilter.h"
#include "log_to_flash.h"
#include "failsafe.h"


#define RAD_TO_DEG 57.295779513082320876798154814105
#define ACCELERATION_RATE 16384.0f
//orjinali buydu #define ACCELERATION_RATE_Z 17700.0f
#define ACCELERATION_RATE_Z 17770.0f
#define GYRO_RATE 131.0f
#define MPU6050_CONNECTION_LOST 0b000000000000001;


#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define MPU6050_TIMEOUT 100

// Setup MPU6050
#define MPU6050_ADDR 0xD0
//float my_alpha=0.2;
float my_alpha=0.9992;
bool set_gyro_angles;
float dF=1/0.004;
extern int timex;
extern TIM_HandleTypeDef htim1;
uint8_t Data_hersey[64];

extern double angle_pitch_output, angle_roll_output , angle_yaw_rate_output;
extern uint16_t DRONE_STATUS;

extern float x_1[4]; // Raw values
extern float y_1[4];

extern float x_2[4]; // Raw values
extern float y_2[4];

extern float x_3[4]; // Raw values
extern float y_3[4];
float gyro_roll_input,gyro_pitch_input,gyro_yaw_input;
uint32_t imu_fault_counter=0;

FIRFilter acc_x_fir,acc_y_fir,acc_z_fir;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1;
extern MPU6050_t MPU6050_1;
extern float calibration_buffer_float[5];

float accel_input[4];float accel_output[4];

float accel_filter_output_coeff[3]={2.874356892677485,-2.756483195225695
		,0.881893130592486};

float accel_filter_input_coeff[4]={2.914649446569766e-05,8.743948339709297e-05,
		8.743948339709297e-05,2.914649446569766e-05};
int cnt=0;
/*
float gyro_input[4];float gyro_output[4];

float gyro_filter_output_coeff[3]={2.937170728449890,-2.876299723479331
		,0.939098940325283};

float gyro_filter_input_coeff[4]={0.969071174031813,-2.907213522095439,
		2.907213522095439,-0.969071174031813};
*/
///EĞER PID DUZGUN CALISMAZSA GYRONUN OLCEKLERINI DEGISTIR
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    while(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, MPU6050_TIMEOUT)!=HAL_OK){
    	DRONE_STATUS |= MPU6050_CONNECTION_LOST ;
    	imu_fault_counter+=1;
    	i2c_disconnected();
    }



    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCELERATION_RATE;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCELERATION_RATE;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCELERATION_RATE_Z; // burayÄ± yapmayÄ± unutma
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_RATE; // buralar 131di
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_RATE;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_RATE;

    DataStruct->Gx = DataStruct->Gx - DataStruct->pitch_calibration_value;
    DataStruct->Gy = DataStruct->Gy - DataStruct->roll_calibration_value;
    DataStruct->Gz = DataStruct->Gz - DataStruct->yaw_calibration_value;

    DataStruct->acc_total_vector = sqrt((DataStruct->Ax*DataStruct->Ax)+(DataStruct->Ay*DataStruct->Ay)+(DataStruct->Az*DataStruct->Az));

        DataStruct->angle_pitch_acc = asin((float)DataStruct->Ay/DataStruct->acc_total_vector)* RAD_TO_DEG;
        DataStruct->angle_roll_acc = asin((float)DataStruct->Ax/DataStruct->acc_total_vector)* -RAD_TO_DEG;


        DataStruct->angle_pitch_acc-=DataStruct->angle_pitch_acc_calib;
    	DataStruct->angle_roll_acc-=DataStruct->angle_roll_acc_calib;

}


bool calibrate_yaw(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	int i;
	double summationZ=0;
	double summationX=0;
	double summationY=0;
	float summation_temp=0;
	for (i=0;i<2000;i++)
	{
		MPU6050_Read_All(I2Cx,DataStruct);
		summationZ+=DataStruct->Gyro_Z_RAW;
		summationX+=DataStruct->Gyro_X_RAW;
		summationY+=DataStruct->Gyro_Y_RAW;
		summation_temp+=DataStruct->Temperature;


	}
	DataStruct->yaw_calibration_value=summationZ/(double)i;
	DataStruct->pitch_calibration_value=summationX/(double)i;
	DataStruct->roll_calibration_value=summationY/(double)i;
	DataStruct->temp_calibration_value=summation_temp/i;
	return 1;
}

bool calibrate_ACC(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	int i;
	double summationRoll=0;
	double summationPitch=0;
	double summation_Z=0;
	for (i=0;i<20;i++)
	{
		MPU6050_Read_All(I2Cx,DataStruct);
		summationRoll+=DataStruct->angle_roll_acc;
		summationPitch+=DataStruct->angle_pitch_acc;
		summation_Z+=DataStruct->Accel_Z_RAW;


	}
	DataStruct->angle_pitch_acc_calib=round((summationPitch/(double)i)*10) /10;
	DataStruct->angle_roll_acc_calib=round((summationRoll/(double)i)*10) /10;
	DataStruct->Accel_Z_Calib=summation_Z/i;
	//DataStruct->angle_pitch_acc_calib=(summationPitch/i);
	//DataStruct->angle_roll_acc_calib=(summationRoll/i);

	//FIRFilter_Init(&acc_x_fir);
	//FIRFilter_Init(&acc_y_fir);
	//FIRFilter_Init(&acc_z_fir);

	return 1;
}

bool calibrate_Benim(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	int i;
	double summationRoll=0;
	double summationPitch=0;
	double summation_Z=0;
	for (i=0;i<2000;i++)
	{
		MPU6050_Read_All(I2Cx,DataStruct);
		summationRoll+=DataStruct->Accel_X_RAW;
		summationPitch+=DataStruct->Accel_Y_RAW;
		summation_Z+=DataStruct->Accel_Z_RAW;
	}
	DataStruct->Accel_X_Raw_Offset=summationRoll/i;
	DataStruct->Accel_Y_Raw_Offset=summationPitch/i;
	DataStruct->Accel_Z_Raw_Rate=summation_Z/i;

	return 1;
}



uint8_t MPU6050_Init_Benim(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    while(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, MPU6050_TIMEOUT)!=HAL_OK){
    	DRONE_STATUS |= MPU6050_CONNECTION_LOST ;
    	imu_fault_counter+=1;
    	i2c_disconnected();
    }



    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, MPU6050_TIMEOUT);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x0; // edit var, eski deÄŸeri 07di.
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, MPU6050_TIMEOUT);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ï¿½ 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, MPU6050_TIMEOUT);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ï¿½ 250 ï¿½/s
        Data = 0x10<<3;
        // Data = 0x00 | 1<<3;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, MPU6050_TIMEOUT);

        //DLPF AÃ‡

        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, 0x1A, 1, &Data, 1, MPU6050_TIMEOUT);
        Data= Data | 0x02;   /// bura 0x06'dı
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1A, 1, &Data, 1, MPU6050_TIMEOUT);

        //HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, 0x0, 1, &Data_hersey, 64, MPU6050_TIMEOUT);




        return 0;
    }
    return 1;
}

void MPU6050_Read_All_Benim(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    while(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, MPU6050_TIMEOUT)!=HAL_OK){
    	DRONE_STATUS |= MPU6050_CONNECTION_LOST ;
    	imu_fault_counter+=1;
    	        	i2c_disconnected();
    }




    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = (DataStruct->Accel_X_RAW -DataStruct->Accel_X_Raw_Offset ) / ACCELERATION_RATE;  /// BURA DEÄÄ°ÅTÄ°
    DataStruct->Ay = (DataStruct->Accel_Y_RAW -DataStruct->Accel_Y_Raw_Offset ) / ACCELERATION_RATE;
    DataStruct->Az = DataStruct->Accel_Z_RAW / DataStruct->Accel_Z_Raw_Rate;
/*
    DataStruct->Ax=FIRFilter_Update(&acc_x_fir, DataStruct->Ax);
    DataStruct->Ay=FIRFilter_Update(&acc_y_fir, DataStruct->Ay);
    DataStruct->Az=FIRFilter_Update(&acc_z_fir, DataStruct->Az);
*/
    //Diferansiyel aÃ§Ä± deÄŸiÅŸimi
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
/*
    DataStruct->Gx = DataStruct->Gyro_X_RAW  - (DataStruct->pitch_calibration_value -  (DataStruct->Temperature-DataStruct->temp_calibration_value)*2); // buralar 131'di
    DataStruct->Gy = DataStruct->Gyro_Y_RAW  - (DataStruct->roll_calibration_value +  (DataStruct->Temperature-DataStruct->temp_calibration_value)*0.2);
    DataStruct->Gz = DataStruct->Gyro_Z_RAW  - (DataStruct->yaw_calibration_value +  (DataStruct->Temperature-DataStruct->temp_calibration_value)*2);
*/

    DataStruct->Gx = DataStruct->Gyro_X_RAW  - DataStruct->pitch_calibration_value; // buralar 131'di
    DataStruct->Gy = DataStruct->Gyro_Y_RAW  - DataStruct->roll_calibration_value ;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW  - DataStruct->yaw_calibration_value;

    DataStruct->Gx = DataStruct->Gx / GYRO_RATE;
    DataStruct->Gy = DataStruct->Gy / GYRO_RATE;
    DataStruct->Gz = DataStruct->Gz / GYRO_RATE;


    //Offset deÄŸerlerini Ã§Ä±kar

    gyro_roll_input = (gyro_roll_input * 0.7) + (DataStruct->Gy * 0.3);   //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + (DataStruct->Gx * 0.3);//Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.7) + (DataStruct->Gz* 0.3);

    //Integral iÅŸleminin yapÄ±ldÄ±ÄŸÄ± yer   //dF= Frekans
/*
    if(DataStruct->Gx < -0.1  || DataStruct->Gx > 0.1){
        	DataStruct->angle_pitch += (float)DataStruct->Gx/dF;
        }

        if(DataStruct->Gy < -0.1  || DataStruct->Gy > 0.1){
        	DataStruct->angle_roll += (float)DataStruct->Gy/dF;
        }


        if(DataStruct->Gz < -0.1  || DataStruct->Gz > 0.1){
        	DataStruct->angle_yaw += (float)DataStruct->Gz/dF;
        }
/*
    DataStruct->angle_pitch += gyro_pitch_input/dF;
    DataStruct->angle_roll += gyro_roll_input/dF;
    DataStruct->angle_yaw += gyro_yaw_input/dF;
*/
    DataStruct->angle_pitch += (float)DataStruct->Gx/dF;
    DataStruct->angle_roll += (float)DataStruct->Gy/dF;
    DataStruct->angle_yaw += (float)DataStruct->Gz/dF;

    //update_filter(DataStruct->angle_pitch);

    DataStruct->acc_total_vector = sqrt((DataStruct->Ax*DataStruct->Ax)+(DataStruct->Ay*DataStruct->Ay)+(DataStruct->Az*DataStruct->Az));

    DataStruct->angle_pitch_acc = asin((float)DataStruct->Ay/DataStruct->acc_total_vector)* RAD_TO_DEG;
    DataStruct->angle_roll_acc = asin((float)DataStruct->Ax/DataStruct->acc_total_vector)* -RAD_TO_DEG;


    //DataStruct->angle_pitch_acc-= DataStruct->angle_pitch_acc_calib;
	//DataStruct->angle_roll_acc-= DataStruct->angle_roll_acc_calib;



// yedek deÄŸerler 0.9992 0.0008
	//0.0001446875


    if(set_gyro_angles){                                                 //If the IMU is already started
    	DataStruct->angle_pitch = DataStruct->angle_pitch * my_alpha + DataStruct->angle_pitch_acc * (1-my_alpha);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    	DataStruct->angle_roll = DataStruct->angle_roll * my_alpha + DataStruct->angle_roll_acc * (1-my_alpha);        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    //	DataStruct->angle_roll = DataStruct->angle_roll * my_alpha + y_1[0] * (1-my_alpha);
    //	DataStruct->angle_pitch = DataStruct->angle_pitch * my_alpha + y_2[0] * (1-my_alpha);

      }
      else{
    	  //At first start
    	  DataStruct->angle_pitch = DataStruct->angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    	  DataStruct->angle_roll = DataStruct->angle_roll_acc;


        set_gyro_angles = true;                                            //Set the IMU started flag
      }
    // Pitch ve Roll'a YAW DÃ¼zeltmesi
        DataStruct->angle_roll-=DataStruct->angle_pitch * sin((DataStruct->Gz/dF)*M_PI/180);
        DataStruct->angle_pitch+=DataStruct->angle_roll * sin((DataStruct->Gz/dF)*M_PI/180);




// LOW-PASS PART
 // Y(n)=y(n-1)*0.9 + x(n)*0.1
    angle_pitch_output = angle_pitch_output * 0.9 + DataStruct->angle_pitch* 0.1 ;
    angle_roll_output = angle_roll_output * 0.9 + DataStruct->angle_roll* 0.1;

    angle_yaw_rate_output=DataStruct->Gz;

    /// VERTICAL VELOCITY CALCULATION
    DataStruct->accel_in_z= -DataStruct->Accel_X_RAW*sin(angle_pitch_output*M_PI/180)
    + DataStruct->Accel_Y_RAW*sin(angle_roll_output*M_PI/180)*cos(angle_pitch_output*M_PI/180)
	+ DataStruct->Accel_Z_RAW*cos(angle_roll_output*M_PI/180)*cos(angle_pitch_output*M_PI/180);

    accel_input[0]=DataStruct->accel_in_z;
    accel_output[0] = accel_filter_output_coeff[0]*accel_output[1] + accel_filter_output_coeff[1]*accel_output[2] + accel_filter_output_coeff[2]*accel_output[3] +accel_filter_input_coeff[0]*accel_input[0] + accel_filter_input_coeff[1]*accel_input[1] + accel_filter_input_coeff[2]*accel_input[2]+ accel_filter_input_coeff[3]*accel_input[3];
    for(int i = 2; i >= 0; i--){
    	accel_input[i+1] = accel_input[i]; // store xi
    	accel_output[i+1] = accel_output[i]; // store yi
    }
    if(cnt<200) cnt++;

    if(cnt>=200){
    	DataStruct->accel_in_z= ((accel_output[0]/DataStruct->Accel_Z_Raw_Rate)-1)*9.81*100;
    	DataStruct->velocity_in_z+=DataStruct->accel_in_z/dF;
    }









}


void calibrate_mpu6050(){

	while(!calibrate_yaw(&hi2c2,&MPU6050_1));
	//while(!calibrate_ACC(&hi2c2,&MPU6050_1));
	while(!calibrate_Benim(&hi2c2,&MPU6050_1));
	fill_calibration_buffer();
	flash_write_calib_data_four_byte(calibration_buffer_float,6);

}
/*
void update_filter(float input){

	gyro_input[0]=input;

	gyro_output[0] = gyro_filter_output_coeff[0]*gyro_output[1] + gyro_filter_output_coeff[1]*gyro_output[2] + gyro_filter_output_coeff[2]*gyro_output[3] +gyro_filter_input_coeff[0]*gyro_input[0] + gyro_filter_input_coeff[1]*gyro_input[1] + gyro_filter_input_coeff[2]*gyro_input[2]+ gyro_filter_input_coeff[3]*gyro_input[3];

	for(int i = 2; i >= 0; i--){

		gyro_input[i+1] = gyro_input[i]; // store xi

		gyro_output[i+1] = gyro_output[i]; // store yi

	}
}
*/
