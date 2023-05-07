/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "compass.h"
#include "mpu6050.h"
#include "pid.h"
#include "receiver.h"
#include "pwm_esc.h"
#include "e220.h"
#include "adc_Battery.h"
#include "log_to_flash.h"
#include "failsafe.h"
#include "gy63-i2c.h"
#include "gps.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_CONNECTION_LOST 0b000000000000001
#define MIN_Duty_cycle 0
#define motorstopValue 1000
#define PACKET_SIZE 34
#define Sector5_Address 0x08020000   // - 0x0803 FFFF 128 Kbyte
#define Sector6_Address 0x08040000   // - 0x0805 FFFF 128 Kbyte
#define Sector7_Address 0x08060000   // - 0x0807 FFFF 128 Kbyte
#define Sector8_Address 0x08080000   // - 0x0809 FFFF 128 Kbyte
#define Sector9_Address 0x080A0000   // - 0x080B FFFF 128 Kbyte
#define Sector10_Address 0x080C0000  // - 0x080D FFFF 128 Kbyte
#define Sector11_Address 0x080E0000  // - 0x080F FFFF 128 Kbyte
#define Sector12_Address 0x080100000 // - 0x0811 FFFF 128 Kbyte
#define Sector13_Address 0x08120000  // - 0x0813 FFFF 128 Kbyte
#define Sector14_Address 0x08140000  // - 0x0815 FFFF 128 Kbyte
#define Sector15_Address 0x08160000  // - 0x0817 FFFF 128 Kbyte


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_uart7_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */


//////////////////////////////////////////////////////////////
/////////////////////PID RELATED VARIABLES////////////////////
//////////////////////////////////////////////////////////////
float roll_level_adjust, pitch_level_adjust;
extern double Roll_Control,Pitch_Control,Yaw_Control;
int Calculated_Throttle1,Calculated_Throttle2,Calculated_Throttle3,Calculated_Throttle4;
int motorStop=1000;
//////////////////////////////////////////////////////////////
/////////////////////TIMER RELATED VARIABLES//////////////////
//////////////////////////////////////////////////////////////
uint32_t time1,time2,timex;
int t=0;
uint32_t loop_counter=0;
//////////////////////////////////////////////////////////////
///////////////////MPU6050 RELATED VARIABLES//////////////////
//////////////////////////////////////////////////////////////
MPU6050_t MPU6050_1;
double angle_pitch_output, angle_roll_output , angle_yaw_rate_output;
extern float gyro_roll_input,gyro_pitch_input,gyro_yaw_input;
extern float accel_input[4];
extern float accel_output[4];

//////////////////////////////////////////////////////////////
//////////////////RF MODULE RELATED VARIABLES/////////////////
//////////////////////////////////////////////////////////////
extern struct receiverdata My_Receiver;
uint16_t DRONE_STATUS;
uint8_t send_buffer[32]="Hi There!";
uint8_t receive_buffer[32]={0};
int16_t result;
uint8_t all_reg_rx[8], all_reg_tx[8];
struct LoRa_Handler LoraTX={0};
uint8_t rf_listen=0;

//////////////////////////////////////////////////////////////
/////////////////////RECEIVER RELATED VARIABLES///////////////
//////////////////////////////////////////////////////////////
char buffer[25];
int _channels[12];

//////////////////////////////////////////////////////////////
/////////////////////BATTERY RELATED VARIABLES////////////////
//////////////////////////////////////////////////////////////
uint32_t battery_raw_data;
float battery_voltage_float=1;

//////////////////////////////////////////////////////////////
////////////////CALIBRATION RELATED VARIABLES/////////////////
//////////////////////////////////////////////////////////////
float calibration_buffer_float[5];
float imu_calibration_values[6];

//////////////////////////////////////////////////////////////
////////////////FLIGHT LOG RELATED VARIABLES//////////////////
//////////////////////////////////////////////////////////////
uint8_t log_buffer[34];
uint16_t status=0;
uint64_t address_counter=0;
float altitude=725.55;
float real_height=0;

//////////////////////////////////////////////////////////////
//////////////////BAROMETER RELATED VARIABLES/////////////////
//////////////////////////////////////////////////////////////
int loop_counter_pressure_requested=0;
extern struct GY63_t GY63;
float baro_coeff=50;
uint8_t alt_hold_flag;
float alt_setpoint;
extern float Alt_Control;

//////////////////////////////////////////////////////////////
////////////BAROMETER IIR FILTER RELATED VARIABLES////////////
//////////////////////////////////////////////////////////////
float baro_filter_input[4];float baro_filter_output[4];
float baro_filter_output_coeff[3]={2.6235,-2.3146
		,0.6855};
float baro_filter_input_coeff[4]={6.996662426856934e-04,0.002098998728057,
		0.002098998728057,6.996662426856934e-04};


//////////////////////////////////////////////////////////////
///////////////COMPASS & GPS RELATED VARIABLES////////////////
//////////////////////////////////////////////////////////////
uint8_t gps_buffer[100];
int16_t MagX,MagY,MagZ;
uint8_t data_compass[13];
uint8_t MAGZ[6];
float heading;
HAL_StatusTypeDef result_compass;
int16_t compass_offset_y;
float compass_scale_y;
int16_t compass_offset_z;
float compass_scale_z;
int16_t compass_offset_x;
int16_t compass_cal_values[6];
float Xsf,Ysf,Xoff,Yoff,Zsf,Zoff;
extern float heading_corrected;
int gps_connected=0;

//////////////////////////////////////////////////////////////
/////////////////GPS HOLD RELATED VARIABLES///////////////////
//////////////////////////////////////////////////////////////
int gps_hold_flag=0;
uint32_t gps_setpoint_lat,gps_setpoint_lon;
float GpsRollControl,GpsPitchControl;
float GpsRollControl_temp,GpsPitchControl_temp;
double gps_go_result[2];
uint8_t gps_go_flag=0;
uint8_t heading_lock=0;
float heading_error=0;
uint8_t gps_fix=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_UART7_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void fill_buffer();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   __HAL_DBGMCU_FREEZE_IWDG(); ///TURN THE WATCHDOG TIMER OFF IN DEBUG MODE, OTHERWISE MCU WILL KEEP RESETTING ITSELF


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  //MX_IWDG_Init();  // CALL THIS FUNCTION BEFORE MAIN LOOP BEGINS
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  MX_I2C3_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */

  //////TRANSMITTER AND IMU INIT/////
  red_led_on();    // assume no transmitter is connected initially
  while(MPU6050_Init_Benim(&hi2c2)==1);       // connect to imu
  start_reading_battery();                    // start the adc for battery voltage

  while(connect_receiver());                  // connect to transmitter
  red_led_off();  // turn the red led off after connection to transsmitter
  while(startup_handler());   //choose a startup mode: imu calibration ,compass calibration, flight;

 //////RF MODUL INITIALIZATION/////
 E220_init_declare_pins(&LoraTX, GPIOB, GPIOE, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_15,&huart5); //declare E220 pins.
 E220_enter_normal_mode(&LoraTX); //switch the E220 to transceiver


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /////INIT MOTORS/////
  while(arm_handler());           //motors will be activated after this function,
  stop_motors();                    //this function shuts the motors ,

  /////INIT BAROMETER/////
  MS5611_Initilize(); // Never start without this function call
  MS5611_Request_Temp();  //for the timing algorithm to work properly

  /////INIT LOG RECORD/////
  //flash_init_for_log();    //this function erases the sectors from 5 to 13 to flight data logging, warning: This function takes around 4-5 seconds to execute.


  while(gps_init());   //gps initialization
  HAL_TIM_Base_Start(&htim3);              //initiate the loop timer counter
  HAL_Delay(1000);                       // wait 1s to see if the data from gps will corrupt
  while(is_gps_connected()); //wait until gps data frame fits into 100 byte buffer
  green_led_on(); /// turn the green led on, ready for flight
  E220_receive_payload_DMA(&LoraTX,receive_buffer,32);  // start DMA for RF module receive

  MX_IWDG_Init();                            //start the watchdog timer
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  htim3.Instance->CNT=0;         // reset timer
	  time1=htim3.Instance->CNT;     // keep the initial value

	  MPU6050_Read_All_Benim(&hi2c2, &MPU6050_1);    //read the imu data

	  if(decode_receiver()==1) // if the connection with transmitter this function will return 1
	  {
		  red_led_on();  //turn the red led on
		  failsafe_handler();                       //shut motors and wait (mcu will reset itself)
		  //rf_listen=1;       //trivial
		  //_channels[0]=1000;  //trivial
		  //_channels[1]=1000;  //trivial
		  //_channels[3]=1000;  //trivial
		  //if(rf_listen) decode_rf(); //trivial
	  }
	  else {
		  //rf_listen=0;  //trivial
		  red_led_off();

	  }



	  //CALCULATE THE PID CONTROL SIGNALS
	  //adjust parameters are the feedback control signals of the cascaded pid loop
	  pitch_level_adjust = angle_pitch_output * 31;
	  roll_level_adjust = angle_roll_output * 31;
	  calculate_motor_powers();


	  //CALCULATE THE SIGNALS FOR THE MOTORS (PITCH ,ROLL AND YAW)
	  Calculated_Throttle1=(_channels[2] + (int)Roll_Control + (int)Pitch_Control  + (int)Yaw_Control);
	  if(Calculated_Throttle1<1100) Calculated_Throttle1=1100;
	  if(Calculated_Throttle1>1800) Calculated_Throttle1=1800;

	  Calculated_Throttle2=(_channels[2] - (int)Roll_Control + (int)Pitch_Control - (int)Yaw_Control);
	  if(Calculated_Throttle2<1100) Calculated_Throttle2=1100;
	  if(Calculated_Throttle2>1800) Calculated_Throttle2=1800;

	  Calculated_Throttle3=(_channels[2] + (int)Roll_Control - (int)Pitch_Control - (int)Yaw_Control);
	  if(Calculated_Throttle3<1100) Calculated_Throttle3=1100;
	  if(Calculated_Throttle3>1800) Calculated_Throttle3=1800;

	  Calculated_Throttle4=(_channels[2] - (int)Roll_Control - (int)Pitch_Control + (int)Yaw_Control);
	  if(Calculated_Throttle4<1100) Calculated_Throttle4=1100;
	  if(Calculated_Throttle4>1800) Calculated_Throttle4=1800;

	  if(_channels[4]==1000){   //ALT HOLD ENABLED
		  //if(loop_counter%3==0)
		  altitude_hold();
		  status=2;
		  /*
		  calc_gps((double) latitude/1e7,(double) longitude/1e7, 39.938801, 32.818207, gps_go_result);
		  heading_error=gps_go_result[1]-heading_corrected;
		  if(gps_go_result[0]>1){
			  gps_go_flag=1;
		  }
		  else{
			  gps_hold();
			  gps_go_flag=0;
		  }
		  */
	  }
	  else if(_channels[4]==0 && gps_fix==1){ //GPS HOLD ENABLED
		  gps_go_flag=0;
		  gps_hold();
		  altitude_hold();
		  status=3;
	  }

	  else if(_channels[4]==2000 || (_channels[4]==0 && gps_fix==0 && _channels[4]!=1000) ){ //ATTIDE MODE  ENABLED
		  gps_go_flag=0;
		  reset_alt_pid();
		  alt_hold_flag=0;
		  alt_setpoint=0;
		  reset_gps_pid();
		  gps_hold_flag=0;
		  gps_setpoint_lat=0;
		  gps_setpoint_lon=0;
		  status=1;
	  }


	  //APPLY THE PID CONTROL SIGNALS TO THE MOTORS
	  if(_channels[9]==2000){  // if reset pid & shut motors switch is enabled
		  reset_pid();
		  changespeedM1(&motorStop,MIN_Duty_cycle);
		  changespeedM2(&motorStop,MIN_Duty_cycle);
		  changespeedM3(&motorStop,MIN_Duty_cycle);
		  changespeedM4(&motorStop,MIN_Duty_cycle);
		  HAL_IWDG_Refresh(&hiwdg);
	  }
	  else{
		  if(battery_voltage_float>10 && battery_voltage_float<12.70){ // battery voltage compensation
			  Calculated_Throttle1+=(12-battery_voltage_float)*40;
			  Calculated_Throttle2+=(12-battery_voltage_float)*40;
			  Calculated_Throttle3+=(12-battery_voltage_float)*40;
			  Calculated_Throttle4+=(12-battery_voltage_float)*40;
			  if(Calculated_Throttle1<1100) Calculated_Throttle1=1100;
			  if(Calculated_Throttle1>1800) Calculated_Throttle1=1800;
			  if(Calculated_Throttle2<1100) Calculated_Throttle2=1100;
			  if(Calculated_Throttle2>1800) Calculated_Throttle2=1800;
			  if(Calculated_Throttle3<1100) Calculated_Throttle3=1100;
			  if(Calculated_Throttle3>1800) Calculated_Throttle3=1800;
			  if(Calculated_Throttle4<1100) Calculated_Throttle4=1100;
			  if(Calculated_Throttle4>1800) Calculated_Throttle4=1800;
			  changespeedM1(&Calculated_Throttle1,MIN_Duty_cycle);
			  changespeedM2(&Calculated_Throttle2,MIN_Duty_cycle);
			  changespeedM3(&Calculated_Throttle3,MIN_Duty_cycle);
			  changespeedM4(&Calculated_Throttle4,MIN_Duty_cycle);
		  }
		  else{ // if the battery is belov 10 volts, shut the motors for the sake of li-po battery
			  changespeedM1(&Calculated_Throttle1,MIN_Duty_cycle);
			  changespeedM2(&Calculated_Throttle2,MIN_Duty_cycle);
			  changespeedM3(&Calculated_Throttle3,MIN_Duty_cycle);
			  changespeedM4(&Calculated_Throttle4,MIN_Duty_cycle);
			  status=99;
		  }

	  }
	  //CONVERT THE ADC READING INTO VOLTAGE  1.012145748987854 is calibration value
	  battery_voltage_float=(float)battery_raw_data /4095*5.7*3.3*(double)1.012145748987854;

	  //if 100 ms passed, send information to the ground station, rf_listen is trivial
	  if(!rf_listen && loop_counter%25==0)
	  {
		  E220_transmit_payload_DMA(&LoraTX,log_buffer,32);
	  }
	  // fit the information into 32 byte buffer
	  fill_buffer();
	  //
/*
	  if(loop_counter%25==0){  /// log the flight data into flash
		  log_write(log_buffer, &address_counter);
	  }
*/

	  baro_read();  //read the barometer (timing will be handled inside this function)
	  if(loop_counter%50==5 && loop_counter!=0){
		  if(gps_decode(gps_buffer,&latitude,&longitude)==0){ // extract the latitude and longitude information from ubx message
			  gps_fix=1;
		  }
		  else gps_fix=0;
	  }


	  compass_read_corrected(); //Read compass data

	  if(_channels[3]<100 && _channels[2]<1100 && _channels[0]>1800 && _channels[1]>1800){
		  while(1); // left joystick (throttle and yaw) down-right and right joystick (pitch and roll) down-right
		 //this causes mcu to reset
	  }

	  loop_counter++;
	  if(loop_counter==1200){
		  loop_counter=0;
	  }
	  timex=htim3.Instance->CNT-time1;       //how much time did it take the code to come here
	  while((htim3.Instance->CNT-time1)<=4000){  //wait until 4ms passes
	  }
	  time2=(htim3.Instance->CNT-time1);   //how much time did the main loop take
	  HAL_IWDG_Refresh(&hiwdg);       //feed the watchdog timer


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 36;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 96-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 96-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|RF_M0_PIN_Pin|RF_M1_PIN_Pin|LED3_Pin
                          |LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin RF_M0_PIN_Pin RF_M1_PIN_Pin LED3_Pin
                           LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|RF_M0_PIN_Pin|RF_M1_PIN_Pin|LED3_Pin
                          |LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	/*
	 *
	 * 	  x_roll[0]=MPU6050_1.angle_roll;
	y_roll[0] = coeff_a[0]*y_roll[1] + coeff_a[1]*y_roll[2] +coeff_a[2]*y_roll[3] +
	coeff_b[0]*x_roll[0] + coeff_b[1]*x_roll[1] + coeff_b[2]*x_roll[2]+coeff_b[3]*x_roll[3];

	  for(int i = 2; i >= 0; i--){
		  x_roll[i+1] = x_roll[i]; // store xi
	      y_roll[i+1] = y_roll[i]; // store yi
	    }

	 */
	//HAL_UART_AbortTransmit(huart);
	//E220_receive_payload_DMA(&LoraTX,receive_buffer,32);

}


void fill_buffer()
{
	log_buffer[0]='!';
	log_buffer[1]=0;

	log_buffer[3]=(status>>8)&0xFF;
	log_buffer[2]=(status)&0xFF;

	log_buffer[5]=(Calculated_Throttle1>>8)&0xFF;
	log_buffer[4]=(Calculated_Throttle1)&0xFF;

	log_buffer[7]=(Calculated_Throttle2>>8)&0xFF;
	log_buffer[6]=(Calculated_Throttle2)&0xFF;

	log_buffer[9]=(Calculated_Throttle3>>8)&0xFF;
	log_buffer[8]=(Calculated_Throttle3)&0xFF;

	log_buffer[11]=(Calculated_Throttle4>>8)&0xFF;
	log_buffer[10]=(Calculated_Throttle4)&0xFF;

	log_buffer[13]=((int16_t)(battery_voltage_float*100)>>8)&0xFF;
	log_buffer[12]=((int16_t)(battery_voltage_float*100))&0xFF;

	log_buffer[15]=((int16_t)(angle_roll_output*100)>>8)&0xFF;
	log_buffer[14]=((int16_t)(angle_roll_output*100))&0xFF;

	log_buffer[17]=((int16_t)(angle_pitch_output*100)>>8)&0xFF;
	log_buffer[16]=((int16_t)(angle_pitch_output*100))&0xFF;

	log_buffer[19]=((int16_t)(heading_corrected*100)>>8)&0xFF;
	log_buffer[18]=((int16_t)(heading_corrected*100))&0xFF;

	log_buffer[21]=((uint16_t)(baro_filter_output[0]*10)>>8)&0xFF;
	log_buffer[20]=((uint16_t)(baro_filter_output[0]*10))&0xFF;

	log_buffer[23]=((latitude)>>8)&0xFF;
	log_buffer[22]=((latitude))&0xFF;

	log_buffer[25]=((latitude)>>24)&0xFF;
	log_buffer[24]=((latitude)>>16)&0xFF;
/*
	longitude_address
	latitude
	longitude
	*/

	log_buffer[27]=((longitude)>>8)&0xFF;
	log_buffer[26]=((longitude))&0xFF;

	log_buffer[29]=((longitude)>>24)&0xFF;
	log_buffer[28]=((longitude)>>16)&0xFF;

	log_buffer[31]=((int16_t)(gyro_roll_input*10)>>8)&0xFF;
	log_buffer[30]=((int16_t)(gyro_roll_input*10))&0xFF;

	log_buffer[33]=((int16_t)(gyro_pitch_input*10)>>8)&0xFF;
	log_buffer[32]=((int16_t)(gyro_pitch_input*10))&0xFF;


}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
