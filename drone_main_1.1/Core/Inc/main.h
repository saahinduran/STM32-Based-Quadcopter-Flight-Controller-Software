/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_SDA_Pin GPIO_PIN_0
#define IMU_SDA_GPIO_Port GPIOF
#define IMU_SCL_Pin GPIO_PIN_1
#define IMU_SCL_GPIO_Port GPIOF
#define KUMANDA_TX_Pin GPIO_PIN_2
#define KUMANDA_TX_GPIO_Port GPIOA
#define KUMANDA_RX_Pin GPIO_PIN_3
#define KUMANDA_RX_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define BATARYA_VOLTAJ_GIRIS_Pin GPIO_PIN_1
#define BATARYA_VOLTAJ_GIRIS_GPIO_Port GPIOB
#define MOTOR1_Pin GPIO_PIN_9
#define MOTOR1_GPIO_Port GPIOE
#define MOTOR2_Pin GPIO_PIN_11
#define MOTOR2_GPIO_Port GPIOE
#define MOTOR3_Pin GPIO_PIN_13
#define MOTOR3_GPIO_Port GPIOE
#define MOTOR4_Pin GPIO_PIN_14
#define MOTOR4_GPIO_Port GPIOE
#define RF_M0_PIN_Pin GPIO_PIN_10
#define RF_M0_PIN_GPIO_Port GPIOB
#define RF_M1_PIN_Pin GPIO_PIN_11
#define RF_M1_PIN_GPIO_Port GPIOB
#define RF_RX_Pin GPIO_PIN_12
#define RF_RX_GPIO_Port GPIOB
#define RF_TX_Pin GPIO_PIN_13
#define RF_TX_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_6
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_7
#define GPS_RX_GPIO_Port GPIOC
#define PUSULA_SDA_Pin GPIO_PIN_9
#define PUSULA_SDA_GPIO_Port GPIOC
#define PUSULA_SCL_Pin GPIO_PIN_8
#define PUSULA_SCL_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB
#define BARO_SCL_Pin GPIO_PIN_8
#define BARO_SCL_GPIO_Port GPIOB
#define BARO_SDA_Pin GPIO_PIN_9
#define BARO_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
