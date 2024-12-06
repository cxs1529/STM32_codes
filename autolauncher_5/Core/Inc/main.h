/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {MUX_GPS, MUX_STM32} mux_t; // TX from GPS MUX=0, TX from microcontroller MUX=1

typedef struct {
	uint8_t serialNumber; // 2 digit autolauncher S/N XX [0-255]
	char tubeCount; // 6 or 8
	char type; // R regular or X extended (amvereseasv6.0, v8.0, v8.1 (long) )
	uint8_t pcbSerial; // 2 digit PCB serialnum ALB3XX [0-255]
	char configured; // if the launcher was configured (stored in memory) this will have a 'Y', otherwise 'N'
} launcher_t;

typedef enum  {reFree, reLocked} relayLock_t; // relay mutex to ensure one coil is driven at a time
typedef enum  {idle, active} rxStatus_t; // flag, indicate if a new char was sent over serial. Set to 'active' in the UART interrupt callback, and 'idle' after processing command
typedef enum  {mainMenu, configMenu} activeMenu_t; // menu state to determine how the received command will be processed (config or main)
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SSR_3_Pin GPIO_PIN_13
#define SSR_3_GPIO_Port GPIOC
#define SSR_2_Pin GPIO_PIN_14
#define SSR_2_GPIO_Port GPIOC
#define SSR_1_Pin GPIO_PIN_15
#define SSR_1_GPIO_Port GPIOC
#define ADC_VOLT_Pin GPIO_PIN_0
#define ADC_VOLT_GPIO_Port GPIOC
#define ADC_AMP_Pin GPIO_PIN_1
#define ADC_AMP_GPIO_Port GPIOC
#define ENABLE_M1_Pin GPIO_PIN_2
#define ENABLE_M1_GPIO_Port GPIOC
#define ENABLE_M2_Pin GPIO_PIN_3
#define ENABLE_M2_GPIO_Port GPIOC
#define ENABLE_M3_Pin GPIO_PIN_0
#define ENABLE_M3_GPIO_Port GPIOA
#define ENABLE_M4_Pin GPIO_PIN_1
#define ENABLE_M4_GPIO_Port GPIOA
#define MUX_SELECT_Pin GPIO_PIN_2
#define MUX_SELECT_GPIO_Port GPIOA
#define ENABLE_M5_Pin GPIO_PIN_4
#define ENABLE_M5_GPIO_Port GPIOA
#define ENABLE_M6_Pin GPIO_PIN_5
#define ENABLE_M6_GPIO_Port GPIOA
#define ENABLE_M7_Pin GPIO_PIN_6
#define ENABLE_M7_GPIO_Port GPIOA
#define ENABLE_M8_Pin GPIO_PIN_7
#define ENABLE_M8_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_4
#define DIR_GPIO_Port GPIOC
#define MOTOR_PWM_Pin GPIO_PIN_0
#define MOTOR_PWM_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define SSR_8_Pin GPIO_PIN_14
#define SSR_8_GPIO_Port GPIOB
#define SSR_7_Pin GPIO_PIN_15
#define SSR_7_GPIO_Port GPIOB
#define SSR_6_Pin GPIO_PIN_6
#define SSR_6_GPIO_Port GPIOC
#define SSR_5_Pin GPIO_PIN_7
#define SSR_5_GPIO_Port GPIOC
#define RELAY_RESET_3_Pin GPIO_PIN_8
#define RELAY_RESET_3_GPIO_Port GPIOC
#define RELAY_RESET_2_Pin GPIO_PIN_9
#define RELAY_RESET_2_GPIO_Port GPIOC
#define RELAY_K12_CAL_RES_Pin GPIO_PIN_8
#define RELAY_K12_CAL_RES_GPIO_Port GPIOA
#define RELAY_K11_CAL_CONT_Pin GPIO_PIN_11
#define RELAY_K11_CAL_CONT_GPIO_Port GPIOA
#define RELAY_K9_K10_GND_COND_Pin GPIO_PIN_12
#define RELAY_K9_K10_GND_COND_GPIO_Port GPIOA
#define RELAY_K8_Pin GPIO_PIN_15
#define RELAY_K8_GPIO_Port GPIOA
#define RELAY_K7_Pin GPIO_PIN_10
#define RELAY_K7_GPIO_Port GPIOC
#define RELAY_K1_Pin GPIO_PIN_11
#define RELAY_K1_GPIO_Port GPIOC
#define RELAY_K2_Pin GPIO_PIN_12
#define RELAY_K2_GPIO_Port GPIOC
#define RELAY_K3_Pin GPIO_PIN_2
#define RELAY_K3_GPIO_Port GPIOD
#define RELAY_K4_Pin GPIO_PIN_3
#define RELAY_K4_GPIO_Port GPIOB
#define RELAY_K5_Pin GPIO_PIN_4
#define RELAY_K5_GPIO_Port GPIOB
#define RELAY_K6_Pin GPIO_PIN_5
#define RELAY_K6_GPIO_Port GPIOB
#define RELAY_RESET_1_Pin GPIO_PIN_8
#define RELAY_RESET_1_GPIO_Port GPIOB
#define SSR_4_Pin GPIO_PIN_9
#define SSR_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
