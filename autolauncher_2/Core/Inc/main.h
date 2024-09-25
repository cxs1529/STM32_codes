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
#define SSR_XBT3_Pin GPIO_PIN_13
#define SSR_XBT3_GPIO_Port GPIOC
#define SSR_XBT2_Pin GPIO_PIN_14
#define SSR_XBT2_GPIO_Port GPIOC
#define SSR_XBT1_Pin GPIO_PIN_15
#define SSR_XBT1_GPIO_Port GPIOC
#define ENABLE_M1_Pin GPIO_PIN_2
#define ENABLE_M1_GPIO_Port GPIOC
#define ENABLE_M2_Pin GPIO_PIN_3
#define ENABLE_M2_GPIO_Port GPIOC
#define MUX_SELECT_Pin GPIO_PIN_2
#define MUX_SELECT_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_4
#define DIR_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define RELAY_CAL_RES_Pin GPIO_PIN_8
#define RELAY_CAL_RES_GPIO_Port GPIOA
#define RELAY_CAL_CONT_Pin GPIO_PIN_11
#define RELAY_CAL_CONT_GPIO_Port GPIOA
#define RELAY_GND_COND_Pin GPIO_PIN_12
#define RELAY_GND_COND_GPIO_Port GPIOA
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
#define SSR_XBT4_Pin GPIO_PIN_9
#define SSR_XBT4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
