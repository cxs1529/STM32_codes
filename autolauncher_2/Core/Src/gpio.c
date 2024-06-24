/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  /* original
  HAL_GPIO_WritePin(GPIOC, SSR_XBT3_Pin|SSR_XBT2_Pin|SSR_XBT1_Pin|ENABLE_M1_Pin
                          |DIR_Pin|RELAY_K7_Pin|RELAY_K1_Pin|RELAY_K2_Pin, GPIO_PIN_RESET);
                          */
  HAL_GPIO_WritePin(GPIOC, SSR_XBT3_Pin|SSR_XBT2_Pin|SSR_XBT1_Pin
                          |DIR_Pin|RELAY_K7_Pin|RELAY_K1_Pin|RELAY_K2_Pin, GPIO_PIN_RESET);
  // Start ENABLE_M1_Pin HIGH
  HAL_GPIO_WritePin(GPIOC, ENABLE_M1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MUX_SELECT_Pin|RELAY_CAL_RES_Pin|RELAY_CAL_CONT_Pin|RELAY_GND_COND_Pin
                          |RELAY_K8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|RELAY_K4_Pin|RELAY_K5_Pin|RELAY_K6_Pin
                          |RELAY_RESET_1_Pin|SSR_XBT4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_K3_GPIO_Port, RELAY_K3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = SSR_XBT3_Pin|SSR_XBT2_Pin|SSR_XBT1_Pin|ENABLE_M1_Pin
                          |DIR_Pin|RELAY_K7_Pin|RELAY_K1_Pin|RELAY_K2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                           PAPin */
  GPIO_InitStruct.Pin = MUX_SELECT_Pin|RELAY_CAL_RES_Pin|RELAY_CAL_CONT_Pin|RELAY_GND_COND_Pin
                          |RELAY_K8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin */
  GPIO_InitStruct.Pin = LED_Pin|RELAY_K4_Pin|RELAY_K5_Pin|RELAY_K6_Pin
                          |RELAY_RESET_1_Pin|SSR_XBT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = RELAY_K3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_K3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
