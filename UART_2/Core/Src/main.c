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
// Check ANSI formats for console display:
// https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WELCOME_MSG "Welcome to the UART_2 console test\r\n"
#define MAIN_MENU "Select an option:\r\n1-Toggle green led\r\n2-Toggle blue led\r\n3-Make a sound"
#define PROMPT "\r\n>"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void printWelcomeMessage(void); // Print initial message
uint8_t readInput(void);
uint8_t processInput(uint8_t option);
void printMessage(void);


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
	uint8_t option = 0; // Initial option value
	uint8_t result = 0;


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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  printMessage:

	  printWelcomeMessage();
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
		  option = readInput();
		  result = processInput(option);
		  if(result==1) goto printMessage;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* Print welcome message at startup*/
void printWelcomeMessage(void){
	HAL_UART_Transmit(&huart1, WELCOME_MSG, strlen(WELCOME_MSG), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, MAIN_MENU, strlen(MAIN_MENU), HAL_MAX_DELAY);
}


/* Read user input and return the option selected 1-2-3*/
uint8_t readInput(void){
	char rxBuffer[1];

	HAL_UART_Transmit(&huart1, PROMPT, strlen(PROMPT), HAL_MAX_DELAY);
	HAL_UART_Receive(&huart1, rxBuffer, 1, HAL_MAX_DELAY);

	return atoi(rxBuffer);
}

uint8_t processInput(uint8_t option){
	char msg[30];

	// print what was selected
	sprintf(msg, "\r\n>Executing OPTION %d...\r\n",option);
	HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);

	// Execute selected action
	switch(option){
	case 1: // toggle green led
		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
		return 0;
	case 2: // toggle blue led
		HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
		return 0;
	case 3:
		HAL_UART_Transmit(&huart1, "\a", 1, HAL_MAX_DELAY);
		return 0;
	default:
		sprintf(msg, "\r\n> %d is not a valid option");
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
		return 1;

	}

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
