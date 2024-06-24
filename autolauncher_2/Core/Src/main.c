/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WELCOME_MSG "Welcome to the Autolauncher console test\r\n"
//#define MAIN_MENU "Select an option:\r\n1-Toggle green led\r\n2-Set relay\r\n3-Reset relay\r\n4-Run motor CW\r\n5-Run motor CCW\r\n6-MUX change to GPS\r\n7-MUX change to STM32\r\n8-Read Vin\r\n9-Write/read eeprom\r\n0-Read eeprom\r\n"
#define PROMPT "\r\n> Enter command: "
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const char menu[] = "Select an option:"
		"\r\n0-Read eeprom\r\n1-Toggle green led\r\n2-Set relay K1\r\n3-Reset relays 1-6\r\n4-Run motor CW\r\n5-Run motor CCW"
		"\r\n6-MUX change to GPS\r\n7-MUX change to STM32\r\n8-Read Vin\r\n9-Write/read eeprom"
		"\r\na-Set relay K2\r\nb-Set relay K3\r\nc-Set relay K4\r\nd-Set relay K5\r\ne-Set relay K6\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void printWelcomeMessage(void); // Print initial message
//uint8_t readInput(void);
char readInput(void);
//uint8_t processInput(uint8_t option);
uint8_t processInput(char option);
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, SET); // Active low >> start disabled

  //uint8_t option = 0; // Initial option value
  char option = '\0';
  uint8_t result = 0;

  // SET = UART-tx / RESET = Din from GPS
  HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, SET);

  // Initialize relay in reset state
  HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, SET); // reset relay
  HAL_Delay(10);
  HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, RESET); // release reset coil
  HAL_Delay(5);

  // Start timer for STEP signal
  //HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz


  printWelcomeMessage();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  option = readInput();
	  result = processInput(option);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void printWelcomeMessage(void){
	HAL_UART_Transmit(&huart1, WELCOME_MSG, strlen(WELCOME_MSG), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, menu, strlen(menu), HAL_MAX_DELAY);
}


/* Read user input and return the option selected 1-2-3*/
char readInput(void){
	char rxBuffer[1];

	HAL_UART_Transmit(&huart1, PROMPT, strlen(PROMPT), HAL_MAX_DELAY);
	HAL_UART_Receive(&huart1, rxBuffer, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, rxBuffer, 1, HAL_MAX_DELAY); // echo

	//return atoi(rxBuffer);
	return rxBuffer[0];
}

uint8_t processInput(char option){
	// default
	char msg[30];
	// ADC measurement
	char adcmsg[30];
	float vin = 0.0;
	uint16_t adcReading = 0;
	// EEPROM
	uint8_t chipAddress = 0xA0; // 0b1010000 7 bit address
	uint8_t dataReceive[10];
	uint8_t dataByte[4]; // = {0x00, 5, 12, 4}; // store 5 in memory 0x00 and 12 in (n+1)= 0x01
	uint8_t startAddress[1] = {0x00};
	char output[20];

	// print what was selected
	sprintf(msg, "\r\n> Executing OPTION %d...\r\n",option);
	HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);

	// Execute selected action
	switch(option){
	case 'a': // Set relay
		// SET relay k2
		HAL_GPIO_WritePin(RELAY_K2_GPIO_Port, RELAY_K2_Pin, SET); // set relay
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_K2_GPIO_Port, RELAY_K2_Pin, RESET); // release coil
		HAL_Delay(5);
		return 0;
	case 'b': // Set relay
		// SET relay k3
		HAL_GPIO_WritePin(RELAY_K3_GPIO_Port, RELAY_K3_Pin, SET); // set relay
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_K3_GPIO_Port, RELAY_K3_Pin, RESET); // release coil
		HAL_Delay(5);
		return 0;
	case 'c': // Set relay
		// SET relay k4
		HAL_GPIO_WritePin(RELAY_K4_GPIO_Port, RELAY_K4_Pin, SET); // set relay
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_K4_GPIO_Port, RELAY_K4_Pin, RESET); // release coil
		HAL_Delay(5);
		return 0;
	case 'd': // Set relay
		// SET relay k5
		HAL_GPIO_WritePin(RELAY_K5_GPIO_Port, RELAY_K5_Pin, SET); // set relay
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_K5_GPIO_Port, RELAY_K5_Pin, RESET); // release coil
		HAL_Delay(5);
		return 0;
	case 'e': // Set relay
		// SET relay k6
		HAL_GPIO_WritePin(RELAY_K6_GPIO_Port, RELAY_K6_Pin, SET); // set relay
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_K6_GPIO_Port, RELAY_K6_Pin, RESET); // release coil
		HAL_Delay(5);
		return 0;
	case '1': // toggle green led
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		return 0;
	case '2': // Set relay
		// SET relay k1
		HAL_GPIO_WritePin(RELAY_K1_GPIO_Port, RELAY_K1_Pin, SET); // set relay
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_K1_GPIO_Port, RELAY_K1_Pin, RESET); // release coil
		HAL_Delay(5);
		// SET SSR1 XBT1
		HAL_GPIO_WritePin(SSR_XBT1_GPIO_Port, SSR_XBT1_Pin, SET); // set SSR latch
		HAL_Delay(10);
		HAL_GPIO_WritePin(SSR_XBT1_GPIO_Port, SSR_XBT1_Pin, RESET); // release SSR latch
		HAL_Delay(5);
		return 0;
	case '3': // Reset relay
		// RESET relay k1
		HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, SET); // reset relay and SSR
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, RESET); // release reset coil
		HAL_Delay(5);
		return 0;
	case '4': // Run motor CW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
		// motor
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, SET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, SET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // enable driver to run motor
		// read current
		HAL_ADC_Start(&hadc2);

		HAL_ADC_PollForConversion(&hadc2, 100);
		adcReading = (uint16_t) HAL_ADC_GetValue(&hadc2);
		sprintf(adcmsg, "Im_1 = %d\r\n", (uint32_t) adcReading);
		HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
		HAL_Delay(1000);

		HAL_ADC_PollForConversion(&hadc2, 100);
		adcReading = (uint16_t) HAL_ADC_GetValue(&hadc2);
		sprintf(adcmsg, "Im_2 = %d\r\n", (uint32_t) adcReading);
		HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
		HAL_Delay(1000);

		HAL_ADC_PollForConversion(&hadc2, 100);
		adcReading = (uint16_t) HAL_ADC_GetValue(&hadc2);
		sprintf(adcmsg, "Im_3 = %d\r\n", (uint32_t) adcReading);
		HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
		HAL_Delay(1000);

		HAL_ADC_PollForConversion(&hadc2, 100);
		adcReading = (uint16_t) HAL_ADC_GetValue(&hadc2);
		sprintf(adcmsg, "Im_4 = %d\r\n", (uint32_t) adcReading);
		HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
		HAL_Delay(1000);

		HAL_ADC_Stop(&hadc1);

		//HAL_Delay(5000); // run 5 seconds
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, SET); // disable driver
		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );

		return 0;
	case '5': // Run motor CCW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 );
		// motor
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, SET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, RESET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // enable driver to run motor
		HAL_Delay(5000); // run 5 seconds
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, SET); // disable driver

		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );
		return 0;
	case '6': // change MUX to GPS
		HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, RESET);
		return 0;
	case '7': // change MUX to STM32
		HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, SET);
		return 0;
	case '8': // read ADC Vin
		HAL_ADC_Start(&hadc1);
		for(uint8_t i=0; i<10; i++){
			HAL_ADC_PollForConversion(&hadc1, 100);
			adcReading += (uint16_t) HAL_ADC_GetValue(&hadc1);
		}
		HAL_ADC_Stop(&hadc1);
		adcReading = adcReading/10;
		vin = adcReading * 17.0/1867.0;

		//char adcmsg[30];
		sprintf(adcmsg, "Vin = %d ", (int)vin);
		HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
		return 0;
	case '9':
		// get values to store in eeprom
		uint8_t a,b;
		char rxBuffer[1];
		HAL_UART_Transmit(&huart1, "Val-1= ", strlen("Val-1= "), HAL_MAX_DELAY);
		HAL_UART_Receive(&huart1, rxBuffer, 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, rxBuffer, 1, HAL_MAX_DELAY); // echo
		a = atoi(rxBuffer);
		HAL_UART_Transmit(&huart1, "\r\nVal-2= ", strlen("\r\nVal-2= "), HAL_MAX_DELAY);
		HAL_UART_Receive(&huart1, rxBuffer, 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, rxBuffer, 1, HAL_MAX_DELAY); // echo
		b = atoi(rxBuffer);

		// store 2 bytes starting in address 0x00 total 1 kbit = 1024 bit = 128 bytes
		// 1 page = 8 bytes >> 16 pages >> (0x00) 0-7, 8-15, 16-23, ... 120-127 (0x7F)
		dataByte[0] = 0x00;
		dataByte[1] = a;
		dataByte[2] = b;
		dataByte[3] = 249; // 0-255 8bits
		HAL_I2C_Master_Transmit(&hi2c1, chipAddress , dataByte, 4, HAL_MAX_DELAY); // send word address, value
		HAL_Delay(10);

		// read 2 bytes from data address 0x00, 0x01
		HAL_I2C_Master_Transmit(&hi2c1, chipAddress , startAddress, 1, HAL_MAX_DELAY); // dummy write with word address 0x00 as starting address
		HAL_Delay(10);
		HAL_I2C_Master_Receive(&hi2c1, chipAddress, dataReceive, 3, HAL_MAX_DELAY);

		sprintf(output,"\r\nStored values: %i, %i, %i\r\n", dataReceive[0], dataReceive[1], dataReceive[2]);
		HAL_UART_Transmit(&huart1, output, strlen(output), HAL_MAX_DELAY);
		return 0;
	case '0':
		// read 2 bytes from data address 0x00, 0x01
		HAL_I2C_Master_Transmit(&hi2c1, chipAddress , startAddress, 1, HAL_MAX_DELAY); // dummy write with word address 0x00 as starting address
		HAL_Delay(10);
		HAL_I2C_Master_Receive(&hi2c1, chipAddress, dataReceive, 3, HAL_MAX_DELAY);

		sprintf(output,"Stored values: %i, %i, %i\r\n", dataReceive[0], dataReceive[1], dataReceive[2]);
		HAL_UART_Transmit(&huart1, output, strlen(output), HAL_MAX_DELAY);
		return 0;
	default:
		sprintf(msg, "\r\n> %d is not a valid option!");
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
