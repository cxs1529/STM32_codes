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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WELCOME_MSG "*** Welcome to the Autolauncher V3.0 console test ***\r\n"
//#define MAIN_MENU "Select an option:\r\n1-Toggle green led\r\n2-Set relay\r\n3-Reset relay\r\n4-Run motor CW\r\n5-Run motor CCW\r\n6-MUX change to GPS\r\n7-MUX change to STM32\r\n8-Read Vin\r\n9-Write/read eeprom\r\n0-Read eeprom\r\n"
#define PROMPT "\r\n> Enter command: "
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const char menu_relays[] = "***SELECT AN OPTION***\r\n"
		"\r\n>[0]-Toggle green led"
		"\r\n-- RELAYS --"
		"\r\n>[1]-Set K1 & SSR1\r\n>[2]-Set K2 & SSR2\r\n>[3]-Set K3 & SSR3\r\n>[4]-Set K4 & SSR4"
		"\r\n>[5]-Set K5 & SSR5\r\n>[6]-Set K6 & SSR6\r\n>[7]-Set K7 & SSR7\r\n>[8]-Set K8 & SSR8"
		"\r\n>[9]-Set K9 & K10 GND CONT\r\n>[q]-Set K11 CAL CONT\r\n>[w]-Set K12 CAL RES"
		"\r\n>[e]-Reset 1 K1-4 SSR1-4\r\n>[r]-Reset 2 K5-8 SSR5-8\r\n>[t]-Reset 3 K9-12 AUX RELAYS";
const char menu_functions[] = "\r\n-- AUX FUNCTIONS --"
		"\r\n>[y]-Read Vin\r\n>[u]-Write EEPROM\r\n>[i]-Read EEPROM\r\n>[o]-MUX input GPS-TX\r\n>[p]-MUX input STM32-TX";
const char menu_motors[] = "\r\n-- MOTORS --"
		"\r\n>[a]-Run M1 CW+\r\n>[s]-Run M1 CCW-\r\n>[d]-Run M2 CW+\r\n>[f]-Run M2 CCW-";


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

  // Initialize stepper motors
  HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // start disabled
  HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // start disabled

  //uint8_t option = 0; // Initial option value
  char option = '\0';
  uint8_t result = 0;

  // initialize multiplexer
  // SET = UART-tx / RESET = Din from GPS
  HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, SET);

  // Initialize relays
  // RESET relay k1, k2, k3, k4, SSR1, SSR2, SSR3, SSR4
  HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, SET); // reset relay
  HAL_Delay(10);
  HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, RESET); // release reset coil
  HAL_Delay(5);
  // RESET relay k5, k6, k7, k8, SSR5, SSR6, SSR7, SSR8
  HAL_GPIO_WritePin(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, SET); // reset relay
  HAL_Delay(10);
  HAL_GPIO_WritePin(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, RESET); // release reset coil
  HAL_Delay(5);
  // RESET relay k9, k10, k11, k12 (GND, calibration and continuity circuit)
  HAL_GPIO_WritePin(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, SET); // reset relay
  HAL_Delay(10);
  HAL_GPIO_WritePin(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, RESET); // release reset coil
  HAL_Delay(5);


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
	HAL_UART_Transmit(&huart1, menu_relays, strlen(menu_relays), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, menu_functions, strlen(menu_functions), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, menu_motors, strlen(menu_motors), HAL_MAX_DELAY);
}


/* Read user input and return the option selected 1-2-3*/
char readInput(void){
	char rxBuffer[1];
//	HAL_UART_Transmit(&huart1, menu_relays, strlen(menu_relays), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart1, menu_functions, strlen(menu_functions), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart1, menu_motors, strlen(menu_motors), HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart1, PROMPT, strlen(PROMPT), HAL_MAX_DELAY);
	HAL_UART_Receive(&huart1, rxBuffer, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, rxBuffer, 1, HAL_MAX_DELAY); // echo

	return rxBuffer[0];
}

uint8_t processInput(char option){
	// default
	char msg[50];
	// ADC measurement
	char adcmsg[50];
	float vin = 0.0;
	uint16_t motor_i = 0;
	uint32_t adcReading = 0;
	uint32_t runtime = 5000; // motor runtime ms
	uint32_t t0; // init time for runtime

	// EEPROM
	uint8_t chipAddress = 0xA0; // 0b1010000 7 bit address
	uint8_t dataReceive[10];
	uint8_t dataByte[4]; // = {0x00, 5, 12, 4}; // store 5 in memory 0x00 and 12 in (n+1)= 0x01
	uint8_t startAddress[1] = {0x00};
	char output[50];

	// print what was selected
	sprintf(msg, "\r\n> Executing OPTION %d...\r\n",option);
	HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);

	// Execute selected action
	switch(option){
	case '0': // toggle green led
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		return 0;
	case '1': // Set relay XBT
		// SET relay k1
		HAL_GPIO_WritePin(RELAY_K1_GPIO_Port, RELAY_K1_Pin, SET); // set relay
		HAL_Delay(8); // relays need 4 ms to set/reset
		HAL_GPIO_WritePin(RELAY_K1_GPIO_Port, RELAY_K1_Pin, RESET); // release coil
		HAL_Delay(2);
		// SET SSR1
		HAL_GPIO_WritePin(SSR_1_GPIO_Port, SSR_1_Pin, SET); // set SSR latch
		HAL_Delay(1); // latch need 300 ns to set/reset
		HAL_GPIO_WritePin(SSR_1_GPIO_Port, SSR_1_Pin, RESET); // release SSR latch
		HAL_Delay(1);
		return 0;
	case '2': // Set relay XBT
		// SET relay k2
		HAL_GPIO_WritePin(RELAY_K2_GPIO_Port, RELAY_K2_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K2_GPIO_Port, RELAY_K2_Pin, RESET); // release coil
		HAL_Delay(2);
		// SET SSR2
		HAL_GPIO_WritePin(SSR_2_GPIO_Port, SSR_2_Pin, SET); // set SSR latch
		HAL_Delay(1);
		HAL_GPIO_WritePin(SSR_2_GPIO_Port, SSR_2_Pin, RESET); // release SSR latch
		HAL_Delay(1);
		return 0;
	case '3': // Set relay XBT
		// SET relay k3
		HAL_GPIO_WritePin(RELAY_K3_GPIO_Port, RELAY_K3_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K3_GPIO_Port, RELAY_K3_Pin, RESET); // release coil
		HAL_Delay(2);
		// SET SSR3
		HAL_GPIO_WritePin(SSR_3_GPIO_Port, SSR_3_Pin, SET); // set SSR latch
		HAL_Delay(1);
		HAL_GPIO_WritePin(SSR_3_GPIO_Port, SSR_3_Pin, RESET); // release SSR latch
		HAL_Delay(1);
		return 0;
	case '4': // Set relay XBT
		// SET relay k4
		HAL_GPIO_WritePin(RELAY_K4_GPIO_Port, RELAY_K4_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K4_GPIO_Port, RELAY_K4_Pin, RESET); // release coil
		HAL_Delay(2);
		// SET SSR4
		HAL_GPIO_WritePin(SSR_4_GPIO_Port, SSR_4_Pin, SET); // set SSR latch
		HAL_Delay(1);
		HAL_GPIO_WritePin(SSR_4_GPIO_Port, SSR_4_Pin, RESET); // release SSR latch
		HAL_Delay(1);
		return 0;
	case '5': // Set relay XBT
		// SET relay k5
		HAL_GPIO_WritePin(RELAY_K5_GPIO_Port, RELAY_K5_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K5_GPIO_Port, RELAY_K5_Pin, RESET); // release coil
		HAL_Delay(2);
		// SET SSR5
		HAL_GPIO_WritePin(SSR_5_GPIO_Port, SSR_5_Pin, SET); // set SSR latch
		HAL_Delay(1);
		HAL_GPIO_WritePin(SSR_5_GPIO_Port, SSR_5_Pin, RESET); // release SSR latch
		HAL_Delay(1);
		return 0;
	case '6': // Set relay XBT
		// SET relay k6
		HAL_GPIO_WritePin(RELAY_K6_GPIO_Port, RELAY_K6_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K6_GPIO_Port, RELAY_K6_Pin, RESET); // release coil
		HAL_Delay(2);
		// SET SSR6
		HAL_GPIO_WritePin(SSR_6_GPIO_Port, SSR_6_Pin, SET); // set SSR latch
		HAL_Delay(1);
		HAL_GPIO_WritePin(SSR_6_GPIO_Port, SSR_6_Pin, RESET); // release SSR latch
		HAL_Delay(1);
		return 0;
	case '7': // Set relay XBT
		// SET relay k7
		HAL_GPIO_WritePin(RELAY_K7_GPIO_Port, RELAY_K7_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K7_GPIO_Port, RELAY_K7_Pin, RESET); // release coil
		HAL_Delay(2);
		// SET SSR7
		HAL_GPIO_WritePin(SSR_7_GPIO_Port, SSR_7_Pin, SET); // set SSR latch
		HAL_Delay(1);
		HAL_GPIO_WritePin(SSR_7_GPIO_Port, SSR_7_Pin, RESET); // release SSR latch
		HAL_Delay(1);
		return 0;
	case '8': // Set relay XBT
		// SET relay k8
		HAL_GPIO_WritePin(RELAY_K8_GPIO_Port, RELAY_K8_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K8_GPIO_Port, RELAY_K8_Pin, RESET); // release coil
		HAL_Delay(2);
		// SET SSR8
		HAL_GPIO_WritePin(SSR_8_GPIO_Port, SSR_8_Pin, SET); // set SSR latch
		HAL_Delay(1);
		HAL_GPIO_WritePin(SSR_8_GPIO_Port, SSR_8_Pin, RESET); // release SSR latch
		HAL_Delay(1);
		return 0;
	case '9': // Set relay GND
		// SET relay k9 k10
		HAL_GPIO_WritePin(RELAY_K9_K10_GND_COND_GPIO_Port, RELAY_K9_K10_GND_COND_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K9_K10_GND_COND_GPIO_Port, RELAY_K9_K10_GND_COND_Pin, RESET); // release coil
		HAL_Delay(2);
		return 0;
	case 'q': // Set relay CAL CONT
		// SET relay k11
		HAL_GPIO_WritePin(RELAY_K11_CAL_CONT_GPIO_Port, RELAY_K11_CAL_CONT_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K11_CAL_CONT_GPIO_Port, RELAY_K11_CAL_CONT_Pin, RESET); // release coil
		HAL_Delay(2);
		return 0;
	case 'w': // Set relay CAL RES
		// SET relay k12
		HAL_GPIO_WritePin(RELAY_K12_CAL_RES_GPIO_Port, RELAY_K12_CAL_RES_Pin, SET); // set relay
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_K12_CAL_RES_GPIO_Port, RELAY_K12_CAL_RES_Pin, RESET); // release coil
		HAL_Delay(2);
		return 0;
	case 'e': // Reset 1 (1st half)
		// RESET relay k1, k2, k3, k4, SSR1, SSR2, SSR3, SSR4
		HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, SET); // reset relay and SSR
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, RESET); // release reset coil
		HAL_Delay(2);
		return 0;
	case 'r': // Reset 2 (2nd half)
		// RESET relay k5, k6, k7, k8, SSR5, SSR6, SSR7, SSR8
		HAL_GPIO_WritePin(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, SET); // reset relay and SSR
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, RESET); // release reset coil
		HAL_Delay(2);
		return 0;
	case 't': // Reset 3 (aux relays)
		// RESET relay k9, k10, k11, k12
		HAL_GPIO_WritePin(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, SET); // reset relay and SSR
		HAL_Delay(8);
		HAL_GPIO_WritePin(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, RESET); // release reset coil
		HAL_Delay(2);
		return 0;
	case 'y': // read ADC Vin
		// take 10 an average of samples
		for(uint8_t i=0; i<10; i++){
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adcReading += HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			HAL_Delay(1);
		}
		adcReading = adcReading/10;
		vin = adcReading * 0.0083 + 0.3963; // 15.23
		// get 1 decimal
		int dec = (int)(vin * 10 - ((int)vin * 10)); // 152 - 150 = 2

		sprintf(adcmsg, "[AD# %d] Vin= %d.%d V\r\n", (int)adcReading,(int)vin, dec);
		HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
		return 0;
	case 'u': // write eeprom
		// store 2 8-bit values in eeprom
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
	case 'i': // read epprom
		// read 2 bytes from data address 0x00, 0x01
		HAL_I2C_Master_Transmit(&hi2c1, chipAddress , startAddress, 1, HAL_MAX_DELAY); // dummy write with word address 0x00 as starting address
		HAL_Delay(10);
		HAL_I2C_Master_Receive(&hi2c1, chipAddress, dataReceive, 3, HAL_MAX_DELAY);

		sprintf(output,"Stored values: %i, %i, %i\r\n", dataReceive[0], dataReceive[1], dataReceive[2]);
		HAL_UART_Transmit(&huart1, output, strlen(output), HAL_MAX_DELAY);
		return 0;
	case 'o': // change MUX to GPS
		HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, RESET);
		return 0;
	case 'p': // change MUX to STM32
		HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, SET);
		return 0;
	case 'a': // Run motor CW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
		// motor
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, SET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, SET); // enable driver to run motor
		// read current
		// should launch a timer here and stop it after X seconds
		//uint32_t runtime = 5000; // ms
		t0 = HAL_GetTick();

		for (uint8_t ci = 1; ci < 20; ci++){
			adcReading = 0;
			for(uint8_t cj = 0; cj<100; cj++){
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, 100);
				adcReading += HAL_ADC_GetValue(&hadc2);
				HAL_ADC_Stop(&hadc2);
				HAL_Delay(1);
			}
			adcReading = adcReading/100;
			motor_i = (uint16_t) (adcReading * 0.163 + 7.3581); // mA - opAmp G = 50, Rsense = 0.10 ohm

			sprintf(adcmsg, "[AD# %d] Im_%d = %d mA\r\n", (int)adcReading, ci ,(int) motor_i);
			HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
			HAL_Delay(1000);
			if((HAL_GetTick() - t0) > runtime) break;
		}
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // disable driver
		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );

		return 0;
	case 's': // Run motor CCW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
		// motor
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, RESET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, SET); // enable driver to run motor
		// read current
		// should launch a timer here and stop it after X seconds

		t0 = HAL_GetTick();

		for (uint8_t ci = 1; ci < 20; ci++){
			adcReading = 0;
			for(uint8_t cj = 0; cj<100; cj++){
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, 100);
				adcReading += HAL_ADC_GetValue(&hadc2);
				HAL_ADC_Stop(&hadc2);
				HAL_Delay(1);
			}
			adcReading = adcReading/100;
			motor_i = (uint16_t) (adcReading * 0.163 + 7.3581); // mA - opAmp G = 50, Rsense = 0.10 ohm

			sprintf(adcmsg, "[AD# %d] Im_%d = %d mA\r\n", (int)adcReading, ci ,(int) motor_i);
			HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
			HAL_Delay(1000);
			if((HAL_GetTick() - t0) > runtime) break;
		}
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // disable driver
		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );
		return 0;
	case 'd': // Run motor 2 CW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
		// motor
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, SET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, SET); // enable driver to run motor
		// read current
		// should launch a timer here and stop it after X seconds
		t0 = HAL_GetTick();

		for (uint8_t ci = 1; ci < 20; ci++){
			adcReading = 0;
			for(uint8_t cj = 0; cj<100; cj++){
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, 100);
				adcReading += HAL_ADC_GetValue(&hadc2);
				HAL_ADC_Stop(&hadc2);
				HAL_Delay(1);
			}
			adcReading = adcReading/100;
			motor_i = (uint16_t) (adcReading * 0.163 + 7.3581); // mA - opAmp G = 50, Rsense = 0.10 ohm

			sprintf(adcmsg, "[AD# %d] Im_%d = %d mA\r\n", (int)adcReading, ci ,(int) motor_i);
			HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
			HAL_Delay(1000);
			if((HAL_GetTick() - t0) > runtime) break;
		}
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // disable driver
		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );

		return 0;
	case 'f': // Run motor 2 CCW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
		// motor
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, RESET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, SET); // enable driver to run motor
		// read current
		// should launch a timer here and stop it after X seconds

		t0 = HAL_GetTick();

		for (uint8_t ci = 1; ci < 20; ci++){
			adcReading = 0;
			for(uint8_t cj = 0; cj<100; cj++){
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, 100);
				adcReading += HAL_ADC_GetValue(&hadc2);
				HAL_ADC_Stop(&hadc2);
				HAL_Delay(1);
			}
			adcReading = adcReading/100;
			motor_i = (uint16_t) (adcReading * 0.163 + 7.3581); // mA - opAmp G = 50, Rsense = 0.10 ohm

			sprintf(adcmsg, "[AD# %d] Im_%d = %d mA\r\n", (int)adcReading, ci ,(int) motor_i);
			HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
			HAL_Delay(1000);
			if((HAL_GetTick() - t0) > runtime) break;
		}
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // disable driver
		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );
		return 0;

	default:
		sprintf(msg, "\r\n> %d is not a valid option!");
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
		return 1;

	}



/*
	// OLD
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
		// RESET relay k1, k2, k3, k4, SSR1, SSR2, SSR3, SSR4
		HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, SET); // reset relay and SSR
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, RESET); // release reset coil
		HAL_Delay(5);
		return 0;
	case '4': // Run motor CW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
		// motor
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, SET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, SET); // enable driver to run motor
		// read current
		// should launch a timer here and stop it after X seconds
		//uint32_t runtime = 5000; // ms
		t0 = HAL_GetTick();

		for (uint8_t ci = 1; ci < 20; ci++){
			adcReading = 0;
			for(uint8_t cj = 0; cj<100; cj++){
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, 100);
				adcReading += HAL_ADC_GetValue(&hadc2);
				HAL_ADC_Stop(&hadc2);
				HAL_Delay(1);
			}
			adcReading = adcReading/100;
			motor_i = (uint16_t) (adcReading * 0.163 + 7.3581); // mA - opAmp G = 50, Rsense = 0.10 ohm

			sprintf(adcmsg, "[AD# %d] Im_%d = %d mA\r\n", (int)adcReading, ci ,(int) motor_i);
			HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
			HAL_Delay(1000);
			if((HAL_GetTick() - t0) > runtime) break;
		}
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // disable driver
		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );

		return 0;
	case '5': // Run motor CCW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
		// motor
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, RESET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, SET); // enable driver to run motor
		// read current
		// should launch a timer here and stop it after X seconds

		t0 = HAL_GetTick();

		for (uint8_t ci = 1; ci < 20; ci++){
			adcReading = 0;
			for(uint8_t cj = 0; cj<100; cj++){
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, 100);
				adcReading += HAL_ADC_GetValue(&hadc2);
				HAL_ADC_Stop(&hadc2);
				HAL_Delay(1);
			}
			adcReading = adcReading/100;
			motor_i = (uint16_t) (adcReading * 0.163 + 7.3581); // mA - opAmp G = 50, Rsense = 0.10 ohm

			sprintf(adcmsg, "[AD# %d] Im_%d = %d mA\r\n", (int)adcReading, ci ,(int) motor_i);
			HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
			HAL_Delay(1000);
			if((HAL_GetTick() - t0) > runtime) break;
		}
		HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // disable driver
		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );
		return 0;
	case '6': // change MUX to GPS
		HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, RESET);
		return 0;
	case '7': // change MUX to STM32
		HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, SET);
		return 0;
	case '8': // read ADC Vin
		for(uint8_t i=0; i<10; i++){
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adcReading += HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			HAL_Delay(1);
		}
		adcReading = adcReading/10;
		vin = adcReading * 0.0083 + 0.3963; // 15.23
		// get 1 decimal
		int dec = (int)(vin * 10 - ((int)vin * 10)); // 152 - 150 = 2

		//char adcmsg[30];
		sprintf(adcmsg, "[AD# %d] Vin= %d.%d V\r\n", (int)adcReading,(int)vin, dec);
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
	case 'a': // Set relay
		// SET relay k2
		HAL_GPIO_WritePin(RELAY_K2_GPIO_Port, RELAY_K2_Pin, SET); // set relay
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_K2_GPIO_Port, RELAY_K2_Pin, RESET); // release coil
		HAL_Delay(5);
		// SET SSR XBT2
		HAL_GPIO_WritePin(SSR_XBT2_GPIO_Port, SSR_XBT2_Pin, SET); // set SSR latch
		HAL_Delay(10);
		HAL_GPIO_WritePin(SSR_XBT2_GPIO_Port, SSR_XBT2_Pin, RESET); // release SSR latch
		HAL_Delay(5);
		return 0;
	case 'b': // Set relay
		// SET relay k3
		HAL_GPIO_WritePin(RELAY_K3_GPIO_Port, RELAY_K3_Pin, SET); // set relay
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_K3_GPIO_Port, RELAY_K3_Pin, RESET); // release coil
		HAL_Delay(5);
		// SET SSR XBT3
		HAL_GPIO_WritePin(SSR_XBT3_GPIO_Port, SSR_XBT3_Pin, SET); // set SSR latch
		HAL_Delay(10);
		HAL_GPIO_WritePin(SSR_XBT3_GPIO_Port, SSR_XBT3_Pin, RESET); // release SSR latch
		HAL_Delay(5);
		return 0;
	case 'c': // Set relay
		// SET relay k4
		HAL_GPIO_WritePin(RELAY_K4_GPIO_Port, RELAY_K4_Pin, SET); // set relay
		HAL_Delay(10);
		HAL_GPIO_WritePin(RELAY_K4_GPIO_Port, RELAY_K4_Pin, RESET); // release coil
		HAL_Delay(5);
		// SET SSR XBT4
		HAL_GPIO_WritePin(SSR_XBT4_GPIO_Port, SSR_XBT4_Pin, SET); // set SSR latch
		HAL_Delay(10);
		HAL_GPIO_WritePin(SSR_XBT4_GPIO_Port, SSR_XBT4_Pin, RESET); // release SSR latch
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
	case 'f': // Run motor 2 CW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
		// motor
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, SET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, SET); // enable driver to run motor
		// read current
		// should launch a timer here and stop it after X seconds
		t0 = HAL_GetTick();

		for (uint8_t ci = 1; ci < 20; ci++){
			adcReading = 0;
			for(uint8_t cj = 0; cj<100; cj++){
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, 100);
				adcReading += HAL_ADC_GetValue(&hadc2);
				HAL_ADC_Stop(&hadc2);
				HAL_Delay(1);
			}
			adcReading = adcReading/100;
			motor_i = (uint16_t) (adcReading * 0.163 + 7.3581); // mA - opAmp G = 50, Rsense = 0.10 ohm

			sprintf(adcmsg, "[AD# %d] Im_%d = %d mA\r\n", (int)adcReading, ci ,(int) motor_i);
			HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
			HAL_Delay(1000);
			if((HAL_GetTick() - t0) > runtime) break;
		}
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // disable driver
		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );

		return 0;
	case 'g': // Run motor 2 CCW
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
		// motor
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // disable driver
		HAL_Delay(500); // wait for the motor to stop
		HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, RESET); // set motor direction
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, SET); // enable driver to run motor
		// read current
		// should launch a timer here and stop it after X seconds

		t0 = HAL_GetTick();

		for (uint8_t ci = 1; ci < 20; ci++){
			adcReading = 0;
			for(uint8_t cj = 0; cj<100; cj++){
				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, 100);
				adcReading += HAL_ADC_GetValue(&hadc2);
				HAL_ADC_Stop(&hadc2);
				HAL_Delay(1);
			}
			adcReading = adcReading/100;
			motor_i = (uint16_t) (adcReading * 0.163 + 7.3581); // mA - opAmp G = 50, Rsense = 0.10 ohm

			sprintf(adcmsg, "[AD# %d] Im_%d = %d mA\r\n", (int)adcReading, ci ,(int) motor_i);
			HAL_UART_Transmit(&huart1, adcmsg, strlen(adcmsg), HAL_MAX_DELAY);
			HAL_Delay(1000);
			if((HAL_GetTick() - t0) > runtime) break;
		}
		HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // disable driver
		HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );
		return 0;
	default:
		sprintf(msg, "\r\n> %d is not a valid option!");
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
		return 1;

	}
*/

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