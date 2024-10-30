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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {CCW, CW} motorDir_t; // Relay Control variables
//typedef enum {XBT1, XBT2, XBT3, XBT4, XBT5, XBT6, XBT7, XBT8, CAL_GND, CAL_CONT, CAL_RES, RESET1, RESET2, RESET3} activeRelay_t;


typedef struct {
	char serialNumber[2]; // 2 digit autolauncher S/N
	char tubeCount[1]; // 6 or 8
	char launcherType[1]; // R regular or X extended (amvereseasv6.0, v8.0, v8.1 (long) )
	char pcbSerial[3]; // PCB serialnum ALB3-XXX
} launcher_t;

typedef struct {
	uint16_t size; // memory size in bytes
	uint16_t maxAddress; // memory address 0x00 - maxAddress >> 0x7F
} eeprom_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXECHO 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum motorLock_t {mFree, mLocked} motorLock = mFree; // motor mutex to ensure one motor runs at a time
enum relayLock_t {reFree, reLocked} relayLock = reFree; // relay mutex to ensure one coil is driven at a time

launcher_t launcher = {"NN", "N", "N", "NNN"};

char configed = '\0';

enum rxStatus_t {idle, active} rxStatus = idle; // flag, indicate if a new char was sent over serial. Set to NEW_CHAR in the UART interrupt callback, and IDLE after processing command
char rxChar[1] = "\0";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//char read_input(uart_echo_t echo); // read incoming character with interrupt
void process_input(char option);
uint8_t processInput(char option); // process the character received
void drive_motor(GPIO_TypeDef * motorPort, uint16_t motorPin, motorDir_t motorDirection, uint32_t runtime ); // drive the desired motor
void drive_relay(GPIO_TypeDef * relayPort, uint16_t relayPin, uint8_t onTime); // drive the desired relay
void relay_init(void); // initialized relays in reset state
void motor_init(void); // disable all motor enable pins
void print(char text[]);
void menu_init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);
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
  // Initialize autolauncher parameters i.e. read eeprom
  parameter_init();

  // Initialize stepper motors
  motor_init();
  motorLock = mFree;

  // initialize multiplexer
  HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, SET); // SET = UART-tx / RESET = Din from GPS

  // Initialize relays
  relay_init();
  relayLock = reFree;

  // menu init
  menu_init();
  HAL_UART_Receive_IT(&huart1, rxChar, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //rxChar = read_input(ECHO_ON);

	  if(active == rxStatus){ // set to active with interrupt
		  //process_input(rxChar); // go to main switch case menu
		  rxStatus = idle;
	  }
	  // do some idle routine here
	  // monitor voltage and send alarm if it's below a threshold
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(1000);


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
/***************************************** START AUTOLAUNCHER FUNCTIONS *****************************************/

void menu_init(void){
	// print status message and anything else required to initialize
	status_message();
}

/* Wrapper, Send a string to the terminal */
void print( char text[]){
	HAL_UART_Transmit_IT(&huart1, text, sizeof(text));
}


/* Initialize autolauncher parameters */
void parameter_init(void){
	// get parameters from eeprom or assign default values

}


/* Read user input and return the option selected 1-2-3*/
//char read_input(uart_echo_t echo){
//	char buffer[1];
//
//	HAL_UART_Receive_IT(&huart1, buffer, 1); // always call again to reactivate UART interrupt
//	if (ECHO_ON == echo){
//		HAL_UART_Transmit(&huart1, buffer, 1, 100); // echo
//	}
//	return buffer[0];
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	//char temp[1];
//	if(huart->Instance == USART1){
//		temp[0] = rxChar[0];
//		HAL_UART_Transmit_IT(&huart1, temp, 1);
//	}
//	HAL_UART_Receive_IT(&huart1, rxChar, 1);
	if(huart->Instance == USART1){ // check that uart1 triggered the interrupt callback
		rxStatus = active;
		if(RXECHO == 1){
			//temp[0] = rxChar[0];
			HAL_UART_Transmit_IT(&huart1, rxChar, 1);
		}
		HAL_UART_Receive_IT(&huart1, rxChar, 1); // reactivate rx interrupt
	}
}

void process_input(char option){
	char msg[50];
	sprintf(msg, "\r\n> Executing OPTION %d...\r\n",option);
	print(msg);
}



void status_message() {
    print("\n\r");
    print("=========================================\n\r");
    print("|  AOML auto launcher board version 3.0 |\n\r");
    print("|  Firmware version 2024.mm.dd.hhmm     |\n\r");
    print("=========================================\n\r");
    print("|    Model #ALV3.0      S/N ");
//    print_serial_number();
//    print("       |\n\r");
    print("=========================================\n\r");
    print("|               COMMANDS                |\n\r");
//    if (configed != '|') {
//        print("|!!! YOU MUST ASSIGN A SERIAL NUMBER !!!|\n\r");
//    }
    print("=========================================\n\r");
    print("| Connect  cal Sim BT  0                |\n\r");
//    if (tubeCount == '6') {
//        print("| Connect  XBT 1-6     1,2,3,4,5,6      |\n\r");
//        print("| Extend   Pin 1-6     U,V,W,X,Y,Z      |\n\r");
//        print("| Retract  Pin 1-6     A,B,C,D,E,F      |\n\r");
//    } else {
//
//        print("| Connect  XBT 1-8     1,2,3,4,5,6,7,8  |\n\r");
//        print("| Extend   Pin 1-8     U,V,W,X,Y,Z,S,T  |\n\r");
//        print("| Retract  Pin 1-8     A,B,C,D,E,F,H,I  |\n\r");
//    }
    print("| Unground   XBT       G                |\n\r");
    print("| Calibrate on         K                |\n\r");
    print("| Cal resistor         L                |\n\r");
    print("| Reset Relays         R                |\n\r");
    print("| Print serial number  s                |\n\r");
    print("| This Menu            M                |\n\r");
    print("=========================================\n\r");
}//end status_message

void print_serial_number(void){
	char msg[5];

    if(configed == '|'){

    	sprintf(msg, "AL%c%s", launcher.launcherType, launcher.serialNumber);
    	print(msg);
    } else {
        if (launcher.tubeCount == '6')
            print("AL6XX");
        else
            print("ALRXX");
    }
}



/* DO NOT USE */
uint8_t processInput(char option){
	// default
	char msg[50];
	// ADC measurement
	char adcmsg[50];
	float vin = 0.0;
	uint32_t adcReading = 0;


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
	case '1': // Set relay XBT1
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K1_GPIO_Port, RELAY_K1_Pin, 10); // SET relay k1
			drive_relay(SSR_1_GPIO_Port, SSR_1_Pin, 1); // SET SSR1
			relayLock = reFree;
		}

		return 0;
	case '2': // Set relay XBT2
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K2_GPIO_Port, RELAY_K2_Pin, 10); // SET relay k1
			drive_relay(SSR_2_GPIO_Port, SSR_2_Pin, 1); // SET SSR1
			relayLock = reFree;
		}

		return 0;
	case '3': // Set relay XBT3
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K3_GPIO_Port, RELAY_K3_Pin, 10); // SET relay k3
			drive_relay(SSR_3_GPIO_Port, SSR_3_Pin, 1); // SET SSR3
			relayLock = reFree;
		}

		return 0;
	case '4': // Set relay XBT4
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K4_GPIO_Port, RELAY_K4_Pin, 10); // SET relay k4
			drive_relay(SSR_4_GPIO_Port, SSR_4_Pin, 1); // SET SSR4
			relayLock = reFree;
		}

		return 0;
	case '5': // Set relay XBT5
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K5_GPIO_Port, RELAY_K5_Pin, 10); // SET relay k5
			drive_relay(SSR_5_GPIO_Port, SSR_5_Pin, 1); // SET SSR5
			relayLock = reFree;
		}


		return 0;
	case '6': // Set relay XBT6
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K6_GPIO_Port, RELAY_K6_Pin, 10); // SET relay k6
			drive_relay(SSR_6_GPIO_Port, SSR_6_Pin, 1); // SET SSR6
			relayLock = reFree;
		}

		return 0;
	case '7': // Set relay XBT7
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K7_GPIO_Port, RELAY_K7_Pin, 10); // SET relay k7
			drive_relay(SSR_7_GPIO_Port, SSR_7_Pin, 1); // SET SSR7
			relayLock = reFree;
		}

		return 0;
	case '8': // Set relay XBT8
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K8_GPIO_Port, RELAY_K8_Pin, 10); // SET relay k8
			drive_relay(SSR_8_GPIO_Port, SSR_8_Pin, 1); // SET SSR8
			relayLock = reFree;
		}

		return 0;
	case '9': // Set relay GND
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K9_K10_GND_COND_GPIO_Port, RELAY_K9_K10_GND_COND_Pin, 10); // SET relay k9 k10
			relayLock = reFree;
		}

		return 0;
	case 'q': // Set relay CAL CONT
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K11_CAL_CONT_GPIO_Port, RELAY_K11_CAL_CONT_Pin, 10); // SET relay k11
			relayLock = reFree;
		}

		return 0;
	case 'w': // Set relay CAL RES
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_K12_CAL_RES_GPIO_Port, RELAY_K12_CAL_RES_Pin, 10); // SET relay k12
			relayLock = reFree;
		}

		return 0;
	case 'e': // Reset 1 (1st half)
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, 10); // RESET relay k1, k2, k3, k4, SSR1, SSR2, SSR3, SSR4
			relayLock = reFree;
		}

		return 0;
	case 'r': // Reset 2 (2nd half)
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, 10); // RESET relay k5, k6, k7, k8, SSR5, SSR6, SSR7, SSR8
			relayLock = reFree;
		}

		return 0;
	case 't': // Reset 3 (aux relays)
		if(relayLock == reFree){
			relayLock = reLocked;
			drive_relay(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, 10); // RESET relay k9, k10, k11, k12
			relayLock = reFree;
		}

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
		vin = adcReading * 0.0083 + 0.3963; // 15.23 store coef. in eeprom
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
	case 'a': // Run motor 1 CW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, CW, 5000);
			motorLock = mFree;
		}
		return 0;

	case 's': // Run motor 1 CCW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, CCW, 5000);
			motorLock = mFree;
		}
		return 0;
	case 'd': // Run motor 2 CW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, CW, 5000);
			motorLock = mFree;
		}
		return 0;
	case 'f': // Run motor 2 CCW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, CCW, 5000);
			motorLock = mFree;
		}
		return 0;
	case 'g': // Run motor 3 CW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M3_GPIO_Port, ENABLE_M3_Pin, CW, 5000);
			motorLock = mFree;
		}
		return 0;
	case 'h': // Run motor 3 CCW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M3_GPIO_Port, ENABLE_M3_Pin, CCW, 5000);
			motorLock = mFree;
		}
		return 0;
	case 'j': // Run motor 4 CW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M4_GPIO_Port, ENABLE_M4_Pin, CW, 5000);
			motorLock = mFree;
		}
		return 0;
	case 'k': // Run motor 4 CCW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M4_GPIO_Port, ENABLE_M4_Pin, CCW, 5000);
			motorLock = mFree;
		}
		return 0;
	case 'l': // Run motor 5 CW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M5_GPIO_Port, ENABLE_M5_Pin, CW, 5000);
			motorLock = mFree;
		}
		return 0;
	case ';': // Run motor 5 CCW
		if(motorLock == mFree){
			motorLock = mLocked;
			drive_motor(ENABLE_M5_GPIO_Port, ENABLE_M5_Pin, CCW, 5000);
			motorLock = mFree;
		}
		return 0;

	default:
		sprintf(msg, "\r\n> %d is not a valid option!", option);
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
		return 1;
	}
}


void relay_init(void){
	drive_relay(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, 10);  // RESET relay k1, k2, k3, k4, SSR1, SSR2, SSR3, SSR4
	drive_relay(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, 10); // RESET relay k5, k6, k7, k8, SSR5, SSR6, SSR7, SSR8
	drive_relay(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, 10); // RESET relay k9, k10, k11, k12 (GND, calibration and continuity circuit)

}


void drive_relay(GPIO_TypeDef * relayPort, uint16_t relayPin, uint8_t onTime){
	// SET relay k
	HAL_GPIO_WritePin(relayPort, relayPin, SET); // set
	HAL_Delay(onTime); // time coil is driven in ms
	HAL_GPIO_WritePin(relayPort, relayPin, RESET); // release
	HAL_Delay(2);
}


void motor_init(void){
	  HAL_GPIO_WritePin(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, RESET); // start disabled
	  HAL_GPIO_WritePin(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, RESET); // start disabled
	  HAL_GPIO_WritePin(ENABLE_M3_GPIO_Port, ENABLE_M3_Pin, RESET); // start disabled
	  HAL_GPIO_WritePin(ENABLE_M4_GPIO_Port, ENABLE_M4_Pin, RESET); // start disabled
	  HAL_GPIO_WritePin(ENABLE_M5_GPIO_Port, ENABLE_M5_Pin, RESET); // start disabled
	  HAL_GPIO_WritePin(ENABLE_M6_GPIO_Port, ENABLE_M6_Pin, RESET); // start disabled
	  HAL_GPIO_WritePin(ENABLE_M7_GPIO_Port, ENABLE_M7_Pin, RESET); // start disabled
	  HAL_GPIO_WritePin(ENABLE_M8_GPIO_Port, ENABLE_M8_Pin, RESET); // start disabled
}

void drive_motor(GPIO_TypeDef * motorPort, uint16_t motorPin, motorDir_t motorDirection, uint32_t runtime ){
	uint32_t t0, adcReading = 0;
	uint16_t motor_i = 0;
	char adcmsg[50];

	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3 ); // Start STEP signal >> counter toggle to toggle every 20/1000 sec = 50hz
	// motor
	HAL_GPIO_WritePin(motorPort, motorPin, RESET); // make sure to disable driver
	HAL_Delay(10); // wait for the motor to stop
	HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, motorDirection); // set motor direction
	HAL_GPIO_WritePin(motorPort, motorPin, SET); // enable driver to run motor
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
	HAL_GPIO_WritePin(motorPort, motorPin, RESET); // disable driver
	HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_3 );

}


/***************************************** END AUTOLAUNCHER FUNCTIONS *****************************************/
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
