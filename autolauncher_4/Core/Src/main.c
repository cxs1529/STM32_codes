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
typedef enum {MUX_GPS, MUX_STM32} mux_t; // TX from GPS MUX=0, TX from microcontroller MUX=1

typedef struct {
	char serialNumber[3]; // 2 digit autolauncher S/N + NULL termination
	char tubeCount; // 6 or 8
	char type; // R regular or X extended (amvereseasv6.0, v8.0, v8.1 (long) )
	char pcbSerial[4]; // 3 digit PCB serialnum ALB3-XXX + NULL termination
} launcher_t;

typedef struct {
	uint16_t size; // memory size in bytes
	uint16_t maxAddress; // memory address 0x00 - maxAddress >> 0x7F
	char configured;
} eeprom_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum motorLock_t {mFree, mLocked} motorLock = mFree; // motor mutex to ensure one motor runs at a time
enum relayLock_t {reFree, reLocked} relayLock = reFree; // relay mutex to ensure one coil is driven at a time
enum rxStatus_t {idle, active} rxStatus = idle; // flag, indicate if a new char was sent over serial. Set to NEW_CHAR in the UART interrupt callback, and IDLE after processing command
enum activeMenu_t {mainMenu, configMenu} activeMenu = mainMenu;
launcher_t launcher = {"00", '?', '?', "000"};
eeprom_t eeprom = {1024, 0x7F, '\0'};
char rxBuffer[1] = "\0"; // UART1 receive buffer from computer, a char will be stored here with UART interrupt
char rxChar = '\0'; // UART1 receive character, == xBuffer[0]
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Auxiliar functions
void print_inline(char * text);
uint8_t is_num(char c);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);
void print_serial_number(void);

// Menu control functions
void menu_init(void);
void menu_main(void);
void menu_config(void);
void main_process_input(char option); // process the character received if in main menu
void config_process_input(char option); // process the character received if in config menu
uint8_t processInput(char option); // process the character received *NOT IN USE*

// Relay control functions
void connect_xbt_pin(uint8_t xbtNum); // connect ABC to a desired XBT 1-8
void calibrate_on(void);
void calibration_resistor(void);
void unground_xbt(void);
void reset_relay(void); // reset all relays, ground ABC
void drive_relay(GPIO_TypeDef * relayPort, uint16_t relayPin, uint8_t onTime); // drive the desired relay: port, pin & time coil is energized [ms]
void relay_init(void); // initialized relays in reset state

// Motor control functions
void drive_motor(GPIO_TypeDef * motorPort, uint16_t motorPin, motorDir_t motorDirection, uint32_t runtime ); // drive the desired motor
void drive_relay(GPIO_TypeDef * relayPort, uint16_t relayPin, uint8_t onTime); // drive the desired relay
void relay_init(void); // initialized relays in reset state
void motor_init(void); // disable all motor enable pins

// RS232 control
void multiplexer_set(mux_t select);

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
  // initialize multiplexer
  multiplexer_set(MUX_STM32);
  // Initialize relays
  relay_init();
  // menu init
  //menu_init();
  // enable receive interrupt
  HAL_UART_Receive_IT(&huart1, rxBuffer, 1); // enable UART receive interrupt, store received char in rxChar buffer
  // display main menu at startup
  menu_main();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // menu control loop
	  if(active == rxStatus){ // set to active with UART RX interrupt
		  rxStatus = idle;
		  if( mainMenu == activeMenu){
			  main_process_input(rxChar); // go to main switch case menu
		  } else if ( configMenu == activeMenu){
			  config_process_input(rxChar);
		  }
	  }
	  // monitor voltage and send alarm if it's below a threshold
	  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(100);

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


void print_inline(char * text){
	char temp = ' ';
	for(uint8_t i = 0; i<=255 && temp!= '\0' ; i++){
		temp = text[i];
		HAL_UART_Transmit(&huart1, (uint8_t *) &temp, 1, 100);
	}
}

/* Support printf over UART
   Warning: printf() only empties the buffer and prints after seeing an \n */
int __io_putchar(int ch){
	(void) HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

/* Initialize autolauncher parameters */
void parameter_init(void){
	// get parameters from eeprom or assign default values

}

/* UART Receive interrupt callback, set rxStatus flag for new char received and echo, re-enable uart rx interrupt*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	// check that uart1 triggered the interrupt callback
	if(huart->Instance == USART1){
		rxChar = rxBuffer[0]; // store the only element in buffer to a char for easier variable handling
		rxStatus = active; // set flag to enter main menu char processing
		HAL_UART_Receive_IT(&huart1, (uint8_t *) rxBuffer, 1); // reactivate rx interrupt
	}
}

/* Process char received while in Main menu */
void main_process_input(char option){
	printf("\r\n> Executing OPTION (%c) ...\r\n", option);

	switch (option){
		// Connect XBT pins
    case '0':
        //engage calibration resistor
    	printf(" -- unground_xbt(),calibration_resistor(),calibrate_on() --\r\n");
        unground_xbt();
        calibration_resistor();
        calibrate_on();
        break;
    case '1':
    	printf("connect_xbt_pin(1)\r\n");
        connect_xbt_pin(1);
        break;
    case '2':
        connect_xbt_pin(2);
        printf("connect_xbt_pin(2)\r\n");
        break;
    case '3':
        connect_xbt_pin(3);
        printf("connect_xbt_pin(3)\r\n");
        break;
    case '4':
        connect_xbt_pin(4);
        printf("connect_xbt_pin(4)\r\n");
        break;
    case '5':
        connect_xbt_pin(5);
        printf("connect_xbt_pin(5)\r\n");
        break;
    case '6':
        connect_xbt_pin(6);
        printf("connect_xbt_pin(6)\r\n");
        break;
    case '7':
        if (launcher.tubeCount == '8'){
            connect_xbt_pin(7);
            printf("connect_xbt_pin(7)\r\n");
        } else {
        	printf("\r\n* ERROR: tube 7 not available *\r\n");
        }
        break;
    case '8':
        if (launcher.tubeCount == '8'){
        	connect_xbt_pin(8);
        	printf("connect_xbt_pin(8)\r\n");
        } else {
        	printf("\r\n* ERROR: tube 8 not available *\r\n");
        }
        break;
        //EXTEND PINS
    case 'U':
        //extend_pin('1');
    	printf("extend_pin(1)\r\n");
        break;
    case 'V':
        //extend_pin('2');
        printf("extend_pin(2)\r\n");
        break;
    case 'W':
        //extend_pin('3');
        printf("extend_pin(3)\r\n");
        break;
    case 'X':
        //extend_pin('4');
        printf("extend_pin(4)\r\n");
        break;
    case 'Y':
        //extend_pin('5');
        printf("extend_pin(5)\r\n");
        break;
    case 'Z':
        //extend_pin('6');
        printf("extend_pin(6)\r\n");
        break;
    case 'S':
        if (launcher.tubeCount == '8'){
        	//extend_pin('7');
        	printf("extend_pin(7)\r\n");
        } else {
        	printf("Error, tube 7 not available\r\n");
        }
        break;
    case 'T':
        if (launcher.tubeCount == '8'){
        	//extend_pin('8');
        	printf("extend_pin(8)\r\n");
        } else {
        	printf("Error, tube 8 not available\r\n");
        }
        break;
        //RETRACT PINS
    case 'A':
        //retract_pin('1');
    	printf("retract_pin(1)\r\n");
        break;
    case 'B':
        //retract_pin('2');
        printf("retract_pin(2)\r\n");
        break;
    case 'C':
        //retract_pin('3');
        printf("retract_pin(3)\r\n");
        break;
    case 'D':
        //retract_pin('4');
        printf("retract_pin(4)\r\n");
        break;
    case 'E':
        //retract_pin('5');
        printf("retract_pin(5)\r\n");
        break;
    case 'F':
        //retract_pin('6');
        printf("retract_pin(6)\r\n");
        break;
    case 'H':
        if (launcher.tubeCount == '8'){
        	//retract_pin('7');
        	printf("retract_pin(7)\r\n");
        } else {
        	printf("Error, tube 7 not available\r\n");
        }
        break;
    case 'I':
        if (launcher.tubeCount == '8'){
        	//retract_pin('8');
        	printf("retract_pin(8)\r\n");
        } else {
        	printf("Error, tube 8 not available\r\n");
        }
        break;
    case 'K':
    	printf("calibrate_on()\r\n");
        calibrate_on();
        break;
    case 'R':
    	printf("reset_relay()\r\n");
        reset_relay();
        break;
    case 'L':
    	printf("calibration_resistor()\r\n");
        calibration_resistor();
        break;
    case 'G':
    	printf("unground_xbt()\r\n");
        unground_xbt();
        break;
    case 'M':
        menu_main();
        break;
    case '~':
        menu_config();
        activeMenu = configMenu; // set configuration menu flag
        break;
    case 's':
        print_serial_number();
        printf("\r\n");
        break;
    default:
        printf("\r\n** Unrecognized command!!** \r\n");
        break;
	}
}

/* Process char received while in configuration menu */
void config_process_input(char option){
    switch (option) {
        case 'Q':
            printf("\n\rLeaving Auto launcher configuration menu\n\r");
            activeMenu = mainMenu; // set active menu flag to main menu
            menu_main();
            break;
        case 'M':
            menu_config();
            break;
        case '1':
        	// get the autolauncher tube count
            print_inline("\r\nEnter AL tube count [6] or [8]: ");
            while(1){
            	if(rxStatus == active){
            		rxStatus = idle;
            		printf(" %c\r\n", rxChar);
            		if(rxChar == '6' || rxChar == '8'){
            			launcher.tubeCount = rxChar;
            			break;
            		} else {
            			printf("\r\nError, Enter 6 or 8 !\r\n");
            			print_inline("\r\nEnter AL tube count [6] or [8]: ");
            		}
            	}
            }
            // get the autolauncher type, R regular or X extended, only for 8 tube AL
            if(launcher.tubeCount == '8'){
            	print_inline("Enter launcher type, [X] extended or [R] regular: ");
                while(1){
                	if(rxStatus == active){
                		rxStatus = idle;
                		printf(" %c\r\n", rxChar);
                		if(rxChar == 'X' || rxChar == 'R'){
                			launcher.type = rxChar;
                			break;
                		} else {
                			printf("\r\nError, Enter X or R !\r\n");
                			print_inline("Enter launcher type, [X] extended or [R] regular: ");
                		}
                	}
                }
            } else {
            	launcher.type = '?'; // if not 8 tubes, reset type to unknown
            }
            print_inline("Enter a two-digit autolauncher serial number [0-99]: ");
            // get the 2 digit serial number
    		for(uint8_t i = 0; i < 2; i++){
    			while(1){
					if(rxStatus == active){
						rxStatus = idle;
						if(is_num(rxChar) == 1){ // check it's a number to store it
							launcher.serialNumber[i] = rxChar;
							break;
						} else {
							printf("\r\nEnter only numbers!\r\n");
							i = 0; // restart index count
							print_inline("Enter a two-digit autolauncher serial number [0-99]: ");
						}
    				}
    			}
    		}
    		printf("%s\r\n", launcher.serialNumber);
    		// set the AL configured flag and print configuration
            eeprom.configured = '|';
            printf("\r\nNew autolauncher configuration: Tubes: %c | Type: %c | Serial: %s\r\n", launcher.tubeCount, launcher.type, launcher.serialNumber);

            // store parameters in eeprom
            printf("Settings saved!\r\n");
//            eeprom_write(0x00, numOfTubes);
//            eeprom_write(0x01, launcherLength);
//            eeprom_write(0x02, num1);
//            eeprom_write(0x03, num2);
//            eeprom_write(0x04, configed);

            //activeMenu = configMenu;
            menu_config();
            break;
        case 'J':
            //extend_all_pins();
        	printf("extend_all_pins()\r\n");
            break;
        case 'N':
            //retract_all_pins();
        	printf("retract_all_pins()\r\n");
            break;
        case 'G':
            printf("\n\rSend the \"@\" symbol repeatedly to exit grease pins mode\r\n");
            //grease_pins();
            printf("grease_pins();");
            break;
        default:
        	printf("\r\n** Unrecognized command!!** \r\n");
            break;
    }
}


void menu_main(void) {
    printf("\r\n\n\r");
    printf("=========================================\n\r");
    printf("|  AOML auto launcher board version 3.0 |\n\r");
    printf("|  Firmware version 2024.mm.dd.hhmm     |\n\r");
    printf("=========================================\n\r");
    printf("|    Model #ALV3.0      S/N ");
    print_serial_number();
    printf("       |\n\r");
    printf("=========================================\n\r");
    printf("|               COMMANDS                |\n\r");
    if (eeprom.configured != '|') {
        printf("| ERROR, NO SERIAL NUMBER ASSIGNED  |\n\r");
    }
    printf("=========================================\n\r");
    printf("| Connect  cal Sim BT  0                |\n\r");
    if (launcher.tubeCount == '6') {
        printf("| Connect  XBT 1-6     1,2,3,4,5,6      |\n\r");
        printf("| Extend   Pin 1-6     U,V,W,X,Y,Z      |\n\r");
        printf("| Retract  Pin 1-6     A,B,C,D,E,F      |\n\r");
    } else if (launcher.tubeCount == '8') {

        printf("| Connect  XBT 1-8     1,2,3,4,5,6,7,8  |\n\r");
        printf("| Extend   Pin 1-8     U,V,W,X,Y,Z,S,T  |\n\r");
        printf("| Retract  Pin 1-8     A,B,C,D,E,F,H,I  |\n\r");
    } else {
    	printf("| ERROR, NO TUBE COUNT!!     	        |\n\r");
    }
    printf("| Unground   XBT       G                |\n\r");
    printf("| Calibrate on         K                |\n\r");
    printf("| Cal resistor         L                |\n\r");
    printf("| Reset Relays         R                |\n\r");
    printf("| Print serial number  s                |\n\r");
    printf("| This Menu            M                |\n\r");
    printf("=========================================\n\r");
    printf("\r\n");
}//end status_message


void menu_config(void) {
    printf("\n\r");
    printf("=========================================\n\r");
    printf("|  AOML auto launcher config menu       |\n\r");
    printf("=========================================\n\r");
    printf("|    Model #ALV3.0      S/N ");
    print_serial_number();
    printf("       |\n\r");
    printf("=========================================\n\r");
    printf("|               COMMANDS                |\n\r");
    printf("=========================================\n\r");
    printf("| Set tubes & S/N      1                |\n\r");
    printf("| This Menu            M                |\n\r");
    printf("| Extend all   pins    J                |\n\r");
    printf("| Retract all  pins    N                |\n\r");
    printf("| Grease pins  mode    G                |\n\r");
    printf("| Quit config menu     Q                |\n\r");
    printf("=========================================\n\r");
    printf("\r\n");
}//end status_message

void print_serial_number(void){
	//printf( "AL%c%s", launcher.type[0], launcher.serialNumber);
    if(eeprom.configured == '|'){
    	printf( "AL%c%s", launcher.type, launcher.serialNumber);
    } else {
        if (launcher.tubeCount == '6')
            printf("AL6XX");
        else
            printf("ALRXX");
    }
}

/* is_num()
 * Parameters: character c
 * Return: 1 if c is a digit, 0 if not */
uint8_t is_num(char c){
	uint8_t isNum = 0;
	isNum = ('0' == c || '1' == c || '2' == c || '3' == c || '4' == c || '5' == c || '6' == c || '7' == c || '8' == c || '9' == c);
	return isNum;
}

/* RELAY AND MOTOR CONTROL */
void multiplexer_set(mux_t select){
	HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, select); // SET = UART-tx / RESET = Din from GPS
}

/* Disconnect the XBT ABC pins from ground
 * 3 relays can be used as ground when SET, or ground when RESET based on jumpers JP6-7-8
 * If 1-2 pads are soldered, SET relays to unground, RESET relays to ground
 * Note: reset signal 3 is tied to other relays: CAL cont & CAL res */
void unground_xbt(void){
	if(relayLock == reFree){
		relayLock = reLocked;
		drive_relay(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, 10); // RESET relay k9, k10, k11, k12
		relayLock = reFree;
	}
}

void calibration_resistor(void){
	if(relayLock == reFree){
		relayLock = reLocked;
		drive_relay(RELAY_K12_CAL_RES_GPIO_Port, RELAY_K12_CAL_RES_Pin, 10); // SET relay k12
		relayLock = reFree;
	}
}

void calibrate_on(void){
	if(relayLock == reFree){
		relayLock = reLocked;
		drive_relay(RELAY_K11_CAL_CONT_GPIO_Port, RELAY_K11_CAL_CONT_Pin, 10); // SET relay k11
		relayLock = reFree;
	}
}

void reset_relay(void){
	if(relayLock == reFree){
		relayLock = reLocked;
		drive_relay(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, 10); // RESET relay k1, k2, k3, k4, SSR1, SSR2, SSR3, SSR4
		HAL_Delay(10);
		drive_relay(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, 10); // RESET relay k5, k6, k7, k8, SSR5, SSR6, SSR7, SSR8
		HAL_Delay(10);
		drive_relay(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, 10); // RESET relay k9, k10, k11, k12 - This grounds ABC
		relayLock = reFree;
	}
}


void connect_xbt_pin(uint8_t xbtNum){

	if(relayLock == reFree){
		relayLock = reLocked;

		switch (xbtNum){
		case 1:
			drive_relay(RELAY_K1_GPIO_Port, RELAY_K1_Pin, 10); // SET relay k1
			drive_relay(SSR_1_GPIO_Port, SSR_1_Pin, 1); // SET SSR1
			break;
		case 2:
			drive_relay(RELAY_K2_GPIO_Port, RELAY_K2_Pin, 10); // SET relay k2
			drive_relay(SSR_2_GPIO_Port, SSR_2_Pin, 1); // SET SSR2
			break;
		case 3:
			drive_relay(RELAY_K3_GPIO_Port, RELAY_K3_Pin, 10); // SET relay k3
			drive_relay(SSR_3_GPIO_Port, SSR_3_Pin, 1); // SET SSR3
			break;
		case 4:
			drive_relay(RELAY_K4_GPIO_Port, RELAY_K4_Pin, 10); // SET relay k4
			drive_relay(SSR_4_GPIO_Port, SSR_4_Pin, 1); // SET SSR4
			break;
		case 5:
			drive_relay(RELAY_K5_GPIO_Port, RELAY_K5_Pin, 10); // SET relay k5
			drive_relay(SSR_5_GPIO_Port, SSR_5_Pin, 1); // SET SSR5
			break;
		case 6:
			drive_relay(RELAY_K6_GPIO_Port, RELAY_K6_Pin, 10); // SET relay k6
			drive_relay(SSR_6_GPIO_Port, SSR_6_Pin, 1); // SET SSR6
			break;
		case 7:
			drive_relay(RELAY_K7_GPIO_Port, RELAY_K7_Pin, 10); // SET relay k7
			drive_relay(SSR_7_GPIO_Port, SSR_7_Pin, 1); // SET SSR7
			break;
		case 8:
			drive_relay(RELAY_K8_GPIO_Port, RELAY_K8_Pin, 10); // SET relay k8
			drive_relay(SSR_8_GPIO_Port, SSR_8_Pin, 1); // SET SSR8
			break;
		default:
			printf("\r\n* ERROR: xbt not found *\r\n");
			break;
		}
		relayLock = reFree;
	}
}
















/* EEPROM CONTROL */


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
	HAL_Delay(10);
	drive_relay(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, 10); // RESET relay k5, k6, k7, k8, SSR5, SSR6, SSR7, SSR8
	HAL_Delay(10);
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
