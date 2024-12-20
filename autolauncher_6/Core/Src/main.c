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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "retargetio.h"
#include "eeprom.h"
#include "stepper_drv8826.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
relayLock_t relayLock = reFree; // relay mutex to ensure one coil is driven at a time
rxStatus_t rxStatus = idle; // flag, indicate if a new char was sent over serial. Set to 'active' in the UART interrupt callback, and 'idle' after processing command
activeMenu_t activeMenu = mainMenu; // menu state to determine how the received command will be processed (config or main)
mux_t txMode = MUX_STM32; // flag to indicate current status of the Tx line, always start in stm32 mode
button_t btnStatus = btn_idle; // flag to detect push button was pressed
uint32_t btnTimeStart;
// initialize main structures
launcher_t launcher = {0, '\0', '\0', 0, LAUNCHER_MODE_DEFAULT, LAUNCHER_VERBOSE_DEFAULT, LAUNCHER_TIMEOUT_DEFAULT, 'N'};
eeprom_t eeprom = {127, 0, EEPROM_BUS_ADDRESS, 10};
motor_t motor = {MOTOR_RUNTIME_DEFAULT, MOTOR_SAMPLE_PERIOD_DEFAULT, MOTOR_PWM_FREQ_DEFAULT, MOTOR_WIRING_DEFAULT ,{0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,}, MOTOR_ADC_DISPLAY_DEFAULT}; // runtime, samplePeriod, configed, wiring, imax, use count
// buffer and flags for byte received by UART in interrupt mode
char rxBuffer[1] = "\0"; // UART1 receive buffer from computer, a char will be stored here with UART interrupt
char rxChar = '\0'; // UART1 receive character, == xBuffer[0]
// ADC conversion flags
uint8_t adcComplete = 0; // flag used to print ADC values
uint8_t adcDMAFull = 0; // this flag is set by the DMA IRQ when the adc buffer is full, located in DMA1_Channel1_IRQHandler(void) (stm32f1xx_it.c)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  uint32_t timeoutStart; // used to keep track of time after a command was received to then go back to stream GPS
  uint8_t timeoutFlag = 0, gpsFlag = 0;
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Retarget IO stream to UART
  RetargetInit(&huart1);
  // Initialize stepper motors
  motor_init();
  // initialize multiplexer in STM32 mode
  multiplexer_set(MUX_STM32);

  // Initialize relays
  relay_init();
  // enable receive interrupt
  uartrx_interrupt_init();
  // Initialize autolauncher parameters i.e. read eeprom
  parameter_init();
  // display main menu at startup
  menu_main_print();
  if(launcher.verbose == 1) printf("\r\n> ");
  // Start countdown to switch to GPS after timeout at startup
  timeoutStart = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // menu control loop
	  if(active == rxStatus){ // set to active with UART RX interrupt
		  // set mux to STM32 to display info in terminal
		  multiplexer_set(MUX_STM32);
		  HAL_Delay(1);
		  // reset uart flag
		  rxStatus = idle;
		  // add new line
		  if(gpsFlag == 1){
			  gpsFlag = 0;
			  printf("\r\n\r\n");
		  }
		  // process command based on active menu
		  if( mainMenu == activeMenu){
			  // go to main switch case menu
			  menu_main_process_input(rxChar);
		  } else if ( configMenu == activeMenu){
			  // go to configuration switch case menu
			  menu_config_process_input(rxChar);
		  }
		  // start keeping track of time after processing the rx character. At startup it resets the time before the while(1)
		  timeoutStart = HAL_GetTick();
	  }
	  // send command if button was pressed
	  debounce_button();
	  // Go back to GPS stream is launcher mode = 0
	  if(launcher.mode == 0){
		  // check if elapsed time > timeout
		  timeoutFlag = check_menu_timeout(timeoutStart, launcher.timeout);
		  if(timeoutFlag == 1){
			  printf("\r\n|========================================|\n\r");
			  printf("|   ** Switching back to GPS stream **   |\r\n");
			  printf("|========================================|\r\n\r\n");
			  multiplexer_set(MUX_GPS);
			  // set active menu back to main in order to accept control commands
			  activeMenu = mainMenu;
			  gpsFlag = 1;
		  }
	  }
	  HAL_Delay(1); // needed to debug
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


/********************************************** MENU FUNCTIONS **********************************************/

/* Print Main Menu options */
void menu_main_print(void) {
    printf("\r\n\n\r");
    printf("|========================================|\n\r");
    printf("|   AOML Autolauncher board version 3.0  |\n\r");
    printf("|        Firmware version 2024.12.20     |\n\r");
    printf("|========================================|\n\r");
    printf("|     Model #ALV3.0      S/N ");
    print_serial_number();
    printf("       |\n\r");
    printf("|========================================|\n\r");
    printf("|               COMMANDS                 |\r\n");
    printf("|========================================|\n\r");
    printf("| Connect  Cal Sim BT  	0                |\n\r");
    if (launcher.tubeCount == '6') {
        printf("| Connect  XBT 1-6     1,2,3,4,5,6       |\n\r");
        printf("| Extend   Pin 1-6     U,V,W,X,Y,Z       |\n\r");
        printf("| Retract  Pin 1-6     A,B,C,D,E,F       |\n\r");
    } else if (launcher.tubeCount == '8') {
        printf("| Connect  XBT 1-8      1,2,3,4,5,6,7,8  |\n\r");
        printf("| Extend   Pin 1-8      U,V,W,X,Y,Z,S,T  |\n\r");
        printf("| Retract  Pin 1-8      A,B,C,D,E,F,H,I  |\n\r");
    } else {
    	printf("| ERROR: NO TUBE COUNT SET!              |\n\r");
    }
    printf("| Unground XBT         	G                |\n\r");
    printf("| Calibrate ON         	K                |\n\r");
    printf("| Calibration resistor	L                |\n\r");
    printf("| Reset all relays    	R                |\n\r");
    printf("| Print serial number  	s                |\n\r");
    printf("| This menu            	M                |\n\r");
    printf("| Read voltage & temp   P                |\n\r");
    printf("| Set Tx to GPS         N                |\n\r");
    printf("| Set Tx to STM32       O                |\n\r");
    printf("|========================================|\n\r");
    printf("\r\n");

    if(launcher.mode == 0){
    	printf("> GPS will start streaming data after %d seconds of inactivity\r\n", launcher.timeout);
    }
}//end status_message


/* Process char received while in Main menu state */
void menu_main_process_input(char option){
	print_char(option);
	if(launcher.verbose == 1){
		printf("\r\n> Executing OPTION (%c) --> ", option);
	}

	switch (option){
		// Connect XBT pins
    case '0':
        //engage calibration resistor
    	if(launcher.verbose == 1){
    		printf("unground_xbt(), calibration_resistor(), calibrate_on()\r\n");
    	}
        unground_xbt();
        calibration_resistor();
        calibrate_on();
        break;
    case '1':
    	if(launcher.verbose == 1){
    		printf("connect_xbt_pin(1)\r\n");
    	}
        connect_xbt_pin(1);
        break;
    case '2':
    	if(launcher.verbose == 1){
    		printf("connect_xbt_pin(2)\r\n");
    	}
        connect_xbt_pin(2);
        break;
    case '3':
    	if(launcher.verbose == 1){
    		printf("connect_xbt_pin(3)\r\n");
    	}
        connect_xbt_pin(3);
        break;
    case '4':
    	if(launcher.verbose == 1){
    		printf("connect_xbt_pin(4)\r\n");
    	}
        connect_xbt_pin(4);
        break;
    case '5':
    	if(launcher.verbose == 1){
    		printf("connect_xbt_pin(5)\r\n");
    	}
        connect_xbt_pin(5);
        break;
    case '6':
    	if(launcher.verbose == 1){
    		printf("connect_xbt_pin(6)\r\n");
    	}
        connect_xbt_pin(6);
        break;
    case '7':
        if (launcher.tubeCount == '8'){
        	if(launcher.verbose == 1){
        		printf("connect_xbt_pin(7)\r\n");
        	}
            connect_xbt_pin(7);
        } else {
        	if(launcher.verbose == 1){
        		printf("\r\n* ERROR: tube 7 not available *\r\n");
        	}
        }
        break;
    case '8':
        if (launcher.tubeCount == '8'){
        	if(launcher.verbose == 1){
        		printf("connect_xbt_pin(8)\r\n");
        	}
        	connect_xbt_pin(8);
        } else {
        	if(launcher.verbose == 1){
        		printf("\r\n* ERROR: tube 8 not available *\r\n");
        	}
        }
        break;
        //EXTEND PINS
    case 'U':
    	if(launcher.verbose == 1){
    		printf("extend_pin(1)\r\n");
    	}
    	extend_pin(1);
        break;
    case 'V':
    	if(launcher.verbose == 1){
    		printf("extend_pin(2)\r\n");
    	}
        extend_pin(2);
        break;
    case 'W':
    	if(launcher.verbose == 1){
    		printf("extend_pin(3)\r\n");
    	}
        extend_pin(3);
        break;
    case 'X':
    	if(launcher.verbose == 1){
    		printf("extend_pin(4)\r\n");
    	}
        extend_pin(4);
        break;
    case 'Y':
    	if(launcher.verbose == 1){
    		printf("extend_pin(5)\r\n");
    	}
        extend_pin(5);
        break;
    case 'Z':
    	if(launcher.verbose == 1){
    		printf("extend_pin(6)\r\n");
    	}
        extend_pin(6);
        break;
    case 'S':
        if (launcher.tubeCount == '8'){
        	if(launcher.verbose == 1){
        		printf("extend_pin(7)\r\n");
        	}
        	extend_pin(7);
        } else {
        	if(launcher.verbose == 1){
        		printf("\r\n* ERROR: tube 7 not available *\r\n");
        	}
        }
        break;
    case 'T':
        if (launcher.tubeCount == '8'){
        	if(launcher.verbose == 1){
        		printf("extend_pin(8)\r\n");
        	}
        	extend_pin(8);
        } else {
        	if(launcher.verbose == 1){
        		printf("\r\n* ERROR: tube 8 not available *\r\n");
        	}
        }
        break;
        //RETRACT PINS
    case 'A':
    	if(launcher.verbose == 1){
    		printf("retract_pin(1)\r\n");
    	}
    	retract_pin(1);
        break;
    case 'B':
    	if(launcher.verbose == 1){
    		printf("retract_pin(2)\r\n");
    	}
        retract_pin(2);
        break;
    case 'C':
    	if(launcher.verbose == 1){
    		printf("retract_pin(3)\r\n");
    	}
        retract_pin(3);
        break;
    case 'D':
    	if(launcher.verbose == 1){
    		printf("retract_pin(4)\r\n");
    	}
        retract_pin(4);
        break;
    case 'E':
    	if(launcher.verbose == 1){
    		printf("retract_pin(5)\r\n");
    	}
        retract_pin(5);
        break;
    case 'F':
    	if(launcher.verbose == 1){
    		printf("retract_pin(6)\r\n");
    	}
        retract_pin(6);
        break;
    case 'H':
        if (launcher.tubeCount == '8'){
        	if(launcher.verbose == 1){
        		printf("retract_pin(7)\r\n");
        	}
        	retract_pin(7);
        } else {
        	if(launcher.verbose == 1){
        		printf("* ERROR: tube 7 not available *\r\n");
        	}
        }
        break;
    case 'I':
        if (launcher.tubeCount == '8'){
        	if(launcher.verbose == 1){
        		printf("retract_pin(8)\r\n");
        	}
        	retract_pin(8);
        } else {
        	if(launcher.verbose == 1){
        		printf("* ERROR: tube 8 not available *\r\n");
        	}
        }
        break;
    case 'K':
    	if(launcher.verbose == 1){
    		printf("calibrate_on()\r\n");
    	}
        calibrate_on();
        break;
    case 'R':
    	if(launcher.verbose == 1){
    		printf("reset_relay(), ground XBT\r\n");
    	}
        reset_relay();
        break;
    case 'L':
    	if(launcher.verbose == 1){
    		printf("calibration_resistor()\r\n");
    	}
        calibration_resistor();
        break;
    case 'G':
    	if(launcher.verbose == 1){
    		printf("unground_xbt()\r\n");
    	}
        unground_xbt();
        break;
    case 'M':
        menu_main_print();
        break;
    case '~':
    	if(launcher.verbose == 1){
    		printf("menu_config_print()\r\n");
    	}
        menu_config_print();
        activeMenu = configMenu; // set configuration menu flag
        break;
    case 's':
    	if(launcher.verbose == 1){
    		printf("print_serial_number()\r\n");
    	}
    	printf("\r\n> AL Serial Number: ");
        print_serial_number();
        printf("\r\n");
        break;
    case 'P':
    	if(launcher.verbose == 1){
    		printf("menu_print_volt_temp()\r\n");
    	}
    	// read input voltage and internal temp on autolauncher
    	menu_print_volt_temp();
    	break;
    case 'N':
    	if(launcher.verbose == 1){
        	printf("multiplexer_set(MUX_GPS)\r\n");
        	printf("\r\n** Data TX from local GPS --> press 'O' to set Tx to STM32 **\r\n\r\n");
    	}
    	multiplexer_set(MUX_GPS);
    	break;
    case 'O':
    	multiplexer_set(MUX_STM32);
    	if(launcher.verbose == 1){
    		printf("\r\n> Executing OPTION (%c) --> ", option);
        	printf("\r\nmultiplexer_set(MUX_STM32)\r\n");
        	printf("\r\n** Data TX from STM32 **\r\n");
    	}
    	menu_main_print();
    	break;
    default:
        printf("\r\n** Unrecognized command!!** \r\n");
        break;
	}
	if(launcher.verbose == 1){
		printf("\r\n> ");
	}
}

/* Prints Input voltage and STM32 chip internal temperature */
void menu_print_volt_temp(void){
	adcScan_t adcReading = adc_get_values();
	printf("\r\n> Voltage [AD# %i]: %i.%i V | STM32 Temperature [AD# %i]: %i.%i C\r\n",
				 (int)adcReading.voltage.rawValue, (int)adcReading.voltage.realValue, get_decimal(adcReading.voltage.realValue, 1),
				 (int)adcReading.temperature.rawValue, (int)adcReading.temperature.realValue, get_decimal(adcReading.temperature.realValue, 1));
}


/* Print secret configuration menu options
 * Access this menu with '~' */
void menu_config_print(void) {
    printf("\n\r");
    printf("=========================================\n\r");
    printf("| AOML Autolauncher configuration menu	|\n\r");
    printf("=========================================\n\r");
    printf("|    Model #ALV3.0      S/N ");
    print_serial_number();
    printf("       |\n\r");
    printf("|=======================================|\n\r");
    printf("|               COMMANDS                |\n\r");
    printf("|=======================================|\n\r");
    printf("| <Z> Help                              |\n\r");
    printf("| <M> This Menu                         |\n\r");
    printf("| <Q> QUIT to main menu                 |\n\r");
    printf("| <A> Set AL tubes, type & S/N          |\n\r");
    printf("| <T> Set launcher mode                 |\n\r");
    printf("| <Y> Set verbose mode                  |\n\r");
    printf("| <U> Read launcher parameters          |\n\r");
    printf("|---------------------------------------|\n\r");
    printf("| <S> Extend all pins                   |\n\r");
    printf("| <D> Retract all pins                  |\n\r");
    printf("| <F> Grease pins mode                  |\n\r");
    printf("|---------------------------------------|\n\r");
    printf("| <R> Read motor configuration          |\n\r");
    printf("| <H> Read motor statistics             |\n\r");
    printf("| <J> Set motor runtime                 |\n\r");
    printf("| <K> Set motor ADC sampling period     |\n\r");
    printf("| <I> Set motor ADC display on/off      |\n\r");
    printf("| <L> Set motor wiring mode             |\n\r");
    printf("| <W> Set motor PWM frequency           |\n\r");
    printf("| <E> Reset motor statistics            |\n\r");
    printf("| <G> Clear memory range (caution!)     |\n\r");
    printf("|=======================================|\n\r");
    printf("\r\n");
}//end status_message


/* Process char received while in configuration menu */
void menu_config_process_input(char option){
	print_char(option);
	if(launcher.verbose == 1){
		printf("\r\n> Executing OPTION (%c) --> ", option);
	}

    switch (option) {
        case 'Q':
            printf("\n\r> Leaving Auto launcher configuration menu\n\r");
            activeMenu = mainMenu; // set active menu flag to main menu
            menu_main_print();
            break;
        case 'M':
        	if(launcher.verbose == 1){
        		printf("menu_config_print()\r\n");
        	}
            menu_config_print();
            break;
        case 'A':
        	if(launcher.verbose == 1){
        		printf("menu_config_tubes_type_serial()\r\n");
        	}
        	// get the autolauncher tube count
        	menu_config_tubes_type_serial();
            // print config menu again
            menu_config_print();
            break;
        case 'S':
        	if(launcher.verbose == 1){
        		printf("extend_all_pins()\r\n");
        	}
        	if(launcher.tubeCount == '6')
        		extend_all_pins(6);
        	if(launcher.tubeCount == '8')
        		extend_all_pins(8);
            break;
        case 'D':
        	if(launcher.verbose == 1){
        		printf("retract_all_pins()\r\n");
        	}
        	if(launcher.tubeCount == '6')
        		retract_all_pins(6);
        	if(launcher.tubeCount == '8')
        		retract_all_pins(8);
            break;
        case 'F':
            printf("\n\rSend the @ symbol repeatedly to exit grease pins mode\r\n");
            if(launcher.verbose == 1){
            	printf("grease_pins()\r\n");
            }
            grease_pins(MOTOR_GREASE_CYCLES);
            break;
        case 'G':
        	// clear range of eeprom memory blocks
        	if(launcher.verbose == 1){
        		printf("eeprom_clear_memory_range()\r\n");
        	}
        	eeprom_clear_memory_range();
        	break;
        case 'H':
        	// read motor statistics (imax, count)
        	if(launcher.verbose == 1){
        		printf("motor_read_stats()");
        	}
        	motor_read_stats();
        	break;
        case 'J':
        	// set motor runtime
        	if(launcher.verbose == 1){
        		printf("motor_set_runtime()");
        	}
        	motor_set_runtime();
    		break;
        case 'K':
        	// set sampling period for ADC when motor is running
        	if(launcher.verbose == 1){
        		printf("motor_set_sampling_period()");
        	}
        	motor_set_sampling_period();
        	break;
        case 'L':
        	// set motor wiring type: 0 or 1. Based on value it'll change the direction the motor runs in CW/CCW
        	if(launcher.verbose == 1){
        		printf("motor_set_wiring()");
        	}
        	motor_set_wiring();
        	break;
        case 'W':
        	// set the motor step PWM freq for the DRV8826
        	if(launcher.verbose == 1){
        		printf("motor_set_pwm_freq()");
        	}
        	motor_set_pwm_freq();
        	break;
        case 'E':
        	// reset all motor statistics to 0
        	if(launcher.verbose == 1){
        		printf("motor_reset_stats()");
        	}
        	motor_reset_stats();
        	break;
        case 'R':
        	// read all motor configuration parameters
        	if(launcher.verbose == 1){
        		printf("motor_read_parameters()");
        	}
        	motor_read_parameters();
        	break;
        case 'Z':
        	// print help for config menu
        	if(launcher.verbose == 1){
        		printf("menu_help_print()");
        	}
        	menu_help_print();
        	break;
        case 'T':
        	// set launcher mode to stream GPS after a timeout or not
        	if(launcher.verbose == 1){
        		printf("menu_set_launcher_mode()");
        	}
        	menu_set_launcher_mode();
        	break;
        case 'Y':
        	// set verbose mode or not to print more information after each command
        	if(launcher.verbose == 1){
        		printf("menu_set_verbose()");
        	}
        	menu_set_verbose();
        	break;
        case 'U':
        	// read launcher parameters
        	if(launcher.verbose == 1){
        		printf("launcher_read_parameters()");
        	}
        	launcher_read_parameters();
        	break;
        case 'I':
        	// set ADC display enable/disable
        	if(launcher.verbose == 1){
        		printf("motor_set_adc_display()");
        	}
        	motor_set_adc_display();
        	break;
        default:
        	printf("\r\n** Unrecognized command!!** \r\n");
            break;
    }
    printf("\r\n> ");
}


/* Configure autloauncher parameters for display
 * Tube count, serial number and type */
void menu_config_tubes_type_serial(void){
	printf("> Current tube count: %c\r\n", launcher.tubeCount);
	char tubes[1];
	char tubePrompt[] = "\r\n> Enter AL tube count [6] or [8]: ";
	char tubeError[] = "\r\n** ERROR: Enter 6 or 8 ! **\r\n";
	char tubeCheck[] = {'6','8'};
	get_user_input(tubePrompt, tubeError, 1, tubeCheck, tubes);

    launcher.tubeCount = tubes[0];
    // get the autolauncher type, R regular or X extended, only for 8 tube AL
    if(launcher.tubeCount == '8'){
    	printf("> Current type: %c\r\n", launcher.type);
    	char type[1];
    	char typePrompt[] = "> Enter launcher type, [X] extended or [R] regular: ";
    	char typeError[] = "\r\n** ERROR: Enter X or R ! **\r\n";
    	char typeCheck[] = {'R','X'};
    	get_user_input(typePrompt, typeError, 1, typeCheck, type);
    	launcher.type = type[0];

    } else {
    	launcher.type = '0'; // if not 8 tubes, reset type to unknown
    }
    printf("> Current serial number: %i\r\n", launcher.serialNumber);
	char serial[2];
	char serialPrompt[] = "> Enter a two-digit Autolauncher serial number [00-99]: ";
	char serialError[] = "\r\n** Enter only numbers! **\r\n";
	char serialCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	get_user_input(serialPrompt, serialError, 2, serialCheck, serial);
	launcher.serialNumber = (uint8_t) ( (serial[0] - '0') * 10 + (serial[1] - '0') ); // convert to number, subtract '0' (48 dec)
    launcher.configured = 'Y';
    printf("\r\n> Tubes: %c | Type: %c | Serial: %i\r\n", launcher.tubeCount, launcher.type, launcher.serialNumber);

    // store parameters in eeprom
    eeprom_write_nbytes(AL_TUBECOUNT1B, sizeof(launcher.tubeCount), &launcher.tubeCount);
    eeprom_write_nbytes(AL_TYPE1B, sizeof(launcher.type), &launcher.type);
    eeprom_write_nbytes(AL_SN1B, sizeof(launcher.serialNumber), &launcher.serialNumber);
    eeprom_write_nbytes(AL_CONFIGED1B, sizeof(launcher.configured), &launcher.configured);
    printf("> Settings saved!");
    // test eeprom memory
    eeprom_read_nbytes(AL_TUBECOUNT1B, sizeof(launcher.tubeCount), &launcher.tubeCount);
    eeprom_read_nbytes(AL_TYPE1B, sizeof(launcher.type), &launcher.type);
    eeprom_read_nbytes(AL_SN1B, sizeof(launcher.serialNumber), &launcher.serialNumber);
    eeprom_read_nbytes(AL_CONFIGED1B, sizeof(launcher.configured), &launcher.configured);
    // print stored values
    printf("\r\n> Stored autolauncher configuration: Tubes: %c | Type: %c | Serial: %i | Configured: %c\r\n",
    		launcher.tubeCount, launcher.type, launcher.serialNumber, launcher.configured);
}


/* Clear a range of the eeprom memory
 * It will write a 0 to the selected range of memory addresses [0-127]
 * To clear 1 byte memory start = end */
void eeprom_clear_memory_range(void){
	uint8_t memStart, memEnd;
	uint8_t mFlag; // valid memory value flag
	// print eeprom memory map
	eeprom_print_memory_map();
	// get the memory range to clear - start
	char mem[3]; // buffer to store digits
	char mStartPrompt[100];
	sprintf(mStartPrompt, "\r\n> Enter (3-digit) START memory address [%03i-%03i]: ", eeprom.MEMORY_MIN, eeprom.MEMORY_MAX);
	char mEndPrompt[100];
	sprintf(mEndPrompt, "\r\n> Enter (3-digit) END memory address [%03i-%03i]: ", eeprom.MEMORY_MIN, eeprom.MEMORY_MAX);
	char memError[] = "\r\n** ERROR: enter valid numbers **\r\n";
	char memCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	// get start address
	do{
		mFlag = 0;
		get_user_input(mStartPrompt, memError, 3, memCheck, mem);
		memStart = (uint8_t) ( (mem[0] - '0') * 100 + (mem[1] - '0') * 10 + (mem[2] - '0') ); // convert to number, subtract '0' (48 dec)
		if((memStart >= eeprom.MEMORY_MIN) && (memStart <= eeprom.MEMORY_MAX)){
			mFlag = 1;
		} else {
			printf("\r\n** Memory out of range! **\r\n");
		}
	} while ( !mFlag );
	// get end address
	do{
		mFlag = 0;
		get_user_input(mEndPrompt, memError, 3, memCheck, mem);
		memEnd = (uint8_t)( (mem[0] - '0') * 100 + (mem[1] - '0') * 10 + (mem[2] - '0') ); // convert to number, subtract '0' (48 dec)
		if((memEnd >= eeprom.MEMORY_MIN) && (memEnd <= eeprom.MEMORY_MAX) && memStart <= memEnd){
			mFlag = 1;
		} else {
			printf("\r\n** Memory out of range or start>end **\r\n");
		}
	} while ( !mFlag );
	printf("> %i block/s cleared!\r\n", eeprom_clear(memStart, memEnd));
	// update variables with new stored values
	// read launcher config
	parameter_init();
}


/* Read motor use count and Imax stored in eeprom memory */
void motor_read_stats(void){
	// use count
	eeprom_read_nbytes(M_1COUNT2B, sizeof(motor.count[0]), &motor.count[0]);
	eeprom_read_nbytes(M_2COUNT2B, sizeof(motor.count[1]), &motor.count[1]);
	eeprom_read_nbytes(M_3COUNT2B, sizeof(motor.count[2]), &motor.count[2]);
	eeprom_read_nbytes(M_4COUNT2B, sizeof(motor.count[3]), &motor.count[3]);
	eeprom_read_nbytes(M_5COUNT2B, sizeof(motor.count[4]), &motor.count[4]);
	eeprom_read_nbytes(M_6COUNT2B, sizeof(motor.count[5]), &motor.count[5]);
	eeprom_read_nbytes(M_7COUNT2B, sizeof(motor.count[6]), &motor.count[6]);
	eeprom_read_nbytes(M_8COUNT2B, sizeof(motor.count[7]), &motor.count[7]);
	// max current logged
	eeprom_read_nbytes(M_1MXAMP2B, sizeof(motor.imax[0]), &motor.imax[0]);
	eeprom_read_nbytes(M_2MXAMP2B, sizeof(motor.imax[1]), &motor.imax[1]);
	eeprom_read_nbytes(M_3MXAMP2B, sizeof(motor.imax[2]), &motor.imax[2]);
	eeprom_read_nbytes(M_4MXAMP2B, sizeof(motor.imax[3]), &motor.imax[3]);
	eeprom_read_nbytes(M_5MXAMP2B, sizeof(motor.imax[4]), &motor.imax[4]);
	eeprom_read_nbytes(M_6MXAMP2B, sizeof(motor.imax[5]), &motor.imax[5]);
	eeprom_read_nbytes(M_7MXAMP2B, sizeof(motor.imax[6]), &motor.imax[6]);
	eeprom_read_nbytes(M_8MXAMP2B, sizeof(motor.imax[7]), &motor.imax[7]);
	// print stats
	printf("\r\n\r\n|        STEPPER MOTOR STATS          |\r\n");
	printf("|=====================================|\r\n");
	for(uint8_t i = 0; i < 8; i++){
		printf("| [M%i] IMAX= %03i mA, USE_COUNT= %05i |\r\n", i+1 ,(int)motor.imax[i], (int)motor.count[i]);
	}
	printf("|=====================================|\r\n");
}


/* Set the runtime for the stepper motors */
void motor_set_runtime(void){
	printf("\r\n> Current runtime: %i ms [default=%i]\r\n", motor.runTime, MOTOR_RUNTIME_DEFAULT);
	char runtime[5];
	char runtimePrompt[100];
	sprintf(runtimePrompt, "> [exit=%05i] Enter motor runtime (5-digits) in milliseconds [%05i-%05i]: ", 0, MOTOR_RUNTIME_MIN, MOTOR_RUNTIME_MAX);
	char runtimeError[] = "\r\n** Enter only numbers! **\r\n";
	char runtimeCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	uint8_t rtFlag, exitFlag = 0;
	// loop until a good value is set or 0 to exit
	do{
		rtFlag = 0;
		get_user_input(runtimePrompt, runtimeError, 5, runtimeCheck, runtime);
		uint32_t rt = ( (runtime[0] - '0') * 10000 + (runtime[1] - '0') * 1000 + (runtime[2] - '0') * 100 + (runtime[3] - '0') * 10 + (runtime[4] - '0') );
		if(rt == 0){
			printf("\r\n** Exit **\r\n");
			exitFlag = 1;
			break;
		}
		// check values are within range
		if((rt >= MOTOR_RUNTIME_MIN) && (rt <= MOTOR_RUNTIME_MAX)){
			motor.runTime = (uint16_t) rt;
			rtFlag = 1;
		} else {
			printf("\r\n** Value out of range! **\r\n");
		}
	} while( !rtFlag);
	// store variables if it was not an exit
	if(exitFlag == 0){
		// Print all inputs
		printf("> Motor Runtime: %i ms\r\n", (int)motor.runTime);
		// store in eeprom
		eeprom_write_nbytes(M_RUNTIME2B, sizeof(motor.runTime), &motor.runTime);
		// test memory
		eeprom_read_nbytes(M_RUNTIME2B, sizeof(motor.runTime), &motor.runTime);
		printf("> Setting saved! Runtime: %i\r\n\r\n", (int)motor.runTime);
	}
}


/* Set the ADC sampling period while motor is running
 * This time should be shorter than motor runtime
 * The ADC will take a sample of Vin, motor I and chip temperature every x ms */
void motor_set_sampling_period(void){
	printf("\r\n> Current ADC sampling period: %i ms [default=%i]\r\n", motor.samplePeriod, MOTOR_SAMPLE_PERIOD_DEFAULT);
	char sPeriod[4];
	char sPeriodPrompt[100];
	sprintf(sPeriodPrompt, "> [exit=%04i] Enter ADC sampling time (4-digits) in milliseconds [%04i-%04i]: ", 0, MOTOR_SAMPLE_PERIOD_MIN, MOTOR_SAMPLE_PERIOD_MAX);
	char sPeriodError[] = "\r\n** Enter only numbers! **\r\n";
	char sPeriodCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	uint8_t spFlag, exitFlag = 0;
	do{
		spFlag = 0;
		get_user_input(sPeriodPrompt, sPeriodError, 4, sPeriodCheck, sPeriod);
		uint32_t sp = ( (sPeriod[0] - '0') * 1000 + (sPeriod[1] - '0') * 100 + (sPeriod[2] - '0') * 10 + (sPeriod[3] - '0') );
		// check to exit
		if(sp == 0){
			printf("\r\n** Exit **\r\n");
			exitFlag = 1;
			break;
		}
		// check if values are valid
		if(( sp >= MOTOR_SAMPLE_PERIOD_MIN) && (sp <= MOTOR_SAMPLE_PERIOD_MAX) && sp < motor.runTime){
			motor.samplePeriod = (uint16_t) sp;
			spFlag = 1;
		} else {
			printf("\r\n** Value out of range or greater than runtime! **\r\n");
		}
	} while( !spFlag);
	// update values if it was not an exit
	if(!exitFlag){
		// Print all inputs
		printf("> Motor ADC Sample Period: %i ms\r\n", (int)motor.samplePeriod);
		// store in eeprom
		eeprom_write_nbytes(M_SAMPLEPERIOD2B, sizeof(motor.samplePeriod), &motor.samplePeriod);
		// test memory
		eeprom_read_nbytes(M_SAMPLEPERIOD2B, sizeof(motor.samplePeriod), &motor.samplePeriod);
		printf("> Setting saved! Sample Period: %i\r\n\r\n", (int)motor.samplePeriod);
		// verify value read is ok
		if( !(motor.samplePeriod >= MOTOR_SAMPLE_PERIOD_MIN && motor.samplePeriod <= MOTOR_SAMPLE_PERIOD_MAX)){
			motor.samplePeriod = MOTOR_SAMPLE_PERIOD_DEFAULT;
		}
		// update timer PWM registers
		update_timer(&htim4, motor.samplePeriod, 4, 0.5);
	}
}


/* Set motor PWM frequency Hz to drive DRV8826 STEP pin */
void motor_set_pwm_freq(void){
	printf("\r\n> Current PWM frequency: %i Hz [default=%i]\r\n", motor.pwmFreq, MOTOR_PWM_FREQ_DEFAULT);
	char pwmf[4];
	char pwmfPrompt[100];
	sprintf(pwmfPrompt, "> [exit=%04i] Enter PWM frequency (4-digits) in Hz [%04i-%04i]: ", 0 ,MOTOR_PWM_FREQ_MIN, MOTOR_PWM_FREQ_MAX);
	char pwmfError[] = "\r\n** Enter only numbers! **\r\n";
	char pwmfCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	uint8_t pwmfFlag, exitFlag = 0;
	do{
		pwmfFlag = 0;
		get_user_input(pwmfPrompt, pwmfError, 4, pwmfCheck, pwmf);
		uint32_t f = ( (pwmf[0] - '0') * 1000 + (pwmf[1] - '0') * 100 + (pwmf[2] - '0') * 10 + (pwmf[3] - '0') );
		// exit if user entered 0
		if(f == 0){
			printf("\r\n** Exit **\r\n");
			exitFlag = 1;
			break;
		}
		if(( f >= MOTOR_PWM_FREQ_MIN) && (f <= MOTOR_PWM_FREQ_MAX)){
			motor.pwmFreq = (uint16_t) f;
			pwmfFlag = 1;
		} else {
			printf("\r\n** Value out of range! **\r\n");
		}
	} while(!pwmfFlag);
	// update values if it was not an exit
	if(!exitFlag){
		// Print all inputs
		printf("> Motor PWM frequency: %i Hz\r\n", (int)motor.pwmFreq);
		// store in eeprom
		eeprom_write_nbytes(M_PWM_FREQ2B, sizeof(motor.pwmFreq), &motor.pwmFreq);
		// test memory
		eeprom_read_nbytes(M_PWM_FREQ2B, sizeof(motor.pwmFreq), &motor.pwmFreq);
		printf("> Setting saved! PWM Freq: %i\r\n\r\n", (int)motor.pwmFreq);
		// verify value read is ok
		if( !(motor.pwmFreq >= MOTOR_PWM_FREQ_MIN && motor.pwmFreq <= MOTOR_PWM_FREQ_MAX)){
			motor.pwmFreq = MOTOR_PWM_FREQ_DEFAULT;
		}
		// get period from frequency
		uint32_t pwmPeriod = 1E6/motor.pwmFreq; // PWM period in us -> 1E6/200 Hz = 5000 us
		// update timer PWM registers
		update_timer(&htim3, pwmPeriod, 3, 0.5);
	}
}


/* update timer Pulse (CCRx) and Period (ARR) registers */
void update_timer(TIM_HandleTypeDef * htim, uint32_t period, uint8_t channel , float dutyCycle){
	// update timer registers with period and duty cycle See TIM_TypeDef definition
	// Example: Prescaler is 8, so  TIMER CLK = 8 MHz/8-> PWM f = 1 MHz  -> 1 cycle/1 us -> 1 rising edge / 1 us
	// ARR Auto Reload Register, counter Period: 5000-1 (5 ms)
	// CCR3 Capture Compare Register, channel 3 Pulse: 2500-1 (PWM mode 1 -> 0-2500 off, 2500-4999 on) 50% Duty cycle
	if( channel >= 1 && channel <= 4 && dutyCycle < 1 && dutyCycle > 0){
		uint32_t pulse = (uint32_t) ((float)period * dutyCycle);
		htim->Instance->ARR = (uint32_t) (period - 1);

		switch (channel){
		case 1:
			htim->Instance->CCR1 = pulse - 1;
			break;
		case 2:
			htim->Instance->CCR2 = pulse - 1;
			break;
		case 3:
			htim->Instance->CCR3 = pulse - 1;
			break;
		case 4:
			htim->Instance->CCR4 = pulse - 1;
			break;
		default:
			break;
		}
	} else {
		printf("\r\n** Timer registers could not be updated! **\r\n");
	}
}


/* Set the motor wiring mode according to the color sequence crimped
 * This will change the behavior of the retract/extend functions
 * cable sequence: blue, gren, black, red will make motor extend pin in CW direction with wiring mode = 0 */
void motor_set_wiring(void){
	printf("\r\n> Current motor wiring: %i [default=%i]\r\n", motor.wiring, MOTOR_WIRING_DEFAULT);
	char mw[1];
	char mwPrompt[] = "> Enter motor wiring code (1-digit number) [0-1]: ";
	char mwError[] = "\r\n** Enter only numbers! **\r\n";
	char mwCheck[] = {'0','1'};
	uint8_t mwFlag;
	do{
		mwFlag = 0;
		get_user_input(mwPrompt, mwError, 1, mwCheck, mw);
		uint32_t w = (mw[0] - '0');
		if( w == 1 || w == 0){
			motor.wiring = (uint8_t) w;
			mwFlag = 1;
		} else {
			printf("\r\n** Value out of range! **\r\n");
		}
	} while(!mwFlag);
	// Print all inputs
	printf("> Motor wiring code: %i\r\n", (int)motor.wiring);
	// store in eeprom
	eeprom_write_nbytes(M_WIRING1B, sizeof(motor.wiring), &motor.wiring);
	// test memory
	eeprom_read_nbytes(M_WIRING1B, sizeof(motor.wiring), &motor.wiring);
	printf("> Setting saved! Motor wiring: %i\r\n\r\n", (int)motor.wiring);
}

/* Set the ADC values visibility while a motor is running */
void motor_set_adc_display(void){
	printf("\r\n> Current motor ADC display: %i [default=%i]\r\n", motor.adcDisplay, MOTOR_ADC_DISPLAY_DEFAULT);
	char adcdisp[1];
	char adcdispPrompt[] = "> Enter motor ADC display [0] disable or [1] enable: ";
	char adcdispError[] = "\r\n** Enter only numbers! **\r\n";
	char adcdispCheck[] = {'0','1'};
	uint8_t aFlag;
	do{
		aFlag = 0;
		get_user_input(adcdispPrompt, adcdispError, 1, adcdispCheck, adcdisp);
		uint32_t a = (adcdisp[0] - '0');
		if( a == 1 || a == 0){
			motor.adcDisplay = (uint8_t) a;
			aFlag = 1;
		} else {
			printf("\r\n** Value out of range! **\r\n");
		}
	} while(!aFlag);
	// Print all inputs
	printf("> Motor ADC display: %i\r\n", (int)motor.adcDisplay);
	// store in eeprom
	eeprom_write_nbytes(M_ADCDISPLAY1B, sizeof(motor.adcDisplay), &motor.adcDisplay);
	// test memory
	eeprom_read_nbytes(M_ADCDISPLAY1B, sizeof(motor.adcDisplay), &motor.adcDisplay);
	printf("> Setting saved! ADC display: %i\r\n\r\n", (int)motor.adcDisplay);
}


/* Clear the memory locations corresponding to motor count and Imax for ALL motors */
void motor_reset_stats(void){
	uint8_t motorNum;
	char opt[1];
	char optPrompt[] = "\r\n> [exit=0] Enter motor number or [9]-all [1-9]: ";
	char optError[] = "\r\n** Enter only numbers! **\r\n";
	char optCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	uint8_t optFlag, exitFlag = 0;
	do{
		optFlag = 0;
		get_user_input(optPrompt, optError, 1, optCheck, opt);
		uint32_t op = (opt[0] - '0');
		// check to exit
		if(op == 0){
			printf("\r\n** Exit **\r\n");
			exitFlag = 1;
			break;
		}
		// check if values are valid
		if( op >= 0 && op <= 9 ){
			motorNum = op;
			optFlag = 1;
		} else {
			printf("\r\n** Value out of range or greater than runtime! **\r\n");
		}
	} while(!optFlag);
	// update values if it was not an exit
	if(!exitFlag){
		// clear selected stat
		uint8_t slots = eeprom_reset_stats(motorNum);
		printf("\r\n> %i bytes cleared\r\n", slots);
	}
	// print all stats
	motor_read_stats();
}

/* clear motor statistics for a particular motor or all (9) */
uint8_t eeprom_reset_stats(uint8_t num){
	uint8_t slots = 0;
	if(num >= 0 && num <= 9){
		switch(num){
		case 0:
			// exit do nothing
			break;
		case 1:
			// reset motor 1 stats
			slots = eeprom_clear(M_1COUNT2B, M_1COUNT2B+1);
			slots = eeprom_clear(M_1MXAMP2B, M_1MXAMP2B+1) + slots;
			break;
		case 2:
			slots = eeprom_clear(M_2COUNT2B, M_2COUNT2B+1);
			slots = eeprom_clear(M_2MXAMP2B, M_2MXAMP2B+1) + slots;
			break;
		case 3:
			slots = eeprom_clear(M_3COUNT2B, M_3COUNT2B+1);
			slots = eeprom_clear(M_3MXAMP2B, M_3MXAMP2B+1) + slots;
			break;
		case 4:
			slots = eeprom_clear(M_4COUNT2B, M_4COUNT2B+1);
			slots = eeprom_clear(M_4MXAMP2B, M_4MXAMP2B+1) + slots;
			break;
		case 5:
			slots = eeprom_clear(M_5COUNT2B, M_5COUNT2B+1);
			slots = eeprom_clear(M_5MXAMP2B, M_5MXAMP2B+1) + slots;
			break;
		case 6:
			slots = eeprom_clear(M_6COUNT2B, M_6COUNT2B+1);
			slots = eeprom_clear(M_6COUNT2B, M_6COUNT2B+1) + slots;
			break;
		case 7:
			slots = eeprom_clear(M_7COUNT2B, M_7COUNT2B+1);
			slots = eeprom_clear(M_7MXAMP2B, M_7MXAMP2B+1) + slots;
		case 8:
			slots = eeprom_clear(M_8COUNT2B, M_8COUNT2B+1);
			slots = eeprom_clear(M_8MXAMP2B, M_8MXAMP2B+1) + slots;
		case 9:
			// reset all
			slots = eeprom_clear(M_1COUNT2B, M_8MXAMP2B+1);
			break;
		default:
			break;
		}
	}
	return slots;
}


/* Read all motor configuration parameters
 * And display if the program is using values from memory or defaults */
void motor_read_parameters(void){
	// read motor parameters from eeprom
	motor_t tempMotor;
	eeprom_read_nbytes(M_RUNTIME2B, sizeof(tempMotor.runTime), &tempMotor.runTime);
	eeprom_read_nbytes(M_SAMPLEPERIOD2B, sizeof(tempMotor.samplePeriod), &tempMotor.samplePeriod);
	eeprom_read_nbytes(M_PWM_FREQ2B, sizeof(tempMotor.pwmFreq), &tempMotor.pwmFreq);
	eeprom_read_nbytes(M_WIRING1B, sizeof(tempMotor.wiring), &tempMotor.wiring);
	eeprom_read_nbytes(M_ADCDISPLAY1B, sizeof(tempMotor.adcDisplay), &tempMotor.adcDisplay);
	// check values are within range or use defaults {flag = 'N'}
	char rtFlag = 'D', spFlag = 'D', pfFlag = 'D', wFlag = 'D', aFlag = 'D'; // using default values?
	if(tempMotor.runTime >= MOTOR_RUNTIME_MIN && tempMotor.runTime <= MOTOR_RUNTIME_MAX){
		motor.runTime = tempMotor.runTime;
		rtFlag = 'M';
	}
	if(tempMotor.samplePeriod >= MOTOR_SAMPLE_PERIOD_MIN && tempMotor.samplePeriod <= MOTOR_SAMPLE_PERIOD_MAX && tempMotor.samplePeriod <= motor.runTime){
		motor.samplePeriod = tempMotor.samplePeriod;
		spFlag = 'M';
	}
	if(tempMotor.pwmFreq >= MOTOR_PWM_FREQ_MIN && tempMotor.pwmFreq <= MOTOR_PWM_FREQ_MAX){
		motor.pwmFreq = tempMotor.pwmFreq;
		pfFlag = 'M';
	}
	if(tempMotor.wiring == 0 || tempMotor.wiring == 1){
		motor.wiring = tempMotor.wiring;
		wFlag = 'M';
	}
	if(tempMotor.adcDisplay == 0 || tempMotor.adcDisplay == 1){
		motor.adcDisplay = tempMotor.adcDisplay;
		aFlag = 'M';
	}
	// update timer registers
	uint32_t pwmPeriod = 1E6/motor.pwmFreq; // PWM period in us -> 1E6/200 Hz = 5000 us
	update_timer(&htim3, pwmPeriod, 3, 0.5); // timer 3 motor PWM steps
	update_timer(&htim4, motor.samplePeriod, 4, 0.5); // tiemr 4 ADC sampling period
	// print all parameters
	printf("\r\n> Non-Volatile Memory {M} OR Default {D}\r\n"
			" <MOTOR> Runtime: %i ms {%c} | PWM Frequency: %i Hz {%c} | Sample Period: %i ms {%c} | Wiring: %i {%c} | ADC Display: %i {%c}\r\n",
			 (int)motor.runTime, rtFlag, (int)motor.pwmFreq, spFlag, (int)motor.samplePeriod, pfFlag, (int)motor.wiring, wFlag, (int)motor.adcDisplay, aFlag);
}


/* Read autolauncher configuration parameters */
void launcher_read_parameters(void){
	// get parameters from eeprom or assign default values
		eeprom_read_nbytes(AL_CONFIGED1B, sizeof(launcher.configured), &launcher.configured);
		// read values if they were configured previously
		if(launcher.configured == 'Y'){
			printf("\r\n** Launcher configuration found in memory **\r\n");
			eeprom_read_nbytes(AL_TUBECOUNT1B, sizeof(launcher.tubeCount), &launcher.tubeCount);
			eeprom_read_nbytes(AL_TYPE1B, sizeof(launcher.type), &launcher.type);
			eeprom_read_nbytes(AL_SN1B, sizeof(launcher.serialNumber), &launcher.serialNumber);
			eeprom_read_nbytes(AL_MODE1B, sizeof(launcher.mode), &launcher.mode);
			eeprom_read_nbytes(AL_TIMEOUT2B, sizeof(launcher.timeout), &launcher.timeout);
			eeprom_read_nbytes(AL_VERBOSE1B, sizeof(launcher.verbose), &launcher.verbose);
			// print launcher parameters
			printf("\r\n <AL> Tubes: %c | Type: %c | Serial: %03i | Mode: %i | Timeout: %03i | Verbose: %i <AL>\r\n",
					launcher.tubeCount, launcher.type, launcher.serialNumber, launcher.mode, launcher.timeout, launcher.verbose);
		} else {
			printf("\r\n** Launcher Configuration NOT found in memory **\r\n");
		}
}


/* Set the runtime for the stepper motors */
void menu_set_launcher_mode(void){
	// set mode
	printf("\r\n> Current launcher mode: %i [default=%i]\r\n", launcher.mode, LAUNCHER_MODE_DEFAULT);
	char mode[1];
	char modePrompt[] = "> Enter launcher mode [0] GPS auto stream or [1] normal menu: ";
	char modeError[] = "\r\n** Enter only numbers! **\r\n";
	char modeCheck[] = {'0','1'};
	uint8_t mFlag;
	// loop until a good value is set or 0 to exit
	do{
		mFlag = 0;
		get_user_input(modePrompt, modeError, 1, modeCheck, mode);
		uint32_t m =  (mode[0] - '0');
		// check values are within range
		if(m == 1 || m == 0){
			launcher.mode = (uint8_t) m;
			mFlag = 1;
		} else {
			printf("\r\n** Value out of range! **\r\n");
		}
	} while(!mFlag);

	// set timeout
	printf("\r\n> Current menu timeout: %i s [default=%i]\r\n", launcher.timeout, LAUNCHER_TIMEOUT_DEFAULT);
	char timeout[3];
	char timeoutPrompt[100];
	sprintf(timeoutPrompt, "> [exit=%03i] Enter launcher timeout (3-digits) in seconds [%03i-%03i]: ", 0, LAUNCHER_TIMEOUT_MIN, LAUNCHER_TIMEOUT_MAX);
	char timeoutError[] = "\r\n** Enter only numbers! **\r\n";
	char timeoutCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	uint8_t toFlag, exitFlag = 0;
	// loop until a good value is set or 0 to exit
	do{
		toFlag = 0;
		get_user_input(timeoutPrompt, timeoutError, 3, timeoutCheck, timeout);
		uint32_t to = ( (timeout[0] - '0') * 100 + (timeout[1] - '0') * 10 + (timeout[2] - '0') );
		if(to == 0){
			printf("\r\n** Exit **\r\n");
			exitFlag = 1;
			break;
		}
		// check values are within range
		if( (to >= LAUNCHER_TIMEOUT_MIN) && (to <= LAUNCHER_TIMEOUT_MAX) ){
			launcher.timeout = (uint16_t) to;
			toFlag = 1;
		} else {
			printf("\r\n** Value out of range! **\r\n");
		}
	} while(!toFlag);

	// store launcher mode
	printf("> Launcher mode: %i\r\n", (int)launcher.mode);
	// store in eeprom
	eeprom_write_nbytes(AL_MODE1B, sizeof(launcher.mode), &launcher.mode);
	// test memory
	eeprom_read_nbytes(AL_MODE1B, sizeof(launcher.mode), &launcher.mode);
	printf("> Setting saved! mode: %i\r\n\r\n", (int)launcher.mode);

	// store timeout
	// store variables if it was not an exit
	if(exitFlag == 0){
		// Print all inputs
		printf("> Launcher timeout: %i s\r\n", (int)launcher.timeout);
		// store in eeprom
		eeprom_write_nbytes(AL_TIMEOUT2B, sizeof(launcher.timeout), &launcher.timeout);
		// test memory
		eeprom_read_nbytes(AL_TIMEOUT2B, sizeof(launcher.timeout), &launcher.timeout);
		printf("> Setting saved! timeout: %i\r\n\r\n", (int)launcher.timeout);
	}
}

/* Set verbose mode in main menu to print extra debuggin info after each command is sent */
void menu_set_verbose(void){
	printf("\r\n> Current verbose mode: %i [default=%i]\r\n", launcher.verbose, LAUNCHER_VERBOSE_DEFAULT);
	char verbose[1];
	char verbosePrompt[] = "> Enter [0] only echo or [1] display extra info: ";
	char verboseError[] = "\r\n** Enter only numbers! **\r\n";
	char verboseCheck[] = {'0','1'};
	uint8_t vFlag;
	do{
		vFlag = 0;
		get_user_input(verbosePrompt, verboseError, 1, verboseCheck, verbose);
		uint32_t v = (verbose[0] - '0');
		if( v == 1 || v == 0){
			launcher.verbose = (uint8_t) v;
			vFlag = 1;
		} else {
			printf("\r\n** Value out of range! **\r\n");
		}
	} while(!vFlag);
	// Print all inputs
	printf("> Launcher verbose mode: %i\r\n", (int)launcher.verbose);
	// store in eeprom
	eeprom_write_nbytes(AL_VERBOSE1B, sizeof(launcher.verbose), &launcher.verbose);
	// test memory
	eeprom_read_nbytes(AL_VERBOSE1B, sizeof(launcher.verbose), &launcher.verbose);
	printf("> Setting saved! verbose: %i\r\n\r\n", (int)launcher.verbose);
}


/* Print instructions on how to use configuration menu */
void menu_help_print(void){
	// text terminal format
//	uint8_t fBold_GreenBgnd[] = "\x1b[1;42m"; // bold 1, 42 green background
//	uint8_t fGreenBk[] = "\x1b[42m"; //  42 green background
//	uint8_t fYellowBk[] = "\x1b[43m"; // 43 yellow background
	uint8_t fBlueBk[] = "\x1b[44m"; // 44 blue background
//	uint8_t fMagentaFg[] = "\x1b[35m"; // 35 magenta foreground
	uint8_t fRedFg[] = "\x1b[35m"; // 35 magenta foreground
	uint8_t fReset[] = "\x1b[0m"; // reset all formats
	printf("\r\n|===============================================================================================|\r\n");
	printf("|                                   CONFIGURATION MENU HELP                                     |\r\n");
	printf("|===============================================================================================|\r\n");
	// A
	printf("| %s<A> Set AL tubes, type & S/N%s\r\n", fBlueBk, fReset);
	printf("| Set the autolauncher parameters\r\n");
	printf("| %sTube count%s: number of XBT tubes available (6 or 8)\r\n", fRedFg, fReset);
	printf("| This parameter affects the possibility of driving pins and connecting XBTs 7 & 8\r\n");
	printf("| %sType%s: type of launcher large (X) or regular (R)\r\n", fRedFg, fReset );
	printf("| This is option is not available if Tube count = 6\r\n");
	printf("| %sSerial Number%s: autolauncher unique serial number\r\n", fRedFg, fReset );
	printf("|\r\n");
	// S
	printf("| %s<S> Extend all pins%s\r\n", fBlueBk, fReset);
	printf("| Extend all pins from 1 to 6 or 8\r\n");
	printf("|\r\n");
	// D
	printf("| %s<D> Retract all pins%s\r\n", fBlueBk, fReset);
	printf("| Retract all pins from 1 to 6 or 8\r\n");
	printf("|\r\n");
	// F
	printf("| %s<F> Grease pins mode%s\r\n", fBlueBk, fReset);
	printf("| Retract and extend each pin up to 10 times\r\n");
	printf("| or until the user sends '@'\r\n");
	printf("|\r\n");
	// G
	printf("| %s<G> Clear memory range%s\r\n", fBlueBk, fReset);
	printf("| Resets to 0 a range of bytes in EEPROM  \r\n");
	printf("| Memory addresses are printed out  \r\n");
	printf("| %sStart address%s: 0 to 127\r\n", fRedFg, fReset );
	printf("| %sEnd address%s: 0 to 127 & should be greater than Start address\r\n", fRedFg, fReset );
	printf("|\r\n");
	// H
	printf("| %s<H> Read motor statistics%s\r\n", fBlueBk, fReset);
	printf("| Prints the maximum logged curent & use count for each motor\r\n");
	printf("|\r\n");
	// J
	printf("| %s<J> Set motor runtime%s\r\n", fBlueBk, fReset);
	printf("| Set the time (ms) the motors run to retract/extend pin  \r\n");
	printf("|\r\n");
	// K
	printf("| %s<K> Set ADC sampling period%s\r\n", fBlueBk, fReset);
	printf("| Set the time interval (ms) between ADC samples while motor is running\r\n");
	printf("| ADC samples taken: current, input voltage, internal STM32 temperature\r\n");
	printf("|\r\n");
	// L
	printf("| %s<L> Set motor wiring mode%s\r\n", fBlueBk, fReset);
	printf("| Set the wiring code for the stepper motors to 0 or 1\r\n");
	printf("| Based on the color sequence connected in the motor terminal,    \r\n");
	printf("| a 0 or 1 will make a motor turn CW or CCW when a extending/retracting a pin\r\n");
	printf("| Color code is indicated in the PCB: blue, white, green, white, black, red\r\n");
	printf("|\r\n");
	// W
	printf("| %s<W> Set motor PWM frequency%s\r\n", fBlueBk, fReset);
	printf("| Set the step signal frequency (Hz) to drive the DRV8826 controller\r\n");
	printf("|\r\n");
	// E
	printf("| %s<E> Reset motor statistics%s\r\n", fBlueBk, fReset);
	printf("| Clears to 0 max current and use count for the selected motor\r\n");
	printf("|\r\n");
	// R
	printf("| %s<R> Read motor configuration%s\r\n", fBlueBk, fReset);
	printf("| Read all motor parameters stored in memory:\r\n");
	printf("| Runtime, ADC sample interval, Wiring mode, PWM frequency\r\n");
	printf("|\r\n");
	// T
	printf("| %s<T> Launcher mode %s\r\n", fBlueBk, fReset);
	printf("| %sMode%s: The launcher mode determines the behavior of the AL Tx line in standby\r\n", fRedFg, fReset);
	printf("| Mode 0: the launcher will stream GPS data by default after a period (timeout) of inactivity\r\n");
	printf("| Mode 1: the launcher behaves as usual, only displaying the output from the STM32\r\n");
	printf("| %sTimeout%s: The time in seconds after a period of inactivity to start streaming GPS data\r\n", fRedFg, fReset);
	printf("|\r\n");
	// Y
	printf("| %s<Y> Verbose mode %s\r\n", fBlueBk, fReset);
	printf("| Mode 0: the launcher will only echo characters received while in main menu\r\n");
	printf("| Mode 1: the launcher will display additional information for debugging purposes\r\n");
	printf("|\r\n");
	// U
	printf("| %s<U> Read launcher parameters %s\r\n", fBlueBk, fReset);
	printf("| Read current launcher S/N, tubes, type, mode, timeout & verbose mode\r\n");
	printf("|\r\n");
	printf("|===============================================================================================|\r\n\r\n");

}


/********************************************** AUXILIAR FUNCTIONS **********************************************/

/* Get the use input from serial terminal
 * Parameters:
 * - Message prompt to direct the user what to do
 * - Message in case of error
 * - Count of character to be entered by the user
 * - A check list to limit the char input from the user to values in this list only
 * - Pointer to the list of characters entered */
void get_user_input(char promptMsg[], char errorMsg[], uint8_t count, char checkList[], char * output){
	//const uint8_t checkListSize = 10;
	print_inline(promptMsg);
    for(uint8_t i = 0; i < count; i++){
		while(1){
			HAL_Delay(1); // needed to debug, remove
			if(rxStatus == active){
				rxStatus = idle;
				print_char(rxChar);
				uint8_t checkFlag = 0;
				// check that belongs to the checkList
				for(uint8_t j = 0; j < MAX_CHECKLIST_SIZE; j++){
					// if there is a match, set flag, store value and break
					if(rxChar == checkList[j]){
						checkFlag = 1;
						output[i] = rxChar;// store the value
						break;
					}
				} // if no match, flag is 0
				if(checkFlag == 0){
					printf(errorMsg);
					print_inline(promptMsg);
					i = 0; // reinitialize counter to start over
				}
				// break while loop if value is good
				if(checkFlag == 1) break;
			}
		}
    }
    printf("\r\n");
}


/* Print a single character for echo in line */
void print_char(uint8_t ch){
	HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 10);
}


/* Print serial number based on AL configuration saved */
void print_serial_number(void){
	//printf( "AL%c%s", launcher.type[0], launcher.serialNumber);
    if(launcher.configured == 'Y'){
    	if(launcher.tubeCount == '6'){
    		printf( "AL%i ", launcher.serialNumber);
    	} else { // if 8 tubes
    		printf( "AL%c%i", launcher.type, launcher.serialNumber);
    	}
    } else {
    	printf( "AL000");
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



/* Select the source of RS232
 * Parameters: select {MUX_GPS, MUX_STM32} */
void multiplexer_set(mux_t select){
	txMode = select; // update tx line mode
	HAL_GPIO_WritePin(MUX_SELECT_GPIO_Port, MUX_SELECT_Pin, select); // SET = UART-tx / RESET = Din from GPS
}


/* Print line without a '\n' newline at the end
 * Use for data entry prompts or partial text inline */
void print_inline(char * text){
	char temp = ' ';
	for(uint8_t i = 0; i<=255 && temp!= '\0' ; i++){
		temp = text[i];
		HAL_UART_Transmit(&huart1, (uint8_t *) &temp, 1, 100);
	}
}


/* Initialize autolauncher parameters from memory or defaults */
void parameter_init(void){
	// read launcher parameters from eeprom
	launcher_read_parameters();
	// read motor parameters from eeprom
	motor_read_parameters();
	// Read motor stats from eeprom
	motor_read_stats();
}


/* UART Receive complete interrupt callback, set rxStatus flag for new char received
 * re-enable uart rx interrupt */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	// check that uart1 triggered the interrupt callback
	if(huart->Instance == USART1){
		rxChar = rxBuffer[0]; // store the only element in buffer to a char for easier variable handling
		rxStatus = active; // set flag to enter main menu char processing
		HAL_UART_Receive_IT(&huart1, (uint8_t *) rxBuffer, 1); // reactivate rx interrupt
	}
}


/* When the ADC conversion in DMA mode is complete (all samples in adc scan), set flag
 * Then the IRQ calls this function */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//HAL_ADC_Stop_DMA(&hadc1);
	if(hadc->Instance == ADC1){
		adcComplete = 1;
	}
}


/* Wrapper for 1st uart_rx call
 * The interrupt is enabled for rx after this function is called, and then disabled until called again */
void uartrx_interrupt_init(void){
	HAL_UART_Receive_IT(&huart1, (uint8_t *) rxBuffer, 1); // enable UART receive interrupt, store received char in rxChar buffer
}

/* External interrupt callback for push button */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == EXTI_BUTTON_Pin){
		if(btnStatus == btn_idle){
			btnTimeStart = HAL_GetTick();
			btnStatus = btn_pushed;
		}
	}
}


/* Check button status with debounce timer */
void debounce_button(void){
	GPIO_PinState currentState = HAL_GPIO_ReadPin(EXTI_BUTTON_GPIO_Port, EXTI_BUTTON_Pin);
	uint8_t timeoutFlag = tick_timeout(btnTimeStart, BUTTON_DEBOUNCE_TIME);
	// if debounce time has elapsed, then check the button status again
	if(timeoutFlag == 1){
		if(btnStatus == btn_pushed && currentState == GPIO_PIN_RESET){
			uint8_t command[] = "\r\n\r\n> Button pushed! <COMMAND TO AMVERSEAS>\r\n\r\n";
			// send command to Maverseas
			if(txMode == MUX_GPS){
				  multiplexer_set(MUX_STM32);
				  HAL_Delay(5);
				  HAL_UART_Transmit(&huart1, (const uint8_t *) command, sizeof(command), 100);
				  //HAL_Delay(5);
				  multiplexer_set(MUX_GPS);
			} else { // if it was already in stm32 mode
				  HAL_UART_Transmit(&huart1, (const uint8_t *) command, sizeof(command), 100);
			}
		}
		// reset button flag
		btnStatus = btn_idle;
	}
}

/* Controls if a given time has elapsed and sets a flag*/
uint8_t check_menu_timeout(uint32_t startTime, uint32_t timeout){
	uint8_t tflag = 0;
	// check timeout is valid
	if( timeout < LAUNCHER_TIMEOUT_MIN || timeout > LAUNCHER_TIMEOUT_MAX){
		timeout = LAUNCHER_TIMEOUT_DEFAULT;
	}
	timeout = timeout * 1000; // convert to ms
	tflag = tick_timeout(startTime, timeout);
	return tflag;
}

/* Returns 1 if a timeout time has elapsed, compared to a start time or a 0 if not
 * Values are in milliseconds */
uint8_t tick_timeout(uint32_t startTime, uint32_t timeout){
	uint8_t tflag = 0;
	uint32_t timeNow = HAL_GetTick();
	if(timeNow >= startTime){
		if((timeNow - startTime) >= timeout) tflag = 1;
	} else { // if timeNow < timeStart, this only happens after an overflow >> uwTick ~ 2^32 (50 days)
		if( (HAL_MAX_DELAY - startTime + timeNow) >= timeout) tflag = 1;
	}
	return tflag;
}


/********************************************** RELAY CONTROL FUNCTIONS **********************************************/

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
		drive_relay(RELAY_K12_CAL_RES_GPIO_Port, RELAY_K12_CAL_RES_Pin, RELAY_ON_TIME); // SET relay k12
		relayLock = reFree;
	}
}

void calibrate_on(void){
	if(relayLock == reFree){
		relayLock = reLocked;
		drive_relay(RELAY_K11_CAL_CONT_GPIO_Port, RELAY_K11_CAL_CONT_Pin, RELAY_ON_TIME); // SET relay k11
		relayLock = reFree;
	}
}

void reset_relay(void){
	if(relayLock == reFree){
		relayLock = reLocked;
		drive_relay(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, RELAY_ON_TIME); // RESET relay k1, k2, k3, k4, SSR1, SSR2, SSR3, SSR4
		HAL_Delay(RELAY_INTERVAL_TIME);
		drive_relay(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, RELAY_ON_TIME); // RESET relay k5, k6, k7, k8, SSR5, SSR6, SSR7, SSR8
		HAL_Delay(RELAY_INTERVAL_TIME);
		drive_relay(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, RELAY_ON_TIME); // RESET relay k9, k10, k11, k12 - This grounds ABC
		relayLock = reFree;
	}
}

void connect_xbt_pin(uint8_t xbtNum){

	if(relayLock == reFree){
		relayLock = reLocked;

		switch (xbtNum){
		case 1:
			drive_relay(RELAY_K1_GPIO_Port, RELAY_K1_Pin, RELAY_ON_TIME); // SET relay k1
			drive_relay(SSR_1_GPIO_Port, SSR_1_Pin, 1); // SET SSR1
			break;
		case 2:
			drive_relay(RELAY_K2_GPIO_Port, RELAY_K2_Pin, RELAY_ON_TIME); // SET relay k2
			drive_relay(SSR_2_GPIO_Port, SSR_2_Pin, 1); // SET SSR2
			break;
		case 3:
			drive_relay(RELAY_K3_GPIO_Port, RELAY_K3_Pin, RELAY_ON_TIME); // SET relay k3
			drive_relay(SSR_3_GPIO_Port, SSR_3_Pin, 1); // SET SSR3
			break;
		case 4:
			drive_relay(RELAY_K4_GPIO_Port, RELAY_K4_Pin, RELAY_ON_TIME); // SET relay k4
			drive_relay(SSR_4_GPIO_Port, SSR_4_Pin, 1); // SET SSR4
			break;
		case 5:
			drive_relay(RELAY_K5_GPIO_Port, RELAY_K5_Pin, RELAY_ON_TIME); // SET relay k5
			drive_relay(SSR_5_GPIO_Port, SSR_5_Pin, 1); // SET SSR5
			break;
		case 6:
			drive_relay(RELAY_K6_GPIO_Port, RELAY_K6_Pin, RELAY_ON_TIME); // SET relay k6
			drive_relay(SSR_6_GPIO_Port, SSR_6_Pin, 1); // SET SSR6
			break;
		case 7:
			drive_relay(RELAY_K7_GPIO_Port, RELAY_K7_Pin, RELAY_ON_TIME); // SET relay k7
			drive_relay(SSR_7_GPIO_Port, SSR_7_Pin, 1); // SET SSR7
			break;
		case 8:
			drive_relay(RELAY_K8_GPIO_Port, RELAY_K8_Pin, RELAY_ON_TIME); // SET relay k8
			drive_relay(SSR_8_GPIO_Port, SSR_8_Pin, 1); // SET SSR8
			break;
		default:
			printf("\r\n** ERROR: XBT %i relay not found **\r\n", xbtNum);
			break;
		}
		relayLock = reFree;
	}
}

void relay_init(void){
	drive_relay(RELAY_RESET_1_GPIO_Port, RELAY_RESET_1_Pin, RELAY_ON_TIME);  // RESET relay k1, k2, k3, k4, SSR1, SSR2, SSR3, SSR4
	HAL_Delay(RELAY_INTERVAL_TIME);
	drive_relay(RELAY_RESET_2_GPIO_Port, RELAY_RESET_2_Pin, RELAY_ON_TIME); // RESET relay k5, k6, k7, k8, SSR5, SSR6, SSR7, SSR8
	HAL_Delay(RELAY_INTERVAL_TIME);
	drive_relay(RELAY_RESET_3_GPIO_Port, RELAY_RESET_3_Pin, RELAY_ON_TIME); // RESET relay k9, k10, k11, k12 (GND, calibration and continuity circuit)
}


void drive_relay(GPIO_TypeDef * relayPort, uint16_t relayPin, uint8_t onTime){
	// SET relay k
	HAL_GPIO_WritePin(relayPort, relayPin, SET); // set
	HAL_Delay(onTime); // time coil is driven in ms
	HAL_GPIO_WritePin(relayPort, relayPin, RESET); // release
	HAL_Delay(2);
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
