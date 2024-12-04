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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {CCW, CW} motorDir_t; // Relay Control variables
typedef enum {MUX_GPS, MUX_STM32} mux_t; // TX from GPS MUX=0, TX from microcontroller MUX=1
typedef enum {AL_TUBECOUNT1B, AL_TYPE1B, AL_SN1B, AL_CONFIGED1B,
				M_RUNTIME2B = 8, M_PWM_FREQ2B = 10 , M_SAMPLEPERIOD2B = 12, M_WIRING1B = 14,
				M_1COUNT2B = 16, M_2COUNT2B = 18, M_3COUNT2B = 20, M_4COUNT2B = 22, M_5COUNT2B = 24, M_6COUNT2B = 26, M_7COUNT2B = 28, M_8COUNT2B = 30,
				M_1MXAMP2B = 32, M_2MXAMP2B = 34, M_3MXAMP2B = 36, M_4MXAMP2B = 38, M_5MXAMP2B = 40, M_6MXAMP2B = 42, M_7MXAMP2B = 44, M_8MXAMP2B = 46} memoryMap_t; // pages 0-7, 8-15, 16-23, 24-31, 32-39,


typedef struct {
	uint8_t serialNumber; // 2 digit autolauncher S/N XX [0-255]
	char tubeCount; // 6 or 8
	char type; // R regular or X extended (amvereseasv6.0, v8.0, v8.1 (long) )
	uint8_t pcbSerial; // 2 digit PCB serialnum ALB3XX [0-255]
	char configured; // if the launcher was configured (stored in memory) this will have a 'Y', otherwise 'N'
} launcher_t;

typedef struct {
	const uint16_t SIZE; // memory size in bytes
	const uint16_t MAX_MEM_ADDRESS; // memory address 0x00 - maxAddress >> 0x7F
	const uint8_t  BUS_ADDRESS;
} eeprom_t;

typedef struct {
	uint16_t runTime; // motor runtime in ms [0-65535]
	uint16_t samplePeriod; // ADC sample period in ms for Im, Vin and internal temp
	uint16_t pwmFreq;
	uint8_t wiring; // 0 or 1 will make CW/CCW run the motors in one direction of the other
	uint16_t imax[8]; // max current logged for each motor
	uint16_t count[8]; // counter, number of uses for each motor
} motor_t;

typedef struct {
	uint16_t rawValue; // AD# counts
	float realValue; // physical value
} analog_t;

typedef struct {
	analog_t voltage;
	analog_t current;
	analog_t temperature;
} adcScan_t;

typedef struct {
	float t_slope;
	float t_offset;
	float v_slope;
	float v_offset;
	float i_slope;
	float i_offset;
} adcCal_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Stepper motor parameters
#define MOTOR_RUNTIME_DEFAULT 8000 // default time to run motor in milliseconds to extend/retract pin, if not configured and stored in memory
#define MOTOR_RUNTIME_MAX 20000 // max allowed time in ms
#define MOTOR_RUNTIME_MIN 2000 // min allowed time in ms
#define MOTOR_SAMPLE_PERIOD_DEFAULT 500 // default time period to sample ADC current, voltage and internal temp
#define MOTOR_SAMPLE_PERIOD_MAX 2000 // max allowed time in ms
#define MOTOR_SAMPLE_PERIOD_MIN 100 // min allowed time in ms
#define MOTOR_PWM_FREQ_DEFAULT 200 // Hz pwm step freq, if not configured in memory
#define MOTOR_PWM_FREQ_MIN 50 // Hz lower limit for config
#define MOTOR_PWM_FREQ_MAX 500 // Hz upper limit for config
#define MOTOR_WIRING_DEFAULT 0 // Use 1 or 0 based on motor wiring. This changes the direction to retract/extend pins
// Relay parameters
#define RELAY_ON_TIME 10 // time to keep relay coils energized in milliseconds
#define RELAY_INTERVAL_TIME 10 // time in between relay activations when in sequence, in milliseconds
// EEPROM parameters
#define EEPROM_BUS_ADDRESS 0xA0 // 0b1010000 7-bit device address
// ADC parameters
#define ADC_BUFFER 3 // number of channels to read with ADC in scan mode (Vin, current and internal temperature)
#define ADC_SAMPLES 100 // number of samples to average ADC values
#define MAX_CHECKLIST_SIZE 10 // max number of items to check against when user inputs a character. Used in get_user_input()
#define ADC_T_SLOPE 4.3 // Internal Temp sensor coef: AVG_SLOPE_avg = 4.3, AVG_SLOPE_min = 4.0, AVG_SLOPE_max = 4.6; // average slope [mV/C]
#define ADC_T_V25_OFFSET 1430 // V25_avg = 1430, V25_min = 1340, V25_max = 1520 ; // Voltage at 25 degrees [mV]
#define ADC_V_SLOPE 0.0083
#define ADC_V_OFFSET 0.3963
#define ADC_I_SLOPE 0.163
#define ADC_I_OFFSET 7.3581
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
//enum memoryMap_t {AL_TUBECOUNT, AL_TYPE, AL_SN1, AL_SN2, AL_CONFIGED, M_RUNTIME } memoryMap;
launcher_t launcher = {0, '\0', '\0', 0, 'N'};
eeprom_t eeprom = {1024, 0x7F, EEPROM_BUS_ADDRESS};
motor_t motor = {MOTOR_RUNTIME_DEFAULT, MOTOR_SAMPLE_PERIOD_DEFAULT, MOTOR_PWM_FREQ_DEFAULT, MOTOR_WIRING_DEFAULT ,{0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,}}; // runtime, samplePeriod, configed, wiring, imax, use count
char rxBuffer[1] = "\0"; // UART1 receive buffer from computer, a char will be stored here with UART interrupt
char rxChar = '\0'; // UART1 receive character, == xBuffer[0]
uint8_t adcComplete = 0; // flag used to print ADC values
uint8_t adcTimerTrigger = 0; // flag used to trigger an ADC conversion in DMA mode, located in TIM4_IRQHandler(void) (stm32f1xx_it.c)
uint8_t adcDMAFull = 0; // this flag is set by the DMA IRQ when the adc buffer is full, located in DMA1_Channel1_IRQHandler(void) (stm32f1xx_it.c)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Auxiliar functions
void get_user_input(char promptMsg[], char errorMsg[], uint8_t count, char checkList[], char * output);
void print_inline(char * text);
void print_char(uint8_t ch);
uint8_t is_num(char c);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);
void print_serial_number(void);
void uartrx_interrupt_init(void);
void test_func(void); // random tests
uint8_t get_decimal(float value, uint8_t digits);
// ADC functions
adcScan_t adc_get_values(void);
// Menu control functions
void menu_init(void);
void menu_main_print(void);
void menu_config_print(void);
void menu_main_process_input(char option); // process the character received if in main menu
void menu_config_process_input(char option); // process the character received if in config menu
void menu_clear_memory(void);
void motor_read_stats(void);
void motor_set_runtime(void);
void motor_set_sampling_period(void);
void motor_set_wiring(void);
void motor_set_pwm_freq(void);
void update_timer(TIM_HandleTypeDef * htim, uint32_t period, uint8_t channel ,float dutyCycle);
void motor_reset_stats(void);
void motor_read_parameters(void);
void menu_help_print(void);
void menu_config_tubes_type_serial(void);
void menu_print_volt_temp(void);
//uint8_t processInput(char option); // process the character received *NOT IN USE*

// Relay control functions
void connect_xbt_pin(uint8_t xbtNum); // connect ABC to a desired XBT 1-8
void calibrate_on(void);
void calibration_resistor(void);
void unground_xbt(void);
void reset_relay(void); // reset all relays, ground ABC
void drive_relay(GPIO_TypeDef * relayPort, uint16_t relayPin, uint8_t onTime); // drive the desired relay: port, pin & time coil is energized [ms]
void relay_init(void); // initialized relays in reset state

// Motor control functions
uint16_t motor_drive(GPIO_TypeDef * motorPort, uint16_t motorPin, motorDir_t motorDirection, uint32_t runtime ); // drive the desired motor
void motor_init(void); // disable all motor enable pins
void motor_select(uint8_t xbtNum, motorDir_t dir);
void motor_count_update(uint8_t xbtNum);
void motor_imax_update(uint8_t xbtNum, uint16_t imax);
void retract_pin(uint8_t xbtNum);
void extend_pin(uint8_t xbtNum);
void retract_all_pins(uint8_t countLimit);
void extend_all_pins(uint8_t countLimit);
// RS232 TX control
void multiplexer_set(mux_t select);
// EEPROM control
uint8_t eeprom_read(uint8_t memoryAddress);
void eeprom_write(uint8_t memoryAddress, uint8_t dataByte);
void eeprom_print_memory_map(void);
uint8_t eeprom_clear(uint8_t memoryStart, uint8_t memoryEnd);
//uint32_t eeprom_read_uint32(uint8_t memoryStart);
//void eeprom_write_uint32(uint8_t baseAddress, uint32_t data);
void eeprom_write_nbytes(uint8_t baseAddress, uint8_t bytes, void * pData);
void eeprom_read_nbytes(uint8_t baseAddress, uint8_t bytes, void * pData);
//void * eeprom_read_struct(module_t module);
void parameter_init(void);
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
  // initialize multiplexer
  multiplexer_set(MUX_STM32);
  // Initialize relays
  relay_init();
  // enable receive interrupt
  uartrx_interrupt_init();
  // Initialize autolauncher parameters i.e. read eeprom
  parameter_init();
  // display main menu at startup
  menu_main_print();
  printf("\r\n> ");

//  test_func();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // menu control loop
	  if(active == rxStatus){ // set to active with UART RX interrupt
		  rxStatus = idle;
		  if( mainMenu == activeMenu){
			  menu_main_process_input(rxChar); // go to main switch case menu
		  } else if ( configMenu == activeMenu){
			  menu_config_process_input(rxChar);
		  }
	  }
	  // monitor voltage and send alarm if it's below a threshold
	  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(1); // needed to debug, remove
	  //HAL_ADC_Start_DMA(&hadc1, &buf, 10);


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

void test_func(void){
	uint16_t a = 30000;
	eeprom_write_nbytes(64, 2, &a);
	uint16_t b;
	eeprom_read_nbytes(64, 2, &b);
	printf("b is: %i", (int)b);

	float c = 3.1415;
	eeprom_write_nbytes(64, 4, &c);
	float d;
	eeprom_read_nbytes(64, 4, &d);
	printf("d is: %i.%i", (int)d, (int)(10000 * (float)(d - (uint8_t)d)));
}

/********************************************** MENU FUNCTIONS **********************************************/

/* Print Main Menu options */
void menu_main_print(void) {
    printf("\r\n\n\r");
    printf("|========================================|\n\r");
    printf("|   AOML Autolauncher board version 3.0  |\n\r");
    printf("|        Firmware version 2024.12.dd     |\n\r");
    printf("|========================================|\n\r");
    printf("|     Model #ALV3.0      S/N ");
    print_serial_number();
    printf("       |\n\r");
    printf("|========================================|\n\r");
    printf("|               COMMANDS                 |\r\n");
//    if (launcher.configured != 'Y') {
//        printf("| ERROR, NO SERIAL NUMBER ASSIGNED 	 |\n\r");
//    }
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
    	printf("| ERROR, NO TUBE COUNT!!                |\n\r");
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
}//end status_message


/* Process char received while in Main menu */
void menu_main_process_input(char option){
	printf("\r\n> Executing OPTION (%c) --> ", option);

	switch (option){
		// Connect XBT pins
    case '0':
        //engage calibration resistor
    	printf("unground_xbt(), calibration_resistor(), calibrate_on()\r\n");
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
    	printf("extend_pin(1)\r\n");
    	extend_pin(1);
        break;
    case 'V':
        printf("extend_pin(2)\r\n");
        extend_pin(2);
        break;
    case 'W':
        printf("extend_pin(3)\r\n");
        extend_pin(3);
        break;
    case 'X':
        printf("extend_pin(4)\r\n");
        extend_pin(4);
        break;
    case 'Y':
        printf("extend_pin(5)\r\n");
        extend_pin(5);
        break;
    case 'Z':
        printf("extend_pin(6)\r\n");
        extend_pin(6);
        break;
    case 'S':
        if (launcher.tubeCount == '8'){
        	printf("extend_pin(7)\r\n");
        	extend_pin(7);
        } else {
        	printf("\r\n* ERROR: tube 7 not available *\r\n");
        }
        break;
    case 'T':
        if (launcher.tubeCount == '8'){
        	printf("extend_pin(8)\r\n");
        	extend_pin(8);
        } else {
        	printf("\r\n* ERROR: tube 8 not available *\r\n");
        }
        break;
        //RETRACT PINS
    case 'A':
    	printf("retract_pin(1)\r\n");
    	retract_pin(1);
        break;
    case 'B':
        printf("retract_pin(2)\r\n");
        retract_pin(2);
        break;
    case 'C':
        printf("retract_pin(3)\r\n");
        retract_pin(3);
        break;
    case 'D':
        printf("retract_pin(4)\r\n");
        retract_pin(4);
        break;
    case 'E':
        printf("retract_pin(5)\r\n");
        retract_pin(5);
        break;
    case 'F':
        printf("retract_pin(6)\r\n");
        retract_pin(6);
        break;
    case 'H':
        if (launcher.tubeCount == '8'){
        	printf("retract_pin(7)\r\n");
        	retract_pin(7);
        } else {
        	printf("* ERROR: tube 7 not available *\r\n");
        }
        break;
    case 'I':
        if (launcher.tubeCount == '8'){
        	printf("retract_pin(8)\r\n");
        	retract_pin(8);
        } else {
        	printf("* ERROR: tube 8 not available *\r\n");
        }
        break;
    case 'K':
    	printf("calibrate_on()\r\n");
        calibrate_on();
        break;
    case 'R':
    	printf("reset_relay(), ground XBT\r\n");
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
        menu_main_print();
        break;
    case '~':
    	printf("menu_config_print()\r\n");
        menu_config_print();
        activeMenu = configMenu; // set configuration menu flag
        break;
    case 's':
    	printf("print_serial_number()\r\n");
    	printf("AL Serial Number: ");
        print_serial_number();
        printf("\r\n");
        break;
    case 'P':
    	printf("menu_print_volt_temp()\r\n");
    	// read input voltage and internal temp on autolauncher
    	menu_print_volt_temp();
    	break;
    case 'N':
    	printf("multiplexer_set(MUX_GPS)\r\n");
    	printf("\r\n** Data TX from local GPS --> press 'O' to set Tx to STM32 **\r\n\r\n");
    	multiplexer_set(MUX_GPS);
    	break;
    case 'O':
    	printf("multiplexer_set(MUX_STM32)\r\n");
    	printf("\r\n** Data TX from STM32 **\r\n");
    	multiplexer_set(MUX_STM32);
    	menu_main_print();
    	break;
    default:
        printf("\r\n** Unrecognized command!!** \r\n");
        break;
	}
	printf("\r\n> ");
}

/* Prints Input voltage and STM32 internal temperature */
void menu_print_volt_temp(void){
	adcScan_t adcReading = adc_get_values();
	printf("\r\n<> Voltage [AD# %i]: %i.%i V | STM32 Temperature [AD# %i]: %i.%i C\r\n",
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
    printf("| <M> This Menu                         |\n\r");
    printf("| <A> Set AL tubes, type & S/N          |\n\r");
    printf("| <S> Extend all pins                   |\n\r");
    printf("| <D> Retract all pins                  |\n\r");
    printf("| <F> Grease pins mode                  |\n\r");
    printf("| <G> Clear memory range                |\n\r");
    printf("| <H> Read motor statistics             |\n\r");
    printf("| <J> Set motor runtime                 |\n\r");
    printf("| <K> Set ADC sampling period           |\n\r");
    printf("| <L> Set motor wiring mode             |\n\r");
    printf("| <W> Set motor PWM frequency           |\n\r");
    printf("| <E> Reset motor statistics            |\n\r");
    printf("| <R> Read motor configuration          |\n\r");
    printf("| <Z> Help                              |\n\r");
    printf("| <Q> QUIT to main menu                 |\n\r");
    printf("|=======================================|\n\r");
    printf("\r\n");
}//end status_message


/* Process char received while in configuration menu */
void menu_config_process_input(char option){
	printf("\r\n> Executing OPTION (%c) --> ", option);

    switch (option) {
        case 'Q':
            printf("\n\rLeaving Auto launcher configuration menu\n\r");
            activeMenu = mainMenu; // set active menu flag to main menu
            menu_main_print();
            break;
        case 'M':
            menu_config_print();
            break;
        case 'A':
        	// get the autolauncher tube count
        	menu_config_tubes_type_serial();
            // print config menu again
            menu_config_print();
            break;
        case 'S':
        	printf("extend_all_pins()\r\n");
        	if(launcher.tubeCount == '6')
        		extend_all_pins(6);
        	if(launcher.tubeCount == '8')
        		extend_all_pins(8);
            break;
        case 'D':
        	printf("retract_all_pins()\r\n");
        	if(launcher.tubeCount == '6')
        		retract_all_pins(6);
        	if(launcher.tubeCount == '8')
        		retract_all_pins(8);
            break;
        case 'F':
            printf("\n\rSend the \"@\" symbol repeatedly to exit grease pins mode\r\n");
            printf("grease_pins();");
            //grease_pins();
            break;
        case 'G':
        	menu_clear_memory();
        	break;
        case 'H':
        	motor_read_stats();
        	break;
        case 'J':
        	// motor runtime
        	motor_set_runtime();
    		break;
        case 'K':
        	motor_set_sampling_period();
        	break;
        case 'L':
        	motor_set_wiring();
        	break;
        case 'W':
        	motor_set_pwm_freq();
        	break;
        case 'E':
        	motor_reset_stats();
        	break;
        case 'R':
        	motor_read_parameters();
        	break;
        case 'Z':
        	menu_help_print();
        default:
        	printf("\r\n** Unrecognized command!!** \r\n");
            break;
    }
    printf("\r\n> ");
}



void menu_config_tubes_type_serial(void){
	printf("Current tube count: %c\r\n", launcher.tubeCount);
	char tubes[1];
	char tubePrompt[] = "\r\nEnter AL tube count [6] or [8]: ";
	char tubeError[] = "\r\nERROR: Enter 6 or 8 !\r\n";
	char tubeCheck[] = {'6','8'};
	get_user_input(tubePrompt, tubeError, 1, tubeCheck, tubes);

    launcher.tubeCount = tubes[0];
    // get the autolauncher type, R regular or X extended, only for 8 tube AL
    if(launcher.tubeCount == '8'){
    	char type[1];
    	char typePrompt[] = "Enter launcher type, [X] extended or [R] regular: ";
    	char typeError[] = "\r\nERROR: Enter X or R !\r\n";
    	char typeCheck[] = {'R','X'};
    	get_user_input(typePrompt, typeError, 1, typeCheck, type);
    	launcher.type = type[0];

    } else {
    	launcher.type = '0'; // if not 8 tubes, reset type to unknown
    }
    printf("Current serial number: %c\r\n", launcher.serialNumber);
	char serial[2];
	char serialPrompt[] = "Enter a two-digit Autolauncher serial number [00-99]: ";
	char serialError[] = "\r\nEnter only numbers!\r\n";
	char serialCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	get_user_input(serialPrompt, serialError, 2, serialCheck, serial);
	launcher.serialNumber = (uint8_t) ( (serial[0] - '0') * 10 + (serial[1] - '0') ); // convert to number, subtract '0' (48 dec)
    launcher.configured = 'Y';
    printf("\r\nTubes: %c | Type: %c | Serial: %i\r\n", launcher.tubeCount, launcher.type, launcher.serialNumber);

    // store parameters in eeprom
    eeprom_write_nbytes(AL_TUBECOUNT1B, sizeof(launcher.tubeCount), &launcher.tubeCount);
    eeprom_write_nbytes(AL_TYPE1B, sizeof(launcher.type), &launcher.type);
    eeprom_write_nbytes(AL_SN1B, sizeof(launcher.serialNumber), &launcher.serialNumber);
    eeprom_write_nbytes(AL_CONFIGED1B, sizeof(launcher.configured), &launcher.configured);
    printf("Settings saved!");
    // test eeprom memory
    eeprom_read_nbytes(AL_TUBECOUNT1B, sizeof(launcher.tubeCount), &launcher.tubeCount);
    eeprom_read_nbytes(AL_TYPE1B, sizeof(launcher.type), &launcher.type);
    eeprom_read_nbytes(AL_SN1B, sizeof(launcher.serialNumber), &launcher.serialNumber);
    eeprom_read_nbytes(AL_CONFIGED1B, sizeof(launcher.configured), &launcher.configured);
    // print stored values
    printf("\r\nStored autolauncher configuration: Tubes: %c | Type: %c | Serial: %i | Configured: %c\r\n",
    		launcher.tubeCount, launcher.type, launcher.serialNumber, launcher.configured);
}


void menu_clear_memory(void){
	uint8_t memStart, memEnd;
	uint8_t validMemory = 0; // valid memory value flag
	// print memory map
	eeprom_print_memory_map();
	// get the memory range to clear - start
	char mem[3]; // buffer to store digits
	char mStartPrompt[] = "\r\n>Enter 3 digit START memory address [000-127]: ";
	char mEndPrompt[] = "\r\n>Enter 3 digit END memory address [000-127]: ";
	char memError[] = "\r\n* ERROR: enter valid numbers *\r\n";
	char memCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	// get start address
	do{
		get_user_input(mStartPrompt, memError, 3, memCheck, mem);
		memStart = (uint8_t) ( (mem[0] - '0') * 100 + (mem[1] - '0') * 10 + (mem[2] - '0') ); // convert to number, subtract '0' (48 dec)
		if((memStart >= 0) && (memStart <= 127)){
			validMemory = 1;
		} else {
			printf("\r\n** Memory out of range! **\r\n");
		}
	} while ( validMemory == 0 );
	// get end address
	validMemory = 0;
	do{
		mem[0] = '\0', mem[1] = '\0' , mem[2] = '\0';

		get_user_input(mEndPrompt, memError, 3, memCheck, mem);
		memEnd = (uint8_t)( (mem[0] - '0') * 100 + (mem[1] - '0') * 10 + (mem[2] - '0') ); // convert to number, subtract '0' (48 dec)
		if((memEnd >= 0) && (memEnd <= 127)){
			validMemory = 1;
		} else {
			printf("\r\n** Memory out of range! **\r\n");
		}
	} while ( validMemory == 0 );
	printf("> %i block/s cleared!\r\n", eeprom_clear(memStart, memEnd));
	// update variables with new stored values
	// read launcher config
	eeprom_read_nbytes(AL_TUBECOUNT1B, sizeof(launcher.tubeCount), &launcher.tubeCount);
	eeprom_read_nbytes(AL_TYPE1B, sizeof(launcher.type), &launcher.type);
	eeprom_read_nbytes(AL_SN1B, sizeof(launcher.serialNumber), &launcher.serialNumber);
	eeprom_read_nbytes(AL_CONFIGED1B, sizeof(launcher.configured), &launcher.configured);
	// read motor config
	eeprom_read_nbytes(M_RUNTIME2B, sizeof(motor.runTime), &motor.runTime);
	eeprom_read_nbytes(M_PWM_FREQ2B, sizeof(motor.pwmFreq), &motor.pwmFreq);
	eeprom_read_nbytes(M_SAMPLEPERIOD2B, sizeof(motor.samplePeriod), &motor.samplePeriod);
	eeprom_read_nbytes(M_WIRING1B, sizeof(motor.wiring), &motor.wiring);
	// print new values
	printf("\r\n<EEPROM>\r\n");
	printf("<AL> Tubes: %c | Type: %c | Serial: %i\r\n", launcher.tubeCount, launcher.type, launcher.serialNumber);
	printf("<MOTOR> Runtime: %i ms | PWM Frequency: %i Hz | Sample Period: %i ms | Wiring: %i\r\n",
			(int)motor.runTime, (int)motor.pwmFreq, (int)motor.samplePeriod, (int)motor.wiring);
	motor_read_stats();
}



/* Read motor use and Imax stored in eeprom memory */
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



void motor_set_runtime(void){
	printf("\r\n>Current runtime: %i ms [default=%i]\r\n", motor.runTime, MOTOR_RUNTIME_DEFAULT);
	char runtime[5];
	char runtimePrompt[100];
	sprintf(runtimePrompt, ">Enter motor runtime (5-digit number) in milliseconds [%05i-%05i]: ", MOTOR_RUNTIME_MIN, MOTOR_RUNTIME_MAX);
	char runtimeError[] = "\r\n** Enter only numbers! **\r\n";
	char runtimeCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	uint8_t rtFlag;
	do{
		rtFlag = 0;
		get_user_input(runtimePrompt, runtimeError, 5, runtimeCheck, runtime);
		uint32_t rt = ( (runtime[0] - '0') * 10000 + (runtime[1] - '0') * 1000 + (runtime[2] - '0') * 100 + (runtime[3] - '0') * 10 + (runtime[4] - '0') );
		if((rt >= MOTOR_RUNTIME_MIN) && (rt <= MOTOR_RUNTIME_MAX)){
			motor.runTime = (uint16_t) rt;
		} else {
			printf("\r\n** Value out of range! **\r\n");
			rtFlag = 1;
		}

	} while(rtFlag);
	// Print all inputs
	printf(">Motor Runtime: %i ms\r\n", (int)motor.runTime);
	// store in eeprom
	eeprom_write_nbytes(M_RUNTIME2B, sizeof(motor.runTime), &motor.runTime);
	// test memory
	eeprom_read_nbytes(M_RUNTIME2B, sizeof(motor.runTime), &motor.runTime);
	printf(">Setting saved! Runtime: %i\r\n\r\n", (int)motor.runTime);
}


void motor_set_sampling_period(void){
	printf("\r\n>Current ADC sampling period: %i ms [default=%i]\r\n", motor.samplePeriod, MOTOR_SAMPLE_PERIOD_DEFAULT);
	char sPeriod[4];
	char sPeriodPrompt[100];
	sprintf(sPeriodPrompt, ">Enter ADC sampling time (4-digit number) in milliseconds [%04i-%04i]: ", MOTOR_SAMPLE_PERIOD_MIN, MOTOR_SAMPLE_PERIOD_MAX);
	char sPeriodError[] = "\r\n** Enter only numbers! **\r\n";
	char sPeriodCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	uint8_t spFlag;
	do{
		spFlag = 0;
		get_user_input(sPeriodPrompt, sPeriodError, 4, sPeriodCheck, sPeriod);
		uint32_t sp = ( (sPeriod[0] - '0') * 1000 + (sPeriod[1] - '0') * 100 + (sPeriod[2] - '0') * 10 + (sPeriod[3] - '0') );
		if(( sp >= MOTOR_SAMPLE_PERIOD_MIN) && (sp <= MOTOR_SAMPLE_PERIOD_MAX) && sp < motor.runTime){
			motor.samplePeriod = (uint16_t) sp;
		} else {
			printf("\r\n** Value out of range or greater than runtime! **\r\n");
			spFlag = 1;
		}
	} while(spFlag);
	// Print all inputs
	printf(">Motor ADC Sample Period: %i ms\r\n", (int)motor.samplePeriod);
	// store in eeprom
	eeprom_write_nbytes(M_SAMPLEPERIOD2B, sizeof(motor.samplePeriod), &motor.samplePeriod);
	// test memory
	eeprom_read_nbytes(M_SAMPLEPERIOD2B, sizeof(motor.samplePeriod), &motor.samplePeriod);
	printf(">Setting saved! Sample Period: %i\r\n\r\n", (int)motor.samplePeriod);
	// verify value read is ok
	if( !(motor.samplePeriod >= MOTOR_SAMPLE_PERIOD_MIN && motor.samplePeriod <= MOTOR_SAMPLE_PERIOD_MAX)){
		motor.samplePeriod = MOTOR_SAMPLE_PERIOD_DEFAULT;
	}
	// update timer PWM registers
	update_timer(&htim4, motor.samplePeriod, 4, 0.5);
	// update timer registers with new adc period (timer 4, channel 4 pwm). See TIM_TypeDef definition
	// Prescaler is 8000, so  TIMER CLK = 8 MHz/8000 -> PWM f = 1 kHz -> 1 cycle/1 ms -> 1 rising edge / 1 ms
//	TIM4->ARR = (uint32_t) motor.samplePeriod-1; // ARR Auto Reload Register, counter Period: 500-1 (500 ms)
//	TIM4->CCR4 = (uint32_t) ((motor.samplePeriod)/2) - 1; // CCR4 Capture Compare Register, channel 4 Pulse: 250-1 (PWM mode 1 -> 0-250 off, 250-499 on) 50% Duty cycle
}


void motor_set_pwm_freq(void){
	printf("\r\n>Current PWM frequency: %i Hz [default=%i]\r\n", motor.pwmFreq, MOTOR_PWM_FREQ_DEFAULT);
	char pwmf[4];
	char pwmfPrompt[100];
	sprintf(pwmfPrompt, ">Enter PWM frequency (4-digit number) in Hz [%04i-%04i]: ", MOTOR_PWM_FREQ_MIN, MOTOR_PWM_FREQ_MAX);
	char pwmfError[] = "\r\n** Enter only numbers! **\r\n";
	char pwmfCheck[] = {'0','1','2','3','4','5','6','7','8','9'};
	uint8_t pwmfFlag;
	do{
		pwmfFlag = 0;
		get_user_input(pwmfPrompt, pwmfError, 4, pwmfCheck, pwmf);
		uint32_t f = ( (pwmf[0] - '0') * 1000 + (pwmf[1] - '0') * 100 + (pwmf[2] - '0') * 10 + (pwmf[3] - '0') );
		if(( f >= MOTOR_PWM_FREQ_MIN) && (f <= MOTOR_PWM_FREQ_MAX)){
			motor.pwmFreq = (uint16_t) f;
		} else {
			printf("\r\n** Value out of range! **\r\n");
			pwmfFlag = 1;
		}
	} while(pwmfFlag);
	// Print all inputs
	printf(">Motor PWM frequency: %i Hz\r\n", (int)motor.pwmFreq);
	// store in eeprom
	eeprom_write_nbytes(M_PWM_FREQ2B, sizeof(motor.pwmFreq), &motor.pwmFreq);
	// test memory
	eeprom_read_nbytes(M_PWM_FREQ2B, sizeof(motor.pwmFreq), &motor.pwmFreq);
	printf(">Setting saved! PWM Freq: %i\r\n\r\n", (int)motor.pwmFreq);
	// verify value read is ok
	if( !(motor.pwmFreq >= MOTOR_PWM_FREQ_MIN && motor.pwmFreq <= MOTOR_PWM_FREQ_MAX)){
		motor.pwmFreq = MOTOR_PWM_FREQ_DEFAULT;
	}
	// get period from frequency
	uint32_t pwmPeriod = 1E6/motor.pwmFreq; // PWM period in us -> 1E6/200 Hz = 5000 us
	// update timer PWM registers
	update_timer(&htim3, pwmPeriod, 3, 0.5);
	// update timer registers with new motor step PWM period (timer 3, channel 3 pwm output). See TIM_TypeDef definition
	// Prescaler is 8, so  TIMER CLK = 8 MHz/8-> PWM f = 1 MHz  -> 1 cycle/1 us -> 1 rising edge / 1 us
//	TIM3->ARR = (uint32_t) pwmPeriod-1; // ARR Auto Reload Register, counter Period: 5000-1 (5 ms)
//	TIM3->CCR3 = (uint32_t) (pwmPeriod/2) - 1; // CCR3 Capture Compare Register, channel 3 Pulse: 2500-1 (PWM mode 1 -> 0-2500 off, 2500-4999 on) 50% Duty cycle
}

void update_timer(TIM_HandleTypeDef * htim, uint32_t period, uint8_t channel ,float dutyCycle){
	// update timer registers with period and duty cycle See TIM_TypeDef definition
	// Example: Prescaler is 8, so  TIMER CLK = 8 MHz/8-> PWM f = 1 MHz  -> 1 cycle/1 us -> 1 rising edge / 1 us
	// ARR Auto Reload Register, counter Period: 5000-1 (5 ms)
	// CCR3 Capture Compare Register, channel 3 Pulse: 2500-1 (PWM mode 1 -> 0-2500 off, 2500-4999 on) 50% Duty cycle
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
}



void motor_set_wiring(void){
	printf("\r\n>Current motor wiring: %i [default=%i]\r\n", motor.wiring, MOTOR_WIRING_DEFAULT);
	char mw[1];
	char mwPrompt[] = ">Enter motor wiring code (1-digit number) [0-1]: ";
	char mwError[] = "\r\n** Enter only numbers! **\r\n";
	char mwCheck[] = {'0','1'};
	uint8_t mwFlag;
	do{
		mwFlag = 0;
		get_user_input(mwPrompt, mwError, 1, mwCheck, mw);
		uint32_t w = (mw[0] - '0');
		if( w == 1 || w == 0){
			motor.wiring = (uint8_t) w;
		} else {
			printf("\r\n** Value out of range! **\r\n");
			mwFlag = 1;
		}
	} while(mwFlag);
	// Print all inputs
	printf(">Motor wiring code: %i\r\n", (int)motor.wiring);
	// store in eeprom
	eeprom_write_nbytes(M_WIRING1B, sizeof(motor.wiring), &motor.wiring);
	// test memory
	eeprom_read_nbytes(M_WIRING1B, sizeof(motor.wiring), &motor.wiring);
	printf(">Setting saved! Motor wiring: %i\r\n\r\n", (int)motor.wiring);
}


void motor_reset_stats(void){
	uint8_t slots = eeprom_clear(M_1COUNT2B, M_8MXAMP2B+2);
	printf("\r\n>%i bytes cleared\r\n", slots);
	motor_read_stats();
}


void motor_read_parameters(void){
	// read motor parameters from eeprom
	motor_t tempMotor;
	eeprom_read_nbytes(M_RUNTIME2B, sizeof(tempMotor.runTime), &tempMotor.runTime);
	eeprom_read_nbytes(M_SAMPLEPERIOD2B, sizeof(tempMotor.samplePeriod), &tempMotor.samplePeriod);
	eeprom_read_nbytes(M_PWM_FREQ2B, sizeof(tempMotor.pwmFreq), &tempMotor.pwmFreq);
	eeprom_read_nbytes(M_WIRING1B, sizeof(tempMotor.wiring), &tempMotor.wiring);

	// check values are within range or use defaults {flag = 'N'}
	char rtFlag = 'D', spFlag = 'D', pfFlag = 'D', wFlag = 'D'; // using default values?
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
	// update timer registers
	uint32_t pwmPeriod = 1E6/motor.pwmFreq; // PWM period in us -> 1E6/200 Hz = 5000 us
	update_timer(&htim3, pwmPeriod, 3, 0.5); // timer 3 motor PWM steps
	update_timer(&htim4, motor.samplePeriod, 4, 0.5); // tiemr 4 ADC sampling period

	printf("\r\n>Non-Volatile Memory {M} OR Default {D}\r\n<MOTOR> Runtime: %i ms {%c} | PWM Frequency: %i Hz {%c} | Sample Period: %i ms {%c} | Wiring: %i {%c} \r\n",
			 (int)motor.runTime, rtFlag, (int)motor.pwmFreq, spFlag, (int)motor.samplePeriod, pfFlag,  (int)motor.wiring, wFlag);
}

void menu_help_print(void){

}


/********************************************** AUXILIAR FUNCTIONS **********************************************/

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

/* Returns the decimal digits of a float as an integer
 * Parameters: float number to retreive decimals, number of decimal digits */
uint8_t get_decimal(float value, uint8_t digits){
	uint8_t dec;
	uint32_t exp = 1;

	for(uint8_t i = 0; i < digits ; i++){
		exp = exp * 10;
	}
	dec = (value - (int)value) * exp;
	return dec;
}

/* Select the source of RS232
 * Parameters: select {MUX_GPS, MUX_STM32} */
void multiplexer_set(mux_t select){
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



/* Initialize autolauncher parameters */
void parameter_init(void){
	// get parameters from eeprom or assign default values
	eeprom_read_nbytes(AL_CONFIGED1B, sizeof(launcher.configured), &launcher.configured);

	if(launcher.configured == 'Y'){
		printf("\r\n** AL configuration found in memory **\r\n");
		eeprom_read_nbytes(AL_TUBECOUNT1B, sizeof(launcher.tubeCount), &launcher.tubeCount);
		eeprom_read_nbytes(AL_TYPE1B, sizeof(launcher.type), &launcher.type);
		eeprom_read_nbytes(AL_SN1B, sizeof(launcher.serialNumber), &launcher.serialNumber);

		printf("\r\n<AL> Tubes: %c | Type: %c | Serial: %03i <AL>\r\n", launcher.tubeCount, launcher.type, launcher.serialNumber);
	} else {
		printf("\r\n** AL Configuration NOT found in memory **\r\n");
	}
	// read motor parameters from eeprom
	motor_read_parameters();
	// Read motor stats
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


/* Timer period finished callback for TIM2
 * Used to trigger an ADC scan conversion in DMA mode for current, voltage and internal temperature */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim->Instance == TIM4){
//		adcTimerTrigger = 1;
//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	}
//}

/* When the ADC conversion in DMA mode is complete (all samples in adc scan)
 * Then the IRQ calls this function */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//HAL_ADC_Stop_DMA(&hadc1);
	if(hadc->Instance == ADC1){
		adcComplete = 1;
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	}
}



/* Wrapper for 1st uart_rx call
 * The interrupt is enabled for rx after this function is called, and then disabled until called again */
void uartrx_interrupt_init(void){
	HAL_UART_Receive_IT(&huart1, (uint8_t *) rxBuffer, 1); // enable UART receive interrupt, store received char in rxChar buffer
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




/*************************************** MOTOR CONTROL FUNCTIONS ***************************************/

// ALV2 (previous firmware) had a sequence with 4 delays of 8 ms, repeated in 300 steps = 4 * 8 ms * 300 = 7200 ms

/* Extend all pins up to countLimit or eeprom.tubes, whichever is smaller
 * Parameter: countLimit, extend all pins up to this number */
void extend_all_pins(uint8_t countLimit){
	if(countLimit > 0){
		for(uint8_t i = 1; (i <= countLimit) && (i <= launcher.tubeCount); i++){
			extend_pin(i);
		}
	}
}

/* Retract all pins up to countLimit or eeprom.tubes, whichever is smaller
 * Parameter: countLimit, retract all pins up to this number */
void retract_all_pins(uint8_t countLimit){
	if(countLimit > 0){
		for(uint8_t i = 1; (i <= countLimit) && (i <= launcher.tubeCount); i++){
			retract_pin(i);
		}
	}
}


/* Extend pin wrapper
 * Parameter: xbtNum [1-8] */
void extend_pin(uint8_t xbtNum){
	if (motor.wiring == 0){ // select spin direction based on wiring
		motor_select(xbtNum, CW);
	} else {
		motor_select(xbtNum, CCW);
	}
}

/* Retract pin wrapper
 * Parameter: xbtNum [1-8] */
void retract_pin(uint8_t xbtNum){
	if (motor.wiring == 0){ // select spin direction based on wiring
		motor_select(xbtNum, CCW);
	} else {
		motor_select(xbtNum, CW);
	}
}

/* Motor driver selector
 * direction to retract/extend may be different based on wiring
 * Parameters: XBT number, direction {CW,CCW} */
void motor_select(uint8_t xbtNum, motorDir_t dir){
	uint16_t rtime = motor.runTime;
	uint16_t imax = 0;

	if((motor.runTime < MOTOR_RUNTIME_MIN) || (motor.runTime > MOTOR_RUNTIME_MAX)){
		rtime = MOTOR_RUNTIME_DEFAULT; // run with default runtime
	}

	if(motorLock == mFree){
		motorLock = mLocked;
		switch (xbtNum){
		case 1:
			imax = motor_drive(ENABLE_M1_GPIO_Port, ENABLE_M1_Pin, dir, rtime);
			break;
		case 2:
			imax = motor_drive(ENABLE_M2_GPIO_Port, ENABLE_M2_Pin, dir, rtime);
			break;
		case 3:
			imax = motor_drive(ENABLE_M3_GPIO_Port, ENABLE_M3_Pin, dir, rtime);
			break;
		case 4:
			imax = motor_drive(ENABLE_M4_GPIO_Port, ENABLE_M4_Pin, dir, rtime);
			break;
		case 5:
			imax = motor_drive(ENABLE_M5_GPIO_Port, ENABLE_M5_Pin, dir, rtime);
			break;
		case 6:
			imax = motor_drive(ENABLE_M6_GPIO_Port, ENABLE_M6_Pin, dir, rtime);
			break;
		case 7:
			imax = motor_drive(ENABLE_M7_GPIO_Port, ENABLE_M7_Pin, dir, rtime);
			break;
		case 8:
			imax = motor_drive(ENABLE_M8_GPIO_Port, ENABLE_M8_Pin, dir, rtime);
			break;
		default:
			printf("\r\n** ERROR: XBT %i motor not found **\r\n", xbtNum);
			break;
		}
		// release motor lock
		motorLock = mFree;
		// update motor stats
		if(xbtNum >= 1 && xbtNum <= 8 ){
			// store use count
			motor_count_update(xbtNum);
			// store Imax if new max is found
			motor_imax_update(xbtNum, imax);
		}
	}
}

/* Update the stepper motor use count
 * Parameter: xbt tube used
 * Warning: The EEPROM memory locations for motor.count[i] must be cleared to 0,
 * otherwise values present from factory will be 0xFFFF (65535) */
void motor_count_update(uint8_t xbtNum){
	motor.count[xbtNum-1]++;
	eeprom_write_nbytes(M_1COUNT2B + (xbtNum-1)*2, sizeof(motor.count[xbtNum-1]), &motor.count[xbtNum-1]);
}

/* Update the maximum logged current for each stepper, if applicable
 * Warning: The EEPROM memory locations for motor.imax[i] must be cleared to 0,
 * otherwise values present from factory will be 0xFFFF (65535) >> any imax */
void motor_imax_update(uint8_t xbtNum, uint16_t imax){
	if( motor.imax[xbtNum-1] < imax){
		motor.imax[xbtNum-1] = imax;
		eeprom_write_nbytes(M_1MXAMP2B + (xbtNum-1)*2, sizeof(motor.imax[xbtNum-1]), &motor.imax[xbtNum-1]);
	}
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
	  // calibrate ADC
	  HAL_ADCEx_Calibration_Start(&hadc1);
}


uint16_t motor_drive(GPIO_TypeDef * motorPort, uint16_t motorPin, motorDir_t motorDirection, uint32_t runTime ){
	uint16_t imax = 0;
	uint32_t timeStart, timeNow;
	uint16_t adcSampleCount = 0;
	adcScan_t adcReading;

	// Initialize PWM
	HAL_GPIO_WritePin(motorPort, motorPin, RESET); // make sure driver pin is disabled
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // initialize PWM pulses for DRV8826
	HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, motorDirection); // set motor direction
	HAL_GPIO_WritePin(motorPort, motorPin, SET); // enable driver to run motor

	timeStart = HAL_GetTick(); // initial timer count using SysTick timer (32 bit variable uwTick incremented every 1 ms, MAX = 50 days)

	// get 1 current, voltage, temp reading every 500 ms using TIM2 interrupts
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4); // trigger adc conversions in DMA mode every x ms
	printf("\r\n");

	while(1){
		// track motor runtime and break loop after desired time elapsed
		timeNow = HAL_GetTick();
		if(timeNow >= timeStart){
			if((timeNow - timeStart) >= runTime) break;
		} else { // if timeNow < timeStart, this only happens when uwTick ~ 2^32 (50 days) and there was an overflow
			if( (HAL_MAX_DELAY - timeStart + timeNow) >= runTime) break;
		}
		// check if user sent stop signal
		if(active == rxStatus){ // set to active with UART RX interrupt
			rxStatus = idle;
			if(rxChar == '@'){
				printf("\r\n** Motor Stopped by user! **\r\n");
				break;
			}
		}
		if(adcTimerTrigger == 1){
			adcTimerTrigger = 0;
			adcReading = adc_get_values();
			// print
			printf("<%02i> Current [AD# %i]: %i.%i mA | Voltage [AD# %i]: %i.%i V | Temperature [AD# %i]: %i.%i C\r\n", (int)adcSampleCount,
					 	 (int)adcReading.current.rawValue, (int)adcReading.current.realValue, get_decimal(adcReading.current.realValue, 1),
						 (int)adcReading.voltage.rawValue, (int)adcReading.voltage.realValue, get_decimal(adcReading.voltage.realValue, 1),
						 (int)adcReading.temperature.rawValue, (int)adcReading.temperature.realValue, get_decimal(adcReading.temperature.realValue, 1));
			adcSampleCount++;
			// update imax
			if(imax < (uint16_t)adcReading.current.realValue){ // if previous imax is less than new one, replace it
				imax = (uint16_t)adcReading.current.realValue;
			}
		}
	}
	// stop PWM and ADC sample timers
	HAL_GPIO_WritePin(motorPort, motorPin, RESET); // disable motor driver
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3); // stop PWM signal to step the motor
	//HAL_ADC_Stop_DMA(&hadc1); // stop ADC conversion if there was one triggered before exiting the while(1)
	HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_4); // stop timer triggering adc conversions

	return imax;
}


/*************************************** ADC CONTROL FUNCTIONS ***************************************/

adcScan_t adc_get_values(void){
	uint16_t adcBuffer[ADC_BUFFER] = {'\0'}; // store 3 ADC measurements in DMA mode: [Vin0,Im0,TempInt0,Vin1,Im1,...]
	uint32_t vAccum = 0, iAccum = 0, tAccum = 0;
	adcScan_t adc = {.current = {0,0}, .voltage = {0,0}, .temperature = {0,0} };

	// Sample ADC scan and fill the DMA buffer (3 channels: AIN10, AIN11, TEMPINT)
	for(uint16_t j = 0; j < ADC_SAMPLES; j++){
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_BUFFER);
		vAccum += adcBuffer[0];
		iAccum += adcBuffer[1];
		tAccum += adcBuffer[2];
//		HAL_Delay(1); // needed for debugging
	}
	HAL_ADC_Stop_DMA(&hadc1); // stop ADC conversions
	// calculate averages and real values
	// voltage
	adc.voltage.rawValue = (uint16_t) ( vAccum / ADC_SAMPLES ); // ADC counts, divide by 3 num of buffer slots since each scan has 3 readings
	adc.voltage.realValue = (float) adc.voltage.rawValue * ADC_V_SLOPE + ADC_V_OFFSET; // calibration coeff should be taken from eeprom
	// current
	adc.current.rawValue = (uint16_t) ( iAccum / ADC_SAMPLES ); // ADC counts
	adc.current.realValue =  (float) adc.current.rawValue * ADC_I_SLOPE + ADC_I_OFFSET; // mA - opAmp G = 50, Rsense = 0.10 ohm
	// internal temperature
	adc.temperature.rawValue = (uint16_t) ( tAccum / ADC_SAMPLES ); // ADC counts
	adc.temperature.realValue = ( (ADC_T_V25_OFFSET - (adc.temperature.rawValue * (3300.0/4096.0) ) )  / ADC_T_SLOPE) + 25.0 ;

	return adc;
}




/*************************************** EEPROM FUNCTIONS ***************************************/
/* Model: Microchip AT24XX01
 * Max freq 1 MHz, 1 Kbit memory (1024 bit), 128 x 8-bit block, 5 ms page write,
 * 8-Byte write pages, fixed device address 1010-xxxRW, 128 bytes memory range {00-7F} */

/* Write 1 byte in epprom
 * Parameters: memory address [0-127], 1 byte of data */
void eeprom_write(uint8_t memoryAddress, uint8_t dataByte){
	uint8_t txBuff[2] = {memoryAddress, dataByte};
	if(memoryAddress <= eeprom.MAX_MEM_ADDRESS ){
		HAL_I2C_Master_Transmit(&hi2c1, EEPROM_BUS_ADDRESS , txBuff, 2, HAL_MAX_DELAY); // send word address, value
		HAL_Delay(10); // wait for data to be written
	} else {
		printf("** ERROR: memory address %x out of range [0-%i] **\r\n", memoryAddress, eeprom.MAX_MEM_ADDRESS);
	}
}

/* Read 1 byte from epprom
 * Parameters: memory address [0-127]
 * Returns 1 byte of data */
uint8_t eeprom_read(uint8_t memoryAddress){
	uint8_t addressBuffer[1] = {memoryAddress};
	uint8_t rxBuff[1] = {0};
	if(memoryAddress <= eeprom.MAX_MEM_ADDRESS ){
		HAL_I2C_Master_Transmit(&hi2c1, EEPROM_BUS_ADDRESS , addressBuffer, 1, HAL_MAX_DELAY); // dummy write to set pointer to desired memory address
		HAL_Delay(10);
		HAL_I2C_Master_Receive(&hi2c1, EEPROM_BUS_ADDRESS, rxBuff, 1, HAL_MAX_DELAY); // send command to read 1 byte at current memory address pointer
		HAL_Delay(10);
	} else {
		printf("** ERROR: memory address %x out of range [0-%i] **\r\n", memoryAddress, eeprom.MAX_MEM_ADDRESS);
	}
	return ((uint8_t) rxBuff[0]);
}

/* Clear memory within a given range of addresses
 * Parameters: start address and end address (inclusive) [0-127]
 * Returns number of blocks cleared */
uint8_t eeprom_clear(uint8_t memoryStart, uint8_t memoryEnd){
	uint8_t i;
	if( (memoryStart >= 0) && (memoryEnd <= eeprom.MAX_MEM_ADDRESS) ){
		for(i = memoryStart ; i <= memoryEnd ; i++){
			eeprom_write(i, 0); // write 0 to corresponding byte
		}
	} else {
		printf("** ERROR: memory out of range [0-%i] **\r\n", eeprom.MAX_MEM_ADDRESS);
	}
	return (i-memoryStart);
}

/* print memory map on eeprom
 * {AL_TUBECOUNT, AL_TYPE, AL_SN1, AL_SN2, AL_CONFIGED, M_RUNTIME } */
void eeprom_print_memory_map(void){
	printf("\r\n");
	printf("|========================================|\r\n");
	printf("|              MEMORY MAP                |\r\n");
	printf("|========================================|\r\n");
	// autolauncher aprameters
	printf("| [%03i]        AL_TUBECOUNT              |\r\n", AL_TUBECOUNT1B);
	printf("| [%03i]        AL_TYPE                   |\r\n", AL_TYPE1B);
	printf("| [%03i]        AL_SN                     |\r\n", AL_SN1B);
	printf("| [%03i]        AL_CONFIGED               |\r\n", AL_CONFIGED1B);
	printf("|========================================|\r\n");
	// motor parameters
	printf("| [%03i-%03i]    M_RUNTIME               |\r\n", M_RUNTIME2B,M_RUNTIME2B+1);
	printf("| [%03i-%03i]    M_PWM_FREQ              |\r\n", M_PWM_FREQ2B, M_PWM_FREQ2B+1);
	printf("| [%03i-%03i]    M_SAMPLEPERIOD          |\r\n", M_SAMPLEPERIOD2B, M_SAMPLEPERIOD2B+1);
	printf("| [%03i-%03i]    M_WIRING1B              |\r\n", M_WIRING1B, M_WIRING1B+1);
	printf("|========================================|\r\n");
	// use count
	printf("| [%03i-%03i]    M_1COUNT                  |\r\n", M_1COUNT2B, M_1COUNT2B+1);
	printf("| [%03i-%03i]    M_2COUNT                  |\r\n", M_2COUNT2B, M_2COUNT2B+1);
	printf("| [%03i-%03i]    M_3COUNT                  |\r\n", M_3COUNT2B, M_3COUNT2B+1);
	printf("| [%03i-%03i]    M_4COUNT                  |\r\n", M_4COUNT2B, M_4COUNT2B+1);
	printf("| [%03i-%03i]    M_5COUNT                  |\r\n", M_5COUNT2B, M_5COUNT2B+1);
	printf("| [%03i-%03i]    M_6COUNT                  |\r\n", M_6COUNT2B, M_6COUNT2B+1);
	printf("| [%03i-%03i]    M_7COUNT                  |\r\n", M_7COUNT2B, M_7COUNT2B+1);
	printf("| [%03i-%03i]    M_8COUNT                  |\r\n", M_8COUNT2B, M_8COUNT2B+1);
	printf("|========================================|\r\n");
	// Max current
	printf("| [%03i-%03i]    M_1MXAMP                  |\r\n", M_1MXAMP2B, M_1MXAMP2B+1);
	printf("| [%03i-%03i]    M_2MXAMP                  |\r\n", M_2MXAMP2B, M_2MXAMP2B+1);
	printf("| [%03i-%03i]    M_3MXAMP                  |\r\n", M_3MXAMP2B, M_3MXAMP2B+1);
	printf("| [%03i-%03i]    M_4MXAMP                  |\r\n", M_4MXAMP2B, M_4MXAMP2B+1);
	printf("| [%03i-%03i]    M_5MXAMP                  |\r\n", M_5MXAMP2B, M_5MXAMP2B+1);
	printf("| [%03i-%03i]    M_6MXAMP                  |\r\n", M_6MXAMP2B, M_6MXAMP2B+1);
	printf("| [%03i-%03i]    M_7MXAMP                  |\r\n", M_7MXAMP2B, M_7MXAMP2B+1);
	printf("| [%03i-%03i]    M_8MXAMP                  |\r\n", M_8MXAMP2B, M_8MXAMP2B+1);
	printf("|========================================|\r\n");
}


/* Write N bytes to eeprom
 * Parameters: starting address on eeprom, number of bytes to write, pointer to data of any type */
void eeprom_write_nbytes(uint8_t baseAddress, uint8_t bytes, void * pData){
    uint8_t *ptr = 0;
    //uint8_t data = 0;
 	for( uint8_t i = 0; i < bytes && i < 4; i++){
		//uint8_t address = baseAddress+i;
 		ptr = pData+i; // cast to 1 byte before adding 1 to address
		//data = *ptr;
		eeprom_write(baseAddress+i, *ptr);
	}
}

/* Read N bytes from eeprom
 * Parameters: starting address on eeprom, number of bytes to read, pointer to store data of any type */
void eeprom_read_nbytes(uint8_t baseAddress, uint8_t bytes, void * pData){
	uint8_t *ptr = 0;
 	for( uint8_t i = 0; i < bytes && i < 4; i++){
		//uint8_t address = baseAddress+i;
 		ptr = pData+i;
		//data = *ptr;
		*ptr = eeprom_read(baseAddress+i);
	}
}

/* Reads a 32-bit number from memory [0- 4,294,967,296]
 * or any 4-byte value  */
//uint32_t eeprom_read_uint32(uint8_t baseAddress){
//	//[byte0][byte1][byte2][byte3] = 32 bit data
//	uint8_t dataByte[4];
//	uint32_t number;
//	for(uint8_t i = 0; i < 4; i++){
//		dataByte[i] = eeprom_read(baseAddress+i);
//	}
//	number = (dataByte[3]<<24) + (dataByte[2]<<16) + (dataByte[1]<<8) + dataByte[0]; // put back the 32 bit number [byte0]+[byte1]+[byte2]+[byte3]
//
//	return number;
//}

/* Writes a 32-bit number to memory [0- 4,294,967,296]
 * or any 4-byte value */
//void eeprom_write_uint32(uint8_t baseAddress, uint32_t data){
//	//[byte0][byte1][byte2][byte3] = 32 bit data
//	uint8_t dataByte[4] = {(data>>0), (data>>8), (data>>16), (data>>24)}; // break up each byte of the 32 bit number
//	for(uint8_t i = 0; i < 4; i++){
//		eeprom_write(baseAddress+i, dataByte[i]);
//	}
//}



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
