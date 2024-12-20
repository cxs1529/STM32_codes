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
//typedef enum {gps_mode, user_mode} menuMode_t; // used to flag current state of TX line from uC

typedef struct {
	uint8_t serialNumber; // 2 digit autolauncher S/N XX [0-255]
	char tubeCount; // 6 or 8
	char type; // R regular or X extended (amvereseasv6.0, v8.0, v8.1 (long) )
	uint8_t pcbSerial; // 2 digit PCB serialnum ALB3XX [0-255]
	uint8_t mode; // mode 0 to stream GPS after timeout, mode 1 normal AL
	uint8_t verbose; // mode 0 to display only echo or 1 to display debugging info
	uint16_t timeout; // time limit to swith MUX to GPS streaming mode in ms
	char configured; // if the launcher was configured (stored in memory) this will have a 'Y', otherwise 'N'
} launcher_t;

typedef enum  {btn_pushed, btn_idle} button_t; // flag, track when button is pushed
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
// Auxiliar functions
void get_user_input(char promptMsg[], char errorMsg[], uint8_t count, char checkList[], char * output);
void print_inline(char * text);
void print_char(uint8_t ch);
uint8_t is_num(char c);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);
void print_serial_number(void);
void uartrx_interrupt_init(void);
void parameter_init(void);
uint8_t check_menu_timeout(uint32_t startTime, uint32_t timeout);
uint8_t tick_timeout(uint32_t startTime, uint32_t timeout);
void debounce_button(void);
// Menu control functions
void menu_init(void);
void menu_main_print(void);
void menu_config_print(void);
void menu_main_process_input(char option); // process the character received if in main menu
void menu_config_process_input(char option); // process the character received if in config menu
void motor_read_stats(void);
void motor_set_runtime(void);
void motor_set_sampling_period(void);
void motor_set_wiring(void);
void motor_set_pwm_freq(void);
void motor_set_adc_display(void);
void update_timer(TIM_HandleTypeDef * htim, uint32_t period, uint8_t channel ,float dutyCycle);
void motor_reset_stats(void);
void motor_read_parameters(void);
void launcher_read_parameters(void);
void menu_help_print(void);
void menu_config_tubes_type_serial(void);
void menu_print_volt_temp(void);
uint8_t eeprom_reset_stats(uint8_t num);
void menu_set_launcher_mode(void);
void menu_set_verbose(void);
// Relay control functions
void connect_xbt_pin(uint8_t xbtNum); // connect ABC to a desired XBT 1-8
void calibrate_on(void);
void calibration_resistor(void);
void unground_xbt(void);
void reset_relay(void); // reset all relays, ground ABC
void drive_relay(GPIO_TypeDef * relayPort, uint16_t relayPin, uint8_t onTime); // drive the desired relay: port, pin & time coil is energized [ms]
void relay_init(void); // initialized relays in reset state
// RS232 TX control
void multiplexer_set(mux_t select);
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
#define EXTI_BUTTON_Pin GPIO_PIN_13
#define EXTI_BUTTON_GPIO_Port GPIOB
#define EXTI_BUTTON_EXTI_IRQn EXTI15_10_IRQn
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
// motor parameters
#define MOTOR_GREASE_CYCLES 10 // grease pin cycle count
// Relay parameters
#define RELAY_ON_TIME 10 // time to keep relay coils energized in milliseconds
#define RELAY_INTERVAL_TIME 10 // time in between relay activations when in sequence, in milliseconds
// Menu parameters
#define BUTTON_DEBOUNCE_TIME 50 // in ms
#define LAUNCHER_MODE_DEFAULT 0 // [0] GPS stream in standby, [1] normal mode w/GPS after sending command
#define LAUNCHER_TIMEOUT_DEFAULT 30 // timeout in ms. Once this time has elapsed, go back to stream GPS
#define LAUNCHER_TIMEOUT_MIN 1
#define LAUNCHER_TIMEOUT_MAX 600
#define LAUNCHER_VERBOSE_DEFAULT 0 // while in maun menu: [0] to just echo a command while in main menu or [1] to print extra debugging information
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
