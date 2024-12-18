/*
 * stepper_drv8826.h
 *
 *  Created on: Dec 6, 2024
 *      Author: Christian.Saiz
 */

#ifndef API_STEPPER_DRV8826_H_
#define API_STEPPER_DRV8826_H_

#include <stdio.h>
#include "retargetio.h"
#include "adc_reading.h"
#include "main.h" // all port, pin definitions

/* DEFINES */
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
#define MOTOR_TOGGLE_LED 1 // toggle led while motor is running after each ADC conversion
#define MOTOR_ADC_DISPLAY_DEFAULT 1 // 1 to print values sampled Imax, Vin & T int, 0 to not display

/* TYPEDEFS */
typedef enum {CCW, CW} motorDir_t; // Relay Control variables

typedef struct {
	uint16_t runTime; // motor runtime in ms [0-65535]
	uint16_t samplePeriod; // ADC sample period in ms for Im, Vin and internal temp
	uint16_t pwmFreq; // PWM frequency to drive stepper motor (step pin drv8826)
	uint8_t wiring; // 0 or 1 will make CW/CCW run the motors in one direction of the other
	uint16_t imax[8]; // max current logged for each motor
	uint16_t count[8]; // counter, number of uses for each motor
	uint8_t adcDisplay;
} motor_t;


/* EXTERNAL VARIABLES */
extern motor_t motor;
extern launcher_t launcher;
extern rxStatus_t rxStatus; // flag, indicate if a new char was sent over serial. Set to 'active' in the UART interrupt callback, and 'idle' after processing command
extern activeMenu_t activeMenu; // menu state to determine how the received command will be processed (config or main)
extern char rxBuffer[1]; // UART1 receive buffer from computer, a char will be stored here with UART interrupt
extern char rxChar; // UART1 receive character, == xBuffer[0]
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* FUNCTION PROTOTYPES */
uint16_t motor_drive(GPIO_TypeDef * motorPort, uint16_t motorPin, motorDir_t motorDirection, uint32_t runtime ); // drive the desired motor
void motor_init(void); // disable all motor enable pins
void motor_select(uint8_t xbtNum, motorDir_t dir);
void motor_count_update(uint8_t xbtNum);
void motor_imax_update(uint8_t xbtNum, uint16_t imax);
void retract_pin(uint8_t xbtNum);
void extend_pin(uint8_t xbtNum);
void retract_all_pins(uint8_t countLimit);
void extend_all_pins(uint8_t countLimit);
void grease_pins(uint8_t cycles);


#endif /* API_STEPPER_DRV8826_H_ */
