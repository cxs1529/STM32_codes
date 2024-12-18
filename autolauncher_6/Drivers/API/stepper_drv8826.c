/*
 * stepper_drv8826.c
 *
 *	STEPPER MOTOR CONTROLLER
 *	Stepper motor: Haydon 36362-12 | driver: TI DRV8826
 *	ALV2 (previous firmware) had a sequence with 4 delays of 8 ms, repeated in 300 steps = 4 * 8 ms * 300 = 7200 ms
 *
 *
 *  Created on: Dec 6, 2024
 *      Author: Christian.Saiz
 */

#include "stepper_drv8826.h"
#include "eeprom.h"


/* GLOBAL VARIABLES */
enum motorLock_t {mFree, mLocked} motorLock = mFree; // motor mutex to ensure one motor runs at a time
uint8_t adcTimerTrigger = 0; // flag used to trigger an ADC conversion in DMA mode, located in TIM4_IRQHandler(void) (stm32f1xx_it.c)


// ALV2 (previous firmware) had a sequence with 4 delays of 8 ms, repeated in 300 steps = 4 * 8 ms * 300 = 7200 ms

/* Extend all pins up to countLimit or eeprom.tubes, whichever is smaller
 * Parameter: countLimit, extend all pins up to this number */
void extend_all_pins(uint8_t countLimit){
	if(countLimit > 0){
		for(uint8_t i = 1; (i <= countLimit) && (i <= launcher.tubeCount); i++){
			printf("\r\n> Extending pin %i\r\n", i);
			extend_pin(i);
			if(rxChar == '@'){
				printf("\r\n** Process finished by user **\r\n");
				break;
			}
		}
	}
}

/* Retract all pins up to countLimit or eeprom.tubes, whichever is smaller
 * Parameter: countLimit, retract all pins up to this number */
void retract_all_pins(uint8_t countLimit){
	if(countLimit > 0){
		for(uint8_t i = 1; (i <= countLimit) && (i <= launcher.tubeCount); i++){
			printf("\r\n> Retracting pin %i\r\n", i);
			retract_pin(i);
			if(rxChar == '@'){
				printf("\r\n** Retract all process finished by user **\r\n");
				break;
			}
		}
	}
}

void grease_pins(uint8_t cycles){
	uint8_t stopFlag = 0;

	printf("\r\n> Greasing pins ...\r\n");
	for(uint8_t j = 0; j < cycles && j < 20 ; j++){
		for(uint8_t k = 1; k <= launcher.tubeCount; k++){
			printf("\r\n> Retracting pin %i\r\n", k);
			retract_pin(k);
			if(rxChar == '@'){
				stopFlag = 1;
				break;
			}
			printf("\r\n> Extending pin %i\r\n", k);
			extend_pin(k);
			if(rxChar == '@'){
				stopFlag = 1;
				break;
			}
		}
		if(stopFlag) break;
	}
	if(stopFlag){
		printf("\r\n** Grease process finished by user **\r\n");
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

	if( !(motor.runTime >= MOTOR_RUNTIME_MIN && motor.runTime <= MOTOR_RUNTIME_MAX) ){
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

/* Initialize all motor drivers in disabled mode */
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


/* Drive desired motor to extend/retract pin, stop with '@'
 * ADC will sample Voltage, current and internal STM32 temperature periodically based on ADC sampling parameters
 * Parameters:
 * - Motor enable port and pin
 * - Motor direction CW/CCW
 * - Motor runtime in ms
 * Return: max current logged in mA */
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
	// initial timer count using SysTick timer (32 bit variable uwTick incremented every 1 ms, MAX = 50 days)
	timeStart = HAL_GetTick();
	// get 1 current, voltage, temp reading every samplePeriod ms using TIM4 events
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4); // trigger adc conversions in DMA mode every x ms
	printf("\r\nMotor Running (Stop with '@') ...\r\n");
	// Run motor checking Systick time against runtime, sample ADC values and stop if '@' is received
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
				printf("\r\n** Motor stopped by user! **\r\n");
				break;
			}
		}
		// Print ADC values when flag is set in TIM4_IRQHandler (stm32f1xx_it.c)
		if(adcTimerTrigger == 1){
			// toggle led if control flag is set by user
			if(MOTOR_TOGGLE_LED == 1){
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			}
			// reset adc timer flag
			adcTimerTrigger = 0;
			// average all values and get conversions
			adcReading = adc_get_values();
			// print values read
			if(motor.adcDisplay == 1){
				printf("<%02i> Current [AD# %i]: %i.%i mA | Voltage [AD# %i]: %i.%i V | Temperature [AD# %i]: %i.%i C\r\n", (int)adcSampleCount,
							 (int)adcReading.current.rawValue, (int)adcReading.current.realValue, get_decimal(adcReading.current.realValue, 1),
							 (int)adcReading.voltage.rawValue, (int)adcReading.voltage.realValue, get_decimal(adcReading.voltage.realValue, 1),
							 (int)adcReading.temperature.rawValue, (int)adcReading.temperature.realValue, get_decimal(adcReading.temperature.realValue, 1));
			}

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
