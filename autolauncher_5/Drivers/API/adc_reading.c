/*
 * adc_reading.c
 *
 *  Created on: Dec 6, 2024
 *      Author: Christian.Saiz
 */

#include "adc_reading.h"


/* Read voltage, current and internal STM32 temperature
 * Average a number of readings defined in ADC_SAMPLES and converts them to real values
 * Returns a struct with AD counts and physical values */
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
