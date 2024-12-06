/*
 * adc_reading.h
 *
 *  Created on: Dec 6, 2024
 *      Author: Christian.Saiz
 */

#ifndef API_ADC_READING_H_
#define API_ADC_READING_H_

#include "adc.h"

/* DEFINES */
#define ADC_BUFFER 3 // number of channels to read with ADC in scan mode (Vin, current and internal temperature)
#define ADC_SAMPLES 100 // number of samples to average ADC values
#define MAX_CHECKLIST_SIZE 10 // max number of items to check against when user inputs a character. Used in get_user_input()
#define ADC_T_SLOPE 4.3 // Internal Temp sensor coef: AVG_SLOPE_avg = 4.3, AVG_SLOPE_min = 4.0, AVG_SLOPE_max = 4.6; // average slope [mV/C]
#define ADC_T_V25_OFFSET 1430 // V25_avg = 1430, V25_min = 1340, V25_max = 1520 ; // Voltage at 25 degrees [mV]
#define ADC_V_SLOPE 0.0083
#define ADC_V_OFFSET 0.3963
#define ADC_I_SLOPE 0.163
#define ADC_I_OFFSET 7.3581

/* TYPEDEFS */
typedef struct {
	uint16_t rawValue; // AD# counts
	float realValue; // physical value
} analog_t;

typedef struct {
	analog_t voltage;
	analog_t current;
	analog_t temperature;
} adcScan_t;

/* EXTERNAL VARIABLES */
extern ADC_HandleTypeDef hadc1;

/* FUNCTION PROTOTYPES */
adcScan_t adc_get_values(void);
uint8_t get_decimal(float value, uint8_t digits);




#endif /* API_ADC_READING_H_ */
