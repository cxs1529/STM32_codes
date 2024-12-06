/*
 * eeprom.h
 *
 *  Created on: Dec 6, 2024
 *      Author: Christian.Saiz
 */


/* INCLUDES */
#ifndef API_EEPROM_H_
#define API_EEPROM_H_

/* INCLUDES */
#include <stdint.h>
#include <stdio.h>
#include <i2c.h>
#include "retargetio.h"

/* DEFINES */
#define EEPROM_BUS_ADDRESS 0xA0 // 0b1010000 7-bit device address

/* TYPEDEFS */
typedef enum {AL_TUBECOUNT1B, AL_TYPE1B, AL_SN1B, AL_CONFIGED1B,
				M_RUNTIME2B = 8, M_PWM_FREQ2B = 10 , M_SAMPLEPERIOD2B = 12, M_WIRING1B = 14,
				M_1COUNT2B = 16, M_2COUNT2B = 18, M_3COUNT2B = 20, M_4COUNT2B = 22, M_5COUNT2B = 24, M_6COUNT2B = 26, M_7COUNT2B = 28, M_8COUNT2B = 30,
				M_1MXAMP2B = 32, M_2MXAMP2B = 34, M_3MXAMP2B = 36, M_4MXAMP2B = 38, M_5MXAMP2B = 40, M_6MXAMP2B = 42, M_7MXAMP2B = 44, M_8MXAMP2B = 46} memoryMap_t; // pages 0-7, 8-15, 16-23, 24-31, 32-39,

typedef struct {
	const uint16_t MEMORY_MAX; // maxAddress >> 0x7F (127)
	const uint16_t MEMORY_MIN; // memory address 0x00 (0)
	const uint8_t I2C_BUS_ADDRESS;
	const uint8_t WAIT; // delay in ms after a write operation
} eeprom_t;

/* FUNCTION PROTOTYPES */
uint8_t eeprom_read(uint8_t memoryAddress);
void eeprom_write(uint8_t memoryAddress, uint8_t dataByte);
void eeprom_print_memory_map(void);
uint8_t eeprom_clear(uint8_t memoryStart, uint8_t memoryEnd);
void eeprom_write_nbytes(uint8_t baseAddress, uint8_t bytes, void * pData);
void eeprom_read_nbytes(uint8_t baseAddress, uint8_t bytes, void * pData);
void eeprom_clear_memory_range(void);

/* VARIABLES */
extern eeprom_t eeprom;
extern I2C_HandleTypeDef hi2c1;

#endif /* API_EEPROM_H_ */
