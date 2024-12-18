/*
 * eeprom.c
 *
 *  Created on: Dec 6, 2024
 *      Author: Christian.Saiz
 *
 *
 * Model: Microchip AT24XX01
 * Max freq 1 MHz, 1 Kbit memory (1024 bit), 128 x 8-bit block, 5 ms page write,
 * 8-Byte write pages, fixed device address 1010-xxxRW, 128 bytes memory range {00-7F}
 *
 */

/* INCLUDES */
#include "eeprom.h"


/*************************************** EEPROM FUNCTIONS ***************************************/


/* Write 1 byte in epprom
 * Parameters: memory address [0-127], 1 byte of data */
void eeprom_write(uint8_t memoryAddress, uint8_t dataByte){
	uint8_t txBuff[2] = {memoryAddress, dataByte};
	if( memoryAddress >= eeprom.MEMORY_MIN && memoryAddress <= eeprom.MEMORY_MAX ){
		HAL_I2C_Master_Transmit(&hi2c1, EEPROM_BUS_ADDRESS , txBuff, 2, HAL_MAX_DELAY); // send word address, value
		HAL_Delay(eeprom.WAIT); // wait for data to be written
	} else {
		printf("** ERROR: memory address %i out of range [%i-%i] **\r\n", memoryAddress, eeprom.MEMORY_MIN, eeprom.MEMORY_MAX);
	}
}

/* Read 1 byte from epprom
 * Parameters: memory address [0-127]
 * Returns 1 byte of data */
uint8_t eeprom_read(uint8_t memoryAddress){
	uint8_t addressBuffer[1] = {memoryAddress};
	uint8_t rxBuff[1] = {0};
	if( memoryAddress >= eeprom.MEMORY_MIN && memoryAddress <= eeprom.MEMORY_MAX ){
		HAL_I2C_Master_Transmit(&hi2c1, EEPROM_BUS_ADDRESS , addressBuffer, 1, HAL_MAX_DELAY); // dummy write to set pointer to desired memory address
		HAL_Delay(eeprom.WAIT);
		HAL_I2C_Master_Receive(&hi2c1, EEPROM_BUS_ADDRESS, rxBuff, 1, HAL_MAX_DELAY); // send command to read 1 byte at current memory address pointer
		HAL_Delay(eeprom.WAIT);
	} else {
		printf("** ERROR: memory address %i out of range [%i-%i] **\r\n", memoryAddress, eeprom.MEMORY_MIN, eeprom.MEMORY_MAX);
	}
	return ((uint8_t) rxBuff[0]);
}

/* Clear memory within a given range of addresses
 * Parameters: start address and end address (inclusive) [0-127]
 * Returns number of blocks cleared */
uint8_t eeprom_clear(uint8_t memoryStart, uint8_t memoryEnd){
	uint8_t i;
	if( memoryStart >= eeprom.MEMORY_MIN && memoryEnd <= eeprom.MEMORY_MAX && memoryStart <= memoryEnd ){
		for(i = memoryStart ; i <= memoryEnd ; i++){
			eeprom_write(i, 0); // write 0 to corresponding byte
		}
	} else {
		printf("** ERROR: incorrect memory range [%i-%i] or start > end **\r\n", eeprom.MEMORY_MIN , eeprom.MEMORY_MAX);
	}
	return (i-memoryStart);
}

/* print memory map on eeprom */
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
	printf("| [%03i-%03i]    M_RUNTIME                 |\r\n", M_RUNTIME2B,M_RUNTIME2B+1);
	printf("| [%03i-%03i]    M_PWM_FREQ                |\r\n", M_PWM_FREQ2B, M_PWM_FREQ2B+1);
	printf("| [%03i-%03i]    M_SAMPLEPERIOD            |\r\n", M_SAMPLEPERIOD2B, M_SAMPLEPERIOD2B+1);
	printf("| [%03i]        M_WIRING1B                |\r\n", M_WIRING1B);
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
	printf("| [%03i-%03i]    AL_TIMEOUT                |\r\n", AL_TIMEOUT2B, AL_TIMEOUT2B+1);
	printf("| [%03i]        AL_MODE                   |\r\n", AL_MODE1B);
	printf("| [%03i]        AL_VERBOSE                |\r\n", AL_VERBOSE1B);
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
