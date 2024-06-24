/*
 * API_lcd_i2c.h
 *
 *  Created on: Nov 15, 2023
 *      Author: chris
 */

#ifndef API_INC_API_LCD_I2C_H_
#define API_INC_API_LCD_I2C_H_

#endif /* API_INC_API_LCD_I2C_H_ */

/************************************************* INCLUDES *************************************************
 */

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"


/************************************************* LCD DEFINES *************************************************
 * LCD-specific defines
 * Control bit positions and values for the LCD controller HD44780
 */

#define LCD_ADDRESS 0b00100111 // 0x27 >> PCF8574 0 0 1 0 0 A2 A1 A0  & A2=A1=A0=1

// Control bit positions in sent byte
#define RS_POS	0 // P0
#define RW_POS	1 // P1
#define E_POS	2 // P2
#define BT_POS	3 // P3
#define DB4_POS	4 // P4
#define DB5_POS 5 // P5
#define DB6_POS 6 // P6
#define DB7_POS 7 // P7

#define DB3_POS 3
#define DB2_POS 2
#define DB1_POS 1
#define DB0_POS 0

// Control bit values
#define RS_DATA 1
#define RS_INSTRUCTION 0
#define RW_READ 1
#define RW_WRITE 0
#define E_HIGH 1
#define E_LOW 0
#define BT_ON 1
#define BT_OFF 0

// Configuration bit position ---
// Entry Mode
#define INC_DEC_POS 1
#define SHIFT_DISPLAY_ON_OFF_POS	0
// Display control
#define DISPLAY_ON_OFF_POS 2
#define CURSOR_ON_OFF_POS 1
#define BLINK_ON_OFF_POS 0
// Cursor & display shift
#define CURSOR_DISPLAY_SHIFT_POS 3
#define SHIFT_DIRECTION_POS 2
// Function set
#define DATA_LENGTH_POS	4
#define DISPLAY_LINES_POS 3
#define CHAR_FONT_POS 2
// Read & busy flag address
#define BUSY_FLAG_POS 7

// Configuration bit values ---
// Entry mode
#define INCREMENT 1 // I/D
#define DECREMENT 0 // I/D
#define SHIFT_DISPLAY_ON	1 // S
#define SHIFT_DISPLAY_OFF	0 // S
// Display control
#define DISPLAY_ON	1 // D
#define DISPLAY_OFF	0 // D
#define CURSOR_ON	1 // C
#define CURSOR_OFF	0 // C
#define CURSOR_BLINK_ON		1 // B
#define CURSOR_BLINK_OFF	0 // B
// Cursor & display shift
#define DISPLAY_SHIFT	1 // S/C
#define CURSOR_SHIFT	0 // S/C
#define SHIFT_RIGHT		1 // R/L
#define SHIFT_LEFT		0 // R/L
// Function set
#define DATA_LENGTH_8BIT	1 // DL
#define DATA_LENGTH_4BIT	0 // DL
#define DISPLAY_LINES_1		0 // N
#define DISPLAY_LINES_2		1 // N
#define CHAR_FONT_5X10	1 // F
#define CHAR_FONT_5X8	0 // F
// Read & busy flag address
#define BUSY		1 // BF
#define NOT_BUSY	0 // BF

// Byte operations
#define HIGH_NIBBLE 0xF0
#define LOW_NIBBLE	0x0F

// I2C RW bit
#define I2C_WRITE	0
#define I2C_READ	1

// Address map for the 16x2 LCD screen 'DDRAM'
static const uint8_t ddram_address_16x2[2][16] = {
		{0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F},
		{0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F}
};



/************************************************* LCD FUNCTION DECLARATIONS *************************************************
 * LCD control functions
 * Write to char or strings to LCD, set position, initialize LCD, clear screen, control backlight
 * Enough time in between instructions is very important to execute all of them
 */

void lcd_send_byte(uint8_t byte, bool rs, bool rw); // send a byte to the LCD / chose RS_COMMAND/RS_DATA, RW_READ/RW_WRITE
void send_bytes_i2c(uint8_t slaveAddress, uint8_t byteSequence[], uint8_t sequenceSize, bool i2c_rw); // send bytes over I2C / specify if it's a READ/WRITE operation
void lcd_init(); // LCD initialization for 16x2 I2c
void lcd_set_position(uint8_t row, uint8_t column); // set cursor before writing a new char [1-2,1-16]
void lcd_print_text(uint8_t text[], uint8_t size); // print a string to the LCD
void lcd_clear(); // clear LCD screen
void return_home(); // move cursor to 1,1
void control_backlight(bool state); // turn on/off backlight
void i2c_linker(I2C_HandleTypeDef * i2cInstance); // bring i2c handle from main / use AFTER MX_I2C1_Init();
void shift_display(bool shiftType, bool direction);
void create_character(uint8_t index, uint8_t mychar[]); // 0-7 storage index, 8 memory locations, array with character shape
