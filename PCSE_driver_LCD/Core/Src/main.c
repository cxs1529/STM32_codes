/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "API_lcd_i2c.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  i2c_linker(&hi2c1);
  lcd_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t text0[] = "* PCSE *";
  uint8_t text1[] = "CESE 2023";
  uint8_t text2[] = "UBA";
  const uint8_t space[] = " ";

  lcd_clear();
  HAL_Delay(1000);
  lcd_print_text(text0, sizeof(text0));
  HAL_Delay(1000);
  lcd_clear();
  HAL_Delay(1000);
  //lcd_set_position(1, 1);
  return_home();
  lcd_print_text(text1, sizeof(text1));
  lcd_set_position(2, 1);
  lcd_print_text(text2, sizeof(text2));

  lcd_print_text(space, sizeof(space));

  uint8_t mychar1[] = { 0x00, 0x0E, 0x11, 0x11, 0x11,0x0E, 0x00,0x00 }; // empty circle
  uint8_t mychar2[] = {0x00, 0x00, 0x0A, 0x00, 0x11, 0x0E, 0x00, 0x00}; // face
  uint8_t mychar3[] = {0x00,0x0E,0x1F,0x1F,0x1F,0x0E,0x00,0x00}; // full circle

  create_character(0, mychar1);
  create_character(1, mychar2);
  create_character(3, mychar3);

  lcd_set_position(2, 5);

  lcd_send_byte(0, RS_DATA, RW_WRITE);
  lcd_send_byte(1, RS_DATA, RW_WRITE);
  lcd_send_byte(3, RS_DATA, RW_WRITE);

  uint8_t charIndex = 0;

  while (1)
  {
	  // blinking forever
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	  HAL_Delay(500);

	  //lcd_send_byte(0x00, RS_DATA, RW_WRITE);
	  //return_home();
	  //shift_display(DISPLAY_SHIFT,SHIFT_RIGHT);

	  // display available characters
	  lcd_set_position(2, 14);
	  if(charIndex>207) charIndex = 0;
	  lcd_send_byte(charIndex, RS_DATA, RW_WRITE);
	  charIndex++;

	  HAL_Delay(500);


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/*************************************** LCD INSTRUCTIONS *********************************************
 *
 * RS  RW  DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * 0	0	0	0	0	0	0	0	0	1		>> CLEAR SCREEN
 * 0	0	0	0	0	0	0	0	1	x		>> RETURN HOME (GO TO POSITION 0;0)
 * 0	0	0	0	0	0	0	1	ID	S		>> ENTRY MODE ID= 1-increment 0-decrement / S=1-shift display
 * 0	0	0	0	0	0	1	D	C	B		>> DISPLAY CONTROL D=1-display ON, C=1-Cursor ON, B=1-blink ON
 * 0	0	0	0	0	1	SC	RL	x	x		>> CURSOR DISPLAY SHIFT SC=1-display shift 0-cursor move, RL=1-right 0-left
 * 0	0	0	0	1	DL	N	F	x	x		>> Function Set DL=1-8bit mode 0-4bit mode, N=1-2 lines 0-1line, F=15x10 0-5x8
 * 0	0	0	1	ACG	ACG	ACG	ACG	ACG	ACG		>> CGRAM address, data is sent after this command
 * 0	0	1	ADD	ADD	ADD	ADD	ADD	ADD	ADD		>> DDRAM address (lcd position), data is sent after this command
 *
 */
/**************************** WRITE TO LCD FUNCTIONS ***************************/

/* Serial->Parallel: output Byte Px: D7 D6 D5 D4 BT E RW RS */

/*
void lcd_send_byte(uint8_t byte, bool rs, bool rw){
	// byte contains 8 bits of information  / byteType can be INSTRUCTION or DATA
	uint8_t upperByte = (byte & HIGH_NIBBLE); // mask with 1111 0000
	uint8_t lowerByte = (byte << 4); // shift 4 to the left so lower nibble becomes high

	// add rs and rw bits
	upperByte |= ((rs<<RS_POS) | (rw<<RW_POS) | (BT_ON<<BT_POS));
	lowerByte |= ((rs<<RS_POS) | (rw<<RW_POS) | (BT_ON<<BT_POS));
	// E bit 1 0 1 0 to create pulses in LCD controller

	uint8_t byteSequence[4] = {
			(upperByte | (E_HIGH<<E_POS)),
			(upperByte | (E_LOW<<E_POS)),
			(lowerByte | (E_HIGH<<E_POS)),
			(lowerByte | (E_LOW<<E_POS))
	};

	send_bytes_i2c(LCD_ADDRESS, byteSequence, 4, I2C_WRITE);

}

void send_bytes_i2c(uint8_t slaveAddress, uint8_t byteSequence[], uint8_t sequenceSize, bool i2c_rw){
	slaveAddress = ((slaveAddress<<1) | i2c_rw); // i2c WRITE or READ
	HAL_I2C_Master_Transmit(&hi2c1, slaveAddress, byteSequence, sequenceSize, 100);
}

void lcd_clear(){
	lcd_send_byte(0x01, RS_INSTRUCTION, RW_WRITE);
}

void return_home(){
	lcd_send_byte(0x02, RS_INSTRUCTION, RW_WRITE);
}

void lcd_set_position(uint8_t row, uint8_t column){
	// for LCD 16x2 check ranges
	if(row>2) row = 2;
	if(row<1) row = 1;
	if(column>16) column = 16;
	if(column<1) column = 1;

	uint8_t ddram = ddram_address_16x2[row-1][column-1];
	uint8_t ddram_cmd = ddram | (1<<7); // add a 1 in DB7 for DDRAM command

	lcd_send_byte(ddram_cmd, RS_INSTRUCTION, RW_WRITE); // send address as instruction, not data
}

void lcd_print_text(uint8_t text[], uint8_t size){

	for(uint8_t i = 0; i < size; i++){
		HAL_Delay(1); // without this some chars go missing
		lcd_send_byte(text[i], RS_DATA, RW_WRITE);
	}
}

void control_backlight(bool state){
	uint8_t cmd = (state<<BT_POS); // send straight to I2C / not part of the LCD controller
	uint8_t slaveAddress = ((LCD_ADDRESS<<1) | I2C_WRITE); // i2c WRITE or READ
	HAL_I2C_Master_Transmit(&hi2c1, slaveAddress, cmd, 1, 100);
}


void lcd_init(){
	// initialization sequence p46 HD44780 datasheet
	HAL_Delay(50); // wait >40 ms
	lcd_send_byte(0x30, RS_INSTRUCTION, RW_WRITE); // 0 0 1 1 x x x x -> 0x30
	HAL_Delay(10); // wait > 4 ms
	lcd_send_byte(0x30, RS_INSTRUCTION, RW_WRITE);
	HAL_Delay(10); // wait > 0.1 ms
	lcd_send_byte(0x30, RS_INSTRUCTION, RW_WRITE);
	HAL_Delay(10);
	lcd_send_byte(0x20, RS_INSTRUCTION, RW_WRITE); // 0 0 1 DL=0 x x x x -> 0x20 / Function set: DL=4-bit mode
	// start in 4 bit mode
	lcd_send_byte(0x28, RS_INSTRUCTION, RW_WRITE); // 0 0 1 0 N=1 F=0 x x -> 0x28 / Function set: N=2-lines, F=5x8
	HAL_Delay(2);
	lcd_send_byte(0x08, RS_INSTRUCTION, RW_WRITE); // 0 0 0 0 1 D=0 C=0 B=0 -> 0x08 / Display control: D=display off, C=cursor off, B=blink off
	HAL_Delay(2);
	lcd_send_byte(0x01, RS_INSTRUCTION, RW_WRITE); // 0 0 0 0 0 0 0 1 -> 0x01 / Display clear
	HAL_Delay(2);
	lcd_send_byte(0x06, RS_INSTRUCTION, RW_WRITE); // 0 0 0 0 0 1 ID=1 S=0 -> 0x06/ Entry mode: ID=increment, S=no display shift
	HAL_Delay(2);
	lcd_send_byte(0x0C, RS_INSTRUCTION, RW_WRITE); // 0 0 0 0 1 D=1 C=0 B=0 -> 0x0C / Display control:D-display on, C=cursor off, B=blink off

}
*/

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
