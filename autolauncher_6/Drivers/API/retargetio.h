/*
 * retargetio.h
 *
 *  Created on: Nov 12, 2024
 *      Author: Christian.Saiz
 */

#ifndef API_RETARGETIO_H_
#define API_RETARGETIO_H_

#include "stm32f1xx_hal.h"
#include <sys/stat.h>

void RetargetInit(UART_HandleTypeDef *huart);
int _write(int fd, char* ptr, int len);
int _read(int fd, char* ptr, int len);

#endif /* API_RETARGETIO_H_ */
