/*
 * ATH_serial.h
 *
 *  Created on: Sep 18, 2016
 *      Author: andrewh
 */

#include <stdbool.h>
#include "usart.h"

#ifndef __ATH_SERIAL_H_
#define __ATH_SERIAL_H_

// Return true if a byte is waiting.
//
bool txReady(UART_HandleTypeDef *);
//
// Return true if it's safe to send a byte.
//
bool rxReady(UART_HandleTypeDef *);
//
// Send a byte.
//
void txByte(UART_HandleTypeDef *, uint8_t);
//
// Receive a byte.
//
uint8_t rxByte(UART_HandleTypeDef *);


#endif /* ATH_SERIAL_H_ */
