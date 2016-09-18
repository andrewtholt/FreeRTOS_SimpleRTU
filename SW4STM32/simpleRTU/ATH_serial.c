/*
 * ATH_serial.c
 *
 *  Created on: Sep 18, 2016
 *      Author: andrewh
 */

#include "ATH_serial.h"

// Return true if a byte is waiting.
//
bool txReady(UART_HandleTypeDef *huart) {

	bool status=false;

	return status;

}
//
// Return true if it's safe to send a byte.
//
// TODO Test this.
//
bool rxReady(UART_HandleTypeDef *huart) {
	bool status=false;
	status = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) ? SET : RESET ;

	return status;

}
//
// Send a byte.
//
void txByte(UART_HandleTypeDef *huart, uint8_t data) {

}
//
// Receive a byte.
//
uint8_t rxByte(UART_HandleTypeDef *huart) {
	uint8_t data = 0;

	return data;
}

