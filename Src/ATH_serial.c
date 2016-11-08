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
	volatile uint32_t statusReg=0;
	volatile bool status=false;

	statusReg = huart->Instance->SR ;
	status = ((statusReg & TX_EMPTY) == 0)?false:true;

	return status;

}
//
// Return true if it's safe to send a byte.
//
// TODO Test this.
//
bool rxReady(UART_HandleTypeDef *huart) {
	bool status=false;
	volatile uint32_t statusReg;

	statusReg = huart->Instance->SR ;
	status = ((statusReg & RX_NOT_EMPTY) == 0)?false:true;

	return status;

}
//
// Send a byte.
//
void txByte(UART_HandleTypeDef *huart, uint8_t data) {
	huart->Instance->DR = data;

}

// TODO make timeout work.
void txByteWait(UART_HandleTypeDef *huart,uint8_t data, int timeout) {

	while( txReady(huart) == false ) {
		osThreadYield();
	}

	txByte(huart, data);
}

//
// Receive a byte.
//
uint8_t rxByte(UART_HandleTypeDef *huart) {
	uint8_t data = 0;
	uint32_t dataReg;

	dataReg = huart->Instance->DR ;

	data = (uint8_t) dataReg & 0xff;

	return data;
}

//
// Wait until a byte is received.
//
// TODO make timeout work.
uint8_t rxByteWait(UART_HandleTypeDef *huart,int timeout) {

	while( rxReady(huart) == false ) {
			osThreadYield();
	}
	return( rxByte(huart));
}
