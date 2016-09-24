/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */
  /* COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"
//
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "ATH_serial.h"

#include "container.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define STACK_SIZE 256
#define USART_BUFFER 32
// 
// Commands
//
#define READ_CMD  0x10
#define WRITE_CMD 0x20
#define TOGGLE_CMD 0x30 // Only makes sense for digital  Command is WT
#define SUB   0x80

#define MASK_CMD  0xf0

// Sub-command, or functions
//
#define DIGITAL_CMD  0x01
#define ANALOG_CMD   0x02
#define SETTINGS_CMD 0x04

#define SUB_CMD 0x01
#define UNSUB_CMD 0x02


#define MASK_FUNC    0x0f

enum appTaskQs {
	NO_TASK=0,
	RLY_TASK,
	SER_L_TASK,
    SER_S_TASK,
    BTN_TASK,
	LAST_TASK
};

struct taskData {
	osMutexId lock;
	QueueHandle_t q;

	uint8_t dest;
	uint8_t defaultDest;
};

struct taskData *task[LAST_TASK];

QueueHandle_t getQ(uint8_t idx) {
	QueueHandle_t q;
	struct taskData *d;

	d=task[idx];

	if( d != NULL ) {
		osMutexWait(d->lock, osWaitForever);
		q = d->q;
		osMutexRelease(d->lock);
	} else {
		q = NULL;
	}

	return q;
}

struct cmdMessage {
	uint8_t cmd;
	uint8_t addr;
	uint16_t data;
};

// osMessageQId taskQs[LAST_TASK];


// TODO Move this into main
#define QDEPTH 8
osMessageQDef(relayQ, QDEPTH, uint32_t ); // Declare a Message queue
osMessageQDef(senderQ, QDEPTH, uint32_t ); // Declare a Message queue

osThreadId relayThreadHandle;
osThreadId serialListenerThreadHandle;
osThreadId serialSenderThreadHandle;

osMutexDef(uart2txLock);
osMutexId uart2txLockId;

osMutexDef(uart2rxLock);
osMutexId uart2rxLockId;

// Ignore attempt to address beyond that.
//
// 0 -> PD 12
// 1 -> PD 13
// 2 -> PD 14
// 3 -> PD 15
//
void switchRelay( uint8_t addr, bool state) {

	uint16_t pin;
	uint16_t base = 0x1000; // Relay 0
	addr &= 0x03 ; // Limit to range 0 to 3
	pin = base << addr;

// NOTE Relays attached invert the output, so o/p hi means relay off
//
	state = (state)?false:true;

	HAL_GPIO_WritePin(GPIOD, pin,(GPIO_PinState) state);

}
bool readRelay( uint8_t addr) {
	GPIO_PinState state;
	uint16_t pin;
	uint16_t base = 0x1000; // Relay 0

	addr &= 0x03 ; // Limit to range 0 to 3
	pin = base << addr;

	// GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
	state = HAL_GPIO_ReadPin(GPIOD, pin);

	return( (state==0)?true:false  );

}

void allRelaysOff() {
	int i;

	for(i=0;i<4;i++) {
		switchRelay(i,false);
	}
}
// In this version there are 4 outputs, 0-3

osStatus safeSerialSend( UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t len) {
	osStatus rc;
	volatile HAL_StatusTypeDef ustatus;
	int i;

	rc = osMutexWait(uart2txLockId, osWaitForever);
	for(i=0;i<len;i++) {
		txByteWait(uart,buffer[i],-1);
	}
	rc = osMutexRelease(uart2txLockId);

	return rc;
}

osStatus safeSerialReceive( UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t len) {
	osStatus rc;
	HAL_StatusTypeDef ustatus;

	rc = osMutexWait(uart2rxLockId, osWaitForever);

	for(int i=0;i<len;i++) {
		buffer[i] = rxByteWait(uart,-1);
	}
	rc = osMutexRelease(uart2rxLockId);
	return rc;
}

#define MAX_SUB 4  // Maximum number of subscribers;

void relayThread(void const *args) {
	osEvent evt;

	HAL_StatusTypeDef status;
	struct cmdMessage data;
	bool valid=false;
	bool readData  = false;
	bool writeData = false;
	bool toggleData = false;
	bool subscribeCmd = false;

	volatile QueueHandle_t qh;

	uint8_t subList[MAX_SUB];
	uint32_t xfer;
	uint8_t idx=0;

	struct taskData *myData;

	myData = malloc(sizeof(struct taskData ));
	memset(myData,0,sizeof(struct taskData));

	osMutexDef(relayLock);

	myData->lock = osMutexCreate(osMutex(relayLock));
	myData->q    = osMessageCreate(osMessageQ(relayQ), NULL);

	task[ RLY_TASK] = myData ;

	memset(&subList,0,sizeof(subList));

	while(1) {
		qh = getQ(RLY_TASK);
		evt = osMessageGet(qh, osWaitForever);

		memcpy(&data,&evt.value.v,sizeof(uint32_t));

		readData   = (((data.cmd & MASK_CMD) == READ_CMD)  && (( data.cmd & MASK_FUNC ) == DIGITAL_CMD))?true:false ;
		writeData  = (((data.cmd & MASK_CMD) == WRITE_CMD) && (( data.cmd & MASK_FUNC ) == DIGITAL_CMD))?true:false ;
		toggleData = (((data.cmd & MASK_CMD) == TOGGLE_CMD) && (( data.cmd & MASK_FUNC ) == DIGITAL_CMD))?true:false ;

		subscribeCmd =  (((data.cmd & MASK_CMD) == SUB) && (( data.cmd & MASK_FUNC ) == SUB_CMD))?true:false ;
		subscribeCmd |= (((data.cmd & MASK_CMD) == SUB) && (( data.cmd & MASK_FUNC ) == UNSUB_CMD))?true:false ;

		valid = (readData || writeData || toggleData || subscribeCmd )?true:false;

		if (valid ) {
			data.cmd &=0x0f;
			data.cmd |= WRITE_CMD ;

			if (writeData) {
				switchRelay(data.addr,data.data );
			}

			if(readData) {
				if(readRelay(data.addr) ) {
					data.data = 0x01;
				} else {
					data.data = 0x00;
				}
			}
			// TODO add in toggle data.

			if( subscribeCmd ) {
				//
				// If I get to here you have either asked to subscribe, or un-subscribe
				//
				bool subFlag = false;

				if( (data.cmd & MASK_FUNC) == SUB_CMD ) {
					if( idx < MAX_SUB) {
						subList[idx++]=data.data & 0x0ff;
					}
					// TODO No unsub
				}
			}


			if( readData || writeData) {
				volatile QueueHandle_t qh;

				// TODO Loop over sub list
				memcpy(&xfer, &data, sizeof(uint32_t));

				for(int i=0;i<MAX_SUB;i++) {
					if( subList[i] != 0) {
						qh = getQ( subList[i]) ;
						status=osMessagePut(qh, (uint32_t )xfer, osWaitForever);
					}
				}
			}

		}

		osThreadYield();
	}
}

void serialSenderThread(void const *args) {
	osEvent evt;
	uint8_t txBuffer[8];
	volatile HAL_StatusTypeDef status;
	struct cmdMessage data;
	bool cmdValid = false ;
	uint32_t xfer;

	struct taskData *myData;
	volatile QueueHandle_t qh;

	myData = malloc(sizeof(struct taskData ));
	memset(myData,0,sizeof(struct taskData));

	osMutexDef(senderLock);

	myData->lock = osMutexCreate(osMutex(senderLock));
	myData->q    = osMessageCreate(osMessageQ(senderQ), NULL);

	task[SER_S_TASK] = myData;

	status = safeSerialSend( &huart2, (uint8_t*)"\0ST   ", 6);

	// TODO this needs refactoring into a subscribe/unsubscribe function
	memset(&data,0,sizeof(struct cmdMessage));

	data.cmd = SUB | SUB_CMD;
	data.addr = 0x00;
	data.data = SER_S_TASK;

	memcpy(&xfer, &data, sizeof(uint32_t));

	qh = getQ(RLY_TASK);
	do {
		qh = getQ(RLY_TASK);
		if(qh == NULL) {
			osDelay(10);
		}
	} while( qh == NULL);

	status=osMessagePut(qh, (uint32_t )xfer, osWaitForever);


	memset(&data,0,sizeof(struct cmdMessage));
	while(1) {
		status = safeSerialSend( &huart2, (uint8_t*)"\0ST   ", 6);

		qh = getQ(SER_S_TASK);
		evt = osMessageGet(qh, osWaitForever);
		//
		// TODO Build command to send from rx message.
		//
		memcpy(&data,&evt.value.v,sizeof(uint32_t));

		if (( data.cmd & MASK_CMD ) == WRITE_CMD ) {
			txBuffer[1] = 'W';
			cmdValid=true;
		} else if (( data.cmd & MASK_CMD ) == READ_CMD ) {
			txBuffer[1] = 'R';
			cmdValid=true;
		}

		if (cmdValid) {
			if((data.cmd & MASK_FUNC) == DIGITAL_CMD ) {
				txBuffer[2] = 'D';
			} else {
				cmdValid = false;
			}
		}

		if( cmdValid ) {
			txBuffer[0] = (uint8_t)0; // Address, dummy for now.
			txBuffer[3] = data.addr;
			txBuffer[4] = data.data & 0x00ff ;
			txBuffer[5] = (data.data & 0xff00) >> 8;
			status = safeSerialSend( &huart2, txBuffer, 6);
		}
		osThreadYield();
	}
}

void serialListenerThread(void const *args) {
	uint8_t rxBuffer[8];
	osStatus status;

	struct cmdMessage cmd;
	uint32_t data;
	bool valid = false;
	volatile QueueHandle_t qh;
	struct taskData *myData;

	myData = malloc(sizeof(struct taskData ));
	memset(myData,0,sizeof(struct taskData));

	osMutexDef(listenerLock);

	myData->lock = osMutexCreate(osMutex(listenerLock));
	myData->q = NULL;
	// TODO use this
	myData->defaultDest = RLY_TASK ;

	task[SER_L_TASK] = myData;

	printf("Hello\n");


	while(1) {
		valid = false;
		memset(rxBuffer,0,sizeof(rxBuffer));
		memset(&cmd,0,sizeof(cmd));

		status = safeSerialReceive( &huart2, rxBuffer, 6);

		if( rxBuffer[1] == 'W') {
			cmd.cmd = (uint8_t) WRITE_CMD ;
			valid = true;

		} else if(rxBuffer[1] == 'R') {
			cmd.cmd = (uint8_t) READ_CMD ;
			valid = true;
		} else if(rxBuffer[1] == 'T') {
			cmd.cmd = (uint8_t) TOGGLE_CMD ;
			valid = true;
		}

		if( rxBuffer[2] == 'D') {
			cmd.cmd |= DIGITAL_CMD;
			valid = true;
		} else if ( rxBuffer[2] == 'A') {
			// TODO if cmd is toggle then it's and invalid command
			cmd.cmd |= ANALOG_CMD;
			// TODO Add anolog reading, some day
			valid = false;
		}
		//
		// If valid is false this is not a good command, so
		// ignore it.
		//
		// TODO test address 0<= address <=3

		if(valid ) {
			cmd.addr = rxBuffer[3];
			cmd.data = rxBuffer[4] | (rxBuffer[5] << 8);

			memcpy( &data, (void *)&cmd, sizeof(uint32_t) );

			qh = getQ(RLY_TASK);
			status=osMessagePut(qh, (uint32_t)data, osWaitForever);
		}
		osThreadYield();
	}
}


osThreadDef(RLY,   relayThread, osPriorityNormal,0, STACK_SIZE);
osThreadDef(SER_L, serialListenerThread, osPriorityNormal,0, STACK_SIZE);
osThreadDef(SER_S, serialSenderThread, osPriorityNormal,0, STACK_SIZE);

/* USER CODE END 0 */


int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  allRelaysOff();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  uart2rxLockId = osMutexCreate(osMutex(uart2rxLock));
  uart2txLockId = osMutexCreate(osMutex(uart2txLock));

  relayThreadHandle          = osThreadCreate (osThread(RLY), NULL);
  serialSenderThreadHandle   = osThreadCreate (osThread(SER_S), NULL);
  serialListenerThreadHandle = osThreadCreate (osThread(SER_L), NULL);

  /*
   * TODO C++ works !!!!!!!
  struct container* newClass();
  volatile void *a;
  a=newClass();
  runLoop(a);
  */

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
