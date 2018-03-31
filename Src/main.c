/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "msgProcessing.h"
#include "deviceParameters.h"
#include "inttypes.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osMessageQId UART1QueueHandle;
osMessageQId UART2QueueHandle;
osSemaphoreId UART1BinarySemHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define TXRXBUFFERSIZE	100
#define TRUE			1
#define FALSE			0
#define VERBOSE			TRUE

char* TX1Buffer;
char* TX2Buffer;
char* generalBuffer;
char RX1Buffer_Flag;

int ticker;

osThreadId UART1RXTaskHandle;
osThreadId UART2RXTaskHandle;

HAL_StatusTypeDef status;

QueueHandle_t msgQueueHandle;

struct MSG {
	uint8_t type;
	uint8_t ID;
	uint8_t sign;
	uint32_t value;
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void StartUART1ReceiveTask(void const * argument);
void StartUART2ReceiveTask(void const * argument);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void transmit(int channel, char* b);
void toMsg(char* b, char* msg);
void initMsg(struct MSG msg);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	TX1Buffer = (char*) malloc(sizeof(char) * TXRXBUFFERSIZE);
	TX2Buffer = (char*) malloc(sizeof(char) * TXRXBUFFERSIZE);
	generalBuffer = (char*) malloc(sizeof(char) * TXRXBUFFERSIZE);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of UART1BinarySem */
  osSemaphoreDef(UART1BinarySem);
  UART1BinarySemHandle = osSemaphoreCreate(osSemaphore(UART1BinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(UART1RXTask, StartUART1ReceiveTask, osPriorityNormal, 0, 128);
  UART1RXTaskHandle = osThreadCreate(osThread(UART1RXTask), NULL);
  osThreadDef(UART2RXTask, StartUART2ReceiveTask, osPriorityNormal, 0, 128);
  UART2RXTaskHandle = osThreadCreate(osThread(UART2RXTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of UART1Queue */
  osMessageQDef(UART1Queue, 8, uint8_t);
  UART1QueueHandle = osMessageCreate(osMessageQ(UART1Queue), NULL);

  /* definition and creation of UART2Queue */
  osMessageQDef(UART2Queue, 8, uint8_t);
  UART2QueueHandle = osMessageCreate(osMessageQ(UART2Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	msgQueueHandle = xQueueCreate(2, sizeof(struct MSG));
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);

	//Start receiving
	toMsg(generalBuffer, "\n\r\n\r\n\r>>BOOTING\n\r");
	transmit(1, generalBuffer);
	transmit(2, generalBuffer);

	if (msgQueueHandle == NULL) {
		toMsg(generalBuffer, ">>msgQueue Creation Failed\n\r");
		transmit(1, generalBuffer);
		transmit(2, generalBuffer);
	}

	osDelay(1000);

  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
	while (1) {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		toMsg(generalBuffer,
				">>YOU SHOULD NEVER SEE THIS: SCHEDULER FAILED\n\r");
		transmit(1, generalBuffer);
		transmit(2, generalBuffer);

	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//Check if UART1
	if (huart->Instance == USART1) {
		uint8_t byte;
		//Receive one byte
		HAL_UART_Receive_IT(&huart1, &byte, 1);
		//Send byte to queue
		if (xQueueSendToBackFromISR(UART1QueueHandle, (void * ) &byte,
				NULL) != pdTRUE) {
			Error_Handler();
		}
	}
	//Check if UART2
	if (huart->Instance == USART2) {
		uint8_t byte;
		//Receive one byte
		HAL_UART_Receive_IT(&huart2, &byte, 1);
		//Send byte to queue
		if (xQueueSendToBackFromISR(UART2QueueHandle, (void * ) &byte,
				NULL) != pdTRUE) {
			Error_Handler();
		}
	}

}

/* StartUART1TransmitTask function */
void transmit(int channel, char* b) {
	UART_HandleTypeDef c;
	if (channel == 1) {
		c = huart1;
	} else {
		c = huart2;
	}

	if (HAL_UART_Transmit(&c, (uint8_t*) b,
	TXRXBUFFERSIZE, 1000) != HAL_OK) {
		Error_Handler();
	}
}

/* StartUART1TransmitTask function */
void toMsg(char* b, char* msg) {
	memset(b, 0, TXRXBUFFERSIZE);
	sprintf(b, msg);
	return;
}

/* StartUART1TransmitTask function */
void StartUART1ReceiveTask(void const * argument) {
	uint8_t byte;
	memset(TX1Buffer, 0, TXRXBUFFERSIZE);
	sprintf(TX1Buffer, ">>StartUART1TransmitTask: %d\n\r", getEGO());
	transmit(1, TX1Buffer);

	struct MSG msg;

	initMsg(msg);

	osDelay(1000);
	HAL_UART_Receive_IT(&huart1, &byte, 1);
	memset(TX1Buffer, 0, TXRXBUFFERSIZE);
	for (;;) {
		//Check messages from queue
		if (uxQueueMessagesWaiting(UART1QueueHandle) > 0) {
			while (uxQueueMessagesWaiting(UART1QueueHandle) > 0) {
				//Get messages from queue
				xQueueReceive(UART1QueueHandle, &(byte), (TickType_t ) 10);

				if (VERBOSE) {
					memset(TX1Buffer, 0, TXRXBUFFERSIZE);
					sprintf(TX1Buffer, "\n\rGOT: |%c|%d|)", byte, byte);
					transmit(1, TX1Buffer);
				}

				if (aLetter(byte)) {
					// Read message
					if (readMSG(&msg, UART1QueueHandle, byte)) {
						contructMSG(TX1Buffer, &msg,
						TXRXBUFFERSIZE);
						transmit(2, TX1Buffer);
						if (VERBOSE) {
							transmit(1, TX1Buffer);
						}
						if (xQueueSendToBack(msgQueueHandle, (void * ) &msg,
								NULL) != pdTRUE) {
							Error_Handler();
						}

					}

				}
			}
		}
		osDelay(125);
	}
}

/* StartUART2TransmitTask function */
void StartUART2ReceiveTask(void const * argument) {

	uint8_t byte;
	memset(TX2Buffer, 0, TXRXBUFFERSIZE);
	sprintf(TX2Buffer, ">>StartUART2TransmitTask: %d\n\r", getEGO());
	transmit(2, TX2Buffer);

	struct MSG msg;

	initMsg(msg);

	osDelay(1000);
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	memset(TX2Buffer, 0, TXRXBUFFERSIZE);
	for (;;) {
		//Check messages from queue
		if (uxQueueMessagesWaiting(UART2QueueHandle) > 0) {
			while (uxQueueMessagesWaiting(UART2QueueHandle) > 0) {
				//Get messages from queue
				xQueueReceive(UART2QueueHandle, &(byte), (TickType_t ) 10);

				if (VERBOSE) {
					memset(TX2Buffer, 0, TXRXBUFFERSIZE);
					sprintf(TX2Buffer, "\n\rGOT: |%c|%d|)", byte, byte);
					transmit(2, TX2Buffer);
				}

				if (aLetter(byte)) {
					// Read message
					if (readMSG(&msg, UART2QueueHandle, byte)) {
						contructMSG(TX2Buffer, &msg,
						TXRXBUFFERSIZE);
						transmit(1, TX2Buffer);
						if (VERBOSE) {

							transmit(2, TX2Buffer);
						}
						if (xQueueSendToBack(msgQueueHandle, (void * ) &msg,
								NULL) != pdTRUE) {
							Error_Handler();
						}

					}

				}
			}
		}
		osDelay(125);
	}
}

/**
 * @brief  initialise a message structure
 */
void initMsg(struct MSG msg) {
	msg.type = 1;
	msg.ID = 0;
	msg.sign = 0;
	msg.value = 0;
	return;
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */


	struct MSG message;
	ticker = 0;
	int wait;
	osDelay(1000);

	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		osDelay(25);
		ticker = (ticker + 1) % 10;
		if (!ticker) {
			toMsg(generalBuffer, "*");
			transmit(1, generalBuffer);
			transmit(2, generalBuffer);

		}

		if (uxQueueMessagesWaiting(msgQueueHandle) > 0) {
			while (uxQueueMessagesWaiting(msgQueueHandle) > 0) {
				//Get messages from queue
				xQueueReceive(msgQueueHandle, &(message), (TickType_t ) 10);
				contructMSG(generalBuffer, &message, TXRXBUFFERSIZE);
				//transmit(2, generalBuffer);
				//transmit(1, generalBuffer);
			}
			ticker = 1;
		}
	}
  /* USER CODE END 5 */ 
}

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
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		osDelay(1000);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
