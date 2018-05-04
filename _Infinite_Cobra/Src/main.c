/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define B_SIZE			50
#define TRUE			1
#define FALSE			0
#define PIN_HI			1
#define PIN_LO			0
#define LSB				0
#define MSB				1
#define VERBOSE	TRUE

#define DAT_A_READ 		HAL_GPIO_ReadPin(DAT_A_GPIO_Port, DAT_A_Pin)
#define DAT_B_READ 		HAL_GPIO_ReadPin(DAT_B_GPIO_Port, DAT_B_Pin)
#define CLK_A_SET		HAL_GPIO_WritePin(CLK_A_GPIO_Port, CLK_A_Pin, GPIO_PIN_SET)
#define CLK_A_RESET		HAL_GPIO_WritePin(CLK_A_GPIO_Port, CLK_A_Pin, GPIO_PIN_RESET)
#define CLK_B_SET		HAL_GPIO_WritePin(CLK_B_GPIO_Port, CLK_B_Pin, GPIO_PIN_SET)
#define CLK_B_RESET		HAL_GPIO_WritePin(CLK_B_GPIO_Port, CLK_B_Pin, GPIO_PIN_RESET)
#define LED3_ON			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET)
#define LED3_OFF		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET)

uint8_t buffer[B_SIZE];
uint8_t empty_buffer[B_SIZE];
uint8_t RX_buffer1[B_SIZE];
uint8_t RX_buffer2[B_SIZE];
uint8_t TX_buffer1[B_SIZE];
uint8_t TX_buffer2[B_SIZE];
uint8_t ADC_buffer[B_SIZE];
uint16_t len, i, j, hmmmm;
int trans_delay = 75;
int ticker = 0;
unsigned long Count;

volatile uint32_t a;
volatile uint32_t ADC_A_Value;
volatile uint32_t ADC_B_Value;
volatile uint32_t ADC_C_Value;
volatile uint32_t ADC_D_Value;
volatile uint32_t ADC_E_Value;
volatile uint32_t ADC_F_Value;

ADC_ChannelConfTypeDef sConfig_A;
ADC_ChannelConfTypeDef sConfig_B;
ADC_ChannelConfTypeDef sConfig_C;
ADC_ChannelConfTypeDef sConfig_D;
ADC_ChannelConfTypeDef sConfig_E;
ADC_ChannelConfTypeDef sConfig_F;

extern __IO uint32_t microDelay;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Update_ADC_Values(void);
void read_HX711(void);
void HAL_Delay_Microseconds(__IO uint32_t);
int isTransmitting(UART_HandleTypeDef *, UART_HandleTypeDef *);
int strip_str(uint8_t[], uint8_t[]);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */
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
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_ADC2_Init();
	MX_ADC1_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	hmmmm = 0;
	char c = '*';
	for (i = 0; i < B_SIZE - 2; ++i) {
		buffer[i] = c;
		RX_buffer1[i] = c;
		RX_buffer2[i] = c;
		ADC_buffer[i] = c;
	}
	buffer[B_SIZE - 2] = '\n';
	buffer[B_SIZE - 1] = '\r';
	RX_buffer1[B_SIZE - 2] = '\n';
	RX_buffer1[B_SIZE - 1] = '\r';
	RX_buffer2[B_SIZE - 2] = '\n';
	RX_buffer2[B_SIZE - 1] = '\r';
	ADC_buffer[B_SIZE - 2] = '\n';
	ADC_buffer[B_SIZE - 1] = '\r';
	len = sizeof(buffer);

	memset(empty_buffer, 0, len);

	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_C) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/* USER CODE BEGIN WHILE */
	int ADC_ENABLE = 0;
	int TX_ENABLE = 1;
	int RX_ENABLE_PASS = 1;
	int LED_ENABLE = 1;
	int HX_ENABLE = 0;

	// EPOCHS
	int epoch_LED = 0;

	// DELAYS
	int delay_LED = 10;
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (LED_ENABLE) {
			if (HAL_GetTick() > (epoch_LED + delay_LED)) {
				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
				epoch_LED = HAL_GetTick();
			}
		}

		if (TX_ENABLE) {
			hmmmm = ((hmmmm + 1) % 2);
			for (i = 0; i < (B_SIZE - 2); ++i) {
				buffer[i] = 'x' + hmmmm;
			}
			buffer[B_SIZE - 2] = '\n';
			buffer[B_SIZE - 1] = '\r';

			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart1, buffer, len);
			HAL_UART_Transmit_DMA(&huart2, buffer, len);
		}

		if (RX_ENABLE_PASS) {
			///////////////////////////// RX
			__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);

			memset(RX_buffer1, 0, len);
			HAL_UART_Receive_DMA(&huart1, RX_buffer1, len);
			RX_buffer1[B_SIZE - 2] = '\n';
			RX_buffer1[B_SIZE - 1] = '\r';
			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart2, RX_buffer1, len);
		}

		if (RX_ENABLE_PASS) {
			///////////////////////////// RX
			__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);

			memset(RX_buffer2, 0, len);
			HAL_UART_Receive_DMA(&huart2, RX_buffer2, len);
			RX_buffer2[B_SIZE - 2] = '\n';
			RX_buffer2[B_SIZE - 1] = '\r';
			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart1, RX_buffer2, len);

		}

		if (ADC_ENABLE) {
			Update_ADC_Values();
			memset(ADC_buffer, 0, B_SIZE);
			sprintf(ADC_buffer, "|C|%lu|D|%lu|E|%lu|F|%lu|\n\r", ( unsigned long )ADC_C_Value,
					( unsigned long )ADC_D_Value, ( unsigned long )ADC_E_Value, ( unsigned long )ADC_F_Value);
			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart1, ADC_buffer, len);
			HAL_UART_Transmit_DMA(&huart2, ADC_buffer, len);
		}

		if (HX_ENABLE) {
			read_HX711();
			memset(ADC_buffer, 0, B_SIZE);
			sprintf(ADC_buffer, "|A|%lu|B|%lu|\n\r", ( unsigned long )ADC_A_Value, ( unsigned long )ADC_B_Value);
			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart1, ADC_buffer, len);
			HAL_UART_Transmit_DMA(&huart2, ADC_buffer, len);
		}

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* ADC1_2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
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
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* ADC2 init function */
static void MX_ADC2_Init(void) {

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
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig_C.Channel = ADC_CHANNEL_1;
	sConfig_C.Rank = ADC_REGULAR_RANK_1;
	sConfig_C.SingleDiff = ADC_SINGLE_ENDED;
	sConfig_C.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig_C.OffsetNumber = ADC_OFFSET_NONE;
	sConfig_C.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_C) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfig_D.Channel = ADC_CHANNEL_2;
	sConfig_D.Rank = ADC_REGULAR_RANK_1;
	sConfig_D.SingleDiff = ADC_SINGLE_ENDED;
	sConfig_D.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig_D.OffsetNumber = ADC_OFFSET_NONE;
	sConfig_D.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_D) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfig_E.Channel = ADC_CHANNEL_3;
	sConfig_E.Rank = ADC_REGULAR_RANK_1;
	sConfig_E.SingleDiff = ADC_SINGLE_ENDED;
	sConfig_E.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig_E.OffsetNumber = ADC_OFFSET_NONE;
	sConfig_E.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_E) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfig_F.Channel = ADC_CHANNEL_4;
	sConfig_F.Rank = ADC_REGULAR_RANK_1;
	sConfig_F.SingleDiff = ADC_SINGLE_ENDED;
	sConfig_F.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig_F.OffsetNumber = ADC_OFFSET_NONE;
	sConfig_F.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_F) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, CLK_A_Pin | LD3_Pin | CLK_B_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : CLK_A_Pin LD3_Pin CLK_B_Pin */
	GPIO_InitStruct.Pin = CLK_A_Pin | LD3_Pin | CLK_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : DAT_B_Pin DAT_A_Pin */
	GPIO_InitStruct.Pin = DAT_B_Pin | DAT_A_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/**/
	HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_PB6_FMP);

}

/* USER CODE BEGIN 4 */
void Update_ADC_Values(void) {
	/* Read ADC_C
	 * ADC C = PA4 = A3 = ADC2 Channel 1 */
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_C) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ADC_Start(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ADC_PollForConversion(&hadc2, 50) == HAL_OK) {
		ADC_C_Value = HAL_ADC_GetValue(&hadc2);
	}
	if (HAL_ADC_Stop(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/* Read ADC_D
	 * ADC D = PA5 = A4 = ADC2 Channel 2 */
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_D) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ADC_Start(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ADC_PollForConversion(&hadc2, 50) == HAL_OK) {
		ADC_D_Value = HAL_ADC_GetValue(&hadc2);
	}
	if (HAL_ADC_Stop(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/* Read ADC_E
	 * ADC E = PA6 = A5 = ADC2 Channel 3 */
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_E) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ADC_Start(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ADC_PollForConversion(&hadc2, 50) == HAL_OK) {
		ADC_E_Value = HAL_ADC_GetValue(&hadc2);
	}
	if (HAL_ADC_Stop(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/* Read ADC_F
	 * ADC F = PA7 = A6 = ADC2 Channel 4 */
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_F) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ADC_Start(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ADC_PollForConversion(&hadc2, 50) == HAL_OK) {
		ADC_F_Value = HAL_ADC_GetValue(&hadc2);
	}
	if (HAL_ADC_Stop(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	return;
}

int emptyRX(uint8_t RX_buffer[]) {
	int i = 0;
	for (i = 0; i < (B_SIZE - 2); i++) {
		if (RX_buffer[i] != 0) {
			return 1;
		}
	}
	return 0;

}

int strip_str(uint8_t RX_buffer[], uint8_t TX_buffer[]) {
	int i = 0;
	int index = 0;
	for (i = 0; i < (B_SIZE); i++) {
		if (RX_buffer[i] != 0) {
			TX_buffer[index] = RX_buffer[i];
			index++;
		}
	}
	return index;
}

int isTransmitting(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2) {
	return ((huart1->gState != HAL_UART_STATE_READY)
			|| (huart2->gState != HAL_UART_STATE_READY)) ? 1 : 0;
}

void read_HX711(void) {
	/* Adapted from Arduino Script "ShiftIn()" */
// Count should always be 24
// order is MSB for HX711
	Count = 0;
	LED3_ON;
	while (DAT_A_READ) {
		;
	}

	for (uint8_t i = 0; i < 24; i++) {
		CLK_A_SET;
		Count = Count << 1;
		DAT_A_READ ? Count++ : 0; // if High
		CLK_A_RESET;
	}

	for (i = 0; i < 3; i++) {
		CLK_A_SET;
		CLK_A_RESET;
	}

	ADC_A_Value = Count ^ 0x800000;
	CLK_A_RESET;
	LED3_OFF;
	return;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_Delay(trans_delay * 50);
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
