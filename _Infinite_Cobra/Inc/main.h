/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include "deviceParameters.h"
#include "circ_buff.h"
#include "control.h"
#include "msg.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ADC_A_Pin GPIO_PIN_0
#define ADC_A_GPIO_Port GPIOA
#define ADC_B_Pin GPIO_PIN_1
#define ADC_B_GPIO_Port GPIOA
#define ADC_C_Pin GPIO_PIN_4
#define ADC_C_GPIO_Port GPIOA
#define ADC_D_Pin GPIO_PIN_5
#define ADC_D_GPIO_Port GPIOA
#define ADC_E_Pin GPIO_PIN_6
#define ADC_E_GPIO_Port GPIOA
#define ADC_F_Pin GPIO_PIN_7
#define ADC_F_GPIO_Port GPIOA
#define CLK_A_Pin GPIO_PIN_0
#define CLK_A_GPIO_Port GPIOB
#define DAT_B_Pin GPIO_PIN_1
#define DAT_B_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define CLK_B_Pin GPIO_PIN_6
#define CLK_B_GPIO_Port GPIOB
#define DAT_A_Pin GPIO_PIN_7
#define DAT_A_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
 * @brief Uncomment the line below to expanse the "assert_param" macro in the
 *        HAL drivers code
 */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define TRUE			1
#define FALSE			0
#define true			TRUE
#define false			FALSE
#define B_SIZE			50
#define errorMsgSize	128

#define  ADC_ENABLE 	false
#define  TX_ENABLE 		false
#define  RX_ENABLE_PASS true
#define  LED_ENABLE 	true
#define  HX_ENABLE		false

// DELAYS
#define D_LED		 	100

void transmit(int, char*);
void msgERROR_init(void);
void msgERROR(int, uint8_t);
/* USER CODE END Private defines */

#ifdef __cplusplus
extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
