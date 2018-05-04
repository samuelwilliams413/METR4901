/**
  ******************************************************************************
  * @file    msgProcessing.h
  * @brief   This file contains the handler files for parsing input messages
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __control_H
#define __control_H

#include "main.h"
/* Exported types ------------------------------------------------------------*/
typedef struct {
	uint32_t*  buffer;
	int head;
	int len;
} MAA; // Moving average array
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void control(void);
uint32_t get_integral(MAA*);
void maaPush(MAA*, uint8_t);



#endif /* __msgProcessing_H */

/*****************************END OF FILE****/
