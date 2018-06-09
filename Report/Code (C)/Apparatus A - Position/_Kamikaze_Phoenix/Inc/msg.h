/**
  ******************************************************************************
  * @file    msg.h
  * @brief   This file contains the handler files for parsing input messages
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __msg_H
#define __msg_H

#include "main.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "stm32f3xx_hal.h"

/* Exported types ------------------------------------------------------------*/
#define BAD_TYPE 	0
#define BAD_ID 		1
#define BAD_SIGN 	2
#define BAD_LHS 	3
#define BAD_DOT 	4
#define BAD_RHS 	5
#define BAD_COLON 	6
#define BAD_LEN 	7

struct MSG {
	uint8_t type;
	uint8_t ID;
	uint8_t sign;
	uint32_t value;
	uint8_t complete;
};

typedef struct MSG MSG;

struct CBUFF;

/* Exported constants --------------------------------------------------------*/
extern char acceptedTypes[];

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint8_t aLetter(uint8_t);
uint8_t aSign(uint8_t);
uint8_t aDot(uint8_t);
uint8_t aColon(uint8_t);
uint8_t aNumber(uint8_t);
uint8_t numToValue(uint8_t, uint8_t);
void contructMSG(char*, struct MSG*, int);
void contruct_X_msg(char, struct PARAMETERS*,struct MSG*, char*);
void readMSG(struct MSG* , uint8_t* , uint8_t*, uint8_t*);

#define MSG_DEBUG_MODE 	TRUE


#endif /* __msg_H */

/*****************************END OF FILE****/
