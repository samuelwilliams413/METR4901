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

/* Exported types ------------------------------------------------------------*/
enum messageERROR {
	BAD_TYPE = 0,
	BAD_ID = 1,
	BAD_SIGN = 2,
	BAD_LHS = 3,
	BAD_DOT = 4,
	BAD_RHS = 5,
	BAD_COLON = 6,
	BAD_LEN = 7
};

struct MSG {
	uint8_t type;
	uint8_t ID;
	uint8_t sign;
	uint32_t value;
};


/* Exported constants --------------------------------------------------------*/
extern char acceptedTypes[];

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void msgERROR(enum messageERROR, uint8_t);
uint8_t aLetter(uint8_t);
uint8_t aSign(uint8_t);
uint8_t aDot(uint8_t);
uint8_t aColon(uint8_t);
uint8_t aNumber(uint8_t);
uint8_t numToValue(uint8_t, uint8_t);
void contructMSG(char*, struct MSG*, int);

#define MSG_DEBUG_MODE 	FALSE
#define errorMsgSize	100

#endif /* __msg_H */

/*****************************END OF FILE****/
