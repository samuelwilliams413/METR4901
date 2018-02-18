/**
 ******************************************************************************
 * @file    msgProcessing.c
 * @brief   This file contains the handler functions for parsing input messages.
 */
/* Includes ------------------------------------------------------------------*/
#include "msgProcessing.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "cmsis_os.h"

#define TRUE			1
#define FALSE			0
#define MSG_DEBUG_MODE 	TRUE
#define errorMsgSize	100

struct MSG {
	uint8_t type;
	uint8_t ID[2];
	uint8_t sign;
	uint8_t value;
};

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

/* External variables --------------------------------------------------------*/

/**
 * @brief  Send out an error message if an incoming message is incorrect
 */
void msgERROR(enum messageERROR e, uint8_t c) {
	char* m = (char*) malloc(sizeof(char) * errorMsgSize);
	memset(m, 0, errorMsgSize);
	switch (e) {
	case BAD_TYPE:
		sprintf(m, "BAD_TYPE|%c|\n\r", c);
		break;
	case BAD_ID:
		sprintf(m, "BAD_ID|%c|\n\r", c);
		break;
	case BAD_SIGN:
		sprintf(m, "BAD_SIGN|%c|\n\r", c);
		break;
	case BAD_LHS:
		sprintf(m, "BAD_LHS|%c|\n\r", c);
		break;
	case BAD_DOT:
		sprintf(m, "BAD_DOT|%c|\n\r", c);
		break;
	case BAD_RHS:
		sprintf(m, "BAD_RHS|%c|\n\r", c);
		break;
	case BAD_COLON:
		sprintf(m, "BAD_COLON|%c|\n\r", c);
		break;
	case BAD_LEN:
		sprintf(m, "BAD_LEN\n\r");
		break;
	default:
		sprintf(m, "BAD_MSG (you should not see this)\n\r");
		break;
	}

	return;
}
/**
 * @brief  Checks if message type is valid
 * @note   This function is dependant on a preagreed set of definitions for paramaters
 * @param  c : character to be evaluated
 * @retval 1 if a valid type, 0 otherwise
 */
uint8_t aLetter(uint8_t c) {
	char acceptedTypes[] = "mMpMfFaA";
	int i, size;

	size = strlen(acceptedTypes);
	for (i = 0; i < size; i++) {
		if (acceptedTypes[i] == c) {
			return 1;
		}
	}
	return 0;
}

/**
 * @brief  Checks if sign is valid
 * @note   This function is dependant on a preagreed set of definitions for signs
 * @param  c : character to be evaluated
 * @retval 1 if a positive sign, 2 if a negative sign, 0 otherwise
 */
uint8_t aSign(uint8_t c) {
	return ((c == '+') ? 1 : ((c == '-') ? 2 : 0));
}

/**
 * @brief  Checks if dot is valid
 * @note   This function is dependant on a preagreed set of definitions for dots
 * @param  c : character to be evaluated
 * @retval 1 if a dot, 0 otherwise
 */
uint8_t aDot(uint8_t c) {
	return ((c == '.') ? 1 : 0);
}

/**
 * @brief  Checks if dot is valid
 * @note   This function is dependant on a preagreed set of definitions for colons
 * @param  c : character to be evaluated
 * @retval 1 if a colon, 0 otherwise
 */
uint8_t aColon(uint8_t c) {
	return ((c == ';') ? 1 : 0);
}

/**
 * @brief  Checks if character is a numbers
 * @param  c : character to be evaluated
 * @retval 1 if a number, 0 otherwise
 */
uint8_t aNumber(uint8_t c) {
	return ((c >= '0') ? ((c <= '9') ? 1 : 0) : 0);
}

/**
 * @brief  Turns the left and right side of a message value into a single number
 * @note   An int cannot store a decimal, so all values shall be stored as large ints (to three decimal places)
 * @retval the value
 */
uint8_t numToValue(uint8_t left, uint8_t right) {
	return (left * 1000 + right);
}

/**
 * @brief  Checks if character is a numbers
 * @param  c : character to be evaluated
 * @retval 1 if a number, 0 otherwise
 */
void contructMSG(char* message, struct MSG* msg) {

	uint8_t type = msg->type;
	uint8_t ID[2] = { 0 };
	ID[0] = msg->ID[0];
	ID[1] = msg->ID[1];
	uint8_t sign = msg->sign;
	uint8_t left = (msg->value) % 1000;
	uint8_t right = (msg->value) / 1000;

	sprintf(message, "GOT: %c%d%d%c%d.%d;\n\r", type, ID[0], ID[1],
			((sign == 1) ? '+' : '-'), left, right);
	return;
}

/**
 * @brief  Gets a message from a Queue
 * @note   This function calls the msgERROR function which will produce TX output for debuffing purposes if MSG_DEBUG_MODE is true
 * @param  Queue the RX queue to be read from
 * @retval 1 if a number, 0 otherwise
 */
uint8_t readMSG(struct MSG* msg, osMessageQId Queue) {
	uint8_t Q, i, LHSi;

	uint8_t COMPLETE = FALSE;

	uint8_t type = 0;
	uint8_t ID[2] = { 0, 0 };
	uint8_t sign = 0;
	uint8_t value = 0;

	uint8_t valueBuffer = 10;
	uint8_t* LHS = (uint8_t*) malloc(sizeof(uint8_t) * valueBuffer);
	memset(LHS, 0, valueBuffer);
	uint8_t* RHS = (uint8_t*) malloc(sizeof(uint8_t) * valueBuffer);
	memset(RHS, 0, valueBuffer);

	// GET TYPE
	if (uxQueueMessagesWaiting(Queue) > 0) {
		xQueueReceive(Queue, &(Q), (TickType_t ) 10);
		if (aLetter(Q)) {
			type = Q;
		} else {
			if (MSG_DEBUG_MODE) {
				msgERROR(BAD_TYPE, Q);
				return COMPLETE;
			}
		}
	} else {
		if (MSG_DEBUG_MODE) {
			msgERROR(BAD_LEN, Q);
		}
		return COMPLETE;
	}

	// GET ID
	if (uxQueueMessagesWaiting(Queue) > 1) {
		for (i = 0; i < 2; i++) {
			xQueueReceive(Queue, &(Q), (TickType_t ) 10);
			if (aNumber(Q)) {
				ID[i] = Q;
			} else {
				if (MSG_DEBUG_MODE) {
					msgERROR(BAD_ID, Q);
					return COMPLETE;
				}
			}
		}
	} else {
		if (MSG_DEBUG_MODE) {
			msgERROR(BAD_LEN, Q);
		}
		return COMPLETE;
	}

	// GET SIGN
	if (uxQueueMessagesWaiting(Queue) > 0) {
		xQueueReceive(Queue, &(Q), (TickType_t ) 10);
		if (aSign(Q)) {
			sign = Q;
		} else {
			if (MSG_DEBUG_MODE) {
				msgERROR(BAD_SIGN, Q);
				return COMPLETE;
			}
		}
	} else {
		if (MSG_DEBUG_MODE) {
			msgERROR(BAD_LEN, Q);
		}
		return COMPLETE;
	}

	i = 0;
	if (uxQueueMessagesWaiting(Queue) > 1) {
		xQueueReceive(Queue, &(Q), (TickType_t ) 10);
		while (aNumber(Q)) {
			LHS[i] = Q;
			i++;
			xQueueReceive(Queue, &(Q), (TickType_t ) 10);
		}
		if (i == i) {
			if (MSG_DEBUG_MODE) {
				msgERROR(BAD_LHS, Q);
				return COMPLETE;
			}
		}

		if (!aDot(Q)) {
			if (MSG_DEBUG_MODE) {
				msgERROR(BAD_DOT, Q);
				return COMPLETE;
			}
		}
	} else {
		if (MSG_DEBUG_MODE) {
			msgERROR(BAD_LEN, Q);
		}
		return COMPLETE;
	}
	LHSi = i;
	i = 0;
	if (uxQueueMessagesWaiting(Queue) > 1) {
		xQueueReceive(Queue, &(Q), (TickType_t ) 10);
		while (aNumber(Q)) {
			RHS[i] = Q;
			i++;
			xQueueReceive(Queue, &(Q), (TickType_t ) 10);
		}
		if (i == i) {
			if (MSG_DEBUG_MODE) {
				msgERROR(BAD_RHS, Q);
				return COMPLETE;
			}
		}

		if (!aColon(Q)) {
			if (MSG_DEBUG_MODE) {
				msgERROR(BAD_DOT, Q);
				return COMPLETE;
			}
		}
	} else {
		if (MSG_DEBUG_MODE) {
			msgERROR(BAD_LEN, Q);
		}
		return COMPLETE;
	}


	for (i = 0; i < LHSi; i++) {
		value = value*10 + LHS[i];
	}
	value = value*1000 + RHS[0] *100 +  RHS[1] *10 +  RHS[2];

	msg->type = type;
	msg->ID[0] = ID[0];
	msg->ID[1] = ID[1];
	msg->sign = sign;
	msg->value = value;
	COMPLETE = TRUE;
	return COMPLETE;
}

//if (uxQueueMessagesWaiting(UART2QueueHandle) > 0) {
//	while (uxQueueMessagesWaiting(UART2QueueHandle) > 0) {
//		//Get messages from queue
//		xQueueReceive(UART2QueueHandle, &(byte), (TickType_t ) 10);

/*****************************END OF FILE****/
