/**
  ******************************************************************************
  * @file    msg.c
  * @brief   This file contains the handler functions for parsing input messages.
  */
/* Includes ------------------------------------------------------------------*/
#include "msg.h"


/* External variables --------------------------------------------------------*/
 char acceptedTypes[] = "mMpMfFaA";

/**
 * @brief  Send out an error message if an incoming message is incorrect
 */
void msgERROR(enum messageERROR e, uint8_t c) {
	char* m = (char*) malloc(sizeof(char) * errorMsgSize);
	memset(m, 0, errorMsgSize);

	switch (e) {
	case BAD_TYPE:
		sprintf(m, "\n\rBAD_TYPE|%c|\n\r", c);
		break;
	case BAD_ID:
		sprintf(m, "\n\rBAD_ID|%c|\n\r", c);
		break;
	case BAD_SIGN:
		sprintf(m, "\n\rBAD_SIGN|%c|\n\r", c);
		break;
	case BAD_LHS:
		sprintf(m, "\n\rBAD_LHS|%c|\n\r", c);
		break;
	case BAD_DOT:
		sprintf(m, "\n\rBAD_DOT|%c|\n\r", c);
		break;
	case BAD_RHS:
		sprintf(m, "\n\rBAD_RHS|%c|\n\r", c);
		break;
	case BAD_COLON:
		sprintf(m, "\n\rBAD_COLON|%c|\n\r", c);
		break;
	case BAD_LEN:
		sprintf(m, "\n\rBAD_LEN\n\r");
		break;
	default:
		sprintf(m, "\n\rBAD_MSG (you should not see this)\n\r");
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
void contructMSG(char* message, struct MSG* msg, int size) {
	memset(message, 0, size);
	sprintf(message, "\n\r Received: %c%d%c%lu.%lu;\n\r", msg->type, msg->ID,
			msg->sign, (msg->value / 1000), (msg->value % 1000));

	return;
}
/*****************************END OF FILE****/
