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

/* External variables --------------------------------------------------------*/

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
 * @note   An int cannot store a decimal, so all values shall be
 * @param  c : character to be evaluated
 * @retval 1 if a colon, 0 otherwise
 */
uint8_t numToValue(uint8_t left, uint8_t right) {
	return (left * 1000 + right);
}

/**
 * @brief  Checks if character is a numbers
 * @param  c : character to be evaluated
 * @retval 1 if a number, 0 otherwise
 */
void contructMSG(char* msg, uint8_t type, uint8_t ID[2], uint8_t sign,
		uint8_t value) {
	uint8_t left = value % 1000;
	uint8_t right = value / 1000;
	sprintf(msg, "GOT: %c%d%d%c%d.%d;\n\r", type, ID[0], ID[1],
			((sign == 1) ? '+' : '-'), left, right);
	return;
}

/*****************************END OF FILE****/
