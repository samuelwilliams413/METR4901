/**
 ******************************************************************************
 * @file    msg.c
 * @brief   This file contains the handler functions for parsing input messages.
 */
/* Includes ------------------------------------------------------------------*/
#include "msg.h"

/* External variables --------------------------------------------------------*/
char acceptedTypes[] = "pPtT";

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
	sprintf(message, "\0%c%d%c%lu.%lu;\0", msg->type, msg->ID,
			msg->sign, (msg->value / 1000), (msg->value % 1000));
	return;
}

/**
 * @brief  Gets a message from a Queue
 * @note   This function calls the msgERROR function which will produce TX output for debuffing purposes if MSG_DEBUG_MODE is true
 * @param  Queue the RX queue to be read from
 * @retval 1 if a number, 0 otherwise
 */
void readMSG(struct MSG* msg, uint8_t* buff, uint8_t* x, uint8_t* x0) {
	uint8_t i, c, xi;
	xi = *x;
	msg->complete = 0;
	c = buff[*x];
	*x = ((*x + 1) % B_SIZE);
	if (!c) {
		return;
	}

	uint8_t type = 0;
	uint8_t ID[2] = { 0, 0 };
	uint8_t sign = 0;
	uint32_t value = 0;

	uint8_t valueBuffer = 10;
	char* LHS = (char*) malloc(sizeof(char) * valueBuffer);
	memset(LHS, 0, valueBuffer);
	char* RHS = (char*) malloc(sizeof(char) * valueBuffer);
	memset(RHS, 0, valueBuffer);
	RHS[0] = 0;
	RHS[1] = 0;
	RHS[2] = 0;

	char* m = (char*) malloc(sizeof(char) * errorMsgSize);

	/* GET TYPE */
	if (!aLetter(c)) {
		MSG_DEBUG_MODE ? msgERROR(BAD_TYPE, c) : 0;
		free(LHS);
		free(RHS);
		free(m);
		return;
	}
	type = c;

	/* GET ID */
	for (i = 0; i < 2; i++) {
		c = buff[*x];
		*x = ((*x + 1) % B_SIZE);
		if (aNumber(c)) {
			ID[i] = c;
		} else {
			MSG_DEBUG_MODE ? msgERROR(BAD_ID, c) : 0;
			free(LHS);
			free(RHS);
			free(m);
			return;
		}
	}
	if (ID[0] == 0 && ID[1] == 1) {
		MSG_DEBUG_MODE ? msgERROR(BAD_ID, c) : 0;
		free(LHS);
		free(RHS);
		free(m);
		return;
	}

	/* GET SIGN */
	c = buff[*x];

	*x = ((*x + 1) % B_SIZE);
	if (aSign(c)) {
		sign = c;
		if (MSG_DEBUG_MODE) {
			memset(m, 0, errorMsgSize);
			sprintf(m, "\n\r\t\t\t\t Sign: |%c|", sign);
			free(LHS);
			free(RHS);
			free(m);
			transmit(1, m);
			transmit(2, m);
		}
	} else {
		MSG_DEBUG_MODE ? msgERROR(BAD_SIGN, c) : 0;
		free(LHS);
		free(RHS);
		free(m);
		return;
	}

	/* GET LHS */
	i = 0;
	c = buff[*x];

	*x = ((*x + 1) % B_SIZE);
	while (aNumber(c)) {
		LHS[i] = c;
		i++;
		c = buff[*x];
		*x = ((*x + 1) % B_SIZE);
	}
	if (i == 0) {
		MSG_DEBUG_MODE ? msgERROR(BAD_LHS, c) : 0;
		free(LHS);
		free(RHS);
		free(m);
		return;
	}
	if (MSG_DEBUG_MODE) {
		memset(m, 0, errorMsgSize);
		sprintf(m, "\n\r\t\t\t\t Val: |%d|",  atoi(LHS));
		transmit(1, m);
		transmit(2, m);
	}

	/* GET DOT */
	if (!aDot(c)) {
		MSG_DEBUG_MODE ? msgERROR(BAD_DOT, c) : 0;
		free(LHS);
		free(RHS);
		free(m);
		return;
	}

	/* GET RHS */
	i = 0;
	c = buff[*x];
	*x = ((*x + 1) % B_SIZE);
	while (aNumber(c)) {
		RHS[i] = c;
		i++;
		c = buff[*x];
		*x = ((*x + 1) % B_SIZE);
	}
	if (i == 0) {
		MSG_DEBUG_MODE ? msgERROR(BAD_RHS, c) : 0;
		free(LHS);
		free(RHS);
		free(m);
		return;
	}
	if (MSG_DEBUG_MODE) {
		memset(m, 0, errorMsgSize);
		sprintf(m, "\n\r\t\t\t\t Val: |%d|", atoi(RHS));
		transmit(1, m);
		transmit(2, m);
	}

	/* GET DOT */
	if (!aColon(c)) {
		MSG_DEBUG_MODE ? msgERROR(BAD_DOT, c) : 0;
		free(LHS);
		free(RHS);
		free(m);
		return;
	}

	/* BUILD MSG */
	value = (uint32_t) ((1000 * atoi(LHS)) + atoi(RHS));
	if (MSG_DEBUG_MODE) {
		memset(m, 0, errorMsgSize);
		sprintf(m, "\n\r\t\t\t\t Value: |%lu|", value);
		transmit(1, m);
		transmit(2, m);
	}

	msg->type = type;
	msg->ID = atoi((char*) ID);
	msg->sign = sign;
	msg->value = (uint32_t) ((1000 * atoi(LHS)) + atoi(RHS));
	msg->complete = 1;

	*x0 = xi;
	free(LHS);
	free(RHS);
	free(m);
	return;
}

/*****************************END OF FILE****/
