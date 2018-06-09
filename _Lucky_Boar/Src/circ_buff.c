/**
 ******************************************************************************
 * @file    circ_buff.c
 */
/* Includes ------------------------------------------------------------------*/
#include "circ_buff.h"
#include "stdlib.h"
#include "stdio.h"

/* External variables --------------------------------------------------------*/
int out = 0;

/**
 * @brief	Init the circular buffer
 * @retval	cbuff : fully initialised circular buffer
 */
CBUFF* circ_buff_init(void) {
	CBUFF* cbuff = (struct CBUFF*) malloc(sizeof(struct CBUFF)*1);
	cbuff->h = 0;
	cbuff->i = 0;
	cbuff->b = (uint8_t*) malloc(sizeof(uint8_t)*CB_SIZE);
	memset(cbuff->b,0,CB_SIZE);
	return cbuff;
}

/**
 * @brief	Get a values from the circular buffer and then set that value as zero (removing it from the buffer)
 * @note
 * @param	cbuff : circular buffer
 * @retval	out : value stored at the head of the circular buffer
 */
uint8_t get(struct CBUFF* cbuff) {
	out = cbuff->b[cbuff->h];
	cbuff->b[cbuff->h] = 0;
	if (cbuff->b[cbuff->h] != cbuff->b[cbuff->i]) {
		cbuff->h = (((cbuff->h) + 1) % CB_SIZE);
	}
	return out;
}

/**
 * @brief	Add a value to the circular buffer
 * @note	This can override values if the buffer is full
 * @param	cbuff : circular buffer
 * @param	e : element to be added
 */
void put(struct CBUFF* cbuff, uint8_t e) {
	cbuff->b[cbuff->i] = e;
	cbuff->i = (((cbuff->i) + 1) % CB_SIZE);
	return;
}

/**
 * @brief	Pours an array of values into the circular buffer
 * @note	This allows for the putting of multiple values
 * @param	cbuff : circular buffer
 * @param	buff : buffer of values to be added to the circular buffer
 */
void pour(struct CBUFF* cbuff, uint8_t buff[]) {
	int i;
	for (i = 0 ;i < B_SIZE; i++) {
		put(cbuff, buff[i]);
	}
	return;
}

/*****************************END OF FILE****/
