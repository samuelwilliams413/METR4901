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

CBUFF* circ_buff_init(void) {
	CBUFF* cbuff = (struct CBUFF*) malloc(sizeof(struct CBUFF)*1);
	cbuff->h = 0;
	cbuff->i = 0;
	cbuff->b = (uint8_t*) malloc(sizeof(uint8_t)*CB_SIZE);
	memset(cbuff->b,0,CB_SIZE);
	return cbuff;
}

uint8_t get(struct CBUFF* cbuff) {
	out = cbuff->b[cbuff->h];
	cbuff->b[cbuff->h] = 0;
	if (cbuff->b[cbuff->h] != cbuff->b[cbuff->i]) {
		cbuff->h = (((cbuff->h) + 1) % CB_SIZE);
	}
	return out;
}

void put(struct CBUFF* cbuff, uint8_t e) {
	cbuff->b[cbuff->i] = e;
	cbuff->i = (((cbuff->i) + 1) % CB_SIZE);
	return;
}

void pour(struct CBUFF* cbuff, uint8_t buff[]) {
	int i;
	for (i = 0 ;i < B_SIZE; i++) {
		put(cbuff, buff[i]);
	}
	return;
}

/*****************************END OF FILE****/
