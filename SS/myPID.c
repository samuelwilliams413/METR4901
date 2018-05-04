/**
  ******************************************************************************
  * @file    myPID.h
  * @brief   This file contains the handler functions for parsing input messages.
  */
/* Includes ------------------------------------------------------------------*/
#include "myPID.h"
#include "stdlib.h"
#include "stdio.h"

typedef struct {
	unit8_t * const buffer,
	int head,
	const int len
} MAA; // Moving average array

void maaPush (MAA* q, uint8_t val) {
	int i = 1 + q->head;
	i = (i > q->len) ? 0 : i;
	q->buffer[q->head] = val;
	q->head = i;
	return
}

/*****************************END OF FILE****/
