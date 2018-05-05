/**
 ******************************************************************************
 * @file    control.c
 * @brief   This file contains the handler functions for parsing input messages.
 */
/* Includes ------------------------------------------------------------------*/
#include "control.h"
#include "deviceParameters.h"
#include "stdlib.h"
#include "stdio.h"

/* External variables --------------------------------------------------------*/
enum LOC {
	LF = 0, LS = 1, LT = 2, RF = 3, RS = 4, RT = 5
};

void control(void) {
	uint32_t controlValues[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
	uint32_t Kp, Ki, Kd;
	uint32_t control_var = 0;
	switch (getEGO()) {
	case RF:
		Kp = controlValues[1];
		Ki = controlValues[2];
		Kd = controlValues[3];
		break;
	case LF:
		Kp = controlValues[1];
		Ki = controlValues[2];
		Kd = controlValues[3];
		break;
	case LS:
		Kp = controlValues[4];
		Ki = controlValues[5];
		Kd = controlValues[6];
		break;
	case RS:
		Kp = controlValues[4];
		Ki = controlValues[5];
		Kd = controlValues[6];
		break;
	case LT:
		Kp = controlValues[7];
		Ki = controlValues[8];
		Kd = controlValues[9];
		break;
	case RT:
		Kp = controlValues[7];
		Ki = controlValues[8];
		Kd = controlValues[9];
		break;
	}

	uint32_t p, p_last, p_target, e, e_last, Ep, Ei, Ed;

	MAA* q = (MAA*) malloc(sizeof(MAA));
	q->len = 5;
	q->buffer = (uint32_t*) malloc(sizeof(uint32_t) * q->len);

	p = get_p();
	p_target = get_p_target();

	e = p_target - p;
	maaPush(q, e);

	Ep = e;
	Ei = get_integral(q);
	Ed = e - e_last;

	control_var = Kp * Ep + Ki * Ei + Kd * Ed;

	e_last = e;
	p_last = p;
	return;
}

uint32_t get_integral(MAA* q) {
	uint32_t sum = 0;
	int i = 0;

	for (i = 0; i < q->len; i++) {
		sum += q->buffer[i];
	}
	return sum;
}

void maaPush(MAA* q, uint8_t e) {
	int i = 1 + q->head;
	i = (i > q->len) ? 0 : i;
	q->buffer[q->head] = e;
	q->head = i;
	return;
}

/*****************************END OF FILE****/
