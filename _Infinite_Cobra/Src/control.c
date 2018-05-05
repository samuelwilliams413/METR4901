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

uint32_t e, Ep, Ei, Ed;

void getPIDparameters(uint32_t* Kp, uint32_t* Ki, uint32_t* Kd) {

	uint32_t controlValues[9] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };

	switch (getEGO()) {
	case RF:
		*Kp = controlValues[0];
		*Ki = controlValues[1];
		*Kd = controlValues[2];
		break;
	case LF:
		*Kp = controlValues[0];
		*Ki = controlValues[1];
		*Kd = controlValues[2];
		break;
	case LS:
		*Kp = controlValues[3];
		*Ki = controlValues[4];
		*Kd = controlValues[5];
		break;
	case RS:
		*Kp = controlValues[3];
		*Ki = controlValues[4];
		*Kd = controlValues[5];
		break;
	case LT:
		*Kp = controlValues[6];
		*Ki = controlValues[7];
		*Kd = controlValues[8];
		break;
	case RT:
		*Kp = controlValues[6];
		*Ki = controlValues[7];
		*Kd = controlValues[8];
		break;
	}
	return;
}

void updateControl(struct PARAMETERS* par) {
	e = get_p_target(par) - get_p(par);
	maaPush(par->q, e);

	Ep = e;
	Ei = get_integral(par->q);
	Ed = e - par->e_last;

	set_T_target(par, (par->Kp * Ep + par->Ki * Ei + par->Kd * Ed));
	par->e_last = e;
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
