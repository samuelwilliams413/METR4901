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

	switch (getEGO()) {
	case RF:
		*Kp = Kp_F;
		*Ki = Ki_F;
		*Kd = Kd_F;
		break;
	case LF:
		*Kp = Kp_F;
		*Ki = Ki_F;
		*Kd = Kd_F;
		break;
	case LS:
		*Kp = Kp_S;
		*Ki = Ki_S;
		*Kd = Kd_S;
		break;
	case RS:
		*Kp = Kp_S;
		*Ki = Ki_S;
		*Kd = Kd_S;
		break;
	case LT:
		*Kp = Kp_T;
		*Ki = Ki_T;
		*Kd = Kd_T;
		break;
	case RT:
		*Kp = Kp_T;
		*Ki = Ki_T;
		*Kd = Kd_T;
		break;
	}
	return;
}

void update_control(struct PARAMETERS* par) {
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
