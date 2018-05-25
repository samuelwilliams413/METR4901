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

int e, Ep, Ei, Ed;

void getPIDparameters(int* Kp, int* Ki, int* Kd) {

	switch (getEGO()) {
	case RF: // DEMO
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
	int T;

	//e = get_p_target(par) - get_p(par);
	e = get_p(par);
	maaPush(par->q, e);

	Ep = e;
	Ei = get_integral(par->q);
	Ed = e - par->e_last;

	par->e_last = e;

	T = (par->Kp * Ep + par->Ki * Ei + par->Kd * Ed);
	T = T/100;
	//T = (T < 0) ? 0 : T;

	set_T_target(par, T);
	return;
}

int get_integral(MAA* q) {
	int sum = 0;
	int i = 0;

	for (i = 0; i < q->len; i++) {
		sum += q->buffer[i];
	}
	return sum;
}

void maaPush(MAA* q, int e) {
	int i = 1 + q->head;
	i = (i > q->len) ? 0 : i;
	q->buffer[i] = e;
	q->head = i;
	return;
}


/*****************************END OF FILE****/
