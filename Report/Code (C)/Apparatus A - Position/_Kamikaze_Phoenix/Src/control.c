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

/**
 * @brief 	get and set the PID paramaters for the system based on its EGO
 * @note	Feet share values, as do shins and thighs
 * @param	Kp : pointer to the Kp term of the control structure
 * @param	Ki : pointer to the Ki term of the control structure
 * @param	Kd : pointer to the Kd term of the control structure
 */
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

/**
 * @brief 	Updates the control value, this is the output of K(s) and is our command to the actuators to achieve error == 0
 * @param	par : system paramaters
 */
void update_control(struct PARAMETERS* par) {
	int T;

	//e = get_p_target(par) - get_p(par);
	e = get_p(par);

	maaPush(par->q, e);

	Ep = e;
	Ei = get_integral(par->q);
	Ei = Ei;
	Ed = e - par->e_last;

	par->e_last = e;

	T = (par->Kp * Ep + (par->Ki * Ei)/1000 + par->Kd * Ed);
	T = T/10000;
	set_T_target(par, T);
	return;
}

/**
 * @brief Get the integral error for the integral term of the control structures
 * @param	q : Queue from the moving average array
 * @retval	sum : here we define the integral is the sum of all the errors
 * (area under the curve, using Newton's approximation method)
 */
int get_integral(MAA* q) {
	int sum = 0;
	int i = 0;

	for (i = 0; i < q->len; i++) {
		sum += q->buffer[i];
	}
	return sum;
}

/**
 * @brief	The moving average array is actually a circular buffer,
 * this method updates the oldest value with a new value
 * (maintaining the most recent values in the queue)
 * @param	q : the moving average queue
 * @param	e : element to be pushed into the queue, this will become the new tail
 */
void maaPush(MAA* q, int e) {
	int i = 1 + q->head;
	i = (i > q->len) ? 0 : i;
	q->head = i;
	q->buffer[i] = e;
	return;
}


/*****************************END OF FILE****/
