/**
 ******************************************************************************
 * @file    deviceParameters.c
 * @brief   This file contains the deviceParameters
 */
/* Includes ------------------------------------------------------------------*/
#include "deviceParameters.h"
#include "stdlib.h"
#include "stdio.h"

/* External variables --------------------------------------------------------*/

/**
 * Device Location
 * 1 - Left Foot
 * 2 - Left Shin
 * 3 - Left Thigh
 * 4 - Right Foot
 * 5 - Right Shin
 * 6 - Right Thigh
 */
enum LOC {
	LF = 0, LS = 1, LT = 2, RF = 3, RS = 4, RT = 5
};

/* This is the identity of the current device */
#define		EGO		RF

/**
 * @brief  Returns a preassigned ID for each component of the exoskeleton
 * @note   This function is dependant on a preagreed set of definitions for each component
 * @param  s : sensor number (must not be negative)
 * @retval ID if found, 0 otherwise
 */
int getID(uint8_t s) {

	if (s < 4) { // Proximity sensors
		return (getEGO() * 4 + s);

	} else { // Force sensors
		return (getEGO() * 2 + (s - 4) + 24);
	}
}

/**
 * @brief  Returns the ego of the current device (this is who the device thinks it is)
 */
int getEGO(void) {
	return (int) EGO;
}

PARAMETERS* parameters_init(void) {
	int h, w, i;
	h = 6; // six units to the suit
	w = 11; // ten types of points stored
	/*
	 * 00: ADC_A
	 * 01: ADC_B
	 * 02: ADC_C
	 * 03: ADC_D
	 * 04: ADC_E
	 * 05: ADC_F
	 *
	 * 06: R(TORQUE) // Desire Torque
	 * 07: Y(TORQUE) // Current Torque
	 * 08: E(TORQUE) // Error Torque
	 *
	 * 09: R(ANGLE) // Desire Angle
	 * 10: Y(ANGLE) // Current Angle
	 * 11: E(ANGLE) // Error Angle
	 *
	 */
	PARAMETERS* par = (struct PARAMETERS*) malloc(
			sizeof(struct PARAMETERS) * 1);
	par->p = (int*) malloc(sizeof(int) * w * h);
	memset(par->p, 0, w * h);

	par->w = w;

	switch (getEGO()) {
	case LF:
		par->l = LENGTH_FOOT;
		break;
	case LS:
		par->l = LENGTH_SHIN;
		break;
	case LT:
		par->l = LENGTH_THIGH;
		break;
	case RF:
		par->l = LENGTH_FOOT;
		break;
	case RS:
		par->l = LENGTH_SHIN;
		break;
	case RT:
		par->l = LENGTH_THIGH;
		break;
	}

	getPIDparameters(&(par->Kp), &(par->Ki), &(par->Kd));
	par->q = (MAA*) malloc(sizeof(MAA));
	par->q->head = 0;
	par->q->len = 100;
	par->q->buffer = (int*) malloc(sizeof(int) * par->q->len);

	for (i = 0; i < par->q->len; i++) {
		par->q->buffer[i] = i;
	}
	return par;
}

void update_values(struct PARAMETERS* par, int A, int B, int C,
		int D, int E, int F) {

	par->p[(getEGO() * par->w) + LA] = A;
	par->p[(getEGO() * par->w) + LB] = B;

	par->p[(getEGO() * par->w) + PC] = C;
	par->p[(getEGO() * par->w) + PD] = D;
	par->p[(getEGO() * par->w) + PE] = E;
	par->p[(getEGO() * par->w) + PF] = F;

	return;
}

void update_value(struct PARAMETERS* par, struct MSG* msg) {
	int type = 0, sign;
	switch (msg->type) {
	case 't':
		type = YT;
		break;
	case 'T':
		type = YT;
		break;
	case 'a':
		type = YA;
		break;
	case 'A':
		type = YA;
		break;
	}

	sign = ((msg->sign == '+') ? 1 : ((msg->sign == '-') ? -1 : 0));

	par->p[msg->ID * par->w + type] = sign * msg->value;

	return;
}

int deg2rad(int angle) {
	int val;
	val = 180 / PI;
	return angle * val;
}

void update_state(struct PARAMETERS* par) {
	int average_f, average_r;
	int dx, angle;

	/* Get average signal strength in mm  */
	average_f = (sig2mm(par->p[(getEGO() * par->w) + PC])
			+ sig2mm(par->p[(getEGO() * par->w) + PD])) / 2;
	average_r = (sig2mm(par->p[(getEGO() * par->w) + PE])
			+ sig2mm(par->p[(getEGO() * par->w) + PF])) / 2;

	/* Get difference between sides */
	dx = average_f - average_r;

	/* Convert to angle */
	angle = atan(dx / (par->l));
	angle = deg2rad(angle);


	par->p[(getEGO() * par->w) + DA] = angle; // The desired angle is the pilots angle

	return;
}

/* Get Methods --------------------------------------------------------*/
int get_p(struct PARAMETERS* par) {
	return par->p[(getEGO() * par->w) + YA];
}

int get_p_target(struct PARAMETERS* par) {
	return par->p[(getEGO() * par->w) + DA];
}

int get_p_error(struct PARAMETERS* par) {
	return par->p[(getEGO() * par->w) + EA];
}

int get_T_target(struct PARAMETERS* par) {
	return par->p[(getEGO() * par->w) + DT];
}

/* Set Methods --------------------------------------------------------*/
void set_p(struct PARAMETERS* par, int p) {
	par->p[(getEGO() * par->w) + YA] = p;
}

void set_p_target(struct PARAMETERS* par, int p) {
	p = (p > 4096) ? 4096 : p;
	p = (p < 0) ? 0 : p;
	par->p[(getEGO() * par->w) + DA] = p;
}

void set_p_error(struct PARAMETERS* par, int p) {
	par->p[(getEGO() * par->w) + EA] = p;
}

void set_T_target(struct PARAMETERS* par, int p) {
	par->p[(getEGO() * par->w) + DT] = p;
}

/*****************************END OF FILE****/
