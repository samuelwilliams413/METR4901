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

int getEGO(void);

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

/*****************************END OF FILE****/
