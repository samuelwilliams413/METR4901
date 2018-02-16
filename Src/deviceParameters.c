/**
  ******************************************************************************
  * @file    msgProcessing.c
  * @brief   This file contains the hadeviceParameters
  */
/* Includes ------------------------------------------------------------------*/
#include "deviceParameters.h"


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
	LF = 0, 
	LS = 1, 
	LT = 2, 
	RF = 4,
	RS = 5,
	RT = 6
};

/* This is the identity of the current device */
#define		EGO		LF

/**
* @brief This function shifts characters by one in a buffer.
*/

/*****************************END OF FILE****/
