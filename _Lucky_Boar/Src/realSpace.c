/**
 ******************************************************************************
 * @file    realSpace.c
 */
/* Includes ------------------------------------------------------------------*/
#include "realSpace.h"
#include "stdlib.h"
#include "stdio.h"

/* External variables --------------------------------------------------------*/

/**
 * @brief  	Translates an ADC signal to a distance in mm
 * @note 	Impliment when actual distance measurement is relevant.
 * @param  sig : Distance signal [0-0496]
 * @retval mm : Distance [mm]
 */
uint32_t sig2mm (uint32_t sig) {
	uint32_t mm = 0;
	mm = sig*1;
	return mm;
}

/*****************************END OF FILE****/
