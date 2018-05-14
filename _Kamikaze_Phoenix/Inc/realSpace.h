/**
  ******************************************************************************
  * @file    realSpace.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __realSpace_H
#define __realSpace_H

#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* System Values */
#define LENGTH_THIGH 	1000 // [m]
#define LENGTH_SHIN  	1000 // [m]
#define LENGTH_FOOT 	1000 // [m]
#define PI 				3.14159265359

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint32_t sig2mm (uint32_t);

#endif /* __realSpace_H */
/*****************************END OF FILE****/
