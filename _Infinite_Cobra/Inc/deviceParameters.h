/**
 ******************************************************************************
 * @file    msgProcessing.h
 * @brief   This file contains the deviceParameters
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __deviceParameters_H
#define __deviceParameters_H

/* Parameter indices */
/* Load Cell Parameters */
#define LA		0
#define LB		1
/* Position Parameters */
#define PC		2
#define PD		3
#define PE		4
#define PF		5
/* Torque Parameters */
#define DT		6
#define YT		7
#define ET		8
/* Angle Parameters */
#define DA		9
#define YA		10
#define EA		11

#include "main.h"
#include "math.h"
#include "realSpace.h"
/* Exported types ------------------------------------------------------------*/
struct PARAMETERS {
	uint32_t* p; // parameters
	uint8_t w; // width (number of parameters per unit)
	uint32_t l; // limb segment length
};

typedef struct PARAMETERS PARAMETERS;

struct CBUFF;
struct MSG;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int getEGO(void);
int getID(uint8_t);
PARAMETERS* parameters_init(void);
void update_values(struct PARAMETERS*, uint32_t, uint32_t, uint32_t, uint32_t,
		uint32_t, uint32_t);
void update_value(struct PARAMETERS*, struct MSG*);
void update_state(struct PARAMETERS* );
double deg2rad(double);

#endif /* __deviceParameters_H */

/*****************************END OF FILE****/
