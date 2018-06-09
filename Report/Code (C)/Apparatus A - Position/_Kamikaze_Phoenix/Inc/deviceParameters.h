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
#include "control.h"
#include "realSpace.h"
/* Exported types ------------------------------------------------------------*/

struct CBUFF;
struct MSG;
struct MAA;

struct PARAMETERS {
	int* p; 	// parameters
	uint8_t w; 		// width (number of parameters per unit)
	int l; 	// limb segment length
	int Kp; 	// limb segment length
	int Ki; 	// limb segment length
	int Kd; 	// limb segment length
	int e_last; 	// limb segment length
	struct MAA* q; 	// Moving average of tracked angle for integral component of PID
};

typedef struct PARAMETERS PARAMETERS;



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int getEGO(void);
int getID(uint8_t);
PARAMETERS* parameters_init(void);
void update_values(struct PARAMETERS*, int, int, int, int,
		int, int);
void update_value(struct PARAMETERS*, struct MSG*);
void update_state(struct PARAMETERS* );
int deg2rad(int);

/* Get Methods --------------------------------------------------------*/
int get_p(struct PARAMETERS*);
int get_p_target(struct PARAMETERS*);
int get_p_error(struct PARAMETERS*);
int get_T_target(struct PARAMETERS*);
/* Set Methods --------------------------------------------------------*/
void set_p(struct PARAMETERS*, int);
void set_p_target(struct PARAMETERS*, int);
void set_p_error(struct PARAMETERS*, int);
void set_T_target(struct PARAMETERS*, int);

#endif /* __deviceParameters_H */

/*****************************END OF FILE****/
