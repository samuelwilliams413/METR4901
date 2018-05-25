/**
  ******************************************************************************
  * @file    msgProcessing.h
  * @brief   This file contains the handler files for parsing input messages
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __control_H
#define __control_H

#include "main.h"
/* Exported types ------------------------------------------------------------*/
 struct MAA {
	int*  buffer;
	int head;
	int len;
}; // Moving average array

typedef struct MAA MAA;


struct CBUFF;
struct MSG;
struct PARAMETERS;


/* Exported constants --------------------------------------------------------*/
// Expected Response time: 0.2s
#define Kp_T	4785
#define Ki_T	8665
#define Kd_T	658

#define Kp_S	3243
#define Ki_S	5885
#define Kd_S	447

#define Kp_F	150		//10
#define Ki_F	0
#define Kd_F	0


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void control(void);
int get_integral(MAA*);
void maaPush(MAA*, int);
void getPIDparameters(int*, int*, int*);
void update_control(struct PARAMETERS* par);
int maaAverage(MAA* q);

#endif /* __msgProcessing_H */

/*****************************END OF FILE****/
