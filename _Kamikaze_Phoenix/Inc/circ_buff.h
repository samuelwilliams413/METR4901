/**
  ******************************************************************************
  * @file    circ_buff.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __circ_buff_H
#define __circ_buff_H

#include "main.h"

/* Exported types ------------------------------------------------------------*/
struct CBUFF {
	uint8_t h; // HEAD
	uint8_t i; // INDEX
	uint8_t* b; // BUFFER
};

typedef struct CBUFF CBUFF;


/* Exported constants --------------------------------------------------------*/
#define CB_SIZE 	512
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
CBUFF* circ_buff_init(void);
uint8_t get(struct CBUFF*);
void put(struct CBUFF*, uint8_t);
void pour(struct CBUFF*, uint8_t []);


#endif /* __circ_buff_H */

/*****************************END OF FILE****/
