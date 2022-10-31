#ifndef COMMON_H
#define COMMON_H
#include "stdint.h"

typedef signed char 	sint8_t;
typedef signed short int 	sint16_t;
typedef signed int 	sint32_t;

typedef union {
	
	uint8_t TX_buf[16];
	
	#pragma pack(1)
	struct {
		uint8_t 			START;		/* 1 byte start frame +2*/ 
		uint32_t  		FRAME_CNT;		/* 4 byte counting frame  */
		sint8_t				TEMP;		/* 1 byte temperature value +1 */ 
		uint16_t		  PRES;		/* 2 byte pressure value */
		sint16_t			VEL;		/* 2 byte velocity */ 
		uint8_t 			BAT ;     /* 1 byte battrety value +1 */ 
		uint32_t			RFU ;      /* 4 byte reserved  for future use */
		uint8_t				END;			/* 1 byte end of frame +3 */  
	} __attribute__((__packed__)) TX_buf_b;
	#pragma pack()

}	VAL_FRAME_t;

void swapBytes(void* number, int bytes_num);
#endif /* COMMON_H */