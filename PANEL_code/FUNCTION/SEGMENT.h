#ifndef SEGMENT
	#define SEGMENT
	#include "stdint.h"
	#include "stdio.h"
	#include "string.h"
	#include "stm32f1xx_hal.h"
//DEFINE SEGMENT  PORT
#define SEGMENT_PORT  GPIOC
extern 			TIM_HandleTypeDef 			htim2;
//DEFINE SEGMENT PIN

//Dfine number of segment
extern uint8_t last_ascii;
//define decoder PORT
#define DECODER_PORT  GPIOB
//define decoder PIN

//DEFINE FUNCTION

//define segment count
#define SEGMENT_COUNT   15

void 		delayUS(uint32_t us);
	
void 		segment_put(char *str , uint8_t position);
	
void 		segment_clear(void);
	
void 		segment_put_decmial(uint8_t position);

uint8_t next_position(uint8_t position);



#endif
