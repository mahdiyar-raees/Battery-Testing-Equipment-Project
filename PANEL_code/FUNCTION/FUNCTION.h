#ifndef FUNCTION
	#define FUNCTION
	#include <SEGMENT.h>
	#include <TYPEDEF.h>
	#include <SEMIFUNCTION.h>
	#include <KEY.h>
	// DEFINE GLOBAL VARIABLE
//	LCD_t lcd;
	extern 			uint8_t 								SEG_menue_level;
	extern 			FLAG_t									flag;
	extern			UART_HandleTypeDef 			huart1;
	extern 			TIM_HandleTypeDef 			htim2;
	extern			TIM_HandleTypeDef 			htim4;
	extern			TIM_HandleTypeDef 			htim5;
	extern			TIM_HandleTypeDef 			htim6;
	extern			process_excute_t				process;
	extern			panel_send_buff_t 	   	panel_send;
	extern			float    								real_num[13];	
	extern			uint8_t									tx_buffer[12];
	extern			uint8_t									exist_cell_num ;
	extern			uint8_t								  real_count; 
	extern			uint16_t 								timer4_counter;
	extern			uint16_t 								timer6_counter;
	extern			uint16_t 								timer5_counter;	
	extern			uint8_t								  panel_command_counter;
	extern			uint8_t									choose_operation_counter;
	extern			uint16_t 								timer6_wait;
	extern			uint8_t		 							reset_counter;
	extern			uint8_t								send_send_counter;
	//DEFINE FUNCTION
						uint8_t 	Read_key_scan1		(void);
	extern		void 			delayUS(uint32_t us);
						
						void 			reset_state(void);	
						
						void		  SEGMENT_MASTER(uint8_t key);
						
						void			SEGMENT_MENU(uint8_t key);
						
						float 		BCD_to_float(void);
						
						uint16_t 			float_to_BCD( float number);
						
						void 			SEG_NUMBER_MANAGER(uint8_t LCD_KEYS );
					
						void 			error_manegment(uint8_t error_type);
						
						void 			show_num(uint8_t cell_num);
	
#endif
