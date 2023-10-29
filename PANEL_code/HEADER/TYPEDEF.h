#ifndef TYPEDEF
	#define TYPEDEF
	
	
	#include "stdint.h"
	// structure for LCD Vartiable
	
	typedef struct
		{
			uint8_t 			lcd_menue;
			uint8_t 			key;
			uint8_t				number_position;
			uint8_t      	first_menu_choose;
			uint8_t   		board_number[4];
			uint8_t 			counter;

		}LCD_t;

typedef		union
{
	uint8_t 						send_buffer[40];
	struct 
		{
			uint32_t			 	 total_time;
			float				   	 valueA;
			uint32_t				 timeA;
			float			  	   valueB;
			uint32_t			 	 timeB;
			float			 		 	 valueC;
			uint32_t				 timeC;
			float						 Duty;
			uint16_t				 Frequency;
			uint16_t				 pulse_count; // DET T
			uint16_t				 null_detect; // DET A
			uint8_t 				 test_mode;
			uint8_t 				 range;
		}detail;
}process_excute_t;

typedef enum		
{
	DCYC,
	FREQ,
	MODE,
	ACTIVE,
	TWID,
}TRAN_MENU_t;		

typedef  enum
{
	CURR,
	TLEV,
	SLWP,
	SLWN,
	SLW,
}CURRENT_MENU_t;


typedef enum
		{
				MODE_COUNT,
				ACTIVE_ON,
				MODE_PULLS,
				MODE_TOGG,
		}INPUT_UP_DOWN_t;
		
		
		typedef enum
		{
				FUNCTIO,
				TRANSACTION,
				CURRENT,
				RES,
		}UP_DOWN_t;
		// structure for LCD Vartiable
	typedef struct
		{
				uint8_t function;
				uint8_t step;
			
				struct 
					{
						uint8_t CURR;
						uint8_t TLEV;
						uint8_t SLW_N;
						uint8_t SLW_P;
						uint8_t RANGE; ///
					}CC;
				struct 
					{
						uint8_t RES;
						uint8_t TLEV;
						uint8_t SLW_N;
						uint8_t SLW_P;
						uint8_t RANGE; ///
					}CR;
				struct 
					{
						uint8_t MODE;
						uint8_t FREQ;
						uint8_t WIDE;
						uint8_t ACTIVE;

					}TRAN;
				

		}MODULE_t;
		

	


typedef union
	{
	uint32_t FLAG;
		struct
			{
				uint32_t key_bunce:1;
				uint32_t DOT_EXIS:1;
				uint32_t CLEAR_PANNEL:1;
				uint32_t DOT_FIRST:1;
				uint32_t CURRENT_active:1;
				uint32_t RES_active:1;
				uint32_t shift_insert:1;
				uint32_t input_ONOFF:1;
				uint32_t not_available_key:1;
				uint32_t command_or_data:1;
				uint32_t Enter_trigged:1;
				uint32_t panel_ask_for_data:1;
				uint32_t panel_first_ask_data:1;
				uint32_t initial_coff_ask:1;
				uint32_t first_cof_get:1;
				uint32_t first_one_cal_save:1;
				
			}BIT;
	}FLAG_t;
	
typedef union
{
	uint8_t get_buff[40];
			struct 
				{
					uint8_t		start_buff_1;
					uint8_t		start_buff_2;
					uint8_t		fault_or_command;
					uint8_t		type_or_ok;
					uint8_t		data[26];
					uint8_t		finish_test;
					uint8_t		error_type;
					uint8_t		reserve[6];
					uint8_t		stop_buff_1;
					uint8_t		stop_buff_2;					
				}detail;
}panel_get_buff_t;

typedef union
{
	uint16_t	U16;
	uint8_t   U8[2];	
}U16_to_U8;


typedef union
{
	float	    real_num;
	uint8_t   U8[4];	
}float_to_u8;

typedef union
{
	uint8_t send_buff[20];
			struct 
				{
					uint8_t		start_buff_1;
					uint8_t		start_buff_2;
					uint8_t		command;
					uint8_t		availabele_board[6];
					uint8_t		reserve[9];
					uint8_t		stop_buff_1;
					uint8_t		stop_buff_2;					
				}detail;
}panel_send_buff_t;


typedef union
{
	uint16_t big_data;
	uint8_t	 small_pack[2];
}U16_to_U8_t;

typedef union
{
uint8_t data;
struct 
{
uint8_t SW0:1;
uint8_t SW1:1;
uint8_t SW2:1;
uint8_t SW3:1;
uint8_t SW4:1;
uint8_t SW5:1;
uint8_t SW6:1;
uint8_t SW7:1;
}data_BIT;
}KEYPAD_SCAN_t;
#endif
