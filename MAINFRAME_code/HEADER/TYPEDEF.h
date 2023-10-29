#ifndef TYPEDEF
	#define TYPEDEF
	#include "stdint.h"
	#include <defines.h>
	/// define type for MCU flag	
typedef union
	{
		uint32_t 	ADC_MAIN;
		uint16_t	adc_buff[2];
		uint8_t		adc_little_buff[4];
	}ADC_DATA_t;


	typedef union
	{
			uint32_t flag_byte;
				struct 
				{
					uint32_t    flag_ask_for_data:1;
					uint32_t	  panel_ask_for_data:1;
					uint32_t	  data_ask:1;
					uint32_t	  calibration_activated:1;		
					uint32_t	  finish_flag:1;
					uint32_t	  panel_is_running:1;
					uint32_t	  panel_send_process:1;	
					uint32_t 		process_runnig:1;
					uint32_t		if_proccess_start:1;
					uint32_t		init_state_activated:1;
					uint32_t		timer_counter_on:1;
					uint32_t		panel_get_data_trigger:1;
					uint32_t		send_to_panel_bus_busy:1;
					uint32_t		send_or_Nsend:1;
					uint32_t		send_50_50:1;
					uint32_t		write_to_eeprom:1;
				}flag_BIT;

			}MCU_FLAG_t;



	/// define type for MCU flag		
	typedef union
	{
			uint32_t flag_byte;
				struct 
				{
					uint8_t    timer1:1;
					uint8_t    timer2:1;
					uint8_t    timer3:1;
					uint8_t    timer4:1;
					uint8_t    timer5:1;
					uint8_t    timer6:1;
					uint8_t    timer7:1;
					uint8_t    timer8:1;
				
				}flag_BIT;

			}timer_flag_t;
			

			
	typedef union
	{
			uint8_t byte;
				struct 
				{
					uint8_t    BIT6:1;
					uint8_t    BIT5:1;
					uint8_t    BIT4:1;
					uint8_t    BIT3:1;
					uint8_t    BIT2:1;
					uint8_t    BIT1:1;
					uint8_t    BIT8:1;
					uint8_t    BIT7:1;
				}bit;
			}BYTE_TO_BIT_t;			
			
			
			
			
typedef enum
{
	defult_test =1,
	online_test =3,
	calibration =2,
	init = 0,
}STATE_MODE_t;

typedef enum
{
	step1,
	step2,
	step3,
}command_state_t;

typedef enum
{
	ethernet,
	usb,
	rs485,
}port_selector_t;





typedef union
{
float data;
uint8_t byte[4];


}float_to_byte_t;

typedef union
{
	uint64_t  main_data;
			struct 
				{
				uint8_t byte_data[8];

				}byte;
}uint64_to_byte;

typedef union
{
	uint32_t  main_data;
			struct 
				{
				uint8_t byte_data[4];

				}byte;
}uint32_to_byte;


typedef union
{
	uint8_t rx_buffer[technical_buffer_size];
			struct 
				{
				uint8_t	start_buffer[5];
				uint8_t can_buffer_send[(technical_buffer_size-7)/8][8];
				uint8_t end_buffer[2];	
					
				}detail;

	
}technical_array_t;

typedef union
{
	uint8_t rx_buffer[30];
			struct 
				{
					uint8_t		get_data[26];
					uint32_t 	datacounter;					
				}detail;
}send_data_buffer_t;


typedef union
{
	uint8_t send_buff[40];
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
}panel_buff_t;

typedef union
{
	uint16_t big_data;
	uint8_t	 small_pack[2];
}U16_to_U8_t;

#endif
