#ifndef TYPEDEF
	#define TYPEDEF
	#include "stdint.h"
	/// define type for MCU flag		
	typedef union
	{
			uint32_t flag_byte;
				struct 
				{

					uint32_t    online_test_config_first_time:1;//1
					uint32_t    online_test_config_cc:1;//2
					uint32_t    online_test_config_cr:1;
					uint32_t    start_level_counter:1;
					uint32_t    start_buffer_set:1;
					uint32_t    stop_first:1;
					uint32_t    fire_or_not:1;
					uint32_t    state_process_1:1;
					uint32_t    ask_data_trigger:1;
					uint32_t    start_process_counter:1;
					uint32_t    state_process_2:1;
					uint32_t    command_or_data:1;
					uint32_t		 process_data_ready:1;
					uint32_t		 finish_command:1;
					uint32_t		 ASK_DATA:1;
					uint32_t		 mode_bt_has_done:1;
					uint32_t		 mode_cu_has_done:1;		
					uint32_t		 time_to_mode_bat:1;	
					uint32_t		 time_to_mode_cur:1;
					uint32_t		 fan_turn_off_flag:1;	
					uint32_t		 timer9_start:1;
					uint32_t		 check_gate_over_volage:1;
					uint32_t		 FAN_IS_ON:1;
					uint32_t		 cheach_reverse_voltage_done:1;
					uint32_t		 reerse_before_start:1;	
					uint32_t		 RV_not_check:1;
					uint32_t		 run_or_stop:1;
					uint32_t		 panel_send_process_s:1;
					uint32_t		 wait_to_run_erro:1;
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
			




	/// define type for MCU flag		
typedef struct
{
	uint8_t duty;
	uint16_t freq;
	float		volt;
}dac_t;


typedef union
{
				uint8_t 	byte;
				struct 
				{
					uint8_t    BIT1:1;
					uint8_t    BIT2:1;
					uint8_t    BIT3:1;
					uint8_t    BIT4:1;
					uint8_t    BIT5:1;
					uint8_t    BIT6:1;
					uint8_t    BIT7:1;
					uint8_t    BIT8:1;
				
				}BITS;

		}BYTE_to_BIT_t;

		
		
typedef		union
				{
					uint16_t ADC_DATA;
					uint8_t  adc_buff[2];
				}ADC_BUFFER_t;

				
typedef		union
{
	uint8_t		small_buffer[40];
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
			uint16_t				 pulse_count;
			uint16_t				 null_detect;
			uint8_t 				 test_mode;
			uint8_t 				 range;
		}detail;
}process_excute_t;



				
typedef		union
{
	process_excute_t big_ptr;
	struct 
		{
		

			uint32_t			OCP;
			uint32_t			OCP_DELAY;
			uint32_t			OVP;
			uint32_t			OVP_DELAY;
			uint8_t				polarity;
			uint8_t				operation;
		}detail;
}static_parameter_ptr;
typedef		union
{
uint8_t    					 receive_buffer;							 	
process_excute_t		 process_buffer[51];
	
}process_complete_buffer_t;


typedef enum
{
	step1,
	step2,
	step3,
	step4,
}STATE_count_t;				

typedef enum
{
	next,
	start,
	getdata,
	panel_set_proccess,
}recive_command_t;


typedef enum
{
	defult_test,
	online_test,
	calibration,
	init,
}STATE_MODE_t;


typedef union
{
				uint8_t 	send_buffer[5];
				struct 
				{
					
					uint16_t    voltage_shunt;
					uint16_t    voltage_battery;
					uint8_t    firstdata;
				
				}detail;

		}send_data_t;

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
float data;
uint8_t byte[4];


}float_to_byte_t;

#endif
