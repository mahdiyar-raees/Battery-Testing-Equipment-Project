#ifndef FUNCTION
#define FUNCTION
	#include "stdio.h"
	#include "string.h"
	#include "stdint.h"
	#include "TYPEDEF.h"
	#include "SEMI_FUNCTION.h"
	#include "stm32f7xx_hal.h"
	#include "main.h"
	#include "defines.h"
	
		//****************PARAMETER*********************//
	extern									uint16_t								test_1;
		
		
		extern 								CAN_HandleTypeDef 							hcan1;
		extern								TIM_HandleTypeDef 							htim6;
		extern 								TIM_HandleTypeDef 							htim7;
		extern								TIM_HandleTypeDef 							htim3;
		extern 								TIM_HandleTypeDef 							htim5;
		extern 								TIM_HandleTypeDef 							htim14;
		extern								SPI_HandleTypeDef 							hspi2;
		extern								TIM_HandleTypeDef 							htim4;
		extern								TIM_HandleTypeDef 							htim9;
		extern								TIM_HandleTypeDef 							htim10;	
		extern								ADC_BUFFER_t										temp_adc1 , temp_adc2;
		extern 								process_complete_buffer_t				process_buffer;	
		extern 								process_complete_buffer_t				*process_buffer_ptr;
		extern 								STATE_MODE_t 										system_state;
		extern 								float														passed_time;
		extern 								process_complete_buffer_t				process_buffer;	
		extern 								process_complete_buffer_t				*process_buffer_ptr;
		extern 								process_excute_t								executing_command;
		extern 								process_excute_t								*executing_command_ptr;
		extern 								uint8_t													step_count;
		extern 								STATE_count_t 									system_step;
		extern								static_parameter_ptr						*test_parameter_ptr;
		extern								STATE_count_t 									set_process_step;
		extern 								ADC_BUFFER_t										executing_adc_valu;
		extern 								uint16_t												dac_change_value[DAC_CHANGE_STEP_COUNT];
		extern								uint8_t 												dac_step_counter;
		extern								uint8_t 												fan_duty;
		extern								ADC_BUFFER_t										validate_shunt, validate_battery;
		extern								uint16_t												shunt_send_data,battery_send_data;
		extern								uint8_t													fan_counter;
		extern								uint8_t 												fan_read[100];
		extern								uint32_t												new_resistance;
		extern								float_to_byte_t 								READ_VOLTAGE_CALIBRATION_ALPHA		;							 // alpha *x + bete
		extern								float_to_byte_t 								READ_VOLTAGE_CALIBRATION_BETA			;							// alpha *x + bete
		extern								float_to_byte_t 								READ_SHUNT_CALIBRATION_ALPHA		  ;          // alpha *x + bete
		extern								float_to_byte_t 								READ_SHUNT_CALIBRATION_BETA			  ;         	// alpha *x + bete
		extern								uint32_t												battery_moving_sum;
		extern								uint32_t												shunt_moving_sum;
		extern								uint8_t													last_fan_value;	
		extern								float_to_byte_t 								mean_voltage ;
		extern								float_to_byte_t 								mean_shunt 	;
		extern								float 													power_protection;
		extern								float														current_protection;
		extern								MCU_FLAG_t											flag;
		extern								double 													power_mean; 		
		extern								float														last_cc_value;
		extern								float														last_cr_value;
		extern								uint8_t													Error_code;
		extern								uint8_t 												odd_or_even;
		extern								uint16_t 												fan_run;
		extern								uint8_t 												battery_cnt;
		extern								uint8_t 												shunt_cnt;		
		extern								uint32_t												t_sum;
		extern								uint8_t													fna_turn_off_counter;
		extern								uint8_t													not_syncronize_signal;
		extern								uint16_t 												time_reverse_cnt;
		extern								uint32_t												debug_variable;
		extern								uint8_t													timer6_counter;
		extern								uint8_t													timer6_compare_var;
		extern		    				uint8_t	         								over_voltage_counter;
		extern								uint8_t													trigger_counter;
		extern								uint16_t												current_buffer[MODE_BUFFER_SIZE];
		extern								uint16_t												voltage_buffer[MODE_BUFFER_SIZE];
		//*******************FUNCTIOM*****************//
		__inline					void 									SPI_handeler				(SPI_HandleTypeDef *hspi);
		__inline					void 									trigg_spi						(SPI_HandleTypeDef *hspi);
											void 									timer_reset					(uint8_t timer_count);
		extern 						void 									Error_Handler				(void);
											void 									INIT_STATE					(void);
											void 									CAN_INIT						(void);
			__inline				void 									delayUS							(uint32_t us);
											void									delayMS			      	(uint32_t us);
											void									trigger_rst					(void);
											void 									CAN_SEND_DATA				(uint8_t *transmit_buffer , uint16_t identifier,uint8_t lenth);
											void 									MY_Error_HANDLE			(uint16_t error_code);																
											void 									START_STATE					(void);
			__inline				void 									DAC_CONFIG					(float value,float  last_value);
			__inline				void 									FAN_CONFIG					(uint8_t duty);
		  __inline		  	uint32_t 							set_process					(process_excute_t *command);
											void 									DAC_OFF							(void);
											uint8_t								Error_cheack				(void);
			__inline				void 									dac_set_value				(ADC_BUFFER_t dac_value);
			__inline				uint8_t 							one_min_error_cheak	(void);
											//void 									dac_change_Step			(uint8_t new_value);
											HAL_StatusTypeDef 		MY_RECIVE_FIFO0			(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
											HAL_StatusTypeDef 		MY_RECIVE_FIFO1			(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
											HAL_StatusTypeDef 		MY_SEND_CAN					(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
											//uint32_t 							flash_read					(uint32_t address);
											//void					  			flash_write					(uint32_t address, uint32_t data);
							
																
							
#endif
