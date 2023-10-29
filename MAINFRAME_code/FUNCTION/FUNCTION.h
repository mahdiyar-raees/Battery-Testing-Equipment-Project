#ifndef FUNCTION
#define FUNCTION
	#include "stdio.h"
	#include "string.h"
	#include "stdint.h"
	#include <TYPEDEF.h>
	#include "stm32f4xx_hal.h"

		extern 					CAN_HandleTypeDef 			hcan1;
		extern					TIM_HandleTypeDef 			htim2;
		extern 					TIM_HandleTypeDef 			htim4;		
		extern					TIM_HandleTypeDef 			htim5;
		extern 					TIM_HandleTypeDef 			htim6;
		extern 					TIM_HandleTypeDef 			htim7;
		extern					TIM_HandleTypeDef 			htim8;
		extern					TIM_HandleTypeDef 			htim9;
		extern 					port_selector_t					port;
		extern					DAC_HandleTypeDef 			hdac;
		extern 					UART_HandleTypeDef 			USB_port;

		extern					uint8_t	 	   						bus_busy;
		extern 					UART_HandleTypeDef 			huart4;
		extern 					UART_HandleTypeDef 			huart1;
		extern 					UART_HandleTypeDef 			huart2;
		extern					uint16_t 								detenator_time;
		extern					uint16_t 								detenator_value;
		extern					uint8_t									current_module;
		extern					uint8_t									get_data_counter;
		extern 					uint8_t 								command_state;
		extern					uint8_t 					   		rx_command_code[command_buffer_size];
		extern 					STATE_MODE_t 						system_state;
		extern					uint8_t									cal_sub_menu;
		extern 					uint8_t									execut_command;
		extern 					technical_array_t 			technical_buffer;
		extern					uint8_t 								send_counter;
		extern		   		uint8_t 								available_module[6];
		extern					MCU_FLAG_t							flag;
		extern					uint16_t 								command_counter;
		extern				  uint8_t 								start_counter;

		extern					uint8_t									finish_module[6];
		extern					uint8_t 								send_finish_counter;
		extern				  uint8_t									transmit_error;
		extern 					uint8_t									get_coff_count;
		extern 					CAN_HandleTypeDef			  hcan1;
		extern 					CAN_HandleTypeDef 			hcan2;
		extern					uint64_to_byte					time_counter;	
		extern					uint8_t    							fan_counter_get;

		extern					uint8_t									panel_send_process_counter;
		//panel 
			// panel variable 
		extern	uint8_t								panel_command_counter;
		extern	uint8_t								panel_command_buffer[20];
		extern	uint8_t								panel_comand_for_module[40];
		extern	panel_buff_t					panel_send_buff;	
		extern	uint16_t 							timer_2_counter;
		extern	uint16_t 							timer_6_counter;
		extern	uint16_t 							timer_7_counter;
		extern	uint16_t 							timer_3_counter;
		extern	uint16_t 							timer_4_counter;
		extern  uint16_t 							tim5_send;
		extern	uint8_t 							get_board[6];
		extern  uint8_t								fault_send[6];
		extern	uint8_t 							get_data_buffer[38];
		extern	uint16_t							can_bus_turn_off_counter;
		extern	uint8_t						  	panel_bus_busy;
		extern	uint8_t						  	PC_bus_busy;
		extern	uint8_t						  	process_valid;
		extern	uint32_to_byte				in_syncronize;
		extern	uint8_t f1;
		extern	uint8_t f2;
//***********************************//**********************************//

//********************************FUNCTION*******************************//

//***********************************//**********************************//
		extern void Error_Handler(void);
			
		void 								initial_function(void);
		//panel slector
		void 								port_selector(uint8_t port);
		
		// for reset system to init state
		void 								reset_state(void);
		// for valitate internal command from PC
		void 								validate_command(void);
		void 								panel_command_hadeler(void);								
		void 								command_handler(uint8_t get_command);
										
		void 								SEND_command_handler(uint8_t *data);
		void 								START_command_handler(uint8_t *rx_buffer);										
		void 								panel_send_process_f(void);
		void 								detect_module_address(uint8_t *module1, uint8_t terminal_count);
		void 								CAN_INIT(void);													
		void 								delayUS(uint32_t us);
		void 								delayMS(uint32_t ms);
		uint16_t 						return_module_address(uint8_t module_count);
		void  							panel_send (void);
		void 								uart_init(void);
		void 								comunication_reset(void);	
		void		PC_SEND(uint8_t *send_array , uint8_t size);

		
		void 								CAN_SEND_DATA(uint8_t *transmit_buffer , uint16_t identifier ,uint8_t lenth);
		HAL_StatusTypeDef 	MY_SEND_CAN(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
		HAL_StatusTypeDef 	MY_RECEIVE_FIFO0(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
		HAL_StatusTypeDef 	MY_RECEIVE_FIFO1(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
		void 								CAN_Filtter_config(void);
		void 								dac_off(void);
		void 								dac_set_value(uint16_t value);

#endif

