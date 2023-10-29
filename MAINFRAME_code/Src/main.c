/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include	<AT24Cxx.h>
#include <FUNCTION.h>
#include <SEMI_FUNCTION.h>
#include <defines.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t detenator_time;
uint16_t detenator_value;


		#if !test
		uint8_t data[8] = {0,0,0,0,0,0,0,0};
		#endif
		 uint16_t 								command_counter=0;
		 MCU_FLAG_t						  	flag;										
		 STATE_MODE_t 						system_state = init;
		 uint8_t									command_state=0;
		 uint8_t 									rx_command_code[command_buffer_size];
		 uint8_t 									static_buffer[static_buffersize];
		 uint8_t									current_module;	
		 technical_array_t 				technical_buffer;
		 uint8_t									execut_command;
		 port_selector_t					port = usb;	
		 uint8_t 									available_module[6];
		 uint8_t 									send_counter 				 = 0;
		 uint8_t 									send_finish_counter=0;
		 uint8_t 									start_counter=0;
		 uint8_t									get_data_counter=0;	
		 uint8_t 									get_data_buffer[38];
		 uint8_t									transmit_error =0;
		 ADC_DATA_t							  ADC_DATA[2];
		 uint8_t									finish_module[6] = {0,0,0,0,0,0};
		 uint8_t									send_request=0;
		 CAN_TxHeaderTypeDef 			tx_CAN_Init_1;
		 uint32_t 					 			TX_mailbox_1;
		 uint16_t 								tim5_send=100;
		 uint8_t 									calibration_buffer[10];
		 uint8_t									calibration_cof_buffer[22];	
		 uint8_t									cal_sub_menu=0;
		 uint8_t									counter_calibration=0;
		 uint8_t									cof_buff[110];	
		 uint8_t									get_coff_count=0;
	   uint8_t									fault_send[6];	
		 uint16_t									can_bus_turn_off_counter=0;
		 uint8_t									process_valid;
		 uint32_t									get_data_counter_[6];
			#if test
			uint8_t rx_buffer[8] = {0,0,0,0,0,0,0,0};
			uint8_t *data[2];
				
			#endif
		 
		 uint8_t rx_buffer[8] = {0,0,0,0,0,0,0,0};
			uint8_t    					fan_counter_get;
		 
		 // test variable
			uint32_t    				test_counter=0;
			uint32_t						send_13_counter=0;
			uint32_to_byte			in_syncronize;
		 uint8_t 					wait_for_send_to_panel=0;
			uint64_to_byte			time_counter;
		 			// can recive massage header
			CAN_RxHeaderTypeDef rx_CAN_Init;
			
			uint8_t	 panel_bus_busy;
			uint8_t	 PC_bus_busy;
			uint8_t	 f1;
			uint8_t	 f2;
			
			uint16_t timer_2_counter =0;
			uint16_t timer_6_counter =0;
			uint16_t timer_7_counter =0;
			uint16_t timer_3_counter =0;
			uint16_t timer_4_counter =0;
			
			
// panel variable 
			uint8_t								panel_command_counter;
			uint8_t								panel_command_buffer[20];
			uint8_t								panel_comand_for_module[40];
			panel_buff_t					panel_send_buff;	
			uint8_t								panel_send_process_counter=0;
			uint8_t get_board[6];
			
			
			float_to_byte_t READ_VOLTAGE_CALIBRATION_ALPHA	;
			float_to_byte_t READ_VOLTAGE_CALIBRATION_BETA		; 
			
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM5_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

void initial_get_cof(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	// regular comunication data

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
		#if test 
							
		#endif
		#if !test
		 //uint8_t rx_buffer[8] = {0,0,0,0,0,0,0,0};
								
	TIM6->CNT=0;
		 timer_6_counter=0;
		switch(execut_command)// send command
			{
			case 0:// next
							MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
							SEND_command_handler(rx_buffer);										
			return;
			case 1:// start
							MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);							
							START_command_handler(rx_buffer);
			return;
			case 2:// calibration						
						switch(cal_sub_menu)
						{case 1:
							{
								MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
								HAL_TIM_Base_Stop_IT(&htim6);
								uint8_t data[10] = {0,0,0,0,0,0,0,0,0,0};
								if(rx_buffer[0] == 2 & rx_buffer[1] == 1)
									{
										data[0] = 2;
										data[1] = 1;
										data[2] = rx_buffer[2];
										cal_sub_menu=0;
										flag.flag_BIT.flag_ask_for_data =1;
										PC_SEND(data,10);
									}
								
							}
							return;
						case 2:
							{
											MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
											TIM6->CNT=0;
											timer_6_counter=0;
											uint8_t can_data[8];
											static uint8_t cal_22_data[20];
											if(rx_buffer[0] == 2 & rx_buffer[1] == 2 & rx_buffer[2] == counter_calibration)
											{
												cal_22_data[0] = 2;
												cal_22_data[1] = 2;	
												can_data[0] = 2;
												can_data[1] = 2;											
												for(register uint8_t fast_counter=0;fast_counter<4;fast_counter++)
													{
														cal_22_data[fast_counter+2+ (counter_calibration *4)] = rx_buffer[fast_counter+3];
													}
												counter_calibration++;										
												can_data[2] = counter_calibration;
												if(counter_calibration ==3)
													{
														cal_sub_menu=0;
														__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
														HAL_TIM_Base_Stop_IT(&htim6);
														flag.flag_BIT.flag_ask_for_data =1;													
														PC_SEND(cal_22_data,20);	
														return;
													}
												else 
													{
														delayUS(50);
														CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);	
													}
											}	
									}

							return;
						case	3:
							{
											MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
											TIM6->CNT=0;
											timer_6_counter=0;
											uint8_t can_data[8];
											static uint8_t cal_23_data[20];
											if(rx_buffer[0] == 2 & rx_buffer[1] == 3 & rx_buffer[2] == counter_calibration)
											{
												cal_23_data[0] = 2;
												cal_23_data[1] = 3;	
												can_data[0] = 2;
												can_data[1] = 3;
											
												for(register uint8_t fast_counter=0;fast_counter<4;fast_counter++)
													{
														cal_23_data[fast_counter+2+ (counter_calibration *4)] = rx_buffer[fast_counter+3];
													}
												counter_calibration++;
											
												can_data[2] = counter_calibration;
												if(counter_calibration ==4)
													{
														cal_sub_menu=0;
														HAL_TIM_Base_Stop_IT(&htim6);
														flag.flag_BIT.flag_ask_for_data =1;													
														PC_SEND(cal_23_data,20);
														return;
													}
												else 
													{
														delayUS(50);
														CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);	
													}
											}	
									}
						return;
						case 4: // submit dac value
						{
								MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
								if(rx_buffer[0] == 2 & rx_buffer[1] == 4)
									{										
										switch(counter_calibration)
											{
											case 3:
											{
													uint8_t can_data[8];
													TIM6->CNT=0;
													timer_6_counter=0;
													can_data[0] = 2;
													can_data[1] =4;
													can_data[2] =1; 
													can_data[3] =calibration_cof_buffer[8];
													can_data[4] =calibration_cof_buffer[9];
													can_data[5] =calibration_cof_buffer[10];
													can_data[6] =calibration_cof_buffer[11];
													counter_calibration--;
													delayUS(50);
													CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);
													
											}
												return;
											case 2:
											{
													uint8_t can_data[8];
													TIM6->CNT=0;
													timer_6_counter=0;
													can_data[0] = 2;
													can_data[1] =4;
													can_data[2] =2; 
													can_data[3] =calibration_cof_buffer[12];
													can_data[4] =calibration_cof_buffer[13];
													can_data[5] =calibration_cof_buffer[14];
													can_data[6] =calibration_cof_buffer[15];
													counter_calibration--;
													delayUS(50);
													CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);
													
											}
												return;
											case 1:
											{
													uint8_t can_data[8];
													TIM6->CNT = 0;
													can_data[0]   = 2;
													can_data[1]   = 4;
													can_data[2]   = 3; 
													can_data[3] =calibration_cof_buffer[16];
													can_data[4] =calibration_cof_buffer[17];
													can_data[5] =calibration_cof_buffer[18];
													can_data[6] =calibration_cof_buffer[19];
													HAL_TIM_Base_Stop_IT(&htim6);
													counter_calibration =0 ;
													delayUS(50);												
													CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);
													
											}
											return;
											case 0:
												{
													uint8_t data[10] = {0,0,0,0,0,0,0,0,0,0};
													data[0] = 2;
													data[1] = 4;
													data[2] = 3;
													command_state =0;
													HAL_TIM_Base_Stop_IT(&htim6);
														counter_calibration =0 ;
														flag.flag_BIT.flag_ask_for_data =1;
													PC_SEND(data,10);
												}
												return;	
								}	
									
									}						
								}
							return;						
						case 5:// dac of calibration	
						{
							uint8_t data[10] = {0,0,0,0,0,0,0,0,0,0};
							HAL_TIM_Base_Stop_IT(&htim5);
							MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
							data[0] = rx_buffer[0];
							data[1] = rx_buffer[1];
							cal_sub_menu=0;
							flag.flag_BIT.flag_ask_for_data =1;
							PC_SEND(data,10);
						}
							return;						
						case 6: // exist calibration
							{
							uint8_t data[10] = {0,0,0,0,0,0,0,0,0,0};
							HAL_TIM_Base_Stop_IT(&htim5);
							MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
							data[0] = rx_buffer[0];
							data[1] = rx_buffer[1];
								flag.flag_BIT.flag_ask_for_data =1;
							reset_state();
							PC_SEND(data,10);
							}
							return;
						default:
							MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
						}			
				return;
			case 3:// internal get data
								{
											//static uint8_t get_board[6];
										 uint8_t get_board_counter=0;
											if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
													{
														Error_Handler();
													}
											//MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);	
											// note that module should send address beging from 2 6 10 14 18 22  NOT 20
											get_data_buffer[rx_buffer[4] + 	   buffer_size] = rx_buffer[0];
											get_data_buffer[rx_buffer[4] + 1 + buffer_size]	= rx_buffer[1];
											get_data_buffer[rx_buffer[4] + 2 + buffer_size]	= rx_buffer[2];
											get_data_buffer[rx_buffer[4] + 3 + buffer_size]	= rx_buffer[3];
											get_board[(uint8_t)(rx_buffer[4]/4)] =1;
											get_data_counter_[(uint8_t)(rx_buffer[4]/4)]++;
											// get data counter shows the number of available module 
											for(register uint8_t cnt=0; cnt<6;cnt++)
													{
													if(get_board[cnt] == 1)
														{
															get_board_counter++;
														}
													}
//												HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING |  \
//																CAN_IT_RX_FIFO1_MSG_PENDING );		
											if(get_board_counter >= get_data_counter)
												{
													f2 =1;
													transmit_error=0;
													memset(get_board,0,6);
													TIM6->CNT =0;
													get_data_buffer[33] =	ADC_DATA[0].adc_little_buff[0];
													get_data_buffer[34] = ADC_DATA[0].adc_little_buff[1];
													get_data_buffer[29] = time_counter.byte.byte_data[0];
													get_data_buffer[30] = time_counter.byte.byte_data[1];
													get_data_buffer[31] = time_counter.byte.byte_data[2];
													get_data_buffer[32] = time_counter.byte.byte_data[3];
													if(flag.flag_BIT.send_or_Nsend)
														{
														HAL_UART_Transmit(&huart1,get_data_buffer,38,100);	
														flag.flag_BIT.timer_counter_on =1;
														process_valid= 3;
		
														can_bus_turn_off_counter =0;	
														send_13_counter++;	
															
														
														}
												}
										}
										return;
			case 4:
				{
					MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
					TIM6->CNT=0;
					timer_6_counter=0;
					if(rx_buffer[0] == 2 & rx_buffer[1] == 3)
						{
							if(rx_buffer[2] == get_coff_count & rx_buffer[7] == (current_module +19))
								{
									uint8_t initial_array_counter;
									initial_array_counter = (16 * (current_module-1)) + ( 4 * get_coff_count)+2;
									for(uint8_t cnt=0; cnt<4;cnt++)
										{
											cof_buff[initial_array_counter + cnt] = rx_buffer[cnt+3];											
										}
									get_coff_count++;
									if(get_coff_count ==4) // finish one module try next module 
										{
											get_coff_count=0;
											if(current_module ==6)
												{
													TIM6->CNT=0;
													HAL_TIM_Base_Stop_IT(&htim6);
													flag.flag_BIT.flag_ask_for_data =1;
													cof_buff[98]  = READ_VOLTAGE_CALIBRATION_ALPHA.byte[0];
													cof_buff[99]  = READ_VOLTAGE_CALIBRATION_ALPHA.byte[1];
													cof_buff[100] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[2];
													cof_buff[101] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[3];
													cof_buff[102] =	READ_VOLTAGE_CALIBRATION_BETA.byte[0];
													cof_buff[103] = READ_VOLTAGE_CALIBRATION_BETA.byte[1];
													cof_buff[104] = READ_VOLTAGE_CALIBRATION_BETA.byte[2];
													cof_buff[105] =	READ_VOLTAGE_CALIBRATION_BETA.byte[3];	
													PC_SEND(cof_buff,110);
													return;
												}
											for(uint8_t cnt = current_module;cnt<7;cnt++)
													{

													if(available_module[cnt]==1)
														{
															current_module = cnt+1;															
															data[0] = 2;
															data[1] = 3;
															data[2] = get_coff_count;	
															uint8_t data[8]= {0,0,0,0,0,0,0,0};
															data[0] = 2;
															data[1] = 3;
															data[2] = get_coff_count;
															TIM6->CNT=0;
															timer_6_counter=0;
															CAN_SEND_DATA(data,return_module_address(current_module-1),8);															
															return;
														}
													else if(cnt ==5)
														{
															TIM6->CNT=0;
															HAL_TIM_Base_Stop_IT(&htim6);
															flag.flag_BIT.flag_ask_for_data =1;
															cof_buff[98]  = READ_VOLTAGE_CALIBRATION_ALPHA.byte[0];
													cof_buff[99]  = READ_VOLTAGE_CALIBRATION_ALPHA.byte[1];
													cof_buff[100] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[2];
													cof_buff[101] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[3];
													cof_buff[102] =	READ_VOLTAGE_CALIBRATION_BETA.byte[0];
													cof_buff[103] = READ_VOLTAGE_CALIBRATION_BETA.byte[1];
													cof_buff[104] = READ_VOLTAGE_CALIBRATION_BETA.byte[2];
													cof_buff[105] =	READ_VOLTAGE_CALIBRATION_BETA.byte[3];
															PC_SEND(cof_buff,110);
														}
													}

										}
									else
										{
											uint8_t data[8]= {0,0,0,0,0,0,0,0};
											data[0] = 2;
											data[1] = 3;
											data[2] = get_coff_count;
											TIM6->CNT=0;
											timer_6_counter=0;
											CAN_SEND_DATA(data,return_module_address(current_module-1),8);
										}
								}
						}
				
				}
				return;
			case 5: // cheack what board is available and get their cof var 
				{
					timer_6_counter =0;
					uint8_t initial_array_counter;
					MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
					if(rx_buffer[0] == 2 && rx_buffer[1] == 3 && rx_buffer[2] == get_coff_count && rx_buffer[7] ==current_module+19 )
					{
					initial_array_counter = (16 * (current_module-1)) + ( 4 * get_coff_count)+2;
					for(uint8_t cnt=0; cnt<4;cnt++)
						{
							cof_buff[initial_array_counter + cnt] = rx_buffer[cnt+3];											
						}
					
						get_coff_count++;
						
						initial_get_cof();
					}
					else
					{
					initial_get_cof();
					}
					return;
				}
			case 40: // cheak fan state

				{
					
						static uint8_t  fan_detect[6];
									 uint8_t	fan_ok_count=0;
									 uint8_t	PC_data[10] = {1,2,0,0,0,0,0,0,0,0};
						MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
						if(rx_buffer[0] == 1 & rx_buffer[1] == 2)
						{
							fan_detect[rx_buffer[2]-20] =  rx_buffer[3];
							fan_counter_get++;
							TIM6->CNT =0;
							if(fan_counter_get==get_data_counter)
								{
								for(register uint8_t fast_cnt =0; fast_cnt<6;fast_cnt++)
									{
										if(fan_detect[fast_cnt]==1)
										{											
											fan_ok_count++;											
											fan_detect[fast_cnt] =0;	
										}
										else if(fan_detect[fast_cnt]==2)
										{																
											PC_data[2+fast_cnt] = 2;
										}
										else
										{
											PC_data[2+fast_cnt] = 1;
										}
									}
							if(fan_ok_count == get_data_counter) // no fault
									{
										TIM5->CNT =0;
										execut_command = 3;
										TIM6->CNT =0;
										flag.flag_BIT.process_runnig =1;
										flag.flag_BIT.if_proccess_start =1;
										HAL_TIM_Base_Stop_IT(&htim6);
										__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
										timer_6_counter=0;
										TIM6->CNT=0;
										__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);	
										test_counter=0;
										send_13_counter=0;
										flag.flag_BIT.timer_counter_on =0;
										time_counter.main_data =0;
										delayMS(1000);
										flag.flag_BIT.send_or_Nsend =1;
										HAL_TIM_Base_Start_IT(&htim5);
										
										detenator_value = (DAC_cal_set_alph * detenator_value) + DAC_cal_set_beta;
										detenator_time = (detenator_time*10) -1;
										
										if(detenator_time > 10000)
											{
											detenator_time = 10000;
											}
											
											f1 =1;
											f2 =0;
//										TIM7->ARR = detenator_time;
//										dac_set_value(detenator_value);
//										//HAL_TIM_Base_Start_IT(&htim7);
//										TIM7->CNT=0;
//										__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
//										__HAL_TIM_ENABLE(&htim7);	
										flag.flag_BIT.flag_ask_for_data =1;
										uint8_t CAN_data2[2] ={1,1};	
										fan_counter_get =0;
										TIM6->CNT =0;		
										timer_6_counter=0;
										CAN_SEND_DATA(CAN_data2,module_commen_address,2);	
										if(flag.flag_BIT.panel_is_running)
												{
													flag.flag_BIT.panel_ask_for_data= 1;
													panel_send_buff.detail.type_or_ok = OK;
													panel_send();												
												}
											else
											{
												command_counter=0;
												for(uint8_t cnt=0;cnt<8;cnt++)
												{PC_data[cnt+2]= 0;}												
												PC_SEND(PC_data,10);
											}												
									}
								else // it mean fault
									{
									if(flag.flag_BIT.panel_is_running)
												{
													flag.flag_BIT.panel_ask_for_data= 1;
													panel_send_buff.detail.type_or_ok = 20;
													panel_send();												
												}	
									for(uint8_t cnt=0;cnt<6;cnt++)
											{
												fan_detect[cnt] =0;
											}
									PC_SEND(PC_data,10);		
									reset_state();	
									return;		
									}
									}


								
						}
				}
				return;
			case 60: //Online Test
				{
					MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
					uint8_t   getmodule = rx_buffer[1] -19;
					uint8_t		PC_data[10];
					PC_data[0] = 4;
					PC_data[2] = getmodule;
					if(getmodule ==current_module)
					{
						__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
						HAL_TIM_Base_Stop_IT(&htim6);
						TIM6->CNT=0;
						switch(rx_buffer[0])
							{
							case 1: //CC value
								PC_data[1] = 1;
								flag.flag_BIT.flag_ask_for_data=1;
								PC_SEND(PC_data,10);
								return;
							case 2: 	// CR value
								PC_data[1] = 2;
								flag.flag_BIT.flag_ask_for_data=1;
								PC_SEND(PC_data,10);
								return;
							case 3: // stop
								if(PC_bus_busy>0)
									{
									PC_bus_busy--;
									}

								available_module[getmodule -1] =0;
								
								PC_data[1] = 3;
								flag.flag_BIT.flag_ask_for_data=1;
								PC_SEND(PC_data,10);
								return;
							case 4: // start
								PC_bus_busy++;
								panel_send_buff.detail.finish_test =0;
								panel_send_buff.detail.error_type =0;
								available_module[getmodule -1] =1;
								fault_send[getmodule-1] = 0;	
								PC_data[1] = 4;
								flag.flag_BIT.flag_ask_for_data=1;
								PC_SEND(PC_data,10);
								
								return;
							case 5: // start and fire
							{
								PC_bus_busy++;
								panel_send_buff.detail.finish_test =0;
								panel_send_buff.detail.error_type =0;
								U16_to_U8_t temp_8_16;
								temp_8_16.small_pack[0] = rx_command_code[4];
								temp_8_16.small_pack[1] = rx_command_code[5];
								
								
								
								detenator_value = (DAC_cal_set_alph * temp_8_16.big_data) + DAC_cal_set_beta;
								dac_set_value(detenator_value); //; 
								// fire detenator 
								// using dac
								available_module[getmodule -1] =1;
								fault_send[getmodule-1] = 0;
								temp_8_16.small_pack[0] = rx_command_code[6];
								temp_8_16.small_pack[1] = rx_command_code[7];
								
								detenator_time = (temp_8_16.big_data * 10)-1;
								if(detenator_time > 10000)
									{
									detenator_time = 10000;
									}
								
								TIM7->ARR = detenator_time;
								
								TIM7->CNT = 0;
								//HAL_TIM_Base_Start_IT(&htim7);
										TIM7->CNT=0;
									__HAL_TIM_CLEAR_FLAG(&htim7, TIM_SR_UIF);
										__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
										__HAL_TIM_ENABLE(&htim7);
								// set timer for fire 
								PC_data[1] = 5;
								flag.flag_BIT.flag_ask_for_data=1;
								PC_SEND(PC_data,10);
							}
								return;
							case 6: // current protection
								PC_data[1] = 6; 
								flag.flag_BIT.flag_ask_for_data=1;
								PC_SEND(PC_data,10);
								return;
							case 7: // powert protection
								PC_data[1] = 7;
								flag.flag_BIT.flag_ask_for_data=1;
								PC_SEND(PC_data,10);
								return;
							case 8: // get data 
								PC_data[1] = 8;
								PC_data[3] = rx_buffer[3];
								PC_data[4] = rx_buffer[4];
								PC_data[5] = rx_buffer[5];
								PC_data[6] = rx_buffer[6];
								PC_data[9] = rx_buffer[7];
							// fill data for panel send 
								get_data_buffer[(rx_buffer[1]-20)*4 + 5] = PC_data[3];
								get_data_buffer[(rx_buffer[1]-20)*4 + 6] = PC_data[4];
								get_data_buffer[(rx_buffer[1]-20)*4 + 7] = PC_data[5];
								get_data_buffer[(rx_buffer[1]-20)*4 + 8] = PC_data[6];
								flag.flag_BIT.flag_ask_for_data=1;
								
								PC_SEND(PC_data,10);
								return;
							case 9:
								{
									PC_data[1] = 9;
									
									PC_data[3] = rx_buffer[2];
									PC_data[4] = rx_buffer[3];
									PC_data[5] = rx_buffer[4];
									PC_data[6] = rx_buffer[5];
									PC_data[7] = rx_buffer[6];
									PC_SEND(PC_data,10);
									if(rx_buffer[2] ==3)
										{
											flag.flag_BIT.flag_ask_for_data=1;
										}
									else
										{
											uint8_t can_buffer[8];
											can_buffer[0]   =4 ;
											can_buffer[1] =9 ;
											can_buffer[2] =rx_buffer[2] +1 ;
											CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
										}
										return;
								}
							
							}
						}
					else 
					{
					//fault
					}
				}
					return;
			case 70:
				{
					MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
					
					if(rx_buffer[0] ==5 & rx_buffer[1] == current_module +19)
						{
							panel_send_process_f();
						}
				}
			default:
				{
					MY_RECEIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
				}
			
			}
#endif
	}

// ERROR comunication port
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
			
	uint8_t rx_buffer[8]; 
	MY_RECEIVE_FIFO1(hcan,CAN_RX_FIFO1,&rx_CAN_Init,rx_buffer);
#if 1								
		switch(rx_buffer[0])// Finish process
			{
			case 0xFE:
						{
							uint8_t aval_module_count=0;
							available_module[rx_buffer[1]-20] =0;
							uint8_t count_step = ((rx_buffer[1]-20) * 4)+5 ;
							get_data_buffer[count_step] 	= 0;
							get_data_buffer[count_step+1] = 0;
							get_data_buffer[count_step+2] = 0;
							get_data_buffer[count_step+3] = 0;
							for(register	uint8_t cnt=0; cnt<6; cnt++)
								{
								if(available_module[cnt]==1)
									{
									aval_module_count++;
									}
								}
							get_data_counter =aval_module_count;
							if(aval_module_count ==0)
								{
								// send hand shake to PC
									flag.flag_BIT.finish_flag=1;
									reset_state();
								}
						}
				return;
			case 0XFF: //ERROR
						{
							uint8_t	Error_data[2] = {0XFF,0};
							uint8_t send_state =0;
							switch(system_state)
								{
								case defult_test:
									send_state = 1;
								break;
								case calibration:
									send_state =2;
								break;
								default:
									send_state =0;
								}
								if(PC_bus_busy>0 && fault_send[rx_buffer[1]-20] == 0 &&	available_module[rx_buffer[1]-20]==1)
									{
									PC_bus_busy--;
									}
//							if(fault_send[rx_buffer[1]-20] == 0 &&	available_module[rx_buffer[1]-20]==1)
//								{
									panel_send_buff.detail.error_type = rx_buffer[2];
									uint8_t	PC_data[10] = {0XFF,0XFF, rx_buffer[1]-19,rx_buffer[2],send_state,0,0,0,0,0};
									PC_SEND(PC_data,10);
									fault_send[rx_buffer[1]-20] = 1;
									available_module[rx_buffer[1]-20]=0;
								//}
							if(system_state ==defult_test)
								{
									CAN_SEND_DATA(Error_data,module_commen_address,2);
									reset_state();
								}
						
						}
				return;
			
			
			}
#endif
	}



// external interupt handeler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin ==GPIO_PIN_2) // Supply_OK
		{
			{
				uint8_t can_data[8] = {0xFE, 0xFE,POWER_TURN_OFF,0xFE, 0xFE,0xFE,0xFE,0xFE};
				CAN_SEND_DATA(can_data,module_commen_address,8);
				uint8_t 	data[10] = {0xFF,POWER_TURN_OFF ,0, 0,0,0,0,0,0,0};
				PC_SEND(data,10);
				reset_state();
				AT24Cxx_WriteEEPROM(0,&get_data_buffer,38);
				// OR
//				for(uint8_t cnt=0; cnt<38;cnt++)
//					{
//						AT24_Write_8(cnt,get_data_buffer[cnt]);
//					}
			}
		}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{

		if(htim->Instance ==TIM5) // 1ms ask data 
			{
				// send get_data
				
				TIM5->CNT=0;
				#if !comunaction_test
				if(f1+ f2 >=2)
					{
						f1 =0;
						
						TIM7->ARR = detenator_time;
						dac_set_value(detenator_value);
						//HAL_TIM_Base_Start_IT(&htim7);
						TIM7->CNT=0;
						//TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
						__HAL_TIM_CLEAR_FLAG(&htim7, TIM_SR_UIF);
						__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
						__HAL_TIM_ENABLE(&htim7);	
					}
				test_counter++;
				//memset(get_data_buffer, 0 ,);
				if(flag.flag_BIT.timer_counter_on)
				{
				//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
				in_syncronize.main_data++;
				time_counter.main_data++;
				}
				uint8_t CAN_data[5];
					CAN_data[0] = process_valid;		
					CAN_data[1] = in_syncronize.byte.byte_data[3];
					CAN_data[2] = in_syncronize.byte.byte_data[2];
					CAN_data[3] = in_syncronize.byte.byte_data[1];
					CAN_data[4] = in_syncronize.byte.byte_data[0];
				
				if(transmit_error==0)
					{
						CAN_SEND_DATA(CAN_data,module_commen_address,5);
						
						//MY_SEND_CAN(&hcan1,&tx_CAN_Init_1, CAN_data , &TX_mailbox_1);
						transmit_error++;
						return;
					}
				
				transmit_error++;
				if(transmit_error%5==0)
					{
						CAN_SEND_DATA(CAN_data,module_commen_address,5);
						
						can_bus_turn_off_counter++;
						if(can_bus_turn_off_counter >1000)
							{
								for(uint8_t temp_counter=0;temp_counter<6;temp_counter++)
									{
										if(available_module[temp_counter] == 1 && get_board[temp_counter] ==0)
											{
											reset_state();	
											uint8_t 	data[10] = {0xFF,0xFF ,temp_counter+1, NOT_FOUND_CHARGER,3,0,0,0,0,0};
											PC_SEND(data,10);
											break;
											}											
									}
								reset_state();	
							}
						//MY_SEND_CAN(&hcan1,&tx_CAN_Init_1, CAN_data , &TX_mailbox_1);
					}

				return;
				#endif
			}
		
		else if(htim->Instance ==TIM2)
		{
				timer_2_counter++;
				if(timer_2_counter>4)
				{
					if(system_state != defult_test)
						{
						reset_state();
						}
					else
						{
						comunication_reset();	
						}
				}
			}
		else 	if(htim->Instance ==TIM6)
			{
				timer_6_counter++;
				if(timer_6_counter >4)
				{
					if(flag.flag_BIT.init_state_activated)
						{
							current_module++;
							get_coff_count=0;
							if(current_module ==7) // finish getting coeffitent send to panel and run reset state
								{
																						cof_buff[98]  = READ_VOLTAGE_CALIBRATION_ALPHA.byte[0];
													cof_buff[99]  = READ_VOLTAGE_CALIBRATION_ALPHA.byte[1];
													cof_buff[100] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[2];
													cof_buff[101] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[3];
													cof_buff[102] =	READ_VOLTAGE_CALIBRATION_BETA.byte[0];
													cof_buff[103] = READ_VOLTAGE_CALIBRATION_BETA.byte[1];
													cof_buff[104] = READ_VOLTAGE_CALIBRATION_BETA.byte[2];
													cof_buff[105] =	READ_VOLTAGE_CALIBRATION_BETA.byte[3];
									HAL_UART_Transmit(&huart4,cof_buff,110, 100);
									flag.flag_BIT.init_state_activated =0;
									reset_state();
								}
							else
								{
								initial_get_cof();
								}
							return;
						}
					else
						{
							timer_6_counter =0;
							if(flag.flag_BIT.panel_is_running)
								{
									
									panel_send_buff.detail.fault_or_command = 0xFF;
									panel_send_buff.detail.fault_or_command = 20;
									// send respond to panel 
									reset_state();
									return;
								}
							uint8_t 	data[10] = {0xFF,0xff ,current_module, NOT_FOUND_CHARGER,3,0,0,0,0,0};
							PC_SEND(data,10);
							comunication_reset();
							flag.flag_BIT.flag_ask_for_data =1;
						}
				}
			}
		else 	if(htim->Instance ==TIM7)	
			{
				// turn off detenator
				HAL_TIM_Base_Stop_IT(&htim7);
				dac_off();
			}
		else if(htim->Instance ==TIM3)
			{
				// panel not complete test
			}
		else if(htim->Instance == TIM4)
			{
				// nor respond to panel command 
			}
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
panel_send_buff.detail.start_buff_1 = panel_start_1;
panel_send_buff.detail.start_buff_2 = panel_start_2;
panel_send_buff.detail.stop_buff_1 = panel_end_1;
panel_send_buff.detail.stop_buff_2 = panel_end_2;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
	MX_TIM9_Init();
 // MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
//port_selector(1);
HAL_TIM_Base_Start_IT(&htim7);
HAL_TIM_Base_Start(&htim8);
dac_off();
CAN_INIT();
tx_CAN_Init_1.DLC = 2;
tx_CAN_Init_1.StdId = module_commen_address;
tx_CAN_Init_1.IDE =  CAN_ID_STD   ; 
tx_CAN_Init_1.RTR = CAN_RTR_DATA;
cof_buff[0] = 1 ;
cof_buff[1] = 6;
HAL_Delay(1000);
{
uint8_t can_data[8] = {0xFE, 0xFE,0xFE,0xFE, 0xFE,0xFE,0xFE,0xFE};
CAN_SEND_DATA(can_data,module_commen_address,8);
}
HAL_Delay(1000);

initial_function();
dac_off();
//reset_state();
//HAL_UART_Receive_IT(&huart2,data,8);

		port = rs485;
__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
send_finish_counter =0;

__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);




AT24_Read_8(0,&READ_VOLTAGE_CALIBRATION_ALPHA.byte[0]);
AT24_Read_8(1,&READ_VOLTAGE_CALIBRATION_ALPHA.byte[1]);
AT24_Read_8(2,&READ_VOLTAGE_CALIBRATION_ALPHA.byte[2]);
AT24_Read_8(3,&READ_VOLTAGE_CALIBRATION_ALPHA.byte[3]);


AT24_Read_8(4,&READ_VOLTAGE_CALIBRATION_BETA.byte[0]);
AT24_Read_8(5,&READ_VOLTAGE_CALIBRATION_BETA.byte[1]);
AT24_Read_8(6,&READ_VOLTAGE_CALIBRATION_BETA.byte[2]);
AT24_Read_8(7,&READ_VOLTAGE_CALIBRATION_BETA.byte[3]);
																									
get_data_buffer[3] = 1; 
get_data_buffer[4] = 3;
get_data_buffer[0] = first_frame_1;				
get_data_buffer[1] = first_frame_2;
get_data_buffer[2] = first_frame_3;
get_data_buffer[35] = second_frame_1;				
get_data_buffer[36] = second_frame_2;
get_data_buffer[37] = second_frame_3;	

flag.flag_BIT.flag_ask_for_data =1;
HAL_ADC_Start_DMA(&hadc1,&ADC_DATA[0].ADC_MAIN , 2);

flag.flag_BIT.panel_ask_for_data =1;	
MX_IWDG_Init();
reset_state();	

	cof_buff[98]  = READ_VOLTAGE_CALIBRATION_ALPHA.byte[0];
	cof_buff[99]  = READ_VOLTAGE_CALIBRATION_ALPHA.byte[1];
	cof_buff[100] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[2];
	cof_buff[101] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[3];
	cof_buff[102] =	READ_VOLTAGE_CALIBRATION_BETA.byte[0];
	cof_buff[103] = READ_VOLTAGE_CALIBRATION_BETA.byte[1];
	cof_buff[104] = READ_VOLTAGE_CALIBRATION_BETA.byte[2];
	cof_buff[105] =	READ_VOLTAGE_CALIBRATION_BETA.byte[3];
	
	READ_VOLTAGE_CALIBRATION_ALPHA.data = 1.2072;
	READ_VOLTAGE_CALIBRATION_BETA.data  = 3.2802;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_IWDG_Refresh(&hiwdg);
		if(flag.flag_BIT.finish_flag)
			{
				flag.flag_BIT.finish_flag=0;
				panel_send_buff.detail.finish_test =1;
				uint8_t data_14[16] = {first_frame_1 , first_frame_2, first_frame_3 ,1 , 4,0,0,0,0,0,0,0,0,second_frame_1 ,second_frame_2,second_frame_3};
				//PC_SEND(data,10);
				HAL_UART_Transmit(&huart1,data_14,16,100);
				reset_state();
			
			}
			if(flag.flag_BIT.panel_get_data_trigger)
				{
					if(wait_for_send_to_panel> 10)
						{
							panel_send();
							wait_for_send_to_panel =0;
							flag.flag_BIT.panel_get_data_trigger =0;
						}
					else
						{
							wait_for_send_to_panel++;
						}
				}
			if(flag.flag_BIT.send_to_panel_bus_busy)
				{
					if(wait_for_send_to_panel> 10)
						{
							panel_send();
							wait_for_send_to_panel =0;
							flag.flag_BIT.send_to_panel_bus_busy=0;
						}
					else
						{
							wait_for_send_to_panel++;
						}
				}
			if(flag.flag_BIT.send_50_50)
				{
					flag.flag_BIT.send_50_50 =0;
					uint8_t pc_data[10];
					pc_data[0] = 50;
					pc_data[1] = 50;

					delayMS(1000);	
					PC_SEND(pc_data,10);
					delayMS(10);
					PC_SEND(pc_data,10);
					delayMS(10);
					PC_SEND(pc_data,10);
					delayMS(10);
					PC_SEND(pc_data,10);
				
				}

		HAL_GPIO_TogglePin(LED_NET_DATA_GPIO_Port,LED_NET_DATA_Pin);
			LED_ON;
			HAL_Delay(100);
			LED_OFF;
			HAL_Delay(100);
  }
  /* USER CODE END 3 */
}
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 3000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = ENABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = ENABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 14200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 84;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 50-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}
/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 14200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 30000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
	// prescaler = 840 -1 
	// period = 100 -1
  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 840;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 14200;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8400;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 84;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65500;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */

static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 921600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_NET_DATA_GPIO_Port, LED_NET_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LAN_NRST_GPIO_Port, LAN_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : supply_ok_Pin */
//  GPIO_InitStruct.Pin = supply_ok_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(supply_ok_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_NET_DATA_Pin */
  GPIO_InitStruct.Pin = LED_NET_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_NET_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LAN_NRST_Pin */
  GPIO_InitStruct.Pin = LAN_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LAN_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD15 LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_15|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */





void initial_get_cof(void)
{
		uint8_t can_data[8] = {2,3,0,0 ,0,0,0,0};
	
		can_data[2] = get_coff_count;
		TIM6->CNT=0;
		timer_6_counter=0;
		
		if(get_coff_count ==4) //finish one module
			{
				get_coff_count =0;
				current_module++;
				if(current_module ==7) // finish getting coeffitent send to panel and run reset state
					{
						HAL_UART_Transmit(&huart4, cof_buff,110, 100);
						flag.flag_BIT.init_state_activated =0;
						reset_state();
					}
				else // ask for next module 
					{
						CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);
					}
			}
		else // ask for next coeffitent
			{
				CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);	
			}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
