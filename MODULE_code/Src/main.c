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
#include "AT24Cxx.h"
#include <FUNCTION.h>
#include "defines.h"
#include <stdio.h>
#include <stdlib.h>
//#include "TYPEDEF.h"



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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
// real parameter
uint16_t								test_1=0;



uint8_t 							odd_or_even=0;

uint16_t 							timer_9_waittime=0;
uint16_t							timer9_cnt=0;

uint16_t							voltage_buffer[MODE_BUFFER_SIZE];
uint16_t							current_buffer[MODE_BUFFER_SIZE];
STATE_MODE_t 										system_state;
STATE_count_t 									system_step;
uint32_t												battery_moving_sum=0;
uint32_t												shunt_moving_sum=0;
uint16_t												shunt_send_data,battery_send_data;
ADC_BUFFER_t										temp_adc1 , temp_adc2;
ADC_BUFFER_t										validate_shunt, validate_battery;
CAN_RxHeaderTypeDef 						rx_CAN_Init;
process_complete_buffer_t				process_buffer;	
process_complete_buffer_t				*process_buffer_ptr = &process_buffer;
process_excute_t								executing_command;
process_excute_t								*executing_command_ptr;
uint8_t													step_count;
float														passed_time;
uint32_t												time_counter=0;
uint32_t												wait_time=0;
recive_command_t								recive_command;
uint16_t												cnt1, cnt2;
static_parameter_ptr						*test_parameter_ptr;
STATE_count_t 									set_process_step= step1;
ADC_BUFFER_t										executing_adc_valu;
uint16_t												dac_change_value[DAC_CHANGE_STEP_COUNT];
send_data_t											send_data;
uint8_t 												dac_step_counter =1;
uint8_t 												fan_duty=10;
uint8_t													temp_spi[2];
uint8_t													temp_spi2[2];
uint8_t 												fan_read[100];
uint8_t													fan_counter=0;
MCU_FLAG_t											flag;
uint8_t 												battery_cnt;
uint8_t 												shunt_cnt;	
uint8_t													trigger_counter;
uint32_t												t_sum=0;

float_to_byte_t READ_VOLTAGE_CALIBRATION_ALPHA	;									 // alpha *x + bete
float_to_byte_t READ_VOLTAGE_CALIBRATION_BETA		;										// alpha *x + bete 
float_to_byte_t READ_SHUNT_CALIBRATION_ALPHA	;						            // alpha *x + bete
float_to_byte_t READ_SHUNT_CALIBRATION_BETA		;						           	// alpha *x + bete
float READ_temp_CALIBRATION_ALPHA	=1;						            // alpha *x + bete
float READ_temp_CALIBRATION_BETA		=0;						           	// alpha *x + bete
uint8_t var_test=0;
uint32_t												new_resistance=0;

uint16_t												CR_COUNTER = CR_PAR_COUNTER;
// test parameter
uint8_t can_data[8];
uint8_t													last_fan_value=0;	
float														last_cc_value;
float														last_cr_value;
float 													power_protection;
float														current_protection;
uint8_t													Error_code=0;
uint8_t													fna_turn_off_counter=0;
uint8_t													over_voltage_counter=0;
uint16_t 												fan_run=0;

uint32_to_byte									mainframe_sync;


//EEPROM TEST

uint8_t shift1 = 1<<1;
uint8_t shift2 = 1<<7;



float_to_byte_t 									mean_voltage ;
float_to_byte_t 									mean_shunt 	;
uint32_t 													time_10us_counter=0;
uint16_t 													time_reverse_cnt=0;
uint16_t													error_1ms_counter=0;
float 														test_flash_data=0;
uint8_t														not_syncronize_signal=0;
uint8_t 													temp=0;
uint8_t														timer6_counter;
uint8_t														timer6_compare_var;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM13_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */




uint8_t frequency_counter[17];

// ,uint8_t cur_or_volt for current = 2 for voltage =1 
uint16_t cal_mode(uint16_t buff[MODE_BUFFER_SIZE],uint8_t cur_or_volt)
{
	//register uint16_t fast_cnt ;
	//uint8_t frequency_counter[17];
	/// calculate mode range

register uint8_t 		fast_cnt;
register uint32_t 	sum_buff=0;
register uint8_t 		in_fast_cnt =1;
register uint8_t		mode_index=1;
register uint8_t		mode ;
	
uint32_t step_state = 	2000 *cur_or_volt ;
		for (fast_cnt =0; fast_cnt<MODE_BUFFER_SIZE;fast_cnt++)
			{
			for(in_fast_cnt =1; in_fast_cnt<=17;in_fast_cnt++)
				{
					if(buff[fast_cnt] < (in_fast_cnt * step_state))
						{
							frequency_counter[in_fast_cnt-1]++; 
							break;
						}
				}
			}
			
			// calculate mode itself
		//register uint8_t	mode =frequency_counter[0];
			if(frequency_counter[1] > frequency_counter[0])
				{
					mode =frequency_counter[1];	
				}
			else
				{
					mode =frequency_counter[0];
				}
		mode_index=2;
			uint8_t correct_num=0;
		for(fast_cnt =2; fast_cnt<17;fast_cnt++)
			{
				if(frequency_counter[fast_cnt] > mode)
					{
						mode =frequency_counter[fast_cnt];
						mode_index = fast_cnt+1;
					}
			}
			//uint32_t sum_buff=0;
		
			sum_buff =0;
		for(register uint8_t in_fast_cnt =0; in_fast_cnt <17;in_fast_cnt++)
					{
					frequency_counter[in_fast_cnt] = 0;
					}
			// select values in mode range
		for (fast_cnt =0; fast_cnt<MODE_BUFFER_SIZE;fast_cnt++)
			{
					if(buff[fast_cnt] < ((mode_index+1) * step_state) && buff[fast_cnt]> (mode_index-2) * step_state )
						{
							sum_buff = (buff[fast_cnt] + sum_buff);
							correct_num++;
							continue;
						}		
			}
			
//				float mean_correct ;
//	uint16_t correct_output;
//			mean_correct = (float)sum_buff/correct_num;
//			correct_output= mean_correct;
//			fast_cnt =0;
	
			return (uint16_t)(sum_buff/correct_num);
}	




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if(GPIO_Pin == GPIO_PIN_7) // sycronan timer
		{
			//time_counter++;
			error_1ms_counter =0;
			not_syncronize_signal =0;
			return;
		}
	else if(GPIO_Pin == GPIO_PIN_4) // FAN detect
		{
			if(flag.flag_BIT.FAN_IS_ON)
			{
				delayUS(5);
				if(HAL_GPIO_ReadPin(FAN_12V_GPIO_Port,FAN_12V_Pin)==1)
						{
							delayUS(5);
							if(HAL_GPIO_ReadPin(FAN_12V_GPIO_Port,FAN_12V_Pin)==1)
									{										
										MY_Error_HANDLE(FAN_TURN_OFF);
										Error_code =40;
										return;								
									}
						}
			}
		}
	else if(GPIO_Pin == GPIO_PIN_10) // GATE over Voltage 
		{
 			if(	flag.flag_BIT.check_gate_over_volage ==0 && flag.flag_BIT.wait_to_run_erro ==1)
			{
				MY_Error_HANDLE(gate_over_voltage);
				Error_code = gate_over_voltage;
				HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);	
				FAN_ON;
				fan_duty = 80;
				delayUS(10);	
				FAN_CONFIG(fan_duty);
				HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
				fna_turn_off_counter =0;
				flag.flag_BIT.fan_turn_off_flag =1;
				TIM4->CNT=0;
				__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
				__HAL_TIM_ENABLE(&htim4);									
				INIT_STATE();
			}
		}
	else if(GPIO_Pin == GPIO_PIN_13) // push buttom
		{
		
		}
}



	
	
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		 
		// 10 us timmer it,s better to enter this command in irq handler	
		// alternative slution is using timer3 
		if(htim->Instance == TIM5) //20u+s timer
			{
				//TIM5->CNT=0;

					if(flag.flag_BIT.start_process_counter ==1)
					{
						error_1ms_counter++;
						if(error_1ms_counter > 60)
							{
								error_1ms_counter =0;
								time_counter++;
								not_syncronize_signal++;
//								if(not_syncronize_signal == 20 && system_state == defult_test)
//									{
//										MY_Error_HANDLE(NOT_SYCRONIZE_SIGNAL);
//										Error_code =NOT_SYCRONIZE_SIGNAL;
//										return;	
//									}
							}
						}
						//HAL_GPIO_WritePin(Finder_FAN_GPIO_Port,Finder_FAN_Pin,GPIO_PIN_SET); 
						SPI3_ON;
						SPI1_ON;
						time_10us_counter++;
						HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING |  \
											CAN_IT_RX_FIFO1_MSG_PENDING 	);
						if(time_10us_counter >= 50 && flag.flag_BIT.wait_to_run_erro ==1)
							{						
								time_10us_counter=0;
						
								mean_voltage.data  = 	(READ_VOLTAGE_CALIBRATION_ALPHA.data* battery_send_data + READ_VOLTAGE_CALIBRATION_BETA.data );
								mean_shunt.data 	 = 	(READ_SHUNT_CALIBRATION_ALPHA.data  * shunt_send_data +READ_SHUNT_CALIBRATION_BETA.data);
								//
								if (one_min_error_cheak())
										{
											INIT_STATE();
											flag.flag_BIT.FAN_IS_ON =0;	
											for(uint8_t fan_cnt=0;fan_cnt <20;fan_cnt++)
												{
												
												//FAN_OFF;
												HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);	
												FAN_ON;
												fan_duty = 80;
												delayUS(10);	
												FAN_CONFIG(fan_duty);
												HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
												}

											fna_turn_off_counter =0;
											flag.flag_BIT.fan_turn_off_flag =1;
											TIM4->CNT=0;
											__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
											__HAL_TIM_ENABLE(&htim4);									
											HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING |  \
															CAN_IT_RX_FIFO1_MSG_PENDING 	);
											//HAL_NVIC_SystemReset();
											return;
										}
								HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING |  \
																					CAN_IT_RX_FIFO1_MSG_PENDING 	);
								
								
							}

						if(time_counter >= wait_time)
						{
							t_sum = t_sum +wait_time;
							time_counter = 0;
							
							dac_step_counter=1;
							wait_time 	 = set_process(&executing_command);
							
							
						}
						// 10 step rising 
						else
						{
						if(dac_step_counter<DAC_CHANGE_STEP_COUNT)
						{
									dac_step_counter++;
									executing_adc_valu.ADC_DATA = dac_change_value[dac_step_counter-1];							
									dac_set_value(executing_adc_valu);
						}
						else
						{
							register uint8_t temp_con = flag.flag_BIT.mode_bt_has_done + flag.flag_BIT.mode_cu_has_done;
							
							if(test_parameter_ptr->detail.operation==0) //cc
								{
									if((temp_con <2) && flag.flag_BIT.ask_data_trigger==0)
										{
											if(odd_or_even ==0 && time_10us_counter%2 ==0)
													{
														if(flag.flag_BIT.time_to_mode_bat)
															{
														
																flag.flag_BIT.time_to_mode_bat =0;
																battery_send_data = cal_mode(voltage_buffer,1);
																flag.flag_BIT.mode_bt_has_done =1;
																
															} 
														else
															{
																SPI3_OFF;
																delayUS(2);
																trigg_spi(&hspi3);
																return;	
															}										
													}
											else if(odd_or_even ==1 && time_10us_counter%2 ==1)
													{
														if(flag.flag_BIT.time_to_mode_cur)
															{
													
																flag.flag_BIT.time_to_mode_cur =0;
																shunt_send_data = cal_mode(current_buffer,2);
																flag.flag_BIT.mode_cu_has_done =1;
														
															}
														else
															{
															SPI1_OFF;					
															delayUS(2);
															trigg_spi(&hspi1);
															return;						
															}
													}
											}
								}
							else // CR
								{									
									if(CR_COUNTER==0)
									{								
										CR_COUNTER=CR_PAR_COUNTER;								
										executing_adc_valu.ADC_DATA =   (uint16_t)(	FULL_SCALE_GAIN*(mean_voltage.data/new_resistance) + beta_calibrition_coeffitiont_CC);											
										dac_set_value(executing_adc_valu);
									}
									if((temp_con <2) && flag.flag_BIT.ask_data_trigger==0)
										{		
											if(odd_or_even ==0 && time_10us_counter%2 ==0)
												{
														if(flag.flag_BIT.time_to_mode_bat)
															{
															
																flag.flag_BIT.time_to_mode_bat =0;
																battery_send_data = cal_mode(voltage_buffer,1);
																flag.flag_BIT.mode_bt_has_done =1;
																
															} 
														else
															{
																CR_COUNTER--;
																SPI3_OFF;								
																delayUS(2);
																trigg_spi(&hspi3);
															}
												}
											else if(odd_or_even ==1 && time_10us_counter%2 ==1)
												{
														if(flag.flag_BIT.time_to_mode_cur)
															{
															
																flag.flag_BIT.time_to_mode_cur =0;
																shunt_send_data = cal_mode(current_buffer,2);
																flag.flag_BIT.mode_cu_has_done =1;
																
															}
														else
															{
																CR_COUNTER--;
																SPI1_OFF;									
																delayUS(2);
																trigg_spi(&hspi1);			
															}
												}
											}
								}			
						}
						}
						if(flag.flag_BIT.ask_data_trigger)
							{
								if(trigger_counter >= w_20us_step)
									{
										flag.flag_BIT.mode_bt_has_done =0;
										flag.flag_BIT.mode_cu_has_done =0;
										flag.flag_BIT.ask_data_trigger =0;
										trigger_counter =0;
										send_data.detail.voltage_shunt		= shunt_send_data;//cal_mode(current_buffer);//shunt_send_data;//// // shunt_send_data;//shunt_voltage_ptr->ADC_DATA;// shunt_send_data;//shunt_voltage_ptr->ADC_DATA;///*shunt_voltage_ptr->ADC_DATA*/;
										send_data.detail.voltage_battery	=  battery_send_data;//cal_mode(voltage_buffer);//battery_send_data;//debug_variable;//	battery_send_data;//battery_voltage_ptr->ADC_DATA;///*battery_voltage_ptr->ADC_DATA*/;									
										CAN_SEND_DATA(send_data.send_buffer, MAIN_BOARD_ADDRESS,5);
									}
								else
									{
										trigger_counter++;
									}
									
							}	
						return;
					
			}
		else if(htim->Instance == TIM4) // FAN turn off 
				{
					
					// 20S FAN OFF
						if(flag.flag_BIT.fan_turn_off_flag ==1)
							{
								fna_turn_off_counter++;
								if(fna_turn_off_counter > 20)
									{
										flag.flag_BIT.FAN_IS_ON =0;
										fna_turn_off_counter =0;
										flag.flag_BIT.fan_turn_off_flag =0;
										FAN_OFF;
										HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
										TIM4->CNT=0;
										__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);				
										__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
										__HAL_TIM_DISABLE(&htim4);	
										HAL_TIM_Base_Stop_IT(&htim4);	
									}
							}
						else
							{
								HAL_TIM_Base_Stop_IT(&htim4);
							}
				}
		else if(htim->Instance == TIM6) // connection errror
				{
					timer6_counter++;
					if(timer6_counter > timer6_compare_var)
						{
							TIM4->CNT=0;
							fna_turn_off_counter =0;
							flag.flag_BIT.fan_turn_off_flag =1;
							__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
							__HAL_TIM_ENABLE(&htim4);		
							//1S comunication FAIL
							INIT_STATE();
						}
				}
		else if(htim->Instance == TIM9) // not respond during test
				{
					if(flag.flag_BIT.timer9_start)
					{
						timer9_cnt++;
						if(timer9_cnt > timer_9_waittime)
							{
								INIT_STATE();	
								timer9_cnt =0;
								fna_turn_off_counter=0;
								HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
								flag.flag_BIT.fan_turn_off_flag =1;
								__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
								__HAL_TIM_ENABLE(&htim4);		
								//__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
								TIM4->CNT=0;
								__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
								__HAL_TIM_ENABLE(&htim4);							
							}
					}
				}
			else if(htim->Instance == TIM10) // reverse voltage cheak
				{
					if(odd_or_even==1)
						{
							odd_or_even =0;
							SPI3_ON;
							delayUS(90);		
							SPI3_OFF;								
							delayUS(4);
							trigg_spi(&hspi3);
						}	
					else
					{
					odd_or_even= 1;
					}

						if(time_reverse_cnt>1000)
							{
								
								time_reverse_cnt =0;
								flag.flag_BIT.cheach_reverse_voltage_done =1;
								HAL_TIM_Base_Stop_IT(&htim10);
								flag.flag_BIT.check_gate_over_volage =1;
								DAC_OFF();
								DAC_OFF();
								delayMS(10);
								CONTACTOR_ON;
								delayMS(100);
								DAC_OFF();
								CONTACTOR_ON;
								delayMS(100);
								DAC_OFF();
								flag.flag_BIT.RV_not_check =1;
								// if online test
								if(flag.flag_BIT.reerse_before_start)
								{
									DAC_OFF();
									ADC_BUFFER_t temp;
									temp.ADC_DATA =0;
									dac_set_value(temp);
									timer9_cnt =0;
									timer_9_waittime = 20;
									flag.flag_BIT.timer9_start =1;
									HAL_TIM_Base_Start_IT(&htim9);
									trigger_rst();
									battery_moving_sum =0;
									shunt_moving_sum = 0;
									delayMS(100);
									if(HAL_GPIO_ReadPin(GATE_OF_ALARM_GPIO_Port,GATE_OF_ALARM_Pin)==0)
											{
												MY_Error_HANDLE(gate_over_voltage);
												Error_code = gate_over_voltage;
												return;
											}										
									
									TIM5->CNT =0;
									HAL_TIM_Base_Start_IT(&htim5);
											
											
									dac_set_value(executing_adc_valu);		
									uint8_t can_data[8];	
									can_data[1] = MODULE_BOARD_ADDRESS;
									can_data[0] = 4+ flag.flag_BIT.fire_or_not;
									can_data[2] = (4 );
									CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
								}
							
							}
				}
			
	}
	
	
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
	{		

	
	}
	
// Error 	
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)	
 {
	timer9_cnt =0;
	uint8_t rx_buffer[8];
	MY_RECIVE_FIFO1(hcan,CAN_RX_FIFO1,&rx_CAN_Init,rx_buffer);
	TIM6->CNT=0;
	if(rx_buffer[0]==0xFE &&	rx_buffer[1]==0xFE )
		{
			if(rx_buffer[2]==0xFE)
				{
					INIT_STATE();
					TIM4->CNT=0;
					fna_turn_off_counter =0;
					flag.flag_BIT.fan_turn_off_flag =1;
					__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
					__HAL_TIM_ENABLE(&htim4);		
					return;				
				}
			else	if(rx_buffer[2]==POWER_TURN_OFF)
				{
//					AT24Cxx_WriteEEPROM(20,&executing_command.small_buffer,40);
					// or
//					for(uint8_t fast_cnt=0;fast_cnt<40;fast_cnt++)
//						{
//							AT24_Write_8(20+fast_cnt,executing_command.small_buffer[fast_cnt]);
//						}
					INIT_STATE();
				}
		}
	else
		{
			switch(system_state)
				
			{	
			case defult_test:
							if(rx_buffer[0] == 3) 
									{
									//time_counter++;
										flag.flag_BIT.ask_data_trigger =1;
										error_1ms_counter=0;
										not_syncronize_signal=0;
										mainframe_sync.byte.byte_data[3] = rx_buffer[1];
										mainframe_sync.byte.byte_data[2] = rx_buffer[2];
										mainframe_sync.byte.byte_data[1] = rx_buffer[3];
										mainframe_sync.byte.byte_data[0] = rx_buffer[4];
										if(flag.flag_BIT.start_process_counter ==0)
											{
												dac_set_value(executing_adc_valu);
											}											
										flag.flag_BIT.start_process_counter =1;
										if(mainframe_sync.main_data >t_sum)
										{
										time_counter = mainframe_sync.main_data - t_sum;
										}
										else
											{
												time_counter++;
											}
									return;
									}	
							else if(rx_buffer[0] == 2) // SEND DATA
									{
											//delayUS(delay_sample_rate);
										
										flag.flag_BIT.ask_data_trigger =1;
										
//											send_data.detail.voltage_shunt		= shunt_send_data;//cal_mode(current_buffer);//shunt_send_data;//// // shunt_send_data;//shunt_voltage_ptr->ADC_DATA;// shunt_send_data;//shunt_voltage_ptr->ADC_DATA;///*shunt_voltage_ptr->ADC_DATA*/;
//											send_data.detail.voltage_battery	=  battery_send_data;//cal_mode(voltage_buffer);//battery_send_data;//debug_variable;//	battery_send_data;//battery_voltage_ptr->ADC_DATA;///*battery_voltage_ptr->ADC_DATA*/;									
//											CAN_SEND_DATA(send_data.send_buffer, MAIN_BOARD_ADDRESS,5);
										return;
									}
	 
									else if(rx_buffer[0] == 1 & rx_buffer[1] == 1 & rx_buffer[2] == 5 )
									{
										INIT_STATE();	
										TIM4->CNT=0;
										fna_turn_off_counter =0;
										flag.flag_BIT.fan_turn_off_flag =1;
									__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
									__HAL_TIM_ENABLE(&htim4);	
									return;
									}
							else// error
									{
									MY_Error_HANDLE(0);
									return;
									}
			
			case init:
					switch(system_step)
								{
									case step3:
												if(rx_buffer[0] == 1)
													{
															 
																			uint8_t    							zero_erro_count 		=0;
																			uint8_t    							one_erro_count 			=0;	
																			uint8_t									fan_read_local[200];
														
														
															if(flag.flag_BIT.start_level_counter ==0 & rx_buffer[1] == 0) // FAN cheack
																	{
																	// start FAN
																	// if fan is OK start_level_counter =1
																	// start time for respond time
																		flag.flag_BIT.FAN_IS_ON =0;
																			fna_turn_off_counter =0;
																			flag.flag_BIT.fan_turn_off_flag =0;
																			TIM4->CNT=0;
																			__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);				
																			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
																			__HAL_TIM_DISABLE(&htim4);
																			TIM6->CNT =0;
																			timer6_counter =0;
																			HAL_TIM_Base_Stop_IT(&htim6);
																			__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
																			FAN_ON;
																			fan_duty = 80;
																			FAN_CONFIG(fan_duty);
																			HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
																			for(uint16_t cnt=0; cnt<15000; cnt++)
																					{
																					delayUS(10000);
																					}
																			for(uint8_t cnt=0;cnt<200;cnt++)
																				{	
																					fan_read_local[cnt] = HAL_GPIO_ReadPin(FAN_DETECT_GPIO_Port,FAN_DETECT_Pin);	
																					delayUS(2000);
																				}
																				one_erro_count  =0;
																				zero_erro_count =0;
																				for(register uint8_t fast_cnt=0;fast_cnt<100;fast_cnt++)
																					{
																						if(fan_read_local[fast_cnt] ==1)
																							{
																								one_erro_count++;
																							}
																						else
																							{
																								zero_erro_count++;																	
																							}
																					}
																					fan_counter=0;
																					if((one_erro_count>80) ||(zero_erro_count >80))
																					{
																						// this mean fault
																						flag.flag_BIT.start_level_counter =0;
																						INIT_STATE();
																						uint8_t   can_data[8];
																						can_data[0] = 1;
																						can_data[1] = 2;
																						can_data[2] = MODULE_BOARD_ADDRESS;
																						can_data[3] = 0;
																						Error_code= FAN_TURN_OFF;
																						CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																						return;
																					}
																					else if(HAL_GPIO_ReadPin(FAN_12V_GPIO_Port,FAN_12V_Pin) == 1)
																						{
																							// this mean fault
																							
																							flag.flag_BIT.start_level_counter =0;
																							INIT_STATE();
																							uint8_t   can_data[8];
																							can_data[0] = 1;
																							can_data[1] = 2;
																							can_data[2] = MODULE_BOARD_ADDRESS;
																							can_data[3] = 0;
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);																						
																							Error_code= FAN_TURN_OFF;
																							return;																						
																						}		
																					else if(flag.flag_BIT.cheach_reverse_voltage_done ==0) // reverse voltage 
																					{
																							flag.flag_BIT.start_level_counter =0;
																							INIT_STATE();
																							uint8_t   can_data[8];
																							can_data[0] = 1;
																							can_data[1] = 2;
																							can_data[2] = MODULE_BOARD_ADDRESS;
																							can_data[3] = 2;
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);																						
																							Error_code= reverse_voltage;
																							return;		
																					}
																					else
																					{
																						#if timer_state
																							TIM6->CNT=0;
																							timer6_counter =0;	
																							timer6_compare_var = 10;																						
																							HAL_TIM_Base_Start_IT(&htim6);
																						#endif
																						
																						flag.flag_BIT.start_level_counter =1;
																						uint8_t   can_data[8];
																						can_data[0] = 1;
																						can_data[1] = 2;
																						can_data[2] = MODULE_BOARD_ADDRESS;
																						can_data[3] = 1;
																						CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);																																										
																					}		
																	}
																else if(flag.flag_BIT.start_level_counter ==1 & rx_buffer[1] == 1 ) // START PROCESS default test
																	{
																		
																		if(flag.flag_BIT.cheach_reverse_voltage_done==1)
																		{																		
																		trigger_rst();	
																		delayMS(10);	
																		if(HAL_GPIO_ReadPin(GATE_OF_ALARM_GPIO_Port,GATE_OF_ALARM_Pin)==0)
																			{
																				MY_Error_HANDLE(gate_over_voltage);
																				Error_code = gate_over_voltage;
																				return;
																			}		
																								
																		flag.flag_BIT.reerse_before_start=1;
																			timer6_counter =0;
																		HAL_TIM_Base_Stop_IT(&htim6);
																		__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
																		time_counter = 0;
																		power_protection 	 = OPP_PROTECTION;
																		current_protection = OCP_PROTECTION;
																			if(flag.flag_BIT.panel_send_process_s)
																			{
																			step_count =0;		
																			executing_command = process_buffer.process_buffer[0];
																			test_parameter_ptr->detail.operation=process_buffer.process_buffer[0].detail.test_mode;	
																			}
																			else
																			{
																		step_count =1;
																		executing_command = process_buffer.process_buffer[step_count];
																		test_parameter_ptr->big_ptr = process_buffer.process_buffer[0];
																			}
																			

																		// cheack see comand not end
																		flag.flag_BIT.start_level_counter=0;
																		battery_moving_sum =0;
																		shunt_moving_sum = 0;
																		
																		
																		timer9_cnt =0;
																		timer_9_waittime = 5;
																		flag.flag_BIT.timer9_start =1;	
																		HAL_TIM_Base_Start_IT(&htim9);
																		if(executing_command.detail.test_mode==2)//END
																			{
																			uint8_t data[2] = {0xFE, MODULE_BOARD_ADDRESS};	
																			TIM4->CNT=0;
																			fna_turn_off_counter =0;
																			flag.flag_BIT.fan_turn_off_flag =1;
																			HAL_TIM_Base_Start_IT(&htim4);
																			CAN_SEND_DATA(data,MAIN_FAULT_ADDRESS,2);
																			INIT_STATE();
																			delayUS(delay_sample_rate);
																			CAN_SEND_DATA(data,MAIN_FAULT_ADDRESS,2);
																			return;	
																			}
																		else if(executing_command.detail.timeA>0)
																		{
																			if(executing_command.detail.range ==1)
																				{
																					HAL_GPIO_WritePin(CONTROL_GAIN_GPIO_Port,CONTROL_GAIN_Pin,GPIO_PIN_RESET);
																				}
																			else
																				{
																					HAL_GPIO_WritePin(CONTROL_GAIN_GPIO_Port,CONTROL_GAIN_Pin,GPIO_PIN_SET);
																				}
																			system_state = defult_test;	
																			last_fan_value = 20;
																			TIM5->CNT =0;
																			time_counter  =0;	
																			flag.flag_BIT.FAN_IS_ON =1;
																			flag.flag_BIT.wait_to_run_erro =1;
																			HAL_TIM_Base_Start_IT(&htim5);
																			wait_time 	=executing_command.detail.timeA/50;
																			passed_time =executing_command.detail.timeA;
																			
																			if(test_parameter_ptr->detail.operation==0)//CC
																				{
																					DAC_CONFIG(executing_command.detail.valueA,0);
																				}
																			else //CR
																				{
																					DAC_CONFIG(executing_command.detail.valueA,100000);
																				}
																			dac_step_counter=1;																	
																			executing_adc_valu.ADC_DATA = dac_change_value[dac_step_counter-1];
																			//dac_set_value(executing_adc_valu);
																			time_counter  =0;	
																			}
																		}
																	else
																	{
																		Error_code =NOT_RIGHT_TIMER_SETTING;
																		INIT_STATE();
																		
																	}
																	}

													}
									break;
					
								default:
//									if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&rx_CAN_Init,rx_buffer)!= HAL_OK)
//									{
//										Error_Handler();
//									}
									//MY_Error_HANDLE(0);
								break;
								}
								
			}	
			
		}
	}

//data
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rx_buffer[8];
	TIM6->CNT=0;
	timer9_cnt =0;
	
		
	switch(system_state)
	{
		case init:
				switch(system_step)
						{
							case  step1: // command state
													MY_RECIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
													switch(rx_buffer[0])
															{
															case 0:// next
															{	
																flag.flag_BIT.panel_send_process_s =0;																
																uint8_t data[2] = {0, MODULE_BOARD_ADDRESS};
																#if timer_state
																	TIM6->CNT=0;
																	timer6_counter =0;
																	timer6_compare_var = 5;
																	HAL_TIM_Base_Start_IT(&htim6);
																#endif
																system_step    = step2;
																recive_command = next;
																cnt1	=	 0; 
																cnt2	 = 0;																
																CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,2);
																
															}	
															return;
												
															case 1: // start
																		{
																		time_reverse_cnt =0;
																		uint8_t data[2] = {MODULE_BOARD_ADDRESS, 0};	
																		HAL_GPIO_WritePin(Relay_output_GPIO_Port,Relay_output_Pin,GPIO_PIN_SET);
																		delayMS(100);
																		DAC_OFF();
																		
																		temp = HAL_GPIO_ReadPin(GATE_OF_ALARM_GPIO_Port,GATE_OF_ALARM_Pin);
																		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port,SPI3_CS_Pin,GPIO_PIN_SET);
																		delayUS(10);
																		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)==1) // not sycronize signal 
																				{
																					data[1] =3;
																				}	
																		else
																				{
																					data[1] = 0;//Error_cheack(); 
																				}
																		
																		if(data[1] !=0)// it means fualt
																			{
																				data[0] = 0xFF;
																				CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS, 2);
																			}
																		else
																			{
																				// not fault
																				TIM6->CNT=0;
																				#if timer_state
																					TIM6->CNT=0;
																				timer6_counter =0;
																					timer6_compare_var = 15;
																					HAL_TIM_Base_Start_IT(&htim6);
																				#endif
																				HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING |  \
																																							CAN_IT_RX_FIFO1_MSG_PENDING 	);
	
																				TIM10->CNT=0;
																				time_10us_counter=0;
																				__HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);				
																				HAL_TIM_Base_Stop_IT(&htim10);	
																				__HAL_TIM_ENABLE_IT(&htim10, TIM_IT_UPDATE);
																				__HAL_TIM_ENABLE(&htim10);		
																				system_step = step2;
																				recive_command = start;
																				CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS, 2);
																			}
																		}

															return;
															case 2: // enter calibration mode
																		{
																		uint8_t data[8];
																		data[0] =2;
																		data[1] =1;
																	
																		fna_turn_off_counter =0;
																		flag.flag_BIT.fan_turn_off_flag =0;

																		HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
																		TIM4->CNT=0;
																		__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);				
																		__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
																		__HAL_TIM_DISABLE(&htim4);	
																		HAL_TIM_Base_Stop_IT(&htim4);
																		flag.flag_BIT.FAN_IS_ON =0;
																		data[2] =0;// Error_cheack(); 
																		if(data[2]==0 & rx_buffer[1] == 1)
																			{
																				HAL_GPIO_WritePin(Relay_output_GPIO_Port,Relay_output_Pin,GPIO_PIN_SET);
																				delayMS(100);																				
																				DAC_OFF();
																				delayMS(10);
																				CONTACTOR_ON;
																				delayMS(100);	
																				flag.flag_BIT.reerse_before_start=1;
																				trigger_rst();
																				delayUS(10);
																				DAC_OFF();
																				timer9_cnt =0;
																				timer_9_waittime = 20;
																				flag.flag_BIT.timer9_start =1;
																				HAL_TIM_Base_Start_IT(&htim9);
																				system_state = calibration;
																				time_counter = 0;
																				wait_time = 40000000; 
																				executing_adc_valu.adc_buff[0] =rx_buffer[2];
																				executing_adc_valu.adc_buff[1] =rx_buffer[3];
																				dac_step_counter = DAC_CHANGE_STEP_COUNT;
																				power_protection 	 = OPP_PROTECTION;
																				current_protection = OCP_PROTECTION;				
																				executing_command.detail.test_mode =0;
																				FAN_ON;
																				
																				last_fan_value= 20;
																				fan_duty = 80;
																				FAN_CONFIG(fan_duty);
																				TIM4->CNT=0;
																				delayUS(2);
																				if(HAL_GPIO_ReadPin(GATE_OF_ALARM_GPIO_Port,GATE_OF_ALARM_Pin)==0)
																					{
																						data[2] = gate_over_voltage;
																						Error_code = gate_over_voltage;
																						CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);
																						return;
																					}																				
																				
																				if(HAL_GPIO_ReadPin(FAN_12V_GPIO_Port,FAN_12V_Pin)==1)
																					{
																						data[2] = FAN_TURN_OFF; 
																						Error_code =FAN_TURN_OFF;
																						CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);
																						return;	
																					}
																				
																				fna_turn_off_counter =0;
																				flag.flag_BIT.fan_turn_off_flag =0;
																				__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
																				HAL_TIM_Base_Stop_IT(&htim4);																				
																				HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
																				TIM5->CNT =0;
																				HAL_TIM_Base_Start_IT(&htim5);
																				delayUS(100);
																				flag.flag_BIT.check_gate_over_volage =0;
																				dac_set_value(executing_adc_valu);
																					
																				CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);
																				return;
																			}
																		else if(rx_buffer[1] == 3)
																			{
																				uint8_t can_data[8];
																				switch(rx_buffer[2])
																					{
																						case 0:
																							can_data[0] = 2;
																							can_data[1] = 3;
																							can_data[2] = 0;
																							can_data[3] = READ_SHUNT_CALIBRATION_ALPHA.byte[0];
																							can_data[4] = READ_SHUNT_CALIBRATION_ALPHA.byte[1];
																							can_data[5] = READ_SHUNT_CALIBRATION_ALPHA.byte[2];
																							can_data[6] = READ_SHUNT_CALIBRATION_ALPHA.byte[3];
																							can_data[7] = MODULE_BOARD_ADDRESS;
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																							return;
																						case 1:
																							can_data[0] = 2;
																							can_data[1] = 3;
																							can_data[2] = 1;
																							can_data[3] = READ_SHUNT_CALIBRATION_BETA.byte[0];
																							can_data[4] = READ_SHUNT_CALIBRATION_BETA.byte[1];
																							can_data[5] = READ_SHUNT_CALIBRATION_BETA.byte[2];
																							can_data[6] = READ_SHUNT_CALIBRATION_BETA.byte[3];

																							can_data[7] = MODULE_BOARD_ADDRESS;

																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																							return;
																						case 2:
																							can_data[0] = 2;
																							can_data[1] = 3;
																							can_data[2] = 2;
																							can_data[3] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[0];
																							can_data[4] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[1];
																							can_data[5] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[2];
																							can_data[6] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[3];
																							can_data[7] = MODULE_BOARD_ADDRESS;
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																							return;
																						case 3:
																							can_data[0] = 2;
																							can_data[1] = 3;
																							can_data[2] = 3;
																							can_data[3] = READ_VOLTAGE_CALIBRATION_BETA.byte[0];
																							can_data[4] = READ_VOLTAGE_CALIBRATION_BETA.byte[1];
																							can_data[5] = READ_VOLTAGE_CALIBRATION_BETA.byte[2];
																							can_data[6] = READ_VOLTAGE_CALIBRATION_BETA.byte[3];
																							can_data[7] = MODULE_BOARD_ADDRESS;
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																							return;
																						
																					}
																		}
																		else if(rx_buffer[1] == 6)
																			{
																				uint8_t data[8];
																				data[0] = 2;
																				data[1] = 6;
																				INIT_STATE();
																				//FAN_OFF;
																				
																				HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
																				CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);	
																				return;
																			}
																		
																	}
															case 3:
																	{
																		uint8_t data[8];
																			
																		CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);
																	}
															case 4: // online test
																	{
																	uint8_t can_data[8];	
																	can_data[1] = MODULE_BOARD_ADDRESS;
																	switch(rx_buffer[1])
																	{
																		case 1:// set
																			{
																				float_to_byte_t new_value;
																				flag.flag_BIT.online_test_config_cc =1;
																				flag.flag_BIT.online_test_config_cr =0;
																				flag.flag_BIT.panel_send_process_s =0;
																				test_parameter_ptr->detail.operation=0;	
																				can_data[0] = 1;
																				can_data[2] = 4;
																				new_value.byte[0] = rx_buffer[2];
																				new_value.byte[1] = rx_buffer[3];
																				new_value.byte[2] = rx_buffer[4];
																				new_value.byte[3] = rx_buffer[5];
																				executing_command.detail.valueA   =     new_value.data;
																				
//																				if(flag.flag_BIT.online_test_config_first_time)
//																					{			
																						time_counter = 0;																						
																						dac_step_counter = 1;
																						DAC_CONFIG(executing_command.detail.valueA,last_cc_value);
																						last_cc_value = executing_command.detail.valueA;	
																						executing_adc_valu.ADC_DATA = dac_change_value[dac_step_counter-1];
																						dac_set_value(executing_adc_valu);
//																					}
//																				else
//																					{
																						//cheack fault //*****************************
																						FAN_ON;

																						FAN_CONFIG(last_fan_value);
																						TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
																						__HAL_TIM_ENABLE(&htim14);
																						HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
																						fna_turn_off_counter =0;
																						flag.flag_BIT.fan_turn_off_flag =0;																						
																						__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
																						__HAL_TIM_DISABLE(&htim4);
																							
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																				timer9_cnt =0;
																				timer_9_waittime = 20;
																				flag.flag_BIT.timer9_start =1;
																				__HAL_TIM_CLEAR_FLAG(&htim9, TIM_SR_UIF);
																				__HAL_TIM_ENABLE_IT(&htim9, TIM_IT_UPDATE);
																				__HAL_TIM_ENABLE(&htim9);
																			}
																			
																			return;
																		case 2://Set CR
																			{
																				float_to_byte_t new_value;
																				flag.flag_BIT.online_test_config_cc =0;
																				flag.flag_BIT.online_test_config_cr =1;
																				
																				new_value.byte[0] = rx_buffer[2];
																				new_value.byte[1] = rx_buffer[3];
																				new_value.byte[2] = rx_buffer[4];
																				new_value.byte[3] = rx_buffer[5];
																				can_data[0] = 2;
																				can_data[2] = 4;
																				executing_command.detail.valueA =  new_value.data;
																				test_parameter_ptr->detail.operation=1;
																				delayUS(10);
//																				if(flag.flag_BIT.online_test_config_first_time)
//																					{			
																						time_counter = 0;
																						dac_step_counter = 1;
																						DAC_CONFIG(executing_command.detail.valueA,last_cr_value);
																						last_cr_value = executing_command.detail.valueA;
																						executing_adc_valu.ADC_DATA = dac_change_value[dac_step_counter-1];
																						dac_set_value(executing_adc_valu);
																						
//																					}
//																				else
//																					{
																						//cheack fault //*****************************
																						FAN_ON;

																						FAN_CONFIG(last_fan_value);
																						TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
																						__HAL_TIM_ENABLE(&htim14);
																						HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
																						//__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);	

//																					}
																				timer9_cnt =0;
																				timer_9_waittime = 20;
																				flag.flag_BIT.timer9_start =1;
																				__HAL_TIM_CLEAR_FLAG(&htim9, TIM_SR_UIF);
																				__HAL_TIM_ENABLE_IT(&htim9, TIM_IT_UPDATE);
																				__HAL_TIM_ENABLE(&htim9);
																			CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																			}																			
																		
																			return;
																		case 3: // stop
																			{	
																			flag.flag_BIT.FAN_IS_ON=0;
																			INIT_STATE();	
																			TIM4->CNT=0;
																			fna_turn_off_counter =0;
																			flag.flag_BIT.fan_turn_off_flag =1;
																			flag.flag_BIT.online_test_config_first_time =0;
																			__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
																			__HAL_TIM_ENABLE(&htim4);	
																			can_data[0] = 3;
																			can_data[2] = 4;																			
																			CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																			}																			
																			return;
																		case 4: // start
																			{
																				flag.flag_BIT.wait_to_run_erro =0;
																				flag.flag_BIT.panel_send_process_s =0;
																				HAL_GPIO_WritePin(Relay_output_GPIO_Port,Relay_output_Pin,GPIO_PIN_SET);
																				flag.flag_BIT.run_or_stop =1; 
																				flag.flag_BIT.FAN_IS_ON =0;		
																				flag.flag_BIT.reerse_before_start=1;		
																				flag.flag_BIT.mode_bt_has_done = 0;
																				flag.flag_BIT.mode_cu_has_done= 0;
																				flag.flag_BIT.time_to_mode_bat =0;
																				flag.flag_BIT.time_to_mode_cur =0;
																				flag.flag_BIT.ask_data_trigger=0;
																				time_reverse_cnt =0;
																				time_reverse_cnt =0;
																				process_buffer.process_buffer[2].detail.test_mode=2;
																				wait_time 	 = 4000000000; 
																				passed_time	 = 4000000000;
																				dac_step_counter = 1;
																				time_counter = 0;
																				step_count = 1;
																				fan_run =5000;
																				executing_command.detail.test_mode =0;
																				flag.flag_BIT.online_test_config_first_time =1;
																				FAN_ON;
																				fan_duty = 80;
																				last_fan_value =20;
																				FAN_CONFIG(fan_duty);	
																				flag.flag_BIT.fire_or_not=0;
																				TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
																				__HAL_TIM_ENABLE(&htim14);																				
																				HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);																	
																				if(flag.flag_BIT.online_test_config_cc) //CC
																					{																				
																						last_cc_value = executing_command.detail.valueA;																																										
																						DAC_CONFIG(executing_command.detail.valueA,0);

																					}
																				else //CR
																					{
																						last_cr_value = executing_command.detail.valueA;
																						DAC_CONFIG(executing_command.detail.valueA,100000);																																											
																					}

																				can_data[0] = 4;
																				can_data[2] = 4;	
																			delayMS(500);
																			if(HAL_GPIO_ReadPin(FAN_12V_GPIO_Port,FAN_12V_Pin)==1)
																				{
																					MY_Error_HANDLE(FAN_TURN_OFF); 
																					Error_code =FAN_TURN_OFF;
																					return;	
																				}
																			executing_adc_valu.ADC_DATA = dac_change_value[dac_step_counter-1];
																			TIM10->CNT =0;
																			__HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);			
																			over_voltage_counter =0;																				
																			HAL_TIM_Base_Stop_IT(&htim10);	
																			HAL_TIM_Base_Start_IT(&htim10);
																			//CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																			fna_turn_off_counter =0;
																			flag.flag_BIT.fan_turn_off_flag =0;																			
																			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
																			__HAL_TIM_DISABLE(&htim4);
																			}																			
																			return;
																		case 5: // fire and start
																			flag.flag_BIT.wait_to_run_erro =0;
//																			{	
//																			{
//																			HAL_GPIO_WritePin(Relay_output_GPIO_Port,Relay_output_Pin,GPIO_PIN_SET);																			 
//																			flag.flag_BIT.FAN_IS_ON =0;																				
//																			flag.flag_BIT.reerse_before_start=1;																		
//																			time_reverse_cnt =0;
//																			flag.flag_BIT.time_to_mode_bat =0;
//																			flag.flag_BIT.time_to_mode_cur =0;
//																			flag.flag_BIT.run_or_stop =1;
//																			flag.flag_BIT.ask_data_trigger=0;
//																			time_reverse_cnt =0;
//																			process_buffer.process_buffer[2].detail.test_mode=2;
//																			wait_time 	 = 4000000000; 
//																			passed_time	 = 4000000000;
//																			dac_step_counter = 1;
//																			time_counter = 0;
//																			step_count = 1;
//																			flag.flag_BIT.mode_bt_has_done = 0;
//																			flag.flag_BIT.mode_cu_has_done= 0;
//																			executing_command.detail.test_mode =0;
//																			flag.flag_BIT.online_test_config_first_time =1;
//																			FAN_ON;
//																			fan_duty = 80;
//																			last_fan_value =20;
//																			FAN_CONFIG(fan_duty);	
//																			TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
//																			__HAL_TIM_ENABLE(&htim14);																				
//																			HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);																			
//																			if(flag.flag_BIT.online_test_config_cc) //CC
//																			{																				
//																			last_cc_value = executing_command.detail.valueA;																																										
//																			DAC_CONFIG(executing_command.detail.valueA,0);
//																			}
//																			else //CR
//																			{
//																			last_cr_value = executing_command.detail.valueA;
//																			DAC_CONFIG(executing_command.detail.valueA,100000);																																											
//																			}
//																			//																					uint8_t can_data[8];	
//																			//																		can_data[1] = MODULE_BOARD_ADDRESS;
//																			can_data[0] = 4;
//																			can_data[2] = 4;	
//																			delayMS(500);
//																			if(HAL_GPIO_ReadPin(FAN_12V_GPIO_Port,FAN_12V_Pin)==1)
//																			{
//																			MY_Error_HANDLE(FAN_TURN_OFF); 
//																			Error_code =FAN_TURN_OFF;
//																			return;	
//																			}
//																			executing_adc_valu.ADC_DATA = dac_change_value[dac_step_counter-1];
//																			TIM10->CNT =0;
//																			__HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);				
//																			HAL_TIM_Base_Stop_IT(&htim10);	
//																			HAL_TIM_Base_Start_IT(&htim10);
//																			//CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
//																			fna_turn_off_counter =0;
//																			flag.flag_BIT.fan_turn_off_flag =0;
//																			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
//																			__HAL_TIM_DISABLE(&htim4);
//																			}
//																			}		
																			{
																			HAL_GPIO_WritePin(Relay_output_GPIO_Port,Relay_output_Pin,GPIO_PIN_SET);
																			flag.flag_BIT.run_or_stop =1; 
																		flag.flag_BIT.FAN_IS_ON =0;		
																		
																		flag.flag_BIT.reerse_before_start=1;		
																		flag.flag_BIT.mode_bt_has_done = 0;
																		flag.flag_BIT.mode_cu_has_done= 0;
																				flag.flag_BIT.time_to_mode_bat =0;
																				flag.flag_BIT.time_to_mode_cur =0;
																				flag.flag_BIT.ask_data_trigger=0;
																		time_reverse_cnt =0;
																				time_reverse_cnt =0;
																				process_buffer.process_buffer[2].detail.test_mode=2;
																				wait_time 	 = 4000000000; 
																				passed_time	 = 4000000000;
																				dac_step_counter = 1;
																				time_counter = 0;
																				step_count = 1;
																				fan_run =5000;
																				executing_command.detail.test_mode =0;
																				flag.flag_BIT.online_test_config_first_time =1;
																				FAN_ON;
																				fan_duty = 80;
																				last_fan_value =20;
																				FAN_CONFIG(fan_duty);	
																				flag.flag_BIT.fire_or_not=1;
																				
	
																				TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
																				__HAL_TIM_ENABLE(&htim14);																				
																						HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
																				
																				if(flag.flag_BIT.online_test_config_cc) //CC
																					{																				
																						last_cc_value = executing_command.detail.valueA;																																										
																						DAC_CONFIG(executing_command.detail.valueA,0);

																					}
																				else //CR
																					{
																						last_cr_value = executing_command.detail.valueA;
																						DAC_CONFIG(executing_command.detail.valueA,100000);																																											
																					}
//																					uint8_t can_data[8];	
//																	can_data[1] = MODULE_BOARD_ADDRESS;
																				can_data[0] = 4;
																				can_data[2] = 5;	

																				
																			delayMS(500);
																			if(HAL_GPIO_ReadPin(FAN_12V_GPIO_Port,FAN_12V_Pin)==1)
																				{
																					MY_Error_HANDLE(FAN_TURN_OFF); 
																					Error_code =FAN_TURN_OFF;
																					return;	
																				}
																							
											
																			executing_adc_valu.ADC_DATA = dac_change_value[dac_step_counter-1];
																				
																				

																			TIM10->CNT =0;
																			__HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);				
																			HAL_TIM_Base_Stop_IT(&htim10);	
																			HAL_TIM_Base_Start_IT(&htim10);
																			//CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																			fna_turn_off_counter =0;
																			flag.flag_BIT.fan_turn_off_flag =0;
																			
																			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
																			__HAL_TIM_DISABLE(&htim4);
																			}
																			return;
																		case 6:
																			{
																				
																				float_to_byte_t new_value;
																				new_value.byte[0] = rx_buffer[2];																			
																				FAN_ON;
																				fan_duty = 80;
																				//last_fan_value =20;
																				FAN_CONFIG(fan_duty);		
																				TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
																				__HAL_TIM_ENABLE(&htim14);																				
																				HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);																			
																				new_value.byte[1] = rx_buffer[3];
																				new_value.byte[2] = rx_buffer[4];
																				new_value.byte[3] = rx_buffer[5];
																				current_protection = new_value.data;
																				can_data[0] = 6;
																				can_data[2] = 4;	
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);

																			}																			
																			return;
																		case 7:
																			{

																				
																				float_to_byte_t new_value;
																				new_value.byte[0] = rx_buffer[2];																		
																				new_value.byte[1] = rx_buffer[3];
																				new_value.byte[2] = rx_buffer[4];
																				new_value.byte[3] = rx_buffer[5];
																				power_protection = new_value.data;																		
																				can_data[0] = 7;
																				can_data[2] = 4;	
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																			}																			
																			return;
																		case 8:	
																			{
																				flag.flag_BIT.mode_bt_has_done = 0;
																				flag.flag_BIT.mode_cu_has_done= 0;
																				flag.flag_BIT.time_to_mode_bat =0;
																				flag.flag_BIT.time_to_mode_cur =0;
																				flag.flag_BIT.ask_data_trigger=0;
																				flag.flag_BIT.wait_to_run_erro =1;
																				ADC_BUFFER_t send_buffer_1 ;
																				ADC_BUFFER_t send_buffer_2 ;
																				send_buffer_1.ADC_DATA = shunt_send_data;//cal_mode(current_buffer,2); 
																				send_buffer_2.ADC_DATA = battery_send_data;//cal_mode(voltage_buffer,1);
																				flag.flag_BIT.check_gate_over_volage =0;
																				can_data[0] = 8;
																				can_data[2] = 4;
																				can_data[3] = send_buffer_1.adc_buff[0];
																				can_data[4] = send_buffer_1.adc_buff[1];
																				can_data[5] = send_buffer_2.adc_buff[0];
																				can_data[6] = send_buffer_2.adc_buff[1];
																				can_data[7] = flag.flag_BIT.run_or_stop;
																				flag.flag_BIT.FAN_IS_ON =1;
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
				
																			
																			}																			
																			return;
																		case 9: // online test send calibration coeffitiont
																			{
																			
																				switch(rx_buffer[2])
																				{
																						case 0:
																							can_data[0] = 9;
																							
																							can_data[2] = 0;
																							can_data[3] = READ_SHUNT_CALIBRATION_ALPHA.byte[0];
																							can_data[4] = READ_SHUNT_CALIBRATION_ALPHA.byte[1];
																							can_data[5] = READ_SHUNT_CALIBRATION_ALPHA.byte[2];
																							can_data[6] = READ_SHUNT_CALIBRATION_ALPHA.byte[3];
																							can_data[7] = MODULE_BOARD_ADDRESS;
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																							return;
																						case 1:
																							can_data[0] = 9;
																							
																							can_data[2] = 1;
																						
																							can_data[3] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[0];
																							can_data[4] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[1];
																							can_data[5] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[2];
																							can_data[6] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[3];
//																							can_data[3] = READ_SHUNT_CALIBRATION_BETA.byte[0];
//																							can_data[4] = READ_SHUNT_CALIBRATION_BETA.byte[1];
//																							can_data[5] = READ_SHUNT_CALIBRATION_BETA.byte[2];
//																							can_data[6] = READ_SHUNT_CALIBRATION_BETA.byte[3];	

																							can_data[7] = MODULE_BOARD_ADDRESS;

																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																							return;
																						case 2:
																							can_data[0] = 9;
																							
																							can_data[2] = 2;

																							can_data[3] = READ_SHUNT_CALIBRATION_BETA.byte[0];
																							can_data[4] = READ_SHUNT_CALIBRATION_BETA.byte[1];
																							can_data[5] = READ_SHUNT_CALIBRATION_BETA.byte[2];
																							can_data[6] = READ_SHUNT_CALIBRATION_BETA.byte[3];
																							can_data[7] = MODULE_BOARD_ADDRESS;
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																							return;
																						case 3:
																							can_data[0] = 9;
																							
																							can_data[2] = 3;
																							can_data[3] = READ_VOLTAGE_CALIBRATION_BETA.byte[0];
																							can_data[4] = READ_VOLTAGE_CALIBRATION_BETA.byte[1];
																							can_data[5] = READ_VOLTAGE_CALIBRATION_BETA.byte[2];
																							can_data[6] = READ_VOLTAGE_CALIBRATION_BETA.byte[3];
																							can_data[7] = MODULE_BOARD_ADDRESS;
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																							return;
																						
																					}
																			
																			
																			}
																	}
																}
															case 5:
																	{
																		//set process for panel comunication
																		uint8_t data[2] = {5, MODULE_BOARD_ADDRESS};
																		#if timer_state
																			TIM6->CNT=0;
																			timer6_counter =0;
																			timer6_compare_var = 5;
																			HAL_TIM_Base_Start_IT(&htim6);
																		#endif
																		system_step    = step2;
																		recive_command = panel_set_proccess;
																		timer6_compare_var =20;
																		cnt1	=	 0; 
																		cnt2	 = 0;										
																		process_buffer_ptr = &process_buffer;
																		test_parameter_ptr->big_ptr = process_buffer.process_buffer[0];
																		CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,2);
																		return;
																	}
															}
																				
								break;
							case  step2:
												switch(recive_command)
														{
														case next:
																{
																	
																MY_RECIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,(&process_buffer_ptr->receive_buffer+cnt1));

																		cnt1 +=8;  // increment poinnter location 


																	cnt2++;		 // procees get count
																		uint8_t can_data[2] = {0 , MODULE_BOARD_ADDRESS};
																		if(cnt2==process_count)
																			{
																				system_state	= init;
																				system_step   = step1;
																				timer6_counter =0;
																				HAL_TIM_Base_Stop_IT(&htim6);
																				__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
																				process_buffer_ptr = &process_buffer;
																				test_parameter_ptr->big_ptr = process_buffer.process_buffer[0];
																			}
																		else
																				{
																					delayUS(10);
																					CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,2);
																				}
																			}																			
														return;
														case start:
																{
																	
																	
																	MY_RECIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer);
																	
																	
																	
																	//cheack if fan and contactor are on 
																	TIM6->CNT=0;				
																	if(rx_buffer[0] ==1)
																	{
																		system_step   = step3;
																	}
																	//HAL_Delay(5);
																	
																	//CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,2);
																}
															return;
														case panel_set_proccess:
																{
																		MY_RECIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,(&process_buffer_ptr->receive_buffer+cnt1));

																		cnt1 +=8;  // increment poinnter location 
																		cnt2++;		 // procees get count
																		uint8_t can_data_5[2] = {5 , MODULE_BOARD_ADDRESS};
																		if(cnt2==5)
																			{
																				process_excute_t temp;
																				
																				system_state	= init;
																				system_step   = step1;
																				timer6_counter =0;
																				HAL_TIM_Base_Stop_IT(&htim6);
																				__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
																				delayUS(100);
																				executing_command = process_buffer.process_buffer[0];
																				flag.flag_BIT.panel_send_process_s =1;
																				CAN_SEND_DATA(can_data_5,MAIN_BOARD_ADDRESS,2);
																				process_buffer.process_buffer[1].detail.test_mode=2;
																				process_buffer_ptr = &process_buffer;
																				step_count =1;
																				if(process_buffer.process_buffer[0].detail.test_mode==0)
																					{
																					test_parameter_ptr->detail.operation=1 ;// CR
																					}
																				else
																					{
																					test_parameter_ptr->detail.operation=0 ;//cc
																					}																					
																				//test_parameter_ptr->big_ptr = process_buffer.process_buffer[0];
																					
																					return;
																			}
																		else
																				{
																					delayUS(100);
																					timer6_counter =0;
																					CAN_SEND_DATA(can_data_5,MAIN_BOARD_ADDRESS,2);
																					return;
																				}
																}
														default:
															MY_RECIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,(&process_buffer_ptr->receive_buffer+cnt1));

														
														}
								break;
							case  step3:
												switch(recive_command)
														{
														case next:
																MY_RECIVE_FIFO0(hcan,CAN_RX_FIFO0,&rx_CAN_Init,(&process_buffer_ptr->receive_buffer+cnt1));

														break;
														case start:
																	// this step shulden't executed
																	TIM8->CNT =0;
																	
																	executing_command = process_buffer.process_buffer[1];
																	if(executing_command.detail.timeA>0)
																	{
																		system_state = defult_test;
																		
																		wait_time =executing_command.detail.timeA/50;
																		// SET value A;
																	}
																	// start process }
															return;
														default:
															break;
														
														
														}			
							
							
								break;
							case	step4:
								
								break;
						}
			break;
		case defult_test:
															if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
															{
																Error_Handler();
															}
			break;
		case online_test:
																if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
															{
																Error_Handler();
															}
			break;
		case calibration:
															if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_CAN_Init,rx_buffer)!= HAL_OK)
															{
																Error_Handler();
															}
															if(rx_buffer[0]==2)
															{
															switch(rx_buffer[1])
																{
																case	1: // dac set value
																{
																		uint8_t data[8];
																		data[0] =2;
																		data[1] =1;
																		data[2] = 0/*Error_cheack()*/; 
																		if(data[2]==0)
																			{
																			power_protection 	 = OPP_PROTECTION;
																			current_protection = OCP_PROTECTION;	
																			time_counter = 0;
																			wait_time = 40000000; 
																			dac_step_counter = DAC_CHANGE_STEP_COUNT;
																			executing_adc_valu.adc_buff[0] =rx_buffer[2];
																			executing_adc_valu.adc_buff[1] =rx_buffer[3]; 
																				timer6_counter =0;
																			HAL_TIM_Base_Stop_IT(&htim6);
																			__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);		
																			executing_command.detail.test_mode =0;
																			FAN_ON;
																			CONTACTOR_ON;	
																			last_fan_value= 20;
																			fan_duty = 80;
																			
																			fna_turn_off_counter =0;
																			flag.flag_BIT.fan_turn_off_flag =0;
																				timer9_cnt =0;
																				timer_9_waittime = 20;
																				flag.flag_BIT.timer9_start =1;
																				HAL_TIM_Base_Start_IT(&htim9);
																			__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);	
																			HAL_TIM_Base_Stop_IT(&htim4);
																			FAN_CONFIG(fan_duty);
																			//__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);	
																			HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
																			//__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);	
																			TIM5->CNT =0;
																			if(HAL_GPIO_ReadPin(FAN_12V_GPIO_Port,FAN_12V_Pin)==1)
																					{
																						MY_Error_HANDLE(FAN_TURN_OFF); 
																						Error_code =FAN_TURN_OFF;
																						data[2] = FAN_TURN_OFF;
																						CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);
																						INIT_STATE();
																						return;	
																					}
																				
																			HAL_TIM_Base_Start_IT(&htim5);
																			//__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);	
																			//HAL_TIM_Base_Start_IT(&htim3);
																			
																			dac_set_value(executing_adc_valu);
																			CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);
																			}
																		else
																			{
																			//fault exist	
																			CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);
																			}
																	}
																	return;
																case 2: // data
																{
																	uint8_t can_data[8];
																	flag.flag_BIT.check_gate_over_volage =0;
																	flag.flag_BIT.FAN_IS_ON=1;			
																	switch(rx_buffer[2])
																		{
																			case 0:
																				can_data[0] = 2;
																				can_data[1] = 2;
																				can_data[2] = 0;
																				can_data[3] = mean_shunt.byte[0];
																				can_data[4] = mean_shunt.byte[1];
																				can_data[5] = mean_shunt.byte[2];
																				can_data[6] = mean_shunt.byte[3];
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																				return;
																			case 1:
																				can_data[0] = 2;
																				can_data[1] = 2;
																				can_data[2] = 1;
																				can_data[3] = mean_voltage.byte[0];
																				can_data[4] = mean_voltage.byte[1];
																				can_data[5] = mean_voltage.byte[2];
																				can_data[6] = mean_voltage.byte[3];
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																				return;
																			case 2:
																				can_data[0] = 2;
																				can_data[1] = 2;
																				can_data[2] = 2;
																				*(uint16_t *)(can_data + 3) = shunt_send_data;
																				*(uint16_t *)(can_data + 5) = battery_send_data;
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																				return;
																			case 3:
																				can_data[0] = 2;
																				can_data[1] = 2;
																				can_data[2] = 2;
																				*(uint16_t *)(can_data + 3) = shunt_send_data;
																				*(uint16_t *)(can_data + 5) = battery_send_data;
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																				return;
																		}
																		return;
																}
																	
																case 3:// cofeitient
																{
																	uint8_t can_data[8];
																	switch(rx_buffer[2])
																		{
																			case 0:
																				can_data[0] = 2;
																				can_data[1] = 3;
																				can_data[2] = 0;
																				can_data[3] = READ_SHUNT_CALIBRATION_ALPHA.byte[0];
																				can_data[4] = READ_SHUNT_CALIBRATION_ALPHA.byte[1];
																				can_data[5] = READ_SHUNT_CALIBRATION_ALPHA.byte[2];
																				can_data[6] = READ_SHUNT_CALIBRATION_ALPHA.byte[3];
																				//*(float*)(can_data+3)		= READ_SHUNT_CALIBRATION_ALPHA;
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																				return;
																			case 1:
																				can_data[0] = 2;
																				can_data[1] = 3;
																				can_data[2] = 1;
																				can_data[3] = READ_SHUNT_CALIBRATION_BETA.byte[0];
																				can_data[4] = READ_SHUNT_CALIBRATION_BETA.byte[1];
																				can_data[5] = READ_SHUNT_CALIBRATION_BETA.byte[2];
																				can_data[6] = READ_SHUNT_CALIBRATION_BETA.byte[3];
																				//*(float*)(can_data+3)	= READ_VOLTAGE_CALIBRATION_ALPHA ; 

																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																				return;
																			case 2:
																				can_data[0] = 2;
																				can_data[1] = 3;
																				can_data[2] = 2;
																			
																				can_data[3] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[0];
																				can_data[4] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[1];
																				can_data[5] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[2];
																				can_data[6] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[3];

																				//*(float*)(can_data+3)		=  READ_SHUNT_CALIBRATION_BETA;
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																				return;
																			case 3:
																				can_data[0] = 2;
																				can_data[1] = 3;
																				can_data[2] = 3;
																				can_data[3] = READ_VOLTAGE_CALIBRATION_BETA.byte[0];
																				can_data[4] = READ_VOLTAGE_CALIBRATION_BETA.byte[1];
																				can_data[5] = READ_VOLTAGE_CALIBRATION_BETA.byte[2];
																				can_data[6] = READ_VOLTAGE_CALIBRATION_BETA.byte[3];
																				//*(float*)(can_data+3)		=  READ_VOLTAGE_CALIBRATION_BETA;
																				CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																				return;
																			
																		}
																		}
																	return;
																case 4: // submit value
																				{
																					uint8_t can_data[8];
																					can_data[0] = 2; 
																					can_data[1] = 4;
																					switch(rx_buffer[2])
																					{
																						case 0:
																							READ_VOLTAGE_CALIBRATION_ALPHA.byte[0] = rx_buffer[3];
																							READ_VOLTAGE_CALIBRATION_ALPHA.byte[1] = rx_buffer[4];
																							READ_VOLTAGE_CALIBRATION_ALPHA.byte[2] = rx_buffer[5];
																							READ_VOLTAGE_CALIBRATION_ALPHA.byte[3] = rx_buffer[6];
																						
																							AT24_Write_8(0,READ_VOLTAGE_CALIBRATION_ALPHA.byte[0]);
																							AT24_Write_8(1,READ_VOLTAGE_CALIBRATION_ALPHA.byte[1]);
																							AT24_Write_8(2,READ_VOLTAGE_CALIBRATION_ALPHA.byte[2]);
																							AT24_Write_8(3,READ_VOLTAGE_CALIBRATION_ALPHA.byte[3]);
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																						return;
																						case 1:
																							READ_VOLTAGE_CALIBRATION_BETA.byte[0] = rx_buffer[3];
																							READ_VOLTAGE_CALIBRATION_BETA.byte[1] = rx_buffer[4];
																							READ_VOLTAGE_CALIBRATION_BETA.byte[2] = rx_buffer[5];
																							READ_VOLTAGE_CALIBRATION_BETA.byte[3] = rx_buffer[6];
																							//READ_VOLTAGE_CALIBRATION_BETA = *(float*)(rx_buffer+3);
																						
																							AT24_Write_8(4,READ_VOLTAGE_CALIBRATION_BETA.byte[0]);
																							AT24_Write_8(5,READ_VOLTAGE_CALIBRATION_BETA.byte[1]);
																							AT24_Write_8(6,READ_VOLTAGE_CALIBRATION_BETA.byte[2]);
																							AT24_Write_8(7,READ_VOLTAGE_CALIBRATION_BETA.byte[3]);

																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																						return;
																						case 2:
																							READ_SHUNT_CALIBRATION_ALPHA.byte[0] = rx_buffer[3];
																							READ_SHUNT_CALIBRATION_ALPHA.byte[1] = rx_buffer[4];
																							READ_SHUNT_CALIBRATION_ALPHA.byte[2] = rx_buffer[5];
																							READ_SHUNT_CALIBRATION_ALPHA.byte[3] = rx_buffer[6];
																							//READ_SHUNT_CALIBRATION_ALPHA = *(float*)(rx_buffer+3);
																							

																							AT24_Write_8(8 ,READ_SHUNT_CALIBRATION_ALPHA.byte[0]);
																							AT24_Write_8(9 ,READ_SHUNT_CALIBRATION_ALPHA.byte[1]);
																							AT24_Write_8(10,READ_SHUNT_CALIBRATION_ALPHA.byte[2]);
																							AT24_Write_8(11,READ_SHUNT_CALIBRATION_ALPHA.byte[3]);

																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																						return;
																						
																						case 3:
																							READ_SHUNT_CALIBRATION_BETA.byte[0] = rx_buffer[3];
																							READ_SHUNT_CALIBRATION_BETA.byte[1] = rx_buffer[4];
																							READ_SHUNT_CALIBRATION_BETA.byte[2] = rx_buffer[5];
																							READ_SHUNT_CALIBRATION_BETA.byte[3] = rx_buffer[6];
																							//READ_SHUNT_CALIBRATION_BETA =*(float*)(rx_buffer+3);
																							AT24_Write_8(12 ,READ_SHUNT_CALIBRATION_BETA.byte[0]);
																							AT24_Write_8(13 ,READ_SHUNT_CALIBRATION_BETA.byte[1]);
																							AT24_Write_8(14,READ_SHUNT_CALIBRATION_BETA.byte[2]);
																							AT24_Write_8(15,READ_SHUNT_CALIBRATION_BETA.byte[3]);
																							CAN_SEND_DATA(can_data,MAIN_BOARD_ADDRESS,8);
																						return;

																						
																					}
																					
																				}
																	return;
																case 5: // dac off
																			{
																				uint8_t data[8];
																				data[0] = 2;
																				data[1] = 5;
																				DAC_OFF();
																				fna_turn_off_counter =0;
																				flag.flag_BIT.fan_turn_off_flag =1;
																				TIM4->CNT=0;
																				__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
																				__HAL_TIM_ENABLE(&htim4);																					
																				
																				HAL_TIM_Base_Stop_IT(&htim5);
																				__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
																				HAL_TIM_Base_Stop_IT(&htim5);
																				CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);
																			}
																	return;
																case 6: // exit calibration
																		{
																			uint8_t data[8];
																			data[0] = 2;
																			data[1] = 6;
																			INIT_STATE();
																			FAN_OFF;
																			HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
																			CAN_SEND_DATA(data,MAIN_BOARD_ADDRESS,8);
																			
																		}
																	return;
																
																}
															}
			break;
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
	START_STATE();
	send_data.detail.firstdata = get_data_first_buff;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_TIM13_Init();
  MX_SPI3_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	test_flash_data=0;

HAL_Delay(100);
DAC_OFF();
DAC_OFF();
// *********  only fpr test purposes
test_1 = DAC_LIMITATION;
// ************

AT24_Read_8(0,&READ_VOLTAGE_CALIBRATION_ALPHA.byte[0]);
AT24_Read_8(1,&READ_VOLTAGE_CALIBRATION_ALPHA.byte[1]);
AT24_Read_8(2,&READ_VOLTAGE_CALIBRATION_ALPHA.byte[2]);
AT24_Read_8(3,&READ_VOLTAGE_CALIBRATION_ALPHA.byte[3]);


AT24_Read_8(4,&READ_VOLTAGE_CALIBRATION_BETA.byte[0]);
AT24_Read_8(5,&READ_VOLTAGE_CALIBRATION_BETA.byte[1]);
AT24_Read_8(6,&READ_VOLTAGE_CALIBRATION_BETA.byte[2]);
AT24_Read_8(7,&READ_VOLTAGE_CALIBRATION_BETA.byte[3]);


AT24_Read_8(8 ,&READ_SHUNT_CALIBRATION_ALPHA.byte[0]);
AT24_Read_8(9 ,&READ_SHUNT_CALIBRATION_ALPHA.byte[1]);
AT24_Read_8(10,&READ_SHUNT_CALIBRATION_ALPHA.byte[2]);
AT24_Read_8(11,&READ_SHUNT_CALIBRATION_ALPHA.byte[3]);

AT24_Read_8(12,&READ_SHUNT_CALIBRATION_BETA.byte[0]);
AT24_Read_8(13,&READ_SHUNT_CALIBRATION_BETA.byte[1]);
AT24_Read_8(14,&READ_SHUNT_CALIBRATION_BETA.byte[2]);
AT24_Read_8(15,&READ_SHUNT_CALIBRATION_BETA.byte[3]);

		HAL_TIM_Base_Start(&htim7);
		HAL_SPI_Receive_IT(&hspi1,temp_spi,2);
		HAL_SPI_Receive_IT(&hspi3,temp_spi2,2);
{
	flag.flag_BIT.check_gate_over_volage 	=1;
	uint8_t zero_data[2] ={0,0};
	HAL_SPI_Transmit(&hspi2,zero_data,2,10);

	HAL_SPI_Transmit(&hspi2,zero_data,2,10);
	HAL_Delay(1);
		// turn on power ciruit
		HAL_GPIO_WritePin(GATE_OF_RST_GPIO_Port,GATE_OF_RST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GATE_OF_RST_GPIO_Port,GATE_OF_RST_Pin, GPIO_PIN_SET);
		
	
	
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port,SPI3_CS_Pin,GPIO_PIN_SET);
	
		// turn dac off
		for(uint8_t cnt=0;cnt<10;cnt++)
			{
				DAC_OFF();
			}

		// turn on power ciruit
		HAL_GPIO_WritePin(GATE_OF_RST_GPIO_Port,GATE_OF_RST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GATE_OF_RST_GPIO_Port,GATE_OF_RST_Pin, GPIO_PIN_SET);
		
		// turn dac off
		for(uint8_t cnt=0;cnt<10;cnt++)
			{
				DAC_OFF();
			}
		HAL_Delay(100);
	}	

		delayUS(200);
		CAN_INIT();
		SPI1_OFF;
		SPI3_OFF;
		HAL_TIM_Base_Start_IT(&htim10);
		INIT_STATE();
		
		for(uint8_t cnt=0;cnt<100;cnt++)
			{
				DAC_OFF();
			}
		DAC_OFF();
HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);	
//		
temp = HAL_GPIO_ReadPin(GATE_OF_ALARM_GPIO_Port,GATE_OF_ALARM_Pin);
//HAL_GPIO_WritePin(Relay_output_GPIO_Port,Relay_output_Pin,GPIO_PIN_SET);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */

static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
	//************************************************************************************
// 1ms timer for ERRORCHEACK
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 108-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
		//************************************************************************************
// 20 S timer for FAN 
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 43200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
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

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 108;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20-1;
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
	//************************************************************************************
	// 1S timer for comunication with mainframe 
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10800;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
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
  htim7.Init.Prescaler = 1;
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
	//************************************************************************************
	// 10us timer  for proccess 
  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 108;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 21600;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 10000-1;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 108;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 40-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */
	//************************************************************************************
	// FAN PWM 
  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 5400-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 1000;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 54;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

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
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|FINDER_CONTACTOR_Pin|Finder_FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Relay_output_Pin|CONTROL_GAIN_Pin|GATE_OF_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1_Pin */
  GPIO_InitStruct.Pin = PB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin FINDER_CONTACTOR_Pin Finder_FAN_Pin */
  GPIO_InitStruct.Pin = LED_Pin|FINDER_CONTACTOR_Pin|Finder_FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_12V_Pin */
  GPIO_InitStruct.Pin = FAN_12V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAN_12V_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_output_Pin CONTROL_GAIN_Pin GATE_OF_RST_Pin */
  GPIO_InitStruct.Pin = Relay_output_Pin|CONTROL_GAIN_Pin|GATE_OF_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_DETECT_Pin */
  GPIO_InitStruct.Pin = FAN_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAN_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AX_CON_Pin */
  GPIO_InitStruct.Pin = AX_CON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AX_CON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GATE_OF_ALARM_Pin */
  GPIO_InitStruct.Pin = GATE_OF_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GATE_OF_ALARM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */






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
