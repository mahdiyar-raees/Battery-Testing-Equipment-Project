/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <defines.h>
#include <FUNCTION.h>
#include "AT24Cxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern 													uint8_t 							command_state;
extern													uint8_t 							rx_command_code[command_buffer_size];
//extern 													uint8_t 						command_code[command_buffer_size];
extern 													uint8_t 							static_buffer[static_buffersize];
extern 							           	technical_array_t 		technical_buffer;
extern													uint16_t 							command_counter;
extern													MCU_FLAG_t						flag;	
extern													uint8_t 							send_finish_counter;
extern 													STATE_MODE_t 					system_state;
extern													uint8_t 							calibration_buffer[10];
extern													uint8_t								calibration_cof_buffer[22];
extern													uint8_t								counter_calibration;
extern													uint8_t 							get_data_buffer[38];
extern													ADC_DATA_t						ADC_DATA[2];
// panel variable 
extern 													uint8_t								panel_command_counter;
extern													uint8_t								panel_command_buffer[20];
extern													uint8_t								panel_comand_for_module[40];
extern													panel_buff_t					panel_send_buff;
extern													uint8_t								panel_send_process_counter;
extern													uint16_t 							detenator_time;
extern													uint16_t 							detenator_value;
extern													uint8_t 					wait_for_send_to_panel;
extern													uint8_t	 panel_bus_busy;
extern													uint8_t	 PC_bus_busy;

extern			uint16_t 							timer_2_counter;
extern			uint16_t 							timer_6_counter;
extern			uint16_t 							timer_7_counter;
extern			uint16_t 							timer_3_counter;
extern			uint16_t 							timer_4_counter;
extern			float_to_byte_t READ_VOLTAGE_CALIBRATION_ALPHA	;
extern			float_to_byte_t READ_VOLTAGE_CALIBRATION_BETA		;
extern 			ADC_DATA_t							  ADC_DATA[2];
uint8_t			panel_available_board[6];
			#if test
			extern uint8_t rx_buffer[8];
			#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
void RX_function(uint8_t x);
void PANEL_COMMAND_handeler(uint8_t x);
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
__HAL_CAN_CLEAR_FLAG(&hcan1,CAN_FLAG_FF0);
	HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
	return;
  /* USER CODE END CAN1_RX0_IRQn 0 */
 // HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */
__HAL_CAN_CLEAR_FLAG(&hcan1,CAN_FLAG_FF1);
	HAL_CAN_RxFifo1MsgPendingCallback(&hcan1);
	return;
  /* USER CODE END CAN1_RX1_IRQn 0 */
  //HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
		uint8_t x;	
		x = USART1->DR;
		RX_function(x);
		return;
  /* USER CODE END USART1_IRQn 0 */
  //HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
		uint8_t x;	
		x = USART2->DR;
		RX_function(x);
		return;
	
	
	
  /* USER CODE END USART2_IRQn 0 */
 // HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	PANEL_COMMAND_handeler(UART4->DR);
	return;
  /* USER CODE END UART4_IRQn 0 */
  //HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_DAC_IRQHandler(&hdac);
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

__HAL_CAN_CLEAR_FLAG(&hcan2,CAN_FLAG_FF0);
	HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
	return;
  /* USER CODE END CAN2_RX0_IRQn 0 */
  //HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX1 interrupt.
  */
void CAN2_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX1_IRQn 0 */

__HAL_CAN_CLEAR_FLAG(&hcan2,CAN_FLAG_FF1);
	HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
	return;

  /* USER CODE END CAN2_RX1_IRQn 0 */
  //HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX1_IRQn 1 */

  /* USER CODE END CAN2_RX1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void PANEL_COMMAND_handeler(uint8_t x)
	{
		if(flag.flag_BIT.panel_ask_for_data)
			{
				if(flag.flag_BIT.panel_send_process==0) // for sending proccess (2 step)
					{
						if(panel_command_counter ==0)
							{
								if(x == panel_start_1)
									{
										//flag.flag_BIT.flag_ask_for_data = 0;
										panel_command_buffer[0] = x;
										panel_command_counter++;
										#if WAIT_RESPOND_TIME
										TIM3->CNT=0;
										timer_3_counter=0;
										__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);	
										HAL_TIM_Base_Start_IT(&htim3);
										#endif
									}
								else
									{									
										panel_command_counter=0;
									}
							}
						else if(panel_command_counter ==1)
							{
								if(x == panel_start_2)
									{
										panel_command_buffer[1] =x;
										panel_command_counter++;										
									}
								else
									{
										panel_command_counter=0;
									}
							}
						else
							{
								panel_command_buffer[panel_command_counter] =x;
								panel_command_counter++;
								if(panel_command_counter ==20)
									{
										
										//flag.flag_BIT.panel_ask_for_data= 0;
										panel_command_counter =0;
										if(panel_command_buffer[19] == panel_end_2 & panel_command_buffer[18] == panel_end_1)
											{
												__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
												HAL_TIM_Base_Stop_IT(&htim3);
												panel_send_buff.detail.fault_or_command = panel_command_buffer[2];
												switch(panel_command_buffer[2])
													{
													case 1: // start		
														if(system_state == init && PC_bus_busy == 0) // *** this part should be recheak because it is a default test 
														{ 
															delayMS(3000);
															panel_send_buff.detail.error_type =0;
															// we should be in init state
															// detect module
																HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
															panel_bus_busy =1;
															panel_send_buff.detail.finish_test =0;
															flag.flag_BIT.panel_is_running=1;		
															delayMS(3000);		
														
															for(uint8_t cnt=0;cnt<6; cnt++)
																{
																	if(panel_command_buffer[cnt+3] ==1)
																		{
																			available_module[cnt] =1;
																			panel_available_board[cnt] =1;
																		}
																	else
																		{
																		available_module[cnt] =0;
																		}
																}																
																execut_command = 1;
																start_counter =0;
																time_counter.main_data=0;
																while(1)
																	{
																		if(available_module[start_counter]==1)
																			{
																				uint8_t data[1] ={1};
																				#if WAIT_RESPOND_TIME
																				TIM6->CNT =0;
																				timer_6_counter=0;
																				HAL_TIM_Base_Start_IT(&htim6);
																				#endif
																				current_module =start_counter+1;
																				CAN_SEND_DATA(data,return_module_address(start_counter),1);
																				return;
																			}
																		else
																			{
																			start_counter++;
																			if(start_counter==6) // there is no module we expected code would never reach here 		
																				{
																					// no module is selected
																					// send hand shake to PC
																					reset_state();
																					start_counter=0;
																					return;
																				}
																			}
																		}													
																	
														 				
														}
														else // bust is busy
														{
														wait_for_send_to_panel = 0;	
														flag.flag_BIT.send_to_panel_bus_busy =1;
														panel_send_buff.detail.type_or_ok = 1;
														
														return;	
														}
													case 2: // stop														
														{
														
														}											
														return;
													case 3:	// force stop
														{															
															{
																uint8_t can_data[8] = {0xFE, 0xFE,0xFE,0xFE, 0xFE,0xFE,0xFE,0xFE};
																CAN_SEND_DATA(can_data,module_commen_address,8);
															}
															reset_state();
															uint8_t pc_data2[10];
															pc_data2[0] = 50;
															pc_data2[1] = 50;															
															PC_SEND(pc_data2,10);
															flag.flag_BIT.panel_ask_for_data= 1;
															panel_send_buff.detail.type_or_ok = OK;
															delayMS(3000);
															delayMS(3000);
															panel_send();
														}
														return;
													case 4:	// get data
														if(PC_bus_busy == 0)
														{
															// request data
															flag.flag_BIT.panel_ask_for_data= 1;
															if(flag.flag_BIT.process_runnig)
																{																	
																	panel_send_buff.detail.type_or_ok =OK;
																	for(register uint8_t fast_cnt =0;fast_cnt <24;fast_cnt++)
																		{
																			panel_send_buff.detail.data[fast_cnt] = get_data_buffer[fast_cnt +5];
																		}
																	panel_send_buff.detail.data[24] =	ADC_DATA[0].adc_little_buff[1];
																	panel_send_buff.detail.data[25] = ADC_DATA[0].adc_little_buff[0];
					
																		wait_for_send_to_panel =0;
																		flag.flag_BIT.panel_get_data_trigger =1;
															
																}
															else // NO process running
																{
																	panel_send_buff.detail.type_or_ok =OK;
																	delayMS(6000);
																	panel_send();
																}
														}
														else
														{
														wait_for_send_to_panel = 0;	
														flag.flag_BIT.send_to_panel_bus_busy =1;
														panel_send_buff.detail.type_or_ok = 1;
														
														}
														return;
													case 5: // set profile
															if(system_state == init && PC_bus_busy == 0)
																{
																	// we should be in init state
																	// detect module															
																	flag.flag_BIT.panel_ask_for_data= 1;
																	panel_send_buff.detail.type_or_ok =OK;
																		panel_bus_busy =1;
																	for(uint8_t cnt=0;cnt<6; cnt++)
																		{
																			if(panel_command_buffer[cnt+3] ==1)
																				{
																					available_module[cnt] =1;
																				}
																			else
																				{
																				available_module[cnt] =0;
																				}
																		}	
																		panel_send_process_counter =0;																
																		flag.flag_BIT.panel_send_process =1;																		
																		delayMS(6000);
																		#if WAIT_RESPOND_TIME
																			TIM6->CNT =0;
																			timer_2_counter =0;
																			HAL_TIM_Base_Start_IT(&htim2);
																		#endif	
																		panel_send();
																		return;
																	// set prof module
																}
														else 
															{
																wait_for_send_to_panel = 0;	
																flag.flag_BIT.send_to_panel_bus_busy =1;
																panel_send_buff.detail.type_or_ok = 1;
														
															}
														return;
													case 6:
														if(system_state == init && PC_bus_busy == 0)
															{ // get coff
															execut_command =5;	
															flag.flag_BIT.init_state_activated =1;
															uint8_t can_data[8] = {2,3,0,0,0,0,0,0};
															get_coff_count =0;
															current_module =1;															
															timer_6_counter =0;
															TIM6->CNT =0;
															HAL_TIM_Base_Start_IT(&htim6);
															CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);
															}
														return;
													}
											}
									}
							}
						
					}
				else // PANEL SENDING PROCESS
					{
						technical_buffer.rx_buffer[panel_command_counter+5] =x;
						panel_command_counter++;
						if(panel_command_counter == 40)
							{
								timer_2_counter=0;
								HAL_TIM_Base_Stop_IT(&htim2);
								__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
								panel_command_counter = 0;
								U16_to_U8_t temp;
								temp.small_pack[0] 	= technical_buffer.rx_buffer[34]; // set detenator value 
								temp.small_pack[1] 	= technical_buffer.rx_buffer[35]; // 
								detenator_value 	 	= temp.big_data;
								temp.small_pack[0] 	= technical_buffer.rx_buffer[36]; // 
								temp.small_pack[1] 	= technical_buffer.rx_buffer[37]; // 
								detenator_time 		 	= temp.big_data;
								// for sending data to Module board
								technical_buffer.rx_buffer[10] = 0; // sure
								technical_buffer.rx_buffer[10] = 0;
								technical_buffer.rx_buffer[10] = 0;
								technical_buffer.rx_buffer[10] = 0;
								execut_command =70; // for panel send proccess
								// for geting next command
								flag.flag_BIT.panel_send_process =0;
								for(uint8_t fast_cnt =0; fast_cnt<7;fast_cnt++)
									{
										if(available_module[fast_cnt])
											{
												current_module = fast_cnt+1;
												// SET module to get station
												break;
											}
										else if(fast_cnt ==5)
											{
												reset_state();
												return;
											}
									}
								uint8_t can_data[8];
								can_data[0] = 5;
								#if WAIT_RESPOND_TIME
										TIM6->CNT =0;
										timer_6_counter=0;
										HAL_TIM_Base_Start_IT(&htim6);
								#endif
								CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);
								
							}
							
					}
			}
		else
			{
			// you count send command now  
				//send respond to panel
			
			}
	}


void RX_function(uint8_t x)
{
		TIM2->CNT=0;
		timer_2_counter=0;
		if(panel_bus_busy ==0)
		{
		
		switch(system_state)
			{
			case init:

									switch (command_state)
										{
										case 0: // step 1;
											if(command_counter==0)
												{
													if(x=='S')
														{
															//flag.flag_BIT.panel_ask_for_data =0;
															rx_command_code[command_counter] =x;
															command_counter++;
															#if WAIT_RESPOND_TIME
															TIM2->CNT=0;
															timer_2_counter=0;
															__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);	
															HAL_TIM_Base_Start_IT(&htim2);
															#endif
														}
													else
														{
														command_counter =0;	
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
											else if(command_counter==1)
												{
													if(x=='T')
														{
															rx_command_code[command_counter] =x;
															command_counter++;
														}
													else
														{
														command_counter =0;												
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
											else
											{
											rx_command_code[command_counter] =x;
											command_counter++;
											if(command_counter==command_buffer_size)
												{													
													timer_2_counter =0;
													HAL_TIM_Base_Stop_IT(&htim2);
													__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);	
													command_counter=0;
													if((rx_command_code[0]=='S'&rx_command_code[1]=='T'& rx_command_code[command_buffer_size-2]=='E'&rx_command_code[command_buffer_size-1]=='N'))
															{
																//flag.flag_BIT.flag_ask_for_data =0;
																command_handler(rx_command_code[2]);
																return;
															}
													else
															{
															uint8_t 	data[4] = {0xFF , 0xFF , 0,NOT_VALID_COMMAND} ;
												
															PC_SEND(data,10);
															if(system_state != defult_test)
															{
															reset_state();
															}
															else
															{
															comunication_reset();	
															}
															return;
															}
												}

											}
										break;
										case 1:// step 2
												
													static_buffer[command_counter] = x;
													command_counter++;

													TIM2->CNT =0;
													
													if(command_counter==static_buffersize)
													{
															command_counter =0;
															timer_2_counter =0;
															HAL_TIM_Base_Stop_IT(&htim2);
															__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);	
														if((static_buffer[0]=='S'&static_buffer[1]=='T'&static_buffer[static_buffersize-2]=='E'&static_buffer[static_buffersize-1]=='N'))
															{
																uint8_t PC_data_PTR[2];
																PC_data_PTR[0] = execut_command;
																PC_data_PTR[1] = command_state;
																detect_module_address( &static_buffer[5],static_buffer[4]);
																//AT24Cxx_WriteEEPROM(0x00,static_buffer,static_buffersize);
																#if WAIT_RESPOND_TIME
																TIM2->CNT =0;
																__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
																HAL_TIM_Base_Start_IT(&htim2);
																#endif
																
																U16_to_U8_t temp;
																temp.small_pack[0] 	= static_buffer[89]; // test
																temp.small_pack[1] 	= static_buffer[90]; // test
																detenator_value 	 	= temp.big_data;
																temp.small_pack[0] 	= static_buffer[91]; // test
																temp.small_pack[1] 	= static_buffer[92]; // test
																detenator_time 		 	= temp.big_data;
																PC_SEND(PC_data_PTR,10);
																command_state = 2;
																return;
																//}
															}
															else 
															{
															uint8_t 	data[4] = {0xFF ,0xFF , 0,NOT_VALID_COMMAND} ;
															PC_SEND(data,10);
															if(system_state != defult_test)
															{
															reset_state();
															}
														else
															{
															comunication_reset();	
															}
															
															return;
															}
														}
										break;
										case 2:// step 3
												TIM2->CNT=0;
												timer_2_counter=0;
												technical_buffer.rx_buffer[command_counter] =x;
												command_counter++;				
												if(command_counter ==technical_buffer_size)
												{
													command_counter=0;
													HAL_TIM_Base_Stop_IT(&htim2);
													__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
													if((technical_buffer.rx_buffer[0]=='S' & technical_buffer.rx_buffer[1]=='T'& technical_buffer.rx_buffer[technical_buffer_size-2]=='E' & technical_buffer.rx_buffer[technical_buffer_size-1]=='N'))
													{

														if(technical_buffer.rx_buffer[4] !=0)
																	{
																	
																		//flag.flag_BIT.flag_ask_for_data=0;
																		TIM6->CNT=0;
																		#if WAIT_RESPOND_TIME	
																		timer_6_counter=0;
																		HAL_TIM_Base_Start_IT(&htim6);
																		#endif
																		uint8_t can_data[2] = {0,0};
																		current_module =technical_buffer.rx_buffer[4];
																		CAN_SEND_DATA(can_data,return_module_address(current_module-1),2);
																		return;
																	}
															else
																	{
																		// if number of terminal equal to zero
																		uint8_t 	PC_data_ptr[2];
																		PC_data_ptr[0] = 0;
																		PC_data_ptr[1] = 2;
																		send_finish_counter++;
																				{
																					TIM2->CNT=0;
																					timer_2_counter=0;
																					HAL_TIM_Base_Start_IT(&htim2);
																					PC_SEND(PC_data_ptr,10);
																					if(send_finish_counter==6)
																						{
																						PC_SEND(PC_data_ptr,10);	
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
																		return;
																	
																	}
	
													}
													else 
														{
														uint8_t 	data[4] = { 0xFF ,0xFF,0 , NOT_VALID_COMMAND};

														PC_SEND(data,10);	
														if(system_state != defult_test)
															{
															reset_state();
															}
														else
															{
															comunication_reset();	
															}
														return;
														}
												}
										break;
											}
								
			
			
			
				break;
			case calibration:
								
								// 	0  	module command 
								switch (command_state)
										{
											case 0:
														if(command_counter==0)
																	{
																		#if WAIT_RESPOND_TIME
																			TIM2->CNT=0;
																			timer_2_counter=0;
																			__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);	
																			HAL_TIM_Base_Start_IT(&htim2);
																		#endif
																		if(x=='S')
																			{
																				calibration_buffer[command_counter] =x;
																				command_counter++;

																			}
																		else
																			{
																				command_counter =0;														
																			}												
																	}
																else if(command_counter==1)
																	{
																		if(x=='T')
																			{
																				TIM2->CNT=0;
																				timer_2_counter=0;
																				calibration_buffer[command_counter] =x;
																				command_counter++;
																			}
																		else
																			{
																				command_counter =0;												
																			}
																	}
																else
																	{
																	calibration_buffer[command_counter] =x;
																	TIM2->CNT=0;
																		timer_2_counter=0;
																	command_counter++;	
																	if(command_counter==command_buffer_size)
																	{
																		HAL_TIM_Base_Stop_IT(&htim2);
																			if((calibration_buffer[0]=='S' & calibration_buffer[1]=='T'& calibration_buffer[command_buffer_size-2]=='E' & calibration_buffer[command_buffer_size-1]=='N'))
																				{
																				//flag.flag_BIT.flag_ask_for_data =0;
																				command_counter=0;
																				if(calibration_buffer[2] ==	 67 )
																				{
																					uint8_t PC_data[4];
																					flag.flag_BIT.flag_ask_for_data =1;
																					PC_data[0] = 67;
																					PC_data[1] = 78;
																					PC_data[2] = 2;
																					PC_SEND(PC_data,10);
																				}
																				else if(calibration_buffer[2] != 2)
																				{
																					uint8_t PC_data[4];
																					flag.flag_BIT.flag_ask_for_data =1;
																					PC_data[0] = 0xFF;
																					PC_data[1] = 78;
																					PC_data[2] = 2;
																					if(system_state != defult_test)
																						{
																						reset_state();
																						}
																					else
																						{
																						comunication_reset();	
																						}
																					PC_SEND(PC_data,10);
																				
																				}
																				else if(calibration_buffer[2] == 2)
																						{
																						switch(calibration_buffer[3])
																							{
																								case 1:	// set dac value 
																								{																									
																									if(calibration_buffer[4] <7)
																										{
																											uint8_t data[8] = {0,0,0,0,0,0,0,0};
																											data[0] = 2; 
																											data[1] = 1;
																											data[2] = calibration_buffer[5];
																											data[3] = calibration_buffer[6];
																											cal_sub_menu=1;
																											TIM6->CNT=0;
																											#if WAIT_RESPOND_TIME
																											//__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
																											timer_6_counter=0;
																											HAL_TIM_Base_Start_IT(&htim6);
																											#endif
																											current_module = calibration_buffer[4];
																											CAN_SEND_DATA(data,return_module_address(current_module-1),8);
																										}
																									else // main frame set dac value 
																										{
																											uint8_t data[10] = {0,0,0,0,0,0,0,0,0,0};
																											data[0] = 2;
																											data[1] = 1;
																											data[2] = 7;
																											U16_to_U8_t temp;
																											temp.small_pack[0] 	= calibration_buffer[5]; // test
																											temp.small_pack[1] 	= calibration_buffer[6]; // test
																											detenator_value 	 	= temp.big_data;
																											dac_set_value(detenator_value);
																											flag.flag_BIT.flag_ask_for_data =1;
																											PC_SEND(data,10);
																										}
																								}
																									return;
																								case 2: // request data
																								{
																									if(calibration_buffer[4] <7)
																										{
																											counter_calibration =0;
																											cal_sub_menu =2;
																											uint8_t data[8]= {0,0,0,0,0,0,0,0};
																											data[0] = 2;
																											data[1] = 2;
																											data[2] = counter_calibration;
																											TIM6->CNT=0;
																											timer_6_counter=0;
																											
																											current_module = calibration_buffer[4];
																											#if WAIT_RESPOND_TIME
																											__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
																											HAL_TIM_Base_Start_IT(&htim6);
																											#endif
																											CAN_SEND_DATA(data,return_module_address(current_module-1),8);
																										}
																									else // main frame request data
																										{
																											uint8_t cal_22_data[20];
																											cal_22_data[0] = 2;
																											cal_22_data[1] = 2;
																											float_to_byte_t temp;
																											temp.data = (READ_VOLTAGE_CALIBRATION_ALPHA.data * ADC_DATA[0].ADC_MAIN )  + READ_VOLTAGE_CALIBRATION_BETA.data;
																											
																											cal_22_data[2] = temp.byte[0];
																											cal_22_data[3] = temp.byte[1];
																											cal_22_data[4] = temp.byte[2];
																											cal_22_data[5] = temp.byte[3];
																											// 6 7 8 9  zero
																											cal_22_data[10] = ADC_DATA[0].adc_little_buff[0];
																											cal_22_data[11] = ADC_DATA[0].adc_little_buff[1];
																											flag.flag_BIT.flag_ask_for_data =1;													
																											PC_SEND(cal_22_data,20);	
																										}
																								return;
																								}
																								case 3:	// get  coff
																								{	
																									if(calibration_buffer[4]<7)
																										{
																											counter_calibration = 0;
																											uint8_t data[8]= {0,0,0,0,0,0,0,0};
																											data[0] = 2;
																											data[1] =3;
																											data[2] = counter_calibration;
																											current_module = calibration_buffer[4];
																											cal_sub_menu =3;
																											TIM6->CNT=0;
																											#if WAIT_RESPOND_TIME
																											__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
																											timer_6_counter=0;
																											HAL_TIM_Base_Start_IT(&htim6);
																											#endif
																											CAN_SEND_DATA(data,return_module_address(current_module-1),8);
																										}
																									else
																										{
																											uint8_t cal_23_data[20];
																											cal_23_data[0] = 2;
																											cal_23_data[1] = 3;	
																											cal_23_data[2] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[0];
																											cal_23_data[3] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[1];
																											cal_23_data[4] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[2];
																											cal_23_data[5] = READ_VOLTAGE_CALIBRATION_ALPHA.byte[3];
																											//  6 7 8 9
																											cal_23_data[10] = READ_VOLTAGE_CALIBRATION_BETA.byte[0];
																											cal_23_data[11] = READ_VOLTAGE_CALIBRATION_BETA.byte[1];
																											cal_23_data[12] = READ_VOLTAGE_CALIBRATION_BETA.byte[2];
																											cal_23_data[13] = READ_VOLTAGE_CALIBRATION_BETA.byte[3];
																											flag.flag_BIT.flag_ask_for_data =1;													
																											PC_SEND(cal_23_data,20);
																										}
																								return;
																								}
																								case 4:// submit
																								{
																											counter_calibration = 3;	
																											uint8_t data[8]= {0,0,0,0,0,0,0,0};
																											data[0] =2;
																											data[1] = 4;
																											cal_sub_menu =4;
																											command_state =1;
																											#if WAIT_RESPOND_TIME
																											__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
																											HAL_TIM_Base_Start_IT(&htim2);
																											#endif
																											current_module = calibration_buffer[4];
																												flag.flag_BIT.flag_ask_for_data =1;
																											PC_SEND(data,10);
																								}
																									
																								return;
																								case 5:	// pause 	
																								{
																									if(calibration_buffer[4] < 7)
																									{
																									uint8_t data[8]= {0,0,0,0,0,0,0,0};
																									data[0] = 2;
																									data[1] = 5;
																									current_module = calibration_buffer[4];
																									#if WAIT_RESPOND_TIME
																									__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
																									HAL_TIM_Base_Start_IT(&htim6);
																									#endif
																									CAN_SEND_DATA(data,return_module_address(current_module-1),8);
																									
																									cal_sub_menu=5;																				
																									}
																									else
																									{
																									dac_off();
																									uint8_t data[10] = {0,0,0,0,0,0,0,0,0,0};
																									data[0] = 2;
																									data[1] = 5;	
																									PC_SEND(data,10);
																									}
																								}
																								return;
																								case 6:	// ext 	
																								{
																									if( calibration_buffer[4] < 7)
																									{
																									uint8_t data[8];
																									data[1] = 6;
																									data[0] = 2;
																									#if WAIT_RESPOND_TIME
																									__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
																									HAL_TIM_Base_Start_IT(&htim6);
																									#endif
																									current_module = calibration_buffer[4];
																									cal_sub_menu=6;
																									CAN_SEND_DATA(data,return_module_address(current_module-1),8);
																									//reset_state();
																									}
																									else
																									{
																									dac_off();	
																									uint8_t data[10] = {0,0,0,0,0,0,0,0,0,0};
																									data[1] = 6;
																									data[0] = 2;
																									reset_state();
																									PC_SEND(data,10);
																									}
																								}
																								return;
																							}
																						}
																			}
																		else 
																			{
																			uint8_t 	data[4] = { 0xFF , 0xFF ,0 , NOT_VALID_COMMAND};

																			PC_SEND(data,10);	
																				if(system_state != defult_test)
																						{
																						reset_state();
																						}
																					else
																						{
																						comunication_reset();	
																						}
																			return;
																			}
																		}
																	}
												break;
											case 1:
																	{
																	calibration_cof_buffer[command_counter] = x;
																	command_counter++;
																	if(command_counter== 22)
																		{
																		command_counter=0;
																			if((calibration_cof_buffer[0]=='S' & calibration_cof_buffer[1]=='T'& calibration_cof_buffer[20]=='E' & calibration_cof_buffer[21]=='N'))
																			{
																				if(	calibration_cof_buffer[2]==2 & calibration_cof_buffer[3]==4)
																				{
																					timer_2_counter=0;
																					HAL_TIM_Base_Stop_IT(&htim2);
																					
																					if(current_module <7)
																					{
																					uint8_t data[8] = {2,4,1,0,0,0,0,0};
																					data[0] = 2;
																					data[1] =4;
																					data[2] =0; 
																					data[3] =calibration_cof_buffer[4];
																					data[4] =calibration_cof_buffer[5];
																					data[5] =calibration_cof_buffer[6];
																					data[6] =calibration_cof_buffer[7];
																					counter_calibration = 3;
																					#if WAIT_RESPOND_TIME
																					__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
																					timer_6_counter=0;
																					HAL_TIM_Base_Start_IT(&htim6);
																					#endif
																					CAN_SEND_DATA(data,return_module_address(current_module-1),8);
																					}
																					else // main frame set value 
																					{
																						{
																						
																						READ_VOLTAGE_CALIBRATION_ALPHA.byte[0] = calibration_cof_buffer[3];
																						READ_VOLTAGE_CALIBRATION_ALPHA.byte[1] = calibration_cof_buffer[4];
																						READ_VOLTAGE_CALIBRATION_ALPHA.byte[2] = calibration_cof_buffer[5];
																						READ_VOLTAGE_CALIBRATION_ALPHA.byte[3] = calibration_cof_buffer[6];

																						AT24_Write_8(0,READ_VOLTAGE_CALIBRATION_ALPHA.byte[0]);
																						AT24_Write_8(1,READ_VOLTAGE_CALIBRATION_ALPHA.byte[1]);
																						AT24_Write_8(2,READ_VOLTAGE_CALIBRATION_ALPHA.byte[2]);
																						AT24_Write_8(3,READ_VOLTAGE_CALIBRATION_ALPHA.byte[3]);

																						READ_VOLTAGE_CALIBRATION_BETA.byte[0] = calibration_cof_buffer[7];
																						READ_VOLTAGE_CALIBRATION_BETA.byte[1] = calibration_cof_buffer[8];
																						READ_VOLTAGE_CALIBRATION_BETA.byte[2] = calibration_cof_buffer[9];
																						READ_VOLTAGE_CALIBRATION_BETA.byte[3] = calibration_cof_buffer[10];
																						//READ_VOLTAGE_CALIBRATION_BETA = *(float*)(rx_buffer+3);

																						AT24_Write_8(4,READ_VOLTAGE_CALIBRATION_BETA.byte[0]);
																						AT24_Write_8(5,READ_VOLTAGE_CALIBRATION_BETA.byte[1]);
																						AT24_Write_8(6,READ_VOLTAGE_CALIBRATION_BETA.byte[2]);
																						AT24_Write_8(7,READ_VOLTAGE_CALIBRATION_BETA.byte[3]);
																						cal_sub_menu=0;
																						__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
																						HAL_TIM_Base_Stop_IT(&htim6);
																						flag.flag_BIT.flag_ask_for_data =1;			
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

																					}
																					}
																			}
																		}
																	
																	}
												break;
										
										}

			
				break;
			
			case	defult_test:
										if(command_counter==0)
											{
												if(x=='S')
													{
														rx_command_code[command_counter] =x;
														command_counter++;
													}
												else
													{
														command_counter =0;														
													}												
											}
										else if(command_counter==1)
											{
												if(x=='T')
													{
														TIM2->CNT=0;
														
														timer_2_counter=0;
														rx_command_code[command_counter] =x;
														command_counter++;
													}
												else 
													{
													command_counter =0;	
													
													}
											}
										else
											{
											rx_command_code[command_counter] =x;				
											command_counter++;
											if(command_counter==command_buffer_size)
												{
													//flag.flag_BIT.flag_ask_for_data =0;
													command_counter=0;
													if((rx_command_code[0]=='S' & rx_command_code[1]=='T'& rx_command_code[command_buffer_size-2]=='E'&rx_command_code[command_buffer_size-1]=='N'))
															{																
																if(rx_command_code[2] ==50  & rx_command_code[3] == 50)
																	{
																		flag.flag_BIT.send_or_Nsend =0;
																		reset_state();
																		flag.flag_BIT.send_50_50 =1;

																		return;																		
																	}
															else if(rx_command_code[2] == 67 )
															{
																uint8_t PC_data[10];
																PC_data[0] = 67;
																PC_data[1] = 78;
																PC_data[2] = 20;
																PC_SEND(PC_data,10);
															}
															}
													else
															{
															uint8_t 	data[4] = {0xFF , 0xFF , 0XFF,NOT_VALID_COMMAND} ;
												
															PC_SEND(data,10);
															
															return;
															}
														}
											
				break;
			}
	}
}
else{
			uint8_t	PC_data[10] = {0XFF,0XFF,0XFF,0,0,0,0,0,0,0};
			PC_SEND(PC_data,10);
		}	
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
