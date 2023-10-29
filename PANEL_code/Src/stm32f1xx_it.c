/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <FUNCTION.h>
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
extern													uint8_t 							SEG_menue_level;
extern													FLAG_t								flag;
extern 													uint8_t								panel_command_counter;
extern													panel_send_buff_t			panel_send;
extern													panel_get_buff_t			panel_get;
extern													float 								real_num[13];			
extern												  float									calibration_coeffitent[26];
extern													uint16_t 								timer4_counter;
extern													uint16_t 								timer6_counter;
extern													uint16_t 								timer5_counter;	
extern													uint8_t									choose_operation_counter;
extern													uint8_t								send_send_counter;
extern													uint8_t								complete_get_coff_count;
extern													uint8_t								cof_4byte_count;
void PANEL_COMMAND_handeler(uint8_t x);
void get_coff_hadeler(uint8_t x);

void bus_bussy_command(void);

extern uint8_t    anghezi[40] ;
extern uint8_t		 anghezi_test;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
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

if(flag.BIT.initial_coff_ask)
	{
		get_coff_hadeler(USART1->DR);
	}
else
	{	
		PANEL_COMMAND_handeler(USART1->DR);
	}
		return;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
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
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void get_coff_hadeler(uint8_t x)
	{
		timer6_counter=0;
		static float_to_u8 float_u8;
				if(panel_command_counter ==0)
					{
						if(x == 1)
							{
								// maybe turn on timer 
								#if wait_time
								TIM4->CNT=0;
								timer4_counter=0;
								HAL_TIM_Base_Start_IT(&htim4);
								#endif
								panel_get.get_buff[0] = x;
								panel_command_counter++;
							}
						else
							{								
								panel_command_counter=0;
							}
					}
				else if(panel_command_counter ==1)
					{
						if(x == 6)
							{
								panel_get.get_buff[1] =x;
								panel_command_counter++;
								
							}
						else
							{
								reset_state();
							}
					}
				else
					{
							anghezi[anghezi_test] = USART1->DR;	
							anghezi_test++;
							if(anghezi_test==110)
							{
								
							anghezi_test =0;
							}
						float_u8.U8[cof_4byte_count] = x;
						cof_4byte_count++;
						panel_command_counter++;
						if(cof_4byte_count ==4)
							{
								cof_4byte_count =0;
								calibration_coeffitent[complete_get_coff_count] = float_u8.real_num;
								complete_get_coff_count++;								
							}
						if(panel_command_counter ==110)
							{
								flag.BIT.first_cof_get =0;
								reset_state();
							}
					}
	
	}
void PANEL_COMMAND_handeler(uint8_t x)
	{
		TIM6->CNT =0; 

				if(panel_command_counter ==0)
					{
						if(x == panel_start_1 )
							{
								panel_get.get_buff[0] = x;
								panel_command_counter++;
								return;
							}
						else if(x == 1)
							{
							panel_command_counter++;	
							flag.BIT.first_cof_get =1;
							return;	
							}
						else
							{								
								panel_command_counter=0;
								//reset_state();
							}
					}

				else if(panel_command_counter ==1)
					{
						if(x == panel_start_2)
							{
								panel_get.get_buff[1] =x;
								panel_command_counter++;
								return;
							}
						else if(x == 6 && flag.BIT.first_cof_get ==1)
							{
								flag.BIT.initial_coff_ask =1;
								panel_command_counter++;
								return;
							}
						else
							{
								panel_command_counter=0;
								reset_state();
							}
					}
				else
					{
						panel_get.get_buff[panel_command_counter] =x;
						panel_command_counter++;
						if(panel_command_counter ==40)
							{
								panel_command_counter=0;
								timer6_counter =0; 
								if(panel_get.get_buff[39] == panel_end_2 & panel_get.get_buff[38] == panel_end_1)
									{
											switch(panel_get.detail.fault_or_command)
												{
												case 1:// start
													{
														if(panel_get.detail.type_or_ok == OK)
															{
																//show statrt has done and switch menu to show data
																segment_clear();
																SEG_menue_level =100;
																segment_put("PROCESS STRTED",0);
																TIM6->CNT=0;
																delayUS(wait_menue_time);
																TIM6->CNT=0;
																flag.BIT.panel_first_ask_data =1;
																segment_clear();																
																segment_put("REQUEST DATA",0);
																panel_send.detail.command=4;
																#if wait_time
																timer6_counter =0;
																HAL_TIM_Base_Start_IT(&htim6);
																#endif
																delayUS(100);
																HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);
																
															}
															else if(panel_get.detail.type_or_ok == 20)
															{
																		__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
																		HAL_TIM_Base_Stop_IT(&htim6);	
																		__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
																		HAL_TIM_Base_Stop_IT(&htim5);	
																		__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
																		HAL_TIM_Base_Stop_IT(&htim4);	
																		segment_clear();
																		segment_put("ERROR",0);
																		{
																		char str[3];
																		sprintf(str,"%d",20);
																		segment_put(str,7);
																		}
																		LED_E1_ON;
																		delayUS(3000);
																		reset_state();
															}
														else
															{
																bus_bussy_command();
															}
													}
													return;
												case 2: // stop
													{
														if(panel_get.detail.type_or_ok == OK)
															{
																//show stop has done and switch menu home
																segment_clear();
																segment_put("SYSTEM STOPED",0);
																delayUS(2000);
																reset_state();
															}
														else
															{
																bus_bussy_command();
															}
													}
													return; // force stop
												case 3:
													{
																//show force stop has done and switch menu home
																segment_clear();
																segment_put("SYSTEM STOPED",0);
																delayUS(2000);
																reset_state();																
															
													}
													return;
												case 4: // get data 
													{														
														if(panel_get.detail.type_or_ok == OK) // there is process running
															{
																if(panel_get.detail.finish_test ==1) // test finishied
																	{
																		__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
																		HAL_TIM_Base_Stop_IT(&htim6);	
																		__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
																		HAL_TIM_Base_Stop_IT(&htim5);	
																		__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
																		HAL_TIM_Base_Stop_IT(&htim4);	
																		segment_clear();
																		segment_put("FINISH",0);
																		return;
																	}
																	else if(panel_get.detail.error_type>0)
																	{
																		__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
																		HAL_TIM_Base_Stop_IT(&htim6);	
																		__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
																		HAL_TIM_Base_Stop_IT(&htim5);	
																		__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
																		HAL_TIM_Base_Stop_IT(&htim4);	
																		segment_clear();
																		segment_put("ERROR",0);
																		{
																		char str[3];
																		sprintf(str,"%d",panel_get.detail.error_type);
																		segment_put(str,7);
																		}
																		LED_emerg_ON;
																	}
																	else
																		{
																			HAL_TIM_Base_Stop_IT(&htim6);
																			for(uint8_t cnt=0; cnt<13; cnt++)
																				{
																					U16_to_U8 temp_buffer;
																					temp_buffer.U8[0] =  panel_get.detail.data[cnt*2];
																					temp_buffer.U8[1] =  panel_get.detail.data[(cnt*2)+1];
																					real_num[cnt]     =  calibration_coeffitent[cnt*2] * temp_buffer.U16 + calibration_coeffitent[(cnt*2)+1];
																				}
																			if(flag.BIT.panel_first_ask_data)
																				{
																					__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
																					HAL_TIM_Base_Stop_IT(&htim4);	
																					flag.BIT.panel_first_ask_data =0;
																					// turn on timer that ask data every 1s
																					TIM5->CNT =0;
																					HAL_TIM_Base_Start_IT(&htim5);
																					show_num(0);	
																				}
																			}
															}
														else
															{
																bus_bussy_command();
//																//FINISH PROCESS 
//																segment_clear();
//																segment_put("FINISH PROCESS",0);
//																reset_state();
															}
													}
													return;
												case 5: // set profile
													{
														if(panel_get.detail.type_or_ok == OK)
															{
																if(send_send_counter==0)
																	{
																		flag.BIT.first_one_cal_save =0;
																			timer6_counter =0;
																		send_send_counter++;
																		HAL_TIM_Base_Stop_IT(&htim6);
																			delayUS(3000);
																		HAL_TIM_Base_Start_IT(&htim6);
																		HAL_UART_Transmit(&huart1,process.send_buffer,40,100);
																		
																	}
																else //show profole has beed seted and dont change
																	{
																	send_send_counter= 0;	
																	flag.BIT.first_one_cal_save =0;
																	timer6_counter =0;
																	HAL_TIM_Base_Stop_IT(&htim6);
																	choose_operation_counter =0;
																	segment_clear();
																	segment_put("CHOOSE OP START",0);
																	SEG_menue_level =2;
																	}																														
															}
														else
															{
																bus_bussy_command();
															}
													}
													return;

													case 255: //fault 
													{
														//error_manegment(panel_get.detail.type_or_ok);
														reset_state();														
													}
													return;
												
												
												}
									}
							}
					}
					
				

	}
	
	
void bus_bussy_command(void)
	{
		// bus busy not available command 
		segment_clear();
		segment_put("BUS IS BUSY",0);
		delayUS(2000);
		segment_clear();
		segment_put("TRY AGAIN",0);
		delayUS(2000);
		reset_state();
		
	}	
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
