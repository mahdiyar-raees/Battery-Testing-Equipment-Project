#include <FUNCTION.h>

#include "AT24Cxx.h"

uint8_t start_data[8] ={1,1,0,0,0,0,0,0};


void cal_available_module(BYTE_TO_BIT_t data);

void CAN_SEND_DATA(uint8_t *transmit_buffer , uint16_t identifier,uint8_t lenth)
{
	CAN_TxHeaderTypeDef tx_CAN_Init;
	uint32_t TX_mailbox;
	
	//how many byte we want to send must be less than a 9
	tx_CAN_Init.DLC = lenth;
	
	
	// the identifiyer of the massage that we want to send 
	tx_CAN_Init.StdId = identifier;


	// what kind of identifier we want to use 
	//	
		//CAN_ID_STD      for		standard comuncation		          
		//CAN_ID_EXT   		for		for extended comunication
	tx_CAN_Init.IDE =  CAN_ID_STD   ; 

  
	// is it data fram or remote frame 
	// CAN_RTR_DATA  for data fram
	// CAN_RTR_REMOTE for remote frame 
	tx_CAN_Init.RTR = CAN_RTR_DATA;
	
	//send massage
	if(identifier < 10 )
	{
//			//MY_SEND_CAN(&hcan2,&tx_CAN_Init, transmit_buffer , &TX_mailbox) ;
			HAL_CAN_AddTxMessage(&hcan1,&tx_CAN_Init, transmit_buffer , &TX_mailbox);

			HAL_CAN_AddTxMessage(&hcan2,&tx_CAN_Init, transmit_buffer , &TX_mailbox) ;

			//MY_SEND_CAN(&hcan1,&tx_CAN_Init, transmit_buffer , &TX_mailbox) ;
	
	}
	else if(identifier<23)
	{
		MY_SEND_CAN(&hcan1,&tx_CAN_Init, transmit_buffer , &TX_mailbox);

	}		
	else if(identifier <30)
	{
			MY_SEND_CAN(&hcan2,&tx_CAN_Init, transmit_buffer , &TX_mailbox) ;
			
	
			
	
	}
	
	
	
	
// we don't want to use interupt for sending a message
	
}

void command_handler(uint8_t get_command)
		{
			
			uint8_t PC_data[10];


									switch(get_command)
									{
										case 0:
												execut_command = get_command;
												flag.flag_BIT.flag_ask_for_data =1;
												
												PC_data[0] = execut_command;
												PC_data[1] = command_state;
												TIM2->CNT=0;
												#if WAIT_RESPOND_TIME
												timer_2_counter=0;
												HAL_TIM_Base_Start_IT(&htim2);
												#endif
												PC_SEND(PC_data,10);
												command_state  = 1;
											return;
										
										case 1:
										{
												
												PC_data[1] = command_state;
												execut_command = get_command;
												PC_data[0] = execut_command;
												PC_bus_busy=1;
												PC_SEND(PC_data,10);
												start_counter =0;
												time_counter.main_data=0;
												HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);

												while(1)
													{
														if(available_module[start_counter]==1)
															{
																uint8_t data[1] ={1};
																#if WAIT_RESPOND_TIME
																TIM6->CNT =0;
																timer_6_counter=0;
																__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
																HAL_TIM_Base_Start_IT(&htim6);
																#endif
																current_module =start_counter+1;
																CAN_SEND_DATA(data,return_module_address(start_counter),1);
																return;
															
															}
														else
															{
															start_counter++;
															if(start_counter==6)
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
										case 2: // calibration
												execut_command = get_command;
												
												flag.flag_BIT.flag_ask_for_data =1;												
												system_state = calibration;
												PC_data[0] = 2;
												cal_sub_menu=0;
												PC_data[1] = 0;
												#if WAIT_RESPOND_TIME
//												TIM2->CNT=0;
//												HAL_TIM_Base_Start_IT(&htim2);
												#endif
												command_state  = 0;
												PC_SEND(PC_data,10);
												
												
											break;
										case 3: // get coeffitient
												{
													
														execut_command =4;												
														
														uint8_t data[8]= {0,0,0,0,0,0,0,0};
														data[0] = 2;
														data[1] =3;
														get_data_counter=0;
														for(uint8_t cnt=0; cnt<6; cnt++)
															{
															if(available_module[cnt]==1)
																{
																	get_data_counter++;
																}
															}
														for(uint8_t cnt=0; cnt<6; cnt++)
															{
															if(available_module[cnt]==1)
																{
																	current_module = cnt+1;
																	break;
																}
															}
														get_coff_count =0;	
														
														
														TIM6->CNT=0;
														#if WAIT_RESPOND_TIME
														timer_6_counter=0;
														__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
														HAL_TIM_Base_Start_IT(&htim6);
														#endif
														CAN_SEND_DATA(data,return_module_address(current_module-1),8);
														return;

												}
										case 41: //cc value
												{
													uint8_t can_buffer[8];
													
													current_module  = rx_command_code[3];
													execut_command =60;
													flag.flag_BIT.flag_ask_for_data =1;
													can_buffer[0] =4 ; 
													can_buffer[1] =1 ;
													can_buffer[2] =rx_command_code[4];
													can_buffer[3] =rx_command_code[5];
													can_buffer[4] =rx_command_code[6];
													can_buffer[5] =rx_command_code[7];
													
													#if WAIT_RESPOND_TIME
														TIM6->CNT =0;
														timer_6_counter=0;
													__HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);	
													HAL_TIM_Base_Start_IT(&htim6);
													#endif
													CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
													
												}
												return;
										case 42: //CR value
												{
													uint8_t can_buffer[8];
													
													current_module  = rx_command_code[3];
													execut_command =60;
													flag.flag_BIT.flag_ask_for_data =1;
													can_buffer[0] =4 ; 
													can_buffer[1] =2 ;
													can_buffer[2] =rx_command_code[4];
													can_buffer[3] =rx_command_code[5];
													can_buffer[4] =rx_command_code[6];
													can_buffer[5] =rx_command_code[7];
													
													#if WAIT_RESPOND_TIME
													timer_6_counter=0;
														TIM6->CNT =0;
													__HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);	
													HAL_TIM_Base_Start_IT(&htim6);
													#endif
													CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
													
													
												}
												return;
										case 43:// stop
												{
													uint8_t can_buffer[8];
													
													current_module  = rx_command_code[3];
													execut_command =60;
													flag.flag_BIT.flag_ask_for_data =1;
													can_buffer[0] =4 ; 
													can_buffer[1] =3 ;
													
													#if WAIT_RESPOND_TIME
													timer_6_counter=0;
														TIM6->CNT =0;
													__HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);	
													HAL_TIM_Base_Start_IT(&htim6);
													#endif													
													CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
												}
												return;
										case 44: // start
												{
													uint8_t can_buffer[8];
													
													current_module  = rx_command_code[3];
													flag.flag_BIT.flag_ask_for_data =1;
													execut_command =60;
													can_buffer[0] =4 ; 
													can_buffer[1] =4 ;
													
													#if WAIT_RESPOND_TIME
													timer_6_counter=0;
														TIM6->CNT =0;
													__HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);	
													HAL_TIM_Base_Start_IT(&htim6);
													#endif													
													CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
												}
												return;
										case 45:// start & fire
												{
													uint8_t can_buffer[8];
													
													current_module  = rx_command_code[3];
													flag.flag_BIT.flag_ask_for_data =1;
													execut_command =60;
													can_buffer[0] =4 ; 
													can_buffer[1] =5 ;
													U16_to_U8_t temp;
													temp.small_pack[0] 	= rx_command_code[4]; // test
													temp.small_pack[1] 	= rx_command_code[5]; // test
													detenator_value 	 	= temp.big_data;
													temp.small_pack[0] 	= rx_command_code[6]; // test
													temp.small_pack[1] 	= rx_command_code[7]; // test
													detenator_time 		 	= temp.big_data;
													// 23456write fire code
													#if WAIT_RESPOND_TIME
													timer_6_counter=0;
														TIM6->CNT =0;
													__HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);	
													HAL_TIM_Base_Start_IT(&htim6);
													#endif													
													CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
													
												}
												return;
										case 46: // current protection
												{
													uint8_t can_buffer[8];
													
													current_module  = rx_command_code[3];
													flag.flag_BIT.flag_ask_for_data =1;
													execut_command =60;
													can_buffer[0] =4 ; 
													can_buffer[1] =6 ;
													can_buffer[2] =rx_command_code[4];
													can_buffer[3] =rx_command_code[5];
													can_buffer[4] =rx_command_code[6];
													can_buffer[5] =rx_command_code[7];
													
													#if WAIT_RESPOND_TIME
													timer_6_counter=0;
														TIM6->CNT =0;
													__HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);	
													HAL_TIM_Base_Start_IT(&htim6);
													#endif
													CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
																										
													
													
												}
												return;
										case 47: // power protection
												{
													uint8_t can_buffer[8];
													
													current_module  = rx_command_code[3];
													flag.flag_BIT.flag_ask_for_data =1;
													execut_command =60;
													can_buffer[0] =4 ; 
													can_buffer[1] =7 ;
													can_buffer[2] =rx_command_code[4];
													can_buffer[3] =rx_command_code[5];
													can_buffer[4] =rx_command_code[6];
													can_buffer[5] =rx_command_code[7];
												
													#if WAIT_RESPOND_TIME
													timer_6_counter=0;
														TIM6->CNT =0;
													__HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);
														HAL_TIM_Base_Start_IT(&htim6);
													#endif
													CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
																										
													
												}
												return;
										case 48: // get data
												{
													uint8_t can_buffer[8];
													
													execut_command =60;
													current_module  = rx_command_code[3];
													can_buffer[0] =4 ;
													can_buffer[1] =8 ;													
													#if WAIT_RESPOND_TIME
														TIM6->CNT =0;
													timer_6_counter=0;
													__HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);
														HAL_TIM_Base_Start_IT(&htim6);
													#endif													
													CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
												}
												return;
										case 49: 
												{
													uint8_t can_buffer[8];									
													execut_command =60;
													current_module  = rx_command_code[3];
													can_buffer[0] =4 ;
													can_buffer[1] =9 ;
													can_buffer[2] =0 ;
													#if WAIT_RESPOND_TIME
													timer_6_counter=0;
														TIM6->CNT =0;
														HAL_TIM_Base_Start_IT(&htim6);
													#endif													
													CAN_SEND_DATA(can_buffer,return_module_address(current_module-1),8);
												
												}
												return;
										case 67:
												flag.flag_BIT.flag_ask_for_data =1;
												PC_data[0] = 67;
												PC_data[1] = 78;
												PC_data[2] = 1;
												PC_SEND(PC_data,10);
												#if test
													uint8_t		statrt_sec[2] = {1,2};
													PC_SEND(statrt_sec,2);
												__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);	
												HAL_TIM_Base_Start_IT(&htim5);
										
												#endif
										
											break;
											
									
									}

		}


void SEND_command_handler(uint8_t *data)
	{
			
			 
			TIM6->CNT=0;
			if(data[0] == 0 & data[1] == (return_module_address(technical_buffer.rx_buffer[4]-1)) )
			{
			// maybe send hand shake to pc to show that board is correct
			
				
				current_module =technical_buffer.rx_buffer[4];
				delayUS(5);
				timer_6_counter=0;
			CAN_SEND_DATA(technical_buffer.detail.can_buffer_send[send_counter],return_module_address(current_module-1),8);
			send_counter++;
			timer_6_counter=0;	
			if(send_counter == 254)
				{
					send_finish_counter++;
					if(send_finish_counter<7)
					{
						__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
						HAL_TIM_Base_Stop_IT(&htim6);
						#if WAIT_RESPOND_TIME
						TIM2->CNT=0;
						timer_2_counter=0;
						HAL_TIM_Base_Start_IT(&htim2);
						#endif
						flag.flag_BIT.flag_ask_for_data=1; 		
						uint8_t data[2];
						data[0] = 0;
						data[1] = 2;
				
						send_counter=0;
						if(send_finish_counter==6)
							{
							command_counter=0;
							send_finish_counter=0;	
							reset_state();
							}
						PC_SEND(data,10);

							
					}
					else
					{
						command_counter=0;
						send_finish_counter=0;	
						
						reset_state();
					}
					}
			}
	}
	
	
	
void detect_module_address(uint8_t *module1, uint8_t terminal_count)
	{
		BYTE_TO_BIT_t 				module_detect;
		uint8_t 							*module_location;
		module_location 			=	 module1;
		uint8_t 							cnt=0;
	
		memset(available_module, 0, 6);

		
		
			for(;terminal_count>=1; terminal_count--)
					{		
					module_detect.byte =*(module_location+cnt) ;
					cal_available_module(module_detect);
						if(module_detect.bit.BIT1)
							{
								
								available_module[0] =1;
								cnt++;
								
								continue;
							}
						else 	if(module_detect.bit.BIT2)
							{
						
								cnt++;
								available_module[1] =1;
								continue;
							}
						else 	if(module_detect.bit.BIT3)
							{

								cnt++;
								available_module[2] =1;
								continue;
							}
						else 	if(module_detect.bit.BIT4)
							{

								cnt++;
								available_module[3] =1;
								continue;
							}
						else 	if(module_detect.bit.BIT5)
							{
					
								cnt++;
								available_module[4] =1;
								continue;
							}
						else 	if(module_detect.bit.BIT6)
							{
				
								cnt++;
								available_module[5] =1;
								continue;
							}
						}

	}	
	
uint16_t return_module_address(uint8_t module_count)
{
const uint16_t module_address[6]= {20,21,22,23,24,25};

return module_address[module_count];

}	

void cal_available_module(BYTE_TO_BIT_t data)
{
	

	available_module[0] = (data.bit.BIT1 | available_module[0]);
	available_module[1] = (data.bit.BIT2 | available_module[1]);
	available_module[2] = (data.bit.BIT3 | available_module[2]);
	available_module[3] = (data.bit.BIT4 | available_module[3]);
	available_module[4] = (data.bit.BIT5 | available_module[4]);
	available_module[5] = (data.bit.BIT6 | available_module[5]);
	

}
void 		START_command_handler(uint8_t *rx_buffer)
	{

			// address fualt
			// - 20+ 2 = 18
		start_data[*(rx_buffer)-18]= *(rx_buffer+1);
		start_counter++;
		__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
		HAL_TIM_Base_Stop_IT(&htim6);
		timer_6_counter=0;
		
		while(1)
				{
					if(available_module[start_counter] && (start_counter < 6))
						{
							uint8_t can_data_start[1] ={1};
							current_module =start_counter +1;
							CAN_SEND_DATA(can_data_start,return_module_address(current_module-1),1);
							#if WAIT_RESPOND_TIME
							TIM6->CNT=0;
							timer_6_counter=0;
							__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);													
							HAL_TIM_Base_Start_IT(&htim6);
							#endif
							return;						
						}
					else
					{
					start_counter++;
					if(start_counter==7) // finish command
						{
						TIM6->CNT =0;		
						timer_6_counter=0;
						__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
						HAL_TIM_Base_Stop_IT(&htim6);
						
						if(flag.flag_BIT.panel_is_running ==0) // PANEL is not running 
							{
								PC_SEND(start_data,10);	
							}
						// for next state equal 2 
						start_counter =2;
						// cheack fault
						while(1)
						{
							if(start_data[start_counter])// fault existence
									{
										if(flag.flag_BIT.panel_is_running)
											{

													flag.flag_BIT.panel_ask_for_data= 1;
													panel_send_buff.detail.type_or_ok = 20;
													panel_send();												
												
											}
										reset_state(); 

										return;
									}
							else
									{
										start_counter++;
										if(start_counter==8) // there is no fault
											{
												memset(start_data,0,10);
												start_data[0] = 1;
												start_data[1] = 1;												
												start_counter =0;
												for(uint8_t cnt=0; cnt<6;cnt++)
														{
															if(available_module[cnt])
																	{
																		uint8_t CAN_data1[1] ={1};					
																		current_module = cnt +1;
																		CAN_SEND_DATA(CAN_data1,return_module_address(cnt),1);
																		delayUS(100);
																	}
														}
													for(uint16_t temp =0; temp<5000;)
															{
																temp++;
																delayUS(1000);
															}
													transmit_error=0;
													get_data_counter =0;
													TIM6->CNT =0;
													execut_command = 40; // cheak fan state
													system_state = defult_test;
													for(uint8_t cnt=0; cnt<6;cnt++)
														{
														if(available_module[cnt] ==1)
															{
															get_data_counter++;
															}
														
														}
													uint8_t CAN_data2[2] ={1,0};	
													fan_counter_get =0;
													TIM6->CNT=0;
													
													timer_6_counter=0;
													HAL_TIM_Base_Start_IT(&htim6);
													HAL_TIM_Base_Start_IT(&htim6);
													CAN_SEND_DATA(CAN_data2,module_commen_address,2);
												break;
											}
									}
						}

						return;
						}
					}
				}
	}
	
void 	  port_selector(uint8_t port)
	{
	
	switch(port)
	{
		case 1://rs 485
				__HAL_UART_DISABLE_IT(&huart4, UART_IT_RXNE);	
				__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		break;
		case 2://usb
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
				__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
			break;
		
		case 3://ehternet
			
			break;
	
	
	}
	
	}
	
void reset_state(void)
	{				
			HAL_TIM_Base_Stop_IT(&htim5);
			__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);	
			HAL_TIM_Base_Stop_IT(&htim2);
			__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);	
			HAL_TIM_Base_Stop_IT(&htim6);
			__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
		{
			uint8_t	can_data[8] = {0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE};
			CAN_SEND_DATA(can_data,module_commen_address,8);
		}
		
		in_syncronize.main_data =0;
		process_valid= 2;
		PC_bus_busy=0;
		panel_bus_busy =0;
		memset(get_board,0,6);
		flag.flag_BIT.process_runnig =0;
		panel_command_counter =0;
		dac_off();
		flag.flag_BIT.send_or_Nsend =0;
		flag.flag_BIT.if_proccess_start =0;
		flag.flag_BIT.panel_is_running =0;
		flag.flag_BIT.panel_send_process = 0;
		flag.flag_BIT.panel_ask_for_data=1;
		memset(start_data,0,8);
		start_data[0]=1;
		start_data[1] =1;
		memset( finish_module	,0,6);	
		memset( fault_send	,0,6);	
		get_data_buffer[3] = 1; 
		get_data_buffer[4] = 3;
		f1 =0;f2 =0;
		can_bus_turn_off_counter =0;
		get_data_buffer[0] = first_frame_1;				
		get_data_buffer[1] = first_frame_2;
		get_data_buffer[2] = first_frame_3;
		get_data_buffer[35] = second_frame_1;				
		get_data_buffer[36] = second_frame_2;
		get_data_buffer[37] = second_frame_3;	
		send_counter=0;		
		execut_command =0;
		send_finish_counter =0;
		command_state = 0;
		system_state = init;
		flag.flag_BIT.panel_get_data_trigger =0;
		dac_off();

	
		flag.flag_BIT.flag_ask_for_data =1;

	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING |  \
																CAN_IT_RX_FIFO1_MSG_PENDING );	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING |  \
																CAN_IT_RX_FIFO1_MSG_PENDING );

		command_counter=0;

		
		
//		// new
//		panel_command_counter =0;
//		HAL_UART_Abort(&huart1);
//		system_state = init;
//		command_state =0;
//		uint8_t temp;
//		HAL_UART_Abort_IT(&huart1);
//		for(uint8_t cnt=0; cnt<50;cnt++)
//			{
//				temp =USART1->DR;
//				HAL_UART_Transmit(&huart1,&temp,1,100);
//			}
//		__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);	
//		uart_init();
//		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
//				__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);	
//				uart_init();
//		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
//				__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);	
//		uart_init();
//		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);	
//				
//			command_counter=0;
	}
	
void comunication_reset(void)
	{
			HAL_TIM_Base_Stop_IT(&htim2);
			__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);	
			HAL_TIM_Base_Stop_IT(&htim6);
			__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	
			panel_command_counter =0;

		system_state = init;
		command_state =0;

		flag.flag_BIT.flag_ask_for_data =1;
		command_counter=0;

	}	
	
void uart_init(void)
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
void delayUS(uint32_t us)
	{   // Sets the delay in microseconds.
	//uint8_t tim = 0;
		__HAL_TIM_SET_COUNTER(&htim8,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim8) < us);
	}	


void delayMS(uint32_t ms)
{
	uint32_t temp = ms;
	while(temp>1)
		{
			temp--;
			delayUS(1000);
		}

}	
void 	CAN_INIT(void)
{
	
	CAN_Filtter_config();


	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING |  \
																CAN_IT_RX_FIFO1_MSG_PENDING );	
																

if(HAL_CAN_Start(&hcan2)!= HAL_OK)
{
	Error_Handler();
}

	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING |  \
																CAN_IT_RX_FIFO1_MSG_PENDING );

if(HAL_CAN_Start(&hcan1)!= HAL_OK)
{
	Error_Handler();
}



}	

		
void CAN_Filtter_config(void)
	{
		CAN_FilterTypeDef can1_filter_Init;
		CAN_FilterTypeDef can2_filter_Init;
		CAN_FilterTypeDef can3_filter_Init;
		CAN_FilterTypeDef can4_filter_Init;
		

//*************************//***************************************
//*************************//***************************************
//*************************//***************************************			
			


					
		
		
		

		
		can1_filter_Init.FilterActivation= ENABLE;
		
		//which of 28 fillter we want to use
		can1_filter_Init.FilterBank=9;
		
		//which of 2 FIFO we want to use 
		can1_filter_Init.FilterFIFOAssignment= CAN_RX_FIFO1;
		
		
		// 32 bit identifier high xxxx****
		can1_filter_Init.FilterIdHigh= main_board_ERROR_address<<5;
	
		// 32 bit identifier low ****xxxx	
		can1_filter_Init.FilterIdLow= 0x0000;
		
		
		// 32 bit mask high xxxx****
		can1_filter_Init.FilterMaskIdHigh=0x0000;
		
		// 32 bit mask low ****xxxx	
		can1_filter_Init.FilterMaskIdLow=0x0000;
		
		// chose filter type it can be
		//  CAN_FILTERMODE_IDLIST 	for ID MODE
		//  CAN_FILTERMODE_IDMASK 	for MASK MODE
		can1_filter_Init.FilterMode= CAN_FILTERMODE_IDLIST;
	
		
		can1_filter_Init.FilterScale= CAN_FILTERSCALE_32BIT;
		
		
	if(HAL_CAN_ConfigFilter(&hcan2,&can1_filter_Init)!= HAL_OK)
			{
			Error_Handler();
			}
			//*************************//***************************************
//*************************//***************************************
//*************************//***************************************			
//	
			
		can4_filter_Init.FilterActivation= ENABLE;
		
		//which of 28 fillter we want to use
		can4_filter_Init.FilterBank=6;
		
		//which of 2 FIFO we want to use 
		can4_filter_Init.FilterFIFOAssignment= CAN_RX_FIFO1;
		
		
		// 32 bit identifier high xxxx****
		can4_filter_Init.FilterIdHigh= main_board_ERROR_address<<5;
	
		// 32 bit identifier low ****xxxx	
		can4_filter_Init.FilterIdLow= 0x0000;
		
		
		// 32 bit mask high xxxx****
		can4_filter_Init.FilterMaskIdHigh=0;
		
		// 32 bit mask low ****xxxx	
		can4_filter_Init.FilterMaskIdLow=0x0000;
		
		// chose filter type it can be
		//  CAN_FILTERMODE_IDLIST 	for ID MODE
		//  CAN_FILTERMODE_IDMASK 	for MASK MODE
		can4_filter_Init.FilterMode= CAN_FILTERMODE_IDLIST;
	
		
		can4_filter_Init.FilterScale= CAN_FILTERSCALE_32BIT;
		
		
	if(HAL_CAN_ConfigFilter(&hcan1,&can4_filter_Init)!= HAL_OK)
			{
			Error_Handler();
			}	
			

			//*************************//***************************************
//*************************//***************************************
//*************************//***************************************			
//	
			
			
			
			

			
		can3_filter_Init.FilterActivation= ENABLE;
		
		//which of 28 fillter we want to use
		can3_filter_Init.FilterBank=3;
		
		//which of 2 FIFO we want to use 
		can3_filter_Init.FilterFIFOAssignment= CAN_RX_FIFO0;
		
		
		// 32 bit identifier high xxxx****
		can3_filter_Init.FilterIdHigh= main_board_address<<5;
	
		// 32 bit identifier low ****xxxx	
		can3_filter_Init.FilterIdLow= 0x0000;
		
		
		// 32 bit mask high xxxx****
		can3_filter_Init.FilterMaskIdHigh=0;
		
		// 32 bit mask low ****xxxx	
		can3_filter_Init.FilterMaskIdLow=0x0000;
		
		// chose filter type it can be
		//  CAN_FILTERMODE_IDLIST 	for ID MODE
		//  CAN_FILTERMODE_IDMASK 	for MASK MODE
		can3_filter_Init.FilterMode= CAN_FILTERMODE_IDLIST;
	
		
		can3_filter_Init.FilterScale= CAN_FILTERSCALE_32BIT;
		
		
	if(HAL_CAN_ConfigFilter(&hcan1,&can3_filter_Init)!= HAL_OK)
			{
			Error_Handler();
			}
	//*************************//***************************************
//*************************//***************************************
//*************************//***************************************			
//		

		can2_filter_Init.FilterActivation= ENABLE;
		
		//which of 28 fillter we want to use
		can2_filter_Init.FilterBank=0;
		
		//which of 2 FIFO we want to use 
		can2_filter_Init.FilterFIFOAssignment= CAN_RX_FIFO0;
		
		
		// 32 bit identifier high xxxx****
		can2_filter_Init.FilterIdHigh= main_board_address<<5;
	
		// 32 bit identifier low ****xxxx	
		can2_filter_Init.FilterIdLow= 0x0000;
		
		
		// 32 bit mask high xxxx****
		can2_filter_Init.FilterMaskIdHigh=0x0000;
		
		// 32 bit mask low ****xxxx	
		can2_filter_Init.FilterMaskIdLow=0x0000;
		
		// chose filter type it can be
		//  CAN_FILTERMODE_IDLIST 	for ID MODE
		//  CAN_FILTERMODE_IDMASK 	for MASK MODE
		can2_filter_Init.FilterMode= CAN_FILTERMODE_IDLIST;
	
		
		can2_filter_Init.FilterScale= CAN_FILTERSCALE_32BIT;
		
		
	if(HAL_CAN_ConfigFilter(&hcan2,&can2_filter_Init)!= HAL_OK)
			{
			Error_Handler();
			}


			
			
	}
	
	void USART_Transmit(UART_HandleTypeDef *huart, void *istr, uint16_t len)
{
	const uint8_t *str = istr;
	uint16_t i=0;

	while(len)
	{
		i=0;
		while((__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) ? SET : RESET) == RESET) 
		{
			i++;
			if(i > 1000)
				return;
			//delayUS(10);
		}
		
		huart->Instance->DR = (*str++ & (uint8_t)0xFF);		
		len--;
	}
	i=0;
	while((__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) ? SET : RESET) == RESET) 
	{
		i++;
		if(i > 1000)
			return;
	}
}



	
	

	void		PC_SEND(uint8_t *send_array , uint8_t size)
		{
				uint8_t  start_buffer[buffer_size] = {first_frame_1 , first_frame_2, first_frame_3  };
				uint8_t  stop_buffer [buffer_size] = {second_frame_1 ,second_frame_2,second_frame_3 };
			
			
			switch(port)
			{
				case usb:
							HAL_UART_Transmit(&huart2, start_buffer,buffer_size,100);
							HAL_UART_Transmit(&huart2, send_array,size, 100);
							HAL_UART_Transmit(&huart2, stop_buffer,buffer_size,100);
					break;
				case ethernet:
					
					break;
				case rs485:
							HAL_UART_Transmit(&huart1, start_buffer,buffer_size,100);
							HAL_UART_Transmit(&huart1, send_array,size, 100);
							HAL_UART_Transmit(&huart1, stop_buffer,buffer_size,100);
					break;
			
			
}		
		}
	
void  panel_send (void)
{
		HAL_UART_Transmit(&huart4, panel_send_buff.send_buff,40, 100);
}

HAL_StatusTypeDef MY_SEND_CAN(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
{
  uint32_t transmitmailbox;
  HAL_CAN_StateTypeDef state = hcan->State;
  uint32_t tsr = READ_REG(hcan->Instance->TSR);




      /* Select an empty transmit mailbox */
      transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;


      /* Store the Tx mailbox */
      *pTxMailbox = (uint32_t)1 << transmitmailbox;

      /* Set up the Id */

        hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->StdId << CAN_TI0R_STID_Pos) |
                                                           pHeader->RTR);
      


      /* Set up the DLC */
      hcan->Instance->sTxMailBox[transmitmailbox].TDTR = (pHeader->DLC);

      /* Set up the Transmit Global Time mode */
      if (pHeader->TransmitGlobalTime == ENABLE)
      {
        SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TDTR, CAN_TDT0R_TGT);
      }

      /* Set up the data field */
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDHR,
                ((uint32_t)aData[7] << CAN_TDH0R_DATA7_Pos) |
                ((uint32_t)aData[6] << CAN_TDH0R_DATA6_Pos) |
                ((uint32_t)aData[5] << CAN_TDH0R_DATA5_Pos) |
                ((uint32_t)aData[4] << CAN_TDH0R_DATA4_Pos));
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDLR,
                ((uint32_t)aData[3] << CAN_TDL0R_DATA3_Pos) |
                ((uint32_t)aData[2] << CAN_TDL0R_DATA2_Pos) |
                ((uint32_t)aData[1] << CAN_TDL0R_DATA1_Pos) |
                ((uint32_t)aData[0] << CAN_TDL0R_DATA0_Pos));

      /* Request transmission */
      SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);

      /* Return function status */
      return HAL_OK;
   

}

HAL_StatusTypeDef MY_RECEIVE_FIFO0(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	

	
	
	
	
	
	
  HAL_CAN_StateTypeDef state = hcan->State;


 
		pHeader->IDE = CAN_RI0R_IDE & hcan->Instance->sFIFOMailBox[RxFifo].RIR;
		pHeader->ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & hcan->Instance->sFIFOMailBox[RxFifo].RIR) >> CAN_RI0R_EXID_Pos;
    
    pHeader->RTR = (CAN_RI0R_RTR & hcan->Instance->sFIFOMailBox[RxFifo].RIR);
    pHeader->DLC = (CAN_RDT0R_DLC & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_DLC_Pos;
    pHeader->FilterMatchIndex = (CAN_RDT0R_FMI & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_FMI_Pos;
    pHeader->Timestamp = (CAN_RDT0R_TIME & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_TIME_Pos;

    /* Get the data */
    aData[0] = (uint8_t)((CAN_RDL0R_DATA0 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA0_Pos);
    aData[1] = (uint8_t)((CAN_RDL0R_DATA1 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA1_Pos);
    aData[2] = (uint8_t)((CAN_RDL0R_DATA2 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA2_Pos);
    aData[3] = (uint8_t)((CAN_RDL0R_DATA3 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA3_Pos);
    aData[4] = (uint8_t)((CAN_RDH0R_DATA4 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA4_Pos);
    aData[5] = (uint8_t)((CAN_RDH0R_DATA5 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA5_Pos);
    aData[6] = (uint8_t)((CAN_RDH0R_DATA6 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA6_Pos);
    aData[7] = (uint8_t)((CAN_RDH0R_DATA7 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA7_Pos);


      /* Release RX FIFO 0 */
      SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
    

      /* Release RX FIFO 1 */
     // SET_BIT(hcan->Instance->RF1R, CAN_RF1R_RFOM1);
    

    /* Return function status */
    return HAL_OK;

}



HAL_StatusTypeDef MY_RECEIVE_FIFO1(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
    HAL_CAN_StateTypeDef state = hcan->State;


 
		pHeader->IDE = CAN_RI0R_IDE & hcan->Instance->sFIFOMailBox[RxFifo].RIR;
		pHeader->ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & hcan->Instance->sFIFOMailBox[RxFifo].RIR) >> CAN_RI0R_EXID_Pos;
    
    pHeader->RTR = (CAN_RI0R_RTR & hcan->Instance->sFIFOMailBox[RxFifo].RIR);
    pHeader->DLC = (CAN_RDT0R_DLC & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_DLC_Pos;
    pHeader->FilterMatchIndex = (CAN_RDT0R_FMI & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_FMI_Pos;
    pHeader->Timestamp = (CAN_RDT0R_TIME & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_TIME_Pos;

    /* Get the data */
    aData[0] = (uint8_t)((CAN_RDL0R_DATA0 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA0_Pos);
    aData[1] = (uint8_t)((CAN_RDL0R_DATA1 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA1_Pos);
    aData[2] = (uint8_t)((CAN_RDL0R_DATA2 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA2_Pos);
    aData[3] = (uint8_t)((CAN_RDL0R_DATA3 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA3_Pos);
    aData[4] = (uint8_t)((CAN_RDH0R_DATA4 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA4_Pos);
    aData[5] = (uint8_t)((CAN_RDH0R_DATA5 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA5_Pos);
    aData[6] = (uint8_t)((CAN_RDH0R_DATA6 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA6_Pos);
    aData[7] = (uint8_t)((CAN_RDH0R_DATA7 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA7_Pos);


      /* Release RX FIFO 0 */
      //SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
    

      /* Release RX FIFO 1 */
      SET_BIT(hcan->Instance->RF1R, CAN_RF1R_RFOM1);
    

    /* Return function status */
    return HAL_OK;
}

void dac_set_value(uint16_t value)
	{
		uint16_t temp = value;
		
		if(temp> Detenator_MAX)
			{
				temp =Detenator_MAX;
			}
		if(HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,temp)== HAL_OK)
			{
				HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
			}
	}
	
void dac_off(void)
	{
		if(HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0)== HAL_OK)
			{
				HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
			}
			HAL_DAC_Stop(&hdac,DAC_CHANNEL_2);
	}
	
void  panel_send_process_f(void)
	{
		panel_send_process_counter++;	
		if(panel_send_process_counter ==6) // next board
			{
				panel_send_process_counter =0;
				if(current_module< 6)
				{
				for(uint8_t fast_cnt =current_module; fast_cnt<7;fast_cnt++)
					{
						if(available_module[fast_cnt])
							{
								current_module = fast_cnt+1;
								// SET module to get station	
								uint8_t can_data[8];
								can_data[0] = 5;
								#if WAIT_RESPOND_TIME
								TIM6->CNT =0;
								timer_6_counter=0;
								HAL_TIM_Base_Start_IT(&htim6);
								#endif
								delayUS(100);	
								CAN_SEND_DATA(can_data,return_module_address(current_module-1),8);
								return;	
							}
						else if(fast_cnt ==5) // finish 
							{
								HAL_TIM_Base_Stop_IT(&htim6);
								__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
								flag.flag_BIT.panel_ask_for_data= 1;
								panel_send_buff.detail.type_or_ok =OK;
								panel_send();
								reset_state();
								return;
							}
					}
				}
				else
				{
								HAL_TIM_Base_Stop_IT(&htim6);
								__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
								flag.flag_BIT.panel_ask_for_data= 1;
								panel_send_buff.detail.type_or_ok =OK;
								panel_send();
								reset_state();
								return;
				}
				
			}
		else
			{
				TIM6->CNT=0;
				timer_6_counter=0;
				delayUS(100);
				CAN_SEND_DATA(technical_buffer.detail.can_buffer_send[panel_send_process_counter-1],return_module_address(current_module-1),8);
			}
}
void 	initial_function(void)
{
uint8_t	temp;
for( temp =0 ; temp<200;temp++)
{
command_handler(temp);
}
SEND_command_handler(&temp);
START_command_handler(&temp);		
}


