#include <FUNCTION.h>

#include "math.h"







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
	MY_SEND_CAN(&hcan1,&tx_CAN_Init, transmit_buffer , &TX_mailbox) ;

	
// we don't want to use interupt for sending a message
	
}


		
void CAN_Filtter_config(void)
	{
		CAN_FilterTypeDef can1_filter_Init;
		
		
		
		can1_filter_Init.FilterActivation= ENABLE;
		
		//which of 28 fillter we want to use
		can1_filter_Init.FilterBank=0;
		
		//which of 2 FIFO we want to use 
		can1_filter_Init.FilterFIFOAssignment= CAN_RX_FIFO0;
		
		
		// 32 bit identifier high xxxx****
		can1_filter_Init.FilterIdHigh= MODULE_BOARD_ADDRESS<<5;
	
		// 32 bit identifier low ****xxxx	
		can1_filter_Init.FilterIdLow= 0x0000;
		
		
		// 32 bit mask high xxxx****
		can1_filter_Init.FilterMaskIdHigh= 0x0000;
		
		// 32 bit mask low ****xxxx	
		can1_filter_Init.FilterMaskIdLow=0x0000;
		
		// chose filter type it can be
		//  CAN_FILTERMODE_IDLIST 	for ID MODE
		//  CAN_FILTERMODE_IDMASK 	for MASK MODE
		can1_filter_Init.FilterMode= CAN_FILTERMODE_IDLIST;
	
		
		can1_filter_Init.FilterScale= CAN_FILTERSCALE_32BIT;
		
		
	if(HAL_CAN_ConfigFilter(&hcan1,&can1_filter_Init)!= HAL_OK)
			{
			Error_Handler();
			}			

	
	

		//****************************************??***
		//****************************************??***
		//****************************************??***
		//****************************************??***	
		
		can1_filter_Init.FilterActivation= ENABLE;
		
		//which of 28 fillter we want to use
		can1_filter_Init.FilterBank=1;
		
		//which of 2 FIFO we want to use 
		can1_filter_Init.FilterFIFOAssignment= CAN_RX_FIFO1;
		
		
		// 32 bit identifier high xxxx****
		can1_filter_Init.FilterIdHigh= commen_address<<5;
	
		// 32 bit identifier low ****xxxx	
		can1_filter_Init.FilterIdLow= 0x0000;
		
		
		// 32 bit mask high xxxx****
		can1_filter_Init.FilterMaskIdHigh= 0x0000;
		
		// 32 bit mask low ****xxxx	
		can1_filter_Init.FilterMaskIdLow=0x0000;
		
		// chose filter type it can be
		//  CAN_FILTERMODE_IDLIST 	for ID MODE
		//  CAN_FILTERMODE_IDMASK 	for MASK MODE
		can1_filter_Init.FilterMode= CAN_FILTERMODE_IDLIST;
	
		
		can1_filter_Init.FilterScale= CAN_FILTERSCALE_32BIT;
		
		
	if(HAL_CAN_ConfigFilter(&hcan1,&can1_filter_Init)!= HAL_OK)
			{
			Error_Handler();
			}			

	}
	



	void CAN_INIT(void)
	{
	
		CAN_Filtter_config();

																
			HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING |  \
																CAN_IT_RX_FIFO1_MSG_PENDING 	);
	

		if(HAL_CAN_Start(&hcan1)!= HAL_OK)
		{
			Error_Handler();
		}		

	}

uint32_t moving_average(uint32_t sum_buff, uint16_t last_value, uint16_t new_value)
	{
			
	 return (sum_buff - last_value +new_value);

	
	
	}
	
	
	
void SPI_handeler(SPI_HandleTypeDef *hspi)
{

		static uint32_t reverse_voltage_error_count =0;
	if(hspi->Instance == SPI1)
		{
			
			__IO uint8_t  *ptmpreg8;
			ptmpreg8 = (__IO uint8_t *)&hspi->Instance->DR;
			__HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));			
			__IO uint8_t  tmpreg8 = 0;
			 __HAL_SPI_DISABLE(hspi);
			while ((hspi->Instance->SR & SPI_FLAG_FRLVL) != SPI_FRLVL_EMPTY)
				{
					if (SPI_FLAG_FRLVL == SPI_SR_FRLVL)
					{
						/* Flush Data Register by a blank read */
						tmpreg8  = *ptmpreg8;
						UNUSED(tmpreg8);					
					}
				}
				odd_or_even =0;
				hspi->State = HAL_SPI_STATE_READY;
				validate_shunt.adc_buff[0] = temp_adc1.adc_buff[1];
				validate_shunt.adc_buff[1] = temp_adc1.adc_buff[0];
				shunt_cnt++;
				current_buffer[shunt_cnt] = validate_shunt.ADC_DATA;

				if(shunt_cnt >=MODE_BUFFER_SIZE)
						{
							shunt_cnt =0;
							flag.flag_BIT.time_to_mode_cur =1;
						}
					
			//HAL_GPIO_WritePin(Finder_FAN_GPIO_Port,Finder_FAN_Pin,GPIO_PIN_RESET);
			return;
		}
	else if(hspi->Instance == SPI3)
		{
			odd_or_even =1;
			
			__IO uint8_t  *ptmpreg8;
			ptmpreg8 = (__IO uint8_t *)&hspi->Instance->DR;
			__HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));			
			__IO uint8_t  tmpreg8 = 0;
			 __HAL_SPI_DISABLE(hspi);
			while ((hspi->Instance->SR & SPI_FLAG_FRLVL) != SPI_FRLVL_EMPTY)
				{
					if (SPI_FLAG_FRLVL == SPI_SR_FRLVL)
					{
						/* Flush Data Register by a blank read */
						tmpreg8  = *ptmpreg8;
						UNUSED(tmpreg8);					
					}
				}
				

				hspi->State = HAL_SPI_STATE_READY;
				validate_battery.adc_buff[0] = temp_adc2.adc_buff[1];
				validate_battery.adc_buff[1] = temp_adc2.adc_buff[0];	
				
				time_reverse_cnt++;
				if(validate_battery.ADC_DATA >65000)   // no ac vltage connected
					{
						over_voltage_counter++;												
						if(over_voltage_counter > 500)
							{								
								over_voltage_counter =0;
								Error_code = NO_POWER;
								MY_Error_HANDLE(NO_POWER);
								__HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);
								HAL_TIM_Base_Stop_IT(&htim10);
								return;		
							}
						return;
					}
				else if(validate_battery.ADC_DATA <32500)
					{
						reverse_voltage_error_count++;
						if(reverse_voltage_error_count > 250)
							{
								flag.flag_BIT.RV_not_check =1;
								reverse_voltage_error_count =0;
								if(flag.flag_BIT.reerse_before_start==0) // reverse voltage before start
									{
										flag.flag_BIT.cheach_reverse_voltage_done =0;
										Error_code = reverse_voltage;
										__HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);
										HAL_TIM_Base_Stop_IT(&htim10);
									}
								else
									{
									Error_code = reverse_voltage;
									MY_Error_HANDLE(reverse_voltage);
									return;
									}
							}
						return;
					}
				else 
					{
						if(validate_battery.ADC_DATA<32767)
							{
								validate_battery.ADC_DATA =0;
							}
						else
							{
								validate_battery.ADC_DATA = validate_battery.ADC_DATA -32767;								
							}
							
				reverse_voltage_error_count =0;				
				voltage_buffer[battery_cnt] = validate_battery.ADC_DATA;
				battery_cnt++;
				if(battery_cnt >=MODE_BUFFER_SIZE)
					{
						battery_cnt =0;
						flag.flag_BIT.time_to_mode_bat =1;
					}
				}
				//HAL_GPIO_WritePin(Finder_FAN_GPIO_Port,Finder_FAN_Pin,GPIO_PIN_RESET);
				return;
				}
		
			}
void trigg_spi(SPI_HandleTypeDef *hspi)
{
				//delay 1 us
									// delay for activate clock after seting CS

					
					hspi->State       = HAL_SPI_STATE_BUSY_RX;
					// be ko ja bayad dade rikhte beshe
					if(hspi->Instance == SPI1)
						{
						hspi->pRxBuffPtr  = (uint8_t *)temp_adc1.adc_buff;
						hspi->RxXferSize  = 2;
						hspi->RxXferCount = 2;
						__HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));
						__HAL_SPI_ENABLE(hspi);
						return;
						}
					else
						{
							hspi->pRxBuffPtr  = (uint8_t *)temp_adc2.adc_buff;
							hspi->RxXferSize  = 2;
							hspi->RxXferCount = 2;
							__HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));
							__HAL_SPI_ENABLE(hspi);
							return;
						}

					


}
void trigger_rst(void)
	{
		if(HAL_GPIO_ReadPin(GATE_OF_ALARM_GPIO_Port,GATE_OF_ALARM_Pin)==0)
		{
		flag.flag_BIT.check_gate_over_volage =1;
		{
				uint8_t start_trigger_count=0;
				while(start_trigger_count<10)
					{
						HAL_GPIO_WritePin(GATE_OF_RST_GPIO_Port,GATE_OF_RST_Pin, GPIO_PIN_RESET);																		
						delayMS(100);
						
						HAL_GPIO_WritePin(GATE_OF_RST_GPIO_Port,GATE_OF_RST_Pin, GPIO_PIN_SET);
						delayMS(100);
						DAC_OFF();
						start_trigger_count++;
						if(HAL_GPIO_ReadPin(GATE_OF_ALARM_GPIO_Port,GATE_OF_ALARM_Pin))
							{
								break;
							}
					}
			}
		
		delayMS(100);
			
		}
	}
void delayUS(uint32_t us)
	{   // Sets the delay in microseconds.
	//uint8_t tim = 0;
		__HAL_TIM_SET_COUNTER(&htim7,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim7) < us);
	}	
	
void delayMS(uint32_t us)
	{   // Sets the delay in microseconds.
	//uint8_t tim = 0;
		uint16_t ms =us;
		uint16_t counter=0;
		while(counter<ms)
		{
		counter++;	
		delayUS(50000);
		delayUS(50000);
		}
		}	
		
void INIT_STATE(void)
{
		HAL_TIM_Base_Stop_IT(&htim5);	
	__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);	
	shunt_send_data =0;
	battery_send_data=0;
	battery_cnt =0;
	shunt_cnt =0;
	flag.flag_BIT.time_to_mode_bat =0;
	flag.flag_BIT.time_to_mode_cur =0;
	flag.flag_BIT.mode_bt_has_done =0;
	flag.flag_BIT.mode_cu_has_done =0;
	flag.flag_BIT.wait_to_run_erro =0;
	flag.flag_BIT.fire_or_not=0;
	//test
	timer6_counter =0;
	time_reverse_cnt =0;
	flag.flag_BIT.FAN_IS_ON=0;
	flag.flag_BIT.check_gate_over_volage =1;
	DAC_OFF();
	fan_run =0;
	last_cr_value = 100000;
	flag.flag_BIT.ask_data_trigger =0;
	trigger_counter =0;
	flag.flag_BIT.run_or_stop = 0;
	not_syncronize_signal =0;
	over_voltage_counter =0;
	flag.flag_BIT.RV_not_check=0;
	t_sum=0;
	flag.flag_BIT.reerse_before_start=0;
	flag.flag_BIT.timer9_start =0;
	__HAL_TIM_CLEAR_IT(&htim9,TIM_IT_UPDATE);				
	HAL_TIM_Base_Stop_IT(&htim9);	
	__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);				
	HAL_TIM_Base_Stop_IT(&htim6);		
	__HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);				
	HAL_TIM_Base_Stop_IT(&htim10);	
	__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);	
	HAL_GPIO_WritePin(CONTROL_GAIN_GPIO_Port,CONTROL_GAIN_Pin,GPIO_PIN_RESET);
	flag.flag_BIT.online_test_config_first_time =0;
	flag.flag_BIT.start_level_counter =0;
	HAL_TIM_Base_Stop_IT(&htim5);
	shunt_send_data =0;
	battery_send_data=0;
	DAC_OFF();
	delayMS(1);
	HAL_GPIO_WritePin(Relay_output_GPIO_Port,Relay_output_Pin,GPIO_PIN_RESET);
	delayMS(1);
	CONTACTOR_OFF;	
	dac_step_counter =1;
	system_state = init;
	system_step  = step1; 
	executing_adc_valu.ADC_DATA=0;
	mean_voltage.data=0;
	mean_shunt.data=0;
	flag.flag_BIT.start_process_counter =0;
	flag.flag_BIT.cheach_reverse_voltage_done =0;
	TIM6->CNT=0;
	TIM8->CNT=0;
	TIM3->CNT=0;
	//FAN_OFF;
	//turn off all the timer
	DAC_OFF();
	CONTACTOR_OFF;
	
}	

void MY_Error_HANDLE(uint16_t error_code)
{
// PC6 FAN
// PC7 Auxilary_contactor
// PA10 gate over voltage	
	switch(error_code)
		{	
			
			case 0: // can ERROR
					
					INIT_STATE();
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

			return;

			default:
					{
						__HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);				
						HAL_TIM_Base_Stop_IT(&htim10);
						__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);				
						HAL_TIM_Base_Stop_IT(&htim5);
						uint8_t data[3] = {0xFF ,MODULE_BOARD_ADDRESS, error_code};	
						CAN_SEND_DATA(data,MAIN_FAULT_ADDRESS,3);
						DAC_OFF();
						delayUS(delay_sample_rate);
						CAN_SEND_DATA(data,MAIN_FAULT_ADDRESS,3);
							//delayUS(10);
						//delay must greather than 1 ms					
						CONTACTOR_OFF;					
						system_state =init;
						TIM4->CNT=0;
					fna_turn_off_counter =0;
					flag.flag_BIT.fan_turn_off_flag =1;
					__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
					__HAL_TIM_ENABLE(&htim4);	
					}
			return;
		}
//					INIT_STATE();
//					FAN_ON;
//					fan_duty =80;
//					FAN_CONFIG(fan_duty);
//					TIM4->CNT=0;
//					HAL_TIM_Base_Start_IT(&htim4);
	
	
//	dac turn off
//	contactoroff
//	fan max level 
//  start 1m timer
// 	find what cause the error (switch case)
// return to init state	
	
	
}

void DAC_CONFIG(float new_value,float  last_value)
{	
	
		uint16_t  cal_new_dac_val;
		float 		changing_step;
		uint16_t  cal_last_dac_val;
		if(test_parameter_ptr->detail.operation==0)//CC
				{
					
					//Full scale gain = 70 amp / 65536  = 936
					cal_new_dac_val=   ((uint16_t)(FULL_SCALE_GAIN * new_value)) +beta_calibrition_coeffitiont_CC;
					cal_last_dac_val = ((uint16_t)(FULL_SCALE_GAIN * last_value)) +beta_calibrition_coeffitiont_CC;
					changing_step= (cal_new_dac_val-cal_last_dac_val )/DAC_CHANGE_STEP_COUNT;	
					for(uint8_t cnt=0;cnt<DAC_CHANGE_STEP_COUNT;cnt++ )
						{
							dac_change_value[cnt]	=cal_last_dac_val+ ((cnt+1)*(changing_step));
						}
					dac_change_value[DAC_CHANGE_STEP_COUNT-1] = cal_new_dac_val;
					executing_adc_valu.ADC_DATA = dac_change_value[0];
				}
		else // CR
				{				// alpha(Vadc/ valueA) + betA
					
					//cal_new_val = alpha_calibrition_coeffitiont_CR*((battery_voltage_ptr->ADC_DATA)/new_value) + beta_calibrition_coeffitiont_CR; 
					cal_new_dac_val  = ((uint16_t)(FULL_SCALE_GAIN *(mean_voltage.data/new_value))) +beta_calibrition_coeffitiont_CC;
					cal_last_dac_val = ((uint16_t)(FULL_SCALE_GAIN *(mean_voltage.data/last_value) )) +beta_calibrition_coeffitiont_CC;
					new_resistance   =  new_value;
					changing_step    = (cal_new_dac_val - cal_last_dac_val)/DAC_CHANGE_STEP_COUNT; 
					for(uint8_t cnt=0;cnt<DAC_CHANGE_STEP_COUNT;cnt++ )
						{
							dac_change_value[cnt]	=cal_last_dac_val+ ((cnt+1)*(changing_step));
						}
					dac_change_value[DAC_CHANGE_STEP_COUNT-1] = cal_new_dac_val;

					executing_adc_valu.ADC_DATA =dac_change_value[0];
				}
				


							
}
void dac_set_value(ADC_BUFFER_t dac_value)
{

//							GPIOB->ODR &= 0xBFFF;			//select chip select							
//							while(!((SPI2->SR)&(1<<1))){};				
//							SPI2->DR = dac_value.adc_buff[0];
//							while(!((SPI2->SR)&(1<<1))){};
//							SPI2->DR =dac_value.adc_buff[1];
//							while (!((SPI2->SR)&(1<<1))){};		//wait for transmitt ended
//							while (((SPI2->SR)&(1<<7))){};
//							uint8_t temp = SPI2->DR;		//empty transmit buffer
//							temp = SPI2->SR;			
//							GPIOB->ODR |= 0x4000;		//select chip select
//								
													ADC_BUFFER_t temp_dac;
	
								if(dac_value.ADC_DATA>DAC_LIMITATION)
									{
										return;
									}
								else
							{
							GPIOB->ODR &= 0xBFFF;			//select chip select	
							temp_dac.adc_buff[0] = dac_value.adc_buff[1];
							temp_dac.adc_buff[1] = dac_value.adc_buff[0];

							HAL_SPI_Transmit(&hspi2, temp_dac.adc_buff, 2, 10);	
							GPIOB->ODR |= 0x4000;		//select chip select		
							}
}


void START_STATE(void)
		{
			process_buffer_ptr =	&process_buffer;
		
		}
		
uint32_t set_process(process_excute_t *command)
		{
			
			
			if(command->detail.total_time - passed_time >0)
					{
						switch(set_process_step)
						{
							case step1: // time A pass 
												
												DAC_CONFIG(command->detail.valueB, command->detail.valueA);												
												dac_set_value(executing_adc_valu);
												set_process_step =step2;
												passed_time +=command->detail.timeB;
												return command->detail.timeB/50;	
								
							case step2:  // time B pass 
											if(command->detail.timeC>0)
												{													
													DAC_CONFIG(command->detail.valueC,  command->detail.valueB);
													dac_set_value(executing_adc_valu);
													set_process_step= step3;
													passed_time +=command->detail.timeC;
													return command->detail.timeC/50;	
												}
											else
												{													
													DAC_CONFIG(command->detail.valueA, command->detail.valueB);
													dac_set_value(executing_adc_valu);
													set_process_step = step1;
													passed_time +=command->detail.timeA;
													return command->detail.timeA/50;	
												}
								default:	// time c pass													
													DAC_CONFIG(command->detail.valueA, command->detail.valueC);
													dac_set_value(executing_adc_valu);
													set_process_step = step1;
													passed_time +=command->detail.timeA;
													return command->detail.timeA/50;	
							
						}
					}
			else
				{
					step_count++;
					// check if next step is end
					executing_command = process_buffer.process_buffer[step_count];
					if(executing_command.detail.test_mode==2) // END PROCEss 
						{
						uint8_t data[2] = {0xFE, MODULE_BOARD_ADDRESS};	
						CAN_SEND_DATA(data,MAIN_FAULT_ADDRESS,2);
						INIT_STATE();	
						delayUS(delay_sample_rate);
						CAN_SEND_DATA(data,MAIN_FAULT_ADDRESS,2);
						fna_turn_off_counter=0;
						
					flag.flag_BIT.fan_turn_off_flag =1;
					__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
					__HAL_TIM_ENABLE(&htim4);		
					//__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
					TIM4->CNT=0;
					__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
					__HAL_TIM_ENABLE(&htim4);
					
						
						return 0; 
						}
					else{	
					switch(set_process_step)
						{
						case step1:
							DAC_CONFIG(executing_command.detail.valueA, command->detail.valueA);
							break;
						case step2:
							DAC_CONFIG(executing_command.detail.valueA, command->detail.valueB);
							break;
						default:
							DAC_CONFIG(executing_command.detail.valueA, command->detail.valueC);
							break;
						}
					set_process_step = step1;
					passed_time = executing_command.detail.timeA;
					dac_set_value(executing_adc_valu);
						return command->detail.timeA/50;	
					
					}
				}
		}

void DAC_OFF(void)
{

//								
//							GPIOB->ODR &= 0xBFFF;			//select chip select				
//							while(!((SPI2->SR)&(1<<1))){};				
//							SPI2->DR =0;
//							while(!((SPI2->SR)&(1<<1))){};
//							SPI2->DR =0;
//							while (!((SPI2->SR)&(1<<1))){};		//wait for transmitt ended
//							while (((SPI2->SR)&(1<<7))){};
//							uint8_t temp = SPI2->DR;		//empty transmit buffer
//							temp = SPI2->SR;			
//							GPIOB->ODR |= 0x4000;			//select chip select
//								
							ADC_BUFFER_t temp_dac;
							GPIOB->ODR &= 0xBFFF;			//select chip select	
							temp_dac.ADC_DATA=0;
								
							HAL_SPI_Transmit(&hspi2, temp_dac.adc_buff, 2, 10);	
							GPIOB->ODR |= 0x4000;		//select chip select
}
uint8_t Error_cheack(void)
	{
		uint8_t error_code =0 ;
//		if(HAL_GPIO_ReadPin(uGPI1_GPIO_Port,uGPI1_Pin)==0)
//		{
//			
//			error_code = FAN_TURN_OFF;
//			return error_code;
//		}
//		else 
		if(HAL_GPIO_ReadPin(GATE_OF_ALARM_GPIO_Port,GATE_OF_ALARM_Pin)==0)
		{
			error_code = gate_over_voltage;
			return error_code;
		}
		// reverse voltage
		//	over_voltage
		
		
		
		return error_code;
	
	}
static void TIM_OC1_SetConfig1(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Disable the Channel 1: Reset the CC1E Bit */
  TIMx->CCER &= ~TIM_CCER_CC1E;

  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;
  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;

  /* Get the TIMx CCMR1 register value */
  tmpccmrx = TIMx->CCMR1;

  /* Reset the Output Compare Mode Bits */
  tmpccmrx &= ~TIM_CCMR1_OC1M;
  tmpccmrx &= ~TIM_CCMR1_CC1S;
  /* Select the Output Compare Mode */
  tmpccmrx |= OC_Config->OCMode;

  /* Reset the Output Polarity level */
  tmpccer &= ~TIM_CCER_CC1P;
  /* Set the Output Compare Polarity */
  tmpccer |= OC_Config->OCPolarity;

  if (IS_TIM_CCXN_INSTANCE(TIMx, TIM_CHANNEL_1))
  {
//    /* Check parameters */
//    assert_param(IS_TIM_OCN_POLARITY(OC_Config->OCNPolarity));

    /* Reset the Output N Polarity level */
    tmpccer &= ~TIM_CCER_CC1NP;
    /* Set the Output N Polarity */
    tmpccer |= OC_Config->OCNPolarity;
    /* Reset the Output N State */
    tmpccer &= ~TIM_CCER_CC1NE;
  }

  if (IS_TIM_BREAK_INSTANCE(TIMx))
  {
//    /* Check parameters */
//    assert_param(IS_TIM_OCNIDLE_STATE(OC_Config->OCNIdleState));
//    assert_param(IS_TIM_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare and Output Compare N IDLE State */
    tmpcr2 &= ~TIM_CR2_OIS1;
    tmpcr2 &= ~TIM_CR2_OIS1N;
    /* Set the Output Idle state */
    tmpcr2 |= OC_Config->OCIdleState;
    /* Set the Output N Idle state */
    tmpcr2 |= OC_Config->OCNIdleState;
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;

  /* Write to TIMx CCMR1 */
  TIMx->CCMR1 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TIMx->CCR1 = OC_Config->Pulse;

  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}
void FAN_CONFIG(uint8_t duty)
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
//  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
//  {
//    Error_Handler();
//  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (uint32_t)(round((double)((htim14.Init.Period/100.0)*duty)));
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	
	TIM_OC1_SetConfig1(htim14.Instance, &sConfigOC);

	/* Set the Preload enable bit for channel1 */
	htim14.Instance->CCMR1 |= TIM_CCMR1_OC1PE;

	/* Configure the Output Fast mode */
	htim14.Instance->CCMR1 &= ~TIM_CCMR1_OC1FE;
	htim14.Instance->CCMR1 |= sConfigOC.OCFastMode;
	
//  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
 // HAL_TIM_MspPostInit(&htim14);



}

uint8_t 	one_min_error_cheak	(void)
{
uint8_t 								error=0;	
												
static 									uint8_t					  	 fan_reduce_time_of_update =0;		
static 									uint16_t 						OPP_wait		=0;	
static 									uint16_t 						OVP_wait		=0;	
static 									uint16_t 						OCP_wait		=0;	
register double 							power_mean 					=0;
power_mean   =	mean_voltage.data * mean_shunt.data;
uint8_t  new_fan_value ;
								  											
	
if(fan_run> 5000)
{	
		if(HAL_GPIO_ReadPin(GATE_OF_ALARM_GPIO_Port,GATE_OF_ALARM_Pin)==0)
		{
				MY_Error_HANDLE(gate_over_voltage);		
				Error_code =gate_over_voltage;
				return error;
		}		
	flag.flag_BIT.check_gate_over_volage =0;	
	fan_read[fan_counter] = HAL_GPIO_ReadPin(FAN_DETECT_GPIO_Port,FAN_DETECT_Pin);	
	fan_counter++;
	
	if(fan_counter >=100)
		{
		register												uint8_t    					zero_erro_count 		=0;
		register												uint8_t    					one_erro_count 			=0;	

			for(register uint8_t fast_cnt=0;fast_cnt<100;fast_cnt++)
				{
					if(fan_read[fast_cnt] ==1)
						{
						one_erro_count++;			
						}
					else
						{
						zero_erro_count++;
						}
				}
			fan_counter=0;
			fan_reduce_time_of_update++;
			if((one_erro_count>90) ||(zero_erro_count >90))
			{
			// fan turn off and we have error
					MY_Error_HANDLE(FAN_TURN_OFF);
					error=1;
					Error_code =FAN_TURN_OFF;
					return error;	
			}
			{
			if(fan_reduce_time_of_update>10)
				{
					fan_reduce_time_of_update =0;
					new_fan_value = ((0.25*power_mean) + 5);	
					if(power_mean <100)
						{
								//delayUS(5);
								HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
								FAN_CONFIG(last_fan_value);
								
								TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
								__HAL_TIM_ENABLE(&htim14);
								HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
								fan_run=0;
								fan_counter=0;
						}
					else
						{
							
							if(new_fan_value >=last_fan_value)
								{
									//FAN_ON;
									HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
									last_fan_value = new_fan_value;
									//delayUS(5);
									FAN_CONFIG(last_fan_value);
									
									TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
									__HAL_TIM_ENABLE(&htim14);
									HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
									fan_run=0;
									fan_counter=0;
								}
							else
								{									
									//delayUS(5);
									HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
									FAN_CONFIG(last_fan_value);									
									TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
									__HAL_TIM_ENABLE(&htim14);
									HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
									fan_run=0;
									fan_counter=0;
								
								}

						}	
				}
			}
		}		
	}
else
{
fan_run++;
}





	
	
	//OVP
	if(mean_voltage.data > OVP_PROTECTION)
		{
		OVP_wait++;
		if(OVP_wait ==OVP_WAIT_TIME)
			{
				// fault
				OVP_wait= 0;
				MY_Error_HANDLE(OVP);
				error =1;
				Error_code =OVP;
				return error;
			}
		}
	else
			{
				OVP_wait = 0;
			
			}
	//OCP
		if(mean_shunt.data > current_protection)
		{
		OCP_wait++;
		if(OCP_wait ==OCP_WAIT_TIME)
			{
				//fault
				OCP_wait =0;
				MY_Error_HANDLE(OCP);
				error =1;
				Error_code = OCP;
				return error;
			}
		}
		else
				{
					OCP_wait = 0;
				
				}

	//OPP
		if(power_mean > power_protection)
		{
		OPP_wait++;
		if(OPP_wait ==OPP_WAIT_TIME)
			{
				//fault
				OPP_wait =0;
				MY_Error_HANDLE(OPP);
				Error_code = OPP;
				error =1;
				return error;
			}
		}
		else
				{
					OPP_wait = 0;
				
				}
			return error;
}

HAL_StatusTypeDef MY_RECIVE_FIFO0(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
  HAL_CAN_StateTypeDef state = hcan->State;




    /* Get the header */
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
      //SET_BIT(hcan->Instance->RF1R, CAN_RF1R_RFOM1);
    

    /* Return function status */
    return HAL_OK;
 
}

HAL_StatusTypeDef MY_RECIVE_FIFO1(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
  HAL_CAN_StateTypeDef state = hcan->State;




    /* Get the header */
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


//uint32_t flash_read(uint32_t address){
//		uint32_t init_address = 0x8000000 + 0x9C40;
//		address = init_address +address;
//    return *(uint32_t*)address;
//}

//void flash_write(uint32_t address, uint32_t data){
//    uint32_t init_address = 0x8000000 + 0x9C40;
//		address = init_address +address;
//	
////		HAL_FLASH_Unlock();
////    FLASH_Erase_Sector(11U,VOLTAGE_RANGE_3);
////    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,data);
////    HAL_FLASH_Lock();
//	
//	FLASH_EraseInitTypeDef Eraseinit;
//	uint32_t		PAGEError =0;
//	
//	HAL_FLASH_Unlock();
////	Eraseinit.TypeErase = FLASH_TYPEERASE_SECTORS;
////	Eraseinit.NbSectors =1;
////	Eraseinit.VoltageRange= FLASH_VOLTAGE_RANGE_3;
////	Eraseinit.Sector =FLASH_SECTOR_3;
////	
////	
////	HAL_FLASHEx_Erase(&Eraseinit,&PAGEError);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,data);
//	HAL_FLASH_Lock();
//	


//}


HAL_StatusTypeDef MY_SEND_CAN(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
{
  uint32_t transmitmailbox;
  HAL_CAN_StateTypeDef state = hcan->State;
  uint32_t tsr = READ_REG(hcan->Instance->TSR);


      /* Select an empty transmit mailbox */
      transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

      /* Check transmit mailbox value */
      if (transmitmailbox > 2U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_INTERNAL;

        return HAL_ERROR;
      }

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

  





/*
	#include "FUNCTION.h"
	SPI_handeler(hspi);
	return;




*/

//	static void SPI_CloseRx_ISR(SPI_HandleTypeDef *hspi)
//{
//  /* Disable RXNE and ERR interrupt */
//	__IO uint8_t  *ptmpreg8;
//	ptmpreg8 = (__IO uint8_t *)&hspi->Instance->DR;
//  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));
//	uint8_t temp;
//	__IO uint8_t  tmpreg8 = 0;
//   __HAL_SPI_DISABLE(hspi);

//	//SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_DEFAULT_TIMEOUT, HAL_GetTick());
//	
//	
//	while ((hspi->Instance->SR & SPI_FLAG_FRLVL) != SPI_FRLVL_EMPTY)
//  {
//    if (SPI_FLAG_FRLVL == SPI_SR_FRLVL)
//    {
//      /* Flush Data Register by a blank read */
//      tmpreg8  = *ptmpreg8;
//      UNUSED(tmpreg8);
//      
//    }
//	}
//	hspi->State = HAL_SPI_STATE_READY;

//#if (USE_SPI_CRC != 0U)
//  /* Check if CRC error occurred */
//  if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET)
//  {
//    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
//    __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
//    /* Call user error callback */
//#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
//    hspi->ErrorCallback(hspi);
//#else
//    HAL_SPI_ErrorCallback(hspi);
//#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
//  }
//  else
//  {
//#endif /* USE_SPI_CRC */

//      /* Call user Rx complete callback */
//#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
//      hspi->RxCpltCallback(hspi);
//#else

//						GPIOD->ODR |= 0x0004;
//			
//			dac.adc_buff[0] = adc.adc_buff[1];
//			dac.adc_buff[1] = adc.adc_buff[0];

//			
//				delayUS(17);


//			GPIOD->ODR &= 0xFFFB;
//							
//	
////			
//	
//			delayUS(2);
//			hspi->State       = HAL_SPI_STATE_BUSY_RX;
//			hspi->pRxBuffPtr  = (uint8_t *)adc.adc_buff;
//			hspi->RxXferSize  = 2;
//			hspi->RxXferCount = 2;
//				 __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));
//				 __HAL_SPI_ENABLE(hspi);
//#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

//#if (USE_SPI_CRC != 0U)
//  }
//#endif /* USE_SPI_CRC */
//}


//void DAC_CONFIG(uint8_t  *buffer)
//{

	
	
	
	
	
//				dac.ADC_DATA= (uint16_t)((65536/2.5)*dac_config.volt);
//	 

//				TIM_OC_InitTypeDef sConfigOC = {0};
//				htim13.Instance = TIM13;
//				htim13.Init.Prescaler = 108-1;
//				htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
//				htim13.Init.Period = ((uint32_t)(1000000/dac_config.freq)-1);
//				htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//				htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//				if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
//				{
//					Error_Handler();
//				}
//				if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
//				{
//					Error_Handler();
//				}
//				
//				sConfigOC.OCMode = TIM_OCMODE_PWM1;
//				sConfigOC.Pulse =(uint32_t)(round((double)((htim13.Init.Period/100.0)*dac_config.duty)));
//				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//				if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//				{
//					Error_Handler();
//				}
			
//}



