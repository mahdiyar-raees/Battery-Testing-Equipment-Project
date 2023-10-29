
#include <FUNCTION.h>
#include "main.h"

#include <KEY.H>


uint8_t								choose_prof_counter=0;
process_excute_t			tx_buff;
uint8_t 							dot_counter=0;
uint8_t 							SEG_ARRAY[7];
uint8_t								last_position=0;


uint8_t       				integer[4]; //LCD integer part
uint8_t       				fractal[2];	//LCD fractal PART

							


uint8_t 							LCD_BUFFER[6];	

uint8_t   						module_counter;	
MODULE_t      				module[6];
UP_DOWN_t     				first_up_down;
INPUT_UP_DOWN_t 			second_up_down;	

TRAN_MENU_t 					tran_menu;
CURRENT_MENU_t				current_menu;
uint8_t 							function_code;

// DECLARATION

void 			home_menue(uint8_t key);

void 			choose_module(uint8_t key);

void			choose_opperation(uint8_t key);

void			choose_prof(uint8_t key);

void 			measure_prof(uint8_t key);

void 			show_data(uint8_t key);

void 			uart_init(void);


/// DEFENITION
void 	reset_state(void)
	{
		LED_CC_OFF;
		LED_CR_OFF;
		flag.BIT.first_one_cal_save =0;
		reset_counter--;
		flag.BIT.initial_coff_ask = 0;
		timer4_counter = 0;
		timer5_counter = 0;
		LED_emerg_OFF;
		LED_E1_OFF;
		timer6_counter = 0;
		__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
		HAL_TIM_Base_Stop_IT(&htim6);			
		__HAL_TIM_CLEAR_IT(&htim4,TIM_IT_UPDATE);
		HAL_TIM_Base_Stop_IT(&htim4);	
		__HAL_TIM_CLEAR_IT(&htim5,TIM_IT_UPDATE);
		HAL_TIM_Base_Stop_IT(&htim5);		
		flag.BIT.key_bunce =1;		
		SEG_menue_level=0;
		flag.BIT.command_or_data =0;
		segment_clear();
		segment_put("COMMAND",0);
		panel_command_counter =0;
		timer6_wait =8;
		if(reset_counter ==0)
			{
			
			uint8_t temp;	
			temp = USART1->DR;
			HAL_UART_Transmit(&huart1,&temp,1,10);	
			reset_counter = 100;		
			}
			uint8_t temp;	
			temp = USART1->DR;
			HAL_UART_Transmit(&huart1,&temp,1,10);	
		for(register uint8_t fast_cnt =0;fast_cnt<6;fast_cnt++)
			{
				panel_send.detail.availabele_board[fast_cnt] =0;
			}
		flag.BIT.DOT_EXIS =0;
			
			

//for(uint8_t counter =0;counter<100;counter++)
//	{
//		temp = USART1->DR;
//		HAL_UART_Transmit(&huart1,&temp,1,10);
//	}			
//	
//HAL_UART_Abort_IT(&huart1);
//for(uint8_t counter =0;counter<100;counter++)
//	{
//		temp = USART1->DR;
//		HAL_UART_Transmit(&huart1,&temp,1,10);
//	}				
//uart_init();
//	HAL_UART_Abort_IT(&huart1);
//	uart_init();
//for(uint8_t counter =0;counter<100;counter++)
//	{
//		temp = USART1->DR;
//	}			
//			

//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);			
	}

float BCD_to_float(void)
	{
		
		float 	 frac;	
		float			frac1 = SEG_ARRAY[2];
		float			frac2 = SEG_ARRAY[1];
		float			frac3 = SEG_ARRAY[0];
		
		frac1 = frac1/10;
		frac2 = frac2/100;
		frac3 = frac3/1000;
		frac = SEG_ARRAY[3] + (SEG_ARRAY[4] * 10) + (SEG_ARRAY[5] * 100) + (SEG_ARRAY[6] * 1000) + frac1 + frac2 + frac3;

		
		return frac;
	}

	
uint16_t float_to_BCD( float number)
		{
			uint16_t  output=0;
			uint16_t real_num;
			float    float_num;
			
			real_num  = (uint16_t )number;
			if(real_num > 0)
			{
			float_num = number - real_num;
			}
			else
			{
			float_num = number;
			}
			SEG_ARRAY[0] =  ((uint8_t )( float_num * 1000)%10);
			SEG_ARRAY[1] =	((uint8_t )( float_num * 100)%10);					// second float
			SEG_ARRAY[2] = 	((uint8_t )( float_num * 10));				 //firt float 
			SEG_ARRAY[3] = 	( uint8_t )( real_num  % 10);			  	// first digit
			SEG_ARRAY[4] = 	( uint8_t )((real_num  % 100) /10);  // second digit
			SEG_ARRAY[5] = 	( uint8_t )((real_num  % 1000)/100); // third digit
			SEG_ARRAY[6] =  ( uint8_t )( real_num  / 1000);
		
			
			output = ((real_num + float_num) *100);
			return output;
		}

void show_num(uint8_t cell_num)
	{
		
		segment_clear();
		uint16_t temp ;
		uint8_t decimal_point;
		char str[3];
		if(cell_num == 6)
			{				
				segment_put("D",0);
				float_to_BCD(real_num[(cell_num*2)]);	 
				sprintf(str,"%d",SEG_ARRAY[5]);
				segment_put(str,2);
				sprintf(str,"%d",SEG_ARRAY[4]);
				segment_put(str,3);
				sprintf(str,"%d",SEG_ARRAY[3]);
				segment_put(str,4);
				last_ascii = str[0];
				segment_put_decmial(4);
				sprintf(str,"%d",SEG_ARRAY[2]);
				segment_put(str,5);
				sprintf(str,"%d",SEG_ARRAY[1]);
				segment_put(str,6);							
				return;			
			}
		else
			{
				sprintf(str,"%d",cell_num+1);
				segment_put(str,0);
				
				


				float_to_BCD(real_num[(cell_num*2)+1]);	 
				sprintf(str,"%d",SEG_ARRAY[5]);
				segment_put(str,2);
				sprintf(str,"%d",SEG_ARRAY[4]);
				segment_put(str,3);
				sprintf(str,"%d",SEG_ARRAY[3]);
				segment_put(str,4);
				last_ascii = str[0];
				segment_put_decmial(4);
				sprintf(str,"%d",SEG_ARRAY[2]);
				segment_put(str,5);
				sprintf(str,"%d",SEG_ARRAY[1]);
				segment_put(str,6);		
				
				
				


				float_to_BCD(real_num[(cell_num*2)]);
				sprintf(str,"%d",SEG_ARRAY[5]);
				segment_put(str,9);
				sprintf(str,"%d",SEG_ARRAY[4]);
				segment_put(str,10);
				sprintf(str,"%d",SEG_ARRAY[3]);
				segment_put(str,11);
				last_ascii = str[0];
				segment_put_decmial(11);
				sprintf(str,"%d",SEG_ARRAY[2]);
				segment_put(str,12);		
				sprintf(str,"%d",SEG_ARRAY[1]);
				segment_put(str,13);	


				return;
			}
	}
void SEG_NUMBER_MANAGER(uint8_t LCD_KEYS)
			{
				if(flag.BIT.DOT_EXIS)
						{	
							switch(dot_counter)
								{
									case 0:
										dot_counter++;
										SEG_ARRAY[2] = LCD_KEYS;
									return;
									case 1:
										dot_counter++;
										SEG_ARRAY[1] = LCD_KEYS;
									return;
									case 2:
										dot_counter++;
										SEG_ARRAY[0] = LCD_KEYS;
									return;
								}
						}
				else
						{
							real_count++;
							SEG_ARRAY[6] = SEG_ARRAY[5];
							SEG_ARRAY[5] = SEG_ARRAY[4];
							SEG_ARRAY[4] = SEG_ARRAY[3];
							SEG_ARRAY[3] = LCD_KEYS;
						}
			}

	
			
void SEGMENT_MASTER(uint8_t key)
	{
		switch(SEG_menue_level)
			{
				case 0:
					{
						home_menue(key);					
					}
					return;
				case 1:
					{
						choose_module(key);
					}
					return;
				case 2:
					{
						choose_opperation(key);
					}
					return;
				case 3:
					{
						choose_prof(key);
					}
					return;
				case 100:
					{
						show_data(key);
					}
			}
	}
void measure_prof(uint8_t key)
	{
		switch(key)
			{
				case UP_k:
					{
					
					}
					return;
				case DOWN_k:
					{
					
					}
					return;
				case ENTER_k:
					{

					}
				}
	
	
	}
void choose_prof(uint8_t key)
	{
		if(flag.BIT.Enter_trigged)
			{
				if(key == BACK_K)// retun to last menue 
						{
													SEG_menue_level=1;
						for(register uint8_t fast_cnt =0;fast_cnt<6;fast_cnt++)
							{
								panel_send.detail.availabele_board[fast_cnt] =0;
							}
						segment_clear();
						segment_put("CHOOSE MODULE" ,0);
							
										flag.BIT.Enter_trigged =0;
						}
				else
					{
						switch(choose_prof_counter)
							{
								case 0: //MODE
									{
										if(key == UP_k)
											{
												if(process.detail.test_mode==1) // last menue was CC
													{
														process.detail.test_mode=0;
														segment_put("CC",6);
														LED_CC_ON;
														LED_CR_OFF;
													}
												else // last menue was CR
													{
														process.detail.test_mode=1;
														segment_put("CR",6);
														LED_CR_ON;
														LED_CC_OFF;
													}
											}										
										else if(key == DOWN_k)
											{
												if(process.detail.test_mode==0) // last menue was CC
													{
														process.detail.test_mode=1;
														segment_put("CR",6);
														LED_CC_OFF;
														LED_CR_ON;
													}
												else // last menue was CR
													{
														process.detail.test_mode=0;
														segment_put("CC",6);
														LED_CC_ON;
														LED_CR_OFF;
													}
											}
										else if(key == ENTER_k) // send value saved and return to main menue 
											{
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("MODE ",0);
											}

									}
									return;
								case 1:	// SAVE
										if(key == ENTER_k)
											{
												///***********************************///***********************************///
												
												uint8_t  temp=1;
												temp = 2;
												
												///***********************************///***********************************///
											}
									return;
								case 2: // VA
									{
										if(key == ENTER_k)
											{
												process.detail.valueA = BCD_to_float();
												// return to main menu
												memset(SEG_ARRAY,0,7);
												dot_counter =0;
												real_count =0;
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("VAL A",0);
											}
										else if(key == CLEARE_k)
											{
												flag.BIT.DOT_EXIS =0;
												last_position =5;
												real_count = 0;
												dot_counter =0;
												memset(SEG_ARRAY,0,7);
												segment_clear();
												segment_put("VALUE CLEAR",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("SET",0);
											}
										else if(key == DOT_k)
											{
												if(flag.BIT.DOT_EXIS ==0)
													{
														dot_counter =0;
														flag.BIT.DOT_EXIS =1;
														segment_put_decmial(last_position);
													}
											}	
										else if(key >= 0 & key <10)
											{
												uint8_t condition;
												condition = (flag.BIT.DOT_EXIS ==1 & dot_counter < 3) | (real_count <4 & flag.BIT.DOT_EXIS ==0);
												if(condition)
													{		
														last_position  = next_position(last_position);
														SEG_NUMBER_MANAGER(key);
														char str[3];
														sprintf(str,"%d",key);
														segment_put(str,last_position);
													}	
											}
									}
									return;	
								case 3:	// TIME A
									{
										if(key == ENTER_k)
											{
												float temp;
												temp = BCD_to_float();
												process.detail.timeA = 50 * temp;
												// return to main menu
												memset(SEG_ARRAY,0,7);
												dot_counter =0;
												real_count=0;
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("TIM A",0);
											}
										else if(key == CLEARE_k)
											{
												flag.BIT.DOT_EXIS =0;
												last_position =5;
												dot_counter =0;
												real_count =0;
												memset(SEG_ARRAY,0,7);
												segment_clear();
												segment_put("VALUE CLEAR",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("SET",0);
											}
										else if(key == DOT_k)
											{
												if(flag.BIT.DOT_EXIS==0)
													{
														dot_counter =0;
														flag.BIT.DOT_EXIS =1;
														segment_put_decmial(last_position);
													}
											}	
										else if(key >= 0 & key <10)
											{
												uint8_t condition;
												condition = (flag.BIT.DOT_EXIS ==1 & dot_counter < 3) | (real_count <4 & flag.BIT.DOT_EXIS ==0);
												if(condition)
													{
														last_position  = next_position(last_position);
														SEG_NUMBER_MANAGER(key);
														char str[3];
														sprintf(str,"%d",key);
														segment_put(str,last_position);
													}
											}
									}
									return;
								case 4: // VB
									{
										if(key == ENTER_k)
											{
												process.detail.valueB = BCD_to_float();
												// return to main menu
												memset(SEG_ARRAY,0,7);
												real_count =0;
												dot_counter =0;
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("VAl B",0);
											}
										else if(key == CLEARE_k)
											{
												flag.BIT.DOT_EXIS =0;
												last_position =5;
												dot_counter =0;
												real_count =0;
												memset(SEG_ARRAY,0,7);
												segment_clear();
												segment_put("VALUE CLEAR",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("SET",0);
											}
										else if(key == DOT_k)
											{
												if(flag.BIT.DOT_EXIS ==0)
													{
														dot_counter =0;
														flag.BIT.DOT_EXIS =1;
														segment_put_decmial(last_position);
													}
											}	
										else if(key >= 0 & key <10)
											{
												uint8_t condition;
												condition = (flag.BIT.DOT_EXIS ==1 & dot_counter < 3) | (real_count <4 & flag.BIT.DOT_EXIS ==0);
												if(condition)
													{
														last_position  = next_position(last_position);
														SEG_NUMBER_MANAGER(key);
														char str[3];
														sprintf(str,"%d",key);
														segment_put(str,last_position);
													}
											}
									}
									return;
								case 5:	// TIME B
									{
										if(key == ENTER_k)
											{
												float temp;
												temp = BCD_to_float();
												process.detail.timeB = 50 * temp;
												// return to main menu
												memset(SEG_ARRAY,0,7);
												dot_counter =0;
												real_count =0;
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("TIM B",0);
											}
										else if(key == CLEARE_k)
											{
												flag.BIT.DOT_EXIS =0;
												last_position =5;
												dot_counter =0;
												real_count=0;
												memset(SEG_ARRAY,0,7);
												segment_clear();
												segment_put("VALUE CLEAR",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("SET",0);
											}
										else if(key == DOT_k)
											{
												if(flag.BIT.DOT_EXIS)
													{
														dot_counter =0;
														flag.BIT.DOT_EXIS =1;
														segment_put_decmial(last_position);
													}
											}	
										else if(key >= 0 & key <10)
											{
												uint8_t condition;
												condition = (flag.BIT.DOT_EXIS ==1 & dot_counter < 3) | (real_count <4 & flag.BIT.DOT_EXIS ==0);
												if(condition)
													{
														last_position  = next_position(last_position);
														SEG_NUMBER_MANAGER(key);
														char str[3];
														sprintf(str,"%d",key);
														segment_put(str,last_position);
													}	
												}
									}
									return;
								case 6: // VC
									{
										if(key == ENTER_k)
											{
												process.detail.valueC = BCD_to_float();
												// return to main menu
												memset(SEG_ARRAY,0,7);
												dot_counter =0;
												real_count=0;
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("VAL C",0);
											}
										else if(key == CLEARE_k)
											{
												flag.BIT.DOT_EXIS =0;
												last_position =5;
												dot_counter =0;
												real_count=0;
												memset(SEG_ARRAY,0,7);
												segment_clear();
												segment_put("VALUE CLEAR",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("SET",0);
											}
										else if(key == DOT_k)
											{
												if(flag.BIT.DOT_EXIS ==0)
													{
														dot_counter =0;
														flag.BIT.DOT_EXIS =1;
														segment_put_decmial(last_position);
													}
											}	
										else if(key >= 0 & key <10)
											{
												uint8_t condition;
												condition = (flag.BIT.DOT_EXIS ==1 & dot_counter < 3) | (real_count <4 & flag.BIT.DOT_EXIS ==0);
												if(condition)
													{
														last_position  = next_position(last_position);
														SEG_NUMBER_MANAGER(key);
														char str[3];
														sprintf(str,"%d",key);
														segment_put(str,last_position);
													}
											}
									}
									return;
								case 7: // TIMEC
									{
										if(key == ENTER_k)
											{
												float temp;
												temp = BCD_to_float();
												process.detail.timeC = 50 * temp;
												// return to main menu
												memset(SEG_ARRAY,0,7);
												real_count=0;
												dot_counter =0;
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("TIM C",0);
											}
										else if(key == CLEARE_k)
											{
												flag.BIT.DOT_EXIS =0;
												last_position =5;
												real_count=0;
												dot_counter =0;
												memset(SEG_ARRAY,0,7);
												segment_clear();
												segment_put("VALUE CLEAR",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("SET ",0);
											}
										else if(key == DOT_k)
											{
												if(flag.BIT.DOT_EXIS==0)
													{
														dot_counter =0;
														flag.BIT.DOT_EXIS =1;
														segment_put_decmial(last_position);
													}
											}	
										else if(key >= 0 & key <10)
											{
												uint8_t condition;
												condition = (flag.BIT.DOT_EXIS ==1 & dot_counter < 3) | (real_count <4 & flag.BIT.DOT_EXIS ==0);
												if(condition)
													{
														last_position  = next_position(last_position);
														SEG_NUMBER_MANAGER(key);
														char str[3];
														sprintf(str,"%d",key);
														segment_put(str,last_position);
													}
											}
									}
									return;
								case 8: // pulse count
									{
										if(key == ENTER_k)
											{
												process.detail.total_time = (uint16_t)BCD_to_float();
												// return to main menu
												memset(SEG_ARRAY,0,7);
												dot_counter =0;
												real_count =0;
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("PULS",0);
											}
										else if(key == CLEARE_k)
											{
												flag.BIT.DOT_EXIS =0;
												last_position =5;
												dot_counter =0;
												real_count=0;
												memset(SEG_ARRAY,0,7);
												segment_clear();
												segment_put("VALUE CLEAR",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("SET",0);
											}
										else if(key >= 0 & key <10)
											{
												uint8_t condition;
												condition = (flag.BIT.DOT_EXIS ==1 & dot_counter < 3) | (real_count <4 & flag.BIT.DOT_EXIS ==0);
												if(condition)
													{
														last_position  = next_position(last_position);
														SEG_NUMBER_MANAGER(key);
														char str[3];
														sprintf(str,"%d",key);
														segment_put(str,last_position);
													}
											}
									}
									return;
								case 9: // DET T 
									{
										if(key == ENTER_k)
											{
												process.detail.null_detect = (uint16_t)BCD_to_float();
												// return to main menu
												memset(SEG_ARRAY,0,7);
												dot_counter =0;
												real_count =0;
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("DET T",0);
											}
										else if(key == CLEARE_k)
											{
												flag.BIT.DOT_EXIS =0;
												last_position =5;
												dot_counter =0;
												real_count=0;
												memset(SEG_ARRAY,0,7);
												segment_clear();
												segment_put("VALUE CLEAR",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("SET",0);
											}
										else if(key >= 0 & key <10)
											{
												uint8_t condition;
												condition = (flag.BIT.DOT_EXIS ==1 & dot_counter < 3) | (real_count <4 & flag.BIT.DOT_EXIS ==0);
												if(condition)
													{
														last_position  = next_position(last_position);
														SEG_NUMBER_MANAGER(key);
														char str[3];
														sprintf(str,"%d",key);
														segment_put(str,last_position);
													}
											}
									}
									return;
								case 10: // DET A
									{
										if(key == ENTER_k)
											{
												process.detail.pulse_count = (uint16_t)BCD_to_float();
												// return to main menu
												memset(SEG_ARRAY,0,7);
												dot_counter =0;
												real_count=0;
												flag.BIT.Enter_trigged =0;
												segment_clear();
												segment_put("VALUE SAVED",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("DET A",0);
											}
										else if(key == CLEARE_k)
											{
												flag.BIT.DOT_EXIS =0;
												last_position =5;
												dot_counter =0;
												real_count=0;
												memset(SEG_ARRAY,0,7);
												segment_clear();
												segment_put("VALUE CLEAR",0);
												delayUS(wait_menue_time);
												segment_clear();
												segment_put("SET",0);
											}
										else if(key >= 0 & key <10)
											{
												uint8_t condition;
												condition = (flag.BIT.DOT_EXIS ==1 & dot_counter < 3) | (real_count <4 & flag.BIT.DOT_EXIS ==0);
												if(condition)
													{
														last_position  = next_position(last_position);
														SEG_NUMBER_MANAGER(key);
														char str[3];
														sprintf(str,"%d",key);
														segment_put(str,last_position);
													}
											}
									}
									return;

							}		
					}
				}
		// maybe cheack if enter select or not 
		else 
			{
				switch(key)
					{
						case UP_k:
							{
								switch(choose_prof_counter)
									{
										case 0: //MODE
											choose_prof_counter=10;
											segment_put("DET A",0);											
											return;
										case 1:	// SAVE
											choose_prof_counter=0;
											segment_put("MODE ",0);
											return;
										case 2: // VA
											choose_prof_counter=1;
											segment_put("SAVE ",0);
											return;	
										case 3:	// TIME A
											choose_prof_counter=2;
											segment_put("VAL A",0);
											return;
										case 4: // VB
											choose_prof_counter=3;
											segment_put("TIM A",0);
											return;
										case 5:	// TIME B
											choose_prof_counter=4;
											segment_put("VAL B",0);
											return;
										case 6: // VC
											choose_prof_counter=5;
											segment_put("TIM B",0);
											return;
										case 7: // TIMEC
											choose_prof_counter=6;
											segment_put("VAL C",0);
											return;
										case 8: // pulse count
											choose_prof_counter=7;
											segment_put("TIM C",0);
											return;
										case 9:
											choose_prof_counter =8;										
											segment_put("PULS ",0);
											return;
										case 10:
											choose_prof_counter=9;
											segment_put("DET T",0);											
											return;
											return;
									}
							}
							return;
						case DOWN_k:
							{
								switch(choose_prof_counter)
									{
										case 0: //MODE
											choose_prof_counter=1;
											segment_put("SAVE ",0);
											return;
										case 1:	// SAVE
											choose_prof_counter=2;
											segment_put("VAL A",0);
											return;
										case 2: // VA
											choose_prof_counter=3;
											segment_put("TIM A",0);
											return;
										case 3:	// TIME A
											choose_prof_counter=4;
											segment_put("VAL B",0);
											return;
										case 4: // VB
											choose_prof_counter=5;
											segment_put("TIM B",0);
											return;
										case 5:	// TIME B
											choose_prof_counter=6;
											segment_put("VAL C",0);
											return;
										case 6: // VC
											choose_prof_counter=7;
											segment_put("TIM C",0);
											return;
										case 7: // TIMEC
											choose_prof_counter=8;
											segment_put("PULS ",0);
											return;
										case 8: // pulse count
											choose_prof_counter=9;
											segment_put("DET T ",0);
											return;
										case 9:
											choose_prof_counter=10;
											segment_put("DET A",0);											
											return;
										case 10:
											choose_prof_counter=0;
											segment_put("MODE ",0);											
											return;
									}
							}
							return;
						case ENTER_k:
							{
								if(choose_prof_counter ==1 && flag.BIT.first_one_cal_save ==0)
									{
										flag.BIT.first_one_cal_save =1;
										// Send the procces to main frame 
										flag.BIT.panel_ask_for_data =1;
										segment_clear();
										TIM6->CNT=0;
										send_send_counter =0;
										process.detail.total_time = (process.detail.timeA + process.detail.timeB + process.detail.timeC)*process.detail.total_time ;
										segment_put("SENDING PACKET",0);
										panel_send.detail.command=5;
										
										#if wait_time
										timer6_counter =0;
										HAL_TIM_Base_Start_IT(&htim6);
										#endif
										delayUS(2000);
										HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);
										return;
									}
								else if(choose_prof_counter !=1)
									{
										segment_clear();
										segment_put("SET ",0);
										last_position =5;
										flag.BIT.Enter_trigged=1;
										flag.BIT.DOT_EXIS =0;
										dot_counter =0;
										memset(SEG_ARRAY,0,7);
									}
							}
							return;
						case BACK_K:
							{
								SEG_menue_level=1;
								for(register uint8_t fast_cnt =0;fast_cnt<6;fast_cnt++)
									{
										panel_send.detail.availabele_board[fast_cnt] =0;
									}
								segment_clear();
								segment_put("CHOOSE MODULE" ,0);
							}
							return;
						default:
							{
								switch(choose_prof_counter)
									{
										case 0: //MODE
											{
											
											}
											return;
										case 1:	// SAVE
											{
											
											}
											return;
										case 2: // VA
											{
											
											}
											return;
										case 3:	// TIME A
											{
											
											}
											return;
										case 4: // VB
											{
											
											}
											return;
										case 5:	// TIME B
											{
											
											}
											return;
										case 6: // VC
											{
											
											}
											return;
										case 7: // TIMEC
											{
											
											}
											return;
										case 8: // pulse count
											{
											
											}
											return;
									}
							}
					}
			}
	}
void 	show_data(uint8_t key)
	{
		switch(key)
			{
				case UP_k:
					{
						if(exist_cell_num ==0)
							{
							
							}
						else 
							{
								exist_cell_num--;
								show_num(exist_cell_num);
							}
					}
					return;
				case DOWN_k:
					{
						if(exist_cell_num ==6)
							{
							
							}
						else 
							{
								exist_cell_num++;
								show_num(exist_cell_num);
							}
					}
					return;
				case BACK_K:
					{
						reset_state();
					}
					return;
			}
	}	
void choose_opperation(uint8_t key)
	{
		switch(key)
			{
//				case UP_k:
//					{
//						switch(choose_operation_counter)
//							{
//								case 0:
//										choose_operation_counter =2;
//										segment_put("PROF ",10);
//									return;
//								case 1:
//										choose_operation_counter =0;
//										segment_put("START",10);
//									return;
//								case 2:
//										choose_operation_counter =1;
//										segment_put("STOP ",10);
//									return;
//							}
//					}
//					return;
//				case DOWN_k:
//					{
//						switch(choose_operation_counter)
//							{
//								case 0:
//										choose_operation_counter =1;
//										segment_put("STOP ",10);
//									return;
//								case 1:
//										choose_operation_counter =2;
//										segment_put("PROF ",10);
//									return;
//								case 2:
//										choose_operation_counter =0;
//										segment_put("START",10);
//									return;
//							}
//					}
//					return;
//				case ENTER_k:
//					{
//						switch(choose_operation_counter)
//							{
//								case 0:		// START
//								{}
//									return;
//								case 1:		// STOP
//										flag.BIT.panel_ask_for_data =1;
//										segment_clear();
//										TIM6->CNT=0;
//										HAL_TIM_Base_Start_IT(&htim6);
//										segment_put("SENDING PACKET",0);		
//										panel_send.detail.command=2;
//										#if wait_time
//										timer6_counter =0;
//										HAL_TIM_Base_Start_IT(&htim6);
//										#endif		
//										HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);
//									return;
//								case 2:		// MODE
//										process.detail.Duty =0 ;
//										process.detail.Frequency=0;
//										process.detail.null_detect=0;
//										process.detail.pulse_count=0;
//										process.detail.range=0;
//										process.detail.test_mode=0;
//										process.detail.timeA=0;
//										process.detail.timeB=0;
//										process.detail.timeC=0;
//										process.detail.total_time=1;
//										process.detail.valueA=0;
//										process.detail.valueB=0;
//										process.detail.valueC=0;
//										flag.BIT.Enter_trigged =0;
//										segment_clear();
//										SEG_menue_level=3;
//										flag.BIT.DOT_EXIS =0;
//										dot_counter=0;
//										real_count=0;
//										process.detail.test_mode =0;
//										segment_put("MODE ",0);
//										choose_prof_counter = 0;
//									return;
//							}
//					}
//					return;
				case BACK_K:
					{
						SEG_menue_level=1;
						for(register uint8_t fast_cnt =0;fast_cnt<6;fast_cnt++)
							{
								panel_send.detail.availabele_board[fast_cnt] =0;
							}
						segment_clear();
						segment_put("CHOOSE MODULE" ,0);
					}
				case RUN_k:
					{
						if(choose_operation_counter ==0)
							{
									flag.BIT.panel_ask_for_data =1;
										segment_clear();
										TIM6->CNT=0;
										HAL_TIM_Base_Start_IT(&htim6);
										segment_put("SENDING PACKET",0);
										panel_send.detail.command=1;
										#if wait_time
											timer6_counter =0;
											HAL_TIM_Base_Start_IT(&htim6);
										#endif
										HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);							
							}
					}
			}
	}	
void home_menue(uint8_t key)
	{
		switch(key)
			{
			case UP_k:
				{
					if(flag.BIT.command_or_data) // last menue was data
						{
							flag.BIT.command_or_data =0;
							segment_clear();
							segment_put("COMMAND", 0);
						}
					else
						{
							flag.BIT.command_or_data =0;
							segment_clear();
							segment_put("COMMAND", 0);
						}
				}
				return;
			case DOWN_k:
				{
					if(flag.BIT.command_or_data) // last menue was data
						{
							
							flag.BIT.command_or_data =0;
							segment_clear();
							segment_put("COMMAND", 0);
							
						}
					else
						{
							flag.BIT.command_or_data =0;
							segment_clear();
							segment_put("COMMAND", 0);
						}
				}
				return;
			case ENTER_k:
				{
					if(flag.BIT.command_or_data==1) // last menue was data
						{
							SEG_menue_level =100;
							segment_clear();
							segment_put("PLEASE WAIT", 0);
							TIM6->CNT =0;
							exist_cell_num =0;
							#if wait_time
								timer6_counter =0;
								HAL_TIM_Base_Start_IT(&htim6);
							#endif
							flag.BIT.panel_first_ask_data =1;
							exist_cell_num =0;
							panel_send.detail.command = 4;
							HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);
							return;
							
						}
					else
						{
							TIM4->CNT=0;
							HAL_TIM_Base_Start_IT(&htim4);
							SEG_menue_level=1;
							exist_cell_num=0;
							for(register uint8_t fast_cnt =0;fast_cnt<6;fast_cnt++)
								{
									panel_send.detail.availabele_board[fast_cnt] =0;
								}
							delayUS(100);
							segment_clear();
							segment_put("CHOOSE MODULE" ,0);
							return;
						}
				}
			}
	}	
void choose_module(uint8_t key)
	{
		switch(key)
			{
				case CLEARE_k:
					{
						segment_clear();
						segment_put("RESET MODULES",0);
						for(register uint8_t fast_cnt =0;fast_cnt<6;fast_cnt++)
							{
								panel_send.detail.availabele_board[fast_cnt] =0;
							}
						delayUS(wait_menue_time);
						segment_clear();
						segment_put("CHOOSE MODULE" ,0);						
					}
					return;
				case BACK_K:
					{
						reset_state();
					}
					return;
				case ENTER_k:
					{
						choose_operation_counter =0; // for next step 
						for(uint8_t cnt=0;cnt<6;cnt++)
							{
								if(panel_send.detail.availabele_board[cnt] ==1)
									{
										break;
									}
								else if(cnt==5)
									{
										segment_clear();
										segment_put("SELECT FIRST",0);
										delayUS(wait_menue_time);
										segment_clear();
										segment_put("CHOOSE MODULE" ,0);
										return;
									}
							}
							// atleast one module has been added 
							//choose_operation_counter =0;
							
							SEG_menue_level =3;							
							process.detail.Duty =0 ;
							process.detail.Frequency=0;
							process.detail.null_detect=0;
							process.detail.pulse_count=0;
							process.detail.range=0;
							process.detail.test_mode=0;
							process.detail.timeA=0;
							process.detail.timeB=0;
							process.detail.timeC=0;
							process.detail.total_time=1;
							process.detail.valueA=0;
							process.detail.valueB=0;
							process.detail.valueC=0;
							flag.BIT.Enter_trigged =0;
							segment_clear();
							SEG_menue_level=3;
							flag.BIT.DOT_EXIS =0;
							dot_counter=0;
							real_count=0;
							process.detail.test_mode =0;
							segment_put("MODE ",0);
							choose_prof_counter = 0;
					}
					return;
				default:
					{
						if(key >0 & key < 7)
							{								
								panel_send.detail.availabele_board[key-1] = 1;
								segment_clear();
								char str[15];
								sprintf(str,"MODULE %d ADDED",key);
								segment_put(str,0);
								delayUS(wait_menue_time);
								segment_clear();
								segment_put("CHOOSE MODULE",0);
							}
					}
			}
	}	


uint8_t Read_key_scan1(void)
{
		const uint8_t key_matrix[32] = {1 ,2,3 	,BACK_K, \
																		4 ,5	,6,ENTER_k, \
																		7 ,8	,9 ,UP_k,\
																		CLEARE_k ,0	,DOT_k	,DOWN_k \
																		,RUN_k ,52,53	,54,\
																		HOME_K,56,57	,58,\
																		FUNC_k ,60,61	,62,\
																		STOP_k ,64,65	,66,};
		KEYPAD_SCAN_t key ;


		key.data_BIT.SW0 = HAL_GPIO_ReadPin(KEYPAD1_A_GPIO_Port,KEYPAD1_A_Pin);
		key.data_BIT.SW1 = HAL_GPIO_ReadPin(KEYPAD1_B_GPIO_Port,KEYPAD1_B_Pin);
		key.data_BIT.SW2 = HAL_GPIO_ReadPin(KEYPAD1_C_GPIO_Port,KEYPAD1_C_Pin);
		key.data_BIT.SW3 = HAL_GPIO_ReadPin(KEYPAD1_D_GPIO_Port,KEYPAD1_D_Pin);
		key.data_BIT.SW4 = HAL_GPIO_ReadPin(KEYPAD1_D_GPIO_Port,KEYPAD1_E_Pin);
		key.data_BIT.SW5 = 0;
		key.data_BIT.SW6 = 0;
		key.data_BIT.SW7 = 0;
	
return key_matrix[key.data] ; 
}																		

void uart_init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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

}




