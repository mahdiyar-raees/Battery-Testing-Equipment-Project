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
	* 													Designer = Mahdiyar raees almohaddesin
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <FUNCTION.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t								real_count; 
uint8_t 							last_ascii;
uint8_t 							SEG_menue_level;
process_excute_t			process;
FLAG_t								flag;
uint8_t								panel_command_counter;
panel_get_buff_t			panel_get;		
uint8_t 							key=0;
uint8_t 							menu_count=0;
uint8_t								exist_cell_num=0;
uint8_t								tx_buffer[12];
FLAG_t								flag;
float    						  real_num[13];
panel_send_buff_t 		panel_send;
uint8_t								complete_get_coff_count=0;
uint8_t								cof_4byte_count=0;
float									calibration_coeffitent[26]={1,0,\
																												1,0,\
																												1,0,\
																												1,0,\
																												1,0,\
																												1,0,\
																												1,0,\
																												1,0,\
																												1,0,\
																												1,0,\
																												1,0,\
																												1,0,\
																												1,0};

uint16_t 							timer6_wait=8;																											
uint16_t 							timer4_counter=0;
uint16_t 							timer6_counter=0;
uint16_t 							timer5_counter=0;
uint8_t								choose_operation_counter=0;
uint8_t		 						reset_counter=0;
uint8_t    						anghezi[110] ;
uint8_t		 						anghezi_test=0;
uint8_t								send_send_counter=0;																						
uint8_t								flag_trigger_key;
uint8_t								key_trigger;		
uint8_t								re_try_counter =0;																												
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void key_manager(void)
	{
				key = key_trigger;
			flag.BIT.key_bunce =0;
			if(key == HOME_K)
				{
					reset_state();
					delayUS(100);
					flag.BIT.key_bunce =1;
					return;
				}
			else if(key == STOP_k)
				{	
					panel_send.detail.command = 3;
					// turn on timmer for respond ****
					#if wait_time
						timer6_counter =0;
						HAL_TIM_Base_Start_IT(&htim6);
					#endif
					delayUS(100);
					HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);
					delayUS(900);
					HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);
					delayUS(900);
					HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);
					delayUS(900);
					
					flag.BIT.key_bunce =1;
					return;
				}
			else
				{
					TIM4->CNT =0;
					SEGMENT_MASTER(key);
					delayUS(500);
					flag.BIT.key_bunce =1;
				}
	}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t		temp;
	temp      =  Read_key_scan1();
	if(flag.BIT.initial_coff_ask == 0)
	{
	if(flag.BIT.key_bunce)
		{
			flag_trigger_key=1;
			key_trigger = temp;			
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		if(htim->Instance == TIM4) // wait time for complete the command 
			{
				timer4_counter++;
				if(timer4_counter >timer6_wait)
				{
					if(flag.BIT.initial_coff_ask)
						{
							segment_put("RESET SYSTEM",0);
							return;
						}
					timer4_counter =0;
					reset_state();
				}
			}
		else if(htim->Instance == TIM6) // conaction fail
			{
				timer6_counter++;
				if(timer6_counter > timer6_wait) 
					{
						if(flag.BIT.initial_coff_ask == 1)
							{
								re_try_counter++;
								if(re_try_counter > 2)
								{
								__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
								
								timer6_counter =0;
								__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
								HAL_TIM_Base_Stop_IT(&htim6);	
								segment_clear();
								segment_put("PERMANENT FAIL",0);	
								}
								else
								{
									timer6_counter =0;
									panel_send.detail.command=6;
									HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);	
								}
								}
						else
							{
								timer6_counter =0;
								__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
								HAL_TIM_Base_Stop_IT(&htim6);	
								segment_clear();
								segment_put("CONNECT FAIL",0);
								delayUS(2000);
								reset_state();
							}
						}
			}
		else if(htim->Instance == TIM5) // 1S ask data from main frame 
			{
				
				show_num(exist_cell_num);
				panel_send.detail.command=4;
				#if wait_time // diffrent style must be added
						timer6_counter =0;
						HAL_TIM_Base_Start_IT(&htim6);
				#endif
				
				HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);
			}
			//timer2 make 1 us 
	}		
 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
uint8_t buffer1[8]= { 1,2,3,4,5,6,7,8};
uint8_t buffer2[8]= { 9,10,11,12,13,14,15,16};
uint8_t buffer3[8] = {17,18,19,20,21,22,23,24};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
panel_send.detail.start_buff_1 = panel_start_1;
panel_send.detail.start_buff_2 = panel_start_2;
panel_send.detail.stop_buff_1 = panel_end_1;
panel_send.detail.stop_buff_2 = panel_end_2;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	LED_on_ON;
__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
flag.BIT.initial_coff_ask = 1;
timer6_wait =20;
timer6_counter = 0; 
//HAL_TIM_Base_Start_IT(&htim6); 
	//load_parameter();
HAL_TIM_Base_Start(&htim2);
segment_clear();
		segment_put("WELCOME",4);		
		HAL_Delay(3000);
	


segment_clear();
segment_put("SARI 3KW TESTER",0);
complete_get_coff_count=0;
cof_4byte_count =0;

HAL_Delay(3000);
segment_clear();

segment_put("INITIALIZATION",0);
flag.BIT.panel_ask_for_data =1;
timer6_counter=0;
TIM6->CNT=0;
HAL_TIM_Base_Start_IT(&htim6);

#if wait_time
timer6_counter =0;
HAL_TIM_Base_Start_IT(&htim6);
#endif
panel_send.detail.command=6;
HAL_UART_Transmit(&huart1,panel_send.send_buff,20,100);	

//reset_state();

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		reset_counter = 10;	
		if(flag_trigger_key)
			{
				flag_trigger_key = 0; 
				key_manager();
			}	
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

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
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  htim4.Init.Prescaler = 57600;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
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
  htim5.Init.Prescaler = 28800;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000;
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
  htim6.Init.Prescaler = 7200;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 30000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEG14_Pin|SEG15_Pin|SEG1_Pin|SEG2_Pin
                          |SEG3_Pin|SEG4_Pin|SEG5_Pin|SEG6_Pin
                          |SEG7_Pin|SEG8_Pin|SEG9_Pin|SEG10_Pin
                          |SEG11_Pin|SEG12_Pin|SEG13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin|LED7_Pin|LED8_Pin
                          |LED9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED10_Pin|SEG_ADDRESS_2_Pin|SEG_ADDRESS_3_Pin|SEG_ADDRESS_4_Pin
                          |SEG_ADDRESS_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEG14_Pin SEG15_Pin SEG1_Pin SEG2_Pin
                           SEG3_Pin SEG4_Pin SEG5_Pin SEG6_Pin
                           SEG7_Pin SEG8_Pin SEG9_Pin SEG10_Pin
                           SEG11_Pin SEG12_Pin SEG13_Pin */
  GPIO_InitStruct.Pin = SEG14_Pin|SEG15_Pin|SEG1_Pin|SEG2_Pin
                          |SEG3_Pin|SEG4_Pin|SEG5_Pin|SEG6_Pin
                          |SEG7_Pin|SEG8_Pin|SEG9_Pin|SEG10_Pin
                          |SEG11_Pin|SEG12_Pin|SEG13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                           LED5_Pin LED6_Pin LED7_Pin LED8_Pin
                           LED9_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin|LED7_Pin|LED8_Pin
                          |LED9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED10_Pin SEG_ADDRESS_2_Pin SEG_ADDRESS_3_Pin SEG_ADDRESS_4_Pin
                           SEG_ADDRESS_1_Pin */
  GPIO_InitStruct.Pin = LED10_Pin|SEG_ADDRESS_2_Pin|SEG_ADDRESS_3_Pin|SEG_ADDRESS_4_Pin
                          |SEG_ADDRESS_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_EXT_Pin */
  GPIO_InitStruct.Pin = KEY_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_EXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD1_A_Pin KEYPAD1_B_Pin KEYPAD1_C_Pin KEYPAD1_D_Pin
                           KEYPAD1_E_Pin */
  GPIO_InitStruct.Pin = KEYPAD1_A_Pin|KEYPAD1_B_Pin|KEYPAD1_C_Pin|KEYPAD1_D_Pin
                          |KEYPAD1_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
