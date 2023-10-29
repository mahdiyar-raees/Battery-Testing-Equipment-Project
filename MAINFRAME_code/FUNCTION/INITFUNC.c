

#include <INITFUNC.h>





	void 		panel_send			(uint8_t *send_array , uint8_t size);


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
	if(identifier <10)
			{
				if(HAL_CAN_AddTxMessage(&hcan1,&tx_CAN_Init, transmit_buffer , &TX_mailbox) != HAL_OK)
				{
				Error_Handler();
				}
//				if(HAL_CAN_AddTxMessage(&hcan2,&tx_CAN_Init, transmit_buffer , &TX_mailbox) != HAL_OK)
//				{
//				Error_Handler();
//				}
			}
		else if(identifier <23)
			{
			if(HAL_CAN_AddTxMessage(&hcan1,&tx_CAN_Init, transmit_buffer , &TX_mailbox) != HAL_OK)
				{
				Error_Handler();
				}
			}
		else if(identifier <30)
			{
			if(HAL_CAN_AddTxMessage(&hcan2,&tx_CAN_Init, transmit_buffer , &TX_mailbox) != HAL_OK)
				{
				Error_Handler();
				}
			}
	//send massage


	
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
		can1_filter_Init.FilterIdHigh= main_board_address<<5;
	
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
		
		
	if(HAL_CAN_ConfigFilter(&hcan1,&can1_filter_Init)!= HAL_OK)
			{
			Error_Handler();
			}		

//*************************//***************************************
//*************************//***************************************
//*************************//***************************************			
			
//		can1_filter_Init.FilterActivation= ENABLE;
//		
//		//which of 28 fillter we want to use
//		can1_filter_Init.FilterBank=0;
//		
//		//which of 2 FIFO we want to use 
//		can1_filter_Init.FilterFIFOAssignment= CAN_RX_FIFO0;
//		
//		
//		// 32 bit identifier high xxxx****
//		can1_filter_Init.FilterIdHigh= main_board_address<<5;
//	
//		// 32 bit identifier low ****xxxx	
//		can1_filter_Init.FilterIdLow= 0x0000;
//		
//		
//		// 32 bit mask high xxxx****
//		can1_filter_Init.FilterMaskIdHigh=0x0000;
//		
//		// 32 bit mask low ****xxxx	
//		can1_filter_Init.FilterMaskIdLow=0x0000;
//		
//		// chose filter type it can be
//		//  CAN_FILTERMODE_IDLIST 	for ID MODE
//		//  CAN_FILTERMODE_IDMASK 	for MASK MODE
//		can1_filter_Init.FilterMode= CAN_FILTERMODE_IDLIST;
//	
//		
//		can1_filter_Init.FilterScale= CAN_FILTERSCALE_32BIT;
//		
//		
//	if(HAL_CAN_ConfigFilter(&hcan2,&can1_filter_Init)!= HAL_OK)
//			{
//			Error_Handler();
//			}

					
		
		
		can1_filter_Init.FilterActivation= ENABLE;
		
		//which of 28 fillter we want to use
		can1_filter_Init.FilterBank=0;
		
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
		
		
	if(HAL_CAN_ConfigFilter(&hcan1,&can1_filter_Init)!= HAL_OK)
			{
			Error_Handler();
			}		

//*************************//***************************************
//*************************//***************************************
//*************************//***************************************			
//			
//		can1_filter_Init.FilterActivation= ENABLE;
//		
//		//which of 28 fillter we want to use
//		can1_filter_Init.FilterBank=0;
//		
//		//which of 2 FIFO we want to use 
//		can1_filter_Init.FilterFIFOAssignment= CAN_RX_FIFO1;
//		
//		
//		// 32 bit identifier high xxxx****
//		can1_filter_Init.FilterIdHigh= main_board_ERROR_address<<5;
//	
//		// 32 bit identifier low ****xxxx	
//		can1_filter_Init.FilterIdLow= 0x0000;
//		
//		
//		// 32 bit mask high xxxx****
//		can1_filter_Init.FilterMaskIdHigh=0x0000;
//		
//		// 32 bit mask low ****xxxx	
//		can1_filter_Init.FilterMaskIdLow=0x0000;
//		
//		// chose filter type it can be
//		//  CAN_FILTERMODE_IDLIST 	for ID MODE
//		//  CAN_FILTERMODE_IDMASK 	for MASK MODE
//		can1_filter_Init.FilterMode= CAN_FILTERMODE_IDLIST;
//	
//		
//		can1_filter_Init.FilterScale= CAN_FILTERSCALE_32BIT;
//		
//		
//	if(HAL_CAN_ConfigFilter(&hcan2,&can1_filter_Init)!= HAL_OK)
//			{
//			Error_Handler();
//			}


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
							HAL_UART_Transmit(&RS_485_PORT, start_buffer,buffer_size,100);
							HAL_UART_Transmit(&RS_485_PORT, send_array,size, 100);
							HAL_UART_Transmit(&RS_485_PORT, stop_buffer,buffer_size,100);
					break;
			
			}		
		}
	
	void 		panel_send			(uint8_t *send_array , uint8_t size)
{


		HAL_UART_Transmit(&PANEL_PORT, send_array,size, 100);

			
	
}
