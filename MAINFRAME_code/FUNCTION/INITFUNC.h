#ifndef INITFUNC
	#define INITFUNC
	#include "stdio.h"
	#include "string.h"
	#include "stm32f4xx_hal.h"
	#include <defines.h>
	#include <TYPEDEF.H>

	#define RS_485_PORT     huart1
	#define USB_port				huart2
	#define PANEL_PORT			huart4	



	extern 		port_selector_t					port;
	extern	 	UART_HandleTypeDef PANEL_PORT;
	extern	 	UART_HandleTypeDef RS_485_PORT;
	extern 		UART_HandleTypeDef USB_port;


	extern 		CAN_HandleTypeDef hcan1;
	extern 		CAN_HandleTypeDef hcan2;

	
	

	

	
#endif	
