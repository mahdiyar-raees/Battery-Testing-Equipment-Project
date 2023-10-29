
#ifndef SEMIFUNCTION
	#define SEMIFUNCTION
	#define LED_on_ON		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);
	#define LED_emerg_ON		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET);
	#define LED_F1_ON		HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_SET);
	#define LED_E1_ON		HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin, GPIO_PIN_SET);
	#define LED_OPP_ON		HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin, GPIO_PIN_SET);
	#define LED_OVP_ON		HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin, GPIO_PIN_SET);
	#define LED_OCP_ON		HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin, GPIO_PIN_SET);
	#define LED_range_ON		HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin, GPIO_PIN_SET);
	#define LED_CR_ON		HAL_GPIO_WritePin(LED9_GPIO_Port,LED9_Pin, GPIO_PIN_SET);
	#define LED_CC_ON		HAL_GPIO_WritePin(LED10_GPIO_Port,LED10_Pin, GPIO_PIN_SET);
	
	
	#define LED_on_OFF		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
	#define LED_emerg_OFF		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_RESET);
	#define LED_F1_OFF		HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_RESET);
	#define LED_E1_OFF		HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin, GPIO_PIN_RESET);
	#define LED_OPP_OFF		HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin, GPIO_PIN_RESET);
	#define LED_OVP_OFF		HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin, GPIO_PIN_RESET);
	#define LED_OCP_OFF		HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin, GPIO_PIN_RESET);
	#define LED_range_OFF		HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin, GPIO_PIN_RESET);
	#define LED_CR_OFF		HAL_GPIO_WritePin(LED9_GPIO_Port,LED9_Pin, GPIO_PIN_RESET);
	#define LED_CC_OFF		HAL_GPIO_WritePin(LED10_GPIO_Port,LED10_Pin, GPIO_PIN_RESET);
	
#endif
