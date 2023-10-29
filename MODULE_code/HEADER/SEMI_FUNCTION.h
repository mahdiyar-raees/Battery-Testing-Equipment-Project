#ifndef SEMI_FUNCTION
#define SEMI_FUNCTION


#define 		CONTACTOR_ON		HAL_GPIO_WritePin(FINDER_CONTACTOR_GPIO_Port,FINDER_CONTACTOR_Pin,GPIO_PIN_SET)
#define 		CONTACTOR_OFF		HAL_GPIO_WritePin(FINDER_CONTACTOR_GPIO_Port,FINDER_CONTACTOR_Pin,GPIO_PIN_RESET)

#define 		FAN_ON					HAL_GPIO_WritePin(Finder_FAN_GPIO_Port,Finder_FAN_Pin,GPIO_PIN_SET)
#define 		FAN_OFF					HAL_GPIO_WritePin(Finder_FAN_GPIO_Port,Finder_FAN_Pin,GPIO_PIN_RESET)

#define 		SPI1_OFF				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET)
#define 		SPI1_ON					HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET)

#define 		SPI2_OFF				GPIOB->ODR &= 0xBFFF
#define 		SPI2_ON					GPIOB->ODR |= 0x4000

#define 		SPI3_OFF				GPIOD->ODR &= 0xFFFB
#define 		SPI3_ON					GPIOD->ODR |= 0x0004


		




#endif
