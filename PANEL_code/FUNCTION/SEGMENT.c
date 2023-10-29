#include <SEGMENT.h>






void put_on_segment(uint16_t value , uint8_t position);
uint16_t ascii_to_hex(uint8_t code);
/*
0 4B66
1 1900
2 4C61
3 4D21
4	0D03
5	4223
6	4563
7	4900	
8	4D63
9	4D23
A	4D43
B 6D28
C 4062
D	6928
E	4463
F	4043
G 4562
H	0D43
I	2008
J	0960
K	1243
L	0062
M	1946
N	0B46
O	4962
P	4C43
Q	4B62
R	4E43
S	4523
T	6008
U	0962
V 10C2
W 0BC2
X 1284
Y 100C
Z 50A0






*/
void delayUS(uint32_t us)
	{   // Sets the delay in microseconds.
		uint32_t ms_value = us;
		while (ms_value>0)
			{
				__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
				while (__HAL_TIM_GET_COUNTER(&htim2) < 1000);
				ms_value--;
			}
	}

uint16_t ascii_to_hex(uint8_t code)
{
	uint8_t real =0;
	
	const uint16_t segment_CODE[43] = {0x4B66,0x1900,0x4C61,0x4D21,0x0D03,0x4223,0x4563,0x4900	,0x4D63,0x4D23,\
		0,0,0,0,0,0,0,\
		0x4D43,0x6D28,0x4062,0x6928 \
		,0x4463,0x4043,0x4562,0x0D43 ,0x2008 \
		,0x0960,0x1243,0x0062,0x1946,0x0B46,0x4962,0x4C43,0x4B62,0x4E43,0x4523,0x6008,0x0962,0x10C2,0x0BC2,0x1284,0x100C,0x50A0};	
	if(code ==32)
		{
			real=11;
		}
	else
		{
			real =code -48;
		}
	// i think we have to modifi for blank space
	return segment_CODE[real];
	

}

void put_on_segment(uint16_t value , uint8_t position)
{
	HAL_GPIO_WritePin(GPIOC , 0xFFFF, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC , value, GPIO_PIN_SET);
	delayUS(1);	
	position =~position;
	HAL_GPIO_WritePin(GPIOB , 0x1E00, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,(position<<9),GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOB , 0x1E00, GPIO_PIN_SET);
	delayUS(1);
}

void segment_put_decmial(uint8_t position)// we should do it with last num position
{
	uint16_t			temp_seg_value =0;
	temp_seg_value  =	ascii_to_hex(last_ascii);
	temp_seg_value = temp_seg_value | 0x10;
	put_on_segment( temp_seg_value , position);
}

void segment_put(char *str , uint8_t position)
{
	register	uint8_t cnt=0;
		while(1)
		{
			if(str[cnt] == '\0')
				{
					//finish
					break;
				}
			else
				{
					uint16_t SEG_value;
					last_ascii = str[cnt];
					SEG_value 		  =	ascii_to_hex(str[cnt]);
					
					
					put_on_segment(SEG_value, position);
					cnt++;
					switch(position)
						{
							case 11:
								position =13;
							break;
							case 13:	
								position =12;
							break;
							case 12:
								position = 14;
							break;
							default:
								position++;
							break;
						}
				}
		}
	}
uint8_t next_position(uint8_t position)
	{
		uint8_t X;
		X= position;
		switch(X)
			{
				case 11:
					X =13;
				break;
				case 13:	
					X =12;
				break;
				case 12:
					X = 14;
				break;
				default:
					X++;
				break;
			}
		return X;
	}
	
void segment_clear(void)
{		
	HAL_GPIO_WritePin(SEGMENT_PORT , 0xFFFF, GPIO_PIN_RESET);		
	for(register uint8_t cnt=0; cnt<=SEGMENT_COUNT ;  cnt++)
		{
			delayUS(1);	
			HAL_GPIO_WritePin(DECODER_PORT , 0x1E00, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DECODER_PORT,cnt<<9,GPIO_PIN_SET );	
			delayUS(1);
		}
}







