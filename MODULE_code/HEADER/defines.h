#ifndef	DEFINES
#define DEFINES


		//**********************USER DEFINE ************************//

		#define alpha_calibrition_coeffitiont_CC										1 // alpha *x + bete 
	//	#define beta_calibrition_coeffitiont_CC											2.1284f		// alpha *x + bete 
		
		#define MODE_BUFFER_SIZE																		5
		#define ADC_BUFFER_SIZE																			40
		#define BATTERY_ADC_SIZE																		40
		#define DAC_CHANGE_STEP_COUNT																1// min =1;
				
		
		#define MEAN_BUFFER_FOR_SEND															10
				
				
		// FULL SCALE foumula = 70 amp / 65536		
		//#define 				FULL_SCALE_GAIN								506.31f
		
		
		#define OPP_PROTECTION																			500.0F	
		#define OPP_WAIT_TIME																				100U // uint 1ms

		#define OVP_PROTECTION																			150.0F	
		#define OVP_WAIT_TIME																				50U // uint 1ms
		
		#define OCP_PROTECTION																			63.0F	
		#define OCP_WAIT_TIME																				50U // uint 1ms
		
		#define 				BOARD_NUMBER																6
		#define 				CR_PAR_COUNTER 															100
		
		//*************ADMIN DEFINE*****************//
		#define 				ON												1
		#define 				OFF												0
		#define 				commen_address																			1
		
		#define 				MAIN_BOARD_ADDRESS 				4
		#define 				MAIN_FAULT_ADDRESS				3
		
		
		#define 				get_data_first_buffer			10			// this part must be modify by ADMIN 	
		#define 				process_count       			254		// this part must be modify by ADMIN 
		#define 				NOT_FOUND_CHARGER 				0xFF
		#define 				NOT_VALID_COMMAND 				0xFE
		#define 				DAC_LIMITATION						(((FULL_SCALE_GAIN * 63)) +beta_calibrition_coeffitiont_CC)
		
		//ERROR_CODE
		#define 				OVP												1U
		#define 				OCP												2U
		#define 				OPP												3U
		#define 				reverse_voltage						4
		#define 				gate_over_voltage					5
		#define 				FAN_TURN_OFF							6
		#define 				contactor_fail						7
		#define 				over_voltage							8
		#define 				POWER_TURN_OFF						9
		#define					NOT_SYCRONIZE_SIGNAL			10
		#define					NOT_RIGHT_TIMER_SETTING   11
		#define 				NO_POWER									12
		
		#define 				timer_state								ON
		#define 				MAX_DAC_OUTPUT						ON

				#if 						BOARD_NUMBER ==1 // BOARD 1
		#define 				MODULE_BOARD_ADDRESS      													20
		#define					delay_sample_rate 							1 
		#define 				w_20us_step											1
		#define 				FULL_SCALE_GAIN								517.06f
		#define 				beta_calibrition_coeffitiont_CC			-2.2717f						// alpha *x + bete 
		#define					get_data_first_buff				2			// this part shoult only be 2 6 10 14 16 18 
		#endif
		#if 						BOARD_NUMBER ==2 // BOARD 2
		#define 				MODULE_BOARD_ADDRESS      													21
		#define					delay_sample_rate 							10000//7500
		#define 				w_20us_step											5
		#define 				FULL_SCALE_GAIN								518.044f
		#define				  beta_calibrition_coeffitiont_CC			22.063f									// alpha *x + bete 
		#define					get_data_first_buff				6			// this part shoult only be 2 6 10 14 16 18 
		#endif
		#if 						BOARD_NUMBER ==3 // BOARD 3
		#define 				MODULE_BOARD_ADDRESS      													22
		#define					delay_sample_rate 						17500//	15000 
		#define 				w_20us_step											7
		#define 				FULL_SCALE_GAIN								543.9f
		#define 				beta_calibrition_coeffitiont_CC		10.512f									// alpha *x + bete 
		#define					get_data_first_buff				10			// this part shoult only be 2 6 10 14 16 18 
		#endif
		#if 						BOARD_NUMBER ==4 // BOARD 4
		#define 				MODULE_BOARD_ADDRESS      													23
		#define 				FULL_SCALE_GAIN							506.31f
		#define					delay_sample_rate 					5000//	2500
		#define 				w_20us_step											3
		#define 				beta_calibrition_coeffitiont_CC		2.1284f							// alpha *x + bete 
		#define					get_data_first_buff				14			// this part shoult only be 2 6 10 14 16 18 
		#endif
		#if 						BOARD_NUMBER ==5 // BOARD 5
		#define 				MODULE_BOARD_ADDRESS      													24
		#define 				FULL_SCALE_GAIN							525.71f
		#define					delay_sample_rate 					12500//	10000
		#define 				w_20us_step											10
		#define				  beta_calibrition_coeffitiont_CC		-1.4818f							// alpha *x + bete 
		#define					get_data_first_buff				18			// this part shoult only be 2 6 10 14 16 22 
		#endif
		#if 						BOARD_NUMBER ==6 // BOARD 6
		#define 				MODULE_BOARD_ADDRESS      													25
		#define 				FULL_SCALE_GAIN							541.211f
		#define					delay_sample_rate 					20000//	17500
		#define 				w_20us_step											12
		#define				  beta_calibrition_coeffitiont_CC		9.0f								// alpha *x + bete 
		#define					get_data_first_buff				22			// this part shoult only be 2 6 10 14 18 22 
		#endif
#endif
