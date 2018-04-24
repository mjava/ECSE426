/**
  ******************************************************************************
  * File Name          : supporting_functions.c
  * Description        : User defined common functions that can be used across projects
	* Author						 : Ashraf Suyyagh
	* Version            : 1.0.0
	* Date							 : January 14th, 2016
  ******************************************************************************
  */
#include "supporting_functions.h"

/**
   * @brief A function used to relay information (PRINTF) that a specific error has occured 
	 * and gets the system into infinite loop. User can modify the output error message
   * @retval None
   */
	 
	 //AT TAs & STUDENTS
	 //Feel free to use or discard this function, you can add more Error definitions and assign 
	 //them codes in the supporting_functions.h file
	 
	 //Many functions in the drivers return a status code, usually HAL_OK or HAL_ERROR. You can check
	 //the output of your driver function call and either proceed or call this function to print
	 //whatever error message you want. Helps you in debugging your code at many points during developing
	 //your codes
	 
	 
void Error_Handler			(uint16_t error_code){
	//User error handling code, could use printf to relay information to user
	switch (error_code){	
		case RCC_CONFIG_FAIL: 
			printf("ERROR: Error in initialising System Clocks \n");
			break;
		case TIM_INIT_FAIL: 
			printf("ERROR: Error in initialising Timer base \n");
			break;
		case ADC_INIT_FAIL: 
			printf("ERROR: Error in initialising ADC \n");
			break;
		case ADC_CH_CONFIG_FAIL: 
			printf("ERROR: Error in initialising ADC Channel Configuration \n");
			break;
		case ADC_MULTIMODE_FAIL: 
			printf("ERROR: Error in initialising ADC Multimode\\DMA \n");
			break;
		case EXTI_SPI1_FAIL: 
			printf("ERROR: Error in initialising SPI1 \n");
			break;
		default: printf ("ERROR: AN ERROR OCCURED \n");
		break;
	}
}
