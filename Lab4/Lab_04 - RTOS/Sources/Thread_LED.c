/*******************************************************************************
  * @file    Thread_LED.c
  * @author  Amirhossein Shahshahani and Ashraf Suyyagh
	* @version V1.0.0
  * @date    17-January-2016
  * @brief   This file initializes one LED as an output, implements the LED thread 
  *					 which toggles and LED, and function which creates and starts the thread	
  ******************************************************************************
  */
	
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "stm32f4xx_hal.h"

void Thread_LED (void const *argument);                 // thread function
void Thread_LED_2 (void const *argument);                 // thread function
void Thread_LED_3 (void const *argument);                 // thread function
osThreadId tid_Thread_LED;             
osThreadId tid_Thread_LED;                              // thread id
osThreadId tid_Thread_LED_2;                              // thread id
osThreadId tid_Thread_LED_3;  // thread id
osThreadDef(Thread_LED, osPriorityNormal, 1, 0);
osThreadDef(Thread_LED_2, osPriorityNormal, 1, 0);
osThreadDef(Thread_LED_3, osPriorityNormal, 1, 0);

/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/
int start_Thread_LED (void) {

  tid_Thread_LED = osThreadCreate(osThread(Thread_LED ), NULL); // Start LED_Thread
	tid_Thread_LED_2 = osThreadCreate(osThread(Thread_LED_2 ), NULL); // Start LED_Thread
	tid_Thread_LED_3 = osThreadCreate(osThread(Thread_LED_3 ), NULL); // Start LED_Thread
  if (!tid_Thread_LED) return(-1); 
  return(0);
}

 /*----------------------------------------------------------------------------
*      Thread  'LED_Thread': Toggles LED
 *---------------------------------------------------------------------------*/
	void Thread_LED (void const *argument) {
		while(1){
				osDelay(1000);
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			}
	}
	
	
	void Thread_LED_2 (void const *argument) {
		while(1){
				osDelay(230);
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			}
	}	
	
		void Thread_LED_3 (void const *argument) {
		while(1){
				osDelay(230);
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			}
	}
/*----------------------------------------------------------------------------
 *      
 *---------------------------------------------------------------------------*/