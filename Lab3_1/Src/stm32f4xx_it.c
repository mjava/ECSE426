/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
void ledDriver(int ledPos, int number, int modeFlag);
int getKeypadValue(void);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

extern int displaySwitchTrigger; //triggers the switch between LEDs
extern int ledPosition;
extern int modeFlag;
extern int outputDigits[4];
extern int keyValid;
extern int sleepKey;
extern int keyReadTrigger;
extern int restartKey;
extern int holdCount;
extern int sleepCount;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
	displaySwitchTrigger++;
  /* USER CODE BEGIN SysTick_IRQn 1 */
	if(displaySwitchTrigger >= LED_POSITION_TIMER){
			displaySwitchTrigger = 0;
			ledPosition++;
			/*reset to 0 when ledPosition reaches 4*/
			ledPosition = ledPosition % 4;
			/*at ledPosition x, display outputDigit[x]*/
			ledDriver(ledPosition, outputDigits[ledPosition], modeFlag);
	}
	/*start timer when button pressed*/
	if(getKeypadValue()>-1 && getKeypadValue() < 12) {
		keyReadTrigger++;
		/*regular button press*/
		if(keyReadTrigger >= KEY_TIMER) { 
			keyValid = 1;
			/*if star or pound pressed, start the reset and sleep timers*/
			if(getKeypadValue() == 10 || getKeypadValue() == 11) {
				holdCount++;
				sleepCount++;
				/*if held for more than 4 seconds, turn on sleep mode*/
				if(sleepCount >= KEY_TIMER*3){ 
					sleepKey = 1;
					sleepCount = 0;
				} 
				/*if held for more 1-2 seconds, turn on reset mode*/
				if(holdCount >= KEY_TIMER*2) {
					restartKey = 1;
					holdCount = 0;
				}				
			}
			keyReadTrigger = 0;
		} else {
				keyValid = 0;
		}
	}
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
