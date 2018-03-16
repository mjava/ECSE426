/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim3;

osThreadId generalTimerThread;
osThreadId keypadTimerThread;
osThreadId stateThread;
osThreadId displayThread;
osThreadId keypadButtonThread;

osSemaphoreId myBinarySem01Handle;
osSemaphoreId keyboardButtonSemaphore;
osSemaphoreId sleepSemaphore;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t adcConversion = 0;
uint32_t buttonSignal = 0;
uint32_t displaySignal = 0;
uint32_t sleepSignal = 0;
int buttonPressed;
int buttonPressedOld;
int key;
int keypadMatrix[3][4] = {{1,4,7,10}, {2,5,8,0},{3,6,9,11}}; //10 = star, 11 = pound
int state; //state for keypad
float mathInput;
float mathOutput;
int x[5] = {0};
float rms_value = 0;
float filterOutput;
int keyReadTrigger; //timer for debouncing for keypad
int displaySwitchTrigger; //triggers the switch between LEDs
int timer;
int counter;
int last_counter;
int ledPosition;
int outputDigits[4];
int keyValid = 0; //flag for normal key
int sleepKey = 0; //flag for putting keypad to sleep or waking up
int restartKey = 0; //flag for restarting keypad
int holdCount = 0; //timer for restarting keypad
int sleepCount = 0; //timer for putting keypad to sleep or waking up
float controllerTarget;
float error;
int pulseWidth;
int valueReturned;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

void generalTimer(void const * argument);
void keypadTimer(void const * argument);
void stateController(void const * argument);
void getKeypadValue(void const *argument);
void ledDriver(void const *argument);

osThreadDef(timerThread, generalTimer, osPriorityHigh, 0, 128); //timer priority MUST be high
osThreadDef(keyTimerThread, keypadTimer, osPriorityHigh, 0, 128); //timer priority MUST be high
osThreadDef(fsmThread, stateController, osPriorityNormal, 0, 256);
osThreadDef(buttonThread, getKeypadValue, osPriorityNormal, 0, 256);
osThreadDef(ledThread, ledDriver, osPriorityNormal, 0, 128);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void pwmSetValue(uint16_t pulseValue);
void valueParse(float mathOutput);
void c_math(float input);
void FIR_C(uint32_t input, float* filterOutput);
void pwmController(float input, float output);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	
	/* Start timer for PWM generation */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	/* Start ADC interrupt triggered by timer */
	HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */
	state = 1;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  osSemaphoreDef(myBinarySem01);
	myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);
	
	osSemaphoreDef(keyboardButtonSem);
	keyboardButtonSemaphore = osSemaphoreCreate(osSemaphore(keyboardButtonSem), 1);
	
	osSemaphoreDef(sleepSem);
	sleepSemaphore = osSemaphoreCreate(osSemaphore(sleepSem), 1);

	/* Create the thread(s) */
	generalTimerThread = osThreadCreate(osThread(timerThread), NULL);
	
	keypadTimerThread = osThreadCreate(osThread(keyTimerThread), NULL);

	stateThread = osThreadCreate(osThread(fsmThread), NULL);
	
	displayThread = osThreadCreate(osThread(ledThread), NULL);
	
	keypadButtonThread = osThreadCreate(osThread(buttonThread), NULL);

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and valueReturned of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin Cathode_1_Pin Cathode_2_Pin Cathode_3_Pin 
                           Cathode_4_Pin Segment_A_Pin Segment_B_Pin Segment_C_Pin 
                           Segment_D_Pin Segment_E_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Segment_F_Pin Segment_G_Pin Row_Keypad_1_Pin Row_Keypad_2_Pin 
                           Row_Keypad_3_Pin Row_Keypad_4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Column_Keypad_1_Pin Column_Keypad_2_Pin Column_Keypad_3_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void generalTimer(void const * argument) {
	while(1) {
		osDelay(1);
		displaySwitchTrigger++;
		if(displaySwitchTrigger >= LED_POSITION_TIMER) {
			displaySwitchTrigger = 0;
			ledPosition++;
			/*reset to 0 when ledPosition reaches 4*/
			ledPosition = ledPosition % 4;
			osSignalSet(displayThread,displaySignal);
		}
	}
}

void keypadTimer(void const * argument) {
	while(1) {
		//debouncing
		osDelay(KEY_TIMER);
		if(valueReturned > -1 && valueReturned < 12) {
			keyValid = 1;
			if(valueReturned == 10 || valueReturned == 11) {
				sleepCount++;
				holdCount++;
				/*if held for more 1-2 seconds, turn on reset mode*/
				if(holdCount >= KEY_TIMER*1.5) {
					restartKey = 1;
					holdCount = 0;
				}	
				/*if held for more than 4 seconds, turn on sleep mode or wakeup from sleep*/
				if(sleepCount >= KEY_TIMER*2) {
					sleepKey = 1;
					sleepCount = 0;
					/*if in sleep mode -> waking up, turn on threads*/
					if(state == 0) {
						/* Start timer for PWM generation */
						HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
						/* Start ADC interrupt triggered by timer */
						HAL_ADC_Start_IT(&hadc1);
						
						generalTimerThread = osThreadCreate(osThread(timerThread), NULL);
						stateThread = osThreadCreate(osThread(fsmThread), NULL);
						displayThread = osThreadCreate(osThread(ledThread), NULL);
						sleepKey = 0;
						state = 1;
					}
				}
			}
		} 
		else if(valueReturned == -1) {
			holdCount = 0;
			sleepCount = 0;
		}
	}
}

void ledDriver(void const * argument){
	int number = -1;
	while(1) {
		osSignalWait(displaySignal,osWaitForever);
		displaySignal = 0;
		number = outputDigits[ledPosition];
		if(state > 0) {
			if(number == 0){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_RESET);
			}
			else if(number == 1){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_RESET);
			}
			else if(number == 2){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_SET);
			}
			else if(number == 3){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_SET);
			}
			else if(number == 4){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_SET);
			}
			else if(number == 5){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_SET);
			}
			else if(number == 6){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_SET);
			}
			else if(number == 7){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_RESET);
			}
			else if(number == 8){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_SET);
			}
			else if(number == 9){
					HAL_GPIO_WritePin(SEGMENT_A, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_B, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_C, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_D, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_E, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(SEGMENT_F, GPIO_PIN_SET);
					HAL_GPIO_WritePin(SEGMENT_G, GPIO_PIN_SET);
			}
			if(ledPosition == 0){
				HAL_GPIO_WritePin(CATHODE_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(CATHODE_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(CATHODE_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(CATHODE_4, GPIO_PIN_RESET);
			}
			else if(ledPosition == 1){ //position (X)XX
					HAL_GPIO_WritePin(CATHODE_1, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(CATHODE_2, GPIO_PIN_SET);
					HAL_GPIO_WritePin(CATHODE_3, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(CATHODE_4, GPIO_PIN_RESET);
			}
			else if(ledPosition == 2){		//position X(X)X
					HAL_GPIO_WritePin(CATHODE_1, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(CATHODE_2, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(CATHODE_3, GPIO_PIN_SET);
					HAL_GPIO_WritePin(CATHODE_4, GPIO_PIN_RESET);
			}
			else if(ledPosition == 3){//position XX(X)
					HAL_GPIO_WritePin(CATHODE_1, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(CATHODE_2, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(CATHODE_3, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(CATHODE_4, GPIO_PIN_SET);
			}
			osDelay(2);
		}
		/* if sleep mode: turn off cathodes to turn off display */
		else if(state == 0) {
			HAL_GPIO_WritePin(CATHODE_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CATHODE_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CATHODE_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CATHODE_4, GPIO_PIN_RESET);
			osSignalSet(stateThread, sleepSignal);
		}
	}
}

void stateController(void const * argument) {
	while(1) {		
		/* waiting for signal that a key has been pressed */
		osSignalWait(buttonSignal,osWaitForever);
		buttonSignal = 0;
		/* wait for semaphore, then when received save value from keypad handler into variable and release semaphore */
		osSemaphoreWait(keyboardButtonSemaphore, osWaitForever);
		key	= valueReturned;
		valueReturned	= -1;
		osSemaphoreRelease(keyboardButtonSemaphore);
		switch(state) {
			case 0: //sleep mode			
				/* terminate threads in sleep mode for power consumption */
				osSignalWait(sleepSignal, osWaitForever);
				/* Stop timer for PWM generation */
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				/* Stop ADC interrupt triggered by timer */
				HAL_ADC_Stop_IT(&hadc1);
				//osThreadTerminate(displayThread);
				//osThreadTerminate(generalTimerThread);	
				//return;
				
				
				/* if pound held and sleep flag is on, turn on keypad */
//				if(key == 11 && sleepKey && keyValid) {
//					sleepKey = 0;
//					keyValid = 0;
//				}			
				break;
			case 1: //awake (reset) mode
				//reset all LEDs to 0
				outputDigits[0] =0;
				outputDigits[1] =0;
				outputDigits[2] =0;
				outputDigits[3] =0;
				/* if a key between 0-9 has been pressed, move to next state (first digit inputted) */
				if(key > -1 && key < 10 && keyValid) {
					buttonPressed = key;
					keyValid = 0;
					state = 2;
				}
				/* if star held and sleep flag is on, turn off keypad */
				else if(key == 10 && sleepKey && keyValid) {
					sleepKey = 0;
					keyValid = 0;
					state = 0;
				}
				break;
			case 2: //first digit
				/* input button pressed from state 1 into last digit */
				outputDigits[0] =0;
				outputDigits[1] =0;
				outputDigits[2] =0;
				outputDigits[3] = buttonPressed;
				/* if star held and sleep flag is on, turn off keypad */
				if(key == 10 && sleepKey && keyValid) {
					sleepKey = 0;
					keyValid = 0;
					state = 0;
				}
				/* if star held and restart flag is on, go to state 1 to reset keypad */
				else if(key == 10 && restartKey && keyValid) {
					restartKey = 0;
					keyValid = 0;
					state = 1;
				}
				/* if star pressed, delete inputted value by returning to reset state */
				else if(key == 10 && keyValid) {
					keyValid = 0;
					state = 1;
				}
				/* if a key between 0-9 has been pressed, move to next state (second digit inputted) */
				else if(key>-1 && key < 10 && keyValid) {
					buttonPressedOld = buttonPressed;
					buttonPressed = key;
					keyValid = 0;
					state = 3;
				}
				break;
			case 3: //second digit
				/* input button pressed from state 1 into third digit and input button from state 2 into last digit */
				outputDigits[0] =0;
				outputDigits[1] =0;
				outputDigits[2] = buttonPressedOld;
				outputDigits[3] = buttonPressed;
				/* if star held and sleep flag is on, turn off keypad */
				if(key == 10 && sleepKey && keyValid) {
					keyValid = 0;
					sleepKey = 0;
					state = 0;
				}
				/* if star held and restart flag is on, reset keypad by moving to state 1*/
				else if(key == 10 && restartKey && keyValid) {
					keyValid = 0;
					restartKey = 0;
					state = 1;
				}
				/* if star pressed, delete inputted value by returning to reset state */
				else if(key == 10 && keyValid) {
					buttonPressed = buttonPressedOld;
					keyValid = 0;
					state = 2;
				}
				/* if pound pressed, move to state 4 to submit value */
				else if(key == 11 && keyValid) {
					keyValid = 0;
					state = 4;
				}				
				break;
			case 4:
				/* submit user inputte value to controller */
				mathInput = (float) (buttonPressedOld + (buttonPressed/10.0));
				pwmController(mathInput, mathOutput);
				/* if star held and sleep flag is on, turn off keypad */
				if(key == 10 && sleepKey && keyValid) {
					keyValid = 0;
					sleepKey = 0;
					state = 0;
				}
				/* if star held and restart flag is on, reset keypad by moving to state 1*/
				else if(key == 10 && restartKey && keyValid) {
					keyValid = 0;
					sleepKey = 0;
					state = 1;
				}		
				break;
		}
	}
}
			
			
void getKeypadValue(void const * argument) {
	static int counter = 0;
	while(1) {
			/*go through rows and turn each one on, then check columns for input*/
			for(counter =0; counter < 4; counter++) {
				/*if(state == 0 && valueReturned != 11) {
					osSemaphoreWait(sleepSemaphore, osWaitForever);
					sleepCount = 0;
					osSemaphoreRelease(sleepSemaphore);
				} else if(state == 0) {
						valueReturned = -1;
				} else {
						osSignalSet(stateThread,buttonSignal);
				}*/
				if (counter == 0){
					HAL_GPIO_WritePin(ROW_1, GPIO_PIN_SET );
					HAL_GPIO_WritePin(ROW_2, GPIO_PIN_RESET );
					HAL_GPIO_WritePin(ROW_3, GPIO_PIN_RESET );
					HAL_GPIO_WritePin(ROW_4, GPIO_PIN_RESET );				
				}			
				else if (counter == 1) {
					HAL_GPIO_WritePin(ROW_1, GPIO_PIN_RESET );
					HAL_GPIO_WritePin(ROW_2, GPIO_PIN_SET );
					HAL_GPIO_WritePin(ROW_3, GPIO_PIN_RESET );
					HAL_GPIO_WritePin(ROW_4, GPIO_PIN_RESET );
				}
				else if (counter == 2){
					HAL_GPIO_WritePin(ROW_1, GPIO_PIN_RESET );
					HAL_GPIO_WritePin(ROW_2, GPIO_PIN_RESET );
					HAL_GPIO_WritePin(ROW_3, GPIO_PIN_SET );
					HAL_GPIO_WritePin(ROW_4, GPIO_PIN_RESET );
				}
				else if (counter == 3){
					HAL_GPIO_WritePin(ROW_1, GPIO_PIN_RESET );
					HAL_GPIO_WritePin(ROW_2, GPIO_PIN_RESET );
					HAL_GPIO_WritePin(ROW_3, GPIO_PIN_RESET );
					HAL_GPIO_WritePin(ROW_4, GPIO_PIN_SET );
				} 

				/*read the columns: if column is set to high, that means the button in the output row that is on is being pressed, so save that value
					wait until keyboard semaphore is free */
				osSemaphoreWait(keyboardButtonSemaphore, osWaitForever);
				if (HAL_GPIO_ReadPin(COLUMN_1) > 0) {
						valueReturned = keypadMatrix[0][counter]; 
						osSignalSet(stateThread,buttonSignal);
				}
				else if (HAL_GPIO_ReadPin(COLUMN_2) > 0) {
						valueReturned = keypadMatrix[1][counter]; 
						osSignalSet(stateThread,buttonSignal);
				}
				else if (HAL_GPIO_ReadPin(COLUMN_3) > 0) {
						valueReturned = keypadMatrix[2][counter]; 
						osSignalSet(stateThread,buttonSignal);
				}
				osSemaphoreRelease(keyboardButtonSemaphore);
			}	
		
	}
}

/**
  * @brief  PWM controller that controls the pulse of the PWM generation depending on user inputted value from keypad
	* @param  input: inputted value from keypad
	*	@param	output: RMS value from filter and c_math
  * @retval None
  */
void pwmController(float input, float output) {
	controllerTarget = input;
	/*calculate error between user inputted value and RMS value from ADC conversion*/
	error = input - output;	
	float pValue = 100.0f;
	
	/*adjust pulseWidth until within 5% of user inputted value*/
	while(error >= 0.04*controllerTarget) {
	
		/*if user value is higher than RMS value, increase pulse width*/
		if(error > 0 && pulseWidth < 8400) {
			pulseWidth+=5;
		}
		/*if user value is less than RMS value, decrease pulse width*/
		if(error < 0 && pulseWidth > 0) {
			pulseWidth-=5;
		}
		pulseWidth = (int) (error * pValue);
		/*set pulse width of PWM generation to the continuously updating result*/
		pwmSetValue(pulseWidth);
		/*update error with every updating pulse width value*/
		error = input-mathOutput;
		/*display value*/
		valueParse(mathOutput);
		/* if star held and sleep flag is on, turn off keypad */
				if(key == 10 && sleepKey && keyValid) {
					keyValid = 0;
					sleepKey = 0;
					state = 0;
				}
				/* if star held and restart flag is on, reset keypad by moving to state 1*/
				else if(key == 10 && restartKey && keyValid) {
					keyValid = 0;
					sleepKey = 0;
					state = 1;
				}	
		/*if at 5% of user inputted value, stop updating*/
		if(error == 0.04*controllerTarget) {
			HAL_ADC_Stop(&hadc1);
			//HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
			break;
		}
	}
}

/**
  * @brief  Reconfigure timer for PWM generation and set the pulse width to the output of the controller
	* @param  pulseValue: the pulse width returned from the PWM controller
  * @retval None
  */
void pwmSetValue(uint16_t pulseValue) {
	
		TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulseValue;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  
}
/**
  * @brief  Retrive ADC conversion value and put through filter and RMS calculation
  * @param  hadc: pointer to ADC handler
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
	adcConversion = HAL_ADC_GetValue(hadc);
	
	FIR_C(adcConversion, &filterOutput);
		
	filterOutput = filterOutput * 3.0 / 255.0; /* 8bit conversion, Vref at 3V */
	
	c_math(filterOutput);

}

/**
  * @brief  Filter inputted value from ADC conversion
	* @param  input: value from ADC conversion
	* @param 	output: pointer to array of outputted filter values
  * @retval None
  */
void FIR_C(uint32_t input, float *output) {
	
	float coefficients[5] = {0.2,0.2,0.2,0.2,0.2};
	/* moving window */
	for(int i = 0; i < 4; i++){
		x[i] = x[i+1];
	}
	/* set new input to final value in window */
	x[4] = input;
	float returnedOutput = 0;
	/* sum of products of coefficient and input */
	for(int i = 0; i < 5; i++){
		returnedOutput += x[i] * coefficients[4-i];
	}
	*output = returnedOutput;
}

/**
  * @brief  RMS calculation
  * @param  input: value from filter output
  * @retval None
  */
void c_math(float input){
	static int mathCounter;

	rms_value += input*input;
	/*increment for every rms_value to find length*/
	mathCounter++;

	/*rms is square root of (sum of squares divided by length)*/
	mathOutput = (float) (sqrt(rms_value/((float)(mathCounter))));

}

/**
  * @brief  Parsing of values returned from RMS calculation for display on LED display
	* @param  mathOutput: value returned from RMS calculation
  * @retval None
  */
void valueParse(float mathOutput){ 	
	
	/* first, second, third and fourth digit from RMS calculation */
	int firstDigit, secondDigit, thirdDigit, fourthDigit = 0;
	
	float temp = mathOutput;
	
	/*parse first, second and third digits in calculated value*/
	firstDigit = (int) (temp);
	secondDigit = (int) ((temp - (float)(firstDigit)) * 10.0f);
	thirdDigit = (int) ((((temp - (float)(firstDigit)) * 10.0f) - (float)(secondDigit)) * 10.0f);
	fourthDigit = (int) ((((((temp - (float)(firstDigit)) * 10.0f) - (float)(secondDigit)) * 10.0f - (float)(thirdDigit))) * 10.0f);
	
	/*set digits to output variables for LED display	*/
	outputDigits[0] = firstDigit;
	outputDigits[1] = secondDigit;
	outputDigits[2] = thirdDigit;
	outputDigits[3] = fourthDigit;
}
/* USER CODE END 4 */

///* StartDefaultTask function */
//void StartDefaultTask(void const * argument)
//{

//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END 5 */ 
//}

///* StartTask02 function */
//void StartTask02(void const * argument)
//{
//  /* USER CODE BEGIN StartTask02 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END StartTask02 */
//}

///* StartTask03 function */
//void StartTask03(void const * argument)
//{
//  /* USER CODE BEGIN StartTask03 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END StartTask03 */
//}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line valueReturned
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source valueReturned
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line valueReturned,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
