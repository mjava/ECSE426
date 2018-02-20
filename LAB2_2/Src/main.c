/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "math.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int input_vector[INPUT_LENGTH] = {0};
float filterOutputVector[INPUT_LENGTH-ORDER] = {0};
int filterOutputLength = INPUT_LENGTH - ORDER;
uint32_t adcConversion;
float filterOutput;
int digits[3] = {0};
float outputVector[3] = {0};
int outputDigits[4] = {0};
int x[5] = {0};

int ledPosition = 0;
int mathCounter = 0;
	
float rms_value = 0;
float max_value = 0;
float min_value = 0;

volatile int adcTrigger = 0; //triggers the callback conversion
volatile int displaySwitchTrigger = 0; //triggers the switch between LEDs
volatile int displayedValue = 0; //triggers the change between displayed values (RMS, MAX, MIN)
volatile int buttonReadTrigger = 0; //trigger sent from button press


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_NVIC_Init(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void FIR_C(uint32_t input, float *output);
void digitCalc(float* output, int value);
void ledDriver(int ledPos, int number);
void c_math(float input, float outputVector[]);
extern int buttonPressed(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void){
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_NVIC_Init();

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1500);
	
  while (1)
  {
	if(adcTrigger >= ADC_TIMER){
		adcTrigger = 0;
		HAL_ADC_Start_IT(&hadc1); 
	}
	
	if(displaySwitchTrigger >= LED_POSITION_TIMER){
		displaySwitchTrigger = 0;
		ledPosition++;
		ledPosition = ledPosition % 4;
		ledDriver(ledPosition, outputDigits[ledPosition]);
	}	
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  // ADC_IRQn interrupt configuration 
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
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

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
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
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_1_CATHODE_Pin|SEVEN_SEG_2_CATHODE_Pin|SEVEN_SEG_3_CATHODE_Pin|SEVEN_SEG_4_CATHODE_Pin 
                          |SEVEN_SEG_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin|SEVEN_SEG_C_Pin|SEVEN_SEG_D_Pin|SEVEN_SEG_E_Pin 
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin|SEVEN_SEG_G_Pin|SEVEN_SEG_DP_Pin|SEVEN_SEG_L3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEVEN_SEG_L1_Pin|SEVEN_SEG_L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin SEVEN_SEG_F_Pin SEVEN_SEG_G_Pin SEVEN_SEG_DP_Pin 
                           SEVEN_SEG_L3_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|SEVEN_SEG_F_Pin|SEVEN_SEG_G_Pin|SEVEN_SEG_DP_Pin 
                          |SEVEN_SEG_L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEVEN_SEG_1_CATHODE_Pin SEVEN_SEG_2_CATHODE_Pin SEVEN_SEG_3_CATHODE_Pin SEVEN_SEG_4_CATHODE_Pin 
                           SEVEN_SEG_A_Pin */
  GPIO_InitStruct.Pin = SEVEN_SEG_1_CATHODE_Pin|SEVEN_SEG_2_CATHODE_Pin|SEVEN_SEG_3_CATHODE_Pin|SEVEN_SEG_4_CATHODE_Pin 
                          |SEVEN_SEG_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SEVEN_SEG_B_Pin SEVEN_SEG_C_Pin SEVEN_SEG_D_Pin SEVEN_SEG_E_Pin 
                           LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = SEVEN_SEG_B_Pin|SEVEN_SEG_C_Pin|SEVEN_SEG_D_Pin|SEVEN_SEG_E_Pin 
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
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

  /*Configure GPIO pins : SEVEN_SEG_L1_Pin SEVEN_SEG_L2_Pin */
  GPIO_InitStruct.Pin = SEVEN_SEG_L1_Pin|SEVEN_SEG_L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
		adcConversion = HAL_ADC_GetValue(hadc);
		
		FIR_C(adcConversion, &filterOutput);
		
		filterOutput = filterOutput * 3.0 / 4096.0; //12bit conversion
		
		c_math(filterOutput, outputVector);
		
		digitCalc(outputVector, displayedValue, ledPosition); //value indicator in outputDigit[0], tens in outputDigit[1], tenths in outputDigit[2], hundredths in outputDigit[3]
		
		HAL_ADC_Stop_IT(&hadc1); //disable the ADC interrupt
}

void FIR_C(uint32_t input, float *output) {
	
	float coefficients[5] = {0.2,0.2,0.2,0.2,0.2};
	for(int i = 0; i < 4; i++){
		x[i] = x[i+1];
	}
	x[4] = input;
	float returnedOutput = 0;
	for(int i = 0; i < 5; i++){
		returnedOutput += x[i] * coefficients[4-i];
	}
	*output = returnedOutput;
		//printf("The input of the filter is: %d \n", input);
		//printf("The output of the filter is: %f \n", *output);
}

void c_math(float input, float outputVector[]){
	
	printf("The input is: %f \n", input);
	if(mathCounter == 0){
		min_value = input;
		max_value = input;
		rms_value = input*input;
	}
	else {
		if(input < min_value){
			min_value = input;
		}
		else if(input > max_value){
			max_value = input;
		}
		rms_value = rms_value + (input*input);
	}
	mathCounter++;
	if(mathCounter >= 500){
		mathCounter = 0;
	}
	//rms is square root of (sum of squares divided by length)
	outputVector[0] = (float) (sqrt(rms_value/((float)(mathCounter))));
	outputVector[1] = max_value;
	outputVector[2] = min_value;
}

void digitCalc(float* outputVector, int value){ 	//where outputVector is {rms, max, min}
	
	int firstDigit, secondDigit, thirdDigit = 0;
	
	float temp = outputVector[value];
	
	firstDigit = (int) (temp);
	secondDigit = (int) ((temp - (float)(firstDigit)) * 10.0f);
	thirdDigit = (int) ((((temp - (float)(firstDigit)) * 10.0f) - (float)(secondDigit)) * 10.0f);
	
	outputDigits[0] = value;
	outputDigits[1] = firstDigit;
	outputDigits[2] = secondDigit;
	outputDigits[3] = thirdDigit;
}

void ledDriver(int ledPos, int number){
	if(number == 0){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_RESET);
	}
	else if(number == 1){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_RESET);
	}
	else if(number == 2){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_SET);
	}
	else if(number == 3){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_SET);
	}
	else if(number == 4){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_SET);
	}
	else if(number == 5){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_SET);
	}
	else if(number == 6){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_SET);
	}
	else if(number == 7){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_RESET);
		}
	else if(number == 8){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_SET);
		}
	else if(number == 9){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEVEN_SEG_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SEVEN_SEG_G_Pin, GPIO_PIN_SET);
		}
		if(ledPos == 0){
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_1_CATHODE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_2_CATHODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_3_CATHODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_4_CATHODE_Pin, GPIO_PIN_RESET);
		}
	else if(ledPos == 1){ //position (X)XX
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_1_CATHODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_2_CATHODE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_3_CATHODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_4_CATHODE_Pin, GPIO_PIN_RESET);
		}
	else if(ledPos == 2){		//position X(X)X
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_1_CATHODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_2_CATHODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_3_CATHODE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_4_CATHODE_Pin, GPIO_PIN_RESET);
		}
	else if(ledPos == 3){//position XX(X)
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_1_CATHODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_2_CATHODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_3_CATHODE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, SEVEN_SEG_4_CATHODE_Pin, GPIO_PIN_SET);
		}
}


int buttonPressed(){
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
		return 1;
	}
	else{ 
		return 0;
	}	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
