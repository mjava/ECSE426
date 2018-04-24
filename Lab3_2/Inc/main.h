/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Cathode_1_Pin GPIO_PIN_7
#define Cathode_1_GPIO_Port GPIOE
#define Cathode_2_Pin GPIO_PIN_8
#define Cathode_2_GPIO_Port GPIOE
#define Cathode_3_Pin GPIO_PIN_9
#define Cathode_3_GPIO_Port GPIOE
#define Cathode_4_Pin GPIO_PIN_10
#define Cathode_4_GPIO_Port GPIOE
#define Segment_A_Pin GPIO_PIN_11
#define Segment_A_GPIO_Port GPIOE
#define Segment_B_Pin GPIO_PIN_12
#define Segment_B_GPIO_Port GPIOE
#define Segment_C_Pin GPIO_PIN_13
#define Segment_C_GPIO_Port GPIOE
#define Segment_D_Pin GPIO_PIN_14
#define Segment_D_GPIO_Port GPIOE
#define Segment_E_Pin GPIO_PIN_15
#define Segment_E_GPIO_Port GPIOE
#define Segment_F_Pin GPIO_PIN_10
#define Segment_F_GPIO_Port GPIOB
#define Segment_G_Pin GPIO_PIN_11
#define Segment_G_GPIO_Port GPIOB
#define Row_Keypad_1_Pin GPIO_PIN_12
#define Row_Keypad_1_GPIO_Port GPIOB
#define Row_Keypad_2_Pin GPIO_PIN_13
#define Row_Keypad_2_GPIO_Port GPIOB
#define Row_Keypad_3_Pin GPIO_PIN_14
#define Row_Keypad_3_GPIO_Port GPIOB
#define Row_Keypad_4_Pin GPIO_PIN_15
#define Row_Keypad_4_GPIO_Port GPIOB
#define Column_Keypad_1_Pin GPIO_PIN_8
#define Column_Keypad_1_GPIO_Port GPIOD
#define Column_Keypad_2_Pin GPIO_PIN_9
#define Column_Keypad_2_GPIO_Port GPIOD
#define Column_Keypad_3_Pin GPIO_PIN_10
#define Column_Keypad_3_GPIO_Port GPIOD

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
