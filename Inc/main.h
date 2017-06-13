/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define amberLed_Pin GPIO_PIN_13
#define amberLed_GPIO_Port GPIOC
#define led24_Pin GPIO_PIN_14
#define led24_GPIO_Port GPIOC
#define led25_Pin GPIO_PIN_15
#define led25_GPIO_Port GPIOC
#define led26_Pin GPIO_PIN_0
#define led26_GPIO_Port GPIOD
#define led27_Pin GPIO_PIN_1
#define led27_GPIO_Port GPIOD
#define led21_Pin GPIO_PIN_0
#define led21_GPIO_Port GPIOA
#define led22_Pin GPIO_PIN_1
#define led22_GPIO_Port GPIOA
#define led23_Pin GPIO_PIN_2
#define led23_GPIO_Port GPIOA
#define led2_Pin GPIO_PIN_3
#define led2_GPIO_Port GPIOA
#define led3_Pin GPIO_PIN_4
#define led3_GPIO_Port GPIOA
#define led4_Pin GPIO_PIN_5
#define led4_GPIO_Port GPIOA
#define led5_Pin GPIO_PIN_6
#define led5_GPIO_Port GPIOA
#define led6_Pin GPIO_PIN_7
#define led6_GPIO_Port GPIOA
#define led16_Pin GPIO_PIN_0
#define led16_GPIO_Port GPIOB
#define led17_Pin GPIO_PIN_1
#define led17_GPIO_Port GPIOB
#define led18_Pin GPIO_PIN_2
#define led18_GPIO_Port GPIOB
#define led19_Pin GPIO_PIN_10
#define led19_GPIO_Port GPIOB
#define led20_Pin GPIO_PIN_11
#define led20_GPIO_Port GPIOB
#define led12_Pin GPIO_PIN_12
#define led12_GPIO_Port GPIOB
#define led13_Pin GPIO_PIN_13
#define led13_GPIO_Port GPIOB
#define led14_Pin GPIO_PIN_14
#define led14_GPIO_Port GPIOB
#define led15_Pin GPIO_PIN_15
#define led15_GPIO_Port GPIOB
#define led7_Pin GPIO_PIN_8
#define led7_GPIO_Port GPIOA
#define led8_Pin GPIO_PIN_9
#define led8_GPIO_Port GPIOA
#define led9_Pin GPIO_PIN_10
#define led9_GPIO_Port GPIOA
#define led10_Pin GPIO_PIN_11
#define led10_GPIO_Port GPIOA
#define led11_Pin GPIO_PIN_12
#define led11_GPIO_Port GPIOA
#define led28_Pin GPIO_PIN_15
#define led28_GPIO_Port GPIOA
#define led29_Pin GPIO_PIN_3
#define led29_GPIO_Port GPIOB
#define led30_Pin GPIO_PIN_4
#define led30_GPIO_Port GPIOB
#define led31_Pin GPIO_PIN_5
#define led31_GPIO_Port GPIOB
#define led32_Pin GPIO_PIN_6
#define led32_GPIO_Port GPIOB
#define led33_Pin GPIO_PIN_7
#define led33_GPIO_Port GPIOB
#define led34_Pin GPIO_PIN_8
#define led34_GPIO_Port GPIOB
#define led35_Pin GPIO_PIN_9
#define led35_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
