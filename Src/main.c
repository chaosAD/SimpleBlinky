/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct Led Led;
struct Led {
	int state;
	int tickstart;
	GPIO_TypeDef* port;
	uint16_t pin;
	int onPeriod;
	int offPeriod;
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void doBlinking(void);
void blinkLed(Led *led);
void initLed(Led *led, GPIO_TypeDef* port, uint16_t pin,			\
			 int onPeriod, int offPeriod);
void ledWaitAndSwitch(Led *led, int period, int pinState, int nextState);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//  Led led1;
//  Led led2;
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

  /* USER CODE BEGIN 2 */
//  initLed(&led1, amberLed_GPIO_Port, amberLed_Pin, 500, 500);
//  initLed(&led2, led7_GPIO_Port, led7_Pin, 150, 150);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  blinkLed(&led1);
//	  blinkLed(&led2);
	  doBlinking();
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(amberLed_GPIO_Port, amberLed_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, led24_Pin|led25_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, led26_Pin|led27_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led21_Pin|led22_Pin|led23_Pin|led7_Pin 
                          |led8_Pin|led9_Pin|led10_Pin|led11_Pin 
                          |led28_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led2_Pin|led3_Pin|led4_Pin|led5_Pin 
                          |led6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led16_Pin|led17_Pin|led18_Pin|led19_Pin 
                          |led20_Pin|led12_Pin|led13_Pin|led14_Pin 
                          |led15_Pin|led29_Pin|led30_Pin|led31_Pin 
                          |led32_Pin|led33_Pin|led34_Pin|led35_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : amberLed_Pin */
  GPIO_InitStruct.Pin = amberLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(amberLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led24_Pin led25_Pin */
  GPIO_InitStruct.Pin = led24_Pin|led25_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : led26_Pin led27_Pin */
  GPIO_InitStruct.Pin = led26_Pin|led27_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : led21_Pin led22_Pin led23_Pin led4_Pin 
                           led5_Pin led6_Pin led7_Pin led8_Pin 
                           led9_Pin led10_Pin led11_Pin led28_Pin */
  GPIO_InitStruct.Pin = led21_Pin|led22_Pin|led23_Pin|led4_Pin 
                          |led5_Pin|led6_Pin|led7_Pin|led8_Pin 
                          |led9_Pin|led10_Pin|led11_Pin|led28_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led2_Pin led3_Pin */
  GPIO_InitStruct.Pin = led2_Pin|led3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led16_Pin led17_Pin led18_Pin led19_Pin 
                           led20_Pin led12_Pin led13_Pin led14_Pin 
                           led15_Pin led29_Pin led30_Pin led31_Pin 
                           led32_Pin led33_Pin led34_Pin led35_Pin */
  GPIO_InitStruct.Pin = led16_Pin|led17_Pin|led18_Pin|led19_Pin 
                          |led20_Pin|led12_Pin|led13_Pin|led14_Pin 
                          |led15_Pin|led29_Pin|led30_Pin|led31_Pin 
                          |led32_Pin|led33_Pin|led34_Pin|led35_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

}

/* USER CODE BEGIN 4 */
void initLed(Led *led, GPIO_TypeDef* port, uint16_t pin,			\
			int onPeriod, int offPeriod) {
	led->state = 0;
	led->tickstart = 0;
	led->port = port;
	led->pin = pin;
	led->onPeriod = onPeriod;
	led->offPeriod = offPeriod;
}

void ledWaitAndSwitch(Led *led, int period, int pinState, int nextState) {
	if((HAL_GetTick() - led->tickstart) > period) {
		HAL_GPIO_WritePin(led->port, led->pin, pinState);
		led->tickstart = HAL_GetTick();
		led->state = nextState;
	}
}

void blinkLed(Led *led) {
  switch(led->state) {
  case 0:	// WAIT TO ON
	  ledWaitAndSwitch(led, led->offPeriod, GPIO_PIN_RESET, 1);
	  break;
  case 1:   // WAIT TO OFF
	  ledWaitAndSwitch(led, led->onPeriod, GPIO_PIN_SET, 0);
	  break;
  default:
	  led->state = 0;
  }
}

void doBlinking() {
    HAL_GPIO_TogglePin(amberLed_GPIO_Port, amberLed_Pin);
    HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
    HAL_GPIO_TogglePin(led3_GPIO_Port, led3_Pin);
    HAL_GPIO_TogglePin(led4_GPIO_Port, led4_Pin);
    HAL_GPIO_TogglePin(led5_GPIO_Port, led5_Pin);
    HAL_GPIO_TogglePin(led6_GPIO_Port, led6_Pin);
    HAL_GPIO_TogglePin(led7_GPIO_Port, led7_Pin);
    HAL_GPIO_TogglePin(led8_GPIO_Port, led8_Pin);
    HAL_GPIO_TogglePin(led9_GPIO_Port, led9_Pin);
    HAL_GPIO_TogglePin(led10_GPIO_Port, led10_Pin);
    HAL_GPIO_TogglePin(led11_GPIO_Port, led11_Pin);
    HAL_GPIO_TogglePin(led12_GPIO_Port, led12_Pin);
    HAL_GPIO_TogglePin(led13_GPIO_Port, led13_Pin);
    HAL_GPIO_TogglePin(led14_GPIO_Port, led14_Pin);
    HAL_GPIO_TogglePin(led15_GPIO_Port, led15_Pin);
    HAL_GPIO_TogglePin(led16_GPIO_Port, led16_Pin);
    HAL_GPIO_TogglePin(led17_GPIO_Port, led17_Pin);
    HAL_GPIO_TogglePin(led18_GPIO_Port, led18_Pin);
    HAL_GPIO_TogglePin(led19_GPIO_Port, led19_Pin);
    HAL_GPIO_TogglePin(led20_GPIO_Port, led20_Pin);
    HAL_GPIO_TogglePin(led21_GPIO_Port, led21_Pin);
    HAL_GPIO_TogglePin(led22_GPIO_Port, led22_Pin);
    HAL_GPIO_TogglePin(led23_GPIO_Port, led23_Pin);
    HAL_GPIO_TogglePin(led24_GPIO_Port, led24_Pin);
    HAL_GPIO_TogglePin(led25_GPIO_Port, led25_Pin);
    HAL_GPIO_TogglePin(led26_GPIO_Port, led26_Pin);
    HAL_GPIO_TogglePin(led27_GPIO_Port, led27_Pin);
    HAL_GPIO_TogglePin(led28_GPIO_Port, led28_Pin);
    HAL_GPIO_TogglePin(led29_GPIO_Port, led29_Pin);
    HAL_GPIO_TogglePin(led30_GPIO_Port, led30_Pin);
    HAL_GPIO_TogglePin(led31_GPIO_Port, led31_Pin);
    HAL_GPIO_TogglePin(led32_GPIO_Port, led32_Pin);
    HAL_GPIO_TogglePin(led33_GPIO_Port, led33_Pin);
    HAL_GPIO_TogglePin(led34_GPIO_Port, led34_Pin);
    HAL_GPIO_TogglePin(led35_GPIO_Port, led35_Pin);
    HAL_Delay(500);
}
/* USER CODE END 4 */

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
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
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
