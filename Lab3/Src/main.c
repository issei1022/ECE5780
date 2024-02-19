/**
  *
  * Brandon Mouser
  * U0962682
  *
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
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  // RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  // RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  // RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // GPIOC->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2))); // clear
  // GPIOC->MODER |= (1 << (8 * 2)); // PC8
  // GPIOC->MODER |= (1 << (9 * 2)); // PC9
  // GPIOC->OTYPER &= ~((1 << 8) | (1 << 9)); // clear and both push-pull type
  // GPIOC->OSPEEDR &= ~((1 << (8 * 2)) | (1 << (9 * 2))); // clear and both low speed type
  // GPIOC->PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2))); // clear and both no pull-up down


  // //uint32_t T_target = 8000;
  // //uint8_t F_target = 4;

  // uint32_t psc = 7999;
  // uint32_t arr = 125;

  // TIM2->PSC = psc;  
  // TIM2->ARR = arr;
  // TIM2->EGR = TIM_EGR_UG; 
  // TIM2->DIER |= TIM_DIER_UIE; //TIM_DIER_UIE enable update interapt
  // // enable timer
  // TIM2->CR1 |= TIM_CR1_CEN;

  // // // PC9 to high green
  // GPIOC->BSRR |= (1 << 9);

  // NVIC_EnableIRQ(TIM2_IRQn);

  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  uint32_t psc_tim3 = 39; // prescaler
  uint32_t arr_tim3 = 125; // auto reload register

  // PWM
  TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // channel 1
  TIM3->CCMR1 |= TIM_CCMR1_OC2PE; // channel 2

  TIM3->CCMR1 |= (0x06 << 4);  // OC1M[2:0] = 110 PWM mode 2
  TIM3->CCMR1 |= (0x06 << 12); // OC2M[2:0] = 110 PWM mode 2

  // channel 1 and 2 as out
  TIM3->CCER |= TIM_CCER_CC1E; // channel 1 enabled
  TIM3->CCER |= TIM_CCER_CC2E; // channel 2 enabled

  // 800 Hz
  TIM3->PSC = psc_tim3;
  TIM3->ARR = arr_tim3;

  // set CCR to 20% of AER
  TIM3->CCR1 = arr_tim3 / 5;
  TIM3->CCR2 = arr_tim3 / 5;

  //enable counter of taimer
  TIM3->CR1 |= TIM_CR1_CEN;

  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2))); 
  GPIOC->MODER |= (2 << (6 * 2)) | (2 << (7 * 2)); 

  //GPIOC->AFR[0] |= (1 << (6 * 4)) | (1 << (7 * 4));


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* USER CODE BEGIN 4 */

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
