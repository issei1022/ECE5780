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
char receivedData;
char receivedNum;
int dataFlag;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* sending char */
void Transmit_USART(char[]);

/* toggle LED */
void LED_Toggle(char);

/* handler */
void USART3_4_IRQHandler(void);

/* LED action */
void Control_LED(char, char);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  //RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  GPIOB->MODER &= ~((3 << (10 * 2)) | (3 << (11 * 2))); // clear
  GPIOB->MODER |= (2 << (10 * 2)); // PB10 to alternate function 
  GPIOB->MODER |= (2 << (11 * 2)); // PB11 to alternate function 
  GPIOB->AFR[1] |= ((4 <<  8) | (4 <<  12)); // AFRH10 and AFRH11 to AF4

  //GPIOB->BRR = 115200; // baud rate
  USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;

  USART3->CR1 |= USART_CR1_TE; // Transmitter enable 
  USART3->CR1 |= USART_CR1_RE; // Reciever enable
  USART3->CR1 |= USART_CR1_UE; // Enable USART
  //USART3->CR1 |= USART_CR1_RXNEIE; // 

  // clear LED
  GPIOC->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
  // set red green blue LED
  GPIOC->MODER |= (1 << (6 * 2)) | (1 << (7 * 2)) | (1 << (9 * 2));
  GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 9)); // clear and both push-pull type
  GPIOC->OSPEEDR &= ~((1 << (6 * 2)) | (1 << (7 * 2)) | (1 << (9 * 2))); // clear and both low speed type
  GPIOC->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2)) | (3 << (9 * 2))); // clear and both no pull-up down

  // //part 1
  //char msg[] = "hello world!\n";

  // while(1) {
  //   //Transmit_USART(msg);
  //   //msg = null;
  //       //for (int i = 0; i < 1000000; i++); // simple delay
  //   HAL_Delay(1000);

  // }

    // part 2
   //char receivedChar;

   // while(1){
   //  LED_Toggle(receivedChar);
   //      if (USART3->ISR & USART_ISR_RXNE){
   //          receivedChar = USART3->RDR;// read
   //          LED_Toggle(receivedChar);
   //      }
   //  }

   // part 3
    NVIC_EnableIRQ(USART3_4_IRQn);
   dataFlag = 0;
   while(1){

    Transmit_USART("CMD?\n");
    HAL_Delay(1000);

  }

}


/**
 * sending a char 
*/
void Transmit_USART(char symbol[])
{
  int i = 0;
  while(*symbol != '\0') {
      while (!(USART3->ISR & USART_ISR_TXE)) {
        // wait until TXE (Transmit Data Register Empty) flag is set
      }
      USART3->TDR = symbol[i]; // send the character
      i++;
  }
  while (!(USART3->ISR & USART_ISR_TC)) {
        // wait until TC (Transmission Complete) flag is set
  }
}

/**
 *  depanding od a command to toggle LED  
 */
void LED_Toggle(char command) {
    switch (command) {
        case 'r':
            GPIOC->ODR ^= (1 << 7); // Toggle red LED connected to PC7
            break;
        case 'g':
            GPIOC->ODR ^= (1 << 8); // Toggle green LED connected to PC8
            break;
        case 'b':
            GPIOC->ODR ^= (1 << 9); // Toggle blue LED connected to PC9
            break;
        default:
            Transmit_USART("Invalid\n");
            break;
    }
}

void USART3_4_IRQHandler(void) {
    //if (USART3->ISR & USART_ISR_RXNE) {}
    if (dataFlag == 0) {
        receivedData = USART3->RDR;
        dataFlag = 1;
    } else if(dataFlag == 1){
        receivedNum = USART3->RDR;
        dataFlag = 0;
        Control_LED(receivedData, receivedNum);
    }
}

void Control_LED(char led, char action){

  switch (led) {
     case 'r':
         if (action == '0') GPIOC->ODR &= ~(1 << 6); // OFF
         else if (action == '1') GPIOC->ODR |= (1 << 6); // ON
         else if (action == '2') GPIOC->ODR ^= (1 << 6); // TOGGLE
         else Transmit_USART("Invalid");
     case 'g':
         if (action == '0') GPIOC->ODR &= ~(1 << 6); // OFF
         else if (action == '1') GPIOC->ODR |= (1 << 6); // ON
         else if (action == '2') GPIOC->ODR ^= (1 << 6); // TOGGLE
         else Transmit_USART("Invalid");
     case 'b':
         if (action == '0') GPIOC->ODR &= ~(1 << 6); // OFF
         else if (action == '1') GPIOC->ODR |= (1 << 6); // ON
         else if (action == '2') GPIOC->ODR ^= (1 << 6); // TOGGLE
         else Transmit_USART("Invalid");
     default:
         Transmit_USART("Invalid");
  }



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
