/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sendChar(char);
void stringArray(char[]);

//void USART3_4_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char temp;
char number;
int flag;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  HAL_Init();
  SystemClock_Config();
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  //PB11 and PB13
  //PB13 to alternate function mode, open-drain output type
  
  GPIOB->MODER &= ~(3 << 11 * 2);
  GPIOB->MODER |= (2 << 11 * 2); // alternate mode
  GPIOB->OTYPER &= ~(1 << 11);
  GPIOB->OTYPER |= (1 << 11); // open-drain
  GPIOB->AFR[1] &= ~(0xF << 12);
  GPIOB->AFR[1] |= (1 << 12);// Set alternate function to AF5 (I2C2_SDA)

  GPIOB->MODER &= ~(3 << 13 * 2);
  GPIOB->MODER |= (2 << 13 * 2); // alternate mode
  GPIOB->OTYPER &= ~(1 << 13); 
  GPIOB->OTYPER |= (1 << 13); // open-drain
  GPIOB->AFR[1] &= ~(0xF << 20);
  GPIOB->AFR[1] |= (5 << 20); // Set alternate function to AF1 (I2C2_SCL)

  //PB14  output mode, push-pull output type, and initialize/set the pin high
  GPIOB->MODER &= ~(3 << 14 * 2);
  GPIOB->MODER |= (1 << 14 * 2); // output mode
  GPIOB->OTYPER &= ~(1 << 14); // push-pull output type
  //GPIOB->ODR |= (1 << 7); // initialize/set the pin high
  GPIOB->BSRR = (1 << 14); // initialize/set the pin high

  //PC0 output mode, push-pull output type, and initialize/set the pin high.
  GPIOC->MODER &= ~(3 << 0); 
  GPIOC->MODER |= (1 << 0); // output mode
  GPIOC->OTYPER &= ~(1 << 0); // push-pull output type
  //GPIOC->ODR |= (1 << 0); // initialize/set the pin high
  GPIOC->BSRR = (1 << 0);         // Set pin 0 high

  GPIOC->MODER &= ~(3 << (9 * 2) | 3 << (6 * 2)); // clear
  GPIOC->MODER |= (1 << (9 * 2) | 1 << (6 * 2) ); // PC9

  // I2C2 setting
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
  //I2C2->TIMINGR |= (1 << 28) | (0x13 << 20) | (0xF << 16) | (0x2 << 8) | 0x4;
  I2C2->TIMINGR = (1 << 28) | // PRESC
                (0x13 << 0) | // SCLL
                (0xF << 8) |  // SCLH
                (0x2 << 16) | // SDADEL
                (0x4 << 20);  // SCLDEL

  I2C2->CR1 |= I2C_CR1_PE; // Enable the I2C peripheral using the PE bit in the CR1 register.
// part 1
  // // Clear the NBYTES and SADD bit fields
  // I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  // // Set NBYTES = 1 and SADD = 0x69
  //  //Transmit
  // I2C2->CR2 |= (1 << 16);
  // I2C2->CR2 |= (0x69 << 1);

  // // Clear RD_WRN and 0 is write option
  // I2C2->CR2 &= ~I2C_CR2_RD_WRN;
  // // Ser START bit
  // I2C2->CR2 |= I2C_CR2_START;

  // // #2　Wait until either TXIS or NACKF flags are set
  // while (!(I2C2->ISR & (1 << 1)) && !(I2C2->ISR & (1 << 4)));
      
  // if (I2C2->ISR & (1 << 4)) { 
        
  // // error(wiring)
  // } else if (I2C2->ISR & (1 << 1)) {

  //   //WHO_AM_I register 
  //   I2C2->TXDR = 0x0F;

  //   // wait until TC flag is set (1 << 6)
  //   while(!(I2C2->ISR & I2C_ISR_TC));

  //   // 5. Reload the CR2 register with the parameters for a read operation I2C_CR2_RD_WRN(10bits)
  //   I2C2->CR2 &= ~I2C_CR2_RD_WRN; // Clear the RD_WRN bit first if it was previously set for write
  //   I2C2->CR2 |= I2C_CR2_RD_WRN;  // Set the RD_WRN bit to indicate a read operation
  //   I2C2->CR2 |= I2C_CR2_START;   // Set the START bit again to perform a I2C restart condition

  //   // 6. Wait until either of the RXNE or NACKF flags are set
  //   // same as while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {}
  //   while (!(I2C2->ISR & (1 << 2)) && !(I2C2->ISR & (1 << 4))){
  //   //  wait here until one of the flags is set
  //   }

  //   // Check if NACKF flag is set
  //   if (I2C2->ISR & I2C_ISR_NACKF) {
  //     //wiring
  //   } else if (I2C2->ISR & I2C_ISR_RXNE) {
  //      // wait until TC flag is set (1 << 6)
  //      while(!(I2C2->ISR & I2C_ISR_TC));

  //     if(I2C2->RXDR == 0xD3) {
  //       // // PC9 to high green
  //       GPIOC->ODR |= (1 << 9);        
  //     }
    
  //     // stop
  //     I2C2->CR2 |= I2C_CR2_STOP;

  //   }
  
  // }
  
  // part 2
   // Clear the NBYTES and SADD bit fields
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  // Set NBYTES = 1 and SADD = 0x69
   //Transmit
  I2C2->CR2 |= (1 << 16);
  I2C2->CR2 |= (0x69 << 1);

  // Clear RD_WRN and 0 is write option
  I2C2->CR2 &= ~I2C_CR2_RD_WRN;
  // Ser START bit
  I2C2->CR2 |= I2C_CR2_START;

  // // #2　Wait until either TXIS or NACKF flags are set
  while (!(I2C2->ISR & (1 << 1)) && !(I2C2->ISR & (1 << 4)));
      
  if (I2C2->ISR & I2C_ISR_NACKF) {
      GPIOC->ODR |= (1 << 6); 
  }

  // write CTRL_REG1 into I2C transmit register & set PD to Xen & Yen
  // CTRL_REG1 |DR1|DR0|BW1|BW0|PD|Zen|Yen|Xen| (PD =1 Yen=1 Xen=1 ) = 0x0B
  I2C2->TXDR |= (0x20 | 0x0B);

  //waiting for TC
  while(!(I2C2->ISR & I2C_ISR_TC));
  //stop
  I2C2->CR2 |= I2C_CR2_STOP;

  char x_LSB, x_MSB, y_LSB, y_MSB;
  int x_data, y_data;
  int THRESHOLD = 1000;

  while(1) {
    
     // Request X-axis LSB and MSB
    // Assuming X-axis LSB at 0x28 and MSB at 0x29
    x_LSB = read_I2C_register(0x28);
    x_MSB = read_I2C_register(0x29);
    // Combine to form X data
    x_data = ((int)x_MSB << 8) | x_LSB;

    // Request Y-axis LSB and MSB
    // Assuming Y-axis LSB at 0x2A and MSB at 0x2B
    y_LSB = read_I2C_register(0x2A);
    y_MSB = read_I2C_register(0x2B);
    // Combine to form Y data
    y_data = ((int)y_MSB << 8) | y_LSB;

     // Apply threshold and control LEDs for X-axis
    if (x_data > THRESHOLD) {
        // Light up LED for positive X rotation
        GPIO_SetBits(GPIOC, LED_POS_X);
        GPIO_ResetBits(GPIOC, LED_NEG_X);
    } else if (x_data < -THRESHOLD) {
        // Light up LED for negative X rotation
        GPIO_ResetBits(GPIOC, LED_POS_X);
        GPIO_SetBits(GPIOC, LED_NEG_X);
    }

    // Apply threshold and control LEDs for Y-axis
    if (y_data > THRESHOLD) {
        // Light up LED for positive Y rotation
        GPIO_SetBits(GPIOC, LED_POS_Y);
        GPIO_ResetBits(GPIOC, LED_NEG_Y);
    } else if (y_data < -THRESHOLD) {
        // Light up LED for negative Y rotation
        GPIO_ResetBits(GPIOC, LED_POS_Y);
        GPIO_SetBits(GPIOC, LED_NEG_Y);
    }

    for(int i = 0; i < 1000000; ++i) {}
      
  //   // Check if new data is available
  //  // int status = I2C_ReadRegister(0x0F, 0x27);
  //  // if(status & 0x08) { // Check if ZYXDA bit is set

  //   if(status) { 
  //       // Read X(0x28) and Y(0x29)
  //       int x_data = I2C_ReadData(0x28, 0x29);
  //       int y_data = I2C_ReadData(0x29, 0x28);

  //       // Process the data (implement your rotation indicator logic here)

  //       // Simple delay loop; replace with a proper delay mechanism to match sensor data rate
  //       for(int i = 0; i < 1000000; ++i) {}
  //   }
   }
}
/*
void I2C_WriteRegister(int deviceAddr, int registerAddr, int data) {
    // Send START condition and slave address for write
    I2C2->CR2 = (deviceAddr << 1) | (1 << 16); // SADD = deviceAddr, NBYTES = 1
    I2C2->CR2 &= ~I2C_CR2_RD_WRN;              // Write operation
    I2C2->CR2 |= I2C_CR2_START;                // Start condition

    // Wait until TXIS flag is set or NACKF flag is set
    while (!(I2C2->ISR & I2C_ISR_TXIS) && !(I2C2->ISR & I2C_ISR_NACKF));

    // Check for NACKF error
    if (I2C2->ISR & I2C_ISR_NACKF) {
        // Error handling
        return;
    }

    //  Send register address
    I2C2->TXDR = registerAddr;

    // Wait until TXIS flag is set for data transmission
    while (!(I2C2->ISR & I2C_ISR_TXIS));

    // Send data
    I2C2->TXDR = data;

    //  Wait until TC flag is set
    while (!(I2C2->ISR & I2C_ISR_TC));

    // Send STOP condition
    I2C2->CR2 |= I2C_CR2_STOP;
}

int I2C_ReadData(int lowAddr, int highAddr) {
    int lowByte = I2C_ReadRegister(L3GD20_ADDR, lowAddr | 0x80); // Set MSB for auto-increment
    int highByte = I2C_ReadRegister(L3GD20_ADDR, highAddr);
    return (int)((highByte << 8) | lowByte);
}


int I2C_ReadRegister(int deviceAddr, int registerAddr) {
    // Send START condition and slave address for write
    I2C2->CR2 = (deviceAddr << 1) | (1 << 16); // SADD = deviceAddr, NBYTES = 1
    I2C2->CR2 &= ~I2C_CR2_RD_WRN;              // Write operation
    I2C2->CR2 |= I2C_CR2_START;                // Start condition

    //  Wait until TXIS flag is set or NACKF flag is set
    while (!(I2C2->ISR & I2C_ISR_TXIS) && !(I2C2->ISR & I2C_ISR_NACKF));

    // Check for NACKF error
    if (I2C2->ISR & I2C_ISR_NACKF) {
        // Error handling
        return 0; // Should be an error code indicating failure
    }

    //  Send register address
    I2C2->TXDR = registerAddr;

    // Wait until TC flag is set
    while (!(I2C2->ISR & I2C_ISR_TC));

    // Send repeated START condition and slave address for read
    I2C2->CR2 = (deviceAddr << 1) | (1 << 16) | I2C_CR2_RD_WRN; // Set RD_WRN bit for read
    I2C2->CR2 |= I2C_CR2_START;                                 // Start condition

    // Wait until RXNE flag is set
    while (!(I2C2->ISR & I2C_ISR_RXNE));

    // Read data
    int data = I2C2->RXDR;

    //  Send STOP condition
    I2C2->CR2 |= I2C_CR2_STOP;

    return data;
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */