#include "uart.h"

/*
Delay_ms():
    Delay in milliseconds
*/
void Delay_ms(int delay){
    HAL_Delay(delay);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
  void SystemClock_Config(void)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 90;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* 


Clock_Init():
    Initializes GPIOA and USART1 Clock
*/
void Clock_Init(){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
}

/*
GPIO_Init():
    Initialize GPIOA port and PINS 9, 10 & 5
*/
void GPIOA_Init(){
    // Initialize GPIOA Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Step 1. Initalize I/O Pins as alternate mode : PA9 - USART1_TX & PA10 - USART1_RX
    GPIOA->MODER |= (2 << 18); // alternate function for PA9
    GPIOA->MODER |= (2 << 20); // alternate function for PA10
    GPIOA->OSPEEDR |= ((3 << 18) | (3 << 20)); // High Speed for PA9 & PA10
    // Step 2. Set alternate functions for PA9 & PA10 : 0111 <- USART
    GPIOA->AFR[1] |= (7 << 4); // PA9  -> AF7 (USART_TX)
    GPIOA->AFR[1] |= (7 << 8); // PA10 -> AF7 (USART_RX)

    // PIN 5 SETUP
    GPIOA->MODER |= (1 << 10); // Output Mode for Pin5 : (1 @ bit 2y)
    GPIOA->MODER &= ~(1 << 11); // Output Mode for Pin5 : (0 @ bit 2y+1)
    GPIOA->OTYPER &= ~(1 << 5); // Output Push-pull : (0 @ bit 5)
    GPIOA->OSPEEDR &= ~((1 << 11) | (1 << 10)); // Slow speed for Pin5
    GPIOA->PUPDR &= ~((1 << 10) | (1 << 11)); // No Pull-up/down resistor : (00 @ bits 2y:2y+1)
}

void GPIOA_Set_Pin(){
    GPIOA->BSRR = (1 << 5); // Set Pin PA5 High
}

void GPIOA_Reset_Pin(){
    GPIOA->BSRR = (1 << 21); // Reset Pin PA5 Low
}

/*
ISO_Init():
    Initialize ISO-9141 communication by sending 0x33 @ 5 baud
*/
void ISO_Init(){
    uint8_t data = 0x33;
    // 1. Output PIN5 LOW
    GPIOA->BSRR = (1 << 21);
    Delay_ms(200);

    // 2. Send 8 bit data 0x33
    for(int i = 0; i < 8; i++){
        uint8_t bit = (data >> i) & 1;
        if(bit){
            GPIOA->BSRR = (1 << 5);
        }else{
            GPIOA->BSRR = (1 << 21);
        }
        Delay_ms(200);
    }

    // 3. Output PIN5 HIGH
    GPIOA->BSRR = (1 << 5);
    Delay_ms(1000);
}

/*
UART_Init():
    Initialize USART1
*/
void USART1_Init(){
    // Initialize USART1 Clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    // Step 3. Initialize USART
    USART1->CR1 |= (1 << 13);  // USART Enable
    USART1->CR1 &= ~(1 << 12); // 0: 1 Start bit, 8 Data bits, n Stop bit
    USART1->CR2 &= ~(3 << 12); // 1 stop bit
    USART1->BRR = USART1_CLK_FREQ / USART1_BAUD_RATE; // Initial Baud Rate of 5

    USART1->CR1 |= (1 << 3); // Transmitter Enabled
    USART1->CR1 |= (1 << 2); // Receiver Enabled

}

/*
USART1_SendChar():
    Transmits character over UART
*/
void USART1_SendData(uint8_t d){
    while(!(USART1->SR & (1 << 7))); // Wait for Transmit Data Register to be empty
    USART1->DR = d;
    while(!(USART1->SR & (1 << 6))); // Wait for TC to SET
}

/*
USART1_SendString():
    Transmits string over UART
*/
void USART1_SendString(char *str){
    while(*str){
        USART1_SendData(*str++);
    }
}

/*
USART1_ReceiveChar():
    Receives character over UART
*/
uint8_t USART1_ReceiveChar(){
    while(!(USART1->SR & (1 << 5))); // wait for received data
    uint8_t data = USART1->DR; // read data
    return data;
}