#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"

#define GPIOA_PIN5 5

void Delay_ms();
void USART1_Clock_Init();
void GPIOA_Clock_Init();
void GPIOA_Init();
void GPIOA_Set_Pin();
void GPIOA_Reset_Pin();
void USART1_Set_Baud(int baud);
void USART1_Init();
void USART1_SendData(uint8_t c);
void USART1_SendString(char *str);
uint8_t USART1_ReceiveData();


#endif /* UART_H */