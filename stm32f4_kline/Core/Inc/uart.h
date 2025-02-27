#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"

#define GPIOA_PIN5 5
#define USART1_CLK_FREQ 90000000
#define USART1_BAUD_RATE 10400

void Delay_ms();
void Clock_Init();
void GPIOA_Init();
void GPIOA_Set_Pin();
void GPIOA_Reset_Pin();
int USART1_Set_Baud(int baud);
void USART1_Init();
void USART1_SendData(uint8_t d);
void USART1_SendString(char *str);
uint8_t USART1_ReceiveChar();


#endif /* UART_H */