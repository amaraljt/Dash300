#include "main.h"
#include "uart.h"
#include <stdio.h>

int main(void){
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system, GPIOA & USART1 clock */
    SystemClock_Config();
    GPIOA_Init();
    USART1_Init();

    while(1){
        ISO_Init();
    }
}

// 0x9A - 10011010
// 0x6A - 10100110
// 0x33 - 00110011 (WORKS AFTER WAITING 10 SECONDS)
