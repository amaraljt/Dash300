#include "main.h"
#include "uart.h"
#include <stdio.h>

// void SystemClock_Config(void);

int main(void){
    GPIOA_Init();
    
    USART1_Init();
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    while(1){
        USART1_SendData(0x33);
        Delay_ms(200);
    }
}
