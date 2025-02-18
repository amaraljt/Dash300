#include "main.h"
#include "uart.h"
#include <stdio.h>

int main(){
    GPIOA_Clock_Init();
    USART1_Clock_Init();

    GPIOA_Init();
    
    USART1_Init();
    
    //USART1_SendString("to be lame in the eyes of somebody i would never want to be like is nothing but an absolute success");
    // USART1_SendData(0x55);
    // Delay_ms(1000000);

    //uint8_t data = USART1_ReceiveData();

    while(1){
        // if(data == 0x55){
        //     Delay_ms(1000000);
        //     GPIO_Set_Pin(); // blink LED
        //     Delay_ms(1000000);
        //     GPIO_Reset_Pin();
        // }
        USART1_SendData(0x55);
        Delay_ms(1000000);
    }

    return 0;
}