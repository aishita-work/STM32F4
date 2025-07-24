
#include "stdio.h"
#include "stdint.h"
#include "stm32f4xx.h"


#define GPIOAEN            (1U<<0)

#define USART2EN           (1U<<17)

#define PIN0               (1U<<0)  // PA0 for A (HMC241ALP4E)
#define PIN1               (1U<<1)  // PA1 for B (HMC241ALP4E)
#define PIN2               (1U<<2)  // PA2 for USART2 TX

#define SYS_FREQ           16000000
#define APB1_CLK           SYS_FREQ

#define UART_BAUDRATE      115200
#define CR1_TE            (1U<<3)
#define CR1_UE            (1U<<13)
#define SR_TXE            (1U<<7)

static void usrt_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t Baudrate);
static uint16_t compute_usrt_bd(uint32_t PeriphClk, uint32_t Baudrate);
void uart2_tx_init(void);
void uart2_write(int ch);
void delay_ms(uint32_t ms);
int __io_putchar(int ch){ //to use printf we need to implement to put character function,this takes ch character argument
	uart2_write(ch); //inside we call uart2_write function and pass the argument ch and return ch
	return ch;
}
int main(void){
	uart2_tx_init();

	// Configure PA0, PA1 as outputs for HMC241ALP4E (A, B)
	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER |= (1U<<0);   // PA0 output (MODER[1:0] = 01)
	GPIOA->MODER &= ~(1U<<1);
	GPIOA->MODER |= (1U<<2);   // PA1 output (MODER[3:2] = 01)
	GPIOA->MODER &= ~(1U<<3);

	while(1){
		printf("Hello from STM32....\n\r"); //go to realterm app.In port option correct the baudrate and click open and terminal should show hello from stm32
		// RF1: A=0, B=0
		GPIOA->ODR &= ~(PIN0 | PIN1);
		printf("RF1: A=0, B=0\n\r");
		delay_ms(1000);

		// RF2: A=0, B=1
	    GPIOA->ODR &= ~PIN0;
	    GPIOA->ODR |= PIN1;
		printf("RF2: A=0, B=1\n\r");
		delay_ms(1000);

		// RF3: A=1, B=0
		GPIOA->ODR |= PIN0;
		GPIOA->ODR &= ~PIN1;
		printf("RF3: A=1, B=0\n\r");
		delay_ms(1000);

		// RF4: A=1, B=1
		GPIOA->ODR |= (PIN0 | PIN1);
		printf("RF4: A=1, B=1\n\r");
        delay_ms(1000);
	}

}


void uart2_tx_init(void){

	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER &=~(1U<<4);
    GPIOA->MODER |=(1U<<5);
    GPIOA->AFR[0] |=(1U<<8);
	GPIOA->AFR[0] |=(1U<<9);
    GPIOA->AFR[0] |=(1U<<10);
    GPIOA->AFR[0] &=~(1U<<11);

    RCC->APB1ENR |= USART2EN;

    usrt_set_baudrate(USART2,APB1_CLK, UART_BAUDRATE);

    USART2->CR1 = CR1_TE;

    USART2->CR1 |= CR1_UE;
}

static void usrt_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t Baudrate){
	USARTx->BRR = compute_usrt_bd(PeriphClk,Baudrate);
}

static uint16_t compute_usrt_bd(uint32_t PeriphClk, uint32_t Baudrate){
	return ((PeriphClk + (Baudrate/2U))/Baudrate);

}

void uart2_write(int ch){

	while(!(USART2->SR & SR_TXE)){};

	USART2->DR = (ch & 0xFF);
}

void delay_ms(uint32_t ms) {
    // Approximate delay for 16 MHz HSI clock (~1600 cycles per ms)
    for (volatile uint32_t i = 0; i < ms * 1600; i++) {}
}


