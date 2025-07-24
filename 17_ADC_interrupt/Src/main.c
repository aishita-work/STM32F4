
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "adc.h"
#include "uart.h"

uint32_t sensor_value;

int main(void)
{
	uart2_tx_init();
	pa1_adc_interrupt_init();//initializing the ADC
	start_conversion();//start conversion
	while(1){
		sensor_value= adc_read();
		printf("Sensor Value : %d\n\r",(int)sensor_value);
	}
}

/*We need to implement the interrupt request handler. When we check the vector table, we realize the name for the ADC interrupt request handler is ADC_IRQhandler and this is supposed to be a void void function.
 like this, open and close. So the first thing we want to do is, we have to check if the end of conversion flag is raised. So we say check for EOC in status register, and then once that happens, we would clear it.*/

