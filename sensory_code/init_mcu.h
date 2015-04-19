#include "STM32vldiscovery.h"

void RCC_Configuration(void);
void GPIO_init(void);
void DMA_init(void);
void ADC_init(void);
void Timer_init_4(void);
void Timer_init_2(void);
void USART_init(uint32_t speed);
void Delay_ms( uint32_t ms );
void PWM_init(void);
