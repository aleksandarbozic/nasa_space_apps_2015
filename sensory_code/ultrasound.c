#include <stdio.h>      /* printf */
#include <stdlib.h>     /* abs */
#include "init_mcu.h"

const float SPEED_OF_SOUND_20C = 0.0003432; 		//meters per micro-second
extern uint32_t musTicks;
/* 
* ADC is 12 bit, meaning resolution is 3.3V / 4096 = 0.806 V 
* We will measure on oscilloscope the value for threshold.
*/
#define THRESHOLD 100
/* ------------------------- function separator -----------------------------*/
void distance_init( void )
{
	
  RCC_Configuration();                  //Clock initialization
	
	DMA_init();														//DMA, for fast A/D conversion
  
  ADC_init();														//12 bit ADC
	
	PWM_init();														//PWM, 40 KHz
	
	USART_init(115200);										//UART , for communication with PC
	
	Timer_init_2();												//precise time base for microseconds
	
	SysTick_Config(SystemCoreClock / 1000);      /* Configure SysTick to generate an interrupt every microsecond */
}

/* ------------------------- function separator -----------------------------*/
void start_PWM(void){
	TIM_Cmd(TIM3, ENABLE);
}

/* ------------------------- function separator -----------------------------*/
void stop_PWM(void){
	TIM_Cmd(TIM3, DISABLE);
}

/* ------------------------- function separator -----------------------------*/
void distance_scan(void)													//ToA ultrasonic measure
{
	uint32_t 		tPeak = 0;
	uint32_t 		i;
	uint32_t 		vCurr, vOld;
	float 			distance = 0;
	char 				str[30];
	
	musTicks = 0;
	while( musTicks != 100);
	vOld = ADC_Read(1);														
	
	/* sending short pulse */
	start_PWM();																				// 40KHz means period is 25us
	TIM2->CNT = 0;
	TIM_Cmd( TIM2, ENABLE );
  while( TIM2->CNT < 300 );														//Speed of sound is around 340 m/s. For 1 ms object has to be on 17 cm distance. 1 meter takes 6ms
  stop_PWM();
 
	/* reading distance */
  while( TIM2->CNT < 700 );															//Delay for waiting return wave

  for (i = 0; i < 2048; i++) {												//A/D conversion is around 3us , measured with timer, meaning we will catch 2048*3 = 6 ms, it is enough for obstacles closer than 1 meter
    vCurr = ADC_Read(1);
    if ( abs(vCurr - vOld) > THRESHOLD) {
      tPeak = TIM2->CNT;
			distance = (float) tPeak * (float)SPEED_OF_SOUND_20C / 2.0;
      break;
    }
  }
	//sprintf( str, "Udaljenost je %f", distance ); 
 // for( i = 0; str[i]!=0; i++ )USART_send(str[i]);
}/*end of distance()*/
