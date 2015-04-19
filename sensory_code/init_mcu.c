#include "STM32vldiscovery.h"
#include "init_mcu.h"
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define BufferLenght       5

__IO uint16_t ADC1ConvertedValue[BufferLenght];
static char channels[6]={											//Pins on board
                        ADC_Channel_10,     	//PC0
                        ADC_Channel_11,				//PC1
                        ADC_Channel_12,				//PC2
                        ADC_Channel_13,				//PC3
                        ADC_Channel_2,				//PA2
                        ADC_Channel_3 				//PA3
};

/* ------------------------- function separator -----------------------------*/
void DMA_init(void)
{
	
	DMA_InitTypeDef   DMA_InitStructure;
	/* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* DMA1 channel1 configuration ---------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC1ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferLenght;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
	
}

/* ------------------------- function separator -----------------------------*/
void Timer_init_4(void)			//milisecond timer
{
		TIM_TimeBaseInitTypeDef timerInitStructure;
 
	  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM4EN, ENABLE);
    timerInitStructure.TIM_Prescaler = 24000 - 1;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 100;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
		
    TIM_TimeBaseInit(TIM4, &timerInitStructure);
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

/* ------------------------- function separator -----------------------------*/
void Timer_init_2(void)			//microsecond timer
{
		TIM_TimeBaseInitTypeDef timerInitStructure;
 
	  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN, ENABLE);
    timerInitStructure.TIM_Prescaler = 1800 - 1;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 0xFFFF;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
		
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
		//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		//TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

/* ------------------------- function separator -----------------------------*/
void ADC_init(void)
{

	ADC_InitTypeDef   ADC_InitStructure;
	/* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = BufferLenght;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel11, channel14, channel16 and channel17 configurations */ 
  ADC_RegularChannelConfig(ADC1, channels[0], 1, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, channels[1], 2, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, channels[2], 3, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, channels[3], 4, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, channels[4], 5, ADC_SampleTime_7Cycles5);
  /*Treba nam ln2(Rul*Cul)=ln2*5k4*4,4pF=16,632ns (minimum)
  Dobili smo 7,5*1/12=0,625us=625ns. Ovo je tabela 42, strana 68. SAmpling time.
  Znaci da treba da radi!*/
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Enable TempSensor and Vrefint channels: channel16 and Channel17 */
//  ADC_TempSensorVrefintCmd(ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
  /* Test on Channel 1 DMA1_FLAG_TC flag */
  while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
  
  /* Clear Channel 1 DMA1_FLAG_TC flag */
  DMA_ClearFlag(DMA1_FLAG_TC1);	
}

/* ------------------------- function separator -----------------------------*/
void USART_init( uint32_t speed ) 
{
	USART_InitTypeDef   USART_InitStructure;
	GPIO_InitTypeDef 		GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	USART_Cmd( USART1, DISABLE);
	USART_DeInit( USART1 );
	USART_InitStructure.USART_BaudRate = speed;			
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		
	USART_Init( USART1, &USART_InitStructure );
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);

	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 1);
	
	USART_Cmd( USART1, ENABLE );
	  
	//Pins, UART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                 //Tx
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                //Rx
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                //CTS
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;                //RTS
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
/* ------------------------- function separator -----------------------------*/
void RCC_Configuration(void)
{
	
	ErrorStatus	 HSEStartUpStatus;
	
    /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
		/*000 Zero wait state, if 0  MHz < SYSCLK <= 24 MHz                                         
    001 One wait state, if  24 MHz < SYSCLK <= 48 MHz                                         
    010 Two wait states, if 48 MHz < SYSCLK <= 72 MHz */
    FLASH_SetLatency(FLASH_Latency_2);
  
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */  
    RCC_PCLK2Config(RCC_HCLK_Div1); //Ovde su ADC i SPI1 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);     //USART3, mogu da menjam brzinu vrlo lako! Ali kad je promenim sjebem nesto golemo kod SPI2!

    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div2); //Nikola: AD clock je isti kao SYSCLK/2=12MHz
  
#ifndef STM32F10X_CL  
    /* PLLCLK = 8MHz * 3 = 24 MHz */
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_3); //Maximum je 24MHz

#else
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {}

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 7 = 56 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_7);
#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  
}

/* ------------------------- function separator -----------------------------*/
void PWM_init(void)
{
	TIM_TimeBaseInitTypeDef timerInitStructure;
	TIM_OCInitTypeDef outputChannelInit;
	GPIO_InitTypeDef GPIO_InitStructure;
	
//pin PC8 i PC9
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );// Setup Blue LED on STM32-Discovery Board to use PWM. 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // Alt Function - Push Pull 
	GPIO_Init( GPIOC, &GPIO_InitStructure ); 
	GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );        // Map TIM3_CH3 to GPIOC.Pin8 
	
//timer t3	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	timerInitStructure.TIM_Prescaler = 24 - 1;			//mikrosekunde
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 25;						//Treba nam 40KHz. 
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;				//N puta ponoviti overflow pre nego sto se generise prekid
	TIM_TimeBaseInit(TIM3, &timerInitStructure);

/* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM3, ENABLE);
	
//pwm 
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 12;
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM3, &outputChannelInit);								//led 3
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM3, &outputChannelInit);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);			//led 4

}

void Capture_init(void)
{

}	

/* ------------------------- function separator -----------------------------*/
void GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the device
     immunity against EMI/EMC *************************************************/
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, DISABLE); 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  		//mora da se dozvoli clk
	
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE ); //budi pazljiv jer SWJ mora da radi ako hoces da programiras i debagujes
	
	//A15 je hardverski reset, mora da se remapira
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
