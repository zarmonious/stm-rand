#include "stm32f30x_conf.h"
#include "random.h"
#include "display.h"
#include <math.h>

uint16_t random_value = 0;

void Random_Init(void)
{
    // 1)  USART
    // 2a) Timer
    // 3)  ADC
    // 2b) Interrupt Channel für TIM2
    // 4)  Interrupt Handler

    GPIO_InitTypeDef         GPIO_InitStructure;
    USART_InitTypeDef        USART_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure; 


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);		// Takt freigeben
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
	  GPIO_StructInit(&GPIO_InitStructure);									// mit Default-Werten befüllen
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;								// USART
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

		GPIO_StructInit(&GPIO_InitStructure);                 // mit Default-Werten befüllen
		/* Configure as analog input */	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* Configure as input */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);


// -------------------------- Schritt 1: USART ---------------------------------

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	// Takt freigeben

    USART_InitStructure.USART_BaudRate=9600;                                      // Baudrate = 9600
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;                     // Datenbits = 8 ("Wortlänge")
    USART_InitStructure.USART_StopBits=USART_StopBits_1;                          // Stopbits = 1
    USART_InitStructure.USART_Parity=USART_Parity_No;                             // kein Paritybit
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; // keine Hardware Flow Control
    USART_InitStructure.USART_Mode=USART_Mode_Tx;                                 // Dieses Mal aktivieren wir nur TX
    USART_Init(USART2, &USART_InitStructure);
	
    USART_Cmd(USART2, ENABLE);

// -------------------------- Schritt 2a: Timer --------------------------------		
    // Wir starten den Timer noch nicht und aktivieren auch den zugehörigen Interrupt Channel vorerst nicht
    // Warum? Weil wir Timer 2 gleich bei der ADC Konfiguration noch benötigen werden und ihn erst danach "normal" starten wollen
	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE );
	
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Prescaler=7999;         // 8 MHz Takt dividiert durch 8000 -> 1kHz Zählfrequenz                         
    TIM_TimeBaseInitStructure.TIM_Period= 9;              // Von 0 bis 99 zählen -> 0.01 Sekunden
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
		
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		
    // Timer noch NICHT starten

    // -------------------------- Schritt 3: ADC ------------------------------------	
    
		uint32_t calibration_value = 0;
		
    ADC_CommonInitTypeDef    ADC_CommonInitStructure;	    // common Struktur: Parameter für beide ADCS (1,2)
    ADC_InitTypeDef          ADC_InitStructure;					  // Init-Struktur: Parameter für einen ADC
		
		ADC_DeInit(ADC1);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);		// Takt freigeben // ADC 1,2 beide am AHB
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_OFF);								// nicht PLL -> kein "ADC12_CK" Takt ( weil schon AHB genommen)
		
    ADC_CommonStructInit(&ADC_CommonInitStructure); 		  					// mit Default-Werten befüllen
    ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv1;		// AHB Bus als Taktquelle ausgewählt
    ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
		
    ADC_StructInit(&ADC_InitStructure);																							// mit Default-Werten befüllen	
    ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;			// durchgehend messen
    ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Enable;										  // alten Messwert sofort überschreiben	
		ADC_InitStructure.ADC_NbrOfRegChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);


		ADC_RegularChannelSequencerLengthConfig(ADC1, 1);		// Anzahl an Channels in "Channel Sequence" Liste
    ADC_VoltageRegulatorCmd(ADC1, ENABLE);							// Um ADC staretn, "Voltage Regulator" aktivieren
		
    // min 10us warten,  bevor wir den ADC kalibrieren oder starten können
    TIM_Cmd(TIM2, ENABLE);
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);					// Update Interrupt Flag zurücksetzen
		
    while(TIM_GetFlagStatus(TIM2,TIM_IT_Update)==0){};	// warten bis Flag gesetzt wird
    TIM_Cmd(TIM2, DISABLE);
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);					// Update Interrupt Flag zurücksetzen
    NVIC_ClearPendingIRQ(TIM2_IRQn);										// Interrupt Channel Flag zurücksetzen
		
    ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
    ADC_StartCalibration(ADC1);
  
    while(ADC_GetCalibrationStatus(ADC1) != RESET );		// warten, bis die Kalibrierung fertig ist
    calibration_value = ADC_GetCalibrationValue(ADC1);	// Kalibierungswert auslesen damit Zeit vergeht (soll 4 Takzyklen bis hin zu dem ADC-Aktivierungs-Befehl)
    
    ADC_Cmd(ADC1, ENABLE);
		
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));			// Flag ob  Startvorgang abgeschlossen ist
			
    // ADC_StartConversion(ADC1);   										// Messvorgang (Conversion) starten, damit ADC beginnt Channels zu messen


// ------ Schritt 2b: TIM2 Interrupt Channel aktivieren und TIM2 starten --------

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
    NVIC_Init(&NVIC_InitStructure);
		
    TIM_Cmd(TIM2, ENABLE);
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}
// ------------------- Schritt 4: Interrupt Handler ------------------------------	

		// alle 0.01 Sekunden aufgerufen und aktuellen ADC Wert auslesen

void TIM2_IRQHandler()
{
	  static uint8_t counter = 0;
	  uint16_t random_adc_value = 0;
	  int16_t x, y;
	  float angle;
	  uint8_t b_button = 1;
		uint8_t d_button = 1;
	  static uint8_t end = 0;

	  if (counter == 0)
		{
			random_adc_value = Adc_Read_Channel(ADC_Channel_18);
			random_value = ((CRC_CalcCRC16bits(random_adc_value) & 0xFFFF) / 182); // 0xFFFF schneidet letzte 16 digits ab 
		}
		++counter;
		counter = (counter % 500);  // bei 500 counter wird zu 0 (5 Sekunden)

		x = (int16_t) Adc_Read_Channel(ADC_Channel_1) - 2047 - 62;							 // Calibration
		y = (int16_t) Adc_Read_Channel(ADC_Channel_2) - 2047;
	
	  if (x > 100 || x < -100 || y > 100 || y < -100)
		{
			if (x != 0)
			{
					angle = atan(fabs((float)y)/fabs((float)x));
					if (x < 0)
					{
							if (y >= 0)
							{
								angle = (float)M_PI - angle;
							}
							else
							{
								angle = (float)M_PI + angle;
							}
					}
					else if (y < 0)
					{
						angle = (2. * M_PI) - angle;
					}
			}
			else
			{
				 if (y >= 0)
				 {
						angle = M_PI / 2;
				 }
				 else
				 {
						angle = -M_PI / 2;
				 }
			}
			
			angle = (360. / (2. * M_PI)) * angle;     // radiant to grad
		}
		else
		{
			angle = 0;
		}
		
		if (end == 0)
		{
			b_button = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
			
			if (b_button == 0)
			{
				if (angle >= (random_value - 3) && angle <= (random_value + 3))
				{
						Display_Clear();
						Display_Font(11);
						Display_Font(11);
						Display_Font(11);
					  Display_Font(10);
					  Display_Number(counter * 10);
						
						end = 1;
				}
			}
			else
			{
				if (counter % 10 == 0)
				{
					Display_Clear();
					Display_Number(random_value);																			// Random Number
					Display_Font(10);
					Display_Number((uint16_t)angle);																	// Winkel
					Display_Font(10);
					Display_Number((uint16_t)fabs((float)random_value - angle));			// Differenz
				}
			}
		}
		else
		{
			d_button = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
			
			if (d_button == 0)
			{
				end = 0;
			}
		}

    //while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET){};
    //USART_SendData(USART2, angle);
			
		//while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=SET){};
    //USART_SendData(USART2, y);
   
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	 // Pending Bit zurücksetzen
}


uint16_t Adc_Read_Channel(uint8_t channel)
{
    uint16_t adc_value = 0;
    
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_7Cycles5 ); // Set the conversion period of ADC1 channel ch to 7.5 sampling periods, sampling order is 1
 
    ADC_StartConversion(ADC1);												//Enable software trigger
    
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)){};	// Waiting for conversion to complete
        
    adc_value = ADC_GetConversionValue(ADC1); 				// Get the converted value
  
    return adc_value;
}
