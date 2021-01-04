#include "stm32f30x_conf.h"
#include "display.h"
#include "random.h"
#include <stdio.h>
#include <string.h>

const uint8_t digit_font[12][6] = 
{
	{0x7E, 0xFF, 0xC3, 0xFF, 0x7E, 0x00}, // 0
	{0x00, 0x02, 0xFF, 0xFF, 0x00, 0x00}, // 1
  {0xF3, 0xF3, 0xDB, 0xDF, 0xCE, 0x00}, // 2
  {0xC3, 0xC3, 0xDB, 0xFF, 0x7E, 0x00}, // 3
  {0x1F, 0x1F, 0x18, 0xFF, 0xFF, 0x00}, // 4
  {0xCF, 0xCF, 0xDB, 0xFB, 0x73, 0x00}, // 5
  {0x7E, 0xFF, 0xDB, 0xFB, 0x72, 0x00}, // 6
  {0x03, 0xF3, 0xFB, 0x0F, 0x07, 0x00}, // 7
  {0xFF, 0xFF, 0xDB, 0xFF, 0xFF, 0x00}, // 8
  {0x4E, 0xDF, 0xDB, 0xFF, 0x7E, 0x00}, // 9
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // space
	{0x00, 0x00, 0xDF, 0xDF, 0x00, 0x00}  // !
};

void Display_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;

	
	/* Periph clock enable */
  RCC_AHBPeriphClockCmd( DISPLAY_SPI_MOSI_GPIO_CLK | DISPLAY_SPI_SCK_GPIO_CLK 
															| DISPLAY_RES_GPIO_CLK | DISPLAY_SCE_GPIO_CLK | DISPLAY_DC_GPIO_CLK, ENABLE);

  /* Periph clock enable */
  RCC_APB2PeriphClockCmd(DISPLAY_SPI_CLK, ENABLE); 

  /* Configure SCK */
  GPIO_InitStructure.GPIO_Pin = DISPLAY_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(DISPLAY_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* Configure MOSI */
  GPIO_InitStructure.GPIO_Pin = DISPLAY_SPI_MOSI_PIN;
  GPIO_Init(DISPLAY_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* Configure DC */
  GPIO_InitStructure.GPIO_Pin = DISPLAY_DC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(DISPLAY_DC_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure SCE */
  GPIO_InitStructure.GPIO_Pin = DISPLAY_SCE_PIN;
  GPIO_Init(DISPLAY_SCE_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure RES */
  GPIO_InitStructure.GPIO_Pin = DISPLAY_RES_PIN;
  GPIO_Init(DISPLAY_RES_GPIO_PORT, &GPIO_InitStructure);

  /* Connect PXx to SD_SPI_SCK */
  GPIO_PinAFConfig(DISPLAY_SPI_SCK_GPIO_PORT, DISPLAY_SPI_SCK_SOURCE, DISPLAY_SPI_SCK_AF);

  /* Connect PXx to SD_SPI_MOSI */
  GPIO_PinAFConfig(DISPLAY_SPI_MOSI_GPIO_PORT, DISPLAY_SPI_MOSI_SOURCE, DISPLAY_SPI_MOSI_AF);  
  
  /* SPI Config */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;

  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(DISPLAY_SPI, &SPI_InitStructure);
  
  SPI_RxFIFOThresholdConfig(DISPLAY_SPI, SPI_RxFIFOThreshold_QF);
  
  /* SPI enable */
  SPI_Cmd(DISPLAY_SPI, ENABLE);

  // toggle RST low to reset
	GPIO_ResetBits(DISPLAY_RES_GPIO_PORT, DISPLAY_RES_PIN);				// RES low to start reset
  GPIO_SetBits(DISPLAY_RES_GPIO_PORT, DISPLAY_RES_PIN);				  // RES high to stop reset

  Display_Set_Bias(DISPLAY_BIAS_DEFAULT);
  Display_Set_Contrast(DISPLAY_CONTRAST_DEFAULT);

  // normal mode
  Display_Send_Command(DISPLAY_FUNCTIONSET);

  // Set display to normal
  Display_Send_Command(DISPLAY_DISPLAYCONTROL | DISPLAY_DISPLAYNORMAL);
	
	// Clear display
	Display_Clear();
}

void Spi_WriteByte(uint8_t Data)					// sends one byte with SPI
{
  /*!< Wait until the transmit buffer is empty */
  while(SPI_I2S_GetFlagStatus(DISPLAY_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
  }
  
  /*!< Send the byte */
  SPI_SendData8(DISPLAY_SPI, Data);
	
	/*!< Wait to receive a byte*/
  while(SPI_I2S_GetFlagStatus(DISPLAY_SPI, SPI_I2S_FLAG_RXNE) == RESET) 		// RXNE = 1 when frame transaction is finished
  {
  }
}

void Display_Send_Command(uint8_t Command) 
{
  GPIO_ResetBits(DISPLAY_DC_GPIO_PORT, DISPLAY_DC_PIN);			// GPIO is 0 -> ready to receive a command 
  GPIO_ResetBits(DISPLAY_SCE_GPIO_PORT, DISPLAY_SCE_PIN);		// SCE is 0 -> SPI ready to write 
  Spi_WriteByte(Command);
  GPIO_SetBits(DISPLAY_SCE_GPIO_PORT, DISPLAY_SCE_PIN);			// SCE is 1 -> SPI done 
}

void Display_Send_Data(uint8_t Data) 
{
  GPIO_SetBits(DISPLAY_DC_GPIO_PORT, DISPLAY_DC_PIN);			  // GPIO is 0 -> ready to receive a command 
  GPIO_ResetBits(DISPLAY_SCE_GPIO_PORT, DISPLAY_SCE_PIN);		// SCE is 0 -> SPI ready to write 
  Spi_WriteByte(Data);
  GPIO_SetBits(DISPLAY_SCE_GPIO_PORT, DISPLAY_SCE_PIN);			// SCE is 1 -> SPI done 
}

void Display_Set_Bias(uint8_t Bias)
{
  if (Bias > 7) 
	{
    Bias = 7;
  }

  Display_Send_Command(DISPLAY_EXTENDEDINSTRUCTION);
  Display_Send_Command(DISPLAY_SETBIAS | Bias);
  Display_Send_Command(DISPLAY_FUNCTIONSET);
}

void Display_Set_Contrast(uint8_t Contrast)
{
  if (Contrast > 127) 
	{
    Contrast = 127;
  }

  Display_Send_Command(DISPLAY_EXTENDEDINSTRUCTION);
  Display_Send_Command(DISPLAY_SETVOP | Contrast);
  Display_Send_Command(DISPLAY_FUNCTIONSET);
}

void Display_Clear(void)
{
	Display_Send_Command(DISPLAY_X_ADDRESS | 0x00);
	Display_Send_Command(DISPLAY_Y_ADDRESS | 0x00);

	for (uint8_t y_addr = 0; y_addr < 6; y_addr++)					// Alle Zeilen durchgehen
	{
		for (uint8_t x_addr = 0; x_addr < 84; x_addr++)				// Alle Spalten der Zeile durchgehen
		{
			Display_Send_Data(0x00000000);
		}
	}
	
	Display_Send_Command(DISPLAY_X_ADDRESS | 0x00);
	Display_Send_Command(DISPLAY_Y_ADDRESS | 0x00);
}

void Display_Number(uint16_t number)
{
	char string_number[6]; 
	
	sprintf( string_number, "%d" , number);
	
	for ( uint8_t i = 0; i < strlen(string_number); i++ )
	{
		uint8_t digit;

		switch(string_number[i]) 
		{
			case '0': digit = 0; 
			break;
			case '1': digit = 1; 
			break;
			case '2': digit = 2; 
			break;
			case '3': digit = 3; 
			break;
			case '4': digit = 4; 
			break;
			case '5': digit = 5; 
			break;
			case '6': digit = 6; 
			break;
			case '7': digit = 7; 
			break;
			case '8': digit = 8; 
			break;
			case '9': digit = 9; 
			break;
			default: digit = 10;
			break;
		}
		
		Display_Font(digit);
	}
}

void Display_Font(uint8_t index)
{
		for ( uint8_t j = 0; j < 6; j++ )
		{
			Display_Send_Data( digit_font[index][j] );
		}
}

