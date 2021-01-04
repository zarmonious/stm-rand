#include "stm32f30x_conf.h"

#ifndef __DISPLAY_H
#define __DISPLAY_H

#define DISPLAY_SPI														SPI1
#define DISPLAY_SPI_CLK												RCC_APB2Periph_SPI1

#define DISPLAY_SPI_SCK_PIN                   GPIO_Pin_5                  // PA5
#define DISPLAY_SPI_SCK_GPIO_PORT             GPIOA
#define DISPLAY_SPI_SCK_GPIO_CLK              RCC_AHBPeriph_GPIOA
#define DISPLAY_SPI_SCK_SOURCE                GPIO_PinSource5
#define DISPLAY_SPI_SCK_AF                    GPIO_AF_5										// SPI1_SCK

#define DISPLAY_SPI_MOSI_PIN                  GPIO_Pin_7                  // PA7
#define DISPLAY_SPI_MOSI_GPIO_PORT            GPIOA
#define DISPLAY_SPI_MOSI_GPIO_CLK             RCC_AHBPeriph_GPIOA
#define DISPLAY_SPI_MOSI_SOURCE               GPIO_PinSource7
#define DISPLAY_SPI_MOSI_AF                   GPIO_AF_5 									// SPI1_MOSI


#define DISPLAY_DC_PIN												GPIO_Pin_6									// PA6
#define DISPLAY_DC_GPIO_PORT									GPIOA
#define DISPLAY_DC_GPIO_CLK										RCC_AHBPeriph_GPIOA

#define DISPLAY_SCE_PIN										    GPIO_Pin_7									// PC7
#define DISPLAY_SCE_GPIO_PORT									GPIOC
#define DISPLAY_SCE_GPIO_CLK									RCC_AHBPeriph_GPIOC

#define DISPLAY_RES_PIN										  	GPIO_Pin_6									// PB6
#define DISPLAY_RES_GPIO_PORT									GPIOB
#define DISPLAY_RES_GPIO_CLK									RCC_AHBPeriph_GPIOB

#define DISPLAY_BIAS_DEFAULT 0x04

#define DISPLAY_CONTRAST_DEFAULT 0x28

#define DISPLAY_FUNCTIONSET 0x20
#define DISPLAY_EXTENDEDINSTRUCTION 0x21
#define DISPLAY_SETBIAS 0x10
#define DISPLAY_SETVOP 0x80
#define DISPLAY_DISPLAYCONTROL 0x8

#define DISPLAY_DISPLAYBLANK 0x0
#define DISPLAY_DISPLAYNORMAL 0x4
#define DISPLAY_DISPLAYALLON 0x1
#define DISPLAY_DISPLAYINVERTED 0x5

#define DISPLAY_Y_ADDRESS 0x40
#define DISPLAY_X_ADDRESS 0x80


void Display_Init(void);

void Spi_WriteByte(uint8_t Data);

void Display_Send_Command(uint8_t Command);
void Display_Send_Data(uint8_t Data); 

void Display_Set_Bias(uint8_t Bias);
void Display_Set_Contrast(uint8_t Contrast);

void Display_Clear(void);

void Display_Number(uint16_t number);

void Display_Font(uint8_t index);

#endif /* __DISPLAY_H */
