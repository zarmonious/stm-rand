/*
  ******************************************************************************
  MIKROCOMPUTER LABOR - HOME-STATION

Patapava Lalita 01347083

  In diesem Projekt gilt:
  *=============================================================================
  *        SYSCLK(Hz)                             | 64000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 2
  *=============================================================================
  ******************************************************************************
*/
#include "stm32f30x_conf.h"
#include "display.h"
#include "random.h"

int main(void)
{
	Display_Init();
	Random_Init();
	
  while(1)
  {
	}
}