#include "stm32f30x_conf.h"

#ifndef __RANDOM_H
#define __RANDOM_H

extern uint16_t random_value;

void Random_Init(void);

uint16_t Adc_Read_Channel(uint8_t channel);

#define M_PI 3.14159265358979323846

#endif /* __RANDOM_H */
