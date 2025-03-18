#ifndef ADC_H_
#define ADC_H_

#include "stm32f0xx_hal.h"

void ADC_Init();
void ADC_Start();
uint16_t ADC_Read();

#endif /* ADC_H_ */
