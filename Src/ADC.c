#include <ADC.h>
void ADC_Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;//Тактирование АЦП

	GPIOA->MODER |= GPIO_MODER_MODER4;//Режим PA4 аналоговый вход

    ADC1->CR |= ADC_CR_ADCAL; 				//Запуск калибровки АЦП
    while ((ADC1->CR & ADC_CR_ADCAL)); 		//Ожидаем окончания калибровки
    ADC1->SMPR |= 0b000<<ADC_SMPR_SMP_Pos; //Задаем длительность выборки
    ADC1->CR |= ADC_CR_ADEN;				//Включаем АЦП
//    while (!(ADC1->ISR & ADC_ISR_ADRDY)); 	//ждем пока АЦП включиться
    ADC1->CHSELR = ADC_CHSELR_CHSEL4; 		//Задаем номер канала (выбран ADC4)
    ADC1->CFGR1 &= ~ADC_CFGR1_RES; 			//устанавливаем разрешение 12 бит
}
void ADC_Start()
{
	/*if(ADC1->CR & ADC_CR_ADEN)
	{
		ADC1->CR |= ADC_CR_ADSTART; //Запуск преобразований
	}*/

}
uint16_t ADC_Read()
{
	if(ADC1->CR & ADC_CR_ADEN)
	{
		ADC1->CR |= ADC_CR_ADSTART; //Запуск преобразований
	}
    while ((ADC1->ISR & ADC_ISR_EOC)==0)
    {

    }
    return ADC1->DR;              // возврат преоброазованного значения;
}
