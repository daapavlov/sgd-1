#include <TimerConf.h>


void InitTIM3() //ИНИЦИАЛИЗАЦИЯ ТАЙМЕРА ДЛЯ измерений напряжения
{
	 // Включение тактирования TIM3
	    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	    // Коэффициент предделителя таймера - 48 (частота тактирования - 10 КГц)
	    TIM3->PSC = 4800 - 1;
	    // Значение регистра автоматической перезагрузки (длительность импульса - 1с)
	    TIM3->ARR = 10000-1;
	    // Обработчик прерываний от TIM3
	    NVIC_EnableIRQ(TIM3_IRQn);
	    // Разрешение прерывания от TIM3
	    TIM3->DIER |= TIM_DIER_UIE;
	    // Включение TIM3
	    TIM3->CR1 |= TIM_CR1_CEN;
}

void InitTIM17() //ИНИЦИАЛИЗАЦИЯ ТАЙМЕРА ДЛЯ RCLK
{
	 // Включение тактирования TIM17
	    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	    // Коэффициент предделителя таймера - 48 (частота тактирования - 1 МГц)
	    TIM17->PSC = 48 - 1;
	    // Значение регистра автоматической перезагрузки (длительность импульса - 10мкс)
	    TIM17->ARR = 10;
	    // Одноимпульсный режим - разрешить
	    TIM17->CR1 |= TIM_CR1_OPM;
	    // Включение TIM17
	    TIM17->CR1 |= TIM_CR1_CEN;
	    // Обработчик прерываний от TIM17
	    NVIC_EnableIRQ(TIM17_IRQn);
	    // Утановить приоритет прерываний 1
//	    NVIC_SetPriority(TIM17_IRQn, 14);
	    // Разрешение прерывания от TIM17
	    TIM17->DIER |= TIM_DIER_UIE;
}

void InitTIM14()
{
	// Включение тактирования TIM14
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->CR1 = 0; //СЧЕТЧИК ИСПОЛЬЗУЕТСЯ КАК ПОВЫШАЮЩИЙ and ВЫРАВНИВАНИЕ ПО КРАЯМ

	// Коэффициент предделителя таймера - 48 (частота тактирования - 1 MГц)
	TIM14->PSC = 48 - 1;
	// Значение регистра автоматической перезагрузки (длительность импульса - 1 мс)
	TIM14->ARR = 1000;
	// Одноимпульсный режим - разрешить
//	TIM14->CR1 |= TIM_CR1_OPM;
	// Включение TIM17
	//////////////////
	// Обработчик прерываний от TIM14
	NVIC_EnableIRQ(TIM14_IRQn);
	// Утановить приоритет прерываний 1
//	NVIC_SetPriority(TIM17_IRQn, 14);
	// Разрешение прерывания от TIM17
	TIM14->DIER |= TIM_DIER_UIE;


}

void InitTIM15()
{
	// Включение тактирования TIM14
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	TIM15->CR1 = 0; //СЧЕТЧИК ИСПОЛЬЗУЕТСЯ КАК ПОВЫШАЮЩИЙ and ВЫРАВНИВАНИЕ ПО КРАЯМ

	// Коэффициент предделителя таймера - 48 (частота тактирования - 1 kГц)
	TIM15->PSC = 480 - 1;
	// Значение регистра автоматической перезагрузки (длительность импульса - 500 мс)
	TIM15->ARR = 50000;
	// Обработчик прерываний от TIM15
	NVIC_EnableIRQ(TIM15_IRQn);

	// Разрешение прерывания от TIM15
	TIM15->DIER |= TIM_DIER_UIE;
}
