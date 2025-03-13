#include <Mode_SGD-1.h>


uint8_t ModeConfiguration(uint8_t Adress)
{
	uint8_t AdressSensors=Adress;
	indicator_sgd4(SPI1, 0x00, "ПРГ", 0x00); //ВЫВОД НА ТАБЛО СООБЩЕНИЯ
	GPIOC->ODR |= GPIO_ODR_13; //ДОЛЖЕН МИГАТЬ


	return AdressSensors;
}

uint8_t ModeNorm()
{
	return 0;
}

uint8_t ModeMalfunction()
{
	return 0;
}

uint8_t ModeAnxiety()
{
	return 0;
}
