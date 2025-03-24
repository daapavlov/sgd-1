#include <FlashMemory.h>


void FlashUnlock()//функция разблокировки
{
    FLASH->KEYR = FkashKey1;
    FLASH->KEYR = FkashKey2;
}

void FlashLock()//функция блокировки
{
    FLASH->CR |= FLASH_CR_LOCK;
}

void WriteToFleshMemory(uint32_t Adress, uint8_t *Data, uint32_t LengthData)
{
	FlashUnlock();//Разблокируем
	FLASH->CR |= FLASH_CR_PER;//Установим бит очистки
	FLASH->AR = Adress;
	FLASH->CR|= FLASH_CR_STRT;//Старт очистки
	while ((FLASH->SR & FLASH_SR_BSY));//Ожидание очистки
    FLASH->CR &= ~FLASH_CR_PER;//Сбросим бит очистки

    FLASH->CR |= FLASH_CR_PG; //Разрешаем запись в память
    while ((FLASH->SR & FLASH_SR_BSY));//Ожидаем
    int j=0;
    for(int i=0; i<LengthData*2; i+=2)
    {
    	*(__IO uint16_t*)(Adress+i) = (uint16_t)(Data[j]<<8 | Data[j+1]);
    	j++;
    }
    FLASH->SR |= FLASH_SR_EOP;//Запрещаем программирование
    FlashLock();//Заблокируем
}

void ReadToFleshMemory(uint32_t Adress, uint8_t *Data, uint32_t LengthData)
{
	int j=0;
    for(int i=0; i<LengthData*2; i+=2)
    {
    	Data[j] = *(__IO uint16_t*)(Adress+i)>>8;
    	Data[j+1] = *(__IO uint16_t*)(Adress+i);
    	j++;
    }
}
