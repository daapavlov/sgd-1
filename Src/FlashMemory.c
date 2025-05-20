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
void EraseFlashPage(uint32_t address)
{
	while (FLASH->SR & FLASH_SR_BSY);
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = address;
	FLASH->CR |= FLASH_CR_STRT;
	while (!(FLASH->SR & FLASH_SR_EOP));
	FLASH->SR = FLASH_SR_EOP;
	FLASH->CR &= ~FLASH_CR_PER;
}
void WriteToFleshMemory(uint32_t Address, uint8_t *Data, uint32_t LengthData)
{
	FlashUnlock();//Разблокируем

	EraseFlashPage(Address);


	while (FLASH->SR & FLASH_SR_BSY);
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR |= FLASH_CR_PG;

    int j=0;
    for(size_t i = 0; i < LengthData*2; i += 2) {
        *(volatile uint16_t *)(Address + i) = (uint16_t)(((Data[j]) << 8) | Data[j + 1]);
        j++;
        FLASH->SR = FLASH_SR_EOP;
    }
    FLASH->CR &= ~(FLASH_CR_PG);
	FlashLock();//Заблокируем
}
void ReadToFleshMemory(uint32_t Address, uint8_t *Data, uint32_t LengthData)
{
	int j=0;
    for(int i=0; i<LengthData*2; i+=2)
    {
    	Data[j] = *(__IO uint16_t*)(Address+i)>>8;
    	Data[j+1] = *(__IO uint16_t*)(Address+i);
    	j++;
    }
}
