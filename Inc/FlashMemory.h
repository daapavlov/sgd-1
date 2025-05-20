/*
 * FlashMemory.h
 *
 *  Created on: 24 мар. 2025 г.
 *      Author: HP
 */
#include "stm32f0xx.h"

#ifndef FLASHMEMORY_H_
#define FLASHMEMORY_H_



#define FkashKey1	0x45670123
#define FkashKey2	0xCDEF89AB


void FlashUnlock();
void FlashLock();
void EraseFlashPage(uint32_t address);
void WriteToFleshMemory(uint32_t Address, uint8_t *Data, uint32_t LengthData);
void ReadToFleshMemory(uint32_t Address, uint8_t *Data, uint32_t LengthData);

#endif /* FLASHMEMORY_H_ */
