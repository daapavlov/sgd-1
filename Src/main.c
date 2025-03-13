/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "mb.h"
#include "mbport.h"
#include "mt_port.h"

#include <indicator_7s_sr.h>
#include <TimerConf.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BaudRateModBusRTU	9600U
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void CallbackTIM14();
void IndicationSensitivity(uint8_t Sensitivity);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*Для modBus RTU*/
#define REG_ADRESS_INPUT_START	1 //НАЧАЛЬНЫЙ АДРЕСС РЕГИСТРОВ
#define REG_INPUT_NUMBER_REGS	100 //КОЛИЧЕСВТО РЕГИСТРОВ

static USHORT usRegAdressInputStart = REG_ADRESS_INPUT_START;
static USHORT usRegAnalog[REG_INPUT_NUMBER_REGS]={0}; //Регистры аналоговых чисел

uint8_t FlagAnalogMessageFromMaster=0; //Флаг, который выставляется, когда приходит изменение в регистр от мастера


/*Для кнопок управления*/
uint8_t ClickFlag_PB8 = 0;
uint8_t ShortPressKey_PB8=0;
uint8_t LongPressKey_PB8=0;//3 сек нажатие

uint8_t ClickFlag_PB2 = 0;
uint8_t ShortPressKey_PB2=0;
uint8_t LongPressKey_PB2=0;//3 сек нажатие

uint8_t LongDoublePressKey_PB2_PB8=0;//3 сек нажатие
/*Для таймеров*/
uint32_t TimerCounterTIM14 = 0;
uint32_t TimerCounterTIM15 = 0;
/*Настройки датчика*/
uint8_t GlobalAdres=0; //Адрес датчика
char GlobalAdres_str[] = "000";
uint8_t Sensitivity = 0; //чувствительность датчика
uint8_t ModeRele = 0b000; //режим реле по умолчанию
char ModeRele_str[] = "000";
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //Пересылка структур с настройками
  /*MT_PORT_SetTimerModule(&htim16);
  MT_PORT_SetUartModule(&huart2);

  eMBErrorCode eStatus;
  eStatus = eMBInit(MB_RTU, 0x0A, 0, BaudRateModBusRTU, MB_PAR_NONE); //начальные настройки modBus
  eStatus = eMBEnable();

  if (eStatus != MB_ENOERR)
  {
    // Error handling
  }*/
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();



  /* USER CODE BEGIN 2 */
  InitTIM14();
  InitTIM15();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  if((LongPressKey_PB8))//Сработало длинное нажатие ЭТО ДЛЯ НАСТРОЙКИ АДРЕСА
	  {
		  TimerCounterTIM15=0;
//		  indicator_sgd4(SPI1, 0x00, "ПРГ", 0x00);//Процесс индикации режима настройки
		  HAL_Delay(1000);
//		  indicator_sgd4(SPI1, 0x00, GlobalAdress_str, 0x00);//Индикация текущего адреса
		  TIM15->CR1 |= TIM_CR1_CEN;

		  while(TimerCounterTIM15<=6) //Если таймер больше 3 сек, то заканчиваем настройку
		  {
			if(ShortPressKey_PB8)//короткое нжатие
			{
				if(GlobalAdres<99)
				{
					GlobalAdres++;
				}
				else
				{
					GlobalAdres = 99;
				}

				sprintf(GlobalAdres_str, "%d", GlobalAdres);
//		  indicator_sgd4(SPI1, 0x00, GlobalAdress_str, 0x00);//Индикация текущего адреса
				TimerCounterTIM15=0;
				ShortPressKey_PB8=0;
			}
			if(ShortPressKey_PB2)//короткое нжатие
			{
				if(GlobalAdres>0)
				{
					GlobalAdres--;
				}
				else
				{
					GlobalAdres = 0;
				}
				sprintf(GlobalAdres_str, "%d", GlobalAdres);
//		  indicator_sgd4(SPI1, 0x00, GlobalAdress_str, 0x00);//Индикация текущего адреса
				TimerCounterTIM15=0;
				ShortPressKey_PB2=0;
			}
		  }
		  TIM15->CR1 &= ~TIM_CR1_CEN;//выключаем таймер мигания
		  GPIOC->BSRR &= ~GPIO_BSRR_BR_13;
		  LongPressKey_PB8=0;
//		  eStatus = eMBInit(MB_RTU, GlobalAdres, 0, BaudRateModBusRTU, MB_PAR_NONE); //начальные настройки modBus


	  }
	  if((LongPressKey_PB2))//Сработало длинное нажатие ЭТО ДЛЯ НАСТРОЙКИ ЧУВСТВИТЕЛЬНОСТИ
	  {
		  TimerCounterTIM15=0;
//		  indicator_sgd4(SPI1, 0x00, "ПРГ", 0x00);//Процесс индикации режима настройки
		  HAL_Delay(1000);
//		  IndicationSensitivity(Sensitivity);//Показывает настройку чувствительности
		  TIM15->CR1 |= TIM_CR1_CEN;
		  while(TimerCounterTIM15<=6) //Если таймер больше 3 сек, то заканчиваем настройку
		  {
			  if(ShortPressKey_PB8)//короткое нжатие
			  {
				if(Sensitivity>=3)
				{
					Sensitivity = 0;
				}
				else
				{
					Sensitivity++;
				}
//				IndicationSensitivity(Sensitivity);
				TimerCounterTIM15=0;
				ShortPressKey_PB8=0;
			  }
			  if(ShortPressKey_PB2)//короткое нжатие
			  {
				if(Sensitivity<=0)
				{
					Sensitivity = 3;
				}
				else
				{
					Sensitivity--;
				}
//				IndicationSensitivity(Sensitivity);
				TimerCounterTIM15=0;
				ShortPressKey_PB2=0;
			  }
		  }
		  TIM15->CR1 &= ~TIM_CR1_CEN;//выключаем таймер мигания
		  GPIOC->BSRR &= ~GPIO_BSRR_BR_13;
		  LongPressKey_PB2=0;
	  }

	  if(LongDoublePressKey_PB2_PB8) //сработали обе кнопки в длинную ЭТО ДЛЯ НАСТРОЙКИ ЗАЛИПАНИЯ
	  {
		  TimerCounterTIM15=0;
//		  indicator_sgd4(SPI1, 0x00, "ПРГ", 0x00);//Процесс индикации режима настройки
		  HAL_Delay(1000);
		  sprintf(ModeRele_str, "%d", ModeRele);
//		  indicator_sgd4(SPI1, 0x00, GlobalAdress_str, 0x00);//Индикация текущего адреса
		  TIM15->CR1 |= TIM_CR1_CEN;
		  while(TimerCounterTIM15<=6) //Если таймер больше 3 сек, то заканчиваем настройку
		  {
			  if(ShortPressKey_PB8)//короткое нжатие
			  {
				  ModeRele = 0b111;
				  sprintf(ModeRele_str, "%d", ModeRele);
			  }
			  if(ShortPressKey_PB2)//короткое нжатие
			  {
				  ModeRele = 0b000;
				  sprintf(ModeRele_str, "%d", ModeRele);
			  }
		  }
		  TIM15->CR1 &= ~TIM_CR1_CEN;//выключаем таймер мигания
		  GPIOC->BSRR &= ~GPIO_BSRR_BR_13;
		  LongDoublePressKey_PB2_PB8=0;
	  }

  /* USER CODE BEGIN 3 */
	  eMBPoll();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM17 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 47;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 50;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = BaudRateModBusRTU;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  //DE-RE:
  GPIOA->MODER |= 0b01<<GPIO_MODER_MODER0;//OUTPUT MODE
  GPIOA->OTYPER |= 0b01<<GPIO_OTYPER_OT_0;//OPEN-DRAIN
  GPIOA->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEEDR0_Pos;//HIGH SPEED

  //LED G:
  GPIOC->MODER = 10;
  GPIOC->MODER |= 0b01<<GPIO_MODER_MODER13_Pos; //OUTPUT MODE
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_13;//PUSH-PULL
  GPIOC->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEEDR13_Pos;//HIGH SPEED
  GPIOC->PUPDR |= 0b10<<GPIO_PUPDR_PUPDR13_Pos; //PULL DOWN

  //TER:
  GPIOA->MODER |= 0b01<<GPIO_MODER_MODER12_Pos;//OUTPUT MODE
  GPIOA->OTYPER |= GPIO_OTYPER_OT_12;//OPEN-DRAIN
  GPIOA->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEEDR12_Pos;//HIGH SPEED

  //REL:
  GPIOB->MODER |= 0b01<<GPIO_MODER_MODER12_Pos;//OUTPUT MODE
  GPIOB->OTYPER |= GPIO_OTYPER_OT_12;//OPEN-DRAIN
  GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEEDR12_Pos;//HIGH SPEED

  /*EXTI PB2 и PB8*/
  GPIOB->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER8);
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;
  SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PB;
  EXTI->IMR = 0b100000100;// Настройка битов маски
  EXTI->RTSR = 0b100000100;// Срабатывание по переднему фронту
  EXTI->FTSR = 0b100000100;// Срабатывание по заднему фронту
  // Разрешение прерываний
  NVIC_EnableIRQ(EXTI2_3_IRQn);
  NVIC_EnableIRQ(EXTI4_15_IRQn);

  //RCLK
  GPIOB->MODER |= 0b01<<GPIO_MODER_MODER9_Pos;//OUTPUT MODE
  GPIOB->OTYPER |= GPIO_OTYPER_OT_9;//OPEN-DRAIN
  GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEEDR9_Pos;//HIGH SPEED
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //Обработчик прерываний EXTI
{
	if((EXTI->PR & EXTI_PR_PR2) > 0)
	{
		EXTI->PR |= EXTI_PR_PR2;
		if(!ClickFlag_PB2)
		{
			ClickFlag_PB2=1;
			TimerCounterTIM14=0;
			TIM14->CR1 |= TIM_CR1_CEN;//Включение таймер
		}
		else if(ClickFlag_PB2)
		{
			if(TimerCounterTIM14<3000)
			{
				//КОРОТКОЕ НАЖАТИЕ
				ShortPressKey_PB2=1;
			}
			TimerCounterTIM14=0;
			ClickFlag_PB2=0;
			TIM14->CR1 &= ~TIM_CR1_CEN;//Выключение таймера
		}
	}

	if((EXTI->PR & EXTI_PR_PR8) > 0)
	{
		EXTI->PR |= EXTI_PR_PR8;
		if(!ClickFlag_PB8)
		{
			ClickFlag_PB8=1;
			TimerCounterTIM14=0;
			TIM14->CR1 |= TIM_CR1_CEN;//Включение таймер
		}
		else if(ClickFlag_PB8)
		{
			if(TimerCounterTIM14<3000)
			{
				//КОРОТКОЕ НАЖАТИЕ
				ShortPressKey_PB8=1;

			}
			TimerCounterTIM14=0;
			ClickFlag_PB8=0;
			TIM14->CR1 &= ~TIM_CR1_CEN;//Выключение счетчика
		}
	}

}
void CallbackTIM14()
{
	TimerCounterTIM14++;
	if(TimerCounterTIM14>=3000 && ClickFlag_PB8==1 && ClickFlag_PB2==0)
	{
		//ДЛИННОЕ НАЖАТИ
		LongPressKey_PB8=1;
		TimerCounterTIM14=0;
		TIM14->CR1 &= ~TIM_CR1_CEN;//Выключение счетчика
	}
	else if(TimerCounterTIM14>=3000 && ClickFlag_PB2==1 && ClickFlag_PB8==0)
	{
		//ДЛИННОЕ НАЖАТИ
		LongPressKey_PB2=1;
		TimerCounterTIM14=0;
		TIM14->CR1 &= ~TIM_CR1_CEN;//Выключение счетчика
	}
	else if(TimerCounterTIM14>=3000 && ClickFlag_PB8==1 && ClickFlag_PB2==1)
	{
		LongDoublePressKey_PB2_PB8=1;
		TimerCounterTIM14=0;
		TIM14->CR1 &= ~TIM_CR1_CEN;//Выключение счетчика
	}
}

eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
  eMBErrorCode eStatus = MB_ENOERR;
  int iRegIndex;

  if ((usAddress >= REG_ADRESS_INPUT_START) &&
      (usAddress + usNRegs <= REG_ADRESS_INPUT_START + REG_INPUT_NUMBER_REGS))
  {
    iRegIndex = (int)(usAddress - usRegAdressInputStart);

    while(usNRegs > 0)
    {
        *pucRegBuffer++ = (unsigned char)(usRegAnalog[iRegIndex] >> 8);
        *pucRegBuffer++ = (unsigned char)(usRegAnalog[iRegIndex] & 0xFF);

        iRegIndex++;
        usNRegs--;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }

  return eStatus;
}



/*----------------------------------------------------------------------------*/
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                             eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex=0;
	if ((usAddress >= REG_ADRESS_INPUT_START) &&
	      (usAddress + usNRegs <= REG_ADRESS_INPUT_START + REG_INPUT_NUMBER_REGS))
	  {
		if(eMode==MB_REG_READ)
		{
			while(usNRegs > 0)
			{ /*в буфер pucRegBuffer записываются полезные данные последовательно, в зависимости от того,
				сколько регистров считывает мастер (хранится в пе6ременной usNRegs)*/
				*pucRegBuffer++ = (unsigned char)(usRegAnalog[iRegIndex] >> 8);
				*pucRegBuffer++ = (unsigned char)(usRegAnalog[iRegIndex] & 0xFF);

				iRegIndex++;
				usNRegs--;
			}
		}
		else if(eMode==MB_REG_WRITE)
		{
			FlagAnalogMessageFromMaster=1;
			/*Если выставлен флаг на запись в регистр, то pucRegBuffer хранит в себе данные о числах
			 	 записываемых в регистр
			 	 если число больше 255, то по адресу pucRegBuffer[0] записывается количественное значение*/
			usRegAnalog[usAddress-1] = pucRegBuffer[1] + 256*pucRegBuffer[0];
		}

	  }
	else
	{
		eStatus = MB_ENOREG;
	}

  return eStatus;
}



/*----------------------------------------------------------------------------*/
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
                           eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	/*if ((usAddress >= REG_ADRESS_INPUT_START) &&
      (usAddress + usNCoils <= REG_ADRESS_INPUT_START + REG_INPUT_NUMBER_REGS))
	{
		if(eMode==MB_REG_READ)
		{
			int iRegIndex=0;
			while(usNCoils > 0)
			{
				*pucRegBuffer++ = (unsigned char)usRegDiscreteFlag[iRegIndex];
				iRegIndex++;
				usNCoils--;
			}
		}
		else if(eMode==MB_REG_WRITE)
		{
			FlagDiscreteMessageFromMaster=1;
			uint8_t Numbers=0;
			Numbers=(uint8_t)usRegDiscreteFlag[(uint8_t)((usAddress-1)/8)];
			if((Numbers>>(usAddress-1)%8)&0x01)
			{
				Numbers &= ~usNCoils<<((usAddress-1)%8);
			}
			else
			{
				Numbers |= usNCoils<<((usAddress-1)%8);
			}
			usRegDiscreteFlag[(usAddress-1)/8] = Numbers;
			Numbers=0;
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}*/
	eStatus = MB_ENOREG;

  return eStatus;
}



/*----------------------------------------------------------------------------*/
eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
	eMBErrorCode eStatus = MB_ENOERR;
	/*int iRegIndex=0;
	if ((usAddress >= REG_ADRESS_INPUT_START) &&
	      (usAddress + usNDiscrete <= REG_ADRESS_INPUT_START + REG_INPUT_NUMBER_REGS))
	  {
	    while(usNDiscrete > 0)
	    {
	        *pucRegBuffer++ = (unsigned char)usRegDiscreteFlag[iRegIndex];

	        iRegIndex++;
	        usNDiscrete--;
	    }

	  }
	else
	{
		eStatus = MB_ENOREG;
	}*/
	eStatus = MB_ENOREG;
  return eStatus;
}


void IndicationSensitivity(uint8_t Sensitivity)
{
	switch (Sensitivity)
	{

	case	0:
	{
		indicator_sgd4(SPI1, 0x00, " НЧ", 0x00);
	}break;
	case	1:
	{
		indicator_sgd4(SPI1, 0x00, " СЧ", 0x00);
	}break;
	case	2:
	{
		indicator_sgd4(SPI1, 0x00, " ВЧ", 0x00);
	}break;
	}
}
/* USER CODE END 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
