/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "mb.h"
#include "mbport.h"
#include "mt_port.h"

#include <indicator_7s_sr.h>
#include <TimerConf.h>
#include <ADC.h>
#include <FlashMemory.h>

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

#define	R120_SET	GPIOA->BSRR |= GPIO_BSRR_BS_12;
#define	R120_RESET	GPIOA->BSRR |= GPIO_BSRR_BR_12;

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);//инициализация системного тактирования
static void MX_GPIO_Init(void);//инициализация gpio
static void MX_SPI1_Init(void);//инициализация spi
static void MX_USART1_UART_Init(void);//инициализация usart1
static void MX_USART2_UART_Init(void);//инициализация usart2
static void MX_TIM16_Init(void);//инициализация таймера 16 для модбаса

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void CheckingKeyTimings();//проверка длителльности нажатия кнопок
void IndicationSensitivity(uint8_t Sensitivity, uint8_t led);//индикация режима чувствительности на лед дисплее
void KeyPress();//функция реализации алгоритмов работы меню управления
void IWDG_Init();//инициализация сторожевого таймера
void IWDG_Reset();//сброс сторожевого таймера
void Setting_Init();//функция инициализации основных настроек после сброса
void GasMeasurement();//функция измерения условной концентрации
void ModeAlarm();//функция реализации режима тревоги
void USART_send_symbol(char symbol);
void USART_send_string(char *string);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*Для modBus RTU*/
#define REG_ADRESS_INPUT_START	81 //НАЧАЛЬНЫЙ АДРЕСС РЕГИСТРОВ
#define REG_INPUT_NUMBER_REGS	45 //КОЛИЧЕСВТО РЕГИСТРОВ

static USHORT usRegAdressInputStart = REG_ADRESS_INPUT_START;
static USHORT usRegAnalog[REG_INPUT_NUMBER_REGS]={0, 0, 0}; //Регистры аналоговых чисел
/*
 *[3] - 83 - Корректировка нуля
 *[8] - 88 - Чистое значение
 *[13] - 93 - Корректировка нуля
 *[18] - 98 - Тип датчика
 *[19] - 99 - Статус датчика
 *[20] - 100 - Текущее значение концентрации сигнального газа
 *[24] - 104 - Уровень чувствительности
 *[29] - 109 - Количество срабатываний
 *[35] - 115 - Уровень порога
 *[40] - 120 - Заводской номер
 *[41] - 121 - Заводской номер
 *[42] - 122 - Заводской номер
 */

#define SENSOR_NULLC_LONG	84-REG_ADRESS_INPUT_START
#define SENSOR_CLEAR	89-REG_ADRESS_INPUT_START
#define	SENSOR_NULLC	94-REG_ADRESS_INPUT_START
#define	DEVICE_TYPE	99-REG_ADRESS_INPUT_START
#define	DEVICE_STATUS	100-REG_ADRESS_INPUT_START
#define	SENSOR	101-REG_ADRESS_INPUT_START
#define	STATUS_ADDR	105-REG_ADRESS_INPUT_START
#define	TOTAL_ALARMS_ADDR	110-REG_ADRESS_INPUT_START
#define SENSOR_IND_LEVEL	116-REG_ADRESS_INPUT_START
#define	ZN_ADDR	121-REG_ADRESS_INPUT_START
#define	ZN_ADDR1	122-REG_ADRESS_INPUT_START
#define	ZN_ADDR2	123-REG_ADRESS_INPUT_START
#define DEVICE_regime	3333;// режим
#define DEVICE_ADDR	7777;// адрес датчика

uint8_t FlagAnalogMessageFromMaster=0; //Флаг, который выставляется, когда приходит изменение в регистр от мастера

uint16_t EXTI_PRreg; //регистры PR

/*Для кнопок управления*/
uint8_t ClickFlag_PB8 = 0;
uint8_t ShortPressKey_PB8=0;
uint8_t LongPressKey_PB8=0;//3 сек нажатие

uint8_t ClickFlag_PB2 = 0;
uint8_t ShortPressKey_PB2=0;//короткое нажатие
uint8_t LongPressKey_PB2=0;//3 сек нажатие

uint8_t LongDoublePressKey_PB2_PB8=0;//3 сек нажатие

/*Для таймеров*/
uint16_t TimerCounterTIM14 = 0;
uint8_t TimerCounterTIM15 = 0;
uint8_t FlagMogan=0; //флаг моргания светодиодом
uint8_t TimerFlagTIM3 = 0;//флаг срабатывания таймера 3

/*Настройки датчика*/
uint8_t GlobalAddress=1; //Адрес датчика
uint8_t GlobalAddressFlesh=0;
uint8_t Sensitivity = 1; //чувствительность датчика
uint8_t ModeRele = 0; //режим реле по умолчанию
uint8_t Resistor120 = 0;
uint8_t FlagChangeSetting=0;
uint8_t DataSettingMemory[10] = {1};//массив записи данных в память
uint8_t StatusMode=13;
char StringIndication[] = "---";


uint16_t NH = 300;	//Низкая чувствительность
uint16_t CH = 225;	//Средняя чувствительность
uint16_t VH = 150;	//Высокая чувствительность

float VoltageInR2=0;
float Ir2=0.f; //Ток в цепи
float Use = 0.f;//Напряжение чувствительного элемента
float R2=3.0f; //3kOm
float R1=1.6f; //1.6kOm
float Rse = 0.f; //Сопротивление чувствительного элемента
uint16_t R_average = 0; //Напряжение усредненное
float S=0.f, A=31.25f, B=3.3f, C=10.f, D=0.98f, F=0.5f, G=1.06f;
uint8_t SecondsCounter = 0, SecondsCounter5 = 0;
uint8_t MinuteCounter = 0, MinuteCounter15 = 0;
uint16_t ArrayOfResistanceMeasurementsPerMinute[60];
uint16_t ArrayOfResistanceMeasurementsPerSecond[60];
uint16_t ExceedanceCounter=0;
uint16_t TargetConcentration;
uint8_t FlagHourExpired=0;
uint8_t Flag_MinuteCounter15=0;
uint8_t Flag_SecondsCounter15=0;
uint16_t ArrayOfResistanceMeasurementsPerMinute15[15];
uint16_t ArrayOfResistanceMeasurementsPerSecond5[15];

uint16_t R15min=0, R5sek=0;
float S15min=0.f, S5sek=0.f;
uint16_t Signal=0;
uint8_t Flag15min=0;

char ServiceMessage[100];



char ZN[]="Q-0000";
char ZN1[]="Q-0000";

#define VERSION "Version 1.17"

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

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
  InitTIM3();
  ADC_Init();//инициализация ацп
  Setting_Init();
  /* USER CODE END 2 */
  //Пересылка структур с настройками для ModBus
  MT_PORT_SetTimerModule(&htim16);
  MT_PORT_SetUartModule(&huart2);

  IWDG_Init();


  usRegAnalog[DEVICE_TYPE]=11;

  while(SecondsCounter<30 && MinuteCounter<1) //1.5 минуты прогреваем
  {
	  if(SecondsCounter%2 == 0)
	  {
		  indicator_sgd4(SPI1, 0x00, StringIndication, 0b010);
	  }
	  else
	  {
		  indicator_sgd4(SPI1, 0x00, StringIndication, 0b000);
	  }
	  if(TimerFlagTIM3)//каждую секунду меряем
	  {
		  GasMeasurement();
		  TimerFlagTIM3=0;
	  }
	  IWDG_Reset(); //Обновление сторожевого таймера
  }
  R_average = Rse; //берем как среднее текущее значение сопротивления

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  if(FlagChangeSetting)//Если сработал флаг изменения настроек датчика
	  {
		  //Записываем в массив основные настройки, которые пойдут во флеш память
		  DataSettingMemory[0] = GlobalAddress;
		  DataSettingMemory[1] = Sensitivity;
		  DataSettingMemory[2] = ModeRele;
		  DataSettingMemory[3] = Resistor120;

		  DataSettingMemory[4] = usRegAnalog[ZN_ADDR]>>8;
		  DataSettingMemory[5] = usRegAnalog[ZN_ADDR];
		  DataSettingMemory[6] = usRegAnalog[ZN_ADDR1]>>8;
		  DataSettingMemory[7] = usRegAnalog[ZN_ADDR1];
		  DataSettingMemory[8] = usRegAnalog[ZN_ADDR2]>>8;
		  DataSettingMemory[9] = usRegAnalog[ZN_ADDR2];

		  WriteToFleshMemory(0xFC00, DataSettingMemory, 10);//то записываем изменения в память


		  //Так как были изменения - заново инициализируем настройки
		  usRegAnalog[STATUS_ADDR] = Sensitivity;



		  if(Resistor120==120)//включаем резистор 120 Ом
		  {
			  R120_SET;
		  }
		  else if(Resistor120==0)
		  {

			  R120_RESET;
		  }

		  switch (Sensitivity)//Выбираем чувствительность
		  {
			  case 1:
			  {
				TargetConcentration=NH;
			  }break;

			  case 2:
			  {
				TargetConcentration=CH;
			  }break;

			  case 3:
			  {
				TargetConcentration=VH;
			  }break;
		  }

		  FlagChangeSetting=0;
	  }

	  KeyPress(); //Обработка нажатий клавиш

	  eMBPoll(); //Проверка сообщений по modBus

	  if(TimerFlagTIM3)//каждую секунду
	  {
			if(GlobalAddress != GlobalAddressFlesh)//если изменен адрес устройства, то проводится переинициализация
			{
				eMBDisable();
				eMBInit(MB_RTU, (UCHAR)GlobalAddress, 0, BaudRateModBusRTU, MB_PAR_NONE); //начальные настройки modBus
				eMBEnable();
				GlobalAddressFlesh = GlobalAddress;
			}

		  GasMeasurement();//Измерение и расчет концентрации

		  ZN1[0]=(char)(usRegAnalog[121-REG_ADRESS_INPUT_START]) & 0xff;
		  ZN1[1]=(char)(usRegAnalog[121-REG_ADRESS_INPUT_START]>>8) &  0xff;
		  ZN1[3]=(char)(usRegAnalog[122-REG_ADRESS_INPUT_START]>>8) & 0xff;
		  ZN1[4]=(char)(usRegAnalog[123-REG_ADRESS_INPUT_START]) & 0xff;
		  ZN1[5]=(char)(usRegAnalog[123-REG_ADRESS_INPUT_START]>>8) & 0xff;

		  sprintf(ServiceMessage, "SGD-1(%s)\tADDRESS = %d\tR0 = %d\tR = %d\tR15min = %d\tCONCENTRATION = %d\n",ZN1, GlobalAddress, (uint16_t)R_average,
					  (uint16_t)Rse, (uint16_t)R15min, (uint16_t)S);
		  USART_send_string(ServiceMessage);


		  if(StatusMode==13)
		  {
			  usRegAnalog[DEVICE_STATUS] = (uint16_t)13; //передается сообщение
			  sprintf(StringIndication, "%d", GlobalAddress);
			  indicator_sgd4(SPI1, 0x00, StringIndication, 0b010);
		  }
		  else if(StatusMode==23)
		  {
			  usRegAnalog[DEVICE_STATUS] = (uint16_t)23; //передается сообщение
			  sprintf(StringIndication, "%d", GlobalAddress);
			  indicator_sgd4(SPI1, 0x00, StringIndication, 0b110);
		  }

		  TimerFlagTIM3=0;
	  }

	  if(S>(TargetConcentration+TargetConcentration*0.1))//если концентрация превысила заданное значение на 10%
	  {
		  ModeAlarm();
	  }

	  IWDG_Reset(); //Обновление сторожевого таймера
  }

}
void ModeAlarm()
{
	ExceedanceCounter++;//Счетчик ошибок
	usRegAnalog[TOTAL_ALARMS_ADDR] = ExceedanceCounter;
	TIM15->CR1 |= TIM_CR1_CEN;//запуск таймера 15
	if(ModeRele==1)//Режим залипания реле
	{
		GPIOB->BSRR |= GPIO_BSRR_BS_12;
		uint8_t FlagZalip=0;
		//крутимся если концентрация выше уставки на 50% и если она упала, но не было выключено реле, то тоже крутимся
		while(S>(TargetConcentration-TargetConcentration*0.5) || (S<(TargetConcentration-TargetConcentration*0.5) && FlagZalip==0))
		{
			usRegAnalog[DEVICE_STATUS] = (uint16_t)18; //передается сообщение тревоги
			StatusMode = 18;
			if(FlagMogan == 0)//моргание красным диодом
			{
				indicator_sgd4(SPI1, 0x00, StringIndication, 0b100);
			}
			else
			{
				indicator_sgd4(SPI1, 0x00, StringIndication, 0b000);
			}

			if((ShortPressKey_PB8==1 || ShortPressKey_PB2==1))
			{
				FlagZalip=1;
			}
			if(TimerFlagTIM3)
			{
				GasMeasurement();//продолжаем измерять
				  sprintf(ServiceMessage, "SGD-1(%s)\tADDRESS = %d\tR0 = %d\tR = %d\tR15min = %d\tCONCENTRATION = %d\n",ZN1, GlobalAddress, (uint16_t)R_average,
							  (uint16_t)Rse, (uint16_t)R15min, (uint16_t)S);
				  USART_send_string(ServiceMessage);
				TimerFlagTIM3=0;
			}
			eMBPoll(); //Проверка сообщений по modBus

			IWDG_Reset(); //Обновление сторожевого таймера (чтобы не выкинуло)
		}
		GPIOB->BSRR |= GPIO_BSRR_BR_12;//Выключаем реле
		//если были нажаты кнопки, сбраываем флаги
		if(ShortPressKey_PB8==1)
		{
			ShortPressKey_PB8=0;
		}
		if(ShortPressKey_PB2==1)
		{
			ShortPressKey_PB2=0;
		}

		TimerCounterTIM14=0;

		TIM14->CR1 &= ~TIM_CR1_CEN;//Выключение таймер
	}
	else if(ModeRele==0)
	{
		while(S>(TargetConcentration-TargetConcentration*0.5))//крутимся, пока концентрация выше
		{
			usRegAnalog[DEVICE_STATUS] = (uint16_t)18; //передается сообщение тревоги
			StatusMode = 18;
			if(FlagMogan == 0)
			{
				indicator_sgd4(SPI1, 0x00, StringIndication, 0b100);
			}
			else
			{
				indicator_sgd4(SPI1, 0x00, StringIndication, 0b000);
			}

			eMBPoll(); //Проверка сообщений по modBus
			GPIOB->BSRR |= GPIO_BSRR_BR_12;
			if(TimerFlagTIM3)
			{
				GasMeasurement();//продолжаем измерять
				  sprintf(ServiceMessage, "SGD-1(%s)\tADDRESS = %d\tR0 = %d\tR = %d\tR15min = %d\tCONCENTRATION = %d\n",ZN1, GlobalAddress, (uint16_t)R_average,
							  (uint16_t)Rse, (uint16_t)R15min, (uint16_t)S);
				  USART_send_string(ServiceMessage);
				TimerFlagTIM3=0;
			}
			IWDG_Reset(); //Обновление сторожевого таймера (чтобы не выкинуло)
		}


	}
	TIM15->CR1 &= ~TIM_CR1_CEN;//Выключение таймер
	StatusMode = 13;//переходим в режим норма
}
void USART_send_string(char *string)
{
	//  Цикл передачи строки по USART
	for(uint16_t no_sym = 0; *(string + no_sym) != '\0' ; no_sym++)
    {
		//  Передача символа по USART
		USART_send_symbol(*(string + no_sym));
    }//end for

    // Передача символа окончания строки '\0' по USART
    USART_send_symbol('\0');
}
void USART_send_symbol(char symbol)
{

    // Ожидание окончания передачи байта из TDR в TSR
    //while((usart_x->SR & USART_SR_TXE) == 0);
    while((USART1->ISR & USART_ISR_TXE) == 0);

    // Отправка байта по USART
    //usart_x->DR = symbol;
    USART1->TDR = symbol;

    // Ожидание окончания передачи байта из TDR в TSR
    //while((usart_x->SR & USART_SR_TXE) == 0);
    while((USART1->ISR & USART_ISR_TXE) == 0);

}
void Setting_Init()
{
	  ReadToFleshMemory(0xFC00, DataSettingMemory, 10);//читаем из памяти основные настроки и потом записываем их в нужные переменные

	  if(DataSettingMemory[0]<1 || DataSettingMemory[0]>99)//на случай, если изначально в памяти не было записи
	  {
		  GlobalAddress=1;
	  }
	  else
	  {
		  GlobalAddress = (uint8_t)DataSettingMemory[0];
	  }

	  if(DataSettingMemory[1]<1 && DataSettingMemory[1]>3)
	  {
		  Sensitivity=3;
	  }
	  else
	  {
		  Sensitivity = (uint8_t)DataSettingMemory[1];
	  }
	  if(DataSettingMemory[2]==1 || DataSettingMemory[2]==0)
	  {
		 ModeRele = (uint8_t)DataSettingMemory[2];
	  }
	  else
	  {
		  ModeRele = 0;
	  }

	  if(DataSettingMemory[3]==0 || DataSettingMemory[3]==1)
	  {
		  Resistor120 = (uint8_t)DataSettingMemory[3];
	  }
	  else
	  {
		  Resistor120 = 0;
	  }



	  if(DataSettingMemory[5]==0xFF)
	  {
		  usRegAnalog[ZN_ADDR]  = *((uint16_t*)ZN);
		  usRegAnalog[ZN_ADDR1] = *((uint16_t*)(ZN)+1);
		  usRegAnalog[ZN_ADDR2] = *((uint16_t*)(ZN)+2);
	  }
	  else
	  {
		  usRegAnalog[ZN_ADDR] = DataSettingMemory[4]<<8 | DataSettingMemory[5];
		  usRegAnalog[ZN_ADDR1] = DataSettingMemory[6]<<8 | DataSettingMemory[7];
		  usRegAnalog[ZN_ADDR2] = DataSettingMemory[8]<<8 | DataSettingMemory[9];
	  }




	  usRegAnalog[STATUS_ADDR] = Sensitivity;
	  usRegAnalog[TOTAL_ALARMS_ADDR] = ExceedanceCounter;

	  switch (Sensitivity)//Выбираем чувствительность
	  {
		  case 1:
		  {
			TargetConcentration=NH;
		  }break;

		  case 2:
		  {
			TargetConcentration=CH;
		  }break;

		  case 3:
		  {
			TargetConcentration=VH;
		  }break;
	  }

	  if(Resistor120==120)//включаем резистор 120 Ом
	  {
		  R120_SET;
	  }
	  else if(Resistor120==0)
	  {
		  R120_RESET;
	  }
}
void GasMeasurement()
{
	SecondsCounter++; //Считаем одну секунду
	SecondsCounter5++;

	//Производим измерение сопротивления сенсора
	VoltageInR2 = 3.3f * ADC_Read() / 4096.0f;//Переводим значение АЦП (12бит) в вольты
	Ir2 = (double)VoltageInR2/(double)(R2*1000);//Вычисляем ток в цепи
	Use = 5.0f - VoltageInR2 - Ir2*(double)(R1*1000);//вычиcляем напряжение на чувств элементе
	Rse = Use/(double)(Ir2*1000.f);//находим сопротивление чувствительного элемента в кОм

	//Записываем каждую секунду данные в массывы, для нахождения скользящего среднего каждую секунду
	if(SecondsCounter>0 && SecondsCounter<=60)//проверяем счетчик на правильный доступ к памяти
	{
		ArrayOfResistanceMeasurementsPerSecond[SecondsCounter-1] = (uint16_t)Rse;//записываем значение сопротивления сенсора в массив для нахождения среднего за 1 минуту
	}
	if(SecondsCounter5>0 && SecondsCounter5<=5)//проверяем счетчик на правильный доступ к памяти
	{
		ArrayOfResistanceMeasurementsPerSecond5[SecondsCounter5-1] = (uint16_t)Rse;//записываем значение сопротивления сенсора в массив для нахождения среднего за 15 секунд
	}

	//Находим скользящее среднее за 15 секунд
	if(MinuteCounter>0 || FlagHourExpired==1)
	{
		uint32_t Sum5sek=0;
		for(uint8_t i=0; i<5; i++)
		{
			Sum5sek+=ArrayOfResistanceMeasurementsPerSecond5[i];
		}
		R5sek = Sum5sek/5;//нашли скользящее среднее за 5 секунд
	}

	if(SecondsCounter5==5)
	{
		SecondsCounter5=0;
	}

	if(SecondsCounter>=60)//Прошла одна минута
	{
		MinuteCounter++;//Считаем минуты
		MinuteCounter15++;//Считаем минуты

		//Находим бегущее среднее за минуту
		uint32_t Sum=0;
		uint16_t Srednee=0;
		for(int i=0; i<60; i++)
		{
			Sum+=ArrayOfResistanceMeasurementsPerSecond[i];
		}
		Srednee = Sum/60;

		//Записываем каждую минуту данные в массивы, для нахождения скользящего среднего
		if(MinuteCounter>0 && MinuteCounter<61)
		{
			ArrayOfResistanceMeasurementsPerMinute[MinuteCounter-1] = (uint8_t)Srednee;//Записали минутное измеренное значение в массив
		}
		if(MinuteCounter>0 && MinuteCounter<=15)
		{
			ArrayOfResistanceMeasurementsPerMinute15[MinuteCounter15-1] = (uint8_t)Srednee;//Записали минутное измеренное значение в массив
		}

		//Находим скользящее среднее за 15 минут
		if(MinuteCounter>=15 || FlagHourExpired==1)//Если массив минут уже имеет 15 значений или прошел час, тогда массив точно имеет 15 значений и каждую минуту вычисляем среднее r15мин
		{
			uint32_t Sum15min=0;

			for(uint8_t i=0; i<15; i++)
			{
				Sum15min+=ArrayOfResistanceMeasurementsPerMinute15[i];
			}
			R15min = Sum15min/15;//нашли скользящее среднее за 15 минут
		}
		else
		{
			R15min = R_average;
		}


		if(MinuteCounter>=60)//прошел один час
		{
			FlagHourExpired=1;//Установили флаг того, что прошел один час
			MinuteCounter=0;//обнуляем счетчик
		}
		if(MinuteCounter==15)
		{
			MinuteCounter15=0;//обнуляем счетчик
		}

		//Находим R0
		if(FlagHourExpired)
		{
			uint32_t Sum2=0;
			//Находим среднее сопротивление за час
			for(int i=0; i<60; i++)
			{
				Sum2+=ArrayOfResistanceMeasurementsPerMinute[i];
			}
			R_average = Sum2/60;;//записали среднее сопротивление за час, и дальше вычисляем каждую минуту среднее сопроитвление
		}
		SecondsCounter=0;
	}

	//если текущее сопротивление выше, чем сопротивление среднее, то пишем
	if(R_average<R5sek && FlagHourExpired!=1)
	{
		R_average = R5sek;//записали среднее сопротивление
	}

	if(R_average!=0)
	{
		if(R15min>0)
		{
			S15min = (-A*(B+C/(R15min))*logf(-(-D-F/R_average + (R_average - R15min)/(R_average * G))));//вычисляем концентацию для R15минут

			if(R5sek>0)
			{
				S5sek = (-A*(B+C/(R5sek))*logf(-(-D-F/R_average + (R_average - R5sek)/(R_average * G))));//вычисляем концентацию для R15секунд
			}
		}
	}

	//находим разницу концентраций измеренных за 15 минут и 15 секунд
	if(S5sek>S15min)
	{
		Signal = S5sek-S15min;
	}
	else
	{
		Signal = 0;
	}

	S=Signal;

	//пишем значения в регистры модбас
	usRegAnalog[SENSOR] = (uint16_t)(Signal);
	usRegAnalog[SENSOR_NULLC_LONG] = (uint16_t)(R_average);
	usRegAnalog[SENSOR_CLEAR] = (uint16_t)(Rse);
	usRegAnalog[SENSOR_NULLC] = (uint16_t)(R15min);

	if((MinuteCounter>30 && R_average<10) || VoltageInR2==0) //Если за 30 минут сопротивление R0 ниже 10кОм или не поступает сигнал с датчика
	{
		StatusMode = 23;//выставляем статус ошибки
	}

}
void IWDG_Init()
{
	IWDG->KR = 0xCCCC;//Запуск таймера IWDG
	IWDG->KR = 0x5555;//Разрешение доступа к таймеру
	IWDG->PR = 0b111;//Предделитель /256
	IWDG->RLR = 156*10;//Значение регистра перезагрузки (время сброса таймера ~1c)
	while(IWDG->SR);//Ожидание обновления регистров
	IWDG->KR = 0xAAAA;//Обновление значения счетчика
}
void IWDG_Reset()
{
	IWDG->KR = 0xAAAA;//сбрасываем сторожевой таймер
}
void KeyPress()
{
	if(TimerCounterTIM14>0)//таймер включается, когда нажаты кнопки
	{
		CheckingKeyTimings();//если кнопки нажаты, то смотрим длину нажатия
	}

	if((LongPressKey_PB8))//Сработало длинное нажатие ЭТО ДЛЯ НАСТРОЙКИ АДРЕСА
	{
	  TimerCounterTIM15=0;
	  LongPressKey_PB8=0;
	  indicator_sgd4(SPI1, 0x00, "PRG", 0b010);//Процесс индикации режима настройки
	  HAL_Delay(1000);
	  TIM15->CR1 |= TIM_CR1_CEN;//запуск таймера 15
	  while(TimerCounterTIM15<=6) //Если таймер больше 3 сек, то заканчиваем настройку
	  {
		IWDG_Reset(); //Обновление сторожевого таймера
		sprintf(StringIndication, "%d", GlobalAddress);
		if(FlagMogan == 0)//моргаем светодиодом
		{
			indicator_sgd4(SPI1, 0x00, StringIndication, 0b010);
		}
		else
		{
			indicator_sgd4(SPI1, 0x00, StringIndication, 0b000);
		}
		if(ShortPressKey_PB8)//короткое нжатие
		{
			if(GlobalAddress<99)
			{
				GlobalAddress++;
			}
			else
			{
				GlobalAddress = 99;
			}
			ShortPressKey_PB8=0;
			TimerCounterTIM15=0;
		}
		if(ShortPressKey_PB2)//короткое нжатие
		{
			if(GlobalAddress>1)
			{
				GlobalAddress--;
			}
			else
			{
				GlobalAddress = 1;
			}
			ShortPressKey_PB2=0;
			TimerCounterTIM15=0;
		}
	  }
	  FlagChangeSetting=1;//выставляем флаг изменения настроек
	  TIM15->CR1 &= ~TIM_CR1_CEN;//выключаем таймер мигания
	}
	if((LongPressKey_PB2))//Сработало длинное нажатие ЭТО ДЛЯ НАСТРОЙКИ ЧУВСТВИТЕЛЬНОСТИ
	{
	  TimerCounterTIM15=0;
	  LongPressKey_PB2=0;
	  indicator_sgd4(SPI1, 0x00, "PRG", 0b010);//Процесс индикации режима настройки
	  HAL_Delay(1000);
	  TIM15->CR1 |= TIM_CR1_CEN;//запуск таймера 15
	  while(TimerCounterTIM15<=6) //Если таймер больше 3 сек, то заканчиваем настройку
	  {
		  IWDG_Reset(); //Обновление сторожевого таймера
		  if(FlagMogan == 0)//моргаем светодиодом
		  {
			  IndicationSensitivity(Sensitivity, 0b010);
		  }
		  else
		  {
			  IndicationSensitivity(Sensitivity, 0b000);
		  }
		  if(ShortPressKey_PB8)//короткое нжатие
		  {
			if(Sensitivity<3)
			{
				Sensitivity++;
			}
			else
			{
				Sensitivity = 3;
			}
			TimerCounterTIM15=0;
			ShortPressKey_PB8=0;
		  }
		  if(ShortPressKey_PB2)//короткое нжатие
		  {
			if(Sensitivity>1)
			{
				Sensitivity--;
			}
			else
			{
				Sensitivity = 1;
			}
			TimerCounterTIM15=0;
			ShortPressKey_PB2=0;
		  }
		  usRegAnalog[STATUS_ADDR]=Sensitivity;
	  }
	  FlagChangeSetting=1;//выставляем флаг изменения настроек
	  TIM15->CR1 &= ~TIM_CR1_CEN;//выключаем таймер мигания
	}

	if(LongDoublePressKey_PB2_PB8) //сработали обе кнопки в длинную ЭТО ДЛЯ НАСТРОЙКИ ЗАЛИПАНИЯ
	{
		TimerCounterTIM15=0;
		indicator_sgd4(SPI1, 0x00, "PRG", 0b010);//Процесс индикации режима настройки
		HAL_Delay(2000);
		if(ClickFlag_PB2==1 && ClickFlag_PB8==1)//нажатие обеих кнопок три секунды
		{
			indicator_sgd4(SPI1, 0x00, "RE3", 0b010);//Процесс индикации режима настройки
			HAL_Delay(1000);
			TIM15->CR1 |= TIM_CR1_CEN;//запуск таймера 15
			while(TimerCounterTIM15<=6) //Если таймер больше 3 сек, то заканчиваем настройку
			{

			  IWDG_Reset(); //Обновление сторожевого таймера
			  sprintf(StringIndication, "%d",  Resistor120);
			  if(FlagMogan == 0)
			  {
				  indicator_sgd4(SPI1, 0x00, StringIndication, 0b010);//Индикация текущей настройки релеЁ
			  }
			  else
			  {
				  indicator_sgd4(SPI1, 0x00, StringIndication, 0b000);//Индикация текущей настройки релеЁ
			  }
			  if(ShortPressKey_PB8)//короткое нжатие
			  {
				  Resistor120 = 120;
				  ShortPressKey_PB8=0;
				  TimerCounterTIM15=0;
			  }
			  if(ShortPressKey_PB2)//короткое нжатие
			  {
				  Resistor120 = 0;
				  ShortPressKey_PB2=0;
				  TimerCounterTIM15=0;
			  }
			}
		}
		else//нажатие обеих кнопок более 3х секунд
		{
			TIM15->CR1 |= TIM_CR1_CEN;//запуск таймера 15//запуск таймера 15
			while(TimerCounterTIM15<=6) //Если таймер больше 3 сек, то заканчиваем настройку
			{

			  IWDG_Reset(); //Обновление сторожевого таймера
			  if(ModeRele)
			  {
				  StringIndication[0] = '1';
				  StringIndication[1] = '1';
				  StringIndication[2] = '1';
			  }
			  else
			  {
				  StringIndication[0] = '0';
				  StringIndication[1] = '0';
				  StringIndication[2] = '0';
			  }
			  if(FlagMogan == 0)
			  {
				  indicator_sgd4(SPI1, 0x00, StringIndication, 0b010);//Индикация текущей настройки релеЁ
			  }
			  else
			  {
				  indicator_sgd4(SPI1, 0x00, StringIndication, 0b000);//Индикация текущей настройки релеЁ
			  }

			  if(ShortPressKey_PB8)//короткое нжатие
			  {
				  ModeRele = 1;
				  ShortPressKey_PB8=0;
				  TimerCounterTIM15=0;
			  }
			  if(ShortPressKey_PB2)//короткое нжатие
			  {
				  ModeRele = 0;
				  ShortPressKey_PB2=0;
				  TimerCounterTIM15=0;
			  }
			}
		}
		TIM15->CR1 &= ~TIM_CR1_CEN;
		FlagChangeSetting=1;//выставляем флаг изменения настроек
		LongDoublePressKey_PB2_PB8=0;
	}
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
	 // Включение тактирования PORTA
	    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	    // Включение тактирования PORTB
	    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	    // PA5 (SCK), PA7 (MOSI) - альтернативная функция
	    GPIOA->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1;
	    GPIOB->MODER |= GPIO_MODER_MODER9_0;

	    // PA5, PA7 - двухтактные выходы
	    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_7);
	    // PB9 - двухтактный выход
	    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_9;
	    // PA5, PA7 - высокая скорость
	    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR7;
	    // PB9 - высокая скорость
	    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;
	    // PA5, PA7 - альтернативная функция AF0
	    GPIOA->AFR[0] &= ~(0xF0000 | 0xF0000000);
	    // Включение тактирования SPI1
	    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	    // Скорость передачи: fPCLK/64  f
	    SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
	    // Однопроводный режим, мастер использует только вывод MOSI
	    SPI1->CR1 |= SPI_CR1_BIDIMODE;
	    // Данные только передаются в однопроводном режиме
	    SPI1->CR1 |= SPI_CR1_BIDIOE;
	    // Работа в режиме ведущего
	    SPI1->CR1 |= SPI_CR1_MSTR;
	    //
	    SPI1->CR1 |= SPI_CR1_SSM;
	    SPI1->CR1 |= SPI_CR1_SSI;
	    // 8-битный формат
	    SPI1->CR2 &= ~SPI_CR2_DS;
	    SPI1->CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
	    // Включение SPI2
	    SPI1->CR1 |= SPI_CR1_SPE;
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
  GPIOA->MODER |= GPIO_MODER_MODER0_0;
  GPIOA->OTYPER &= ~ GPIO_OTYPER_OT_0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0;

  //LED G:
  GPIOB->MODER |= 0b01<<GPIO_MODER_MODER10_Pos; //OUTPUT MODE
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT_10;//PUSH-PULL
  GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEEDR10_Pos;//HIGH SPEED
  GPIOB->PUPDR |= 0b10<<GPIO_PUPDR_PUPDR10_Pos; //PULL DOWN

  //TER:
  GPIOA->MODER |= 0b01<<GPIO_MODER_MODER12_Pos;//OUTPUT MODE
  GPIOA->OTYPER &= ~ GPIO_OTYPER_OT_12;//OPEN-DRAIN
  GPIOA->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEEDR12_Pos;//HIGH SPEED

  //REL:
  GPIOB->MODER |= 0b01<<GPIO_MODER_MODER12_Pos;//OUTPUT MODE
  GPIOB->OTYPER &= ~ GPIO_OTYPER_OT_12;//OPEN-DRAIN
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
	EXTI_PRreg = EXTI->PR;
	EXTI->PR |= 1;
	if((EXTI_PRreg & EXTI_PR_PR2)>0)
	{
		EXTI_PRreg &= ~EXTI_PR_PR2;
		if(!ClickFlag_PB2)
		{
			TimerCounterTIM14=0;
			ClickFlag_PB2=1;
		}
		else
		{
			if(TimerCounterTIM14<3000)
			{
				//КОРОТКОЕ НАЖАТИЕ
				ShortPressKey_PB2=1;
			}
			ClickFlag_PB2=0;
		}
	}

	if((EXTI_PRreg & EXTI_PR_PR8)>0)
	{
		EXTI_PRreg &= ~EXTI_PR_PR8;
		if(!ClickFlag_PB8)
		{
			TimerCounterTIM14=0;
			ClickFlag_PB8=1;
		}
		else
		{
			if(TimerCounterTIM14<3000)
			{
				//КОРОТКОЕ НАЖАТИЕ
				ShortPressKey_PB8=1;
			}
			ClickFlag_PB8=0;
		}
	}

	TimerCounterTIM14=0;
	if(ClickFlag_PB2 || ClickFlag_PB8)
	{
		TIM14->CR1 |= TIM_CR1_CEN;//Включение таймер
	}
	else
	{
		TIM14->CR1 &= ~TIM_CR1_CEN;//Включение таймер
	}
}
void CheckingKeyTimings()
{
	if(TimerCounterTIM14==3000 && ClickFlag_PB8==1 && ClickFlag_PB2==0)
	{
		//ДЛИННОЕ НАЖАТИЕ
		LongPressKey_PB8=1;
	}
	else if(TimerCounterTIM14==3000 && ClickFlag_PB2==1 && ClickFlag_PB8==0)
	{
		//ДЛИННОЕ НАЖАТИЕ
		LongPressKey_PB2=1;
	}
	else if(TimerCounterTIM14==3000 && ClickFlag_PB8==1 && ClickFlag_PB2==1)
	{
		LongDoublePressKey_PB2_PB8=1;
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
  else if(usAddress==3334)
	{
		*pucRegBuffer++ = (unsigned char)(ModeRele >> 8);
		*pucRegBuffer++ = (unsigned char)(ModeRele & 0xFF);
	}
  else if(usAddress==7778)
	{
		*pucRegBuffer++ = (unsigned char)(GlobalAddress >> 8);
		*pucRegBuffer++ = (unsigned char)(GlobalAddress & 0xFF);
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
		iRegIndex = (int)(usAddress - usRegAdressInputStart);
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
			if(usAddress==105)
			{
				uint16_t buf = pucRegBuffer[1] + 256*pucRegBuffer[0];
				if(buf>0 && buf<4)
				{
					Sensitivity = buf;
					usRegAnalog[STATUS_ADDR] = Sensitivity;
					FlagChangeSetting=1;
				}
				else
				{
					eStatus = MB_EIO;
				}
			}
			if(usAddress==121)
			{
				uint16_t buf = pucRegBuffer[1] + 256*pucRegBuffer[0];
				usRegAnalog[ZN_ADDR] = buf;
				FlagChangeSetting=1;
			}
			if(usAddress==122)
			{
				uint16_t buf1 = pucRegBuffer[1] + 256*pucRegBuffer[0];
				usRegAnalog[ZN_ADDR1] = buf1;
				FlagChangeSetting=1;
			}
			if(usAddress==123)
			{
				uint16_t buf2 = pucRegBuffer[1] + 256*pucRegBuffer[0];
				usRegAnalog[ZN_ADDR2] = buf2;
				FlagChangeSetting=1;
			}

		}

	  }
	else if(usAddress==3334)
	{
		if(eMode==MB_REG_READ)
		{
			*pucRegBuffer++ = (unsigned char)(ModeRele >> 8);
			*pucRegBuffer++ = (unsigned char)(ModeRele & 0xFF);
		}
		if(eMode==MB_REG_WRITE)
		{
			uint16_t buf=pucRegBuffer[1] + 256*pucRegBuffer[0];
			if(buf==0 || buf==1)
			{
				ModeRele = (uint8_t) buf;
				FlagChangeSetting=1;
			}
			else
			{
				eStatus = MB_EIO;
			}
		}
	}
	else if(usAddress==7778)
	{
		if(eMode==MB_REG_READ)
		{
			*pucRegBuffer++ = (unsigned char)(GlobalAddress >> 8);
			*pucRegBuffer++ = (unsigned char)(GlobalAddress & 0xFF);
		}
		else if(eMode==MB_REG_WRITE)
		{
			uint16_t buf = pucRegBuffer[1] + 256*pucRegBuffer[0];
			if(buf>0 && buf<100)
			{
				GlobalAddress = (uint8_t)buf;
				FlagChangeSetting=1;
			}
			else
			{
				eStatus = MB_EIO;
			}

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


void IndicationSensitivity(uint8_t Sensitivity, uint8_t led)
{
	switch (Sensitivity)
	{

	case	1:
	{
		indicator_sgd4(SPI1, 0x00, " NH", led);
	}break;
	case	2:
	{
		indicator_sgd4(SPI1, 0x00, " CH", led);
	}break;
	case	3:
	{
		indicator_sgd4(SPI1, 0x00, " BH", led);
	}break;
	}
}
/* USER CODE END 4 */

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
