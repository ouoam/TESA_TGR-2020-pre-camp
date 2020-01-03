/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include <stdio.h>
#include <string.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LORAWAN_MAX_BAT   254

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                 5 * 60 * 1000         // 6 * 60 * 60 * 1000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_OFF
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

#define SENSOR_TX_Pin GPIO_PIN_10
#define SENSOR_TX_GPIO_Port GPIOA
#define SENSOR_RX_Pin GPIO_PIN_9
#define SENSOR_RX_GPIO_Port GPIOA

#define SENSOR_RESP_UNKNOWN 0
#define SENSOR_RESP_ACK 1
#define SENSOR_RESP_NAK 2
#define SENSOR_RESP_SINGLE 3
#define SENSOR_RESP_AUTO 4
#define SENSOR_RESP_COEFF 5
#define SENSOR_RESP_AUTO_ZERO 6
#define SENSOR_RESP_TIMEOUT 7

#define SENSOR_COEFF_ADDR 0x08080000
#define ENGINEER_MODE_ADDR 0x08080080

static void MX_USART1_UART_Init(void);
int SENSOR_Parse_Resp(void);
int SENSOR_Start_Measuring(void);
int SENSOR_Stop_Measuring(void);
int SENSOR_Read_Measuring(void);
int SENSOR_Set_Coefficient(uint8_t coeff);
int SENSOR_Read_Coefficient(void);
int SENSOR_Stop_Auto_Send(void);
int SENSOR_Enable_Auto_Send(void);

void writeToEEPROM (uint32_t address, uint32_t value);
uint32_t readFromEEPROM (uint32_t address);

uint16_t pm2_5 = -1;
uint16_t pm10 = -1;
uint8_t coefficient = 100;

uint8_t RxBuff[9];
uint8_t RxBuffI = 0;
uint8_t enEngineer = false;

uint8_t sendStack = 0;

void RxCpltCallback(uint8_t *rxChar) ;

void InfiniteLoop() {
	PRINTF("\r\n\r\n!!!ENTER INFINITE LOOP!!!\r\n");
	HW_RTC_DelayMs(500);
	DISABLE_IRQ();
	while(1) __NOP();
}

uint8_t enterLoop = 0;

void InfiniteLoop1() {
	enterLoop = 1;
}

UART_HandleTypeDef huart1;

/*!
 * User application data structure
 */
//static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
lora_AppData_t AppData = { AppDataBuff,  0, 0 };

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData(lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined(void);

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass(DeviceClass_t Class);

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded(void);

/* callback to get the battery level in % of full charge (254 full charge, 0 no charge)*/
static uint8_t LORA_GetBatteryLevel(void);

/* LoRa endNode send request*/
static void Send(void *context);

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent(void *context);

/* tx timer callback function*/
static void LoraMacProcessNotify(void);

static void MX_USART1_UART_Init(void);

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { LORA_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded,
                                                LoraMacProcessNotify
                                              };
LoraFlagStatus LoraMacProcessRequest = LORA_RESET;
LoraFlagStatus AppProcessRequest = LORA_RESET;

static TimerEvent_t TxTimer;

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit = {LORAWAN_ADR_STATE,
                                     LORAWAN_DEFAULT_DATA_RATE,
                                     LORAWAN_PUBLIC_NETWORK
                                    };

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/* STM32 HAL library initialization*/
	HAL_Init();

	/* Configure the system clock*/
	SystemClock_Config();

	/* Configure the debug mode*/
	DBG_Init();

	/* Configure the hardware*/
	HW_Init();

	vcom_ReceiveInit(RxCpltCallback);

	/* USER CODE BEGIN 1 */
	enEngineer = readFromEEPROM(ENGINEER_MODE_ADDR);
	if (enEngineer) PRINTF("\r\n\r\n !! !! ENGINEER MODE IS ENABLE !! !!\r\n\r\n\r\n");
	/* USER CODE END 1 */

	/*Disbale Stand-by mode*/
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

	PRINTF("APP_VERSION= %02X.%02X.%02X.%02X\r\n", (uint8_t)(__APP_VERSION >> 24), (uint8_t)(__APP_VERSION >> 16), (uint8_t)(__APP_VERSION >> 8), (uint8_t)__APP_VERSION);
	PRINTF("MAC_VERSION= %02X.%02X.%02X.%02X\r\n", (uint8_t)(__LORA_MAC_VERSION >> 24), (uint8_t)(__LORA_MAC_VERSION >> 16), (uint8_t)(__LORA_MAC_VERSION >> 8), (uint8_t)__LORA_MAC_VERSION);

	/* Configure the Lora Stack*/
	LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

	LORA_Join();

	LoraStartTx(TX_ON_TIMER) ;
	while (1)
	{
		if (AppProcessRequest == LORA_SET)
		{
			/*reset notification flag*/
			AppProcessRequest = LORA_RESET;
			/*Send*/
			sendStack = 0;
			Send(NULL);
		}
		if (LoraMacProcessRequest == LORA_SET)
		{
			/*reset notification flag*/
			LoraMacProcessRequest = LORA_RESET;
			LoRaMacProcess();
		}

		/*If a flag is set at this point, mcu must not enter low power and must loop*/
		DISABLE_IRQ();

		/* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
		* and cortex will not enter low power anyway  */
		if ((LoraMacProcessRequest != LORA_SET) && (AppProcessRequest != LORA_SET))
		{
#ifndef LOW_POWER_DISABLE
			LPM_EnterLowPower();
#endif
		}

		ENABLE_IRQ();

		if (enterLoop) {
			InfiniteLoop();
		}

		/* USER CODE BEGIN 2 */
		/* USER CODE END 2 */
	}
}


void LoraMacProcessNotify(void)
{
  LoraMacProcessRequest = LORA_SET;
}


static void LORA_HasJoined(void)
{
#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\r\n");
#endif
  LORA_RequestClass(LORAWAN_DEFAULT_CLASS);
}

static void Send( void* context )
{
	sendStack++;
	if (enEngineer) PRINTF("\r\n");
	if ( LORA_JoinStatus () != LORA_SET )
	{
		/*Not joined, try again later*/
		LORA_Join();
		return;
	}

	int success = 0;

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	if (enEngineer) PRINTF("SENSOR : WAIT for start\r\n");
	HW_RTC_DelayMs(2000);
	MX_USART1_UART_Init();

	success = SENSOR_Parse_Resp() != SENSOR_RESP_TIMEOUT;
	if (enEngineer) PRINTF("SENSOR : Get First\r\n");
	if (!success) {
		if (enEngineer) PRINTF("SENSOR : try to enable auto send\r\n");
		success = SENSOR_Start_Measuring();
		if (success) success = SENSOR_Enable_Auto_Send();
		if (success) success = SENSOR_Parse_Resp(); // wait for first auto send
	}

	if (success) {
		coefficient = readFromEEPROM(SENSOR_COEFF_ADDR);
		if (enEngineer) PRINTF("EEPROM COEFFICIENT = %d\r\n", coefficient);
		if ( coefficient < 30 || 200 < coefficient ) {
			coefficient = 100;
			writeToEEPROM(SENSOR_COEFF_ADDR, coefficient);
		}
		success = SENSOR_Set_Coefficient(coefficient);
	}

	if (success) {
		if (enEngineer) PRINTF("SENSOR : Start Measuring\r\n");

		for (int i = 0; i < 6; i++) {
			HW_RTC_DelayMs(800);
			success = SENSOR_Parse_Resp() == SENSOR_RESP_AUTO;
		}
	}

	HAL_UART_DeInit(&huart1);

	__HAL_RCC_USART1_CLK_DISABLE();
	GPIO_InitStruct.Pin = SENSOR_TX_Pin|SENSOR_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	if (success) {
		AppData.Buff[0] = 32;
		AppData.Buff[1] = pm2_5 & 0xFF;
		if (AppData.Buff[1] == 191) {
			AppData.Buff[1] = 192;
		}

		if (enEngineer) PRINTF("SENSOR : Success Measuring %u\r\n", pm2_5);
	} else if (sendStack < 3) {
		if (enEngineer) PRINTF("SENSOR : ERROR and Try Again\r\n");
		HW_RTC_DelayMs(5000);
		Send(NULL);
		return; // don't send  send only at top of stack
	} else {
		if (enEngineer) PRINTF("SENSOR : ERROR\r\n");

		AppData.Buff[0] = 32;
		AppData.Buff[1] = 191;
	}

	//set size and port
	AppData.BuffSize = 2;
	AppData.Port = LORAWAN_APP_PORT;

	//Send to LoRaWAN
	LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE );
  /* USER CODE END 3 */
}


static void LORA_RxData(lora_AppData_t *AppData)
{
  /* USER CODE BEGIN 4 */
  PRINTF("PACKET RECEIVED ON PORT %d\r\n", AppData->Port);
  /* USER CODE END 4 */
}

static void OnTxTimerEvent(void *context)
{
  /*Wait for next tx slot*/
  TimerStart(&TxTimer);

  AppProcessRequest = LORA_SET;
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit(&TxTimer, OnTxTimerEvent);
    TimerSetValue(&TxTimer,  APP_TX_DUTYCYCLE);
    OnTxTimerEvent(NULL);
  }
  //else
  //{
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct = {0};

    initStruct.Mode = GPIO_MODE_IT_RISING;
    initStruct.Pull = GPIO_PULLUP;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct);
    HW_GPIO_SetIrq(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, InfiniteLoop1);
  //}
}

static void LORA_ConfirmClass(DeviceClass_t Class)
{
  PRINTF("switch to class %c done\r\n", "ABC"[Class]);

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded(void)
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

/**
  * @brief This function return the battery level
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t LORA_GetBatteryLevel(void)
{
  return 254;
}

int SENSOR_Parse_Resp() {
	uint8_t respBuf[40];
	memset(respBuf, 0, 40);
	if (HAL_UART_Receive(&huart1, respBuf, 2, 500) != HAL_OK) {
		return SENSOR_RESP_TIMEOUT;
	}
	for (uint8_t i = 0; i < 10; i++) {
		if (respBuf[0] != 0xA5 && respBuf[0] != 0x96 &&
				respBuf[0] != 0x40 && respBuf[0] != 0x42) {
			respBuf[0] = respBuf[1];
			HAL_UART_Receive(&huart1, &respBuf[1], 1, 100);
		} else {
			break;
		}
	}
	if (respBuf[0] == 0xA5 && respBuf[1] == 0xA5) return SENSOR_RESP_ACK;
	else if (respBuf[0] == 0x96 && respBuf[1] == 0x96) return SENSOR_RESP_NAK;
	else if (respBuf[0] == 0x40) { // read particle and coefficient
		uint8_t len = respBuf[1];
		uint16_t calChecksum = 0;

		HAL_UART_Receive(&huart1, &respBuf[2], len + 1, 100); // get cmd, data and checksum
		for (int i = 0; i < len + 2; i++) { // with head, len and cmd ( +3 ) but not checksum ( -1 )
			calChecksum += respBuf[i];
		}
		calChecksum = (65536 - calChecksum) & 255;
		if (calChecksum == respBuf[2 + len]) {
			if (respBuf[2] == 0x04) { // check for cmd
				pm2_5 = ((uint16_t)respBuf[3] << 8) | respBuf[4];
				pm10 = ((uint16_t)respBuf[5] << 8) | respBuf[6];
				return SENSOR_RESP_SINGLE;
			} else if (respBuf[2] == 0x10) {
				coefficient = respBuf[3];
				return SENSOR_RESP_COEFF;
			}
		}
	}
	else if (respBuf[0] == 0x42 && respBuf[1] == 0x4d) {
		// Auto send
		uint8_t len;
		uint16_t calChecksum = 0;

		HAL_UART_Receive(&huart1, &respBuf[2], 2, 100); // get len

		len = ((uint16_t)respBuf[2] << 8) | respBuf[3];

		HAL_UART_Receive(&huart1, &respBuf[4], len, 100); //get data and checksum
		for (int i = 0; i < len + 2; i++) { // with head and len ( +4 ) but not checksum ( -2 )
			calChecksum += respBuf[i];
		}
		if (calChecksum == (((uint16_t)respBuf[2 + len] << 8) | respBuf[2 + len + 1])) {
			uint16_t pm2_5_temp = ((uint16_t)respBuf[6] << 8) | respBuf[7];
			uint16_t pm10_temp = ((uint16_t)respBuf[8] << 8) | respBuf[9];
			if (!(pm2_5_temp == 0 && pm10_temp == 0)) {
				pm2_5 = pm2_5_temp;
				pm10 = pm10_temp;
				return SENSOR_RESP_AUTO;
			} else {
				return SENSOR_RESP_AUTO_ZERO;
			}
		}
	} else { // clear
		HAL_UART_Receive(&huart1, &respBuf[2], 38, 200);
	}

	return SENSOR_RESP_UNKNOWN;
}

int SENSOR_Start_Measuring() {
	uint8_t buff[] = {0x68, 0x01, 0x01, 0x96};
	HAL_UART_Transmit(&huart1, buff, 4, 100);
	return (SENSOR_Parse_Resp() == SENSOR_RESP_ACK);
}

int SENSOR_Stop_Measuring() {
	uint8_t buff[] = {0x68, 0x01, 0x02, 0x95};
	HAL_UART_Transmit(&huart1, buff, 4, 100);
	return (SENSOR_Parse_Resp() == SENSOR_RESP_ACK);
}

int SENSOR_Read_Measuring() {
	uint8_t buff[] = {0x68, 0x01, 0x04, 0x93};
	HAL_UART_Transmit(&huart1, buff, 4, 100);
	return (SENSOR_Parse_Resp() == SENSOR_RESP_SINGLE);
}

int SENSOR_Set_Coefficient(uint8_t coeff) {
	uint8_t buff[] = {0x68, 0x02, 0x08, coeff, 0x8E - coeff};
	HAL_UART_Transmit(&huart1, buff, 5, 100);
	return (SENSOR_Parse_Resp() == SENSOR_RESP_ACK);
}

int SENSOR_Read_Coefficient() {
	uint8_t buff[] = {0x68, 0x01, 0x10, 0x87};
	HAL_UART_Transmit(&huart1, buff, 4, 100);
	return (SENSOR_Parse_Resp() == SENSOR_RESP_COEFF);
}

int SENSOR_Stop_Auto_Send() {
	uint8_t buff[] = {0x68, 0x01, 0x20, 0x77};
	HAL_UART_Transmit(&huart1, buff, 4, 100);
	return (SENSOR_Parse_Resp() == SENSOR_RESP_ACK);
}

int SENSOR_Enable_Auto_Send() {
	uint8_t buff[] = {0x68, 0x01, 0x40, 0x57};
	HAL_UART_Transmit(&huart1, buff, 4, 100);
	return (SENSOR_Parse_Resp() == SENSOR_RESP_ACK);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */

static void MX_USART1_UART_Init(void)
{
	__HAL_RCC_USART1_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART1 GPIO Configuration
	PA10     ------> USART1_RX
	PA9     ------> USART1_TX
	*/
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = SENSOR_TX_Pin|SENSOR_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART1 interrupt Init */
	// HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
	// HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}


void writeToEEPROM (uint32_t address, uint32_t value)
{
	HAL_StatusTypeDef flash_ok;
	for (flash_ok = HAL_ERROR; flash_ok != HAL_OK; )
	{
		flash_ok = HAL_FLASHEx_DATAEEPROM_Unlock();
	}
	for (flash_ok = HAL_ERROR; flash_ok != HAL_OK; )
	{
		flash_ok = HAL_FLASHEx_DATAEEPROM_Erase(address);
	}
	for (flash_ok = HAL_ERROR; flash_ok != HAL_OK; )
	{
		flash_ok = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address, value);
	}
	for (flash_ok = HAL_ERROR; flash_ok != HAL_OK; )
	{
		flash_ok = HAL_FLASHEx_DATAEEPROM_Lock();
	}
}

uint32_t readFromEEPROM (uint32_t address)
{
	return *( uint32_t *)address;
}

void RxCpltCallback(uint8_t *rxChar) {
	if ( RxBuffI == 9 ) {
		RxBuffI = 0;
		PRINTF("Rx Buffer overflow\r\n");
	}
	RxBuff[RxBuffI++] = *rxChar;
	if (RxBuff[RxBuffI-1] == '\n'||RxBuff[RxBuffI-1] == '\r'||RxBuff[RxBuffI-1] == ' ') {
		if ( RxBuff[0] == 'E' && RxBuff[1] == 'n' && RxBuff[2] == 'G' && RxBuff[3] == 'n' ) {
			enEngineer = true;
			writeToEEPROM(ENGINEER_MODE_ADDR, 1);
			PRINTF("Engineer mode enable\r\n");
		} else if ( RxBuff[0] == 'D' && RxBuff[1] == 's' && RxBuff[2] == 'G' && RxBuff[3] == 'n' ) {
			enEngineer = false;
			writeToEEPROM(ENGINEER_MODE_ADDR, 0);
			PRINTF("Engineer mode disable\r\n");
		}
		if (enEngineer) {
			if ( RxBuff[0] == 'G' ) {
				coefficient = readFromEEPROM(SENSOR_COEFF_ADDR);
				PRINTF("EEPROM COEFFICIENT = %d\r\n", coefficient);
			} else if ( RxBuff[0] == 'S' ) {
				uint8_t i = 1;
				uint16_t temp = 0;
				while ( RxBuff[i] != '\n' && RxBuff[i] != '\r' && RxBuff[i] != ' ' ) {
					temp *= 10;
					temp += (RxBuff[i] - '0') % 10;
					i++;
				}
				if ( temp < 30 || 200 < temp ) {
					PRINTF("COEFFICIENT out of range (only 30 - 200)\r\n");
				} else {
					PRINTF("Set COEFFICIENT to %u\r\n", temp);
					coefficient = temp;
					writeToEEPROM(SENSOR_COEFF_ADDR, coefficient);
				}
			} else if ( RxBuff[0] == 'T' ) {
				Send(NULL);
			}
		}
		RxBuffI = 0;
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
