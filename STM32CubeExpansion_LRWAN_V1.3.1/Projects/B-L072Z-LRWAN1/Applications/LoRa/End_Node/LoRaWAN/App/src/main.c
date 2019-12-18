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
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include <stdio.h>
#include <string.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LORAWAN_MAX_BAT   254

/*!
 * CAYENNE_LPP is myDevices Application server.
 */
//#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_APP_PORT 99
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            300000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
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


#define SENSOR_RESP_ACK 1
#define SENSOR_RESP_NAK 2
#define SENSOR_RESP_SINGLE 3
#define SENSOR_RESP_AUTO 4

static void MX_USART1_UART_Init(void);
void SENSOR_Start_Measuring(void);
void SENSOR_Stop_Measuring(void);
void SENSOR_Read_Measuring(void);
void SENSOR_Stop_Auto_Send(void);
void SENSOR_Enable_Auto_Send(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t respBuf[50];
uint16_t pm2_5 = -1;
uint16_t pm10 = -1;
uint8_t sensorResp = 0;

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
/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;

static TimerEvent_t TxTimer;

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent(void *context);
#endif
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
  // DBG_Init();

  /* Configure the hardware*/
  HW_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 1 */
  HAL_Delay(1500);
	while (1) {
	  HAL_UART_Receive(&huart1, respBuf, 20, 500);
	  if (respBuf[0] == 0xA5 && respBuf[1] == 0xA5) break;
	  PRINTF("STOP\n\r");
	  SENSOR_Stop_Auto_Send();
	}
	SENSOR_Read_Measuring();
  /* USER CODE END 1 */

  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

  PRINTF("APP_VERSION= %02X.%02X.%02X.%02X\r\n", (uint8_t)(__APP_VERSION >> 24), (uint8_t)(__APP_VERSION >> 16), (uint8_t)(__APP_VERSION >> 8), (uint8_t)__APP_VERSION);
  PRINTF("MAC_VERSION= %02X.%02X.%02X.%02X\r\n", (uint8_t)(__LORA_MAC_VERSION >> 24), (uint8_t)(__LORA_MAC_VERSION >> 16), (uint8_t)(__LORA_MAC_VERSION >> 8), (uint8_t)__LORA_MAC_VERSION);

  /* Configure the Lora Stack*/
  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

  LORA_Join();

  LoraStartTx(TX_ON_TIMER) ;
  PRINTF("1234556\n\r");
  while (1)
  {
    if (AppProcessRequest == LORA_SET)
    {
      /*reset notification flag*/
      AppProcessRequest = LORA_RESET;
      /*Send*/
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
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass(LORAWAN_DEFAULT_CLASS);
}

static void Send( void* context )
{
	MX_USART1_UART_Init();
	SENSOR_Start_Measuring();
	HAL_UART_Receive(&huart1, respBuf, 2, 1000);
	respBuf[2] = 0;
	PRINTF(respBuf);
	PRINTF("1\n\r");
  /* USER CODE BEGIN 3 */
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    SENSOR_Stop_Measuring();
    HAL_UART_Receive(&huart1, respBuf, 2, 1000);
    return;
  }

  PRINTF("2\n\r");
  HAL_UART_Receive(&huart1, respBuf, 40, 6000);
  SENSOR_Read_Measuring();

  sensorResp = 0;
  PRINTF("21\n\r");
  HAL_UART_Receive(&huart1, respBuf, 2, 1000);
  PRINTF("3\n\r");
  if (respBuf[0] == 0x40) {
	uint8_t len = respBuf[1];
	uint16_t calChecksum = 0;

	HAL_UART_Receive(&huart1, &respBuf[2], len + 1, 30); // get cmd, data and checksum
	for (int i = 0; i < len + 2; i++) { // with head, len and cmd ( +3 ) but not checksum ( -1 )
		calChecksum += respBuf[i];
	}
	calChecksum = (65536 - calChecksum) & 255;
	if (calChecksum == respBuf[2 + len]) {
		pm2_5 = ((uint16_t)respBuf[3] << 8) | respBuf[4];
		pm10 = ((uint16_t)respBuf[5] << 8) | respBuf[6];
		sensorResp = SENSOR_RESP_SINGLE;
	}
  } else {
	  HAL_UART_Receive(&huart1, &respBuf[2], 40, 30);
  }
  PRINTF("4\n\r");
  SENSOR_Stop_Measuring();
  if (sensorResp == SENSOR_RESP_SINGLE) {
	  AppData.Buff[0] = 32;
	  AppData.Buff[1] = respBuf[4];
	  if (AppData.Buff[1] == 191) {
		  AppData.Buff[1] = 192;
	  }
  } else {
	  AppData.Buff[0] = 32;
	  AppData.Buff[1] = 191;
	  for (int i = 0; i < 42; i++)
		  PRINTF("%c", respBuf[i]);
	  PRINTF("55\n\r");
  }
  //set size and port
	AppData.BuffSize = 2;
	AppData.Port = LORAWAN_APP_PORT;

	PRINTF("5\n\r");
	//Send to LoRaWAN
	LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
  /* USER CODE END 3 */
	PRINTF("6\n\r");
}


static void LORA_RxData(lora_AppData_t *AppData)
{
  /* USER CODE BEGIN 4 */
  PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);
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
  else
  {
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct = {0};

    initStruct.Mode = GPIO_MODE_IT_RISING;
    initStruct.Pull = GPIO_PULLUP;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct);
    HW_GPIO_SetIrq(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send);
  }
}

static void LORA_ConfirmClass(DeviceClass_t Class)
{
  PRINTF("switch to class %c done\n\r", "ABC"[Class]);

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
  uint16_t batteryLevelmV;
  uint8_t batteryLevel = 0;

  batteryLevelmV = HW_GetBatteryLevel();


  /* Convert batterey level from mV to linea scale: 1 (very low) to 254 (fully charged) */
  if (batteryLevelmV > VDD_BAT)
  {
    batteryLevel = LORAWAN_MAX_BAT;
  }
  else if (batteryLevelmV < VDD_MIN)
  {
    batteryLevel = 0;
  }
  else
  {
    batteryLevel = (((uint32_t)(batteryLevelmV - VDD_MIN) * LORAWAN_MAX_BAT) / (VDD_BAT - VDD_MIN));
  }

  return batteryLevel;
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent(void *context)
{
  LED_Off(LED_RED1) ;
}
#endif


void SENSOR_Start_Measuring() {
	uint8_t buff[] = {0x68, 0x01, 0x01, 0x96};
	HAL_UART_Transmit(&huart1, buff, 4, 500);
}

void SENSOR_Stop_Measuring() {
	uint8_t buff[] = {0x68, 0x01, 0x02, 0x95};
	HAL_UART_Transmit(&huart1, buff, 4, 500);
}

void SENSOR_Read_Measuring() {
	uint8_t buff[] = {0x68, 0x01, 0x04, 0x93};
	HAL_UART_Transmit(&huart1, buff, 4, 500);
}

void SENSOR_Stop_Auto_Send() {
	uint8_t buff[] = {0x68, 0x01, 0x20, 0x77};
	HAL_UART_Transmit(&huart1, buff, 4, 500);
}

void SENSOR_Enable_Auto_Send() {
	uint8_t buff[] = {0x68, 0x01, 0x40, 0x57};
	HAL_UART_Transmit(&huart1, buff, 4, 500);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
#ifdef EN_SENSOR_DEBUG
		char printBuf[60];
		sprintf(printBuf, "SENSOR : ERROR");
#endif
		if (respBuf[0] == 0x00) {
			respBuf[0] = respBuf[1];
			HAL_UART_Receive(&huart1, &respBuf[1], 1, 4);
		}
		if (respBuf[0] == 0x40) {
#ifdef EN_SENSOR_DEBUG
			sprintf(printBuf, "SENSOR : ERROR single read");
#endif
			uint8_t len = respBuf[1];
			uint16_t calChecksum = 0;

			HAL_UART_Receive(&huart1, &respBuf[2], len + 1, 500); // get cmd, data and checksum
			for (int i = 0; i < len + 2; i++) { // with head, len and cmd ( +3 ) but not checksum ( -1 )
				calChecksum += respBuf[i];
			}
			calChecksum = (65536 - calChecksum) & 255;
			if (calChecksum == respBuf[2 + len]) {
				pm2_5 = ((uint16_t)respBuf[3] << 8) | respBuf[4];
				pm10 = ((uint16_t)respBuf[5] << 8) | respBuf[6];
#ifdef EN_SENSOR_DEBUG
				sprintf(printBuf, "SENSOR :+PM2.5 : %d PM10 : %d", pm2_5, pm10);
#endif
				sensorResp = SENSOR_RESP_SINGLE;
			}
		}
		else if (respBuf[0] == 0x42 && respBuf[1] == 0x4d) {
#ifdef EN_SENSOR_DEBUG
			sprintf(printBuf, "SENSOR : ERROR continute read");
#endif
			// Auto send
			uint8_t len;
			uint16_t calChecksum = 0;

			HAL_UART_Receive(&huart1, &respBuf[2], 2, 500); // get len

			len = ((uint16_t)respBuf[2] << 8) | respBuf[3];

			HAL_UART_Receive(&huart1, &respBuf[4], len, 500); //get data and checksum
			for (int i = 0; i < len + 2; i++) { // with head and len ( +4 ) but not checksum ( -2 )
				calChecksum += respBuf[i];
			}
			if (calChecksum == (((uint16_t)respBuf[2 + len] << 8) | respBuf[2 + len + 1])) {
				uint16_t pm2_5_temp = ((uint16_t)respBuf[6] << 8) | respBuf[7];
				uint16_t pm10_temp = ((uint16_t)respBuf[8] << 8) | respBuf[9];
				if (!(pm2_5_temp == 0 && pm10_temp == 0 && sensorResp == 0)) {
#ifdef EN_SENSOR_DEBUG
					sprintf(printBuf, "SENSOR :-PM2.5 : %d PM10 : %d", pm2_5, pm10);
#endif
					sensorResp = SENSOR_RESP_AUTO;
				}
#ifdef EN_SENSOR_DEBUG
				else
				{
					sprintf(printBuf, "SENSOR : Init value");
				}
#endif
			}
		}

		else if (respBuf[0] == 0xA5 && respBuf[1] == 0xA5) {
			// ACK
#ifdef EN_SENSOR_DEBUG
			sprintf(printBuf, "SENSOR : OK");
#endif
			sensorResp = SENSOR_RESP_ACK;
		}
		else if (respBuf[0] == 0x96 && respBuf[1] == 0x96) {
			// NAK
#ifdef EN_SENSOR_DEBUG
			sprintf(printBuf, "SENSOR : NAK");
#endif
			sensorResp = SENSOR_RESP_NAK;
		}
#ifdef EN_SENSOR_DEBUG
		HAL_UART_Transmit(&huart2, (uint8_t*)printBuf, strlen(printBuf), 500);
#endif
		HAL_UART_Receive_IT(&huart1, respBuf, 2);
	}
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */

#define SENSOR_TX_Pin GPIO_PIN_10
#define SENSOR_TX_GPIO_Port GPIOA
#define SENSOR_RX_Pin GPIO_PIN_9
#define SENSOR_RX_GPIO_Port GPIOA

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
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART1 interrupt Init */
	HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
