/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SX1278.h"
#include "spi.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for initTask */
osThreadId_t initTaskHandle;
const osThreadAttr_t initTask_attributes = {
  .name = "initTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskRadioLora */
osThreadId_t taskRadioLoraHandle;
const osThreadAttr_t taskRadioLora_attributes = {
  .name = "taskRadioLora",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

int master;
int ret;

char buffer[512];

int message;
int message_length;

/* USER CODE END FunctionPrototypes */

void StartInitTask(void *argument);
void StartTaskRadio(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of initTask */
  initTaskHandle = osThreadNew(StartInitTask, NULL, &initTask_attributes);

  /* creation of taskRadioLora */
  taskRadioLoraHandle = osThreadNew(StartTaskRadio, NULL, &taskRadioLora_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartInitTask */
/**
  * @brief  Function implementing the initTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInitTask */
void StartInitTask(void *argument)
{
  /* USER CODE BEGIN StartInitTask */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartInitTask */
}

/* USER CODE BEGIN Header_StartTaskRadio */
/**
* @brief Function implementing the taskRadioLora thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskRadio */
void StartTaskRadio(void *argument)
{
  /* USER CODE BEGIN StartTaskRadio */
	master = 1;
	if (master == 1) {
		printf("Mode: Master\r\n");
		HAL_GPIO_WritePin(LED_TARGET_GPIO_Port, LED_TARGET_Pin, GPIO_PIN_RESET);
	} else {
		printf("Mode: Slave\r\n");
		HAL_GPIO_WritePin(LED_TARGET_GPIO_Port, LED_TARGET_Pin, GPIO_PIN_SET);
	}

	//initialize LoRa module
	SX1278_hw.dio0.port = LORA_DIO1_GPIO_Port;
	SX1278_hw.dio0.pin = LORA_DIO1_Pin;
	SX1278_hw.nss.port = LORA_NSS_GPIO_Port;
	SX1278_hw.nss.pin = LORA_NSS_Pin;
	SX1278_hw.reset.port = LORA_RST_GPIO_Port;
	SX1278_hw.reset.pin = LORA_RST_Pin;
	SX1278_hw.spi = &hspi1;

	SX1278.hw = &SX1278_hw;

	printf("Configuring LoRa module\r\n");
	//SX1278_init(&SX1278, 434000000, SX1278_POWER_17DBM, SX1278_LORA_SF_7,
	SX1278_init(&SX1278, 433000000, SX1278_POWER_17DBM, SX1278_LORA_SF_7,
	SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_DIS, 8);
	printf("Done configuring LoRaModule\r\n");

	if (master == 1) {
		ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
		HAL_GPIO_WritePin(LED_TARGET_GPIO_Port, LED_TARGET_Pin, GPIO_PIN_SET);
	} else {
		ret = SX1278_LoRaEntryRx(&SX1278, 5, 2000);
		HAL_GPIO_WritePin(LED_TARGET_GPIO_Port, LED_TARGET_Pin, GPIO_PIN_RESET);
	}
  /* Infinite loop */
  for(;;)
  {
	  if (master == 1) {
		printf("Master ...\r\n");
		osDelay(1000);
		printf("Sending package...\r\n");

		message_length = sprintf(buffer, "Hello %d", message);
		ret = SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
		printf("Entry: %d\r\n", ret);

		printf("Sending %s\r\n", buffer);
		ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) buffer,
				message_length, 2000);
		message += 1;

		printf("Transmission: %d\r\n", ret);
		printf("Package sent...\r\n");

	} else {
//		printf("Slave ...\r\n");
		osDelay(500);
//		printf("Receiving package...\r\n");

		ret = SX1278_LoRaRxPacket(&SX1278);
//		printf("Received: %d\r\n", ret);
		if (ret > 0) {
			SX1278_read(&SX1278, (uint8_t*) buffer, ret);
			printf("Content (%d): %s\r\n", ret, buffer);
			printf("Package received ...\r\n");
		}

	}

//	//change mode
//	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin)) {
//		printf("Changing mode\r\n");
//		master = ~master & 0x01;
//		if (master == 1) {
//			ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
//			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//		} else {
//			ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
//			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//		}
//		osDelay(1000);
//		while (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin))
//			;
//	}
  }
  /* USER CODE END StartTaskRadio */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

