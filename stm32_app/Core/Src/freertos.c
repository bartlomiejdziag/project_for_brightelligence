/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include "usart.h"
#include "../../Drivers/BSP/stm32746g_discovery_lcd.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "fatfs.h"
#include "ring_buffer.h"
#include "parse_simple.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct eth_packets {
	uint32_t counter;
	uint32_t random_data;
} eth_pkt_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UDPECHO_THREAD_PRIO  ( tskIDLE_PRIORITY + 4 )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
RingBuffer_t ReceiveBuffer; // Ring Buffer for Receiving from UART
uint8_t ReceiveTmp; // Temporary variable for receiving one byte
volatile uint8_t ReceviedLines; // Complete lines counter

uint8_t ReceivedData[32]; // A buffer for parsing
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for udp_task */
osThreadId_t udp_taskHandle;
const osThreadAttr_t udp_task_attributes = {
  .name = "udp_task",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for sd_card_task */
osThreadId_t sd_card_taskHandle;
const osThreadAttr_t sd_card_task_attributes = {
  .name = "sd_card_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for eth_queue */
osMessageQueueId_t eth_queueHandle;
const osMessageQueueAttr_t eth_queue_attributes = {
  .name = "eth_queue"
};
/* Definitions for retriveSemaphore */
osSemaphoreId_t retriveSemaphoreHandle;
const osSemaphoreAttr_t retriveSemaphore_attributes = {
  .name = "retriveSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void LCD_Config(void);
void udpecho_thread(void const *argument);
void udpecho_init(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void udp_thread(void *argument);
void sdcard_thread(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	HAL_UART_Receive_IT(&huart1, &ReceiveTmp, 1);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of retriveSemaphore */
  retriveSemaphoreHandle = osSemaphoreNew(1, 0, &retriveSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of eth_queue */
  eth_queueHandle = osMessageQueueNew (128, sizeof(eth_pkt_t), &eth_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of udp_task */
//  udp_taskHandle = osThreadNew(udp_thread, NULL, &udp_task_attributes);

  /* creation of sd_card_task */
  sd_card_taskHandle = osThreadNew(sdcard_thread, NULL, &sd_card_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
	LCD_Config();
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	udp_taskHandle = osThreadNew(udp_thread, NULL, &udp_task_attributes);

	char *StartMessage = "Starting\n";
	char *RetrieveMessage = "Retrieve\n";
	char *CompareMessage = "Compare\n";
	BSP_LCD_DisplayStringAt(0, 136, (uint8_t*) "PRESS BLUE BUTTON",
			CENTER_MODE);
	/* Infinite loop */
	for (;;) {
		switch (blue_button) {
		case STARTING:
			HAL_UART_Transmit(&huart1, (uint8_t*) StartMessage,
					strlen(StartMessage), 1000);
			BSP_LCD_Clear(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(0, 136, (uint8_t*) "START", CENTER_MODE);
			blue_button = WAITING;
			break;
		case RETRIVE:
			HAL_UART_Transmit(&huart1, (uint8_t*) RetrieveMessage,
					strlen(RetrieveMessage), 1000);
			BSP_LCD_Clear(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(0, 136, (uint8_t*) "RETRIEVE", CENTER_MODE);
			osSemaphoreRelease(retriveSemaphoreHandle);
			blue_button = WAITING;
			break;
		case COMPARE:
			HAL_UART_Transmit(&huart1, (uint8_t*) CompareMessage,
					strlen(CompareMessage), 1000);
			BSP_LCD_Clear(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(0, 136, (uint8_t*) "COMPARE", CENTER_MODE);
			blue_button = WAITING;
			break;
		case WAITING:
			break;
		default:
			break;
		}

		  // Check if there is something to parse - any complete line
		if (ReceviedLines > 0) {
			// Take one line from the Ring Buffer to work-buffer
			Parser_TakeLine(&ReceiveBuffer, ReceivedData);
			// Decrement complete lines counter
			ReceviedLines--;
			Parser_Parse(ReceivedData);
		}
		osDelay(200);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_udp_thread */
/**
 * @brief Function implementing the udp_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_udp_thread */
void udp_thread(void *argument)
{
  /* USER CODE BEGIN udp_thread */
	struct netconn *conn;
	struct netbuf *buf;
	err_t err, recv_err;
	eth_pkt_t *recv_packets_ptr;
	eth_pkt_t recv_packets = { 0 };

	LWIP_UNUSED_ARG(argument);

	conn = netconn_new(NETCONN_UDP);
	if (conn != NULL) {
		err = netconn_bind(conn, IP_ADDR_ANY, 8888);
		if (err == ERR_OK) {
			while (1) {
				recv_err = netconn_recv(conn, &buf);

				if (recv_err == ERR_OK) {
					recv_packets_ptr = (eth_pkt_t*) buf->p->payload;
					recv_packets.counter = recv_packets_ptr->counter;
					recv_packets.random_data = recv_packets_ptr->random_data;
					osMessageQueuePut(eth_queueHandle, &recv_packets, 0, 1);
					netbuf_delete(buf);
				}
			}
		} else {
			netconn_delete(conn);
		}
	}
  /* USER CODE END udp_thread */
}

/* USER CODE BEGIN Header_sdcard_thread */
/**
 * @brief Function implementing the sd_card_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_sdcard_thread */
void sdcard_thread(void *argument)
{
  /* USER CODE BEGIN sdcard_thread */
	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten, bytesread; /* File write counts */
	uint8_t rtext[930];/* File read buffer */
	eth_pkt_t ethernet_data = { 0 };
	char msg[128];
	osStatus_t status;

	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
		Error_Handler();
	} else {
		if ((res = f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, rtext,
				sizeof(rtext))) != FR_OK) {
			Error_Handler();
		} else {
			//Open file for writing (Create)
			if (f_open(&SDFile, "STM32.TXT",
					FA_OPEN_APPEND | FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
				Error_Handler();
			}
		}
	}
	/* Infinite loop */
	for (;;) {
		if (osOK == osMessageQueueGet(eth_queueHandle, &ethernet_data, NULL, 1000)) {
			sprintf(msg, "%lu, %lu\n", ethernet_data.counter, ethernet_data.random_data);
			res = f_write(&SDFile, msg, strlen(msg), (void*) &byteswritten);
			if ((byteswritten == 0) || (res != FR_OK)) {
				Error_Handler();
			}
			f_sync(&SDFile);
		} else {
			status = osSemaphoreAcquire(retriveSemaphoreHandle, 0);
			if (status == osOK) {
				//			HAL_UART_Transmit(&huart1, "working", strlen("working"), 1000);
				f_close(&SDFile);

				if (f_open(&SDFile, "STM32.TXT", FA_READ) != FR_OK) {
					Error_Handler();
				}

				/*##-8- Read data from the text file ###########################*/
				res = f_read(&SDFile, rtext, sizeof(rtext), (UINT*) &bytesread);

				if ((bytesread == 0) || (res != FR_OK)) {
					/* 'STM32.TXT' file Read or EOF Error */
					Error_Handler();
				}
				HAL_UART_Transmit(&huart1, (uint8_t*) rtext, sizeof(rtext), 1000);
				HAL_UART_Transmit(&huart1, (uint8_t*)"End\n", strlen("End\n"), 1000);
				f_close(&SDFile);
			}
		}
	}
  /* USER CODE END sdcard_thread */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void LCD_Config(void) {
	/* LCD Initialization */
	BSP_LCD_Init();

	/* LCD Initialization */
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_LayerDefaultInit(1,
			LCD_FB_START_ADDRESS
					+ (BSP_LCD_GetXSize() * BSP_LCD_GetYSize() * 4));

	/* Enable the LCD */
	BSP_LCD_DisplayOn();

	/* Select the LCD Background Layer  */
	BSP_LCD_SelectLayer(0);

	/* Clear the Background Layer */
	BSP_LCD_Clear(LCD_COLOR_WHITE);

	/* Select the LCD Foreground Layer  */
	BSP_LCD_SelectLayer(1);

	/* Clear the Foreground Layer */
	BSP_LCD_Clear(LCD_COLOR_WHITE);

	/* Configure the transparency for foreground and background :
	 Increase the transparency */
	BSP_LCD_SetTransparency(0, 0);
	BSP_LCD_SetTransparency(1, 255);

	BSP_LCD_SetFont(&Font24);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1) {
		// Try to write into the Ring Buffer
		if(RB_OK == RB_Write(&ReceiveBuffer, ReceiveTmp))
		{
			// Check if current byte is the endline char
			if(ReceiveTmp == ENDLINE)
			{
				// Increment complete lines
				ReceviedLines++;
			}
		}
		// Start to listening UART again - important!
//		HAL_UART_Transmit(&huart1, &ReceiveTmp, 1, 0);
		HAL_UART_Receive_IT(&huart1, &ReceiveTmp, 1);

	}
}
/* USER CODE END Application */

