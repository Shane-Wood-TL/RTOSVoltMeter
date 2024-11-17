/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_ll_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for spiTask */
osThreadId_t spiTaskHandle;
uint32_t spiTaskBuffer[ 128 ];
osStaticThreadDef_t spiTaskControlBlock;
const osThreadAttr_t spiTask_attributes = {
  .name = "spiTask",
  .cb_mem = &spiTaskControlBlock,
  .cb_size = sizeof(spiTaskControlBlock),
  .stack_mem = &spiTaskBuffer[0],
  .stack_size = sizeof(spiTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for decrypter */
osThreadId_t decrypterHandle;
uint32_t decrypterBuffer[ 128 ];
osStaticThreadDef_t decrypterControlBlock;
const osThreadAttr_t decrypter_attributes = {
  .name = "decrypter",
  .cb_mem = &decrypterControlBlock,
  .cb_size = sizeof(decrypterControlBlock),
  .stack_mem = &decrypterBuffer[0],
  .stack_size = sizeof(decrypterBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for display */
osThreadId_t displayHandle;
uint32_t displayBuffer[ 128 ];
osStaticThreadDef_t displayControlBlock;
const osThreadAttr_t display_attributes = {
  .name = "display",
  .cb_mem = &displayControlBlock,
  .cb_size = sizeof(displayControlBlock),
  .stack_mem = &displayBuffer[0],
  .stack_size = sizeof(displayBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for decryptOutput */
osMessageQueueId_t decryptOutputHandle;
uint8_t decryptOutputBuffer[ 4 * sizeof( uint16_t ) ];
osStaticMessageQDef_t decryptOutputControlBlock;
const osMessageQueueAttr_t decryptOutput_attributes = {
  .name = "decryptOutput",
  .cb_mem = &decryptOutputControlBlock,
  .cb_size = sizeof(decryptOutputControlBlock),
  .mq_mem = &decryptOutputBuffer,
  .mq_size = sizeof(decryptOutputBuffer)
};
/* Definitions for spiOutput */
osMessageQueueId_t spiOutputHandle;
uint8_t spiOutputBuffer[ 8 * sizeof( uint32_t ) ];
osStaticMessageQDef_t spiOutputControlBlock;
const osMessageQueueAttr_t spiOutput_attributes = {
  .name = "spiOutput",
  .cb_mem = &spiOutputControlBlock,
  .cb_size = sizeof(spiOutputControlBlock),
  .mq_mem = &spiOutputBuffer,
  .mq_size = sizeof(spiOutputBuffer)
};
/* Definitions for spiMutex */
osMutexId_t spiMutexHandle;
osStaticMutexDef_t spiMutexControlBlock;
const osMutexAttr_t spiMutex_attributes = {
  .name = "spiMutex",
  .cb_mem = &spiMutexControlBlock,
  .cb_size = sizeof(spiMutexControlBlock),
};
/* Definitions for decryptLock */
osMutexId_t decryptLockHandle;
osStaticMutexDef_t decryptLockControlBlock;
const osMutexAttr_t decryptLock_attributes = {
  .name = "decryptLock",
  .cb_mem = &decryptLockControlBlock,
  .cb_size = sizeof(decryptLockControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void startSpi(void *argument);
void StartDecrypter(void *argument);
void StartDisplay(void *argument);

/* USER CODE BEGIN PFP */
void decrypt (uint32_t *v0I,uint32_t *v1I, const uint32_t k[4]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void decrypt(uint32_t *v0I, uint32_t *v1I, const uint32_t k[4])
{
	uint32_t v0 = *v0I;
	uint32_t v1 = *v1I;
	uint32_t sum=0xC6EF3720;
    uint32_t delta=0x9E3779B9; /* a key schedule constant */
    uint32_t k0=k[0], k1=k[1], k2=k[2], k3=k[3];  /* cache key */
    for (uint32_t i=0; i<32; i++)
    {             /* basic cycle start */
        v1 -= ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
        v0 -= ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
        sum -= delta;
    }                       /* end cycle */
    (*v0I)=v0;
    (*v1I)=v1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of spiMutex */
  spiMutexHandle = osMutexNew(&spiMutex_attributes);

  /* creation of decryptLock */
  decryptLockHandle = osMutexNew(&decryptLock_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of decryptOutput */
  decryptOutputHandle = osMessageQueueNew (4, sizeof(uint16_t), &decryptOutput_attributes);

  /* creation of spiOutput */
  spiOutputHandle = osMessageQueueNew (8, sizeof(uint32_t), &spiOutput_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of spiTask */
  spiTaskHandle = osThreadNew(startSpi, NULL, &spiTask_attributes);

  /* creation of decrypter */
  decrypterHandle = osThreadNew(StartDecrypter, NULL, &decrypter_attributes);

  /* creation of display */
  displayHandle = osThreadNew(StartDisplay, NULL, &display_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : SPI_Select_Pin */
  GPIO_InitStruct.Pin = SPI_Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SPI_Select_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startSpi */
/**
* @brief Function implementing the spiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startSpi */
void startSpi(void *argument)
{
  /* USER CODE BEGIN startSpi */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(spiMutexHandle, osWaitForever);
	  uint8_t toRead[8]; //temp location for data to read

	  if(!((GPIOB->IDR & (1<<1))))
	  {
		  for(uint8_t i = 0; i < 8; i++)
		  {
			  uint8_t rxData = 0;
			  HAL_SPI_Receive(&hspi1, &rxData, 1, HAL_MAX_DELAY);
			  toRead[i] = rxData;
		  }

		  spiTaskBuffer[0] |= (toRead[0] << 24) | (toRead[1] << 16) | (toRead[2] << 8) | toRead[3];
		  spiTaskBuffer[1] |= (toRead[4] << 24) | (toRead[5] << 16) | (toRead[6] << 8) | toRead[7];
	  }
	  osMutexRelease(spiMutexHandle);
  }
  /* USER CODE END startSpi */
}

/* USER CODE BEGIN Header_StartDecrypter */
/**
* @brief Function implementing the decrypter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDecrypter */
void StartDecrypter(void *argument)
{
  /* USER CODE BEGIN StartDecrypter */
	uint32_t unPackage[2];
    uint32_t byteCapture = 0x0000ffff;
	uint16_t values[4] = {0,0,0,0}; //values from the adc queue
	const uint32_t k[4] = {371, 215, 11, 12}; //key
  /* Infinite loop */
  for(;;)
  {

	osMutexAcquire(spiMutexHandle, osWaitForever);
	unPackage[0] = spiTaskBuffer[0];
	unPackage[1] = spiTaskBuffer[1];

	osMutexRelease(spiMutexHandle);

    //Decrypting
	decrypt(&unPackage[0], &unPackage[1], k);

	//unpacking into 4 x 16-bit values
	values[0] = ((unPackage[0] << 16) & byteCapture);
	values[1] = ((unPackage[0]) & byteCapture);
	values[2] = ((unPackage[1] << 16) & byteCapture);
	values[3] = ((unPackage[1]) & byteCapture);

	for(uint8_t i = 0; i < 4; i++)
	{
		osMessageQueuePut(decryptOutputHandle, &values[i], 0, pdMS_TO_TICKS(10));
	}
  }
  /* USER CODE END StartDecrypter */
}

/* USER CODE BEGIN Header_StartDisplay */
/**
* @brief Function implementing the display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplay */
void StartDisplay(void *argument)
{
  /* USER CODE BEGIN StartDisplay */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDisplay */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
