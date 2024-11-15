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
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for adc */
osThreadId_t adcHandle;
uint32_t adcBuffer[ 128 ];
osStaticThreadDef_t adcControlBlock;
const osThreadAttr_t adc_attributes = {
  .name = "adc",
  .cb_mem = &adcControlBlock,
  .cb_size = sizeof(adcControlBlock),
  .stack_mem = &adcBuffer[0],
  .stack_size = sizeof(adcBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for encrypter */
osThreadId_t encrypterHandle;
uint32_t encrypterBuffer[ 128 ];
osStaticThreadDef_t encrypterControlBlock;
const osThreadAttr_t encrypter_attributes = {
  .name = "encrypter",
  .cb_mem = &encrypterControlBlock,
  .cb_size = sizeof(encrypterControlBlock),
  .stack_mem = &encrypterBuffer[0],
  .stack_size = sizeof(encrypterBuffer),
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
/* Definitions for enecryptOutput */
osMessageQueueId_t enecryptOutputHandle;
uint8_t enecryptOutputBuffer[ 8 * sizeof( uint8_t ) ];
osStaticMessageQDef_t enecryptOutputControlBlock;
const osMessageQueueAttr_t enecryptOutput_attributes = {
  .name = "enecryptOutput",
  .cb_mem = &enecryptOutputControlBlock,
  .cb_size = sizeof(enecryptOutputControlBlock),
  .mq_mem = &enecryptOutputBuffer,
  .mq_size = sizeof(enecryptOutputBuffer)
};
/* Definitions for adcOutputQueue1 */
osMessageQueueId_t adcOutputQueue1Handle;
uint8_t adcOutputQueue1Buffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t adcOutputQueue1ControlBlock;
const osMessageQueueAttr_t adcOutputQueue1_attributes = {
  .name = "adcOutputQueue1",
  .cb_mem = &adcOutputQueue1ControlBlock,
  .cb_size = sizeof(adcOutputQueue1ControlBlock),
  .mq_mem = &adcOutputQueue1Buffer,
  .mq_size = sizeof(adcOutputQueue1Buffer)
};
/* Definitions for adcOutputQueue0 */
osMessageQueueId_t adcOutputQueue0Handle;
uint8_t adcOutputQueueBuffer0[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t adcOutputQueueControlBlock0;
const osMessageQueueAttr_t adcOutputQueue0_attributes = {
  .name = "adcOutputQueue0",
  .cb_mem = &adcOutputQueueControlBlock0,
  .cb_size = sizeof(adcOutputQueueControlBlock0),
  .mq_mem = &adcOutputQueueBuffer0,
  .mq_size = sizeof(adcOutputQueueBuffer0)
};
/* Definitions for adcLock0 */
osMutexId_t adcLock0Handle;
osStaticMutexDef_t adcLockControlBlock;
const osMutexAttr_t adcLock0_attributes = {
  .name = "adcLock0",
  .cb_mem = &adcLockControlBlock,
  .cb_size = sizeof(adcLockControlBlock),
};
/* Definitions for enecryptLock */
osMutexId_t enecryptLockHandle;
osStaticMutexDef_t enecryptLockControlBlock;
const osMutexAttr_t enecryptLock_attributes = {
  .name = "enecryptLock",
  .cb_mem = &enecryptLockControlBlock,
  .cb_size = sizeof(enecryptLockControlBlock),
};
/* Definitions for spiMutex */
osMutexId_t spiMutexHandle;
osStaticMutexDef_t spiMutexControlBlock;
const osMutexAttr_t spiMutex_attributes = {
  .name = "spiMutex",
  .cb_mem = &spiMutexControlBlock,
  .cb_size = sizeof(spiMutexControlBlock),
};
/* Definitions for adcLock1 */
osMutexId_t adcLock1Handle;
osStaticMutexDef_t adcLock1ControlBlock;
const osMutexAttr_t adcLock1_attributes = {
  .name = "adcLock1",
  .cb_mem = &adcLock1ControlBlock,
  .cb_size = sizeof(adcLock1ControlBlock),
};
/* Definitions for adcSemaphore */
osSemaphoreId_t adcSemaphoreHandle;
osStaticSemaphoreDef_t adcSemaphoreControlBlock;
const osSemaphoreAttr_t adcSemaphore_attributes = {
  .name = "adcSemaphore",
  .cb_mem = &adcSemaphoreControlBlock,
  .cb_size = sizeof(adcSemaphoreControlBlock),
};
/* Definitions for enecryptSemaphore */
osSemaphoreId_t enecryptSemaphoreHandle;
osStaticSemaphoreDef_t enecryptSemaphoreControlBlock;
const osSemaphoreAttr_t enecryptSemaphore_attributes = {
  .name = "enecryptSemaphore",
  .cb_mem = &enecryptSemaphoreControlBlock,
  .cb_size = sizeof(enecryptSemaphoreControlBlock),
};
/* Definitions for spiSemaphore */
osSemaphoreId_t spiSemaphoreHandle;
osStaticSemaphoreDef_t spiSemaphoreControlBlock;
const osSemaphoreAttr_t spiSemaphore_attributes = {
  .name = "spiSemaphore",
  .cb_mem = &spiSemaphoreControlBlock,
  .cb_size = sizeof(spiSemaphoreControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartAdc(void *argument);
void startEncrypter(void *argument);
void startSpi(void *argument);

/* USER CODE BEGIN PFP */
void encrypt (uint32_t *v0I,uint32_t *v1I, const uint32_t k[4]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void encrypt (uint32_t *v0I,uint32_t *v1I, const uint32_t k[4]) {
  uint32_t v1 = *v1I, v0=*v0I;
  uint32_t sum=0, i;   /* set up */
  uint32_t delta=0x9E3779B9;             /* a key schedule constant */
  uint32_t k0=k[0], k1=k[1], k2=k[2], k3=k[3];  /* cache key */
  for (i=0; i<32; i++) {                 /* basic cycle start */
    sum += delta;
    v0 += ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
    v1 += ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
  }                       /* end cycle */
  (*v0I)= v0;
  (*v1I)= v1;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of adcLock0 */
  adcLock0Handle = osMutexNew(&adcLock0_attributes);

  /* creation of enecryptLock */
  enecryptLockHandle = osMutexNew(&enecryptLock_attributes);

  /* creation of spiMutex */
  spiMutexHandle = osMutexNew(&spiMutex_attributes);

  /* creation of adcLock1 */
  adcLock1Handle = osMutexNew(&adcLock1_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of adcSemaphore */
  adcSemaphoreHandle = osSemaphoreNew(1, 1, &adcSemaphore_attributes);

  /* creation of enecryptSemaphore */
  enecryptSemaphoreHandle = osSemaphoreNew(1, 0, &enecryptSemaphore_attributes);

  /* creation of spiSemaphore */
  spiSemaphoreHandle = osSemaphoreNew(1, 0, &spiSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of enecryptOutput */
  enecryptOutputHandle = osMessageQueueNew (8, sizeof(uint8_t), &enecryptOutput_attributes);

  /* creation of adcOutputQueue1 */
  adcOutputQueue1Handle = osMessageQueueNew (16, sizeof(uint16_t), &adcOutputQueue1_attributes);

  /* creation of adcOutputQueue0 */
  adcOutputQueue0Handle = osMessageQueueNew (16, sizeof(uint16_t), &adcOutputQueue0_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of adc */
  adcHandle = osThreadNew(StartAdc, NULL, &adc_attributes);

  /* creation of encrypter */
  encrypterHandle = osThreadNew(startEncrypter, NULL, &encrypter_attributes);

  /* creation of spiTask */
  spiTaskHandle = osThreadNew(startSpi, NULL, &spiTask_attributes);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/* USER CODE BEGIN Header_StartAdc */
/**
* @brief Function implementing the adc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdc */
void StartAdc(void *argument)
{
  /* USER CODE BEGIN StartAdc */

  /* Infinite loop */
  for(;;)
  {
	osMutexAcquire(adcLock0Handle, osWaitForever);
	for(uint8_t i = 0; i < 4; i++)
	{
	  (void)HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, osWaitForever);
	  adcOutputQueueBuffer0[i] = HAL_ADC_GetValue(&hadc1);
	}

	osMutexRelease(adcLock0Handle);
//	osSemaphoreRelease(adcOutputQueue0Handle);
	osMutexAcquire(adcLock1Handle, osWaitForever);
//	osSemaphoreAcquire(adcOutputQueue1Handle, 0);

	for(uint8_t i = 0; i < 4; i++)
	{
		(void)HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, osWaitForever);
		adcOutputQueue1Buffer[i] =  HAL_ADC_GetValue(&hadc1);
	}

	osMutexRelease(adcLock1Handle);
//	osSemaphoreRelease(adcOutputQueue1Handle);
//	osSemaphoreAcquire(adcOutputQueue0Handle, 0);


  }
  /* USER CODE END StartAdc */
}

/* USER CODE BEGIN Header_startEncrypter */
/**
* @brief Function implementing the encrypter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startEncrypter */
void startEncrypter(void *argument)
{
  /* USER CODE BEGIN startEncrypter */
  /* Infinite loop */
	uint16_t values[4] = {0,0,0,0}; //values from the adc queue
	const uint32_t k[4] = {371, 215, 11, 12}; //key
	uint32_t repackaged[2] = {0,0}; //2 32 bit values, packed version of values
	uint8_t toSend[8]; //encrypted version of repackaged
  for(;;)
  {
	  	  //lock buffer 1
		  	osMutexAcquire(adcLock1Handle, osWaitForever);

		  	//get the values from buffer 1
		  	for(uint8_t i = 0; i < 4; i++)
		  	{
		  		values[i] = adcOutputQueue1Buffer[i];
		  	}
		  	//release buffer 1
		    osMutexRelease(adcLock1Handle);

		    //repack into 2 32 bit values
		    repackaged[0]= values[0] << 16 | values[1];
		    repackaged[1]= values[2] << 16 | values[3];

		    //encrypt the data
		    encrypt(&repackaged[0],&repackaged[1],k);

		    //repack into 8 8 bit values
		    toSend[0] = (uint8_t)(repackaged[1] >> 24);
		    toSend[1] = (uint8_t)(repackaged[1] >> 16);
		    toSend[2] = (uint8_t)(repackaged[1] >> 8);
		    toSend[3] = (uint8_t)(repackaged[1] >> 0);
		    toSend[4] = (uint8_t)(repackaged[0] >> 24);
		    toSend[5] = (uint8_t)(repackaged[0] >> 16);
		    toSend[6] = (uint8_t)(repackaged[0] >> 8);
		    toSend[7] = (uint8_t)(repackaged[0] >> 0);

		    //enqueue the data
		    for(uint8_t i = 0; i < 8; i++){
		    		   osMessageQueuePut(enecryptOutputHandle,&toSend[i], 0, pdMS_TO_TICKS(10));
		    }

		    //lock buffer 0
		    osMutexAcquire(adcLock0Handle, osWaitForever);

		    //get the values from buffer 0
		  	for(uint8_t i = 0; i < 4; i++)
		  	{
		  		values[i] = adcOutputQueueBuffer0[i];
		  	}

		  	//release buffer 1
		    osMutexRelease(adcLock0Handle);

		    //repack into 2 32 bit values
		    repackaged[0]= values[0] << 16 | values[1];
		    repackaged[1]= values[2] << 16 | values[3];

		    //encrypt the data
		    encrypt(&repackaged[0],&repackaged[1],k);

		    //repack into 8 8 bit values
		    toSend[0] = (uint8_t)(repackaged[1] >> 24);
		    toSend[1] = (uint8_t)(repackaged[1] >> 16);
		    toSend[2] = (uint8_t)(repackaged[1] >> 8);
		    toSend[3] = (uint8_t)(repackaged[1] >> 0);
		    toSend[4] = (uint8_t)(repackaged[0] >> 24);
		    toSend[5] = (uint8_t)(repackaged[0] >> 16);
		    toSend[6] = (uint8_t)(repackaged[0] >> 8);
		    toSend[7] = (uint8_t)(repackaged[0] >> 0);

		    //enqueue the data
		    for(uint8_t i = 0; i < 8; i++){
		    	osMessageQueuePut(enecryptOutputHandle,&toSend[i], 0, pdMS_TO_TICKS(10));
		    }
	  }
  /* USER CODE END startEncrypter */
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
	    for(uint8_t i =0; i < 8; i++){ //send 8 bytes
	    	uint8_t toWrite; //temp location for data to write
	    	osMessageQueueGet(enecryptOutputHandle, &toWrite, NULL, pdMS_TO_TICKS(10)); //get new data
	    	GPIOB->ODR &= ~(1<<1);//Set CS low
			HAL_SPI_Transmit(&hspi1, &toWrite, 1, HAL_MAX_DELAY);// Send the data
			while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {} //wait till bus is clear
			GPIOB->ODR |= (1<<1); //set CS high
	    }
  }
  /* USER CODE END startSpi */
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
