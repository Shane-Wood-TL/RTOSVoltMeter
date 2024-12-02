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
#include "stm32l4xx_ll_adc.h"
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
uint8_t adcOutputQueue1Buffer[ 4 * sizeof( uint16_t ) ];
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
uint8_t adcOutputQueueBuffer0[ 4 * sizeof( uint16_t ) ];
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
  adcOutputQueue1Handle = osMessageQueueNew (4, sizeof(uint16_t), &adcOutputQueue1_attributes);

  /* creation of adcOutputQueue0 */
  adcOutputQueue0Handle = osMessageQueueNew (4, sizeof(uint16_t), &adcOutputQueue0_attributes);

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

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration
  PB0   ------> ADC1_IN15
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_15);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_15, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_15, LL_ADC_SINGLE_ENDED);
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA1   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
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
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1|LD3_Pin);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	//Enabling ADC 1
	LL_ADC_Enable(ADC1);

  /* Infinite loop */
  for(;;)
  {
	//Init holders for the ADC readings
	uint16_t holder1[4] = {0};
	uint16_t holder2[4] = {0};

	uint16_t byte = 0x00ff;

	//Lock out buffer0
	(void*)osMutexAcquire(adcLock0Handle, osWaitForever);

	for(uint8_t i = 0; i < 4; i++)
	{
	  //Start ADC conversion for each reading
	  LL_ADC_REG_StartConversion(ADC1);
	  //Wait until the conversion process has started
	  while(LL_ADC_REG_IsConversionOngoing(ADC1)) {}

	  //Assign the 12 bit value to the "holder" buffer
	  holder1[i] = LL_ADC_REG_ReadConversionData12(ADC1);

	  //Stop the conversion
	  LL_ADC_REG_StopConversion(ADC1);
	  //Wait until the conversion has stopped
	  while(LL_ADC_REG_IsStopConversionOngoing(ADC1)) {}

	}

	//breaking up the 4 16-bit values in the holder1 buffer to 8 8-bit values to store into "adcOutputQueueBuffer0"
	adcOutputQueueBuffer0[0] = (holder1[0] >> 8) & byte;
	adcOutputQueueBuffer0[1] = (holder1[0]) & byte;
	adcOutputQueueBuffer0[2] = (holder1[1] >> 8) & byte;
	adcOutputQueueBuffer0[3] = (holder1[1]) & byte;
	adcOutputQueueBuffer0[4] = (holder1[2] >> 8) & byte;
	adcOutputQueueBuffer0[5] = (holder1[2]) & byte;
	adcOutputQueueBuffer0[6] = (holder1[3] >> 8) & byte;
	adcOutputQueueBuffer0[7] = (holder1[3]) & byte;

	//Unlock buffer 0
	(void*)osMutexRelease(adcLock0Handle);

	//Lock out buffer 1
	(void*)osMutexAcquire(adcLock1Handle, osWaitForever);

	for(uint8_t i = 0; i < 4; i++)
	{
		  //Start ADC conversion for each reading
		  LL_ADC_REG_StartConversion(ADC1);
		  //Wait until the conversion process has started
		  while(LL_ADC_REG_IsConversionOngoing(ADC1)) {}

		  //Assign the 12 bit value to the "holder" buffer
		  holder2[i] = LL_ADC_REG_ReadConversionData12(ADC1);

		  //Stop the conversion
		  LL_ADC_REG_StopConversion(ADC1);
		  //Wait until the conversion has stopped
		  while(LL_ADC_REG_IsStopConversionOngoing(ADC1)) {}
	}

	//breaking up the 4 16-bit values in the holder2 buffer to 8 8-bit values to store into "adcOutputQueueBuffer1"
	adcOutputQueue1Buffer[0] = (holder2[0] >> 8) & byte;
	adcOutputQueue1Buffer[1] = (holder2[0]) & byte;
	adcOutputQueue1Buffer[2] = (holder2[1] >> 8) & byte;
	adcOutputQueue1Buffer[3] = (holder2[1]) & byte;
	adcOutputQueue1Buffer[4] = (holder2[2] >> 8) & byte;
	adcOutputQueue1Buffer[5] = (holder2[2]) & byte;
	adcOutputQueue1Buffer[6] = (holder2[3] >> 8) & byte;
	adcOutputQueue1Buffer[7] = (holder2[3]) & byte;

	//Unlock buffer 1
	(void*)osMutexRelease(adcLock1Handle);



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
	uint32_t repackaged[2] = {0,0}; //2 32 bit values, packed version of values
	uint16_t values[8] = {0}; //values from the adc queue
	const uint32_t k[4] = {371, 215, 11, 12}; //key
//	uint32_t repackaged[2] = {0,0}; //2 32 bit values, packed version of values
	uint8_t toSend[8]; //encrypted version of repackaged
  for(;;)
  {
	  	  repackaged[0] = 0;
	  	  repackaged[1] = 0;
	  	  //lock buffer 1

	  	//Claim mutex1
	  	(void*)osMutexAcquire(adcLock1Handle, osWaitForever);
		  	//get the values from buffer 1
		  	for(uint8_t i = 0; i < 8; i++)
		  	{
		  		values[i] = adcOutputQueue1Buffer[i];
		  	}
		  	//release buffer 1
		  	(void*)osMutexRelease(adcLock1Handle);

		    //repack into 2 32 bit values
		  	//Repackage data from buffer1 into an array of 2 x uint32_t
		    repackaged[0]= values[0] << 24 | values[1] << 16 | values[2] << 8 | values[3];
		    repackaged[1]= values[4] << 24 | values[5] << 16 | values[6] << 8 | values[7];

		    //encrypt the data
		    //Encrypt with keys (371,215,11,12)
		    encrypt(&repackaged[0],&repackaged[1],k);

		    //repack into 8 8 bit values
		    //Repackage obscured data as 8 x uint8t_
		    toSend[0] = (uint8_t)(repackaged[0] >> 24);
		    toSend[1] = (uint8_t)(repackaged[0] >> 16);
		    toSend[2] = (uint8_t)(repackaged[0] >> 8);
		    toSend[3] = (uint8_t)(repackaged[0] >> 0);
		    toSend[4] = (uint8_t)(repackaged[1] >> 24);
		    toSend[5] = (uint8_t)(repackaged[1] >> 16);
		    toSend[6] = (uint8_t)(repackaged[1] >> 8);
		    toSend[7] = (uint8_t)(repackaged[1] >> 0);

		    //enqueue the data

		    for(uint8_t i = 0; i < 8; i++){
		    	(void*)osMessageQueuePut(enecryptOutputHandle,&toSend[i], 0, pdMS_TO_TICKS(1000));
		    }
		    //lock buffer 0
		  	//Claim mutex0
		    (void*)osMutexAcquire(adcLock0Handle, osWaitForever);
		    //get the values from buffer 0
		  	for(uint8_t i = 0; i < 8; i++)
		  	{
		  		values[i] = adcOutputQueueBuffer0[i];
		  	}

		  	//release buffer 1
		  	(void*)osMutexRelease(adcLock0Handle);

		    //repack into 2 32 bit values
		    repackaged[0] = 0;
		    repackaged[1] = 0;

		    //Repackage data from buffer1 into an array of 2 x uint32_t
		    repackaged[0]= values[0] << 24 | values[1] << 16 | values[2] << 8 | values[3];
		    repackaged[1]= values[4] << 24 | values[5] << 16 | values[6] << 8 | values[7];

		    //encrypt the data
		    //Encrypt with keys (371,215,11,12)
		    encrypt(&repackaged[0],&repackaged[1],k);

		    //repack into 8 8 bit values
		    //Repackage obscured data as 8 x uint8t_
		    toSend[0] = (uint8_t)(repackaged[0] >> 24);
		    toSend[1] = (uint8_t)(repackaged[0] >> 16);
		    toSend[2] = (uint8_t)(repackaged[0] >> 8);
		    toSend[3] = (uint8_t)(repackaged[0] >> 0);
		    toSend[4] = (uint8_t)(repackaged[1] >> 24);
		    toSend[5] = (uint8_t)(repackaged[1] >> 16);
		    toSend[6] = (uint8_t)(repackaged[1] >> 8);
		    toSend[7] = (uint8_t)(repackaged[1] >> 0);

		    //enqueue the data
		    for(uint8_t i = 0; i < 8; i++){
		    	(void*)osMessageQueuePut(enecryptOutputHandle,&toSend[i], 0, pdMS_TO_TICKS(1000));
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
	//temp location for data to write
	uint8_t currentValue = 0;
	LL_SPI_Enable(SPI1);
  for(;;)
  {
	  uint8_t toWrite[10] = {255};
	    for(uint8_t i =1; i < 9; i++){ //send 8 bytes
	    	(void*)osMessageQueueGet(enecryptOutputHandle, &currentValue, NULL, pdMS_TO_TICKS(1000)); //get new data
	    	toWrite[i] = currentValue;
	    }
	    /*
	    Use HAL to xmit out SPI this way:
	    GPIO assert low (as ~SS)
	    Transmit all 8 bytes
	    GPIO assert ~SS high
		*/
	    for(uint8_t i=0; i<10; i++){
				GPIOB->ODR &= ~(1<<1);//Set CS low
				while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {} //wait for spi transmit buffer to be empty
				LL_SPI_TransmitData8(SPI1, toWrite[i]); //send the data
				while (LL_SPI_IsActiveFlag_BSY(SPI1)) {} //wait for spi to no longer be busy
				GPIOB->ODR |= (1<<1); //set CS high
	    }

	    //change CS for each byte
	    //send data as 255 x x x x _ x x x x 0
	    //start byte is 255
	    //stop byte is 0



  }
  /* USER CODE END startSpi */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
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
