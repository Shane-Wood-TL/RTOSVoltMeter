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
/* Definitions for matrixDriver */
osThreadId_t matrixDriverHandle;
uint32_t matrixDriverBuffer[ 128 ];
osStaticThreadDef_t matrixDriverControlBlock;
const osThreadAttr_t matrixDriver_attributes = {
  .name = "matrixDriver",
  .cb_mem = &matrixDriverControlBlock,
  .cb_size = sizeof(matrixDriverControlBlock),
  .stack_mem = &matrixDriverBuffer[0],
  .stack_size = sizeof(matrixDriverBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for spiTask */
osThreadId_t spiTaskHandle;
uint32_t spiTaskBuffer[ 256 ];
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
void startMatrixDriver(void *argument);
void startSpi(void *argument);
void StartDecrypter(void *argument);

/* USER CODE BEGIN PFP */
void decrypt (int32_t v[2], const uint32_t k[4]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void decrypt (int32_t v[2], const uint32_t k[4]) {
  /* set up; "sum" was computed from the value of delta
  in the "encrypt" function: sum = (delta << 5) & 0xFFFFFFFF */
  uint32_t v0=v[0], v1=v[1], sum=0xC6EF3720, i;
  uint32_t delta=0x9E3779B9;           /* a key schedule constant */
  uint32_t k0=k[0], k1=k[1], k2=k[2], k3=k[3];  /* cache key */
  for (i=0; i<32; i++) {             /* basic cycle start */
    v1 -= ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
    v0 -= ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
    sum -= delta;
  }                       /* end cycle */
  v[0]=v0; v[1]=v1;
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

  /* creation of matrixDriver */
  matrixDriverHandle = osThreadNew(startMatrixDriver, NULL, &matrixDriver_attributes);

  /* creation of spiTask */
  spiTaskHandle = osThreadNew(startSpi, NULL, &spiTask_attributes);

  /* creation of decrypter */
  decrypterHandle = osThreadNew(StartDecrypter, NULL, &decrypter_attributes);

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
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : VCP_TX_Pin */
  GPIO_InitStruct.Pin = VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA5 PA8 PA10
                           PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
  HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

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

/* USER CODE BEGIN Header_startMatrixDriver */
/**
 * @brief Function implementing the matrixDriver thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startMatrixDriver */
void startMatrixDriver(void *argument)
{
  /* USER CODE BEGIN startMatrixDriver */
	/* Infinite loop */

	//pixel data for 0-7 for rows 0-4
	const int8_t allDigits[10][5] = {
			{0x07, 0x05, 0x05, 0x05, 0x07},
			{0x02, 0x03, 0x02, 0x02, 0x07},
			{0x07, 0x04, 0x07, 0x01, 0x07},
			{0x07, 0x04, 0x06, 0x04, 0x07},
			{0x05, 0x05, 0x07, 0x04, 0x04},
			{0x07, 0x01, 0x07, 0x04, 0x07},
			{0x07, 0x01, 0x07, 0x05, 0x07},
			{0x07, 0x04, 0x04, 0x04, 0x04},
			{0x07, 0x05, 0x07, 0x05, 0x07},
			{0x07, 0x05, 0x07, 0x04, 0x07}
	};

	struct pin{
		GPIO_TypeDef *port;
		uint8_t number;
	};

	const struct pin cathodes[5] = {
			{GPIOA, 12}, //A3
			{GPIOA, 5}, //A4
			{GPIOA, 10}, //D0
			{GPIOB, 3}, //D13
			{GPIOB, 4} //D12

	};

	const struct pin anodes[7] = {
			{GPIOB, 0}, //D3
			{GPIOB, 7}, //D4
			{GPIOB, 6}, //D5
			{GPIOB, 1}, //D6
			{GPIOA, 8}, //D9
			{GPIOA, 11}, //D10
			{GPIOB, 5} //D11
	};
	for(;;)
	{

		uint32_t sum = 0;
		for(uint8_t value =0; value < 4; value++){
			uint16_t toWrite;
			osMessageQueueGet(decryptOutputHandle, &toWrite, NULL, pdMS_TO_TICKS(1000)); //get new data
			sum += toWrite;
		}
		sum = (uint32_t)(sum/4.0);
		uint8_t onesPlace = (uint8_t)(sum / 19859.0);
		uint8_t deciPlace = (uint8_t)((sum-(onesPlace*19859.0)) / 1986.0);
		for (uint8_t row = 0; row < 5; row++) {
			//replace PORTB = ~cathodes[row];, as the cathodes are on different ports
			//this display / setup must be somewhat different as the logic is inverted compared to last years
			//code; not sure, but the other part is inverted as well (diodes internal in the display are likely reversed
			//compared to the model used last year)
			for (uint8_t i = 0; i < 5; i++) {
				if (i == row){
					cathodes[i].port->ODR |= (1 << cathodes[i].number); //if this row is being looked at, set the pin
				}else{
					cathodes[i].port->ODR &= ~(1 << cathodes[i].number); //else, leave it low
				}
			}

			//get the value of the row being written to the display for the ones and tens place.
			uint16_t sum = allDigits[onesPlace][row] + (allDigits[deciPlace][row]<<4);

			//if in the bottom row, draw the dot
			if(row == 4){
				sum += 0x08;
			}

			// Set the anodes (columns) based on the row data
			for (uint8_t col = 0; col < 7; col++) {
				if (sum & (1 << col)) {
					anodes[col].port->ODR &= ~(1 << anodes[col].number);//bring the column low / turn the LED on
				} else {
					anodes[col].port->ODR |= (1 << anodes[col].number);//bring the column high / turn the LED off
				}
			}
			osDelay(1); //if there is not some delay here the screen gets angry
		}
	}

  /* USER CODE END startMatrixDriver */
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
  LL_SPI_Enable(hspi1.Instance);
  for(;;)
  {
	  osMutexAcquire(spiMutexHandle, osWaitForever);
	  __disable_irq();
	  uint8_t toRead[10] = {0}; //temp location for data to read
	  uint8_t rxData = 0;
	  if(!((GPIOA->IDR & (1<<12))))
	  {
		  for(int8_t i = 0; i < 10;)
		  {
			  while((GPIOA->IDR & (1<<12))) {}
			  rxData = LL_SPI_ReceiveData8(hspi1.Instance);
			  while(!LL_SPI_IsActiveFlag_RXNE(hspi1.Instance)){}
			  if((i==0) && (rxData != 255)){
				  i=0;
				  continue;
			  }
			  if (i == 9 && rxData != 0)
			  {
				  i=0;
				  continue;
			  }
			  toRead[i] = rxData;

//			  while(!LL_SPI_IsActiveFlag_RXNE(hspi1.Instance)){}
//			  LL_SPI_TransmitData8(hspi1.Instance, toRead[i]);
//			  while (!LL_SPI_IsActiveFlag_RXNE(hspi1.Instance)) {}
			  i++;
		  }
		  uint32_t test[2] = {0};
		  test[0] |= (toRead[1] << 24) | (toRead[2] << 16) | (toRead[3] << 8) | toRead[4];
		  test[1] |= (toRead[5] << 24) | (toRead[6] << 16) | (toRead[7] << 8) | toRead[8];
		  spiTaskBuffer[0] = test[0];
		  spiTaskBuffer[1] = test[1];
	  }
	  __enable_irq();
	  osMutexRelease(spiMutexHandle);
	  osDelay(1);
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
//	  	unPackage[0] = 0x55adfae8;
//	  	unPackage[1] = 0xecb1fa4;

	    //unPackage[0] = 0xc63341c8;
	    //unPackage[1] = 0x263397f4;

	  	osMutexRelease(spiMutexHandle);
	      //Decrypting
	  	int32_t toDecrypt[2] = {(uint32_t)unPackage[0], (uint32_t)unPackage[1]};
	  	decrypt(toDecrypt, k);

	  	//unpacking into 4 x 16-bit values
	  	values[0] = ((toDecrypt[0] << 16) & byteCapture);
	  	values[1] = ((toDecrypt[0]) & byteCapture);
	  	values[2] = ((toDecrypt[1] << 16) & byteCapture);
	  	values[3] = ((toDecrypt[1]) & byteCapture);

	  	for(uint8_t i = 0; i < 4; i++)
	  	{
	  		osMessageQueuePut(decryptOutputHandle, &values[i], 0, pdMS_TO_TICKS(1000));
	  	}
  }
  /* USER CODE END StartDecrypter */
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
