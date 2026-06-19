/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "rd_common.h"
#include "rd_define.h"

#include "rd_uart.h"
//#include "rd_map_dyn.h"      /* rd_comm_dyn.h 포함 */
#include "rd_comm_dpcb.h"
#include "rd_peripheral_dpcb.h"

#include "rd_control.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for periTask */
osThreadId_t periTaskHandle;
const osThreadAttr_t periTask_attributes = {
  .name = "periTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ctrlTask */
osThreadId_t ctrlTaskHandle;
const osThreadAttr_t ctrlTask_attributes = {
  .name = "ctrlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for rs485Task */
osThreadId_t rs485TaskHandle;
const osThreadAttr_t rs485Task_attributes = {
  .name = "rs485Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ResmapMutex */
osMutexId_t ResmapMutexHandle;
const osMutexAttr_t ResmapMutex_attributes = {
  .name = "ResmapMutex"
};
/* USER CODE BEGIN PV */

/*==========수신용 usart4번==========*/
UART_Ring_t DPCA_uart4 = {
    .rx_buffer = {0},           // 배열 초기화
    .head = 0,
    .tail = 0,
    .huart = &huart4,        // 미리 선언된 UART_HandleTypeDef

	.temp_buffer = {0},
	.rx_new = 0,
	.last_rx_tick = 0
};

/*==========수신용 패킷정의==========*/
PACKET_comm_t DPCA_PACKET;


/*==========페리페럴 초기화==========*/
PERIPHERAL_t DPCB_PERIPHERAL;

/*==========제어기 초기화==========*/
CONTROL_DPC_t DPC_CTL;


/*==========USART6 RS485 (Dynamixel)==========*/
UART_Ring_t DPCB_uart6 = {
    .huart      = &huart6,
    .is_running = 0
};
RS485_t DPCB_dyn = {
    .uart_obj = &DPCB_uart6
};

/*==========USART2 RS485 (communication)==========*/
UART_Ring_t DPCB_uart2 = {
    .huart      = &huart2,
    .is_running = 0
};
RS485_t RS485_comm = {
    .uart_obj = &DPCB_uart2
};

/*==========Dynamixel 컨트롤러==========*/
uint32_t Diff_tick;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void *argument);
void Start_comm_Task(void *argument);
void Start_peri_Task(void *argument);
void Start_ctrl_Task(void *argument);
void Startrs485(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_TIM14_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim5);

  /*==========COMM INIT==========*/
  if (RD_UART_INIT(&DPCA_uart4) != RET_OK) Error_Handler();
  RD_PACKET_INIT(&DPCA_PACKET);

  /*==========GPIO INIT==========*/
  RD_PERIPHERAL_INIT(&DPCB_PERIPHERAL);

  HAL_Delay(1000);

  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of ResmapMutex */
  ResmapMutexHandle = osMutexNew(&ResmapMutex_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(Start_comm_Task, NULL, &commTask_attributes);

  /* creation of periTask */
  periTaskHandle = osThreadNew(Start_peri_Task, NULL, &periTask_attributes);

  /* creation of ctrlTask */
  ctrlTaskHandle = osThreadNew(Start_ctrl_Task, NULL, &ctrlTask_attributes);

  /* creation of rs485Task */
  rs485TaskHandle = osThreadNew(Startrs485, NULL, &rs485Task_attributes);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 80-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 8-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 1000000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, EXIO_RST_Pin|LED_G_Pin|LED_R_Pin|RS485_DIR_Pin
                          |SOL_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIGHT_IO_GPIO_Port, LIGHT_IO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS485_EX_DIR_Pin|BOOT_IO_1_Pin|BOOT_IO_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EXIO_RST_Pin LED_G_Pin LED_R_Pin RS485_DIR_Pin
                           SOL_EN_Pin */
  GPIO_InitStruct.Pin = EXIO_RST_Pin|LED_G_Pin|LED_R_Pin|RS485_DIR_Pin
                          |SOL_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LIGHT_IO_Pin */
  GPIO_InitStruct.Pin = LIGHT_IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIGHT_IO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_EX_DIR_Pin BOOT_IO_1_Pin BOOT_IO_2_Pin */
  GPIO_InitStruct.Pin = RS485_EX_DIR_Pin|BOOT_IO_1_Pin|BOOT_IO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CON_A_Pin */
  GPIO_InitStruct.Pin = CON_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CON_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CON_B_Pin CON_C_Pin CON_D_Pin */
  GPIO_InitStruct.Pin = CON_B_Pin|CON_C_Pin|CON_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
	uint32_t id_num = 0;

	switch (DPC_CTL.STATE) {
		case 0:	//manual state
			id_num = 0;
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
			break;
		case 4:
			id_num = 3; //wait state
			break;
		case 10: // error state
			id_num = 0; //always off
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
			break;
		default:
			id_num = 5; //led work
			break;
	}

	for (int i=0; i<id_num; i++){
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		osDelay(100);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		osDelay(100);
	}
	osDelay(1000-(2*id_num*100));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_comm_Task */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_comm_Task */
void Start_comm_Task(void *argument)
{
  /* USER CODE BEGIN Start_comm_Task */
  /* Infinite loop */
  for(;;)
  {
	RD_PACKET_WRITE(&DPCA_uart4, &DPCA_PACKET); // send request first
    RD_RET comm_state = RET_WAIT;
    uint32_t comm_cnt = 0;
    while (comm_state != RET_OK && comm_cnt <= 10){
    	osDelay(1);
    	comm_state = RD_PACKET_READ(&DPCA_uart4, &DPCA_PACKET);
    	if (comm_state == RET_OK){
    		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    	}
    	else if(comm_state == RET_NOK) HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET); //이거 나중에 sys_state를 default task에서 관리하면서 인디케이팅 하는게 맞을듯
    	comm_cnt++;
    }
    osDelay(10);

  }
  /* USER CODE END Start_comm_Task */
}

/* USER CODE BEGIN Header_Start_peri_Task */
/**
* @brief Function implementing the periTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_peri_Task */
void Start_peri_Task(void *argument)
{
  /* USER CODE BEGIN Start_peri_Task */
  /* Infinite loop */
  for(;;)
  {
	uint32_t nowTick = HAL_GetTick();
	DPCB_PERIPHERAL.deltaTick = nowTick-DPCB_PERIPHERAL.oldTick;
	DPCB_PERIPHERAL.oldTick = nowTick;
	RD_PERIPHERAL_READ(&DPCB_PERIPHERAL);
	RD_PERIPHERAL_WRITE(&DPCB_PERIPHERAL);
	RD_DPCA_UPDATE(&DPCB_PERIPHERAL, &DPCA_PACKET);
    osDelay(10);
  }
  /* USER CODE END Start_peri_Task */
}

/* USER CODE BEGIN Header_Start_ctrl_Task */
/**
* @brief Function implementing the ctrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_ctrl_Task */
void Start_ctrl_Task(void *argument)
{
  /* USER CODE BEGIN Start_ctrl_Task */
  /* Infinite loop */
  for(;;)
  {
	RD_CONTROL_LOOP(&DPC_CTL, &DPCB_PERIPHERAL);
    osDelay(10);
  }
  /* USER CODE END Start_ctrl_Task */
}

/* USER CODE BEGIN Header_Startrs485 */
/**
* @brief Function implementing the rs485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startrs485 */
void Startrs485(void *argument)
{
  /* USER CODE BEGIN Startrs485 */
  //static const uint8_t DYN_IDS[DYN_NUM_MOTORS] = {2, 3, 4};

  /* ── 초기화 ──────────────────────────────────*/
  if (RD_RS485_INIT(&DPCB_dyn) != RET_OK) Error_Handler();

  for (int i = 0; i < DYN_NUM_MOTORS; i++) {
	  if (RD_DYN_INIT(&DPCB_PERIPHERAL.MOT[i].dyn_ctrl, DPCB_PERIPHERAL.MOT[i].DYN_IDS) != RET_OK) Error_Handler();
	  for (int j = 0; j < DYN_NUM_MOTORS; j++)
		  if (RD_DYN_INIT_SET(&DPCB_dyn, &DPCB_PERIPHERAL.MOT[i].dyn_ctrl) != RET_WAIT) break;
  }

  /********** Simple Example ********/
  for (int j = 0; j < 3; j++){
	for (int i = 0; i < DYN_NUM_MOTORS; i++) {
	DPCB_PERIPHERAL.MOT[i].dyn_ctrl.inst = INST_WRITE;  			      // Instruction set
	DPCB_PERIPHERAL.MOT[i].dyn_ctrl.addr.start  = DYN_ADDR_GOAL_CURRENT;  // Start Address set
	DPCB_PERIPHERAL.MOT[i].dyn_ctrl.addr.size   = DYN_SIZE_GOAL_CURRENT;  // Data Length set
	DPCB_PERIPHERAL.MOT[i].dyn_ctrl.ram.cmd.goal_current = 750;			  // Cmd data set. 2.69 [mA/U]
	RD_DYN_LOOP(&DPCB_dyn, &DPCB_PERIPHERAL.MOT[i].dyn_ctrl);  			  // return check 가능
	DPCB_PERIPHERAL.MOT[i].dyn_present_tick = DPCB_PERIPHERAL.MOT[i].dyn_ctrl.ram.state.realtime_tick;
	}
	osDelay(10);
	/*
	for (int i = 0; i < DYN_NUM_MOTORS; i++) {
	DPCB_PERIPHERAL.MOT[i].dyn_ctrl.inst = INST_WRITE;  			         // Instruction set
	DPCB_PERIPHERAL.MOT[i].dyn_ctrl.addr.start  = DYN_ADDR_POSITION_I_GAIN;  // Start Address set
	DPCB_PERIPHERAL.MOT[i].dyn_ctrl.addr.size   = DYN_SIZE_POSITION_I_GAIN;  // Data Length set
	DPCB_PERIPHERAL.MOT[i].dyn_ctrl.ram.cmd.goal_current = 100;			     // Position i gain.
	RD_DYN_LOOP(&DPCB_dyn, &DPCB_PERIPHERAL.MOT[i].dyn_ctrl);  			     // return check 가능
	DPCB_PERIPHERAL.MOT[i].dyn_present_tick = DPCB_PERIPHERAL.MOT[i].dyn_ctrl.ram.state.realtime_tick;
	}
	osDelay(10);
	*/
  }
  /**********************************/

  uint32_t tick_cnt = 0;
  for (;;)
  {
  /*====================================INF Loop BEGIN================================*/
	uint32_t start_tick = osKernelGetTickCount();
	for (int i = 0; i < DYN_NUM_MOTORS; i++)
	{
//      if (!dyn_ctrl[i].is_running) continue;
//    dyn_ctrl[i].ram.cmd.goal_position = target_pose[i];
	  if (++tick_cnt % 2 == 0) RD_DYN_UPDATE_STATE(&DPCB_PERIPHERAL.MOT[i].dyn_ctrl);
	  else {
		  if (RD_DYN_OPERATE_ON(&DPCB_PERIPHERAL.MOT[i].dyn_ctrl, DYN_MODE_CUR_POSITION) == RET_OK){
			  RD_DYN_UPDATE_CMD(&DPCB_PERIPHERAL.MOT[i].dyn_ctrl, DYN_MODE_CUR_POSITION);
		  }
	  }
	  RD_DYN_LOOP(&DPCB_dyn, &DPCB_PERIPHERAL.MOT[i].dyn_ctrl);
	  //add function
	  DPCB_PERIPHERAL.MOT[i].LPF_CURRENT =
			  DPCB_PERIPHERAL.MOT[i].LPF_CURRENT*0.95 + DPCB_PERIPHERAL.MOT[i].dyn_ctrl.ram.state.present_current*0.05;
	}
	Diff_tick = osKernelGetTickCount() - start_tick;
  /*======================INF Loop END=======================*/
  }

  /* USER CODE END Startrs485 */
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
  if (htim->Instance == TIM6)
  {
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
#ifdef USE_FULL_ASSERT
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
