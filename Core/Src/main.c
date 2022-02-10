/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//C LIBRARYS
#include "stdio.h"
#include "stdbool.h"
//USER LIBRARYS
#include "lcd_driver.h"
#include "wifi_driver.h"
#include "fatfs_sd.h"
#include "sd_device.h"
#include "logo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STORAGE_MOUNT		"/"
#define SYSTEM_STATE_FILE	"system_state.txt"

#define TIME_OFFSET			6*3600

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
const char SSID[]= "ae38ac";
const char PASSWORD[] = "273665629";

const char *main_menu[]={
		"Almacenamiento",
		"Brillo",
		"Musica",
		"Wi-Fi",
		"Bluetooth",
		"Herramientas",
		"Sistema",
};

weather_info_t info;
time_t date_time;

//SYSTEM TASK
//TASK HANDLE
TaskHandle_t screen_ctrl_master;

//QUEUE
xQueueHandle touch_point_queue = NULL;

//SEMAPHORE

//MUTEX
xSemaphoreHandle lcd_mutex= NULL;
xSemaphoreHandle lcd_control_show = NULL;
xSemaphoreHandle date_time_mutex = NULL;
xSemaphoreHandle wifi_control_mutex = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
//START
void print_logo();

//SD
bool show_file(char *file_name, uint8_t type_file);

//SYSTEM TASK
void check_batery_task(void *arg);
void screen_show_task(void *arg);
void wait_animation_task(void *arg);
void main_menu_show_task(void *arg);
void touch_screen_task(void *arg);

//SYSTEM
bool save_state(char *status);
//void set_time();
//void set_time_sync();

//WIFI TASK
void check_wifi_connection_task(void *arg);
void scan_wifi_task(void *arg);
void time_show_task(void *arg);

//SD TASK
void file_explorer_task(void *arg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void touch_read(touch_point_t *point)
{
	uint32_t adc_read_x=0;
	uint32_t adc_read_y=0;
	////////////////////////////Zread/////////////////////////////////
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);
	MX_ADC1_Init();
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
	MX_ADC2_Init();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

	adc_read_x = HAL_ADC_GetValue(&hadc1);
	adc_read_y = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop(&hadc2);

	point->pressure = adc_read_y-adc_read_x;

	HAL_ADC_DeInit(&hadc2);
	gpio_pin_mode(GPIOC, GPIO_PIN_4, 1);

	////////////////X READ//////////////////////////////////////////
	gpio_pin_mode(GPIOB, GPIO_PIN_1, 0);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);

	HAL_ADC_Start(&hadc1);
	for(uint8_t i=0; i<100; i++)
	{
		adc_read_x += HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	point->x_point = (adc_read_x/1700);

	HAL_ADC_DeInit(&hadc1);

	gpio_pin_mode(GPIOB, GPIO_PIN_1, 1);
	gpio_pin_mode(GPIOC, GPIO_PIN_3, 1);

	///////////////////YREAD////////////////////////////////////////
	gpio_pin_mode(GPIOA, GPIO_PIN_4, 0);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);

	MX_ADC2_Init();

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);

	HAL_ADC_Start(&hadc2);
	for(uint8_t i=0; i<100; i++)
	{
		adc_read_y += HAL_ADC_GetValue(&hadc2);
	}
	HAL_ADC_Stop(&hadc2);
	point->y_point = (adc_read_y/1200);

	HAL_ADC_DeInit(&hadc2);

	gpio_pin_mode(GPIOA, GPIO_PIN_4, 1);
	gpio_pin_mode(GPIOC, GPIO_PIN_4, 1);

}

void system_block_screen_by_tp_crtl()
{
	touch_point_t point;
	while(xQueueReceive(touch_point_queue, &point, portMAX_DELAY) == pdFALSE);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //LCD INIT__________________________________________________________________
  HAL_ADC_DeInit(&hadc1);
  HAL_ADC_DeInit(&hadc2);
  gpio_pin_mode(GPIOC, GPIO_PIN_4, 1);
  gpio_pin_mode(GPIOC, GPIO_PIN_3, 1);

  lcd_init();

  //SYSTEM INIT_______________________________________________________________
  print_logo();
  if(!init_sd(STORAGE_MOUNT))
  {
	  lcd_draw_string("SD FAILED...", strlen("SD FAILED..."), 0, 0, RED, font12);
	  lcd_draw_string("PRESS RESET...", strlen("PRESS RESET..."), 0, 12, YELLOW, font12);
	  while(1)
		  asm("nop");
  }

  lcd_draw_message_widget(100, 0, 1);
  //while(1);

  //INIT MUTEX________________________________________________________________
  lcd_mutex = xSemaphoreCreateMutex();
  lcd_control_show = xSemaphoreCreateMutex();
  date_time_mutex = xSemaphoreCreateMutex();
  wifi_control_mutex = xSemaphoreCreateMutex();

  //INIT QUEUE________________________________________________________________
  touch_point_queue = xQueueCreate(1, sizeof(touch_point_t));

  //INIT SYSTEM TASK__________________________________________________________
  xTaskCreate(&check_batery_task, "batery task", 512, NULL, 1, NULL);
  xTaskCreate(&check_wifi_connection_task, "wifi tasK", 1024, NULL, 2, NULL);
  xTaskCreate(&time_show_task, "time task", 1024, NULL, 1, NULL);

  xTaskCreate(&touch_screen_task, "touch task", 1024, NULL, 5, NULL);
  xTaskCreate(&screen_show_task, "master task", 512*8, NULL, 10, &screen_ctrl_master);

  vTaskStartScheduler();
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_MultiModeTypeDef multimode = {0};
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB13
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//INIT
void print_logo()
{
	lcd_clear(0x18e3);
	lcd_draw_image(jcios_logo, 32, 123, 175, 73);
}

//SD

bool show_file(char *file_name, type_file_t type)
{
	FIL *file = (FIL *)pvPortMalloc(sizeof(FIL));

	switch(type)
	{
		case txt:
		{
			char buffer_file[31];
			uint8_t y_pos=12;

			if(f_open(file, file_name, FA_READ) != FR_OK)
				return false;

			if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
			{
				lcd_set_window_color(0, 12, LCD_WIDTH, LCD_HEIGHT, BLACK);
				xSemaphoreGive(lcd_mutex);
			}

				while(f_gets(buffer_file, 31, file) != NULL)
				{
					if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
					{
						lcd_draw_string(buffer_file, strlen(buffer_file), 0, y_pos, WHITE, font12);
						y_pos +=12;
						xSemaphoreGive(lcd_mutex);
					}
				}
		}
		break;

		case bmp:
		{
			uint8_t buffer_file[3];
			uint size_read;
			int i;

			if(f_open(file, file_name, FA_READ) != FR_OK)
				return false;

			for(i=0; i<18; i++)
			{
				if(f_read(file, buffer_file, 3, &size_read) != FR_OK);
			}


			for(i= LCD_HEIGHT-1; i>=12; i--)
			{
				for(int j=0; j<LCD_WIDTH; j++)
				{
					f_read(file, buffer_file, 3, &size_read);
					if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
					{
						lcd_set_pixel(j, i, convert_from_rgb_8(buffer_file[2], buffer_file[1], buffer_file[0]));
						xSemaphoreGive(lcd_mutex);
					}
				}
			}


		}
		break;

		default:
			return false;
	}

	f_close(file);
	vPortFree(file);
	return true;
}

//SYSTEM TASK
void check_batery_task(void *arg)
{
	uint8_t batery_level=30;

	while(1)
	{
		if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
		{
			lcd_draw_batery_widget(LCD_WIDTH-25, 0, batery_level);
			xSemaphoreGive(lcd_mutex);
		}
		vTaskDelay(1000);
	}
}

void screen_show_task(void *arg)
{
	uint8_t actual_screen=0;
	uint8_t prev_screen = 0;

	while(1)
	{
		switch(actual_screen)
		{
		case 0:
			system_block_screen_by_tp_crtl();
			actual_screen = 4;
			break;

		case 4:
			prev_screen = actual_screen;
			xTaskCreate(&main_menu_show_task, "main menu", 1024, &actual_screen, 4, NULL);
			vTaskSuspend(NULL);
			break;

		case 5:
			prev_screen = actual_screen;
			xTaskCreate(&file_explorer_task, "file show", 1024*4, &actual_screen, 4, NULL);
			vTaskSuspend(NULL);
			break;

		case 6:
			prev_screen = actual_screen;
			//xTaskCreate(&system_constrast_task, "brillo task", 1024*2, &actual_screen, 4, NULL);
			vTaskSuspend(NULL);
			break;

		case 8:
			prev_screen = actual_screen;
			//xTaskCreate(wifi_scan_show_task, "wifi scan", 1024*2, &actual_screen, 4, NULL);
			vTaskSuspend(NULL);
			break;

		case 10:
			//system_block_screen_by_bt_crtl('b', 4);
			actual_screen = prev_screen;
			break;
		default:
			vTaskDelay(10);
			break;
		}
	}
}

void wait_animation_task(void *arg)
{
	uint8_t task_state = 1;

	while(task_state)
	{
		for(uint8_t i=0; i<=120; i++)
		{
			if(xSemaphoreTake(lcd_control_show, 1) == pdTRUE)
			{
				lcd_draw_circular_load_widget(LCD_WIDTH/2, LCD_HEIGHT/2,
						100, i, 24945, 41850);
				xSemaphoreGive(lcd_control_show);
			}
			else
			{
				task_state = 0;
				break;
			}

		}
	}

	vTaskDelete(NULL);
}

void main_menu_show_task(void *arg)
{
	uint8_t *next_state = arg;
	touch_point_t point;
	point.y_point = 20;

	if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
	{
		lcd_set_window_color(0, 12, LCD_WIDTH, LCD_HEIGHT, BLACK);

		for(uint8_t i=0; i<7; i++)
		{
			lcd_draw_string(main_menu[i], strlen(main_menu[i]),
					0, point.y_point, 2047, font20);
			point.y_point +=20;
		}

		xSemaphoreGive(lcd_mutex);
	}

	while(1)
	{
		if(xQueueReceive(touch_point_queue, &point, portMAX_DELAY) == pdTRUE)
		{
			if(point.y_point>20)
			{
				point.y_point = (point.y_point-20)/20;
				*next_state = 3+point.y_point;
				break;
			}
		}
	}


	vTaskResume(screen_ctrl_master);
	vTaskDelete(NULL);
}

void touch_screen_task(void *arg)
{
	touch_point_t point;

	while(1)
	{
		if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
		{
			touch_read(&point);
			xSemaphoreGive(lcd_mutex);
			if(point.pressure<3000)
				xQueueSend(touch_point_queue, &point, portMAX_DELAY);
		}

		vTaskDelay(100);
	}
}

//SYSTEM
bool save_state(char *status)
{
	FIL *file = (FIL *)pvPortMalloc(sizeof(FIL));


	if(f_open(file, SYSTEM_STATE_FILE, FA_WRITE) != FR_OK)
		return false;

	uint16_t size = strlen(status)+19;
	uint16_t len_write;
	char *buffer = (char *) pvPortMalloc(size);

	//sprintf(buffer, "%02d-%02d-%2d %02d:%02d:%02d->%s", gDate.Date, gDate.Month, 2000 + gDate.Year,
		//	gTime.Hours, gTime.Minutes, gTime.Seconds, status);

	if(f_write(file, buffer, size, &len_write) != FR_OK)
	{
		f_close(file);
		vPortFree(buffer);
		vPortFree(file);
		return false;
	}

	if(len_write<size)
	{
		f_close(file);
		vPortFree(buffer);
		vPortFree(file);
		return false;
	}

	f_close(file);
	vPortFree(buffer);
	vPortFree(file);

	return true;
}

pop_mail_t mail;
//WIFI TASK
void check_wifi_connection_task(void *arg)
{
	int8_t rssi;

	set_default_wifi();
	while(init_wifi(1000) != WiFi_OK)
	{
		save_state("Error init wifi");
		vTaskDelay(100);
	}

	while(connect_wifi(SSID, PASSWORD, WPAandWPA2, 100000) != WiFi_OK)
	{
		save_state("Connection lost");
		vTaskDelay(100);
	}

	//char message[10];
	//connect_pop("pop.gmail.com", "995", HAL_MAX_DELAY);
	//open_pop("esp32user@gmail.com", "#include<jossarr.h>", HAL_MAX_DELAY);
	//read_email_pop(17, &mail, HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart1, wifi.buffer_rx, strlen(wifi.buffer_rx), HAL_MAX_DELAY);

	connect_smtp("smtp.gmail.com", "465", 1000);
	open_smtp("esp32user@gmail.com", "#include<jossarr.h>", HAL_MAX_DELAY);
	send_email_smtp("esp32user@gmail.com", "thijoseph@hotmail.com", "jueves 23 de diciembre",
			"Prospero anio y felicidad!!!", HAL_MAX_DELAY);
	close_smtp(1000);

	connect_sntp("pool.ntp.org", "123", HAL_MAX_DELAY);
	if(xSemaphoreTake(date_time_mutex, portMAX_DELAY) == pdTRUE)
	{
		get_time_sntp(&date_time, HAL_MAX_DELAY);
		date_time -= TIME_OFFSET;
		HAL_TIM_Base_Start_IT(&htim2);
		xSemaphoreGive(date_time_mutex);
	}
	disconnect_sntp(HAL_MAX_DELAY);

	http_perform_as_stream_wheather(&info, "19.17,-99.41", HAL_MAX_DELAY);
	//http_perform_as_stream_time(&time, "America", "Mexico_City", HAL_MAX_DELAY);
	//set_time();
	/*
	set_broker_mqtt("ioticos.org", "80", user_name_password, HAL_MAX_DELAY);
	set_credentials_mqtt("jfLOMMKAr7q8KWA", "V3xn9eMkaIy42Sn", "JCIOS", HAL_MAX_DELAY);
	connect_broker_mqtt(HAL_MAX_DELAY);
	set_topic_publish("kcC60pTK0kSqtDm", HAL_MAX_DELAY);
	set_topic_susbcribe("kcC60pTK0kSqtDm", HAL_MAX_DELAY);
	send_message_mqtt("Hola", HAL_MAX_DELAY);
	disconnect_broker_mqtt(HAL_MAX_DELAY);
	*/

	//http_perform_as_stream_wheather(&info, "19.17,-99.41", HAL_MAX_DELAY);
	//http_perform_as_stream_time(&time, "America", "Mexico_City", HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart1, wifi.buffer_rx, strlen(wifi.buffer_rx), HAL_MAX_DELAY);
	while(1)
	{
		if(xSemaphoreTake(wifi_control_mutex, portMAX_DELAY) == pdTRUE)
			rssi = rssi_soft_ap(HAL_MAX_DELAY);

		if(rssi == 0)
		{
			save_state("Connection lost");
			while(init_wifi(1000) != WiFi_OK);
			connect_wifi(SSID, PASSWORD, WPAandWPA2, 10000);
		}
		if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
		{
			lcd_draw_wifi_signal_widget(LCD_WIDTH-30, 12, rssi);
			xSemaphoreGive(lcd_mutex);
		}
		vTaskDelay(1000);
	}
}

void time_show_task(void *arg)
{
	struct tm *ts;
	char buffer[15];


	while(1)
	{
		if(xSemaphoreTake(date_time_mutex, portMAX_DELAY) == pdTRUE)
		{
			ts = localtime(&date_time);
			strftime(buffer, 15, "%H:%M", ts);

			if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
			{
				lcd_set_window_color(0, 0, strlen(buffer)*8, 12, BLACK);
				lcd_draw_string(buffer, strlen(buffer), 0, 0, WHITE, font12);
				xSemaphoreGive(lcd_mutex);
			}

			xSemaphoreGive(date_time_mutex);
		}

		vTaskDelay(10000);
	}
}

//SD TASK
void file_explorer_task(void *arg)
{
	file_linked_list_t *list = NULL;
	file_linked_list_t *aux;
	uint8_t *next_state = arg;

	touch_point_t point;
	point.y_point = 20;

	int size = scan_directory(STORAGE_MOUNT, &list);

	if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
	{
		lcd_set_window_color(0, 12, LCD_WIDTH, LCD_HEIGHT, BLACK);

		if(size == -1)
			lcd_draw_string("SD NOT FOUND...", 15, 0, 20, RED, font20);

		else
		{
			aux = list;
			for(uint8_t i=0; i<size; i++)
			{
				lcd_draw_string(aux->name, strlen(aux->name), 0, point.y_point, 2047, font20);
				point.y_point +=20;
				aux = aux->next;
			}
		}

		xSemaphoreGive(lcd_mutex);
	}

	while(1)
	{
		if(xQueueReceive(touch_point_queue, &point, portMAX_DELAY) == pdTRUE)
		{
			if(point.y_point>20)
			{
				point.y_point = (point.y_point-20)/20;
				aux = list;
				for(uint8_t i=0; i<point.y_point; i++)
					aux = aux->next;

				show_file(aux->name, aux->type);
			}
		}
	}

	delete_file_list(&list, size);
	vTaskResume(screen_ctrl_master);
	vTaskDelete(NULL);

}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if(htim->Instance == TIM2)
	  date_time++;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
