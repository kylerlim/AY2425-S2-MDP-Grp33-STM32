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
#include "oled.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CMD_MAX_LENGTH  5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t pwmVal_servo;
uint16_t SERVO_STRAIGHT = 146;
uint16_t SERVO_LEFT = 72;
uint16_t SERVO_RIGHT = 186; // original at 220
uint16_t DC_RIGHT = 2100;
uint16_t DC_LEFT = 1670;


uint16_t ANGLE_TESTING_PHASE = 0;

int e_brake = 0;
int times_acceptable = 0;
volatile int user_distance = 0;
int readyToExecute = 0;


// Ultrasonic sensor declarations
uint16_t echo = 0;       // To hold the echo pulse width in microseconds
uint16_t tc1, tc2;
uint8_t Is_First_Captured = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for USART3Rx */
osThreadId_t USART3RxHandle;
const osThreadAttr_t USART3Rx_attributes = {
  .name = "USART3Rx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for turnCheck */
osThreadId_t turnCheckHandle;
const osThreadAttr_t turnCheck_attributes = {
  .name = "turnCheck",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
uint8_t task1 [30] = "Task1";
uint8_t task2 [30] = "Task2";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
void USART3Receive(void *argument);
void turnChecks(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t aRxBuffer [CMD_MAX_LENGTH];
volatile uint8_t bufferIndex = 0;

int flagDone = 0;
char key;
char direction;
int magnitude = 0;
int flagRead = 0;
int flagWrite = 0;

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
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

//  HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 30);
//  HAL_UART_Receive_IT(&huart1, (uint8_t *) aRxBuffer, CMD_MAX_LENGTH);
  HAL_UART_Receive_IT(&huart1, &aRxBuffer[bufferIndex], CMD_MAX_LENGTH);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of USART3Rx */
  USART3RxHandle = osThreadNew(USART3Receive, NULL, &USART3Rx_attributes);

  /* creation of turnCheck */
  turnCheckHandle = osThreadNew(turnChecks, NULL, &turnCheck_attributes);

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

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7200;
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
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RS_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RS_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RS_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void blinkLED3(int period){
	  HAL_GPIO_WritePin(GPIOE,LED3_Pin, GPIO_PIN_SET);
	  osDelay(period);
	  HAL_GPIO_WritePin(GPIOE,LED3_Pin, GPIO_PIN_RESET);
}

void ringBuzzer(int period){
  HAL_GPIO_WritePin(GPIOB,BUZZER_Pin, GPIO_PIN_SET);
  osDelay(period);
  HAL_GPIO_WritePin(GPIOB,BUZZER_Pin, GPIO_PIN_RESET);
  osDelay(period);
}

void configSteer(int angle){
  
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = angle;

	osDelay(3000);
}
// TODO: undone function for wider left turn
void turnLeft(int angle){
	if (angle < 0  || angle > 90) return;
	angle  = 90; // remove pls
	  /* Inter-dependent Configurations for Car 18 left turns*/
	  float MILLISECONDS_PER_DEGREE = 2780/90;
	  // uint16_t pwmVal = 1800;

	  /* Initialise Timer and Channels 
    Timer 1 Channel 4 for Servo Motor FRONT Steer
    Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
    Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
    */
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	  /* Turn the wheels left */
	  htim1.Instance->CCR4 = SERVO_LEFT;

    /* Configure RIGHT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	  /* Configure LEFT wheel to cause reverse motion of car */
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

    /* Narrow turn: Cause LEFT wheel to spin FASTER than RIGHT wheel
      Wide turn: Cause LEFT wheel to spin SLOWER than narrow turn */
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, DC_RIGHT);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2 * DC_LEFT);

    /*Dynamic function to cause different degree turn*/
	  osDelay(MILLISECONDS_PER_DEGREE * angle);

	  /* Reset servo to straight and stop motors */
	  htim1.Instance->CCR4 = SERVO_STRAIGHT;
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

    // For Testing purposes, adjust accordingly during release
	  osDelay(2000); 
}

void turnLeftNarrow(int angle){
	if (angle < 0  || angle > 180) return;
	angle  = 90; // remove pls
	  /* Inter-dependent Configurations for Car 18 left turns*/
	float MILLISECONDS_PER_DEGREE = 2680/90;
	  uint16_t pwmVal = 1750;


	  /* Initialise Timer and Channels
    Timer 1 Channel 4 for Servo Motor FRONT Steer
    Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
    Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
    */
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	  /* Turn the wheels left */
	  htim1.Instance->CCR4 = SERVO_LEFT;

    /* Configure RIGHT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	  /* Configure LEFT wheel to cause reverse motion of car */
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

    /* Narrow turn: Cause LEFT wheel to spin FASTER than RIGHT wheel
      Wide turn: Cause LEFT wheel to spin SLOWER than narrow turn */
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, DC_RIGHT);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2 * DC_LEFT);

    /*Dynamic function to cause different degree turn*/
	  osDelay(MILLISECONDS_PER_DEGREE * angle);

	  /* Reset servo to straight and stop motors */
	  htim1.Instance->CCR4 = SERVO_STRAIGHT;
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

    // For Testing purposes, adjust accordingly during release
	  osDelay(2000);
}

void turnLeftReverse(int angle) {
	if (angle < 0  || angle > 90) return;
		angle  = 90; // remove pls
		  /* Inter-dependent Configurations for Car 18 left turns*/
		float MILLISECONDS_PER_DEGREE = 2780/90;
		uint16_t pwmVal = 1750;


		  /* Initialise Timer and Channels
	    Timer 1 Channel 4 for Servo Motor FRONT Steer
	    Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
	    Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
	    */
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
		  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

		  /* Turn the wheels left */
		  htim1.Instance->CCR4 = SERVO_LEFT;

	    /* Configure RIGHT wheel to cause reverse motion of car */
		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		  /* Configure LEFT wheel to cause forward motion of car */
		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	    /* Narrow turn: Cause LEFT wheel to spin FASTER than RIGHT wheel
	      Wide turn: Cause LEFT wheel to spin SLOWER than narrow turn */
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, DC_RIGHT);
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2 * DC_LEFT);

	    /*Dynamic function to cause different degree turn*/
		  osDelay(MILLISECONDS_PER_DEGREE * angle);
//		  osDelay(test_90);

		  /* Reset servo to straight and stop motors */
		  htim1.Instance->CCR4 = SERVO_STRAIGHT;
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	    // For Testing purposes, adjust accordingly during release
		  osDelay(2000);
}

void turnRightNarrow(int angle){

  if (angle < 0  || angle > 180) return;
  /* Inter-dependent Configurations for Car 18 right turns */
  float MILLISECONDS_PER_DEGREE = 2975/90;
  uint16_t pwmVal = 1660;

  /* Initialise Timer and Channels 
  Timer 1 Channel 4 for Servo Motor FRONT Steer
  Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
  Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
  */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  /* Turn the wheels left */
  htim1.Instance->CCR4 = SERVO_RIGHT;

  /* Configure RIGHT wheel to cause reverse motion of car */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
  /* Configure LEFT wheel to cause forward motion of car */
  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

  /* Narrow turn: Cause RIGHT wheel to spin FASTER than LEFT wheel
    Wide turn: Cause RIGHT wheel to spin SLOWER than narrow turn */
  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2 * DC_RIGHT);
  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, DC_LEFT);

  /*Dynamic function to cause different degree turn*/
  osDelay(MILLISECONDS_PER_DEGREE * angle);

  /* Reset servo to straight and stop motors */
  htim1.Instance->CCR4 = SERVO_STRAIGHT;
  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

  // For Testing purposes, adjust accordingly during release
  osDelay(2000); 
}

// TODO: undone function for wider right turn
void turnRight(int angle){

  if (angle < 0  || angle > 90) return;
  angle  = 90; // remove pls
  /* Inter-dependent Configurations for Car 18 right turns */
  float MILLISECONDS_PER_DEGREE = 2780/90;
  uint16_t pwmVal = 1800;

  /* Initialise Timer and Channels
  Timer 1 Channel 4 for Servo Motor FRONT Steer
  Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
  Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
  */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  /* Turn the wheels left */
  htim1.Instance->CCR4 = 186;

  /* Configure RIGHT wheel to cause reverse motion of car */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
  /* Configure LEFT wheel to cause forward motion of car */
  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

  /* Narrow turn: Cause RIGHT wheel to spin FASTER than LEFT wheel
    Wide turn: Cause RIGHT wheel to spin SLOWER than narrow turn */
  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2 * DC_RIGHT);
  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, DC_LEFT);

  /*Dynamic function to cause different degree turn*/
  osDelay(MILLISECONDS_PER_DEGREE * angle);

  /* Reset servo to straight and stop motors */
  htim1.Instance->CCR4 = SERVO_STRAIGHT;
  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

  // For Testing purposes, adjust accordingly during release
  osDelay(2000);
}

// movement is weird
void turnRightReverse(int angle) {
	if (angle < 0  || angle > 90) return;
	  /* Inter-dependent Configurations for Car 18 right turns */
	  float MILLISECONDS_PER_DEGREE = 2780/90;
	  uint16_t pwmVal = 1660;

	  /* Initialise Timer and Channels
	  Timer 1 Channel 4 for Servo Motor FRONT Steer
	  Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
	  Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
	  */
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	  /* Turn the wheels left */
	  htim1.Instance->CCR4 = SERVO_RIGHT;

	  /* Configure RIGHT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	  /* Configure LEFT wheel to cause reverse motion of car */
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	  /* Narrow turn: Cause RIGHT wheel to spin FASTER than LEFT wheel
	    Wide turn: Cause RIGHT wheel to spin SLOWER than narrow turn */
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2 * DC_RIGHT);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, DC_LEFT);

	  /*Dynamic function to cause different degree turn*/
	  osDelay(MILLISECONDS_PER_DEGREE * angle);

	  /* Reset servo to straight and stop motors */
	  htim1.Instance->CCR4 = SERVO_STRAIGHT;
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	  // For Testing purposes, adjust accordingly during release
	  osDelay(2000);
}

void driveForward(int distance_in_cm) {
	
	  /* Inter-dependent Configurations for Car 18 */
	  uint16_t pwmVal = 1700;
    float MILLISECONDS_PER_CENTIMETRE = 39;
	  /* Initialise Timer and Channels
	  Timer 1 Channel 4 for Servo Motor FRONT Steer
	  Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
	  Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
	  */
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	  /* Turn the wheels left */
	  htim1.Instance->CCR4 = SERVO_STRAIGHT;

	  /* Configure RIGHT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	  /* Configure LEFT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);


	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, DC_RIGHT);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, DC_LEFT);

	  /*Dynamic function to cause different degree turn*/
	  osDelay(MILLISECONDS_PER_CENTIMETRE * distance_in_cm);

	  /* Reset servo to straight and stop motors */
	  htim1.Instance->CCR4 = SERVO_STRAIGHT;
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	  // For Testing purposes, adjust accordingly during release
	  osDelay(2000);
}

void driveBackward(int distance_in_cm) {

	  /* Inter-dependent Configurations for Car 18 */
	  uint16_t pwmVal = 1700;
    float MILLISECONDS_PER_CENTIMETRE = 39;
	  /* Initialise Timer and Channels
	  Timer 1 Channel 4 for Servo Motor FRONT Steer
	  Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
	  Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
	  */
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	  /* Turn the wheels left */
	  htim1.Instance->CCR4 = SERVO_STRAIGHT;

	  /* Configure RIGHT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	  /* Configure LEFT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);


	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmVal);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmVal);

	  /*Dynamic function to cause different degree turn*/
	  osDelay(MILLISECONDS_PER_CENTIMETRE * distance_in_cm);

	  /* Reset servo to straight and stop motors */
	  htim1.Instance->CCR4 = SERVO_STRAIGHT;
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	  // For Testing purposes, adjust accordingly during release
	  osDelay(2000);
}

void stopCar() {

	  /* Inter-dependent Configurations for Car 18 */
	  /* Initialise Timer and Channels
	  Timer 1 Channel 4 for Servo Motor FRONT Steer
	  Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
	  Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
	  */
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	  /* Turn the wheels left */
	  htim1.Instance->CCR4 = SERVO_STRAIGHT;

	  /* Configure RIGHT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	  /* Configure LEFT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);


	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	  /*Dynamic function to cause different degree turn*/


	  /* Reset servo to straight and stop motors */
	  htim1.Instance->CCR4 = SERVO_STRAIGHT;
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	  // For Testing purposes, adjust accordingly during release
	  osDelay(2000);
}

uint16_t ultrasonic()
{

    char buf[10];
    uint16_t distance_return = 0;

    // Ensure input capture is enabled before triggering the sensor

    // Send trigger pulse
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
    osDelay(50); // Ensure trigger is low before sending pulse

    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
    delay_us(10); // Send 10us trigger pulse
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);

    // Wait for the echo measurement to complete
    osDelay(50);

    // Display measured values
    // for debug only
    sprintf(buf, "tc1 = %5d us ", tc1);
    OLED_ShowString(10, 10, buf);

    sprintf(buf, "tc2 = %5d us ", tc2);
    OLED_ShowString(10, 20, buf);

    sprintf(buf, "Echo = %5d us ", echo);
    OLED_ShowString(10, 30, buf);

    distance_return = echo * 0.0343 / 2; // Convert to cm

    sprintf(buf, "Dist = %4d cm ", distance_return);
    OLED_ShowString(10, 40, buf);
    OLED_Refresh_Gram();

    return distance_return;
}

uint16_t readIRSensor(ADC_HandleTypeDef *hadc) {
    char buf[20];

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    uint16_t ADC_VAL = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    // Display ADC value
    sprintf(buf, "ADC Val: %5d", ADC_VAL);
    OLED_ShowString(10, 10, buf);

    // Compute and display distance
    uint16_t distance = 136.68 * exp (924.195/ ADC_VAL) - 140.33;
//    		distanceCalc(ADC_VAL);
    sprintf(buf, "Dist: %5d", distance);
    OLED_ShowString(10, 30, buf);
    OLED_Refresh_Gram();

    return ADC_VAL;
}

void turnToNextFace(){

	driveForward(10);
	driveBackward(20);
	turnLeftNarrow(90);
	driveForward(30);
	turnRightNarrow(100);
	driveForward(15);
	turnRightNarrow(100);

}

void delay_us(uint16_t us){
	HAL_TIM_Base_Start(&htim6);
	__HAL_TIM_SET_COUNTER(&htim6, 0);

	while(__HAL_TIM_GET_COUNTER(&htim6) < us);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Check correct timer and channel
    {
        if (Is_First_Captured == 0) // Rising edge detected
        {
            tc1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (Is_First_Captured == 1) // Falling edge detected
        {

            tc2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_COUNTER(htim, 0);

            if (tc2 > tc1)
            {
                echo = tc2 - tc1;
            }
            else
            {
            	echo = (65535 - tc1) + tc2; // Timer overflow correction
            }


            Is_First_Captured = 0; // Reset flag

            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

void threePointTurn(){
	 turnLeftReverse(90);
	 turnRightNarrow(90);

}

void try(){
	 turnRightNarrow(90);
	 driveForward(20);
	 turnLeftNarrow(90);
	 turnLeftNarrow(90);

	 turnRightNarrow(90);
	 driveForward(20);
	 turnLeftNarrow(90);
	 turnLeftNarrow(90);

	 turnRightNarrow(90);
	 driveForward(20);
	 turnLeftNarrow(90);
	 turnLeftNarrow(90);

	 turnRightNarrow(90);
	 driveForward(20);
	 turnLeftNarrow(90);
	 turnLeftNarrow(90);


}

//void start_uart_receive(void) {
//    // Start receiving CMD_MAX_LENGTH bytes.
//    HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, CMD_MAX_LENGTH);
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	
  // for debugging
  // HAL_UART_Transmit(&huart1, (uint8_t *) aRxBuffer, CMD_MAX_LENGTH, 0xFFFF);


    OLED_ShowString(10, 20, aRxBuffer); //sanity check

    OLED_Refresh_Gram();

  	readyToExecute = 1;

	HAL_UART_Receive_IT(&huart1, aRxBuffer, CMD_MAX_LENGTH);
  
}

void acknowledgeCompletion(){
  uint8_t reply [3] = "OK\0";
  HAL_UART_Transmit(&huart1, (uint8_t *) reply, 2, 0xFFFF);
  readyToExecute = 0;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_USART3Receive */
/**
  * @brief  Function implementing the USART3Rx thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_USART3Receive */
void USART3Receive(void *argument)
{
/* USER CODE BEGIN USARTR3x */
	char ch = 'A';
		char old = ')';
//		uint8_t debugMsg[20] = "hello\0";


		int signMagnitude = 1;
	  /* Infinite loop */
//		  aRxBuffer[0] = '-';
//		  aRxBuffer[1] = 'W';
//		  aRxBuffer[2] = 'A';
//		  aRxBuffer[3] = 'I';
//		  aRxBuffer[4] = 'T';
	  for(;;)
	  {
		  magnitude = 0;
		  key = '\0';
		  direction = '\0';

		  key = aRxBuffer[0];
		  direction = aRxBuffer[1];
		  magnitude = ((int)(((int)aRxBuffer[2])-48)*100) + ((int)(((int)aRxBuffer[3])-48)*10) + ((int)(((int)aRxBuffer[4])-48));
		  signMagnitude = 1;



//		  if(direction == 'B' || direction == 'b'){
//			  magnitude *= (int)-1;
//			  signMagnitude = -1;
//		  }

		  //if(aRxBuffer[0] != old){
//			if (aRxBuffer[0]!='D' & aRxBuffer[4]!='!'){
		  if (readyToExecute == 1){
//				old_Buff1[0] = old_Buff[0];
//				old_Buff1[1] = old_Buff[1];
//				old_Buff1[2] = old_Buff[2];
//				old_Buff1[3] = old_Buff[3];
//				old_Buff1[4] = old_Buff[4];
//			old_Buff[0] = aRxBuffer[0];
//			old_Buff[1] = aRxBuffer[1];
//			old_Buff[2] = aRxBuffer[2];
//			old_Buff[3] = aRxBuffer[3];
//			old_Buff[4] = aRxBuffer[4];

			//}

			 //osDelay(500); //og 2000 delay

			  switch (key){
				  case 'D':
					  break;
				  case 'S':
					  times_acceptable=0;
					  driveForward((int)magnitude);
//					  while(finishCheck());
					  flagDone=1;
//					  memset(aRxBuffer, 0 , CMD_MAX_LENGTH);
//					  aRxBuffer[0] = '-';
//					  aRxBuffer[1] = '-';
//					  aRxBuffer[2] = '-';
//					  aRxBuffer[3] = '-';
//					  aRxBuffer[4] = '-';
					  osDelay(1000); //og 100
					  ringBuzzer(3000); // for debug
					  break;

				  case 'B':
					  times_acceptable=0;
					  driveBackward((int)magnitude);
//					  while(finishCheck());
					  flagDone=1;
//					  memset(aRxBuffer, 0 , CMD_MAX_LENGTH);
//					  aRxBuffer[0] = '-';
//					  aRxBuffer[1] = '-';
//					  aRxBuffer[2] = '-';
//					  aRxBuffer[3] = '-';
//					  aRxBuffer[4] = '-';
					  osDelay(1000); //og 100
					  ringBuzzer(3000); // for debug
					  break;

				  case 'R':
					  times_acceptable=0;
					  turnRightNarrow((int)magnitude);
//					  while(finishCheck());
					  flagDone=1;
//					  memset(aRxBuffer, 0 , CMD_MAX_LENGTH);
//					  aRxBuffer[0] = '-';
//					  aRxBuffer[1] = '-';
//					  aRxBuffer[2] = '-';
//					  aRxBuffer[3] = '-';
//					  aRxBuffer[4] = '-';
					  osDelay(1000); //og 100
					  ringBuzzer(3000); // for debug
					  break;

				  case 'L':
					  times_acceptable=0;
					  turnLeftNarrow((int)magnitude);
//					  while(finishCheck());
					  flagDone=1;
//					  memset(aRxBuffer, 0 , CMD_MAX_LENGTH);
//					  aRxBuffer[0] = '-';
//					  aRxBuffer[1] = '-';
//					  aRxBuffer[2] = '-';
//					  aRxBuffer[3] = '-';
//					  aRxBuffer[4] = '-';
					  osDelay(1000); //og 100
					  ringBuzzer(3000); // for debug
					  break;

				  case 'P':
					  turnToNextFace();
					  flagDone=1;
					  break;

				  case '-':
					  times_acceptable=0;
					  stopCar();
//					  while(finishCheck());
					  flagDone=1;
//					  memset(aRxBuffer, 0 , CMD_MAX_LENGTH);

//					  aRxBuffer[0] = '-';
//					  aRxBuffer[1] = '-';
//					  aRxBuffer[2] = '-';
//					  aRxBuffer[3] = '-';
//					  aRxBuffer[4] = '-';
					  osDelay(1000); //og 100
					  break;
				  default:
					  break;
			  }
			  old = aRxBuffer[0];
		  }



		  // send ack back to rpi and ready for next instruction
			if(flagDone==1){
				acknowledgeCompletion();
				osDelay(50); //og 500
				flagDone = 0;
			}
			//flagRead = 0;
			osDelay(1);
	  }

  /* USER CODE END USARTR3x */
}

/* USER CODE BEGIN Header_turnChecks */
/**
* @brief Function implementing the turnCheck thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_turnChecks */
void turnChecks(void *argument)
{
  /* USER CODE BEGIN turnChecks */


  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END turnChecks */
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
}
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
