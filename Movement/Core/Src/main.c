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
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CMD_MAX_LENGTH  5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


// SERVO-MOTOR
uint16_t SERVO_STRAIGHT = 142;
//uint16_t SERVO_LEFT = 105; // the turning rad is too small, need to increase to make it turn wider
uint16_t SERVO_LEFT = 109; // 64cm for 100
uint16_t SERVO_RIGHT = 201; // change it to achieve TURNING RADIUS of 27
//uint16_t SERVO_RIGHT = 220; // TURNING RADIUS is 23.35 atm
uint16_t pwmVal_servo;
const double TURNING_RADIUS = 35; // TO FIND OUT AGAIN, based on SERVO_L/R
//70cm diameter for Right taking middle
const double WHEELBASE = 16.8; // TO FIND OUT AGAIN, separation between two back wheels


// PID-controlled DC Motors
//#define COUNT_PER_REV 1320  // Quadrature-encoded pulses per wheel revolution
// check if need change again - do the measurements
#define COUNT_PER_REV 1560
//1544 right 1576 left


const double PI = 3.141592653;
const double WHEEL_DIAMETER = 6.5;  // in cm
const double WHEEL_CIRCUM = WHEEL_DIAMETER * PI;
const double COUNT_PER_CM = COUNT_PER_REV / WHEEL_CIRCUM;
uint16_t pwmVal_Left, pwmVal_Right = 0;
volatile int start = 0;
volatile int leftDiff = 0, rightDiff = 0;
// PID variables
float dTRight, dTLeft;

// REAR WHEELS ENCODER VARIABLES
int16_t rightEncoderVal = 0, leftEncoderVal = 0;
//int16_t rightTargetVal = 4000, leftTargetVal = 4000; // should be 0 when running/testing the code
int16_t rightTargetVal = 0, leftTargetVal = 0;
int16_t rightErrorVal = 0, leftErrorVal = 0;
int32_t rightIntegral = 0, leftIntegral = 0;

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for USART1Rx */
osThreadId_t USART1RxHandle;
const osThreadAttr_t USART1Rx_attributes = {
  .name = "USART1Rx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for encLeftTask */
osThreadId_t encLeftTaskHandle;
const osThreadAttr_t encLeftTask_attributes = {
  .name = "encLeftTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encRightTask */
osThreadId_t encRightTaskHandle;
const osThreadAttr_t encRightTask_attributes = {
  .name = "encRightTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
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
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void USART1Receive(void *argument);
void encoderLeftTask(void *argument);
void encoderRightTask(void *argument);
void dcMotorTask(void *argument);
void oledTask1(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t aRxBuffer [CMD_MAX_LENGTH];
volatile uint8_t bufferIndex = 0;


// RPI
int flagDone = 0;
char key;
char direction;
uint16_t magnitude = 0;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
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
  /* creation of USART1Rx */
  USART1RxHandle = osThreadNew(USART1Receive, NULL, &USART1Rx_attributes);

  /* creation of encLeftTask */
  encLeftTaskHandle = osThreadNew(encoderLeftTask, NULL, &encLeftTask_attributes);

  /* creation of encRightTask */
  encRightTaskHandle = osThreadNew(encoderRightTask, NULL, &encRightTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(dcMotorTask, NULL, &motorTask_attributes);

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(oledTask1, NULL, &oledTask_attributes);

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOA, BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|DIN2_Pin|DIN1_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin DIN2_Pin DIN1_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|DIN2_Pin|DIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

// NEW DRIVEFORWARD
void moveForward(uint8_t distance_in_cm) {

	char buf[20]; // for debug

	int32_t targetTicks = (int32_t) (distance_in_cm * COUNT_PER_CM);
	leftTargetVal = targetTicks;
	rightTargetVal = targetTicks;
	pwmVal_servo = SERVO_STRAIGHT;
	start = 1;
}

void moveBackward(uint8_t distance_in_cm) {
  

	int32_t targetTicks = (int32_t) - 1 * (distance_in_cm * COUNT_PER_CM);
	leftTargetVal = targetTicks;
	rightTargetVal = targetTicks;
	pwmVal_servo = SERVO_STRAIGHT;
	start = 1;
}

void moveLeftForward(uint16_t angle) {
    float outerArc = (PI * (TURNING_RADIUS + (WHEELBASE / 2)) * angle) / 180.0;
    float innerArc = (PI * (TURNING_RADIUS - (WHEELBASE / 2)) * angle) / 180.0;

    int32_t outerTicks = (int32_t)(outerArc * COUNT_PER_CM);
    int32_t innerTicks = (int32_t)(innerArc * COUNT_PER_CM);

    leftTargetVal = innerTicks;   // Inner wheel moves less
    rightTargetVal = outerTicks;  // Outer wheel moves more
    pwmVal_servo = SERVO_LEFT;
}

void moveLeftBackward(uint16_t angle) {
    float outerArc = (PI * (TURNING_RADIUS + (WHEELBASE / 2)) * angle) / 180.0;
    float innerArc = (PI * (TURNING_RADIUS - (WHEELBASE / 2)) * angle) / 180.0;

    int32_t outerTicks = (int32_t)(outerArc * COUNT_PER_CM);
    int32_t innerTicks = (int32_t)(innerArc * COUNT_PER_CM);

    leftTargetVal = -innerTicks;   // Reverse movement
    rightTargetVal = -outerTicks;
    pwmVal_servo = SERVO_LEFT;
}

void moveRightForward(uint16_t angle) {
    float outerArc = (PI * (TURNING_RADIUS + (WHEELBASE / 2)) * angle) / 180.0;
    float innerArc = (PI * (TURNING_RADIUS - (WHEELBASE / 2)) * angle) / 180.0;

    int32_t outerTicks = (int32_t)(outerArc * COUNT_PER_CM);
    int32_t innerTicks = (int32_t)(innerArc * COUNT_PER_CM);

    leftTargetVal = outerTicks;   // Outer wheel moves more
    rightTargetVal = innerTicks;  // Inner wheel moves less
    pwmVal_servo = SERVO_RIGHT;
}

void moveRightBackward(uint16_t angle) {
    float outerArc = (PI * (TURNING_RADIUS + (WHEELBASE / 2)) * angle) / 180.0;
    float innerArc = (PI * (TURNING_RADIUS - (WHEELBASE / 2)) * angle) / 180.0;

    int32_t outerTicks = (int32_t)(outerArc * COUNT_PER_CM);
    int32_t innerTicks = (int32_t)(innerArc * COUNT_PER_CM);

    leftTargetVal = -outerTicks;  // Reverse movement
    rightTargetVal = -innerTicks;
    pwmVal_servo = SERVO_RIGHT;
}

uint16_t PID_Control_right() {
    // PID gains (tune as needed)
//    const float Kp = 35.0f, Ki = 0.0f, Kd = 3.0f; // configured based on 5V
	const float Kp = 4.0f, Ki = 0.01f, Kd = 2.0f; // testing 12V Power Supply
    const float MAX_INTEGRAL = 10.0f;
    // 2000 achieves 20cm/s
    const uint16_t MAX_PWM_VAL = 1800;  // Adjust as needed
    const uint16_t MIN_PWM_VAL = 500;   // Adjust as needed

    // Static variables for persistent error tracking
    static int prevError = 0;
    static float integral = 0.0f;

    // Compute error
    int errorVal = rightEncoderVal - rightTargetVal;

    // Compute derivative (only if dT > 0)
    float derivative = (dTRight > 0) ? (errorVal - prevError) / dTRight : 0.0f;

    // Integrate error with anti-windup clamping
    integral += errorVal * dTRight;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    else if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;

    // Compute control signal
    float u_float = Kp * errorVal + Kd * derivative + Ki * integral;
    int u = (int)u_float;  // Convert to integer

    // Store previous error for next cycle
    prevError = errorVal;

    // Set motor direction
    HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, (u < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, (u < 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Limit PWM output
    u = abs(u);
    if (u > MAX_PWM_VAL) return MAX_PWM_VAL;
    if (u < MIN_PWM_VAL) return 0;

    return (uint16_t)u; // return pwm value
}

uint16_t PID_Control_left() {
    // PID gains (tune as needed)
//    const float Kp = 20.0f, Ki = 0.0f, Kd = 4.0f; // configured based on 5V
    const float Kp = 2.5f, Ki = 0.01f, Kd = 0.5f; // testing 12V Power Supply
    const float MAX_INTEGRAL = 10.0f;
    // 2000 achieves 20cm/s
    const uint16_t MAX_PWM_VAL = 1800;  // Adjust as needed
    const uint16_t MIN_PWM_VAL = 500;   // Adjust as needed

    // Static variables for persistent error tracking
    static int prevError = 0;
    static float integral = 0.0f;

    // Compute error
    int errorVal = leftEncoderVal - leftTargetVal;

    // Compute derivative (only if dT > 0)
    float derivative = (dTLeft > 0) ? (errorVal - prevError) / dTLeft : 0.0f;

    // Integrate error with anti-windup clamping
    integral += errorVal * dTLeft;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    else if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;

    // Compute control signal
    float u_float = Kp * errorVal + Kd * derivative + Ki * integral;
    int u = (int)u_float;  // Convert to integer

    // Store previous error for next cycle
    prevError = errorVal;

    // Set motor direction
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, (u < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, (u < 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Limit PWM output
    u = abs(u);
    if (u > MAX_PWM_VAL) return MAX_PWM_VAL;
    if (u < MIN_PWM_VAL) return 0;

    return (uint16_t)u; // return pwm value
}


uint16_t ultrasonic()
{

    char buf[20];
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
//    sprintf(buf, "tc1 = %5d us ", tc1);
//    OLED_ShowString(10, 10, buf);
//
//    sprintf(buf, "tc2 = %5d us ", tc2);
//    OLED_ShowString(10, 20, buf);
//
//    sprintf(buf, "Echo = %5d us ", echo);
//    OLED_ShowString(10, 30, buf);
//
//    distance_return = echo * 0.0343 / 2; // Convert to cm
//
//    sprintf(buf, "Dist = %4d cm ", distance_return);
//    OLED_ShowString(10, 40, buf);
//    OLED_Refresh_Gram();

    // debug end

    return distance_return;
}

//uint16_t readIRSensor(ADC_HandleTypeDef *hadc) {
//    char buf[20];
//
//    HAL_ADC_Start(hadc);
//    HAL_ADC_PollForConversion(hadc, 10);
//    uint16_t ADC_VAL = HAL_ADC_GetValue(hadc);
//    HAL_ADC_Stop(hadc);
//
//    // Display ADC value
//    sprintf(buf, "ADC Val: %5d", ADC_VAL);
//    OLED_ShowString(10, 10, buf);
//
//    // Compute and display distance
//    uint16_t distance = 136.68 * exp (924.195/ ADC_VAL) - 140.33;
////    		distanceCalc(ADC_VAL);
//    sprintf(buf, "Dist: %5d", distance);
//    OLED_ShowString(10, 30, buf);
//    OLED_Refresh_Gram();
//
//    return ADC_VAL;
//}

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
//    OLED_ShowString(10, 20, aRxBuffer); //sanity check
//    OLED_Refresh_Gram();
  	readyToExecute = 1;
	HAL_UART_Receive_IT(&huart1, aRxBuffer, CMD_MAX_LENGTH);
}

void acknowledgeCompletion(){
  uint8_t reply [3] = "OK\n";
  HAL_UART_Transmit(&huart1, (uint8_t *) reply, 3, 0xFFFF);
  readyToExecute = 0;
}

int isCarMoving(){
/* Checks if car is moving
 * if car has stopped it means it is
 * ready to receive next command
 * returns true if car is moving
 * if car stopped return false
*/
	int leftStop = 0, rightStop = 0;
//
//  if (leftDiff < 50 && leftDiff > -50) leftStop = 1;
//  if (rightDiff < 50 && rightDiff > -50) rightStop = 1;
//  if (rightStop == 1 && leftStop == 1){
//	  ringBuzzer(40);
//	  ringBuzzer(40);
////	  HAL_UART_Receive_IT(&huart1, aRxBuffer, CMD_MAX_LENGTH);
//	  return 0;
//  }

	if (pwmVal_Left == 0 && pwmVal_Right ==0){
		  return 0;
	  }
  return 1;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_USART1Receive */
/**
  * @brief  Function implementing the USART1Rx thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_USART1Receive */
void USART1Receive(void *argument)
{
  /* USER CODE BEGIN 5 */
		  for(;;)
		  {
			  magnitude = 0;
			  key = '\0';
			  direction = '\0';

			  key = aRxBuffer[0];
			  direction = aRxBuffer[1];
			  magnitude = ((int)(((int)aRxBuffer[2])-48)*100) + ((int)(((int)aRxBuffer[3])-48)*10) + ((int)(((int)aRxBuffer[4])-48));


			  if (readyToExecute == 1){

				  switch (key){
					  case 'D':
						  break;
					  case 'S':
						  moveForward((uint16_t)magnitude);
						  while(isCarMoving());
						  flagDone=1;
						  break;

					  case 'B':
						  moveBackward((uint16_t)magnitude);
						  while(isCarMoving());
						  flagDone=1;
						  break;

					  case 'R':
						  moveRightForward((uint16_t) magnitude);
						  while(isCarMoving());
						  flagDone=1;
						  break;

					  case 'L':
						  moveLeftForward((uint16_t) magnitude);
						  while(isCarMoving());
						  flagDone=1;
						  break;

            
					  case 'W':
						  moveRightBackward((uint16_t) magnitude);
						  while(isCarMoving());
						  flagDone=1;
						  break;

					  case 'V':
						  moveLeftBackward((uint16_t) magnitude);
						  while(isCarMoving());
						  flagDone=1;
						  break;
					  default:
						  break;
				  }
			  }

			  // send ack back to rpi and ready for next instruction
				if(flagDone==1){

					osDelay(3500); //og 500
					acknowledgeCompletion();
					ringBuzzer(30);
					ringBuzzer(30);
					flagDone = 0;
					leftTargetVal = 0;
					leftEncoderVal = 0;

					rightEncoderVal = 0;
					rightTargetVal  = 0;

				}
				//flagRead = 0;
				osDelay(1);
		  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_encoderLeftTask */
/**
* @brief Function implementing the encLeftTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoderLeftTask */
void encoderLeftTask(void *argument)
{
  /* USER CODE BEGIN encoderLeftTask */
	// using motor B
	TIM_HandleTypeDef *htim = &htim3;
	HAL_TIM_Encoder_Start(htim,TIM_CHANNEL_ALL);
	int16_t cnt_B;
	int8_t dirL = 1;
	uint32_t tick = HAL_GetTick();
	uint8_t buf[20]; // for debug
  /* Infinite loop */
  for(;;)
  {
	  if (HAL_GetTick() - tick > 10L){ // 10ms delay, adjust accordingly or 100Hz atm
		  dTLeft =  (HAL_GetTick() - tick) / 1000;
		  cnt_B = __HAL_TIM_GET_COUNTER(htim);
		  if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){ // moving forward
			  dirL = 1;
			  leftDiff = 65536 - cnt_B;
		  }
		  else{
				dirL = -1;
				leftDiff = cnt_B;
		  }

		  if(dirL == 1){
			  leftEncoderVal += (int16_t)leftDiff;
		  }

		  else{
			  leftEncoderVal -= (int16_t)leftDiff;
		  }
		  __HAL_TIM_SET_COUNTER(htim, 0);


      // for debug

		  tick = HAL_GetTick();
	  }
	  osDelay(1);
  }
  /* USER CODE END encoderLeftTask */
}

/* USER CODE BEGIN Header_encoderRightTask */
/**
* @brief Function implementing the encRightTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoderRightTask */
void encoderRightTask(void *argument)
{
  /* USER CODE BEGIN encoderRightTask */
	// using motor D
	TIM_HandleTypeDef *htim = &htim2;
	HAL_TIM_Encoder_Start(htim,TIM_CHANNEL_ALL);
	int16_t cnt_A;
	int8_t dirR = 1;
	uint32_t tick = HAL_GetTick();
    uint8_t buf[20]; // for debug
  /* Infinite loop */
  for(;;)
  {

	  if (HAL_GetTick() - tick > 10L){ // 10ms delay, adjust accordingly or 100Hz atm
		  dTRight = (HAL_GetTick() - tick) / 1000;
		  cnt_A = __HAL_TIM_GET_COUNTER(htim);
		  if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){ // moving forward
			dirR = -1;
			rightDiff = 65536 - cnt_A;
		  }
		  else{
				dirR =  1; // motor D is wired differently idk why;
				rightDiff = cnt_A;
		  }

		  if(dirR == 1){
			  rightEncoderVal += (int16_t)rightDiff;
		  }

		  else{
			  rightEncoderVal -= (int16_t)rightDiff;
		  }
		  __HAL_TIM_SET_COUNTER(htim, 0);

		  tick = HAL_GetTick();

      // for debug


	  }
    osDelay(1);
  }
  /* USER CODE END encoderRightTask */
}

/* USER CODE BEGIN Header_dcMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dcMotorTask */
void dcMotorTask(void *argument)
{
  /* USER CODE BEGIN dcMotorTask */

	  /*
	  Initialise Timer and Channels
	  Timer 1 Channel 4 for Servo Motor FRONT Steer
	  Timer 8 Channel 1 for Motor A -- configured for RIGHT DC Wheel
	  Timer 8 Channel 2 for Motor B -- configured for LEFT DC Wheel
	  */
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	  /* Configure RIGHT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);
	  /* Configure LEFT wheel to cause forward motion of car */
	  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);


  /* Infinite loop */
  for(;;)
  {
    // add the pwm IF statement
	  pwmVal_Left = PID_Control_left();
	  pwmVal_Right = PID_Control_right();

	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmVal_Left);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, pwmVal_Right);

	  htim1.Instance->CCR4 = pwmVal_servo;
	  osDelay(25);
  }
  /* USER CODE END dcMotorTask */
}

/* USER CODE BEGIN Header_oledTask1 */
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_oledTask1 */
void oledTask1(void *argument)
{
  /* USER CODE BEGIN oledTask1 */
	uint8_t buf [20];
  /* Infinite loop */
  for(;;)
  {
	  sprintf(buf, "lC = %5d", leftEncoderVal);
	  OLED_ShowString(10, 10, buf);

	  sprintf(buf, "rC = %5d", rightEncoderVal);
	  OLED_ShowString(10, 20, buf);

	  sprintf(buf, "cmd= %s", aRxBuffer);
	  OLED_ShowString(10, 30, buf);

	  sprintf(buf, "rT= %5d", rightTargetVal);
	  OLED_ShowString(10, 50, buf);

	  sprintf(buf, "lT= %5d", leftTargetVal);
	  OLED_ShowString(10, 40, buf);
	  OLED_Refresh_Gram();

	  osDelay(100);
  }
  /* USER CODE END oledTask1 */
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
