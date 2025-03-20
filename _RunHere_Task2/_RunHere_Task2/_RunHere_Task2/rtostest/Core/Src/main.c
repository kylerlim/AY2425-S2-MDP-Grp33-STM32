/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "../../Drivers/PeripheralDriver/Inc/oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVOCENTER 151
#define SERVORIGHT 210//230 //220
#define SERVOLEFT 95 //100 //100
//uint16_t SERVO_LEFT = 72;
//uint16_t SERVO_RIGHT = 200;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for ultrasonicTask */
osThreadId_t ultrasonicTaskHandle;
const osThreadAttr_t ultrasonicTask_attributes = {
  .name = "ultrasonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gyroTask */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
  .name = "gyroTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for irSensorTask */
osThreadId_t irSensorTaskHandle;
const osThreadAttr_t irSensorTask_attributes = {
  .name = "irSensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoderRightTas */
osThreadId_t encoderRightTasHandle;
const osThreadAttr_t encoderRightTas_attributes = {
  .name = "encoderRightTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoderLeftTask */
osThreadId_t encoderLeftTaskHandle;
const osThreadAttr_t encoderLeftTask_attributes = {
  .name = "encoderLeftTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for jukeTask */
osThreadId_t jukeTaskHandle;
const osThreadAttr_t jukeTask_attributes = {
  .name = "jukeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
osThreadId_t IrSensorsTaskHandle;
const osThreadAttr_t IrSensorsTask_attributes = {
  .name = "IrSensorsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
void StartUSTask(void *argument);
void StartMotorTask(void *argument);
void StartOledTask(void *argument);
void StartGyroTask(void *argument);
void StartIRSensorTask(void *argument);
void StartEncoderRightTask(void *argument);
void StartEncoderLeftTask(void *argument);
void StartJukeTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// communication
uint8_t aRxBuffer[20] = {0};
uint8_t old_Buff[20] = {0};
uint8_t old_Buff1[20] = {0};

int flagDone = 0;
char key;
char direction;
int magnitude = 0;
int update = 0;
//int debug =1 ;
int goodluck =0;

// movement
uint16_t pwmVal_servo = SERVOCENTER;
uint16_t oldPwmVal_servo = SERVOCENTER;
uint16_t pwmVal_R = 0;
uint16_t pwmVal_L = 0;
int times_acceptable=0;
int e_brake = 0;
int extraNudge = 0;
int errorcorrection = 0;
int straightUS = 0;


// MotorEncoder
int32_t rightEncoderVal = 0, leftEncoderVal = 0;
int32_t rightTarget = 0, leftTarget = 0;
double target_angle=0;


// Movement standing still Encoder
int oldTick=0;

//task 2 Juke functions
int activateJuke=0;
int scan=0;
int turnDone=0;
int straightDone=0;
int travel = 0;
char nexttask = '8';
int turn90 = 0;
int movebackL=0;
int movebackR=0;
int usTargetGLOBAL = 28;
double universalDistance = 0;
int moveBackLeftRun1 = 0;
int moveBackRightRun1 = 0;
int moveBackLeftRun2 = 0;
int moveBackRightRun2 = 0;
int obsTwoLength = 0;
int obsTwoFlag = 0;

float voltage1, voltage2 = 0;
int irDistance1, irDistance2 = 0;
uint32_t ADC_VAL1,ADC_VAL2 = 0;


// Gyro
double total_angle=0;
uint8_t gyroBuffer[20];
uint8_t ICMAddress = 0x68;
int notdone=1;
double offset = 0;
double trash= 0;

// Oled
char instructBuffer[40][5]={0};

//UserButton
int start = 0;

//Ultrasound
int Is_First_Captured = 0;
int32_t IC_Val1 = 0;
int32_t IC_Val2 = 0;
double Difference =0;
double Distance = 0;
int usflag=1;


const double PI = 3.1415926535;



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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  // for debug
  //HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 1);
  // for real task
  HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 1);

//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
//
//  /* creation of MotorTask */
//  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);
//
//  /* creation of oledTask */
//  oledTaskHandle = osThreadNew(StartOledTask, NULL, &oledTask_attributes);
//
//  /* creation of rpiTask */
//  rpiTaskHandle = osTreadNew(StartRpiTask, NULL, &rpiTask_attributes);
//
//  /* creation of gyroTask */
//  gyroTaskHandle = osTreadNew(StartGyroTask, NULL, &gyroTask_attributes);
//
//  /* creation of bulleyesTask */
//  bulleyesTaskHandle = osThreadNew(StartBulleyesTask, NULL, &bulleyesTask_attributes);
//
//	/* creation of encoderRightTas */
//	encoderRightTasHandle = osThreadNew(StartEncoderRightTask, NULL, &encoderRightTas_attributes);
//
//  /* creation of encoderLeftTask */
//  encoderLeftTaskHandle = osThreadNew(StartEncoderLeftTask, NULL, &encoderLeftTask_attributes);

  ultrasonicTaskHandle = osThreadNew(StartUSTask, NULL, &ultrasonicTask_attributes);

    /* creation of MotorTask */
    motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

    /* creation of oledTask */
    oledTaskHandle = osThreadNew(StartOledTask, NULL, &oledTask_attributes);

    /* creation of rpiTask */
//    rpiTaskHandle = osThreadNew(StartRpiTask, NULL, &rpiTask_attributes);

    /* creation of gyroTask */
    gyroTaskHandle = osThreadNew(StartGyroTask, NULL, &gyroTask_attributes);

    /* creation of bulleyesTask */
    irSensorTaskHandle = osThreadNew(StartIRSensorTask, NULL, &irSensorTask_attributes);

	/* creation of encoderRightTas */
	encoderRightTasHandle = osThreadNew(StartEncoderRightTask, NULL, &encoderRightTas_attributes);

	/* creation of encoderLeftTask */
	encoderLeftTaskHandle = osThreadNew(StartEncoderLeftTask, NULL, &encoderLeftTask_attributes);

	/* creation of startJukeTask */
	jukeTaskHandle = osThreadNew(StartJukeTask, NULL, &jukeTask_attributes);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();


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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
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
  htim8.Init.Period = 7199;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_PB_Pin */
  GPIO_InitStruct.Pin = USER_PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_PB_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* to prevent unused argument(s) compilation warning */
	UNUSED(huart);

	// for debug
	//HAL_UART_Receive_IT (&huart3, aRxBuffer, 1);
	// for real task
	HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 1);
	update++;
}

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ) {
	if (GPIO_Pin == USER_PB_Pin) {
		if (start == 0){
			start = 1;
		    }
		else
			start = 0;
 	    }
}


void delay(uint16_t time)
{
 __HAL_TIM_SET_COUNTER(&htim4, 0);
 while (__HAL_TIM_GET_COUNTER (&htim4) < time);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
 {
  if (Is_First_Captured == 0)
  {
   IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
   Is_First_Captured = 1;
   __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else if (Is_First_Captured == 1)
  {
   IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
   __HAL_TIM_SET_COUNTER(htim,0);

   if (IC_Val2 > IC_Val1)
   {
    Difference = IC_Val2 - IC_Val1;
   }

   else if (IC_Val1 > IC_Val2)
   {
    Difference = (65535 - IC_Val1) + IC_Val2;

   }

   Distance = Difference * .0343/2;

   Is_First_Captured = 0;

   __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
   __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);


//   char clear[20] = {0};
//   sprintf(clear, "%d|%d|USik:%d\n\r",IC_Val1, IC_Val2, (int)Distance);
//
//   HAL_UART_Transmit(&huart3, clear, 20,0xFFFF);


   usflag=1;

  }
 }
}


void ultrasonic_read(void){
    //code for ultrasound
	usflag=0;
	HAL_GPIO_WritePin(GPIOE, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);
	HAL_GPIO_WritePin(GPIOE, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
}

void buzzerBeep(int time)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); //Buzzer On
	HAL_Delay(time);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); //Buzzer Off
	HAL_Delay(time);
}


void moveCarStraight(double distance)
{
	distance = distance*75.6;
//	pwmVal_servo = 149;
	osDelay(150);
	e_brake = 0;
	times_acceptable=0;
	universalDistance = distance;
	rightTarget = rightEncoderVal; //left
	leftTarget = leftEncoderVal;
	rightTarget += distance;
	leftTarget += distance;
}

void moveCarStraightSensor(int usTarget)
{
	usTargetGLOBAL= usTarget;
	pwmVal_servo = SERVOCENTER;
	osDelay(100);
	e_brake = 0;
	times_acceptable=0;

}
void moveCarStop()
{
	e_brake = 0;
	pwmVal_servo = SERVOCENTER;
	osDelay(200);
}

void moveCarRight(double angle)
{
	pwmVal_servo = SERVORIGHT;
	osDelay(10);
	e_brake = 0;
	times_acceptable=0;
	target_angle -= angle;
}

void moveCarLeft(double angle)
{
	pwmVal_servo = 106;
	osDelay(10);
	e_brake = 0;
	times_acceptable=0;
	target_angle += angle;
}

void moveCarSlideRight(int value){
	int sign = 1;
//	if(forward == 1){
//		sign = 1;
//	}else{
//		sign = -1;
//	}
//	times_acceptable=0;
//	moveCarStraight((600/75.19)*sign);
//	while(finishCheck());
	pwmVal_servo = SERVORIGHT;

	osDelay(200);

	// for 10-40 slide, => 36
	times_acceptable=0;
	moveCarRight(value*sign);
	while(finishCheck());
	pwmVal_servo = 106;
	osDelay(150);

	times_acceptable=0;
	moveCarLeft(value*sign);
	while(finishCheck());
}

void moveCarSlideLeft(int value){
	int sign = 1;
//	if(forward == 1){
//		sign = 1;
//	}else{
//		sign = -1;
//	}
//	times_acceptable=0;
//	moveCarStraight((600/75.19)*sign);
//	while(finishCheck());
	pwmVal_servo = 106;

	osDelay(200);

	// for 10-40 slide, => 36
	times_acceptable=0;
	moveCarLeft(value*sign);
	while(finishCheck());
	pwmVal_servo = SERVORIGHT;
	osDelay(5);

	times_acceptable=0;
	moveCarRight(value*sign);
	while(finishCheck());
}

void moveCarRight90(double angle)
{
	int sign = 1;
	if(angle<0){
		sign=-1;
	}
	e_brake = 0;
	times_acceptable=0;
	if(sign==1)moveCarStraight(1.8);
	else moveCarStraight(8.2);
	while(finishCheck());
	pwmVal_servo = SERVORIGHT;

	osDelay(500);

	times_acceptable=0;
	moveCarRight(angle);
	while(finishCheck());
	pwmVal_servo = SERVOCENTER;

	osDelay(500);

	times_acceptable=0;
	if(sign==1)moveCarStraight(-7.5);
	else moveCarStraight(-1);
	while(finishCheck());
}

void moveCarLeft90(double angle)
{
	int sign = 1;
	if(angle<0){
		sign=-1;
	}
	e_brake = 0;

	times_acceptable=0;
	if(sign==1)moveCarStraight(3.3);
	else moveCarStraight(5.2);
	while(finishCheck());

	pwmVal_servo = 106;

	osDelay(500);

	times_acceptable=0;
	moveCarLeft(angle);
	while(finishCheck());
	pwmVal_servo = 149;

	osDelay(200);

	times_acceptable=0;
	if(sign==1)moveCarStraight(-5.2);
	else moveCarStraight(-4);
	while(finishCheck());
}



int PID_Control(int error, int right)
{
	int outputPWM = 0;

	if(right){//rightMotor
		if(error>0){//go forward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel B- forward
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		}else{//go backward
		    HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel B - reverse
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		}
	}else{//leftMotor
		if(error>0){//go forward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel A - forward
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		}else{//go backward
		    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - reverse
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		}
	}


	if(error<0){
		error *= -1;
	}

	if(error > 2000){
		outputPWM += 3200;
	}else if(error > 500){
		outputPWM += 2500;
	}else if(error > 200){
		outputPWM += 1800;
	}else if(error > 100){
		outputPWM += 800;
	}else if(error > 50){
		outputPWM += 400;
	}else if(error >=1){
		times_acceptable++;
		outputPWM += 0;
	}else{
		times_acceptable +=500;;
		outputPWM += 0;
	}
	return outputPWM;

}

int PID_Angle_30(double errord,  int right)
{
	int outputPWM = 0;

	if(right){//rightMotor
		if(errord>0){//go forward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel B- forward
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		}else{//go backward
		    HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel B - reverse
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		}
	}else{//leftMotor
		if(errord<0){//go forward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel A - forward
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		}else{//go backward
		    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - reverse
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		}
	}

	double error = (errord*10);

	if(error<0){
		error *= -1;
	}


	if(error > 300){
		outputPWM += 2500;
	}else if(error > 200){
		outputPWM += 2000;
	}else if(error > 100){
		outputPWM += 1700;
	}else if(error > 50){
		outputPWM += 1600;
	}else if(error > 20){
		outputPWM += 800;
	}else if(error > 10){
		outputPWM += 500;
	}else if(error > 3){
		times_acceptable++;
		outputPWM += 0;
	}else{
		times_acceptable +=500;;
		outputPWM += 0;
	}
	return outputPWM;


//	if(errord < 0.3){
//		times_acceptable +=500;;
//	}
//
//	outputPWM += -16*errord*errord + 573.333*errord;


}

int PID_Angle_90(double errord, int right)
	{
	int outputPWM = 0;

	if(right){//rightMotor
		if(errord>0){//go forward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel B- forward
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		}else{//go backward
		    HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel B - reverse
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		}
	}else{//leftMotor
		if(errord<0){//go forward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel A - forward
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		}else{//go backward
		    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - reverse
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		}
	}

	double error = (errord*10);

	if(error<0){
		error *= -1;
	}

	if(error > 300){
		outputPWM += 2800;
	}else if(error > 200){
		outputPWM += 2100;
	}else if(error > 100){
		outputPWM += 1600;
	}else if(error > 50){
		outputPWM += 1200;
	}else if(error > 20){
		outputPWM += 1100;
	}else if(error > 10){
		outputPWM += 900;
	}else if(error > 3){
		times_acceptable++;
		outputPWM += 900;
	}else{
		times_acceptable +=500;;
		outputPWM += 900;
	}
	return outputPWM;

}

int PID_Juke(double error, int right)
{
	int outputPWM = 0;
	int temp = 1;

	//degree of acceptance will be 28-28.5  //10

	if (error < usTargetGLOBAL){
		error = usTargetGLOBAL*2 - error ;
		temp = -1;
	}


	if(right){//rightMotor
		if(temp>0){//go forward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel B- forward
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		}else{//go backward
		    HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel B - reverse
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		}
	}else{//leftMotor
		if(temp>0){//go forward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel A - forward
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		}else{//go backward
		    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - reverse
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		}
	}

	if(error<0){
		error *= -1;
	}

	if(error > 40){
		outputPWM += 2800;
	}else if(error > usTargetGLOBAL+9){
		outputPWM += 1800;
	}else if(error > usTargetGLOBAL+5){
		outputPWM += 1400;
	}else if(error > usTargetGLOBAL+.5){
		outputPWM += 700; //900
	}else if(error <=usTargetGLOBAL+.5){
		times_acceptable++;
		outputPWM += 0;
	}else{
		times_acceptable +=500;; //500
		outputPWM += 0;
	}


	return outputPWM;

}

int finishCheck(){

	if (times_acceptable > 4){
		e_brake = 1;
		pwmVal_L = pwmVal_R = 0;
		times_acceptable = 0;

		return 0;
	}
	return 1;
}


void readByte(uint8_t addr, uint8_t* data){
	gyroBuffer[0] = addr;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddress<<1, gyroBuffer, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, ICMAddress<<1, data, 2, 20);
}

void writeByte(uint8_t addr, uint8_t data){
	gyroBuffer[0] = addr;
	gyroBuffer[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddress << 1, gyroBuffer, 2, 20);
}

void gyroInit(){
	writeByte(0x06, 0x00);
	osDelay(10);
	writeByte(0x03, 0x80);
	osDelay(10);
	writeByte(0x07, 0x07);
	osDelay(10);
	writeByte(0x06, 0x01);
	osDelay(10);
	writeByte(0x7F, 0x20);
	osDelay(10);
	writeByte(0x01, 0x2F);
	osDelay(10);
	writeByte(0x0, 0x00);
	osDelay(10);
	writeByte(0x7F, 0x00);
	osDelay(10);
	writeByte(0x07, 0x00);
	osDelay(10);
}

void acknowledgeCompletion(){
  uint8_t reply [4] = "OK1\n";
  HAL_UART_Transmit(&huart3, (uint8_t *) reply, 3, 0xFFFF);
//  readyToExecute = 0;
}

void acknowledgeCompletion1(){
  uint8_t reply [4] = "OK2\n";
  HAL_UART_Transmit(&huart3, (uint8_t *) reply, 3, 0xFFFF);
//  readyToExecute = 0;
}

void acknowledgeCompletion2(){
  uint8_t reply [4] = "OK3\n";
  HAL_UART_Transmit(&huart3, (uint8_t *) reply, 3, 0xFFFF);
//  readyToExecute = 0;
}

void debug(const char* reply) {
    HAL_UART_Transmit_IT(&huart3, (uint8_t*)reply, strlen(reply));  // Non-blocking
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUSTask */
/**
  * @brief  Function implementing the ultrasonicTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUSTask */
void StartUSTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t msg [20];
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
	int cnt = 0;
    int flag1st=1;
	int flag2nd = 1;


	uint32_t tick = HAL_GetTick();
//	while(notdone == 1){
//		if(HAL_GetTick()-tick > 1000L)
//		{
//			buzzerBeep(100);
//			tick = HAL_GetTick();
//		}
//	}


  /* Infinite loop */
  uint8_t ch = 'A';
  for(;;)
  {


		if(usflag){
			ultrasonic_read();
			osDelay(10);
		}
		osDelay(20);



		if(flag1st){
			if(/*update==2 &&*/ aRxBuffer[0]=='L' || aRxBuffer[0]=='R'){
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer On
				flag1st = 0;
			}
		}

		if(flag2nd){
			if(/*update==4 &&*/ (aRxBuffer[0]=='L' || aRxBuffer[0]=='R')){
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer On
				flag2nd = 0;
			}
		}



  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
	pwmVal_R = 0;
	pwmVal_L = 0;
	int straightCorrection=0;
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = 149; //Centre

	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel B- forward
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel A - forward
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	osDelay(1000);


  /* Infinite loop */
  for(;;)
  {

	  if (pwmVal_servo!= oldPwmVal_servo){
		  htim1.Instance->CCR4 = pwmVal_servo;
		  oldPwmVal_servo = pwmVal_servo;
		  osDelay(200);
	  	 }
	  htim1.Instance->CCR4 = pwmVal_servo;

		double error_angle = target_angle - total_angle;

		if (pwmVal_servo < 127){ //106 //TURN LEFT


			if(turn90 == 1){
				pwmVal_R = PID_Angle_90(error_angle, 1)*1.072; //right is master
			}else{
				pwmVal_R = PID_Angle_30(error_angle, 1)*1.072; //right is master

			}
			pwmVal_L = pwmVal_R*(0.35); //left is slave 0.56

			if (pwmVal_L>0 && pwmVal_L <900){
				pwmVal_L = 1200;
			}
			if(error_angle>0){//go forward
				HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel A- forward
				HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);}
			else{//go backward
			    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - reverse
				HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);}
		}

		else if (pwmVal_servo > 189){ //230 //TURN RIGHT
			if(turn90 == 1){
				pwmVal_L = PID_Angle_90(error_angle, 0);
			}else{
				pwmVal_L = PID_Angle_30(error_angle, 0);

			}
			pwmVal_R = pwmVal_L*(0.35); //right is slave 0.56
			if (pwmVal_R>0 && pwmVal_R <900){
				pwmVal_R = 1200;
			}


			if(error_angle<0){//go forward
				HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel B- forward
				HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);	}
			else{//go backward
			    HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel B - reverse
				HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);}
		}
		else {
			if(straightUS == 1){
				pwmVal_R = PID_Juke(Distance, 0)*1.072;
			}else{
				pwmVal_R = PID_Control(leftTarget - leftEncoderVal, 0)*1.072;
			}

			if (abs(leftEncoderVal)<abs(rightEncoderVal)){
				straightCorrection++;
			}else{ straightCorrection--;}
			if (pwmVal_R<1000){
				straightCorrection=0;
			}

			if(straightUS == 1){
				pwmVal_L = PID_Juke(Distance, 1)+straightCorrection;
			}else{
				pwmVal_L = PID_Control(rightTarget - rightEncoderVal, 1)+straightCorrection;
			}


			if (errorcorrection == 1){

				if(Distance>usTargetGLOBAL){

					if (error_angle>5){ // if turn left, 106. right 230. left +. right -.
						pwmVal_servo=((-8*5)/5 + 149);
					}
					else if(error_angle<-5){
						pwmVal_servo=((8*5)/5 + 149);
					}else{
						pwmVal_servo=((-8*error_angle)/5 + 149);
					}
				}else{
					if (error_angle>5){ // if turn left, 106. right 230. left +. right -.
						pwmVal_servo=((8*5)/5 + 149);
					}
					else if(error_angle<-5){
						pwmVal_servo=((-8*5)/5 + 149);
					}else{
						pwmVal_servo=((8*error_angle)/5 + 149);
					}
				}
			}
		}


		if(e_brake){
			pwmVal_L = pwmVal_R = 0;
		}


		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_L);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_R);
		osDelay(10);

		if (times_acceptable>1000){
			times_acceptable=1001;
		}

  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
	uint8_t hello [20] = {0};
	uint8_t clear[20] = {0};
	uint8_t lefty[20] = {0};
	uint8_t righty[20] = {0};
	uint8_t motorD[20] = {0};
	uint8_t check[20] = {0};
	uint8_t debugMsg[20] = "hello\0";
	uint32_t adcValue=0;
	uint32_t adcValue2=0;

  /* Infinite loop */
  for(;;)
  {

	//sprintf(clear, "L:%d | R:%d     ", (int)(leftTarget - leftEncoderVal), (int)(rightTarget-rightEncoderVal));

		//testing
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_Start(&hadc2);
//		if (HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK){
//			adcValue = HAL_ADC_GetValue(&hadc1)-4000;
//		}
//		if (HAL_ADC_PollForConversion(&hadc2, 100)==HAL_OK){
//			adcValue2 = HAL_ADC_GetValue(&hadc2)-4000;
//		}
	  //__ADC_Read_Dist(_ADC, dataPoint, IR_data_raw_acc, adcValue, obsTick);
//		sprintf(clear, "IR: %d-%d \0", (int)irDistance1, (int)irDistance2);
//		OLED_ShowString(0, 10, clear);

	sprintf(clear, "USik: %d-%d \0", (int)Distance, nexttask);
	OLED_ShowString(0, 10, clear);


	//sprintf(righty,"Gyro: %d \0", (int)total_angle);
	int decimals = abs((int)((total_angle-(int)(total_angle))*1000));
	if(total_angle)
	sprintf(righty,"Gyro:%3d.%3d \0", (int)total_angle, decimals);
	OLED_ShowString(0, 20, righty);

	//sprintf(lefty, "US: %d\0", (int)uDistance);


//	sprintf(lefty, "Rec: %d/%d  \0", movebackR,movebackL);
	sprintf(lefty, "IRL: %d \0", irDistance1);
	OLED_ShowString(0, 30, lefty);


//	sprintf(motorD, "Enc: %d/%d  \0", rightEncoderVal,rightEncoderVal);
	sprintf(motorD, "IRR: %d \0", irDistance2);
	OLED_ShowString(0, 40, motorD);


	sprintf(check, "K: %c ,%c, %d-%d\0", aRxBuffer[0], nexttask, update, usflag);
	OLED_ShowString(0, 50, check);

	//memset(clear, 0, 20*sizeof(uint8_t));

	OLED_Refresh_Gram();
	osDelay(100);
  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartGyroTask */
/**
* @brief Function implementing the gyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGyroTask */
void StartGyroTask(void *argument)
{
  /* USER CODE BEGIN StartGyroTask */
	gyroInit();
	uint8_t val[2] = {0,0};

	int16_t angular_speed = 0;

	uint32_t tick = 0;

	volatile int ticker=0;
	osDelay(300);
	buzzerBeep(100);
	while(ticker<100){
		buzzerBeep(25);
//		osDelay(50);
		readByte(0x37, val);
		angular_speed = (val[0] << 8) | val[1];
		trash +=(double)((double)angular_speed)*((HAL_GetTick() - tick)/16400.0);
		tick = HAL_GetTick();
		offset += angular_speed;
		ticker++;
	}
	buzzerBeep(100);
	buzzerBeep(100);
	buzzerBeep(100);
	offset = offset/(ticker);
	tick = HAL_GetTick();
	notdone=0;
  /* Infinite loop */
  for(;;)
  {
		osDelay(10);
		readByte(0x37, val);
		angular_speed = (val[0] << 8) | val[1];
		total_angle +=(double)((double)angular_speed - offset)*((HAL_GetTick() - tick)/16400.0);
		tick = HAL_GetTick();
		ticker -= angular_speed;
		ticker++;

//		char hello[50] = {0};
//		double diff = total_angle - old;
//		int decimals = abs((int)((diff-(int)(diff))*10000));
//		int offdeci = abs((int)((offset-(int)(offset))*10000));
//		sprintf(hello,"G%d.%d: %d.%d \0", (int)offset,offdeci,(int)diff, decimals);
//		old = total_angle;
//		HAL_UART_Transmit(&huart3, hello, 20,0xFFFF);
  }

  /* USER CODE END StartGyroTask */
}

/* USER CODE BEGIN Header_StartIRSensorTask */
/**
* @brief Function implementing the irSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIRSensorTask */
void StartIRSensorTask(void *argument)
{
  /* USER CODE BEGIN StartIRSensorTask */
  /* Infinite loop */
	  for(;;)
	  {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		ADC_VAL1 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);


		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3, 10);
		ADC_VAL2 = HAL_ADC_GetValue(&hadc3);
		HAL_ADC_Stop(&hadc3);


		voltage1 = (float) (ADC_VAL1*5)/4095;
		irDistance1 = roundf(29.988 *pow(voltage1 , -1.173));
		voltage2 = (float) (ADC_VAL2*5)/4095;
		irDistance2 = roundf(29.988 *pow(voltage2 , -1.173));

		osDelay(20);


	  }
  /* USER CODE END StartBulleyesTask */
}

/* USER CODE BEGIN Header_StartEncoderRightTask */
/**
* @brief Function implementing the encoderRightTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncoderRightTask */
void StartEncoderRightTask(void *argument)
{
  /* USER CODE BEGIN StartEncoderRightTask */
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	int cnt1;
	int dirR = 1;
	int diff;
	uint32_t tick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
		if(HAL_GetTick()-tick > 10L)
		{
			cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
			if(cnt1 > 32000){
				dirR = 1;
				diff = (65536 - cnt1);
			} else {
				dirR = -1;
				diff = cnt1;

			}

			if(dirR == 1){
				rightEncoderVal -= diff;
			} else {
				rightEncoderVal += diff;
			}

			__HAL_TIM_SET_COUNTER(&htim3, 0);

			tick = HAL_GetTick();
		}
  }
  /* USER CODE END StartEncoderRightTask */
}

/* USER CODE BEGIN Header_StartEncoderLeftTask */
/**
* @brief Function implementing the encoderLeftTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncoderLeftTask */
void StartEncoderLeftTask(void *argument)
{
  /* USER CODE BEGIN StartEncoderLeftTask */
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	int cnt2;
	int dirL = 1;
	int diff;
	uint32_t tick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
		if(HAL_GetTick()-tick > 10L)
		{
			cnt2 = __HAL_TIM_GET_COUNTER(&htim2);

			if(cnt2 > 32000){
				dirL = 1;
				diff = (65536 - cnt2);
			} else {
				dirL = -1;
				diff = cnt2;
			}
			if(dirL == 1){
				leftEncoderVal += diff;
			} else {
				leftEncoderVal -= diff;
			}

			__HAL_TIM_SET_COUNTER(&htim2, 0);

			tick = HAL_GetTick();
		}
  }


  /* USER CODE END StartEncoderLeftTask */
}

/* USER CODE BEGIN Header_StartJukeTask */
/**
* @brief Function implementing the jukeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJukeTask */
void StartJukeTask(void *argument)
{
  /* USER CODE BEGIN StartJukeTask */
	htim1.Instance->CCR4 = (SERVOCENTER + 15); //Centre + 15
	osDelay(100);
	htim1.Instance->CCR4 = (SERVOCENTER - 15); //Centre - 15
	osDelay(100);
	htim1.Instance->CCR4 = pwmVal_servo; //Centre

  /* Infinite loop */
  for(;;)
  {
	while(notdone){
	  osDelay(10);
	}

	osDelay(150);

//	aRxBuffer[0] = 'S';   //HERE IF HARDCODE -------------------------


	while(aRxBuffer[0]!='S'){
		osDelay(5);
	}
	htim1.Instance->CCR4 = (SERVOCENTER + 15); //Centre + 15
	osDelay(100);
	htim1.Instance->CCR4 = (SERVOCENTER - 15); //Centre - 15
	osDelay(100);
	htim1.Instance->CCR4 = pwmVal_servo; //Centre
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer On

	rightEncoderVal = leftEncoderVal=0;
	straightUS = 1;
	errorcorrection = 1;
	times_acceptable=0;
	moveCarStraightSensor(26); //doesnt change the direction // car stops 26 cm from ultrasonic sensor
	//osDelay(500);
	while(finishCheck());
	osDelay(50);
	errorcorrection = 0;
	straightUS = 0;
	movebackL += leftEncoderVal;
	movebackR += rightEncoderVal;

	// try for triangle
	moveBackLeftRun1 += leftEncoderVal;
	moveBackRightRun1 += rightEncoderVal;
	osDelay(5);


//	aRxBuffer[0] = 'L';   //HERE IF HARDCODE-------------------------
//	update=2;  //HERE IF HARDCODE-------------------------

	nexttask = 'Z';
	while(nexttask == 'Z'){
		if(/*update==2 &&*/ aRxBuffer[0]=='L' || aRxBuffer[0]=='R'){
				nexttask = aRxBuffer[0];
		}
		else {
			uint8_t clear[20] = {0};
			sprintf(clear, "HELLO WORLD");
			OLED_ShowString(0, 10, clear);
		}
	}


	turn90 = 0;
	if (nexttask == 'R'){  //FIRST ARROW IS RIGHT

		pwmVal_servo = SERVORIGHT;
		osDelay(25);

		times_acceptable=0;
		moveCarRight(45);
		while(finishCheck());

		pwmVal_servo = SERVOLEFT;
		osDelay(50);

		times_acceptable=0;
		moveCarLeft(85); // car is facing too left
		while(finishCheck());

		pwmVal_servo = SERVORIGHT;
		osDelay(50);

		times_acceptable=0;
		moveCarRight(38); // 35 // 40
		while(finishCheck());

		pwmVal_servo = 149;
		osDelay(25);
		target_angle -= 5;


	}
	else if(nexttask == 'L'){	//FIRST ARROW IS LEFT
		moveCarStraight(-10);
		goodluck =1;
		pwmVal_servo = SERVOLEFT;
		osDelay(25);

		times_acceptable=0;
		moveCarLeft(50);
		while(finishCheck());

		pwmVal_servo = SERVORIGHT;
		osDelay(50);

		times_acceptable=0;
		moveCarRight(95);
		while(finishCheck());

		pwmVal_servo = SERVOLEFT;
		osDelay(50);

		times_acceptable=0;
		moveCarLeft(45);
		while(finishCheck());

		pwmVal_servo = SERVORIGHT;
		osDelay(500);
		pwmVal_servo = 154;
		osDelay(25);
		target_angle += 5;

		acknowledgeCompletion();
	}
	turn90 = 1;


//	aRxBuffer[0] = 'D';   //HERE IF HARDCODE-------------------------

	if(Distance>900 || Distance <5 ){
		errorcorrection = 1;
		times_acceptable=0;
		leftEncoderVal = rightEncoderVal = 0;
		moveCarStraight(-8);
		while(finishCheck());
		errorcorrection = 0;
	}



//	while(aRxBuffer[0]!='D'){
//		osDelay(5);
//	}
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer On


	htim1.Instance->CCR4 = (SERVOCENTER + 15); //Centre + 15
	osDelay(100);
	htim1.Instance->CCR4 = (SERVOCENTER - 15); //Centre - 15
	osDelay(100);
	pwmVal_servo = SERVOCENTER; //kyle  //MOVE TO THE 2ND OBSTACLE


	rightEncoderVal = 0;
	leftEncoderVal = 0;

	straightUS = 1;
	errorcorrection = 1;
	times_acceptable=0;
	moveCarStraightSensor(28);
	while(finishCheck());
	errorcorrection = 0;
	straightUS = 0;
	movebackL += leftEncoderVal;
	movebackR += rightEncoderVal;

	// try for triangle
	moveBackLeftRun2 += leftEncoderVal;
	moveBackRightRun2 += rightEncoderVal;



//	aRxBuffer[0] = 'R';  //HERE IF HARDCODE-------------------------
//	update=4; //HERE IF HARDCODE-------------------------



	nexttask = 'Z';
	while(nexttask == 'Z'){
		if(/*update==4 &&*/ (aRxBuffer[0]=='L' || aRxBuffer[0]=='R')){
				nexttask = aRxBuffer[0];
		}
		else {
			uint8_t clear[20] = {0};
			sprintf(clear, "HELLO UNIVERSE");
			OLED_ShowString(0, 10, clear);
		}
	}

	// 53 - lab
	if (nexttask == 'R'){  //SECOND ARROW IS RIGHT


		pwmVal_servo = SERVORIGHT;
		osDelay(50);
		times_acceptable=0;
		moveCarRight(85); // 88 // 83
		while(finishCheck());
		osDelay(50);
		pwmVal_servo = 149;
		osDelay(200);

//		times_acceptable=0;
//		moveCarStraight(-13); //-13
//		while(finishCheck());
//		//pwmVal_servo = 149;
//		osDelay(100);

		while(irDistance1<30){
			times_acceptable=0;
			moveCarStraight(8);
			while(finishCheck());
			pwmVal_servo = 149;
			osDelay(100);
		}


		pwmVal_servo = 101;
		osDelay(20);
		times_acceptable=0;
		moveCarLeft(175); // 180
		while(finishCheck());
		osDelay(200);

		pwmVal_servo = 240;
		osDelay(500);
		pwmVal_servo = 149;
		osDelay(20);

		errorcorrection = 1;
		times_acceptable=0;
		leftEncoderVal = rightEncoderVal = 0;

//		moveCarStraight(50);
//		while(finishCheck());
		while(irDistance1<30 || obsTwoFlag==0){
			times_acceptable=0;
			osDelay(200);
			moveCarStraight(12);
			while(finishCheck());
			pwmVal_servo = 149;

			obsTwoLength++;
			if (irDistance1<30) obsTwoFlag=1;
		}

		errorcorrection = 0;

		pwmVal_servo = 110;
		osDelay(20);

		times_acceptable=0;
		moveCarLeft(85);
		while(finishCheck());
		pwmVal_servo = 240;
		osDelay(500);
		pwmVal_servo = 154;
		osDelay(20);

		acknowledgeCompletion1();

	}
	else if(nexttask == 'L'){ //SECOND ARROW IS LEFT
		pwmVal_servo = SERVOLEFT;
		osDelay(50);
		times_acceptable=0;
		if(goodluck==0)
			moveCarLeft(90);
		else
			moveCarLeft(82);
		while(finishCheck());
		osDelay(50);
		pwmVal_servo = 220;
		osDelay(1000);
		pwmVal_servo = 149;
		osDelay(20);

//		times_acceptable=0;
//		moveCarStraight(-13); //-13
//		while(finishCheck());
//		//pwmVal_servo = 149;
//		osDelay(100);

		while(irDistance2<30){
			times_acceptable=0;
			moveCarStraight(8);
			while(finishCheck());
			pwmVal_servo = 149;
			osDelay(100);
		}

		pwmVal_servo = 220;
		osDelay(20);

		times_acceptable=0;
		if(goodluck==0)
			moveCarRight(180);
		else
			moveCarRight(170);
		while(finishCheck());
		osDelay(200);

		pwmVal_servo = 149;
		osDelay(20);

		errorcorrection = 1;
		times_acceptable=0;
		leftEncoderVal = rightEncoderVal = 0;

//		moveCarStraight(50);
//		while(finishCheck());
		while(irDistance2<30 || obsTwoFlag==0){
			times_acceptable=0;
			osDelay(200);
			moveCarStraight(14);
			while(finishCheck());
			pwmVal_servo = 149;

			obsTwoLength++;
			if (irDistance2<30) obsTwoFlag=1;
		}


		errorcorrection = 0;

		pwmVal_servo = SERVORIGHT;
		osDelay(20);

		times_acceptable = 0;
		if (goodluck==0)
			moveCarRight(90);
		else
			moveCarRight(100);
		while(finishCheck());

		pwmVal_servo = 149;
		acknowledgeCompletion1();
		osDelay(20);
	}

	// for slide turning idea
   //movebackR = (movebackR + movebackL)/2 + 11007;
	// move car more than halfway back then slideturn
	movebackR = (moveBackRightRun2 + moveBackLeftRun2)/2 + 13907;
	leftEncoderVal = rightEncoderVal = 0;
	straightUS = 0;
	errorcorrection = 1;
	times_acceptable = 0;
	moveCarStraight((movebackR/75.6) - 84); // - 90
	while(finishCheck());
	errorcorrection = 0;

	// for triangle idea
	// first half
//	int moveBack = 0;
//	moveBack = (moveBackRightRun1 + moveBackLeftRun1)/2 + 11007;
//	leftEncoderVal = rightEncoderVal = 0;
//	straightUS = 0;
//	errorcorrection = 1;
//	times_acceptable = 0;
//	moveCarStraight((moveBack/75.6));
//	while(finishCheck());
//	errorcorrection = 0;

	// second half



	// turn into carpark using slide idea
	if(nexttask == 'R'){ //SECOND ARROW IS RIGHT
//		times_acceptable=0;
//		moveCarSlideLeft(80); //75
//		while(finishCheck());
//		pwmVal_servo = 149;
//		osDelay(20);
		uint8_t hello [20] = {0};

		pwmVal_servo = 100;
		times_acceptable=0;
		moveCarLeft(88); // 90
		while(finishCheck());

		pwmVal_servo = 210;
		osDelay(500);
		pwmVal_servo = 149;
		osDelay(50);
		times_acceptable=0;
		int x = ((((obsTwoLength*14)/2)) > 55)? 55:(((obsTwoLength*14)/2));
		moveCarStraight(x/4.2); // 3.3 // 4
		while(finishCheck());
		osDelay(50);
		uint8_t clear[20] = {0};
		sprintf(clear, "Dist: %d \0", (int)(((obsTwoLength*14)/2)));
		OLED_ShowString(0, 10, clear);

		pwmVal_servo = SERVORIGHT;
		times_acceptable=0;
		moveCarRight(90);
		while(finishCheck());
		osDelay(10);

		pwmVal_servo = 149;
		osDelay(20);

		acknowledgeCompletion2();

	}
	else if(nexttask == 'L'){ //SECOND ARROW IS LEFT
//		times_acceptable=0;
//		moveCarSlideRight(80); //70
//		while(finishCheck());
//		pwmVal_servo = 149;
//		osDelay(20);

		pwmVal_servo = SERVORIGHT;
		times_acceptable=0;
		moveCarRight(70);
		while(finishCheck());


		pwmVal_servo = 149;
		osDelay(50);
		times_acceptable=0;
		int x = ((((obsTwoLength*14)/2)) > 55)? 55:(((obsTwoLength*14)/2));
		moveCarStraight(x/3.3); // 2
		while(finishCheck());
		osDelay(50);
		uint8_t clear[20] = {0};
		sprintf(clear, "Dist: %d \0", (int)(((obsTwoLength*14)/2)));
		OLED_ShowString(0, 10, clear);


		pwmVal_servo = 106;
		times_acceptable=0;
		moveCarLeft(70);
		while(finishCheck());
		osDelay(10);

		pwmVal_servo = 149;
		osDelay(20);

		acknowledgeCompletion2();

	}

	pwmVal_servo = 149;
	osDelay(10);
	rightEncoderVal = 0;
	leftEncoderVal = 0;

	straightUS = 1;
	errorcorrection = 1;
	times_acceptable=0;
	moveCarStraightSensor(12);
	while(finishCheck());
	errorcorrection = 0;
	straightUS = 0;

	// turn into carpark using triangle idea


	osDelay(10000000);

  }
  /* USER CODE END StartJukeTask */
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
