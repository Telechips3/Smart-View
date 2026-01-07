/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
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

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
osThreadId DoorActingHandle;
osThreadId LogicHandle;
osThreadId ActuatorHandle;
osThreadId DriveModeHandle;
/* USER CODE BEGIN PV */
// ----- Macro -----

// 크루즈
#define SAFE_DECEL  (0.5f)  		// 감속 제어 값
#define SAFE_ACCEL   (0.5f)  		// 가속 제어 값
#define PWM_MAX_VALUE (2000-1)      // 큐브MX에서 설정한 Counter Period (ARR) 값에 맞춰 수정하세요
#define PWM_STEP (0.01f)     // 가속도 당 PWM 변화 가중치 (테스트 후 조정 필요)
#define INF_DIST (99.0f)
#define DETECT_TIMEOUT_MS (5000)

// 문
#define TH_PERSON_WARN_M        (30.0f)
#define TH_VEHICLE_WARN_M       (100.0f)   // 그림 기준: 8m 이하면 Warning(이륜차)
#define TH_VEHICLE_LOCK_M       (50.0f)   // 그림 기준: 8m 이하면 Warning(이륜차)
#define SERVO_PULSE_CLOSE_US    (1000)   // 500은 너무 위험할 수 있음(서보마다 다름)
#define SERVO_PULSE_OPEN_US     (1600)   // 필요시 1500/1800 등으로 캘리브레이션
#define DOOR_OPEN_HOLD_MS       (3000)   // 버튼 눌렀을 때 문 열림 유지시간
#define EMERGENCY_HOLD_MS       (5000)

// ----- 전역 구조체 정의 -----

// 크루즈
typedef enum {
    ACTION_MAINTAIN = 0,
    ACTION_DECELERATE,
    ACTION_ACCELERATE
} ActionState;

typedef enum {
	STOPPED = 0,
	DRIVING
} DrivingState;

typedef struct {				//크루즈 주행 시스템 위한 structure
	float distFront;
	uint8_t detectedFront;      // 전방 물체 유무
	    // 후방 데이터
	float distRear;
	uint8_t detectedRear;       // 후방 물체 유무

	float targetSpeed;          // 내 설정 속도 (%)
	//float safeDistThreshold;    // 안전거리 임계값
	uint8_t isEnabled;

} VehicleStatus;

typedef struct {
    VehicleStatus vs;
    uint32_t lastFrontTick;
    uint32_t lastRearTick;
} VehicleBoard;

// 문
typedef enum {
    MODE_MONITORING = 0,
    MODE_WARNING,
    MODE_LOCK,
	MODE_DRIVING
} SystemState_t;

typedef enum {
    CLASS_PERSON = 0,
    CLASS_BYCYCLE,
	CLASS_VEHICLE,
	CLASS_BIKE,
	CLASS_NONE
} ObjectClass_t;

// ---- 전역 변수 및 구조체 선언 ----
//통합
SemaphoreHandle_t startbtn_sem;
volatile DrivingState Drive_st = DRIVING;

//통신
uint8_t spi_tx_dummy[PACKET_SIZE] = {0};
Shared_Buffer_t g_spi_buf;

//크루즈
int32_t current_duty = (int32_t)(60.0f * (PWM_MAX_VALUE / 100.0f));      									// 현재 적용된 PWM Duty 값
volatile VehicleBoard g_board;
volatile ActionState currentAction = ACTION_MAINTAIN;			//현재 차량의 속도 상태

//문
SemaphoreHandle_t semaphoreH_Door;

volatile uint8_t button_pressed = 0;
volatile float currentDistance = 30.0f;
volatile ObjectClass_t currentClass = CLASS_PERSON;
volatile uint8_t emergency_mode = 0;
volatile SystemState_t currentMode = MODE_MONITORING;
volatile uint32_t lastDoorTick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void DoorActingTask(void const * argument);
void LogicTask(void const * argument);
void ActuatorTask(void const * argument);
void DriveModeTask(void const * argument);

/* USER CODE BEGIN PFP */
static SystemState_t EvaluateMode(float dist, ObjectClass_t cls);
static void BoardTimeoutUpdate(void);
static void DecideActionFromBoard(void);
void Maintain_TargetSpeed(float target);
void Apply_Acceleration(float accel_value);
void Apply_Deceleration(float decel_value);
void Servo_Door(uint32_t* arg_open_until, uint8_t* arg_last_btn);
void DoorForceOpen(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  memset((void*)&g_board, 0, sizeof(g_board));

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  g_board.vs.detectedFront = 0;
  g_board.vs.isEnabled = 1;
  g_board.vs.detectedRear = 0;
  g_board.vs.targetSpeed = 60.0f;
  g_board.vs.distFront = INF_DIST;

  g_board.vs.distRear = INF_DIST;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);   // IN1
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // IN2
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  semaphoreH_Door = xSemaphoreCreateBinary();
  startbtn_sem = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of DoorActing */
  osThreadDef(DoorActing, DoorActingTask, osPriorityNormal, 0, 128);
  DoorActingHandle = osThreadCreate(osThread(DoorActing), NULL);

  /* definition and creation of Logic */
  osThreadDef(Logic, LogicTask, osPriorityAboveNormal, 0, 128);
  LogicHandle = osThreadCreate(osThread(Logic), NULL);

  /* definition and creation of Actuator */
  osThreadDef(Actuator, ActuatorTask, osPriorityNormal, 0, 128);
  ActuatorHandle = osThreadCreate(osThread(Actuator), NULL);

  /* definition and creation of DriveMode */
  osThreadDef(DriveMode, DriveModeTask, osPriorityHigh, 0, 128);
  DriveModeHandle = osThreadCreate(osThread(DriveMode), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t hpw = pdFALSE;

    if (GPIO_Pin == GPIO_PIN_7) // PD7 or 유저버튼 주행 모드 전환
    {
    	xSemaphoreGiveFromISR(startbtn_sem, &hpw);
    	portYIELD_FROM_ISR(hpw);
    }
    if (GPIO_Pin == GPIO_PIN_9) // PE9 비상 버튼
    {
        if (!emergency_mode) emergency_mode = 1;
        emergency_mode = 1;
    }
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;

    // 디버그 LED
    HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
    // 1) Header 체크
    if (g_spi_buf.data.header != 0xAA)
        goto restart;

    // 2) Checksum
    uint8_t cs = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++)
        cs ^= g_spi_buf.buffer[i];

    if (cs != g_spi_buf.data.checksum)
        goto restart;

    // 3) Detected 아니면 무시(너 정책)
    if (g_spi_buf.data.detected > 1)		//detected 0이 앞 1이 뒤
        goto restart;

    // 4) front/rear 판별 기준 (지금은 bbox_x로)
    //    bbox_x > 0 : front, else rear
    uint32_t now = xTaskGetTickCountFromISR();

    // ISR 크리티컬 (FreeRTOS)
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    uint8_t cid = g_spi_buf.data.class_ID;

    if (cid == CLASS_VEHICLE){			//기존 코드 -> 가장 바깥쪽 if / else(cid==classvehicle 다 지워버리면 됨)
    if (g_spi_buf.data.detected == 0) {
        g_board.vs.detectedFront = 1;
        g_board.vs.distFront = g_spi_buf.data.distance;
        g_board.lastFrontTick = now;
    } else {
        g_board.vs.detectedRear = 1;
        g_board.vs.distRear = g_spi_buf.data.distance;
        g_board.lastRearTick = now;
    }
    }
    else{
    	if(g_spi_buf.data.detected == 1 ){
    		currentClass =  (ObjectClass_t)cid;
    		currentDistance = g_spi_buf.data.distance;
    		lastDoorTick = now;
    	}
    }
    taskEXIT_CRITICAL_FROM_ISR(saved);

restart:
    // ⭐ 반드시 재시작
    HAL_SPI_TransmitReceive_IT(
        hspi,
        spi_tx_dummy,
        g_spi_buf.buffer,
        PACKET_SIZE
    );
}

static void BoardTimeoutUpdate(void)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t timeout = pdMS_TO_TICKS(DETECT_TIMEOUT_MS);

    taskENTER_CRITICAL();
    if (g_board.vs.detectedFront && (now - g_board.lastFrontTick > timeout)) {
        g_board.vs.detectedFront = 0;
        g_board.vs.distFront = INF_DIST;
    }
    if (g_board.vs.detectedRear && (now - g_board.lastRearTick > timeout)) {
        g_board.vs.detectedRear = 0;
        g_board.vs.distRear = INF_DIST;
    }
    taskEXIT_CRITICAL();
}

static void DecideActionFromBoard(void)
{

	// 안전거리 계산
    float safeDistThreshold = ((g_board.vs.targetSpeed * 0.2f) + 2.0f);


    // 상황판 스냅샷을 복사해서 쓰면 더 안전
    taskENTER_CRITICAL();
    uint8_t detectedFront_cp 	= g_board.vs.detectedFront;
    float distFront_cp     		= g_board.vs.distFront;
    uint8_t detectedRear_cp  	= g_board.vs.detectedRear;
    float distRear_cp      		= g_board.vs.distRear;
    taskEXIT_CRITICAL();

    //전방거리 먼저 따짐
    if (detectedFront_cp && distFront_cp < safeDistThreshold) {
        currentAction = ACTION_DECELERATE;
    }
    //elif로 후방거리 따짐
    else if (detectedRear_cp && distRear_cp < safeDistThreshold) {
        currentAction = ACTION_ACCELERATE;
    }
    else {
        currentAction = ACTION_MAINTAIN;
    }
}

void Motor_StartKick(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_MAX_VALUE * 0.9); // 70%
  osDelay(500);
}

void Maintain_TargetSpeed(float target) {
	// target 값을 기반으로 PWM 수치를 결정 (단순 맵핑)
	    // 예: target이 0~100(%) 이라면 ARR 값에 비례하게 설정

	    current_duty = (int32_t)(target * (PWM_MAX_VALUE / 100.0f));

	    // 안전을 위한 범위 제한
	    if (current_duty > PWM_MAX_VALUE) current_duty = PWM_MAX_VALUE;
	    if (current_duty < 0) current_duty = 0;

	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)current_duty);
}

void Apply_Acceleration(float accel_value)
{
    // 1. 방향 제어 (Forward) - 모터 드라이버 IN 핀 설정
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);   // IN1
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // IN2

    // 2. 가속도에 따른 Duty 증가
    current_duty += (int32_t)(accel_value * (PWM_MAX_VALUE / 100.0f));

    // 3. Max Limit 제한
    if (current_duty > PWM_MAX_VALUE) current_duty = PWM_MAX_VALUE;

    // 4. TIM2 채널 1에 PWM 적용
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)current_duty);
}

void Apply_Deceleration(float decel_value)
{
    // 1. 방향 유지 (Forward)
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);   // IN1
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // IN2

    // 2. 감속도에 따른 Duty 감소 (decel_value가 음수면 더하기, 양수면 빼기로 로직 통일 필요)
    // 여기서는 decel_value가 -3.0f로 들어온다고 가정하여 더해줍니다.
    current_duty -= (int32_t)(decel_value * (PWM_MAX_VALUE / 100.0f));

    // 3. Min Limit 제한 (정지 상태 이하로 떨어지지 않게)
    if (current_duty < 0) current_duty = 0;

    // 4. TIM2 채널 1에 PWM 적용
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)current_duty);
}

static SystemState_t EvaluateMode(float dist, ObjectClass_t cls)
{
//	if (Drive_st) 							return MODE_DRIVING;	//무조건 driving이면 일단 MODE_DRIVING 확인 후 잠구기

    if (cls == CLASS_BIKE || cls == CLASS_BYCYCLE || cls == CLASS_VEHICLE)
    {
        if (dist <= TH_VEHICLE_LOCK_M)      return MODE_LOCK;
        else if (dist < TH_VEHICLE_WARN_M)  return MODE_WARNING;
        else                                return MODE_MONITORING;
    }
    else if (cls == CLASS_PERSON)
    {
        if (dist <= TH_PERSON_WARN_M)       return MODE_WARNING;
        else                                return MODE_MONITORING;
    }

    return MODE_MONITORING;
}

void Servo_Door(uint32_t* arg_open_until, uint8_t* arg_last_btn){
    uint32_t now = osKernelSysTick(); // ms tick (CMSIS-OS1 환경에서 보통 1ms)

     // 2) LOCK: 문 열림 절대 금지(위험 모드=문 잠금)
     if (currentMode == MODE_LOCK) {
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_PULSE_CLOSE_US);
         *arg_open_until = 0;
         *arg_last_btn = button_pressed;
         return;
     }

     // 3) Monitoring/Warning: 문열림 가능
     // 버튼 "눌림 에지"에서만 open hold 타이머 세팅 (누르고 있는 내내 열림 방지)
     if (button_pressed && !(*arg_last_btn)) {
    	 *arg_open_until = now + DOOR_OPEN_HOLD_MS;
     }
     *arg_last_btn = button_pressed;

     if (*arg_open_until != 0 && (int32_t)(*arg_open_until - now) > 0) {
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_PULSE_OPEN_US);
     } else {
    	*arg_open_until = 0;
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_PULSE_CLOSE_US);
     }
}

void DoorForceOpen(void)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_PULSE_OPEN_US);

        // 5초 보장
	osDelay(EMERGENCY_HOLD_MS);

	emergency_mode = 0;
        // 비상 끝나면 닫힘(그림상 다시 잠그는 쪽이 안전)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_PULSE_CLOSE_US);
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
	HAL_SPI_TransmitReceive_IT(&hspi1, spi_tx_dummy, g_spi_buf.buffer, PACKET_SIZE);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DoorActingTask */
/**
* @brief Function implementing the DoorActing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DoorActingTask */
void DoorActingTask(void const * argument)
{
  /* USER CODE BEGIN DoorActingTask */
  /* Infinite loop */
  for(;;)
  {
      if (emergency_mode) {
          // 비상일 땐 "위험" 표시: 빨강 LED 켬
//            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
          osDelay(30);
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
          osDelay(100);
          continue;
      }
      xSemaphoreTake(semaphoreH_Door, portMAX_DELAY);
      if (currentMode == MODE_MONITORING || Drive_st == DRIVING) { //servo 제외하면 주행중엔 일반 상태와 동일
          // Monitoring: 조용히
//            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
          osDelay(100);
      }
      else if (currentMode == MODE_WARNING) {
          // Warning: 파랑 LED 깜빡 + 부저 짧게
//            HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
//            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);

          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
          osDelay(60);
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

          osDelay(100);
      }
      else { // MODE_LOCK
          // Lock: 빨강 LED 켬(위험 느낌)
//            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);

          // 부저는 취향인데, 너무 시끄러우면 끄고, 원하면 짧게 삑삑
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
          osDelay(30);
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
          osDelay(100);
      }
  }
  /* USER CODE END DoorActingTask */
}

/* USER CODE BEGIN Header_LogicTask */
/**
* @brief Function implementing the Logic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LogicTask */
void LogicTask(void const * argument)
{
  /* USER CODE BEGIN LogicTask */

  /* Infinite loop */
  for(;;)
  {
	  	uint32_t now = xTaskGetTickCount();
	  	uint8_t pd7 = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_RESET); // pullup+falling 가정
	  	HAL_GPIO_WritePin(GPIOB, LD2_Pin, pd7 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	  	//HAL_GPIO_WritePin(GPIOB, LD1_Pin, Drive_st ? GPIO_PIN_SET : GPIO_PIN_RESET);

	  	BoardTimeoutUpdate();			//상황판은 계속 최신 상태로 유지 -> 반응성 UP
	    if(Drive_st == DRIVING){
	  	  if (!g_board.vs.isEnabled) {	//cruize가 안켜져있으면 Maintain(주행)으로 continue
	  		  	  	 currentAction = ACTION_MAINTAIN;
	  		  	  	 osDelay(60);
	  	             continue;
	  	         }
	  	  DecideActionFromBoard();   //크루즈 켜져있으면 크루즈 의사결정 함수가 의사결정
	  	  osDelay(60);
	  	  continue;
	    }
	    // 주행중 아닐 때
	    //g_board.vs.isEnabled = 0; // 정지시 크루즈 자동 종료

	    // Door logic
	    // test할 때 if 죽이기
	    if (now - lastDoorTick > pdMS_TO_TICKS(1000)) { // 1초 이상 갱신 없으면
	        currentClass = CLASS_NONE;
	        currentDistance = INF_DIST;
	    }
	    xSemaphoreGive(semaphoreH_Door);
	    currentMode = EvaluateMode(currentDistance, currentClass);
	    //Actuator에서 drive중이면 자동으로 잠구자
	    button_pressed = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12) == GPIO_PIN_RESET) ? 1 : 0;
	    osDelay(40);
	    //HAL_GPIO_TogglePin(GPIOB, LD1_Pin); // <- 눈으로 확인
  }
  /* USER CODE END LogicTask */
}

/* USER CODE BEGIN Header_ActuatorTask */
/**
* @brief Function implementing the Actuator thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ActuatorTask */
void ActuatorTask(void const * argument)
{
  /* USER CODE BEGIN ActuatorTask */
  uint32_t open_until = 0;
  uint8_t last_btn = 0;
  /* Infinite loop */
  for(;;)
  {
		//uint8_t driving_cp;
//		taskENTER_CRITICAL();					//실시간성이 중요하니 Drive_st는 직접 읽기(어차피 여기선 한번만 읽음)
//		driving_cp = Drive_st;
//		taskEXIT_CRITICAL();

		if(emergency_mode){						//emrgency면 door 강제 open 이벤트
			DoorForceOpen();
		}

		if (Drive_st == STOPPED) {				// driving이 0이면 주행 X

		    current_duty = 0;
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		    // ----- pwm 0으로 조절 -----

		    //door actuator
		    Servo_Door(&open_until, &last_btn);
		    osDelay(20);
		    continue;
		}

		//-----주행중일 때------
		//door 강제로 막아버리기
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_PULSE_CLOSE_US);

		//Cruize Actuator
		switch(currentAction) {
			case ACTION_ACCELERATE: Apply_Acceleration(SAFE_ACCEL); break;
			case ACTION_DECELERATE: Apply_Deceleration(SAFE_DECEL); break;
			case ACTION_MAINTAIN:   Maintain_TargetSpeed(g_board.vs.targetSpeed); break;
		}

		osDelay(20);
  }
  /* USER CODE END ActuatorTask */
}

/* USER CODE BEGIN Header_DriveModeTask */
/**
* @brief Function implementing the DriveMode thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DriveModeTask */
void DriveModeTask(void const * argument)
{
  /* USER CODE BEGIN DriveModeTask */
  /* Infinite loop */
	  uint32_t last = 0;

	    for (;;) {
	        xSemaphoreTake(startbtn_sem, portMAX_DELAY);

	        uint32_t now = xTaskGetTickCount();
	        if (now - last < pdMS_TO_TICKS(200)) continue;

	        last = now;
	        if (Drive_st == STOPPED) {
	            Drive_st = DRIVING;
	            Motor_StartKick();
	        } else {
	            Drive_st = STOPPED;
	        }
	        osDelay(1);
	    }


  /* USER CODE END DriveModeTask */
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
