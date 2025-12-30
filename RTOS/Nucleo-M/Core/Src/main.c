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

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
osThreadId Cruize_LogicHandle;
osThreadId Cruize_ControlHandle;
/* USER CODE BEGIN PV */

#define SAFE_DECEL  3.0f  		// 감속 제어 값
#define SAFE_ACCEL   2.5f  		// 가속 제어 값
#define PWM_MAX_VALUE 4499      // 큐브MX에서 설정한 Counter Period (ARR) 값에 맞춰 수정하세요
#define PWM_STEP 5.0f     // 가속도 당 PWM 변화 가중치 (테스트 후 조정 필요)
#define INF_DIST 99.0f
#define DETECT_TIMEOUT_MS 1000

typedef enum{
	Front = 0,
	Rear = 1
} DetectionDir;

typedef enum {
    ACTION_MAINTAIN = 0,
    ACTION_DECELERATE,
    ACTION_ACCELERATE
} ActionState;

typedef struct {		//주행 시스템 위한 structure
	float distFront;
	uint8_t detectedFront;      // 전방 물체 유무
	    // 후방 데이터
	float distRear;
	uint8_t detectedRear;       // 후방 물체 유무

	float targetSpeed;          // 내 설정 속도 (%)
	float safeDistThreshold;    // 안전거리 임계값
	uint8_t isEnabled;
} VehicleStatus;

typedef struct {
    VehicleStatus vs;
    uint32_t lastFrontTick;
    uint32_t lastRearTick;
} VehicleBoard;

uint8_t spi_tx_dummy[PACKET_SIZE] = {0};
int32_t current_duty = 0;       // 현재 적용된 PWM Duty 값

volatile VehicleBoard g_board;

VehicleStatus g_status;
volatile ActionState currentAction = ACTION_MAINTAIN;			//현재 차량의 속도 상태
Shared_Buffer_t g_spi_buf;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void Cruize_LogicTask(void const * argument);
void Cruize_ControlTask(void const * argument);

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  memset((void*)&g_board, 0, sizeof(g_board));
  g_board.vs.distFront = INF_DIST;
  g_board.vs.distRear  = INF_DIST;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  g_status.isEnabled = 1; // 테스트를 위해 기본 활성화
  g_status.targetSpeed = 80.0f;

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
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Cruize_Logic */
  osThreadDef(Cruize_Logic, Cruize_LogicTask, osPriorityIdle, 0, 128);
  Cruize_LogicHandle = osThreadCreate(osThread(Cruize_Logic), NULL);

  /* definition and creation of Cruize_Control */
  osThreadDef(Cruize_Control, Cruize_ControlTask, osPriorityHigh, 0, 128);
  Cruize_ControlHandle = osThreadCreate(osThread(Cruize_Control), NULL);

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static inline uint32_t nowTickISR(void){
    return xTaskGetTickCountFromISR(); // ISR에서만
}

static inline uint32_t nowTick(void){
    return xTaskGetTickCount();        // Task에서만
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
    if (g_spi_buf.data.detected == 0)
        goto restart;

    // 4) front/rear 판별 기준 (지금은 bbox_x로)
    //    bbox_x > 0 : front, else rear
    uint32_t now = xTaskGetTickCountFromISR();

    // ISR 크리티컬 (FreeRTOS)
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    if (g_spi_buf.data.bbox_x > 0) {
        g_board.vs.detectedFront = 1;
        g_board.vs.distFront = g_spi_buf.data.distance;
        g_board.lastFrontTick = now;
    } else {
        g_board.vs.detectedRear = 1;
        g_board.vs.distRear = g_spi_buf.data.distance;
        g_board.lastRearTick = now;
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

static void DecideActionFromBoard(void)
{
    // 안전거리 계산
    g_status.safeDistThreshold = (g_status.targetSpeed * 0.2f) + 2.0f;

    // 상황판 스냅샷을 g_status로 복사해서 쓰면 더 안전
    taskENTER_CRITICAL();
    g_status.detectedFront = g_board.vs.detectedFront;
    g_status.distFront     = g_board.vs.distFront;
    g_status.detectedRear  = g_board.vs.detectedRear;
    g_status.distRear      = g_board.vs.distRear;
    taskEXIT_CRITICAL();

    if (g_status.detectedFront && g_status.distFront < g_status.safeDistThreshold) {
        currentAction = ACTION_DECELERATE;
    }
    else if (g_status.detectedRear && g_status.distRear < g_status.safeDistThreshold) {
        currentAction = ACTION_ACCELERATE;
    }
    else {
        currentAction = ACTION_MAINTAIN;
    }
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
    current_duty += (int32_t)(accel_value * PWM_STEP);

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
    current_duty -= (int32_t)(decel_value * PWM_STEP);

    // 3. Min Limit 제한 (정지 상태 이하로 떨어지지 않게)
    if (current_duty < 0) current_duty = 0;

    // 4. TIM2 채널 1에 PWM 적용
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)current_duty);
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

/* USER CODE BEGIN Header_Cruize_LogicTask */
/**
* @brief Function implementing the Cruize_Logic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cruize_LogicTask */
void Cruize_LogicTask(void const * argument)
{
  /* USER CODE BEGIN Cruize_LogicTask */
	HAL_SPI_TransmitReceive_IT(&hspi1, spi_tx_dummy, g_spi_buf.buffer, PACKET_SIZE);
  /* Infinite loop */
    for (;;)
    {
        if (!g_status.isEnabled) {
            osDelay(50);
            continue;
        }

        BoardTimeoutUpdate();
        DecideActionFromBoard();

        osDelay(10);
    }
  /* USER CODE END Cruize_LogicTask */
}

/* USER CODE BEGIN Header_Cruize_ControlTask */
/**
* @brief Function implementing the Cruize_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cruize_ControlTask */
void Cruize_ControlTask(void const * argument)
{
  /* USER CODE BEGIN Cruize_ControlTask */
  /* Infinite loop */
  for(;;)
  {
	  if(g_status.isEnabled) {
      switch(currentAction) {
          case ACTION_ACCELERATE: Apply_Acceleration(SAFE_ACCEL); break;
          case ACTION_DECELERATE: Apply_Deceleration(SAFE_DECEL); break;
          case ACTION_MAINTAIN:   Maintain_TargetSpeed(g_status.targetSpeed); break;
      }
  }
    osDelay(20);
  }
  /* USER CODE END Cruize_ControlTask */
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
