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
#include "cmsis_os.h"
#include "fatfs.h"
#include "rpm.h"
#include "modbus_master.h"
#include "protocol_map.h"
#include "protocol_values.h"
#include "app_log_sd.h"
#include "app_tx_result.h"
#include "app_usart1.h"
#include "app_modbus_u2.h"
#include "app_calc.h"
#include "app_avg_power_cache.h"
#include "app_param_store.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART1_RX_DMA_BUF_SIZE 128U
#define RPM_PULSES_PER_REV      1U
#define RPM_MIN_PERIOD_US     200U
#define RPM_TIMEOUT_US    10000000U
#define RPM_AVG_WINDOW_US  3000000U
#define RPM_AVG_ENABLE          1U
#define MODBUS_U2_SLAVE_ID      0x01U
#define MODBUS_U2_RX_DMA_SIZE   128U
#define MODBUS_U2_PERIOD_MS    1000U
#define MODBUS_U2_BAUDRATE      9600U
#define MODBUS_U2_USE_8E1      0U
#define SD_LOG_FILE_PATH         "0:/shaft_log.csv"
#define TX_RESULT_PERIOD_MS      1000U
#define TX_RESULT_SLAVE_ID       0x01U
#define TX_RESULT_START_ADDR     0x0000U
#define TX_RESULT_REG_COUNT      8U
#define WDG_CHECK_PERIOD_MS      1000U
#define PC2_ADC_LOW_THRESHOLD_MV 1500U
#define PC2_ADC_LOW_COUNT_LIMIT  2U
#define PC2_ADC_SAMPLE_PERIOD_MS 10U
#define PC2_ADC_VREF_MV          3300U
#define PC2_ADC_MAX_COUNT        4095U
/* IWDG timeout ~ (Reload + 1) * Prescaler / LSI = 2500 * 64 / 32000 = 5s */
#define RTC_INIT_MARKER_VALUE 0xA5A55A5AU
#define TX_ENABLE_USART3_DEFAULT 0U
#define TX_ENABLE_UART6_DEFAULT  0U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;
RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
osThreadId_t wirelessTaskHandle;
osThreadId_t modbusU2TaskHandle;
osThreadId_t calcLogTaskHandle;
osThreadId_t txResultTaskHandle;
osThreadId_t watchdogTaskHandle;
osThreadId_t sdLogTaskHandle;
osThreadId_t monitorTaskHandle;

osMessageQueueId_t qUsart1RxFramesHandle;
osMessageQueueId_t qModbusU2EventsHandle;
osMessageQueueId_t qResult1sHandle;
osEventFlagsId_t evtSystemFlagsHandle;
osMutexId_t mtxSharedDataHandle;

volatile wireless_sample_t g_wireless_latest;
volatile rpm_data_t g_rpm_latest;
volatile result_1s_t g_result_latest;
volatile diag_counters_t g_diag_counters;


const osThreadAttr_t wirelessTask_attributes = {
  .name = "wirelessTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t modbusU2Task_attributes = {
  .name = "modbusU2Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t calcLogTask_attributes = {
  .name = "calcLogTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t txResultTask_attributes = {
  .name = "txResultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

const osThreadAttr_t watchdogTask_attributes = {
  .name = "watchdogTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t sdLogTask_attributes = {
  .name = "sdLogTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

const osThreadAttr_t monitorTask_attributes = {
  .name = "monitorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

static uint8_t usart1_rx_dma_buf[USART1_RX_DMA_BUF_SIZE];
static uint8_t usart2_rx_dma_buf[MODBUS_U2_RX_DMA_SIZE];
static volatile uint8_t usart3_tx_done = 1U;
static volatile uint8_t g_tx_enable_usart3 = TX_ENABLE_USART3_DEFAULT;
static volatile uint8_t g_tx_enable_uart6 = TX_ENABLE_UART6_DEFAULT;
static volatile uint32_t g_pc2_adc_raw = PC2_ADC_MAX_COUNT;
static uint8_t g_pc2_adc_low_count = 0U;
static uint8_t g_board_low_count = 0U;
static app_usart1_ctx_t g_usart1_ctx;
static app_modbus_u2_ctx_t g_modbus_u2_ctx;
static app_calc_ctx_t g_calc_ctx;
static app_log_sd_ctx_t g_sd_log_ctx;
static protocol_values_t g_protocol_values;

static const modbus_poll_item_t s_modbus_07xx_poll_list[] = {
  {0x07D0U, 2U},
  {0x07D2U, 2U},
  {0x07D4U, 2U},
  {0x07D6U, 2U},
  {0x07D8U, 1U},
  {0x07D9U, 1U},
  {0x07DAU, 2U},
  {0x07DCU, 1U},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM5_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);
void StartWirelessTask(void *argument);
void StartModbusU2Task(void *argument);
void StartCalcLogTask(void *argument);
void StartTxResultTask(void *argument);
void StartWatchdogTask(void *argument);
void StartSdLogTask(void *argument);
void StartMonitorTask(void *argument);

/* USER CODE BEGIN PFP */
static void RpmUpdateShared(const rpm_calc_result_t *res);
static void RpmUpdateSharedFromIsr(const rpm_calc_result_t *res);
static void OnWirelessSampleReceived(float ch1, float ch2);
static void OnWirelessBoardSampleReceived(float board);
static void OnWirelessFrameNoReceived(uint16_t frame_no);
static void OnWirelessRxInterrupted(void);
static uint16_t GetWirelessSampleFreqHz(void);
static uint16_t ReadPc2AdcVoltageMv(void);
static void UpdatePc2LowVoltageState(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void AppDebug_PrintResetCause(void)
{
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    LOGI("[RST] IWDG reset\r\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    LOGI("[RST] WWDG reset\r\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
  {
    LOGI("[RST] software reset\r\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
  {
    LOGI("[RST] power-on reset\r\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
  {
    LOGI("[RST] pin reset\r\n");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

int __io_putchar(int ch)
{
#if DEBUG_UART_ENABLE
  uint8_t c = (uint8_t)ch;
#if DEBUG_UART_USE_RS485_DIR
  AppTxResult_SetRs485Usart6(1U);
  if (ch == '\n')
  {
    uint8_t cr = '\r';
    (void)HAL_UART_Transmit(&huart6, &cr, 1U, 10U);
  }
  (void)HAL_UART_Transmit(&huart6, &c, 1U, 10U);
  AppTxResult_SetRs485Usart6(0U);
#else
  if (ch == '\n')
  {
    uint8_t cr = '\r';
    (void)HAL_UART_Transmit(&huart6, &cr, 1U, 10U);
  }
  (void)HAL_UART_Transmit(&huart6, &c, 1U, 10U);
#endif
  return ch;
#else
  (void)ch;
  return ch;
#endif
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
#if WDG_HW_ENABLE
  MX_IWDG_Init();
#endif
  MX_RTC_Init();
  MX_FATFS_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  memset((void *)&g_wireless_latest, 0, sizeof(g_wireless_latest));
  memset((void *)&g_rpm_latest, 0, sizeof(g_rpm_latest));
  memset((void *)&g_result_latest, 0, sizeof(g_result_latest));
  memset((void *)&g_diag_counters, 0, sizeof(g_diag_counters));
  memset((void *)&g_protocol_values, 0, sizeof(g_protocol_values));
  memset((void *)&g_usart1_ctx, 0, sizeof(g_usart1_ctx));
  memset((void *)&g_calc_ctx, 0, sizeof(g_calc_ctx));
  g_protocol_values.torque_kNm = 123.0f; /* 0x07D0 */
  g_protocol_values.power_kW = 222.0f;   /* 0x07D2 */
  g_protocol_values.speed_rpm = 333.3f;  /* 0x07D4 */
  g_protocol_values.thrust_kN = 444.4f;  /* 0x07D6 */
  (void)AppParamStore_Load((protocol_values_t *)&g_protocol_values);
  RPM_Init(RPM_PULSES_PER_REV, RPM_MIN_PERIOD_US, RPM_TIMEOUT_US, RPM_AVG_WINDOW_US, RPM_AVG_ENABLE);
  HAL_TIM_Base_Start(&htim5);
  AppDebug_PrintResetCause();
  LOGI("\r\n[BOOT] ShaftPower start\r\n");
  LOGI("[BOOT] USART6 debug ready, baud=%lu\r\n", (unsigned long)huart6.Init.BaudRate);
  LOGI("[BOOT] USART1/U2/U3/U6 init done\r\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  mtxSharedDataHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  qUsart1RxFramesHandle = osMessageQueueNew(8, sizeof(usart1_frame_msg_t), NULL);
  qModbusU2EventsHandle = osMessageQueueNew(8, sizeof(modbus_event_msg_t), NULL);
  qResult1sHandle = osMessageQueueNew(4, sizeof(result_1s_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_EVENTS */
  evtSystemFlagsHandle = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_EVENTS */

  /* USER CODE BEGIN RTOS_INIT2 */
  g_usart1_ctx.huart = &huart1;
  g_usart1_ctx.hdma_rx = &hdma_usart1_rx;
  g_usart1_ctx.rx_dma_buf = usart1_rx_dma_buf;
  g_usart1_ctx.rx_dma_buf_size = USART1_RX_DMA_BUF_SIZE;
  g_usart1_ctx.q_rx_frames = qUsart1RxFramesHandle;
  g_usart1_ctx.mtx_shared_data = mtxSharedDataHandle;
  g_usart1_ctx.evt_system_flags = evtSystemFlagsHandle;
  g_usart1_ctx.wireless_latest = &g_wireless_latest;
  g_usart1_ctx.diag = &g_diag_counters;
  g_usart1_ctx.on_sample = OnWirelessSampleReceived;
  g_usart1_ctx.on_board_sample = OnWirelessBoardSampleReceived;
  g_usart1_ctx.on_frame_no = OnWirelessFrameNoReceived;
  g_usart1_ctx.on_rx_interrupt = OnWirelessRxInterrupted;
  g_usart1_ctx.get_sample_freq_hz = GetWirelessSampleFreqHz;
  AppUsart1_Init(&g_usart1_ctx);

  g_calc_ctx.mtx_shared_data = mtxSharedDataHandle;
  g_calc_ctx.protocol_values = &g_protocol_values;
  g_calc_ctx.result_latest = &g_result_latest;
  g_calc_ctx.diag = &g_diag_counters;
  AppCalc_Init(&g_calc_ctx);
  AppCalc_RecomputeCoeffs((const protocol_values_t *)&g_protocol_values);
  AppAvgPowerCache_Init();
  AppLogSd_Init(&g_sd_log_ctx, &hrtc, &SDFatFS, SDPath, (diag_counters_t *)&g_diag_counters);

  memset((void *)&g_modbus_u2_ctx, 0, sizeof(g_modbus_u2_ctx));
  g_modbus_u2_ctx.huart2 = &huart2;
  g_modbus_u2_ctx.hdma_usart2_rx = &hdma_usart2_rx;
  g_modbus_u2_ctx.rx_dma_buf = usart2_rx_dma_buf;
  g_modbus_u2_ctx.rx_dma_size = MODBUS_U2_RX_DMA_SIZE;
  g_modbus_u2_ctx.q_modbus_events = qModbusU2EventsHandle;
  g_modbus_u2_ctx.mtx_shared_data = mtxSharedDataHandle;
  g_modbus_u2_ctx.evt_system_flags = evtSystemFlagsHandle;
  g_modbus_u2_ctx.protocol_values = &g_protocol_values;
  g_modbus_u2_ctx.diag = &g_diag_counters;
  g_modbus_u2_ctx.slave_id = MODBUS_U2_SLAVE_ID;
  g_modbus_u2_ctx.poll_list = s_modbus_07xx_poll_list;
  g_modbus_u2_ctx.poll_list_count = sizeof(s_modbus_07xx_poll_list) / sizeof(s_modbus_07xx_poll_list[0]);
  g_modbus_u2_ctx.rs485_set_tx_usart2 = AppTxResult_SetRs485Usart2;
  AppModbusU2_Init(&g_modbus_u2_ctx);
  /* USER CODE END RTOS_INIT2 */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  wirelessTaskHandle = osThreadNew(StartWirelessTask, NULL, &wirelessTask_attributes);
  modbusU2TaskHandle = osThreadNew(StartModbusU2Task, NULL, &modbusU2Task_attributes);
  calcLogTaskHandle = osThreadNew(StartCalcLogTask, NULL, &calcLogTask_attributes);
  txResultTaskHandle = osThreadNew(StartTxResultTask, NULL, &txResultTask_attributes);
  watchdogTaskHandle = osThreadNew(StartWatchdogTask, NULL, &watchdogTask_attributes);
  sdLogTaskHandle = osThreadNew(StartSdLogTask, NULL, &sdLogTask_attributes);
  monitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &monitorTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* events are created before thread startup */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  /* IWDG timeout ~= (Reload + 1) * Prescaler / LSI.
   * Use a valid 12-bit reload value (max 4095). For ~15 s at 32 kHz LSI:
   * (1874 + 1) * 256 / 32000 ~= 15 s.
   */
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 1874;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&g_pc2_adc_raw, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 118;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84 - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
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
  huart2.Init.BaudRate = MODBUS_U2_BAUDRATE;
#if MODBUS_U2_USE_8E1
  huart2.Init.WordLength = UART_WORDLENGTH_9B; /* 8 data + even parity (STM32 HAL rule) */
#else
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
#endif
  huart2.Init.StopBits = UART_STOPBITS_1;
#if MODBUS_U2_USE_8E1
  huart2.Init.Parity = UART_PARITY_EVEN;
#else
  huart2.Init.Parity = UART_PARITY_NONE;
#endif
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
  huart3.Init.BaudRate = 57600;
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
  huart6.Init.BaudRate = 57600;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
  /* SDIO_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SDIO_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SDIO_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  uint32_t marker = 0U;

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  marker = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
  if (marker != RTC_INIT_MARKER_VALUE)
  {
    sTime.Hours = 0;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
      Error_Handler();
    }

    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 1;
    sDate.Year = 26;
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, RTC_INIT_MARKER_VALUE);
  }
}

static void OnWirelessSampleReceived(float ch1, float ch2)
{
  AppCalc_SetForceZeroInput(0U);
  AppCalc_FeedSample(ch1, ch2);
}

static void OnWirelessBoardSampleReceived(float board)
{
  uint16_t state = 0U;

  if (board == 1.0f)
  {
    if (g_board_low_count < 2U)
    {
      g_board_low_count++;
    }
  }
  else
  {
    g_board_low_count = 0U;
  }

  if (g_board_low_count >= 2U)
  {
    state = 1U;
  }

  if ((state != 0U) &&
      (mtxSharedDataHandle != NULL) &&
      (osMutexAcquire(mtxSharedDataHandle, 5U) == osOK))
  {
    g_protocol_values.board_low_voltage_state = 1U;
    osMutexRelease(mtxSharedDataHandle);
  }
}

static void OnWirelessFrameNoReceived(uint16_t frame_no)
{
  if ((mtxSharedDataHandle != NULL) && (osMutexAcquire(mtxSharedDataHandle, 5U) == osOK))
  {
    g_protocol_values.wireless_frame_no = frame_no;
    osMutexRelease(mtxSharedDataHandle);
  }
}

static void OnWirelessRxInterrupted(void)
{
  float zero_ch1 = 0.0f;
  float zero_ch2 = 0.0f;

  if ((mtxSharedDataHandle != NULL) && (osMutexAcquire(mtxSharedDataHandle, 5U) == osOK))
  {
    zero_ch1 = g_protocol_values.zero_voltage[0];
    zero_ch2 = g_protocol_values.zero_voltage[1];
    g_wireless_latest.ch1 = zero_ch1;
    g_wireless_latest.ch2 = zero_ch2;
    g_wireless_latest.valid = 0U;
    g_wireless_latest.timestamp_ms = HAL_GetTick();
    osMutexRelease(mtxSharedDataHandle);
  }
  AppCalc_SetForceZeroInput(1U);
  // LOGI("[WIRELESS] force calc input to zero voltage ch1=%.6f ch2=%.6f\r\n",
  //      (double)zero_ch1,
  //      (double)zero_ch2);
  // LOGI("[WIRELESS] rx interrupted, power-cycle ext supply PC3 low->high\r\n");
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  osDelay(3000U);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  // LOGI("[WIRELESS] ext supply power-cycle complete\r\n");
}

static uint16_t GetWirelessSampleFreqHz(void)
{
  float freq_hz = 200.0f;
  uint16_t freq_u16;

  if ((mtxSharedDataHandle != NULL) && (osMutexAcquire(mtxSharedDataHandle, 2U) == osOK))
  {
    freq_hz = g_protocol_values.sample_freq_Hz;
    osMutexRelease(mtxSharedDataHandle);
  }

  if (freq_hz < 1.0f)
  {
    freq_hz = 200.0f;
  }
  if (freq_hz > 65535.0f)
  {
    freq_hz = 65535.0f;
  }
  freq_u16 = (uint16_t)(freq_hz + 0.5f);
  return (freq_u16 == 0U) ? 200U : freq_u16;
}

static uint16_t ReadPc2AdcVoltageMv(void)
{
  uint32_t raw = g_pc2_adc_raw;
  uint32_t voltage_mv = PC2_ADC_VREF_MV;
  // LOGE("raw %d\n", raw);
  if (raw <= PC2_ADC_MAX_COUNT)
  {
    voltage_mv = ((raw * PC2_ADC_VREF_MV) + (PC2_ADC_MAX_COUNT / 2U)) / PC2_ADC_MAX_COUNT;
  }

  if (voltage_mv > 0xFFFFU)
  {
    voltage_mv = 0xFFFFU;
  }
  return (uint16_t)voltage_mv;
}

static void UpdatePc2LowVoltageState(void)
{
  uint16_t voltage_mv = ReadPc2AdcVoltageMv();
  uint16_t state = 0U;
  // LOGE("V %d\n", voltage_mv);
  if (voltage_mv < PC2_ADC_LOW_THRESHOLD_MV)
  {
    if (g_pc2_adc_low_count < PC2_ADC_LOW_COUNT_LIMIT)
    {
      g_pc2_adc_low_count++;
    }
  }
  else
  {
    g_pc2_adc_low_count = 0U;
  }

  if (g_pc2_adc_low_count >= PC2_ADC_LOW_COUNT_LIMIT)
  {
    state = 1U;
  }

  if ((state != 0U) &&
      (mtxSharedDataHandle != NULL) &&
      (osMutexAcquire(mtxSharedDataHandle, 5U) == osOK))
  {
    g_protocol_values.pc2_low_voltage_state = 1U;
    osMutexRelease(mtxSharedDataHandle);
  }
}

static void RpmUpdateShared(const rpm_calc_result_t *res)
{
  if ((res == NULL) || (mtxSharedDataHandle == NULL))
  {
    return;
  }

  if (osMutexAcquire(mtxSharedDataHandle, 2U) == osOK)
  {
    g_rpm_latest.rpm = res->rpm;
    g_rpm_latest.period_us = res->period_us;
    g_rpm_latest.valid = res->valid;
    osMutexRelease(mtxSharedDataHandle);
  }
}

static void RpmUpdateSharedFromIsr(const rpm_calc_result_t *res)
{
  if (res == NULL)
  {
    return;
  }

  g_rpm_latest.rpm = res->rpm;
  g_rpm_latest.period_us = res->period_us;
  g_rpm_latest.valid = res->valid;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    AppUsart1_OnTxCplt(&g_usart1_ctx);
  }
  else if (huart->Instance == USART2)
  {
    AppModbusU2_OnTxCplt(&g_modbus_u2_ctx);
  }
  else if (huart->Instance == USART3)
  {
    usart3_tx_done = 1U;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if ((huart->Instance == USART1) && (Size > 0U))
  {
    // LOGE("[USART1] HAL RxEvent size=%u\r\n", (unsigned int)Size);
    AppUsart1_OnRxEvent(&g_usart1_ctx, Size);
  }
  else if ((huart->Instance == USART2) && (Size > 0U))
  {
    AppModbusU2_OnRxEvent(&g_modbus_u2_ctx, Size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // LOGE("[USART1] HAL ErrorCallback err=0x%08lX\r\n", (unsigned long)huart->ErrorCode);
    AppUsart1_OnError(&g_usart1_ctx);
  }
  else if (huart->Instance == USART2)
  {
    AppModbusU2_OnError(&g_modbus_u2_ctx);
  }
}

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
  uint32_t last_wake_tick = osKernelGetTickCount();
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    UpdatePc2LowVoltageState();
    last_wake_tick += PC2_ADC_SAMPLE_PERIOD_MS;
    osDelayUntil(last_wake_tick);
  }
  /* USER CODE END 5 */
}

void StartWirelessTask(void *argument)
{
  uint32_t last_wake_tick = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    AppUsart1_Process(&g_usart1_ctx, osKernelGetTickCount());

#if WDG_ENABLE_WIRELESS
    if (evtSystemFlagsHandle != NULL)
    {
      osEventFlagsSet(evtSystemFlagsHandle, EVT_HEALTH_WIRELESS);
    }
#endif
    last_wake_tick += 20U;
    osDelayUntil(last_wake_tick);
  }
}

void StartModbusU2Task(void *argument)
{
  uint32_t last_wake_tick = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    AppModbusU2_Process1Hz(&g_modbus_u2_ctx);
    last_wake_tick += MODBUS_U2_PERIOD_MS;
    osDelayUntil(last_wake_tick);
  }
}

void StartCalcLogTask(void *argument)
{
  uint32_t last_wake_tick = osKernelGetTickCount();
  uint32_t mono_sec = 0U;
  rpm_calc_result_t rpm_res;
  result_1s_t result;
  uint8_t avg_ok = 0U;
  float query_hours = 0.1f;
  float avg_power = 0.0f;
  (void)argument;
  for (;;)
  {
    rpm_res = RPM_OnPeriodicCheck(__HAL_TIM_GET_COUNTER(&htim5));
    RpmUpdateShared(&rpm_res);
    AppCalc_FeedRpm(rpm_res.rpm);
    AppCalc_Compute1s(&g_calc_ctx, &result);
    AppAvgPowerCache_Push1s(result.power, mono_sec);
    mono_sec++;

    if (mtxSharedDataHandle != NULL)
    {
      if (osMutexAcquire(mtxSharedDataHandle, 5U) == osOK)
      {
        query_hours = g_protocol_values.query_time_h;
        avg_power = AppAvgPowerCache_QueryHours(query_hours, &avg_ok);
        if (avg_ok == 0U)
        {
          avg_power = 0.0f;
        }
        g_protocol_values.torque_kNm = result.torque;
        g_protocol_values.power_kW = result.power;
        g_protocol_values.speed_rpm = result.rpm;
        g_protocol_values.thrust_kN = result.thrust;
        g_protocol_values.avg_power_kW = avg_power;
        g_result_latest = result;
        osMutexRelease(mtxSharedDataHandle);
      }
    }
    if (qResult1sHandle != NULL)
    {
      (void)osMessageQueuePut(qResult1sHandle, &result, 0U, 0U);
    }
    AppLogSd_Push1s(&g_sd_log_ctx, &result);

#if WDG_ENABLE_CALC
    if (evtSystemFlagsHandle != NULL)
    {
      osEventFlagsSet(evtSystemFlagsHandle, EVT_HEALTH_CALC);
    }
#endif
    last_wake_tick += 1000U;
    osDelayUntil(last_wake_tick);
  }
}

void StartSdLogTask(void *argument)
{
  uint32_t last_wake_tick = osKernelGetTickCount();
  (void)argument;

  for (;;)
  {
    AppLogSd_Process(&g_sd_log_ctx);
    last_wake_tick += 100U;
    osDelayUntil(last_wake_tick);
  }
}

void StartMonitorTask(void *argument)
{
  uint32_t last_wake_tick = osKernelGetTickCount();
  uint32_t heartbeat_sec = 0U;
  (void)argument;

  for (;;)
  {
    heartbeat_sec += 5U;
    LOGI("[HB] alive sec=%lu rpm_x100=%ld valid=%u sd_ok=%lu sd_fail=%lu\r\n",
         (unsigned long)heartbeat_sec,
         (long)(g_rpm_latest.rpm * 100.0f),
         (unsigned int)g_rpm_latest.valid,
         (unsigned long)g_diag_counters.sd_write_ok,
         (unsigned long)g_diag_counters.sd_write_fail);

    last_wake_tick += 5000U;
    osDelayUntil(last_wake_tick);
  }
}

void StartTxResultTask(void *argument)
{
  uint32_t last_wake_tick = osKernelGetTickCount();
  result_1s_t result = {0};
  uint8_t frame[32];
  uint16_t frame_len;
  protocol_values_t values_snap;
  app_tx_result_ctx_t tx_ctx;
  (void)argument;

  tx_ctx.huart3 = &huart3;
  tx_ctx.huart6 = &huart6;
  tx_ctx.usart3_tx_done = &usart3_tx_done;
  tx_ctx.tx_enable_usart3 = &g_tx_enable_usart3;
  tx_ctx.tx_enable_uart6 = &g_tx_enable_uart6;
  tx_ctx.protocol_values = &g_protocol_values;
  tx_ctx.mtx_shared_data = mtxSharedDataHandle;

  for (;;)
  {
    if ((qResult1sHandle != NULL) &&
        (osMessageQueueGet(qResult1sHandle, &result, NULL, 0U) != osOK))
    {
      if ((mtxSharedDataHandle != NULL) && (osMutexAcquire(mtxSharedDataHandle, 5U) == osOK))
      {
        result = g_result_latest;
        osMutexRelease(mtxSharedDataHandle);
      }
    }

    frame_len = AppTxResult_BuildWriteFrame(TX_RESULT_SLAVE_ID, TX_RESULT_START_ADDR, &result, frame, sizeof(frame));
    if (frame_len > 0U)
    {
      if ((mtxSharedDataHandle != NULL) && (osMutexAcquire(mtxSharedDataHandle, 5U) == osOK))
      {
        values_snap = g_protocol_values;
        osMutexRelease(mtxSharedDataHandle);
        AppTxResult_UpdateEnableFromConfig(&values_snap, &g_tx_enable_usart3, &g_tx_enable_uart6);
      }

      if (g_tx_enable_usart3 != 0U)
      {
        (void)AppTxResult_SendUsart3(&tx_ctx, frame, frame_len, 100U);
      }
      if (g_tx_enable_uart6 != 0U)
      {
#if !DEBUG_UART_ENABLE
        (void)AppTxResult_SendUsart6(&tx_ctx, frame, frame_len, 100U);
#endif
      }
    }

#if WDG_ENABLE_TX
    if (evtSystemFlagsHandle != NULL)
    {
      osEventFlagsSet(evtSystemFlagsHandle, EVT_HEALTH_TX);
    }
#endif
    last_wake_tick += TX_RESULT_PERIOD_MS;
    osDelayUntil(last_wake_tick);
  }
}

void StartWatchdogTask(void *argument)
{
  uint32_t last_wake_tick = osKernelGetTickCount();
  (void)argument;

  for (;;)
  {
#if WDG_HW_ENABLE
    HAL_StatusTypeDef wdg_ret = HAL_IWDG_Refresh(&hiwdg);
    if (wdg_ret == HAL_OK)
    {
      g_diag_counters.wdg_refresh_ok++;
    }
    else
    {
      g_diag_counters.wdg_refresh_skip++;
    }
#endif

    last_wake_tick += WDG_CHECK_PERIOD_MS;
    osDelayUntil(last_wake_tick);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  rpm_calc_result_t rpm_res;
  if (GPIO_Pin == GPIO_PIN_1)
  {
    rpm_res = RPM_OnPulse(__HAL_TIM_GET_COUNTER(&htim5));
    RpmUpdateSharedFromIsr(&rpm_res);
    AppCalc_FeedRpm(rpm_res.rpm);
  }
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
