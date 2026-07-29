/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
  uint16_t frame_no;
  float ch1;
  float ch2;
  float board;
  uint32_t timestamp_ms;
  uint8_t valid;
} wireless_sample_t;

typedef struct
{
  float rpm;
  uint32_t period_us;
  uint8_t valid;
} rpm_data_t;

typedef struct
{
  uint32_t timestamp_ms;
  float torque;
  float thrust;
  float rpm;
  float power;
  /*
   * SD CSV status field bit layout:
   * bits 0..7   rpm_count in the 1 s calculation window
   * bits 8..15  wireless sample_count in the 1 s calculation window
   * bit 16      flag_avg_power
   * bit 17      flag_axis_param
   * bit 18      flag_meas_param
   * bits 24..27 uart2_req_fail low nibble
   * bits 28..31 sd_write_fail low nibble
   */
  uint32_t status_flags;
} result_1s_t;

typedef struct
{
  uint32_t uart1_rx_frames;
  uint32_t uart1_sample_ok;
  uint32_t uart1_sample_drop;
  uint32_t uart2_req_ok;
  uint32_t uart2_req_fail;
  uint32_t uart2_req_timeout;
  uint32_t sd_write_ok;
  uint32_t sd_write_fail;
  uint32_t wdg_refresh_ok;
  uint32_t wdg_refresh_skip;
} diag_counters_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DEBUG_UART_ENABLE    1U
#define DEBUG_UART_USE_RS485_DIR 1U
#define WDG_HW_ENABLE        1U
#define WDG_ENABLE_WIRELESS  0U
#define WDG_ENABLE_MODBUS    0U
#define WDG_ENABLE_CALC      0U
#define WDG_ENABLE_TX        0U
#define EVT_SAMPLE_UPDATED    (1UL << 0)
#define EVT_MODBUS_UPDATED    (1UL << 1)
#define EVT_TICK_1S           (1UL << 2)
#define EVT_HEALTH_WIRELESS   (1UL << 8)
#define EVT_HEALTH_MODBUS     (1UL << 9)
#define EVT_HEALTH_CALC       (1UL << 10)
#define EVT_HEALTH_TX         (1UL << 11)
#define EVT_HEALTH_WATCHDOG   (1UL << 12)

#if DEBUG_UART_ENABLE
#define LOGI(...) printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)
void AppDebug_PrintResetCause(void);
#else
#define LOGI(...) ((void)0)
#define LOGE(...) ((void)0)
static inline void AppDebug_PrintResetCause(void) {}
#endif

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
