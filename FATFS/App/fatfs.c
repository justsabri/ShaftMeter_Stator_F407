/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
#include "fatfs.h"
#include "main.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  uint16_t year;

  extern RTC_HandleTypeDef hrtc;

  if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    return 0;
  }
  if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    return 0;
  }

  year = (uint16_t)(2000U + sDate.Year);
  if (year < 1980U)
  {
    year = 1980U;
  }
  if (sDate.Month < 1U)
  {
    sDate.Month = 1U;
  }
  if (sDate.Date < 1U)
  {
    sDate.Date = 1U;
  }

  return ((DWORD)(year - 1980U) << 25) |
         ((DWORD)sDate.Month << 21) |
         ((DWORD)sDate.Date << 16) |
         ((DWORD)sTime.Hours << 11) |
         ((DWORD)sTime.Minutes << 5) |
         ((DWORD)(sTime.Seconds / 2U));
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
