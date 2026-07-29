#include "app_ext_rtc_sd2506.h"
#include <string.h>

#define SD2506_REG_TIME_START 0x00U
#define SD2506_TIME_REG_COUNT 7U
#define SD2506_I2C_TIMEOUT_MS 100U

static uint8_t AppExtRtcSd2506_BinToBcd(uint8_t value)
{
  return (uint8_t)(((value / 10U) << 4) | (value % 10U));
}

static uint8_t AppExtRtcSd2506_BcdToBin(uint8_t value)
{
  return (uint8_t)(((value >> 4) * 10U) + (value & 0x0FU));
}

static uint8_t AppExtRtcSd2506_RtcWeekdayToSd(uint8_t rtc_weekday)
{
  return (rtc_weekday == RTC_WEEKDAY_SUNDAY) ? 0U : rtc_weekday;
}

static uint8_t AppExtRtcSd2506_SdWeekdayToRtc(uint8_t sd_weekday)
{
  sd_weekday &= 0x07U;
  return (sd_weekday == 0U) ? RTC_WEEKDAY_SUNDAY : sd_weekday;
}

static uint8_t AppExtRtcSd2506_IsValid(const RTC_TimeTypeDef *time, const RTC_DateTypeDef *date)
{
  if ((time == NULL) || (date == NULL))
  {
    return 0U;
  }
  if ((time->Hours > 23U) || (time->Minutes > 59U) || (time->Seconds > 59U))
  {
    return 0U;
  }
  if ((date->Year > 99U) || (date->Month < 1U) || (date->Month > 12U) ||
      (date->Date < 1U) || (date->Date > 31U) ||
      (date->WeekDay < RTC_WEEKDAY_MONDAY) || (date->WeekDay > RTC_WEEKDAY_SUNDAY))
  {
    return 0U;
  }
  return 1U;
}

HAL_StatusTypeDef AppExtRtcSd2506_ReadDateTime(I2C_HandleTypeDef *hi2c,
                                               RTC_TimeTypeDef *time,
                                               RTC_DateTypeDef *date)
{
  uint8_t regs[SD2506_TIME_REG_COUNT];

  if ((hi2c == NULL) || (time == NULL) || (date == NULL))
  {
    return HAL_ERROR;
  }

  if (HAL_I2C_Mem_Read(hi2c,
                       SD2506_I2C_DEV_ADDR,
                       SD2506_REG_TIME_START,
                       I2C_MEMADD_SIZE_8BIT,
                       regs,
                       sizeof(regs),
                       SD2506_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  memset(time, 0, sizeof(*time));
  memset(date, 0, sizeof(*date));

  time->Seconds = AppExtRtcSd2506_BcdToBin((uint8_t)(regs[0] & 0x7FU));
  time->Minutes = AppExtRtcSd2506_BcdToBin((uint8_t)(regs[1] & 0x7FU));
  time->Hours = AppExtRtcSd2506_BcdToBin((uint8_t)(regs[2] & 0x3FU));
  time->TimeFormat = RTC_HOURFORMAT12_AM;
  time->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time->StoreOperation = RTC_STOREOPERATION_RESET;

  date->WeekDay = AppExtRtcSd2506_SdWeekdayToRtc(regs[3]);
  date->Date = AppExtRtcSd2506_BcdToBin((uint8_t)(regs[4] & 0x3FU));
  date->Month = AppExtRtcSd2506_BcdToBin((uint8_t)(regs[5] & 0x1FU));
  date->Year = AppExtRtcSd2506_BcdToBin(regs[6]);

  return (AppExtRtcSd2506_IsValid(time, date) != 0U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef AppExtRtcSd2506_WriteDateTime(I2C_HandleTypeDef *hi2c,
                                                const RTC_TimeTypeDef *time,
                                                const RTC_DateTypeDef *date)
{
  uint8_t regs[SD2506_TIME_REG_COUNT];

  if ((hi2c == NULL) || (AppExtRtcSd2506_IsValid(time, date) == 0U))
  {
    return HAL_ERROR;
  }

  regs[0] = AppExtRtcSd2506_BinToBcd(time->Seconds);
  regs[1] = AppExtRtcSd2506_BinToBcd(time->Minutes);
  regs[2] = AppExtRtcSd2506_BinToBcd(time->Hours); /* 24-hour mode, control bits cleared. */
  regs[3] = AppExtRtcSd2506_RtcWeekdayToSd(date->WeekDay);
  regs[4] = AppExtRtcSd2506_BinToBcd(date->Date);
  regs[5] = AppExtRtcSd2506_BinToBcd(date->Month);
  regs[6] = AppExtRtcSd2506_BinToBcd(date->Year);

  return HAL_I2C_Mem_Write(hi2c,
                           SD2506_I2C_DEV_ADDR,
                           SD2506_REG_TIME_START,
                           I2C_MEMADD_SIZE_8BIT,
                           regs,
                           sizeof(regs),
                           SD2506_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef AppExtRtcSd2506_SyncToInternalRtc(I2C_HandleTypeDef *hi2c,
                                                    RTC_HandleTypeDef *hrtc)
{
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;

  if ((hi2c == NULL) || (hrtc == NULL))
  {
    return HAL_ERROR;
  }
  if (AppExtRtcSd2506_ReadDateTime(hi2c, &time, &date) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_RTC_SetTime(hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_RTC_SetDate(hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef AppExtRtcSd2506_WritePresetDateTime(I2C_HandleTypeDef *hi2c)
{
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;

  memset(&time, 0, sizeof(time));
  memset(&date, 0, sizeof(date));

  time.Hours = EXT_RTC_PRESET_HOUR;
  time.Minutes = EXT_RTC_PRESET_MINUTE;
  time.Seconds = EXT_RTC_PRESET_SECOND;
  time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time.StoreOperation = RTC_STOREOPERATION_RESET;
  date.Year = EXT_RTC_PRESET_YEAR;
  date.Month = EXT_RTC_PRESET_MONTH;
  date.Date = EXT_RTC_PRESET_DATE;
  date.WeekDay = EXT_RTC_PRESET_WEEKDAY;

  return AppExtRtcSd2506_WriteDateTime(hi2c, &time, &date);
}

HAL_StatusTypeDef AppExtRtcSd2506_SetExternalAndInternal(I2C_HandleTypeDef *hi2c,
                                                        RTC_HandleTypeDef *hrtc,
                                                        const RTC_TimeTypeDef *time,
                                                        const RTC_DateTypeDef *date)
{
  if ((hi2c == NULL) || (hrtc == NULL))
  {
    return HAL_ERROR;
  }
  if (AppExtRtcSd2506_WriteDateTime(hi2c, time, date) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_RTC_SetTime(hrtc, (RTC_TimeTypeDef *)time, RTC_FORMAT_BIN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_RTC_SetDate(hrtc, (RTC_DateTypeDef *)date, RTC_FORMAT_BIN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}
