#ifndef __APP_EXT_RTC_SD2506_H
#define __APP_EXT_RTC_SD2506_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal.h"

#define SD2506_I2C_ADDR_7BIT              0x32U
#define SD2506_I2C_DEV_ADDR               (SD2506_I2C_ADDR_7BIT << 1)

/* User config: set to 1 for the first firmware that initializes SD2506 time. */
#define EXT_RTC_WRITE_PRESET_ON_BOOT      0U

/* User config: preset time written when EXT_RTC_WRITE_PRESET_ON_BOOT is 1.
 * Year is 0..99, representing 2000..2099. Hours use 24-hour format.
 */
#define EXT_RTC_PRESET_YEAR               26U
#define EXT_RTC_PRESET_MONTH              7U
#define EXT_RTC_PRESET_DATE               31U
#define EXT_RTC_PRESET_WEEKDAY            RTC_WEEKDAY_FRIDAY
#define EXT_RTC_PRESET_HOUR               9U
#define EXT_RTC_PRESET_MINUTE             30U
#define EXT_RTC_PRESET_SECOND             0U

HAL_StatusTypeDef AppExtRtcSd2506_ReadDateTime(I2C_HandleTypeDef *hi2c,
                                               RTC_TimeTypeDef *time,
                                               RTC_DateTypeDef *date);
HAL_StatusTypeDef AppExtRtcSd2506_WriteDateTime(I2C_HandleTypeDef *hi2c,
                                                const RTC_TimeTypeDef *time,
                                                const RTC_DateTypeDef *date);
HAL_StatusTypeDef AppExtRtcSd2506_SyncToInternalRtc(I2C_HandleTypeDef *hi2c,
                                                    RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef AppExtRtcSd2506_WritePresetDateTime(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef AppExtRtcSd2506_SetExternalAndInternal(I2C_HandleTypeDef *hi2c,
                                                        RTC_HandleTypeDef *hrtc,
                                                        const RTC_TimeTypeDef *time,
                                                        const RTC_DateTypeDef *date);

#ifdef __cplusplus
}
#endif

#endif
