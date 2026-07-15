#ifndef __APP_LOG_SD_H
#define __APP_LOG_SD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fatfs.h"

#define APP_LOG_SD_LINES_PER_MINUTE 60U
#define APP_LOG_SD_LINE_MAX_LEN     160U

typedef struct
{
  RTC_DateTypeDef date;
  RTC_TimeTypeDef time;
  result_1s_t result;
} app_log_sd_entry_t;

typedef struct
{
  app_log_sd_entry_t entries[APP_LOG_SD_LINES_PER_MINUTE];
  uint16_t count;
} app_log_sd_buffer_t;

typedef struct
{
  RTC_HandleTypeDef *hrtc;
  FATFS *fatfs;
  const char *sd_path;
  diag_counters_t *diag;
  uint8_t mounted;
  uint8_t file_opened;
  uint8_t file_header_written;
  uint8_t active_buffer_index;
  uint8_t flush_buffer_ready;
  uint8_t flush_buffer_index;
  uint8_t current_file_date_valid;
  uint8_t card_offline;
  uint8_t probe_ready;
  uint8_t retry_backoff_level;
  uint32_t next_retry_tick_ms;
  RTC_DateTypeDef current_file_date;
  char current_file_path[48];
  FIL file;
  app_log_sd_buffer_t buffers[2];
} app_log_sd_ctx_t;

void AppLogSd_Init(app_log_sd_ctx_t *ctx,
                   RTC_HandleTypeDef *hrtc,
                   FATFS *fatfs,
                   const char *sd_path,
                   diag_counters_t *diag);

void AppLogSd_Push1s(app_log_sd_ctx_t *ctx, const result_1s_t *result);

void AppLogSd_Process(app_log_sd_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif
