#include "app_log_sd.h"
#include "bsp_driver_sd.h"
#include "stm32f4xx_hal_sd.h"
#include <stdio.h>
#include <string.h>

#define APP_LOG_SD_HEADER "time,torque,thrust,rpm,power,status\r\n"
#define APP_LOG_SD_RETRY_BASE_MS 30000U
#define APP_LOG_SD_RETRY_MAX_MS  300000U

typedef enum
{
  APP_LOG_SD_STAGE_NONE = 0,
  APP_LOG_SD_STAGE_MOUNT,
  APP_LOG_SD_STAGE_OPEN,
  APP_LOG_SD_STAGE_LSEEK,
  APP_LOG_SD_STAGE_HEADER,
  APP_LOG_SD_STAGE_WRITE,
  APP_LOG_SD_STAGE_SYNC
} app_log_sd_stage_t;

static uint8_t AppLogSd_SameDate(const RTC_DateTypeDef *a, const RTC_DateTypeDef *b)
{
  if ((a == NULL) || (b == NULL))
  {
    return 0U;
  }

  return (uint8_t)((a->Year == b->Year) &&
                   (a->Month == b->Month) &&
                   (a->Date == b->Date));
}

static void AppLogSd_ResetStorageState(app_log_sd_ctx_t *ctx)
{
  if (ctx == NULL)
  {
    return;
  }

  if (ctx->file_opened != 0U)
  {
    (void)f_close(&ctx->file);
  }

  ctx->mounted = 0U;
  ctx->file_opened = 0U;
  ctx->file_header_written = 0U;
  ctx->current_file_date_valid = 0U;
  ctx->current_file_path[0] = '\0';
}

static uint8_t AppLogSd_IsRemovalLikeError(FRESULT fr, uint32_t hal_err)
{
  if ((fr == FR_NOT_READY) || (fr == FR_INVALID_OBJECT) || (fr == FR_DISK_ERR))
  {
    return 1U;
  }

  if (hal_err != 0U)
  {
    return 1U;
  }

  return 0U;
}

static uint32_t AppLogSd_GetBackoffMs(app_log_sd_ctx_t *ctx)
{
  uint32_t delay_ms = APP_LOG_SD_RETRY_BASE_MS;
  uint8_t i;

  if (ctx == NULL)
  {
    return APP_LOG_SD_RETRY_BASE_MS;
  }

  for (i = 0U; i < ctx->retry_backoff_level; i++)
  {
    if (delay_ms >= (APP_LOG_SD_RETRY_MAX_MS / 2U))
    {
      return APP_LOG_SD_RETRY_MAX_MS;
    }
    delay_ms *= 2U;
  }

  if (delay_ms > APP_LOG_SD_RETRY_MAX_MS)
  {
    delay_ms = APP_LOG_SD_RETRY_MAX_MS;
  }

  return delay_ms;
}

static uint8_t AppLogSd_ProbeCard(app_log_sd_ctx_t *ctx)
{
  extern SD_HandleTypeDef hsd;

  if (ctx == NULL)
  {
    return 0U;
  }

  AppLogSd_ResetStorageState(ctx);
  (void)HAL_SD_GetError(&hsd);

  if (BSP_SD_Init() != MSD_OK)
  {
    return 0U;
  }

  if (BSP_SD_GetCardState() != SD_TRANSFER_OK)
  {
    return 0U;
  }

  ctx->probe_ready = 1U;
  return 1U;
}

static void AppLogSd_LogFailure(app_log_sd_ctx_t *ctx,
                                app_log_sd_stage_t stage,
                                FRESULT fr,
                                uint32_t line_index)
{
  uint32_t card_state = 0xFFFFFFFFUL;
  uint32_t hal_err = 0xFFFFFFFFUL;
  uint32_t hal_state = 0xFFFFFFFFUL;
  const char *stage_name = "unknown";
  extern SD_HandleTypeDef hsd;

  if (ctx == NULL)
  {
    return;
  }

  switch (stage)
  {
    case APP_LOG_SD_STAGE_MOUNT: stage_name = "mount"; break;
    case APP_LOG_SD_STAGE_OPEN:  stage_name = "open";  break;
    case APP_LOG_SD_STAGE_LSEEK: stage_name = "lseek"; break;
    case APP_LOG_SD_STAGE_HEADER: stage_name = "header"; break;
    case APP_LOG_SD_STAGE_WRITE: stage_name = "write"; break;
    case APP_LOG_SD_STAGE_SYNC:  stage_name = "sync";  break;
    default: break;
  }

  if (ctx->mounted != 0U)
  {
    card_state = (uint32_t)BSP_SD_GetCardState();
  }

  hal_err = (uint32_t)HAL_SD_GetError(&hsd);
  hal_state = (uint32_t)HAL_SD_GetState(&hsd);

  LOGE("[SD] fail stage=%s fr=%u line=%lu mounted=%u opened=%u card=%lu herr=0x%08lX hstate=%lu path=%s\r\n",
       stage_name,
       (unsigned int)fr,
       (unsigned long)line_index,
       (unsigned int)ctx->mounted,
       (unsigned int)ctx->file_opened,
       (unsigned long)card_state,
       (unsigned long)hal_err,
       (unsigned long)hal_state,
       (ctx->current_file_path[0] != '\0') ? ctx->current_file_path : "(none)");
}

static FRESULT AppLogSd_EnsureMounted(app_log_sd_ctx_t *ctx)
{
  FRESULT fr;

  if ((ctx == NULL) || (ctx->fatfs == NULL) || (ctx->sd_path == NULL))
  {
    return FR_INVALID_OBJECT;
  }

  if (ctx->mounted != 0U)
  {
    return FR_OK;
  }

  fr = f_mount(ctx->fatfs, (TCHAR const *)ctx->sd_path, 1);
  if (fr == FR_OK)
  {
    ctx->mounted = 1U;
  }
  return fr;
}

static FRESULT AppLogSd_OpenDailyFile(app_log_sd_ctx_t *ctx, const app_log_sd_entry_t *first_entry)
{
  FRESULT fr;
  FSIZE_t size_before = 0U;

  if ((ctx == NULL) || (first_entry == NULL))
  {
    return FR_INVALID_OBJECT;
  }

  if ((ctx->file_opened != 0U) &&
      (ctx->current_file_date_valid != 0U) &&
      AppLogSd_SameDate(&ctx->current_file_date, &first_entry->date))
  {
    return FR_OK;
  }

  if (ctx->file_opened != 0U)
  {
    (void)f_close(&ctx->file);
    ctx->file_opened = 0U;
    ctx->file_header_written = 0U;
  }

  (void)snprintf(ctx->current_file_path,
                 sizeof(ctx->current_file_path),
                 "0:/20%02u-%02u-%02u_%02u-%02u-%02u.csv",
                 first_entry->date.Year,
                 first_entry->date.Month,
                 first_entry->date.Date,
                 first_entry->time.Hours,
                 first_entry->time.Minutes,
                 first_entry->time.Seconds);

  fr = f_open(&ctx->file, ctx->current_file_path, FA_OPEN_ALWAYS | FA_WRITE);
  if (fr != FR_OK)
  {
    return fr;
  }

  size_before = f_size(&ctx->file);
  fr = f_lseek(&ctx->file, size_before);
  if (fr != FR_OK)
  {
    (void)f_close(&ctx->file);
    return fr;
  }

  ctx->file_opened = 1U;
  ctx->file_header_written = (uint8_t)((size_before > 0U) ? 1U : 0U);
  ctx->current_file_date = first_entry->date;
  ctx->current_file_date_valid = 1U;
  return FR_OK;
}

static FRESULT AppLogSd_WriteHeaderIfNeeded(app_log_sd_ctx_t *ctx)
{
  FRESULT fr;
  UINT bw = 0U;
  size_t hdr_len = strlen(APP_LOG_SD_HEADER);

  if ((ctx == NULL) || (ctx->file_opened == 0U))
  {
    return FR_INVALID_OBJECT;
  }

  if (ctx->file_header_written != 0U)
  {
    return FR_OK;
  }

  fr = f_write(&ctx->file, APP_LOG_SD_HEADER, (UINT)hdr_len, &bw);
  if ((fr == FR_OK) && (bw == (UINT)hdr_len))
  {
    ctx->file_header_written = 1U;
    return FR_OK;
  }

  return (fr == FR_OK) ? FR_DISK_ERR : fr;
}

static FRESULT AppLogSd_WriteBuffer(app_log_sd_ctx_t *ctx, app_log_sd_buffer_t *buffer,
                                    app_log_sd_stage_t *fail_stage,
                                    uint32_t *fail_line_index)
{
  FRESULT fr;
  UINT bw = 0U;
  char line[APP_LOG_SD_LINE_MAX_LEN];
  int len;
  uint16_t i;

  if ((ctx == NULL) || (buffer == NULL) || (buffer->count == 0U))
  {
    return FR_OK;
  }

  if (fail_stage != NULL) { *fail_stage = APP_LOG_SD_STAGE_NONE; }
  if (fail_line_index != NULL) { *fail_line_index = 0U; }

  fr = AppLogSd_EnsureMounted(ctx);
  if (fr != FR_OK)
  {
    if (fail_stage != NULL) { *fail_stage = APP_LOG_SD_STAGE_MOUNT; }
    return fr;
  }

  for (i = 0U; i < buffer->count; i++)
  {
    if ((ctx->file_opened == 0U) ||
        (ctx->current_file_date_valid == 0U) ||
        (AppLogSd_SameDate(&ctx->current_file_date, &buffer->entries[i].date) == 0U))
    {
      fr = AppLogSd_OpenDailyFile(ctx, &buffer->entries[i]);
      if (fr != FR_OK)
      {
        if (fail_stage != NULL) { *fail_stage = APP_LOG_SD_STAGE_OPEN; }
        if (fail_line_index != NULL) { *fail_line_index = i; }
        return fr;
      }

      fr = AppLogSd_WriteHeaderIfNeeded(ctx);
      if (fr != FR_OK)
      {
        if (fail_stage != NULL) { *fail_stage = APP_LOG_SD_STAGE_HEADER; }
        if (fail_line_index != NULL) { *fail_line_index = i; }
        return fr;
      }
    }

    len = snprintf(line,
                   sizeof(line),
                   "20%02u-%02u-%02u %02u:%02u:%02u,%.6f,%.6f,%.3f,%.6f,%lu\r\n",
                   buffer->entries[i].date.Year,
                   buffer->entries[i].date.Month,
                   buffer->entries[i].date.Date,
                   buffer->entries[i].time.Hours,
                   buffer->entries[i].time.Minutes,
                   buffer->entries[i].time.Seconds,
                   (double)buffer->entries[i].result.torque,
                   (double)buffer->entries[i].result.thrust,
                   (double)buffer->entries[i].result.rpm,
                   (double)buffer->entries[i].result.power,
                   (unsigned long)buffer->entries[i].result.status_flags);
    if ((len <= 0) || (len >= (int)sizeof(line)))
    {
      if (fail_stage != NULL) { *fail_stage = APP_LOG_SD_STAGE_WRITE; }
      if (fail_line_index != NULL) { *fail_line_index = i; }
      return FR_INVALID_PARAMETER;
    }

    fr = f_write(&ctx->file, line, (UINT)len, &bw);
    if ((fr != FR_OK) || (bw != (UINT)len))
    {
      if (fail_stage != NULL) { *fail_stage = APP_LOG_SD_STAGE_WRITE; }
      if (fail_line_index != NULL) { *fail_line_index = i; }
      return (fr == FR_OK) ? FR_DISK_ERR : fr;
    }
  }

  fr = f_sync(&ctx->file);
  if (fr != FR_OK)
  {
    if (fail_stage != NULL) { *fail_stage = APP_LOG_SD_STAGE_SYNC; }
    if (fail_line_index != NULL) { *fail_line_index = buffer->count; }
    return fr;
  }

  return FR_OK;
}

void AppLogSd_Init(app_log_sd_ctx_t *ctx,
                   RTC_HandleTypeDef *hrtc,
                   FATFS *fatfs,
                   const char *sd_path,
                   diag_counters_t *diag)
{
  if (ctx == NULL)
  {
    return;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->hrtc = hrtc;
  ctx->fatfs = fatfs;
  ctx->sd_path = sd_path;
  ctx->diag = diag;
}

void AppLogSd_Push1s(app_log_sd_ctx_t *ctx, const result_1s_t *result)
{
  app_log_sd_buffer_t *active;
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  if ((ctx == NULL) || (result == NULL) || (ctx->hrtc == NULL))
  {
    return;
  }

  if ((HAL_RTC_GetTime(ctx->hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) ||
      (HAL_RTC_GetDate(ctx->hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK))
  {
    if (ctx->diag != NULL) { ctx->diag->sd_write_fail++; }
    return;
  }

  active = &ctx->buffers[ctx->active_buffer_index];
  if (active->count >= APP_LOG_SD_LINES_PER_MINUTE)
  {
    if (ctx->flush_buffer_ready != 0U)
    {
      if (ctx->diag != NULL) { ctx->diag->sd_write_fail++; }
      return;
    }

    ctx->flush_buffer_index = ctx->active_buffer_index;
    ctx->flush_buffer_ready = 1U;
    ctx->active_buffer_index ^= 1U;
    active = &ctx->buffers[ctx->active_buffer_index];
    active->count = 0U;
  }

  active->entries[active->count].date = sDate;
  active->entries[active->count].time = sTime;
  active->entries[active->count].result = *result;
  active->count++;

  if (active->count >= APP_LOG_SD_LINES_PER_MINUTE)
  {
    if (ctx->flush_buffer_ready == 0U)
    {
      ctx->flush_buffer_index = ctx->active_buffer_index;
      ctx->flush_buffer_ready = 1U;
      ctx->active_buffer_index ^= 1U;
      ctx->buffers[ctx->active_buffer_index].count = 0U;
    }
  }
}

void AppLogSd_Process(app_log_sd_ctx_t *ctx)
{
  app_log_sd_buffer_t *flush_buffer;
  FRESULT fr;
  app_log_sd_stage_t fail_stage = APP_LOG_SD_STAGE_NONE;
  uint32_t fail_line_index = 0U;
  uint32_t now_tick;
  extern SD_HandleTypeDef hsd;
  uint32_t hal_err;

  if ((ctx == NULL) || (ctx->flush_buffer_ready == 0U))
  {
    return;
  }

  now_tick = HAL_GetTick();
  if ((ctx->next_retry_tick_ms != 0U) && ((int32_t)(now_tick - ctx->next_retry_tick_ms) < 0))
  {
    return;
  }

  if (ctx->card_offline != 0U)
  {
    if (AppLogSd_ProbeCard(ctx) == 0U)
    {
      if (ctx->retry_backoff_level < 4U)
      {
        ctx->retry_backoff_level++;
      }
      ctx->next_retry_tick_ms = now_tick + AppLogSd_GetBackoffMs(ctx);
      return;
    }

    ctx->card_offline = 0U;
    ctx->next_retry_tick_ms = now_tick + 100U;
    return;
  }

  if (ctx->probe_ready != 0U)
  {
    ctx->probe_ready = 0U;
  }

  flush_buffer = &ctx->buffers[ctx->flush_buffer_index];
  fr = AppLogSd_WriteBuffer(ctx, flush_buffer, &fail_stage, &fail_line_index);
  if (fr != FR_OK)
  {
    hal_err = (uint32_t)HAL_SD_GetError(&hsd);
    AppLogSd_LogFailure(ctx, fail_stage, fr, fail_line_index);
    AppLogSd_ResetStorageState(ctx);

    if (AppLogSd_IsRemovalLikeError(fr, hal_err) != 0U)
    {
      ctx->card_offline = 1U;
      ctx->probe_ready = 0U;
      ctx->next_retry_tick_ms = now_tick + AppLogSd_GetBackoffMs(ctx);
      if (ctx->retry_backoff_level < 4U)
      {
        ctx->retry_backoff_level++;
      }
    }
    else
    {
      ctx->retry_backoff_level = 0U;
      ctx->next_retry_tick_ms = now_tick + 5000U;
    }

    if (ctx->diag != NULL) { ctx->diag->sd_write_fail++; }
    return;
  }

  flush_buffer->count = 0U;
  ctx->flush_buffer_ready = 0U;
  ctx->card_offline = 0U;
  ctx->probe_ready = 0U;
  ctx->retry_backoff_level = 0U;
  ctx->next_retry_tick_ms = 0U;
  if (ctx->diag != NULL) { ctx->diag->sd_write_ok++; }
}
