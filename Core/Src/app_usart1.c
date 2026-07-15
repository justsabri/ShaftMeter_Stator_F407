#include "app_usart1.h"
#include "modbus_master.h"
#include <string.h>

#define USART1_FRAME_MIN_SIZE      19U
#define USART1_HEARTBEAT_MS        2000U
#define USART1_RX_TIMEOUT_MS       3000U
#define USART1_TIMEOUT_RETRY_MAX   3U
#define USART1_RECOVERY_CMD_GAP_MS 100U
#define USART1_FREQ_BASE           19200U
#define USART1_LOG_SAMPLE_EVERY    0U
#define USART1_LOG_INFO            0U
#define USART1_LOG_VERBOSE_FRAME   0U

#if USART1_LOG_INFO
#define U1LOGI(...) LOGI(__VA_ARGS__)
#else
#define U1LOGI(...) do { } while (0)
#endif

static volatile uint8_t s_usart1_tx_done = 1U;
static uint8_t s_hb_frame[9];
static uint8_t s_sampling_active = 0U;
static uint32_t s_last_sample_tick_ms = 0U;
static uint16_t s_last_frame_no = 0U;
static uint8_t s_frame_no_inited = 0U;
static uint32_t s_sample_log_counter = 0U;
static uint8_t s_start_sampling_pending = 0U;
static uint8_t s_rx_accum_buf[256];
static uint16_t s_rx_accum_len = 0U;
static uint16_t s_rx_dma_last_pos = 0U;
static uint8_t s_timeout_retry_count = 0U;

static float AppUsart1_U32ToFloat(uint32_t raw)
{
  union
  {
    uint32_t u32;
    float f;
  } cvt;
  cvt.u32 = raw;
  return cvt.f;
}

static void AppUsart1_Delay1ms(void)
{
  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(1U);
  }
  else
  {
    HAL_Delay(1U);
  }
}

static void AppUsart1_DelayMs(uint32_t delay_ms)
{
  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static uint16_t AppUsart1_FrequencyToCmdValue(uint16_t frequency)
{
  uint32_t value;

  if (frequency == 0U)
  {
    frequency = 1U;
  }
  value = USART1_FREQ_BASE / (uint32_t)frequency;
  if (value == 0U)
  {
    value = 1U;
  }
  if (value > 0xFFFFU)
  {
    value = 0xFFFFU;
  }
  return (uint16_t)value;
}

static void AppUsart1_LogFrame(const char *tag, const uint8_t *frame, uint16_t len)
{
  uint16_t i;

#if !USART1_LOG_VERBOSE_FRAME
  (void)tag;
  (void)frame;
  (void)len;
  return;
#else
  if ((tag == NULL) || (frame == NULL))
  {
    return;
  }

  U1LOGI("[USART1] %s len=%u data=", tag, (unsigned int)len);
  for (i = 0U; i < len; i++)
  {
    U1LOGI("%02X", frame[i]);
    if ((uint16_t)(i + 1U) < len)
    {
      U1LOGI(" ");
    }
  }
  U1LOGI("\r\n");
#endif
}

static void AppUsart1_LogFrameError(const char *tag, const char *reason, const uint8_t *frame, uint16_t len)
{
  uint16_t i;

  if ((tag == NULL) || (frame == NULL))
  {
    return;
  }

  LOGE("[USART1] %s", tag);
  if (reason != NULL)
  {
    LOGE(" reason=%s", reason);
  }
  LOGE(" len=%u data=", (unsigned int)len);
  for (i = 0U; i < len; i++)
  {
    LOGE("%02X", frame[i]);
    if ((uint16_t)(i + 1U) < len)
    {
      LOGE(" ");
    }
  }
  LOGE("\r\n");
}

static void AppUsart1_OnFrameNoDiscontinuity(uint16_t prev_no, uint16_t curr_no)
{
  LOGE("[USART1] frame discontinuity prev=%u curr=%u\r\n",
       (unsigned int)prev_no,
       (unsigned int)curr_no);
}

static void AppUsart1_OnRxInterrupted(const app_usart1_ctx_t *ctx)
{
  if ((ctx != NULL) && (ctx->on_rx_interrupt != NULL))
  {
    ctx->on_rx_interrupt();
  }
  else
  {
#if WDG_ENABLE_WIRELESS
    NVIC_SystemReset();
#endif
  }
}

static uint16_t AppUsart1_GetConfiguredFrequency(const app_usart1_ctx_t *ctx)
{
  uint16_t freq = 200U;

  if ((ctx != NULL) && (ctx->get_sample_freq_hz != NULL))
  {
    freq = ctx->get_sample_freq_hz();
  }
  if (freq == 0U)
  {
    freq = 200U;
  }
  return freq;
}

static void AppUsart1_BuildFrame(uint8_t cmd1, uint8_t cmd2, uint8_t p1, uint8_t p2, uint8_t *out9)
{
  uint16_t crc;
  out9[0] = 0x32U;
  out9[1] = 0x33U;
  out9[2] = 0x06U;
  out9[3] = cmd1;
  out9[4] = cmd2;
  out9[5] = p1;
  out9[6] = p2;
  crc = ModbusMaster_Crc16(out9, 7U);
  out9[7] = (uint8_t)(crc & 0xFFU);
  out9[8] = (uint8_t)(crc >> 8);
}

static HAL_StatusTypeDef AppUsart1_SendFrameDma(const app_usart1_ctx_t *ctx, const uint8_t *frame, uint16_t len, uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();
  while (s_usart1_tx_done == 0U)
  {
    if ((HAL_GetTick() - start_tick) > timeout_ms)
    {
      LOGE("[USART1] tx wait idle timeout=%lu\r\n", (unsigned long)timeout_ms);
      return HAL_TIMEOUT;
    }
    AppUsart1_Delay1ms();
  }

  s_usart1_tx_done = 0U;
  if (HAL_UART_Transmit_DMA(ctx->huart, (uint8_t *)frame, len) != HAL_OK)
  {
    s_usart1_tx_done = 1U;
    LOGE("[USART1] tx dma start failed err=0x%08lX\r\n", (unsigned long)ctx->huart->ErrorCode);
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while (s_usart1_tx_done == 0U)
  {
    if ((HAL_GetTick() - start_tick) > timeout_ms)
    {
      LOGE("[USART1] tx done timeout=%lu\r\n", (unsigned long)timeout_ms);
      return HAL_TIMEOUT;
    }
    AppUsart1_Delay1ms();
  }
  return HAL_OK;
}

static void AppUsart1_ParseSampleFrame(const app_usart1_ctx_t *ctx, const uint8_t *frame, uint16_t len)
{
  wireless_sample_t sample;
  uint16_t frame_no;
  uint16_t expected_no;
  uint16_t crc_calc;
  uint16_t crc_rx;
  if (len < USART1_FRAME_MIN_SIZE)
  {
    if (ctx->diag != NULL) { ctx->diag->uart1_sample_drop++; }
    LOGE("[USART1] sample frame too short len=%u\r\n", (unsigned int)len);
    return;
  }

  crc_calc = ModbusMaster_Crc16(frame, 17U);
  crc_rx = (uint16_t)frame[17] | ((uint16_t)frame[18] << 8);
  if (crc_calc != crc_rx)
  {
    if (ctx->diag != NULL) { ctx->diag->uart1_sample_drop++; }
    LOGE("[USART1] sample crc mismatch calc=0x%04X rx=0x%04X\r\n",
         (unsigned int)crc_calc,
         (unsigned int)crc_rx);
    AppUsart1_LogFrame("rx-bad", frame, USART1_FRAME_MIN_SIZE);
    return;
  }

  frame_no = ((uint16_t)frame[3] << 8) | frame[4];
  if (s_frame_no_inited == 0U)
  {
    s_last_frame_no = frame_no;
    s_frame_no_inited = 1U;
  }
  else
  {
    expected_no = (uint16_t)(s_last_frame_no + 1U);
    if (frame_no != expected_no)
    {
      AppUsart1_OnFrameNoDiscontinuity(s_last_frame_no, frame_no);
    }
    s_last_frame_no = frame_no;
  }

  sample.ch1 = AppUsart1_U32ToFloat(((uint32_t)frame[8] << 24) |
                                    ((uint32_t)frame[7] << 16) |
                                    ((uint32_t)frame[6] << 8)  |
                                    (uint32_t)frame[5]);
  sample.ch2 = AppUsart1_U32ToFloat(((uint32_t)frame[12] << 24) |
                                    ((uint32_t)frame[11] << 16) |
                                    ((uint32_t)frame[10] << 8)  |
                                    (uint32_t)frame[9]);
  sample.board = AppUsart1_U32ToFloat(((uint32_t)frame[16] << 24) |
                                      ((uint32_t)frame[15] << 16) |
                                      ((uint32_t)frame[14] << 8)  |
                                      (uint32_t)frame[13]);
  sample.frame_no = frame_no;
  sample.timestamp_ms = HAL_GetTick();
  sample.valid = 1U;
  s_last_sample_tick_ms = sample.timestamp_ms;
  s_timeout_retry_count = 0U;

  if ((ctx->mtx_shared_data != NULL) && (ctx->wireless_latest != NULL))
  {
    if (osMutexAcquire(ctx->mtx_shared_data, 2U) == osOK)
    {
      *(wireless_sample_t *)ctx->wireless_latest = sample;
      osMutexRelease(ctx->mtx_shared_data);
    }
  }
  if (ctx->evt_system_flags != NULL)
  {
    osEventFlagsSet(ctx->evt_system_flags, EVT_SAMPLE_UPDATED);
  }
  if (ctx->on_sample != NULL)
  {
    ctx->on_sample(sample.ch1, sample.ch2);
  }
  if (ctx->on_board_sample != NULL)
  {
    ctx->on_board_sample(sample.board);
  }
  if (ctx->on_frame_no != NULL)
  {
    ctx->on_frame_no(sample.frame_no);
  }
  if (ctx->diag != NULL) { ctx->diag->uart1_sample_ok++; }

  s_sample_log_counter++;
  if ((s_sample_log_counter <= 5U) || ((s_sample_log_counter % USART1_LOG_SAMPLE_EVERY) == 0U))
  {
    U1LOGI("[USART1] sample ok no=%u ch1=%.6f ch2=%.6f vb=%.6f tick=%lu total=%lu\r\n",
         (unsigned int)frame_no,
         (double)sample.ch1,
         (double)sample.ch2,
         (double)sample.board,
         (unsigned long)sample.timestamp_ms,
         (unsigned long)s_sample_log_counter);
  }
}

static void AppUsart1_HandleRxChunk(const app_usart1_ctx_t *ctx, const uint8_t *data, uint16_t len)
{
  uint16_t i = 0U;
  uint16_t parsed_count = 0U;
  uint16_t copy_len;

  if ((ctx == NULL) || (data == NULL) || (len == 0U))
  {
    return;
  }

  if (s_rx_accum_len >= sizeof(s_rx_accum_buf))
  {
    s_rx_accum_len = 0U;
  }

  copy_len = len;
  if ((uint16_t)(s_rx_accum_len + copy_len) > (uint16_t)sizeof(s_rx_accum_buf))
  {
    copy_len = (uint16_t)(sizeof(s_rx_accum_buf) - s_rx_accum_len);
  }
  if (copy_len > 0U)
  {
    memcpy(&s_rx_accum_buf[s_rx_accum_len], data, copy_len);
    s_rx_accum_len = (uint16_t)(s_rx_accum_len + copy_len);
  }

  while ((i + USART1_FRAME_MIN_SIZE) <= s_rx_accum_len)
  {
    if ((s_rx_accum_buf[i] == 0x32U) && (s_rx_accum_buf[i + 1U] == 0x33U) && (s_rx_accum_buf[i + 2U] == 0x05U))
    {
      AppUsart1_ParseSampleFrame(ctx, &s_rx_accum_buf[i], USART1_FRAME_MIN_SIZE);
      i = (uint16_t)(i + USART1_FRAME_MIN_SIZE);
      parsed_count++;
    }
    else
    {
      i++;
    }
  }

  if (i > 0U)
  {
    uint16_t remain = (uint16_t)(s_rx_accum_len - i);
    if (remain > 0U)
    {
      memmove(s_rx_accum_buf, &s_rx_accum_buf[i], remain);
    }
    s_rx_accum_len = remain;
  }

  if ((parsed_count == 0U) && (copy_len > 0U) && (s_rx_accum_len == 0U))
  {
    LOGE("[USART1] rx chunk no valid sample frame len=%u\r\n", (unsigned int)len);
    AppUsart1_LogFrame("rx-raw", data, len);
  }
}

void AppUsart1_Init(const app_usart1_ctx_t *ctx)
{
  if ((ctx == NULL) || (ctx->huart == NULL) || (ctx->hdma_rx == NULL) || (ctx->rx_dma_buf == NULL))
  {
    return;
  }
  AppUsart1_BuildFrame(0x01U, 0x06U, 0x00U, 0x01U, s_hb_frame);
  U1LOGI("[USART1] init baud=%lu rx_dma=%u\r\n",
       (unsigned long)ctx->huart->Init.BaudRate,
       (unsigned int)ctx->rx_dma_buf_size);
  AppUsart1_LogFrame("tx-heartbeat-template", s_hb_frame, sizeof(s_hb_frame));
  if (HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart, ctx->rx_dma_buf, ctx->rx_dma_buf_size) != HAL_OK)
  {
    LOGE("[USART1] rx dma init failed err=0x%08lX\r\n", (unsigned long)ctx->huart->ErrorCode);
    return;
  }
  __HAL_DMA_DISABLE_IT(ctx->hdma_rx, DMA_IT_HT);
  U1LOGI("[USART1] rx dma armed\r\n");
  s_start_sampling_pending = 1U;
  s_rx_dma_last_pos = 0U;
  U1LOGI("[USART1] start sampling pending\r\n");
}

void AppUsart1_Process(const app_usart1_ctx_t *ctx, uint32_t now_tick_ms)
{
  static uint32_t hb_tick = 0U;
  usart1_frame_msg_t msg;
  uint32_t now_ms;

  (void)now_tick_ms;

  if (ctx == NULL)
  {
    return;
  }
  now_ms = HAL_GetTick();

  if (s_start_sampling_pending != 0U)
  {
    s_start_sampling_pending = 0U;
    AppUsart1_SendStartSampling(ctx);
  }

  while ((ctx->q_rx_frames != NULL) && (osMessageQueueGet(ctx->q_rx_frames, &msg, NULL, 0U) == osOK))
  {
    AppUsart1_HandleRxChunk(ctx, msg.payload, msg.length);
  }

  if (now_ms < s_last_sample_tick_ms)
  {
    s_last_sample_tick_ms = now_ms;
  }

  if ((s_sampling_active != 0U) && ((now_ms - s_last_sample_tick_ms) > USART1_RX_TIMEOUT_MS))
  {
    LOGE("[USART1] sample timeout last_tick=%lu now=%lu diff=%lu\r\n",
         (unsigned long)s_last_sample_tick_ms,
         (unsigned long)now_ms,
         (unsigned long)(now_ms - s_last_sample_tick_ms));
    if (s_timeout_retry_count < 0xFFU)
    {
      s_timeout_retry_count++;
    }

    if (s_timeout_retry_count <= USART1_TIMEOUT_RETRY_MAX)
    {
      uint16_t freq = AppUsart1_GetConfiguredFrequency(ctx);
      LOGE("[USART1] timeout recovery #%u: stop-setfreq-start freq=%u\r\n",
           (unsigned int)s_timeout_retry_count,
           (unsigned int)freq);
      AppUsart1_SendStopSamplingWithLog(ctx, "timeout-recovery");
      AppUsart1_DelayMs(USART1_RECOVERY_CMD_GAP_MS);
      AppUsart1_SendSetFrequencyWithLog(ctx, freq, "timeout-recovery");
      AppUsart1_DelayMs(USART1_RECOVERY_CMD_GAP_MS);
      AppUsart1_SendStartSamplingWithLog(ctx, "timeout-recovery");
      s_last_sample_tick_ms = now_ms;
    }
    else
    {
      LOGE("[USART1] sample timeout retry exceeded (%u), trigger rx interrupt handler\r\n",
           (unsigned int)s_timeout_retry_count);
      s_timeout_retry_count = 0U;
      AppUsart1_OnRxInterrupted(ctx);
      AppUsart1_DelayMs(USART1_RECOVERY_CMD_GAP_MS);
      AppUsart1_SendSetFrequencyWithLog(ctx, AppUsart1_GetConfiguredFrequency(ctx), "power-cycle-recovery");
      AppUsart1_DelayMs(USART1_RECOVERY_CMD_GAP_MS);
      AppUsart1_SendStartSamplingWithLog(ctx, "power-cycle-recovery");
      s_last_sample_tick_ms = HAL_GetTick();
    }
  }

  if ((now_ms - hb_tick) >= USART1_HEARTBEAT_MS)
  {
    AppUsart1_LogFrame("tx-heartbeat", s_hb_frame, sizeof(s_hb_frame));
    (void)AppUsart1_SendFrameDma(ctx, s_hb_frame, sizeof(s_hb_frame), 100U);
    hb_tick = now_ms;
  }
}

void AppUsart1_OnTxCplt(const app_usart1_ctx_t *ctx)
{
  (void)ctx;
  s_usart1_tx_done = 1U;
}

void AppUsart1_OnRxEvent(const app_usart1_ctx_t *ctx, uint16_t size)
{
  usart1_frame_msg_t msg;
  HAL_StatusTypeDef rearm_ret;
  uint16_t curr_pos;
  uint16_t chunk_len;
  uint16_t first_len;
  uint16_t second_len;
  if ((ctx == NULL) || (ctx->rx_dma_buf == NULL) || (ctx->huart == NULL) || (ctx->hdma_rx == NULL))
  {
    return;
  }
  if ((size > 0U) && (size <= ctx->rx_dma_buf_size))
  {
    curr_pos = size;
    if (curr_pos == s_rx_dma_last_pos)
    {
      goto rearm_dma;
    }

    if (curr_pos > s_rx_dma_last_pos)
    {
      chunk_len = (uint16_t)(curr_pos - s_rx_dma_last_pos);
      if (chunk_len > sizeof(msg.payload))
      {
        chunk_len = (uint16_t)sizeof(msg.payload);
      }
      memcpy(msg.payload, &ctx->rx_dma_buf[s_rx_dma_last_pos], chunk_len);
      msg.length = chunk_len;
    }
    else
    {
      first_len = (uint16_t)(ctx->rx_dma_buf_size - s_rx_dma_last_pos);
      second_len = curr_pos;
      chunk_len = (uint16_t)(first_len + second_len);
      if (chunk_len > sizeof(msg.payload))
      {
        chunk_len = (uint16_t)sizeof(msg.payload);
      }

      if (chunk_len <= first_len)
      {
        memcpy(msg.payload, &ctx->rx_dma_buf[s_rx_dma_last_pos], chunk_len);
      }
      else
      {
        memcpy(msg.payload, &ctx->rx_dma_buf[s_rx_dma_last_pos], first_len);
        memcpy(&msg.payload[first_len], &ctx->rx_dma_buf[0], (uint16_t)(chunk_len - first_len));
      }
      msg.length = chunk_len;
    }

    s_rx_dma_last_pos = curr_pos;
    if (ctx->diag != NULL) { ctx->diag->uart1_rx_frames++; }
    msg.timestamp_ms = HAL_GetTick();
    U1LOGI("[USART1] rx event size=%u tick=%lu\r\n",
         (unsigned int)msg.length,
         (unsigned long)msg.timestamp_ms);
    AppUsart1_LogFrame("rx-event", msg.payload, msg.length);
    if (ctx->q_rx_frames != NULL)
    {
      (void)osMessageQueuePut(ctx->q_rx_frames, &msg, 0U, 0U);
    }
  }
rearm_dma:
  rearm_ret = HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart, ctx->rx_dma_buf, ctx->rx_dma_buf_size);
  if ((rearm_ret != HAL_OK) && (rearm_ret != HAL_BUSY))
  {
    LOGE("[USART1] rx dma rearm failed ret=%ld state=%lu err=0x%08lX\r\n",
         (long)rearm_ret,
         (unsigned long)ctx->huart->RxState,
         (unsigned long)ctx->huart->ErrorCode);
  }
  __HAL_DMA_DISABLE_IT(ctx->hdma_rx, DMA_IT_HT);
}

void AppUsart1_OnError(const app_usart1_ctx_t *ctx)
{
  if ((ctx == NULL) || (ctx->huart == NULL) || (ctx->hdma_rx == NULL) || (ctx->rx_dma_buf == NULL))
  {
    return;
  }
  LOGE("[USART1] uart error err=0x%08lX\r\n", (unsigned long)ctx->huart->ErrorCode);
  if (HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart, ctx->rx_dma_buf, ctx->rx_dma_buf_size) != HAL_OK)
  {
    LOGE("[USART1] rx dma recover failed err=0x%08lX\r\n", (unsigned long)ctx->huart->ErrorCode);
  }
  __HAL_DMA_DISABLE_IT(ctx->hdma_rx, DMA_IT_HT);
}

void AppUsart1_SendStartSampling(const app_usart1_ctx_t *ctx)
{
  uint8_t frame[9];
  HAL_StatusTypeDef ret;
  AppUsart1_BuildFrame(0x01U, 0x03U, 0x00U, 0x01U, frame);
  AppUsart1_LogFrame("tx-start-sampling", frame, sizeof(frame));
  ret = AppUsart1_SendFrameDma(ctx, frame, sizeof(frame), 100U);
  U1LOGI("[USART1] start sampling tx ret=%ld\r\n", (long)ret);
  if (ret == HAL_OK)
  {
    s_sampling_active = 1U;
    s_last_sample_tick_ms = HAL_GetTick();
    s_frame_no_inited = 0U;
    s_last_frame_no = 0U;
    s_sample_log_counter = 0U;
    s_rx_accum_len = 0U;
    U1LOGI("[USART1] sampling active from boot/init\r\n");
  }
  else
  {
    s_sampling_active = 0U;
    LOGE("[USART1] start sampling failed ret=%ld\r\n", (long)ret);
  }
}

void AppUsart1_SendStartSamplingWithLog(const app_usart1_ctx_t *ctx, const char *reason)
{
  uint8_t frame[9];
  HAL_StatusTypeDef ret;
  AppUsart1_BuildFrame(0x01U, 0x03U, 0x00U, 0x01U, frame);
  AppUsart1_LogFrameError("tx-start-sampling", reason, frame, sizeof(frame));
  ret = AppUsart1_SendFrameDma(ctx, frame, sizeof(frame), 100U);
  if (ret == HAL_OK)
  {
    s_sampling_active = 1U;
    s_last_sample_tick_ms = HAL_GetTick();
    s_frame_no_inited = 0U;
    s_last_frame_no = 0U;
    s_sample_log_counter = 0U;
    s_rx_accum_len = 0U;
  }
  else
  {
    s_sampling_active = 0U;
    LOGE("[USART1] start sampling failed ret=%ld reason=%s\r\n",
         (long)ret,
         (reason != NULL) ? reason : "");
  }
}

void AppUsart1_SendStopSampling(const app_usart1_ctx_t *ctx)
{
  uint8_t frame[9];
  AppUsart1_BuildFrame(0x01U, 0x04U, 0x00U, 0x01U, frame);
  AppUsart1_LogFrame("tx-stop-sampling", frame, sizeof(frame));
  (void)AppUsart1_SendFrameDma(ctx, frame, sizeof(frame), 100U);
  s_sampling_active = 0U;
  U1LOGI("[USART1] sampling stopped\r\n");
}

void AppUsart1_SendStopSamplingWithLog(const app_usart1_ctx_t *ctx, const char *reason)
{
  uint8_t frame[9];
  AppUsart1_BuildFrame(0x01U, 0x04U, 0x00U, 0x01U, frame);
  AppUsart1_LogFrameError("tx-stop-sampling", reason, frame, sizeof(frame));
  (void)AppUsart1_SendFrameDma(ctx, frame, sizeof(frame), 100U);
  s_sampling_active = 0U;
}

void AppUsart1_SendSetFrequency(const app_usart1_ctx_t *ctx, uint16_t frequency)
{
  uint8_t frame[9];
  uint16_t cmd_value = AppUsart1_FrequencyToCmdValue(frequency);
  AppUsart1_BuildFrame(0x00U, 0x05U, (uint8_t)(cmd_value >> 8), (uint8_t)(cmd_value & 0xFFU), frame);
  U1LOGI("[USART1] set frequency=%u cmd_value=%u\r\n",
         (unsigned int)frequency,
         (unsigned int)cmd_value);
  AppUsart1_LogFrame("tx-set-frequency", frame, sizeof(frame));
  (void)AppUsart1_SendFrameDma(ctx, frame, sizeof(frame), 100U);
}

void AppUsart1_SendSetFrequencyWithLog(const app_usart1_ctx_t *ctx, uint16_t frequency, const char *reason)
{
  uint8_t frame[9];
  uint16_t cmd_value = AppUsart1_FrequencyToCmdValue(frequency);
  AppUsart1_BuildFrame(0x00U, 0x05U, (uint8_t)(cmd_value >> 8), (uint8_t)(cmd_value & 0xFFU), frame);
  LOGE("[USART1] set frequency=%u cmd_value=%u reason=%s\r\n",
       (unsigned int)frequency,
       (unsigned int)cmd_value,
       (reason != NULL) ? reason : "");
  AppUsart1_LogFrameError("tx-set-frequency", reason, frame, sizeof(frame));
  (void)AppUsart1_SendFrameDma(ctx, frame, sizeof(frame), 100U);
}
