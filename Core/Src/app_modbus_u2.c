#include "app_modbus_u2.h"
#include "app_calc.h"
#include "app_avg_power_cache.h"
#include "app_param_store.h"
#include <string.h>
#include <stddef.h>

#define MODBUS_U2_REQ_TIMEOUT_MS 500U
#define MODBUS_U2_LOG_REG_PREVIEW 4U
#define MODBUS_U2_LOG_VERBOSE_FRAMES 0U
#define MODBUS_U2_LOG_REQ_DETAIL 0U
#define MODBUS_U2_LOG_INFO 0U
#define MODBUS_U2_LOG_ERROR 1U

#if MODBUS_U2_LOG_INFO
#define U2LOGI(...) LOGI(__VA_ARGS__)
#else
#define U2LOGI(...) do { } while (0)
#endif

#if MODBUS_U2_LOG_ERROR
#define U2LOGE(...) LOGE(__VA_ARGS__)
#else
#define U2LOGE(...) do { } while (0)
#endif

static volatile uint8_t s_usart2_tx_done = 1U;
static uint32_t s_modbus_req_seq = 0U;
static uint32_t s_modbus_rx_event_count = 0U;
static volatile uint32_t s_modbus_uart_err_count = 0U;
static volatile uint32_t s_modbus_rx_rearm_fail_count = 0U;
static volatile uint32_t s_modbus_rx_rearm_busy_count = 0U;
static volatile uint32_t s_modbus_rx_recover_count = 0U;
static volatile uint32_t s_modbus_rx_queue_put_ok_count = 0U;
static volatile uint32_t s_modbus_rx_queue_put_fail_count = 0U;
static volatile uint32_t s_modbus_rx_last_tick = 0U;
static volatile uint16_t s_modbus_rx_last_size = 0U;
static volatile uint16_t s_modbus_rx_last_prev_pos = 0U;
static volatile uint16_t s_modbus_rx_last_curr_pos = 0U;
static volatile uint16_t s_modbus_rx_last_chunk_len = 0U;
static volatile int32_t s_modbus_rx_last_put_ret = 0;
static uint32_t s_modbus_write_ok_count = 0U;
static uint32_t s_modbus_write_timeout_count = 0U;
static uint8_t s_modbus_rx_stream[512];
static uint16_t s_modbus_rx_dma_last_pos = 0U;
static uint16_t s_prev_change_flags = 0U;
static void AppModbusU2_WriteMappedValue(protocol_values_t *values,
                                         const reg_meta_t *meta,
                                         const uint16_t *regs);
static void AppModbusU2_RecoverRxDma(const app_modbus_u2_ctx_t *ctx);
static void AppModbusU2_StopRxDma(const app_modbus_u2_ctx_t *ctx);
static HAL_StatusTypeDef AppModbusU2_StartRxDma(const app_modbus_u2_ctx_t *ctx);
static HAL_StatusTypeDef AppModbusU2_StartResponseWindow(const app_modbus_u2_ctx_t *ctx);
static void AppModbusU2_LogBytesError(const char *tag, const uint8_t *buf, uint16_t len);
static void AppModbusU2_LogTimeoutDiag(const app_modbus_u2_ctx_t *ctx,
                                       uint32_t req_id,
                                       const char *phase,
                                       uint16_t start_addr,
                                       uint16_t quantity,
                                       uint16_t rx_stream_len);

static void AppModbusU2_Delay1ms(void)
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

typedef struct
{
  uint16_t flag_mask;
  uint16_t addr;
  uint16_t qty;
} modbus_change_query_t;

static const modbus_change_query_t s_change_queries[] = {
  {0x0001U, 0x09C4U, 2U},
  {0x0002U, 0x09C6U, 12U},
  {0x0004U, 0x09D2U, 19U},
};

typedef struct
{
  uint8_t reg_index;
  uint16_t flag_mask;
} modbus_flag_map_t;

static const modbus_flag_map_t s_flag_maps[] = {
  {0U, 0x0001U}, /* 0x0AF0 */
  {1U, 0x0002U}, /* 0x0AF1 */
  {2U, 0x0004U}, /* 0x0AF2 */
};

static const modbus_poll_item_t s_status_poll_list[] = {
  {0x0AF0U, 4U},
};

typedef struct
{
  uint16_t addr;
  uint16_t qty;
  uint16_t value_offset;
} modbus_write_item_t;

#define OFF_PROTO(x) ((uint16_t)offsetof(protocol_values_t, x))
static const modbus_write_item_t s_write_07xx_list[] = {
  {0x07D0U, 2U, OFF_PROTO(torque_kNm)},
  {0x07D2U, 2U, OFF_PROTO(power_kW)},
  {0x07D4U, 2U, OFF_PROTO(speed_rpm)},
  {0x07D6U, 2U, OFF_PROTO(thrust_kN)},
  {0x07D8U, 1U, OFF_PROTO(pc2_low_voltage_state)},
  {0x07D9U, 1U, OFF_PROTO(board_low_voltage_state)},
  {0x07DCU, 1U, OFF_PROTO(wireless_frame_no)},
};
#undef OFF_PROTO

static void AppModbusU2_WriteFloatToRegs(float value, uint16_t *out_regs)
{
  uint32_t raw;

  if (out_regs == NULL)
  {
    return;
  }

  memcpy(&raw, &value, sizeof(raw));
  out_regs[0] = (uint16_t)(raw & 0xFFFFU);
  out_regs[1] = (uint16_t)(raw >> 16);
}

static void AppModbusU2_LogBytes(const char *tag, const uint8_t *buf, uint16_t len)
{
  uint16_t i;

  if ((tag == NULL) || (buf == NULL))
  {
    return;
  }

  U2LOGI("[MODBUS-U2] %s len=%u data=", tag, (unsigned int)len);
  for (i = 0U; i < len; i++)
  {
    U2LOGI("%02X", buf[i]);
    if ((uint16_t)(i + 1U) < len)
    {
      U2LOGI(" ");
    }
  }
  U2LOGI("\r\n");
}

static void AppModbusU2_LogBytesError(const char *tag, const uint8_t *buf, uint16_t len)
{
  uint16_t i;

  if ((tag == NULL) || (buf == NULL))
  {
    return;
  }

  U2LOGE("[MODBUS-U2] %s len=%u data=", tag, (unsigned int)len);
  for (i = 0U; i < len; i++)
  {
    U2LOGE("%02X", buf[i]);
    if ((uint16_t)(i + 1U) < len)
    {
      U2LOGE(" ");
    }
  }
  U2LOGE("\r\n");
}

static void AppModbusU2_LogTimeoutDiag(const app_modbus_u2_ctx_t *ctx,
                                       uint32_t req_id,
                                       const char *phase,
                                       uint16_t start_addr,
                                       uint16_t quantity,
                                       uint16_t rx_stream_len)
{
  uint32_t state = 0U;
  uint32_t err = 0U;

  if ((ctx != NULL) && (ctx->huart2 != NULL))
  {
    state = (uint32_t)HAL_UART_GetState(ctx->huart2);
    err = ctx->huart2->ErrorCode;
  }

  U2LOGE("[MODBUS-U2] req#%lu %s timeout addr=0x%04X qty=%u rx_len=%u rx_evt=%lu uart_err=%lu rearm_fail=%lu rearm_busy=%lu recover=%lu state=0x%08lX err=0x%08lX\r\n",
         (unsigned long)req_id,
         (phase != NULL) ? phase : "rx",
         (unsigned int)start_addr,
         (unsigned int)quantity,
         (unsigned int)rx_stream_len,
         (unsigned long)s_modbus_rx_event_count,
         (unsigned long)s_modbus_uart_err_count,
         (unsigned long)s_modbus_rx_rearm_fail_count,
         (unsigned long)s_modbus_rx_rearm_busy_count,
         (unsigned long)s_modbus_rx_recover_count,
         (unsigned long)state,
         (unsigned long)err);
  U2LOGE("[MODBUS-U2] rx-last tick=%lu size=%u prev=%u curr=%u chunk=%u put_ret=%ld put_ok=%lu put_fail=%lu\r\n",
         (unsigned long)s_modbus_rx_last_tick,
         (unsigned int)s_modbus_rx_last_size,
         (unsigned int)s_modbus_rx_last_prev_pos,
         (unsigned int)s_modbus_rx_last_curr_pos,
         (unsigned int)s_modbus_rx_last_chunk_len,
         (long)s_modbus_rx_last_put_ret,
         (unsigned long)s_modbus_rx_queue_put_ok_count,
         (unsigned long)s_modbus_rx_queue_put_fail_count);
}

static void AppModbusU2_LogDecodedValues(const char *tag, uint16_t start_addr, const uint16_t *regs, uint16_t qty)
{
  uint16_t i;
  uint16_t addr;
  const reg_meta_t *meta;
  uint16_t needed_regs;
  uint32_t u32;
  int32_t s32;
  float f32;

  if ((tag == NULL) || (regs == NULL) || (qty == 0U))
  {
    return;
  }

  U2LOGI("[MODBUS-U2] %s addr=0x%04X qty=%u\r\n",
        tag,
        (unsigned int)start_addr,
        (unsigned int)qty);
  for (i = 0U; i < qty; i++)
  {
    addr = (uint16_t)(start_addr + i);
    meta = ProtocolMap_Find(addr);
    if (meta == NULL)
    {
      continue;
    }

    needed_regs = (uint16_t)(meta->bytes / 2U);
    if ((needed_regs == 0U) || ((i + needed_regs) > qty))
    {
      continue;
    }

    if (meta->type == REG_TYPE_SHORT)
    {
      U2LOGI("[MODBUS-U2]   %s(0x%04X) = %u\r\n",
            meta->name,
            (unsigned int)addr,
            (unsigned int)regs[i]);
    }
    else if (meta->type == REG_TYPE_INT32)
    {
      u32 = ((uint32_t)regs[i + 1U] << 16) | regs[i];
      s32 = (int32_t)u32;
      U2LOGI("[MODBUS-U2]   %s(0x%04X) = %ld\r\n",
            meta->name,
            (unsigned int)addr,
            (long)s32);
    }
    else
    {
      u32 = ((uint32_t)regs[i + 1U] << 16) | regs[i];
      memcpy(&f32, &u32, sizeof(f32));
      U2LOGI("[MODBUS-U2]   %s(0x%04X) = %.*f\r\n",
            meta->name,
            (unsigned int)addr,
            (int)meta->decimals,
            (double)f32);
    }
  }
}

static uint16_t AppModbusU2_BuildWriteMultiple(const app_modbus_u2_ctx_t *ctx,
                                               uint16_t start_addr,
                                               uint16_t quantity,
                                               const uint16_t *regs,
                                               uint8_t *out,
                                               uint16_t out_size)
{
  uint16_t i;
  uint16_t crc;
  uint16_t data_bytes;

  if ((ctx == NULL) || (regs == NULL) || (out == NULL))
  {
    return 0U;
  }

  data_bytes = (uint16_t)(quantity * 2U);
  if (quantity == 0U)
  {
    return 0U;
  }
  if (out_size < (uint16_t)(9U + data_bytes))
  {
    return 0U;
  }

  out[0] = ctx->slave_id;
  out[1] = 0x10U;
  out[2] = (uint8_t)(start_addr >> 8);
  out[3] = (uint8_t)(start_addr & 0xFFU);
  out[4] = (uint8_t)(quantity >> 8);
  out[5] = (uint8_t)(quantity & 0xFFU);
  out[6] = (uint8_t)data_bytes;
  for (i = 0U; i < quantity; i++)
  {
    out[7U + (2U * i)] = (uint8_t)(regs[i] >> 8);
    out[8U + (2U * i)] = (uint8_t)(regs[i] & 0xFFU);
  }

  crc = ModbusMaster_Crc16(out, (uint16_t)(7U + data_bytes));
  out[7U + data_bytes] = (uint8_t)(crc & 0xFFU);
  out[8U + data_bytes] = (uint8_t)(crc >> 8);
  return (uint16_t)(9U + data_bytes);
}

static uint8_t AppModbusU2_ParseWriteAckFromStream(const uint8_t *buf,
                                                   uint16_t len,
                                                   uint8_t slave_id,
                                                   uint16_t start_addr,
                                                   uint16_t quantity)
{
  uint16_t i;
  uint16_t crc_calc;
  uint16_t crc_rx;

  if ((buf == NULL) || (len < 8U))
  {
    return 0U;
  }

  for (i = 0U; (uint16_t)(i + 8U) <= len; i++)
  {
    if ((buf[i] != slave_id) ||
        (buf[i + 1U] != 0x10U) ||
        (buf[i + 2U] != (uint8_t)(start_addr >> 8)) ||
        (buf[i + 3U] != (uint8_t)(start_addr & 0xFFU)) ||
        (buf[i + 4U] != (uint8_t)(quantity >> 8)) ||
        (buf[i + 5U] != (uint8_t)(quantity & 0xFFU)))
    {
      continue;
    }

    crc_calc = ModbusMaster_Crc16(&buf[i], 6U);
    crc_rx = (uint16_t)buf[i + 6U] | ((uint16_t)buf[i + 7U] << 8);
    if (crc_calc == crc_rx)
    {
      return 1U;
    }
  }

  return 0U;
}

static uint8_t AppModbusU2_ParseExpectedFromStream(const uint8_t *buf,
                                                   uint16_t len,
                                                   uint8_t slave_id,
                                                   uint16_t quantity,
                                                   modbus_read_resp_t *out,
                                                   uint16_t *matched_len)
{
  uint16_t i;
  uint16_t frame_len;
  uint16_t crc_calc;
  uint16_t crc_rx;
  uint8_t expected_byte_count;

  if ((buf == NULL) || (out == NULL) || (len < 5U))
  {
    return 0U;
  }

  expected_byte_count = (uint8_t)(quantity * 2U);
  frame_len = (uint16_t)(5U + expected_byte_count);
  if (frame_len > len)
  {
    return 0U;
  }

  for (i = 0U; (uint16_t)(i + frame_len) <= len; i++)
  {
    if ((buf[i] != slave_id) || (buf[i + 1U] != 0x03U) || (buf[i + 2U] != expected_byte_count))
    {
      continue;
    }

    crc_calc = ModbusMaster_Crc16(&buf[i], (uint16_t)(frame_len - 2U));
    crc_rx = (uint16_t)buf[i + frame_len - 2U] | ((uint16_t)buf[i + frame_len - 1U] << 8);
    if (crc_calc != crc_rx)
    {
      continue;
    }

    if (ModbusMaster_ParseReadHoldingResp(&buf[i], frame_len, out) == 1U)
    {
      if (matched_len != NULL)
      {
        *matched_len = frame_len;
      }
      return 1U;
    }
  }

  return 0U;
}

static HAL_StatusTypeDef AppModbusU2_SendAndWaitRead(const app_modbus_u2_ctx_t *ctx, uint16_t start_addr, uint16_t quantity, modbus_read_resp_t *resp)
{
  modbus_read_req_t req;
  uint8_t tx_frame[8];
  uint16_t tx_len;
  modbus_event_msg_t rx_msg;
  uint32_t start_tick = HAL_GetTick();
  uint32_t req_id;
  uint16_t preview_n;
  uint16_t i;
  uint16_t matched_len = 0U;
  uint16_t rx_stream_len = 0U;
  uint32_t wait_start_tick;
  uint32_t elapsed;
  uint32_t remain_ms;
  HAL_StatusTypeDef rearm_ret;

  if ((ctx == NULL) || (resp == NULL))
  {
    return HAL_ERROR;
  }
  memset(resp, 0, sizeof(*resp));

  req.slave_id = ctx->slave_id;
  req.function_code = 0x03U;
  req.start_addr = start_addr;
  req.quantity = quantity;
  tx_len = ModbusMaster_BuildReadHolding(&req, tx_frame, sizeof(tx_frame));
  if (tx_len == 0U)
  {
    if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
    U2LOGE("[MODBUS-U2] build req failed addr=0x%04X qty=%u\r\n",
         (unsigned int)start_addr, (unsigned int)quantity);
    return HAL_ERROR;
  }

  s_modbus_req_seq++;
  req_id = s_modbus_req_seq;
#if MODBUS_U2_LOG_REQ_DETAIL
  U2LOGI("[MODBUS-U2] req#%lu tx slave=%u addr=0x%04X qty=%u\r\n",
       (unsigned long)req_id,
       (unsigned int)req.slave_id,
       (unsigned int)start_addr,
       (unsigned int)quantity);
#endif
#if MODBUS_U2_LOG_VERBOSE_FRAMES
  AppModbusU2_LogBytes("tx-frame", tx_frame, tx_len);
#endif

  if (ctx->q_modbus_events != NULL)
  {
    while (osMessageQueueGet(ctx->q_modbus_events, &rx_msg, NULL, 0U) == osOK)
    {
      /* Drop stale frames from previous requests. */
    }
  }
  AppModbusU2_StopRxDma(ctx);

  while (s_usart2_tx_done == 0U)
  {
    if ((HAL_GetTick() - start_tick) > MODBUS_U2_REQ_TIMEOUT_MS)
    {
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      U2LOGE("[MODBUS-U2] req#%lu wait-prev-tx timeout\r\n", (unsigned long)req_id);
      return HAL_TIMEOUT;
    }
    AppModbusU2_Delay1ms();
  }

  s_usart2_tx_done = 0U;
  if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(1U); }
  if (HAL_UART_Transmit_DMA(ctx->huart2, tx_frame, tx_len) != HAL_OK)
  {
    if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(0U); }
    s_usart2_tx_done = 1U;
    if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
    U2LOGE("[MODBUS-U2] req#%lu tx dma start failed err=0x%08lX\r\n",
         (unsigned long)req_id,
         (unsigned long)ctx->huart2->ErrorCode);
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while (s_usart2_tx_done == 0U)
  {
    if ((HAL_GetTick() - start_tick) > MODBUS_U2_REQ_TIMEOUT_MS)
    {
      if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(0U); }
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      U2LOGE("[MODBUS-U2] req#%lu tx complete timeout\r\n", (unsigned long)req_id);
      return HAL_TIMEOUT;
    }
    AppModbusU2_Delay1ms();
  }

  /* DMA complete only means data moved to DR; wait for line shift complete (TC) before DE low. */
  start_tick = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(ctx->huart2, UART_FLAG_TC) == RESET)
  {
    if ((HAL_GetTick() - start_tick) > MODBUS_U2_REQ_TIMEOUT_MS)
    {
      if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(0U); }
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      U2LOGE("[MODBUS-U2] req#%lu tx tc timeout\r\n", (unsigned long)req_id);
      return HAL_TIMEOUT;
    }
    AppModbusU2_Delay1ms();
  }

  /* Guard time for RS485 transceiver direction switching. */
  AppModbusU2_Delay1ms();
  if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(0U); }

  rearm_ret = AppModbusU2_StartResponseWindow(ctx);
  if (rearm_ret != HAL_OK)
  {
    if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
    U2LOGE("[MODBUS-U2] req#%lu rx response window start failed ret=%ld err=0x%08lX\r\n",
           (unsigned long)req_id,
           (long)rearm_ret,
           (unsigned long)ctx->huart2->ErrorCode);
    return rearm_ret;
  }

  if (ctx->q_modbus_events == NULL)
  {
    if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
    U2LOGE("[MODBUS-U2] req#%lu queue null\r\n", (unsigned long)req_id);
    return HAL_ERROR;
  }
  wait_start_tick = HAL_GetTick();
  for (;;)
  {
    elapsed = HAL_GetTick() - wait_start_tick;
    if (elapsed >= MODBUS_U2_REQ_TIMEOUT_MS)
    {
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      AppModbusU2_LogTimeoutDiag(ctx, req_id, "rx wait", start_addr, quantity, rx_stream_len);
      if (rx_stream_len > 0U)
      {
        AppModbusU2_LogBytesError("rx-timeout-stream", s_modbus_rx_stream, rx_stream_len);
      }
      AppModbusU2_RecoverRxDma(ctx);
      return HAL_TIMEOUT;
    }

    remain_ms = MODBUS_U2_REQ_TIMEOUT_MS - elapsed;
    if (osMessageQueueGet(ctx->q_modbus_events, &rx_msg, NULL, remain_ms) != osOK)
    {
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      AppModbusU2_LogTimeoutDiag(ctx, req_id, "rx wait", start_addr, quantity, rx_stream_len);
      if (rx_stream_len > 0U)
      {
        AppModbusU2_LogBytesError("rx-timeout-stream", s_modbus_rx_stream, rx_stream_len);
      }
      AppModbusU2_RecoverRxDma(ctx);
      return HAL_TIMEOUT;
    }

#if MODBUS_U2_LOG_REQ_DETAIL
    U2LOGI("[MODBUS-U2] req#%lu rx tick=%lu len=%u\r\n",
         (unsigned long)req_id,
         (unsigned long)rx_msg.timestamp_ms,
         (unsigned int)rx_msg.length);
#endif

    if (rx_msg.length > sizeof(s_modbus_rx_stream))
    {
      rx_stream_len = 0U;
    }
    else
    {
      if ((uint16_t)(rx_stream_len + rx_msg.length) > (uint16_t)sizeof(s_modbus_rx_stream))
      {
        uint16_t keep = (uint16_t)(sizeof(s_modbus_rx_stream) - rx_msg.length);
        memmove(s_modbus_rx_stream, &s_modbus_rx_stream[rx_stream_len - keep], keep);
        rx_stream_len = keep;
      }
      memcpy(&s_modbus_rx_stream[rx_stream_len], rx_msg.payload, rx_msg.length);
      rx_stream_len = (uint16_t)(rx_stream_len + rx_msg.length);
    }

    if (AppModbusU2_ParseExpectedFromStream(s_modbus_rx_stream,
                                            rx_stream_len,
                                            req.slave_id,
                                            quantity,
                                            resp,
                                            &matched_len) != 0U)
    {
      break;
    }
  }

#if MODBUS_U2_LOG_REQ_DETAIL
  U2LOGI("[MODBUS-U2] req#%lu frame matched len=%u\r\n",
       (unsigned long)req_id,
       (unsigned int)matched_len);
#endif

  preview_n = (resp->reg_count > MODBUS_U2_LOG_REG_PREVIEW) ? MODBUS_U2_LOG_REG_PREVIEW : resp->reg_count;
#if MODBUS_U2_LOG_REQ_DETAIL
  U2LOGI("[MODBUS-U2] req#%lu ok reg_count=%u preview=", (unsigned long)req_id, (unsigned int)resp->reg_count);
  for (i = 0U; i < preview_n; i++)
  {
    U2LOGI("0x%04X", (unsigned int)resp->regs[i]);
    if ((uint16_t)(i + 1U) < preview_n)
    {
      U2LOGI(" ");
    }
  }
  U2LOGI("\r\n");
  AppModbusU2_LogDecodedValues("rx-read-values", start_addr, resp->regs, resp->reg_count);
#else
  (void)preview_n;
#endif

  if (ctx->diag != NULL) { ctx->diag->uart2_req_ok++; }
  return HAL_OK;
}

static HAL_StatusTypeDef AppModbusU2_SendAndWaitWrite(const app_modbus_u2_ctx_t *ctx,
                                                      uint16_t start_addr,
                                                      uint16_t quantity,
                                                      const uint16_t *regs)
{
  uint8_t tx_frame[64];
  uint16_t tx_len;
  modbus_event_msg_t rx_msg;
  uint32_t start_tick = HAL_GetTick();
  uint32_t req_id;
  uint16_t rx_stream_len = 0U;
  uint32_t wait_start_tick;
  uint32_t elapsed;
  uint32_t remain_ms;

  if ((ctx == NULL) || (regs == NULL))
  {
    return HAL_ERROR;
  }

  tx_len = AppModbusU2_BuildWriteMultiple(ctx, start_addr, quantity, regs, tx_frame, sizeof(tx_frame));
  if (tx_len == 0U)
  {
    if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
    U2LOGE("[MODBUS-U2] build write failed addr=0x%04X qty=%u\r\n",
         (unsigned int)start_addr,
         (unsigned int)quantity);
    return HAL_ERROR;
  }

  s_modbus_req_seq++;
  req_id = s_modbus_req_seq;
#if MODBUS_U2_LOG_REQ_DETAIL
  U2LOGI("[MODBUS-U2] req#%lu tx-write slave=%u addr=0x%04X qty=%u\r\n",
       (unsigned long)req_id,
       (unsigned int)ctx->slave_id,
       (unsigned int)start_addr,
       (unsigned int)quantity);
  AppModbusU2_LogDecodedValues("tx-write-values", start_addr, regs, quantity);
#endif
#if MODBUS_U2_LOG_VERBOSE_FRAMES
  AppModbusU2_LogBytes("tx-frame", tx_frame, tx_len);
#endif

  if (ctx->q_modbus_events != NULL)
  {
    while (osMessageQueueGet(ctx->q_modbus_events, &rx_msg, NULL, 0U) == osOK) {}
  }
  AppModbusU2_StopRxDma(ctx);

  while (s_usart2_tx_done == 0U)
  {
    if ((HAL_GetTick() - start_tick) > MODBUS_U2_REQ_TIMEOUT_MS)
    {
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      U2LOGE("[MODBUS-U2] req#%lu wait-prev-tx timeout\r\n", (unsigned long)req_id);
      return HAL_TIMEOUT;
    }
    AppModbusU2_Delay1ms();
  }

  s_usart2_tx_done = 0U;
  if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(1U); }
  if (HAL_UART_Transmit_DMA(ctx->huart2, tx_frame, tx_len) != HAL_OK)
  {
    if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(0U); }
    s_usart2_tx_done = 1U;
    if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
    U2LOGE("[MODBUS-U2] req#%lu tx-write dma start failed err=0x%08lX\r\n",
         (unsigned long)req_id,
         (unsigned long)ctx->huart2->ErrorCode);
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while (s_usart2_tx_done == 0U)
  {
    if ((HAL_GetTick() - start_tick) > MODBUS_U2_REQ_TIMEOUT_MS)
    {
      if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(0U); }
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      U2LOGE("[MODBUS-U2] req#%lu tx-write complete timeout\r\n", (unsigned long)req_id);
      return HAL_TIMEOUT;
    }
    AppModbusU2_Delay1ms();
  }

  start_tick = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(ctx->huart2, UART_FLAG_TC) == RESET)
  {
    if ((HAL_GetTick() - start_tick) > MODBUS_U2_REQ_TIMEOUT_MS)
    {
      if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(0U); }
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      U2LOGE("[MODBUS-U2] req#%lu tx-write tc timeout\r\n", (unsigned long)req_id);
      return HAL_TIMEOUT;
    }
    AppModbusU2_Delay1ms();
  }
  AppModbusU2_Delay1ms();
  if (ctx->rs485_set_tx_usart2 != NULL) { ctx->rs485_set_tx_usart2(0U); }

  {
    HAL_StatusTypeDef rx_ret = AppModbusU2_StartResponseWindow(ctx);
    if (rx_ret != HAL_OK)
    {
      if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
      U2LOGE("[MODBUS-U2] req#%lu rx-write response window start failed ret=%ld err=0x%08lX\r\n",
             (unsigned long)req_id,
             (long)rx_ret,
             (unsigned long)ctx->huart2->ErrorCode);
      return rx_ret;
    }
  }

  if (ctx->q_modbus_events == NULL)
  {
    if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
    U2LOGE("[MODBUS-U2] req#%lu queue null\r\n", (unsigned long)req_id);
    return HAL_ERROR;
  }

  wait_start_tick = HAL_GetTick();
  for (;;)
  {
    elapsed = HAL_GetTick() - wait_start_tick;
    if (elapsed >= MODBUS_U2_REQ_TIMEOUT_MS)
    {
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      AppModbusU2_LogTimeoutDiag(ctx, req_id, "rx-write", start_addr, quantity, rx_stream_len);
      if (rx_stream_len > 0U)
      {
        AppModbusU2_LogBytesError("rx-write-timeout-stream", s_modbus_rx_stream, rx_stream_len);
      }
      AppModbusU2_RecoverRxDma(ctx);
      return HAL_TIMEOUT;
    }

    remain_ms = MODBUS_U2_REQ_TIMEOUT_MS - elapsed;
    if (osMessageQueueGet(ctx->q_modbus_events, &rx_msg, NULL, remain_ms) != osOK)
    {
      if (ctx->diag != NULL) { ctx->diag->uart2_req_timeout++; }
      AppModbusU2_LogTimeoutDiag(ctx, req_id, "rx-write", start_addr, quantity, rx_stream_len);
      if (rx_stream_len > 0U)
      {
        AppModbusU2_LogBytesError("rx-write-timeout-stream", s_modbus_rx_stream, rx_stream_len);
      }
      AppModbusU2_RecoverRxDma(ctx);
      return HAL_TIMEOUT;
    }

#if MODBUS_U2_LOG_REQ_DETAIL
    U2LOGI("[MODBUS-U2] req#%lu rx tick=%lu len=%u\r\n",
         (unsigned long)req_id,
         (unsigned long)rx_msg.timestamp_ms,
         (unsigned int)rx_msg.length);
#endif

    if (rx_msg.length > sizeof(s_modbus_rx_stream))
    {
      rx_stream_len = 0U;
    }
    else
    {
      if ((uint16_t)(rx_stream_len + rx_msg.length) > (uint16_t)sizeof(s_modbus_rx_stream))
      {
        uint16_t keep = (uint16_t)(sizeof(s_modbus_rx_stream) - rx_msg.length);
        memmove(s_modbus_rx_stream, &s_modbus_rx_stream[rx_stream_len - keep], keep);
        rx_stream_len = keep;
      }
      memcpy(&s_modbus_rx_stream[rx_stream_len], rx_msg.payload, rx_msg.length);
      rx_stream_len = (uint16_t)(rx_stream_len + rx_msg.length);
    }

    if (AppModbusU2_ParseWriteAckFromStream(s_modbus_rx_stream, rx_stream_len, ctx->slave_id, start_addr, quantity) != 0U)
    {
#if MODBUS_U2_LOG_REQ_DETAIL
      U2LOGI("[MODBUS-U2] req#%lu write-ack matched\r\n", (unsigned long)req_id);
#endif
      if (ctx->diag != NULL) { ctx->diag->uart2_req_ok++; }
      return HAL_OK;
    }
  }
}

static uint16_t AppModbusU2_FloatToRegs(const app_modbus_u2_ctx_t *ctx,
                                        uint16_t value_offset,
                                        uint16_t *out_regs,
                                        uint16_t out_count)
{
  float value;
  uint32_t raw;

  if ((ctx == NULL) || (ctx->protocol_values == NULL) || (out_regs == NULL) || (out_count < 2U))
  {
    return 0U;
  }
  if (ctx->mtx_shared_data == NULL)
  {
    return 0U;
  }

  if (osMutexAcquire(ctx->mtx_shared_data, 5U) != osOK)
  {
    return 0U;
  }
  value = *(float *)((uint8_t *)ctx->protocol_values + value_offset);
  osMutexRelease(ctx->mtx_shared_data);

  memcpy(&raw, &value, sizeof(raw));
  out_regs[0] = (uint16_t)(raw & 0xFFFFU);
  out_regs[1] = (uint16_t)(raw >> 16);
  return 2U;
}

static uint16_t AppModbusU2_U16ToRegs(const app_modbus_u2_ctx_t *ctx,
                                      uint16_t value_offset,
                                      uint16_t *out_regs,
                                      uint16_t out_count)
{
  uint16_t value;

  if ((ctx == NULL) || (ctx->protocol_values == NULL) || (out_regs == NULL) || (out_count < 1U))
  {
    return 0U;
  }
  if (ctx->mtx_shared_data == NULL)
  {
    return 0U;
  }

  if (osMutexAcquire(ctx->mtx_shared_data, 5U) != osOK)
  {
    return 0U;
  }
  value = *(uint16_t *)((uint8_t *)ctx->protocol_values + value_offset);
  osMutexRelease(ctx->mtx_shared_data);

  out_regs[0] = value;
  return 1U;
}

static void AppModbusU2_ClearSentLowVoltageStates(const app_modbus_u2_ctx_t *ctx,
                                                  uint16_t start_addr,
                                                  uint16_t quantity)
{
  uint16_t end_addr = (uint16_t)(start_addr + quantity);

  if ((ctx == NULL) || (ctx->protocol_values == NULL) || (ctx->mtx_shared_data == NULL))
  {
    return;
  }
  if ((start_addr > 0x07D9U) || (end_addr <= 0x07D8U))
  {
    return;
  }

  if (osMutexAcquire(ctx->mtx_shared_data, 5U) == osOK)
  {
    if ((start_addr <= 0x07D8U) && (end_addr > 0x07D8U))
    {
      ((protocol_values_t *)ctx->protocol_values)->pc2_low_voltage_state = 0U;
    }
    if ((start_addr <= 0x07D9U) && (end_addr > 0x07D9U))
    {
      ((protocol_values_t *)ctx->protocol_values)->board_low_voltage_state = 0U;
    }
    osMutexRelease(ctx->mtx_shared_data);
  }
}

static uint8_t AppModbusU2_Write07xx1Hz(const app_modbus_u2_ctx_t *ctx)
{
  uint16_t regs[32];
  HAL_StatusTypeDef ret;
  uint8_t all_ok = 1U;
  uint32_t n;
  uint32_t i;
  uint32_t j;
  uint16_t start_addr;
  uint16_t group_qty;
  uint16_t cur_addr_end;
  uint16_t reg_pos;

  n = (sizeof(s_write_07xx_list) / sizeof(s_write_07xx_list[0]));
  i = 0U;
  while (i < n)
  {
    start_addr = s_write_07xx_list[i].addr;
    group_qty = s_write_07xx_list[i].qty;
    cur_addr_end = (uint16_t)(start_addr + group_qty);

    j = i + 1U;
    while (j < n)
    {
      if (s_write_07xx_list[j].addr != cur_addr_end)
      {
        break;
      }
      group_qty = (uint16_t)(group_qty + s_write_07xx_list[j].qty);
      cur_addr_end = (uint16_t)(cur_addr_end + s_write_07xx_list[j].qty);
      j++;
    }

    if (group_qty > (uint16_t)(sizeof(regs) / sizeof(regs[0])))
    {
      if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
      U2LOGE("[MODBUS-U2] write group too large start=0x%04X qty=%u\r\n",
           (unsigned int)start_addr,
           (unsigned int)group_qty);
      return 0U;
    }

    reg_pos = 0U;
    while (i < j)
    {
      if (s_write_07xx_list[i].qty == 2U)
      {
        if (AppModbusU2_FloatToRegs(ctx, s_write_07xx_list[i].value_offset, &regs[reg_pos], 2U) != 2U)
        {
          return 0U;
        }
      }
      else if (s_write_07xx_list[i].qty == 1U)
      {
        if (AppModbusU2_U16ToRegs(ctx, s_write_07xx_list[i].value_offset, &regs[reg_pos], 1U) != 1U)
        {
          return 0U;
        }
      }
      else
      {
        if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
        U2LOGE("[MODBUS-U2] unsupported write qty=%u addr=0x%04X\r\n",
             (unsigned int)s_write_07xx_list[i].qty,
             (unsigned int)s_write_07xx_list[i].addr);
        return 0U;
      }
      reg_pos = (uint16_t)(reg_pos + s_write_07xx_list[i].qty);
      i++;
    }

    ret = AppModbusU2_SendAndWaitWrite(ctx, start_addr, group_qty, regs);
    if (ret == HAL_TIMEOUT)
    {
      /* retry once for transient bus collisions */
      ret = AppModbusU2_SendAndWaitWrite(ctx, start_addr, group_qty, regs);
    }

    if (ret == HAL_OK)
    {
      s_modbus_write_ok_count++;
      AppModbusU2_ClearSentLowVoltageStates(ctx, start_addr, group_qty);
    }
    else
    {
      all_ok = 0U;
      if (ret == HAL_TIMEOUT)
      {
        s_modbus_write_timeout_count++;
      }
    }
  }
  return all_ok;
}

static void AppModbusU2_UpdateCache(const app_modbus_u2_ctx_t *ctx, uint16_t start_addr, const modbus_read_resp_t *resp)
{
  uint16_t i;
  uint16_t addr;
  uint16_t needed_regs;
  uint8_t save_params = 0U;
  protocol_values_t values_snap;
  const reg_meta_t *meta;

  if ((ctx == NULL) || (resp == NULL) || (resp->valid == 0U) || (resp->reg_count == 0U) ||
      (ctx->mtx_shared_data == NULL) || (ctx->protocol_values == NULL))
  {
    if ((ctx != NULL) && (ctx->diag != NULL) && (resp != NULL) && (resp->valid != 0U) && (resp->reg_count == 0U))
    {
      ctx->diag->uart2_req_fail++;
    }
    return;
  }

  if (osMutexAcquire(ctx->mtx_shared_data, 5U) != osOK)
  {
    return;
  }

  for (i = 0U; i < resp->reg_count; i++)
  {
    addr = (uint16_t)(start_addr + i);
    meta = ProtocolMap_Find(addr);
    if (meta == NULL)
    {
      continue;
    }
    needed_regs = (uint16_t)(meta->bytes / 2U);
    if ((needed_regs == 0U) || ((i + needed_regs) > resp->reg_count))
    {
      if (ctx->diag != NULL) { ctx->diag->uart2_req_fail++; }
      break;
    }
    AppModbusU2_WriteMappedValue((protocol_values_t *)ctx->protocol_values, meta, &resp->regs[i]);
  }
  if ((start_addr <= 0x09E4U) && ((uint16_t)(start_addr + resp->reg_count) > 0x09C4U))
  {
    values_snap = *(const protocol_values_t *)ctx->protocol_values;
    save_params = 1U;
  }
  osMutexRelease(ctx->mtx_shared_data);

  if (ctx->evt_system_flags != NULL)
  {
    osEventFlagsSet(ctx->evt_system_flags, EVT_MODBUS_UPDATED);
  }

  if (save_params != 0U)
  {
    AppCalc_RecomputeCoeffs(&values_snap);
    (void)AppParamStore_SaveIfChanged(&values_snap);
  }
}

static void AppModbusU2_ProcessChangeFlags(const app_modbus_u2_ctx_t *ctx, uint16_t flags)
{
  modbus_read_resp_t resp;
  HAL_StatusTypeDef ret;
  uint32_t i;
  protocol_values_t values_snap;
  uint8_t zero_start_trigger = 0U;
  uint8_t zero_end_trigger = 0U;
  uint8_t zero_collecting = 0U;
  uint32_t zero_sample_count = 0U;
  uint8_t avg_ok = 0U;
  float avg_power = 0.0f;
  float zero_ch1 = 0.0f;
  float zero_ch2 = 0.0f;
  float zero_verify_ch1 = 0.0f;
  float zero_verify_ch2 = 0.0f;
  uint32_t zero_verify_raw;
  uint8_t save_zero_params = 0U;
  uint16_t zero_regs[4];

  for (i = 0U; i < (sizeof(s_change_queries) / sizeof(s_change_queries[0])); i++)
  {
    if ((flags & s_change_queries[i].flag_mask) != 0U)
    {
      if (AppModbusU2_SendAndWaitRead(ctx, s_change_queries[i].addr, s_change_queries[i].qty, &resp) == HAL_OK)
      {
        AppModbusU2_UpdateCache(ctx, s_change_queries[i].addr, &resp);
      }
    }
  }

  if ((flags & 0x0004U) == 0U)
  {
    U2LOGI("[MODBUS-U2] zero-flow skip: rising flag_meas not set (flags=0x%04X)\r\n", (unsigned int)flags);
  }

  if ((flags & 0x0001U) != 0U)
  {
    if ((ctx != NULL) && (ctx->mtx_shared_data != NULL) && (ctx->protocol_values != NULL) &&
        (osMutexAcquire(ctx->mtx_shared_data, 5U) == osOK))
    {
      values_snap = *(const protocol_values_t *)ctx->protocol_values;
      osMutexRelease(ctx->mtx_shared_data);
      avg_power = AppAvgPowerCache_QueryHours(values_snap.query_time_h, &avg_ok);
      if (avg_ok == 0U)
      {
        avg_power = 0.0f;
      }

      AppModbusU2_WriteFloatToRegs(avg_power, &zero_regs[0]);
      ret = AppModbusU2_SendAndWaitWrite(ctx, 0x07DAU, 2U, zero_regs);
      U2LOGI("[MODBUS-U2] avg-flow query_time_h=%.1f avg=%.3f write 0x07DA ret=%ld\r\n",
            (double)values_snap.query_time_h,
            (double)avg_power,
            (long)ret);

      if (ret == HAL_OK)
      {
        if (osMutexAcquire(ctx->mtx_shared_data, 5U) == osOK)
        {
          ((protocol_values_t *)ctx->protocol_values)->avg_power_kW = avg_power;
          osMutexRelease(ctx->mtx_shared_data);
        }
      }
    }
    else
    {
      U2LOGE("[MODBUS-U2] avg-flow skip: shared data unavailable\r\n");
    }
  }

  if ((flags & 0x0004U) == 0U)
  {
    return;
  }
  if ((ctx == NULL) || (ctx->mtx_shared_data == NULL) || (ctx->protocol_values == NULL))
  {
    return;
  }
  if (osMutexAcquire(ctx->mtx_shared_data, 5U) != osOK)
  {
    return;
  }
  values_snap = *(const protocol_values_t *)ctx->protocol_values;
  osMutexRelease(ctx->mtx_shared_data);
  AppCalc_ZeroCaptureGetState(&zero_collecting, &zero_sample_count);
  U2LOGI("[MODBUS-U2] zero-flow snapshot start=%u end=%u z1=%.3f z2=%.3f\r\n",
        (unsigned int)values_snap.zero_start,
        (unsigned int)values_snap.zero_end,
        (double)values_snap.zero_voltage[0],
        (double)values_snap.zero_voltage[1]);
  U2LOGI("[MODBUS-U2] zero-flow state collecting=%u samples=%lu\r\n",
        (unsigned int)zero_collecting,
        (unsigned long)zero_sample_count);

  if (values_snap.zero_start != 0U)
  {
    AppCalc_ZeroCaptureStart(values_snap.avg_mode);
    zero_start_trigger = 1U;
    AppCalc_ZeroCaptureGetState(&zero_collecting, &zero_sample_count);
    U2LOGI("[MODBUS-U2] zero-flow start triggered avg_mode=%u\r\n",
          (unsigned int)values_snap.avg_mode);
    U2LOGI("[MODBUS-U2] zero-flow state after start collecting=%u samples=%lu\r\n",
          (unsigned int)zero_collecting,
          (unsigned long)zero_sample_count);
  }
  if (values_snap.zero_end != 0U)
  {
    zero_end_trigger = 1U;
    U2LOGI("[MODBUS-U2] zero-flow end triggered\r\n");
  }

  if (zero_start_trigger != 0U)
  {
    zero_regs[0] = 0U;
    ret = AppModbusU2_SendAndWaitWrite(ctx, 0x09E3U, 1U, zero_regs);
    U2LOGI("[MODBUS-U2] zero-flow clear start(0x09E3)=0 ret=%ld\r\n", (long)ret);

    if (osMutexAcquire(ctx->mtx_shared_data, 5U) == osOK)
    {
      ((protocol_values_t *)ctx->protocol_values)->zero_start = 0U;
      osMutexRelease(ctx->mtx_shared_data);
    }
  }

  if (zero_end_trigger != 0U)
  {
    AppCalc_ZeroCaptureGetState(&zero_collecting, &zero_sample_count);
    U2LOGI("[MODBUS-U2] zero-flow state before stop collecting=%u samples=%lu\r\n",
          (unsigned int)zero_collecting,
          (unsigned long)zero_sample_count);
    if (AppCalc_ZeroCaptureStop(&zero_ch1, &zero_ch2) != 0U)
    {
      U2LOGI("[MODBUS-U2] zero-capture done zero_ch1=%0.3f zero_ch2=%0.3f\r\n",
            (double)zero_ch1,
            (double)zero_ch2);
      AppModbusU2_WriteFloatToRegs(zero_ch1, &zero_regs[0]);
      AppModbusU2_WriteFloatToRegs(zero_ch2, &zero_regs[2]);
      ret = AppModbusU2_SendAndWaitWrite(ctx, 0x09DFU, 4U, zero_regs);
      U2LOGI("[MODBUS-U2] zero-flow write zero(0x09DF..0x09E2) ret=%ld\r\n", (long)ret);

      if (ret == HAL_OK)
      {
        if (AppModbusU2_SendAndWaitRead(ctx, 0x09DFU, 4U, &resp) == HAL_OK)
        {
          zero_verify_raw = ((uint32_t)resp.regs[1] << 16) | resp.regs[0];
          memcpy(&zero_verify_ch1, &zero_verify_raw, sizeof(zero_verify_ch1));
          zero_verify_raw = ((uint32_t)resp.regs[3] << 16) | resp.regs[2];
          memcpy(&zero_verify_ch2, &zero_verify_raw, sizeof(zero_verify_ch2));
          U2LOGI("[MODBUS-U2] zero-flow verify read z1=%.3f z2=%.3f\r\n",
                (double)zero_verify_ch1,
                (double)zero_verify_ch2);
        }
        else
        {
          U2LOGE("[MODBUS-U2] zero-flow verify read 0x09DF failed\r\n");
        }
      }

      if (osMutexAcquire(ctx->mtx_shared_data, 5U) == osOK)
      {
        ((protocol_values_t *)ctx->protocol_values)->zero_voltage[0] = zero_ch1;
        ((protocol_values_t *)ctx->protocol_values)->zero_voltage[1] = zero_ch2;
        values_snap = *(const protocol_values_t *)ctx->protocol_values;
        save_zero_params = 1U;
        osMutexRelease(ctx->mtx_shared_data);
      }
      if (save_zero_params != 0U)
      {
        (void)AppParamStore_SaveIfChanged(&values_snap);
      }
    }
    else
    {
      AppCalc_ZeroCaptureGetState(&zero_collecting, &zero_sample_count);
      U2LOGI("[MODBUS-U2] zero-capture stop with empty samples\r\n");
      U2LOGI("[MODBUS-U2] zero-flow state after empty stop collecting=%u samples=%lu\r\n",
            (unsigned int)zero_collecting,
            (unsigned long)zero_sample_count);
    }

    zero_regs[0] = 0U;
    ret = AppModbusU2_SendAndWaitWrite(ctx, 0x09E4U, 1U, zero_regs);
    U2LOGI("[MODBUS-U2] zero-flow clear end(0x09E4)=0 ret=%ld\r\n", (long)ret);
    if (osMutexAcquire(ctx->mtx_shared_data, 5U) == osOK)
    {
      ((protocol_values_t *)ctx->protocol_values)->zero_end = 0U;
      osMutexRelease(ctx->mtx_shared_data);
    }
  }
}

static void AppModbusU2_ClearTriggerFlags(const app_modbus_u2_ctx_t *ctx)
{
  uint16_t regs[3] = {0U, 0U, 0U};
  HAL_StatusTypeDef ret;

  if (ctx == NULL)
  {
    return;
  }

  /* Clear only 0x0AF0..0x0AF2 once after trigger handling; do not touch 0x0AF3. */
  ret = AppModbusU2_SendAndWaitWrite(ctx, 0x0AF0U, 3U, regs);
  if (ret == HAL_TIMEOUT)
  {
    ret = AppModbusU2_SendAndWaitWrite(ctx, 0x0AF0U, 3U, regs);
  }

  if (ret == HAL_OK)
  {
    U2LOGI("[MODBUS-U2] trigger flags 0x0AF0..0x0AF2 cleared\r\n");
  }
  else
  {
    U2LOGE("[MODBUS-U2] trigger flags clear failed ret=%ld\r\n", (long)ret);
  }
}

void AppModbusU2_Init(const app_modbus_u2_ctx_t *ctx)
{
  if ((ctx == NULL) || (ctx->huart2 == NULL) || (ctx->hdma_usart2_rx == NULL) || (ctx->rx_dma_buf == NULL))
  {
    return;
  }
  U2LOGI("[MODBUS-U2] init baud=%lu rx_dma=%u slave=%u\r\n",
       (unsigned long)ctx->huart2->Init.BaudRate,
       (unsigned int)ctx->rx_dma_size,
       (unsigned int)ctx->slave_id);
  U2LOGI("[MODBUS-U2] uart cfg wl=%lu parity=%lu stop=%lu\r\n",
       (unsigned long)ctx->huart2->Init.WordLength,
       (unsigned long)ctx->huart2->Init.Parity,
       (unsigned long)ctx->huart2->Init.StopBits);
  if (HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart2, ctx->rx_dma_buf, ctx->rx_dma_size) != HAL_OK)
  {
    U2LOGE("[MODBUS-U2] rx dma init failed err=0x%08lX\r\n", (unsigned long)ctx->huart2->ErrorCode);
    return;
  }
  s_modbus_rx_dma_last_pos = 0U;
  __HAL_DMA_DISABLE_IT(ctx->hdma_usart2_rx, DMA_IT_HT);
  U2LOGI("[MODBUS-U2] rx dma armed\r\n");
}

static void AppModbusU2_StopRxDma(const app_modbus_u2_ctx_t *ctx)
{
  if ((ctx == NULL) || (ctx->huart2 == NULL))
  {
    return;
  }

  (void)HAL_UART_AbortReceive(ctx->huart2);
  s_modbus_rx_dma_last_pos = 0U;
}

static HAL_StatusTypeDef AppModbusU2_StartRxDma(const app_modbus_u2_ctx_t *ctx)
{
  HAL_StatusTypeDef ret;

  if ((ctx == NULL) || (ctx->huart2 == NULL) || (ctx->hdma_usart2_rx == NULL) || (ctx->rx_dma_buf == NULL))
  {
    return HAL_ERROR;
  }

  s_modbus_rx_dma_last_pos = 0U;
  ret = HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart2, ctx->rx_dma_buf, ctx->rx_dma_size);
  if (ret == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(ctx->hdma_usart2_rx, DMA_IT_HT);
  }
  return ret;
}

static HAL_StatusTypeDef AppModbusU2_StartResponseWindow(const app_modbus_u2_ctx_t *ctx)
{
  modbus_event_msg_t rx_msg;

  if (ctx == NULL)
  {
    return HAL_ERROR;
  }

  if (ctx->q_modbus_events != NULL)
  {
    while (osMessageQueueGet(ctx->q_modbus_events, &rx_msg, NULL, 0U) == osOK) {}
  }
  s_modbus_rx_last_size = 0U;
  s_modbus_rx_last_prev_pos = 0U;
  s_modbus_rx_last_curr_pos = 0U;
  s_modbus_rx_last_chunk_len = 0U;
  s_modbus_rx_last_put_ret = 0;
  return AppModbusU2_StartRxDma(ctx);
}

void AppModbusU2_OnRxEvent(const app_modbus_u2_ctx_t *ctx, uint16_t size)
{
  modbus_event_msg_t mb_msg;
  osStatus_t put_ret;
  HAL_StatusTypeDef rearm_ret;
  HAL_UART_StateTypeDef uart_state;
  uint16_t curr_pos;
  uint16_t prev_pos;
  uint16_t chunk_len;
  uint16_t first_len;
  uint16_t second_len;
  if ((ctx == NULL) || (ctx->huart2 == NULL) || (ctx->hdma_usart2_rx == NULL) || (ctx->rx_dma_buf == NULL))
  {
    return;
  }
  if ((size > 0U) && (size <= ctx->rx_dma_size))
  {
    curr_pos = size;
    prev_pos = s_modbus_rx_dma_last_pos;
    s_modbus_rx_last_tick = HAL_GetTick();
    s_modbus_rx_last_size = size;
    s_modbus_rx_last_prev_pos = prev_pos;
    s_modbus_rx_last_curr_pos = curr_pos;
    s_modbus_rx_last_chunk_len = 0U;
    s_modbus_rx_last_put_ret = 0;

    if (curr_pos == prev_pos)
    {
      goto rearm_dma;
    }

    s_modbus_rx_event_count++;
    mb_msg.timestamp_ms = HAL_GetTick();
    mb_msg.reg_addr = 0U;
    if (curr_pos > prev_pos)
    {
      chunk_len = (uint16_t)(curr_pos - prev_pos);
      if (chunk_len > sizeof(mb_msg.payload))
      {
        chunk_len = (uint16_t)sizeof(mb_msg.payload);
      }
      memcpy(mb_msg.payload, &ctx->rx_dma_buf[prev_pos], chunk_len);
      mb_msg.length = chunk_len;
    }
    else
    {
      first_len = (uint16_t)(ctx->rx_dma_size - prev_pos);
      second_len = curr_pos;
      chunk_len = (uint16_t)(first_len + second_len);
      if (chunk_len > sizeof(mb_msg.payload))
      {
        chunk_len = (uint16_t)sizeof(mb_msg.payload);
      }

      if (chunk_len <= first_len)
      {
        memcpy(mb_msg.payload, &ctx->rx_dma_buf[prev_pos], chunk_len);
      }
      else
      {
        memcpy(mb_msg.payload, &ctx->rx_dma_buf[prev_pos], first_len);
        memcpy(&mb_msg.payload[first_len], &ctx->rx_dma_buf[0], (uint16_t)(chunk_len - first_len));
      }
      mb_msg.length = chunk_len;
    }

    s_modbus_rx_dma_last_pos = curr_pos;
    s_modbus_rx_last_chunk_len = mb_msg.length;
    if (ctx->q_modbus_events != NULL)
    {
      put_ret = osMessageQueuePut(ctx->q_modbus_events, &mb_msg, 0U, 0U);
      s_modbus_rx_last_put_ret = (int32_t)put_ret;
      if (put_ret == osOK)
      {
        s_modbus_rx_queue_put_ok_count++;
      }
      else
      {
        s_modbus_rx_queue_put_fail_count++;
      }
    }
    else
    {
      s_modbus_rx_last_put_ret = -1000;
      s_modbus_rx_queue_put_fail_count++;
    }
  }

rearm_dma:
  uart_state = HAL_UART_GetState(ctx->huart2);
  if ((uart_state == HAL_UART_STATE_BUSY_RX) || (uart_state == HAL_UART_STATE_BUSY_TX_RX))
  {
    __HAL_DMA_DISABLE_IT(ctx->hdma_usart2_rx, DMA_IT_HT);
    return;
  }

  rearm_ret = HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart2, ctx->rx_dma_buf, ctx->rx_dma_size);
  if (rearm_ret != HAL_OK)
  {
    if (rearm_ret == HAL_BUSY)
    {
      s_modbus_rx_rearm_busy_count++;
    }
    else
    {
      s_modbus_rx_rearm_fail_count++;
    }
  }
  __HAL_DMA_DISABLE_IT(ctx->hdma_usart2_rx, DMA_IT_HT);
}

void AppModbusU2_OnError(const app_modbus_u2_ctx_t *ctx)
{
  HAL_StatusTypeDef rearm_ret;
  if ((ctx == NULL) || (ctx->huart2 == NULL) || (ctx->hdma_usart2_rx == NULL) || (ctx->rx_dma_buf == NULL))
  {
    return;
  }
  s_modbus_uart_err_count++;
  rearm_ret = AppModbusU2_StartRxDma(ctx);
  if (rearm_ret != HAL_OK)
  {
    if (rearm_ret == HAL_BUSY)
    {
      s_modbus_rx_rearm_busy_count++;
    }
    else
    {
      s_modbus_rx_rearm_fail_count++;
    }
  }
  __HAL_DMA_DISABLE_IT(ctx->hdma_usart2_rx, DMA_IT_HT);
}

static void AppModbusU2_RecoverRxDma(const app_modbus_u2_ctx_t *ctx)
{
  HAL_StatusTypeDef rearm_ret;

  if ((ctx == NULL) || (ctx->huart2 == NULL) || (ctx->hdma_usart2_rx == NULL) || (ctx->rx_dma_buf == NULL))
  {
    return;
  }

  s_modbus_rx_recover_count++;
  (void)HAL_UART_AbortReceive(ctx->huart2);
  rearm_ret = AppModbusU2_StartRxDma(ctx);
  if (rearm_ret != HAL_OK)
  {
    if (rearm_ret == HAL_BUSY)
    {
      s_modbus_rx_rearm_busy_count++;
    }
    else
    {
      s_modbus_rx_rearm_fail_count++;
      U2LOGE("[MODBUS-U2] rx recover rearm failed ret=%ld err=0x%08lX\r\n",
             (long)rearm_ret,
             (unsigned long)ctx->huart2->ErrorCode);
    }
  }
  U2LOGE("[MODBUS-U2] rx recover ret=%ld recover=%lu state=0x%08lX err=0x%08lX\r\n",
         (long)rearm_ret,
         (unsigned long)s_modbus_rx_recover_count,
         (unsigned long)HAL_UART_GetState(ctx->huart2),
         (unsigned long)ctx->huart2->ErrorCode);
  __HAL_DMA_DISABLE_IT(ctx->hdma_usart2_rx, DMA_IT_HT);
}

void AppModbusU2_OnTxCplt(const app_modbus_u2_ctx_t *ctx)
{
  (void)ctx;
  s_usart2_tx_done = 1U;
#if MODBUS_U2_LOG_REQ_DETAIL
  U2LOGI("[MODBUS-U2] tx complete\r\n");
#endif
}

void AppModbusU2_Process1Hz(const app_modbus_u2_ctx_t *ctx)
{
  modbus_read_resp_t resp;
  uint16_t change_flags = 0U;
  uint16_t rising_flags = 0U;
  uint32_t i;
  uint8_t write_all_ok;

  if (ctx == NULL)
  {
    return;
  }

  U2LOGI("[MODBUS-U2] poll cycle start tick=%lu\r\n", (unsigned long)osKernelGetTickCount());
  U2LOGI("[MODBUS-U2] isr stats rx_evt=%lu uart_err=%lu rearm_fail=%lu rearm_busy=%lu recover=%lu\r\n",
       (unsigned long)s_modbus_rx_event_count,
       (unsigned long)s_modbus_uart_err_count,
       (unsigned long)s_modbus_rx_rearm_fail_count,
       (unsigned long)s_modbus_rx_rearm_busy_count,
       (unsigned long)s_modbus_rx_recover_count);

  write_all_ok = AppModbusU2_Write07xx1Hz(ctx);

  if (AppModbusU2_SendAndWaitRead(ctx, s_status_poll_list[0].addr, s_status_poll_list[0].qty, &resp) == HAL_OK)
  {
    AppModbusU2_UpdateCache(ctx, s_status_poll_list[0].addr, &resp);
    change_flags = 0U;
    for (i = 0U; i < (sizeof(s_flag_maps) / sizeof(s_flag_maps[0])); i++)
    {
      if ((resp.reg_count > s_flag_maps[i].reg_index) &&
          (resp.regs[s_flag_maps[i].reg_index] != 0U))
      {
        change_flags |= s_flag_maps[i].flag_mask;
      }
    }

    U2LOGI("[MODBUS-U2] flags 0x0AF0..0x0AF2 = 0x%04X\r\n", (unsigned int)change_flags);
    rising_flags = (uint16_t)(change_flags & (uint16_t)(~s_prev_change_flags));
    s_prev_change_flags = change_flags;
    U2LOGI("[MODBUS-U2] rising flags = 0x%04X\r\n", (unsigned int)rising_flags);

    if ((write_all_ok != 0U) && (rising_flags != 0U))
    {
      AppModbusU2_ProcessChangeFlags(ctx, rising_flags);
    }

    if ((write_all_ok != 0U) && (change_flags != 0U))
    {
      AppModbusU2_ClearTriggerFlags(ctx);
    }
    else if (write_all_ok == 0U)
    {
      U2LOGE("[MODBUS-U2] skip trigger reads due to 0x07xx write timeout\r\n");
    }
  }

  if (ctx->evt_system_flags != NULL)
  {
    osEventFlagsSet(ctx->evt_system_flags, EVT_HEALTH_MODBUS);
  }

  U2LOGI("[MODBUS-U2] poll cycle end ok=%lu fail=%lu timeout=%lu\r\n",
       (unsigned long)((ctx->diag != NULL) ? ctx->diag->uart2_req_ok : 0U),
       (unsigned long)((ctx->diag != NULL) ? ctx->diag->uart2_req_fail : 0U),
       (unsigned long)((ctx->diag != NULL) ? ctx->diag->uart2_req_timeout : 0U));
  U2LOGI("[MODBUS-U2] write07 ok=%lu timeout=%lu\r\n",
       (unsigned long)s_modbus_write_ok_count,
       (unsigned long)s_modbus_write_timeout_count);
}
static void AppModbusU2_WriteMappedValue(protocol_values_t *values,
                                         const reg_meta_t *meta,
                                         const uint16_t *regs)
{
  uint8_t *base;
  uint32_t u32;
  int32_t s32;
  float f32;

  if ((values == NULL) || (meta == NULL) || (regs == NULL))
  {
    return;
  }

  base = (uint8_t *)values + meta->value_offset;
  if (meta->type == REG_TYPE_SHORT)
  {
    *(uint16_t *)base = regs[0];
  }
  else
  {
    u32 = ((uint32_t)regs[1] << 16) | regs[0];
    if (meta->type == REG_TYPE_INT32)
    {
      s32 = (int32_t)u32;
      *(int32_t *)base = s32;
    }
    else
    {
      memcpy(&f32, &u32, sizeof(f32));
      *(float *)base = f32;
    }
  }
}
