#ifndef __APP_USART1_H
#define __APP_USART1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "cmsis_os2.h"

typedef struct
{
  uint32_t timestamp_ms;
  uint8_t payload[128];
  uint16_t length;
} usart1_frame_msg_t;

typedef struct
{
  UART_HandleTypeDef *huart;
  DMA_HandleTypeDef *hdma_rx;
  uint8_t *rx_dma_buf;
  uint16_t rx_dma_buf_size;
  osMessageQueueId_t q_rx_frames;
  osMutexId_t mtx_shared_data;
  osEventFlagsId_t evt_system_flags;
  volatile wireless_sample_t *wireless_latest;
  volatile diag_counters_t *diag;
  void (*on_sample)(float ch1, float ch2);
  void (*on_board_sample)(float board);
  void (*on_frame_no)(uint16_t frame_no);
  void (*on_rx_interrupt)(void);
  uint16_t (*get_sample_freq_hz)(void);
} app_usart1_ctx_t;

void AppUsart1_Init(const app_usart1_ctx_t *ctx);
void AppUsart1_Process(const app_usart1_ctx_t *ctx, uint32_t now_tick_ms);
void AppUsart1_OnTxCplt(const app_usart1_ctx_t *ctx);
void AppUsart1_OnRxEvent(const app_usart1_ctx_t *ctx, uint16_t size);
void AppUsart1_OnError(const app_usart1_ctx_t *ctx);

void AppUsart1_SendStartSampling(const app_usart1_ctx_t *ctx);
void AppUsart1_SendStopSampling(const app_usart1_ctx_t *ctx);
void AppUsart1_SendSetFrequency(const app_usart1_ctx_t *ctx, uint16_t frequency);
void AppUsart1_SendStartSamplingWithLog(const app_usart1_ctx_t *ctx, const char *reason);
void AppUsart1_SendStopSamplingWithLog(const app_usart1_ctx_t *ctx, const char *reason);
void AppUsart1_SendSetFrequencyWithLog(const app_usart1_ctx_t *ctx, uint16_t frequency, const char *reason);

#ifdef __cplusplus
}
#endif

#endif
