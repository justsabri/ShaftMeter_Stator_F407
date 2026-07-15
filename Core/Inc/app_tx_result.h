#ifndef __APP_TX_RESULT_H
#define __APP_TX_RESULT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "protocol_values.h"

typedef struct
{
  UART_HandleTypeDef *huart3;
  UART_HandleTypeDef *huart6;
  volatile uint8_t *usart3_tx_done;
  volatile uint8_t *tx_enable_usart3;
  volatile uint8_t *tx_enable_uart6;
  volatile protocol_values_t *protocol_values;
  osMutexId_t mtx_shared_data;
} app_tx_result_ctx_t;

void AppTxResult_SetRs485Usart2(uint8_t enable);
void AppTxResult_SetRs485Usart3(uint8_t enable);
void AppTxResult_SetRs485Usart6(uint8_t enable);
void AppTxResult_UpdateEnableFromConfig(const protocol_values_t *values,
                                        volatile uint8_t *en_usart3,
                                        volatile uint8_t *en_uart6);
uint16_t AppTxResult_BuildWriteFrame(uint8_t slave_id, uint16_t start_addr, const result_1s_t *result, uint8_t *out, uint16_t out_size);
HAL_StatusTypeDef AppTxResult_SendUsart3(const app_tx_result_ctx_t *ctx, const uint8_t *frame, uint16_t len, uint32_t timeout_ms);
HAL_StatusTypeDef AppTxResult_SendUsart6(const app_tx_result_ctx_t *ctx, const uint8_t *frame, uint16_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
