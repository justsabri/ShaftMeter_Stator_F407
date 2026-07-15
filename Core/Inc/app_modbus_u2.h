#ifndef __APP_MODBUS_U2_H
#define __APP_MODBUS_U2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "cmsis_os2.h"
#include "modbus_master.h"
#include "protocol_map.h"
#include "protocol_values.h"

typedef struct
{
  uint32_t timestamp_ms;
  uint8_t payload[128];
  uint16_t length;
  uint16_t reg_addr;
} modbus_event_msg_t;

typedef struct
{
  uint16_t addr;
  uint16_t qty;
} modbus_poll_item_t;

typedef struct
{
  UART_HandleTypeDef *huart2;
  DMA_HandleTypeDef *hdma_usart2_rx;
  uint8_t *rx_dma_buf;
  uint16_t rx_dma_size;
  osMessageQueueId_t q_modbus_events;
  osMutexId_t mtx_shared_data;
  osEventFlagsId_t evt_system_flags;
  volatile protocol_values_t *protocol_values;
  volatile diag_counters_t *diag;
  uint8_t slave_id;
  const modbus_poll_item_t *poll_list;
  uint32_t poll_list_count;
  void (*rs485_set_tx_usart2)(uint8_t enable);
} app_modbus_u2_ctx_t;

void AppModbusU2_Init(const app_modbus_u2_ctx_t *ctx);
void AppModbusU2_OnRxEvent(const app_modbus_u2_ctx_t *ctx, uint16_t size);
void AppModbusU2_OnError(const app_modbus_u2_ctx_t *ctx);
void AppModbusU2_OnTxCplt(const app_modbus_u2_ctx_t *ctx);
void AppModbusU2_Process1Hz(const app_modbus_u2_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif
