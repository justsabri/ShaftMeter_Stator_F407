#include "app_tx_result.h"
#include "modbus_master.h"
#include <string.h>

#define TX_RESULT_REG_COUNT 8U

static void AppTxResult_FloatToRegs(float value, uint8_t *out_hi, uint8_t *out_lo)
{
  uint32_t raw;
  memcpy(&raw, &value, sizeof(raw));
  out_hi[0] = (uint8_t)(raw >> 24);
  out_hi[1] = (uint8_t)(raw >> 16);
  out_lo[0] = (uint8_t)(raw >> 8);
  out_lo[1] = (uint8_t)raw;
}

void AppTxResult_SetRs485Usart2(uint8_t enable)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void AppTxResult_SetRs485Usart3(uint8_t enable)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void AppTxResult_SetRs485Usart6(uint8_t enable)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void AppTxResult_UpdateEnableFromConfig(const protocol_values_t *values,
                                        volatile uint8_t *en_usart3,
                                        volatile uint8_t *en_uart6)
{
  if ((values == NULL) || (en_usart3 == NULL) || (en_uart6 == NULL))
  {
    return;
  }
  *en_usart3 = ((values->flag_uart_enable == 1U) || (values->flag_uart_enable == 3U)) ? 1U : 0U;
  *en_uart6 = ((values->flag_uart_enable == 2U) || (values->flag_uart_enable == 3U)) ? 1U : 0U;
}

uint16_t AppTxResult_BuildWriteFrame(uint8_t slave_id, uint16_t start_addr, const result_1s_t *result, uint8_t *out, uint16_t out_size)
{
  uint16_t crc;

  if ((result == NULL) || (out == NULL) || (out_size < 25U))
  {
    return 0U;
  }

  out[0] = slave_id;
  out[1] = 0x10U;
  out[2] = (uint8_t)(start_addr >> 8);
  out[3] = (uint8_t)(start_addr & 0xFFU);
  out[4] = 0x00U;
  out[5] = TX_RESULT_REG_COUNT;
  out[6] = 16U;

  AppTxResult_FloatToRegs(result->torque, &out[7], &out[9]);
  AppTxResult_FloatToRegs(result->power, &out[11], &out[13]);
  AppTxResult_FloatToRegs(result->rpm, &out[15], &out[17]);
  AppTxResult_FloatToRegs(result->thrust, &out[19], &out[21]);

  crc = ModbusMaster_Crc16(out, 23U);
  out[23] = (uint8_t)(crc & 0xFFU);
  out[24] = (uint8_t)(crc >> 8);

  return 25U;
}

HAL_StatusTypeDef AppTxResult_SendUsart3(const app_tx_result_ctx_t *ctx, const uint8_t *frame, uint16_t len, uint32_t timeout_ms)
{
  uint32_t start_tick;

  if ((ctx == NULL) || (ctx->huart3 == NULL) || (ctx->usart3_tx_done == NULL))
  {
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while (*(ctx->usart3_tx_done) == 0U)
  {
    if ((HAL_GetTick() - start_tick) > timeout_ms)
    {
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }

  *(ctx->usart3_tx_done) = 0U;
  AppTxResult_SetRs485Usart3(1U);
  if (HAL_UART_Transmit_DMA(ctx->huart3, (uint8_t *)frame, len) != HAL_OK)
  {
    AppTxResult_SetRs485Usart3(0U);
    *(ctx->usart3_tx_done) = 1U;
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while (*(ctx->usart3_tx_done) == 0U)
  {
    if ((HAL_GetTick() - start_tick) > timeout_ms)
    {
      AppTxResult_SetRs485Usart3(0U);
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }

  AppTxResult_SetRs485Usart3(0U);
  return HAL_OK;
}

HAL_StatusTypeDef AppTxResult_SendUsart6(const app_tx_result_ctx_t *ctx, const uint8_t *frame, uint16_t len, uint32_t timeout_ms)
{
  HAL_StatusTypeDef ret;

  if ((ctx == NULL) || (ctx->huart6 == NULL))
  {
    return HAL_ERROR;
  }

  AppTxResult_SetRs485Usart6(1U);
  ret = HAL_UART_Transmit(ctx->huart6, (uint8_t *)frame, len, timeout_ms);
  AppTxResult_SetRs485Usart6(0U);
  return ret;
}
