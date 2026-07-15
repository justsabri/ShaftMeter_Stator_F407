#include "modbus_master.h"

uint16_t ModbusMaster_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t j;

  for (i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (j = 0; j < 8U; j++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (crc >> 1) ^ 0xA001U;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

uint16_t ModbusMaster_BuildReadHolding(const modbus_read_req_t *req, uint8_t *out, uint16_t out_size)
{
  uint16_t crc;

  if ((req == 0) || (out == 0) || (out_size < 8U))
  {
    return 0U;
  }

  out[0] = req->slave_id;
  out[1] = req->function_code;
  out[2] = (uint8_t)(req->start_addr >> 8);
  out[3] = (uint8_t)(req->start_addr & 0xFFU);
  out[4] = (uint8_t)(req->quantity >> 8);
  out[5] = (uint8_t)(req->quantity & 0xFFU);
  crc = ModbusMaster_Crc16(out, 6U);
  out[6] = (uint8_t)(crc & 0xFFU);
  out[7] = (uint8_t)(crc >> 8);
  return 8U;
}

uint8_t ModbusMaster_ParseReadHoldingResp(const uint8_t *buf, uint16_t len, modbus_read_resp_t *out)
{
  uint16_t crc_calc;
  uint16_t crc_rx;
  uint16_t i;
  uint16_t reg_count;

  if ((buf == 0) || (out == 0) || (len < 5U))
  {
    return 0U;
  }

  crc_calc = ModbusMaster_Crc16(buf, (uint16_t)(len - 2U));
  crc_rx = (uint16_t)buf[len - 2U] | ((uint16_t)buf[len - 1U] << 8);
  if (crc_calc != crc_rx)
  {
    return 0U;
  }

  out->slave_id = buf[0];
  out->function_code = buf[1];
  out->byte_count = buf[2];

  if ((out->function_code != 0x03U) || ((uint16_t)out->byte_count + 5U != len))
  {
    return 0U;
  }

  reg_count = (uint16_t)(out->byte_count / 2U);
  if (reg_count > 32U)
  {
    return 0U;
  }

  out->reg_count = reg_count;
  for (i = 0; i < reg_count; i++)
  {
    out->regs[i] = ((uint16_t)buf[3U + (2U * i)] << 8) | buf[4U + (2U * i)];
  }
  out->valid = 1U;
  return 1U;
}
