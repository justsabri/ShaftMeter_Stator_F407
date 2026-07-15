#ifndef __MODBUS_MASTER_H
#define __MODBUS_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  uint8_t slave_id;
  uint8_t function_code;
  uint16_t start_addr;
  uint16_t quantity;
} modbus_read_req_t;

typedef struct
{
  uint8_t slave_id;
  uint8_t function_code;
  uint8_t byte_count;
  uint16_t regs[32];
  uint16_t reg_count;
  uint8_t valid;
} modbus_read_resp_t;

uint16_t ModbusMaster_Crc16(const uint8_t *data, uint16_t len);
uint16_t ModbusMaster_BuildReadHolding(const modbus_read_req_t *req, uint8_t *out, uint16_t out_size);
uint8_t ModbusMaster_ParseReadHoldingResp(const uint8_t *buf, uint16_t len, modbus_read_resp_t *out);

#ifdef __cplusplus
}
#endif

#endif
