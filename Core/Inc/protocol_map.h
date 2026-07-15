#ifndef __PROTOCOL_MAP_H
#define __PROTOCOL_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  REG_TYPE_SHORT = 0,
  REG_TYPE_INT32 = 1,
  REG_TYPE_FLOAT = 2
} reg_data_type_t;

typedef struct
{
  uint16_t addr;
  const char *name;
  reg_data_type_t type;
  uint8_t bytes;
  uint8_t decimals;
  uint16_t value_offset;
} reg_meta_t;

extern const reg_meta_t g_reg_meta_table[];
extern const uint32_t g_reg_meta_count;

const reg_meta_t *ProtocolMap_Find(uint16_t addr);

#ifdef __cplusplus
}
#endif

#endif
