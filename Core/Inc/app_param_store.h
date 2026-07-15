#ifndef __APP_PARAM_STORE_H
#define __APP_PARAM_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "protocol_values.h"

uint8_t AppParamStore_Load(protocol_values_t *values);
uint8_t AppParamStore_SaveIfChanged(const protocol_values_t *values);

#ifdef __cplusplus
}
#endif

#endif
