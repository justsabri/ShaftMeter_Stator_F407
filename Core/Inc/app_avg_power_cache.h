#ifndef __APP_AVG_POWER_CACHE_H
#define __APP_AVG_POWER_CACHE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void AppAvgPowerCache_Init(void);
void AppAvgPowerCache_Push1s(float power_kw, uint32_t mono_sec);
float AppAvgPowerCache_QueryHours(float hours, uint8_t *ok);

#ifdef __cplusplus
}
#endif

#endif
