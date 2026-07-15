#ifndef __RPM_H
#define __RPM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  float rpm;
  uint32_t period_us;
  uint8_t valid;
} rpm_calc_result_t;

void RPM_Init(uint32_t pulses_per_rev,
              uint32_t min_period_us,
              uint32_t timeout_us,
              uint32_t avg_window_us,
              uint8_t avg_enable);
rpm_calc_result_t RPM_OnPulse(uint32_t timestamp_us);
rpm_calc_result_t RPM_OnPeriodicCheck(uint32_t timestamp_us);

#ifdef __cplusplus
}
#endif

#endif
