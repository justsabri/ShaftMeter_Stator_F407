#ifndef __APP_CALC_H
#define __APP_CALC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os2.h"
#include "protocol_values.h"

typedef struct
{
  osMutexId_t mtx_shared_data;
  volatile protocol_values_t *protocol_values;
  volatile result_1s_t *result_latest;
  volatile diag_counters_t *diag;
} app_calc_ctx_t;

void AppCalc_Init(const app_calc_ctx_t *ctx);
void AppCalc_RecomputeCoeffs(const protocol_values_t *values);
void AppCalc_FeedSample(float ch1, float ch2);
void AppCalc_FeedRpm(float rpm);
void AppCalc_Compute1s(const app_calc_ctx_t *ctx, result_1s_t *out);
void AppCalc_SetForceZeroInput(uint8_t enable);
void AppCalc_ZeroCaptureStart(uint16_t avg_mode);
uint8_t AppCalc_ZeroCaptureStop(float *zero_ch1, float *zero_ch2);
void AppCalc_ZeroCaptureGetState(uint8_t *collecting, uint32_t *sample_count);

#ifdef __cplusplus
}
#endif

#endif
