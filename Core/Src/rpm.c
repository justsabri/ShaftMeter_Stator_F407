#include "rpm.h"
#include "main.h"

#define RPM_AVG_SAMPLE_CAP 128U

typedef struct
{
  uint32_t timestamp_us;
  float rpm;
} rpm_avg_sample_t;

typedef struct
{
  uint32_t last_ts_us;
  uint32_t ppr;
  uint32_t min_period_us;
  uint32_t timeout_us;
  uint32_t avg_window_us;
  uint8_t avg_enable;
  uint8_t has_last;
  rpm_calc_result_t latest;
  rpm_avg_sample_t avg_samples[RPM_AVG_SAMPLE_CAP];
  uint16_t avg_head;
  uint16_t avg_count;
} rpm_ctx_t;

static rpm_ctx_t s_rpm_ctx;

static void RPM_AvgPush(uint32_t timestamp_us, float rpm)
{
  s_rpm_ctx.avg_samples[s_rpm_ctx.avg_head].timestamp_us = timestamp_us;
  s_rpm_ctx.avg_samples[s_rpm_ctx.avg_head].rpm = rpm;
  s_rpm_ctx.avg_head = (uint16_t)((s_rpm_ctx.avg_head + 1U) % RPM_AVG_SAMPLE_CAP);
  if (s_rpm_ctx.avg_count < RPM_AVG_SAMPLE_CAP)
  {
    s_rpm_ctx.avg_count++;
  }
}

static float RPM_AvgQuery(uint32_t timestamp_us)
{
  double sum = 0.0;
  uint16_t used = 0U;
  uint16_t n;

  for (n = 0U; n < s_rpm_ctx.avg_count; n++)
  {
    uint16_t idx = (uint16_t)((s_rpm_ctx.avg_head + RPM_AVG_SAMPLE_CAP - 1U - n) % RPM_AVG_SAMPLE_CAP);
    uint32_t age_us = timestamp_us - s_rpm_ctx.avg_samples[idx].timestamp_us;
    if (age_us > s_rpm_ctx.avg_window_us)
    {
      break;
    }
    sum += (double)s_rpm_ctx.avg_samples[idx].rpm;
    used++;
  }

  if (used == 0U)
  {
    return s_rpm_ctx.latest.rpm;
  }
  return (float)(sum / (double)used);
}

void RPM_Init(uint32_t pulses_per_rev,
              uint32_t min_period_us,
              uint32_t timeout_us,
              uint32_t avg_window_us,
              uint8_t avg_enable)
{
  s_rpm_ctx.last_ts_us = 0U;
  s_rpm_ctx.ppr = (pulses_per_rev == 0U) ? 1U : pulses_per_rev;
  s_rpm_ctx.min_period_us = (min_period_us == 0U) ? 200U : min_period_us;
  s_rpm_ctx.timeout_us = (timeout_us == 0U) ? 1000000U : timeout_us;
  s_rpm_ctx.avg_window_us = (avg_window_us == 0U) ? 3000000U : avg_window_us;
  s_rpm_ctx.avg_enable = (avg_enable != 0U) ? 1U : 0U;
  s_rpm_ctx.has_last = 0U;
  s_rpm_ctx.latest.rpm = 0.0f;
  s_rpm_ctx.latest.period_us = 0U;
  s_rpm_ctx.latest.valid = 0U;
  s_rpm_ctx.avg_head = 0U;
  s_rpm_ctx.avg_count = 0U;
}

rpm_calc_result_t RPM_OnPulse(uint32_t timestamp_us)
{
  uint32_t delta_us;
  float instant_rpm;

  if (s_rpm_ctx.has_last == 0U)
  {
    s_rpm_ctx.last_ts_us = timestamp_us;
    s_rpm_ctx.has_last = 1U;
    return s_rpm_ctx.latest;
  }

  delta_us = timestamp_us - s_rpm_ctx.last_ts_us;
  s_rpm_ctx.last_ts_us = timestamp_us;

  if (delta_us < s_rpm_ctx.min_period_us)
  {
    return s_rpm_ctx.latest;
  }

  s_rpm_ctx.latest.period_us = delta_us;
  instant_rpm = (60.0f * 1000000.0f) / ((float)delta_us * (float)s_rpm_ctx.ppr);
  if (s_rpm_ctx.avg_enable != 0U)
  {
    RPM_AvgPush(timestamp_us, instant_rpm);
    s_rpm_ctx.latest.rpm = RPM_AvgQuery(timestamp_us);
  }
  else
  {
    s_rpm_ctx.latest.rpm = instant_rpm;
  }
  s_rpm_ctx.latest.valid = 1U;
  LOGI("rpm=%0.2f instant=%0.2f period_us=%lu avg_enable=%u avg_window_us=%lu\r\n",
       (double)s_rpm_ctx.latest.rpm,
       (double)instant_rpm,
       (unsigned long)(s_rpm_ctx.latest.period_us),
       (unsigned int)s_rpm_ctx.avg_enable,
       (unsigned long)s_rpm_ctx.avg_window_us);
  return s_rpm_ctx.latest;
}

rpm_calc_result_t RPM_OnPeriodicCheck(uint32_t timestamp_us)
{
  if ((s_rpm_ctx.has_last != 0U) && ((timestamp_us - s_rpm_ctx.last_ts_us) > s_rpm_ctx.timeout_us))
  {
    if (s_rpm_ctx.latest.rpm != 0.0f)
    {
      LOGI("rpm timeout last_period_us=%lu idle_us=%lu timeout_us=%lu\r\n",
           (unsigned long)s_rpm_ctx.latest.period_us,
           (unsigned long)(timestamp_us - s_rpm_ctx.last_ts_us),
           (unsigned long)s_rpm_ctx.timeout_us);
      s_rpm_ctx.latest.rpm = 0.0f;
      s_rpm_ctx.latest.period_us = 0U;
      s_rpm_ctx.latest.valid = 1U;
    }
  }
  return s_rpm_ctx.latest;
}
