#include "app_avg_power_cache.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define SEC_WINDOW_SIZE      360U
#define BUCKET_SIZE_SEC      360U
#define BUCKET_COUNT         240U
#define MIN_QUERY_HOURS      0.1f
#define MAX_QUERY_HOURS      24.0f

typedef struct
{
  float sum;
  uint16_t count;
  uint32_t bucket_id;
} avg_bucket_t;

static float s_sec_ring[SEC_WINDOW_SIZE];
static uint32_t s_sec_head = 0U;
static uint32_t s_sec_count = 0U;
static float s_sec_sum = 0.0f;
static uint32_t s_last_mono_sec = 0U;

static avg_bucket_t s_bucket_ring[BUCKET_COUNT];

static uint32_t AppAvgPowerCache_RoundToTenths(float hours)
{
  float scaled = hours * 10.0f;
  if (scaled < 1.0f)
  {
    scaled = 1.0f;
  }
  return (uint32_t)(scaled + 0.5f);
}

void AppAvgPowerCache_Init(void)
{
  taskENTER_CRITICAL();
  memset(s_sec_ring, 0, sizeof(s_sec_ring));
  s_sec_head = 0U;
  s_sec_count = 0U;
  s_sec_sum = 0.0f;
  s_last_mono_sec = 0U;
  memset(s_bucket_ring, 0, sizeof(s_bucket_ring));
  taskEXIT_CRITICAL();
}

void AppAvgPowerCache_Push1s(float power_kw, uint32_t mono_sec)
{
  uint32_t bucket_id;
  uint32_t slot;
  avg_bucket_t *b;

  taskENTER_CRITICAL();
  if (s_sec_count < SEC_WINDOW_SIZE)
  {
    s_sec_ring[s_sec_head] = power_kw;
    s_sec_sum += power_kw;
    s_sec_count++;
  }
  else
  {
    s_sec_sum -= s_sec_ring[s_sec_head];
    s_sec_ring[s_sec_head] = power_kw;
    s_sec_sum += power_kw;
  }
  s_sec_head = (s_sec_head + 1U) % SEC_WINDOW_SIZE;

  bucket_id = mono_sec / BUCKET_SIZE_SEC;
  slot = bucket_id % BUCKET_COUNT;
  b = &s_bucket_ring[slot];
  if (b->bucket_id != bucket_id)
  {
    b->sum = 0.0f;
    b->count = 0U;
    b->bucket_id = bucket_id;
  }
  b->sum += power_kw;
  if (b->count < 0xFFFFU)
  {
    b->count++;
  }
  s_last_mono_sec = mono_sec;
  taskEXIT_CRITICAL();
}

float AppAvgPowerCache_QueryHours(float hours, uint8_t *ok)
{
  uint32_t n_tenths;
  uint32_t cur_bucket_id;
  uint32_t i;
  float sum = 0.0f;
  uint32_t cnt = 0U;

  if (hours < MIN_QUERY_HOURS)
  {
    hours = MIN_QUERY_HOURS;
  }
  else if (hours > MAX_QUERY_HOURS)
  {
    hours = MAX_QUERY_HOURS;
  }

  n_tenths = AppAvgPowerCache_RoundToTenths(hours);
  if (n_tenths < 1U)
  {
    n_tenths = 1U;
  }
  if (n_tenths > BUCKET_COUNT)
  {
    n_tenths = BUCKET_COUNT;
  }

  taskENTER_CRITICAL();
  sum += s_sec_sum;
  cnt += s_sec_count;

  cur_bucket_id = s_last_mono_sec / BUCKET_SIZE_SEC;

  for (i = 1U; i < n_tenths; i++)
  {
    uint32_t id = cur_bucket_id - i;
    avg_bucket_t *b = &s_bucket_ring[id % BUCKET_COUNT];
    if (b->bucket_id == id)
    {
      sum += b->sum;
      cnt += b->count;
    }
  }
  taskEXIT_CRITICAL();

  if (cnt == 0U)
  {
    if (ok != 0)
    {
      *ok = 0U;
    }
    return 0.0f;
  }

  if (ok != 0)
  {
    *ok = 1U;
  }
  return sum / (float)cnt;
}
