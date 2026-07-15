#include "app_param_store.h"
#include "main.h"
#include <string.h>

#define PARAM_STORE_FLASH_ADDR   0x080E0000UL
#define PARAM_STORE_FLASH_SECTOR FLASH_SECTOR_11
#define PARAM_STORE_MAGIC        0x53504350UL
#define PARAM_STORE_VERSION      1UL

typedef struct
{
  float query_time_h;
  float shaft_outer_d_mm;
  float shaft_inner_d_mm;
  float shear_modulus_GPa;
  float elastic_modulus_GPa;
  float poisson_ratio;
  uint16_t prop_dir;
  uint16_t modulus_select;
  float sample_freq_Hz;
  int32_t avg_time_ms;
  uint16_t avg_mode;
  float calib_factor[2];
  float gauge_sensitivity[2];
  float zero_voltage[2];
} param_store_payload_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t payload_size;
  uint32_t crc;
  param_store_payload_t payload;
} param_store_record_t;

static param_store_payload_t s_last_payload;
static uint8_t s_last_payload_valid = 0U;

static uint32_t AppParamStore_Crc32(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;
  uint8_t bit;

  if (data == NULL)
  {
    return 0UL;
  }

  for (i = 0UL; i < len; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1UL) != 0UL)
      {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

static void AppParamStore_PayloadFromValues(const protocol_values_t *values, param_store_payload_t *payload)
{
  if ((values == NULL) || (payload == NULL))
  {
    return;
  }

  payload->query_time_h = values->query_time_h;
  payload->shaft_outer_d_mm = values->shaft_outer_d_mm;
  payload->shaft_inner_d_mm = values->shaft_inner_d_mm;
  payload->shear_modulus_GPa = values->shear_modulus_GPa;
  payload->elastic_modulus_GPa = values->elastic_modulus_GPa;
  payload->poisson_ratio = values->poisson_ratio;
  payload->prop_dir = values->prop_dir;
  payload->modulus_select = values->modulus_select;
  payload->sample_freq_Hz = values->sample_freq_Hz;
  payload->avg_time_ms = values->avg_time_ms;
  payload->avg_mode = values->avg_mode;
  payload->calib_factor[0] = values->calib_factor[0];
  payload->calib_factor[1] = values->calib_factor[1];
  payload->gauge_sensitivity[0] = values->gauge_sensitivity[0];
  payload->gauge_sensitivity[1] = values->gauge_sensitivity[1];
  payload->zero_voltage[0] = values->zero_voltage[0];
  payload->zero_voltage[1] = values->zero_voltage[1];
}

static void AppParamStore_ApplyPayload(protocol_values_t *values, const param_store_payload_t *payload)
{
  if ((values == NULL) || (payload == NULL))
  {
    return;
  }

  values->query_time_h = payload->query_time_h;
  values->shaft_outer_d_mm = payload->shaft_outer_d_mm;
  values->shaft_inner_d_mm = payload->shaft_inner_d_mm;
  values->shear_modulus_GPa = payload->shear_modulus_GPa;
  values->elastic_modulus_GPa = payload->elastic_modulus_GPa;
  values->poisson_ratio = payload->poisson_ratio;
  values->prop_dir = payload->prop_dir;
  values->modulus_select = payload->modulus_select;
  values->sample_freq_Hz = payload->sample_freq_Hz;
  values->avg_time_ms = payload->avg_time_ms;
  values->avg_mode = payload->avg_mode;
  values->calib_factor[0] = payload->calib_factor[0];
  values->calib_factor[1] = payload->calib_factor[1];
  values->gauge_sensitivity[0] = payload->gauge_sensitivity[0];
  values->gauge_sensitivity[1] = payload->gauge_sensitivity[1];
  values->zero_voltage[0] = payload->zero_voltage[0];
  values->zero_voltage[1] = payload->zero_voltage[1];
}

uint8_t AppParamStore_Load(protocol_values_t *values)
{
  const param_store_record_t *record = (const param_store_record_t *)PARAM_STORE_FLASH_ADDR;
  uint32_t crc;

  if (values == NULL)
  {
    return 0U;
  }

  if ((record->magic != PARAM_STORE_MAGIC) ||
      (record->version != PARAM_STORE_VERSION) ||
      (record->payload_size != sizeof(param_store_payload_t)))
  {
    s_last_payload_valid = 0U;
    return 0U;
  }

  crc = AppParamStore_Crc32((const uint8_t *)&record->payload, sizeof(record->payload));
  if (crc != record->crc)
  {
    s_last_payload_valid = 0U;
    return 0U;
  }

  AppParamStore_ApplyPayload(values, &record->payload);
  s_last_payload = record->payload;
  s_last_payload_valid = 1U;
  LOGI("[PARAM] loaded coeff params from flash\r\n");
  return 1U;
}

uint8_t AppParamStore_SaveIfChanged(const protocol_values_t *values)
{
  param_store_record_t record;
  param_store_payload_t payload;
  FLASH_EraseInitTypeDef erase;
  uint32_t sector_error = 0UL;
  uint32_t addr;
  const uint8_t *bytes;
  uint32_t i;
  HAL_StatusTypeDef ret;

  if (values == NULL)
  {
    return 0U;
  }

  AppParamStore_PayloadFromValues(values, &payload);
  if ((s_last_payload_valid != 0U) &&
      (memcmp(&payload, &s_last_payload, sizeof(payload)) == 0))
  {
    return 1U;
  }

  memset(&record, 0xFF, sizeof(record));
  record.magic = PARAM_STORE_MAGIC;
  record.version = PARAM_STORE_VERSION;
  record.payload_size = sizeof(param_store_payload_t);
  record.payload = payload;
  record.crc = AppParamStore_Crc32((const uint8_t *)&record.payload, sizeof(record.payload));

  ret = HAL_FLASH_Unlock();
  if (ret != HAL_OK)
  {
    LOGE("[PARAM] flash unlock failed ret=%ld\r\n", (long)ret);
    return 0U;
  }

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = PARAM_STORE_FLASH_SECTOR;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  ret = HAL_FLASHEx_Erase(&erase, &sector_error);
  if (ret != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    LOGE("[PARAM] flash erase failed ret=%ld sector_err=%lu\r\n", (long)ret, (unsigned long)sector_error);
    return 0U;
  }

  addr = PARAM_STORE_FLASH_ADDR;
  bytes = (const uint8_t *)&record;
  for (i = 0UL; i < sizeof(record); i++)
  {
    ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, bytes[i]);
    if (ret != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      LOGE("[PARAM] flash program failed ret=%ld offset=%lu\r\n", (long)ret, (unsigned long)i);
      return 0U;
    }
  }

  (void)HAL_FLASH_Lock();
  s_last_payload = payload;
  s_last_payload_valid = 1U;
  LOGI("[PARAM] saved coeff params to flash\r\n");
  return 1U;
}
