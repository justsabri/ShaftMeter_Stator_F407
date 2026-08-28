#include "app_calc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define OMEGA_PER_RPM           0.104719755f
#define DEFAULT_ZERO_VOLTAGE    0.0f
#define MM_TO_M                 0.001f
#define MV_TO_V                 0.001f
#define GPA_TO_PA               1000000000.0f
#define MIN_GEOM_EPS            1.0e-9f
#define PI_F                    3.14159265359f
#define MICRO_STRAIN_SCALE      1000000.0f
#define APP_CALC_LOG_DETAIL     0U
#define APP_CALC_LOG_COEFF_SNAPSHOT 1U

typedef struct
{
  double ch1_sum;
  double ch2_sum;
  double ch1_sq_sum;
  double ch2_sq_sum;
  float rpm_sum;
  uint32_t sample_count;
  uint32_t rpm_count;
} calc_accum_t;

static volatile calc_accum_t s_calc_accum;
static uint8_t s_zero_collecting = 0U;
static double s_zero_sum_ch1 = 0.0;
static double s_zero_sum_ch2 = 0.0;
static double s_zero_sq_sum_ch1 = 0.0;
static double s_zero_sq_sum_ch2 = 0.0;
static uint32_t s_zero_count = 0U;
static uint16_t s_zero_avg_mode = 0U;
static volatile uint8_t s_force_zero_input = 0U;
static float s_strain_coeff[2] = {1.0f, 1.0f};
static float s_torque_coeff = 0.0f;
static float s_thrust_coeff = 0.0f;
static float s_last_calib_factor[2] = {0.0f, 0.0f};
static float s_last_gauge_sens[2] = {0.0f, 0.0f};
static float s_last_shear_mod_gpa = 0.0f;
static float s_last_elastic_mod_gpa = 0.0f;
static float s_last_poisson_ratio = 0.0f;
static float s_last_outer_mm = 0.0f;
static float s_last_inner_mm = 0.0f;
static uint16_t s_last_prop_dir = 0xFFFFU;
static uint16_t s_last_modulus_select = 0xFFFFU;
static float s_modulus_g_gpa = 0.0f;
static float s_modulus_e_gpa = 0.0f;

static float AppCalc_RawToVoltage(uint32_t raw)
{
  float v;
  memcpy(&v, &raw, sizeof(v));
  if (isfinite(v))
  {
    return v;
  }
  return (float)raw * 0.000001f;
}

static float AppCalc_PolarInertiaM4(float outer_mm, float inner_mm)
{
  const float pi_over_32 = 0.0981747704f;
  float do_m = outer_mm * MM_TO_M;
  float di_m = inner_mm * MM_TO_M;
  float j;

  if (do_m <= 0.0f)
  {
    return 0.0f;
  }
  if (di_m < 0.0f)
  {
    di_m = 0.0f;
  }
  if (di_m >= do_m)
  {
    di_m = do_m * 0.999f;
  }

  j = pi_over_32 * (do_m * do_m * do_m * do_m - di_m * di_m * di_m * di_m);
  if (j < MIN_GEOM_EPS)
  {
    return 0.0f;
  }
  return j;
}

static void AppCalc_LogCoeffSnapshot(const protocol_values_t *v)
{
#if APP_CALC_LOG_COEFF_SNAPSHOT
  if (v == NULL)
  {
    return;
  }

  LOGI("[CALC-PARAM] outer=%.3f inner=%.3f G=%.3f E=%.3f poisson=%.4f prop=%u mod_sel=%u avg_mode=%u\r\n",
       (double)v->shaft_outer_d_mm,
       (double)v->shaft_inner_d_mm,
       (double)v->shear_modulus_GPa,
       (double)v->elastic_modulus_GPa,
       (double)v->poisson_ratio,
       (unsigned int)v->prop_dir,
       (unsigned int)v->modulus_select,
       (unsigned int)v->avg_mode);
  LOGI("[CALC-PARAM] calib1=%.6f calib2=%.6f sens1=%.6f sens2=%.6f zero1=%.6f zero2=%.6f coeffT=%.9f coeffF=%.9f\r\n",
       (double)v->calib_factor[0],
       (double)v->calib_factor[1],
       (double)v->gauge_sensitivity[0],
       (double)v->gauge_sensitivity[1],
       (double)v->zero_voltage[0],
       (double)v->zero_voltage[1],
       (double)s_torque_coeff,
       (double)s_thrust_coeff);
#else
  (void)v;
#endif
}

static void AppCalc_UpdateCoeffs(const protocol_values_t *v)
{
  float do_m;
  float di_m;
  float g_gpa;
  float g_pa;
  float e_gpa;
  float dir_sign;
  float d4;
  float d2;
  float denom;

  if (v == NULL)
  {
    return;
  }

  if ((v->calib_factor[0] == s_last_calib_factor[0]) &&
      (v->calib_factor[1] == s_last_calib_factor[1]) &&
      (v->gauge_sensitivity[0] == s_last_gauge_sens[0]) &&
      (v->gauge_sensitivity[1] == s_last_gauge_sens[1]) &&
      (v->shear_modulus_GPa == s_last_shear_mod_gpa) &&
      (v->elastic_modulus_GPa == s_last_elastic_mod_gpa) &&
      (v->poisson_ratio == s_last_poisson_ratio) &&
      (v->shaft_outer_d_mm == s_last_outer_mm) &&
      (v->shaft_inner_d_mm == s_last_inner_mm) &&
      (v->prop_dir == s_last_prop_dir) &&
      (v->modulus_select == s_last_modulus_select))
  {
    return;
  }

  s_last_calib_factor[0] = v->calib_factor[0];
  s_last_calib_factor[1] = v->calib_factor[1];
  s_last_gauge_sens[0] = v->gauge_sensitivity[0];
  s_last_gauge_sens[1] = v->gauge_sensitivity[1];
  s_last_shear_mod_gpa = v->shear_modulus_GPa;
  s_last_elastic_mod_gpa = v->elastic_modulus_GPa;
  s_last_poisson_ratio = v->poisson_ratio;
  s_last_outer_mm = v->shaft_outer_d_mm;
  s_last_inner_mm = v->shaft_inner_d_mm;
  s_last_prop_dir = v->prop_dir;
  s_last_modulus_select = v->modulus_select;

  s_strain_coeff[0] = ((v->calib_factor[0] != 0.0f) && (v->gauge_sensitivity[0] != 0.0f))
                        ? (1.0f / (v->calib_factor[0] * v->gauge_sensitivity[0])) : 0.0f; // 系数2：应变->剪切应变
  s_strain_coeff[1] = ((v->calib_factor[1] != 0.0f) && (v->gauge_sensitivity[1] != 0.0f))
                        ? (2.0f / (v->calib_factor[1] * v->gauge_sensitivity[1])) : 0.0f;

  denom = 2.0f * (1.0f + v->poisson_ratio);
  if ((v->modulus_select == 0U) && (v->shear_modulus_GPa > 0.0f))
  {
    g_gpa = v->shear_modulus_GPa;
    e_gpa = g_gpa * denom;
  }
  else if ((v->modulus_select == 1U) && (v->elastic_modulus_GPa > 0.0f))
  {
    e_gpa = v->elastic_modulus_GPa;
    if (denom != 0.0f)
    {
      g_gpa = e_gpa / denom;
    }
    else
    {
      g_gpa = 0.0f;
    }
  }
  else
  {
    g_gpa = 0.0f;
    e_gpa = 0.0f;
  }
  s_modulus_g_gpa = g_gpa;
  s_modulus_e_gpa = e_gpa;

  do_m = v->shaft_outer_d_mm * MM_TO_M;
  di_m = v->shaft_inner_d_mm * MM_TO_M;
  if ((do_m <= 0.0f) || (di_m < 0.0f))
  {
    s_torque_coeff = 0.0f;
    s_thrust_coeff = 0.0f;
    AppCalc_LogCoeffSnapshot(v);
    return;
  }
  if (di_m >= do_m)
  {
    di_m = do_m * 0.999f;
  }

  g_pa = g_gpa * GPA_TO_PA;
  d4 = do_m * do_m * do_m * do_m - di_m * di_m * di_m * di_m;
  d2 = do_m * do_m - di_m * di_m;
  if ((g_pa <= 0.0f) || (d4 <= MIN_GEOM_EPS))
  {
    s_torque_coeff = 0.0f;
  }
  else
  {
    dir_sign = (v->prop_dir == 0U) ? -1.0f : 1.0f;
    s_torque_coeff = dir_sign * 2.0f * g_pa * PI_F * d4 / do_m / 16.0f;
  }

  if ((s_modulus_e_gpa > 0.0f) && (d2 > MIN_GEOM_EPS))
  {
    s_thrust_coeff = s_modulus_e_gpa * GPA_TO_PA * PI_F * d2 / 4.0f;
  }
  else
  {
    s_thrust_coeff = 0.0f;
  }

  AppCalc_LogCoeffSnapshot(v);
}

void AppCalc_RecomputeCoeffs(const protocol_values_t *values)
{
  AppCalc_UpdateCoeffs(values);
}

void AppCalc_Init(const app_calc_ctx_t *ctx)
{
  (void)ctx;
  memset((void *)&s_calc_accum, 0, sizeof(s_calc_accum));
  s_zero_collecting = 0U;
  s_zero_sum_ch1 = 0.0;
  s_zero_sum_ch2 = 0.0;
  s_zero_sq_sum_ch1 = 0.0;
  s_zero_sq_sum_ch2 = 0.0;
  s_zero_count = 0U;
  s_zero_avg_mode = 0U;
  s_strain_coeff[0] = 1.0f;
  s_strain_coeff[1] = 1.0f;
  s_torque_coeff = 0.0f;
  s_thrust_coeff = 0.0f;
  s_last_calib_factor[0] = 0.0f;
  s_last_calib_factor[1] = 0.0f;
  s_last_gauge_sens[0] = 0.0f;
  s_last_gauge_sens[1] = 0.0f;
  s_last_shear_mod_gpa = 0.0f;
  s_last_elastic_mod_gpa = 0.0f;
  s_last_poisson_ratio = 0.0f;
  s_last_outer_mm = 0.0f;
  s_last_inner_mm = 0.0f;
  s_last_prop_dir = 0xFFFFU;
  s_last_modulus_select = 0xFFFFU;
  s_modulus_g_gpa = 0.0f;
  s_modulus_e_gpa = 0.0f;
}

void AppCalc_FeedSample(float ch1, float ch2)
{
  taskENTER_CRITICAL();
  s_calc_accum.ch1_sum += (double)ch1;
  s_calc_accum.ch2_sum += (double)ch2;
  s_calc_accum.ch1_sq_sum += (double)ch1 * (double)ch1;
  s_calc_accum.ch2_sq_sum += (double)ch2 * (double)ch2;
  s_calc_accum.sample_count++;

  if (s_zero_collecting != 0U)
  {
    s_zero_sum_ch1 += (double)ch1;
    s_zero_sum_ch2 += (double)ch2;
    s_zero_sq_sum_ch1 += (double)ch1 * (double)ch1;
    s_zero_sq_sum_ch2 += (double)ch2 * (double)ch2;
    s_zero_count++;
  }
  taskEXIT_CRITICAL();
}

void AppCalc_SetForceZeroInput(uint8_t enable)
{
  s_force_zero_input = (enable != 0U) ? 1U : 0U;
}

void AppCalc_FeedRpm(float rpm)
{
  s_calc_accum.rpm_sum += rpm;
  s_calc_accum.rpm_count++;
}

void AppCalc_ZeroCaptureStart(uint16_t avg_mode)
{
  taskENTER_CRITICAL();
  s_zero_collecting = 1U;
  s_zero_sum_ch1 = 0.0;
  s_zero_sum_ch2 = 0.0;
  s_zero_sq_sum_ch1 = 0.0;
  s_zero_sq_sum_ch2 = 0.0;
  s_zero_count = 0U;
  s_zero_avg_mode = avg_mode;
  taskEXIT_CRITICAL();
}

uint8_t AppCalc_ZeroCaptureStop(float *zero_ch1, float *zero_ch2)
{
  double sum1;
  double sum2;
  double sq_sum1;
  double sq_sum2;
  uint32_t count;
  uint16_t avg_mode;

  if ((zero_ch1 == NULL) || (zero_ch2 == NULL))
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  sum1 = s_zero_sum_ch1;
  sum2 = s_zero_sum_ch2;
  sq_sum1 = s_zero_sq_sum_ch1;
  sq_sum2 = s_zero_sq_sum_ch2;
  count = s_zero_count;
  avg_mode = s_zero_avg_mode;
  s_zero_collecting = 0U;
  taskEXIT_CRITICAL();

  if (count == 0U)
  {
    return 0U;
  }

  if (avg_mode == 1U)
  {
    *zero_ch1 = sqrtf((float)(sq_sum1 / (double)count));
    *zero_ch2 = sqrtf((float)(sq_sum2 / (double)count));
  }
  else
  {
    *zero_ch1 = (float)(sum1 / (double)count);
    *zero_ch2 = (float)(sum2 / (double)count);
  }
  return 1U;
}

void AppCalc_ZeroCaptureGetState(uint8_t *collecting, uint32_t *sample_count)
{
  taskENTER_CRITICAL();
  if (collecting != NULL)
  {
    *collecting = s_zero_collecting;
  }
  if (sample_count != NULL)
  {
    *sample_count = s_zero_count;
  }
  taskEXIT_CRITICAL();
}

void AppCalc_Compute1s(const app_calc_ctx_t *ctx, result_1s_t *out)
{
  calc_accum_t snap;
  protocol_values_t values_snap;
  float avg_ch1 = 0.0f;
  float avg_ch2 = 0.0f;
  float avg_rpm = 0.0f;
  float ch1_eff = 0.0f;
  float ch2_eff = 0.0f;
  float zero_voltage_ch1 = DEFAULT_ZERO_VOLTAGE;
  float zero_voltage_ch2 = DEFAULT_ZERO_VOLTAGE;
  float strain_ch1 = 0.0f;
  float strain_ch2 = 0.0f;
  float micro_strain_ch1 = 0.0f;
  float micro_strain_ch2 = 0.0f;
  float torque_knm_calc = 0.0f;
  float thrust_kn_calc = 0.0f;
  float power_kw_calc = 0.0f;
  uint8_t force_zero_input;

  if ((ctx == NULL) || (out == NULL))
  {
    return;
  }

  taskENTER_CRITICAL();
  snap = s_calc_accum;
  memset((void *)&s_calc_accum, 0, sizeof(s_calc_accum));
  taskEXIT_CRITICAL();

  if ((ctx->mtx_shared_data != NULL) && (ctx->protocol_values != NULL) &&
      (osMutexAcquire(ctx->mtx_shared_data, 2U) == osOK))
  {
    values_snap = *(const protocol_values_t *)ctx->protocol_values;
    osMutexRelease(ctx->mtx_shared_data);
  }
  else
  {
    memset(&values_snap, 0, sizeof(values_snap));
  }

  zero_voltage_ch1 = values_snap.zero_voltage[0];
  zero_voltage_ch2 = values_snap.zero_voltage[1];
  force_zero_input = s_force_zero_input;

  if ((force_zero_input != 0U) || (snap.sample_count == 0U))
  {
    avg_ch1 = zero_voltage_ch1;
    avg_ch2 = zero_voltage_ch2;
    ch1_eff = zero_voltage_ch1;
    ch2_eff = zero_voltage_ch2;
  }
  else
  {
    avg_ch1 = (float)(snap.ch1_sum / (double)snap.sample_count);
    avg_ch2 = (float)(snap.ch2_sum / (double)snap.sample_count);
    if (values_snap.avg_mode == 1U)
    {
      ch1_eff = sqrtf((float)(snap.ch1_sq_sum / (double)snap.sample_count));
      ch2_eff = sqrtf((float)(snap.ch2_sq_sum / (double)snap.sample_count));
    }
    else
    {
      ch1_eff = avg_ch1;
      ch2_eff = avg_ch2;
    }
  }
  if (snap.rpm_count > 0U)
  {
    avg_rpm = snap.rpm_sum / (float)snap.rpm_count;
  }

  strain_ch1 = ((ch1_eff - zero_voltage_ch1) * MV_TO_V) * s_strain_coeff[0];
  strain_ch2 = ((ch2_eff - zero_voltage_ch2) * MV_TO_V) * s_strain_coeff[1];
  micro_strain_ch1 = strain_ch1 * MICRO_STRAIN_SCALE;
  micro_strain_ch2 = strain_ch2 * MICRO_STRAIN_SCALE;
  torque_knm_calc = (strain_ch1 * s_torque_coeff) / 1000.0f;
  thrust_kn_calc = (strain_ch2 * s_thrust_coeff) / 1000.0f;
  power_kw_calc = torque_knm_calc * avg_rpm * OMEGA_PER_RPM;

  out->timestamp_ms = HAL_GetTick();
  out->torque = torque_knm_calc;
  out->thrust = thrust_kn_calc;
  out->rpm = avg_rpm;
  out->power = power_kw_calc;
  /* This bit-packed value is written directly as the SD CSV "status" column. */
  out->status_flags = (((uint32_t)values_snap.flag_avg_power & 0x1U) << 16) |
                      (((uint32_t)values_snap.flag_axis_param & 0x1U) << 17) |
                      (((uint32_t)values_snap.flag_meas_param & 0x1U) << 18) |
                      ((snap.sample_count & 0xFFU) << 8) |
                      (snap.rpm_count & 0xFFU);

  if (ctx->diag != NULL)
  {
    out->status_flags |= ((ctx->diag->uart2_req_fail & 0x0FU) << 24);
    out->status_flags |= ((ctx->diag->sd_write_fail & 0x0FU) << 28);
  }

#if APP_CALC_LOG_DETAIL
  LOGI("[CALC] samples=%lu rpm_cnt=%lu avg_ch1=%.6f avg_ch2=%.6f eff_ch1=%.6f eff_ch2=%.6f\r\n",
       (unsigned long)snap.sample_count,
       (unsigned long)snap.rpm_count,
       (double)avg_ch1,
       (double)avg_ch2,
       (double)ch1_eff,
       (double)ch2_eff);
  LOGI("[CALC] zero1=%.6f zero2=%.6f strain1=%.9f strain2=%.9f micro1=%.3f micro2=%.3f\r\n",
       (double)zero_voltage_ch1,
       (double)zero_voltage_ch2,
       (double)strain_ch1,
       (double)strain_ch2,
       (double)micro_strain_ch1,
       (double)micro_strain_ch2);
  LOGI("[CALC] coeff torque=%.9f thrust=%.9f strain_coeff1=%.9f strain_coeff2=%.9f\r\n",
       (double)s_torque_coeff,
       (double)s_thrust_coeff,
       (double)s_strain_coeff[0],
       (double)s_strain_coeff[1]);
  LOGI("[CALC] result torque=%.6f thrust=%.6f rpm=%.3f power=%.6f status=0x%08lX\r\n",
       (double)out->torque,
       (double)out->thrust,
       (double)out->rpm,
       (double)out->power,
       (unsigned long)out->status_flags);
#endif
}
