#ifndef __PROTOCOL_VALUES_H
#define __PROTOCOL_VALUES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  float torque_kNm;      /* 0x07D0 */
  float power_kW;        /* 0x07D2 */
  float speed_rpm;       /* 0x07D4 */
  float thrust_kN;       /* 0x07D6 */
  uint16_t pc2_low_voltage_state; /* 0x07D8 */
  uint16_t board_low_voltage_state; /* 0x07D9 */
  float avg_power_kW;    /* 0x07DA */
  uint16_t wireless_frame_no; /* 0x07DC */
  float query_time_h;    /* 0x09C4 */

  float shaft_outer_d_mm;    /* 0x09C6 */
  float shaft_inner_d_mm;    /* 0x09C8 */
  float shear_modulus_GPa;   /* 0x09CA */
  float elastic_modulus_GPa; /* 0x09CC */
  float poisson_ratio;       /* 0x09CE */
  uint16_t prop_dir;         /* 0x09D0 */
  uint16_t modulus_select;   /* 0x09D1 */

  float sample_freq_Hz;      /* 0x09D2 */
  int32_t avg_time_ms;       /* 0x09D4 */
  uint16_t avg_mode;         /* 0x09D6 */
  float calib_factor[2];     /* 0x09D7, 0x09D9 */
  float gauge_sensitivity[2];/* 0x09DB, 0x09DD */
  float zero_voltage[2];     /* 0x09DF, 0x09E1 */
  uint16_t zero_start;       /* 0x09E3 */
  uint16_t zero_end;         /* 0x09E4 */

  uint16_t flag_avg_power;   /* 0x0AF0 */
  uint16_t flag_axis_param;  /* 0x0AF1 */
  uint16_t flag_meas_param;  /* 0x0AF2 */
  uint16_t flag_uart_enable; /* 0x0AF3 */
} protocol_values_t;

#ifdef __cplusplus
}
#endif

#endif
