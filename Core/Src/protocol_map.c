#include "protocol_map.h"
#include "protocol_values.h"
#include <stddef.h>

#define OFF(x) ((uint16_t)offsetof(protocol_values_t, x))

const reg_meta_t g_reg_meta_table[] = {
  {0x07D0U, "torque_kNm", REG_TYPE_FLOAT, 4U, 1U, OFF(torque_kNm)},
  {0x07D2U, "power_kW", REG_TYPE_FLOAT, 4U, 1U, OFF(power_kW)},
  {0x07D4U, "speed_rpm", REG_TYPE_FLOAT, 4U, 2U, OFF(speed_rpm)},
  {0x07D6U, "thrust_kN", REG_TYPE_FLOAT, 4U, 2U, OFF(thrust_kN)},
  {0x07D8U, "pc2_low_voltage_state", REG_TYPE_SHORT, 2U, 0U, OFF(pc2_low_voltage_state)},
  {0x07D9U, "board_low_voltage_state", REG_TYPE_SHORT, 2U, 0U, OFF(board_low_voltage_state)},
  {0x07DAU, "avg_power_kW", REG_TYPE_FLOAT, 4U, 1U, OFF(avg_power_kW)},
  {0x07DCU, "wireless_frame_no", REG_TYPE_SHORT, 2U, 0U, OFF(wireless_frame_no)},
  {0x09C4U, "query_time_h", REG_TYPE_FLOAT, 4U, 1U, OFF(query_time_h)},

  {0x09C6U, "shaft_outer_d_mm", REG_TYPE_FLOAT, 4U, 2U, OFF(shaft_outer_d_mm)},
  {0x09C8U, "shaft_inner_d_mm", REG_TYPE_FLOAT, 4U, 2U, OFF(shaft_inner_d_mm)},
  {0x09CAU, "shear_modulus_GPa", REG_TYPE_FLOAT, 4U, 2U, OFF(shear_modulus_GPa)},
  {0x09CCU, "elastic_modulus_GPa", REG_TYPE_FLOAT, 4U, 2U, OFF(elastic_modulus_GPa)},
  {0x09CEU, "poisson_ratio", REG_TYPE_FLOAT, 4U, 4U, OFF(poisson_ratio)},
  {0x09D0U, "prop_dir", REG_TYPE_SHORT, 2U, 0U, OFF(prop_dir)},
  {0x09D1U, "modulus_select", REG_TYPE_SHORT, 2U, 0U, OFF(modulus_select)},

  {0x09D2U, "sample_freq_Hz", REG_TYPE_FLOAT, 4U, 2U, OFF(sample_freq_Hz)},
  {0x09D4U, "avg_time_ms", REG_TYPE_INT32, 4U, 0U, OFF(avg_time_ms)},
  {0x09D6U, "avg_mode", REG_TYPE_SHORT, 2U, 0U, OFF(avg_mode)},
  {0x09D7U, "calib_factor_ch1", REG_TYPE_FLOAT, 4U, 4U, OFF(calib_factor[0])},
  {0x09D9U, "calib_factor_ch2", REG_TYPE_FLOAT, 4U, 4U, OFF(calib_factor[1])},
  {0x09DBU, "gauge_sensitivity_ch1", REG_TYPE_FLOAT, 4U, 3U, OFF(gauge_sensitivity[0])},
  {0x09DDU, "gauge_sensitivity_ch2", REG_TYPE_FLOAT, 4U, 3U, OFF(gauge_sensitivity[1])},
  {0x09DFU, "zero_voltage_ch1", REG_TYPE_FLOAT, 4U, 3U, OFF(zero_voltage[0])},
  {0x09E1U, "zero_voltage_ch2", REG_TYPE_FLOAT, 4U, 3U, OFF(zero_voltage[1])},
  {0x09E3U, "zero_start", REG_TYPE_SHORT, 2U, 0U, OFF(zero_start)},
  {0x09E4U, "zero_end", REG_TYPE_SHORT, 2U, 0U, OFF(zero_end)},

  {0x0AF0U, "flag_avg_power", REG_TYPE_SHORT, 2U, 0U, OFF(flag_avg_power)},
  {0x0AF1U, "flag_axis_param", REG_TYPE_SHORT, 2U, 0U, OFF(flag_axis_param)},
  {0x0AF2U, "flag_meas_param", REG_TYPE_SHORT, 2U, 0U, OFF(flag_meas_param)},
  {0x0AF3U, "flag_uart_enable", REG_TYPE_SHORT, 2U, 0U, OFF(flag_uart_enable)},
};

const uint32_t g_reg_meta_count = sizeof(g_reg_meta_table) / sizeof(g_reg_meta_table[0]);

const reg_meta_t *ProtocolMap_Find(uint16_t addr)
{
  uint32_t i;
  for (i = 0U; i < g_reg_meta_count; i++)
  {
    if (g_reg_meta_table[i].addr == addr)
    {
      return &g_reg_meta_table[i];
    }
  }
  return 0;
}
