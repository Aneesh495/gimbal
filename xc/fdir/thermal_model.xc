/*
 * Gimbal Motor Stator Winding and Power Inverter Thermal RC Model
 * Estimates junction temperatures from I^2 * R ohmic dissipation to throttle current limits.
 */

#include "fdir.h"
#include "fixed_math.h"

void thermal_model_init(thermal_model_t *tm) {
    if (!tm) return;
    tm->ambient_temp_c = 25 * FIXED_Q16_ONE;
    tm->winding_temp_c = tm->ambient_temp_c;
    tm->fet_temp_c = tm->ambient_temp_c;
    tm->r_th_winding = q16_from_float(3.5f);  /* deg C / Watt */
    tm->c_th_winding = q16_from_float(12.0f); /* Joules / deg C */
    tm->max_winding_temp_c = 110 * FIXED_Q16_ONE;
}

void thermal_model_update(thermal_model_t *tm, q16_t phase_current_amps, q16_t dt_sec) {
    if (!tm) return;
    
    /* Ohmic heating: P = I^2 * R (R_phase = 1.2 ohms) */
    int64_t i_sq = ((int64_t)phase_current_amps * phase_current_amps) >> FIXED_Q16_SHIFT;
    q16_t power_watts = (q16_t)((i_sq * q16_from_float(1.2f)) >> FIXED_Q16_SHIFT);
    
    /* Heat loss: P_loss = (T_winding - T_ambient) / R_th */
    q16_t temp_diff = tm->winding_temp_c - tm->ambient_temp_c;
    q16_t p_loss = q16_div(temp_diff, tm->r_th_winding);
    
    /* Net heat flow: dT/dt = (P_heat - P_loss) / C_th */
    q16_t p_net = power_watts - p_loss;
    q16_t dt_temp = q16_mul(q16_div(p_net, tm->c_th_winding), dt_sec);
    
    tm->winding_temp_c = q16_add(tm->winding_temp_c, dt_temp);
}
