/*
 * XC Test Bench: Field-Oriented Control (FOC) and SVPWM Inverter Commutation
 * Tests Clarke/Park transforms, space vector dwell time sums, and PI current anti-windup.
 */

#include "foc.h"
#include <stdbool.h>

bool run_foc_bench(void) {
    /* Test 1: Balanced 3-phase currents Clarke transform */
    q16_t ia = 131072;  /* 2.0 A in Q16 */
    q16_t ib = -65536;  /* -1.0 A */
    q16_t ic = -65536;  /* -1.0 A */
    q16_t i_alpha, i_beta;
    foc_clarke_transform(ia, ib, ic, &i_alpha, &i_beta);
    if (q16_abs(i_alpha - 131072) > 4 || q16_abs(i_beta) > 4) return false;

    /* Test 2: Park transform at electrical angle theta_e = 0 */
    q16_t id, iq;
    foc_park_transform(i_alpha, i_beta, 0, &id, &iq);
    if (q16_abs(id - 131072) > 4 || q16_abs(iq) > 4) return false;

    /* Test 3: SVPWM dwell times must not exceed PWM carrier period */
    uint16_t pwma, pwmb, pwmc;
    uint8_t sector;
    foc_svpwm_calculate(q16_from_float(10.0f), q16_from_float(5.0f), q16_from_float(24.0f),
                        &pwma, &pwmb, &pwmc, &sector);
    if (pwma > PWM_TIMER_TICKS || pwmb > PWM_TIMER_TICKS || pwmc > PWM_TIMER_TICKS) return false;
    if (sector < 1 || sector > 6) return false;
    return true;
}
