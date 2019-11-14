#include <stdlib.h>
/*
 * Unit Test Suite: Field-Oriented Control (FOC) and Space Vector PWM
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "foc.h"

int main(void) {
    printf("Running test_foc... ");
    
    /* Clarke transform */
    q16_t ia = q16_from_float(2.0f);
    q16_t ib = q16_from_float(-1.0f);
    q16_t ic = q16_from_float(-1.0f);
    q16_t ialpha, ibeta;
    foc_clarke_transform(ia, ib, ic, &ialpha, &ibeta);
    assert(abs(ialpha - ia) < 4 && abs(ibeta) < 4);
    
    /* Park transform */
    q16_t id, iq;
    foc_park_transform(ialpha, ibeta, 0, &id, &iq);
    assert(abs(id - ia) < 8 && abs(iq) < 8);
    
    /* Inverse Park */
    q16_t valpha, vbeta;
    foc_inv_park_transform(id, iq, 0, &valpha, &vbeta);
    assert(abs(valpha - ialpha) < 8 && abs(vbeta - ibeta) < 8);
    
    /* SVPWM sector verification */
    uint16_t pa, pb, pc;
    uint8_t sec;
    foc_svpwm_calculate(q16_from_float(5.0f), q16_from_float(2.0f), q16_from_float(24.0f), &pa, &pb, &pc, &sec);
    assert(sec >= 1 && sec <= 6);
    assert(pa <= PWM_TIMER_TICKS && pb <= PWM_TIMER_TICKS && pc <= PWM_TIMER_TICKS);
    
    printf("PASSED\n");
    return 0;
}
