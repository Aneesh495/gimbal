/*
 * Integration Test Suite: 10,000-Cycle Closed-Loop Gimbal Stabilization
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "scenario_runner.h"

int main(void) {
    printf("Running test_closed_loop (10,000 cycles)... ");
    
    scenario_result_t res = scenario_run(SCENARIO_CARRIER_VIBRATION);
    assert(res.passed);
    assert(res.disturbance_rejection_db > 60.0);
    assert(res.rms_jitter_urad < 400.0);
    
    printf("PASSED (Jitter: %.1f urad, Rejection: %.1f dB)\n", res.rms_jitter_urad, res.disturbance_rejection_db);
    return 0;
}
