/*
 * Gimbal Standalone SITL High-Speed Simulation Binary
 * Runs all mission scenarios at > 100x real-time speed and exports performance metrics.
 */

#include <stdio.h>
#include "scenario_runner.h"

int main(int argc, char **argv) {
    printf("==============================================================
");
    printf("  Gimbal Deterministic Multi-Core Real-Time SITL Simulator    
");
    printf("==============================================================
");
    
    const char *names[4] = {
        "10-Degree Step Response",
        "Carrier Vibration Disturbance Rejection",
        "Optical Target Centroid Tracking",
        "Rapid Multi-Target S-Curve Slew"
    };
    
    bool all_ok = true;
    for (int i = 0; i < 4; i++) {
        printf("Running Scenario [%d/4]: %s ... ", i + 1, names[i]);
        scenario_result_t res = scenario_run((scenario_type_t)i);
        if (res.passed) {
            printf("PASSED
");
            printf("   Settling Time: %.3f s | RMS Jitter: %.1f urad | Rejection: %.1f dB
",
                   res.settling_time_sec, res.rms_jitter_urad, res.disturbance_rejection_db);
        } else {
            printf("FAILED
");
            all_ok = false;
        }
    }
    
    printf("--------------------------------------------------------------
");
    if (all_ok) {
        printf("ALL SIMULATION SCENARIOS PASSED WITH ZERO TOLERANCE BREACHES.
");
        return 0;
    } else {
        printf("SIMULATION FAILED.
");
        return 1;
    }
}
