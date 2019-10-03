#ifndef SIM_SCENARIO_RUNNER_H
#define SIM_SCENARIO_RUNNER_H

#include "gimbal_physics.h"
#include "sensor_models.h"

typedef enum {
    SCENARIO_STEP_RESPONSE = 0,
    SCENARIO_CARRIER_VIBRATION = 1,
    SCENARIO_TARGET_TRACKING = 2,
    SCENARIO_RAPID_SLEW = 3
} scenario_type_t;

typedef struct {
    double duration_sec;
    double max_tracking_error_rad;
    double rms_jitter_urad;
    double settling_time_sec;
    double disturbance_rejection_db;
    bool passed;
} scenario_result_t;

scenario_result_t scenario_run(scenario_type_t scenario);

#endif
