#ifndef SIM_SENSOR_MODELS_H
#define SIM_SENSOR_MODELS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    double gyro_bias[3];
    double accel_bias[3];
    double gyro_noise_std;
    double accel_noise_std;
    double encoder_quantization_rad;
    uint32_t seed;
} sensor_synthesizer_t;

void sensor_synth_init(sensor_synthesizer_t *synth);
void sensor_synth_sample(sensor_synthesizer_t *synth, const double true_rates[3], const double true_angles[3],
                         double measured_rates[3], double measured_accel[3], double measured_angles[3]);

#endif
