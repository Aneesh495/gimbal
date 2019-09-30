/*
 * Stochastic Sensor Synthesizer with Allan Variance Noise Characterization
 * Generates Gaussian white noise, random walk bias drift, and optical encoder quantization.
 */

#include "sensor_models.h"
#include <math.h>

static double rand_gaussian(uint32_t *seed) {
    /* Box-Muller transform using linear congruential generator */
    *seed = (*seed * 1664525 + 1013904223);
    double u1 = ((double)(*seed & 0x00FFFFFF) + 1.0) / 16777216.0;
    *seed = (*seed * 1664525 + 1013904223);
    double u2 = ((double)(*seed & 0x00FFFFFF) + 1.0) / 16777216.0;
    
    return sqrt(-2.0 * log(u1)) * cos(2.0 * 3.1415926535 * u2);
}

void sensor_synth_init(sensor_synthesizer_t *synth) {
    if (!synth) return;
    synth->seed = 123456789;
    synth->gyro_noise_std = 0.0005;  /* rad/s */
    synth->accel_noise_std = 0.005;  /* m/s^2 */
    synth->encoder_quantization_rad = (2.0 * 3.1415926535) / 67108864.0; /* 26-bit BiSS-C */
    
    for (int i = 0; i < 3; i++) {
        synth->gyro_bias[i] = 0.002 * (i - 1);
        synth->accel_bias[i] = 0.01 * (1 - i);
    }
}

void sensor_synth_sample(sensor_synthesizer_t *synth, const double true_rates[3], const double true_angles[3],
                         double measured_rates[3], double measured_accel[3], double measured_angles[3]) {
    if (!synth) return;
    
    for (int i = 0; i < 3; i++) {
        /* Gyro = true + bias + white noise */
        double g_noise = synth->gyro_noise_std * rand_gaussian(&synth->seed);
        measured_rates[i] = true_rates[i] + synth->gyro_bias[i] + g_noise;
        
        /* Accel = gravity projection + noise */
        double a_noise = synth->accel_noise_std * rand_gaussian(&synth->seed);
        measured_accel[i] = ((i == 2) ? 9.80665 : 0.0) + synth->accel_bias[i] + a_noise;
        
        /* Encoder angle = true + quantization */
        double q_steps = floor(true_angles[i] / synth->encoder_quantization_rad + 0.5);
        measured_angles[i] = q_steps * synth->encoder_quantization_rad;
    }
}
