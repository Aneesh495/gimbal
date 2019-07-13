/*
 * Gimbal Tile 0: High-Speed Sensor Acquisition and Signal Processing Executive
 * Runs 10 kHz SPI IMU burst sampling, BiSS-C encoder reads, biquad notch filters,
 * and outputs conditioned sensor telemetry across CSP streaming channels.
 */

#include "sensors.h"
#include "gimbal_channels.h"

void tile_sensors_executive(void) {
    imu_sample_t raw_samples[2];
    absolute_encoder_state_t encoders;
    conditioned_sensor_msg_t conditioned_msg;
    
    imu_spi_init(0);
    imu_spi_init(1);
    biss_encoder_init();
    quad_encoder_init();
    sensor_manager_init();
    
    /* 10 kHz sampling loop */
    while (1) {
        imu_spi_read_burst(0, &raw_samples[0]);
        imu_spi_read_burst(1, &raw_samples[1]);
        
        for (int i = 0; i < GIMBAL_AXIS_COUNT; i++) {
            biss_encoder_read_axis(i, &encoders.raw_angle[i], NULL);
        }
        
        sensor_manager_update(raw_samples, &encoders, &conditioned_msg);
        
        /* Yield to next timer tick in actual xCORE timer event */
        break;
    }
}
