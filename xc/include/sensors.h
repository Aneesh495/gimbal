#ifndef GIMBAL_SENSORS_H
#define GIMBAL_SENSORS_H

#include "gimbal_types.h"
#include "vector3.h"

#define IMU_SPI_FREQ_HZ     10000000
#define IMU_FIFO_DEPTH      32
#define NOTCH_STAGES_PER_AXIS 4

typedef struct {
    q31_t b0, b1, b2;
    q31_t a1, a2;
    q31_t s1, s2; /* State delay registers */
} biquad_df2t_t;

typedef struct {
    biquad_df2t_t stages[NOTCH_STAGES_PER_AXIS];
    q16_t center_freq_hz[NOTCH_STAGES_PER_AXIS];
    q16_t bandwidth_hz[NOTCH_STAGES_PER_AXIS];
    bool enabled[NOTCH_STAGES_PER_AXIS];
} notch_bank_axis_t;

typedef struct {
    mat3_q16_t accel_misalignment;
    mat3_q16_t gyro_misalignment;
    vec3_q16_t accel_scale;
    vec3_q16_t gyro_scale;
    vec3_q16_t accel_bias_t0;
    vec3_q16_t gyro_bias_t0;
    vec3_q16_t accel_temp_coeff;
    vec3_q16_t gyro_temp_coeff;
    q16_t reference_temp_c;
} sensor_cal_params_t;

typedef struct {
    uint32_t raw_angle[GIMBAL_AXIS_COUNT];
    q16_t mechanical_angle[GIMBAL_AXIS_COUNT];
    q16_t electrical_angle[GIMBAL_AXIS_COUNT];
    q16_t velocity_filtered[GIMBAL_AXIS_COUNT];
    uint8_t pole_pairs[GIMBAL_AXIS_COUNT];
    bool crc_ok[GIMBAL_AXIS_COUNT];
} absolute_encoder_state_t;

void imu_spi_init(uint8_t imu_index);
bool imu_spi_read_burst(uint8_t imu_index, imu_sample_t *sample_out);

void biss_encoder_init(void);
bool biss_encoder_read_axis(uint8_t axis, uint32_t *raw_counts, bool *error_bit);

void quad_encoder_init(void);
void quad_encoder_update(encoder_state_t *enc);

void biquad_init_notch(biquad_df2t_t *filter, q16_t f0_hz, q16_t bw_hz, q16_t fs_hz);
void biquad_init_lowpass(biquad_df2t_t *filter, q16_t fc_hz, q16_t q_factor, q16_t fs_hz);
q16_t biquad_process_sample(biquad_df2t_t *filter, q16_t input);
void biquad_reset(biquad_df2t_t *filter);

void notch_bank_init(notch_bank_axis_t *bank, q16_t fs_hz);
q16_t notch_bank_process(notch_bank_axis_t *bank, q16_t input);

void sensor_cal_init_defaults(sensor_cal_params_t *params);
void sensor_cal_apply(const sensor_cal_params_t *params, const imu_sample_t *raw, imu_sample_t *cal_out);

void sensor_manager_init(void);
void sensor_manager_update(const imu_sample_t imu_samples[2], const absolute_encoder_state_t *encoders, conditioned_sensor_msg_t *conditioned_out);

#endif
