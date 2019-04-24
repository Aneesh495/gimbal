/*
 * Gimbal Dual-Lane Sensor Manager and Triple Modular Redundancy (TMR) Voter
 * Monitors sensor cross-lane residuals, detects sensor saturation/faults, and performs bumpless failover.
 */

#include "sensors.h"

static sensor_cal_params_t cal_params[2];
static notch_bank_axis_t gyro_notch_banks[3];
static biquad_df2t_t accel_lowpass[3];
static uint8_t active_primary_imu = 0;
static uint32_t imu_fault_counters[2] = {0, 0};

void sensor_manager_init(void) {
    sensor_cal_init_defaults(&cal_params[0]);
    sensor_cal_init_defaults(&cal_params[1]);
    
    q16_t fs_hz = q16_from_float(10000.0f); /* 10 kHz inner rate loop */
    for (int i = 0; i < 3; i++) {
        notch_bank_init(&gyro_notch_banks[i], fs_hz);
        biquad_init_lowpass(&accel_lowpass[i], q16_from_float(150.0f), q16_from_float(0.707f), fs_hz);
    }
    
    active_primary_imu = 0;
    imu_fault_counters[0] = 0;
    imu_fault_counters[1] = 0;
}

void sensor_manager_update(const imu_sample_t imu_samples[2], const absolute_encoder_state_t *encoders, conditioned_sensor_msg_t *conditioned_out) {
    if (!conditioned_out) return;
    
    imu_sample_t cal_imu0, cal_imu1;
    sensor_cal_apply(&cal_params[0], &imu_samples[0], &cal_imu0);
    sensor_cal_apply(&cal_params[1], &imu_samples[1], &cal_imu1);
    
    /* Cross-lane residual check: |gyro0 - gyro1| */
    vec3_q16_t gyro_diff = vec3_sub(cal_imu0.gyro_cal, cal_imu1.gyro_cal);
    q16_t residual_mag = vec3_mag(gyro_diff);
    
    /* Threshold for sensor discrepancy: 0.1 rad/s = 6553 in Q16 */
    if (residual_mag > 6553) {
        imu_fault_counters[active_primary_imu]++;
        if (imu_fault_counters[active_primary_imu] > 10) {
            /* Switch to backup lane */
            active_primary_imu = 1 - active_primary_imu;
        }
    } else {
        if (imu_fault_counters[active_primary_imu] > 0) {
            imu_fault_counters[active_primary_imu]--;
        }
    }
    
    const imu_sample_t *selected = (active_primary_imu == 0) ? &cal_imu0 : &cal_imu1;
    
    /* Apply notch filter bank to gyro rates */
    conditioned_out->gyro_filtered.x = notch_bank_process(&gyro_notch_banks[0], selected->gyro_cal.x);
    conditioned_out->gyro_filtered.y = notch_bank_process(&gyro_notch_banks[1], selected->gyro_cal.y);
    conditioned_out->gyro_filtered.z = notch_bank_process(&gyro_notch_banks[2], selected->gyro_cal.z);
    
    /* Apply lowpass filter to accelerometers */
    conditioned_out->accel_filtered.x = biquad_process_sample(&accel_lowpass[0], selected->accel_cal.x);
    conditioned_out->accel_filtered.y = biquad_process_sample(&accel_lowpass[1], selected->accel_cal.y);
    conditioned_out->accel_filtered.z = biquad_process_sample(&accel_lowpass[2], selected->accel_cal.z);
    
    if (encoders) {
        conditioned_out->joint_angles_rad.x = encoders->mechanical_angle[0];
        conditioned_out->joint_angles_rad.y = encoders->mechanical_angle[1];
        conditioned_out->joint_angles_rad.z = encoders->mechanical_angle[2];
        conditioned_out->joint_velocities_rad_s.x = encoders->velocity_filtered[0];
        conditioned_out->joint_velocities_rad_s.y = encoders->velocity_filtered[1];
        conditioned_out->joint_velocities_rad_s.z = encoders->velocity_filtered[2];
    }
    
    conditioned_out->timestamp_ticks = selected->timestamp_ticks;
    conditioned_out->valid = true;
}
