#ifndef GIMBAL_CHANNELS_H
#define GIMBAL_CHANNELS_H

#include "gimbal_types.h"

/*
 * xCORE Communicating Sequential Processes (CSP) Channel Topology
 * Defines inter-tile and inter-thread synchronous and streaming channels.
 */

#define TILE_SENSORS   0
#define TILE_FOC       1
#define TILE_ESTIMATOR 2
#define TILE_TRACKING  3

typedef struct {
    imu_sample_t imu_data[2];
    encoder_state_t encoders;
    uint32_t sample_tick;
} raw_sensor_msg_t;

typedef struct {
    vec3_q16_t accel_filtered;
    vec3_q16_t gyro_filtered;
    vec3_q16_t joint_angles_rad;
    vec3_q16_t joint_velocities_rad_s;
    uint32_t timestamp_ticks;
    bool valid;
} conditioned_sensor_msg_t;

typedef struct {
    q16_t target_torque_nm[GIMBAL_AXIS_COUNT];
    q16_t feedforward_voltage_v[GIMBAL_AXIS_COUNT];
    bool enable_drive[GIMBAL_AXIS_COUNT];
    uint32_t command_tick;
} foc_command_msg_t;

typedef struct {
    q16_t current_d_amps[GIMBAL_AXIS_COUNT];
    q16_t current_q_amps[GIMBAL_AXIS_COUNT];
    q16_t temperature_c[GIMBAL_AXIS_COUNT];
    uint16_t fault_bits[GIMBAL_AXIS_COUNT];
    uint32_t telemetry_tick;
} foc_telemetry_msg_t;

typedef struct {
    quat_q16_t attitude_quaternion;
    vec3_q16_t angular_rate_body_rad_s;
    vec3_q16_t linear_accel_body_m_s2;
    vec3_q16_t estimated_disturbance_nm;
    uint32_t estimate_tick;
    bool valid;
} state_estimate_msg_t;

typedef struct {
    vec3_q16_t desired_los_rate_rad_s;
    vec3_q16_t base_feedforward_rate_rad_s;
    gimbal_mode_t active_mode;
    uint32_t guidance_tick;
} guidance_command_msg_t;

typedef struct {
    uint8_t payload[256];
    uint16_t length;
    uint16_t message_id;
    uint32_t sequence;
} comms_packet_msg_t;

#endif
