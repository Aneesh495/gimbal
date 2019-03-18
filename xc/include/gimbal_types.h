#ifndef GIMBAL_TYPES_H
#define GIMBAL_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define GIMBAL_AXIS_PAN   0
#define GIMBAL_AXIS_TILT  1
#define GIMBAL_AXIS_ROLL  2
#define GIMBAL_AXIS_COUNT 3

#define FIXED_Q15_SHIFT 15
#define FIXED_Q15_ONE   (1 << FIXED_Q15_SHIFT)
#define FIXED_Q15_HALF  (1 << (FIXED_Q15_SHIFT - 1))

#define FIXED_Q16_SHIFT 16
#define FIXED_Q16_ONE   (1 << FIXED_Q16_SHIFT)
#define FIXED_Q16_HALF  (1 << (FIXED_Q16_SHIFT - 1))

#define FIXED_Q31_SHIFT 31
#define FIXED_Q31_ONE   0x7FFFFFFFLL

typedef int16_t q15_t;
typedef int32_t q16_t;
typedef int32_t q31_t;

typedef enum {
    GIMBAL_MODE_OFF         = 0,
    GIMBAL_MODE_CALIBRATE   = 1,
    GIMBAL_MODE_RATE_HOLD   = 2,
    GIMBAL_MODE_ATT_HOLD    = 3,
    GIMBAL_MODE_TARGET_TRACK= 4,
    GIMBAL_MODE_GEO_POINT   = 5,
    GIMBAL_MODE_RETRACT     = 6,
    GIMBAL_MODE_FAULT       = 7
} gimbal_mode_t;

typedef enum {
    GIMBAL_STATUS_OK           = 0x00,
    GIMBAL_STATUS_IMU_FAULT    = 0x01,
    GIMBAL_STATUS_ENC_FAULT    = 0x02,
    GIMBAL_STATUS_OVERCURRENT  = 0x04,
    GIMBAL_STATUS_OVERTEMP     = 0x08,
    GIMBAL_STATUS_LIMIT_HIT    = 0x10,
    GIMBAL_STATUS_COMMS_LOST   = 0x20,
    GIMBAL_STATUS_WATCHDOG_TRIP= 0x40
} gimbal_status_flags_t;

typedef struct {
    q16_t x;
    q16_t y;
    q16_t z;
} vec3_q16_t;

typedef struct {
    q16_t m[3][3];
} mat3_q16_t;

typedef struct {
    q16_t w;
    q16_t x;
    q16_t y;
    q16_t z;
} quat_q16_t;

typedef struct {
    vec3_q16_t accel_raw;
    vec3_q16_t gyro_raw;
    vec3_q16_t accel_cal;
    vec3_q16_t gyro_cal;
    q16_t temperature;
    uint32_t timestamp_ticks;
    uint8_t sensor_id;
    bool valid;
} imu_sample_t;

typedef struct {
    uint32_t raw_counts[GIMBAL_AXIS_COUNT];
    q16_t angle_rad[GIMBAL_AXIS_COUNT];
    q16_t velocity_rad_s[GIMBAL_AXIS_COUNT];
    bool aligned[GIMBAL_AXIS_COUNT];
    bool fault[GIMBAL_AXIS_COUNT];
} encoder_state_t;

typedef struct {
    q16_t i_alpha;
    q16_t i_beta;
    q16_t i_d;
    q16_t i_q;
    q16_t v_d;
    q16_t v_q;
    q16_t v_alpha;
    q16_t v_beta;
    uint16_t pwm_a;
    uint16_t pwm_b;
    uint16_t pwm_c;
    uint8_t sector;
} foc_state_t;

typedef struct {
    quat_q16_t q_inertial_to_body;
    vec3_q16_t rate_body_rad_s;
    vec3_q16_t gyro_bias_rad_s;
    vec3_q16_t accel_bias_m_s2;
    vec3_q16_t disturbance_torque_nm;
    q16_t covariance_diag[12];
    uint32_t last_update_tick;
    bool converged;
} nav_state_t;

typedef struct {
    gimbal_mode_t mode;
    vec3_q16_t target_rate_cmd;
    vec3_q16_t target_angle_cmd;
    vec3_q16_t geo_lat_lon_alt;
    uint32_t cmd_sequence;
} gimbal_cmd_t;

#endif
