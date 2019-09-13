/*
 * Gimbal Factory Production Calibration and Allan Variance Noise Parameters
 * Calibration datasets, sensor noise densities, and scale factors for 64 serialized flight units.
 */

#include "sensors.h"

#define FACTORY_SERIALIZED_UNITS 64

typedef struct {
    uint32_t serial_number;
    sensor_cal_params_t imu0_cal;
    sensor_cal_params_t imu1_cal;
    q16_t gyro_noise_density_rad_s_rthz;
    q16_t gyro_bias_instability_rad_s;
    q16_t accel_noise_density_m_s2_rthz;
    q16_t encoder_eccentricity_error_rad[GIMBAL_AXIS_COUNT];
    uint32_t calibration_timestamp_epoch;
} factory_calibration_record_t;

const factory_calibration_record_t g_factory_calibration_records[FACTORY_SERIALIZED_UNITS] = {
    /* Unit Serial: 1000100 */
    {
        .serial_number = 1000100,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 0, 0}, {0, 65536, 0}, {0, 0, 65536}}},
            .gyro_misalignment = {.m = {{65536, 0, 0}, {0, 65536, 0}, {0, 0, 65536}}},
            .accel_scale = {65536, 65536, 65536},
            .gyro_scale = {65536, 65536, 65536},
            .accel_bias_t0 = {0, 0, 0},
            .gyro_bias_t0 = {0, 0, 0},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, 0, 0}, {0, 65536, 0}, {0, 0, 65536}}},
            .gyro_misalignment = {.m = {{65536, 0, 0}, {0, 65536, 0}, {0, 0, 65536}}},
            .accel_scale = {65536, 65536, 65536},
            .gyro_scale = {65536, 65536, 65536},
            .accel_bias_t0 = {0, 0, 0},
            .gyro_bias_t0 = {0, 0, 0},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {0, 0, 0},
        .calibration_timestamp_epoch = 1731770000
    },
    /* Unit Serial: 1000101 */
    {
        .serial_number = 1000101,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 2, -3}, {-2, 65536, 1}, {3, -1, 65536}}},
            .gyro_misalignment = {.m = {{65536, -1, 2}, {1, 65536, -2}, {-2, 2, 65536}}},
            .accel_scale = {65537, 65535, 65538},
            .gyro_scale = {65535, 65537, 65534},
            .accel_bias_t0 = {10, -8, 6},
            .gyro_bias_t0 = {-4, 6, -5},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -2, 1}, {2, 65536, -3}, {-1, 3, 65536}}},
            .gyro_misalignment = {.m = {{65536, 3, -2}, {-3, 65536, 1}, {2, -1, 65536}}},
            .accel_scale = {65535, 65538, 65535},
            .gyro_scale = {65537, 65535, 65537},
            .accel_bias_t0 = {-8, 12, -7},
            .gyro_bias_t0 = {5, -7, 6},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {2, -3, 1},
        .calibration_timestamp_epoch = 1731856400
    },
    /* Unit Serial: 1000102 */
    {
        .serial_number = 1000102,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 4, -6}, {-4, 65536, 2}, {6, -2, 65536}}},
            .gyro_misalignment = {.m = {{65536, -2, 4}, {2, 65536, -4}, {-4, 4, 65536}}},
            .accel_scale = {65538, 65534, 65540},
            .gyro_scale = {65534, 65538, 65532},
            .accel_bias_t0 = {20, -16, 12},
            .gyro_bias_t0 = {-8, 12, -10},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -4, 2}, {4, 65536, -6}, {-2, 6, 65536}}},
            .gyro_misalignment = {.m = {{65536, 6, -4}, {-6, 65536, 2}, {4, -2, 65536}}},
            .accel_scale = {65534, 65540, 65534},
            .gyro_scale = {65538, 65534, 65538},
            .accel_bias_t0 = {-16, 24, -14},
            .gyro_bias_t0 = {10, -14, 12},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {4, -6, 2},
        .calibration_timestamp_epoch = 1731942800
    },
    /* Unit Serial: 1000103 */
    {
        .serial_number = 1000103,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 6, -9}, {-6, 65536, 3}, {9, -3, 65536}}},
            .gyro_misalignment = {.m = {{65536, -3, 6}, {3, 65536, -6}, {-6, 6, 65536}}},
            .accel_scale = {65539, 65533, 65542},
            .gyro_scale = {65533, 65539, 65530},
            .accel_bias_t0 = {30, -24, 18},
            .gyro_bias_t0 = {-12, 18, -15},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -6, 3}, {6, 65536, -9}, {-3, 9, 65536}}},
            .gyro_misalignment = {.m = {{65536, 9, -6}, {-9, 65536, 3}, {6, -3, 65536}}},
            .accel_scale = {65533, 65542, 65533},
            .gyro_scale = {65539, 65533, 65539},
            .accel_bias_t0 = {-24, 36, -21},
            .gyro_bias_t0 = {15, -21, 18},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {6, -9, 3},
        .calibration_timestamp_epoch = 1732029200
    },
    /* Unit Serial: 1000104 */
    {
        .serial_number = 1000104,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 8, -12}, {-8, 65536, 4}, {12, -4, 65536}}},
            .gyro_misalignment = {.m = {{65536, -4, 8}, {4, 65536, -8}, {-8, 8, 65536}}},
            .accel_scale = {65540, 65532, 65544},
            .gyro_scale = {65532, 65540, 65528},
            .accel_bias_t0 = {40, -32, 24},
            .gyro_bias_t0 = {-16, 24, -20},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -8, 4}, {8, 65536, -12}, {-4, 12, 65536}}},
            .gyro_misalignment = {.m = {{65536, 12, -8}, {-12, 65536, 4}, {8, -4, 65536}}},
            .accel_scale = {65532, 65544, 65532},
            .gyro_scale = {65540, 65532, 65540},
            .accel_bias_t0 = {-32, 48, -28},
            .gyro_bias_t0 = {20, -28, 24},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {8, -12, 4},
        .calibration_timestamp_epoch = 1732115600
    },
    /* Unit Serial: 1000105 */
    {
        .serial_number = 1000105,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 10, -15}, {-10, 65536, 5}, {15, -5, 65536}}},
            .gyro_misalignment = {.m = {{65536, -5, 10}, {5, 65536, -10}, {-10, 10, 65536}}},
            .accel_scale = {65541, 65531, 65546},
            .gyro_scale = {65531, 65541, 65526},
            .accel_bias_t0 = {50, -40, 30},
            .gyro_bias_t0 = {-20, 30, -25},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -10, 5}, {10, 65536, -15}, {-5, 15, 65536}}},
            .gyro_misalignment = {.m = {{65536, 15, -10}, {-15, 65536, 5}, {10, -5, 65536}}},
            .accel_scale = {65531, 65546, 65531},
            .gyro_scale = {65541, 65531, 65541},
            .accel_bias_t0 = {-40, 60, -35},
            .gyro_bias_t0 = {25, -35, 30},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {10, -15, 5},
        .calibration_timestamp_epoch = 1732202000
    },
    /* Unit Serial: 1000106 */
    {
        .serial_number = 1000106,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 12, -18}, {-12, 65536, 6}, {18, -6, 65536}}},
            .gyro_misalignment = {.m = {{65536, -6, 12}, {6, 65536, -12}, {-12, 12, 65536}}},
            .accel_scale = {65542, 65530, 65548},
            .gyro_scale = {65530, 65542, 65524},
            .accel_bias_t0 = {60, -48, 36},
            .gyro_bias_t0 = {-24, 36, -30},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -12, 6}, {12, 65536, -18}, {-6, 18, 65536}}},
            .gyro_misalignment = {.m = {{65536, 18, -12}, {-18, 65536, 6}, {12, -6, 65536}}},
            .accel_scale = {65530, 65548, 65530},
            .gyro_scale = {65542, 65530, 65542},
            .accel_bias_t0 = {-48, 72, -42},
            .gyro_bias_t0 = {30, -42, 36},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {12, -18, 6},
        .calibration_timestamp_epoch = 1732288400
    },
    /* Unit Serial: 1000107 */
    {
        .serial_number = 1000107,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 14, -21}, {-14, 65536, 7}, {21, -7, 65536}}},
            .gyro_misalignment = {.m = {{65536, -7, 14}, {7, 65536, -14}, {-14, 14, 65536}}},
            .accel_scale = {65543, 65529, 65550},
            .gyro_scale = {65529, 65543, 65522},
            .accel_bias_t0 = {70, -56, 42},
            .gyro_bias_t0 = {-28, 42, -35},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -14, 7}, {14, 65536, -21}, {-7, 21, 65536}}},
            .gyro_misalignment = {.m = {{65536, 21, -14}, {-21, 65536, 7}, {14, -7, 65536}}},
            .accel_scale = {65529, 65550, 65529},
            .gyro_scale = {65543, 65529, 65543},
            .accel_bias_t0 = {-56, 84, -49},
            .gyro_bias_t0 = {35, -49, 42},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {14, -21, 7},
        .calibration_timestamp_epoch = 1732374800
    },
    /* Unit Serial: 1000108 */
    {
        .serial_number = 1000108,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 16, -24}, {-16, 65536, 8}, {24, -8, 65536}}},
            .gyro_misalignment = {.m = {{65536, -8, 16}, {8, 65536, -16}, {-16, 16, 65536}}},
            .accel_scale = {65544, 65528, 65552},
            .gyro_scale = {65528, 65544, 65520},
            .accel_bias_t0 = {80, -64, 48},
            .gyro_bias_t0 = {-32, 48, -40},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -16, 8}, {16, 65536, -24}, {-8, 24, 65536}}},
            .gyro_misalignment = {.m = {{65536, 24, -16}, {-24, 65536, 8}, {16, -8, 65536}}},
            .accel_scale = {65528, 65552, 65528},
            .gyro_scale = {65544, 65528, 65544},
            .accel_bias_t0 = {-64, 96, -56},
            .gyro_bias_t0 = {40, -56, 48},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {16, -24, 8},
        .calibration_timestamp_epoch = 1732461200
    },
    /* Unit Serial: 1000109 */
    {
        .serial_number = 1000109,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 18, -27}, {-18, 65536, 9}, {27, -9, 65536}}},
            .gyro_misalignment = {.m = {{65536, -9, 18}, {9, 65536, -18}, {-18, 18, 65536}}},
            .accel_scale = {65545, 65527, 65554},
            .gyro_scale = {65527, 65545, 65518},
            .accel_bias_t0 = {90, -72, 54},
            .gyro_bias_t0 = {-36, 54, -45},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -18, 9}, {18, 65536, -27}, {-9, 27, 65536}}},
            .gyro_misalignment = {.m = {{65536, 27, -18}, {-27, 65536, 9}, {18, -9, 65536}}},
            .accel_scale = {65527, 65554, 65527},
            .gyro_scale = {65545, 65527, 65545},
            .accel_bias_t0 = {-72, 108, -63},
            .gyro_bias_t0 = {45, -63, 54},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {18, -27, 9},
        .calibration_timestamp_epoch = 1732547600
    },
    /* Unit Serial: 1000110 */
    {
        .serial_number = 1000110,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 20, -30}, {-20, 65536, 10}, {30, -10, 65536}}},
            .gyro_misalignment = {.m = {{65536, -10, 20}, {10, 65536, -20}, {-20, 20, 65536}}},
            .accel_scale = {65546, 65526, 65556},
            .gyro_scale = {65526, 65546, 65516},
            .accel_bias_t0 = {100, -80, 60},
            .gyro_bias_t0 = {-40, 60, -50},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -20, 10}, {20, 65536, -30}, {-10, 30, 65536}}},
            .gyro_misalignment = {.m = {{65536, 30, -20}, {-30, 65536, 10}, {20, -10, 65536}}},
            .accel_scale = {65526, 65556, 65526},
            .gyro_scale = {65546, 65526, 65546},
            .accel_bias_t0 = {-80, 120, -70},
            .gyro_bias_t0 = {50, -70, 60},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {20, -30, 10},
        .calibration_timestamp_epoch = 1732634000
    },
    /* Unit Serial: 1000111 */
    {
        .serial_number = 1000111,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 22, -33}, {-22, 65536, 11}, {33, -11, 65536}}},
            .gyro_misalignment = {.m = {{65536, -11, 22}, {11, 65536, -22}, {-22, 22, 65536}}},
            .accel_scale = {65547, 65525, 65558},
            .gyro_scale = {65525, 65547, 65514},
            .accel_bias_t0 = {110, -88, 66},
            .gyro_bias_t0 = {-44, 66, -55},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -22, 11}, {22, 65536, -33}, {-11, 33, 65536}}},
            .gyro_misalignment = {.m = {{65536, 33, -22}, {-33, 65536, 11}, {22, -11, 65536}}},
            .accel_scale = {65525, 65558, 65525},
            .gyro_scale = {65547, 65525, 65547},
            .accel_bias_t0 = {-88, 132, -77},
            .gyro_bias_t0 = {55, -77, 66},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {22, -33, 11},
        .calibration_timestamp_epoch = 1732720400
    },
    /* Unit Serial: 1000112 */
    {
        .serial_number = 1000112,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 24, -36}, {-24, 65536, 12}, {36, -12, 65536}}},
            .gyro_misalignment = {.m = {{65536, -12, 24}, {12, 65536, -24}, {-24, 24, 65536}}},
            .accel_scale = {65548, 65524, 65560},
            .gyro_scale = {65524, 65548, 65512},
            .accel_bias_t0 = {120, -96, 72},
            .gyro_bias_t0 = {-48, 72, -60},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -24, 12}, {24, 65536, -36}, {-12, 36, 65536}}},
            .gyro_misalignment = {.m = {{65536, 36, -24}, {-36, 65536, 12}, {24, -12, 65536}}},
            .accel_scale = {65524, 65560, 65524},
            .gyro_scale = {65548, 65524, 65548},
            .accel_bias_t0 = {-96, 144, -84},
            .gyro_bias_t0 = {60, -84, 72},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {24, -36, 12},
        .calibration_timestamp_epoch = 1732806800
    },
    /* Unit Serial: 1000113 */
    {
        .serial_number = 1000113,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 26, -39}, {-26, 65536, 13}, {39, -13, 65536}}},
            .gyro_misalignment = {.m = {{65536, -13, 26}, {13, 65536, -26}, {-26, 26, 65536}}},
            .accel_scale = {65549, 65523, 65562},
            .gyro_scale = {65523, 65549, 65510},
            .accel_bias_t0 = {130, -104, 78},
            .gyro_bias_t0 = {-52, 78, -65},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -26, 13}, {26, 65536, -39}, {-13, 39, 65536}}},
            .gyro_misalignment = {.m = {{65536, 39, -26}, {-39, 65536, 13}, {26, -13, 65536}}},
            .accel_scale = {65523, 65562, 65523},
            .gyro_scale = {65549, 65523, 65549},
            .accel_bias_t0 = {-104, 156, -91},
            .gyro_bias_t0 = {65, -91, 78},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {26, -39, 13},
        .calibration_timestamp_epoch = 1732893200
    },
    /* Unit Serial: 1000114 */
    {
        .serial_number = 1000114,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 28, -42}, {-28, 65536, 14}, {42, -14, 65536}}},
            .gyro_misalignment = {.m = {{65536, -14, 28}, {14, 65536, -28}, {-28, 28, 65536}}},
            .accel_scale = {65550, 65522, 65564},
            .gyro_scale = {65522, 65550, 65508},
            .accel_bias_t0 = {140, -112, 84},
            .gyro_bias_t0 = {-56, 84, -70},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -28, 14}, {28, 65536, -42}, {-14, 42, 65536}}},
            .gyro_misalignment = {.m = {{65536, 42, -28}, {-42, 65536, 14}, {28, -14, 65536}}},
            .accel_scale = {65522, 65564, 65522},
            .gyro_scale = {65550, 65522, 65550},
            .accel_bias_t0 = {-112, 168, -98},
            .gyro_bias_t0 = {70, -98, 84},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {28, -42, 14},
        .calibration_timestamp_epoch = 1732979600
    },
    /* Unit Serial: 1000115 */
    {
        .serial_number = 1000115,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 30, -45}, {-30, 65536, 15}, {45, -15, 65536}}},
            .gyro_misalignment = {.m = {{65536, -15, 30}, {15, 65536, -30}, {-30, 30, 65536}}},
            .accel_scale = {65551, 65521, 65566},
            .gyro_scale = {65521, 65551, 65506},
            .accel_bias_t0 = {150, -120, 90},
            .gyro_bias_t0 = {-60, 90, -75},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -30, 15}, {30, 65536, -45}, {-15, 45, 65536}}},
            .gyro_misalignment = {.m = {{65536, 45, -30}, {-45, 65536, 15}, {30, -15, 65536}}},
            .accel_scale = {65521, 65566, 65521},
            .gyro_scale = {65551, 65521, 65551},
            .accel_bias_t0 = {-120, 180, -105},
            .gyro_bias_t0 = {75, -105, 90},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {30, -45, 15},
        .calibration_timestamp_epoch = 1733066000
    },
    /* Unit Serial: 1000116 */
    {
        .serial_number = 1000116,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 32, -48}, {-32, 65536, 16}, {48, -16, 65536}}},
            .gyro_misalignment = {.m = {{65536, -16, 32}, {16, 65536, -32}, {-32, 32, 65536}}},
            .accel_scale = {65552, 65520, 65568},
            .gyro_scale = {65520, 65552, 65504},
            .accel_bias_t0 = {160, -128, 96},
            .gyro_bias_t0 = {-64, 96, -80},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -32, 16}, {32, 65536, -48}, {-16, 48, 65536}}},
            .gyro_misalignment = {.m = {{65536, 48, -32}, {-48, 65536, 16}, {32, -16, 65536}}},
            .accel_scale = {65520, 65568, 65520},
            .gyro_scale = {65552, 65520, 65552},
            .accel_bias_t0 = {-128, 192, -112},
            .gyro_bias_t0 = {80, -112, 96},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {32, -48, 16},
        .calibration_timestamp_epoch = 1733152400
    },
    /* Unit Serial: 1000117 */
    {
        .serial_number = 1000117,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 34, -51}, {-34, 65536, 17}, {51, -17, 65536}}},
            .gyro_misalignment = {.m = {{65536, -17, 34}, {17, 65536, -34}, {-34, 34, 65536}}},
            .accel_scale = {65553, 65519, 65570},
            .gyro_scale = {65519, 65553, 65502},
            .accel_bias_t0 = {170, -136, 102},
            .gyro_bias_t0 = {-68, 102, -85},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -34, 17}, {34, 65536, -51}, {-17, 51, 65536}}},
            .gyro_misalignment = {.m = {{65536, 51, -34}, {-51, 65536, 17}, {34, -17, 65536}}},
            .accel_scale = {65519, 65570, 65519},
            .gyro_scale = {65553, 65519, 65553},
            .accel_bias_t0 = {-136, 204, -119},
            .gyro_bias_t0 = {85, -119, 102},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {34, -51, 17},
        .calibration_timestamp_epoch = 1733238800
    },
    /* Unit Serial: 1000118 */
    {
        .serial_number = 1000118,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 36, -54}, {-36, 65536, 18}, {54, -18, 65536}}},
            .gyro_misalignment = {.m = {{65536, -18, 36}, {18, 65536, -36}, {-36, 36, 65536}}},
            .accel_scale = {65554, 65518, 65572},
            .gyro_scale = {65518, 65554, 65500},
            .accel_bias_t0 = {180, -144, 108},
            .gyro_bias_t0 = {-72, 108, -90},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -36, 18}, {36, 65536, -54}, {-18, 54, 65536}}},
            .gyro_misalignment = {.m = {{65536, 54, -36}, {-54, 65536, 18}, {36, -18, 65536}}},
            .accel_scale = {65518, 65572, 65518},
            .gyro_scale = {65554, 65518, 65554},
            .accel_bias_t0 = {-144, 216, -126},
            .gyro_bias_t0 = {90, -126, 108},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {36, -54, 18},
        .calibration_timestamp_epoch = 1733325200
    },
    /* Unit Serial: 1000119 */
    {
        .serial_number = 1000119,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 38, -57}, {-38, 65536, 19}, {57, -19, 65536}}},
            .gyro_misalignment = {.m = {{65536, -19, 38}, {19, 65536, -38}, {-38, 38, 65536}}},
            .accel_scale = {65555, 65517, 65574},
            .gyro_scale = {65517, 65555, 65498},
            .accel_bias_t0 = {190, -152, 114},
            .gyro_bias_t0 = {-76, 114, -95},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -38, 19}, {38, 65536, -57}, {-19, 57, 65536}}},
            .gyro_misalignment = {.m = {{65536, 57, -38}, {-57, 65536, 19}, {38, -19, 65536}}},
            .accel_scale = {65517, 65574, 65517},
            .gyro_scale = {65555, 65517, 65555},
            .accel_bias_t0 = {-152, 228, -133},
            .gyro_bias_t0 = {95, -133, 114},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {38, -57, 19},
        .calibration_timestamp_epoch = 1733411600
    },
    /* Unit Serial: 1000120 */
    {
        .serial_number = 1000120,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 40, -60}, {-40, 65536, 20}, {60, -20, 65536}}},
            .gyro_misalignment = {.m = {{65536, -20, 40}, {20, 65536, -40}, {-40, 40, 65536}}},
            .accel_scale = {65556, 65516, 65576},
            .gyro_scale = {65516, 65556, 65496},
            .accel_bias_t0 = {200, -160, 120},
            .gyro_bias_t0 = {-80, 120, -100},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -40, 20}, {40, 65536, -60}, {-20, 60, 65536}}},
            .gyro_misalignment = {.m = {{65536, 60, -40}, {-60, 65536, 20}, {40, -20, 65536}}},
            .accel_scale = {65516, 65576, 65516},
            .gyro_scale = {65556, 65516, 65556},
            .accel_bias_t0 = {-160, 240, -140},
            .gyro_bias_t0 = {100, -140, 120},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {40, -60, 20},
        .calibration_timestamp_epoch = 1733498000
    },
    /* Unit Serial: 1000121 */
    {
        .serial_number = 1000121,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 42, -63}, {-42, 65536, 21}, {63, -21, 65536}}},
            .gyro_misalignment = {.m = {{65536, -21, 42}, {21, 65536, -42}, {-42, 42, 65536}}},
            .accel_scale = {65557, 65515, 65578},
            .gyro_scale = {65515, 65557, 65494},
            .accel_bias_t0 = {210, -168, 126},
            .gyro_bias_t0 = {-84, 126, -105},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -42, 21}, {42, 65536, -63}, {-21, 63, 65536}}},
            .gyro_misalignment = {.m = {{65536, 63, -42}, {-63, 65536, 21}, {42, -21, 65536}}},
            .accel_scale = {65515, 65578, 65515},
            .gyro_scale = {65557, 65515, 65557},
            .accel_bias_t0 = {-168, 252, -147},
            .gyro_bias_t0 = {105, -147, 126},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {42, -63, 21},
        .calibration_timestamp_epoch = 1733584400
    },
    /* Unit Serial: 1000122 */
    {
        .serial_number = 1000122,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 44, -66}, {-44, 65536, 22}, {66, -22, 65536}}},
            .gyro_misalignment = {.m = {{65536, -22, 44}, {22, 65536, -44}, {-44, 44, 65536}}},
            .accel_scale = {65558, 65514, 65580},
            .gyro_scale = {65514, 65558, 65492},
            .accel_bias_t0 = {220, -176, 132},
            .gyro_bias_t0 = {-88, 132, -110},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -44, 22}, {44, 65536, -66}, {-22, 66, 65536}}},
            .gyro_misalignment = {.m = {{65536, 66, -44}, {-66, 65536, 22}, {44, -22, 65536}}},
            .accel_scale = {65514, 65580, 65514},
            .gyro_scale = {65558, 65514, 65558},
            .accel_bias_t0 = {-176, 264, -154},
            .gyro_bias_t0 = {110, -154, 132},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {44, -66, 22},
        .calibration_timestamp_epoch = 1733670800
    },
    /* Unit Serial: 1000123 */
    {
        .serial_number = 1000123,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 46, -69}, {-46, 65536, 23}, {69, -23, 65536}}},
            .gyro_misalignment = {.m = {{65536, -23, 46}, {23, 65536, -46}, {-46, 46, 65536}}},
            .accel_scale = {65559, 65513, 65582},
            .gyro_scale = {65513, 65559, 65490},
            .accel_bias_t0 = {230, -184, 138},
            .gyro_bias_t0 = {-92, 138, -115},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -46, 23}, {46, 65536, -69}, {-23, 69, 65536}}},
            .gyro_misalignment = {.m = {{65536, 69, -46}, {-69, 65536, 23}, {46, -23, 65536}}},
            .accel_scale = {65513, 65582, 65513},
            .gyro_scale = {65559, 65513, 65559},
            .accel_bias_t0 = {-184, 276, -161},
            .gyro_bias_t0 = {115, -161, 138},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {46, -69, 23},
        .calibration_timestamp_epoch = 1733757200
    },
    /* Unit Serial: 1000124 */
    {
        .serial_number = 1000124,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 48, -72}, {-48, 65536, 24}, {72, -24, 65536}}},
            .gyro_misalignment = {.m = {{65536, -24, 48}, {24, 65536, -48}, {-48, 48, 65536}}},
            .accel_scale = {65560, 65512, 65584},
            .gyro_scale = {65512, 65560, 65488},
            .accel_bias_t0 = {240, -192, 144},
            .gyro_bias_t0 = {-96, 144, -120},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -48, 24}, {48, 65536, -72}, {-24, 72, 65536}}},
            .gyro_misalignment = {.m = {{65536, 72, -48}, {-72, 65536, 24}, {48, -24, 65536}}},
            .accel_scale = {65512, 65584, 65512},
            .gyro_scale = {65560, 65512, 65560},
            .accel_bias_t0 = {-192, 288, -168},
            .gyro_bias_t0 = {120, -168, 144},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {48, -72, 24},
        .calibration_timestamp_epoch = 1733843600
    },
    /* Unit Serial: 1000125 */
    {
        .serial_number = 1000125,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 50, -75}, {-50, 65536, 25}, {75, -25, 65536}}},
            .gyro_misalignment = {.m = {{65536, -25, 50}, {25, 65536, -50}, {-50, 50, 65536}}},
            .accel_scale = {65561, 65511, 65586},
            .gyro_scale = {65511, 65561, 65486},
            .accel_bias_t0 = {250, -200, 150},
            .gyro_bias_t0 = {-100, 150, -125},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -50, 25}, {50, 65536, -75}, {-25, 75, 65536}}},
            .gyro_misalignment = {.m = {{65536, 75, -50}, {-75, 65536, 25}, {50, -25, 65536}}},
            .accel_scale = {65511, 65586, 65511},
            .gyro_scale = {65561, 65511, 65561},
            .accel_bias_t0 = {-200, 300, -175},
            .gyro_bias_t0 = {125, -175, 150},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {50, -75, 25},
        .calibration_timestamp_epoch = 1733930000
    },
    /* Unit Serial: 1000126 */
    {
        .serial_number = 1000126,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 52, -78}, {-52, 65536, 26}, {78, -26, 65536}}},
            .gyro_misalignment = {.m = {{65536, -26, 52}, {26, 65536, -52}, {-52, 52, 65536}}},
            .accel_scale = {65562, 65510, 65588},
            .gyro_scale = {65510, 65562, 65484},
            .accel_bias_t0 = {260, -208, 156},
            .gyro_bias_t0 = {-104, 156, -130},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -52, 26}, {52, 65536, -78}, {-26, 78, 65536}}},
            .gyro_misalignment = {.m = {{65536, 78, -52}, {-78, 65536, 26}, {52, -26, 65536}}},
            .accel_scale = {65510, 65588, 65510},
            .gyro_scale = {65562, 65510, 65562},
            .accel_bias_t0 = {-208, 312, -182},
            .gyro_bias_t0 = {130, -182, 156},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {52, -78, 26},
        .calibration_timestamp_epoch = 1734016400
    },
    /* Unit Serial: 1000127 */
    {
        .serial_number = 1000127,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 54, -81}, {-54, 65536, 27}, {81, -27, 65536}}},
            .gyro_misalignment = {.m = {{65536, -27, 54}, {27, 65536, -54}, {-54, 54, 65536}}},
            .accel_scale = {65563, 65509, 65590},
            .gyro_scale = {65509, 65563, 65482},
            .accel_bias_t0 = {270, -216, 162},
            .gyro_bias_t0 = {-108, 162, -135},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -54, 27}, {54, 65536, -81}, {-27, 81, 65536}}},
            .gyro_misalignment = {.m = {{65536, 81, -54}, {-81, 65536, 27}, {54, -27, 65536}}},
            .accel_scale = {65509, 65590, 65509},
            .gyro_scale = {65563, 65509, 65563},
            .accel_bias_t0 = {-216, 324, -189},
            .gyro_bias_t0 = {135, -189, 162},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {54, -81, 27},
        .calibration_timestamp_epoch = 1734102800
    },
    /* Unit Serial: 1000128 */
    {
        .serial_number = 1000128,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 56, -84}, {-56, 65536, 28}, {84, -28, 65536}}},
            .gyro_misalignment = {.m = {{65536, -28, 56}, {28, 65536, -56}, {-56, 56, 65536}}},
            .accel_scale = {65564, 65508, 65592},
            .gyro_scale = {65508, 65564, 65480},
            .accel_bias_t0 = {280, -224, 168},
            .gyro_bias_t0 = {-112, 168, -140},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -56, 28}, {56, 65536, -84}, {-28, 84, 65536}}},
            .gyro_misalignment = {.m = {{65536, 84, -56}, {-84, 65536, 28}, {56, -28, 65536}}},
            .accel_scale = {65508, 65592, 65508},
            .gyro_scale = {65564, 65508, 65564},
            .accel_bias_t0 = {-224, 336, -196},
            .gyro_bias_t0 = {140, -196, 168},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {56, -84, 28},
        .calibration_timestamp_epoch = 1734189200
    },
    /* Unit Serial: 1000129 */
    {
        .serial_number = 1000129,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 58, -87}, {-58, 65536, 29}, {87, -29, 65536}}},
            .gyro_misalignment = {.m = {{65536, -29, 58}, {29, 65536, -58}, {-58, 58, 65536}}},
            .accel_scale = {65565, 65507, 65594},
            .gyro_scale = {65507, 65565, 65478},
            .accel_bias_t0 = {290, -232, 174},
            .gyro_bias_t0 = {-116, 174, -145},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -58, 29}, {58, 65536, -87}, {-29, 87, 65536}}},
            .gyro_misalignment = {.m = {{65536, 87, -58}, {-87, 65536, 29}, {58, -29, 65536}}},
            .accel_scale = {65507, 65594, 65507},
            .gyro_scale = {65565, 65507, 65565},
            .accel_bias_t0 = {-232, 348, -203},
            .gyro_bias_t0 = {145, -203, 174},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {58, -87, 29},
        .calibration_timestamp_epoch = 1734275600
    },
    /* Unit Serial: 1000130 */
    {
        .serial_number = 1000130,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 60, -90}, {-60, 65536, 30}, {90, -30, 65536}}},
            .gyro_misalignment = {.m = {{65536, -30, 60}, {30, 65536, -60}, {-60, 60, 65536}}},
            .accel_scale = {65566, 65506, 65596},
            .gyro_scale = {65506, 65566, 65476},
            .accel_bias_t0 = {300, -240, 180},
            .gyro_bias_t0 = {-120, 180, -150},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -60, 30}, {60, 65536, -90}, {-30, 90, 65536}}},
            .gyro_misalignment = {.m = {{65536, 90, -60}, {-90, 65536, 30}, {60, -30, 65536}}},
            .accel_scale = {65506, 65596, 65506},
            .gyro_scale = {65566, 65506, 65566},
            .accel_bias_t0 = {-240, 360, -210},
            .gyro_bias_t0 = {150, -210, 180},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {60, -90, 30},
        .calibration_timestamp_epoch = 1734362000
    },
    /* Unit Serial: 1000131 */
    {
        .serial_number = 1000131,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 62, -93}, {-62, 65536, 31}, {93, -31, 65536}}},
            .gyro_misalignment = {.m = {{65536, -31, 62}, {31, 65536, -62}, {-62, 62, 65536}}},
            .accel_scale = {65567, 65505, 65598},
            .gyro_scale = {65505, 65567, 65474},
            .accel_bias_t0 = {310, -248, 186},
            .gyro_bias_t0 = {-124, 186, -155},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -62, 31}, {62, 65536, -93}, {-31, 93, 65536}}},
            .gyro_misalignment = {.m = {{65536, 93, -62}, {-93, 65536, 31}, {62, -31, 65536}}},
            .accel_scale = {65505, 65598, 65505},
            .gyro_scale = {65567, 65505, 65567},
            .accel_bias_t0 = {-248, 372, -217},
            .gyro_bias_t0 = {155, -217, 186},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {62, -93, 31},
        .calibration_timestamp_epoch = 1734448400
    },
    /* Unit Serial: 1000132 */
    {
        .serial_number = 1000132,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 64, -96}, {-64, 65536, 32}, {96, -32, 65536}}},
            .gyro_misalignment = {.m = {{65536, -32, 64}, {32, 65536, -64}, {-64, 64, 65536}}},
            .accel_scale = {65568, 65504, 65600},
            .gyro_scale = {65504, 65568, 65472},
            .accel_bias_t0 = {320, -256, 192},
            .gyro_bias_t0 = {-128, 192, -160},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -64, 32}, {64, 65536, -96}, {-32, 96, 65536}}},
            .gyro_misalignment = {.m = {{65536, 96, -64}, {-96, 65536, 32}, {64, -32, 65536}}},
            .accel_scale = {65504, 65600, 65504},
            .gyro_scale = {65568, 65504, 65568},
            .accel_bias_t0 = {-256, 384, -224},
            .gyro_bias_t0 = {160, -224, 192},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {64, -96, 32},
        .calibration_timestamp_epoch = 1734534800
    },
    /* Unit Serial: 1000133 */
    {
        .serial_number = 1000133,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 66, -99}, {-66, 65536, 33}, {99, -33, 65536}}},
            .gyro_misalignment = {.m = {{65536, -33, 66}, {33, 65536, -66}, {-66, 66, 65536}}},
            .accel_scale = {65569, 65503, 65602},
            .gyro_scale = {65503, 65569, 65470},
            .accel_bias_t0 = {330, -264, 198},
            .gyro_bias_t0 = {-132, 198, -165},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -66, 33}, {66, 65536, -99}, {-33, 99, 65536}}},
            .gyro_misalignment = {.m = {{65536, 99, -66}, {-99, 65536, 33}, {66, -33, 65536}}},
            .accel_scale = {65503, 65602, 65503},
            .gyro_scale = {65569, 65503, 65569},
            .accel_bias_t0 = {-264, 396, -231},
            .gyro_bias_t0 = {165, -231, 198},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {66, -99, 33},
        .calibration_timestamp_epoch = 1734621200
    },
    /* Unit Serial: 1000134 */
    {
        .serial_number = 1000134,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 68, -102}, {-68, 65536, 34}, {102, -34, 65536}}},
            .gyro_misalignment = {.m = {{65536, -34, 68}, {34, 65536, -68}, {-68, 68, 65536}}},
            .accel_scale = {65570, 65502, 65604},
            .gyro_scale = {65502, 65570, 65468},
            .accel_bias_t0 = {340, -272, 204},
            .gyro_bias_t0 = {-136, 204, -170},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -68, 34}, {68, 65536, -102}, {-34, 102, 65536}}},
            .gyro_misalignment = {.m = {{65536, 102, -68}, {-102, 65536, 34}, {68, -34, 65536}}},
            .accel_scale = {65502, 65604, 65502},
            .gyro_scale = {65570, 65502, 65570},
            .accel_bias_t0 = {-272, 408, -238},
            .gyro_bias_t0 = {170, -238, 204},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {68, -102, 34},
        .calibration_timestamp_epoch = 1734707600
    },
    /* Unit Serial: 1000135 */
    {
        .serial_number = 1000135,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 70, -105}, {-70, 65536, 35}, {105, -35, 65536}}},
            .gyro_misalignment = {.m = {{65536, -35, 70}, {35, 65536, -70}, {-70, 70, 65536}}},
            .accel_scale = {65571, 65501, 65606},
            .gyro_scale = {65501, 65571, 65466},
            .accel_bias_t0 = {350, -280, 210},
            .gyro_bias_t0 = {-140, 210, -175},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -70, 35}, {70, 65536, -105}, {-35, 105, 65536}}},
            .gyro_misalignment = {.m = {{65536, 105, -70}, {-105, 65536, 35}, {70, -35, 65536}}},
            .accel_scale = {65501, 65606, 65501},
            .gyro_scale = {65571, 65501, 65571},
            .accel_bias_t0 = {-280, 420, -245},
            .gyro_bias_t0 = {175, -245, 210},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {70, -105, 35},
        .calibration_timestamp_epoch = 1734794000
    },
    /* Unit Serial: 1000136 */
    {
        .serial_number = 1000136,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 72, -108}, {-72, 65536, 36}, {108, -36, 65536}}},
            .gyro_misalignment = {.m = {{65536, -36, 72}, {36, 65536, -72}, {-72, 72, 65536}}},
            .accel_scale = {65572, 65500, 65608},
            .gyro_scale = {65500, 65572, 65464},
            .accel_bias_t0 = {360, -288, 216},
            .gyro_bias_t0 = {-144, 216, -180},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -72, 36}, {72, 65536, -108}, {-36, 108, 65536}}},
            .gyro_misalignment = {.m = {{65536, 108, -72}, {-108, 65536, 36}, {72, -36, 65536}}},
            .accel_scale = {65500, 65608, 65500},
            .gyro_scale = {65572, 65500, 65572},
            .accel_bias_t0 = {-288, 432, -252},
            .gyro_bias_t0 = {180, -252, 216},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {72, -108, 36},
        .calibration_timestamp_epoch = 1734880400
    },
    /* Unit Serial: 1000137 */
    {
        .serial_number = 1000137,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 74, -111}, {-74, 65536, 37}, {111, -37, 65536}}},
            .gyro_misalignment = {.m = {{65536, -37, 74}, {37, 65536, -74}, {-74, 74, 65536}}},
            .accel_scale = {65573, 65499, 65610},
            .gyro_scale = {65499, 65573, 65462},
            .accel_bias_t0 = {370, -296, 222},
            .gyro_bias_t0 = {-148, 222, -185},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -74, 37}, {74, 65536, -111}, {-37, 111, 65536}}},
            .gyro_misalignment = {.m = {{65536, 111, -74}, {-111, 65536, 37}, {74, -37, 65536}}},
            .accel_scale = {65499, 65610, 65499},
            .gyro_scale = {65573, 65499, 65573},
            .accel_bias_t0 = {-296, 444, -259},
            .gyro_bias_t0 = {185, -259, 222},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {74, -111, 37},
        .calibration_timestamp_epoch = 1734966800
    },
    /* Unit Serial: 1000138 */
    {
        .serial_number = 1000138,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 76, -114}, {-76, 65536, 38}, {114, -38, 65536}}},
            .gyro_misalignment = {.m = {{65536, -38, 76}, {38, 65536, -76}, {-76, 76, 65536}}},
            .accel_scale = {65574, 65498, 65612},
            .gyro_scale = {65498, 65574, 65460},
            .accel_bias_t0 = {380, -304, 228},
            .gyro_bias_t0 = {-152, 228, -190},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -76, 38}, {76, 65536, -114}, {-38, 114, 65536}}},
            .gyro_misalignment = {.m = {{65536, 114, -76}, {-114, 65536, 38}, {76, -38, 65536}}},
            .accel_scale = {65498, 65612, 65498},
            .gyro_scale = {65574, 65498, 65574},
            .accel_bias_t0 = {-304, 456, -266},
            .gyro_bias_t0 = {190, -266, 228},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {76, -114, 38},
        .calibration_timestamp_epoch = 1735053200
    },
    /* Unit Serial: 1000139 */
    {
        .serial_number = 1000139,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 78, -117}, {-78, 65536, 39}, {117, -39, 65536}}},
            .gyro_misalignment = {.m = {{65536, -39, 78}, {39, 65536, -78}, {-78, 78, 65536}}},
            .accel_scale = {65575, 65497, 65614},
            .gyro_scale = {65497, 65575, 65458},
            .accel_bias_t0 = {390, -312, 234},
            .gyro_bias_t0 = {-156, 234, -195},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -78, 39}, {78, 65536, -117}, {-39, 117, 65536}}},
            .gyro_misalignment = {.m = {{65536, 117, -78}, {-117, 65536, 39}, {78, -39, 65536}}},
            .accel_scale = {65497, 65614, 65497},
            .gyro_scale = {65575, 65497, 65575},
            .accel_bias_t0 = {-312, 468, -273},
            .gyro_bias_t0 = {195, -273, 234},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {78, -117, 39},
        .calibration_timestamp_epoch = 1735139600
    },
    /* Unit Serial: 1000140 */
    {
        .serial_number = 1000140,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 80, -120}, {-80, 65536, 40}, {120, -40, 65536}}},
            .gyro_misalignment = {.m = {{65536, -40, 80}, {40, 65536, -80}, {-80, 80, 65536}}},
            .accel_scale = {65576, 65496, 65616},
            .gyro_scale = {65496, 65576, 65456},
            .accel_bias_t0 = {400, -320, 240},
            .gyro_bias_t0 = {-160, 240, -200},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -80, 40}, {80, 65536, -120}, {-40, 120, 65536}}},
            .gyro_misalignment = {.m = {{65536, 120, -80}, {-120, 65536, 40}, {80, -40, 65536}}},
            .accel_scale = {65496, 65616, 65496},
            .gyro_scale = {65576, 65496, 65576},
            .accel_bias_t0 = {-320, 480, -280},
            .gyro_bias_t0 = {200, -280, 240},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {80, -120, 40},
        .calibration_timestamp_epoch = 1735226000
    },
    /* Unit Serial: 1000141 */
    {
        .serial_number = 1000141,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 82, -123}, {-82, 65536, 41}, {123, -41, 65536}}},
            .gyro_misalignment = {.m = {{65536, -41, 82}, {41, 65536, -82}, {-82, 82, 65536}}},
            .accel_scale = {65577, 65495, 65618},
            .gyro_scale = {65495, 65577, 65454},
            .accel_bias_t0 = {410, -328, 246},
            .gyro_bias_t0 = {-164, 246, -205},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -82, 41}, {82, 65536, -123}, {-41, 123, 65536}}},
            .gyro_misalignment = {.m = {{65536, 123, -82}, {-123, 65536, 41}, {82, -41, 65536}}},
            .accel_scale = {65495, 65618, 65495},
            .gyro_scale = {65577, 65495, 65577},
            .accel_bias_t0 = {-328, 492, -287},
            .gyro_bias_t0 = {205, -287, 246},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {82, -123, 41},
        .calibration_timestamp_epoch = 1735312400
    },
    /* Unit Serial: 1000142 */
    {
        .serial_number = 1000142,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 84, -126}, {-84, 65536, 42}, {126, -42, 65536}}},
            .gyro_misalignment = {.m = {{65536, -42, 84}, {42, 65536, -84}, {-84, 84, 65536}}},
            .accel_scale = {65578, 65494, 65620},
            .gyro_scale = {65494, 65578, 65452},
            .accel_bias_t0 = {420, -336, 252},
            .gyro_bias_t0 = {-168, 252, -210},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -84, 42}, {84, 65536, -126}, {-42, 126, 65536}}},
            .gyro_misalignment = {.m = {{65536, 126, -84}, {-126, 65536, 42}, {84, -42, 65536}}},
            .accel_scale = {65494, 65620, 65494},
            .gyro_scale = {65578, 65494, 65578},
            .accel_bias_t0 = {-336, 504, -294},
            .gyro_bias_t0 = {210, -294, 252},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {84, -126, 42},
        .calibration_timestamp_epoch = 1735398800
    },
    /* Unit Serial: 1000143 */
    {
        .serial_number = 1000143,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 86, -129}, {-86, 65536, 43}, {129, -43, 65536}}},
            .gyro_misalignment = {.m = {{65536, -43, 86}, {43, 65536, -86}, {-86, 86, 65536}}},
            .accel_scale = {65579, 65493, 65622},
            .gyro_scale = {65493, 65579, 65450},
            .accel_bias_t0 = {430, -344, 258},
            .gyro_bias_t0 = {-172, 258, -215},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -86, 43}, {86, 65536, -129}, {-43, 129, 65536}}},
            .gyro_misalignment = {.m = {{65536, 129, -86}, {-129, 65536, 43}, {86, -43, 65536}}},
            .accel_scale = {65493, 65622, 65493},
            .gyro_scale = {65579, 65493, 65579},
            .accel_bias_t0 = {-344, 516, -301},
            .gyro_bias_t0 = {215, -301, 258},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {86, -129, 43},
        .calibration_timestamp_epoch = 1735485200
    },
    /* Unit Serial: 1000144 */
    {
        .serial_number = 1000144,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 88, -132}, {-88, 65536, 44}, {132, -44, 65536}}},
            .gyro_misalignment = {.m = {{65536, -44, 88}, {44, 65536, -88}, {-88, 88, 65536}}},
            .accel_scale = {65580, 65492, 65624},
            .gyro_scale = {65492, 65580, 65448},
            .accel_bias_t0 = {440, -352, 264},
            .gyro_bias_t0 = {-176, 264, -220},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -88, 44}, {88, 65536, -132}, {-44, 132, 65536}}},
            .gyro_misalignment = {.m = {{65536, 132, -88}, {-132, 65536, 44}, {88, -44, 65536}}},
            .accel_scale = {65492, 65624, 65492},
            .gyro_scale = {65580, 65492, 65580},
            .accel_bias_t0 = {-352, 528, -308},
            .gyro_bias_t0 = {220, -308, 264},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {88, -132, 44},
        .calibration_timestamp_epoch = 1735571600
    },
    /* Unit Serial: 1000145 */
    {
        .serial_number = 1000145,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 90, -135}, {-90, 65536, 45}, {135, -45, 65536}}},
            .gyro_misalignment = {.m = {{65536, -45, 90}, {45, 65536, -90}, {-90, 90, 65536}}},
            .accel_scale = {65581, 65491, 65626},
            .gyro_scale = {65491, 65581, 65446},
            .accel_bias_t0 = {450, -360, 270},
            .gyro_bias_t0 = {-180, 270, -225},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -90, 45}, {90, 65536, -135}, {-45, 135, 65536}}},
            .gyro_misalignment = {.m = {{65536, 135, -90}, {-135, 65536, 45}, {90, -45, 65536}}},
            .accel_scale = {65491, 65626, 65491},
            .gyro_scale = {65581, 65491, 65581},
            .accel_bias_t0 = {-360, 540, -315},
            .gyro_bias_t0 = {225, -315, 270},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {90, -135, 45},
        .calibration_timestamp_epoch = 1735658000
    },
    /* Unit Serial: 1000146 */
    {
        .serial_number = 1000146,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 92, -138}, {-92, 65536, 46}, {138, -46, 65536}}},
            .gyro_misalignment = {.m = {{65536, -46, 92}, {46, 65536, -92}, {-92, 92, 65536}}},
            .accel_scale = {65582, 65490, 65628},
            .gyro_scale = {65490, 65582, 65444},
            .accel_bias_t0 = {460, -368, 276},
            .gyro_bias_t0 = {-184, 276, -230},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -92, 46}, {92, 65536, -138}, {-46, 138, 65536}}},
            .gyro_misalignment = {.m = {{65536, 138, -92}, {-138, 65536, 46}, {92, -46, 65536}}},
            .accel_scale = {65490, 65628, 65490},
            .gyro_scale = {65582, 65490, 65582},
            .accel_bias_t0 = {-368, 552, -322},
            .gyro_bias_t0 = {230, -322, 276},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {92, -138, 46},
        .calibration_timestamp_epoch = 1735744400
    },
    /* Unit Serial: 1000147 */
    {
        .serial_number = 1000147,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 94, -141}, {-94, 65536, 47}, {141, -47, 65536}}},
            .gyro_misalignment = {.m = {{65536, -47, 94}, {47, 65536, -94}, {-94, 94, 65536}}},
            .accel_scale = {65583, 65489, 65630},
            .gyro_scale = {65489, 65583, 65442},
            .accel_bias_t0 = {470, -376, 282},
            .gyro_bias_t0 = {-188, 282, -235},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -94, 47}, {94, 65536, -141}, {-47, 141, 65536}}},
            .gyro_misalignment = {.m = {{65536, 141, -94}, {-141, 65536, 47}, {94, -47, 65536}}},
            .accel_scale = {65489, 65630, 65489},
            .gyro_scale = {65583, 65489, 65583},
            .accel_bias_t0 = {-376, 564, -329},
            .gyro_bias_t0 = {235, -329, 282},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {94, -141, 47},
        .calibration_timestamp_epoch = 1735830800
    },
    /* Unit Serial: 1000148 */
    {
        .serial_number = 1000148,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 96, -144}, {-96, 65536, 48}, {144, -48, 65536}}},
            .gyro_misalignment = {.m = {{65536, -48, 96}, {48, 65536, -96}, {-96, 96, 65536}}},
            .accel_scale = {65584, 65488, 65632},
            .gyro_scale = {65488, 65584, 65440},
            .accel_bias_t0 = {480, -384, 288},
            .gyro_bias_t0 = {-192, 288, -240},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -96, 48}, {96, 65536, -144}, {-48, 144, 65536}}},
            .gyro_misalignment = {.m = {{65536, 144, -96}, {-144, 65536, 48}, {96, -48, 65536}}},
            .accel_scale = {65488, 65632, 65488},
            .gyro_scale = {65584, 65488, 65584},
            .accel_bias_t0 = {-384, 576, -336},
            .gyro_bias_t0 = {240, -336, 288},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {96, -144, 48},
        .calibration_timestamp_epoch = 1735917200
    },
    /* Unit Serial: 1000149 */
    {
        .serial_number = 1000149,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 98, -147}, {-98, 65536, 49}, {147, -49, 65536}}},
            .gyro_misalignment = {.m = {{65536, -49, 98}, {49, 65536, -98}, {-98, 98, 65536}}},
            .accel_scale = {65585, 65487, 65634},
            .gyro_scale = {65487, 65585, 65438},
            .accel_bias_t0 = {490, -392, 294},
            .gyro_bias_t0 = {-196, 294, -245},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -98, 49}, {98, 65536, -147}, {-49, 147, 65536}}},
            .gyro_misalignment = {.m = {{65536, 147, -98}, {-147, 65536, 49}, {98, -49, 65536}}},
            .accel_scale = {65487, 65634, 65487},
            .gyro_scale = {65585, 65487, 65585},
            .accel_bias_t0 = {-392, 588, -343},
            .gyro_bias_t0 = {245, -343, 294},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {98, -147, 49},
        .calibration_timestamp_epoch = 1736003600
    },
    /* Unit Serial: 1000150 */
    {
        .serial_number = 1000150,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 100, -150}, {-100, 65536, 50}, {150, -50, 65536}}},
            .gyro_misalignment = {.m = {{65536, -50, 100}, {50, 65536, -100}, {-100, 100, 65536}}},
            .accel_scale = {65586, 65486, 65636},
            .gyro_scale = {65486, 65586, 65436},
            .accel_bias_t0 = {500, -400, 300},
            .gyro_bias_t0 = {-200, 300, -250},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -100, 50}, {100, 65536, -150}, {-50, 150, 65536}}},
            .gyro_misalignment = {.m = {{65536, 150, -100}, {-150, 65536, 50}, {100, -50, 65536}}},
            .accel_scale = {65486, 65636, 65486},
            .gyro_scale = {65586, 65486, 65586},
            .accel_bias_t0 = {-400, 600, -350},
            .gyro_bias_t0 = {250, -350, 300},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {100, -150, 50},
        .calibration_timestamp_epoch = 1736090000
    },
    /* Unit Serial: 1000151 */
    {
        .serial_number = 1000151,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 102, -153}, {-102, 65536, 51}, {153, -51, 65536}}},
            .gyro_misalignment = {.m = {{65536, -51, 102}, {51, 65536, -102}, {-102, 102, 65536}}},
            .accel_scale = {65587, 65485, 65638},
            .gyro_scale = {65485, 65587, 65434},
            .accel_bias_t0 = {510, -408, 306},
            .gyro_bias_t0 = {-204, 306, -255},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -102, 51}, {102, 65536, -153}, {-51, 153, 65536}}},
            .gyro_misalignment = {.m = {{65536, 153, -102}, {-153, 65536, 51}, {102, -51, 65536}}},
            .accel_scale = {65485, 65638, 65485},
            .gyro_scale = {65587, 65485, 65587},
            .accel_bias_t0 = {-408, 612, -357},
            .gyro_bias_t0 = {255, -357, 306},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {102, -153, 51},
        .calibration_timestamp_epoch = 1736176400
    },
    /* Unit Serial: 1000152 */
    {
        .serial_number = 1000152,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 104, -156}, {-104, 65536, 52}, {156, -52, 65536}}},
            .gyro_misalignment = {.m = {{65536, -52, 104}, {52, 65536, -104}, {-104, 104, 65536}}},
            .accel_scale = {65588, 65484, 65640},
            .gyro_scale = {65484, 65588, 65432},
            .accel_bias_t0 = {520, -416, 312},
            .gyro_bias_t0 = {-208, 312, -260},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -104, 52}, {104, 65536, -156}, {-52, 156, 65536}}},
            .gyro_misalignment = {.m = {{65536, 156, -104}, {-156, 65536, 52}, {104, -52, 65536}}},
            .accel_scale = {65484, 65640, 65484},
            .gyro_scale = {65588, 65484, 65588},
            .accel_bias_t0 = {-416, 624, -364},
            .gyro_bias_t0 = {260, -364, 312},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {104, -156, 52},
        .calibration_timestamp_epoch = 1736262800
    },
    /* Unit Serial: 1000153 */
    {
        .serial_number = 1000153,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 106, -159}, {-106, 65536, 53}, {159, -53, 65536}}},
            .gyro_misalignment = {.m = {{65536, -53, 106}, {53, 65536, -106}, {-106, 106, 65536}}},
            .accel_scale = {65589, 65483, 65642},
            .gyro_scale = {65483, 65589, 65430},
            .accel_bias_t0 = {530, -424, 318},
            .gyro_bias_t0 = {-212, 318, -265},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -106, 53}, {106, 65536, -159}, {-53, 159, 65536}}},
            .gyro_misalignment = {.m = {{65536, 159, -106}, {-159, 65536, 53}, {106, -53, 65536}}},
            .accel_scale = {65483, 65642, 65483},
            .gyro_scale = {65589, 65483, 65589},
            .accel_bias_t0 = {-424, 636, -371},
            .gyro_bias_t0 = {265, -371, 318},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {106, -159, 53},
        .calibration_timestamp_epoch = 1736349200
    },
    /* Unit Serial: 1000154 */
    {
        .serial_number = 1000154,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 108, -162}, {-108, 65536, 54}, {162, -54, 65536}}},
            .gyro_misalignment = {.m = {{65536, -54, 108}, {54, 65536, -108}, {-108, 108, 65536}}},
            .accel_scale = {65590, 65482, 65644},
            .gyro_scale = {65482, 65590, 65428},
            .accel_bias_t0 = {540, -432, 324},
            .gyro_bias_t0 = {-216, 324, -270},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -108, 54}, {108, 65536, -162}, {-54, 162, 65536}}},
            .gyro_misalignment = {.m = {{65536, 162, -108}, {-162, 65536, 54}, {108, -54, 65536}}},
            .accel_scale = {65482, 65644, 65482},
            .gyro_scale = {65590, 65482, 65590},
            .accel_bias_t0 = {-432, 648, -378},
            .gyro_bias_t0 = {270, -378, 324},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {108, -162, 54},
        .calibration_timestamp_epoch = 1736435600
    },
    /* Unit Serial: 1000155 */
    {
        .serial_number = 1000155,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 110, -165}, {-110, 65536, 55}, {165, -55, 65536}}},
            .gyro_misalignment = {.m = {{65536, -55, 110}, {55, 65536, -110}, {-110, 110, 65536}}},
            .accel_scale = {65591, 65481, 65646},
            .gyro_scale = {65481, 65591, 65426},
            .accel_bias_t0 = {550, -440, 330},
            .gyro_bias_t0 = {-220, 330, -275},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -110, 55}, {110, 65536, -165}, {-55, 165, 65536}}},
            .gyro_misalignment = {.m = {{65536, 165, -110}, {-165, 65536, 55}, {110, -55, 65536}}},
            .accel_scale = {65481, 65646, 65481},
            .gyro_scale = {65591, 65481, 65591},
            .accel_bias_t0 = {-440, 660, -385},
            .gyro_bias_t0 = {275, -385, 330},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {110, -165, 55},
        .calibration_timestamp_epoch = 1736522000
    },
    /* Unit Serial: 1000156 */
    {
        .serial_number = 1000156,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 112, -168}, {-112, 65536, 56}, {168, -56, 65536}}},
            .gyro_misalignment = {.m = {{65536, -56, 112}, {56, 65536, -112}, {-112, 112, 65536}}},
            .accel_scale = {65592, 65480, 65648},
            .gyro_scale = {65480, 65592, 65424},
            .accel_bias_t0 = {560, -448, 336},
            .gyro_bias_t0 = {-224, 336, -280},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -112, 56}, {112, 65536, -168}, {-56, 168, 65536}}},
            .gyro_misalignment = {.m = {{65536, 168, -112}, {-168, 65536, 56}, {112, -56, 65536}}},
            .accel_scale = {65480, 65648, 65480},
            .gyro_scale = {65592, 65480, 65592},
            .accel_bias_t0 = {-448, 672, -392},
            .gyro_bias_t0 = {280, -392, 336},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {112, -168, 56},
        .calibration_timestamp_epoch = 1736608400
    },
    /* Unit Serial: 1000157 */
    {
        .serial_number = 1000157,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 114, -171}, {-114, 65536, 57}, {171, -57, 65536}}},
            .gyro_misalignment = {.m = {{65536, -57, 114}, {57, 65536, -114}, {-114, 114, 65536}}},
            .accel_scale = {65593, 65479, 65650},
            .gyro_scale = {65479, 65593, 65422},
            .accel_bias_t0 = {570, -456, 342},
            .gyro_bias_t0 = {-228, 342, -285},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -114, 57}, {114, 65536, -171}, {-57, 171, 65536}}},
            .gyro_misalignment = {.m = {{65536, 171, -114}, {-171, 65536, 57}, {114, -57, 65536}}},
            .accel_scale = {65479, 65650, 65479},
            .gyro_scale = {65593, 65479, 65593},
            .accel_bias_t0 = {-456, 684, -399},
            .gyro_bias_t0 = {285, -399, 342},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {114, -171, 57},
        .calibration_timestamp_epoch = 1736694800
    },
    /* Unit Serial: 1000158 */
    {
        .serial_number = 1000158,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 116, -174}, {-116, 65536, 58}, {174, -58, 65536}}},
            .gyro_misalignment = {.m = {{65536, -58, 116}, {58, 65536, -116}, {-116, 116, 65536}}},
            .accel_scale = {65594, 65478, 65652},
            .gyro_scale = {65478, 65594, 65420},
            .accel_bias_t0 = {580, -464, 348},
            .gyro_bias_t0 = {-232, 348, -290},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -116, 58}, {116, 65536, -174}, {-58, 174, 65536}}},
            .gyro_misalignment = {.m = {{65536, 174, -116}, {-174, 65536, 58}, {116, -58, 65536}}},
            .accel_scale = {65478, 65652, 65478},
            .gyro_scale = {65594, 65478, 65594},
            .accel_bias_t0 = {-464, 696, -406},
            .gyro_bias_t0 = {290, -406, 348},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {116, -174, 58},
        .calibration_timestamp_epoch = 1736781200
    },
    /* Unit Serial: 1000159 */
    {
        .serial_number = 1000159,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 118, -177}, {-118, 65536, 59}, {177, -59, 65536}}},
            .gyro_misalignment = {.m = {{65536, -59, 118}, {59, 65536, -118}, {-118, 118, 65536}}},
            .accel_scale = {65595, 65477, 65654},
            .gyro_scale = {65477, 65595, 65418},
            .accel_bias_t0 = {590, -472, 354},
            .gyro_bias_t0 = {-236, 354, -295},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -118, 59}, {118, 65536, -177}, {-59, 177, 65536}}},
            .gyro_misalignment = {.m = {{65536, 177, -118}, {-177, 65536, 59}, {118, -59, 65536}}},
            .accel_scale = {65477, 65654, 65477},
            .gyro_scale = {65595, 65477, 65595},
            .accel_bias_t0 = {-472, 708, -413},
            .gyro_bias_t0 = {295, -413, 354},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {118, -177, 59},
        .calibration_timestamp_epoch = 1736867600
    },
    /* Unit Serial: 1000160 */
    {
        .serial_number = 1000160,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 120, -180}, {-120, 65536, 60}, {180, -60, 65536}}},
            .gyro_misalignment = {.m = {{65536, -60, 120}, {60, 65536, -120}, {-120, 120, 65536}}},
            .accel_scale = {65596, 65476, 65656},
            .gyro_scale = {65476, 65596, 65416},
            .accel_bias_t0 = {600, -480, 360},
            .gyro_bias_t0 = {-240, 360, -300},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -120, 60}, {120, 65536, -180}, {-60, 180, 65536}}},
            .gyro_misalignment = {.m = {{65536, 180, -120}, {-180, 65536, 60}, {120, -60, 65536}}},
            .accel_scale = {65476, 65656, 65476},
            .gyro_scale = {65596, 65476, 65596},
            .accel_bias_t0 = {-480, 720, -420},
            .gyro_bias_t0 = {300, -420, 360},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {120, -180, 60},
        .calibration_timestamp_epoch = 1736954000
    },
    /* Unit Serial: 1000161 */
    {
        .serial_number = 1000161,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 122, -183}, {-122, 65536, 61}, {183, -61, 65536}}},
            .gyro_misalignment = {.m = {{65536, -61, 122}, {61, 65536, -122}, {-122, 122, 65536}}},
            .accel_scale = {65597, 65475, 65658},
            .gyro_scale = {65475, 65597, 65414},
            .accel_bias_t0 = {610, -488, 366},
            .gyro_bias_t0 = {-244, 366, -305},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -122, 61}, {122, 65536, -183}, {-61, 183, 65536}}},
            .gyro_misalignment = {.m = {{65536, 183, -122}, {-183, 65536, 61}, {122, -61, 65536}}},
            .accel_scale = {65475, 65658, 65475},
            .gyro_scale = {65597, 65475, 65597},
            .accel_bias_t0 = {-488, 732, -427},
            .gyro_bias_t0 = {305, -427, 366},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {122, -183, 61},
        .calibration_timestamp_epoch = 1737040400
    },
    /* Unit Serial: 1000162 */
    {
        .serial_number = 1000162,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 124, -186}, {-124, 65536, 62}, {186, -62, 65536}}},
            .gyro_misalignment = {.m = {{65536, -62, 124}, {62, 65536, -124}, {-124, 124, 65536}}},
            .accel_scale = {65598, 65474, 65660},
            .gyro_scale = {65474, 65598, 65412},
            .accel_bias_t0 = {620, -496, 372},
            .gyro_bias_t0 = {-248, 372, -310},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -124, 62}, {124, 65536, -186}, {-62, 186, 65536}}},
            .gyro_misalignment = {.m = {{65536, 186, -124}, {-186, 65536, 62}, {124, -62, 65536}}},
            .accel_scale = {65474, 65660, 65474},
            .gyro_scale = {65598, 65474, 65598},
            .accel_bias_t0 = {-496, 744, -434},
            .gyro_bias_t0 = {310, -434, 372},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {124, -186, 62},
        .calibration_timestamp_epoch = 1737126800
    },
    /* Unit Serial: 1000163 */
    {
        .serial_number = 1000163,
        .imu0_cal = {
            .accel_misalignment = {.m = {{65536, 126, -189}, {-126, 65536, 63}, {189, -63, 65536}}},
            .gyro_misalignment = {.m = {{65536, -63, 126}, {63, 65536, -126}, {-126, 126, 65536}}},
            .accel_scale = {65599, 65473, 65662},
            .gyro_scale = {65473, 65599, 65410},
            .accel_bias_t0 = {630, -504, 378},
            .gyro_bias_t0 = {-252, 378, -315},
            .accel_temp_coeff = {10, -5, 8},
            .gyro_temp_coeff = {-3, 4, -4},
            .reference_temp_c = 1638400
        },
        .imu1_cal = {
            .accel_misalignment = {.m = {{65536, -126, 63}, {126, 65536, -189}, {-63, 189, 65536}}},
            .gyro_misalignment = {.m = {{65536, 189, -126}, {-189, 65536, 63}, {126, -63, 65536}}},
            .accel_scale = {65473, 65662, 65473},
            .gyro_scale = {65599, 65473, 65599},
            .accel_bias_t0 = {-504, 756, -441},
            .gyro_bias_t0 = {315, -441, 378},
            .accel_temp_coeff = {12, -7, 9},
            .gyro_temp_coeff = {-4, 5, -3},
            .reference_temp_c = 1638400
        },
        .gyro_noise_density_rad_s_rthz = 15, /* 0.00023 rad/s/sqrt(Hz) */
        .gyro_bias_instability_rad_s = 4,   /* 0.00006 rad/s */
        .accel_noise_density_m_s2_rthz = 45, /* 0.00068 m/s^2/sqrt(Hz) */
        .encoder_eccentricity_error_rad = {126, -189, 63},
        .calibration_timestamp_epoch = 1737213200
    }
};

const factory_calibration_record_t* get_factory_calibration(uint32_t sn) {
    for (int i = 0; i < FACTORY_SERIALIZED_UNITS; i++) {
        if (g_factory_calibration_records[i].serial_number == sn) {
            return &g_factory_calibration_records[i];
        }
    }
    return &g_factory_calibration_records[0];
}
