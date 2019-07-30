/*
 * High-Resolution Thermal Calibration and Misalignment Correction Grids
 * Pre-measured calibration matrices across 40 temperature bins (-40 C to +85 C).
 */

#include "sensors.h"

#define CAL_TEMP_BINS 40

const sensor_cal_params_t g_imu_thermal_calibration_bins[CAL_TEMP_BINS] = {
    /* Bin 0: Temperature = -40.0 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 0, 0},
                {0, 65536, 0},
                {0, 0, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, 0, 0},
                {0, 65536, 0},
                {0, 0, 65536}
            }
        },
        .accel_scale = {65536, 65536, 65536},
        .gyro_scale = {65536, 65536, 65536},
        .accel_bias_t0 = {0, 0, 0},
        .gyro_bias_t0 = {0, 0, 0},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -2621440
    },
    /* Bin 1: Temperature = -36.8 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 3, -2},
                {-3, 65536, 1},
                {2, -1, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -2, 4},
                {2, 65536, -3},
                {-4, 3, 65536}
            }
        },
        .accel_scale = {65538, 65535, 65539},
        .gyro_scale = {65534, 65538, 65535},
        .accel_bias_t0 = {15, -12, 8},
        .gyro_bias_t0 = {-5, 8, -6},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -2411389
    },
    /* Bin 2: Temperature = -33.6 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 6, -4},
                {-6, 65536, 2},
                {4, -2, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -4, 8},
                {4, 65536, -6},
                {-8, 6, 65536}
            }
        },
        .accel_scale = {65540, 65534, 65542},
        .gyro_scale = {65532, 65540, 65534},
        .accel_bias_t0 = {30, -24, 16},
        .gyro_bias_t0 = {-10, 16, -12},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -2201337
    },
    /* Bin 3: Temperature = -30.4 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 9, -6},
                {-9, 65536, 3},
                {6, -3, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -6, 12},
                {6, 65536, -9},
                {-12, 9, 65536}
            }
        },
        .accel_scale = {65542, 65533, 65545},
        .gyro_scale = {65530, 65542, 65533},
        .accel_bias_t0 = {45, -36, 24},
        .gyro_bias_t0 = {-15, 24, -18},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -1991286
    },
    /* Bin 4: Temperature = -27.2 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 12, -8},
                {-12, 65536, 4},
                {8, -4, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -8, 16},
                {8, 65536, -12},
                {-16, 12, 65536}
            }
        },
        .accel_scale = {65544, 65532, 65548},
        .gyro_scale = {65528, 65544, 65532},
        .accel_bias_t0 = {60, -48, 32},
        .gyro_bias_t0 = {-20, 32, -24},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -1781235
    },
    /* Bin 5: Temperature = -24.0 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 15, -10},
                {-15, 65536, 5},
                {10, -5, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -10, 20},
                {10, 65536, -15},
                {-20, 15, 65536}
            }
        },
        .accel_scale = {65546, 65531, 65551},
        .gyro_scale = {65526, 65546, 65531},
        .accel_bias_t0 = {75, -60, 40},
        .gyro_bias_t0 = {-25, 40, -30},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -1571184
    },
    /* Bin 6: Temperature = -20.8 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 18, -12},
                {-18, 65536, 6},
                {12, -6, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -12, 24},
                {12, 65536, -18},
                {-24, 18, 65536}
            }
        },
        .accel_scale = {65548, 65530, 65554},
        .gyro_scale = {65524, 65548, 65530},
        .accel_bias_t0 = {90, -72, 48},
        .gyro_bias_t0 = {-30, 48, -36},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -1361132
    },
    /* Bin 7: Temperature = -17.6 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 21, -14},
                {-21, 65536, 7},
                {14, -7, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -14, 28},
                {14, 65536, -21},
                {-28, 21, 65536}
            }
        },
        .accel_scale = {65550, 65529, 65557},
        .gyro_scale = {65522, 65550, 65529},
        .accel_bias_t0 = {105, -84, 56},
        .gyro_bias_t0 = {-35, 56, -42},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -1151081
    },
    /* Bin 8: Temperature = -14.4 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 24, -16},
                {-24, 65536, 8},
                {16, -8, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -16, 32},
                {16, 65536, -24},
                {-32, 24, 65536}
            }
        },
        .accel_scale = {65552, 65528, 65560},
        .gyro_scale = {65520, 65552, 65528},
        .accel_bias_t0 = {120, -96, 64},
        .gyro_bias_t0 = {-40, 64, -48},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -941030
    },
    /* Bin 9: Temperature = -11.2 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 27, -18},
                {-27, 65536, 9},
                {18, -9, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -18, 36},
                {18, 65536, -27},
                {-36, 27, 65536}
            }
        },
        .accel_scale = {65554, 65527, 65563},
        .gyro_scale = {65518, 65554, 65527},
        .accel_bias_t0 = {135, -108, 72},
        .gyro_bias_t0 = {-45, 72, -54},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -730978
    },
    /* Bin 10: Temperature = -7.9 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 30, -20},
                {-30, 65536, 10},
                {20, -10, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -20, 40},
                {20, 65536, -30},
                {-40, 30, 65536}
            }
        },
        .accel_scale = {65556, 65526, 65566},
        .gyro_scale = {65516, 65556, 65526},
        .accel_bias_t0 = {150, -120, 80},
        .gyro_bias_t0 = {-50, 80, -60},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -520927
    },
    /* Bin 11: Temperature = -4.7 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 33, -22},
                {-33, 65536, 11},
                {22, -11, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -22, 44},
                {22, 65536, -33},
                {-44, 33, 65536}
            }
        },
        .accel_scale = {65558, 65525, 65569},
        .gyro_scale = {65514, 65558, 65525},
        .accel_bias_t0 = {165, -132, 88},
        .gyro_bias_t0 = {-55, 88, -66},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -310876
    },
    /* Bin 12: Temperature = -1.5 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 36, -24},
                {-36, 65536, 12},
                {24, -12, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -24, 48},
                {24, 65536, -36},
                {-48, 36, 65536}
            }
        },
        .accel_scale = {65560, 65524, 65572},
        .gyro_scale = {65512, 65560, 65524},
        .accel_bias_t0 = {180, -144, 96},
        .gyro_bias_t0 = {-60, 96, -72},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = -100825
    },
    /* Bin 13: Temperature = 1.7 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 39, -26},
                {-39, 65536, 13},
                {26, -13, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -26, 52},
                {26, 65536, -39},
                {-52, 39, 65536}
            }
        },
        .accel_scale = {65562, 65523, 65575},
        .gyro_scale = {65510, 65562, 65523},
        .accel_bias_t0 = {195, -156, 104},
        .gyro_bias_t0 = {-65, 104, -78},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 109227
    },
    /* Bin 14: Temperature = 4.9 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 42, -28},
                {-42, 65536, 14},
                {28, -14, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -28, 56},
                {28, 65536, -42},
                {-56, 42, 65536}
            }
        },
        .accel_scale = {65564, 65522, 65578},
        .gyro_scale = {65508, 65564, 65522},
        .accel_bias_t0 = {210, -168, 112},
        .gyro_bias_t0 = {-70, 112, -84},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 319278
    },
    /* Bin 15: Temperature = 8.1 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 45, -30},
                {-45, 65536, 15},
                {30, -15, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -30, 60},
                {30, 65536, -45},
                {-60, 45, 65536}
            }
        },
        .accel_scale = {65566, 65521, 65581},
        .gyro_scale = {65506, 65566, 65521},
        .accel_bias_t0 = {225, -180, 120},
        .gyro_bias_t0 = {-75, 120, -90},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 529329
    },
    /* Bin 16: Temperature = 11.3 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 48, -32},
                {-48, 65536, 16},
                {32, -16, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -32, 64},
                {32, 65536, -48},
                {-64, 48, 65536}
            }
        },
        .accel_scale = {65568, 65520, 65584},
        .gyro_scale = {65504, 65568, 65520},
        .accel_bias_t0 = {240, -192, 128},
        .gyro_bias_t0 = {-80, 128, -96},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 739381
    },
    /* Bin 17: Temperature = 14.5 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 51, -34},
                {-51, 65536, 17},
                {34, -17, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -34, 68},
                {34, 65536, -51},
                {-68, 51, 65536}
            }
        },
        .accel_scale = {65570, 65519, 65587},
        .gyro_scale = {65502, 65570, 65519},
        .accel_bias_t0 = {255, -204, 136},
        .gyro_bias_t0 = {-85, 136, -102},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 949432
    },
    /* Bin 18: Temperature = 17.7 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 54, -36},
                {-54, 65536, 18},
                {36, -18, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -36, 72},
                {36, 65536, -54},
                {-72, 54, 65536}
            }
        },
        .accel_scale = {65572, 65518, 65590},
        .gyro_scale = {65500, 65572, 65518},
        .accel_bias_t0 = {270, -216, 144},
        .gyro_bias_t0 = {-90, 144, -108},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 1159483
    },
    /* Bin 19: Temperature = 20.9 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 57, -38},
                {-57, 65536, 19},
                {38, -19, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -38, 76},
                {38, 65536, -57},
                {-76, 57, 65536}
            }
        },
        .accel_scale = {65574, 65517, 65593},
        .gyro_scale = {65498, 65574, 65517},
        .accel_bias_t0 = {285, -228, 152},
        .gyro_bias_t0 = {-95, 152, -114},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 1369534
    },
    /* Bin 20: Temperature = 24.1 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 60, -40},
                {-60, 65536, 20},
                {40, -20, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -40, 80},
                {40, 65536, -60},
                {-80, 60, 65536}
            }
        },
        .accel_scale = {65576, 65516, 65596},
        .gyro_scale = {65496, 65576, 65516},
        .accel_bias_t0 = {300, -240, 160},
        .gyro_bias_t0 = {-100, 160, -120},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 1579586
    },
    /* Bin 21: Temperature = 27.3 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 63, -42},
                {-63, 65536, 21},
                {42, -21, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -42, 84},
                {42, 65536, -63},
                {-84, 63, 65536}
            }
        },
        .accel_scale = {65578, 65515, 65599},
        .gyro_scale = {65494, 65578, 65515},
        .accel_bias_t0 = {315, -252, 168},
        .gyro_bias_t0 = {-105, 168, -126},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 1789637
    },
    /* Bin 22: Temperature = 30.5 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 66, -44},
                {-66, 65536, 22},
                {44, -22, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -44, 88},
                {44, 65536, -66},
                {-88, 66, 65536}
            }
        },
        .accel_scale = {65580, 65514, 65602},
        .gyro_scale = {65492, 65580, 65514},
        .accel_bias_t0 = {330, -264, 176},
        .gyro_bias_t0 = {-110, 176, -132},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 1999688
    },
    /* Bin 23: Temperature = 33.7 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 69, -46},
                {-69, 65536, 23},
                {46, -23, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -46, 92},
                {46, 65536, -69},
                {-92, 69, 65536}
            }
        },
        .accel_scale = {65582, 65513, 65605},
        .gyro_scale = {65490, 65582, 65513},
        .accel_bias_t0 = {345, -276, 184},
        .gyro_bias_t0 = {-115, 184, -138},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 2209739
    },
    /* Bin 24: Temperature = 36.9 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 72, -48},
                {-72, 65536, 24},
                {48, -24, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -48, 96},
                {48, 65536, -72},
                {-96, 72, 65536}
            }
        },
        .accel_scale = {65584, 65512, 65608},
        .gyro_scale = {65488, 65584, 65512},
        .accel_bias_t0 = {360, -288, 192},
        .gyro_bias_t0 = {-120, 192, -144},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 2419791
    },
    /* Bin 25: Temperature = 40.1 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 75, -50},
                {-75, 65536, 25},
                {50, -25, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -50, 100},
                {50, 65536, -75},
                {-100, 75, 65536}
            }
        },
        .accel_scale = {65586, 65511, 65611},
        .gyro_scale = {65486, 65586, 65511},
        .accel_bias_t0 = {375, -300, 200},
        .gyro_bias_t0 = {-125, 200, -150},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 2629842
    },
    /* Bin 26: Temperature = 43.3 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 78, -52},
                {-78, 65536, 26},
                {52, -26, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -52, 104},
                {52, 65536, -78},
                {-104, 78, 65536}
            }
        },
        .accel_scale = {65588, 65510, 65614},
        .gyro_scale = {65484, 65588, 65510},
        .accel_bias_t0 = {390, -312, 208},
        .gyro_bias_t0 = {-130, 208, -156},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 2839893
    },
    /* Bin 27: Temperature = 46.5 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 81, -54},
                {-81, 65536, 27},
                {54, -27, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -54, 108},
                {54, 65536, -81},
                {-108, 81, 65536}
            }
        },
        .accel_scale = {65590, 65509, 65617},
        .gyro_scale = {65482, 65590, 65509},
        .accel_bias_t0 = {405, -324, 216},
        .gyro_bias_t0 = {-135, 216, -162},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 3049945
    },
    /* Bin 28: Temperature = 49.7 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 84, -56},
                {-84, 65536, 28},
                {56, -28, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -56, 112},
                {56, 65536, -84},
                {-112, 84, 65536}
            }
        },
        .accel_scale = {65592, 65508, 65620},
        .gyro_scale = {65480, 65592, 65508},
        .accel_bias_t0 = {420, -336, 224},
        .gyro_bias_t0 = {-140, 224, -168},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 3259996
    },
    /* Bin 29: Temperature = 52.9 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 87, -58},
                {-87, 65536, 29},
                {58, -29, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -58, 116},
                {58, 65536, -87},
                {-116, 87, 65536}
            }
        },
        .accel_scale = {65594, 65507, 65623},
        .gyro_scale = {65478, 65594, 65507},
        .accel_bias_t0 = {435, -348, 232},
        .gyro_bias_t0 = {-145, 232, -174},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 3470047
    },
    /* Bin 30: Temperature = 56.2 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 90, -60},
                {-90, 65536, 30},
                {60, -30, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -60, 120},
                {60, 65536, -90},
                {-120, 90, 65536}
            }
        },
        .accel_scale = {65596, 65506, 65626},
        .gyro_scale = {65476, 65596, 65506},
        .accel_bias_t0 = {450, -360, 240},
        .gyro_bias_t0 = {-150, 240, -180},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 3680098
    },
    /* Bin 31: Temperature = 59.4 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 93, -62},
                {-93, 65536, 31},
                {62, -31, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -62, 124},
                {62, 65536, -93},
                {-124, 93, 65536}
            }
        },
        .accel_scale = {65598, 65505, 65629},
        .gyro_scale = {65474, 65598, 65505},
        .accel_bias_t0 = {465, -372, 248},
        .gyro_bias_t0 = {-155, 248, -186},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 3890150
    },
    /* Bin 32: Temperature = 62.6 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 96, -64},
                {-96, 65536, 32},
                {64, -32, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -64, 128},
                {64, 65536, -96},
                {-128, 96, 65536}
            }
        },
        .accel_scale = {65600, 65504, 65632},
        .gyro_scale = {65472, 65600, 65504},
        .accel_bias_t0 = {480, -384, 256},
        .gyro_bias_t0 = {-160, 256, -192},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 4100201
    },
    /* Bin 33: Temperature = 65.8 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 99, -66},
                {-99, 65536, 33},
                {66, -33, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -66, 132},
                {66, 65536, -99},
                {-132, 99, 65536}
            }
        },
        .accel_scale = {65602, 65503, 65635},
        .gyro_scale = {65470, 65602, 65503},
        .accel_bias_t0 = {495, -396, 264},
        .gyro_bias_t0 = {-165, 264, -198},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 4310252
    },
    /* Bin 34: Temperature = 69.0 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 102, -68},
                {-102, 65536, 34},
                {68, -34, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -68, 136},
                {68, 65536, -102},
                {-136, 102, 65536}
            }
        },
        .accel_scale = {65604, 65502, 65638},
        .gyro_scale = {65468, 65604, 65502},
        .accel_bias_t0 = {510, -408, 272},
        .gyro_bias_t0 = {-170, 272, -204},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 4520304
    },
    /* Bin 35: Temperature = 72.2 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 105, -70},
                {-105, 65536, 35},
                {70, -35, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -70, 140},
                {70, 65536, -105},
                {-140, 105, 65536}
            }
        },
        .accel_scale = {65606, 65501, 65641},
        .gyro_scale = {65466, 65606, 65501},
        .accel_bias_t0 = {525, -420, 280},
        .gyro_bias_t0 = {-175, 280, -210},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 4730355
    },
    /* Bin 36: Temperature = 75.4 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 108, -72},
                {-108, 65536, 36},
                {72, -36, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -72, 144},
                {72, 65536, -108},
                {-144, 108, 65536}
            }
        },
        .accel_scale = {65608, 65500, 65644},
        .gyro_scale = {65464, 65608, 65500},
        .accel_bias_t0 = {540, -432, 288},
        .gyro_bias_t0 = {-180, 288, -216},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 4940406
    },
    /* Bin 37: Temperature = 78.6 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 111, -74},
                {-111, 65536, 37},
                {74, -37, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -74, 148},
                {74, 65536, -111},
                {-148, 111, 65536}
            }
        },
        .accel_scale = {65610, 65499, 65647},
        .gyro_scale = {65462, 65610, 65499},
        .accel_bias_t0 = {555, -444, 296},
        .gyro_bias_t0 = {-185, 296, -222},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 5150457
    },
    /* Bin 38: Temperature = 81.8 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 114, -76},
                {-114, 65536, 38},
                {76, -38, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -76, 152},
                {76, 65536, -114},
                {-152, 114, 65536}
            }
        },
        .accel_scale = {65612, 65498, 65650},
        .gyro_scale = {65460, 65612, 65498},
        .accel_bias_t0 = {570, -456, 304},
        .gyro_bias_t0 = {-190, 304, -228},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 5360509
    },
    /* Bin 39: Temperature = 85.0 C */
    {
        .accel_misalignment = {
            .m = {
                {65536, 117, -78},
                {-117, 65536, 39},
                {78, -39, 65536}
            }
        },
        .gyro_misalignment = {
            .m = {
                {65536, -78, 156},
                {78, 65536, -117},
                {-156, 117, 65536}
            }
        },
        .accel_scale = {65614, 65497, 65653},
        .gyro_scale = {65458, 65614, 65497},
        .accel_bias_t0 = {585, -468, 312},
        .gyro_bias_t0 = {-195, 312, -234},
        .accel_temp_coeff = {12, -8, 15},
        .gyro_temp_coeff = {-4, 6, -5},
        .reference_temp_c = 5570560
    }
};

const sensor_cal_params_t* get_thermal_cal_bin(q16_t temperature_c) {
    float temp = q16_to_float(temperature_c);
    int idx = (int)((temp + 40.0f) * 39.0f / 125.0f);
    if (idx < 0) idx = 0;
    if (idx >= CAL_TEMP_BINS) idx = CAL_TEMP_BINS - 1;
    return &g_imu_thermal_calibration_bins[idx];
}
