/*
 * Gimbal Airframe Resonant Notch Filter Profiles
 * Pre-tuned structural anti-resonance filter configurations for 16 aerospace payloads.
 */

#include "sensors.h"

#define AIRFRAME_PROFILE_COUNT 16

typedef struct {
    const char *payload_name;
    q16_t resonant_frequencies_hz[NOTCH_STAGES_PER_AXIS];
    q16_t notch_bandwidths_hz[NOTCH_STAGES_PER_AXIS];
    q16_t damping_ratios[NOTCH_STAGES_PER_AXIS];
} airframe_notch_profile_t;

const airframe_notch_profile_t g_airframe_notch_profiles[AIRFRAME_PROFILE_COUNT] = {
    /* Profile 0: UAV_POD_LIGHTWEIGHT */
    {
        .payload_name = "UAV_POD_LIGHTWEIGHT",
        .resonant_frequencies_hz = {7864320, 14155776, 22806528, 33030144},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 1: UAV_POD_HEAVY_EOIR */
    {
        .payload_name = "UAV_POD_HEAVY_EOIR",
        .resonant_frequencies_hz = {9502720, 17104896, 27557888, 39911424},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 2: HELICOPTER_NOSE_TURRET */
    {
        .payload_name = "HELICOPTER_NOSE_TURRET",
        .resonant_frequencies_hz = {11141120, 20054016, 32309248, 46792704},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 3: HELICOPTER_MAST_MOUNT */
    {
        .payload_name = "HELICOPTER_MAST_MOUNT",
        .resonant_frequencies_hz = {12779520, 23003136, 37060608, 53673984},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 4: FIXED_WING_WINGTIP */
    {
        .payload_name = "FIXED_WING_WINGTIP",
        .resonant_frequencies_hz = {14417920, 25952256, 41811968, 60555264},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 5: FIXED_WING_BELLY_BAY */
    {
        .payload_name = "FIXED_WING_BELLY_BAY",
        .resonant_frequencies_hz = {16056320, 28901376, 46563328, 67436544},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 6: NAVAL_MAST_STABILIZED */
    {
        .payload_name = "NAVAL_MAST_STABILIZED",
        .resonant_frequencies_hz = {17694720, 31850496, 51314688, 74317824},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 7: NAVAL_DECK_TRACKER */
    {
        .payload_name = "NAVAL_DECK_TRACKER",
        .resonant_frequencies_hz = {19333120, 34799616, 56066048, 81199104},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 8: GROUND_ROVER_MAST */
    {
        .payload_name = "GROUND_ROVER_MAST",
        .resonant_frequencies_hz = {20971520, 37748736, 60817408, 88080384},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 9: GROUND_VEHICLE_RWS */
    {
        .payload_name = "GROUND_VEHICLE_RWS",
        .resonant_frequencies_hz = {22609920, 40697856, 65568768, 94961664},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 10: AEROSTAT_LONG_RANGE */
    {
        .payload_name = "AEROSTAT_LONG_RANGE",
        .resonant_frequencies_hz = {24248320, 43646976, 70320128, 101842944},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 11: SECURITY_PERIMETER_TOWER */
    {
        .payload_name = "SECURITY_PERIMETER_TOWER",
        .resonant_frequencies_hz = {25886720, 46596096, 75071488, 108724224},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 12: SMALL_SATELLITE_BUS */
    {
        .payload_name = "SMALL_SATELLITE_BUS",
        .resonant_frequencies_hz = {27525120, 49545216, 79822848, 115605504},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 13: SUBORBITAL_PAYLOAD_BAY */
    {
        .payload_name = "SUBORBITAL_PAYLOAD_BAY",
        .resonant_frequencies_hz = {29163520, 52494336, 84574208, 122486784},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 14: CRANE_SURVEY_PLATFORM */
    {
        .payload_name = "CRANE_SURVEY_PLATFORM",
        .resonant_frequencies_hz = {30801920, 55443456, 89325568, 129368064},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    },
    /* Profile 15: EXPERIMENTAL_BENCHTOP */
    {
        .payload_name = "EXPERIMENTAL_BENCHTOP",
        .resonant_frequencies_hz = {32440320, 58392576, 94076928, 136249344},
        .notch_bandwidths_hz = {1310720, 1638400, 1966080, 2621440}, /* 20, 25, 30, 40 Hz in Q16 */
        .damping_ratios = {6553, 4915, 3276, 2621} /* 0.1, 0.075, 0.05, 0.04 */
    }
};

const airframe_notch_profile_t* get_airframe_notch_profile(uint8_t profile_id) {
    if (profile_id >= AIRFRAME_PROFILE_COUNT) profile_id = 0;
    return &g_airframe_notch_profiles[profile_id];
}
