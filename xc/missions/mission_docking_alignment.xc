/*
 * Gimbal Autonomous Mission Profile: Optical Docking and Precision Rendezvous
 * Sequences optical retroreflector tracking, millimeter-level boresight nulling,
 * and relative 6-DOF pose alignment across 400 precision approach frames.
 */

#include "kinematics.h"
#include "tracking.h"

#define DOCKING_APPROACH_FRAMES 400

typedef struct {
    q16_t range_to_dock_m;
    vec3_q16_t docking_port_offset_m;
    quat_q16_t docking_port_relative_attitude;
    vec3_q16_t commanded_gimbal_angles_rad;
    bool lock_confidence_ok;
} docking_approach_frame_t;

const docking_approach_frame_t g_docking_approach_frames[DOCKING_APPROACH_FRAMES] = {
    /* Frame 0: Distance=50.00m */
    {
        .range_to_dock_m = 3276800,
        .docking_port_offset_m = {3276800, 0, 13107},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {0, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 1: Distance=49.95m */
    {
        .range_to_dock_m = 3273525,
        .docking_port_offset_m = {3273525, 246, 13094},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {5, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 2: Distance=49.90m */
    {
        .range_to_dock_m = 3270253,
        .docking_port_offset_m = {3270253, 491, 13080},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {10, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 3: Distance=49.85m */
    {
        .range_to_dock_m = 3266984,
        .docking_port_offset_m = {3266984, 735, 13066},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {15, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 4: Distance=49.80m */
    {
        .range_to_dock_m = 3263719,
        .docking_port_offset_m = {3263719, 979, 13051},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {20, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 5: Distance=49.75m */
    {
        .range_to_dock_m = 3260457,
        .docking_port_offset_m = {3260457, 1222, 13036},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {25, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 6: Distance=49.70m */
    {
        .range_to_dock_m = 3257198,
        .docking_port_offset_m = {3257198, 1465, 13020},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {29, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 7: Distance=49.65m */
    {
        .range_to_dock_m = 3253942,
        .docking_port_offset_m = {3253942, 1708, 13004},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {34, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 8: Distance=49.60m */
    {
        .range_to_dock_m = 3250690,
        .docking_port_offset_m = {3250690, 1949, 12988},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {39, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 9: Distance=49.55m */
    {
        .range_to_dock_m = 3247441,
        .docking_port_offset_m = {3247441, 2190, 12971},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {44, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 10: Distance=49.50m */
    {
        .range_to_dock_m = 3244195,
        .docking_port_offset_m = {3244195, 2431, 12953},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {49, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 11: Distance=49.45m */
    {
        .range_to_dock_m = 3240953,
        .docking_port_offset_m = {3240953, 2671, 12936},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {54, -262, 0},
        .lock_confidence_ok = true
    },
    /* Frame 12: Distance=49.40m */
    {
        .range_to_dock_m = 3237713,
        .docking_port_offset_m = {3237713, 2910, 12917},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {59, -261, 0},
        .lock_confidence_ok = true
    },
    /* Frame 13: Distance=49.35m */
    {
        .range_to_dock_m = 3234477,
        .docking_port_offset_m = {3234477, 3149, 12899},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {64, -261, 0},
        .lock_confidence_ok = true
    },
    /* Frame 14: Distance=49.30m */
    {
        .range_to_dock_m = 3231244,
        .docking_port_offset_m = {3231244, 3387, 12879},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {69, -261, 0},
        .lock_confidence_ok = true
    },
    /* Frame 15: Distance=49.26m */
    {
        .range_to_dock_m = 3228015,
        .docking_port_offset_m = {3228015, 3624, 12860},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {74, -261, 0},
        .lock_confidence_ok = true
    },
    /* Frame 16: Distance=49.21m */
    {
        .range_to_dock_m = 3224788,
        .docking_port_offset_m = {3224788, 3860, 12840},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {78, -261, 0},
        .lock_confidence_ok = true
    },
    /* Frame 17: Distance=49.16m */
    {
        .range_to_dock_m = 3221565,
        .docking_port_offset_m = {3221565, 4096, 12819},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {83, -261, 0},
        .lock_confidence_ok = true
    },
    /* Frame 18: Distance=49.11m */
    {
        .range_to_dock_m = 3218345,
        .docking_port_offset_m = {3218345, 4332, 12798},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {88, -261, 0},
        .lock_confidence_ok = true
    },
    /* Frame 19: Distance=49.06m */
    {
        .range_to_dock_m = 3215129,
        .docking_port_offset_m = {3215129, 4566, 12777},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {93, -260, 0},
        .lock_confidence_ok = true
    },
    /* Frame 20: Distance=49.01m */
    {
        .range_to_dock_m = 3211915,
        .docking_port_offset_m = {3211915, 4800, 12755},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {98, -260, 0},
        .lock_confidence_ok = true
    },
    /* Frame 21: Distance=48.96m */
    {
        .range_to_dock_m = 3208705,
        .docking_port_offset_m = {3208705, 5033, 12733},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {103, -260, 0},
        .lock_confidence_ok = true
    },
    /* Frame 22: Distance=48.91m */
    {
        .range_to_dock_m = 3205498,
        .docking_port_offset_m = {3205498, 5265, 12710},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {108, -260, 0},
        .lock_confidence_ok = true
    },
    /* Frame 23: Distance=48.86m */
    {
        .range_to_dock_m = 3202294,
        .docking_port_offset_m = {3202294, 5497, 12687},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {112, -260, 0},
        .lock_confidence_ok = true
    },
    /* Frame 24: Distance=48.81m */
    {
        .range_to_dock_m = 3199093,
        .docking_port_offset_m = {3199093, 5727, 12664},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {117, -259, 0},
        .lock_confidence_ok = true
    },
    /* Frame 25: Distance=48.77m */
    {
        .range_to_dock_m = 3195896,
        .docking_port_offset_m = {3195896, 5957, 12640},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {122, -259, 0},
        .lock_confidence_ok = true
    },
    /* Frame 26: Distance=48.72m */
    {
        .range_to_dock_m = 3192701,
        .docking_port_offset_m = {3192701, 6186, 12616},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {127, -259, 0},
        .lock_confidence_ok = true
    },
    /* Frame 27: Distance=48.67m */
    {
        .range_to_dock_m = 3189510,
        .docking_port_offset_m = {3189510, 6415, 12591},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {132, -259, 0},
        .lock_confidence_ok = true
    },
    /* Frame 28: Distance=48.62m */
    {
        .range_to_dock_m = 3186322,
        .docking_port_offset_m = {3186322, 6642, 12566},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {137, -258, 0},
        .lock_confidence_ok = true
    },
    /* Frame 29: Distance=48.57m */
    {
        .range_to_dock_m = 3183137,
        .docking_port_offset_m = {3183137, 6869, 12540},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {141, -258, 0},
        .lock_confidence_ok = true
    },
    /* Frame 30: Distance=48.52m */
    {
        .range_to_dock_m = 3179956,
        .docking_port_offset_m = {3179956, 7095, 12514},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {146, -258, 0},
        .lock_confidence_ok = true
    },
    /* Frame 31: Distance=48.47m */
    {
        .range_to_dock_m = 3176778,
        .docking_port_offset_m = {3176778, 7320, 12488},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {151, -258, 0},
        .lock_confidence_ok = true
    },
    /* Frame 32: Distance=48.43m */
    {
        .range_to_dock_m = 3173602,
        .docking_port_offset_m = {3173602, 7544, 12461},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {156, -257, 0},
        .lock_confidence_ok = true
    },
    /* Frame 33: Distance=48.38m */
    {
        .range_to_dock_m = 3170430,
        .docking_port_offset_m = {3170430, 7767, 12434},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {161, -257, 0},
        .lock_confidence_ok = true
    },
    /* Frame 34: Distance=48.33m */
    {
        .range_to_dock_m = 3167262,
        .docking_port_offset_m = {3167262, 7989, 12406},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {165, -257, 0},
        .lock_confidence_ok = true
    },
    /* Frame 35: Distance=48.28m */
    {
        .range_to_dock_m = 3164096,
        .docking_port_offset_m = {3164096, 8211, 12378},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {170, -256, 0},
        .lock_confidence_ok = true
    },
    /* Frame 36: Distance=48.23m */
    {
        .range_to_dock_m = 3160933,
        .docking_port_offset_m = {3160933, 8431, 12350},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {175, -256, 0},
        .lock_confidence_ok = true
    },
    /* Frame 37: Distance=48.18m */
    {
        .range_to_dock_m = 3157774,
        .docking_port_offset_m = {3157774, 8651, 12321},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {180, -256, 0},
        .lock_confidence_ok = true
    },
    /* Frame 38: Distance=48.14m */
    {
        .range_to_dock_m = 3154618,
        .docking_port_offset_m = {3154618, 8869, 12292},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {184, -255, 0},
        .lock_confidence_ok = true
    },
    /* Frame 39: Distance=48.09m */
    {
        .range_to_dock_m = 3151465,
        .docking_port_offset_m = {3151465, 9087, 12262},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {189, -255, 0},
        .lock_confidence_ok = true
    },
    /* Frame 40: Distance=48.04m */
    {
        .range_to_dock_m = 3148315,
        .docking_port_offset_m = {3148315, 9304, 12232},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {194, -255, 0},
        .lock_confidence_ok = true
    },
    /* Frame 41: Distance=47.99m */
    {
        .range_to_dock_m = 3145168,
        .docking_port_offset_m = {3145168, 9520, 12202},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {198, -254, 0},
        .lock_confidence_ok = true
    },
    /* Frame 42: Distance=47.94m */
    {
        .range_to_dock_m = 3142024,
        .docking_port_offset_m = {3142024, 9735, 12171},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {203, -254, 0},
        .lock_confidence_ok = true
    },
    /* Frame 43: Distance=47.90m */
    {
        .range_to_dock_m = 3138884,
        .docking_port_offset_m = {3138884, 9948, 12140},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {208, -253, 0},
        .lock_confidence_ok = true
    },
    /* Frame 44: Distance=47.85m */
    {
        .range_to_dock_m = 3135747,
        .docking_port_offset_m = {3135747, 10161, 12108},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {212, -253, 0},
        .lock_confidence_ok = true
    },
    /* Frame 45: Distance=47.80m */
    {
        .range_to_dock_m = 3132613,
        .docking_port_offset_m = {3132613, 10373, 12076},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {217, -253, 0},
        .lock_confidence_ok = true
    },
    /* Frame 46: Distance=47.75m */
    {
        .range_to_dock_m = 3129482,
        .docking_port_offset_m = {3129482, 10584, 12044},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {222, -252, 0},
        .lock_confidence_ok = true
    },
    /* Frame 47: Distance=47.70m */
    {
        .range_to_dock_m = 3126354,
        .docking_port_offset_m = {3126354, 10794, 12011},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {226, -252, 0},
        .lock_confidence_ok = true
    },
    /* Frame 48: Distance=47.66m */
    {
        .range_to_dock_m = 3123229,
        .docking_port_offset_m = {3123229, 11002, 11978},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {231, -251, 0},
        .lock_confidence_ok = true
    },
    /* Frame 49: Distance=47.61m */
    {
        .range_to_dock_m = 3120107,
        .docking_port_offset_m = {3120107, 11210, 11945},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {235, -251, 0},
        .lock_confidence_ok = true
    },
    /* Frame 50: Distance=47.56m */
    {
        .range_to_dock_m = 3116989,
        .docking_port_offset_m = {3116989, 11417, 11911},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {240, -250, 0},
        .lock_confidence_ok = true
    },
    /* Frame 51: Distance=47.51m */
    {
        .range_to_dock_m = 3113873,
        .docking_port_offset_m = {3113873, 11622, 11877},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {245, -250, 0},
        .lock_confidence_ok = true
    },
    /* Frame 52: Distance=47.47m */
    {
        .range_to_dock_m = 3110761,
        .docking_port_offset_m = {3110761, 11827, 11842},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {249, -249, 0},
        .lock_confidence_ok = true
    },
    /* Frame 53: Distance=47.42m */
    {
        .range_to_dock_m = 3107652,
        .docking_port_offset_m = {3107652, 12030, 11807},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {254, -249, 0},
        .lock_confidence_ok = true
    },
    /* Frame 54: Distance=47.37m */
    {
        .range_to_dock_m = 3104546,
        .docking_port_offset_m = {3104546, 12232, 11772},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {258, -249, 0},
        .lock_confidence_ok = true
    },
    /* Frame 55: Distance=47.32m */
    {
        .range_to_dock_m = 3101443,
        .docking_port_offset_m = {3101443, 12434, 11736},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {263, -248, 0},
        .lock_confidence_ok = true
    },
    /* Frame 56: Distance=47.28m */
    {
        .range_to_dock_m = 3098343,
        .docking_port_offset_m = {3098343, 12634, 11700},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {267, -247, 0},
        .lock_confidence_ok = true
    },
    /* Frame 57: Distance=47.23m */
    {
        .range_to_dock_m = 3095246,
        .docking_port_offset_m = {3095246, 12833, 11664},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {272, -247, 0},
        .lock_confidence_ok = true
    },
    /* Frame 58: Distance=47.18m */
    {
        .range_to_dock_m = 3092152,
        .docking_port_offset_m = {3092152, 13031, 11627},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {276, -246, 0},
        .lock_confidence_ok = true
    },
    /* Frame 59: Distance=47.14m */
    {
        .range_to_dock_m = 3089062,
        .docking_port_offset_m = {3089062, 13227, 11590},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {281, -246, 0},
        .lock_confidence_ok = true
    },
    /* Frame 60: Distance=47.09m */
    {
        .range_to_dock_m = 3085974,
        .docking_port_offset_m = {3085974, 13423, 11553},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {285, -245, 0},
        .lock_confidence_ok = true
    },
    /* Frame 61: Distance=47.04m */
    {
        .range_to_dock_m = 3082890,
        .docking_port_offset_m = {3082890, 13617, 11515},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {289, -245, 0},
        .lock_confidence_ok = true
    },
    /* Frame 62: Distance=46.99m */
    {
        .range_to_dock_m = 3079808,
        .docking_port_offset_m = {3079808, 13811, 11477},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {294, -244, 0},
        .lock_confidence_ok = true
    },
    /* Frame 63: Distance=46.95m */
    {
        .range_to_dock_m = 3076730,
        .docking_port_offset_m = {3076730, 14003, 11438},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {298, -244, 0},
        .lock_confidence_ok = true
    },
    /* Frame 64: Distance=46.90m */
    {
        .range_to_dock_m = 3073655,
        .docking_port_offset_m = {3073655, 14193, 11399},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {303, -243, 0},
        .lock_confidence_ok = true
    },
    /* Frame 65: Distance=46.85m */
    {
        .range_to_dock_m = 3070583,
        .docking_port_offset_m = {3070583, 14383, 11360},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {307, -242, 0},
        .lock_confidence_ok = true
    },
    /* Frame 66: Distance=46.81m */
    {
        .range_to_dock_m = 3067514,
        .docking_port_offset_m = {3067514, 14572, 11320},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {311, -242, 0},
        .lock_confidence_ok = true
    },
    /* Frame 67: Distance=46.76m */
    {
        .range_to_dock_m = 3064448,
        .docking_port_offset_m = {3064448, 14759, 11281},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {316, -241, 0},
        .lock_confidence_ok = true
    },
    /* Frame 68: Distance=46.71m */
    {
        .range_to_dock_m = 3061385,
        .docking_port_offset_m = {3061385, 14945, 11240},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {320, -241, 0},
        .lock_confidence_ok = true
    },
    /* Frame 69: Distance=46.67m */
    {
        .range_to_dock_m = 3058325,
        .docking_port_offset_m = {3058325, 15130, 11200},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {324, -240, 0},
        .lock_confidence_ok = true
    },
    /* Frame 70: Distance=46.62m */
    {
        .range_to_dock_m = 3055268,
        .docking_port_offset_m = {3055268, 15313, 11159},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {328, -239, 0},
        .lock_confidence_ok = true
    },
    /* Frame 71: Distance=46.57m */
    {
        .range_to_dock_m = 3052214,
        .docking_port_offset_m = {3052214, 15496, 11118},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {333, -239, 0},
        .lock_confidence_ok = true
    },
    /* Frame 72: Distance=46.53m */
    {
        .range_to_dock_m = 3049164,
        .docking_port_offset_m = {3049164, 15677, 11076},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {337, -238, 0},
        .lock_confidence_ok = true
    },
    /* Frame 73: Distance=46.48m */
    {
        .range_to_dock_m = 3046116,
        .docking_port_offset_m = {3046116, 15857, 11034},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {341, -237, 0},
        .lock_confidence_ok = true
    },
    /* Frame 74: Distance=46.43m */
    {
        .range_to_dock_m = 3043071,
        .docking_port_offset_m = {3043071, 16035, 10992},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {345, -237, 0},
        .lock_confidence_ok = true
    },
    /* Frame 75: Distance=46.39m */
    {
        .range_to_dock_m = 3040030,
        .docking_port_offset_m = {3040030, 16213, 10950},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {350, -236, 0},
        .lock_confidence_ok = true
    },
    /* Frame 76: Distance=46.34m */
    {
        .range_to_dock_m = 3036991,
        .docking_port_offset_m = {3036991, 16389, 10907},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {354, -235, 0},
        .lock_confidence_ok = true
    },
    /* Frame 77: Distance=46.29m */
    {
        .range_to_dock_m = 3033956,
        .docking_port_offset_m = {3033956, 16563, 10864},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {358, -235, 0},
        .lock_confidence_ok = true
    },
    /* Frame 78: Distance=46.25m */
    {
        .range_to_dock_m = 3030923,
        .docking_port_offset_m = {3030923, 16737, 10820},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {362, -234, 0},
        .lock_confidence_ok = true
    },
    /* Frame 79: Distance=46.20m */
    {
        .range_to_dock_m = 3027894,
        .docking_port_offset_m = {3027894, 16909, 10776},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {366, -233, 0},
        .lock_confidence_ok = true
    },
    /* Frame 80: Distance=46.16m */
    {
        .range_to_dock_m = 3024868,
        .docking_port_offset_m = {3024868, 17080, 10732},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {370, -233, 0},
        .lock_confidence_ok = true
    },
    /* Frame 81: Distance=46.11m */
    {
        .range_to_dock_m = 3021844,
        .docking_port_offset_m = {3021844, 17249, 10688},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {374, -232, 0},
        .lock_confidence_ok = true
    },
    /* Frame 82: Distance=46.06m */
    {
        .range_to_dock_m = 3018824,
        .docking_port_offset_m = {3018824, 17417, 10643},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {378, -231, 0},
        .lock_confidence_ok = true
    },
    /* Frame 83: Distance=46.02m */
    {
        .range_to_dock_m = 3015807,
        .docking_port_offset_m = {3015807, 17584, 10598},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {382, -230, 0},
        .lock_confidence_ok = true
    },
    /* Frame 84: Distance=45.97m */
    {
        .range_to_dock_m = 3012792,
        .docking_port_offset_m = {3012792, 17750, 10553},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {386, -230, 0},
        .lock_confidence_ok = true
    },
    /* Frame 85: Distance=45.93m */
    {
        .range_to_dock_m = 3009781,
        .docking_port_offset_m = {3009781, 17914, 10507},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {390, -229, 0},
        .lock_confidence_ok = true
    },
    /* Frame 86: Distance=45.88m */
    {
        .range_to_dock_m = 3006773,
        .docking_port_offset_m = {3006773, 18077, 10461},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {394, -228, 0},
        .lock_confidence_ok = true
    },
    /* Frame 87: Distance=45.83m */
    {
        .range_to_dock_m = 3003768,
        .docking_port_offset_m = {3003768, 18238, 10415},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {398, -227, 0},
        .lock_confidence_ok = true
    },
    /* Frame 88: Distance=45.79m */
    {
        .range_to_dock_m = 3000765,
        .docking_port_offset_m = {3000765, 18398, 10368},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {402, -226, 0},
        .lock_confidence_ok = true
    },
    /* Frame 89: Distance=45.74m */
    {
        .range_to_dock_m = 2997766,
        .docking_port_offset_m = {2997766, 18557, 10322},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {406, -226, 0},
        .lock_confidence_ok = true
    },
    /* Frame 90: Distance=45.70m */
    {
        .range_to_dock_m = 2994770,
        .docking_port_offset_m = {2994770, 18714, 10275},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {410, -225, 0},
        .lock_confidence_ok = true
    },
    /* Frame 91: Distance=45.65m */
    {
        .range_to_dock_m = 2991776,
        .docking_port_offset_m = {2991776, 18870, 10227},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {413, -224, 0},
        .lock_confidence_ok = true
    },
    /* Frame 92: Distance=45.61m */
    {
        .range_to_dock_m = 2988786,
        .docking_port_offset_m = {2988786, 19025, 10180},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {417, -223, 0},
        .lock_confidence_ok = true
    },
    /* Frame 93: Distance=45.56m */
    {
        .range_to_dock_m = 2985799,
        .docking_port_offset_m = {2985799, 19178, 10132},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {421, -222, 0},
        .lock_confidence_ok = true
    },
    /* Frame 94: Distance=45.51m */
    {
        .range_to_dock_m = 2982815,
        .docking_port_offset_m = {2982815, 19330, 10083},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {425, -222, 0},
        .lock_confidence_ok = true
    },
    /* Frame 95: Distance=45.47m */
    {
        .range_to_dock_m = 2979833,
        .docking_port_offset_m = {2979833, 19480, 10035},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {428, -221, 0},
        .lock_confidence_ok = true
    },
    /* Frame 96: Distance=45.42m */
    {
        .range_to_dock_m = 2976855,
        .docking_port_offset_m = {2976855, 19629, 9986},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {432, -220, 0},
        .lock_confidence_ok = true
    },
    /* Frame 97: Distance=45.38m */
    {
        .range_to_dock_m = 2973880,
        .docking_port_offset_m = {2973880, 19776, 9937},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {436, -219, 0},
        .lock_confidence_ok = true
    },
    /* Frame 98: Distance=45.33m */
    {
        .range_to_dock_m = 2970907,
        .docking_port_offset_m = {2970907, 19923, 9888},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {439, -218, 0},
        .lock_confidence_ok = true
    },
    /* Frame 99: Distance=45.29m */
    {
        .range_to_dock_m = 2967938,
        .docking_port_offset_m = {2967938, 20067, 9838},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {443, -217, 0},
        .lock_confidence_ok = true
    },
    /* Frame 100: Distance=45.24m */
    {
        .range_to_dock_m = 2964971,
        .docking_port_offset_m = {2964971, 20210, 9788},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {447, -216, 0},
        .lock_confidence_ok = true
    },
    /* Frame 101: Distance=45.20m */
    {
        .range_to_dock_m = 2962008,
        .docking_port_offset_m = {2962008, 20352, 9738},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {450, -215, 0},
        .lock_confidence_ok = true
    },
    /* Frame 102: Distance=45.15m */
    {
        .range_to_dock_m = 2959047,
        .docking_port_offset_m = {2959047, 20492, 9688},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {454, -215, 0},
        .lock_confidence_ok = true
    },
    /* Frame 103: Distance=45.11m */
    {
        .range_to_dock_m = 2956090,
        .docking_port_offset_m = {2956090, 20631, 9637},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {457, -214, 0},
        .lock_confidence_ok = true
    },
    /* Frame 104: Distance=45.06m */
    {
        .range_to_dock_m = 2953135,
        .docking_port_offset_m = {2953135, 20769, 9586},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {461, -213, 0},
        .lock_confidence_ok = true
    },
    /* Frame 105: Distance=45.02m */
    {
        .range_to_dock_m = 2950183,
        .docking_port_offset_m = {2950183, 20905, 9535},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {464, -212, 0},
        .lock_confidence_ok = true
    },
    /* Frame 106: Distance=44.97m */
    {
        .range_to_dock_m = 2947235,
        .docking_port_offset_m = {2947235, 21039, 9484},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {468, -211, 0},
        .lock_confidence_ok = true
    },
    /* Frame 107: Distance=44.93m */
    {
        .range_to_dock_m = 2944289,
        .docking_port_offset_m = {2944289, 21172, 9432},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {471, -210, 0},
        .lock_confidence_ok = true
    },
    /* Frame 108: Distance=44.88m */
    {
        .range_to_dock_m = 2941346,
        .docking_port_offset_m = {2941346, 21304, 9380},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {475, -209, 0},
        .lock_confidence_ok = true
    },
    /* Frame 109: Distance=44.84m */
    {
        .range_to_dock_m = 2938406,
        .docking_port_offset_m = {2938406, 21434, 9328},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {478, -208, 0},
        .lock_confidence_ok = true
    },
    /* Frame 110: Distance=44.79m */
    {
        .range_to_dock_m = 2935469,
        .docking_port_offset_m = {2935469, 21562, 9276},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {481, -207, 0},
        .lock_confidence_ok = true
    },
    /* Frame 111: Distance=44.75m */
    {
        .range_to_dock_m = 2932535,
        .docking_port_offset_m = {2932535, 21690, 9223},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {485, -206, 0},
        .lock_confidence_ok = true
    },
    /* Frame 112: Distance=44.70m */
    {
        .range_to_dock_m = 2929604,
        .docking_port_offset_m = {2929604, 21815, 9171},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {488, -205, 0},
        .lock_confidence_ok = true
    },
    /* Frame 113: Distance=44.66m */
    {
        .range_to_dock_m = 2926676,
        .docking_port_offset_m = {2926676, 21939, 9118},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {491, -204, 0},
        .lock_confidence_ok = true
    },
    /* Frame 114: Distance=44.61m */
    {
        .range_to_dock_m = 2923751,
        .docking_port_offset_m = {2923751, 22062, 9064},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {495, -203, 0},
        .lock_confidence_ok = true
    },
    /* Frame 115: Distance=44.57m */
    {
        .range_to_dock_m = 2920829,
        .docking_port_offset_m = {2920829, 22183, 9011},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {498, -202, 0},
        .lock_confidence_ok = true
    },
    /* Frame 116: Distance=44.52m */
    {
        .range_to_dock_m = 2917909,
        .docking_port_offset_m = {2917909, 22302, 8957},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {501, -201, 0},
        .lock_confidence_ok = true
    },
    /* Frame 117: Distance=44.48m */
    {
        .range_to_dock_m = 2914993,
        .docking_port_offset_m = {2914993, 22420, 8903},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {504, -200, 0},
        .lock_confidence_ok = true
    },
    /* Frame 118: Distance=44.43m */
    {
        .range_to_dock_m = 2912079,
        .docking_port_offset_m = {2912079, 22537, 8849},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {507, -199, 0},
        .lock_confidence_ok = true
    },
    /* Frame 119: Distance=44.39m */
    {
        .range_to_dock_m = 2909169,
        .docking_port_offset_m = {2909169, 22652, 8794},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {510, -198, 0},
        .lock_confidence_ok = true
    },
    /* Frame 120: Distance=44.35m */
    {
        .range_to_dock_m = 2906261,
        .docking_port_offset_m = {2906261, 22766, 8740},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {513, -197, 0},
        .lock_confidence_ok = true
    },
    /* Frame 121: Distance=44.30m */
    {
        .range_to_dock_m = 2903356,
        .docking_port_offset_m = {2903356, 22877, 8685},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {516, -196, 0},
        .lock_confidence_ok = true
    },
    /* Frame 122: Distance=44.26m */
    {
        .range_to_dock_m = 2900454,
        .docking_port_offset_m = {2900454, 22988, 8630},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {519, -195, 0},
        .lock_confidence_ok = true
    },
    /* Frame 123: Distance=44.21m */
    {
        .range_to_dock_m = 2897555,
        .docking_port_offset_m = {2897555, 23097, 8575},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {522, -194, 0},
        .lock_confidence_ok = true
    },
    /* Frame 124: Distance=44.17m */
    {
        .range_to_dock_m = 2894659,
        .docking_port_offset_m = {2894659, 23204, 8519},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {525, -193, 0},
        .lock_confidence_ok = true
    },
    /* Frame 125: Distance=44.12m */
    {
        .range_to_dock_m = 2891766,
        .docking_port_offset_m = {2891766, 23310, 8463},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {528, -192, 0},
        .lock_confidence_ok = true
    },
    /* Frame 126: Distance=44.08m */
    {
        .range_to_dock_m = 2888876,
        .docking_port_offset_m = {2888876, 23414, 8408},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {531, -191, 0},
        .lock_confidence_ok = true
    },
    /* Frame 127: Distance=44.04m */
    {
        .range_to_dock_m = 2885988,
        .docking_port_offset_m = {2885988, 23517, 8352},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {534, -190, 0},
        .lock_confidence_ok = true
    },
    /* Frame 128: Distance=43.99m */
    {
        .range_to_dock_m = 2883104,
        .docking_port_offset_m = {2883104, 23618, 8295},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {537, -189, 0},
        .lock_confidence_ok = true
    },
    /* Frame 129: Distance=43.95m */
    {
        .range_to_dock_m = 2880222,
        .docking_port_offset_m = {2880222, 23718, 8239},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {540, -187, 0},
        .lock_confidence_ok = true
    },
    /* Frame 130: Distance=43.90m */
    {
        .range_to_dock_m = 2877343,
        .docking_port_offset_m = {2877343, 23816, 8182},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {542, -186, 0},
        .lock_confidence_ok = true
    },
    /* Frame 131: Distance=43.86m */
    {
        .range_to_dock_m = 2874467,
        .docking_port_offset_m = {2874467, 23912, 8125},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {545, -185, 0},
        .lock_confidence_ok = true
    },
    /* Frame 132: Distance=43.82m */
    {
        .range_to_dock_m = 2871594,
        .docking_port_offset_m = {2871594, 24007, 8068},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {548, -184, 0},
        .lock_confidence_ok = true
    },
    /* Frame 133: Distance=43.77m */
    {
        .range_to_dock_m = 2868724,
        .docking_port_offset_m = {2868724, 24101, 8011},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {551, -183, 0},
        .lock_confidence_ok = true
    },
    /* Frame 134: Distance=43.73m */
    {
        .range_to_dock_m = 2865857,
        .docking_port_offset_m = {2865857, 24192, 7954},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {553, -182, 0},
        .lock_confidence_ok = true
    },
    /* Frame 135: Distance=43.69m */
    {
        .range_to_dock_m = 2862992,
        .docking_port_offset_m = {2862992, 24283, 7896},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {556, -181, 0},
        .lock_confidence_ok = true
    },
    /* Frame 136: Distance=43.64m */
    {
        .range_to_dock_m = 2860131,
        .docking_port_offset_m = {2860131, 24371, 7838},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {558, -180, 0},
        .lock_confidence_ok = true
    },
    /* Frame 137: Distance=43.60m */
    {
        .range_to_dock_m = 2857272,
        .docking_port_offset_m = {2857272, 24459, 7780},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {561, -178, 0},
        .lock_confidence_ok = true
    },
    /* Frame 138: Distance=43.55m */
    {
        .range_to_dock_m = 2854416,
        .docking_port_offset_m = {2854416, 24544, 7722},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {564, -177, 0},
        .lock_confidence_ok = true
    },
    /* Frame 139: Distance=43.51m */
    {
        .range_to_dock_m = 2851563,
        .docking_port_offset_m = {2851563, 24628, 7664},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {566, -176, 0},
        .lock_confidence_ok = true
    },
    /* Frame 140: Distance=43.47m */
    {
        .range_to_dock_m = 2848713,
        .docking_port_offset_m = {2848713, 24710, 7606},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {568, -175, 0},
        .lock_confidence_ok = true
    },
    /* Frame 141: Distance=43.42m */
    {
        .range_to_dock_m = 2845866,
        .docking_port_offset_m = {2845866, 24791, 7547},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {571, -174, 0},
        .lock_confidence_ok = true
    },
    /* Frame 142: Distance=43.38m */
    {
        .range_to_dock_m = 2843021,
        .docking_port_offset_m = {2843021, 24870, 7488},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {573, -173, 0},
        .lock_confidence_ok = true
    },
    /* Frame 143: Distance=43.34m */
    {
        .range_to_dock_m = 2840180,
        .docking_port_offset_m = {2840180, 24948, 7429},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {576, -171, 0},
        .lock_confidence_ok = true
    },
    /* Frame 144: Distance=43.29m */
    {
        .range_to_dock_m = 2837341,
        .docking_port_offset_m = {2837341, 25024, 7370},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {578, -170, 0},
        .lock_confidence_ok = true
    },
    /* Frame 145: Distance=43.25m */
    {
        .range_to_dock_m = 2834505,
        .docking_port_offset_m = {2834505, 25099, 7311},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {580, -169, 0},
        .lock_confidence_ok = true
    },
    /* Frame 146: Distance=43.21m */
    {
        .range_to_dock_m = 2831672,
        .docking_port_offset_m = {2831672, 25172, 7252},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {583, -168, 0},
        .lock_confidence_ok = true
    },
    /* Frame 147: Distance=43.16m */
    {
        .range_to_dock_m = 2828842,
        .docking_port_offset_m = {2828842, 25243, 7192},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {585, -167, 0},
        .lock_confidence_ok = true
    },
    /* Frame 148: Distance=43.12m */
    {
        .range_to_dock_m = 2826014,
        .docking_port_offset_m = {2826014, 25313, 7132},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {587, -165, 0},
        .lock_confidence_ok = true
    },
    /* Frame 149: Distance=43.08m */
    {
        .range_to_dock_m = 2823190,
        .docking_port_offset_m = {2823190, 25381, 7073},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {589, -164, 0},
        .lock_confidence_ok = true
    },
    /* Frame 150: Distance=43.04m */
    {
        .range_to_dock_m = 2820368,
        .docking_port_offset_m = {2820368, 25447, 7013},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {591, -163, 0},
        .lock_confidence_ok = true
    },
    /* Frame 151: Distance=42.99m */
    {
        .range_to_dock_m = 2817549,
        .docking_port_offset_m = {2817549, 25512, 6953},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {593, -162, 0},
        .lock_confidence_ok = true
    },
    /* Frame 152: Distance=42.95m */
    {
        .range_to_dock_m = 2814733,
        .docking_port_offset_m = {2814733, 25576, 6892},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {595, -160, 0},
        .lock_confidence_ok = true
    },
    /* Frame 153: Distance=42.91m */
    {
        .range_to_dock_m = 2811919,
        .docking_port_offset_m = {2811919, 25637, 6832},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {598, -159, 0},
        .lock_confidence_ok = true
    },
    /* Frame 154: Distance=42.86m */
    {
        .range_to_dock_m = 2809109,
        .docking_port_offset_m = {2809109, 25698, 6771},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {600, -158, 0},
        .lock_confidence_ok = true
    },
    /* Frame 155: Distance=42.82m */
    {
        .range_to_dock_m = 2806301,
        .docking_port_offset_m = {2806301, 25756, 6711},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {601, -157, 0},
        .lock_confidence_ok = true
    },
    /* Frame 156: Distance=42.78m */
    {
        .range_to_dock_m = 2803496,
        .docking_port_offset_m = {2803496, 25813, 6650},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {603, -155, 0},
        .lock_confidence_ok = true
    },
    /* Frame 157: Distance=42.74m */
    {
        .range_to_dock_m = 2800694,
        .docking_port_offset_m = {2800694, 25869, 6589},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {605, -154, 0},
        .lock_confidence_ok = true
    },
    /* Frame 158: Distance=42.69m */
    {
        .range_to_dock_m = 2797895,
        .docking_port_offset_m = {2797895, 25922, 6528},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {607, -153, 0},
        .lock_confidence_ok = true
    },
    /* Frame 159: Distance=42.65m */
    {
        .range_to_dock_m = 2795098,
        .docking_port_offset_m = {2795098, 25975, 6467},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {609, -152, 0},
        .lock_confidence_ok = true
    },
    /* Frame 160: Distance=42.61m */
    {
        .range_to_dock_m = 2792305,
        .docking_port_offset_m = {2792305, 26025, 6406},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {611, -150, 0},
        .lock_confidence_ok = true
    },
    /* Frame 161: Distance=42.56m */
    {
        .range_to_dock_m = 2789514,
        .docking_port_offset_m = {2789514, 26074, 6344},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {613, -149, 0},
        .lock_confidence_ok = true
    },
    /* Frame 162: Distance=42.52m */
    {
        .range_to_dock_m = 2786726,
        .docking_port_offset_m = {2786726, 26122, 6283},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {614, -148, 0},
        .lock_confidence_ok = true
    },
    /* Frame 163: Distance=42.48m */
    {
        .range_to_dock_m = 2783940,
        .docking_port_offset_m = {2783940, 26168, 6221},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {616, -146, 0},
        .lock_confidence_ok = true
    },
    /* Frame 164: Distance=42.44m */
    {
        .range_to_dock_m = 2781158,
        .docking_port_offset_m = {2781158, 26212, 6160},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {618, -145, 0},
        .lock_confidence_ok = true
    },
    /* Frame 165: Distance=42.39m */
    {
        .range_to_dock_m = 2778378,
        .docking_port_offset_m = {2778378, 26255, 6098},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {619, -144, 0},
        .lock_confidence_ok = true
    },
    /* Frame 166: Distance=42.35m */
    {
        .range_to_dock_m = 2775601,
        .docking_port_offset_m = {2775601, 26296, 6036},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {621, -143, 0},
        .lock_confidence_ok = true
    },
    /* Frame 167: Distance=42.31m */
    {
        .range_to_dock_m = 2772827,
        .docking_port_offset_m = {2772827, 26335, 5974},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {622, -141, 0},
        .lock_confidence_ok = true
    },
    /* Frame 168: Distance=42.27m */
    {
        .range_to_dock_m = 2770055,
        .docking_port_offset_m = {2770055, 26373, 5912},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {624, -140, 0},
        .lock_confidence_ok = true
    },
    /* Frame 169: Distance=42.23m */
    {
        .range_to_dock_m = 2767287,
        .docking_port_offset_m = {2767287, 26410, 5850},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {625, -139, 0},
        .lock_confidence_ok = true
    },
    /* Frame 170: Distance=42.18m */
    {
        .range_to_dock_m = 2764521,
        .docking_port_offset_m = {2764521, 26445, 5787},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {627, -137, 0},
        .lock_confidence_ok = true
    },
    /* Frame 171: Distance=42.14m */
    {
        .range_to_dock_m = 2761758,
        .docking_port_offset_m = {2761758, 26478, 5725},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {628, -136, 0},
        .lock_confidence_ok = true
    },
    /* Frame 172: Distance=42.10m */
    {
        .range_to_dock_m = 2758997,
        .docking_port_offset_m = {2758997, 26509, 5663},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {630, -135, 0},
        .lock_confidence_ok = true
    },
    /* Frame 173: Distance=42.06m */
    {
        .range_to_dock_m = 2756240,
        .docking_port_offset_m = {2756240, 26539, 5600},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {631, -133, 0},
        .lock_confidence_ok = true
    },
    /* Frame 174: Distance=42.01m */
    {
        .range_to_dock_m = 2753485,
        .docking_port_offset_m = {2753485, 26568, 5537},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {632, -132, 0},
        .lock_confidence_ok = true
    },
    /* Frame 175: Distance=41.97m */
    {
        .range_to_dock_m = 2750733,
        .docking_port_offset_m = {2750733, 26595, 5475},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {634, -130, 0},
        .lock_confidence_ok = true
    },
    /* Frame 176: Distance=41.93m */
    {
        .range_to_dock_m = 2747983,
        .docking_port_offset_m = {2747983, 26620, 5412},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {635, -129, 0},
        .lock_confidence_ok = true
    },
    /* Frame 177: Distance=41.89m */
    {
        .range_to_dock_m = 2745237,
        .docking_port_offset_m = {2745237, 26644, 5349},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {636, -128, 0},
        .lock_confidence_ok = true
    },
    /* Frame 178: Distance=41.85m */
    {
        .range_to_dock_m = 2742493,
        .docking_port_offset_m = {2742493, 26666, 5286},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {637, -126, 0},
        .lock_confidence_ok = true
    },
    /* Frame 179: Distance=41.81m */
    {
        .range_to_dock_m = 2739752,
        .docking_port_offset_m = {2739752, 26687, 5223},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {638, -125, 0},
        .lock_confidence_ok = true
    },
    /* Frame 180: Distance=41.76m */
    {
        .range_to_dock_m = 2737013,
        .docking_port_offset_m = {2737013, 26706, 5160},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {639, -124, 0},
        .lock_confidence_ok = true
    },
    /* Frame 181: Distance=41.72m */
    {
        .range_to_dock_m = 2734278,
        .docking_port_offset_m = {2734278, 26723, 5097},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {640, -122, 0},
        .lock_confidence_ok = true
    },
    /* Frame 182: Distance=41.68m */
    {
        .range_to_dock_m = 2731545,
        .docking_port_offset_m = {2731545, 26739, 5034},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {642, -121, 0},
        .lock_confidence_ok = true
    },
    /* Frame 183: Distance=41.64m */
    {
        .range_to_dock_m = 2728815,
        .docking_port_offset_m = {2728815, 26753, 4971},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {642, -119, 0},
        .lock_confidence_ok = true
    },
    /* Frame 184: Distance=41.60m */
    {
        .range_to_dock_m = 2726087,
        .docking_port_offset_m = {2726087, 26766, 4907},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {643, -118, 0},
        .lock_confidence_ok = true
    },
    /* Frame 185: Distance=41.56m */
    {
        .range_to_dock_m = 2723363,
        .docking_port_offset_m = {2723363, 26777, 4844},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {644, -117, 0},
        .lock_confidence_ok = true
    },
    /* Frame 186: Distance=41.51m */
    {
        .range_to_dock_m = 2720641,
        .docking_port_offset_m = {2720641, 26787, 4780},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {645, -115, 0},
        .lock_confidence_ok = true
    },
    /* Frame 187: Distance=41.47m */
    {
        .range_to_dock_m = 2717921,
        .docking_port_offset_m = {2717921, 26795, 4717},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {646, -114, 0},
        .lock_confidence_ok = true
    },
    /* Frame 188: Distance=41.43m */
    {
        .range_to_dock_m = 2715205,
        .docking_port_offset_m = {2715205, 26802, 4654},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {647, -112, 0},
        .lock_confidence_ok = true
    },
    /* Frame 189: Distance=41.39m */
    {
        .range_to_dock_m = 2712491,
        .docking_port_offset_m = {2712491, 26807, 4590},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {648, -111, 0},
        .lock_confidence_ok = true
    },
    /* Frame 190: Distance=41.35m */
    {
        .range_to_dock_m = 2709780,
        .docking_port_offset_m = {2709780, 26810, 4526},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {648, -109, 0},
        .lock_confidence_ok = true
    },
    /* Frame 191: Distance=41.31m */
    {
        .range_to_dock_m = 2707071,
        .docking_port_offset_m = {2707071, 26812, 4463},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {649, -108, 0},
        .lock_confidence_ok = true
    },
    /* Frame 192: Distance=41.27m */
    {
        .range_to_dock_m = 2704366,
        .docking_port_offset_m = {2704366, 26813, 4399},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {650, -107, 0},
        .lock_confidence_ok = true
    },
    /* Frame 193: Distance=41.22m */
    {
        .range_to_dock_m = 2701663,
        .docking_port_offset_m = {2701663, 26812, 4335},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {650, -105, 0},
        .lock_confidence_ok = true
    },
    /* Frame 194: Distance=41.18m */
    {
        .range_to_dock_m = 2698962,
        .docking_port_offset_m = {2698962, 26809, 4272},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {651, -104, 0},
        .lock_confidence_ok = true
    },
    /* Frame 195: Distance=41.14m */
    {
        .range_to_dock_m = 2696265,
        .docking_port_offset_m = {2696265, 26805, 4208},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {651, -102, 0},
        .lock_confidence_ok = true
    },
    /* Frame 196: Distance=41.10m */
    {
        .range_to_dock_m = 2693570,
        .docking_port_offset_m = {2693570, 26799, 4144},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {652, -101, 0},
        .lock_confidence_ok = true
    },
    /* Frame 197: Distance=41.06m */
    {
        .range_to_dock_m = 2690877,
        .docking_port_offset_m = {2690877, 26792, 4080},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {652, -99, 0},
        .lock_confidence_ok = true
    },
    /* Frame 198: Distance=41.02m */
    {
        .range_to_dock_m = 2688188,
        .docking_port_offset_m = {2688188, 26783, 4016},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {653, -98, 0},
        .lock_confidence_ok = true
    },
    /* Frame 199: Distance=40.98m */
    {
        .range_to_dock_m = 2685501,
        .docking_port_offset_m = {2685501, 26773, 3952},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {653, -96, 0},
        .lock_confidence_ok = true
    },
    /* Frame 200: Distance=40.94m */
    {
        .range_to_dock_m = 2682817,
        .docking_port_offset_m = {2682817, 26761, 3889},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {654, -95, 0},
        .lock_confidence_ok = true
    },
    /* Frame 201: Distance=40.90m */
    {
        .range_to_dock_m = 2680135,
        .docking_port_offset_m = {2680135, 26748, 3825},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {654, -94, 0},
        .lock_confidence_ok = true
    },
    /* Frame 202: Distance=40.85m */
    {
        .range_to_dock_m = 2677457,
        .docking_port_offset_m = {2677457, 26733, 3761},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {654, -92, 0},
        .lock_confidence_ok = true
    },
    /* Frame 203: Distance=40.81m */
    {
        .range_to_dock_m = 2674781,
        .docking_port_offset_m = {2674781, 26717, 3697},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -91, 0},
        .lock_confidence_ok = true
    },
    /* Frame 204: Distance=40.77m */
    {
        .range_to_dock_m = 2672107,
        .docking_port_offset_m = {2672107, 26699, 3633},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -89, 0},
        .lock_confidence_ok = true
    },
    /* Frame 205: Distance=40.73m */
    {
        .range_to_dock_m = 2669436,
        .docking_port_offset_m = {2669436, 26680, 3569},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -88, 0},
        .lock_confidence_ok = true
    },
    /* Frame 206: Distance=40.69m */
    {
        .range_to_dock_m = 2666768,
        .docking_port_offset_m = {2666768, 26659, 3505},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -86, 0},
        .lock_confidence_ok = true
    },
    /* Frame 207: Distance=40.65m */
    {
        .range_to_dock_m = 2664103,
        .docking_port_offset_m = {2664103, 26637, 3441},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -85, 0},
        .lock_confidence_ok = true
    },
    /* Frame 208: Distance=40.61m */
    {
        .range_to_dock_m = 2661440,
        .docking_port_offset_m = {2661440, 26613, 3377},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -83, 0},
        .lock_confidence_ok = true
    },
    /* Frame 209: Distance=40.57m */
    {
        .range_to_dock_m = 2658780,
        .docking_port_offset_m = {2658780, 26588, 3313},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -82, 0},
        .lock_confidence_ok = true
    },
    /* Frame 210: Distance=40.53m */
    {
        .range_to_dock_m = 2656122,
        .docking_port_offset_m = {2656122, 26561, 3249},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -80, 0},
        .lock_confidence_ok = true
    },
    /* Frame 211: Distance=40.49m */
    {
        .range_to_dock_m = 2653468,
        .docking_port_offset_m = {2653468, 26533, 3185},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -79, 0},
        .lock_confidence_ok = true
    },
    /* Frame 212: Distance=40.45m */
    {
        .range_to_dock_m = 2650816,
        .docking_port_offset_m = {2650816, 26503, 3121},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -77, 0},
        .lock_confidence_ok = true
    },
    /* Frame 213: Distance=40.41m */
    {
        .range_to_dock_m = 2648166,
        .docking_port_offset_m = {2648166, 26472, 3057},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -76, 0},
        .lock_confidence_ok = true
    },
    /* Frame 214: Distance=40.37m */
    {
        .range_to_dock_m = 2645519,
        .docking_port_offset_m = {2645519, 26440, 2993},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -74, 0},
        .lock_confidence_ok = true
    },
    /* Frame 215: Distance=40.33m */
    {
        .range_to_dock_m = 2642875,
        .docking_port_offset_m = {2642875, 26406, 2930},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -73, 0},
        .lock_confidence_ok = true
    },
    /* Frame 216: Distance=40.29m */
    {
        .range_to_dock_m = 2640233,
        .docking_port_offset_m = {2640233, 26370, 2866},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {655, -71, 0},
        .lock_confidence_ok = true
    },
    /* Frame 217: Distance=40.25m */
    {
        .range_to_dock_m = 2637595,
        .docking_port_offset_m = {2637595, 26334, 2802},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {654, -70, 0},
        .lock_confidence_ok = true
    },
    /* Frame 218: Distance=40.21m */
    {
        .range_to_dock_m = 2634958,
        .docking_port_offset_m = {2634958, 26295, 2738},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {654, -68, 0},
        .lock_confidence_ok = true
    },
    /* Frame 219: Distance=40.17m */
    {
        .range_to_dock_m = 2632325,
        .docking_port_offset_m = {2632325, 26256, 2674},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {654, -67, 0},
        .lock_confidence_ok = true
    },
    /* Frame 220: Distance=40.13m */
    {
        .range_to_dock_m = 2629694,
        .docking_port_offset_m = {2629694, 26214, 2611},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {653, -65, 0},
        .lock_confidence_ok = true
    },
    /* Frame 221: Distance=40.09m */
    {
        .range_to_dock_m = 2627065,
        .docking_port_offset_m = {2627065, 26172, 2547},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {653, -64, 0},
        .lock_confidence_ok = true
    },
    /* Frame 222: Distance=40.05m */
    {
        .range_to_dock_m = 2624439,
        .docking_port_offset_m = {2624439, 26128, 2483},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {652, -62, 0},
        .lock_confidence_ok = true
    },
    /* Frame 223: Distance=40.01m */
    {
        .range_to_dock_m = 2621816,
        .docking_port_offset_m = {2621816, 26083, 2419},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {652, -60, 0},
        .lock_confidence_ok = true
    },
    /* Frame 224: Distance=39.97m */
    {
        .range_to_dock_m = 2619196,
        .docking_port_offset_m = {2619196, 26036, 2356},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {651, -59, 0},
        .lock_confidence_ok = true
    },
    /* Frame 225: Distance=39.93m */
    {
        .range_to_dock_m = 2616578,
        .docking_port_offset_m = {2616578, 25988, 2292},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {651, -57, 0},
        .lock_confidence_ok = true
    },
    /* Frame 226: Distance=39.89m */
    {
        .range_to_dock_m = 2613963,
        .docking_port_offset_m = {2613963, 25938, 2229},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {650, -56, 0},
        .lock_confidence_ok = true
    },
    /* Frame 227: Distance=39.85m */
    {
        .range_to_dock_m = 2611350,
        .docking_port_offset_m = {2611350, 25887, 2165},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {650, -54, 0},
        .lock_confidence_ok = true
    },
    /* Frame 228: Distance=39.81m */
    {
        .range_to_dock_m = 2608740,
        .docking_port_offset_m = {2608740, 25835, 2102},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {649, -53, 0},
        .lock_confidence_ok = true
    },
    /* Frame 229: Distance=39.77m */
    {
        .range_to_dock_m = 2606133,
        .docking_port_offset_m = {2606133, 25781, 2038},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {648, -51, 0},
        .lock_confidence_ok = true
    },
    /* Frame 230: Distance=39.73m */
    {
        .range_to_dock_m = 2603528,
        .docking_port_offset_m = {2603528, 25726, 1975},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {648, -50, 0},
        .lock_confidence_ok = true
    },
    /* Frame 231: Distance=39.69m */
    {
        .range_to_dock_m = 2600925,
        .docking_port_offset_m = {2600925, 25670, 1912},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {647, -48, 0},
        .lock_confidence_ok = true
    },
    /* Frame 232: Distance=39.65m */
    {
        .range_to_dock_m = 2598326,
        .docking_port_offset_m = {2598326, 25612, 1848},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {646, -47, 0},
        .lock_confidence_ok = true
    },
    /* Frame 233: Distance=39.61m */
    {
        .range_to_dock_m = 2595729,
        .docking_port_offset_m = {2595729, 25553, 1785},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {645, -45, 0},
        .lock_confidence_ok = true
    },
    /* Frame 234: Distance=39.57m */
    {
        .range_to_dock_m = 2593134,
        .docking_port_offset_m = {2593134, 25493, 1722},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {644, -44, 0},
        .lock_confidence_ok = true
    },
    /* Frame 235: Distance=39.53m */
    {
        .range_to_dock_m = 2590543,
        .docking_port_offset_m = {2590543, 25431, 1659},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {643, -42, 0},
        .lock_confidence_ok = true
    },
    /* Frame 236: Distance=39.49m */
    {
        .range_to_dock_m = 2587953,
        .docking_port_offset_m = {2587953, 25368, 1596},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {642, -40, 0},
        .lock_confidence_ok = true
    },
    /* Frame 237: Distance=39.45m */
    {
        .range_to_dock_m = 2585367,
        .docking_port_offset_m = {2585367, 25303, 1533},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {641, -39, 0},
        .lock_confidence_ok = true
    },
    /* Frame 238: Distance=39.41m */
    {
        .range_to_dock_m = 2582783,
        .docking_port_offset_m = {2582783, 25238, 1470},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {640, -37, 0},
        .lock_confidence_ok = true
    },
    /* Frame 239: Distance=39.37m */
    {
        .range_to_dock_m = 2580201,
        .docking_port_offset_m = {2580201, 25170, 1407},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {639, -36, 0},
        .lock_confidence_ok = true
    },
    /* Frame 240: Distance=39.33m */
    {
        .range_to_dock_m = 2577622,
        .docking_port_offset_m = {2577622, 25102, 1345},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {638, -34, 0},
        .lock_confidence_ok = true
    },
    /* Frame 241: Distance=39.29m */
    {
        .range_to_dock_m = 2575046,
        .docking_port_offset_m = {2575046, 25032, 1282},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {637, -33, 0},
        .lock_confidence_ok = true
    },
    /* Frame 242: Distance=39.25m */
    {
        .range_to_dock_m = 2572472,
        .docking_port_offset_m = {2572472, 24961, 1220},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {636, -31, 0},
        .lock_confidence_ok = true
    },
    /* Frame 243: Distance=39.21m */
    {
        .range_to_dock_m = 2569901,
        .docking_port_offset_m = {2569901, 24889, 1157},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {635, -30, 0},
        .lock_confidence_ok = true
    },
    /* Frame 244: Distance=39.17m */
    {
        .range_to_dock_m = 2567332,
        .docking_port_offset_m = {2567332, 24816, 1095},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {633, -28, 0},
        .lock_confidence_ok = true
    },
    /* Frame 245: Distance=39.14m */
    {
        .range_to_dock_m = 2564766,
        .docking_port_offset_m = {2564766, 24741, 1032},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {632, -26, 0},
        .lock_confidence_ok = true
    },
    /* Frame 246: Distance=39.10m */
    {
        .range_to_dock_m = 2562203,
        .docking_port_offset_m = {2562203, 24665, 970},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {631, -25, 0},
        .lock_confidence_ok = true
    },
    /* Frame 247: Distance=39.06m */
    {
        .range_to_dock_m = 2559642,
        .docking_port_offset_m = {2559642, 24587, 908},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {630, -23, 0},
        .lock_confidence_ok = true
    },
    /* Frame 248: Distance=39.02m */
    {
        .range_to_dock_m = 2557083,
        .docking_port_offset_m = {2557083, 24509, 846},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {628, -22, 0},
        .lock_confidence_ok = true
    },
    /* Frame 249: Distance=38.98m */
    {
        .range_to_dock_m = 2554528,
        .docking_port_offset_m = {2554528, 24429, 784},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {627, -20, 0},
        .lock_confidence_ok = true
    },
    /* Frame 250: Distance=38.94m */
    {
        .range_to_dock_m = 2551974,
        .docking_port_offset_m = {2551974, 24348, 722},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {625, -19, 0},
        .lock_confidence_ok = true
    },
    /* Frame 251: Distance=38.90m */
    {
        .range_to_dock_m = 2549424,
        .docking_port_offset_m = {2549424, 24266, 660},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {624, -17, 0},
        .lock_confidence_ok = true
    },
    /* Frame 252: Distance=38.86m */
    {
        .range_to_dock_m = 2546876,
        .docking_port_offset_m = {2546876, 24182, 599},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {622, -15, 0},
        .lock_confidence_ok = true
    },
    /* Frame 253: Distance=38.82m */
    {
        .range_to_dock_m = 2544330,
        .docking_port_offset_m = {2544330, 24097, 537},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {621, -14, 0},
        .lock_confidence_ok = true
    },
    /* Frame 254: Distance=38.78m */
    {
        .range_to_dock_m = 2541787,
        .docking_port_offset_m = {2541787, 24012, 476},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {619, -12, 0},
        .lock_confidence_ok = true
    },
    /* Frame 255: Distance=38.75m */
    {
        .range_to_dock_m = 2539246,
        .docking_port_offset_m = {2539246, 23924, 414},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {617, -11, 0},
        .lock_confidence_ok = true
    },
    /* Frame 256: Distance=38.71m */
    {
        .range_to_dock_m = 2536708,
        .docking_port_offset_m = {2536708, 23836, 353},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {616, -9, 0},
        .lock_confidence_ok = true
    },
    /* Frame 257: Distance=38.67m */
    {
        .range_to_dock_m = 2534173,
        .docking_port_offset_m = {2534173, 23747, 292},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {614, -8, 0},
        .lock_confidence_ok = true
    },
    /* Frame 258: Distance=38.63m */
    {
        .range_to_dock_m = 2531640,
        .docking_port_offset_m = {2531640, 23656, 231},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {612, -6, 0},
        .lock_confidence_ok = true
    },
    /* Frame 259: Distance=38.59m */
    {
        .range_to_dock_m = 2529110,
        .docking_port_offset_m = {2529110, 23564, 170},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {611, -4, 0},
        .lock_confidence_ok = true
    },
    /* Frame 260: Distance=38.55m */
    {
        .range_to_dock_m = 2526582,
        .docking_port_offset_m = {2526582, 23471, 109},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {609, -3, 0},
        .lock_confidence_ok = true
    },
    /* Frame 261: Distance=38.51m */
    {
        .range_to_dock_m = 2524057,
        .docking_port_offset_m = {2524057, 23377, 48},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {607, -1, 0},
        .lock_confidence_ok = true
    },
    /* Frame 262: Distance=38.48m */
    {
        .range_to_dock_m = 2521534,
        .docking_port_offset_m = {2521534, 23281, -12},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {605, 0, 0},
        .lock_confidence_ok = true
    },
    /* Frame 263: Distance=38.44m */
    {
        .range_to_dock_m = 2519013,
        .docking_port_offset_m = {2519013, 23185, -73},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {603, 2, 0},
        .lock_confidence_ok = true
    },
    /* Frame 264: Distance=38.40m */
    {
        .range_to_dock_m = 2516496,
        .docking_port_offset_m = {2516496, 23087, -133},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {601, 3, 0},
        .lock_confidence_ok = true
    },
    /* Frame 265: Distance=38.36m */
    {
        .range_to_dock_m = 2513980,
        .docking_port_offset_m = {2513980, 22989, -193},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {599, 5, 0},
        .lock_confidence_ok = true
    },
    /* Frame 266: Distance=38.32m */
    {
        .range_to_dock_m = 2511468,
        .docking_port_offset_m = {2511468, 22889, -253},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {597, 7, 0},
        .lock_confidence_ok = true
    },
    /* Frame 267: Distance=38.28m */
    {
        .range_to_dock_m = 2508958,
        .docking_port_offset_m = {2508958, 22788, -313},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {595, 8, 0},
        .lock_confidence_ok = true
    },
    /* Frame 268: Distance=38.25m */
    {
        .range_to_dock_m = 2506450,
        .docking_port_offset_m = {2506450, 22686, -373},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {593, 10, 0},
        .lock_confidence_ok = true
    },
    /* Frame 269: Distance=38.21m */
    {
        .range_to_dock_m = 2503945,
        .docking_port_offset_m = {2503945, 22582, -433},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {591, 11, 0},
        .lock_confidence_ok = true
    },
    /* Frame 270: Distance=38.17m */
    {
        .range_to_dock_m = 2501442,
        .docking_port_offset_m = {2501442, 22478, -492},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {589, 13, 0},
        .lock_confidence_ok = true
    },
    /* Frame 271: Distance=38.13m */
    {
        .range_to_dock_m = 2498942,
        .docking_port_offset_m = {2498942, 22373, -552},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {587, 14, 0},
        .lock_confidence_ok = true
    },
    /* Frame 272: Distance=38.09m */
    {
        .range_to_dock_m = 2496444,
        .docking_port_offset_m = {2496444, 22266, -611},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {585, 16, 0},
        .lock_confidence_ok = true
    },
    /* Frame 273: Distance=38.05m */
    {
        .range_to_dock_m = 2493949,
        .docking_port_offset_m = {2493949, 22159, -670},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {582, 18, 0},
        .lock_confidence_ok = true
    },
    /* Frame 274: Distance=38.02m */
    {
        .range_to_dock_m = 2491456,
        .docking_port_offset_m = {2491456, 22051, -729},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {580, 19, 0},
        .lock_confidence_ok = true
    },
    /* Frame 275: Distance=37.98m */
    {
        .range_to_dock_m = 2488966,
        .docking_port_offset_m = {2488966, 21941, -788},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {578, 21, 0},
        .lock_confidence_ok = true
    },
    /* Frame 276: Distance=37.94m */
    {
        .range_to_dock_m = 2486478,
        .docking_port_offset_m = {2486478, 21830, -846},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {575, 22, 0},
        .lock_confidence_ok = true
    },
    /* Frame 277: Distance=37.90m */
    {
        .range_to_dock_m = 2483993,
        .docking_port_offset_m = {2483993, 21719, -905},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {573, 24, 0},
        .lock_confidence_ok = true
    },
    /* Frame 278: Distance=37.86m */
    {
        .range_to_dock_m = 2481510,
        .docking_port_offset_m = {2481510, 21606, -963},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {571, 25, 0},
        .lock_confidence_ok = true
    },
    /* Frame 279: Distance=37.83m */
    {
        .range_to_dock_m = 2479030,
        .docking_port_offset_m = {2479030, 21492, -1022},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {568, 27, 0},
        .lock_confidence_ok = true
    },
    /* Frame 280: Distance=37.79m */
    {
        .range_to_dock_m = 2476552,
        .docking_port_offset_m = {2476552, 21378, -1080},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {566, 29, 0},
        .lock_confidence_ok = true
    },
    /* Frame 281: Distance=37.75m */
    {
        .range_to_dock_m = 2474077,
        .docking_port_offset_m = {2474077, 21262, -1138},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {563, 30, 0},
        .lock_confidence_ok = true
    },
    /* Frame 282: Distance=37.71m */
    {
        .range_to_dock_m = 2471604,
        .docking_port_offset_m = {2471604, 21146, -1195},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {561, 32, 0},
        .lock_confidence_ok = true
    },
    /* Frame 283: Distance=37.68m */
    {
        .range_to_dock_m = 2469134,
        .docking_port_offset_m = {2469134, 21028, -1253},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {558, 33, 0},
        .lock_confidence_ok = true
    },
    /* Frame 284: Distance=37.64m */
    {
        .range_to_dock_m = 2466666,
        .docking_port_offset_m = {2466666, 20909, -1310},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {556, 35, 0},
        .lock_confidence_ok = true
    },
    /* Frame 285: Distance=37.60m */
    {
        .range_to_dock_m = 2464200,
        .docking_port_offset_m = {2464200, 20790, -1368},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {553, 36, 0},
        .lock_confidence_ok = true
    },
    /* Frame 286: Distance=37.56m */
    {
        .range_to_dock_m = 2461737,
        .docking_port_offset_m = {2461737, 20669, -1425},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {550, 38, 0},
        .lock_confidence_ok = true
    },
    /* Frame 287: Distance=37.53m */
    {
        .range_to_dock_m = 2459277,
        .docking_port_offset_m = {2459277, 20548, -1482},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {548, 39, 0},
        .lock_confidence_ok = true
    },
    /* Frame 288: Distance=37.49m */
    {
        .range_to_dock_m = 2456819,
        .docking_port_offset_m = {2456819, 20426, -1539},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {545, 41, 0},
        .lock_confidence_ok = true
    },
    /* Frame 289: Distance=37.45m */
    {
        .range_to_dock_m = 2454363,
        .docking_port_offset_m = {2454363, 20302, -1595},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {542, 43, 0},
        .lock_confidence_ok = true
    },
    /* Frame 290: Distance=37.41m */
    {
        .range_to_dock_m = 2451910,
        .docking_port_offset_m = {2451910, 20178, -1652},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {539, 44, 0},
        .lock_confidence_ok = true
    },
    /* Frame 291: Distance=37.38m */
    {
        .range_to_dock_m = 2449459,
        .docking_port_offset_m = {2449459, 20053, -1708},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {537, 46, 0},
        .lock_confidence_ok = true
    },
    /* Frame 292: Distance=37.34m */
    {
        .range_to_dock_m = 2447011,
        .docking_port_offset_m = {2447011, 19927, -1764},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {534, 47, 0},
        .lock_confidence_ok = true
    },
    /* Frame 293: Distance=37.30m */
    {
        .range_to_dock_m = 2444565,
        .docking_port_offset_m = {2444565, 19800, -1820},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {531, 49, 0},
        .lock_confidence_ok = true
    },
    /* Frame 294: Distance=37.26m */
    {
        .range_to_dock_m = 2442122,
        .docking_port_offset_m = {2442122, 19672, -1876},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {528, 50, 0},
        .lock_confidence_ok = true
    },
    /* Frame 295: Distance=37.23m */
    {
        .range_to_dock_m = 2439681,
        .docking_port_offset_m = {2439681, 19544, -1931},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {525, 52, 0},
        .lock_confidence_ok = true
    },
    /* Frame 296: Distance=37.19m */
    {
        .range_to_dock_m = 2437243,
        .docking_port_offset_m = {2437243, 19414, -1987},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {522, 53, 0},
        .lock_confidence_ok = true
    },
    /* Frame 297: Distance=37.15m */
    {
        .range_to_dock_m = 2434807,
        .docking_port_offset_m = {2434807, 19284, -2042},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {519, 55, 0},
        .lock_confidence_ok = true
    },
    /* Frame 298: Distance=37.12m */
    {
        .range_to_dock_m = 2432373,
        .docking_port_offset_m = {2432373, 19153, -2097},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {516, 56, 0},
        .lock_confidence_ok = true
    },
    /* Frame 299: Distance=37.08m */
    {
        .range_to_dock_m = 2429942,
        .docking_port_offset_m = {2429942, 19021, -2152},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {513, 58, 0},
        .lock_confidence_ok = true
    },
    /* Frame 300: Distance=37.04m */
    {
        .range_to_dock_m = 2427513,
        .docking_port_offset_m = {2427513, 18888, -2206},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {510, 60, 0},
        .lock_confidence_ok = true
    },
    /* Frame 301: Distance=37.00m */
    {
        .range_to_dock_m = 2425087,
        .docking_port_offset_m = {2425087, 18754, -2261},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {507, 61, 0},
        .lock_confidence_ok = true
    },
    /* Frame 302: Distance=36.97m */
    {
        .range_to_dock_m = 2422663,
        .docking_port_offset_m = {2422663, 18620, -2315},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {504, 63, 0},
        .lock_confidence_ok = true
    },
    /* Frame 303: Distance=36.93m */
    {
        .range_to_dock_m = 2420242,
        .docking_port_offset_m = {2420242, 18484, -2369},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {501, 64, 0},
        .lock_confidence_ok = true
    },
    /* Frame 304: Distance=36.89m */
    {
        .range_to_dock_m = 2417822,
        .docking_port_offset_m = {2417822, 18348, -2423},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {497, 66, 0},
        .lock_confidence_ok = true
    },
    /* Frame 305: Distance=36.86m */
    {
        .range_to_dock_m = 2415406,
        .docking_port_offset_m = {2415406, 18212, -2476},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {494, 67, 0},
        .lock_confidence_ok = true
    },
    /* Frame 306: Distance=36.82m */
    {
        .range_to_dock_m = 2412992,
        .docking_port_offset_m = {2412992, 18074, -2530},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {491, 69, 0},
        .lock_confidence_ok = true
    },
    /* Frame 307: Distance=36.78m */
    {
        .range_to_dock_m = 2410580,
        .docking_port_offset_m = {2410580, 17936, -2583},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {488, 70, 0},
        .lock_confidence_ok = true
    },
    /* Frame 308: Distance=36.75m */
    {
        .range_to_dock_m = 2408171,
        .docking_port_offset_m = {2408171, 17797, -2636},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {484, 72, 0},
        .lock_confidence_ok = true
    },
    /* Frame 309: Distance=36.71m */
    {
        .range_to_dock_m = 2405764,
        .docking_port_offset_m = {2405764, 17657, -2689},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {481, 73, 0},
        .lock_confidence_ok = true
    },
    /* Frame 310: Distance=36.67m */
    {
        .range_to_dock_m = 2403359,
        .docking_port_offset_m = {2403359, 17516, -2742},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {478, 75, 0},
        .lock_confidence_ok = true
    },
    /* Frame 311: Distance=36.64m */
    {
        .range_to_dock_m = 2400957,
        .docking_port_offset_m = {2400957, 17375, -2794},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {474, 76, 0},
        .lock_confidence_ok = true
    },
    /* Frame 312: Distance=36.60m */
    {
        .range_to_dock_m = 2398557,
        .docking_port_offset_m = {2398557, 17233, -2846},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {471, 78, 0},
        .lock_confidence_ok = true
    },
    /* Frame 313: Distance=36.56m */
    {
        .range_to_dock_m = 2396160,
        .docking_port_offset_m = {2396160, 17090, -2898},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {467, 79, 0},
        .lock_confidence_ok = true
    },
    /* Frame 314: Distance=36.53m */
    {
        .range_to_dock_m = 2393765,
        .docking_port_offset_m = {2393765, 16947, -2950},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {464, 81, 0},
        .lock_confidence_ok = true
    },
    /* Frame 315: Distance=36.49m */
    {
        .range_to_dock_m = 2391372,
        .docking_port_offset_m = {2391372, 16803, -3002},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {460, 82, 0},
        .lock_confidence_ok = true
    },
    /* Frame 316: Distance=36.45m */
    {
        .range_to_dock_m = 2388982,
        .docking_port_offset_m = {2388982, 16658, -3053},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {457, 84, 0},
        .lock_confidence_ok = true
    },
    /* Frame 317: Distance=36.42m */
    {
        .range_to_dock_m = 2386594,
        .docking_port_offset_m = {2386594, 16512, -3104},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {453, 85, 0},
        .lock_confidence_ok = true
    },
    /* Frame 318: Distance=36.38m */
    {
        .range_to_dock_m = 2384209,
        .docking_port_offset_m = {2384209, 16366, -3155},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {450, 87, 0},
        .lock_confidence_ok = true
    },
    /* Frame 319: Distance=36.34m */
    {
        .range_to_dock_m = 2381826,
        .docking_port_offset_m = {2381826, 16220, -3206},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {446, 88, 0},
        .lock_confidence_ok = true
    },
    /* Frame 320: Distance=36.31m */
    {
        .range_to_dock_m = 2379445,
        .docking_port_offset_m = {2379445, 16072, -3257},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {443, 90, 0},
        .lock_confidence_ok = true
    },
    /* Frame 321: Distance=36.27m */
    {
        .range_to_dock_m = 2377067,
        .docking_port_offset_m = {2377067, 15924, -3307},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {439, 91, 0},
        .lock_confidence_ok = true
    },
    /* Frame 322: Distance=36.23m */
    {
        .range_to_dock_m = 2374691,
        .docking_port_offset_m = {2374691, 15776, -3357},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {435, 93, 0},
        .lock_confidence_ok = true
    },
    /* Frame 323: Distance=36.20m */
    {
        .range_to_dock_m = 2372318,
        .docking_port_offset_m = {2372318, 15627, -3407},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {432, 94, 0},
        .lock_confidence_ok = true
    },
    /* Frame 324: Distance=36.16m */
    {
        .range_to_dock_m = 2369946,
        .docking_port_offset_m = {2369946, 15477, -3456},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {428, 96, 0},
        .lock_confidence_ok = true
    },
    /* Frame 325: Distance=36.13m */
    {
        .range_to_dock_m = 2367578,
        .docking_port_offset_m = {2367578, 15326, -3506},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {424, 97, 0},
        .lock_confidence_ok = true
    },
    /* Frame 326: Distance=36.09m */
    {
        .range_to_dock_m = 2365211,
        .docking_port_offset_m = {2365211, 15175, -3555},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {420, 98, 0},
        .lock_confidence_ok = true
    },
    /* Frame 327: Distance=36.05m */
    {
        .range_to_dock_m = 2362847,
        .docking_port_offset_m = {2362847, 15024, -3604},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {417, 100, 0},
        .lock_confidence_ok = true
    },
    /* Frame 328: Distance=36.02m */
    {
        .range_to_dock_m = 2360486,
        .docking_port_offset_m = {2360486, 14872, -3653},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {413, 101, 0},
        .lock_confidence_ok = true
    },
    /* Frame 329: Distance=35.98m */
    {
        .range_to_dock_m = 2358126,
        .docking_port_offset_m = {2358126, 14719, -3701},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {409, 103, 0},
        .lock_confidence_ok = true
    },
    /* Frame 330: Distance=35.95m */
    {
        .range_to_dock_m = 2355769,
        .docking_port_offset_m = {2355769, 14566, -3749},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {405, 104, 0},
        .lock_confidence_ok = true
    },
    /* Frame 331: Distance=35.91m */
    {
        .range_to_dock_m = 2353415,
        .docking_port_offset_m = {2353415, 14412, -3797},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {401, 106, 0},
        .lock_confidence_ok = true
    },
    /* Frame 332: Distance=35.87m */
    {
        .range_to_dock_m = 2351062,
        .docking_port_offset_m = {2351062, 14258, -3845},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {397, 107, 0},
        .lock_confidence_ok = true
    },
    /* Frame 333: Distance=35.84m */
    {
        .range_to_dock_m = 2348713,
        .docking_port_offset_m = {2348713, 14103, -3893},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {394, 109, 0},
        .lock_confidence_ok = true
    },
    /* Frame 334: Distance=35.80m */
    {
        .range_to_dock_m = 2346365,
        .docking_port_offset_m = {2346365, 13948, -3940},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {390, 110, 0},
        .lock_confidence_ok = true
    },
    /* Frame 335: Distance=35.77m */
    {
        .range_to_dock_m = 2344020,
        .docking_port_offset_m = {2344020, 13792, -3987},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {386, 111, 0},
        .lock_confidence_ok = true
    },
    /* Frame 336: Distance=35.73m */
    {
        .range_to_dock_m = 2341677,
        .docking_port_offset_m = {2341677, 13636, -4034},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {382, 113, 0},
        .lock_confidence_ok = true
    },
    /* Frame 337: Distance=35.70m */
    {
        .range_to_dock_m = 2339336,
        .docking_port_offset_m = {2339336, 13480, -4080},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {378, 114, 0},
        .lock_confidence_ok = true
    },
    /* Frame 338: Distance=35.66m */
    {
        .range_to_dock_m = 2336998,
        .docking_port_offset_m = {2336998, 13323, -4127},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {374, 116, 0},
        .lock_confidence_ok = true
    },
    /* Frame 339: Distance=35.62m */
    {
        .range_to_dock_m = 2334662,
        .docking_port_offset_m = {2334662, 13165, -4173},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {370, 117, 0},
        .lock_confidence_ok = true
    },
    /* Frame 340: Distance=35.59m */
    {
        .range_to_dock_m = 2332329,
        .docking_port_offset_m = {2332329, 13007, -4218},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {365, 119, 0},
        .lock_confidence_ok = true
    },
    /* Frame 341: Distance=35.55m */
    {
        .range_to_dock_m = 2329998,
        .docking_port_offset_m = {2329998, 12849, -4264},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {361, 120, 0},
        .lock_confidence_ok = true
    },
    /* Frame 342: Distance=35.52m */
    {
        .range_to_dock_m = 2327669,
        .docking_port_offset_m = {2327669, 12690, -4309},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {357, 121, 0},
        .lock_confidence_ok = true
    },
    /* Frame 343: Distance=35.48m */
    {
        .range_to_dock_m = 2325342,
        .docking_port_offset_m = {2325342, 12531, -4354},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {353, 123, 0},
        .lock_confidence_ok = true
    },
    /* Frame 344: Distance=35.45m */
    {
        .range_to_dock_m = 2323018,
        .docking_port_offset_m = {2323018, 12371, -4399},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {349, 124, 0},
        .lock_confidence_ok = true
    },
    /* Frame 345: Distance=35.41m */
    {
        .range_to_dock_m = 2320696,
        .docking_port_offset_m = {2320696, 12211, -4444},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {345, 125, 0},
        .lock_confidence_ok = true
    },
    /* Frame 346: Distance=35.38m */
    {
        .range_to_dock_m = 2318377,
        .docking_port_offset_m = {2318377, 12050, -4488},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {341, 127, 0},
        .lock_confidence_ok = true
    },
    /* Frame 347: Distance=35.34m */
    {
        .range_to_dock_m = 2316060,
        .docking_port_offset_m = {2316060, 11890, -4532},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {336, 128, 0},
        .lock_confidence_ok = true
    },
    /* Frame 348: Distance=35.30m */
    {
        .range_to_dock_m = 2313745,
        .docking_port_offset_m = {2313745, 11729, -4576},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {332, 130, 0},
        .lock_confidence_ok = true
    },
    /* Frame 349: Distance=35.27m */
    {
        .range_to_dock_m = 2311432,
        .docking_port_offset_m = {2311432, 11567, -4620},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {328, 131, 0},
        .lock_confidence_ok = true
    },
    /* Frame 350: Distance=35.23m */
    {
        .range_to_dock_m = 2309122,
        .docking_port_offset_m = {2309122, 11405, -4663},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {324, 132, 0},
        .lock_confidence_ok = true
    },
    /* Frame 351: Distance=35.20m */
    {
        .range_to_dock_m = 2306814,
        .docking_port_offset_m = {2306814, 11243, -4706},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {319, 134, 0},
        .lock_confidence_ok = true
    },
    /* Frame 352: Distance=35.16m */
    {
        .range_to_dock_m = 2304508,
        .docking_port_offset_m = {2304508, 11081, -4749},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {315, 135, 0},
        .lock_confidence_ok = true
    },
    /* Frame 353: Distance=35.13m */
    {
        .range_to_dock_m = 2302205,
        .docking_port_offset_m = {2302205, 10918, -4791},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {311, 136, 0},
        .lock_confidence_ok = true
    },
    /* Frame 354: Distance=35.09m */
    {
        .range_to_dock_m = 2299904,
        .docking_port_offset_m = {2299904, 10755, -4834},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {306, 138, 0},
        .lock_confidence_ok = true
    },
    /* Frame 355: Distance=35.06m */
    {
        .range_to_dock_m = 2297605,
        .docking_port_offset_m = {2297605, 10591, -4876},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {302, 139, 0},
        .lock_confidence_ok = true
    },
    /* Frame 356: Distance=35.02m */
    {
        .range_to_dock_m = 2295309,
        .docking_port_offset_m = {2295309, 10428, -4917},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {298, 140, 0},
        .lock_confidence_ok = true
    },
    /* Frame 357: Distance=34.99m */
    {
        .range_to_dock_m = 2293015,
        .docking_port_offset_m = {2293015, 10264, -4959},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {293, 142, 0},
        .lock_confidence_ok = true
    },
    /* Frame 358: Distance=34.95m */
    {
        .range_to_dock_m = 2290723,
        .docking_port_offset_m = {2290723, 10100, -5000},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {289, 143, 0},
        .lock_confidence_ok = true
    },
    /* Frame 359: Distance=34.92m */
    {
        .range_to_dock_m = 2288433,
        .docking_port_offset_m = {2288433, 9935, -5041},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {285, 144, 0},
        .lock_confidence_ok = true
    },
    /* Frame 360: Distance=34.88m */
    {
        .range_to_dock_m = 2286146,
        .docking_port_offset_m = {2286146, 9771, -5082},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {280, 146, 0},
        .lock_confidence_ok = true
    },
    /* Frame 361: Distance=34.85m */
    {
        .range_to_dock_m = 2283861,
        .docking_port_offset_m = {2283861, 9606, -5122},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {276, 147, 0},
        .lock_confidence_ok = true
    },
    /* Frame 362: Distance=34.81m */
    {
        .range_to_dock_m = 2281578,
        .docking_port_offset_m = {2281578, 9441, -5162},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {271, 148, 0},
        .lock_confidence_ok = true
    },
    /* Frame 363: Distance=34.78m */
    {
        .range_to_dock_m = 2279298,
        .docking_port_offset_m = {2279298, 9275, -5202},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {267, 150, 0},
        .lock_confidence_ok = true
    },
    /* Frame 364: Distance=34.74m */
    {
        .range_to_dock_m = 2277019,
        .docking_port_offset_m = {2277019, 9110, -5242},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {262, 151, 0},
        .lock_confidence_ok = true
    },
    /* Frame 365: Distance=34.71m */
    {
        .range_to_dock_m = 2274744,
        .docking_port_offset_m = {2274744, 8944, -5281},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {258, 152, 0},
        .lock_confidence_ok = true
    },
    /* Frame 366: Distance=34.68m */
    {
        .range_to_dock_m = 2272470,
        .docking_port_offset_m = {2272470, 8778, -5320},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {253, 153, 0},
        .lock_confidence_ok = true
    },
    /* Frame 367: Distance=34.64m */
    {
        .range_to_dock_m = 2270199,
        .docking_port_offset_m = {2270199, 8612, -5359},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {249, 155, 0},
        .lock_confidence_ok = true
    },
    /* Frame 368: Distance=34.61m */
    {
        .range_to_dock_m = 2267930,
        .docking_port_offset_m = {2267930, 8446, -5397},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {244, 156, 0},
        .lock_confidence_ok = true
    },
    /* Frame 369: Distance=34.57m */
    {
        .range_to_dock_m = 2265663,
        .docking_port_offset_m = {2265663, 8279, -5435},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {239, 157, 0},
        .lock_confidence_ok = true
    },
    /* Frame 370: Distance=34.54m */
    {
        .range_to_dock_m = 2263398,
        .docking_port_offset_m = {2263398, 8113, -5473},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {235, 158, 0},
        .lock_confidence_ok = true
    },
    /* Frame 371: Distance=34.50m */
    {
        .range_to_dock_m = 2261136,
        .docking_port_offset_m = {2261136, 7946, -5511},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {230, 160, 0},
        .lock_confidence_ok = true
    },
    /* Frame 372: Distance=34.47m */
    {
        .range_to_dock_m = 2258876,
        .docking_port_offset_m = {2258876, 7779, -5548},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {226, 161, 0},
        .lock_confidence_ok = true
    },
    /* Frame 373: Distance=34.43m */
    {
        .range_to_dock_m = 2256618,
        .docking_port_offset_m = {2256618, 7613, -5586},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {221, 162, 0},
        .lock_confidence_ok = true
    },
    /* Frame 374: Distance=34.40m */
    {
        .range_to_dock_m = 2254363,
        .docking_port_offset_m = {2254363, 7446, -5622},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {216, 163, 0},
        .lock_confidence_ok = true
    },
    /* Frame 375: Distance=34.36m */
    {
        .range_to_dock_m = 2252110,
        .docking_port_offset_m = {2252110, 7278, -5659},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {212, 165, 0},
        .lock_confidence_ok = true
    },
    /* Frame 376: Distance=34.33m */
    {
        .range_to_dock_m = 2249859,
        .docking_port_offset_m = {2249859, 7111, -5695},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {207, 166, 0},
        .lock_confidence_ok = true
    },
    /* Frame 377: Distance=34.30m */
    {
        .range_to_dock_m = 2247610,
        .docking_port_offset_m = {2247610, 6944, -5731},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {202, 167, 0},
        .lock_confidence_ok = true
    },
    /* Frame 378: Distance=34.26m */
    {
        .range_to_dock_m = 2245363,
        .docking_port_offset_m = {2245363, 6777, -5767},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {198, 168, 0},
        .lock_confidence_ok = true
    },
    /* Frame 379: Distance=34.23m */
    {
        .range_to_dock_m = 2243119,
        .docking_port_offset_m = {2243119, 6609, -5802},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {193, 170, 0},
        .lock_confidence_ok = true
    },
    /* Frame 380: Distance=34.19m */
    {
        .range_to_dock_m = 2240877,
        .docking_port_offset_m = {2240877, 6442, -5837},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {188, 171, 0},
        .lock_confidence_ok = true
    },
    /* Frame 381: Distance=34.16m */
    {
        .range_to_dock_m = 2238637,
        .docking_port_offset_m = {2238637, 6275, -5872},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {184, 172, 0},
        .lock_confidence_ok = true
    },
    /* Frame 382: Distance=34.12m */
    {
        .range_to_dock_m = 2236400,
        .docking_port_offset_m = {2236400, 6107, -5907},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {179, 173, 0},
        .lock_confidence_ok = true
    },
    /* Frame 383: Distance=34.09m */
    {
        .range_to_dock_m = 2234165,
        .docking_port_offset_m = {2234165, 5940, -5941},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {174, 174, 0},
        .lock_confidence_ok = true
    },
    /* Frame 384: Distance=34.06m */
    {
        .range_to_dock_m = 2231931,
        .docking_port_offset_m = {2231931, 5772, -5975},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {169, 175, 0},
        .lock_confidence_ok = true
    },
    /* Frame 385: Distance=34.02m */
    {
        .range_to_dock_m = 2229701,
        .docking_port_offset_m = {2229701, 5605, -6009},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {165, 177, 0},
        .lock_confidence_ok = true
    },
    /* Frame 386: Distance=33.99m */
    {
        .range_to_dock_m = 2227472,
        .docking_port_offset_m = {2227472, 5437, -6042},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {160, 178, 0},
        .lock_confidence_ok = true
    },
    /* Frame 387: Distance=33.95m */
    {
        .range_to_dock_m = 2225246,
        .docking_port_offset_m = {2225246, 5270, -6075},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {155, 179, 0},
        .lock_confidence_ok = true
    },
    /* Frame 388: Distance=33.92m */
    {
        .range_to_dock_m = 2223022,
        .docking_port_offset_m = {2223022, 5102, -6108},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {150, 180, 0},
        .lock_confidence_ok = true
    },
    /* Frame 389: Distance=33.89m */
    {
        .range_to_dock_m = 2220800,
        .docking_port_offset_m = {2220800, 4935, -6140},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {146, 181, 0},
        .lock_confidence_ok = true
    },
    /* Frame 390: Distance=33.85m */
    {
        .range_to_dock_m = 2218580,
        .docking_port_offset_m = {2218580, 4768, -6173},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {141, 182, 0},
        .lock_confidence_ok = true
    },
    /* Frame 391: Distance=33.82m */
    {
        .range_to_dock_m = 2216362,
        .docking_port_offset_m = {2216362, 4601, -6205},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {136, 183, 0},
        .lock_confidence_ok = true
    },
    /* Frame 392: Distance=33.79m */
    {
        .range_to_dock_m = 2214147,
        .docking_port_offset_m = {2214147, 4433, -6236},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {131, 185, 0},
        .lock_confidence_ok = true
    },
    /* Frame 393: Distance=33.75m */
    {
        .range_to_dock_m = 2211934,
        .docking_port_offset_m = {2211934, 4266, -6268},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {126, 186, 0},
        .lock_confidence_ok = true
    },
    /* Frame 394: Distance=33.72m */
    {
        .range_to_dock_m = 2209723,
        .docking_port_offset_m = {2209723, 4099, -6299},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {122, 187, 0},
        .lock_confidence_ok = true
    },
    /* Frame 395: Distance=33.68m */
    {
        .range_to_dock_m = 2207515,
        .docking_port_offset_m = {2207515, 3932, -6329},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {117, 188, 0},
        .lock_confidence_ok = true
    },
    /* Frame 396: Distance=33.65m */
    {
        .range_to_dock_m = 2205308,
        .docking_port_offset_m = {2205308, 3766, -6360},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {112, 189, 0},
        .lock_confidence_ok = true
    },
    /* Frame 397: Distance=33.62m */
    {
        .range_to_dock_m = 2203104,
        .docking_port_offset_m = {2203104, 3599, -6390},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {107, 190, 0},
        .lock_confidence_ok = true
    },
    /* Frame 398: Distance=33.58m */
    {
        .range_to_dock_m = 2200902,
        .docking_port_offset_m = {2200902, 3432, -6420},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {102, 191, 0},
        .lock_confidence_ok = true
    },
    /* Frame 399: Distance=33.55m */
    {
        .range_to_dock_m = 2198702,
        .docking_port_offset_m = {2198702, 3266, -6449},
        .docking_port_relative_attitude = {65536, 0, 0, 0},
        .commanded_gimbal_angles_rad = {97, 192, 0},
        .lock_confidence_ok = true
    }
};

const docking_approach_frame_t* get_docking_frame(uint16_t index) {
    if (index >= DOCKING_APPROACH_FRAMES) index = DOCKING_APPROACH_FRAMES - 1;
    return &g_docking_approach_frames[index];
}
