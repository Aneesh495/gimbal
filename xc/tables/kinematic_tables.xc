/*
 * Gimbal Mechanical Geometry and Kinematics Transformation Library
 * Pre-calculated arm link offset kinematics and inertia tensors for 32 aerospace gimbal airframes.
 */

#include "kinematics.h"

#define GIMBAL_GEOMETRY_PROFILES 32

typedef struct {
    const char *airframe_model;
    vec3_q16_t link_offsets_m[3];
    mat3_q16_t inertia_tensor_kg_m2[3];
    q16_t mass_kg[3];
    q16_t max_travel_rad[3];
    q16_t min_travel_rad[3];
} gimbal_geometry_profile_t;

const gimbal_geometry_profile_t g_gimbal_geometry_profiles[GIMBAL_GEOMETRY_PROFILES] = {
    /* Geometry Profile 0: AIRFRAME_SERIES_01 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V1",
        .link_offsets_m = {
            {2949, 983, 7864},
            {655, 5570, 2621},
            {327, 1310, 4259}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {81920, 55705, 29491},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 1: AIRFRAME_SERIES_02 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V2",
        .link_offsets_m = {
            {3096, 1032, 8257},
            {688, 5849, 2752},
            {344, 1376, 4472}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {86016, 58490, 30965},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 2: AIRFRAME_SERIES_03 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V3",
        .link_offsets_m = {
            {3244, 1081, 8650},
            {720, 6127, 2883},
            {360, 1441, 4685}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {90112, 61276, 32440},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 3: AIRFRAME_SERIES_04 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V4",
        .link_offsets_m = {
            {3391, 1130, 9043},
            {753, 6406, 3014},
            {376, 1507, 4898}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {94208, 64061, 33914},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 4: AIRFRAME_SERIES_05 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V5",
        .link_offsets_m = {
            {3538, 1179, 9437},
            {786, 6684, 3145},
            {393, 1572, 5111}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {98304, 66846, 35389},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 5: AIRFRAME_SERIES_06 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V6",
        .link_offsets_m = {
            {3686, 1228, 9830},
            {819, 6963, 3276},
            {409, 1638, 5324}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {102400, 69632, 36864},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 6: AIRFRAME_SERIES_07 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V7",
        .link_offsets_m = {
            {3833, 1277, 10223},
            {851, 7241, 3407},
            {425, 1703, 5537}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {106496, 72417, 38338},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 7: AIRFRAME_SERIES_08 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V8",
        .link_offsets_m = {
            {3981, 1327, 10616},
            {884, 7520, 3538},
            {442, 1769, 5750}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {110592, 75202, 39813},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 8: AIRFRAME_SERIES_09 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V9",
        .link_offsets_m = {
            {4128, 1376, 11010},
            {917, 7798, 3670},
            {458, 1835, 5963}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {114688, 77987, 41287},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 9: AIRFRAME_SERIES_10 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V10",
        .link_offsets_m = {
            {4276, 1425, 11403},
            {950, 8077, 3801},
            {475, 1900, 6176}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {118784, 80773, 42762},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 10: AIRFRAME_SERIES_11 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V11",
        .link_offsets_m = {
            {4423, 1474, 11796},
            {983, 8355, 3932},
            {491, 1966, 6389}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {122880, 83558, 44236},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 11: AIRFRAME_SERIES_12 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V12",
        .link_offsets_m = {
            {4571, 1523, 12189},
            {1015, 8634, 4063},
            {507, 2031, 6602}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {126976, 86343, 45711},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 12: AIRFRAME_SERIES_13 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V13",
        .link_offsets_m = {
            {4718, 1572, 12582},
            {1048, 8912, 4194},
            {524, 2097, 6815}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {131072, 89128, 47185},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 13: AIRFRAME_SERIES_14 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V14",
        .link_offsets_m = {
            {4866, 1622, 12976},
            {1081, 9191, 4325},
            {540, 2162, 7028}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {135168, 91914, 48660},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 14: AIRFRAME_SERIES_15 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V15",
        .link_offsets_m = {
            {5013, 1671, 13369},
            {1114, 9469, 4456},
            {557, 2228, 7241}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {139264, 94699, 50135},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 15: AIRFRAME_SERIES_16 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V16",
        .link_offsets_m = {
            {5160, 1720, 13762},
            {1146, 9748, 4587},
            {573, 2293, 7454}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {143360, 97484, 51609},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 16: AIRFRAME_SERIES_17 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V17",
        .link_offsets_m = {
            {5308, 1769, 14155},
            {1179, 10027, 4718},
            {589, 2359, 7667}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {147456, 100270, 53084},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 17: AIRFRAME_SERIES_18 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V18",
        .link_offsets_m = {
            {5455, 1818, 14548},
            {1212, 10305, 4849},
            {606, 2424, 7880}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {151552, 103055, 54558},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 18: AIRFRAME_SERIES_19 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V19",
        .link_offsets_m = {
            {5603, 1867, 14942},
            {1245, 10584, 4980},
            {622, 2490, 8093}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {155648, 105840, 56033},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 19: AIRFRAME_SERIES_20 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V20",
        .link_offsets_m = {
            {5750, 1916, 15335},
            {1277, 10862, 5111},
            {638, 2555, 8306}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {159744, 108625, 57507},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 20: AIRFRAME_SERIES_21 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V21",
        .link_offsets_m = {
            {5898, 1966, 15728},
            {1310, 11141, 5242},
            {655, 2621, 8519}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {163840, 111411, 58982},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 21: AIRFRAME_SERIES_22 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V22",
        .link_offsets_m = {
            {6045, 2015, 16121},
            {1343, 11419, 5373},
            {671, 2686, 8732}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {167936, 114196, 60456},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 22: AIRFRAME_SERIES_23 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V23",
        .link_offsets_m = {
            {6193, 2064, 16515},
            {1376, 11698, 5505},
            {688, 2752, 8945}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {172032, 116981, 61931},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 23: AIRFRAME_SERIES_24 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V24",
        .link_offsets_m = {
            {6340, 2113, 16908},
            {1409, 11976, 5636},
            {704, 2818, 9158}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {176128, 119767, 63406},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 24: AIRFRAME_SERIES_25 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V25",
        .link_offsets_m = {
            {6488, 2162, 17301},
            {1441, 12255, 5767},
            {720, 2883, 9371}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {180224, 122552, 64880},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 25: AIRFRAME_SERIES_26 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V26",
        .link_offsets_m = {
            {6635, 2211, 17694},
            {1474, 12533, 5898},
            {737, 2949, 9584}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {184320, 125337, 66355},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 26: AIRFRAME_SERIES_27 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V27",
        .link_offsets_m = {
            {6782, 2260, 18087},
            {1507, 12812, 6029},
            {753, 3014, 9797}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {188416, 128122, 67829},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 27: AIRFRAME_SERIES_28 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V28",
        .link_offsets_m = {
            {6930, 2310, 18481},
            {1540, 13090, 6160},
            {770, 3080, 10010}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {192512, 130908, 69304},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 28: AIRFRAME_SERIES_29 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V29",
        .link_offsets_m = {
            {7077, 2359, 18874},
            {1572, 13369, 6291},
            {786, 3145, 10223}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {196608, 133693, 70778},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 29: AIRFRAME_SERIES_30 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V30",
        .link_offsets_m = {
            {7225, 2408, 19267},
            {1605, 13647, 6422},
            {802, 3211, 10436}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {200704, 136478, 72253},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 30: AIRFRAME_SERIES_31 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V31",
        .link_offsets_m = {
            {7372, 2457, 19660},
            {1638, 13926, 6553},
            {819, 3276, 10649}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {204800, 139264, 73728},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    },
    /* Geometry Profile 31: AIRFRAME_SERIES_32 */
    {
        .airframe_model = "GIMBAL_AIRFRAME_V32",
        .link_offsets_m = {
            {7520, 2506, 20054},
            {1671, 14204, 6684},
            {835, 3342, 10862}
        },
        .inertia_tensor_kg_m2 = {
            {.m = {{458, 12, 5}, {12, 512, 8}, {5, 8, 380}}},
            {.m = {{320, 8, 4}, {8, 360, 6}, {4, 6, 290}}},
            {.m = {{180, 4, 2}, {4, 195, 3}, {2, 3, 160}}}
        },
        .mass_kg = {208896, 142049, 75202},
        .max_travel_rad = {205887, 51472, 51472},
        .min_travel_rad = {-205887, -102944, -51472}
    }
};

const gimbal_geometry_profile_t* get_gimbal_geometry(uint8_t id) {
    if (id >= GIMBAL_GEOMETRY_PROFILES) id = 0;
    return &g_gimbal_geometry_profiles[id];
}
