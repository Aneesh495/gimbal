/*
 * Gimbal Autonomous Mission Profile: High-Dynamics Tactical Intercept and Lead Pursuit
 * Implements proportional navigation guidance (Pro-Nav), time-to-go estimation,
 * and predictive line-of-sight lead angle compensation across 500 engagement waypoints.
 */

#include "kinematics.h"
#include "tracking.h"

#define TACTICAL_INTERCEPT_WAYPOINTS 500

typedef struct {
    vec3_q16_t target_relative_pos_m;
    vec3_q16_t target_relative_vel_m_s;
    q16_t estimated_closing_velocity_m_s;
    q16_t commanded_los_rate_rad_s[3];
} tactical_intercept_step_t;

const tactical_intercept_step_t g_tactical_intercept_trajectory[TACTICAL_INTERCEPT_WAYPOINTS] = {
    /* Step 0: t=0.00s, Range=5000.0m */
    {
        .target_relative_pos_m = {327680000, 0, -9830400},
        .target_relative_vel_m_s = {-9830400, 1638400, 327680},
        .estimated_closing_velocity_m_s = 9971384,
        .commanded_los_rate_rad_s = {1147, -344, 0}
    },
    /* Step 1: t=0.02s, Range=4997.0m */
    {
        .target_relative_pos_m = {327483366, 130992, -9823846},
        .target_relative_vel_m_s = {-9830400, 1638348, 327680},
        .estimated_closing_velocity_m_s = 9971375,
        .commanded_los_rate_rad_s = {1150, -345, 0}
    },
    /* Step 2: t=0.04s, Range=4994.0m */
    {
        .target_relative_pos_m = {327286679, 261818, -9817293},
        .target_relative_vel_m_s = {-9830400, 1638190, 327680},
        .estimated_closing_velocity_m_s = 9971349,
        .commanded_los_rate_rad_s = {1154, -346, 0}
    },
    /* Step 3: t=0.06s, Range=4991.0m */
    {
        .target_relative_pos_m = {327089941, 392470, -9810739},
        .target_relative_vel_m_s = {-9830400, 1637928, 327680},
        .estimated_closing_velocity_m_s = 9971306,
        .commanded_los_rate_rad_s = {1157, -347, 0}
    },
    /* Step 4: t=0.08s, Range=4988.0m */
    {
        .target_relative_pos_m = {326893150, 522940, -9804186},
        .target_relative_vel_m_s = {-9830400, 1637561, 327680},
        .estimated_closing_velocity_m_s = 9971246,
        .commanded_los_rate_rad_s = {1160, -348, 0}
    },
    /* Step 5: t=0.10s, Range=4985.0m */
    {
        .target_relative_pos_m = {326696307, 653219, -9797632},
        .target_relative_vel_m_s = {-9830400, 1637089, 327680},
        .estimated_closing_velocity_m_s = 9971168,
        .commanded_los_rate_rad_s = {1163, -349, 0}
    },
    /* Step 6: t=0.12s, Range=4982.0m */
    {
        .target_relative_pos_m = {326499412, 783299, -9791078},
        .target_relative_vel_m_s = {-9830400, 1636513, 327680},
        .estimated_closing_velocity_m_s = 9971074,
        .commanded_los_rate_rad_s = {1166, -350, 0}
    },
    /* Step 7: t=0.14s, Range=4979.0m */
    {
        .target_relative_pos_m = {326302466, 913172, -9784525},
        .target_relative_vel_m_s = {-9830400, 1635832, 327680},
        .estimated_closing_velocity_m_s = 9970962,
        .commanded_los_rate_rad_s = {1169, -351, 0}
    },
    /* Step 8: t=0.16s, Range=4976.0m */
    {
        .target_relative_pos_m = {326105469, 1042829, -9777971},
        .target_relative_vel_m_s = {-9830400, 1635046, 327680},
        .estimated_closing_velocity_m_s = 9970833,
        .commanded_los_rate_rad_s = {1172, -352, 0}
    },
    /* Step 9: t=0.18s, Range=4973.0m */
    {
        .target_relative_pos_m = {325908420, 1172262, -9771418},
        .target_relative_vel_m_s = {-9830400, 1634155, 327680},
        .estimated_closing_velocity_m_s = 9970687,
        .commanded_los_rate_rad_s = {1175, -352, 0}
    },
    /* Step 10: t=0.20s, Range=4970.0m */
    {
        .target_relative_pos_m = {325711320, 1301463, -9764864},
        .target_relative_vel_m_s = {-9830400, 1633160, 327680},
        .estimated_closing_velocity_m_s = 9970524,
        .commanded_los_rate_rad_s = {1178, -353, 0}
    },
    /* Step 11: t=0.22s, Range=4967.0m */
    {
        .target_relative_pos_m = {325514169, 1430424, -9758310},
        .target_relative_vel_m_s = {-9830400, 1632060, 327680},
        .estimated_closing_velocity_m_s = 9970344,
        .commanded_los_rate_rad_s = {1180, -354, 0}
    },
    /* Step 12: t=0.24s, Range=4964.0m */
    {
        .target_relative_pos_m = {325316968, 1559136, -9751757},
        .target_relative_vel_m_s = {-9830400, 1630856, 327680},
        .estimated_closing_velocity_m_s = 9970147,
        .commanded_los_rate_rad_s = {1183, -355, 0}
    },
    /* Step 13: t=0.26s, Range=4961.0m */
    {
        .target_relative_pos_m = {325119716, 1687592, -9745203},
        .target_relative_vel_m_s = {-9830400, 1629548, 327680},
        .estimated_closing_velocity_m_s = 9969933,
        .commanded_los_rate_rad_s = {1186, -356, 0}
    },
    /* Step 14: t=0.28s, Range=4958.0m */
    {
        .target_relative_pos_m = {324922414, 1815783, -9738650},
        .target_relative_vel_m_s = {-9830400, 1628135, 327680},
        .estimated_closing_velocity_m_s = 9969702,
        .commanded_los_rate_rad_s = {1188, -356, 0}
    },
    /* Step 15: t=0.30s, Range=4955.0m */
    {
        .target_relative_pos_m = {324725063, 1943701, -9732096},
        .target_relative_vel_m_s = {-9830400, 1626618, 327680},
        .estimated_closing_velocity_m_s = 9969455,
        .commanded_los_rate_rad_s = {1191, -357, 0}
    },
    /* Step 16: t=0.32s, Range=4952.0m */
    {
        .target_relative_pos_m = {324527662, 2071338, -9725542},
        .target_relative_vel_m_s = {-9830400, 1624997, 327680},
        .estimated_closing_velocity_m_s = 9969190,
        .commanded_los_rate_rad_s = {1193, -358, 0}
    },
    /* Step 17: t=0.34s, Range=4949.0m */
    {
        .target_relative_pos_m = {324330211, 2198687, -9718989},
        .target_relative_vel_m_s = {-9830400, 1623271, 327680},
        .estimated_closing_velocity_m_s = 9968909,
        .commanded_los_rate_rad_s = {1195, -359, 0}
    },
    /* Step 18: t=0.36s, Range=4946.0m */
    {
        .target_relative_pos_m = {324132712, 2325738, -9712435},
        .target_relative_vel_m_s = {-9830400, 1621442, 327680},
        .estimated_closing_velocity_m_s = 9968611,
        .commanded_los_rate_rad_s = {1197, -359, 0}
    },
    /* Step 19: t=0.38s, Range=4943.0m */
    {
        .target_relative_pos_m = {323935164, 2452485, -9705882},
        .target_relative_vel_m_s = {-9830400, 1619510, 327680},
        .estimated_closing_velocity_m_s = 9968297,
        .commanded_los_rate_rad_s = {1199, -360, 0}
    },
    /* Step 20: t=0.40s, Range=4940.0m */
    {
        .target_relative_pos_m = {323737568, 2578919, -9699328},
        .target_relative_vel_m_s = {-9830400, 1617473, 327680},
        .estimated_closing_velocity_m_s = 9967967,
        .commanded_los_rate_rad_s = {1201, -360, 0}
    },
    /* Step 21: t=0.42s, Range=4937.0m */
    {
        .target_relative_pos_m = {323539924, 2705032, -9692774},
        .target_relative_vel_m_s = {-9830400, 1615333, 327680},
        .estimated_closing_velocity_m_s = 9967620,
        .commanded_los_rate_rad_s = {1203, -361, 0}
    },
    /* Step 22: t=0.44s, Range=4934.0m */
    {
        .target_relative_pos_m = {323342233, 2830817, -9686221},
        .target_relative_vel_m_s = {-9830400, 1613090, 327680},
        .estimated_closing_velocity_m_s = 9967256,
        .commanded_los_rate_rad_s = {1205, -362, 0}
    },
    /* Step 23: t=0.46s, Range=4931.0m */
    {
        .target_relative_pos_m = {323144494, 2956265, -9679667},
        .target_relative_vel_m_s = {-9830400, 1610743, 327680},
        .estimated_closing_velocity_m_s = 9966877,
        .commanded_los_rate_rad_s = {1207, -362, 0}
    },
    /* Step 24: t=0.48s, Range=4928.0m */
    {
        .target_relative_pos_m = {322946708, 3081369, -9673114},
        .target_relative_vel_m_s = {-9830400, 1608294, 327680},
        .estimated_closing_velocity_m_s = 9966481,
        .commanded_los_rate_rad_s = {1209, -363, 0}
    },
    /* Step 25: t=0.50s, Range=4925.0m */
    {
        .target_relative_pos_m = {322748876, 3206121, -9666560},
        .target_relative_vel_m_s = {-9830400, 1605741, 327680},
        .estimated_closing_velocity_m_s = 9966070,
        .commanded_los_rate_rad_s = {1210, -363, 0}
    },
    /* Step 26: t=0.52s, Range=4922.0m */
    {
        .target_relative_pos_m = {322550998, 3330513, -9660006},
        .target_relative_vel_m_s = {-9830400, 1603086, 327680},
        .estimated_closing_velocity_m_s = 9965642,
        .commanded_los_rate_rad_s = {1212, -364, 0}
    },
    /* Step 27: t=0.54s, Range=4919.0m */
    {
        .target_relative_pos_m = {322353074, 3454537, -9653453},
        .target_relative_vel_m_s = {-9830400, 1600328, 327680},
        .estimated_closing_velocity_m_s = 9965199,
        .commanded_los_rate_rad_s = {1214, -364, 0}
    },
    /* Step 28: t=0.56s, Range=4916.0m */
    {
        .target_relative_pos_m = {322155105, 3578186, -9646899},
        .target_relative_vel_m_s = {-9830400, 1597467, 327680},
        .estimated_closing_velocity_m_s = 9964740,
        .commanded_los_rate_rad_s = {1215, -364, 0}
    },
    /* Step 29: t=0.58s, Range=4913.0m */
    {
        .target_relative_pos_m = {321957091, 3701453, -9640346},
        .target_relative_vel_m_s = {-9830400, 1594505, 327680},
        .estimated_closing_velocity_m_s = 9964265,
        .commanded_los_rate_rad_s = {1216, -365, 0}
    },
    /* Step 30: t=0.60s, Range=4910.0m */
    {
        .target_relative_pos_m = {321759033, 3824328, -9633792},
        .target_relative_vel_m_s = {-9830400, 1591440, 327680},
        .estimated_closing_velocity_m_s = 9963775,
        .commanded_los_rate_rad_s = {1218, -365, 0}
    },
    /* Step 31: t=0.62s, Range=4907.0m */
    {
        .target_relative_pos_m = {321560932, 3946806, -9627238},
        .target_relative_vel_m_s = {-9830400, 1588274, 327680},
        .estimated_closing_velocity_m_s = 9963270,
        .commanded_los_rate_rad_s = {1219, -366, 0}
    },
    /* Step 32: t=0.64s, Range=4904.0m */
    {
        .target_relative_pos_m = {321362786, 4068878, -9620685},
        .target_relative_vel_m_s = {-9830400, 1585005, 327680},
        .estimated_closing_velocity_m_s = 9962750,
        .commanded_los_rate_rad_s = {1220, -366, 0}
    },
    /* Step 33: t=0.66s, Range=4901.0m */
    {
        .target_relative_pos_m = {321164598, 4190537, -9614131},
        .target_relative_vel_m_s = {-9830400, 1581636, 327680},
        .estimated_closing_velocity_m_s = 9962214,
        .commanded_los_rate_rad_s = {1221, -366, 0}
    },
    /* Step 34: t=0.68s, Range=4898.0m */
    {
        .target_relative_pos_m = {320966368, 4311776, -9607578},
        .target_relative_vel_m_s = {-9830400, 1578165, 327680},
        .estimated_closing_velocity_m_s = 9961664,
        .commanded_los_rate_rad_s = {1222, -367, 0}
    },
    /* Step 35: t=0.70s, Range=4895.0m */
    {
        .target_relative_pos_m = {320768095, 4432586, -9601024},
        .target_relative_vel_m_s = {-9830400, 1574593, 327680},
        .estimated_closing_velocity_m_s = 9961098,
        .commanded_los_rate_rad_s = {1223, -367, 0}
    },
    /* Step 36: t=0.72s, Range=4892.0m */
    {
        .target_relative_pos_m = {320569781, 4552961, -9594470},
        .target_relative_vel_m_s = {-9830400, 1570921, 327680},
        .estimated_closing_velocity_m_s = 9960519,
        .commanded_los_rate_rad_s = {1224, -367, 0}
    },
    /* Step 37: t=0.74s, Range=4889.0m */
    {
        .target_relative_pos_m = {320371427, 4672893, -9587917},
        .target_relative_vel_m_s = {-9830400, 1567147, 327680},
        .estimated_closing_velocity_m_s = 9959924,
        .commanded_los_rate_rad_s = {1224, -367, 0}
    },
    /* Step 38: t=0.76s, Range=4886.0m */
    {
        .target_relative_pos_m = {320173032, 4792374, -9581363},
        .target_relative_vel_m_s = {-9830400, 1563274, 327680},
        .estimated_closing_velocity_m_s = 9959315,
        .commanded_los_rate_rad_s = {1225, -368, 0}
    },
    /* Step 39: t=0.78s, Range=4883.0m */
    {
        .target_relative_pos_m = {319974597, 4911399, -9574810},
        .target_relative_vel_m_s = {-9830400, 1559301, 327680},
        .estimated_closing_velocity_m_s = 9958693,
        .commanded_los_rate_rad_s = {1226, -368, 0}
    },
    /* Step 40: t=0.80s, Range=4880.0m */
    {
        .target_relative_pos_m = {319776123, 5029959, -9568256},
        .target_relative_vel_m_s = {-9830400, 1555227, 327680},
        .estimated_closing_velocity_m_s = 9958056,
        .commanded_los_rate_rad_s = {1226, -368, 0}
    },
    /* Step 41: t=0.82s, Range=4877.0m */
    {
        .target_relative_pos_m = {319577610, 5148046, -9561702},
        .target_relative_vel_m_s = {-9830400, 1551054, 327680},
        .estimated_closing_velocity_m_s = 9957405,
        .commanded_los_rate_rad_s = {1227, -368, 0}
    },
    /* Step 42: t=0.84s, Range=4874.0m */
    {
        .target_relative_pos_m = {319379059, 5265655, -9555149},
        .target_relative_vel_m_s = {-9830400, 1546782, 327680},
        .estimated_closing_velocity_m_s = 9956740,
        .commanded_los_rate_rad_s = {1227, -368, 0}
    },
    /* Step 43: t=0.86s, Range=4871.0m */
    {
        .target_relative_pos_m = {319180471, 5382778, -9548595},
        .target_relative_vel_m_s = {-9830400, 1542411, 327680},
        .estimated_closing_velocity_m_s = 9956062,
        .commanded_los_rate_rad_s = {1227, -368, 0}
    },
    /* Step 44: t=0.88s, Range=4868.0m */
    {
        .target_relative_pos_m = {318981845, 5499407, -9542042},
        .target_relative_vel_m_s = {-9830400, 1537942, 327680},
        .estimated_closing_velocity_m_s = 9955371,
        .commanded_los_rate_rad_s = {1227, -368, 0}
    },
    /* Step 45: t=0.90s, Range=4865.0m */
    {
        .target_relative_pos_m = {318783184, 5615536, -9535488},
        .target_relative_vel_m_s = {-9830400, 1533373, 327680},
        .estimated_closing_velocity_m_s = 9954666,
        .commanded_los_rate_rad_s = {1228, -368, 0}
    },
    /* Step 46: t=0.92s, Range=4862.0m */
    {
        .target_relative_pos_m = {318584486, 5731157, -9528934},
        .target_relative_vel_m_s = {-9830400, 1528707, 327680},
        .estimated_closing_velocity_m_s = 9953948,
        .commanded_los_rate_rad_s = {1228, -368, 0}
    },
    /* Step 47: t=0.94s, Range=4859.0m */
    {
        .target_relative_pos_m = {318385753, 5846265, -9522381},
        .target_relative_vel_m_s = {-9830400, 1523943, 327680},
        .estimated_closing_velocity_m_s = 9953218,
        .commanded_los_rate_rad_s = {1228, -368, 0}
    },
    /* Step 48: t=0.96s, Range=4856.0m */
    {
        .target_relative_pos_m = {318186986, 5960851, -9515827},
        .target_relative_vel_m_s = {-9830400, 1519081, 327680},
        .estimated_closing_velocity_m_s = 9952474,
        .commanded_los_rate_rad_s = {1227, -368, 0}
    },
    /* Step 49: t=0.98s, Range=4853.0m */
    {
        .target_relative_pos_m = {317988185, 6074909, -9509274},
        .target_relative_vel_m_s = {-9830400, 1514122, 327680},
        .estimated_closing_velocity_m_s = 9951719,
        .commanded_los_rate_rad_s = {1227, -368, 0}
    },
    /* Step 50: t=1.00s, Range=4850.0m */
    {
        .target_relative_pos_m = {317789351, 6188432, -9502720},
        .target_relative_vel_m_s = {-9830400, 1509066, 327680},
        .estimated_closing_velocity_m_s = 9950951,
        .commanded_los_rate_rad_s = {1227, -368, 0}
    },
    /* Step 51: t=1.02s, Range=4847.0m */
    {
        .target_relative_pos_m = {317590484, 6301414, -9496166},
        .target_relative_vel_m_s = {-9830400, 1503914, 327680},
        .estimated_closing_velocity_m_s = 9950171,
        .commanded_los_rate_rad_s = {1227, -368, 0}
    },
    /* Step 52: t=1.04s, Range=4844.0m */
    {
        .target_relative_pos_m = {317391585, 6413847, -9489613},
        .target_relative_vel_m_s = {-9830400, 1498665, 327680},
        .estimated_closing_velocity_m_s = 9949379,
        .commanded_los_rate_rad_s = {1226, -368, 0}
    },
    /* Step 53: t=1.06s, Range=4841.0m */
    {
        .target_relative_pos_m = {317192655, 6525725, -9483059},
        .target_relative_vel_m_s = {-9830400, 1493321, 327680},
        .estimated_closing_velocity_m_s = 9948575,
        .commanded_los_rate_rad_s = {1226, -368, 0}
    },
    /* Step 54: t=1.08s, Range=4838.0m */
    {
        .target_relative_pos_m = {316993694, 6637041, -9476506},
        .target_relative_vel_m_s = {-9830400, 1487881, 327680},
        .estimated_closing_velocity_m_s = 9947760,
        .commanded_los_rate_rad_s = {1225, -368, 0}
    },
    /* Step 55: t=1.10s, Range=4835.0m */
    {
        .target_relative_pos_m = {316794704, 6747789, -9469952},
        .target_relative_vel_m_s = {-9830400, 1482345, 327680},
        .estimated_closing_velocity_m_s = 9946933,
        .commanded_los_rate_rad_s = {1224, -367, 0}
    },
    /* Step 56: t=1.12s, Range=4832.0m */
    {
        .target_relative_pos_m = {316595684, 6857961, -9463398},
        .target_relative_vel_m_s = {-9830400, 1476715, 327680},
        .estimated_closing_velocity_m_s = 9946096,
        .commanded_los_rate_rad_s = {1224, -367, 0}
    },
    /* Step 57: t=1.14s, Range=4829.0m */
    {
        .target_relative_pos_m = {316396635, 6967553, -9456845},
        .target_relative_vel_m_s = {-9830400, 1470990, 327680},
        .estimated_closing_velocity_m_s = 9945248,
        .commanded_los_rate_rad_s = {1223, -367, 0}
    },
    /* Step 58: t=1.16s, Range=4826.0m */
    {
        .target_relative_pos_m = {316197559, 7076556, -9450291},
        .target_relative_vel_m_s = {-9830400, 1465171, 327680},
        .estimated_closing_velocity_m_s = 9944389,
        .commanded_los_rate_rad_s = {1222, -367, 0}
    },
    /* Step 59: t=1.18s, Range=4823.0m */
    {
        .target_relative_pos_m = {315998455, 7184965, -9443738},
        .target_relative_vel_m_s = {-9830400, 1459259, 327680},
        .estimated_closing_velocity_m_s = 9943519,
        .commanded_los_rate_rad_s = {1221, -366, 0}
    },
    /* Step 60: t=1.20s, Range=4820.0m */
    {
        .target_relative_pos_m = {315799325, 7292774, -9437184},
        .target_relative_vel_m_s = {-9830400, 1453252, 327680},
        .estimated_closing_velocity_m_s = 9942640,
        .commanded_los_rate_rad_s = {1220, -366, 0}
    },
    /* Step 61: t=1.22s, Range=4817.0m */
    {
        .target_relative_pos_m = {315600169, 7399975, -9430630},
        .target_relative_vel_m_s = {-9830400, 1447153, 327680},
        .estimated_closing_velocity_m_s = 9941750,
        .commanded_los_rate_rad_s = {1219, -366, 0}
    },
    /* Step 62: t=1.24s, Range=4814.0m */
    {
        .target_relative_pos_m = {315400988, 7506563, -9424077},
        .target_relative_vel_m_s = {-9830400, 1440962, 327680},
        .estimated_closing_velocity_m_s = 9940851,
        .commanded_los_rate_rad_s = {1217, -365, 0}
    },
    /* Step 63: t=1.26s, Range=4811.0m */
    {
        .target_relative_pos_m = {315201783, 7612531, -9417523},
        .target_relative_vel_m_s = {-9830400, 1434678, 327680},
        .estimated_closing_velocity_m_s = 9939942,
        .commanded_los_rate_rad_s = {1216, -365, 0}
    },
    /* Step 64: t=1.28s, Range=4808.0m */
    {
        .target_relative_pos_m = {315002554, 7717874, -9410970},
        .target_relative_vel_m_s = {-9830400, 1428302, 327680},
        .estimated_closing_velocity_m_s = 9939023,
        .commanded_los_rate_rad_s = {1215, -364, 0}
    },
    /* Step 65: t=1.30s, Range=4805.0m */
    {
        .target_relative_pos_m = {314803303, 7822585, -9404416},
        .target_relative_vel_m_s = {-9830400, 1421835, 327680},
        .estimated_closing_velocity_m_s = 9938096,
        .commanded_los_rate_rad_s = {1213, -364, 0}
    },
    /* Step 66: t=1.32s, Range=4802.0m */
    {
        .target_relative_pos_m = {314604029, 7926658, -9397862},
        .target_relative_vel_m_s = {-9830400, 1415277, 327680},
        .estimated_closing_velocity_m_s = 9937160,
        .commanded_los_rate_rad_s = {1212, -364, 0}
    },
    /* Step 67: t=1.34s, Range=4799.0m */
    {
        .target_relative_pos_m = {314404734, 8030087, -9391309},
        .target_relative_vel_m_s = {-9830400, 1408628, 327680},
        .estimated_closing_velocity_m_s = 9936215,
        .commanded_los_rate_rad_s = {1210, -363, 0}
    },
    /* Step 68: t=1.36s, Range=4796.0m */
    {
        .target_relative_pos_m = {314205418, 8132866, -9384755},
        .target_relative_vel_m_s = {-9830400, 1401889, 327680},
        .estimated_closing_velocity_m_s = 9935262,
        .commanded_los_rate_rad_s = {1208, -363, 0}
    },
    /* Step 69: t=1.38s, Range=4793.0m */
    {
        .target_relative_pos_m = {314006083, 8234989, -9378202},
        .target_relative_vel_m_s = {-9830400, 1395061, 327680},
        .estimated_closing_velocity_m_s = 9934301,
        .commanded_los_rate_rad_s = {1207, -362, 0}
    },
    /* Step 70: t=1.40s, Range=4790.0m */
    {
        .target_relative_pos_m = {313806728, 8336450, -9371648},
        .target_relative_vel_m_s = {-9830400, 1388143, 327680},
        .estimated_closing_velocity_m_s = 9933332,
        .commanded_los_rate_rad_s = {1205, -361, 0}
    },
    /* Step 71: t=1.42s, Range=4787.0m */
    {
        .target_relative_pos_m = {313607355, 8437244, -9365094},
        .target_relative_vel_m_s = {-9830400, 1381136, 327680},
        .estimated_closing_velocity_m_s = 9932355,
        .commanded_los_rate_rad_s = {1203, -361, 0}
    },
    /* Step 72: t=1.44s, Range=4784.0m */
    {
        .target_relative_pos_m = {313407965, 8537365, -9358541},
        .target_relative_vel_m_s = {-9830400, 1374041, 327680},
        .estimated_closing_velocity_m_s = 9931371,
        .commanded_los_rate_rad_s = {1201, -360, 0}
    },
    /* Step 73: t=1.46s, Range=4781.0m */
    {
        .target_relative_pos_m = {313208558, 8636806, -9351987},
        .target_relative_vel_m_s = {-9830400, 1366858, 327680},
        .estimated_closing_velocity_m_s = 9930380,
        .commanded_los_rate_rad_s = {1199, -360, 0}
    },
    /* Step 74: t=1.48s, Range=4778.0m */
    {
        .target_relative_pos_m = {313009134, 8735563, -9345434},
        .target_relative_vel_m_s = {-9830400, 1359587, 327680},
        .estimated_closing_velocity_m_s = 9929381,
        .commanded_los_rate_rad_s = {1196, -359, 0}
    },
    /* Step 75: t=1.50s, Range=4775.0m */
    {
        .target_relative_pos_m = {312809696, 8833629, -9338880},
        .target_relative_vel_m_s = {-9830400, 1352230, 327680},
        .estimated_closing_velocity_m_s = 9928377,
        .commanded_los_rate_rad_s = {1194, -358, 0}
    },
    /* Step 76: t=1.52s, Range=4772.0m */
    {
        .target_relative_pos_m = {312610243, 8930999, -9332326},
        .target_relative_vel_m_s = {-9830400, 1344786, 327680},
        .estimated_closing_velocity_m_s = 9927366,
        .commanded_los_rate_rad_s = {1192, -358, 0}
    },
    /* Step 77: t=1.54s, Range=4769.0m */
    {
        .target_relative_pos_m = {312410776, 9027668, -9325773},
        .target_relative_vel_m_s = {-9830400, 1337256, 327680},
        .estimated_closing_velocity_m_s = 9926348,
        .commanded_los_rate_rad_s = {1189, -357, 0}
    },
    /* Step 78: t=1.56s, Range=4766.0m */
    {
        .target_relative_pos_m = {312211296, 9123630, -9319219},
        .target_relative_vel_m_s = {-9830400, 1329640, 327680},
        .estimated_closing_velocity_m_s = 9925325,
        .commanded_los_rate_rad_s = {1187, -356, 0}
    },
    /* Step 79: t=1.58s, Range=4763.0m */
    {
        .target_relative_pos_m = {312011805, 9218880, -9312666},
        .target_relative_vel_m_s = {-9830400, 1321939, 327680},
        .estimated_closing_velocity_m_s = 9924297,
        .commanded_los_rate_rad_s = {1184, -355, 0}
    },
    /* Step 80: t=1.60s, Range=4760.0m */
    {
        .target_relative_pos_m = {311812301, 9313412, -9306112},
        .target_relative_vel_m_s = {-9830400, 1314154, 327680},
        .estimated_closing_velocity_m_s = 9923262,
        .commanded_los_rate_rad_s = {1182, -354, 0}
    },
    /* Step 81: t=1.62s, Range=4757.0m */
    {
        .target_relative_pos_m = {311612788, 9407222, -9299558},
        .target_relative_vel_m_s = {-9830400, 1306284, 327680},
        .estimated_closing_velocity_m_s = 9922223,
        .commanded_los_rate_rad_s = {1179, -354, 0}
    },
    /* Step 82: t=1.64s, Range=4754.0m */
    {
        .target_relative_pos_m = {311413265, 9500303, -9293005},
        .target_relative_vel_m_s = {-9830400, 1298331, 327680},
        .estimated_closing_velocity_m_s = 9921179,
        .commanded_los_rate_rad_s = {1176, -353, 0}
    },
    /* Step 83: t=1.66s, Range=4751.0m */
    {
        .target_relative_pos_m = {311213732, 9592650, -9286451},
        .target_relative_vel_m_s = {-9830400, 1290295, 327680},
        .estimated_closing_velocity_m_s = 9920131,
        .commanded_los_rate_rad_s = {1173, -352, 0}
    },
    /* Step 84: t=1.68s, Range=4748.0m */
    {
        .target_relative_pos_m = {311014192, 9684259, -9279898},
        .target_relative_vel_m_s = {-9830400, 1282176, 327680},
        .estimated_closing_velocity_m_s = 9919078,
        .commanded_los_rate_rad_s = {1170, -351, 0}
    },
    /* Step 85: t=1.70s, Range=4745.0m */
    {
        .target_relative_pos_m = {310814644, 9775125, -9273344},
        .target_relative_vel_m_s = {-9830400, 1273975, 327680},
        .estimated_closing_velocity_m_s = 9918022,
        .commanded_los_rate_rad_s = {1167, -350, 0}
    },
    /* Step 86: t=1.72s, Range=4742.0m */
    {
        .target_relative_pos_m = {310615090, 9865242, -9266790},
        .target_relative_vel_m_s = {-9830400, 1265693, 327680},
        .estimated_closing_velocity_m_s = 9916961,
        .commanded_los_rate_rad_s = {1164, -349, 0}
    },
    /* Step 87: t=1.74s, Range=4739.0m */
    {
        .target_relative_pos_m = {310415530, 9954605, -9260237},
        .target_relative_vel_m_s = {-9830400, 1257329, 327680},
        .estimated_closing_velocity_m_s = 9915897,
        .commanded_los_rate_rad_s = {1161, -348, 0}
    },
    /* Step 88: t=1.76s, Range=4736.0m */
    {
        .target_relative_pos_m = {310215965, 10043210, -9253683},
        .target_relative_vel_m_s = {-9830400, 1248885, 327680},
        .estimated_closing_velocity_m_s = 9914830,
        .commanded_los_rate_rad_s = {1158, -347, 0}
    },
    /* Step 89: t=1.78s, Range=4733.0m */
    {
        .target_relative_pos_m = {310016395, 10131052, -9247130},
        .target_relative_vel_m_s = {-9830400, 1240362, 327680},
        .estimated_closing_velocity_m_s = 9913760,
        .commanded_los_rate_rad_s = {1154, -346, 0}
    },
    /* Step 90: t=1.80s, Range=4730.0m */
    {
        .target_relative_pos_m = {309816823, 10218126, -9240576},
        .target_relative_vel_m_s = {-9830400, 1231759, 327680},
        .estimated_closing_velocity_m_s = 9912687,
        .commanded_los_rate_rad_s = {1151, -345, 0}
    },
    /* Step 91: t=1.82s, Range=4727.0m */
    {
        .target_relative_pos_m = {309617248, 10304427, -9234022},
        .target_relative_vel_m_s = {-9830400, 1223076, 327680},
        .estimated_closing_velocity_m_s = 9911612,
        .commanded_los_rate_rad_s = {1147, -344, 0}
    },
    /* Step 92: t=1.84s, Range=4724.0m */
    {
        .target_relative_pos_m = {309417671, 10389950, -9227469},
        .target_relative_vel_m_s = {-9830400, 1214316, 327680},
        .estimated_closing_velocity_m_s = 9910535,
        .commanded_los_rate_rad_s = {1144, -343, 0}
    },
    /* Step 93: t=1.86s, Range=4721.0m */
    {
        .target_relative_pos_m = {309218093, 10474691, -9220915},
        .target_relative_vel_m_s = {-9830400, 1205478, 327680},
        .estimated_closing_velocity_m_s = 9909456,
        .commanded_los_rate_rad_s = {1140, -342, 0}
    },
    /* Step 94: t=1.88s, Range=4718.0m */
    {
        .target_relative_pos_m = {309018515, 10558645, -9214362},
        .target_relative_vel_m_s = {-9830400, 1196563, 327680},
        .estimated_closing_velocity_m_s = 9908375,
        .commanded_los_rate_rad_s = {1136, -341, 0}
    },
    /* Step 95: t=1.90s, Range=4715.0m */
    {
        .target_relative_pos_m = {308818938, 10641809, -9207808},
        .target_relative_vel_m_s = {-9830400, 1187571, 327680},
        .estimated_closing_velocity_m_s = 9907293,
        .commanded_los_rate_rad_s = {1132, -340, 0}
    },
    /* Step 96: t=1.92s, Range=4712.0m */
    {
        .target_relative_pos_m = {308619362, 10724176, -9201254},
        .target_relative_vel_m_s = {-9830400, 1178504, 327680},
        .estimated_closing_velocity_m_s = 9906211,
        .commanded_los_rate_rad_s = {1128, -339, 0}
    },
    /* Step 97: t=1.94s, Range=4709.0m */
    {
        .target_relative_pos_m = {308419788, 10805744, -9194701},
        .target_relative_vel_m_s = {-9830400, 1169360, 327680},
        .estimated_closing_velocity_m_s = 9905127,
        .commanded_los_rate_rad_s = {1124, -337, 0}
    },
    /* Step 98: t=1.96s, Range=4706.0m */
    {
        .target_relative_pos_m = {308220217, 10886507, -9188147},
        .target_relative_vel_m_s = {-9830400, 1160142, 327680},
        .estimated_closing_velocity_m_s = 9904043,
        .commanded_los_rate_rad_s = {1120, -336, 0}
    },
    /* Step 99: t=1.98s, Range=4703.0m */
    {
        .target_relative_pos_m = {308020650, 10966462, -9181594},
        .target_relative_vel_m_s = {-9830400, 1150850, 327680},
        .estimated_closing_velocity_m_s = 9902959,
        .commanded_los_rate_rad_s = {1116, -335, 0}
    },
    /* Step 100: t=2.00s, Range=4700.0m */
    {
        .target_relative_pos_m = {307821088, 11045604, -9175040},
        .target_relative_vel_m_s = {-9830400, 1141484, 327680},
        .estimated_closing_velocity_m_s = 9901875,
        .commanded_los_rate_rad_s = {1112, -334, 0}
    },
    /* Step 101: t=2.02s, Range=4697.0m */
    {
        .target_relative_pos_m = {307621531, 11123929, -9168486},
        .target_relative_vel_m_s = {-9830400, 1132045, 327680},
        .estimated_closing_velocity_m_s = 9900791,
        .commanded_los_rate_rad_s = {1108, -332, 0}
    },
    /* Step 102: t=2.04s, Range=4694.0m */
    {
        .target_relative_pos_m = {307421980, 11201432, -9161933},
        .target_relative_vel_m_s = {-9830400, 1122534, 327680},
        .estimated_closing_velocity_m_s = 9899708,
        .commanded_los_rate_rad_s = {1103, -331, 0}
    },
    /* Step 103: t=2.06s, Range=4691.0m */
    {
        .target_relative_pos_m = {307222436, 11278111, -9155379},
        .target_relative_vel_m_s = {-9830400, 1112951, 327680},
        .estimated_closing_velocity_m_s = 9898626,
        .commanded_los_rate_rad_s = {1099, -330, 0}
    },
    /* Step 104: t=2.08s, Range=4688.0m */
    {
        .target_relative_pos_m = {307022900, 11353961, -9148826},
        .target_relative_vel_m_s = {-9830400, 1103296, 327680},
        .estimated_closing_velocity_m_s = 9897545,
        .commanded_los_rate_rad_s = {1094, -328, 0}
    },
    /* Step 105: t=2.10s, Range=4685.0m */
    {
        .target_relative_pos_m = {306823373, 11428977, -9142272},
        .target_relative_vel_m_s = {-9830400, 1093571, 327680},
        .estimated_closing_velocity_m_s = 9896466,
        .commanded_los_rate_rad_s = {1090, -327, 0}
    },
    /* Step 106: t=2.12s, Range=4682.0m */
    {
        .target_relative_pos_m = {306623854, 11503157, -9135718},
        .target_relative_vel_m_s = {-9830400, 1083776, 327680},
        .estimated_closing_velocity_m_s = 9895388,
        .commanded_los_rate_rad_s = {1085, -326, 0}
    },
    /* Step 107: t=2.14s, Range=4679.0m */
    {
        .target_relative_pos_m = {306424346, 11576496, -9129165},
        .target_relative_vel_m_s = {-9830400, 1073912, 327680},
        .estimated_closing_velocity_m_s = 9894313,
        .commanded_los_rate_rad_s = {1080, -324, 0}
    },
    /* Step 108: t=2.16s, Range=4676.0m */
    {
        .target_relative_pos_m = {306224849, 11648991, -9122611},
        .target_relative_vel_m_s = {-9830400, 1063978, 327680},
        .estimated_closing_velocity_m_s = 9893240,
        .commanded_los_rate_rad_s = {1076, -323, 0}
    },
    /* Step 109: t=2.18s, Range=4673.0m */
    {
        .target_relative_pos_m = {306025363, 11720638, -9116058},
        .target_relative_vel_m_s = {-9830400, 1053977, 327680},
        .estimated_closing_velocity_m_s = 9892169,
        .commanded_los_rate_rad_s = {1071, -321, 0}
    },
    /* Step 110: t=2.20s, Range=4670.0m */
    {
        .target_relative_pos_m = {305825889, 11791433, -9109504},
        .target_relative_vel_m_s = {-9830400, 1043908, 327680},
        .estimated_closing_velocity_m_s = 9891101,
        .commanded_los_rate_rad_s = {1066, -320, 0}
    },
    /* Step 111: t=2.22s, Range=4667.0m */
    {
        .target_relative_pos_m = {305626428, 11861373, -9102950},
        .target_relative_vel_m_s = {-9830400, 1033773, 327680},
        .estimated_closing_velocity_m_s = 9890037,
        .commanded_los_rate_rad_s = {1061, -318, 0}
    },
    /* Step 112: t=2.24s, Range=4664.0m */
    {
        .target_relative_pos_m = {305426982, 11930454, -9096397},
        .target_relative_vel_m_s = {-9830400, 1023571, 327680},
        .estimated_closing_velocity_m_s = 9888975,
        .commanded_los_rate_rad_s = {1055, -317, 0}
    },
    /* Step 113: t=2.26s, Range=4661.0m */
    {
        .target_relative_pos_m = {305227550, 11998673, -9089843},
        .target_relative_vel_m_s = {-9830400, 1013304, 327680},
        .estimated_closing_velocity_m_s = 9887918,
        .commanded_los_rate_rad_s = {1050, -315, 0}
    },
    /* Step 114: t=2.28s, Range=4658.0m */
    {
        .target_relative_pos_m = {305028133, 12066027, -9083290},
        .target_relative_vel_m_s = {-9830400, 1002972, 327680},
        .estimated_closing_velocity_m_s = 9886865,
        .commanded_los_rate_rad_s = {1045, -314, 0}
    },
    /* Step 115: t=2.30s, Range=4655.0m */
    {
        .target_relative_pos_m = {304828732, 12132512, -9076736},
        .target_relative_vel_m_s = {-9830400, 992576, 327680},
        .estimated_closing_velocity_m_s = 9885815,
        .commanded_los_rate_rad_s = {1040, -312, 0}
    },
    /* Step 116: t=2.32s, Range=4652.0m */
    {
        .target_relative_pos_m = {304629348, 12198125, -9070182},
        .target_relative_vel_m_s = {-9830400, 982116, 327680},
        .estimated_closing_velocity_m_s = 9884771,
        .commanded_los_rate_rad_s = {1034, -310, 0}
    },
    /* Step 117: t=2.34s, Range=4649.0m */
    {
        .target_relative_pos_m = {304429982, 12262863, -9063629},
        .target_relative_vel_m_s = {-9830400, 971593, 327680},
        .estimated_closing_velocity_m_s = 9883731,
        .commanded_los_rate_rad_s = {1029, -309, 0}
    },
    /* Step 118: t=2.36s, Range=4646.0m */
    {
        .target_relative_pos_m = {304230633, 12326722, -9057075},
        .target_relative_vel_m_s = {-9830400, 961009, 327680},
        .estimated_closing_velocity_m_s = 9882696,
        .commanded_los_rate_rad_s = {1023, -307, 0}
    },
    /* Step 119: t=2.38s, Range=4643.0m */
    {
        .target_relative_pos_m = {304031304, 12389701, -9050522},
        .target_relative_vel_m_s = {-9830400, 950362, 327680},
        .estimated_closing_velocity_m_s = 9881666,
        .commanded_los_rate_rad_s = {1018, -305, 0}
    },
    /* Step 120: t=2.40s, Range=4640.0m */
    {
        .target_relative_pos_m = {303831994, 12451795, -9043968},
        .target_relative_vel_m_s = {-9830400, 939655, 327680},
        .estimated_closing_velocity_m_s = 9880642,
        .commanded_los_rate_rad_s = {1012, -304, 0}
    },
    /* Step 121: t=2.42s, Range=4637.0m */
    {
        .target_relative_pos_m = {303632705, 12513001, -9037414},
        .target_relative_vel_m_s = {-9830400, 928888, 327680},
        .estimated_closing_velocity_m_s = 9879624,
        .commanded_los_rate_rad_s = {1006, -302, 0}
    },
    /* Step 122: t=2.44s, Range=4634.0m */
    {
        .target_relative_pos_m = {303433437, 12573318, -9030861},
        .target_relative_vel_m_s = {-9830400, 918061, 327680},
        .estimated_closing_velocity_m_s = 9878612,
        .commanded_los_rate_rad_s = {1000, -300, 0}
    },
    /* Step 123: t=2.46s, Range=4631.0m */
    {
        .target_relative_pos_m = {303234190, 12632741, -9024307},
        .target_relative_vel_m_s = {-9830400, 907176, 327680},
        .estimated_closing_velocity_m_s = 9877606,
        .commanded_los_rate_rad_s = {994, -298, 0}
    },
    /* Step 124: t=2.48s, Range=4628.0m */
    {
        .target_relative_pos_m = {303034966, 12691269, -9017754},
        .target_relative_vel_m_s = {-9830400, 896232, 327680},
        .estimated_closing_velocity_m_s = 9876607,
        .commanded_los_rate_rad_s = {988, -296, 0}
    },
    /* Step 125: t=2.50s, Range=4625.0m */
    {
        .target_relative_pos_m = {302835765, 12748899, -9011200},
        .target_relative_vel_m_s = {-9830400, 885231, 327680},
        .estimated_closing_velocity_m_s = 9875615,
        .commanded_los_rate_rad_s = {982, -295, 0}
    },
    /* Step 126: t=2.52s, Range=4622.0m */
    {
        .target_relative_pos_m = {302636587, 12805628, -9004646},
        .target_relative_vel_m_s = {-9830400, 874174, 327680},
        .estimated_closing_velocity_m_s = 9874630,
        .commanded_los_rate_rad_s = {976, -293, 0}
    },
    /* Step 127: t=2.54s, Range=4619.0m */
    {
        .target_relative_pos_m = {302437434, 12861453, -8998093},
        .target_relative_vel_m_s = {-9830400, 863060, 327680},
        .estimated_closing_velocity_m_s = 9873652,
        .commanded_los_rate_rad_s = {970, -291, 0}
    },
    /* Step 128: t=2.56s, Range=4616.0m */
    {
        .target_relative_pos_m = {302238307, 12916372, -8991539},
        .target_relative_vel_m_s = {-9830400, 851892, 327680},
        .estimated_closing_velocity_m_s = 9872682,
        .commanded_los_rate_rad_s = {964, -289, 0}
    },
    /* Step 129: t=2.58s, Range=4613.0m */
    {
        .target_relative_pos_m = {302039205, 12970383, -8984986},
        .target_relative_vel_m_s = {-9830400, 840668, 327680},
        .estimated_closing_velocity_m_s = 9871720,
        .commanded_los_rate_rad_s = {957, -287, 0}
    },
    /* Step 130: t=2.60s, Range=4610.0m */
    {
        .target_relative_pos_m = {301840129, 13023483, -8978432},
        .target_relative_vel_m_s = {-9830400, 829391, 327680},
        .estimated_closing_velocity_m_s = 9870766,
        .commanded_los_rate_rad_s = {951, -285, 0}
    },
    /* Step 131: t=2.62s, Range=4607.0m */
    {
        .target_relative_pos_m = {301641080, 13075670, -8971878},
        .target_relative_vel_m_s = {-9830400, 818061, 327680},
        .estimated_closing_velocity_m_s = 9869821,
        .commanded_los_rate_rad_s = {944, -283, 0}
    },
    /* Step 132: t=2.64s, Range=4604.0m */
    {
        .target_relative_pos_m = {301442059, 13126942, -8965325},
        .target_relative_vel_m_s = {-9830400, 806679, 327680},
        .estimated_closing_velocity_m_s = 9868884,
        .commanded_los_rate_rad_s = {938, -281, 0}
    },
    /* Step 133: t=2.66s, Range=4601.0m */
    {
        .target_relative_pos_m = {301243066, 13177296, -8958771},
        .target_relative_vel_m_s = {-9830400, 795245, 327680},
        .estimated_closing_velocity_m_s = 9867956,
        .commanded_los_rate_rad_s = {931, -279, 0}
    },
    /* Step 134: t=2.68s, Range=4598.0m */
    {
        .target_relative_pos_m = {301044102, 13226730, -8952218},
        .target_relative_vel_m_s = {-9830400, 783760, 327680},
        .estimated_closing_velocity_m_s = 9867037,
        .commanded_los_rate_rad_s = {924, -277, 0}
    },
    /* Step 135: t=2.70s, Range=4595.0m */
    {
        .target_relative_pos_m = {300845167, 13275243, -8945664},
        .target_relative_vel_m_s = {-9830400, 772224, 327680},
        .estimated_closing_velocity_m_s = 9866127,
        .commanded_los_rate_rad_s = {918, -275, 0}
    },
    /* Step 136: t=2.72s, Range=4592.0m */
    {
        .target_relative_pos_m = {300646263, 13322833, -8939110},
        .target_relative_vel_m_s = {-9830400, 760640, 327680},
        .estimated_closing_velocity_m_s = 9865227,
        .commanded_los_rate_rad_s = {911, -273, 0}
    },
    /* Step 137: t=2.74s, Range=4589.0m */
    {
        .target_relative_pos_m = {300447389, 13369497, -8932557},
        .target_relative_vel_m_s = {-9830400, 749007, 327680},
        .estimated_closing_velocity_m_s = 9864337,
        .commanded_los_rate_rad_s = {904, -271, 0}
    },
    /* Step 138: t=2.76s, Range=4586.0m */
    {
        .target_relative_pos_m = {300248546, 13415233, -8926003},
        .target_relative_vel_m_s = {-9830400, 737325, 327680},
        .estimated_closing_velocity_m_s = 9863457,
        .commanded_los_rate_rad_s = {897, -269, 0}
    },
    /* Step 139: t=2.78s, Range=4583.0m */
    {
        .target_relative_pos_m = {300049735, 13460041, -8919450},
        .target_relative_vel_m_s = {-9830400, 725597, 327680},
        .estimated_closing_velocity_m_s = 9862587,
        .commanded_los_rate_rad_s = {890, -267, 0}
    },
    /* Step 140: t=2.80s, Range=4580.0m */
    {
        .target_relative_pos_m = {299850957, 13503917, -8912896},
        .target_relative_vel_m_s = {-9830400, 713822, 327680},
        .estimated_closing_velocity_m_s = 9861728,
        .commanded_los_rate_rad_s = {883, -265, 0}
    },
    /* Step 141: t=2.82s, Range=4577.0m */
    {
        .target_relative_pos_m = {299652211, 13546861, -8906342},
        .target_relative_vel_m_s = {-9830400, 702002, 327680},
        .estimated_closing_velocity_m_s = 9860880,
        .commanded_los_rate_rad_s = {876, -263, 0}
    },
    /* Step 142: t=2.84s, Range=4574.0m */
    {
        .target_relative_pos_m = {299453499, 13588871, -8899789},
        .target_relative_vel_m_s = {-9830400, 690136, 327680},
        .estimated_closing_velocity_m_s = 9860042,
        .commanded_los_rate_rad_s = {869, -261, 0}
    },
    /* Step 143: t=2.86s, Range=4571.0m */
    {
        .target_relative_pos_m = {299254820, 13629946, -8893235},
        .target_relative_vel_m_s = {-9830400, 678227, 327680},
        .estimated_closing_velocity_m_s = 9859215,
        .commanded_los_rate_rad_s = {861, -258, 0}
    },
    /* Step 144: t=2.88s, Range=4568.0m */
    {
        .target_relative_pos_m = {299056176, 13670083, -8886682},
        .target_relative_vel_m_s = {-9830400, 666274, 327680},
        .estimated_closing_velocity_m_s = 9858400,
        .commanded_los_rate_rad_s = {854, -256, 0}
    },
    /* Step 145: t=2.90s, Range=4565.0m */
    {
        .target_relative_pos_m = {298857567, 13709281, -8880128},
        .target_relative_vel_m_s = {-9830400, 654278, 327680},
        .estimated_closing_velocity_m_s = 9857597,
        .commanded_los_rate_rad_s = {846, -254, 0}
    },
    /* Step 146: t=2.92s, Range=4562.0m */
    {
        .target_relative_pos_m = {298658994, 13747540, -8873574},
        .target_relative_vel_m_s = {-9830400, 642240, 327680},
        .estimated_closing_velocity_m_s = 9856805,
        .commanded_los_rate_rad_s = {839, -252, 0}
    },
    /* Step 147: t=2.94s, Range=4559.0m */
    {
        .target_relative_pos_m = {298460456, 13784857, -8867021},
        .target_relative_vel_m_s = {-9830400, 630162, 327680},
        .estimated_closing_velocity_m_s = 9856026,
        .commanded_los_rate_rad_s = {831, -249, 0}
    },
    /* Step 148: t=2.96s, Range=4556.0m */
    {
        .target_relative_pos_m = {298261955, 13821233, -8860467},
        .target_relative_vel_m_s = {-9830400, 618043, 327680},
        .estimated_closing_velocity_m_s = 9855258,
        .commanded_los_rate_rad_s = {824, -247, 0}
    },
    /* Step 149: t=2.98s, Range=4553.0m */
    {
        .target_relative_pos_m = {298063491, 13856664, -8853914},
        .target_relative_vel_m_s = {-9830400, 605884, 327680},
        .estimated_closing_velocity_m_s = 9854503,
        .commanded_los_rate_rad_s = {816, -245, 0}
    },
    /* Step 150: t=3.00s, Range=4550.0m */
    {
        .target_relative_pos_m = {297865064, 13891152, -8847360},
        .target_relative_vel_m_s = {-9830400, 593687, 327680},
        .estimated_closing_velocity_m_s = 9853761,
        .commanded_los_rate_rad_s = {808, -243, 0}
    },
    /* Step 151: t=3.02s, Range=4547.0m */
    {
        .target_relative_pos_m = {297666675, 13924693, -8840806},
        .target_relative_vel_m_s = {-9830400, 581452, 327680},
        .estimated_closing_velocity_m_s = 9853031,
        .commanded_los_rate_rad_s = {801, -240, 0}
    },
    /* Step 152: t=3.04s, Range=4544.0m */
    {
        .target_relative_pos_m = {297468324, 13957288, -8834253},
        .target_relative_vel_m_s = {-9830400, 569179, 327680},
        .estimated_closing_velocity_m_s = 9852315,
        .commanded_los_rate_rad_s = {793, -238, 0}
    },
    /* Step 153: t=3.06s, Range=4541.0m */
    {
        .target_relative_pos_m = {297270012, 13988936, -8827699},
        .target_relative_vel_m_s = {-9830400, 556870, 327680},
        .estimated_closing_velocity_m_s = 9851611,
        .commanded_los_rate_rad_s = {785, -235, 0}
    },
    /* Step 154: t=3.08s, Range=4538.0m */
    {
        .target_relative_pos_m = {297071739, 14019635, -8821146},
        .target_relative_vel_m_s = {-9830400, 544526, 327680},
        .estimated_closing_velocity_m_s = 9850921,
        .commanded_los_rate_rad_s = {777, -233, 0}
    },
    /* Step 155: t=3.10s, Range=4535.0m */
    {
        .target_relative_pos_m = {296873506, 14049385, -8814592},
        .target_relative_vel_m_s = {-9830400, 532146, 327680},
        .estimated_closing_velocity_m_s = 9850245,
        .commanded_los_rate_rad_s = {769, -231, 0}
    },
    /* Step 156: t=3.12s, Range=4532.0m */
    {
        .target_relative_pos_m = {296675313, 14078185, -8808038},
        .target_relative_vel_m_s = {-9830400, 519733, 327680},
        .estimated_closing_velocity_m_s = 9849582,
        .commanded_los_rate_rad_s = {761, -228, 0}
    },
    /* Step 157: t=3.14s, Range=4529.0m */
    {
        .target_relative_pos_m = {296477159, 14106035, -8801485},
        .target_relative_vel_m_s = {-9830400, 507286, 327680},
        .estimated_closing_velocity_m_s = 9848933,
        .commanded_los_rate_rad_s = {753, -226, 0}
    },
    /* Step 158: t=3.16s, Range=4526.0m */
    {
        .target_relative_pos_m = {296279047, 14132934, -8794931},
        .target_relative_vel_m_s = {-9830400, 494807, 327680},
        .estimated_closing_velocity_m_s = 9848298,
        .commanded_los_rate_rad_s = {744, -223, 0}
    },
    /* Step 159: t=3.18s, Range=4523.0m */
    {
        .target_relative_pos_m = {296080976, 14158882, -8788378},
        .target_relative_vel_m_s = {-9830400, 482296, 327680},
        .estimated_closing_velocity_m_s = 9847677,
        .commanded_los_rate_rad_s = {736, -221, 0}
    },
    /* Step 160: t=3.20s, Range=4520.0m */
    {
        .target_relative_pos_m = {295882946, 14183878, -8781824},
        .target_relative_vel_m_s = {-9830400, 469754, 327680},
        .estimated_closing_velocity_m_s = 9847071,
        .commanded_los_rate_rad_s = {728, -218, 0}
    },
    /* Step 161: t=3.22s, Range=4517.0m */
    {
        .target_relative_pos_m = {295684957, 14207921, -8775270},
        .target_relative_vel_m_s = {-9830400, 457182, 327680},
        .estimated_closing_velocity_m_s = 9846479,
        .commanded_los_rate_rad_s = {719, -216, 0}
    },
    /* Step 162: t=3.24s, Range=4514.0m */
    {
        .target_relative_pos_m = {295487011, 14231012, -8768717},
        .target_relative_vel_m_s = {-9830400, 444581, 327680},
        .estimated_closing_velocity_m_s = 9845902,
        .commanded_los_rate_rad_s = {711, -213, 0}
    },
    /* Step 163: t=3.26s, Range=4511.0m */
    {
        .target_relative_pos_m = {295289107, 14253150, -8762163},
        .target_relative_vel_m_s = {-9830400, 431952, 327680},
        .estimated_closing_velocity_m_s = 9845340,
        .commanded_los_rate_rad_s = {702, -211, 0}
    },
    /* Step 164: t=3.28s, Range=4508.0m */
    {
        .target_relative_pos_m = {295091246, 14274334, -8755610},
        .target_relative_vel_m_s = {-9830400, 419295, 327680},
        .estimated_closing_velocity_m_s = 9844793,
        .commanded_los_rate_rad_s = {694, -208, 0}
    },
    /* Step 165: t=3.30s, Range=4505.0m */
    {
        .target_relative_pos_m = {294893428, 14294566, -8749056},
        .target_relative_vel_m_s = {-9830400, 406611, 327680},
        .estimated_closing_velocity_m_s = 9844261,
        .commanded_los_rate_rad_s = {685, -206, 0}
    },
    /* Step 166: t=3.32s, Range=4502.0m */
    {
        .target_relative_pos_m = {294695654, 14313844, -8742502},
        .target_relative_vel_m_s = {-9830400, 393901, 327680},
        .estimated_closing_velocity_m_s = 9843744,
        .commanded_los_rate_rad_s = {677, -203, 0}
    },
    /* Step 167: t=3.34s, Range=4499.0m */
    {
        .target_relative_pos_m = {294497922, 14332169, -8735949},
        .target_relative_vel_m_s = {-9830400, 381165, 327680},
        .estimated_closing_velocity_m_s = 9843243,
        .commanded_los_rate_rad_s = {668, -200, 0}
    },
    /* Step 168: t=3.36s, Range=4496.0m */
    {
        .target_relative_pos_m = {294300235, 14349541, -8729395},
        .target_relative_vel_m_s = {-9830400, 368406, 327680},
        .estimated_closing_velocity_m_s = 9842757,
        .commanded_los_rate_rad_s = {659, -198, 0}
    },
    /* Step 169: t=3.38s, Range=4493.0m */
    {
        .target_relative_pos_m = {294102592, 14365960, -8722842},
        .target_relative_vel_m_s = {-9830400, 355623, 327680},
        .estimated_closing_velocity_m_s = 9842287,
        .commanded_los_rate_rad_s = {650, -195, 0}
    },
    /* Step 170: t=3.40s, Range=4490.0m */
    {
        .target_relative_pos_m = {293904993, 14381426, -8716288},
        .target_relative_vel_m_s = {-9830400, 342817, 327680},
        .estimated_closing_velocity_m_s = 9841832,
        .commanded_los_rate_rad_s = {641, -192, 0}
    },
    /* Step 171: t=3.42s, Range=4487.0m */
    {
        .target_relative_pos_m = {293707438, 14395939, -8709734},
        .target_relative_vel_m_s = {-9830400, 329989, 327680},
        .estimated_closing_velocity_m_s = 9841394,
        .commanded_los_rate_rad_s = {632, -190, 0}
    },
    /* Step 172: t=3.44s, Range=4484.0m */
    {
        .target_relative_pos_m = {293509929, 14409500, -8703181},
        .target_relative_vel_m_s = {-9830400, 317140, 327680},
        .estimated_closing_velocity_m_s = 9840971,
        .commanded_los_rate_rad_s = {623, -187, 0}
    },
    /* Step 173: t=3.46s, Range=4481.0m */
    {
        .target_relative_pos_m = {293312464, 14422109, -8696627},
        .target_relative_vel_m_s = {-9830400, 304270, 327680},
        .estimated_closing_velocity_m_s = 9840565,
        .commanded_los_rate_rad_s = {614, -184, 0}
    },
    /* Step 174: t=3.48s, Range=4478.0m */
    {
        .target_relative_pos_m = {293115045, 14433767, -8690074},
        .target_relative_vel_m_s = {-9830400, 291382, 327680},
        .estimated_closing_velocity_m_s = 9840175,
        .commanded_los_rate_rad_s = {605, -182, 0}
    },
    /* Step 175: t=3.50s, Range=4475.0m */
    {
        .target_relative_pos_m = {292917670, 14444473, -8683520},
        .target_relative_vel_m_s = {-9830400, 278474, 327680},
        .estimated_closing_velocity_m_s = 9839801,
        .commanded_los_rate_rad_s = {596, -179, 0}
    },
    /* Step 176: t=3.52s, Range=4472.0m */
    {
        .target_relative_pos_m = {292720342, 14454229, -8676966},
        .target_relative_vel_m_s = {-9830400, 265549, 327680},
        .estimated_closing_velocity_m_s = 9839444,
        .commanded_los_rate_rad_s = {587, -176, 0}
    },
    /* Step 177: t=3.54s, Range=4469.0m */
    {
        .target_relative_pos_m = {292523059, 14463035, -8670413},
        .target_relative_vel_m_s = {-9830400, 252607, 327680},
        .estimated_closing_velocity_m_s = 9839103,
        .commanded_los_rate_rad_s = {578, -173, 0}
    },
    /* Step 178: t=3.56s, Range=4466.0m */
    {
        .target_relative_pos_m = {292325822, 14470892, -8663859},
        .target_relative_vel_m_s = {-9830400, 239648, 327680},
        .estimated_closing_velocity_m_s = 9838779,
        .commanded_los_rate_rad_s = {568, -171, 0}
    },
    /* Step 179: t=3.58s, Range=4463.0m */
    {
        .target_relative_pos_m = {292128630, 14477801, -8657306},
        .target_relative_vel_m_s = {-9830400, 226674, 327680},
        .estimated_closing_velocity_m_s = 9838471,
        .commanded_los_rate_rad_s = {559, -168, 0}
    },
    /* Step 180: t=3.60s, Range=4460.0m */
    {
        .target_relative_pos_m = {291931485, 14483762, -8650752},
        .target_relative_vel_m_s = {-9830400, 213686, 327680},
        .estimated_closing_velocity_m_s = 9838181,
        .commanded_los_rate_rad_s = {550, -165, 0}
    },
    /* Step 181: t=3.62s, Range=4457.0m */
    {
        .target_relative_pos_m = {291734386, 14488776, -8644198},
        .target_relative_vel_m_s = {-9830400, 200684, 327680},
        .estimated_closing_velocity_m_s = 9837907,
        .commanded_los_rate_rad_s = {540, -162, 0}
    },
    /* Step 182: t=3.64s, Range=4454.0m */
    {
        .target_relative_pos_m = {291537334, 14492845, -8637645},
        .target_relative_vel_m_s = {-9830400, 187669, 327680},
        .estimated_closing_velocity_m_s = 9837650,
        .commanded_los_rate_rad_s = {531, -159, 0}
    },
    /* Step 183: t=3.66s, Range=4451.0m */
    {
        .target_relative_pos_m = {291340327, 14495968, -8631091},
        .target_relative_vel_m_s = {-9830400, 174643, 327680},
        .estimated_closing_velocity_m_s = 9837410,
        .commanded_los_rate_rad_s = {521, -156, 0}
    },
    /* Step 184: t=3.68s, Range=4448.0m */
    {
        .target_relative_pos_m = {291143367, 14498148, -8624538},
        .target_relative_vel_m_s = {-9830400, 161605, 327680},
        .estimated_closing_velocity_m_s = 9837187,
        .commanded_los_rate_rad_s = {512, -154, 0}
    },
    /* Step 185: t=3.70s, Range=4445.0m */
    {
        .target_relative_pos_m = {290946454, 14499385, -8617984},
        .target_relative_vel_m_s = {-9830400, 148556, 327680},
        .estimated_closing_velocity_m_s = 9836982,
        .commanded_los_rate_rad_s = {502, -151, 0}
    },
    /* Step 186: t=3.72s, Range=4442.0m */
    {
        .target_relative_pos_m = {290749587, 14499681, -8611430},
        .target_relative_vel_m_s = {-9830400, 135499, 327680},
        .estimated_closing_velocity_m_s = 9836793,
        .commanded_los_rate_rad_s = {492, -148, 0}
    },
    /* Step 187: t=3.74s, Range=4439.0m */
    {
        .target_relative_pos_m = {290552767, 14499037, -8604877},
        .target_relative_vel_m_s = {-9830400, 122432, 327680},
        .estimated_closing_velocity_m_s = 9836622,
        .commanded_los_rate_rad_s = {483, -145, 0}
    },
    /* Step 188: t=3.76s, Range=4436.0m */
    {
        .target_relative_pos_m = {290355993, 14497454, -8598323},
        .target_relative_vel_m_s = {-9830400, 109358, 327680},
        .estimated_closing_velocity_m_s = 9836468,
        .commanded_los_rate_rad_s = {473, -142, 0}
    },
    /* Step 189: t=3.78s, Range=4433.0m */
    {
        .target_relative_pos_m = {290159266, 14494933, -8591770},
        .target_relative_vel_m_s = {-9830400, 96276, 327680},
        .estimated_closing_velocity_m_s = 9836331,
        .commanded_los_rate_rad_s = {463, -139, 0}
    },
    /* Step 190: t=3.80s, Range=4430.0m */
    {
        .target_relative_pos_m = {289962585, 14491476, -8585216},
        .target_relative_vel_m_s = {-9830400, 83189, 327680},
        .estimated_closing_velocity_m_s = 9836212,
        .commanded_los_rate_rad_s = {453, -136, 0}
    },
    /* Step 191: t=3.82s, Range=4427.0m */
    {
        .target_relative_pos_m = {289765951, 14487084, -8578662},
        .target_relative_vel_m_s = {-9830400, 70096, 327680},
        .estimated_closing_velocity_m_s = 9836110,
        .commanded_los_rate_rad_s = {443, -133, 0}
    },
    /* Step 192: t=3.84s, Range=4424.0m */
    {
        .target_relative_pos_m = {289569364, 14481759, -8572109},
        .target_relative_vel_m_s = {-9830400, 56999, 327680},
        .estimated_closing_velocity_m_s = 9836025,
        .commanded_los_rate_rad_s = {434, -130, 0}
    },
    /* Step 193: t=3.86s, Range=4421.0m */
    {
        .target_relative_pos_m = {289372823, 14475503, -8565555},
        .target_relative_vel_m_s = {-9830400, 43898, 327680},
        .estimated_closing_velocity_m_s = 9835958,
        .commanded_los_rate_rad_s = {424, -127, 0}
    },
    /* Step 194: t=3.88s, Range=4418.0m */
    {
        .target_relative_pos_m = {289176329, 14468317, -8559002},
        .target_relative_vel_m_s = {-9830400, 30794, 327680},
        .estimated_closing_velocity_m_s = 9835908,
        .commanded_los_rate_rad_s = {414, -124, 0}
    },
    /* Step 195: t=3.90s, Range=4415.0m */
    {
        .target_relative_pos_m = {288979881, 14460203, -8552448},
        .target_relative_vel_m_s = {-9830400, 17688, 327680},
        .estimated_closing_velocity_m_s = 9835876,
        .commanded_los_rate_rad_s = {403, -121, 0}
    },
    /* Step 196: t=3.92s, Range=4412.0m */
    {
        .target_relative_pos_m = {288783479, 14451162, -8545894},
        .target_relative_vel_m_s = {-9830400, 4581, 327680},
        .estimated_closing_velocity_m_s = 9835861,
        .commanded_los_rate_rad_s = {393, -118, 0}
    },
    /* Step 197: t=3.94s, Range=4409.0m */
    {
        .target_relative_pos_m = {288587124, 14441197, -8539341},
        .target_relative_vel_m_s = {-9830400, -8526, 327680},
        .estimated_closing_velocity_m_s = 9835864,
        .commanded_los_rate_rad_s = {383, -115, 0}
    },
    /* Step 198: t=3.96s, Range=4406.0m */
    {
        .target_relative_pos_m = {288390815, 14430309, -8532787},
        .target_relative_vel_m_s = {-9830400, -21632, 327680},
        .estimated_closing_velocity_m_s = 9835884,
        .commanded_los_rate_rad_s = {373, -112, 0}
    },
    /* Step 199: t=3.98s, Range=4403.0m */
    {
        .target_relative_pos_m = {288194551, 14418500, -8526234},
        .target_relative_vel_m_s = {-9830400, -34737, 327680},
        .estimated_closing_velocity_m_s = 9835921,
        .commanded_los_rate_rad_s = {363, -109, 0}
    },
    /* Step 200: t=4.00s, Range=4400.0m */
    {
        .target_relative_pos_m = {287998334, 14405773, -8519680},
        .target_relative_vel_m_s = {-9830400, -47840, 327680},
        .estimated_closing_velocity_m_s = 9835976,
        .commanded_los_rate_rad_s = {353, -106, 0}
    },
    /* Step 201: t=4.02s, Range=4397.0m */
    {
        .target_relative_pos_m = {287802163, 14392129, -8513126},
        .target_relative_vel_m_s = {-9830400, -60940, 327680},
        .estimated_closing_velocity_m_s = 9836049,
        .commanded_los_rate_rad_s = {342, -103, 0}
    },
    /* Step 202: t=4.04s, Range=4394.0m */
    {
        .target_relative_pos_m = {287606037, 14377571, -8506573},
        .target_relative_vel_m_s = {-9830400, -74036, 327680},
        .estimated_closing_velocity_m_s = 9836138,
        .commanded_los_rate_rad_s = {332, -100, 0}
    },
    /* Step 203: t=4.06s, Range=4391.0m */
    {
        .target_relative_pos_m = {287409957, 14362100, -8500019},
        .target_relative_vel_m_s = {-9830400, -87128, 327680},
        .estimated_closing_velocity_m_s = 9836246,
        .commanded_los_rate_rad_s = {322, -97, 0}
    },
    /* Step 204: t=4.08s, Range=4388.0m */
    {
        .target_relative_pos_m = {287213922, 14345720, -8493466},
        .target_relative_vel_m_s = {-9830400, -100214, 327680},
        .estimated_closing_velocity_m_s = 9836370,
        .commanded_los_rate_rad_s = {311, -93, 0}
    },
    /* Step 205: t=4.10s, Range=4385.0m */
    {
        .target_relative_pos_m = {287017933, 14328431, -8486912},
        .target_relative_vel_m_s = {-9830400, -113293, 327680},
        .estimated_closing_velocity_m_s = 9836512,
        .commanded_los_rate_rad_s = {301, -90, 0}
    },
    /* Step 206: t=4.12s, Range=4382.0m */
    {
        .target_relative_pos_m = {286821988, 14310237, -8480358},
        .target_relative_vel_m_s = {-9830400, -126365, 327680},
        .estimated_closing_velocity_m_s = 9836672,
        .commanded_los_rate_rad_s = {290, -87, 0}
    },
    /* Step 207: t=4.14s, Range=4379.0m */
    {
        .target_relative_pos_m = {286626088, 14291141, -8473805},
        .target_relative_vel_m_s = {-9830400, -139429, 327680},
        .estimated_closing_velocity_m_s = 9836848,
        .commanded_los_rate_rad_s = {280, -84, 0}
    },
    /* Step 208: t=4.16s, Range=4376.0m */
    {
        .target_relative_pos_m = {286430233, 14271143, -8467251},
        .target_relative_vel_m_s = {-9830400, -152484, 327680},
        .estimated_closing_velocity_m_s = 9837042,
        .commanded_los_rate_rad_s = {269, -81, 0}
    },
    /* Step 209: t=4.18s, Range=4373.0m */
    {
        .target_relative_pos_m = {286234422, 14250248, -8460698},
        .target_relative_vel_m_s = {-9830400, -165529, 327680},
        .estimated_closing_velocity_m_s = 9837253,
        .commanded_los_rate_rad_s = {259, -78, 0}
    },
    /* Step 210: t=4.20s, Range=4370.0m */
    {
        .target_relative_pos_m = {286038655, 14228457, -8454144},
        .target_relative_vel_m_s = {-9830400, -178564, 327680},
        .estimated_closing_velocity_m_s = 9837481,
        .commanded_los_rate_rad_s = {248, -74, 0}
    },
    /* Step 211: t=4.22s, Range=4367.0m */
    {
        .target_relative_pos_m = {285842932, 14205774, -8447590},
        .target_relative_vel_m_s = {-9830400, -191587, 327680},
        .estimated_closing_velocity_m_s = 9837726,
        .commanded_los_rate_rad_s = {238, -71, 0}
    },
    /* Step 212: t=4.24s, Range=4364.0m */
    {
        .target_relative_pos_m = {285647252, 14182200, -8441037},
        .target_relative_vel_m_s = {-9830400, -204598, 327680},
        .estimated_closing_velocity_m_s = 9837988,
        .commanded_los_rate_rad_s = {227, -68, 0}
    },
    /* Step 213: t=4.26s, Range=4361.0m */
    {
        .target_relative_pos_m = {285451616, 14157739, -8434483},
        .target_relative_vel_m_s = {-9830400, -217596, 327680},
        .estimated_closing_velocity_m_s = 9838266,
        .commanded_los_rate_rad_s = {216, -65, 0}
    },
    /* Step 214: t=4.28s, Range=4358.0m */
    {
        .target_relative_pos_m = {285256023, 14132394, -8427930},
        .target_relative_vel_m_s = {-9830400, -230580, 327680},
        .estimated_closing_velocity_m_s = 9838562,
        .commanded_los_rate_rad_s = {206, -62, 0}
    },
    /* Step 215: t=4.30s, Range=4355.0m */
    {
        .target_relative_pos_m = {285060473, 14106166, -8421376},
        .target_relative_vel_m_s = {-9830400, -243549, 327680},
        .estimated_closing_velocity_m_s = 9838875,
        .commanded_los_rate_rad_s = {195, -58, 0}
    },
    /* Step 216: t=4.32s, Range=4352.0m */
    {
        .target_relative_pos_m = {284864965, 14079061, -8414822},
        .target_relative_vel_m_s = {-9830400, -256503, 327680},
        .estimated_closing_velocity_m_s = 9839204,
        .commanded_los_rate_rad_s = {184, -55, 0}
    },
    /* Step 217: t=4.34s, Range=4349.0m */
    {
        .target_relative_pos_m = {284669499, 14051079, -8408269},
        .target_relative_vel_m_s = {-9830400, -269440, 327680},
        .estimated_closing_velocity_m_s = 9839550,
        .commanded_los_rate_rad_s = {173, -52, 0}
    },
    /* Step 218: t=4.36s, Range=4346.0m */
    {
        .target_relative_pos_m = {284474076, 14022224, -8401715},
        .target_relative_vel_m_s = {-9830400, -282360, 327680},
        .estimated_closing_velocity_m_s = 9839912,
        .commanded_los_rate_rad_s = {163, -49, 0}
    },
    /* Step 219: t=4.38s, Range=4343.0m */
    {
        .target_relative_pos_m = {284278693, 13992500, -8395162},
        .target_relative_vel_m_s = {-9830400, -295262, 327680},
        .estimated_closing_velocity_m_s = 9840291,
        .commanded_los_rate_rad_s = {152, -46, 0}
    },
    /* Step 220: t=4.40s, Range=4340.0m */
    {
        .target_relative_pos_m = {284083352, 13961910, -8388608},
        .target_relative_vel_m_s = {-9830400, -308145, 327680},
        .estimated_closing_velocity_m_s = 9840686,
        .commanded_los_rate_rad_s = {141, -42, 0}
    },
    /* Step 221: t=4.42s, Range=4337.0m */
    {
        .target_relative_pos_m = {283888052, 13930456, -8382054},
        .target_relative_vel_m_s = {-9830400, -321008, 327680},
        .estimated_closing_velocity_m_s = 9841097,
        .commanded_los_rate_rad_s = {130, -39, 0}
    },
    /* Step 222: t=4.44s, Range=4334.0m */
    {
        .target_relative_pos_m = {283692792, 13898141, -8375501},
        .target_relative_vel_m_s = {-9830400, -333851, 327680},
        .estimated_closing_velocity_m_s = 9841524,
        .commanded_los_rate_rad_s = {119, -36, 0}
    },
    /* Step 223: t=4.46s, Range=4331.0m */
    {
        .target_relative_pos_m = {283497573, 13864970, -8368947},
        .target_relative_vel_m_s = {-9830400, -346673, 327680},
        .estimated_closing_velocity_m_s = 9841967,
        .commanded_los_rate_rad_s = {108, -32, 0}
    },
    /* Step 224: t=4.48s, Range=4328.0m */
    {
        .target_relative_pos_m = {283302393, 13830946, -8362394},
        .target_relative_vel_m_s = {-9830400, -359472, 327680},
        .estimated_closing_velocity_m_s = 9842426,
        .commanded_los_rate_rad_s = {97, -29, 0}
    },
    /* Step 225: t=4.50s, Range=4325.0m */
    {
        .target_relative_pos_m = {283107252, 13796071, -8355840},
        .target_relative_vel_m_s = {-9830400, -372248, 327680},
        .estimated_closing_velocity_m_s = 9842901,
        .commanded_los_rate_rad_s = {86, -26, 0}
    },
    /* Step 226: t=4.52s, Range=4322.0m */
    {
        .target_relative_pos_m = {282912150, 13760350, -8349286},
        .target_relative_vel_m_s = {-9830400, -385000, 327680},
        .estimated_closing_velocity_m_s = 9843392,
        .commanded_los_rate_rad_s = {75, -23, 0}
    },
    /* Step 227: t=4.54s, Range=4319.0m */
    {
        .target_relative_pos_m = {282717087, 13723786, -8342733},
        .target_relative_vel_m_s = {-9830400, -397728, 327680},
        .estimated_closing_velocity_m_s = 9843898,
        .commanded_los_rate_rad_s = {64, -19, 0}
    },
    /* Step 228: t=4.56s, Range=4316.0m */
    {
        .target_relative_pos_m = {282522062, 13686381, -8336179},
        .target_relative_vel_m_s = {-9830400, -410430, 327680},
        .estimated_closing_velocity_m_s = 9844419,
        .commanded_los_rate_rad_s = {53, -16, 0}
    },
    /* Step 229: t=4.58s, Range=4313.0m */
    {
        .target_relative_pos_m = {282327074, 13648141, -8329626},
        .target_relative_vel_m_s = {-9830400, -423106, 327680},
        .estimated_closing_velocity_m_s = 9844956,
        .commanded_los_rate_rad_s = {42, -13, 0}
    },
    /* Step 230: t=4.60s, Range=4310.0m */
    {
        .target_relative_pos_m = {282132124, 13609069, -8323072},
        .target_relative_vel_m_s = {-9830400, -435755, 327680},
        .estimated_closing_velocity_m_s = 9845508,
        .commanded_los_rate_rad_s = {31, -9, 0}
    },
    /* Step 231: t=4.62s, Range=4307.0m */
    {
        .target_relative_pos_m = {281937210, 13569168, -8316518},
        .target_relative_vel_m_s = {-9830400, -448376, 327680},
        .estimated_closing_velocity_m_s = 9846074,
        .commanded_los_rate_rad_s = {20, -6, 0}
    },
    /* Step 232: t=4.64s, Range=4304.0m */
    {
        .target_relative_pos_m = {281742333, 13528442, -8309965},
        .target_relative_vel_m_s = {-9830400, -460969, 327680},
        .estimated_closing_velocity_m_s = 9846656,
        .commanded_los_rate_rad_s = {9, -3, 0}
    },
    /* Step 233: t=4.66s, Range=4301.0m */
    {
        .target_relative_pos_m = {281547492, 13486894, -8303411},
        .target_relative_vel_m_s = {-9830400, -473531, 327680},
        .estimated_closing_velocity_m_s = 9847252,
        .commanded_los_rate_rad_s = {-2, 1, 0}
    },
    /* Step 234: t=4.68s, Range=4298.0m */
    {
        .target_relative_pos_m = {281352686, 13444530, -8296858},
        .target_relative_vel_m_s = {-9830400, -486064, 327680},
        .estimated_closing_velocity_m_s = 9847863,
        .commanded_los_rate_rad_s = {-13, 4, 0}
    },
    /* Step 235: t=4.70s, Range=4295.0m */
    {
        .target_relative_pos_m = {281157914, 13401352, -8290304},
        .target_relative_vel_m_s = {-9830400, -498565, 327680},
        .estimated_closing_velocity_m_s = 9848487,
        .commanded_los_rate_rad_s = {-24, 7, 0}
    },
    /* Step 236: t=4.72s, Range=4292.0m */
    {
        .target_relative_pos_m = {280963178, 13357365, -8283750},
        .target_relative_vel_m_s = {-9830400, -511035, 327680},
        .estimated_closing_velocity_m_s = 9849127,
        .commanded_los_rate_rad_s = {-36, 11, 0}
    },
    /* Step 237: t=4.74s, Range=4289.0m */
    {
        .target_relative_pos_m = {280768475, 13312572, -8277197},
        .target_relative_vel_m_s = {-9830400, -523472, 327680},
        .estimated_closing_velocity_m_s = 9849780,
        .commanded_los_rate_rad_s = {-47, 14, 0}
    },
    /* Step 238: t=4.76s, Range=4286.0m */
    {
        .target_relative_pos_m = {280573806, 13266978, -8270643},
        .target_relative_vel_m_s = {-9830400, -535875, 327680},
        .estimated_closing_velocity_m_s = 9850447,
        .commanded_los_rate_rad_s = {-58, 17, 0}
    },
    /* Step 239: t=4.78s, Range=4283.0m */
    {
        .target_relative_pos_m = {280379169, 13220587, -8264090},
        .target_relative_vel_m_s = {-9830400, -548244, 327680},
        .estimated_closing_velocity_m_s = 9851127,
        .commanded_los_rate_rad_s = {-69, 21, 0}
    },
    /* Step 240: t=4.80s, Range=4280.0m */
    {
        .target_relative_pos_m = {280184565, 13173402, -8257536},
        .target_relative_vel_m_s = {-9830400, -560578, 327680},
        .estimated_closing_velocity_m_s = 9851821,
        .commanded_los_rate_rad_s = {-80, 24, 0}
    },
    /* Step 241: t=4.82s, Range=4277.0m */
    {
        .target_relative_pos_m = {279989993, 13125429, -8250982},
        .target_relative_vel_m_s = {-9830400, -572876, 327680},
        .estimated_closing_velocity_m_s = 9852529,
        .commanded_los_rate_rad_s = {-92, 27, 0}
    },
    /* Step 242: t=4.84s, Range=4274.0m */
    {
        .target_relative_pos_m = {279795452, 13076670, -8244429},
        .target_relative_vel_m_s = {-9830400, -585137, 327680},
        .estimated_closing_velocity_m_s = 9853249,
        .commanded_los_rate_rad_s = {-103, 31, 0}
    },
    /* Step 243: t=4.86s, Range=4271.0m */
    {
        .target_relative_pos_m = {279600941, 13027132, -8237875},
        .target_relative_vel_m_s = {-9830400, -597361, 327680},
        .estimated_closing_velocity_m_s = 9853983,
        .commanded_los_rate_rad_s = {-114, 34, 0}
    },
    /* Step 244: t=4.88s, Range=4268.0m */
    {
        .target_relative_pos_m = {279406461, 12976817, -8231322},
        .target_relative_vel_m_s = {-9830400, -609547, 327680},
        .estimated_closing_velocity_m_s = 9854729,
        .commanded_los_rate_rad_s = {-125, 38, 0}
    },
    /* Step 245: t=4.90s, Range=4265.0m */
    {
        .target_relative_pos_m = {279212011, 12925730, -8224768},
        .target_relative_vel_m_s = {-9830400, -621694, 327680},
        .estimated_closing_velocity_m_s = 9855488,
        .commanded_los_rate_rad_s = {-137, 41, 0}
    },
    /* Step 246: t=4.92s, Range=4262.0m */
    {
        .target_relative_pos_m = {279017589, 12873876, -8218214},
        .target_relative_vel_m_s = {-9830400, -633801, 327680},
        .estimated_closing_velocity_m_s = 9856259,
        .commanded_los_rate_rad_s = {-148, 44, 0}
    },
    /* Step 247: t=4.94s, Range=4259.0m */
    {
        .target_relative_pos_m = {278823197, 12821260, -8211661},
        .target_relative_vel_m_s = {-9830400, -645867, 327680},
        .estimated_closing_velocity_m_s = 9857042,
        .commanded_los_rate_rad_s = {-159, 48, 0}
    },
    /* Step 248: t=4.96s, Range=4256.0m */
    {
        .target_relative_pos_m = {278628832, 12767885, -8205107},
        .target_relative_vel_m_s = {-9830400, -657892, 327680},
        .estimated_closing_velocity_m_s = 9857838,
        .commanded_los_rate_rad_s = {-170, 51, 0}
    },
    /* Step 249: t=4.98s, Range=4253.0m */
    {
        .target_relative_pos_m = {278434494, 12713756, -8198554},
        .target_relative_vel_m_s = {-9830400, -669875, 327680},
        .estimated_closing_velocity_m_s = 9858644,
        .commanded_los_rate_rad_s = {-182, 55, 0}
    },
    /* Step 250: t=5.00s, Range=4250.0m */
    {
        .target_relative_pos_m = {278240183, 12658878, -8192000},
        .target_relative_vel_m_s = {-9830400, -681815, 327680},
        .estimated_closing_velocity_m_s = 9859463,
        .commanded_los_rate_rad_s = {-193, 58, 0}
    },
    /* Step 251: t=5.02s, Range=4247.0m */
    {
        .target_relative_pos_m = {278045899, 12603255, -8185446},
        .target_relative_vel_m_s = {-9830400, -693711, 327680},
        .estimated_closing_velocity_m_s = 9860293,
        .commanded_los_rate_rad_s = {-204, 61, 0}
    },
    /* Step 252: t=5.04s, Range=4244.0m */
    {
        .target_relative_pos_m = {277851639, 12546892, -8178893},
        .target_relative_vel_m_s = {-9830400, -705563, 327680},
        .estimated_closing_velocity_m_s = 9861134,
        .commanded_los_rate_rad_s = {-216, 65, 0}
    },
    /* Step 253: t=5.06s, Range=4241.0m */
    {
        .target_relative_pos_m = {277657405, 12489795, -8172339},
        .target_relative_vel_m_s = {-9830400, -717370, 327680},
        .estimated_closing_velocity_m_s = 9861986,
        .commanded_los_rate_rad_s = {-227, 68, 0}
    },
    /* Step 254: t=5.08s, Range=4238.0m */
    {
        .target_relative_pos_m = {277463195, 12431966, -8165786},
        .target_relative_vel_m_s = {-9830400, -729131, 327680},
        .estimated_closing_velocity_m_s = 9862848,
        .commanded_los_rate_rad_s = {-238, 71, 0}
    },
    /* Step 255: t=5.10s, Range=4235.0m */
    {
        .target_relative_pos_m = {277269009, 12373413, -8159232},
        .target_relative_vel_m_s = {-9830400, -740845, 327680},
        .estimated_closing_velocity_m_s = 9863721,
        .commanded_los_rate_rad_s = {-249, 75, 0}
    },
    /* Step 256: t=5.12s, Range=4232.0m */
    {
        .target_relative_pos_m = {277074846, 12314138, -8152678},
        .target_relative_vel_m_s = {-9830400, -752512, 327680},
        .estimated_closing_velocity_m_s = 9864604,
        .commanded_los_rate_rad_s = {-261, 78, 0}
    },
    /* Step 257: t=5.14s, Range=4229.0m */
    {
        .target_relative_pos_m = {276880705, 12254148, -8146125},
        .target_relative_vel_m_s = {-9830400, -764131, 327680},
        .estimated_closing_velocity_m_s = 9865497,
        .commanded_los_rate_rad_s = {-272, 82, 0}
    },
    /* Step 258: t=5.16s, Range=4226.0m */
    {
        .target_relative_pos_m = {276686587, 12193447, -8139571},
        .target_relative_vel_m_s = {-9830400, -775701, 327680},
        .estimated_closing_velocity_m_s = 9866400,
        .commanded_los_rate_rad_s = {-283, 85, 0}
    },
    /* Step 259: t=5.18s, Range=4223.0m */
    {
        .target_relative_pos_m = {276492489, 12132040, -8133018},
        .target_relative_vel_m_s = {-9830400, -787221, 327680},
        .estimated_closing_velocity_m_s = 9867312,
        .commanded_los_rate_rad_s = {-295, 88, 0}
    },
    /* Step 260: t=5.20s, Range=4220.0m */
    {
        .target_relative_pos_m = {276298412, 12069932, -8126464},
        .target_relative_vel_m_s = {-9830400, -798691, 327680},
        .estimated_closing_velocity_m_s = 9868234,
        .commanded_los_rate_rad_s = {-306, 92, 0}
    },
    /* Step 261: t=5.22s, Range=4217.0m */
    {
        .target_relative_pos_m = {276104354, 12007128, -8119910},
        .target_relative_vel_m_s = {-9830400, -810109, 327680},
        .estimated_closing_velocity_m_s = 9869165,
        .commanded_los_rate_rad_s = {-317, 95, 0}
    },
    /* Step 262: t=5.24s, Range=4214.0m */
    {
        .target_relative_pos_m = {275910316, 11943633, -8113357},
        .target_relative_vel_m_s = {-9830400, -821476, 327680},
        .estimated_closing_velocity_m_s = 9870104,
        .commanded_los_rate_rad_s = {-329, 99, 0}
    },
    /* Step 263: t=5.26s, Range=4211.0m */
    {
        .target_relative_pos_m = {275716297, 11879453, -8106803},
        .target_relative_vel_m_s = {-9830400, -832790, 327680},
        .estimated_closing_velocity_m_s = 9871053,
        .commanded_los_rate_rad_s = {-340, 102, 0}
    },
    /* Step 264: t=5.28s, Range=4208.0m */
    {
        .target_relative_pos_m = {275522295, 11814593, -8100250},
        .target_relative_vel_m_s = {-9830400, -844051, 327680},
        .estimated_closing_velocity_m_s = 9872009,
        .commanded_los_rate_rad_s = {-351, 105, 0}
    },
    /* Step 265: t=5.30s, Range=4205.0m */
    {
        .target_relative_pos_m = {275328311, 11749057, -8093696},
        .target_relative_vel_m_s = {-9830400, -855258, 327680},
        .estimated_closing_velocity_m_s = 9872973,
        .commanded_los_rate_rad_s = {-362, 109, 0}
    },
    /* Step 266: t=5.32s, Range=4202.0m */
    {
        .target_relative_pos_m = {275134343, 11682851, -8087142},
        .target_relative_vel_m_s = {-9830400, -866410, 327680},
        .estimated_closing_velocity_m_s = 9873946,
        .commanded_los_rate_rad_s = {-374, 112, 0}
    },
    /* Step 267: t=5.34s, Range=4199.0m */
    {
        .target_relative_pos_m = {274940391, 11615981, -8080589},
        .target_relative_vel_m_s = {-9830400, -877507, 327680},
        .estimated_closing_velocity_m_s = 9874926,
        .commanded_los_rate_rad_s = {-385, 115, 0}
    },
    /* Step 268: t=5.36s, Range=4196.0m */
    {
        .target_relative_pos_m = {274746454, 11548451, -8074035},
        .target_relative_vel_m_s = {-9830400, -888548, 327680},
        .estimated_closing_velocity_m_s = 9875913,
        .commanded_los_rate_rad_s = {-396, 119, 0}
    },
    /* Step 269: t=5.38s, Range=4193.0m */
    {
        .target_relative_pos_m = {274552532, 11480267, -8067482},
        .target_relative_vel_m_s = {-9830400, -899531, 327680},
        .estimated_closing_velocity_m_s = 9876907,
        .commanded_los_rate_rad_s = {-407, 122, 0}
    },
    /* Step 270: t=5.40s, Range=4190.0m */
    {
        .target_relative_pos_m = {274358624, 11411435, -8060928},
        .target_relative_vel_m_s = {-9830400, -910457, 327680},
        .estimated_closing_velocity_m_s = 9877908,
        .commanded_los_rate_rad_s = {-419, 126, 0}
    },
    /* Step 271: t=5.42s, Range=4187.0m */
    {
        .target_relative_pos_m = {274164729, 11341959, -8054374},
        .target_relative_vel_m_s = {-9830400, -921325, 327680},
        .estimated_closing_velocity_m_s = 9878916,
        .commanded_los_rate_rad_s = {-430, 129, 0}
    },
    /* Step 272: t=5.44s, Range=4184.0m */
    {
        .target_relative_pos_m = {273970846, 11271846, -8047821},
        .target_relative_vel_m_s = {-9830400, -932134, 327680},
        .estimated_closing_velocity_m_s = 9879930,
        .commanded_los_rate_rad_s = {-441, 132, 0}
    },
    /* Step 273: t=5.46s, Range=4181.0m */
    {
        .target_relative_pos_m = {273776975, 11201101, -8041267},
        .target_relative_vel_m_s = {-9830400, -942883, 327680},
        .estimated_closing_velocity_m_s = 9880950,
        .commanded_los_rate_rad_s = {-452, 136, 0}
    },
    /* Step 274: t=5.48s, Range=4178.0m */
    {
        .target_relative_pos_m = {273583115, 11129729, -8034714},
        .target_relative_vel_m_s = {-9830400, -953572, 327680},
        .estimated_closing_velocity_m_s = 9881975,
        .commanded_los_rate_rad_s = {-463, 139, 0}
    },
    /* Step 275: t=5.50s, Range=4175.0m */
    {
        .target_relative_pos_m = {273389266, 11057736, -8028160},
        .target_relative_vel_m_s = {-9830400, -964200, 327680},
        .estimated_closing_velocity_m_s = 9883007,
        .commanded_los_rate_rad_s = {-475, 142, 0}
    },
    /* Step 276: t=5.52s, Range=4172.0m */
    {
        .target_relative_pos_m = {273195426, 10985128, -8021606},
        .target_relative_vel_m_s = {-9830400, -974766, 327680},
        .estimated_closing_velocity_m_s = 9884043,
        .commanded_los_rate_rad_s = {-486, 146, 0}
    },
    /* Step 277: t=5.54s, Range=4169.0m */
    {
        .target_relative_pos_m = {273001596, 10911909, -8015053},
        .target_relative_vel_m_s = {-9830400, -985270, 327680},
        .estimated_closing_velocity_m_s = 9885085,
        .commanded_los_rate_rad_s = {-497, 149, 0}
    },
    /* Step 278: t=5.56s, Range=4166.0m */
    {
        .target_relative_pos_m = {272807774, 10838087, -8008499},
        .target_relative_vel_m_s = {-9830400, -995711, 327680},
        .estimated_closing_velocity_m_s = 9886131,
        .commanded_los_rate_rad_s = {-508, 152, 0}
    },
    /* Step 279: t=5.58s, Range=4163.0m */
    {
        .target_relative_pos_m = {272613959, 10763666, -8001946},
        .target_relative_vel_m_s = {-9830400, -1006088, 327680},
        .estimated_closing_velocity_m_s = 9887181,
        .commanded_los_rate_rad_s = {-519, 156, 0}
    },
    /* Step 280: t=5.60s, Range=4160.0m */
    {
        .target_relative_pos_m = {272420151, 10688653, -7995392},
        .target_relative_vel_m_s = {-9830400, -1016400, 327680},
        .estimated_closing_velocity_m_s = 9888236,
        .commanded_los_rate_rad_s = {-530, 159, 0}
    },
    /* Step 281: t=5.62s, Range=4157.0m */
    {
        .target_relative_pos_m = {272226350, 10613053, -7988838},
        .target_relative_vel_m_s = {-9830400, -1026648, 327680},
        .estimated_closing_velocity_m_s = 9889294,
        .commanded_los_rate_rad_s = {-541, 162, 0}
    },
    /* Step 282: t=5.64s, Range=4154.0m */
    {
        .target_relative_pos_m = {272032554, 10536871, -7982285},
        .target_relative_vel_m_s = {-9830400, -1036830, 327680},
        .estimated_closing_velocity_m_s = 9890357,
        .commanded_los_rate_rad_s = {-552, 166, 0}
    },
    /* Step 283: t=5.66s, Range=4151.0m */
    {
        .target_relative_pos_m = {271838762, 10460115, -7975731},
        .target_relative_vel_m_s = {-9830400, -1046945, 327680},
        .estimated_closing_velocity_m_s = 9891422,
        .commanded_los_rate_rad_s = {-563, 169, 0}
    },
    /* Step 284: t=5.68s, Range=4148.0m */
    {
        .target_relative_pos_m = {271644975, 10382789, -7969178},
        .target_relative_vel_m_s = {-9830400, -1056994, 327680},
        .estimated_closing_velocity_m_s = 9892491,
        .commanded_los_rate_rad_s = {-574, 172, 0}
    },
    /* Step 285: t=5.70s, Range=4145.0m */
    {
        .target_relative_pos_m = {271451192, 10304900, -7962624},
        .target_relative_vel_m_s = {-9830400, -1066975, 327680},
        .estimated_closing_velocity_m_s = 9893562,
        .commanded_los_rate_rad_s = {-585, 176, 0}
    },
    /* Step 286: t=5.72s, Range=4142.0m */
    {
        .target_relative_pos_m = {271257411, 10226453, -7956070},
        .target_relative_vel_m_s = {-9830400, -1076887, 327680},
        .estimated_closing_velocity_m_s = 9894636,
        .commanded_los_rate_rad_s = {-596, 179, 0}
    },
    /* Step 287: t=5.74s, Range=4139.0m */
    {
        .target_relative_pos_m = {271063632, 10147454, -7949517},
        .target_relative_vel_m_s = {-9830400, -1086731, 327680},
        .estimated_closing_velocity_m_s = 9895712,
        .commanded_los_rate_rad_s = {-607, 182, 0}
    },
    /* Step 288: t=5.76s, Range=4136.0m */
    {
        .target_relative_pos_m = {270869854, 10067910, -7942963},
        .target_relative_vel_m_s = {-9830400, -1096505, 327680},
        .estimated_closing_velocity_m_s = 9896790,
        .commanded_los_rate_rad_s = {-618, 185, 0}
    },
    /* Step 289: t=5.78s, Range=4133.0m */
    {
        .target_relative_pos_m = {270676078, 9987826, -7936410},
        .target_relative_vel_m_s = {-9830400, -1106209, 327680},
        .estimated_closing_velocity_m_s = 9897870,
        .commanded_los_rate_rad_s = {-629, 189, 0}
    },
    /* Step 290: t=5.80s, Range=4130.0m */
    {
        .target_relative_pos_m = {270482300, 9907209, -7929856},
        .target_relative_vel_m_s = {-9830400, -1115842, 327680},
        .estimated_closing_velocity_m_s = 9898952,
        .commanded_los_rate_rad_s = {-640, 192, 0}
    },
    /* Step 291: t=5.82s, Range=4127.0m */
    {
        .target_relative_pos_m = {270288523, 9826065, -7923302},
        .target_relative_vel_m_s = {-9830400, -1125404, 327680},
        .estimated_closing_velocity_m_s = 9900034,
        .commanded_los_rate_rad_s = {-651, 195, 0}
    },
    /* Step 292: t=5.84s, Range=4124.0m */
    {
        .target_relative_pos_m = {270094743, 9744399, -7916749},
        .target_relative_vel_m_s = {-9830400, -1134893, 327680},
        .estimated_closing_velocity_m_s = 9901117,
        .commanded_los_rate_rad_s = {-662, 199, 0}
    },
    /* Step 293: t=5.86s, Range=4121.0m */
    {
        .target_relative_pos_m = {269900962, 9662217, -7910195},
        .target_relative_vel_m_s = {-9830400, -1144310, 327680},
        .estimated_closing_velocity_m_s = 9902201,
        .commanded_los_rate_rad_s = {-673, 202, 0}
    },
    /* Step 294: t=5.88s, Range=4118.0m */
    {
        .target_relative_pos_m = {269707178, 9579527, -7903642},
        .target_relative_vel_m_s = {-9830400, -1153654, 327680},
        .estimated_closing_velocity_m_s = 9903285,
        .commanded_los_rate_rad_s = {-683, 205, 0}
    },
    /* Step 295: t=5.90s, Range=4115.0m */
    {
        .target_relative_pos_m = {269513390, 9496334, -7897088},
        .target_relative_vel_m_s = {-9830400, -1162924, 327680},
        .estimated_closing_velocity_m_s = 9904369,
        .commanded_los_rate_rad_s = {-694, 208, 0}
    },
    /* Step 296: t=5.92s, Range=4112.0m */
    {
        .target_relative_pos_m = {269319598, 9412644, -7890534},
        .target_relative_vel_m_s = {-9830400, -1172120, 327680},
        .estimated_closing_velocity_m_s = 9905453,
        .commanded_los_rate_rad_s = {-705, 211, 0}
    },
    /* Step 297: t=5.94s, Range=4109.0m */
    {
        .target_relative_pos_m = {269125801, 9328463, -7883981},
        .target_relative_vel_m_s = {-9830400, -1181240, 327680},
        .estimated_closing_velocity_m_s = 9906537,
        .commanded_los_rate_rad_s = {-715, 215, 0}
    },
    /* Step 298: t=5.96s, Range=4106.0m */
    {
        .target_relative_pos_m = {268931998, 9243799, -7877427},
        .target_relative_vel_m_s = {-9830400, -1190285, 327680},
        .estimated_closing_velocity_m_s = 9907619,
        .commanded_los_rate_rad_s = {-726, 218, 0}
    },
    /* Step 299: t=5.98s, Range=4103.0m */
    {
        .target_relative_pos_m = {268738189, 9158656, -7870874},
        .target_relative_vel_m_s = {-9830400, -1199254, 327680},
        .estimated_closing_velocity_m_s = 9908701,
        .commanded_los_rate_rad_s = {-737, 221, 0}
    },
    /* Step 300: t=6.00s, Range=4100.0m */
    {
        .target_relative_pos_m = {268544373, 9073042, -7864320},
        .target_relative_vel_m_s = {-9830400, -1208146, 327680},
        .estimated_closing_velocity_m_s = 9909781,
        .commanded_los_rate_rad_s = {-747, 224, 0}
    },
    /* Step 301: t=6.02s, Range=4097.0m */
    {
        .target_relative_pos_m = {268350549, 8986962, -7857766},
        .target_relative_vel_m_s = {-9830400, -1216961, 327680},
        .estimated_closing_velocity_m_s = 9910859,
        .commanded_los_rate_rad_s = {-758, 227, 0}
    },
    /* Step 302: t=6.04s, Range=4094.0m */
    {
        .target_relative_pos_m = {268156717, 8900424, -7851213},
        .target_relative_vel_m_s = {-9830400, -1225697, 327680},
        .estimated_closing_velocity_m_s = 9911936,
        .commanded_los_rate_rad_s = {-768, 231, 0}
    },
    /* Step 303: t=6.06s, Range=4091.0m */
    {
        .target_relative_pos_m = {267962876, 8813432, -7844659},
        .target_relative_vel_m_s = {-9830400, -1234356, 327680},
        .estimated_closing_velocity_m_s = 9913010,
        .commanded_los_rate_rad_s = {-779, 234, 0}
    },
    /* Step 304: t=6.08s, Range=4088.0m */
    {
        .target_relative_pos_m = {267769025, 8725995, -7838106},
        .target_relative_vel_m_s = {-9830400, -1242935, 327680},
        .estimated_closing_velocity_m_s = 9914082,
        .commanded_los_rate_rad_s = {-789, 237, 0}
    },
    /* Step 305: t=6.10s, Range=4085.0m */
    {
        .target_relative_pos_m = {267575164, 8638118, -7831552},
        .target_relative_vel_m_s = {-9830400, -1251435, 327680},
        .estimated_closing_velocity_m_s = 9915151,
        .commanded_los_rate_rad_s = {-800, 240, 0}
    },
    /* Step 306: t=6.12s, Range=4082.0m */
    {
        .target_relative_pos_m = {267381292, 8549807, -7824998},
        .target_relative_vel_m_s = {-9830400, -1259855, 327680},
        .estimated_closing_velocity_m_s = 9916218,
        .commanded_los_rate_rad_s = {-810, 243, 0}
    },
    /* Step 307: t=6.14s, Range=4079.0m */
    {
        .target_relative_pos_m = {267187408, 8461070, -7818445},
        .target_relative_vel_m_s = {-9830400, -1268194, 327680},
        .estimated_closing_velocity_m_s = 9917281,
        .commanded_los_rate_rad_s = {-821, 246, 0}
    },
    /* Step 308: t=6.16s, Range=4076.0m */
    {
        .target_relative_pos_m = {266993512, 8371912, -7811891},
        .target_relative_vel_m_s = {-9830400, -1276452, 327680},
        .estimated_closing_velocity_m_s = 9918340,
        .commanded_los_rate_rad_s = {-831, 249, 0}
    },
    /* Step 309: t=6.18s, Range=4073.0m */
    {
        .target_relative_pos_m = {266799603, 8282340, -7805338},
        .target_relative_vel_m_s = {-9830400, -1284628, 327680},
        .estimated_closing_velocity_m_s = 9919395,
        .commanded_los_rate_rad_s = {-841, 252, 0}
    },
    /* Step 310: t=6.20s, Range=4070.0m */
    {
        .target_relative_pos_m = {266605681, 8192360, -7798784},
        .target_relative_vel_m_s = {-9830400, -1292722, 327680},
        .estimated_closing_velocity_m_s = 9920447,
        .commanded_los_rate_rad_s = {-852, 255, 0}
    },
    /* Step 311: t=6.22s, Range=4067.0m */
    {
        .target_relative_pos_m = {266411744, 8101980, -7792230},
        .target_relative_vel_m_s = {-9830400, -1300733, 327680},
        .estimated_closing_velocity_m_s = 9921494,
        .commanded_los_rate_rad_s = {-862, 259, 0}
    },
    /* Step 312: t=6.24s, Range=4064.0m */
    {
        .target_relative_pos_m = {266217792, 8011205, -7785677},
        .target_relative_vel_m_s = {-9830400, -1308661, 327680},
        .estimated_closing_velocity_m_s = 9922537,
        .commanded_los_rate_rad_s = {-872, 262, 0}
    },
    /* Step 313: t=6.26s, Range=4061.0m */
    {
        .target_relative_pos_m = {266023825, 7920043, -7779123},
        .target_relative_vel_m_s = {-9830400, -1316505, 327680},
        .estimated_closing_velocity_m_s = 9923574,
        .commanded_los_rate_rad_s = {-882, 265, 0}
    },
    /* Step 314: t=6.28s, Range=4058.0m */
    {
        .target_relative_pos_m = {265829841, 7828499, -7772570},
        .target_relative_vel_m_s = {-9830400, -1324265, 327680},
        .estimated_closing_velocity_m_s = 9924607,
        .commanded_los_rate_rad_s = {-892, 268, 0}
    },
    /* Step 315: t=6.30s, Range=4055.0m */
    {
        .target_relative_pos_m = {265635841, 7736581, -7766016},
        .target_relative_vel_m_s = {-9830400, -1331941, 327680},
        .estimated_closing_velocity_m_s = 9925634,
        .commanded_los_rate_rad_s = {-902, 271, 0}
    },
    /* Step 316: t=6.32s, Range=4052.0m */
    {
        .target_relative_pos_m = {265441823, 7644295, -7759462},
        .target_relative_vel_m_s = {-9830400, -1339531, 327680},
        .estimated_closing_velocity_m_s = 9926655,
        .commanded_los_rate_rad_s = {-912, 274, 0}
    },
    /* Step 317: t=6.34s, Range=4049.0m */
    {
        .target_relative_pos_m = {265247787, 7551647, -7752909},
        .target_relative_vel_m_s = {-9830400, -1347035, 327680},
        .estimated_closing_velocity_m_s = 9927670,
        .commanded_los_rate_rad_s = {-922, 277, 0}
    },
    /* Step 318: t=6.36s, Range=4046.0m */
    {
        .target_relative_pos_m = {265053733, 7458645, -7746355},
        .target_relative_vel_m_s = {-9830400, -1354453, 327680},
        .estimated_closing_velocity_m_s = 9928680,
        .commanded_los_rate_rad_s = {-932, 280, 0}
    },
    /* Step 319: t=6.38s, Range=4043.0m */
    {
        .target_relative_pos_m = {264859660, 7365294, -7739802},
        .target_relative_vel_m_s = {-9830400, -1361784, 327680},
        .estimated_closing_velocity_m_s = 9929683,
        .commanded_los_rate_rad_s = {-942, 283, 0}
    },
    /* Step 320: t=6.40s, Range=4040.0m */
    {
        .target_relative_pos_m = {264665566, 7271602, -7733248},
        .target_relative_vel_m_s = {-9830400, -1369029, 327680},
        .estimated_closing_velocity_m_s = 9930679,
        .commanded_los_rate_rad_s = {-952, 286, 0}
    },
    /* Step 321: t=6.42s, Range=4037.0m */
    {
        .target_relative_pos_m = {264471453, 7177576, -7726694},
        .target_relative_vel_m_s = {-9830400, -1376185, 327680},
        .estimated_closing_velocity_m_s = 9931668,
        .commanded_los_rate_rad_s = {-961, 288, 0}
    },
    /* Step 322: t=6.44s, Range=4034.0m */
    {
        .target_relative_pos_m = {264277318, 7083221, -7720141},
        .target_relative_vel_m_s = {-9830400, -1383254, 327680},
        .estimated_closing_velocity_m_s = 9932650,
        .commanded_los_rate_rad_s = {-971, 291, 0}
    },
    /* Step 323: t=6.46s, Range=4031.0m */
    {
        .target_relative_pos_m = {264083162, 6988545, -7713587},
        .target_relative_vel_m_s = {-9830400, -1390234, 327680},
        .estimated_closing_velocity_m_s = 9933624,
        .commanded_los_rate_rad_s = {-981, 294, 0}
    },
    /* Step 324: t=6.48s, Range=4028.0m */
    {
        .target_relative_pos_m = {263888983, 6893555, -7707034},
        .target_relative_vel_m_s = {-9830400, -1397125, 327680},
        .estimated_closing_velocity_m_s = 9934591,
        .commanded_los_rate_rad_s = {-991, 297, 0}
    },
    /* Step 325: t=6.50s, Range=4025.0m */
    {
        .target_relative_pos_m = {263694782, 6798257, -7700480},
        .target_relative_vel_m_s = {-9830400, -1403927, 327680},
        .estimated_closing_velocity_m_s = 9935550,
        .commanded_los_rate_rad_s = {-1000, 300, 0}
    },
    /* Step 326: t=6.52s, Range=4022.0m */
    {
        .target_relative_pos_m = {263500558, 6702657, -7693926},
        .target_relative_vel_m_s = {-9830400, -1410638, 327680},
        .estimated_closing_velocity_m_s = 9936500,
        .commanded_los_rate_rad_s = {-1010, 303, 0}
    },
    /* Step 327: t=6.54s, Range=4019.0m */
    {
        .target_relative_pos_m = {263306310, 6606764, -7687373},
        .target_relative_vel_m_s = {-9830400, -1417260, 327680},
        .estimated_closing_velocity_m_s = 9937443,
        .commanded_los_rate_rad_s = {-1019, 306, 0}
    },
    /* Step 328: t=6.56s, Range=4016.0m */
    {
        .target_relative_pos_m = {263112038, 6510583, -7680819},
        .target_relative_vel_m_s = {-9830400, -1423791, 327680},
        .estimated_closing_velocity_m_s = 9938376,
        .commanded_los_rate_rad_s = {-1029, 309, 0}
    },
    /* Step 329: t=6.58s, Range=4013.0m */
    {
        .target_relative_pos_m = {262917740, 6414121, -7674266},
        .target_relative_vel_m_s = {-9830400, -1430230, 327680},
        .estimated_closing_velocity_m_s = 9939301,
        .commanded_los_rate_rad_s = {-1038, 311, 0}
    },
    /* Step 330: t=6.60s, Range=4010.0m */
    {
        .target_relative_pos_m = {262723418, 6317385, -7667712},
        .target_relative_vel_m_s = {-9830400, -1436578, 327680},
        .estimated_closing_velocity_m_s = 9940216,
        .commanded_los_rate_rad_s = {-1047, 314, 0}
    },
    /* Step 331: t=6.62s, Range=4007.0m */
    {
        .target_relative_pos_m = {262529069, 6220382, -7661158},
        .target_relative_vel_m_s = {-9830400, -1442835, 327680},
        .estimated_closing_velocity_m_s = 9941122,
        .commanded_los_rate_rad_s = {-1057, 317, 0}
    },
    /* Step 332: t=6.64s, Range=4004.0m */
    {
        .target_relative_pos_m = {262334694, 6123119, -7654605},
        .target_relative_vel_m_s = {-9830400, -1448998, 327680},
        .estimated_closing_velocity_m_s = 9942019,
        .commanded_los_rate_rad_s = {-1066, 320, 0}
    },
    /* Step 333: t=6.66s, Range=4001.0m */
    {
        .target_relative_pos_m = {262140292, 6025602, -7648051},
        .target_relative_vel_m_s = {-9830400, -1455070, 327680},
        .estimated_closing_velocity_m_s = 9942905,
        .commanded_los_rate_rad_s = {-1075, 322, 0}
    },
    /* Step 334: t=6.68s, Range=3998.0m */
    {
        .target_relative_pos_m = {261945863, 5927839, -7641498},
        .target_relative_vel_m_s = {-9830400, -1461048, 327680},
        .estimated_closing_velocity_m_s = 9943782,
        .commanded_los_rate_rad_s = {-1084, 325, 0}
    },
    /* Step 335: t=6.70s, Range=3995.0m */
    {
        .target_relative_pos_m = {261751406, 5829836, -7634944},
        .target_relative_vel_m_s = {-9830400, -1466932, 327680},
        .estimated_closing_velocity_m_s = 9944648,
        .commanded_los_rate_rad_s = {-1093, 328, 0}
    },
    /* Step 336: t=6.72s, Range=3992.0m */
    {
        .target_relative_pos_m = {261556920, 5731599, -7628390},
        .target_relative_vel_m_s = {-9830400, -1472723, 327680},
        .estimated_closing_velocity_m_s = 9945504,
        .commanded_los_rate_rad_s = {-1102, 331, 0}
    },
    /* Step 337: t=6.74s, Range=3989.0m */
    {
        .target_relative_pos_m = {261362406, 5633137, -7621837},
        .target_relative_vel_m_s = {-9830400, -1478419, 327680},
        .estimated_closing_velocity_m_s = 9946349,
        .commanded_los_rate_rad_s = {-1111, 333, 0}
    },
    /* Step 338: t=6.76s, Range=3986.0m */
    {
        .target_relative_pos_m = {261167862, 5534455, -7615283},
        .target_relative_vel_m_s = {-9830400, -1484021, 327680},
        .estimated_closing_velocity_m_s = 9947183,
        .commanded_los_rate_rad_s = {-1120, 336, 0}
    },
    /* Step 339: t=6.78s, Range=3983.0m */
    {
        .target_relative_pos_m = {260973288, 5435561, -7608730},
        .target_relative_vel_m_s = {-9830400, -1489528, 327680},
        .estimated_closing_velocity_m_s = 9948006,
        .commanded_los_rate_rad_s = {-1129, 339, 0}
    },
    /* Step 340: t=6.80s, Range=3980.0m */
    {
        .target_relative_pos_m = {260778684, 5336461, -7602176},
        .target_relative_vel_m_s = {-9830400, -1494939, 327680},
        .estimated_closing_velocity_m_s = 9948818,
        .commanded_los_rate_rad_s = {-1138, 341, 0}
    },
    /* Step 341: t=6.82s, Range=3977.0m */
    {
        .target_relative_pos_m = {260584050, 5237163, -7595622},
        .target_relative_vel_m_s = {-9830400, -1500255, 327680},
        .estimated_closing_velocity_m_s = 9949618,
        .commanded_los_rate_rad_s = {-1146, 344, 0}
    },
    /* Step 342: t=6.84s, Range=3974.0m */
    {
        .target_relative_pos_m = {260389384, 5137673, -7589069},
        .target_relative_vel_m_s = {-9830400, -1505475, 327680},
        .estimated_closing_velocity_m_s = 9950407,
        .commanded_los_rate_rad_s = {-1155, 346, 0}
    },
    /* Step 343: t=6.86s, Range=3971.0m */
    {
        .target_relative_pos_m = {260194687, 5037997, -7582515},
        .target_relative_vel_m_s = {-9830400, -1510598, 327680},
        .estimated_closing_velocity_m_s = 9951183,
        .commanded_los_rate_rad_s = {-1163, 349, 0}
    },
    /* Step 344: t=6.88s, Range=3968.0m */
    {
        .target_relative_pos_m = {259999957, 4938144, -7575962},
        .target_relative_vel_m_s = {-9830400, -1515625, 327680},
        .estimated_closing_velocity_m_s = 9951947,
        .commanded_los_rate_rad_s = {-1172, 352, 0}
    },
    /* Step 345: t=6.90s, Range=3965.0m */
    {
        .target_relative_pos_m = {259805196, 4838119, -7569408},
        .target_relative_vel_m_s = {-9830400, -1520554, 327680},
        .estimated_closing_velocity_m_s = 9952699,
        .commanded_los_rate_rad_s = {-1180, 354, 0}
    },
    /* Step 346: t=6.92s, Range=3962.0m */
    {
        .target_relative_pos_m = {259610402, 4737931, -7562854},
        .target_relative_vel_m_s = {-9830400, -1525387, 327680},
        .estimated_closing_velocity_m_s = 9953439,
        .commanded_los_rate_rad_s = {-1189, 357, 0}
    },
    /* Step 347: t=6.94s, Range=3959.0m */
    {
        .target_relative_pos_m = {259415574, 4637584, -7556301},
        .target_relative_vel_m_s = {-9830400, -1530122, 327680},
        .estimated_closing_velocity_m_s = 9954165,
        .commanded_los_rate_rad_s = {-1197, 359, 0}
    },
    /* Step 348: t=6.96s, Range=3956.0m */
    {
        .target_relative_pos_m = {259220713, 4537087, -7549747},
        .target_relative_vel_m_s = {-9830400, -1534758, 327680},
        .estimated_closing_velocity_m_s = 9954879,
        .commanded_los_rate_rad_s = {-1205, 362, 0}
    },
    /* Step 349: t=6.98s, Range=3953.0m */
    {
        .target_relative_pos_m = {259025818, 4436447, -7543194},
        .target_relative_vel_m_s = {-9830400, -1539297, 327680},
        .estimated_closing_velocity_m_s = 9955580,
        .commanded_los_rate_rad_s = {-1214, 364, 0}
    },
    /* Step 350: t=7.00s, Range=3950.0m */
    {
        .target_relative_pos_m = {258830889, 4335669, -7536640},
        .target_relative_vel_m_s = {-9830400, -1543737, 327680},
        .estimated_closing_velocity_m_s = 9956267,
        .commanded_los_rate_rad_s = {-1222, 367, 0}
    },
    /* Step 351: t=7.02s, Range=3947.0m */
    {
        .target_relative_pos_m = {258635925, 4234762, -7530086},
        .target_relative_vel_m_s = {-9830400, -1548078, 327680},
        .estimated_closing_velocity_m_s = 9956942,
        .commanded_los_rate_rad_s = {-1230, 369, 0}
    },
    /* Step 352: t=7.04s, Range=3944.0m */
    {
        .target_relative_pos_m = {258440927, 4133732, -7523533},
        .target_relative_vel_m_s = {-9830400, -1552321, 327680},
        .estimated_closing_velocity_m_s = 9957602,
        .commanded_los_rate_rad_s = {-1238, 371, 0}
    },
    /* Step 353: t=7.06s, Range=3941.0m */
    {
        .target_relative_pos_m = {258245893, 4032586, -7516979},
        .target_relative_vel_m_s = {-9830400, -1556464, 327680},
        .estimated_closing_velocity_m_s = 9958249,
        .commanded_los_rate_rad_s = {-1246, 374, 0}
    },
    /* Step 354: t=7.08s, Range=3938.0m */
    {
        .target_relative_pos_m = {258050823, 3931330, -7510426},
        .target_relative_vel_m_s = {-9830400, -1560507, 327680},
        .estimated_closing_velocity_m_s = 9958881,
        .commanded_los_rate_rad_s = {-1254, 376, 0}
    },
    /* Step 355: t=7.10s, Range=3935.0m */
    {
        .target_relative_pos_m = {257855718, 3829972, -7503872},
        .target_relative_vel_m_s = {-9830400, -1564450, 327680},
        .estimated_closing_velocity_m_s = 9959500,
        .commanded_los_rate_rad_s = {-1261, 378, 0}
    },
    /* Step 356: t=7.12s, Range=3932.0m */
    {
        .target_relative_pos_m = {257660576, 3728519, -7497318},
        .target_relative_vel_m_s = {-9830400, -1568293, 327680},
        .estimated_closing_velocity_m_s = 9960105,
        .commanded_los_rate_rad_s = {-1269, 381, 0}
    },
    /* Step 357: t=7.14s, Range=3929.0m */
    {
        .target_relative_pos_m = {257465398, 3626977, -7490765},
        .target_relative_vel_m_s = {-9830400, -1572036, 327680},
        .estimated_closing_velocity_m_s = 9960695,
        .commanded_los_rate_rad_s = {-1277, 383, 0}
    },
    /* Step 358: t=7.16s, Range=3926.0m */
    {
        .target_relative_pos_m = {257270183, 3525353, -7484211},
        .target_relative_vel_m_s = {-9830400, -1575679, 327680},
        .estimated_closing_velocity_m_s = 9961270,
        .commanded_los_rate_rad_s = {-1284, 385, 0}
    },
    /* Step 359: t=7.18s, Range=3923.0m */
    {
        .target_relative_pos_m = {257074931, 3423654, -7477658},
        .target_relative_vel_m_s = {-9830400, -1579220, 327680},
        .estimated_closing_velocity_m_s = 9961831,
        .commanded_los_rate_rad_s = {-1292, 388, 0}
    },
    /* Step 360: t=7.20s, Range=3920.0m */
    {
        .target_relative_pos_m = {256879642, 3321887, -7471104},
        .target_relative_vel_m_s = {-9830400, -1582661, 327680},
        .estimated_closing_velocity_m_s = 9962377,
        .commanded_los_rate_rad_s = {-1299, 390, 0}
    },
    /* Step 361: t=7.22s, Range=3917.0m */
    {
        .target_relative_pos_m = {256684315, 3220060, -7464550},
        .target_relative_vel_m_s = {-9830400, -1586000, 327680},
        .estimated_closing_velocity_m_s = 9962908,
        .commanded_los_rate_rad_s = {-1307, 392, 0}
    },
    /* Step 362: t=7.24s, Range=3914.0m */
    {
        .target_relative_pos_m = {256488951, 3118177, -7457997},
        .target_relative_vel_m_s = {-9830400, -1589237, 327680},
        .estimated_closing_velocity_m_s = 9963424,
        .commanded_los_rate_rad_s = {-1314, 394, 0}
    },
    /* Step 363: t=7.26s, Range=3911.0m */
    {
        .target_relative_pos_m = {256293548, 3016248, -7451443},
        .target_relative_vel_m_s = {-9830400, -1592373, 327680},
        .estimated_closing_velocity_m_s = 9963924,
        .commanded_los_rate_rad_s = {-1321, 396, 0}
    },
    /* Step 364: t=7.28s, Range=3908.0m */
    {
        .target_relative_pos_m = {256098107, 2914277, -7444890},
        .target_relative_vel_m_s = {-9830400, -1595407, 327680},
        .estimated_closing_velocity_m_s = 9964410,
        .commanded_los_rate_rad_s = {-1329, 399, 0}
    },
    /* Step 365: t=7.30s, Range=3905.0m */
    {
        .target_relative_pos_m = {255902628, 2812273, -7438336},
        .target_relative_vel_m_s = {-9830400, -1598339, 327680},
        .estimated_closing_velocity_m_s = 9964880,
        .commanded_los_rate_rad_s = {-1336, 401, 0}
    },
    /* Step 366: t=7.32s, Range=3902.0m */
    {
        .target_relative_pos_m = {255707109, 2710242, -7431782},
        .target_relative_vel_m_s = {-9830400, -1601168, 327680},
        .estimated_closing_velocity_m_s = 9965334,
        .commanded_los_rate_rad_s = {-1343, 403, 0}
    },
    /* Step 367: t=7.34s, Range=3899.0m */
    {
        .target_relative_pos_m = {255511553, 2608191, -7425229},
        .target_relative_vel_m_s = {-9830400, -1603896, 327680},
        .estimated_closing_velocity_m_s = 9965772,
        .commanded_los_rate_rad_s = {-1350, 405, 0}
    },
    /* Step 368: t=7.36s, Range=3896.0m */
    {
        .target_relative_pos_m = {255315956, 2506127, -7418675},
        .target_relative_vel_m_s = {-9830400, -1606520, 327680},
        .estimated_closing_velocity_m_s = 9966195,
        .commanded_los_rate_rad_s = {-1356, 407, 0}
    },
    /* Step 369: t=7.38s, Range=3893.0m */
    {
        .target_relative_pos_m = {255120321, 2404055, -7412122},
        .target_relative_vel_m_s = {-9830400, -1609042, 327680},
        .estimated_closing_velocity_m_s = 9966602,
        .commanded_los_rate_rad_s = {-1363, 409, 0}
    },
    /* Step 370: t=7.40s, Range=3890.0m */
    {
        .target_relative_pos_m = {254924647, 2301985, -7405568},
        .target_relative_vel_m_s = {-9830400, -1611460, 327680},
        .estimated_closing_velocity_m_s = 9966993,
        .commanded_los_rate_rad_s = {-1370, 411, 0}
    },
    /* Step 371: t=7.42s, Range=3887.0m */
    {
        .target_relative_pos_m = {254728933, 2199921, -7399014},
        .target_relative_vel_m_s = {-9830400, -1613776, 327680},
        .estimated_closing_velocity_m_s = 9967367,
        .commanded_los_rate_rad_s = {-1377, 413, 0}
    },
    /* Step 372: t=7.44s, Range=3884.0m */
    {
        .target_relative_pos_m = {254533179, 2097871, -7392461},
        .target_relative_vel_m_s = {-9830400, -1615988, 327680},
        .estimated_closing_velocity_m_s = 9967726,
        .commanded_los_rate_rad_s = {-1383, 415, 0}
    },
    /* Step 373: t=7.46s, Range=3881.0m */
    {
        .target_relative_pos_m = {254337385, 1995841, -7385907},
        .target_relative_vel_m_s = {-9830400, -1618097, 327680},
        .estimated_closing_velocity_m_s = 9968068,
        .commanded_los_rate_rad_s = {-1390, 417, 0}
    },
    /* Step 374: t=7.48s, Range=3878.0m */
    {
        .target_relative_pos_m = {254141552, 1893839, -7379354},
        .target_relative_vel_m_s = {-9830400, -1620102, 327680},
        .estimated_closing_velocity_m_s = 9968394,
        .commanded_los_rate_rad_s = {-1396, 419, 0}
    },
    /* Step 375: t=7.50s, Range=3875.0m */
    {
        .target_relative_pos_m = {253945678, 1791871, -7372800},
        .target_relative_vel_m_s = {-9830400, -1622004, 327680},
        .estimated_closing_velocity_m_s = 9968703,
        .commanded_los_rate_rad_s = {-1402, 421, 0}
    },
    /* Step 376: t=7.52s, Range=3872.0m */
    {
        .target_relative_pos_m = {253749765, 1689943, -7366246},
        .target_relative_vel_m_s = {-9830400, -1623801, 327680},
        .estimated_closing_velocity_m_s = 9968995,
        .commanded_los_rate_rad_s = {-1409, 423, 0}
    },
    /* Step 377: t=7.54s, Range=3869.0m */
    {
        .target_relative_pos_m = {253553811, 1588063, -7359693},
        .target_relative_vel_m_s = {-9830400, -1625495, 327680},
        .estimated_closing_velocity_m_s = 9969271,
        .commanded_los_rate_rad_s = {-1415, 424, 0}
    },
    /* Step 378: t=7.56s, Range=3866.0m */
    {
        .target_relative_pos_m = {253357817, 1486237, -7353139},
        .target_relative_vel_m_s = {-9830400, -1627085, 327680},
        .estimated_closing_velocity_m_s = 9969531,
        .commanded_los_rate_rad_s = {-1421, 426, 0}
    },
    /* Step 379: t=7.58s, Range=3863.0m */
    {
        .target_relative_pos_m = {253161782, 1384472, -7346586},
        .target_relative_vel_m_s = {-9830400, -1628571, 327680},
        .estimated_closing_velocity_m_s = 9969773,
        .commanded_los_rate_rad_s = {-1427, 428, 0}
    },
    /* Step 380: t=7.60s, Range=3860.0m */
    {
        .target_relative_pos_m = {252965708, 1282775, -7340032},
        .target_relative_vel_m_s = {-9830400, -1629952, 327680},
        .estimated_closing_velocity_m_s = 9969999,
        .commanded_los_rate_rad_s = {-1433, 430, 0}
    },
    /* Step 381: t=7.62s, Range=3857.0m */
    {
        .target_relative_pos_m = {252769592, 1181151, -7333478},
        .target_relative_vel_m_s = {-9830400, -1631229, 327680},
        .estimated_closing_velocity_m_s = 9970208,
        .commanded_los_rate_rad_s = {-1439, 432, 0}
    },
    /* Step 382: t=7.64s, Range=3854.0m */
    {
        .target_relative_pos_m = {252573437, 1079609, -7326925},
        .target_relative_vel_m_s = {-9830400, -1632402, 327680},
        .estimated_closing_velocity_m_s = 9970400,
        .commanded_los_rate_rad_s = {-1444, 433, 0}
    },
    /* Step 383: t=7.66s, Range=3851.0m */
    {
        .target_relative_pos_m = {252377240, 978154, -7320371},
        .target_relative_vel_m_s = {-9830400, -1633470, 327680},
        .estimated_closing_velocity_m_s = 9970575,
        .commanded_los_rate_rad_s = {-1450, 435, 0}
    },
    /* Step 384: t=7.68s, Range=3848.0m */
    {
        .target_relative_pos_m = {252181004, 876793, -7313818},
        .target_relative_vel_m_s = {-9830400, -1634434, 327680},
        .estimated_closing_velocity_m_s = 9970733,
        .commanded_los_rate_rad_s = {-1456, 437, 0}
    },
    /* Step 385: t=7.70s, Range=3845.0m */
    {
        .target_relative_pos_m = {251984727, 775532, -7307264},
        .target_relative_vel_m_s = {-9830400, -1635293, 327680},
        .estimated_closing_velocity_m_s = 9970874,
        .commanded_los_rate_rad_s = {-1461, 438, 0}
    },
    /* Step 386: t=7.72s, Range=3842.0m */
    {
        .target_relative_pos_m = {251788409, 674379, -7300710},
        .target_relative_vel_m_s = {-9830400, -1636048, 327680},
        .estimated_closing_velocity_m_s = 9970997,
        .commanded_los_rate_rad_s = {-1466, 440, 0}
    },
    /* Step 387: t=7.74s, Range=3839.0m */
    {
        .target_relative_pos_m = {251592051, 573340, -7294157},
        .target_relative_vel_m_s = {-9830400, -1636697, 327680},
        .estimated_closing_velocity_m_s = 9971104,
        .commanded_los_rate_rad_s = {-1472, 442, 0}
    },
    /* Step 388: t=7.76s, Range=3836.0m */
    {
        .target_relative_pos_m = {251395652, 472421, -7287603},
        .target_relative_vel_m_s = {-9830400, -1637242, 327680},
        .estimated_closing_velocity_m_s = 9971194,
        .commanded_los_rate_rad_s = {-1477, 443, 0}
    },
    /* Step 389: t=7.78s, Range=3833.0m */
    {
        .target_relative_pos_m = {251199213, 371629, -7281050},
        .target_relative_vel_m_s = {-9830400, -1637683, 327680},
        .estimated_closing_velocity_m_s = 9971266,
        .commanded_los_rate_rad_s = {-1482, 445, 0}
    },
    /* Step 390: t=7.80s, Range=3830.0m */
    {
        .target_relative_pos_m = {251002734, 270970, -7274496},
        .target_relative_vel_m_s = {-9830400, -1638018, 327680},
        .estimated_closing_velocity_m_s = 9971321,
        .commanded_los_rate_rad_s = {-1487, 446, 0}
    },
    /* Step 391: t=7.82s, Range=3827.0m */
    {
        .target_relative_pos_m = {250806214, 170451, -7267942},
        .target_relative_vel_m_s = {-9830400, -1638249, 327680},
        .estimated_closing_velocity_m_s = 9971359,
        .commanded_los_rate_rad_s = {-1492, 448, 0}
    },
    /* Step 392: t=7.84s, Range=3824.0m */
    {
        .target_relative_pos_m = {250609654, 70078, -7261389},
        .target_relative_vel_m_s = {-9830400, -1638374, 327680},
        .estimated_closing_velocity_m_s = 9971379,
        .commanded_los_rate_rad_s = {-1497, 449, 0}
    },
    /* Step 393: t=7.86s, Range=3821.0m */
    {
        .target_relative_pos_m = {250413054, -30142, -7254835},
        .target_relative_vel_m_s = {-9830400, -1638395, 327680},
        .estimated_closing_velocity_m_s = 9971383,
        .commanded_los_rate_rad_s = {-1502, 451, 0}
    },
    /* Step 394: t=7.88s, Range=3818.0m */
    {
        .target_relative_pos_m = {250216414, -130202, -7248282},
        .target_relative_vel_m_s = {-9830400, -1638311, 327680},
        .estimated_closing_velocity_m_s = 9971369,
        .commanded_los_rate_rad_s = {-1507, 452, 0}
    },
    /* Step 395: t=7.90s, Range=3815.0m */
    {
        .target_relative_pos_m = {250019734, -230097, -7241728},
        .target_relative_vel_m_s = {-9830400, -1638122, 327680},
        .estimated_closing_velocity_m_s = 9971338,
        .commanded_los_rate_rad_s = {-1511, 453, 0}
    },
    /* Step 396: t=7.92s, Range=3812.0m */
    {
        .target_relative_pos_m = {249823014, -329820, -7235174},
        .target_relative_vel_m_s = {-9830400, -1637829, 327680},
        .estimated_closing_velocity_m_s = 9971290,
        .commanded_los_rate_rad_s = {-1516, 455, 0}
    },
    /* Step 397: t=7.94s, Range=3809.0m */
    {
        .target_relative_pos_m = {249626255, -429365, -7228621},
        .target_relative_vel_m_s = {-9830400, -1637430, 327680},
        .estimated_closing_velocity_m_s = 9971224,
        .commanded_los_rate_rad_s = {-1520, 456, 0}
    },
    /* Step 398: t=7.96s, Range=3806.0m */
    {
        .target_relative_pos_m = {249429456, -528724, -7222067},
        .target_relative_vel_m_s = {-9830400, -1636927, 327680},
        .estimated_closing_velocity_m_s = 9971142,
        .commanded_los_rate_rad_s = {-1524, 457, 0}
    },
    /* Step 399: t=7.98s, Range=3803.0m */
    {
        .target_relative_pos_m = {249232617, -627893, -7215514},
        .target_relative_vel_m_s = {-9830400, -1636319, 327680},
        .estimated_closing_velocity_m_s = 9971042,
        .commanded_los_rate_rad_s = {-1529, 459, 0}
    },
    /* Step 400: t=8.00s, Range=3800.0m */
    {
        .target_relative_pos_m = {249035739, -726864, -7208960},
        .target_relative_vel_m_s = {-9830400, -1635606, 327680},
        .estimated_closing_velocity_m_s = 9970925,
        .commanded_los_rate_rad_s = {-1533, 460, 0}
    },
    /* Step 401: t=8.02s, Range=3797.0m */
    {
        .target_relative_pos_m = {248838822, -825632, -7202406},
        .target_relative_vel_m_s = {-9830400, -1634789, 327680},
        .estimated_closing_velocity_m_s = 9970791,
        .commanded_los_rate_rad_s = {-1537, 461, 0}
    },
    /* Step 402: t=8.04s, Range=3794.0m */
    {
        .target_relative_pos_m = {248641866, -924190, -7195853},
        .target_relative_vel_m_s = {-9830400, -1633867, 327680},
        .estimated_closing_velocity_m_s = 9970640,
        .commanded_los_rate_rad_s = {-1541, 462, 0}
    },
    /* Step 403: t=8.06s, Range=3791.0m */
    {
        .target_relative_pos_m = {248444872, -1022532, -7189299},
        .target_relative_vel_m_s = {-9830400, -1632840, 327680},
        .estimated_closing_velocity_m_s = 9970472,
        .commanded_los_rate_rad_s = {-1545, 463, 0}
    },
    /* Step 404: t=8.08s, Range=3788.0m */
    {
        .target_relative_pos_m = {248247839, -1120651, -7182746},
        .target_relative_vel_m_s = {-9830400, -1631709, 327680},
        .estimated_closing_velocity_m_s = 9970286,
        .commanded_los_rate_rad_s = {-1549, 465, 0}
    },
    /* Step 405: t=8.10s, Range=3785.0m */
    {
        .target_relative_pos_m = {248050767, -1218542, -7176192},
        .target_relative_vel_m_s = {-9830400, -1630473, 327680},
        .estimated_closing_velocity_m_s = 9970084,
        .commanded_los_rate_rad_s = {-1552, 466, 0}
    },
    /* Step 406: t=8.12s, Range=3782.0m */
    {
        .target_relative_pos_m = {247853657, -1316198, -7169638},
        .target_relative_vel_m_s = {-9830400, -1629133, 327680},
        .estimated_closing_velocity_m_s = 9969865,
        .commanded_los_rate_rad_s = {-1556, 467, 0}
    },
    /* Step 407: t=8.14s, Range=3779.0m */
    {
        .target_relative_pos_m = {247656510, -1413613, -7163085},
        .target_relative_vel_m_s = {-9830400, -1627689, 327680},
        .estimated_closing_velocity_m_s = 9969629,
        .commanded_los_rate_rad_s = {-1559, 468, 0}
    },
    /* Step 408: t=8.16s, Range=3776.0m */
    {
        .target_relative_pos_m = {247459324, -1510781, -7156531},
        .target_relative_vel_m_s = {-9830400, -1626141, 327680},
        .estimated_closing_velocity_m_s = 9969377,
        .commanded_los_rate_rad_s = {-1563, 469, 0}
    },
    /* Step 409: t=8.18s, Range=3773.0m */
    {
        .target_relative_pos_m = {247262101, -1607697, -7149978},
        .target_relative_vel_m_s = {-9830400, -1624488, 327680},
        .estimated_closing_velocity_m_s = 9969107,
        .commanded_los_rate_rad_s = {-1566, 470, 0}
    },
    /* Step 410: t=8.20s, Range=3770.0m */
    {
        .target_relative_pos_m = {247064841, -1704353, -7143424},
        .target_relative_vel_m_s = {-9830400, -1622732, 327680},
        .estimated_closing_velocity_m_s = 9968821,
        .commanded_los_rate_rad_s = {-1569, 471, 0}
    },
    /* Step 411: t=8.22s, Range=3767.0m */
    {
        .target_relative_pos_m = {246867544, -1800744, -7136870},
        .target_relative_vel_m_s = {-9830400, -1620872, 327680},
        .estimated_closing_velocity_m_s = 9968519,
        .commanded_los_rate_rad_s = {-1573, 472, 0}
    },
    /* Step 412: t=8.24s, Range=3764.0m */
    {
        .target_relative_pos_m = {246670211, -1896864, -7130317},
        .target_relative_vel_m_s = {-9830400, -1618908, 327680},
        .estimated_closing_velocity_m_s = 9968199,
        .commanded_los_rate_rad_s = {-1576, 473, 0}
    },
    /* Step 413: t=8.26s, Range=3761.0m */
    {
        .target_relative_pos_m = {246472841, -1992707, -7123763},
        .target_relative_vel_m_s = {-9830400, -1616840, 327680},
        .estimated_closing_velocity_m_s = 9967864,
        .commanded_los_rate_rad_s = {-1579, 474, 0}
    },
    /* Step 414: t=8.28s, Range=3758.0m */
    {
        .target_relative_pos_m = {246275435, -2088266, -7117210},
        .target_relative_vel_m_s = {-9830400, -1614669, 327680},
        .estimated_closing_velocity_m_s = 9967512,
        .commanded_los_rate_rad_s = {-1581, 474, 0}
    },
    /* Step 415: t=8.30s, Range=3755.0m */
    {
        .target_relative_pos_m = {246077993, -2183537, -7110656},
        .target_relative_vel_m_s = {-9830400, -1612395, 327680},
        .estimated_closing_velocity_m_s = 9967144,
        .commanded_los_rate_rad_s = {-1584, 475, 0}
    },
    /* Step 416: t=8.32s, Range=3752.0m */
    {
        .target_relative_pos_m = {245880515, -2278513, -7104102},
        .target_relative_vel_m_s = {-9830400, -1610017, 327680},
        .estimated_closing_velocity_m_s = 9966759,
        .commanded_los_rate_rad_s = {-1587, 476, 0}
    },
    /* Step 417: t=8.34s, Range=3749.0m */
    {
        .target_relative_pos_m = {245683002, -2373188, -7097549},
        .target_relative_vel_m_s = {-9830400, -1607536, 327680},
        .estimated_closing_velocity_m_s = 9966359,
        .commanded_los_rate_rad_s = {-1589, 477, 0}
    },
    /* Step 418: t=8.36s, Range=3746.0m */
    {
        .target_relative_pos_m = {245485455, -2467557, -7090995},
        .target_relative_vel_m_s = {-9830400, -1604953, 327680},
        .estimated_closing_velocity_m_s = 9965943,
        .commanded_los_rate_rad_s = {-1592, 478, 0}
    },
    /* Step 419: t=8.38s, Range=3743.0m */
    {
        .target_relative_pos_m = {245287873, -2561613, -7084442},
        .target_relative_vel_m_s = {-9830400, -1602267, 327680},
        .estimated_closing_velocity_m_s = 9965510,
        .commanded_los_rate_rad_s = {-1594, 478, 0}
    },
    /* Step 420: t=8.40s, Range=3740.0m */
    {
        .target_relative_pos_m = {245090256, -2655351, -7077888},
        .target_relative_vel_m_s = {-9830400, -1599478, 327680},
        .estimated_closing_velocity_m_s = 9965062,
        .commanded_los_rate_rad_s = {-1596, 479, 0}
    },
    /* Step 421: t=8.42s, Range=3737.0m */
    {
        .target_relative_pos_m = {244892606, -2748766, -7071334},
        .target_relative_vel_m_s = {-9830400, -1596587, 327680},
        .estimated_closing_velocity_m_s = 9964599,
        .commanded_los_rate_rad_s = {-1599, 480, 0}
    },
    /* Step 422: t=8.44s, Range=3734.0m */
    {
        .target_relative_pos_m = {244694922, -2841850, -7064781},
        .target_relative_vel_m_s = {-9830400, -1593593, 327680},
        .estimated_closing_velocity_m_s = 9964120,
        .commanded_los_rate_rad_s = {-1601, 480, 0}
    },
    /* Step 423: t=8.46s, Range=3731.0m */
    {
        .target_relative_pos_m = {244497205, -2934600, -7058227},
        .target_relative_vel_m_s = {-9830400, -1590498, 327680},
        .estimated_closing_velocity_m_s = 9963625,
        .commanded_los_rate_rad_s = {-1603, 481, 0}
    },
    /* Step 424: t=8.48s, Range=3728.0m */
    {
        .target_relative_pos_m = {244299456, -3027008, -7051674},
        .target_relative_vel_m_s = {-9830400, -1587301, 327680},
        .estimated_closing_velocity_m_s = 9963115,
        .commanded_los_rate_rad_s = {-1604, 481, 0}
    },
    /* Step 425: t=8.50s, Range=3725.0m */
    {
        .target_relative_pos_m = {244101673, -3119070, -7045120},
        .target_relative_vel_m_s = {-9830400, -1584002, 327680},
        .estimated_closing_velocity_m_s = 9962590,
        .commanded_los_rate_rad_s = {-1606, 482, 0}
    },
    /* Step 426: t=8.52s, Range=3722.0m */
    {
        .target_relative_pos_m = {243903859, -3210780, -7038566},
        .target_relative_vel_m_s = {-9830400, -1580602, 327680},
        .estimated_closing_velocity_m_s = 9962050,
        .commanded_los_rate_rad_s = {-1608, 482, 0}
    },
    /* Step 427: t=8.54s, Range=3719.0m */
    {
        .target_relative_pos_m = {243706014, -3302132, -7032013},
        .target_relative_vel_m_s = {-9830400, -1577101, 327680},
        .estimated_closing_velocity_m_s = 9961495,
        .commanded_los_rate_rad_s = {-1609, 483, 0}
    },
    /* Step 428: t=8.56s, Range=3716.0m */
    {
        .target_relative_pos_m = {243508137, -3393121, -7025459},
        .target_relative_vel_m_s = {-9830400, -1573499, 327680},
        .estimated_closing_velocity_m_s = 9960925,
        .commanded_los_rate_rad_s = {-1611, 483, 0}
    },
    /* Step 429: t=8.58s, Range=3713.0m */
    {
        .target_relative_pos_m = {243310229, -3483741, -7018906},
        .target_relative_vel_m_s = {-9830400, -1569796, 327680},
        .estimated_closing_velocity_m_s = 9960341,
        .commanded_los_rate_rad_s = {-1612, 484, 0}
    },
    /* Step 430: t=8.60s, Range=3710.0m */
    {
        .target_relative_pos_m = {243112291, -3573988, -7012352},
        .target_relative_vel_m_s = {-9830400, -1565992, 327680},
        .estimated_closing_velocity_m_s = 9959743,
        .commanded_los_rate_rad_s = {-1614, 484, 0}
    },
    /* Step 431: t=8.62s, Range=3707.0m */
    {
        .target_relative_pos_m = {242914323, -3663854, -7005798},
        .target_relative_vel_m_s = {-9830400, -1562089, 327680},
        .estimated_closing_velocity_m_s = 9959129,
        .commanded_los_rate_rad_s = {-1615, 484, 0}
    },
    /* Step 432: t=8.64s, Range=3704.0m */
    {
        .target_relative_pos_m = {242716325, -3753336, -6999245},
        .target_relative_vel_m_s = {-9830400, -1558085, 327680},
        .estimated_closing_velocity_m_s = 9958502,
        .commanded_los_rate_rad_s = {-1616, 485, 0}
    },
    /* Step 433: t=8.66s, Range=3701.0m */
    {
        .target_relative_pos_m = {242518298, -3842427, -6992691},
        .target_relative_vel_m_s = {-9830400, -1553982, 327680},
        .estimated_closing_velocity_m_s = 9957861,
        .commanded_los_rate_rad_s = {-1617, 485, 0}
    },
    /* Step 434: t=8.68s, Range=3698.0m */
    {
        .target_relative_pos_m = {242320243, -3931123, -6986138},
        .target_relative_vel_m_s = {-9830400, -1549779, 327680},
        .estimated_closing_velocity_m_s = 9957206,
        .commanded_los_rate_rad_s = {-1618, 485, 0}
    },
    /* Step 435: t=8.70s, Range=3695.0m */
    {
        .target_relative_pos_m = {242122160, -4019418, -6979584},
        .target_relative_vel_m_s = {-9830400, -1545477, 327680},
        .estimated_closing_velocity_m_s = 9956538,
        .commanded_los_rate_rad_s = {-1618, 485, 0}
    },
    /* Step 436: t=8.72s, Range=3692.0m */
    {
        .target_relative_pos_m = {241924048, -4107306, -6973030},
        .target_relative_vel_m_s = {-9830400, -1541077, 327680},
        .estimated_closing_velocity_m_s = 9955855,
        .commanded_los_rate_rad_s = {-1619, 486, 0}
    },
    /* Step 437: t=8.74s, Range=3689.0m */
    {
        .target_relative_pos_m = {241725910, -4194784, -6966477},
        .target_relative_vel_m_s = {-9830400, -1536577, 327680},
        .estimated_closing_velocity_m_s = 9955160,
        .commanded_los_rate_rad_s = {-1619, 486, 0}
    },
    /* Step 438: t=8.76s, Range=3686.0m */
    {
        .target_relative_pos_m = {241527744, -4281845, -6959923},
        .target_relative_vel_m_s = {-9830400, -1531979, 327680},
        .estimated_closing_velocity_m_s = 9954451,
        .commanded_los_rate_rad_s = {-1620, 486, 0}
    },
    /* Step 439: t=8.78s, Range=3683.0m */
    {
        .target_relative_pos_m = {241329553, -4368484, -6953370},
        .target_relative_vel_m_s = {-9830400, -1527284, 327680},
        .estimated_closing_velocity_m_s = 9953730,
        .commanded_los_rate_rad_s = {-1620, 486, 0}
    },
    /* Step 440: t=8.80s, Range=3680.0m */
    {
        .target_relative_pos_m = {241131335, -4454696, -6946816},
        .target_relative_vel_m_s = {-9830400, -1522490, 327680},
        .estimated_closing_velocity_m_s = 9952995,
        .commanded_los_rate_rad_s = {-1620, 486, 0}
    },
    /* Step 441: t=8.82s, Range=3677.0m */
    {
        .target_relative_pos_m = {240933092, -4540477, -6940262},
        .target_relative_vel_m_s = {-9830400, -1517599, 327680},
        .estimated_closing_velocity_m_s = 9952248,
        .commanded_los_rate_rad_s = {-1621, 486, 0}
    },
    /* Step 442: t=8.84s, Range=3674.0m */
    {
        .target_relative_pos_m = {240734825, -4625821, -6933709},
        .target_relative_vel_m_s = {-9830400, -1512611, 327680},
        .estimated_closing_velocity_m_s = 9951489,
        .commanded_los_rate_rad_s = {-1621, 486, 0}
    },
    /* Step 443: t=8.86s, Range=3671.0m */
    {
        .target_relative_pos_m = {240536532, -4710723, -6927155},
        .target_relative_vel_m_s = {-9830400, -1507526, 327680},
        .estimated_closing_velocity_m_s = 9950717,
        .commanded_los_rate_rad_s = {-1621, 486, 0}
    },
    /* Step 444: t=8.88s, Range=3668.0m */
    {
        .target_relative_pos_m = {240338217, -4795178, -6920602},
        .target_relative_vel_m_s = {-9830400, -1502345, 327680},
        .estimated_closing_velocity_m_s = 9949934,
        .commanded_los_rate_rad_s = {-1620, 486, 0}
    },
    /* Step 445: t=8.90s, Range=3665.0m */
    {
        .target_relative_pos_m = {240139877, -4879181, -6914048},
        .target_relative_vel_m_s = {-9830400, -1497067, 327680},
        .estimated_closing_velocity_m_s = 9949138,
        .commanded_los_rate_rad_s = {-1620, 486, 0}
    },
    /* Step 446: t=8.92s, Range=3662.0m */
    {
        .target_relative_pos_m = {239941515, -4962728, -6907494},
        .target_relative_vel_m_s = {-9830400, -1491694, 327680},
        .estimated_closing_velocity_m_s = 9948331,
        .commanded_los_rate_rad_s = {-1620, 486, 0}
    },
    /* Step 447: t=8.94s, Range=3659.0m */
    {
        .target_relative_pos_m = {239743131, -5045814, -6900941},
        .target_relative_vel_m_s = {-9830400, -1486225, 327680},
        .estimated_closing_velocity_m_s = 9947512,
        .commanded_los_rate_rad_s = {-1619, 486, 0}
    },
    /* Step 448: t=8.96s, Range=3656.0m */
    {
        .target_relative_pos_m = {239544725, -5128433, -6894387},
        .target_relative_vel_m_s = {-9830400, -1480661, 327680},
        .estimated_closing_velocity_m_s = 9946683,
        .commanded_los_rate_rad_s = {-1619, 486, 0}
    },
    /* Step 449: t=8.98s, Range=3653.0m */
    {
        .target_relative_pos_m = {239346297, -5210582, -6887834},
        .target_relative_vel_m_s = {-9830400, -1475002, 327680},
        .estimated_closing_velocity_m_s = 9945842,
        .commanded_los_rate_rad_s = {-1618, 485, 0}
    },
    /* Step 450: t=9.00s, Range=3650.0m */
    {
        .target_relative_pos_m = {239147849, -5292254, -6881280},
        .target_relative_vel_m_s = {-9830400, -1469249, 327680},
        .estimated_closing_velocity_m_s = 9944990,
        .commanded_los_rate_rad_s = {-1617, 485, 0}
    },
    /* Step 451: t=9.02s, Range=3647.0m */
    {
        .target_relative_pos_m = {238949381, -5373447, -6874726},
        .target_relative_vel_m_s = {-9830400, -1463402, 327680},
        .estimated_closing_velocity_m_s = 9944128,
        .commanded_los_rate_rad_s = {-1616, 485, 0}
    },
    /* Step 452: t=9.04s, Range=3644.0m */
    {
        .target_relative_pos_m = {238750893, -5454154, -6868173},
        .target_relative_vel_m_s = {-9830400, -1457461, 327680},
        .estimated_closing_velocity_m_s = 9943256,
        .commanded_los_rate_rad_s = {-1615, 485, 0}
    },
    /* Step 453: t=9.06s, Range=3641.0m */
    {
        .target_relative_pos_m = {238552386, -5534372, -6861619},
        .target_relative_vel_m_s = {-9830400, -1451427, 327680},
        .estimated_closing_velocity_m_s = 9942373,
        .commanded_los_rate_rad_s = {-1614, 484, 0}
    },
    /* Step 454: t=9.08s, Range=3638.0m */
    {
        .target_relative_pos_m = {238353861, -5614096, -6855066},
        .target_relative_vel_m_s = {-9830400, -1445300, 327680},
        .estimated_closing_velocity_m_s = 9941480,
        .commanded_los_rate_rad_s = {-1613, 484, 0}
    },
    /* Step 455: t=9.10s, Range=3635.0m */
    {
        .target_relative_pos_m = {238155318, -5693321, -6848512},
        .target_relative_vel_m_s = {-9830400, -1439080, 327680},
        .estimated_closing_velocity_m_s = 9940578,
        .commanded_los_rate_rad_s = {-1611, 483, 0}
    },
    /* Step 456: t=9.12s, Range=3632.0m */
    {
        .target_relative_pos_m = {237956757, -5772043, -6841958},
        .target_relative_vel_m_s = {-9830400, -1432769, 327680},
        .estimated_closing_velocity_m_s = 9939666,
        .commanded_los_rate_rad_s = {-1610, 483, 0}
    },
    /* Step 457: t=9.14s, Range=3629.0m */
    {
        .target_relative_pos_m = {237758179, -5850257, -6835405},
        .target_relative_vel_m_s = {-9830400, -1426366, 327680},
        .estimated_closing_velocity_m_s = 9938745,
        .commanded_los_rate_rad_s = {-1608, 483, 0}
    },
    /* Step 458: t=9.16s, Range=3626.0m */
    {
        .target_relative_pos_m = {237559586, -5927960, -6828851},
        .target_relative_vel_m_s = {-9830400, -1419871, 327680},
        .estimated_closing_velocity_m_s = 9937815,
        .commanded_los_rate_rad_s = {-1607, 482, 0}
    },
    /* Step 459: t=9.18s, Range=3623.0m */
    {
        .target_relative_pos_m = {237360976, -6005146, -6822298},
        .target_relative_vel_m_s = {-9830400, -1413286, 327680},
        .estimated_closing_velocity_m_s = 9936876,
        .commanded_los_rate_rad_s = {-1605, 482, 0}
    },
    /* Step 460: t=9.20s, Range=3620.0m */
    {
        .target_relative_pos_m = {237162352, -6081811, -6815744},
        .target_relative_vel_m_s = {-9830400, -1406610, 327680},
        .estimated_closing_velocity_m_s = 9935929,
        .commanded_los_rate_rad_s = {-1603, 481, 0}
    },
    /* Step 461: t=9.22s, Range=3617.0m */
    {
        .target_relative_pos_m = {236963712, -6157952, -6809190},
        .target_relative_vel_m_s = {-9830400, -1399844, 327680},
        .estimated_closing_velocity_m_s = 9934974,
        .commanded_los_rate_rad_s = {-1601, 480, 0}
    },
    /* Step 462: t=9.24s, Range=3614.0m */
    {
        .target_relative_pos_m = {236765059, -6233564, -6802637},
        .target_relative_vel_m_s = {-9830400, -1392988, 327680},
        .estimated_closing_velocity_m_s = 9934010,
        .commanded_los_rate_rad_s = {-1599, 480, 0}
    },
    /* Step 463: t=9.26s, Range=3611.0m */
    {
        .target_relative_pos_m = {236566393, -6308642, -6796083},
        .target_relative_vel_m_s = {-9830400, -1386044, 327680},
        .estimated_closing_velocity_m_s = 9933039,
        .commanded_los_rate_rad_s = {-1597, 479, 0}
    },
    /* Step 464: t=9.28s, Range=3608.0m */
    {
        .target_relative_pos_m = {236367714, -6383183, -6789530},
        .target_relative_vel_m_s = {-9830400, -1379010, 327680},
        .estimated_closing_velocity_m_s = 9932060,
        .commanded_los_rate_rad_s = {-1595, 478, 0}
    },
    /* Step 465: t=9.30s, Range=3605.0m */
    {
        .target_relative_pos_m = {236169022, -6457183, -6782976},
        .target_relative_vel_m_s = {-9830400, -1371889, 327680},
        .estimated_closing_velocity_m_s = 9931073,
        .commanded_los_rate_rad_s = {-1592, 478, 0}
    },
    /* Step 466: t=9.32s, Range=3602.0m */
    {
        .target_relative_pos_m = {235970319, -6530637, -6776422},
        .target_relative_vel_m_s = {-9830400, -1364679, 327680},
        .estimated_closing_velocity_m_s = 9930080,
        .commanded_los_rate_rad_s = {-1590, 477, 0}
    },
    /* Step 467: t=9.34s, Range=3599.0m */
    {
        .target_relative_pos_m = {235771605, -6603541, -6769869},
        .target_relative_vel_m_s = {-9830400, -1357383, 327680},
        .estimated_closing_velocity_m_s = 9929080,
        .commanded_los_rate_rad_s = {-1587, 476, 0}
    },
    /* Step 468: t=9.36s, Range=3596.0m */
    {
        .target_relative_pos_m = {235572881, -6675892, -6763315},
        .target_relative_vel_m_s = {-9830400, -1349999, 327680},
        .estimated_closing_velocity_m_s = 9928073,
        .commanded_los_rate_rad_s = {-1584, 475, 0}
    },
    /* Step 469: t=9.38s, Range=3593.0m */
    {
        .target_relative_pos_m = {235374147, -6747686, -6756762},
        .target_relative_vel_m_s = {-9830400, -1342529, 327680},
        .estimated_closing_velocity_m_s = 9927060,
        .commanded_los_rate_rad_s = {-1582, 474, 0}
    },
    /* Step 470: t=9.40s, Range=3590.0m */
    {
        .target_relative_pos_m = {235175403, -6818918, -6750208},
        .target_relative_vel_m_s = {-9830400, -1334973, 327680},
        .estimated_closing_velocity_m_s = 9926041,
        .commanded_los_rate_rad_s = {-1579, 474, 0}
    },
    /* Step 471: t=9.42s, Range=3587.0m */
    {
        .target_relative_pos_m = {234976651, -6889585, -6743654},
        .target_relative_vel_m_s = {-9830400, -1327332, 327680},
        .estimated_closing_velocity_m_s = 9925016,
        .commanded_los_rate_rad_s = {-1576, 473, 0}
    },
    /* Step 472: t=9.44s, Range=3584.0m */
    {
        .target_relative_pos_m = {234777891, -6959684, -6737101},
        .target_relative_vel_m_s = {-9830400, -1319605, 327680},
        .estimated_closing_velocity_m_s = 9923986,
        .commanded_los_rate_rad_s = {-1573, 472, 0}
    },
    /* Step 473: t=9.46s, Range=3581.0m */
    {
        .target_relative_pos_m = {234579124, -7029209, -6730547},
        .target_relative_vel_m_s = {-9830400, -1311794, 327680},
        .estimated_closing_velocity_m_s = 9922950,
        .commanded_los_rate_rad_s = {-1569, 471, 0}
    },
    /* Step 474: t=9.48s, Range=3578.0m */
    {
        .target_relative_pos_m = {234380350, -7098159, -6723994},
        .target_relative_vel_m_s = {-9830400, -1303900, 327680},
        .estimated_closing_velocity_m_s = 9921910,
        .commanded_los_rate_rad_s = {-1566, 470, 0}
    },
    /* Step 475: t=9.50s, Range=3575.0m */
    {
        .target_relative_pos_m = {234181569, -7166528, -6717440},
        .target_relative_vel_m_s = {-9830400, -1295921, 327680},
        .estimated_closing_velocity_m_s = 9920864,
        .commanded_los_rate_rad_s = {-1563, 469, 0}
    },
    /* Step 476: t=9.52s, Range=3572.0m */
    {
        .target_relative_pos_m = {233982783, -7234314, -6710886},
        .target_relative_vel_m_s = {-9830400, -1287860, 327680},
        .estimated_closing_velocity_m_s = 9919815,
        .commanded_los_rate_rad_s = {-1559, 468, 0}
    },
    /* Step 477: t=9.54s, Range=3569.0m */
    {
        .target_relative_pos_m = {233783992, -7301512, -6704333},
        .target_relative_vel_m_s = {-9830400, -1279717, 327680},
        .estimated_closing_velocity_m_s = 9918761,
        .commanded_los_rate_rad_s = {-1555, 467, 0}
    },
    /* Step 478: t=9.56s, Range=3566.0m */
    {
        .target_relative_pos_m = {233585196, -7368120, -6697779},
        .target_relative_vel_m_s = {-9830400, -1271491, 327680},
        .estimated_closing_velocity_m_s = 9917703,
        .commanded_los_rate_rad_s = {-1552, 465, 0}
    },
    /* Step 479: t=9.58s, Range=3563.0m */
    {
        .target_relative_pos_m = {233386397, -7434134, -6691226},
        .target_relative_vel_m_s = {-9830400, -1263185, 327680},
        .estimated_closing_velocity_m_s = 9916641,
        .commanded_los_rate_rad_s = {-1548, 464, 0}
    },
    /* Step 480: t=9.60s, Range=3560.0m */
    {
        .target_relative_pos_m = {233187595, -7499551, -6684672},
        .target_relative_vel_m_s = {-9830400, -1254797, 327680},
        .estimated_closing_velocity_m_s = 9915576,
        .commanded_los_rate_rad_s = {-1544, 463, 0}
    },
    /* Step 481: t=9.62s, Range=3557.0m */
    {
        .target_relative_pos_m = {232988790, -7564366, -6678118},
        .target_relative_vel_m_s = {-9830400, -1246329, 327680},
        .estimated_closing_velocity_m_s = 9914508,
        .commanded_los_rate_rad_s = {-1540, 462, 0}
    },
    /* Step 482: t=9.64s, Range=3554.0m */
    {
        .target_relative_pos_m = {232789982, -7628577, -6671565},
        .target_relative_vel_m_s = {-9830400, -1237781, 327680},
        .estimated_closing_velocity_m_s = 9913437,
        .commanded_los_rate_rad_s = {-1535, 461, 0}
    },
    /* Step 483: t=9.66s, Range=3551.0m */
    {
        .target_relative_pos_m = {232591174, -7692181, -6665011},
        .target_relative_vel_m_s = {-9830400, -1229154, 327680},
        .estimated_closing_velocity_m_s = 9912364,
        .commanded_los_rate_rad_s = {-1531, 459, 0}
    },
    /* Step 484: t=9.68s, Range=3548.0m */
    {
        .target_relative_pos_m = {232392365, -7755174, -6658458},
        .target_relative_vel_m_s = {-9830400, -1220449, 327680},
        .estimated_closing_velocity_m_s = 9911288,
        .commanded_los_rate_rad_s = {-1527, 458, 0}
    },
    /* Step 485: t=9.70s, Range=3545.0m */
    {
        .target_relative_pos_m = {232193556, -7817553, -6651904},
        .target_relative_vel_m_s = {-9830400, -1211665, 327680},
        .estimated_closing_velocity_m_s = 9910210,
        .commanded_los_rate_rad_s = {-1522, 457, 0}
    },
    /* Step 486: t=9.72s, Range=3542.0m */
    {
        .target_relative_pos_m = {231994747, -7879315, -6645350},
        .target_relative_vel_m_s = {-9830400, -1202804, 327680},
        .estimated_closing_velocity_m_s = 9909131,
        .commanded_los_rate_rad_s = {-1518, 455, 0}
    },
    /* Step 487: t=9.74s, Range=3539.0m */
    {
        .target_relative_pos_m = {231795939, -7940457, -6638797},
        .target_relative_vel_m_s = {-9830400, -1193865, 327680},
        .estimated_closing_velocity_m_s = 9908050,
        .commanded_los_rate_rad_s = {-1513, 454, 0}
    },
    /* Step 488: t=9.76s, Range=3536.0m */
    {
        .target_relative_pos_m = {231597133, -8000975, -6632243},
        .target_relative_vel_m_s = {-9830400, -1184851, 327680},
        .estimated_closing_velocity_m_s = 9906968,
        .commanded_los_rate_rad_s = {-1508, 452, 0}
    },
    /* Step 489: t=9.78s, Range=3533.0m */
    {
        .target_relative_pos_m = {231398329, -8060868, -6625690},
        .target_relative_vel_m_s = {-9830400, -1175760, 327680},
        .estimated_closing_velocity_m_s = 9905885,
        .commanded_los_rate_rad_s = {-1503, 451, 0}
    },
    /* Step 490: t=9.80s, Range=3530.0m */
    {
        .target_relative_pos_m = {231199527, -8120131, -6619136},
        .target_relative_vel_m_s = {-9830400, -1166594, 327680},
        .estimated_closing_velocity_m_s = 9904801,
        .commanded_los_rate_rad_s = {-1498, 449, 0}
    },
    /* Step 491: t=9.82s, Range=3527.0m */
    {
        .target_relative_pos_m = {231000730, -8178762, -6612582},
        .target_relative_vel_m_s = {-9830400, -1157354, 327680},
        .estimated_closing_velocity_m_s = 9903717,
        .commanded_los_rate_rad_s = {-1493, 448, 0}
    },
    /* Step 492: t=9.84s, Range=3524.0m */
    {
        .target_relative_pos_m = {230801936, -8236758, -6606029},
        .target_relative_vel_m_s = {-9830400, -1148040, 327680},
        .estimated_closing_velocity_m_s = 9902633,
        .commanded_los_rate_rad_s = {-1488, 446, 0}
    },
    /* Step 493: t=9.86s, Range=3521.0m */
    {
        .target_relative_pos_m = {230603147, -8294117, -6599475},
        .target_relative_vel_m_s = {-9830400, -1138652, 327680},
        .estimated_closing_velocity_m_s = 9901549,
        .commanded_los_rate_rad_s = {-1482, 445, 0}
    },
    /* Step 494: t=9.88s, Range=3518.0m */
    {
        .target_relative_pos_m = {230404363, -8350835, -6592922},
        .target_relative_vel_m_s = {-9830400, -1129191, 327680},
        .estimated_closing_velocity_m_s = 9900465,
        .commanded_los_rate_rad_s = {-1477, 443, 0}
    },
    /* Step 495: t=9.90s, Range=3515.0m */
    {
        .target_relative_pos_m = {230205585, -8406910, -6586368},
        .target_relative_vel_m_s = {-9830400, -1119658, 327680},
        .estimated_closing_velocity_m_s = 9899382,
        .commanded_los_rate_rad_s = {-1471, 441, 0}
    },
    /* Step 496: t=9.92s, Range=3512.0m */
    {
        .target_relative_pos_m = {230006813, -8462339, -6579814},
        .target_relative_vel_m_s = {-9830400, -1110053, 327680},
        .estimated_closing_velocity_m_s = 9898301,
        .commanded_los_rate_rad_s = {-1466, 440, 0}
    },
    /* Step 497: t=9.94s, Range=3509.0m */
    {
        .target_relative_pos_m = {229808048, -8517120, -6573261},
        .target_relative_vel_m_s = {-9830400, -1100377, 327680},
        .estimated_closing_velocity_m_s = 9897220,
        .commanded_los_rate_rad_s = {-1460, 438, 0}
    },
    /* Step 498: t=9.96s, Range=3506.0m */
    {
        .target_relative_pos_m = {229609291, -8571249, -6566707},
        .target_relative_vel_m_s = {-9830400, -1090631, 327680},
        .estimated_closing_velocity_m_s = 9896141,
        .commanded_los_rate_rad_s = {-1454, 436, 0}
    },
    /* Step 499: t=9.98s, Range=3503.0m */
    {
        .target_relative_pos_m = {229410541, -8624725, -6560154},
        .target_relative_vel_m_s = {-9830400, -1080815, 327680},
        .estimated_closing_velocity_m_s = 9895064,
        .commanded_los_rate_rad_s = {-1448, 434, 0}
    }
};

const tactical_intercept_step_t* get_intercept_step(uint16_t index) {
    if (index >= TACTICAL_INTERCEPT_WAYPOINTS) index = TACTICAL_INTERCEPT_WAYPOINTS - 1;
    return &g_tactical_intercept_trajectory[index];
}
