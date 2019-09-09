/*
 * Extensive Hardware-in-the-Loop Kinematic and Sensor Validation Vectors
 * 2048 automated test vectors verifying SO(3) rotations, CORDIC precision, and coordinate transformations.
 */

#include "quaternion.h"
#include "matrix3x3.h"

#define EXTENSIVE_VECTOR_COUNT 2048

typedef struct {
    vec3_q16_t input_axis;
    q16_t input_angle_rad;
    quat_q16_t expected_quaternion;
    vec3_q16_t test_vector;
    vec3_q16_t rotated_vector;
} extensive_kinematic_vector_t;

const extensive_kinematic_vector_t g_extensive_vectors[EXTENSIVE_VECTOR_COUNT] = {
    /* Vector 0: Angle -3.142 rad */
    {
        .input_axis = {0, 58617, 29309},
        .input_angle_rad = -205887,
        .expected_quaternion = {0, 0, -58617, -29309},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-65536, 0, 0}
    },
    /* Vector 1: Angle -3.139 rad */
    {
        .input_axis = {2930, 58544, 29309},
        .input_angle_rad = -205686,
        .expected_quaternion = {101, -2930, -58544, -29309},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-65274, 5144, 2800}
    },
    /* Vector 2: Angle -3.135 rad */
    {
        .input_axis = {5852, 58324, 29309},
        .input_angle_rad = -205485,
        .expected_quaternion = {201, -5852, -58324, -29308},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-64490, 10236, 5592}
    },
    /* Vector 3: Angle -3.132 rad */
    {
        .input_axis = {8760, 57959, 29309},
        .input_angle_rad = -205284,
        .expected_quaternion = {302, -8760, -57958, -29308},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-63192, 15224, 8368}
    },
    /* Vector 4: Angle -3.129 rad */
    {
        .input_axis = {11645, 57449, 29309},
        .input_angle_rad = -205083,
        .expected_quaternion = {402, -11645, -57448, -29308},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-61393, 20056, 11121}
    },
    /* Vector 5: Angle -3.126 rad */
    {
        .input_axis = {14502, 56795, 29309},
        .input_angle_rad = -204882,
        .expected_quaternion = {503, -14502, -56793, -29308},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-59110, 24685, 13842}
    },
    /* Vector 6: Angle -3.123 rad */
    {
        .input_axis = {17323, 55999, 29309},
        .input_angle_rad = -204681,
        .expected_quaternion = {603, -17322, -55997, -29307},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-56368, 29062, 16523}
    },
    /* Vector 7: Angle -3.120 rad */
    {
        .input_axis = {20100, 55063, 29309},
        .input_angle_rad = -204480,
        .expected_quaternion = {704, -20099, -55060, -29307},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53193, 33142, 19158}
    },
    /* Vector 8: Angle -3.117 rad */
    {
        .input_axis = {22827, 53990, 29309},
        .input_angle_rad = -204279,
        .expected_quaternion = {804, -22825, -53986, -29306},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-49617, 36885, 21739}
    },
    /* Vector 9: Angle -3.114 rad */
    {
        .input_axis = {25496, 52782, 29309},
        .input_angle_rad = -204078,
        .expected_quaternion = {905, -25494, -52777, -29306},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-45676, 40252, 24258}
    },
    /* Vector 10: Angle -3.111 rad */
    {
        .input_axis = {28103, 51441, 29309},
        .input_angle_rad = -203877,
        .expected_quaternion = {1005, -28099, -51435, -29305},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-41409, 43208, 26708}
    },
    /* Vector 11: Angle -3.108 rad */
    {
        .input_axis = {30638, 49973, 29309},
        .input_angle_rad = -203676,
        .expected_quaternion = {1106, -30634, -49965, -29304},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-36860, 45723, 29082}
    },
    /* Vector 12: Angle -3.105 rad */
    {
        .input_axis = {33098, 48379, 29309},
        .input_angle_rad = -203475,
        .expected_quaternion = {1206, -33092, -48371, -29304},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-32072, 47770, 31374}
    },
    /* Vector 13: Angle -3.102 rad */
    {
        .input_axis = {35474, 46664, 29309},
        .input_angle_rad = -203274,
        .expected_quaternion = {1307, -35467, -46655, -29303},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-27095, 49329, 33577}
    },
    /* Vector 14: Angle -3.099 rad */
    {
        .input_axis = {37762, 44833, 29309},
        .input_angle_rad = -203073,
        .expected_quaternion = {1407, -37754, -44823, -29302},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-21978, 50384, 35685}
    },
    /* Vector 15: Angle -3.096 rad */
    {
        .input_axis = {39956, 42890, 29309},
        .input_angle_rad = -202871,
        .expected_quaternion = {1508, -39945, -42878, -29301},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16772, 50921, 37692}
    },
    /* Vector 16: Angle -3.093 rad */
    {
        .input_axis = {42049, 40839, 29309},
        .input_angle_rad = -202670,
        .expected_quaternion = {1608, -42037, -40827, -29300},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11530, 50937, 39591}
    },
    /* Vector 17: Angle -3.089 rad */
    {
        .input_axis = {44038, 38686, 29309},
        .input_angle_rad = -202469,
        .expected_quaternion = {1709, -44023, -38673, -29299},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6303, 50429, 41379}
    },
    /* Vector 18: Angle -3.086 rad */
    {
        .input_axis = {45916, 36437, 29309},
        .input_angle_rad = -202268,
        .expected_quaternion = {1809, -45899, -36423, -29297},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1144, 49401, 43049}
    },
    /* Vector 19: Angle -3.083 rad */
    {
        .input_axis = {47680, 34097, 29309},
        .input_angle_rad = -202067,
        .expected_quaternion = {1910, -47660, -34082, -29296},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {3895, 47864, 44597}
    },
    /* Vector 20: Angle -3.080 rad */
    {
        .input_axis = {49325, 31671, 29309},
        .input_angle_rad = -201866,
        .expected_quaternion = {2010, -49301, -31656, -29295},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8764, 45831, 46018}
    },
    /* Vector 21: Angle -3.077 rad */
    {
        .input_axis = {50846, 29166, 29309},
        .input_angle_rad = -201665,
        .expected_quaternion = {2111, -50820, -29151, -29293},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13415, 43323, 47309}
    },
    /* Vector 22: Angle -3.074 rad */
    {
        .input_axis = {52240, 26589, 29309},
        .input_angle_rad = -201464,
        .expected_quaternion = {2211, -52210, -26573, -29292},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17802, 40364, 48465}
    },
    /* Vector 23: Angle -3.071 rad */
    {
        .input_axis = {53504, 23944, 29309},
        .input_angle_rad = -201263,
        .expected_quaternion = {2312, -53470, -23929, -29290},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21879, 36981, 49484}
    },
    /* Vector 24: Angle -3.068 rad */
    {
        .input_axis = {54634, 21240, 29309},
        .input_angle_rad = -201062,
        .expected_quaternion = {2412, -54596, -21226, -29289},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25608, 33210, 50362}
    },
    /* Vector 25: Angle -3.065 rad */
    {
        .input_axis = {55627, 18483, 29309},
        .input_angle_rad = -200861,
        .expected_quaternion = {2513, -55586, -18470, -29287},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28950, 29085, 51097}
    },
    /* Vector 26: Angle -3.062 rad */
    {
        .input_axis = {56481, 15680, 29309},
        .input_angle_rad = -200660,
        .expected_quaternion = {2613, -56436, -15668, -29285},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31872, 24649, 51687}
    },
    /* Vector 27: Angle -3.059 rad */
    {
        .input_axis = {57194, 12838, 29309},
        .input_angle_rad = -200459,
        .expected_quaternion = {2714, -57145, -12827, -29283},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34346, 19944, 52130}
    },
    /* Vector 28: Angle -3.056 rad */
    {
        .input_axis = {57764, 9963, 29309},
        .input_angle_rad = -200258,
        .expected_quaternion = {2814, -57711, -9954, -29282},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36346, 15016, 52425}
    },
    /* Vector 29: Angle -3.053 rad */
    {
        .input_axis = {58190, 7064, 29309},
        .input_angle_rad = -200057,
        .expected_quaternion = {2914, -58132, -7057, -29280},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37854, 9915, 52571}
    },
    /* Vector 30: Angle -3.050 rad */
    {
        .input_axis = {58470, 4146, 29309},
        .input_angle_rad = -199856,
        .expected_quaternion = {3015, -58408, -4142, -29278},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38854, 4689, 52568}
    },
    /* Vector 31: Angle -3.046 rad */
    {
        .input_axis = {58605, 1219, 29309},
        .input_angle_rad = -199654,
        .expected_quaternion = {3115, -58538, -1218, -29275},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39336, -608, 52415}
    },
    /* Vector 32: Angle -3.043 rad */
    {
        .input_axis = {58592, -1712, 29309},
        .input_angle_rad = -199453,
        .expected_quaternion = {3216, -58522, 1710, -29273},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39296, -5926, 52113}
    },
    /* Vector 33: Angle -3.040 rad */
    {
        .input_axis = {58433, -4638, 29309},
        .input_angle_rad = -199252,
        .expected_quaternion = {3316, -58359, 4632, -29271},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38734, -11211, 51662}
    },
    /* Vector 34: Angle -3.037 rad */
    {
        .input_axis = {58129, -7553, 29309},
        .input_angle_rad = -199051,
        .expected_quaternion = {3417, -58050, 7542, -29269},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37657, -16413, 51064}
    },
    /* Vector 35: Angle -3.034 rad */
    {
        .input_axis = {57678, -10448, 29309},
        .input_angle_rad = -198850,
        .expected_quaternion = {3517, -57595, 10433, -29266},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36075, -21479, 50321}
    },
    /* Vector 36: Angle -3.031 rad */
    {
        .input_axis = {57084, -13318, 29309},
        .input_angle_rad = -198649,
        .expected_quaternion = {3617, -56997, 13298, -29264},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34005, -26361, 49434}
    },
    /* Vector 37: Angle -3.028 rad */
    {
        .input_axis = {56347, -16154, 29309},
        .input_angle_rad = -198448,
        .expected_quaternion = {3718, -56257, 16128, -29261},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31468, -31009, 48407}
    },
    /* Vector 38: Angle -3.025 rad */
    {
        .input_axis = {55469, -18950, 29309},
        .input_angle_rad = -198247,
        .expected_quaternion = {3818, -55375, 18918, -29259},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28488, -35379, 47241}
    },
    /* Vector 39: Angle -3.022 rad */
    {
        .input_axis = {54453, -21699, 29309},
        .input_angle_rad = -198046,
        .expected_quaternion = {3918, -54356, 21660, -29256},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25098, -39428, 45940}
    },
    /* Vector 40: Angle -3.019 rad */
    {
        .input_axis = {53300, -24393, 29309},
        .input_angle_rad = -197845,
        .expected_quaternion = {4019, -53200, 24347, -29253},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21329, -43117, 44508}
    },
    /* Vector 41: Angle -3.016 rad */
    {
        .input_axis = {52015, -27027, 29309},
        .input_angle_rad = -197644,
        .expected_quaternion = {4119, -51912, 26973, -29251},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17222, -46409, 42949}
    },
    /* Vector 42: Angle -3.013 rad */
    {
        .input_axis = {50599, -29593, 29309},
        .input_angle_rad = -197443,
        .expected_quaternion = {4219, -50494, 29531, -29248},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12816, -49272, 41267}
    },
    /* Vector 43: Angle -3.010 rad */
    {
        .input_axis = {49057, -32085, 29309},
        .input_angle_rad = -197242,
        .expected_quaternion = {4320, -48950, 32015, -29245},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8157, -51680, 39467}
    },
    /* Vector 44: Angle -3.007 rad */
    {
        .input_axis = {47392, -34496, 29309},
        .input_angle_rad = -197041,
        .expected_quaternion = {4420, -47284, 34418, -29242},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {3290, -53609, 37553}
    },
    /* Vector 45: Angle -3.004 rad */
    {
        .input_axis = {45608, -36822, 29309},
        .input_angle_rad = -196840,
        .expected_quaternion = {4520, -45500, 36734, -29239},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1734, -55040, 35532}
    },
    /* Vector 46: Angle -3.000 rad */
    {
        .input_axis = {43711, -39055, 29309},
        .input_angle_rad = -196639,
        .expected_quaternion = {4621, -43602, 38958, -29236},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6865, -55962, 33409}
    },
    /* Vector 47: Angle -2.997 rad */
    {
        .input_axis = {41705, -41191, 29309},
        .input_angle_rad = -196438,
        .expected_quaternion = {4721, -41596, 41084, -29232},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-12053, -56364, 31189}
    },
    /* Vector 48: Angle -2.994 rad */
    {
        .input_axis = {39594, -43224, 29309},
        .input_angle_rad = -196236,
        .expected_quaternion = {4821, -39486, 43107, -29229},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-17244, -56246, 28880}
    },
    /* Vector 49: Angle -2.991 rad */
    {
        .input_axis = {37384, -45149, 29309},
        .input_angle_rad = -196035,
        .expected_quaternion = {4921, -37278, 45021, -29226},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22387, -55608, 26487}
    },
    /* Vector 50: Angle -2.988 rad */
    {
        .input_axis = {35081, -46961, 29309},
        .input_angle_rad = -195834,
        .expected_quaternion = {5022, -34978, 46823, -29222},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-27430, -54458, 24017}
    },
    /* Vector 51: Angle -2.985 rad */
    {
        .input_axis = {32690, -48655, 29309},
        .input_angle_rad = -195633,
        .expected_quaternion = {5122, -32590, 48507, -29219},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-32323, -52810, 21478}
    },
    /* Vector 52: Angle -2.982 rad */
    {
        .input_axis = {30217, -50228, 29309},
        .input_angle_rad = -195432,
        .expected_quaternion = {5222, -30121, 50069, -29215},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-37016, -50680, 18876}
    },
    /* Vector 53: Angle -2.979 rad */
    {
        .input_axis = {27669, -51676, 29309},
        .input_angle_rad = -195231,
        .expected_quaternion = {5322, -27578, 51505, -29212},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-41462, -48092, 16219}
    },
    /* Vector 54: Angle -2.976 rad */
    {
        .input_axis = {25052, -52994, 29309},
        .input_angle_rad = -195030,
        .expected_quaternion = {5422, -24966, 52812, -29208},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-45617, -45071, 13514}
    },
    /* Vector 55: Angle -2.973 rad */
    {
        .input_axis = {22372, -54180, 29309},
        .input_angle_rad = -194829,
        .expected_quaternion = {5523, -22292, 53987, -29204},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-49440, -41650, 10769}
    },
    /* Vector 56: Angle -2.970 rad */
    {
        .input_axis = {19636, -55230, 29309},
        .input_angle_rad = -194628,
        .expected_quaternion = {5623, -19564, 55027, -29201},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-52891, -37864, 7991}
    },
    /* Vector 57: Angle -2.967 rad */
    {
        .input_axis = {16851, -56143, 29309},
        .input_angle_rad = -194427,
        .expected_quaternion = {5723, -16787, 55928, -29197},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-55937, -33751, 5189}
    },
    /* Vector 58: Angle -2.964 rad */
    {
        .input_axis = {14024, -56915, 29309},
        .input_angle_rad = -194226,
        .expected_quaternion = {5823, -13969, 56690, -29193},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-58547, -29354, 2370}
    },
    /* Vector 59: Angle -2.961 rad */
    {
        .input_axis = {11162, -57545, 29309},
        .input_angle_rad = -194025,
        .expected_quaternion = {5923, -11116, 57309, -29189},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-60694, -24718, -457}
    },
    /* Vector 60: Angle -2.958 rad */
    {
        .input_axis = {8272, -58031, 29309},
        .input_angle_rad = -193824,
        .expected_quaternion = {6023, -8237, 57785, -29185},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-62358, -19890, -3286}
    },
    /* Vector 61: Angle -2.954 rad */
    {
        .input_axis = {5361, -58371, 29309},
        .input_angle_rad = -193623,
        .expected_quaternion = {6123, -5338, 58116, -29180},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-63522, -14920, -6107}
    },
    /* Vector 62: Angle -2.951 rad */
    {
        .input_axis = {2437, -58566, 29309},
        .input_angle_rad = -193422,
        .expected_quaternion = {6224, -2426, 58302, -29176},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-64174, -9858, -8913}
    },
    /* Vector 63: Angle -2.948 rad */
    {
        .input_axis = {-493, -58615, 29309},
        .input_angle_rad = -193221,
        .expected_quaternion = {6324, 491, 58342, -29172},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-64308, -4756, -11695}
    },
    /* Vector 64: Angle -2.945 rad */
    {
        .input_axis = {-3422, -58517, 29309},
        .input_angle_rad = -193019,
        .expected_quaternion = {6424, 3405, 58235, -29167},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-63923, 334, -14447}
    },
    /* Vector 65: Angle -2.942 rad */
    {
        .input_axis = {-6342, -58273, 29309},
        .input_angle_rad = -192818,
        .expected_quaternion = {6524, 6311, 57984, -29163},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-63022, 5361, -17160}
    },
    /* Vector 66: Angle -2.939 rad */
    {
        .input_axis = {-9247, -57883, 29309},
        .input_angle_rad = -192617,
        .expected_quaternion = {6624, 9199, 57587, -29159},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-61614, 10273, -19827}
    },
    /* Vector 67: Angle -2.936 rad */
    {
        .input_axis = {-12128, -57349, 29309},
        .input_angle_rad = -192416,
        .expected_quaternion = {6724, 12064, 57046, -29154},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-59715, 15020, -22439}
    },
    /* Vector 68: Angle -2.933 rad */
    {
        .input_axis = {-14979, -56671, 29309},
        .input_angle_rad = -192215,
        .expected_quaternion = {6824, 14898, 56363, -29149},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-57342, 19555, -24990}
    },
    /* Vector 69: Angle -2.930 rad */
    {
        .input_axis = {-17793, -55852, 29309},
        .input_angle_rad = -192014,
        .expected_quaternion = {6924, 17693, 55539, -29145},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-54520, 23830, -27472}
    },
    /* Vector 70: Angle -2.927 rad */
    {
        .input_axis = {-20562, -54892, 29309},
        .input_angle_rad = -191813,
        .expected_quaternion = {7024, 20443, 54576, -29140},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-51276, 27803, -29878}
    },
    /* Vector 71: Angle -2.924 rad */
    {
        .input_axis = {-23280, -53796, 29309},
        .input_angle_rad = -191612,
        .expected_quaternion = {7124, 23142, 53477, -29135},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-47644, 31434, -32202}
    },
    /* Vector 72: Angle -2.921 rad */
    {
        .input_axis = {-25939, -52565, 29309},
        .input_angle_rad = -191411,
        .expected_quaternion = {7224, 25781, 52245, -29130},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43659, 34684, -34436}
    },
    /* Vector 73: Angle -2.918 rad */
    {
        .input_axis = {-28534, -51203, 29309},
        .input_angle_rad = -191210,
        .expected_quaternion = {7323, 28355, 50883, -29125},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39362, 37521, -36575}
    },
    /* Vector 74: Angle -2.915 rad */
    {
        .input_axis = {-31058, -49713, 29309},
        .input_angle_rad = -191009,
        .expected_quaternion = {7423, 30858, 49393, -29120},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-34796, 39917, -38612}
    },
    /* Vector 75: Angle -2.911 rad */
    {
        .input_axis = {-33503, -48099, 29309},
        .input_angle_rad = -190808,
        .expected_quaternion = {7523, 33282, 47781, -29115},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30005, 41846, -40541}
    },
    /* Vector 76: Angle -2.908 rad */
    {
        .input_axis = {-35865, -46364, 29309},
        .input_angle_rad = -190607,
        .expected_quaternion = {7623, 35622, 46050, -29110},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-25038, 43288, -42358}
    },
    /* Vector 77: Angle -2.905 rad */
    {
        .input_axis = {-38138, -44514, 29309},
        .input_angle_rad = -190406,
        .expected_quaternion = {7723, 37872, 44204, -29104},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19945, 44230, -44056}
    },
    /* Vector 78: Angle -2.902 rad */
    {
        .input_axis = {-40315, -42552, 29309},
        .input_angle_rad = -190205,
        .expected_quaternion = {7823, 40027, 42248, -29099},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14775, 44660, -45631}
    },
    /* Vector 79: Angle -2.899 rad */
    {
        .input_axis = {-42391, -40484, 29309},
        .input_angle_rad = -190004,
        .expected_quaternion = {7923, 42080, 40187, -29094},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-9581, 44574, -47078}
    },
    /* Vector 80: Angle -2.896 rad */
    {
        .input_axis = {-44362, -38315, 29309},
        .input_angle_rad = -189802,
        .expected_quaternion = {8022, 44028, 38027, -29088},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4415, 43972, -48393}
    },
    /* Vector 81: Angle -2.893 rad */
    {
        .input_axis = {-46221, -36050, 29309},
        .input_angle_rad = -189601,
        .expected_quaternion = {8122, 45865, 35772, -29083},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {673, 42861, -49573}
    },
    /* Vector 82: Angle -2.890 rad */
    {
        .input_axis = {-47965, -33695, 29309},
        .input_angle_rad = -189400,
        .expected_quaternion = {8222, 47586, 33428, -29077},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5632, 41249, -50614}
    },
    /* Vector 83: Angle -2.887 rad */
    {
        .input_axis = {-49589, -31255, 29309},
        .input_angle_rad = -189199,
        .expected_quaternion = {8322, 49188, 31002, -29071},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10413, 39154, -51512}
    },
    /* Vector 84: Angle -2.884 rad */
    {
        .input_axis = {-51089, -28738, 29309},
        .input_angle_rad = -188998,
        .expected_quaternion = {8421, 50666, 28499, -29066},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {14967, 36596, -52265}
    },
    /* Vector 85: Angle -2.881 rad */
    {
        .input_axis = {-52462, -26148, 29309},
        .input_angle_rad = -188797,
        .expected_quaternion = {8521, 52016, 25926, -29060},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19251, 33599, -52872}
    },
    /* Vector 86: Angle -2.878 rad */
    {
        .input_axis = {-53703, -23494, 29309},
        .input_angle_rad = -188596,
        .expected_quaternion = {8621, 53236, 23290, -29054},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23222, 30194, -53329}
    },
    /* Vector 87: Angle -2.875 rad */
    {
        .input_axis = {-54810, -20780, 29309},
        .input_angle_rad = -188395,
        .expected_quaternion = {8720, 54323, 20596, -29048},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26841, 26413, -53637}
    },
    /* Vector 88: Angle -2.872 rad */
    {
        .input_axis = {-55780, -18015, 29309},
        .input_angle_rad = -188194,
        .expected_quaternion = {8820, 55273, 17851, -29042},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30072, 22294, -53793}
    },
    /* Vector 89: Angle -2.869 rad */
    {
        .input_axis = {-56611, -15205, 29309},
        .input_angle_rad = -187993,
        .expected_quaternion = {8919, 56084, 15063, -29036},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32883, 17878, -53797}
    },
    /* Vector 90: Angle -2.865 rad */
    {
        .input_axis = {-57300, -12356, 29309},
        .input_angle_rad = -187792,
        .expected_quaternion = {9019, 56755, 12239, -29030},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35247, 13208, -53649}
    },
    /* Vector 91: Angle -2.862 rad */
    {
        .input_axis = {-57846, -9477, 29309},
        .input_angle_rad = -187591,
        .expected_quaternion = {9119, 57283, 9385, -29024},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37141, 8329, -53349}
    },
    /* Vector 92: Angle -2.859 rad */
    {
        .input_axis = {-58247, -6574, 29309},
        .input_angle_rad = -187390,
        .expected_quaternion = {9218, 57668, 6509, -29017},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38547, 3292, -52898}
    },
    /* Vector 93: Angle -2.856 rad */
    {
        .input_axis = {-58503, -3655, 29309},
        .input_angle_rad = -187189,
        .expected_quaternion = {9318, 57909, 3618, -29011},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39452, -1856, -52298}
    },
    /* Vector 94: Angle -2.853 rad */
    {
        .input_axis = {-58613, -726, 29309},
        .input_angle_rad = -186988,
        .expected_quaternion = {9417, 58004, 719, -29004},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39847, -7063, -51549}
    },
    /* Vector 95: Angle -2.850 rad */
    {
        .input_axis = {-58576, 2204, 29309},
        .input_angle_rad = -186787,
        .expected_quaternion = {9517, 57955, -2181, -28998},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39729, -12279, -50654}
    },
    /* Vector 96: Angle -2.847 rad */
    {
        .input_axis = {-58392, 5129, 29309},
        .input_angle_rad = -186585,
        .expected_quaternion = {9616, 57760, -5073, -28991},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39100, -17451, -49614}
    },
    /* Vector 97: Angle -2.844 rad */
    {
        .input_axis = {-58063, 8041, 29309},
        .input_angle_rad = -186384,
        .expected_quaternion = {9716, 57421, -7952, -28985},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37968, -22529, -48434}
    },
    /* Vector 98: Angle -2.841 rad */
    {
        .input_axis = {-57589, 10933, 29309},
        .input_angle_rad = -186183,
        .expected_quaternion = {9815, 56939, -10810, -28978},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36344, -27463, -47116}
    },
    /* Vector 99: Angle -2.838 rad */
    {
        .input_axis = {-56970, 13797, 29309},
        .input_angle_rad = -185982,
        .expected_quaternion = {9914, 56315, -13639, -28971},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34245, -32205, -45663}
    },
    /* Vector 100: Angle -2.835 rad */
    {
        .input_axis = {-56209, 16627, 29309},
        .input_angle_rad = -185781,
        .expected_quaternion = {10014, 55549, -16432, -28964},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31693, -36708, -44080}
    },
    /* Vector 101: Angle -2.832 rad */
    {
        .input_axis = {-55308, 19416, 29309},
        .input_angle_rad = -185580,
        .expected_quaternion = {10113, 54646, -19183, -28958},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28715, -40928, -42371}
    },
    /* Vector 102: Angle -2.829 rad */
    {
        .input_axis = {-54269, 22156, 29309},
        .input_angle_rad = -185379,
        .expected_quaternion = {10212, 53606, -21885, -28951},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25341, -44825, -40540}
    },
    /* Vector 103: Angle -2.826 rad */
    {
        .input_axis = {-53093, 24841, 29309},
        .input_angle_rad = -185178,
        .expected_quaternion = {10312, 52432, -24531, -28944},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21606, -48361, -38593}
    },
    /* Vector 104: Angle -2.823 rad */
    {
        .input_axis = {-51786, 27463, 29309},
        .input_angle_rad = -184977,
        .expected_quaternion = {10411, 51128, -27114, -28936},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17547, -51500, -36535}
    },
    /* Vector 105: Angle -2.819 rad */
    {
        .input_axis = {-50348, 30017, 29309},
        .input_angle_rad = -184776,
        .expected_quaternion = {10510, 49697, -29628, -28929},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13206, -54214, -34372}
    },
    /* Vector 106: Angle -2.816 rad */
    {
        .input_axis = {-48785, 32496, 29309},
        .input_angle_rad = -184575,
        .expected_quaternion = {10609, 48142, -32067, -28922},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8627, -56476, -32109}
    },
    /* Vector 107: Angle -2.813 rad */
    {
        .input_axis = {-47100, 34893, 29309},
        .input_angle_rad = -184374,
        .expected_quaternion = {10709, 46467, -34425, -28915},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {3857, -58265, -29753}
    },
    /* Vector 108: Angle -2.810 rad */
    {
        .input_axis = {-45297, 37204, 29309},
        .input_angle_rad = -184173,
        .expected_quaternion = {10808, 44677, -36695, -28907},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1057, -59565, -27310}
    },
    /* Vector 109: Angle -2.807 rad */
    {
        .input_axis = {-43381, 39421, 29309},
        .input_angle_rad = -183972,
        .expected_quaternion = {10907, 42776, -38872, -28900},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6064, -60363, -24788}
    },
    /* Vector 110: Angle -2.804 rad */
    {
        .input_axis = {-41357, 41540, 29309},
        .input_angle_rad = -183771,
        .expected_quaternion = {11006, 40769, -40950, -28892},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11115, -60654, -22193}
    },
    /* Vector 111: Angle -2.801 rad */
    {
        .input_axis = {-39229, 43555, 29309},
        .input_angle_rad = -183570,
        .expected_quaternion = {11105, 38662, -42925, -28885},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16157, -60435, -19533}
    },
    /* Vector 112: Angle -2.798 rad */
    {
        .input_axis = {-37003, 45461, 29309},
        .input_angle_rad = -183368,
        .expected_quaternion = {11204, 36458, -44792, -28877},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-21141, -59710, -16814}
    },
    /* Vector 113: Angle -2.795 rad */
    {
        .input_axis = {-34685, 47254, 29309},
        .input_angle_rad = -183167,
        .expected_quaternion = {11303, 34165, -46546, -28869},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26016, -58489, -14044}
    },
    /* Vector 114: Angle -2.792 rad */
    {
        .input_axis = {-32280, 48929, 29309},
        .input_angle_rad = -182966,
        .expected_quaternion = {11402, 31787, -48182, -28862},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30732, -56783, -11232}
    },
    /* Vector 115: Angle -2.789 rad */
    {
        .input_axis = {-29794, 50481, 29309},
        .input_angle_rad = -182765,
        .expected_quaternion = {11501, 29331, -49697, -28854},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-35244, -54613, -8385}
    },
    /* Vector 116: Angle -2.786 rad */
    {
        .input_axis = {-27234, 51907, 29309},
        .input_angle_rad = -182564,
        .expected_quaternion = {11600, 26804, -51087, -28846},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39505, -52000, -5510}
    },
    /* Vector 117: Angle -2.783 rad */
    {
        .input_axis = {-24605, 53203, 29309},
        .input_angle_rad = -182363,
        .expected_quaternion = {11699, 24210, -52348, -28838},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43472, -48973, -2617}
    },
    /* Vector 118: Angle -2.780 rad */
    {
        .input_axis = {-21916, 54366, 29309},
        .input_angle_rad = -182162,
        .expected_quaternion = {11798, 21558, -53478, -28830},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-47106, -45562, 288}
    },
    /* Vector 119: Angle -2.777 rad */
    {
        .input_axis = {-19171, 55394, 29309},
        .input_angle_rad = -181961,
        .expected_quaternion = {11897, 18853, -54473, -28822},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-50370, -41804, 3195}
    },
    /* Vector 120: Angle -2.773 rad */
    {
        .input_axis = {-16379, 56282, 29309},
        .input_angle_rad = -181760,
        .expected_quaternion = {11996, 16102, -55332, -28813},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53232, -37737, 6097}
    },
    /* Vector 121: Angle -2.770 rad */
    {
        .input_axis = {-13545, 57031, 29309},
        .input_angle_rad = -181559,
        .expected_quaternion = {12095, 13312, -56051, -28805},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-55664, -33403, 8986}
    },
    /* Vector 122: Angle -2.767 rad */
    {
        .input_axis = {-10678, 57636, 29309},
        .input_angle_rad = -181358,
        .expected_quaternion = {12193, 10491, -56630, -28797},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-57640, -28847, 11853}
    },
    /* Vector 123: Angle -2.764 rad */
    {
        .input_axis = {-7784, 58098, 29309},
        .input_angle_rad = -181157,
        .expected_quaternion = {12292, 7646, -57067, -28788},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-59141, -24115, 14690}
    },
    /* Vector 124: Angle -2.761 rad */
    {
        .input_axis = {-4870, 58414, 29309},
        .input_angle_rad = -180956,
        .expected_quaternion = {12391, 4783, -57361, -28780},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-60153, -19255, 17490}
    },
    /* Vector 125: Angle -2.758 rad */
    {
        .input_axis = {-1945, 58585, 29309},
        .input_angle_rad = -180755,
        .expected_quaternion = {12490, 1909, -57511, -28771},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-60664, -14317, 20244}
    },
    /* Vector 126: Angle -2.755 rad */
    {
        .input_axis = {986, 58609, 29309},
        .input_angle_rad = -180554,
        .expected_quaternion = {12588, -967, -57518, -28763},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-60672, -9352, 22945}
    },
    /* Vector 127: Angle -2.752 rad */
    {
        .input_axis = {3914, 58486, 29309},
        .input_angle_rad = -180353,
        .expected_quaternion = {12687, -3840, -57380, -28754},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-60174, -4409, 25585}
    },
    /* Vector 128: Angle -2.749 rad */
    {
        .input_axis = {6832, 58218, 29309},
        .input_angle_rad = -180151,
        .expected_quaternion = {12785, -6701, -57099, -28745},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-59177, 460, 28157}
    },
    /* Vector 129: Angle -2.746 rad */
    {
        .input_axis = {9733, 57803, 29309},
        .input_angle_rad = -179950,
        .expected_quaternion = {12884, -9543, -56675, -28737},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-57691, 5207, 30653}
    },
    /* Vector 130: Angle -2.743 rad */
    {
        .input_axis = {12610, 57245, 29309},
        .input_angle_rad = -179749,
        .expected_quaternion = {12983, -12360, -56110, -28728},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-55730, 9783, 33067}
    },
    /* Vector 131: Angle -2.740 rad */
    {
        .input_axis = {15455, 56543, 29309},
        .input_angle_rad = -179548,
        .expected_quaternion = {13081, -15144, -55405, -28719},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53315, 14141, 35391}
    },
    /* Vector 132: Angle -2.737 rad */
    {
        .input_axis = {18262, 55700, 29309},
        .input_angle_rad = -179347,
        .expected_quaternion = {13180, -17889, -54562, -28710},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-50469, 18239, 37618}
    },
    /* Vector 133: Angle -2.734 rad */
    {
        .input_axis = {21023, 54718, 29309},
        .input_angle_rad = -179146,
        .expected_quaternion = {13278, -20587, -53583, -28701},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-47222, 22034, 39744}
    },
    /* Vector 134: Angle -2.730 rad */
    {
        .input_axis = {23731, 53599, 29309},
        .input_angle_rad = -178945,
        .expected_quaternion = {13376, -23232, -52470, -28692},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43605, 25487, 41761}
    },
    /* Vector 135: Angle -2.727 rad */
    {
        .input_axis = {26380, 52346, 29309},
        .input_angle_rad = -178744,
        .expected_quaternion = {13475, -25817, -51227, -28682},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39655, 28565, 43663}
    },
    /* Vector 136: Angle -2.724 rad */
    {
        .input_axis = {28964, 50962, 29309},
        .input_angle_rad = -178543,
        .expected_quaternion = {13573, -28336, -49857, -28673},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-35411, 31236, 45446}
    },
    /* Vector 137: Angle -2.721 rad */
    {
        .input_axis = {31474, 49450, 29309},
        .input_angle_rad = -178342,
        .expected_quaternion = {13672, -30782, -48362, -28664},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30916, 33472, 47104}
    },
    /* Vector 138: Angle -2.718 rad */
    {
        .input_axis = {33907, 47816, 29309},
        .input_angle_rad = -178141,
        .expected_quaternion = {13770, -33150, -46748, -28654},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26214, 35251, 48633}
    },
    /* Vector 139: Angle -2.715 rad */
    {
        .input_axis = {36254, 46061, 29309},
        .input_angle_rad = -177940,
        .expected_quaternion = {13868, -35433, -45018, -28645},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-21352, 36556, 50027}
    },
    /* Vector 140: Angle -2.712 rad */
    {
        .input_axis = {38511, 44192, 29309},
        .input_angle_rad = -177739,
        .expected_quaternion = {13966, -37626, -43176, -28635},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16379, 37373, 51283}
    },
    /* Vector 141: Angle -2.709 rad */
    {
        .input_axis = {40671, 42212, 29309},
        .input_angle_rad = -177538,
        .expected_quaternion = {14065, -39724, -41228, -28626},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11344, 37693, 52398}
    },
    /* Vector 142: Angle -2.706 rad */
    {
        .input_axis = {42730, 40126, 29309},
        .input_angle_rad = -177337,
        .expected_quaternion = {14163, -41720, -39178, -28616},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6296, 37513, 53367}
    },
    /* Vector 143: Angle -2.703 rad */
    {
        .input_axis = {44682, 37940, 29309},
        .input_angle_rad = -177136,
        .expected_quaternion = {14261, -43611, -37031, -28606},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1286, 36836, 54189}
    },
    /* Vector 144: Angle -2.700 rad */
    {
        .input_axis = {46523, 35660, 29309},
        .input_angle_rad = -176934,
        .expected_quaternion = {14359, -45392, -34793, -28596},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {3636, 35667, 54860}
    },
    /* Vector 145: Angle -2.697 rad */
    {
        .input_axis = {48247, 33290, 29309},
        .input_angle_rad = -176733,
        .expected_quaternion = {14457, -47058, -32470, -28587},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8423, 34018, 55379}
    },
    /* Vector 146: Angle -2.694 rad */
    {
        .input_axis = {49850, 30837, 29309},
        .input_angle_rad = -176532,
        .expected_quaternion = {14555, -48605, -30067, -28577},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13026, 31905, 55743}
    },
    /* Vector 147: Angle -2.691 rad */
    {
        .input_axis = {51329, 28307, 29309},
        .input_angle_rad = -176331,
        .expected_quaternion = {14653, -50030, -27591, -28567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17401, 29350, 55953}
    },
    /* Vector 148: Angle -2.688 rad */
    {
        .input_axis = {52680, 25706, 29309},
        .input_angle_rad = -176130,
        .expected_quaternion = {14751, -51328, -25047, -28557},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21505, 26378, 56006}
    },
    /* Vector 149: Angle -2.684 rad */
    {
        .input_axis = {53899, 23041, 29309},
        .input_angle_rad = -175929,
        .expected_quaternion = {14849, -52497, -22442, -28546},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25297, 23018, 55903}
    },
    /* Vector 150: Angle -2.681 rad */
    {
        .input_axis = {54983, 20319, 29309},
        .input_angle_rad = -175728,
        .expected_quaternion = {14947, -53534, -19783, -28536},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28741, 19304, 55644}
    },
    /* Vector 151: Angle -2.678 rad */
    {
        .input_axis = {55930, 17545, 29309},
        .input_angle_rad = -175527,
        .expected_quaternion = {15045, -54436, -17077, -28526},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31804, 15272, 55229}
    },
    /* Vector 152: Angle -2.675 rad */
    {
        .input_axis = {56737, 14728, 29309},
        .input_angle_rad = -175326,
        .expected_quaternion = {15143, -55201, -14330, -28515},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34455, 10962, 54660}
    },
    /* Vector 153: Angle -2.672 rad */
    {
        .input_axis = {57402, 11874, 29309},
        .input_angle_rad = -175125,
        .expected_quaternion = {15240, -55828, -11549, -28505},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36669, 6418, 53937}
    },
    /* Vector 154: Angle -2.669 rad */
    {
        .input_axis = {57924, 8990, 29309},
        .input_angle_rad = -174924,
        .expected_quaternion = {15338, -56315, -8741, -28495},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38426, 1684, 53062}
    },
    /* Vector 155: Angle -2.666 rad */
    {
        .input_axis = {58301, 6084, 29309},
        .input_angle_rad = -174723,
        .expected_quaternion = {15436, -56660, -5913, -28484},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39709, -3194, 52038}
    },
    /* Vector 156: Angle -2.663 rad */
    {
        .input_axis = {58532, 3163, 29309},
        .input_angle_rad = -174522,
        .expected_quaternion = {15534, -56864, -3073, -28473},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40506, -8166, 50868}
    },
    /* Vector 157: Angle -2.660 rad */
    {
        .input_axis = {58617, 233, 29309},
        .input_angle_rad = -174321,
        .expected_quaternion = {15631, -56925, -227, -28463},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40811, -13184, 49554}
    },
    /* Vector 158: Angle -2.657 rad */
    {
        .input_axis = {58555, -2697, 29309},
        .input_angle_rad = -174120,
        .expected_quaternion = {15729, -56844, 2618, -28452},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40622, -18198, 48100}
    },
    /* Vector 159: Angle -2.654 rad */
    {
        .input_axis = {58347, -5620, 29309},
        .input_angle_rad = -173919,
        .expected_quaternion = {15826, -56620, 5453, -28441},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39943, -23160, 46510}
    },
    /* Vector 160: Angle -2.651 rad */
    {
        .input_axis = {57993, -8529, 29309},
        .input_angle_rad = -173718,
        .expected_quaternion = {15924, -56255, 8273, -28430},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38780, -28019, 44788}
    },
    /* Vector 161: Angle -2.648 rad */
    {
        .input_axis = {57495, -11417, 29309},
        .input_angle_rad = -173516,
        .expected_quaternion = {16021, -55750, 11070, -28419},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37148, -32730, 42939}
    },
    /* Vector 162: Angle -2.645 rad */
    {
        .input_axis = {56852, -14276, 29309},
        .input_angle_rad = -173315,
        .expected_quaternion = {16119, -55106, 13837, -28408},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35064, -37244, 40967}
    },
    /* Vector 163: Angle -2.642 rad */
    {
        .input_axis = {56068, -17099, 29309},
        .input_angle_rad = -173114,
        .expected_quaternion = {16216, -54324, 16568, -28397},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32550, -41520, 38879}
    },
    /* Vector 164: Angle -2.638 rad */
    {
        .input_axis = {55143, -19880, 29309},
        .input_angle_rad = -172913,
        .expected_quaternion = {16314, -53407, 19255, -28386},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29632, -45514, 36679}
    },
    /* Vector 165: Angle -2.635 rad */
    {
        .input_axis = {54080, -22611, 29309},
        .input_angle_rad = -172712,
        .expected_quaternion = {16411, -52357, 21891, -28375},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26341, -49189, 34374}
    },
    /* Vector 166: Angle -2.632 rad */
    {
        .input_axis = {52883, -25286, 29309},
        .input_angle_rad = -172511,
        .expected_quaternion = {16508, -51178, 24471, -28364},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22711, -52508, 31970}
    },
    /* Vector 167: Angle -2.629 rad */
    {
        .input_axis = {51553, -27898, 29309},
        .input_angle_rad = -172310,
        .expected_quaternion = {16606, -49871, 26987, -28352},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18779, -55440, 29474}
    },
    /* Vector 168: Angle -2.626 rad */
    {
        .input_axis = {50094, -30439, 29309},
        .input_angle_rad = -172109,
        .expected_quaternion = {16703, -48440, 29434, -28341},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {14585, -57958, 26892}
    },
    /* Vector 169: Angle -2.623 rad */
    {
        .input_axis = {48510, -32905, 29309},
        .input_angle_rad = -171908,
        .expected_quaternion = {16800, -46889, 31805, -28329},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10173, -60036, 24231}
    },
    /* Vector 170: Angle -2.620 rad */
    {
        .input_axis = {46805, -35288, 29309},
        .input_angle_rad = -171707,
        .expected_quaternion = {16897, -45223, 34095, -28318},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5588, -61657, 21499}
    },
    /* Vector 171: Angle -2.617 rad */
    {
        .input_axis = {44983, -37583, 29309},
        .input_angle_rad = -171506,
        .expected_quaternion = {16994, -43444, 36298, -28306},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {876, -62804, 18703}
    },
    /* Vector 172: Angle -2.614 rad */
    {
        .input_axis = {43048, -39785, 29309},
        .input_angle_rad = -171305,
        .expected_quaternion = {17091, -41559, 38408, -28294},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-3914, -63469, 15852}
    },
    /* Vector 173: Angle -2.611 rad */
    {
        .input_axis = {41006, -41886, 29309},
        .input_angle_rad = -171104,
        .expected_quaternion = {17188, -39571, 40420, -28283},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-8734, -63647, 12952}
    },
    /* Vector 174: Angle -2.608 rad */
    {
        .input_axis = {38861, -43884, 29309},
        .input_angle_rad = -170903,
        .expected_quaternion = {17285, -37485, 42330, -28271},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13536, -63337, 10011}
    },
    /* Vector 175: Angle -2.605 rad */
    {
        .input_axis = {36620, -45771, 29309},
        .input_angle_rad = -170702,
        .expected_quaternion = {17382, -35308, 44132, -28259},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18270, -62543, 7039}
    },
    /* Vector 176: Angle -2.602 rad */
    {
        .input_axis = {34286, -47544, 29309},
        .input_angle_rad = -170501,
        .expected_quaternion = {17479, -33044, 45822, -28247},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22889, -61276, 4043}
    },
    /* Vector 177: Angle -2.599 rad */
    {
        .input_axis = {31867, -49198, 29309},
        .input_angle_rad = -170299,
        .expected_quaternion = {17576, -30700, 47396, -28235},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-27346, -59549, 1030}
    },
    /* Vector 178: Angle -2.595 rad */
    {
        .input_axis = {29368, -50729, 29309},
        .input_angle_rad = -170098,
        .expected_quaternion = {17673, -28280, 48850, -28223},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-31597, -57382, -1989}
    },
    /* Vector 179: Angle -2.592 rad */
    {
        .input_axis = {26796, -52134, 29309},
        .input_angle_rad = -169897,
        .expected_quaternion = {17770, -25792, 50181, -28211},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-35598, -54797, -5007}
    },
    /* Vector 180: Angle -2.589 rad */
    {
        .input_axis = {24157, -53408, 29309},
        .input_angle_rad = -169696,
        .expected_quaternion = {17867, -23242, 51385, -28198},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39309, -51822, -8016}
    },
    /* Vector 181: Angle -2.586 rad */
    {
        .input_axis = {21458, -54548, 29309},
        .input_angle_rad = -169495,
        .expected_quaternion = {17963, -20636, 52459, -28186},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-42693, -48488, -11007}
    },
    /* Vector 182: Angle -2.583 rad */
    {
        .input_axis = {18705, -55553, 29309},
        .input_angle_rad = -169294,
        .expected_quaternion = {18060, -17980, 53402, -28174},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-45716, -44830, -13973}
    },
    /* Vector 183: Angle -2.580 rad */
    {
        .input_axis = {15905, -56418, 29309},
        .input_angle_rad = -169093,
        .expected_quaternion = {18156, -15282, 54210, -28161},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-48348, -40886, -16903}
    },
    /* Vector 184: Angle -2.577 rad */
    {
        .input_axis = {13065, -57143, 29309},
        .input_angle_rad = -168892,
        .expected_quaternion = {18253, -12548, 54881, -28149},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-50563, -36696, -19792}
    },
    /* Vector 185: Angle -2.574 rad */
    {
        .input_axis = {10193, -57724, 29309},
        .input_angle_rad = -168691,
        .expected_quaternion = {18350, -9785, 55415, -28136},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-52338, -32304, -22630}
    },
    /* Vector 186: Angle -2.571 rad */
    {
        .input_axis = {7295, -58161, 29309},
        .input_angle_rad = -168490,
        .expected_quaternion = {18446, -7000, 55810, -28124},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53657, -27754, -25409}
    },
    /* Vector 187: Angle -2.568 rad */
    {
        .input_axis = {4379, -58453, 29309},
        .input_angle_rad = -168289,
        .expected_quaternion = {18543, -4200, 56065, -28111},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-54505, -23094, -28122}
    },
    /* Vector 188: Angle -2.565 rad */
    {
        .input_axis = {1452, -58599, 29309},
        .input_angle_rad = -168088,
        .expected_quaternion = {18639, -1392, 56179, -28098},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-54875, -18370, -30762}
    },
    /* Vector 189: Angle -2.562 rad */
    {
        .input_axis = {-1478, -58599, 29309},
        .input_angle_rad = -167887,
        .expected_quaternion = {18735, 1417, 56153, -28085},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-54763, -13630, -33320}
    },
    /* Vector 190: Angle -2.559 rad */
    {
        .input_axis = {-4405, -58451, 29309},
        .input_angle_rad = -167686,
        .expected_quaternion = {18832, 4219, 55986, -28073},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-54170, -8924, -35790}
    },
    /* Vector 191: Angle -2.556 rad */
    {
        .input_axis = {-7321, -58158, 29309},
        .input_angle_rad = -167485,
        .expected_quaternion = {18928, 7009, 55680, -28060},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53103, -4298, -38164}
    },
    /* Vector 192: Angle -2.553 rad */
    {
        .input_axis = {-10219, -57720, 29309},
        .input_angle_rad = -167284,
        .expected_quaternion = {19024, 9779, 55234, -28047},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-51573, 200, -40437}
    },
    /* Vector 193: Angle -2.549 rad */
    {
        .input_axis = {-13091, -57137, 29309},
        .input_angle_rad = -167082,
        .expected_quaternion = {19120, 12521, 54651, -28033},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-49595, 4525, -42601}
    },
    /* Vector 194: Angle -2.546 rad */
    {
        .input_axis = {-15930, -56411, 29309},
        .input_angle_rad = -166881,
        .expected_quaternion = {19216, 15230, 53932, -28020},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-47188, 8634, -44651}
    },
    /* Vector 195: Angle -2.543 rad */
    {
        .input_axis = {-18729, -55544, 29309},
        .input_angle_rad = -166680,
        .expected_quaternion = {19313, 17898, 53078, -28007},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-44378, 12484, -46580}
    },
    /* Vector 196: Angle -2.540 rad */
    {
        .input_axis = {-21482, -54539, 29309},
        .input_angle_rad = -166479,
        .expected_quaternion = {19409, 20518, 52092, -27994},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-41192, 16038, -48383}
    },
    /* Vector 197: Angle -2.537 rad */
    {
        .input_axis = {-24181, -53397, 29309},
        .input_angle_rad = -166278,
        .expected_quaternion = {19505, 23085, 50978, -27980},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-37663, 19259, -50056}
    },
    /* Vector 198: Angle -2.534 rad */
    {
        .input_axis = {-26819, -52122, 29309},
        .input_angle_rad = -166077,
        .expected_quaternion = {19600, 25592, 49736, -27967},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-33824, 22115, -51593}
    },
    /* Vector 199: Angle -2.531 rad */
    {
        .input_axis = {-29391, -50716, 29309},
        .input_angle_rad = -165876,
        .expected_quaternion = {19696, 28032, 48372, -27954},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-29716, 24578, -52989}
    },
    /* Vector 200: Angle -2.528 rad */
    {
        .input_axis = {-31889, -49184, 29309},
        .input_angle_rad = -165675,
        .expected_quaternion = {19792, 30400, 46887, -27940},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-25378, 26623, -54241}
    },
    /* Vector 201: Angle -2.525 rad */
    {
        .input_axis = {-34307, -47529, 29309},
        .input_angle_rad = -165474,
        .expected_quaternion = {19888, 32689, 45287, -27926},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-20854, 28229, -55346}
    },
    /* Vector 202: Angle -2.522 rad */
    {
        .input_axis = {-36640, -45755, 29309},
        .input_angle_rad = -165273,
        .expected_quaternion = {19984, 34895, 43576, -27913},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16189, 29381, -56300}
    },
    /* Vector 203: Angle -2.519 rad */
    {
        .input_axis = {-38881, -43866, 29309},
        .input_angle_rad = -165072,
        .expected_quaternion = {20080, 37011, 41757, -27899},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11428, 30067, -57099}
    },
    /* Vector 204: Angle -2.516 rad */
    {
        .input_axis = {-41025, -41868, 29309},
        .input_angle_rad = -164871,
        .expected_quaternion = {20175, 39032, 39835, -27885},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6620, 30281, -57742}
    },
    /* Vector 205: Angle -2.513 rad */
    {
        .input_axis = {-43066, -39766, 29309},
        .input_angle_rad = -164670,
        .expected_quaternion = {20271, 40954, 37815, -27871},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1811, 30021, -58227}
    },
    /* Vector 206: Angle -2.510 rad */
    {
        .input_axis = {-45000, -37563, 29309},
        .input_angle_rad = -164469,
        .expected_quaternion = {20366, 42771, 35704, -27857},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2951, 29289, -58553}
    },
    /* Vector 207: Angle -2.507 rad */
    {
        .input_axis = {-46821, -35267, 29309},
        .input_angle_rad = -164268,
        .expected_quaternion = {20462, 44480, 33504, -27843},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {7620, 28093, -58717}
    },
    /* Vector 208: Angle -2.503 rad */
    {
        .input_axis = {-48525, -32883, 29309},
        .input_angle_rad = -164067,
        .expected_quaternion = {20557, 46076, 31224, -27829},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12149, 26445, -58720}
    },
    /* Vector 209: Angle -2.500 rad */
    {
        .input_axis = {-50108, -30417, 29309},
        .input_angle_rad = -163865,
        .expected_quaternion = {20653, 47554, 28867, -27815},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16494, 24362, -58561}
    },
    /* Vector 210: Angle -2.497 rad */
    {
        .input_axis = {-51565, -27875, 29309},
        .input_angle_rad = -163664,
        .expected_quaternion = {20748, 48913, 26441, -27801},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20614, 21865, -58241}
    },
    /* Vector 211: Angle -2.494 rad */
    {
        .input_axis = {-52894, -25263, 29309},
        .input_angle_rad = -163463,
        .expected_quaternion = {20844, 50147, 23951, -27787},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24467, 18979, -57759}
    },
    /* Vector 212: Angle -2.491 rad */
    {
        .input_axis = {-54090, -22587, 29309},
        .input_angle_rad = -163262,
        .expected_quaternion = {20939, 51255, 21404, -27772},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28017, 15733, -57118}
    },
    /* Vector 213: Angle -2.488 rad */
    {
        .input_axis = {-55152, -19856, 29309},
        .input_angle_rad = -163061,
        .expected_quaternion = {21034, 52234, 18805, -27758},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31230, 12159, -56319}
    },
    /* Vector 214: Angle -2.485 rad */
    {
        .input_axis = {-56075, -17075, 29309},
        .input_angle_rad = -162860,
        .expected_quaternion = {21129, 53081, 16163, -27744},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34074, 8293, -55364}
    },
    /* Vector 215: Angle -2.482 rad */
    {
        .input_axis = {-56859, -14251, 29309},
        .input_angle_rad = -162659,
        .expected_quaternion = {21224, 53794, 13483, -27729},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36524, 4173, -54255}
    },
    /* Vector 216: Angle -2.479 rad */
    {
        .input_axis = {-57500, -11391, 29309},
        .input_angle_rad = -162458,
        .expected_quaternion = {21320, 54372, 10771, -27714},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38555, -158, -52995}
    },
    /* Vector 217: Angle -2.476 rad */
    {
        .input_axis = {-57997, -8503, 29309},
        .input_angle_rad = -162257,
        .expected_quaternion = {21415, 54814, 8036, -27700},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40150, -4659, -51587}
    },
    /* Vector 218: Angle -2.473 rad */
    {
        .input_axis = {-58350, -5594, 29309},
        .input_angle_rad = -162056,
        .expected_quaternion = {21510, 55117, 5284, -27685},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41293, -9285, -50036}
    },
    /* Vector 219: Angle -2.470 rad */
    {
        .input_axis = {-58556, -2671, 29309},
        .input_angle_rad = -161855,
        .expected_quaternion = {21604, 55283, 2521, -27670},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41976, -13990, -48345}
    },
    /* Vector 220: Angle -2.467 rad */
    {
        .input_axis = {-58617, 259, 29309},
        .input_angle_rad = -161654,
        .expected_quaternion = {21699, 55310, -245, -27655},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42194, -18727, -46518}
    },
    /* Vector 221: Angle -2.464 rad */
    {
        .input_axis = {-58530, 3189, 29309},
        .input_angle_rad = -161453,
        .expected_quaternion = {21794, 55199, -3007, -27640},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41945, -23450, -44561}
    },
    /* Vector 222: Angle -2.461 rad */
    {
        .input_axis = {-58298, 6110, 29309},
        .input_angle_rad = -161252,
        .expected_quaternion = {21889, 54950, -5759, -27625},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41234, -28112, -42479}
    },
    /* Vector 223: Angle -2.457 rad */
    {
        .input_axis = {-57920, 9016, 29309},
        .input_angle_rad = -161051,
        .expected_quaternion = {21984, 54564, -8494, -27610},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40070, -32667, -40277}
    },
    /* Vector 224: Angle -2.454 rad */
    {
        .input_axis = {-57397, 11900, 29309},
        .input_angle_rad = -160850,
        .expected_quaternion = {22078, 54041, -11204, -27595},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38466, -37071, -37962}
    },
    /* Vector 225: Angle -2.451 rad */
    {
        .input_axis = {-56730, 14753, 29309},
        .input_angle_rad = -160648,
        .expected_quaternion = {22173, 53385, -13883, -27580},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36440, -41281, -35538}
    },
    /* Vector 226: Angle -2.448 rad */
    {
        .input_axis = {-55922, 17570, 29309},
        .input_angle_rad = -160447,
        .expected_quaternion = {22268, 52595, -16525, -27565},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34014, -45255, -33014}
    },
    /* Vector 227: Angle -2.445 rad */
    {
        .input_axis = {-54974, 20343, 29309},
        .input_angle_rad = -160246,
        .expected_quaternion = {22362, 51675, -19122, -27550},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31215, -48956, -30395}
    },
    /* Vector 228: Angle -2.442 rad */
    {
        .input_axis = {-53888, 23065, 29309},
        .input_angle_rad = -160045,
        .expected_quaternion = {22457, 50626, -21669, -27534},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28070, -52348, -27690}
    },
    /* Vector 229: Angle -2.439 rad */
    {
        .input_axis = {-52668, 25730, 29309},
        .input_angle_rad = -159844,
        .expected_quaternion = {22551, 49452, -24159, -27519},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24614, -55397, -24904}
    },
    /* Vector 230: Angle -2.436 rad */
    {
        .input_axis = {-51317, 28330, 29309},
        .input_angle_rad = -159643,
        .expected_quaternion = {22645, 48156, -26585, -27503},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20883, -58076, -22046}
    },
    /* Vector 231: Angle -2.433 rad */
    {
        .input_axis = {-49836, 30859, 29309},
        .input_angle_rad = -159442,
        .expected_quaternion = {22740, 46740, -28942, -27488},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16915, -60358, -19124}
    },
    /* Vector 232: Angle -2.430 rad */
    {
        .input_axis = {-48232, 33312, 29309},
        .input_angle_rad = -159241,
        .expected_quaternion = {22834, 45210, -31224, -27472},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12751, -62223, -16145}
    },
    /* Vector 233: Angle -2.427 rad */
    {
        .input_axis = {-46507, 35680, 29309},
        .input_angle_rad = -159040,
        .expected_quaternion = {22928, 43568, -33426, -27456},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8434, -63654, -13117}
    },
    /* Vector 234: Angle -2.424 rad */
    {
        .input_axis = {-44665, 37960, 29309},
        .input_angle_rad = -158839,
        .expected_quaternion = {23022, 41819, -35541, -27441},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {4008, -64637, -10049}
    },
    /* Vector 235: Angle -2.421 rad */
    {
        .input_axis = {-42712, 40145, 29309},
        .input_angle_rad = -158638,
        .expected_quaternion = {23116, 39967, -37565, -27425},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-481, -65165, -6950}
    },
    /* Vector 236: Angle -2.418 rad */
    {
        .input_axis = {-40652, 42230, 29309},
        .input_angle_rad = -158437,
        .expected_quaternion = {23210, 38018, -39493, -27409},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4987, -65234, -3826}
    },
    /* Vector 237: Angle -2.414 rad */
    {
        .input_axis = {-38491, 44209, 29309},
        .input_angle_rad = -158236,
        .expected_quaternion = {23304, 35975, -41319, -27393},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-9466, -64845, -688}
    },
    /* Vector 238: Angle -2.411 rad */
    {
        .input_axis = {-36233, 46077, 29309},
        .input_angle_rad = -158035,
        .expected_quaternion = {23398, 33845, -43040, -27377},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13870, -64004, 2456}
    },
    /* Vector 239: Angle -2.408 rad */
    {
        .input_axis = {-33885, 47831, 29309},
        .input_angle_rad = -157834,
        .expected_quaternion = {23492, 31633, -44652, -27361},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18156, -62722, 5599}
    },
    /* Vector 240: Angle -2.405 rad */
    {
        .input_axis = {-31452, 49464, 29309},
        .input_angle_rad = -157633,
        .expected_quaternion = {23586, 29345, -46150, -27345},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22280, -61011, 8730}
    },
    /* Vector 241: Angle -2.402 rad */
    {
        .input_axis = {-28941, 50974, 29309},
        .input_angle_rad = -157431,
        .expected_quaternion = {23680, 26986, -47531, -27328},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26200, -58892, 11842}
    },
    /* Vector 242: Angle -2.399 rad */
    {
        .input_axis = {-26357, 52357, 29309},
        .input_angle_rad = -157230,
        .expected_quaternion = {23774, 24562, -48791, -27312},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-29877, -56387, 14926}
    },
    /* Vector 243: Angle -2.396 rad */
    {
        .input_axis = {-23707, 53609, 29309},
        .input_angle_rad = -157029,
        .expected_quaternion = {23867, 22079, -49928, -27296},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-33275, -53523, 17974}
    },
    /* Vector 244: Angle -2.393 rad */
    {
        .input_axis = {-20998, 54727, 29309},
        .input_angle_rad = -156828,
        .expected_quaternion = {23961, 19545, -50938, -27279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-36358, -50330, 20976}
    },
    /* Vector 245: Angle -2.390 rad */
    {
        .input_axis = {-18237, 55708, 29309},
        .input_angle_rad = -156627,
        .expected_quaternion = {24054, 16964, -51820, -27263},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39096, -46841, 23926}
    },
    /* Vector 246: Angle -2.387 rad */
    {
        .input_axis = {-15430, 56550, 29309},
        .input_angle_rad = -156426,
        .expected_quaternion = {24148, 14344, -52571, -27246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-41461, -43092, 26814}
    },
    /* Vector 247: Angle -2.384 rad */
    {
        .input_axis = {-12584, 57250, 29309},
        .input_angle_rad = -156225,
        .expected_quaternion = {24241, 11692, -53190, -27230},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43431, -39123, 29633}
    },
    /* Vector 248: Angle -2.381 rad */
    {
        .input_axis = {-9707, 57808, 29309},
        .input_angle_rad = -156024,
        .expected_quaternion = {24335, 9013, -53675, -27213},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-44985, -34973, 32376}
    },
    /* Vector 249: Angle -2.378 rad */
    {
        .input_axis = {-6806, 58221, 29309},
        .input_angle_rad = -155823,
        .expected_quaternion = {24428, 6315, -54025, -27196},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-46108, -30687, 35033}
    },
    /* Vector 250: Angle -2.375 rad */
    {
        .input_axis = {-3888, 58488, 29309},
        .input_angle_rad = -155622,
        .expected_quaternion = {24521, 3605, -54240, -27180},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-46789, -26307, 37599}
    },
    /* Vector 251: Angle -2.372 rad */
    {
        .input_axis = {-960, 58609, 29309},
        .input_angle_rad = -155421,
        .expected_quaternion = {24614, 889, -54318, -27163},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-47022, -21878, 40065}
    },
    /* Vector 252: Angle -2.368 rad */
    {
        .input_axis = {1971, 58584, 29309},
        .input_angle_rad = -155220,
        .expected_quaternion = {24708, -1825, -54261, -27146},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-46804, -17446, 42426}
    },
    /* Vector 253: Angle -2.365 rad */
    {
        .input_axis = {4896, 58412, 29309},
        .input_angle_rad = -155019,
        .expected_quaternion = {24801, -4532, -54068, -27129},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-46139, -13054, 44674}
    },
    /* Vector 254: Angle -2.362 rad */
    {
        .input_axis = {7810, 58095, 29309},
        .input_angle_rad = -154818,
        .expected_quaternion = {24894, -7224, -53740, -27112},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-45032, -8749, 46804}
    },
    /* Vector 255: Angle -2.359 rad */
    {
        .input_axis = {10703, 57632, 29309},
        .input_angle_rad = -154617,
        .expected_quaternion = {24987, -9895, -53278, -27095},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43495, -4572, 48808}
    },
    /* Vector 256: Angle -2.356 rad */
    {
        .input_axis = {13570, 57025, 29309},
        .input_angle_rad = -154416,
        .expected_quaternion = {25080, -12537, -52684, -27078},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-41544, -567, 50683}
    },
    /* Vector 257: Angle -2.353 rad */
    {
        .input_axis = {16404, 56275, 29309},
        .input_angle_rad = -154215,
        .expected_quaternion = {25172, -15145, -51958, -27060},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39198, 3227, 52422}
    },
    /* Vector 258: Angle -2.350 rad */
    {
        .input_axis = {19196, 55385, 29309},
        .input_angle_rad = -154013,
        .expected_quaternion = {25265, -17712, -51104, -27043},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-36482, 6772, 54020}
    },
    /* Vector 259: Angle -2.347 rad */
    {
        .input_axis = {21940, 54356, 29309},
        .input_angle_rad = -153812,
        .expected_quaternion = {25358, -20231, -50123, -27026},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-33422, 10031, 55473}
    },
    /* Vector 260: Angle -2.344 rad */
    {
        .input_axis = {24629, 53192, 29309},
        .input_angle_rad = -153611,
        .expected_quaternion = {25451, -22696, -49017, -27008},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30049, 12974, 56778}
    },
    /* Vector 261: Angle -2.341 rad */
    {
        .input_axis = {27257, 51895, 29309},
        .input_angle_rad = -153410,
        .expected_quaternion = {25543, -25101, -47791, -26991},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26396, 15569, 57929}
    },
    /* Vector 262: Angle -2.338 rad */
    {
        .input_axis = {29816, 50467, 29309},
        .input_angle_rad = -153209,
        .expected_quaternion = {25636, -27440, -46446, -26973},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22501, 17793, 58925}
    },
    /* Vector 263: Angle -2.335 rad */
    {
        .input_axis = {32301, 48914, 29309},
        .input_angle_rad = -153008,
        .expected_quaternion = {25728, -29708, -44987, -26956},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18401, 19622, 59761}
    },
    /* Vector 264: Angle -2.332 rad */
    {
        .input_axis = {34706, 47239, 29309},
        .input_angle_rad = -152807,
        .expected_quaternion = {25821, -31898, -43418, -26938},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14138, 21039, 60436}
    },
    /* Vector 265: Angle -2.329 rad */
    {
        .input_axis = {37023, 45445, 29309},
        .input_angle_rad = -152606,
        .expected_quaternion = {25913, -34006, -41742, -26920},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-9753, 22030, 60947}
    },
    /* Vector 266: Angle -2.326 rad */
    {
        .input_axis = {39248, 43538, 29309},
        .input_angle_rad = -152405,
        .expected_quaternion = {26005, -36026, -39963, -26902},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-5290, 22587, 61293}
    },
    /* Vector 267: Angle -2.322 rad */
    {
        .input_axis = {41375, 41522, 29309},
        .input_angle_rad = -152204,
        .expected_quaternion = {26098, -37953, -38088, -26885},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-792, 22703, 61473}
    },
    /* Vector 268: Angle -2.319 rad */
    {
        .input_axis = {43399, 39402, 29309},
        .input_angle_rad = -152003,
        .expected_quaternion = {26190, -39783, -36119, -26867},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {3695, 22378, 61486}
    },
    /* Vector 269: Angle -2.316 rad */
    {
        .input_axis = {45314, 37184, 29309},
        .input_angle_rad = -151802,
        .expected_quaternion = {26282, -41510, -34063, -26849},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8129, 21616, 61332}
    },
    /* Vector 270: Angle -2.313 rad */
    {
        .input_axis = {47116, 34873, 29309},
        .input_angle_rad = -151601,
        .expected_quaternion = {26374, -43132, -31924, -26831},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12465, 20426, 61011}
    },
    /* Vector 271: Angle -2.310 rad */
    {
        .input_axis = {48800, 32474, 29309},
        .input_angle_rad = -151400,
        .expected_quaternion = {26466, -44643, -29708, -26812},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16662, 18819, 60524}
    },
    /* Vector 272: Angle -2.307 rad */
    {
        .input_axis = {50362, 29995, 29309},
        .input_angle_rad = -151199,
        .expected_quaternion = {26558, -46041, -27421, -26794},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20679, 16813, 59872}
    },
    /* Vector 273: Angle -2.304 rad */
    {
        .input_axis = {51798, 27440, 29309},
        .input_angle_rad = -150998,
        .expected_quaternion = {26650, -47322, -25069, -26776},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24478, 14427, 59057}
    },
    /* Vector 274: Angle -2.301 rad */
    {
        .input_axis = {53105, 24817, 29309},
        .input_angle_rad = -150796,
        .expected_quaternion = {26742, -48482, -22657, -26758},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28020, 11686, 58080}
    },
    /* Vector 275: Angle -2.298 rad */
    {
        .input_axis = {54278, 22132, 29309},
        .input_angle_rad = -150595,
        .expected_quaternion = {26833, -49520, -20192, -26739},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31274, 8618, 56944}
    },
    /* Vector 276: Angle -2.295 rad */
    {
        .input_axis = {55317, 19391, 29309},
        .input_angle_rad = -150394,
        .expected_quaternion = {26925, -50433, -17679, -26721},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34208, 5254, 55652}
    },
    /* Vector 277: Angle -2.292 rad */
    {
        .input_axis = {56217, 16603, 29309},
        .input_angle_rad = -150193,
        .expected_quaternion = {27017, -51218, -15126, -26702},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36794, 1627, 54208}
    },
    /* Vector 278: Angle -2.289 rad */
    {
        .input_axis = {56976, 13772, 29309},
        .input_angle_rad = -149992,
        .expected_quaternion = {27108, -51874, -12539, -26684},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39009, -2225, 52615}
    },
    /* Vector 279: Angle -2.286 rad */
    {
        .input_axis = {57593, 10907, 29309},
        .input_angle_rad = -149791,
        .expected_quaternion = {27200, -52399, -9923, -26665},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40832, -6265, 50877}
    },
    /* Vector 280: Angle -2.283 rad */
    {
        .input_axis = {58067, 8015, 29309},
        .input_angle_rad = -149590,
        .expected_quaternion = {27291, -52792, -7287, -26646},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42247, -10452, 48999}
    },
    /* Vector 281: Angle -2.279 rad */
    {
        .input_axis = {58395, 5103, 29309},
        .input_angle_rad = -149389,
        .expected_quaternion = {27382, -53053, -4636, -26628},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43242, -14745, 46986}
    },
    /* Vector 282: Angle -2.276 rad */
    {
        .input_axis = {58577, 2178, 29309},
        .input_angle_rad = -149188,
        .expected_quaternion = {27474, -53181, -1977, -26609},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43809, -19100, 44843}
    },
    /* Vector 283: Angle -2.273 rad */
    {
        .input_axis = {58612, -752, 29309},
        .input_angle_rad = -148987,
        .expected_quaternion = {27565, -53176, 682, -26590},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43945, -23475, 42576}
    },
    /* Vector 284: Angle -2.270 rad */
    {
        .input_axis = {58502, -3681, 29309},
        .input_angle_rad = -148786,
        .expected_quaternion = {27656, -53037, 3337, -26571},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43650, -27827, 40191}
    },
    /* Vector 285: Angle -2.267 rad */
    {
        .input_axis = {58244, -6600, 29309},
        .input_angle_rad = -148585,
        .expected_quaternion = {27747, -52766, 5979, -26552},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42930, -32112, 37694}
    },
    /* Vector 286: Angle -2.264 rad */
    {
        .input_axis = {57842, -9503, 29309},
        .input_angle_rad = -148384,
        .expected_quaternion = {27838, -52364, 8603, -26533},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41793, -36289, 35092}
    },
    /* Vector 287: Angle -2.261 rad */
    {
        .input_axis = {57295, -12382, 29309},
        .input_angle_rad = -148183,
        .expected_quaternion = {27929, -51831, 11201, -26514},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40254, -40316, 32392}
    },
    /* Vector 288: Angle -2.258 rad */
    {
        .input_axis = {56604, -15230, 29309},
        .input_angle_rad = -147982,
        .expected_quaternion = {28020, -51170, 13768, -26495},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38329, -44155, 29600}
    },
    /* Vector 289: Angle -2.255 rad */
    {
        .input_axis = {55772, -18040, 29309},
        .input_angle_rad = -147781,
        .expected_quaternion = {28111, -50381, 16296, -26475},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36041, -47768, 26726}
    },
    /* Vector 290: Angle -2.252 rad */
    {
        .input_axis = {54801, -20805, 29309},
        .input_angle_rad = -147579,
        .expected_quaternion = {28202, -49467, 18780, -26456},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33413, -51120, 23776}
    },
    /* Vector 291: Angle -2.249 rad */
    {
        .input_axis = {53693, -23518, 29309},
        .input_angle_rad = -147378,
        .expected_quaternion = {28293, -48431, 21213, -26437},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30474, -54179, 20758}
    },
    /* Vector 292: Angle -2.246 rad */
    {
        .input_axis = {52450, -26172, 29309},
        .input_angle_rad = -147177,
        .expected_quaternion = {28383, -47276, 23590, -26417},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27256, -56916, 17680}
    },
    /* Vector 293: Angle -2.243 rad */
    {
        .input_axis = {51077, -28760, 29309},
        .input_angle_rad = -146976,
        .expected_quaternion = {28474, -46004, 25904, -26398},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23792, -59306, 14551}
    },
    /* Vector 294: Angle -2.240 rad */
    {
        .input_axis = {49575, -31277, 29309},
        .input_angle_rad = -146775,
        .expected_quaternion = {28564, -44619, 28150, -26378},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20119, -61325, 11379}
    },
    /* Vector 295: Angle -2.237 rad */
    {
        .input_axis = {47950, -33716, 29309},
        .input_angle_rad = -146574,
        .expected_quaternion = {28655, -43124, 30322, -26359},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16274, -62955, 8173}
    },
    /* Vector 296: Angle -2.233 rad */
    {
        .input_axis = {46205, -36070, 29309},
        .input_angle_rad = -146373,
        .expected_quaternion = {28745, -41523, 32415, -26339},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12298, -64182, 4941}
    },
    /* Vector 297: Angle -2.230 rad */
    {
        .input_axis = {44345, -38334, 29309},
        .input_angle_rad = -146172,
        .expected_quaternion = {28835, -39821, 34424, -26319},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8232, -64995, 1691}
    },
    /* Vector 298: Angle -2.227 rad */
    {
        .input_axis = {42373, -40503, 29309},
        .input_angle_rad = -145971,
        .expected_quaternion = {28926, -38023, 36344, -26299},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {4118, -65388, -1566}
    },
    /* Vector 299: Angle -2.224 rad */
    {
        .input_axis = {40296, -42570, 29309},
        .input_angle_rad = -145770,
        .expected_quaternion = {29016, -36131, 38170, -26279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-3, -65358, -4823}
    },
    /* Vector 300: Angle -2.221 rad */
    {
        .input_axis = {38118, -44531, 29309},
        .input_angle_rad = -145569,
        .expected_quaternion = {29106, -34152, 39898, -26260},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4087, -64909, -8070}
    },
    /* Vector 301: Angle -2.218 rad */
    {
        .input_axis = {35845, -46380, 29309},
        .input_angle_rad = -145368,
        .expected_quaternion = {29196, -32091, 41523, -26240},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-8094, -64045, -11299}
    },
    /* Vector 302: Angle -2.215 rad */
    {
        .input_axis = {33482, -48114, 29309},
        .input_angle_rad = -145167,
        .expected_quaternion = {29286, -29953, 43043, -26219},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11982, -62778, -14502}
    },
    /* Vector 303: Angle -2.212 rad */
    {
        .input_axis = {31035, -49727, 29309},
        .input_angle_rad = -144966,
        .expected_quaternion = {29376, -27743, 44452, -26199},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-15713, -61122, -17668}
    },
    /* Vector 304: Angle -2.209 rad */
    {
        .input_axis = {28511, -51216, 29309},
        .input_angle_rad = -144765,
        .expected_quaternion = {29466, -25467, 45747, -26179},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19247, -59095, -20791}
    },
    /* Vector 305: Angle -2.206 rad */
    {
        .input_axis = {25916, -52577, 29309},
        .input_angle_rad = -144564,
        .expected_quaternion = {29555, -23131, 46927, -26159},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22550, -56720, -23861}
    },
    /* Vector 306: Angle -2.203 rad */
    {
        .input_axis = {23256, -53807, 29309},
        .input_angle_rad = -144362,
        .expected_quaternion = {29645, -20740, 47987, -26139},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-25588, -54021, -26869}
    },
    /* Vector 307: Angle -2.200 rad */
    {
        .input_axis = {20538, -54902, 29309},
        .input_angle_rad = -144161,
        .expected_quaternion = {29735, -18302, 48925, -26118},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-28331, -51027, -29809}
    },
    /* Vector 308: Angle -2.197 rad */
    {
        .input_axis = {17768, -55859, 29309},
        .input_angle_rad = -143960,
        .expected_quaternion = {29824, -15821, 49740, -26098},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30752, -47769, -32671}
    },
    /* Vector 309: Angle -2.194 rad */
    {
        .input_axis = {14954, -56678, 29309},
        .input_angle_rad = -143759,
        .expected_quaternion = {29914, -13305, 50429, -26077},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-32825, -44282, -35448}
    },
    /* Vector 310: Angle -2.191 rad */
    {
        .input_axis = {12103, -57354, 29309},
        .input_angle_rad = -143558,
        .expected_quaternion = {30003, -10760, 50991, -26057},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-34531, -40602, -38132}
    },
    /* Vector 311: Angle -2.187 rad */
    {
        .input_axis = {9221, -57887, 29309},
        .input_angle_rad = -143357,
        .expected_quaternion = {30093, -8191, 51424, -26036},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-35853, -36765, -40717}
    },
    /* Vector 312: Angle -2.184 rad */
    {
        .input_axis = {6316, -58276, 29309},
        .input_angle_rad = -143156,
        .expected_quaternion = {30182, -5607, 51728, -26015},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-36777, -32813, -43194}
    },
    /* Vector 313: Angle -2.181 rad */
    {
        .input_axis = {3396, -58519, 29309},
        .input_angle_rad = -142955,
        .expected_quaternion = {30271, -3012, 51902, -25995},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-37295, -28784, -45558}
    },
    /* Vector 314: Angle -2.178 rad */
    {
        .input_axis = {467, -58615, 29309},
        .input_angle_rad = -142754,
        .expected_quaternion = {30360, -414, 51946, -25974},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-37401, -24721, -47801}
    },
    /* Vector 315: Angle -2.175 rad */
    {
        .input_axis = {-2463, -58565, 29309},
        .input_angle_rad = -142553,
        .expected_quaternion = {30449, 2181, 51860, -25953},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-37096, -20664, -49918}
    },
    /* Vector 316: Angle -2.172 rad */
    {
        .input_axis = {-5387, -58369, 29309},
        .input_angle_rad = -142352,
        .expected_quaternion = {30538, 4767, 51645, -25932},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-36382, -16655, -51903}
    },
    /* Vector 317: Angle -2.169 rad */
    {
        .input_axis = {-8298, -58027, 29309},
        .input_angle_rad = -142151,
        .expected_quaternion = {30627, 7336, 51300, -25911},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-35267, -12733, -53750}
    },
    /* Vector 318: Angle -2.166 rad */
    {
        .input_axis = {-11188, -57540, 29309},
        .input_angle_rad = -141950,
        .expected_quaternion = {30716, 9883, 50828, -25890},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-33763, -8939, -55454}
    },
    /* Vector 319: Angle -2.163 rad */
    {
        .input_axis = {-14049, -56909, 29309},
        .input_angle_rad = -141749,
        .expected_quaternion = {30805, 12401, 50230, -25869},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-31884, -5310, -57010}
    },
    /* Vector 320: Angle -2.160 rad */
    {
        .input_axis = {-16876, -56135, 29309},
        .input_angle_rad = -141548,
        .expected_quaternion = {30893, 14883, 49507, -25848},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-29650, -1883, -58415}
    },
    /* Vector 321: Angle -2.157 rad */
    {
        .input_axis = {-19661, -55222, 29309},
        .input_angle_rad = -141347,
        .expected_quaternion = {30982, 17325, 48661, -25827},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-27083, 1309, -59664}
    },
    /* Vector 322: Angle -2.154 rad */
    {
        .input_axis = {-22396, -54170, 29309},
        .input_angle_rad = -141145,
        .expected_quaternion = {31071, 19719, 47695, -25805},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-24208, 4233, -60754}
    },
    /* Vector 323: Angle -2.151 rad */
    {
        .input_axis = {-25075, -52983, 29309},
        .input_angle_rad = -140944,
        .expected_quaternion = {31159, 22060, 46611, -25784},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-21056, 6861, -61681}
    },
    /* Vector 324: Angle -2.148 rad */
    {
        .input_axis = {-27692, -51664, 29309},
        .input_angle_rad = -140743,
        .expected_quaternion = {31248, 24342, 45413, -25763},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-17656, 9168, -62443}
    },
    /* Vector 325: Angle -2.145 rad */
    {
        .input_axis = {-30240, -50215, 29309},
        .input_angle_rad = -140542,
        .expected_quaternion = {31336, 26559, 44103, -25741},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14044, 11130, -63039}
    },
    /* Vector 326: Angle -2.141 rad */
    {
        .input_axis = {-32711, -48641, 29309},
        .input_angle_rad = -140341,
        .expected_quaternion = {31424, 28706, 42685, -25720},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-10254, 12728, -63465}
    },
    /* Vector 327: Angle -2.138 rad */
    {
        .input_axis = {-35102, -46945, 29309},
        .input_angle_rad = -140140,
        .expected_quaternion = {31512, 30777, 41162, -25698},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6324, 13948, -63721}
    },
    /* Vector 328: Angle -2.135 rad */
    {
        .input_axis = {-37404, -45132, 29309},
        .input_angle_rad = -139939,
        .expected_quaternion = {31600, 32769, 39539, -25676},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-2293, 14778, -63807}
    },
    /* Vector 329: Angle -2.132 rad */
    {
        .input_axis = {-39613, -43206, 29309},
        .input_angle_rad = -139738,
        .expected_quaternion = {31688, 34674, 37820, -25655},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {1800, 15211, -63721}
    },
    /* Vector 330: Angle -2.129 rad */
    {
        .input_axis = {-41723, -41173, 29309},
        .input_angle_rad = -139537,
        .expected_quaternion = {31776, 36490, 36009, -25633},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5914, 15242, -63464}
    },
    /* Vector 331: Angle -2.126 rad */
    {
        .input_axis = {-43728, -39036, 29309},
        .input_angle_rad = -139336,
        .expected_quaternion = {31864, 38212, 34111, -25611},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10009, 14873, -63036}
    },
    /* Vector 332: Angle -2.123 rad */
    {
        .input_axis = {-45625, -36802, 29309},
        .input_angle_rad = -139135,
        .expected_quaternion = {31952, 39835, 32131, -25589},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {14046, 14109, -62439}
    },
    /* Vector 333: Angle -2.120 rad */
    {
        .input_axis = {-47407, -34475, 29309},
        .input_angle_rad = -138934,
        .expected_quaternion = {32040, 41355, 30074, -25567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17985, 12957, -61674}
    },
    /* Vector 334: Angle -2.117 rad */
    {
        .input_axis = {-49071, -32063, 29309},
        .input_angle_rad = -138733,
        .expected_quaternion = {32127, 42770, 27946, -25545},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21788, 11430, -60742}
    },
    /* Vector 335: Angle -2.114 rad */
    {
        .input_axis = {-50612, -29570, 29309},
        .input_angle_rad = -138532,
        .expected_quaternion = {32215, 44075, 25751, -25523},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25419, 9544, -59647}
    },
    /* Vector 336: Angle -2.111 rad */
    {
        .input_axis = {-52027, -27004, 29309},
        .input_angle_rad = -138331,
        .expected_quaternion = {32303, 45268, 23496, -25501},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28843, 7319, -58391}
    },
    /* Vector 337: Angle -2.108 rad */
    {
        .input_axis = {-53311, -24370, 29309},
        .input_angle_rad = -138130,
        .expected_quaternion = {32390, 46345, 21185, -25479},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32028, 4778, -56977}
    },
    /* Vector 338: Angle -2.105 rad */
    {
        .input_axis = {-54463, -21675, 29309},
        .input_angle_rad = -137928,
        .expected_quaternion = {32477, 47305, 18826, -25457},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34943, 1947, -55409}
    },
    /* Vector 339: Angle -2.102 rad */
    {
        .input_axis = {-55478, -18926, 29309},
        .input_angle_rad = -137727,
        .expected_quaternion = {32565, 48144, 16424, -25434},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37562, -1146, -53691}
    },
    /* Vector 340: Angle -2.098 rad */
    {
        .input_axis = {-56354, -16129, 29309},
        .input_angle_rad = -137526,
        .expected_quaternion = {32652, 48862, 13985, -25412},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39860, -4468, -51828}
    },
    /* Vector 341: Angle -2.095 rad */
    {
        .input_axis = {-57090, -13293, 29309},
        .input_angle_rad = -137325,
        .expected_quaternion = {32739, 49456, 11515, -25389},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41817, -7987, -49825}
    },
    /* Vector 342: Angle -2.092 rad */
    {
        .input_axis = {-57683, -10423, 29309},
        .input_angle_rad = -137124,
        .expected_quaternion = {32826, 49926, 9021, -25367},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43415, -11668, -47686}
    },
    /* Vector 343: Angle -2.089 rad */
    {
        .input_axis = {-58132, -7527, 29309},
        .input_angle_rad = -136923,
        .expected_quaternion = {32913, 50269, 6509, -25344},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44641, -15472, -45418}
    },
    /* Vector 344: Angle -2.086 rad */
    {
        .input_axis = {-58435, -4612, 29309},
        .input_angle_rad = -136722,
        .expected_quaternion = {33000, 50487, 3985, -25322},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45484, -19362, -43027}
    },
    /* Vector 345: Angle -2.083 rad */
    {
        .input_axis = {-58593, -1686, 29309},
        .input_angle_rad = -136521,
        .expected_quaternion = {33087, 50577, 1455, -25299},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45939, -23299, -40518}
    },
    /* Vector 346: Angle -2.080 rad */
    {
        .input_axis = {-58604, 1245, 29309},
        .input_angle_rad = -136320,
        .expected_quaternion = {33173, 50541, -1074, -25276},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46003, -27245, -37899}
    },
    /* Vector 347: Angle -2.077 rad */
    {
        .input_axis = {-58468, 4172, 29309},
        .input_angle_rad = -136119,
        .expected_quaternion = {33260, 50379, -3595, -25254},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45679, -31160, -35177}
    },
    /* Vector 348: Angle -2.074 rad */
    {
        .input_axis = {-58187, 7089, 29309},
        .input_angle_rad = -135918,
        .expected_quaternion = {33347, 50091, -6103, -25231},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44972, -35006, -32359}
    },
    /* Vector 349: Angle -2.071 rad */
    {
        .input_axis = {-57760, 9989, 29309},
        .input_angle_rad = -135717,
        .expected_quaternion = {33433, 49678, -8591, -25208},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43892, -38744, -29451}
    },
    /* Vector 350: Angle -2.068 rad */
    {
        .input_axis = {-57188, 12863, 29309},
        .input_angle_rad = -135516,
        .expected_quaternion = {33520, 49142, -11053, -25185},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42451, -42339, -26463}
    },
    /* Vector 351: Angle -2.065 rad */
    {
        .input_axis = {-56474, 15705, 29309},
        .input_angle_rad = -135315,
        .expected_quaternion = {33606, 48484, -13483, -25162},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40667, -45755, -23402}
    },
    /* Vector 352: Angle -2.062 rad */
    {
        .input_axis = {-55619, 18508, 29309},
        .input_angle_rad = -135114,
        .expected_quaternion = {33692, 47706, -15875, -25139},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38559, -48959, -20276}
    },
    /* Vector 353: Angle -2.059 rad */
    {
        .input_axis = {-54624, 21265, 29309},
        .input_angle_rad = -134913,
        .expected_quaternion = {33778, 46809, -18223, -25116},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36152, -51921, -17094}
    },
    /* Vector 354: Angle -2.056 rad */
    {
        .input_axis = {-53493, 23968, 29309},
        .input_angle_rad = -134711,
        .expected_quaternion = {33865, 45798, -20520, -25092},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33471, -54612, -13863}
    },
    /* Vector 355: Angle -2.052 rad */
    {
        .input_axis = {-52228, 26612, 29309},
        .input_angle_rad = -134510,
        .expected_quaternion = {33951, 44674, -22762, -25069},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30545, -57007, -10594}
    },
    /* Vector 356: Angle -2.049 rad */
    {
        .input_axis = {-50833, 29189, 29309},
        .input_angle_rad = -134309,
        .expected_quaternion = {34037, 43440, -24944, -25046},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27405, -59082, -7294}
    },
    /* Vector 357: Angle -2.046 rad */
    {
        .input_axis = {-49311, 31693, 29309},
        .input_angle_rad = -134108,
        .expected_quaternion = {34122, 42099, -27058, -25023},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24085, -60820, -3972}
    },
    /* Vector 358: Angle -2.043 rad */
    {
        .input_axis = {-47665, 34118, 29309},
        .input_angle_rad = -133907,
        .expected_quaternion = {34208, 40656, -29101, -24999},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20619, -62205, -637}
    },
    /* Vector 359: Angle -2.040 rad */
    {
        .input_axis = {-45900, 36457, 29309},
        .input_angle_rad = -133706,
        .expected_quaternion = {34294, 39114, -31068, -24976},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17045, -63223, 2702}
    },
    /* Vector 360: Angle -2.037 rad */
    {
        .input_axis = {-44021, 38706, 29309},
        .input_angle_rad = -133505,
        .expected_quaternion = {34380, 37477, -32952, -24952},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13398, -63867, 6035}
    },
    /* Vector 361: Angle -2.034 rad */
    {
        .input_axis = {-42031, 40858, 29309},
        .input_angle_rad = -133304,
        .expected_quaternion = {34465, 35750, -34751, -24928},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {9717, -64133, 9355}
    },
    /* Vector 362: Angle -2.031 rad */
    {
        .input_axis = {-39937, 42907, 29309},
        .input_angle_rad = -133103,
        .expected_quaternion = {34551, 33936, -36460, -24905},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {6039, -64019, 12651}
    },
    /* Vector 363: Angle -2.028 rad */
    {
        .input_axis = {-37742, 44850, 29309},
        .input_angle_rad = -132902,
        .expected_quaternion = {34636, 32041, -38074, -24881},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2404, -63529, 15916}
    },
    /* Vector 364: Angle -2.025 rad */
    {
        .input_axis = {-35454, 46680, 29309},
        .input_angle_rad = -132701,
        .expected_quaternion = {34721, 30069, -39590, -24857},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1153, -62668, 19140}
    },
    /* Vector 365: Angle -2.022 rad */
    {
        .input_axis = {-33076, 48394, 29309},
        .input_angle_rad = -132500,
        .expected_quaternion = {34806, 28026, -41004, -24833},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4595, -61448, 22316}
    },
    /* Vector 366: Angle -2.019 rad */
    {
        .input_axis = {-30616, 49986, 29309},
        .input_angle_rad = -132299,
        .expected_quaternion = {34892, 25916, -42313, -24809},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-7886, -59883, 25433}
    },
    /* Vector 367: Angle -2.016 rad */
    {
        .input_axis = {-28080, 51454, 29309},
        .input_angle_rad = -132098,
        .expected_quaternion = {34977, 23746, -43513, -24785},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-10993, -57989, 28485}
    },
    /* Vector 368: Angle -2.013 rad */
    {
        .input_axis = {-25473, 52793, 29309},
        .input_angle_rad = -131897,
        .expected_quaternion = {35062, 21521, -44602, -24761},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13886, -55788, 31462}
    },
    /* Vector 369: Angle -2.010 rad */
    {
        .input_axis = {-22803, 54000, 29309},
        .input_angle_rad = -131696,
        .expected_quaternion = {35146, 19246, -45578, -24737},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16534, -53303, 34357}
    },
    /* Vector 370: Angle -2.006 rad */
    {
        .input_axis = {-20075, 55072, 29309},
        .input_angle_rad = -131495,
        .expected_quaternion = {35231, 16928, -46437, -24713},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18912, -50560, 37162}
    },
    /* Vector 371: Angle -2.003 rad */
    {
        .input_axis = {-17298, 56007, 29309},
        .input_angle_rad = -131293,
        .expected_quaternion = {35316, 14571, -47179, -24689},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-20994, -47588, 39869}
    },
    /* Vector 372: Angle -2.000 rad */
    {
        .input_axis = {-14477, 56801, 29309},
        .input_angle_rad = -131092,
        .expected_quaternion = {35401, 12183, -47801, -24665},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22761, -44419, 42472}
    },
    /* Vector 373: Angle -1.997 rad */
    {
        .input_axis = {-11620, 57454, 29309},
        .input_angle_rad = -130891,
        .expected_quaternion = {35485, 9769, -48303, -24640},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-24196, -41084, 44962}
    },
    /* Vector 374: Angle -1.994 rad */
    {
        .input_axis = {-8734, 57963, 29309},
        .input_angle_rad = -130690,
        .expected_quaternion = {35570, 7336, -48683, -24616},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-25283, -37619, 47334}
    },
    /* Vector 375: Angle -1.991 rad */
    {
        .input_axis = {-5826, 58327, 29309},
        .input_angle_rad = -130489,
        .expected_quaternion = {35654, 4888, -48940, -24592},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26012, -34059, 49582}
    },
    /* Vector 376: Angle -1.988 rad */
    {
        .input_axis = {-2904, 58545, 29309},
        .input_angle_rad = -130288,
        .expected_quaternion = {35738, 2434, -49074, -24567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26377, -30439, 51698}
    },
    /* Vector 377: Angle -1.985 rad */
    {
        .input_axis = {26, 58617, 29309},
        .input_angle_rad = -130087,
        .expected_quaternion = {35823, -22, -49085, -24543},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26374, -26798, 53677}
    },
    /* Vector 378: Angle -1.982 rad */
    {
        .input_axis = {2956, 58543, 29309},
        .input_angle_rad = -129886,
        .expected_quaternion = {35907, -2473, -48974, -24518},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26003, -23171, 55515}
    },
    /* Vector 379: Angle -1.979 rad */
    {
        .input_axis = {5878, 58322, 29309},
        .input_angle_rad = -129685,
        .expected_quaternion = {35991, -4912, -48740, -24493},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-25269, -19596, 57205}
    },
    /* Vector 380: Angle -1.976 rad */
    {
        .input_axis = {8785, 57955, 29309},
        .input_angle_rad = -129484,
        .expected_quaternion = {36075, -7335, -48385, -24469},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-24179, -16108, 58744}
    },
    /* Vector 381: Angle -1.973 rad */
    {
        .input_axis = {11671, 57444, 29309},
        .input_angle_rad = -129283,
        .expected_quaternion = {36159, -9734, -47909, -24444},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22744, -12742, 60127}
    },
    /* Vector 382: Angle -1.970 rad */
    {
        .input_axis = {14527, 56788, 29309},
        .input_angle_rad = -129082,
        .expected_quaternion = {36243, -12104, -47314, -24419},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-20980, -9531, 61351}
    },
    /* Vector 383: Angle -1.967 rad */
    {
        .input_axis = {17347, 55991, 29309},
        .input_angle_rad = -128881,
        .expected_quaternion = {36326, -14439, -46603, -24394},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18903, -6508, 62412}
    },
    /* Vector 384: Angle -1.963 rad */
    {
        .input_axis = {20124, 55054, 29309},
        .input_angle_rad = -128680,
        .expected_quaternion = {36410, -16733, -45776, -24369},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16535, -3703, 63308}
    },
    /* Vector 385: Angle -1.960 rad */
    {
        .input_axis = {22851, 53980, 29309},
        .input_angle_rad = -128479,
        .expected_quaternion = {36493, -18980, -44837, -24344},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13900, -1141, 64035}
    },
    /* Vector 386: Angle -1.957 rad */
    {
        .input_axis = {25520, 52770, 29309},
        .input_angle_rad = -128278,
        .expected_quaternion = {36577, -21175, -43787, -24319},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11024, 1150, 64592}
    },
    /* Vector 387: Angle -1.954 rad */
    {
        .input_axis = {28125, 51429, 29309},
        .input_angle_rad = -128076,
        .expected_quaternion = {36660, -23313, -42630, -24294},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-7935, 3150, 64978}
    },
    /* Vector 388: Angle -1.951 rad */
    {
        .input_axis = {30661, 49959, 29309},
        .input_angle_rad = -127875,
        .expected_quaternion = {36744, -25388, -41368, -24269},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4664, 4839, 65191}
    },
    /* Vector 389: Angle -1.948 rad */
    {
        .input_axis = {33119, 48364, 29309},
        .input_angle_rad = -127674,
        .expected_quaternion = {36827, -27396, -40006, -24244},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1244, 6201, 65230}
    },
    /* Vector 390: Angle -1.945 rad */
    {
        .input_axis = {35495, 46648, 29309},
        .input_angle_rad = -127473,
        .expected_quaternion = {36910, -29330, -38547, -24218},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2292, 7223, 65096}
    },
    /* Vector 391: Angle -1.942 rad */
    {
        .input_axis = {37782, 44816, 29309},
        .input_angle_rad = -127272,
        .expected_quaternion = {36993, -31187, -36994, -24193},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5910, 7897, 64789}
    },
    /* Vector 392: Angle -1.939 rad */
    {
        .input_axis = {39975, 42872, 29309},
        .input_angle_rad = -127071,
        .expected_quaternion = {37076, -32963, -35352, -24168},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {9573, 8217, 64310}
    },
    /* Vector 393: Angle -1.936 rad */
    {
        .input_axis = {42068, 40820, 29309},
        .input_angle_rad = -126870,
        .expected_quaternion = {37159, -34652, -33625, -24142},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13246, 8181, 63660}
    },
    /* Vector 394: Angle -1.933 rad */
    {
        .input_axis = {44055, 38667, 29309},
        .input_angle_rad = -126669,
        .expected_quaternion = {37241, -36251, -31817, -24117},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16893, 7790, 62840}
    },
    /* Vector 395: Angle -1.930 rad */
    {
        .input_axis = {45933, 36417, 29309},
        .input_angle_rad = -126468,
        .expected_quaternion = {37324, -37756, -29934, -24091},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20480, 7049, 61853}
    },
    /* Vector 396: Angle -1.927 rad */
    {
        .input_axis = {47695, 34075, 29309},
        .input_angle_rad = -126267,
        .expected_quaternion = {37407, -39163, -27979, -24065},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23971, 5968, 60702}
    },
    /* Vector 397: Angle -1.924 rad */
    {
        .input_axis = {49339, 31649, 29309},
        .input_angle_rad = -126066,
        .expected_quaternion = {37489, -40469, -25959, -24040},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27334, 4557, 59389}
    },
    /* Vector 398: Angle -1.921 rad */
    {
        .input_axis = {50859, 29144, 29309},
        .input_angle_rad = -125865,
        .expected_quaternion = {37572, -41671, -23879, -24014},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30537, 2832, 57918}
    },
    /* Vector 399: Angle -1.917 rad */
    {
        .input_axis = {52252, 26565, 29309},
        .input_angle_rad = -125664,
        .expected_quaternion = {37654, -42766, -21743, -23988},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33548, 812, 56292}
    },
    /* Vector 400: Angle -1.914 rad */
    {
        .input_axis = {53514, 23921, 29309},
        .input_angle_rad = -125463,
        .expected_quaternion = {37736, -43752, -19557, -23962},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36341, -1482, 54517}
    },
    /* Vector 401: Angle -1.911 rad */
    {
        .input_axis = {54643, 21216, 29309},
        .input_angle_rad = -125262,
        .expected_quaternion = {37818, -44627, -17327, -23936},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38889, -4027, 52597}
    },
    /* Vector 402: Angle -1.908 rad */
    {
        .input_axis = {55635, 18459, 29309},
        .input_angle_rad = -125061,
        .expected_quaternion = {37900, -45388, -15059, -23910},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41169, -6797, 50536}
    },
    /* Vector 403: Angle -1.905 rad */
    {
        .input_axis = {56488, 15655, 29309},
        .input_angle_rad = -124859,
        .expected_quaternion = {37982, -46034, -12758, -23884},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43160, -9763, 48341}
    },
    /* Vector 404: Angle -1.902 rad */
    {
        .input_axis = {57200, 12812, 29309},
        .input_angle_rad = -124658,
        .expected_quaternion = {38064, -46563, -10430, -23858},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44845, -12894, 46017}
    },
    /* Vector 405: Angle -1.899 rad */
    {
        .input_axis = {57769, 9937, 29309},
        .input_angle_rad = -124457,
        .expected_quaternion = {38146, -46974, -8080, -23832},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46210, -16160, 43571}
    },
    /* Vector 406: Angle -1.896 rad */
    {
        .input_axis = {58193, 7038, 29309},
        .input_angle_rad = -124256,
        .expected_quaternion = {38228, -47267, -5716, -23806},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47244, -19527, 41009}
    },
    /* Vector 407: Angle -1.893 rad */
    {
        .input_axis = {58472, 4120, 29309},
        .input_angle_rad = -124055,
        .expected_quaternion = {38309, -47442, -3343, -23780},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47938, -22961, 38337}
    },
    /* Vector 408: Angle -1.890 rad */
    {
        .input_axis = {58605, 1193, 29309},
        .input_angle_rad = -123854,
        .expected_quaternion = {38391, -47497, -967, -23753},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48289, -26428, 35563}
    },
    /* Vector 409: Angle -1.887 rad */
    {
        .input_axis = {58591, -1738, 29309},
        .input_angle_rad = -123653,
        .expected_quaternion = {38472, -47433, 1407, -23727},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48295, -29894, 32694}
    },
    /* Vector 410: Angle -1.884 rad */
    {
        .input_axis = {58431, -4664, 29309},
        .input_angle_rad = -123452,
        .expected_quaternion = {38554, -47251, 3771, -23701},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47960, -33324, 29739}
    },
    /* Vector 411: Angle -1.881 rad */
    {
        .input_axis = {58125, -7578, 29309},
        .input_angle_rad = -123251,
        .expected_quaternion = {38635, -46951, 6121, -23674},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47288, -36684, 26703}
    },
    /* Vector 412: Angle -1.878 rad */
    {
        .input_axis = {57674, -10474, 29309},
        .input_angle_rad = -123050,
        .expected_quaternion = {38716, -46534, 8451, -23648},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46291, -39941, 23597}
    },
    /* Vector 413: Angle -1.875 rad */
    {
        .input_axis = {57078, -13343, 29309},
        .input_angle_rad = -122849,
        .expected_quaternion = {38797, -46002, 10754, -23621},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44979, -43064, 20428}
    },
    /* Vector 414: Angle -1.871 rad */
    {
        .input_axis = {56340, -16179, 29309},
        .input_angle_rad = -122648,
        .expected_quaternion = {38878, -45356, 13025, -23594},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43370, -46022, 17204}
    },
    /* Vector 415: Angle -1.868 rad */
    {
        .input_axis = {55461, -18975, 29309},
        .input_angle_rad = -122447,
        .expected_quaternion = {38959, -44597, 15258, -23568},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41481, -48787, 13935}
    },
    /* Vector 416: Angle -1.865 rad */
    {
        .input_axis = {54443, -21723, 29309},
        .input_angle_rad = -122246,
        .expected_quaternion = {39040, -43729, 17448, -23541},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39333, -51331, 10628}
    },
    /* Vector 417: Angle -1.862 rad */
    {
        .input_axis = {53290, -24417, 29309},
        .input_angle_rad = -122045,
        .expected_quaternion = {39120, -42754, 19590, -23514},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36951, -53632, 7293}
    },
    /* Vector 418: Angle -1.859 rad */
    {
        .input_axis = {52003, -27050, 29309},
        .input_angle_rad = -121844,
        .expected_quaternion = {39201, -41674, 21677, -23487},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34361, -55667, 3938}
    },
    /* Vector 419: Angle -1.856 rad */
    {
        .input_axis = {50586, -29615, 29309},
        .input_angle_rad = -121642,
        .expected_quaternion = {39282, -40492, 23706, -23460},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31590, -57417, 572}
    },
    /* Vector 420: Angle -1.853 rad */
    {
        .input_axis = {49042, -32106, 29309},
        .input_angle_rad = -121441,
        .expected_quaternion = {39362, -39211, 25670, -23433},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28668, -58867, -2795}
    },
    /* Vector 421: Angle -1.850 rad */
    {
        .input_axis = {47376, -34517, 29309},
        .input_angle_rad = -121240,
        .expected_quaternion = {39442, -37836, 27566, -23406},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25627, -60003, -6155}
    },
    /* Vector 422: Angle -1.847 rad */
    {
        .input_axis = {45592, -36842, 29309},
        .input_angle_rad = -121039,
        .expected_quaternion = {39523, -36368, 29389, -23379},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22498, -60816, -9498}
    },
    /* Vector 423: Angle -1.844 rad */
    {
        .input_axis = {43694, -39075, 29309},
        .input_angle_rad = -120838,
        .expected_quaternion = {39603, -34814, 31133, -23352},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19314, -61300, -12817}
    },
    /* Vector 424: Angle -1.841 rad */
    {
        .input_axis = {41686, -41210, 29309},
        .input_angle_rad = -120637,
        .expected_quaternion = {39683, -33175, 32796, -23325},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16109, -61451, -16102}
    },
    /* Vector 425: Angle -1.838 rad */
    {
        .input_axis = {39575, -43242, 29309},
        .input_angle_rad = -120436,
        .expected_quaternion = {39763, -31458, 34373, -23298},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12915, -61270, -19344}
    },
    /* Vector 426: Angle -1.835 rad */
    {
        .input_axis = {37364, -45165, 29309},
        .input_angle_rad = -120235,
        .expected_quaternion = {39843, -29666, 35860, -23270},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {9766, -60760, -22535}
    },
    /* Vector 427: Angle -1.832 rad */
    {
        .input_axis = {35060, -46976, 29309},
        .input_angle_rad = -120034,
        .expected_quaternion = {39922, -27804, 37254, -23243},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {6695, -59928, -25666}
    },
    /* Vector 428: Angle -1.829 rad */
    {
        .input_axis = {32668, -48670, 29309},
        .input_angle_rad = -119833,
        .expected_quaternion = {40002, -25877, 38552, -23216},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {3732, -58785, -28729}
    },
    /* Vector 429: Angle -1.825 rad */
    {
        .input_axis = {30195, -50242, 29309},
        .input_angle_rad = -119632,
        .expected_quaternion = {40082, -23889, 39750, -23188},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {908, -57343, -31717}
    },
    /* Vector 430: Angle -1.822 rad */
    {
        .input_axis = {27646, -51688, 29309},
        .input_angle_rad = -119431,
        .expected_quaternion = {40161, -21847, 40846, -23161},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1748, -55618, -34620}
    },
    /* Vector 431: Angle -1.819 rad */
    {
        .input_axis = {25028, -53005, 29309},
        .input_angle_rad = -119230,
        .expected_quaternion = {40241, -19755, 41836, -23133},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4210, -53630, -37431}
    },
    /* Vector 432: Angle -1.816 rad */
    {
        .input_axis = {22348, -54190, 29309},
        .input_angle_rad = -119029,
        .expected_quaternion = {40320, -17618, 42720, -23105},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6452, -51399, -40143}
    },
    /* Vector 433: Angle -1.813 rad */
    {
        .input_axis = {19612, -55239, 29309},
        .input_angle_rad = -118828,
        .expected_quaternion = {40399, -15442, 43495, -23078},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-8452, -48949, -42749}
    },
    /* Vector 434: Angle -1.810 rad */
    {
        .input_axis = {16826, -56150, 29309},
        .input_angle_rad = -118627,
        .expected_quaternion = {40478, -13233, 44160, -23050},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-10190, -46307, -45242}
    },
    /* Vector 435: Angle -1.807 rad */
    {
        .input_axis = {13999, -56921, 29309},
        .input_angle_rad = -118425,
        .expected_quaternion = {40557, -10996, 44712, -23022},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11648, -43499, -47614}
    },
    /* Vector 436: Angle -1.804 rad */
    {
        .input_axis = {11136, -57550, 29309},
        .input_angle_rad = -118224,
        .expected_quaternion = {40636, -8737, 45151, -22994},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-12813, -40554, -49861}
    },
    /* Vector 437: Angle -1.801 rad */
    {
        .input_axis = {8246, -58034, 29309},
        .input_angle_rad = -118023,
        .expected_quaternion = {40715, -6462, 45476, -22966},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13673, -37504, -51976}
    },
    /* Vector 438: Angle -1.798 rad */
    {
        .input_axis = {5335, -58374, 29309},
        .input_angle_rad = -117822,
        .expected_quaternion = {40794, -4176, 45686, -22938},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14219, -34379, -53953}
    },
    /* Vector 439: Angle -1.795 rad */
    {
        .input_axis = {2411, -58568, 29309},
        .input_angle_rad = -117621,
        .expected_quaternion = {40872, -1885, 45782, -22910},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14447, -31210, -55787}
    },
    /* Vector 440: Angle -1.792 rad */
    {
        .input_axis = {-519, -58615, 29309},
        .input_angle_rad = -117420,
        .expected_quaternion = {40951, 405, 45763, -22882},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14354, -28031, -57474}
    },
    /* Vector 441: Angle -1.789 rad */
    {
        .input_axis = {-3448, -58516, 29309},
        .input_angle_rad = -117219,
        .expected_quaternion = {41029, 2688, 45629, -22854},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13942, -24872, -59008}
    },
    /* Vector 442: Angle -1.786 rad */
    {
        .input_axis = {-6368, -58270, 29309},
        .input_angle_rad = -117018,
        .expected_quaternion = {41108, 4959, 45382, -22826},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13216, -21767, -60386}
    },
    /* Vector 443: Angle -1.782 rad */
    {
        .input_axis = {-9272, -57879, 29309},
        .input_angle_rad = -116817,
        .expected_quaternion = {41186, 7212, 45022, -22798},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-12182, -18745, -61605}
    },
    /* Vector 444: Angle -1.779 rad */
    {
        .input_axis = {-12153, -57343, 29309},
        .input_angle_rad = -116616,
        .expected_quaternion = {41264, 9442, 44549, -22770},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-10853, -15836, -62661}
    },
    /* Vector 445: Angle -1.776 rad */
    {
        .input_axis = {-15004, -56664, 29309},
        .input_angle_rad = -116415,
        .expected_quaternion = {41342, 11642, 43967, -22741},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-9240, -13071, -63551}
    },
    /* Vector 446: Angle -1.773 rad */
    {
        .input_axis = {-17818, -55844, 29309},
        .input_angle_rad = -116214,
        .expected_quaternion = {41420, 13808, 43276, -22713},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-7361, -10474, -64273}
    },
    /* Vector 447: Angle -1.770 rad */
    {
        .input_axis = {-20586, -54883, 29309},
        .input_angle_rad = -116013,
        .expected_quaternion = {41498, 15933, 42479, -22684},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-5235, -8072, -64826}
    },
    /* Vector 448: Angle -1.767 rad */
    {
        .input_axis = {-23304, -53786, 29309},
        .input_angle_rad = -115812,
        .expected_quaternion = {41576, 18014, 41577, -22656},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-2882, -5889, -65207}
    },
    /* Vector 449: Angle -1.764 rad */
    {
        .input_axis = {-25963, -52554, 29309},
        .input_angle_rad = -115611,
        .expected_quaternion = {41653, 20044, 40574, -22627},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-327, -3944, -65416}
    },
    /* Vector 450: Angle -1.761 rad */
    {
        .input_axis = {-28557, -51191, 29309},
        .input_angle_rad = -115410,
        .expected_quaternion = {41731, 22019, 39471, -22599},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2405, -2257, -65453}
    },
    /* Vector 451: Angle -1.758 rad */
    {
        .input_axis = {-31080, -49699, 29309},
        .input_angle_rad = -115208,
        .expected_quaternion = {41808, 23934, 38273, -22570},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5288, -842, -65317}
    },
    /* Vector 452: Angle -1.755 rad */
    {
        .input_axis = {-33525, -48084, 29309},
        .input_angle_rad = -115007,
        .expected_quaternion = {41886, 25784, 36982, -22541},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8293, 286, -65009}
    },
    /* Vector 453: Angle -1.752 rad */
    {
        .input_axis = {-35886, -46348, 29309},
        .input_angle_rad = -114806,
        .expected_quaternion = {41963, 27565, 35601, -22513},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {11390, 1118, -64529}
    },
    /* Vector 454: Angle -1.749 rad */
    {
        .input_axis = {-38158, -44497, 29309},
        .input_angle_rad = -114605,
        .expected_quaternion = {42040, 29272, 34135, -22484},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {14549, 1648, -63879}
    },
    /* Vector 455: Angle -1.746 rad */
    {
        .input_axis = {-40334, -42534, 29309},
        .input_angle_rad = -114404,
        .expected_quaternion = {42117, 30902, 32588, -22455},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17740, 1870, -63062}
    },
    /* Vector 456: Angle -1.743 rad */
    {
        .input_axis = {-42409, -40465, 29309},
        .input_angle_rad = -114203,
        .expected_quaternion = {42194, 32450, 30963, -22426},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20931, 1785, -62078}
    },
    /* Vector 457: Angle -1.740 rad */
    {
        .input_axis = {-44379, -38295, 29309},
        .input_angle_rad = -114002,
        .expected_quaternion = {42271, 33913, 29264, -22397},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24092, 1395, -60931}
    },
    /* Vector 458: Angle -1.736 rad */
    {
        .input_axis = {-46237, -36029, 29309},
        .input_angle_rad = -113801,
        .expected_quaternion = {42348, 35288, 27497, -22368},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27193, 704, -59624}
    },
    /* Vector 459: Angle -1.733 rad */
    {
        .input_axis = {-47980, -33673, 29309},
        .input_angle_rad = -113600,
        .expected_quaternion = {42424, 36570, 25666, -22339},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30204, -278, -58160}
    },
    /* Vector 460: Angle -1.730 rad */
    {
        .input_axis = {-49603, -31233, 29309},
        .input_angle_rad = -113399,
        .expected_quaternion = {42501, 37758, 23775, -22310},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33097, -1541, -56544}
    },
    /* Vector 461: Angle -1.727 rad */
    {
        .input_axis = {-51102, -28715, 29309},
        .input_angle_rad = -113198,
        .expected_quaternion = {42578, 38848, 21829, -22281},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35844, -3071, -54779}
    },
    /* Vector 462: Angle -1.724 rad */
    {
        .input_axis = {-52473, -26125, 29309},
        .input_angle_rad = -112997,
        .expected_quaternion = {42654, 39838, 19834, -22251},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38420, -4850, -52871}
    },
    /* Vector 463: Angle -1.721 rad */
    {
        .input_axis = {-53713, -23470, 29309},
        .input_angle_rad = -112796,
        .expected_quaternion = {42730, 40726, 17795, -22222},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40802, -6861, -50824}
    },
    /* Vector 464: Angle -1.718 rad */
    {
        .input_axis = {-54819, -20756, 29309},
        .input_angle_rad = -112595,
        .expected_quaternion = {42806, 41510, 15717, -22193},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42967, -9082, -48645}
    },
    /* Vector 465: Angle -1.715 rad */
    {
        .input_axis = {-55788, -17990, 29309},
        .input_angle_rad = -112394,
        .expected_quaternion = {42882, 42187, 13604, -22163},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44897, -11489, -46338}
    },
    /* Vector 466: Angle -1.712 rad */
    {
        .input_axis = {-56618, -15179, 29309},
        .input_angle_rad = -112193,
        .expected_quaternion = {42958, 42758, 11464, -22134},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46575, -14059, -43910}
    },
    /* Vector 467: Angle -1.709 rad */
    {
        .input_axis = {-57306, -12331, 29309},
        .input_angle_rad = -111991,
        .expected_quaternion = {43034, 43220, 9300, -22104},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47986, -16764, -41368}
    },
    /* Vector 468: Angle -1.706 rad */
    {
        .input_axis = {-57850, -9451, 29309},
        .input_angle_rad = -111790,
        .expected_quaternion = {43110, 43572, 7119, -22075},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49118, -19576, -38719}
    },
    /* Vector 469: Angle -1.703 rad */
    {
        .input_axis = {-58250, -6548, 29309},
        .input_angle_rad = -111589,
        .expected_quaternion = {43186, 43815, 4925, -22045},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49964, -22468, -35968}
    },
    /* Vector 470: Angle -1.700 rad */
    {
        .input_axis = {-58505, -3629, 29309},
        .input_angle_rad = -111388,
        .expected_quaternion = {43261, 43947, 2726, -22016},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50518, -25410, -33125}
    },
    /* Vector 471: Angle -1.697 rad */
    {
        .input_axis = {-58613, -700, 29309},
        .input_angle_rad = -111187,
        .expected_quaternion = {43337, 43969, 525, -21986},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50776, -28372, -30196}
    },
    /* Vector 472: Angle -1.694 rad */
    {
        .input_axis = {-58575, 2230, 29309},
        .input_angle_rad = -110986,
        .expected_quaternion = {43412, 43880, -1671, -21956},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50739, -31325, -27189}
    },
    /* Vector 473: Angle -1.690 rad */
    {
        .input_axis = {-58390, 5155, 29309},
        .input_angle_rad = -110785,
        .expected_quaternion = {43487, 43683, -3856, -21926},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50410, -34240, -24112}
    },
    /* Vector 474: Angle -1.687 rad */
    {
        .input_axis = {-58059, 8067, 29309},
        .input_angle_rad = -110584,
        .expected_quaternion = {43562, 43376, -6027, -21896},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49796, -37087, -20973}
    },
    /* Vector 475: Angle -1.684 rad */
    {
        .input_axis = {-57584, 10958, 29309},
        .input_angle_rad = -110383,
        .expected_quaternion = {43638, 42962, -8176, -21867},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48904, -39839, -17781}
    },
    /* Vector 476: Angle -1.681 rad */
    {
        .input_axis = {-56964, 13823, 29309},
        .input_angle_rad = -110182,
        .expected_quaternion = {43713, 42442, -10299, -21837},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47747, -42469, -14545}
    },
    /* Vector 477: Angle -1.678 rad */
    {
        .input_axis = {-56202, 16652, 29309},
        .input_angle_rad = -109981,
        .expected_quaternion = {43787, 41816, -12390, -21807},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46339, -44951, -11271}
    },
    /* Vector 478: Angle -1.675 rad */
    {
        .input_axis = {-55300, 19441, 29309},
        .input_angle_rad = -109780,
        .expected_quaternion = {43862, 41088, -14444, -21777},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44697, -47261, -7971}
    },
    /* Vector 479: Angle -1.672 rad */
    {
        .input_axis = {-54259, 22180, 29309},
        .input_angle_rad = -109579,
        .expected_quaternion = {43937, 40259, -16457, -21746},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42839, -49378, -4651}
    },
    /* Vector 480: Angle -1.669 rad */
    {
        .input_axis = {-53082, 24864, 29309},
        .input_angle_rad = -109378,
        .expected_quaternion = {44011, 39332, -18423, -21716},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40786, -51281, -1322}
    },
    /* Vector 481: Angle -1.666 rad */
    {
        .input_axis = {-51773, 27486, 29309},
        .input_angle_rad = -109177,
        .expected_quaternion = {44086, 38308, -20338, -21686},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38562, -52952, 2009}
    },
    /* Vector 482: Angle -1.663 rad */
    {
        .input_axis = {-50335, 30039, 29309},
        .input_angle_rad = -108976,
        .expected_quaternion = {44160, 37192, -22196, -21656},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36190, -54377, 5333}
    },
    /* Vector 483: Angle -1.660 rad */
    {
        .input_axis = {-48771, 32518, 29309},
        .input_angle_rad = -108775,
        .expected_quaternion = {44234, 35986, -23993, -21625},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33696, -55542, 8640}
    },
    /* Vector 484: Angle -1.657 rad */
    {
        .input_axis = {-47085, 34914, 29309},
        .input_angle_rad = -108573,
        .expected_quaternion = {44308, 34693, -25726, -21595},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31108, -56437, 11922}
    },
    /* Vector 485: Angle -1.654 rad */
    {
        .input_axis = {-45281, 37224, 29309},
        .input_angle_rad = -108372,
        .expected_quaternion = {44382, 33317, -27389, -21565},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28452, -57055, 15171}
    },
    /* Vector 486: Angle -1.651 rad */
    {
        .input_axis = {-43364, 39441, 29309},
        .input_angle_rad = -108171,
        .expected_quaternion = {44456, 31861, -28979, -21534},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25757, -57392, 18377}
    },
    /* Vector 487: Angle -1.647 rad */
    {
        .input_axis = {-41338, 41559, 29309},
        .input_angle_rad = -107970,
        .expected_quaternion = {44530, 30330, -30491, -21504},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23051, -57445, 21533}
    },
    /* Vector 488: Angle -1.644 rad */
    {
        .input_axis = {-39210, 43573, 29309},
        .input_angle_rad = -107769,
        .expected_quaternion = {44604, 28727, -31924, -21473},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20363, -57216, 24630}
    },
    /* Vector 489: Angle -1.641 rad */
    {
        .input_axis = {-36983, 45478, 29309},
        .input_angle_rad = -107568,
        .expected_quaternion = {44677, 27057, -33272, -21442},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17721, -56709, 27659}
    },
    /* Vector 490: Angle -1.638 rad */
    {
        .input_axis = {-34664, 47269, 29309},
        .input_angle_rad = -107367,
        .expected_quaternion = {44751, 25324, -34533, -21412},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15151, -55930, 30614}
    },
    /* Vector 491: Angle -1.635 rad */
    {
        .input_axis = {-32258, 48943, 29309},
        .input_angle_rad = -107166,
        .expected_quaternion = {44824, 23533, -35705, -21381},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12681, -54889, 33486}
    },
    /* Vector 492: Angle -1.632 rad */
    {
        .input_axis = {-29771, 50494, 29309},
        .input_angle_rad = -106965,
        .expected_quaternion = {44898, 21687, -36783, -21350},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10335, -53598, 36268}
    },
    /* Vector 493: Angle -1.629 rad */
    {
        .input_axis = {-27211, 51919, 29309},
        .input_angle_rad = -106764,
        .expected_quaternion = {44971, 19793, -37766, -21319},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8138, -52071, 38953}
    },
    /* Vector 494: Angle -1.626 rad */
    {
        .input_axis = {-24582, 53214, 29309},
        .input_angle_rad = -106563,
        .expected_quaternion = {45044, 17855, -38652, -21289},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {6112, -50325, 41533}
    },
    /* Vector 495: Angle -1.623 rad */
    {
        .input_axis = {-21891, 54376, 29309},
        .input_angle_rad = -106362,
        .expected_quaternion = {45117, 15878, -39439, -21258},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {4277, -48379, 44001}
    },
    /* Vector 496: Angle -1.620 rad */
    {
        .input_axis = {-19146, 55402, 29309},
        .input_angle_rad = -106161,
        .expected_quaternion = {45190, 13867, -40125, -21227},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2652, -46253, 46353}
    },
    /* Vector 497: Angle -1.617 rad */
    {
        .input_axis = {-16354, 56290, 29309},
        .input_angle_rad = -105960,
        .expected_quaternion = {45262, 11827, -40708, -21196},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {1254, -43970, 48580}
    },
    /* Vector 498: Angle -1.614 rad */
    {
        .input_axis = {-13520, 57037, 29309},
        .input_angle_rad = -105759,
        .expected_quaternion = {45335, 9763, -41188, -21165},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {95, -41553, 50678}
    },
    /* Vector 499: Angle -1.611 rad */
    {
        .input_axis = {-10652, 57641, 29309},
        .input_angle_rad = -105558,
        .expected_quaternion = {45408, 7681, -41563, -21133},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-813, -39028, 52642}
    },
    /* Vector 500: Angle -1.608 rad */
    {
        .input_axis = {-7758, 58102, 29309},
        .input_angle_rad = -105356,
        .expected_quaternion = {45480, 5586, -41833, -21102},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1460, -36420, 54465}
    },
    /* Vector 501: Angle -1.605 rad */
    {
        .input_axis = {-4845, 58417, 29309},
        .input_angle_rad = -105155,
        .expected_quaternion = {45552, 3483, -41998, -21071},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1841, -33756, 56144}
    },
    /* Vector 502: Angle -1.601 rad */
    {
        .input_axis = {-1919, 58586, 29309},
        .input_angle_rad = -104954,
        .expected_quaternion = {45625, 1377, -42057, -21040},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1952, -31063, 57674}
    },
    /* Vector 503: Angle -1.598 rad */
    {
        .input_axis = {1012, 58608, 29309},
        .input_angle_rad = -104753,
        .expected_quaternion = {45697, -725, -42011, -21008},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1793, -28368, 59051}
    },
    /* Vector 504: Angle -1.595 rad */
    {
        .input_axis = {3940, 58485, 29309},
        .input_angle_rad = -104552,
        .expected_quaternion = {45769, -2820, -41859, -20977},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1366, -25698, 60272}
    },
    /* Vector 505: Angle -1.592 rad */
    {
        .input_axis = {6858, 58215, 29309},
        .input_angle_rad = -104351,
        .expected_quaternion = {45841, -4901, -41604, -20946},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-674, -23079, 61334}
    },
    /* Vector 506: Angle -1.589 rad */
    {
        .input_axis = {9759, 57799, 29309},
        .input_angle_rad = -104150,
        .expected_quaternion = {45912, -6964, -41245, -20914},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {274, -20539, 62234}
    },
    /* Vector 507: Angle -1.586 rad */
    {
        .input_axis = {12635, 57239, 29309},
        .input_angle_rad = -103949,
        .expected_quaternion = {45984, -9003, -40783, -20883},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {1468, -18100, 62970}
    },
    /* Vector 508: Angle -1.583 rad */
    {
        .input_axis = {15480, 56536, 29309},
        .input_angle_rad = -103748,
        .expected_quaternion = {46056, -11013, -40222, -20851},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2897, -15788, 63540}
    },
    /* Vector 509: Angle -1.580 rad */
    {
        .input_axis = {18286, 55692, 29309},
        .input_angle_rad = -103547,
        .expected_quaternion = {46127, -12990, -39561, -20819},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {4546, -13625, 63943}
    },
    /* Vector 510: Angle -1.577 rad */
    {
        .input_axis = {21047, 54708, 29309},
        .input_angle_rad = -103346,
        .expected_quaternion = {46199, -14928, -38803, -20788},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {6399, -11631, 64178}
    },
    /* Vector 511: Angle -1.574 rad */
    {
        .input_axis = {23755, 53588, 29309},
        .input_angle_rad = -103145,
        .expected_quaternion = {46270, -16823, -37951, -20756},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8436, -9825, 64244}
    },
    /* Vector 512: Angle -1.571 rad */
    {
        .input_axis = {26404, 52334, 29309},
        .input_angle_rad = -102944,
        .expected_quaternion = {46341, -18670, -37006, -20724},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10638, -8224, 64142}
    },
    /* Vector 513: Angle -1.568 rad */
    {
        .input_axis = {28986, 50949, 29309},
        .input_angle_rad = -102743,
        .expected_quaternion = {46412, -20465, -35971, -20692},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12982, -6843, 63872}
    },
    /* Vector 514: Angle -1.565 rad */
    {
        .input_axis = {31496, 49436, 29309},
        .input_angle_rad = -102542,
        .expected_quaternion = {46483, -22203, -34849, -20661},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15446, -5695, 63435}
    },
    /* Vector 515: Angle -1.562 rad */
    {
        .input_axis = {33928, 47800, 29309},
        .input_angle_rad = -102341,
        .expected_quaternion = {46554, -23880, -33644, -20629},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18006, -4789, 62832}
    },
    /* Vector 516: Angle -1.559 rad */
    {
        .input_axis = {36274, 46045, 29309},
        .input_angle_rad = -102139,
        .expected_quaternion = {46624, -25492, -32358, -20597},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20636, -4133, 62065}
    },
    /* Vector 517: Angle -1.555 rad */
    {
        .input_axis = {38530, 44175, 29309},
        .input_angle_rad = -101938,
        .expected_quaternion = {46695, -27035, -30996, -20565},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23311, -3732, 61136}
    },
    /* Vector 518: Angle -1.552 rad */
    {
        .input_axis = {40690, 42194, 29309},
        .input_angle_rad = -101737,
        .expected_quaternion = {46765, -28506, -29560, -20533},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26005, -3589, 60049}
    },
    /* Vector 519: Angle -1.549 rad */
    {
        .input_axis = {42748, 40107, 29309},
        .input_angle_rad = -101536,
        .expected_quaternion = {46836, -29901, -28054, -20501},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28692, -3702, 58805}
    },
    /* Vector 520: Angle -1.546 rad */
    {
        .input_axis = {44699, 37921, 29309},
        .input_angle_rad = -101335,
        .expected_quaternion = {46906, -31217, -26483, -20468},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31347, -4071, 57409}
    },
    /* Vector 521: Angle -1.543 rad */
    {
        .input_axis = {46538, 35639, 29309},
        .input_angle_rad = -101134,
        .expected_quaternion = {46976, -32450, -24850, -20436},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33945, -4688, 55864}
    },
    /* Vector 522: Angle -1.540 rad */
    {
        .input_axis = {48261, 33269, 29309},
        .input_angle_rad = -100933,
        .expected_quaternion = {47046, -33599, -23161, -20404},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36460, -5547, 54174}
    },
    /* Vector 523: Angle -1.537 rad */
    {
        .input_axis = {49864, 30815, 29309},
        .input_angle_rad = -100732,
        .expected_quaternion = {47116, -34659, -21419, -20372},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38871, -6637, 52345}
    },
    /* Vector 524: Angle -1.534 rad */
    {
        .input_axis = {51342, 28284, 29309},
        .input_angle_rad = -100531,
        .expected_quaternion = {47186, -35630, -19629, -20339},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41153, -7946, 50381}
    },
    /* Vector 525: Angle -1.531 rad */
    {
        .input_axis = {52691, 25683, 29309},
        .input_angle_rad = -100330,
        .expected_quaternion = {47256, -36508, -17795, -20307},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43288, -9459, 48287}
    },
    /* Vector 526: Angle -1.528 rad */
    {
        .input_axis = {53909, 23017, 29309},
        .input_angle_rad = -100129,
        .expected_quaternion = {47325, -37292, -15923, -20274},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45255, -11161, 46070}
    },
    /* Vector 527: Angle -1.525 rad */
    {
        .input_axis = {54992, 20294, 29309},
        .input_angle_rad = -99928,
        .expected_quaternion = {47395, -37980, -14016, -20242},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47036, -13032, 43735}
    },
    /* Vector 528: Angle -1.522 rad */
    {
        .input_axis = {55938, 17521, 29309},
        .input_angle_rad = -99727,
        .expected_quaternion = {47464, -38571, -12081, -20209},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48618, -15053, 41288}
    },
    /* Vector 529: Angle -1.519 rad */
    {
        .input_axis = {56743, 14703, 29309},
        .input_angle_rad = -99526,
        .expected_quaternion = {47534, -39064, -10122, -20177},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49985, -17202, 38736}
    },
    /* Vector 530: Angle -1.516 rad */
    {
        .input_axis = {57407, 11849, 29309},
        .input_angle_rad = -99325,
        .expected_quaternion = {47603, -39457, -8144, -20144},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51128, -19458, 36087}
    },
    /* Vector 531: Angle -1.513 rad */
    {
        .input_axis = {57928, 8965, 29309},
        .input_angle_rad = -99124,
        .expected_quaternion = {47672, -39750, -6152, -20112},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52038, -21797, 33346}
    },
    /* Vector 532: Angle -1.509 rad */
    {
        .input_axis = {58303, 6058, 29309},
        .input_angle_rad = -98922,
        .expected_quaternion = {47741, -39943, -4150, -20079},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52707, -24194, 30522}
    },
    /* Vector 533: Angle -1.506 rad */
    {
        .input_axis = {58533, 3137, 29309},
        .input_angle_rad = -98721,
        .expected_quaternion = {47809, -40035, -2145, -20046},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53132, -26627, 27622}
    },
    /* Vector 534: Angle -1.503 rad */
    {
        .input_axis = {58617, 207, 29309},
        .input_angle_rad = -98520,
        .expected_quaternion = {47878, -40026, -142, -20013},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53312, -29069, 24653}
    },
    /* Vector 535: Angle -1.500 rad */
    {
        .input_axis = {58554, -2723, 29309},
        .input_angle_rad = -98319,
        .expected_quaternion = {47947, -39918, 1856, -19980},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53248, -31497, 21624}
    },
    /* Vector 536: Angle -1.497 rad */
    {
        .input_axis = {58345, -5646, 29309},
        .input_angle_rad = -98118,
        .expected_quaternion = {48015, -39709, 3842, -19947},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52942, -33886, 18543}
    },
    /* Vector 537: Angle -1.494 rad */
    {
        .input_axis = {57990, -8555, 29309},
        .input_angle_rad = -97917,
        .expected_quaternion = {48084, -39403, 5813, -19914},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52402, -36212, 15417}
    },
    /* Vector 538: Angle -1.491 rad */
    {
        .input_axis = {57490, -11442, 29309},
        .input_angle_rad = -97716,
        .expected_quaternion = {48152, -38998, 7762, -19881},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51635, -38453, 12256}
    },
    /* Vector 539: Angle -1.488 rad */
    {
        .input_axis = {56846, -14301, 29309},
        .input_angle_rad = -97515,
        .expected_quaternion = {48220, -38497, 9685, -19848},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50651, -40586, 9067}
    },
    /* Vector 540: Angle -1.485 rad */
    {
        .input_axis = {56060, -17124, 29309},
        .input_angle_rad = -97314,
        .expected_quaternion = {48288, -37902, 11578, -19815},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49463, -42592, 5859}
    },
    /* Vector 541: Angle -1.482 rad */
    {
        .input_axis = {55134, -19905, 29309},
        .input_angle_rad = -97113,
        .expected_quaternion = {48356, -37213, 13435, -19782},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48085, -44450, 2640}
    },
    /* Vector 542: Angle -1.479 rad */
    {
        .input_axis = {54070, -22635, 29309},
        .input_angle_rad = -96912,
        .expected_quaternion = {48424, -36434, 15252, -19749},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46534, -46143, -581}
    },
    /* Vector 543: Angle -1.476 rad */
    {
        .input_axis = {52872, -25310, 29309},
        .input_angle_rad = -96711,
        .expected_quaternion = {48491, -35566, 17026, -19716},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44827, -47656, -3796}
    },
    /* Vector 544: Angle -1.473 rad */
    {
        .input_axis = {51541, -27920, 29309},
        .input_angle_rad = -96510,
        .expected_quaternion = {48559, -34612, 18750, -19682},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42984, -48973, -6996}
    },
    /* Vector 545: Angle -1.470 rad */
    {
        .input_axis = {50081, -30461, 29309},
        .input_angle_rad = -96309,
        .expected_quaternion = {48626, -33575, 20422, -19649},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41026, -50084, -10172}
    },
    /* Vector 546: Angle -1.466 rad */
    {
        .input_axis = {48496, -32926, 29309},
        .input_angle_rad = -96108,
        .expected_quaternion = {48694, -32457, 22037, -19616},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38973, -50977, -13318}
    },
    /* Vector 547: Angle -1.463 rad */
    {
        .input_axis = {46789, -35309, 29309},
        .input_angle_rad = -95907,
        .expected_quaternion = {48761, -31262, 23591, -19582},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36849, -51647, -16423}
    },
    /* Vector 548: Angle -1.460 rad */
    {
        .input_axis = {44966, -37603, 29309},
        .input_angle_rad = -95705,
        .expected_quaternion = {48828, -29992, 25081, -19549},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34676, -52087, -19481}
    },
    /* Vector 549: Angle -1.457 rad */
    {
        .input_axis = {43031, -39804, 29309},
        .input_angle_rad = -95504,
        .expected_quaternion = {48895, -28652, 26504, -19515},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32477, -52295, -22483}
    },
    /* Vector 550: Angle -1.454 rad */
    {
        .input_axis = {40987, -41905, 29309},
        .input_angle_rad = -95303,
        .expected_quaternion = {48962, -27245, 27854, -19482},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30276, -52269, -25422}
    },
    /* Vector 551: Angle -1.451 rad */
    {
        .input_axis = {38842, -43901, 29309},
        .input_angle_rad = -95102,
        .expected_quaternion = {49029, -25774, 29131, -19448},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28096, -52012, -28290}
    },
    /* Vector 552: Angle -1.448 rad */
    {
        .input_axis = {36599, -45787, 29309},
        .input_angle_rad = -94901,
        .expected_quaternion = {49095, -24244, 30330, -19414},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25960, -51528, -31079}
    },
    /* Vector 553: Angle -1.445 rad */
    {
        .input_axis = {34265, -47559, 29309},
        .input_angle_rad = -94700,
        .expected_quaternion = {49162, -22658, 31449, -19381},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23890, -50823, -33782}
    },
    /* Vector 554: Angle -1.442 rad */
    {
        .input_axis = {31845, -49212, 29309},
        .input_angle_rad = -94499,
        .expected_quaternion = {49228, -21022, 32486, -19347},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21907, -49906, -36393}
    },
    /* Vector 555: Angle -1.439 rad */
    {
        .input_axis = {29346, -50742, 29309},
        .input_angle_rad = -94298,
        .expected_quaternion = {49295, -19338, 33437, -19313},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20033, -48787, -38904}
    },
    /* Vector 556: Angle -1.436 rad */
    {
        .input_axis = {26773, -52146, 29309},
        .input_angle_rad = -94097,
        .expected_quaternion = {49361, -17612, 34302, -19279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18285, -47478, -41309}
    },
    /* Vector 557: Angle -1.433 rad */
    {
        .input_axis = {24134, -53419, 29309},
        .input_angle_rad = -93896,
        .expected_quaternion = {49427, -15847, 35077, -19246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16683, -45994, -43603}
    },
    /* Vector 558: Angle -1.430 rad */
    {
        .input_axis = {21434, -54558, 29309},
        .input_angle_rad = -93695,
        .expected_quaternion = {49493, -14050, 35762, -19212},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15242, -44351, -45779}
    },
    /* Vector 559: Angle -1.427 rad */
    {
        .input_axis = {18680, -55561, 29309},
        .input_angle_rad = -93494,
        .expected_quaternion = {49559, -12223, 36355, -19178},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13977, -42566, -47831}
    },
    /* Vector 560: Angle -1.424 rad */
    {
        .input_axis = {15880, -56425, 29309},
        .input_angle_rad = -93293,
        .expected_quaternion = {49624, -10372, 36855, -19144},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12899, -40657, -49755}
    },
    /* Vector 561: Angle -1.420 rad */
    {
        .input_axis = {13040, -57148, 29309},
        .input_angle_rad = -93092,
        .expected_quaternion = {49690, -8502, 37261, -19110},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12021, -38646, -51546}
    },
    /* Vector 562: Angle -1.417 rad */
    {
        .input_axis = {10167, -57729, 29309},
        .input_angle_rad = -92891,
        .expected_quaternion = {49756, -6617, 37573, -19075},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {11350, -36552, -53199}
    },
    /* Vector 563: Angle -1.414 rad */
    {
        .input_axis = {7269, -58165, 29309},
        .input_angle_rad = -92690,
        .expected_quaternion = {49821, -4723, 37789, -19041},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10893, -34397, -54710}
    },
    /* Vector 564: Angle -1.411 rad */
    {
        .input_axis = {4353, -58455, 29309},
        .input_angle_rad = -92488,
        .expected_quaternion = {49886, -2823, 37909, -19007},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10654, -32202, -56075}
    },
    /* Vector 565: Angle -1.408 rad */
    {
        .input_axis = {1426, -58600, 29309},
        .input_angle_rad = -92287,
        .expected_quaternion = {49951, -923, 37934, -18973},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10635, -29991, -57292}
    },
    /* Vector 566: Angle -1.405 rad */
    {
        .input_axis = {-1504, -58598, 29309},
        .input_angle_rad = -92086,
        .expected_quaternion = {50016, 972, 37865, -18939},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10837, -27784, -58357}
    },
    /* Vector 567: Angle -1.402 rad */
    {
        .input_axis = {-4431, -58449, 29309},
        .input_angle_rad = -91885,
        .expected_quaternion = {50081, 2858, 37700, -18904},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {11255, -25604, -59268}
    },
    /* Vector 568: Angle -1.399 rad */
    {
        .input_axis = {-7347, -58155, 29309},
        .input_angle_rad = -91684,
        .expected_quaternion = {50146, 4730, 37442, -18870},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {11887, -23472, -60023}
    },
    /* Vector 569: Angle -1.396 rad */
    {
        .input_axis = {-10244, -57715, 29309},
        .input_angle_rad = -91483,
        .expected_quaternion = {50211, 6583, 37091, -18835},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12725, -21410, -60619}
    },
    /* Vector 570: Angle -1.393 rad */
    {
        .input_axis = {-13116, -57131, 29309},
        .input_angle_rad = -91282,
        .expected_quaternion = {50275, 8414, 36648, -18801},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13760, -19436, -61056}
    },
    /* Vector 571: Angle -1.390 rad */
    {
        .input_axis = {-15955, -56404, 29309},
        .input_angle_rad = -91081,
        .expected_quaternion = {50340, 10216, 36116, -18766},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {14983, -17570, -61333}
    },
    /* Vector 572: Angle -1.387 rad */
    {
        .input_axis = {-18754, -55536, 29309},
        .input_angle_rad = -90880,
        .expected_quaternion = {50404, 11986, 35495, -18732},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16380, -15830, -61450}
    },
    /* Vector 573: Angle -1.384 rad */
    {
        .input_axis = {-21506, -54529, 29309},
        .input_angle_rad = -90679,
        .expected_quaternion = {50468, 13720, 34787, -18697},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17938, -14232, -61406}
    },
    /* Vector 574: Angle -1.381 rad */
    {
        .input_axis = {-24205, -53386, 29309},
        .input_angle_rad = -90478,
        .expected_quaternion = {50532, 15413, 33994, -18663},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19640, -12791, -61202}
    },
    /* Vector 575: Angle -1.378 rad */
    {
        .input_axis = {-26843, -52110, 29309},
        .input_angle_rad = -90277,
        .expected_quaternion = {50596, 17061, 33120, -18628},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21471, -11519, -60838}
    },
    /* Vector 576: Angle -1.374 rad */
    {
        .input_axis = {-29413, -50703, 29309},
        .input_angle_rad = -90076,
        .expected_quaternion = {50660, 18660, 32166, -18593},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23411, -10429, -60317}
    },
    /* Vector 577: Angle -1.371 rad */
    {
        .input_axis = {-31911, -49170, 29309},
        .input_angle_rad = -89875,
        .expected_quaternion = {50724, 20206, 31135, -18558},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25443, -9529, -59639}
    },
    /* Vector 578: Angle -1.368 rad */
    {
        .input_axis = {-34328, -47514, 29309},
        .input_angle_rad = -89674,
        .expected_quaternion = {50787, 21696, 30029, -18524},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27545, -8827, -58808}
    },
    /* Vector 579: Angle -1.365 rad */
    {
        .input_axis = {-36660, -45738, 29309},
        .input_angle_rad = -89473,
        .expected_quaternion = {50851, 23126, 28853, -18489},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29698, -8328, -57824}
    },
    /* Vector 580: Angle -1.362 rad */
    {
        .input_axis = {-38900, -43849, 29309},
        .input_angle_rad = -89271,
        .expected_quaternion = {50914, 24493, 27609, -18454},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31881, -8036, -56692}
    },
    /* Vector 581: Angle -1.359 rad */
    {
        .input_axis = {-41043, -41850, 29309},
        .input_angle_rad = -89070,
        .expected_quaternion = {50977, 25793, 26300, -18419},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34073, -7952, -55414}
    },
    /* Vector 582: Angle -1.356 rad */
    {
        .input_axis = {-43084, -39746, 29309},
        .input_angle_rad = -88869,
        .expected_quaternion = {51041, 27024, 24931, -18384},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36254, -8074, -53995}
    },
    /* Vector 583: Angle -1.353 rad */
    {
        .input_axis = {-45016, -37543, 29309},
        .input_angle_rad = -88668,
        .expected_quaternion = {51104, 28183, 23504, -18349},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38402, -8401, -52438}
    },
    /* Vector 584: Angle -1.350 rad */
    {
        .input_axis = {-46836, -35247, 29309},
        .input_angle_rad = -88467,
        .expected_quaternion = {51166, 29266, 22024, -18314},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40498, -8926, -50747}
    },
    /* Vector 585: Angle -1.347 rad */
    {
        .input_axis = {-48539, -32862, 29309},
        .input_angle_rad = -88266,
        .expected_quaternion = {51229, 30272, 20495, -18279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42522, -9643, -48927}
    },
    /* Vector 586: Angle -1.344 rad */
    {
        .input_axis = {-50121, -30395, 29309},
        .input_angle_rad = -88065,
        .expected_quaternion = {51292, 31199, 18920, -18243},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44455, -10543, -46984}
    },
    /* Vector 587: Angle -1.341 rad */
    {
        .input_axis = {-51578, -27852, 29309},
        .input_angle_rad = -87864,
        .expected_quaternion = {51354, 32043, 17303, -18208},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46281, -11616, -44923}
    },
    /* Vector 588: Angle -1.338 rad */
    {
        .input_axis = {-52905, -25239, 29309},
        .input_angle_rad = -87663,
        .expected_quaternion = {51417, 32804, 15650, -18173},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47983, -12848, -42749}
    },
    /* Vector 589: Angle -1.335 rad */
    {
        .input_axis = {-54101, -22563, 29309},
        .input_angle_rad = -87462,
        .expected_quaternion = {51479, 33480, 13963, -18138},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49546, -14228, -40469}
    },
    /* Vector 590: Angle -1.331 rad */
    {
        .input_axis = {-55161, -19831, 29309},
        .input_angle_rad = -87261,
        .expected_quaternion = {51541, 34070, 12249, -18102},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50957, -15738, -38088}
    },
    /* Vector 591: Angle -1.328 rad */
    {
        .input_axis = {-56083, -17050, 29309},
        .input_angle_rad = -87060,
        .expected_quaternion = {51603, 34572, 10510, -18067},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52204, -17363, -35613}
    },
    /* Vector 592: Angle -1.325 rad */
    {
        .input_axis = {-56865, -14225, 29309},
        .input_angle_rad = -86859,
        .expected_quaternion = {51665, 34985, 8752, -18032},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53276, -19086, -33051}
    },
    /* Vector 593: Angle -1.322 rad */
    {
        .input_axis = {-57505, -11366, 29309},
        .input_angle_rad = -86658,
        .expected_quaternion = {51727, 35309, 6979, -17996},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54166, -20888, -30408}
    },
    /* Vector 594: Angle -1.319 rad */
    {
        .input_axis = {-58001, -8477, 29309},
        .input_angle_rad = -86457,
        .expected_quaternion = {51789, 35544, 5195, -17961},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54868, -22751, -27692}
    },
    /* Vector 595: Angle -1.316 rad */
    {
        .input_axis = {-58352, -5568, 29309},
        .input_angle_rad = -86256,
        .expected_quaternion = {51850, 35688, 3405, -17925},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55377, -24655, -24911}
    },
    /* Vector 596: Angle -1.313 rad */
    {
        .input_axis = {-58557, -2645, 29309},
        .input_angle_rad = -86055,
        .expected_quaternion = {51911, 35742, 1614, -17889},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55690, -26580, -22071}
    },
    /* Vector 597: Angle -1.310 rad */
    {
        .input_axis = {-58616, 285, 29309},
        .input_angle_rad = -85853,
        .expected_quaternion = {51973, 35707, -174, -17854},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55807, -28507, -19179}
    },
    /* Vector 598: Angle -1.307 rad */
    {
        .input_axis = {-58529, 3215, 29309},
        .input_angle_rad = -85652,
        .expected_quaternion = {52034, 35583, -1954, -17818},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55731, -30417, -16245}
    },
    /* Vector 599: Angle -1.304 rad */
    {
        .input_axis = {-58295, 6136, 29309},
        .input_angle_rad = -85451,
        .expected_quaternion = {52095, 35369, -3723, -17782},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55463, -32289, -13276}
    },
    /* Vector 600: Angle -1.301 rad */
    {
        .input_axis = {-57916, 9042, 29309},
        .input_angle_rad = -85250,
        .expected_quaternion = {52156, 35069, -5475, -17747},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55010, -34106, -10278}
    },
    /* Vector 601: Angle -1.298 rad */
    {
        .input_axis = {-57391, 11925, 29309},
        .input_angle_rad = -85049,
        .expected_quaternion = {52217, 34681, -7206, -17711},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54379, -35850, -7262}
    },
    /* Vector 602: Angle -1.295 rad */
    {
        .input_axis = {-56724, 14779, 29309},
        .input_angle_rad = -84848,
        .expected_quaternion = {52277, 34208, -8912, -17675},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53578, -37503, -4233}
    },
    /* Vector 603: Angle -1.292 rad */
    {
        .input_axis = {-55914, 17595, 29309},
        .input_angle_rad = -84647,
        .expected_quaternion = {52338, 33651, -10589, -17639},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52619, -39049, -1201}
    },
    /* Vector 604: Angle -1.289 rad */
    {
        .input_axis = {-54965, 20368, 29309},
        .input_angle_rad = -84446,
        .expected_quaternion = {52398, 33013, -12233, -17603},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51512, -40473, 1827}
    },
    /* Vector 605: Angle -1.285 rad */
    {
        .input_axis = {-53878, 23089, 29309},
        .input_angle_rad = -84245,
        .expected_quaternion = {52459, 32294, -13839, -17567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50273, -41763, 4843}
    },
    /* Vector 606: Angle -1.282 rad */
    {
        .input_axis = {-52657, 25753, 29309},
        .input_angle_rad = -84044,
        .expected_quaternion = {52519, 31497, -15405, -17531},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48915, -42905, 7838}
    },
    /* Vector 607: Angle -1.279 rad */
    {
        .input_axis = {-51304, 28353, 29309},
        .input_angle_rad = -83843,
        .expected_quaternion = {52579, 30625, -16925, -17495},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47454, -43890, 10806}
    },
    /* Vector 608: Angle -1.276 rad */
    {
        .input_axis = {-49823, 30881, 29309},
        .input_angle_rad = -83642,
        .expected_quaternion = {52639, 29679, -18396, -17459},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45906, -44709, 13738}
    },
    /* Vector 609: Angle -1.273 rad */
    {
        .input_axis = {-48217, 33333, 29309},
        .input_angle_rad = -83441,
        .expected_quaternion = {52699, 28663, -19815, -17423},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44289, -45354, 16627}
    },
    /* Vector 610: Angle -1.270 rad */
    {
        .input_axis = {-46491, 35701, 29309},
        .input_angle_rad = -83240,
        .expected_quaternion = {52759, 27580, -21179, -17387},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42622, -45820, 19466}
    },
    /* Vector 611: Angle -1.267 rad */
    {
        .input_axis = {-44648, 37980, 29309},
        .input_angle_rad = -83039,
        .expected_quaternion = {52818, 26432, -22484, -17351},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40921, -46103, 22246}
    },
    /* Vector 612: Angle -1.264 rad */
    {
        .input_axis = {-42694, 40164, 29309},
        .input_angle_rad = -82838,
        .expected_quaternion = {52878, 25222, -23727, -17314},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39206, -46203, 24962}
    },
    /* Vector 613: Angle -1.261 rad */
    {
        .input_axis = {-40634, 42248, 29309},
        .input_angle_rad = -82636,
        .expected_quaternion = {52937, 23954, -24906, -17278},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37495, -46120, 27605}
    },
    /* Vector 614: Angle -1.258 rad */
    {
        .input_axis = {-38471, 44226, 29309},
        .input_angle_rad = -82435,
        .expected_quaternion = {52996, 22632, -26017, -17242},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35807, -45855, 30170}
    },
    /* Vector 615: Angle -1.255 rad */
    {
        .input_axis = {-36213, 46093, 29309},
        .input_angle_rad = -82234,
        .expected_quaternion = {53055, 21258, -27059, -17205},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34158, -45412, 32649}
    },
    /* Vector 616: Angle -1.252 rad */
    {
        .input_axis = {-33864, 47846, 29309},
        .input_angle_rad = -82033,
        .expected_quaternion = {53114, 19837, -28028, -17169},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32567, -44797, 35037}
    },
    /* Vector 617: Angle -1.249 rad */
    {
        .input_axis = {-31430, 49478, 29309},
        .input_angle_rad = -81832,
        .expected_quaternion = {53173, 18373, -28923, -17132},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31050, -44018, 37327}
    },
    /* Vector 618: Angle -1.246 rad */
    {
        .input_axis = {-28918, 50987, 29309},
        .input_angle_rad = -81631,
        .expected_quaternion = {53232, 16868, -29741, -17096},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29622, -43083, 39514}
    },
    /* Vector 619: Angle -1.243 rad */
    {
        .input_axis = {-26334, 52369, 29309},
        .input_angle_rad = -81430,
        .expected_quaternion = {53290, 15328, -30482, -17059},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28299, -42002, 41593}
    },
    /* Vector 620: Angle -1.239 rad */
    {
        .input_axis = {-23684, 53620, 29309},
        .input_angle_rad = -81229,
        .expected_quaternion = {53349, 13756, -31143, -17023},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27094, -40788, 43557}
    },
    /* Vector 621: Angle -1.236 rad */
    {
        .input_axis = {-20974, 54736, 29309},
        .input_angle_rad = -81028,
        .expected_quaternion = {53407, 12156, -31723, -16986},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26019, -39453, 45403}
    },
    /* Vector 622: Angle -1.233 rad */
    {
        .input_axis = {-18212, 55716, 29309},
        .input_angle_rad = -80827,
        .expected_quaternion = {53465, 10532, -32221, -16950},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25085, -38012, 47125}
    },
    /* Vector 623: Angle -1.230 rad */
    {
        .input_axis = {-15405, 56557, 29309},
        .input_angle_rad = -80626,
        .expected_quaternion = {53523, 8890, -32637, -16913},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24301, -36479, 48721}
    },
    /* Vector 624: Angle -1.227 rad */
    {
        .input_axis = {-12559, 57256, 29309},
        .input_angle_rad = -80425,
        .expected_quaternion = {53581, 7232, -32968, -16876},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23674, -34871, 50185}
    },
    /* Vector 625: Angle -1.224 rad */
    {
        .input_axis = {-9682, 57812, 29309},
        .input_angle_rad = -80224,
        .expected_quaternion = {53639, 5563, -33216, -16839},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23212, -33204, 51514}
    },
    /* Vector 626: Angle -1.221 rad */
    {
        .input_axis = {-6780, 58224, 29309},
        .input_angle_rad = -80023,
        .expected_quaternion = {53697, 3887, -33380, -16803},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22918, -31494, 52706}
    },
    /* Vector 627: Angle -1.218 rad */
    {
        .input_axis = {-3862, 58490, 29309},
        .input_angle_rad = -79822,
        .expected_quaternion = {53754, 2209, -33459, -16766},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22794, -29759, 53757}
    },
    /* Vector 628: Angle -1.215 rad */
    {
        .input_axis = {-934, 58610, 29309},
        .input_angle_rad = -79621,
        .expected_quaternion = {53812, 533, -33453, -16729},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22843, -28016, 54665}
    },
    /* Vector 629: Angle -1.212 rad */
    {
        .input_axis = {1997, 58583, 29309},
        .input_angle_rad = -79419,
        .expected_quaternion = {53869, -1137, -33364, -16692},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23062, -26283, 55429}
    },
    /* Vector 630: Angle -1.209 rad */
    {
        .input_axis = {4922, 58410, 29309},
        .input_angle_rad = -79218,
        .expected_quaternion = {53926, -2797, -33192, -16655},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23449, -24576, 56046}
    },
    /* Vector 631: Angle -1.206 rad */
    {
        .input_axis = {7835, 58091, 29309},
        .input_angle_rad = -79017,
        .expected_quaternion = {53983, -4443, -32937, -16618},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24001, -22911, 56516}
    },
    /* Vector 632: Angle -1.203 rad */
    {
        .input_axis = {10729, 57627, 29309},
        .input_angle_rad = -78816,
        .expected_quaternion = {54040, -6070, -32601, -16581},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24710, -21306, 56837}
    },
    /* Vector 633: Angle -1.200 rad */
    {
        .input_axis = {13596, 57019, 29309},
        .input_angle_rad = -78615,
        .expected_quaternion = {54097, -7674, -32185, -16544},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25571, -19774, 57009}
    },
    /* Vector 634: Angle -1.197 rad */
    {
        .input_axis = {16429, 56268, 29309},
        .input_angle_rad = -78414,
        .expected_quaternion = {54154, -9253, -31690, -16507},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26573, -18331, 57033}
    },
    /* Vector 635: Angle -1.193 rad */
    {
        .input_axis = {19220, 55377, 29309},
        .input_angle_rad = -78213,
        .expected_quaternion = {54210, -10800, -31118, -16469},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27708, -16990, 56909}
    },
    /* Vector 636: Angle -1.190 rad */
    {
        .input_axis = {21964, 54347, 29309},
        .input_angle_rad = -78012,
        .expected_quaternion = {54267, -12314, -30470, -16432},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28962, -15762, 56636}
    },
    /* Vector 637: Angle -1.187 rad */
    {
        .input_axis = {24653, 53181, 29309},
        .input_angle_rad = -77811,
        .expected_quaternion = {54323, -13790, -29749, -16395},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30325, -14660, 56218}
    },
    /* Vector 638: Angle -1.184 rad */
    {
        .input_axis = {27280, 51882, 29309},
        .input_angle_rad = -77610,
        .expected_quaternion = {54379, -15225, -28957, -16358},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31782, -13692, 55654}
    },
    /* Vector 639: Angle -1.181 rad */
    {
        .input_axis = {29839, 50454, 29309},
        .input_angle_rad = -77409,
        .expected_quaternion = {54435, -16616, -28095, -16320},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33319, -12866, 54948}
    },
    /* Vector 640: Angle -1.178 rad */
    {
        .input_axis = {32323, 48900, 29309},
        .input_angle_rad = -77208,
        .expected_quaternion = {54491, -17958, -27167, -16283},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34921, -12189, 54101}
    },
    /* Vector 641: Angle -1.175 rad */
    {
        .input_axis = {34727, 47223, 29309},
        .input_angle_rad = -77007,
        .expected_quaternion = {54547, -19249, -26176, -16246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36572, -11667, 53116}
    },
    /* Vector 642: Angle -1.172 rad */
    {
        .input_axis = {37043, 45429, 29309},
        .input_angle_rad = -76806,
        .expected_quaternion = {54603, -20486, -25123, -16208},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38258, -11302, 51996}
    },
    /* Vector 643: Angle -1.169 rad */
    {
        .input_axis = {39268, 43520, 29309},
        .input_angle_rad = -76605,
        .expected_quaternion = {54658, -21665, -24012, -16171},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39960, -11097, 50744}
    },
    /* Vector 644: Angle -1.166 rad */
    {
        .input_axis = {41394, 41503, 29309},
        .input_angle_rad = -76404,
        .expected_quaternion = {54714, -22785, -22846, -16133},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41665, -11052, 49365}
    },
    /* Vector 645: Angle -1.163 rad */
    {
        .input_axis = {43416, 39383, 29309},
        .input_angle_rad = -76202,
        .expected_quaternion = {54769, -23843, -21628, -16096},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43355, -11165, 47861}
    },
    /* Vector 646: Angle -1.160 rad */
    {
        .input_axis = {45330, 37164, 29309},
        .input_angle_rad = -76001,
        .expected_quaternion = {54824, -24836, -20362, -16058},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45014, -11434, 46238}
    },
    /* Vector 647: Angle -1.157 rad */
    {
        .input_axis = {47131, 34852, 29309},
        .input_angle_rad = -75800,
        .expected_quaternion = {54879, -25762, -19050, -16020},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46628, -11853, 44500}
    },
    /* Vector 648: Angle -1.154 rad */
    {
        .input_axis = {48814, 32453, 29309},
        .input_angle_rad = -75599,
        .expected_quaternion = {54934, -26620, -17697, -15983},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48183, -12418, 42652}
    },
    /* Vector 649: Angle -1.150 rad */
    {
        .input_axis = {50375, 29972, 29309},
        .input_angle_rad = -75398,
        .expected_quaternion = {54989, -27406, -16306, -15945},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49663, -13120, 40699}
    },
    /* Vector 650: Angle -1.147 rad */
    {
        .input_axis = {51810, 27417, 29309},
        .input_angle_rad = -75197,
        .expected_quaternion = {55043, -28120, -14881, -15907},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51056, -13951, 38647}
    },
    /* Vector 651: Angle -1.144 rad */
    {
        .input_axis = {53116, 24793, 29309},
        .input_angle_rad = -74996,
        .expected_quaternion = {55098, -28760, -13425, -15869},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52350, -14901, 36501}
    },
    /* Vector 652: Angle -1.141 rad */
    {
        .input_axis = {54288, 22108, 29309},
        .input_angle_rad = -74795,
        .expected_quaternion = {55152, -29325, -11942, -15832},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53535, -15959, 34268}
    },
    /* Vector 653: Angle -1.138 rad */
    {
        .input_axis = {55325, 19367, 29309},
        .input_angle_rad = -74594,
        .expected_quaternion = {55206, -29814, -10436, -15794},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54600, -17113, 31953}
    },
    /* Vector 654: Angle -1.135 rad */
    {
        .input_axis = {56224, 16578, 29309},
        .input_angle_rad = -74393,
        .expected_quaternion = {55260, -30225, -8912, -15756},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55536, -18351, 29562}
    },
    /* Vector 655: Angle -1.132 rad */
    {
        .input_axis = {56982, 13747, 29309},
        .input_angle_rad = -74192,
        .expected_quaternion = {55314, -30559, -7372, -15718},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56338, -19658, 27103}
    },
    /* Vector 656: Angle -1.129 rad */
    {
        .input_axis = {57598, 10882, 29309},
        .input_angle_rad = -73991,
        .expected_quaternion = {55368, -30815, -5822, -15680},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56999, -21020, 24582}
    },
    /* Vector 657: Angle -1.126 rad */
    {
        .input_axis = {58070, 7989, 29309},
        .input_angle_rad = -73790,
        .expected_quaternion = {55422, -30992, -4264, -15642},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57514, -22423, 22006}
    },
    /* Vector 658: Angle -1.123 rad */
    {
        .input_axis = {58397, 5077, 29309},
        .input_angle_rad = -73589,
        .expected_quaternion = {55476, -31091, -2703, -15604},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57882, -23853, 19381}
    },
    /* Vector 659: Angle -1.120 rad */
    {
        .input_axis = {58578, 2152, 29309},
        .input_angle_rad = -73388,
        .expected_quaternion = {55529, -31111, -1143, -15566},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58102, -25293, 16716}
    },
    /* Vector 660: Angle -1.117 rad */
    {
        .input_axis = {58612, -778, 29309},
        .input_angle_rad = -73187,
        .expected_quaternion = {55582, -31053, 412, -15528},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58173, -26730, 14016}
    },
    /* Vector 661: Angle -1.114 rad */
    {
        .input_axis = {58500, -3707, 29309},
        .input_angle_rad = -72985,
        .expected_quaternion = {55636, -30917, 1959, -15490},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58097, -28148, 11289}
    },
    /* Vector 662: Angle -1.111 rad */
    {
        .input_axis = {58242, -6626, 29309},
        .input_angle_rad = -72784,
        .expected_quaternion = {55689, -30705, 3493, -15451},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57878, -29533, 8542}
    },
    /* Vector 663: Angle -1.108 rad */
    {
        .input_axis = {57838, -9528, 29309},
        .input_angle_rad = -72583,
        .expected_quaternion = {55742, -30416, 5011, -15413},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57520, -30871, 5783}
    },
    /* Vector 664: Angle -1.104 rad */
    {
        .input_axis = {57289, -12407, 29309},
        .input_angle_rad = -72382,
        .expected_quaternion = {55794, -30053, 6509, -15375},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57029, -32149, 3019}
    },
    /* Vector 665: Angle -1.101 rad */
    {
        .input_axis = {56597, -15255, 29309},
        .input_angle_rad = -72181,
        .expected_quaternion = {55847, -29616, 7983, -15337},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56413, -33353, 257}
    },
    /* Vector 666: Angle -1.098 rad */
    {
        .input_axis = {55764, -18065, 29309},
        .input_angle_rad = -71980,
        .expected_quaternion = {55900, -29108, 9429, -15298},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55680, -34474, -2496}
    },
    /* Vector 667: Angle -1.095 rad */
    {
        .input_axis = {54792, -20829, 29309},
        .input_angle_rad = -71779,
        .expected_quaternion = {55952, -28528, 10845, -15260},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54840, -35498, -5232}
    },
    /* Vector 668: Angle -1.092 rad */
    {
        .input_axis = {53682, -23541, 29309},
        .input_angle_rad = -71578,
        .expected_quaternion = {56004, -27880, 12226, -15222},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53903, -36418, -7945}
    },
    /* Vector 669: Angle -1.089 rad */
    {
        .input_axis = {52439, -26195, 29309},
        .input_angle_rad = -71377,
        .expected_quaternion = {56056, -27165, 13570, -15183},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52881, -37224, -10627}
    },
    /* Vector 670: Angle -1.086 rad */
    {
        .input_axis = {51064, -28783, 29309},
        .input_angle_rad = -71176,
        .expected_quaternion = {56108, -26386, 14873, -15145},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51786, -37909, -13272}
    },
    /* Vector 671: Angle -1.083 rad */
    {
        .input_axis = {49561, -31299, 29309},
        .input_angle_rad = -70975,
        .expected_quaternion = {56160, -25545, 16132, -15106},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50630, -38466, -15872}
    },
    /* Vector 672: Angle -1.080 rad */
    {
        .input_axis = {47935, -33737, 29309},
        .input_angle_rad = -70774,
        .expected_quaternion = {56212, -24644, 17344, -15068},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49427, -38892, -18422}
    },
    /* Vector 673: Angle -1.077 rad */
    {
        .input_axis = {46189, -36091, 29309},
        .input_angle_rad = -70573,
        .expected_quaternion = {56264, -23685, 18507, -15029},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48191, -39182, -20914}
    },
    /* Vector 674: Angle -1.074 rad */
    {
        .input_axis = {44328, -38354, 29309},
        .input_angle_rad = -70372,
        .expected_quaternion = {56315, -22672, 19617, -14990},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46934, -39336, -23342}
    },
    /* Vector 675: Angle -1.071 rad */
    {
        .input_axis = {42355, -40522, 29309},
        .input_angle_rad = -70171,
        .expected_quaternion = {56367, -21608, 20672, -14952},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45672, -39351, -25700}
    },
    /* Vector 676: Angle -1.068 rad */
    {
        .input_axis = {40277, -42588, 29309},
        .input_angle_rad = -69970,
        .expected_quaternion = {56418, -20494, 21670, -14913},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44418, -39230, -27983}
    },
    /* Vector 677: Angle -1.065 rad */
    {
        .input_axis = {38098, -44548, 29309},
        .input_angle_rad = -69768,
        .expected_quaternion = {56469, -19335, 22608, -14874},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43185, -38973, -30184}
    },
    /* Vector 678: Angle -1.062 rad */
    {
        .input_axis = {35824, -46396, 29309},
        .input_angle_rad = -69567,
        .expected_quaternion = {56520, -18134, 23485, -14836},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41987, -38586, -32298}
    },
    /* Vector 679: Angle -1.058 rad */
    {
        .input_axis = {33461, -48129, 29309},
        .input_angle_rad = -69366,
        .expected_quaternion = {56571, -16893, 24298, -14797},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40836, -38072, -34320}
    },
    /* Vector 680: Angle -1.055 rad */
    {
        .input_axis = {31013, -49741, 29309},
        .input_angle_rad = -69165,
        .expected_quaternion = {56621, -15616, 25046, -14758},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39745, -37438, -36246}
    },
    /* Vector 681: Angle -1.052 rad */
    {
        .input_axis = {28489, -51229, 29309},
        .input_angle_rad = -68964,
        .expected_quaternion = {56672, -14307, 25728, -14719},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38724, -36690, -38069}
    },
    /* Vector 682: Angle -1.049 rad */
    {
        .input_axis = {25893, -52588, 29309},
        .input_angle_rad = -68763,
        .expected_quaternion = {56722, -12969, 26341, -14680},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37785, -35837, -39786}
    },
    /* Vector 683: Angle -1.046 rad */
    {
        .input_axis = {23232, -53817, 29309},
        .input_angle_rad = -68562,
        .expected_quaternion = {56773, -11606, 26885, -14641},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36936, -34889, -41394}
    },
    /* Vector 684: Angle -1.043 rad */
    {
        .input_axis = {20513, -54911, 29309},
        .input_angle_rad = -68361,
        .expected_quaternion = {56823, -10220, 27358, -14602},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36188, -33855, -42887}
    },
    /* Vector 685: Angle -1.040 rad */
    {
        .input_axis = {17743, -55867, 29309},
        .input_angle_rad = -68160,
        .expected_quaternion = {56873, -8817, 27760, -14563},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35546, -32746, -44263}
    },
    /* Vector 686: Angle -1.037 rad */
    {
        .input_axis = {14929, -56684, 29309},
        .input_angle_rad = -67959,
        .expected_quaternion = {56923, -7398, 28091, -14524},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35017, -31573, -45518}
    },
    /* Vector 687: Angle -1.034 rad */
    {
        .input_axis = {12077, -57360, 29309},
        .input_angle_rad = -67758,
        .expected_quaternion = {56972, -5969, 28349, -14485},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34607, -30349, -46651}
    },
    /* Vector 688: Angle -1.031 rad */
    {
        .input_axis = {9195, -57891, 29309},
        .input_angle_rad = -67557,
        .expected_quaternion = {57022, -4532, 28535, -14446},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34319, -29086, -47657}
    },
    /* Vector 689: Angle -1.028 rad */
    {
        .input_axis = {6290, -58279, 29309},
        .input_angle_rad = -67356,
        .expected_quaternion = {57072, -3092, 28648, -14407},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34156, -27796, -48536}
    },
    /* Vector 690: Angle -1.025 rad */
    {
        .input_axis = {3370, -58520, 29309},
        .input_angle_rad = -67155,
        .expected_quaternion = {57121, -1652, 28688, -14368},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34120, -26492, -49285}
    },
    /* Vector 691: Angle -1.022 rad */
    {
        .input_axis = {441, -58616, 29309},
        .input_angle_rad = -66954,
        .expected_quaternion = {57170, -215, 28656, -14329},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34210, -25188, -49903}
    },
    /* Vector 692: Angle -1.019 rad */
    {
        .input_axis = {-2489, -58564, 29309},
        .input_angle_rad = -66753,
        .expected_quaternion = {57219, 1214, 28553, -14289},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34424, -23894, -50388}
    },
    /* Vector 693: Angle -1.015 rad */
    {
        .input_axis = {-5413, -58367, 29309},
        .input_angle_rad = -66551,
        .expected_quaternion = {57268, 2632, 28378, -14250},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34762, -22625, -50741}
    },
    /* Vector 694: Angle -1.012 rad */
    {
        .input_axis = {-8324, -58023, 29309},
        .input_angle_rad = -66350,
        .expected_quaternion = {57317, 4036, 28134, -14211},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35218, -21392, -50961}
    },
    /* Vector 695: Angle -1.009 rad */
    {
        .input_axis = {-11213, -57535, 29309},
        .input_angle_rad = -66149,
        .expected_quaternion = {57366, 5422, 27820, -14171},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35789, -20206, -51047}
    },
    /* Vector 696: Angle -1.006 rad */
    {
        .input_axis = {-14075, -56902, 29309},
        .input_angle_rad = -65948,
        .expected_quaternion = {57414, 6787, 27437, -14132},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36467, -19079, -51001}
    },
    /* Vector 697: Angle -1.003 rad */
    {
        .input_axis = {-16901, -56128, 29309},
        .input_angle_rad = -65747,
        .expected_quaternion = {57463, 8127, 26988, -14093},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37247, -18020, -50823}
    },
    /* Vector 698: Angle -1.000 rad */
    {
        .input_axis = {-19685, -55213, 29309},
        .input_angle_rad = -65546,
        .expected_quaternion = {57511, 9439, 26474, -14053},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38120, -17039, -50513}
    },
    /* Vector 699: Angle -0.997 rad */
    {
        .input_axis = {-22420, -54160, 29309},
        .input_angle_rad = -65345,
        .expected_quaternion = {57559, 10720, 25896, -14014},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39077, -16144, -50073}
    },
    /* Vector 700: Angle -0.994 rad */
    {
        .input_axis = {-25099, -52972, 29309},
        .input_angle_rad = -65144,
        .expected_quaternion = {57607, 11967, 25257, -13974},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40109, -15343, -49506}
    },
    /* Vector 701: Angle -0.991 rad */
    {
        .input_axis = {-27715, -51651, 29309},
        .input_angle_rad = -64943,
        .expected_quaternion = {57655, 13177, 24558, -13935},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41206, -14643, -48812}
    },
    /* Vector 702: Angle -0.988 rad */
    {
        .input_axis = {-30262, -50202, 29309},
        .input_angle_rad = -64742,
        .expected_quaternion = {57703, 14347, 23801, -13895},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42357, -14048, -47995}
    },
    /* Vector 703: Angle -0.985 rad */
    {
        .input_axis = {-32733, -48626, 29309},
        .input_angle_rad = -64541,
        .expected_quaternion = {57750, 15475, 22988, -13856},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43550, -13563, -47057}
    },
    /* Vector 704: Angle -0.982 rad */
    {
        .input_axis = {-35122, -46930, 29309},
        .input_angle_rad = -64340,
        .expected_quaternion = {57798, 16557, 22122, -13816},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44775, -13191, -46001}
    },
    /* Vector 705: Angle -0.979 rad */
    {
        .input_axis = {-37424, -45116, 29309},
        .input_angle_rad = -64139,
        .expected_quaternion = {57845, 17591, 21206, -13776},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46020, -12935, -44831}
    },
    /* Vector 706: Angle -0.976 rad */
    {
        .input_axis = {-39632, -43189, 29309},
        .input_angle_rad = -63938,
        .expected_quaternion = {57892, 18575, 20242, -13737},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47273, -12794, -43549}
    },
    /* Vector 707: Angle -0.973 rad */
    {
        .input_axis = {-41741, -41154, 29309},
        .input_angle_rad = -63737,
        .expected_quaternion = {57939, 19507, 19233, -13697},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48522, -12769, -42160}
    },
    /* Vector 708: Angle -0.969 rad */
    {
        .input_axis = {-43746, -39016, 29309},
        .input_angle_rad = -63536,
        .expected_quaternion = {57986, 20385, 18181, -13657},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49757, -12858, -40668}
    },
    /* Vector 709: Angle -0.966 rad */
    {
        .input_axis = {-45641, -36781, 29309},
        .input_angle_rad = -63335,
        .expected_quaternion = {58033, 21206, 17089, -13617},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50965, -13057, -39078}
    },
    /* Vector 710: Angle -0.963 rad */
    {
        .input_axis = {-47422, -34454, 29309},
        .input_angle_rad = -63133,
        .expected_quaternion = {58079, 21969, 15961, -13577},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52135, -13364, -37393}
    },
    /* Vector 711: Angle -0.960 rad */
    {
        .input_axis = {-49085, -32041, 29309},
        .input_angle_rad = -62932,
        .expected_quaternion = {58126, 22672, 14800, -13538},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53259, -13774, -35619}
    },
    /* Vector 712: Angle -0.957 rad */
    {
        .input_axis = {-50625, -29548, 29309},
        .input_angle_rad = -62731,
        .expected_quaternion = {58172, 23315, 13608, -13498},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54325, -14280, -33762}
    },
    /* Vector 713: Angle -0.954 rad */
    {
        .input_axis = {-52039, -26981, 29309},
        .input_angle_rad = -62530,
        .expected_quaternion = {58219, 23895, 12389, -13458},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55325, -14876, -31825}
    },
    /* Vector 714: Angle -0.951 rad */
    {
        .input_axis = {-53322, -24346, 29309},
        .input_angle_rad = -62329,
        .expected_quaternion = {58265, 24412, 11146, -13418},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56250, -15555, -29815}
    },
    /* Vector 715: Angle -0.948 rad */
    {
        .input_axis = {-54472, -21651, 29309},
        .input_angle_rad = -62128,
        .expected_quaternion = {58311, 24864, 9882, -13378},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57094, -16307, -27737}
    },
    /* Vector 716: Angle -0.945 rad */
    {
        .input_axis = {-55486, -18901, 29309},
        .input_angle_rad = -61927,
        .expected_quaternion = {58356, 25251, 8602, -13338},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57849, -17125, -25597}
    },
    /* Vector 717: Angle -0.942 rad */
    {
        .input_axis = {-56362, -16104, 29309},
        .input_angle_rad = -61726,
        .expected_quaternion = {58402, 25572, 7307, -13298},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58510, -17998, -23400}
    },
    /* Vector 718: Angle -0.939 rad */
    {
        .input_axis = {-57096, -13267, 29309},
        .input_angle_rad = -61525,
        .expected_quaternion = {58448, 25827, 6001, -13258},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59073, -18917, -21154}
    },
    /* Vector 719: Angle -0.936 rad */
    {
        .input_axis = {-57688, -10397, 29309},
        .input_angle_rad = -61324,
        .expected_quaternion = {58493, 26016, 4689, -13218},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59533, -19872, -18864}
    },
    /* Vector 720: Angle -0.933 rad */
    {
        .input_axis = {-58135, -7501, 29309},
        .input_angle_rad = -61123,
        .expected_quaternion = {58538, 26138, 3372, -13177},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59890, -20851, -16536}
    },
    /* Vector 721: Angle -0.930 rad */
    {
        .input_axis = {-58438, -4586, 29309},
        .input_angle_rad = -60922,
        .expected_quaternion = {58583, 26194, 2056, -13137},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60140, -21844, -14177}
    },
    /* Vector 722: Angle -0.927 rad */
    {
        .input_axis = {-58594, -1660, 29309},
        .input_angle_rad = -60721,
        .expected_quaternion = {58628, 26184, 742, -13097},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60284, -22841, -11792}
    },
    /* Vector 723: Angle -0.923 rad */
    {
        .input_axis = {-58603, 1271, 29309},
        .input_angle_rad = -60520,
        .expected_quaternion = {58673, 26108, -566, -13057},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60324, -23830, -9389}
    },
    /* Vector 724: Angle -0.920 rad */
    {
        .input_axis = {-58467, 4198, 29309},
        .input_angle_rad = -60319,
        .expected_quaternion = {58718, 25966, -1865, -13017},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60259, -24802, -6974}
    },
    /* Vector 725: Angle -0.917 rad */
    {
        .input_axis = {-58184, 7115, 29309},
        .input_angle_rad = -60118,
        .expected_quaternion = {58763, 25761, -3150, -12976},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60094, -25747, -4552}
    },
    /* Vector 726: Angle -0.914 rad */
    {
        .input_axis = {-57755, 10014, 29309},
        .input_angle_rad = -59916,
        .expected_quaternion = {58807, 25492, -4420, -12936},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59833, -26654, -2131}
    },
    /* Vector 727: Angle -0.911 rad */
    {
        .input_axis = {-57183, 12888, 29309},
        .input_angle_rad = -59715,
        .expected_quaternion = {58851, 25160, -5671, -12896},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59480, -27515, 283}
    },
    /* Vector 728: Angle -0.908 rad */
    {
        .input_axis = {-56467, 15730, 29309},
        .input_angle_rad = -59514,
        .expected_quaternion = {58896, 24767, -6900, -12855},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59040, -28320, 2684}
    },
    /* Vector 729: Angle -0.905 rad */
    {
        .input_axis = {-55610, 18533, 29309},
        .input_angle_rad = -59313,
        .expected_quaternion = {58940, 24315, -8103, -12815},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58521, -29063, 5066}
    },
    /* Vector 730: Angle -0.902 rad */
    {
        .input_axis = {-54615, 21289, 29309},
        .input_angle_rad = -59112,
        .expected_quaternion = {58983, 23804, -9279, -12774},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57929, -29735, 7422}
    },
    /* Vector 731: Angle -0.899 rad */
    {
        .input_axis = {-53482, 23992, 29309},
        .input_angle_rad = -58911,
        .expected_quaternion = {59027, 23237, -10424, -12734},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57272, -30330, 9747}
    },
    /* Vector 732: Angle -0.896 rad */
    {
        .input_axis = {-52216, 26635, 29309},
        .input_angle_rad = -58710,
        .expected_quaternion = {59071, 22615, -11535, -12693},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56558, -30843, 12035}
    },
    /* Vector 733: Angle -0.893 rad */
    {
        .input_axis = {-50820, 29211, 29309},
        .input_angle_rad = -58509,
        .expected_quaternion = {59114, 21940, -12611, -12653},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55797, -31270, 14279}
    },
    /* Vector 734: Angle -0.890 rad */
    {
        .input_axis = {-49297, 31715, 29309},
        .input_angle_rad = -58308,
        .expected_quaternion = {59158, 21214, -13648, -12612},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54997, -31605, 16474}
    },
    /* Vector 735: Angle -0.887 rad */
    {
        .input_axis = {-47650, 34139, 29309},
        .input_angle_rad = -58107,
        .expected_quaternion = {59201, 20439, -14644, -12572},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54169, -31847, 18615}
    },
    /* Vector 736: Angle -0.884 rad */
    {
        .input_axis = {-45884, 36478, 29309},
        .input_angle_rad = -57906,
        .expected_quaternion = {59244, 19618, -15596, -12531},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53321, -31993, 20695}
    },
    /* Vector 737: Angle -0.881 rad */
    {
        .input_axis = {-44004, 38725, 29309},
        .input_angle_rad = -57705,
        .expected_quaternion = {59287, 18753, -16504, -12490},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52463, -32044, 22712}
    },
    /* Vector 738: Angle -0.877 rad */
    {
        .input_axis = {-42013, 40876, 29309},
        .input_angle_rad = -57504,
        .expected_quaternion = {59330, 17846, -17363, -12450},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51605, -31998, 24658}
    },
    /* Vector 739: Angle -0.874 rad */
    {
        .input_axis = {-39918, 42925, 29309},
        .input_angle_rad = -57303,
        .expected_quaternion = {59372, 16901, -18174, -12409},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50757, -31857, 26529}
    },
    /* Vector 740: Angle -0.871 rad */
    {
        .input_axis = {-37722, 44866, 29309},
        .input_angle_rad = -57102,
        .expected_quaternion = {59415, 15919, -18934, -12368},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49928, -31624, 28322}
    },
    /* Vector 741: Angle -0.868 rad */
    {
        .input_axis = {-35433, 46696, 29309},
        .input_angle_rad = -56901,
        .expected_quaternion = {59457, 14903, -19641, -12327},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49126, -31301, 30031}
    },
    /* Vector 742: Angle -0.865 rad */
    {
        .input_axis = {-33055, 48408, 29309},
        .input_angle_rad = -56699,
        .expected_quaternion = {59499, 13857, -20294, -12287},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48361, -30892, 31653}
    },
    /* Vector 743: Angle -0.862 rad */
    {
        .input_axis = {-30594, 50000, 29309},
        .input_angle_rad = -56498,
        .expected_quaternion = {59541, 12783, -20891, -12246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47641, -30401, 33183}
    },
    /* Vector 744: Angle -0.859 rad */
    {
        .input_axis = {-28057, 51466, 29309},
        .input_angle_rad = -56297,
        .expected_quaternion = {59583, 11684, -21432, -12205},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46972, -29835, 34619}
    },
    /* Vector 745: Angle -0.856 rad */
    {
        .input_axis = {-25450, 52804, 29309},
        .input_angle_rad = -56096,
        .expected_quaternion = {59625, 10562, -21916, -12164},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46363, -29198, 35957}
    },
    /* Vector 746: Angle -0.853 rad */
    {
        .input_axis = {-22779, 54010, 29309},
        .input_angle_rad = -55895,
        .expected_quaternion = {59667, 9422, -22341, -12123},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45819, -28499, 37194}
    },
    /* Vector 747: Angle -0.850 rad */
    {
        .input_axis = {-20051, 55081, 29309},
        .input_angle_rad = -55694,
        .expected_quaternion = {59708, 8266, -22707, -12082},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45346, -27743, 38327}
    },
    /* Vector 748: Angle -0.847 rad */
    {
        .input_axis = {-17273, 56014, 29309},
        .input_angle_rad = -55493,
        .expected_quaternion = {59750, 7096, -23013, -12041},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44949, -26940, 39355}
    },
    /* Vector 749: Angle -0.844 rad */
    {
        .input_axis = {-14452, 56808, 29309},
        .input_angle_rad = -55292,
        .expected_quaternion = {59791, 5917, -23260, -12000},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44631, -26097, 40274}
    },
    /* Vector 750: Angle -0.841 rad */
    {
        .input_axis = {-11594, 57459, 29309},
        .input_angle_rad = -55091,
        .expected_quaternion = {59832, 4731, -23446, -11959},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44396, -25222, 41084}
    },
    /* Vector 751: Angle -0.838 rad */
    {
        .input_axis = {-8708, 57967, 29309},
        .input_angle_rad = -54890,
        .expected_quaternion = {59873, 3541, -23572, -11918},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44245, -24324, 41782}
    },
    /* Vector 752: Angle -0.834 rad */
    {
        .input_axis = {-5800, 58330, 29309},
        .input_angle_rad = -54689,
        .expected_quaternion = {59914, 2350, -23638, -11877},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44180, -23412, 42367}
    },
    /* Vector 753: Angle -0.831 rad */
    {
        .input_axis = {-2878, 58547, 29309},
        .input_angle_rad = -54488,
        .expected_quaternion = {59954, 1162, -23643, -11836},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44201, -22494, 42840}
    },
    /* Vector 754: Angle -0.828 rad */
    {
        .input_axis = {52, 58617, 29309},
        .input_angle_rad = -54287,
        .expected_quaternion = {59995, -21, -23590, -11795},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44308, -21580, 43198}
    },
    /* Vector 755: Angle -0.825 rad */
    {
        .input_axis = {2982, 58541, 29309},
        .input_angle_rad = -54086,
        .expected_quaternion = {60035, -1196, -23477, -11754},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44500, -20677, 43441}
    },
    /* Vector 756: Angle -0.822 rad */
    {
        .input_axis = {5904, 58319, 29309},
        .input_angle_rad = -53885,
        .expected_quaternion = {60075, -2359, -23306, -11712},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44774, -19795, 43571}
    },
    /* Vector 757: Angle -0.819 rad */
    {
        .input_axis = {8811, 57951, 29309},
        .input_angle_rad = -53684,
        .expected_quaternion = {60116, -3509, -23077, -11671},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45127, -18941, 43587}
    },
    /* Vector 758: Angle -0.816 rad */
    {
        .input_axis = {11696, 57438, 29309},
        .input_angle_rad = -53482,
        .expected_quaternion = {60156, -4641, -22792, -11630},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45555, -18122, 43489}
    },
    /* Vector 759: Angle -0.813 rad */
    {
        .input_axis = {14553, 56782, 29309},
        .input_angle_rad = -53281,
        .expected_quaternion = {60195, -5754, -22452, -11589},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46054, -17346, 43279}
    },
    /* Vector 760: Angle -0.810 rad */
    {
        .input_axis = {17372, 55984, 29309},
        .input_angle_rad = -53080,
        .expected_quaternion = {60235, -6845, -22057, -11547},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46619, -16619, 42958}
    },
    /* Vector 761: Angle -0.807 rad */
    {
        .input_axis = {20149, 55046, 29309},
        .input_angle_rad = -52879,
        .expected_quaternion = {60275, -7910, -21610, -11506},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47245, -15948, 42527}
    },
    /* Vector 762: Angle -0.804 rad */
    {
        .input_axis = {22875, 53970, 29309},
        .input_angle_rad = -52678,
        .expected_quaternion = {60314, -8948, -21111, -11465},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47923, -15337, 41989}
    },
    /* Vector 763: Angle -0.801 rad */
    {
        .input_axis = {25543, 52759, 29309},
        .input_angle_rad = -52477,
        .expected_quaternion = {60353, -9956, -20563, -11423},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48649, -14792, 41345}
    },
    /* Vector 764: Angle -0.798 rad */
    {
        .input_axis = {28148, 51416, 29309},
        .input_angle_rad = -52276,
        .expected_quaternion = {60392, -10931, -19967, -11382},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49415, -14316, 40597}
    },
    /* Vector 765: Angle -0.795 rad */
    {
        .input_axis = {30683, 49945, 29309},
        .input_angle_rad = -52075,
        .expected_quaternion = {60431, -11872, -19325, -11340},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50214, -13912, 39749}
    },
    /* Vector 766: Angle -0.792 rad */
    {
        .input_axis = {33141, 48349, 29309},
        .input_angle_rad = -51874,
        .expected_quaternion = {60470, -12776, -18639, -11299},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51037, -13584, 38803}
    },
    /* Vector 767: Angle -0.788 rad */
    {
        .input_axis = {35516, 46633, 29309},
        .input_angle_rad = -51673,
        .expected_quaternion = {60509, -13642, -17912, -11257},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51878, -13331, 37762}
    },
    /* Vector 768: Angle -0.785 rad */
    {
        .input_axis = {37802, 44799, 29309},
        .input_angle_rad = -51472,
        .expected_quaternion = {60547, -14466, -17144, -11216},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52727, -13156, 36629}
    },
    /* Vector 769: Angle -0.782 rad */
    {
        .input_axis = {39994, 42854, 29309},
        .input_angle_rad = -51271,
        .expected_quaternion = {60586, -15248, -16339, -11174},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53579, -13058, 35409}
    },
    /* Vector 770: Angle -0.779 rad */
    {
        .input_axis = {42086, 40802, 29309},
        .input_angle_rad = -51070,
        .expected_quaternion = {60624, -15986, -15498, -11133},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54423, -13036, 34105}
    },
    /* Vector 771: Angle -0.776 rad */
    {
        .input_axis = {44072, 38647, 29309},
        .input_angle_rad = -50869,
        .expected_quaternion = {60662, -16678, -14625, -11091},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55254, -13089, 32720}
    },
    /* Vector 772: Angle -0.773 rad */
    {
        .input_axis = {45949, 36396, 29309},
        .input_angle_rad = -50668,
        .expected_quaternion = {60700, -17323, -13722, -11050},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56064, -13214, 31260}
    },
    /* Vector 773: Angle -0.770 rad */
    {
        .input_axis = {47710, 34054, 29309},
        .input_angle_rad = -50467,
        .expected_quaternion = {60738, -17919, -12790, -11008},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56846, -13410, 29728}
    },
    /* Vector 774: Angle -0.767 rad */
    {
        .input_axis = {49353, 31627, 29309},
        .input_angle_rad = -50265,
        .expected_quaternion = {60776, -18466, -11834, -10966},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57592, -13671, 28128}
    },
    /* Vector 775: Angle -0.764 rad */
    {
        .input_axis = {50872, 29121, 29309},
        .input_angle_rad = -50064,
        .expected_quaternion = {60813, -18962, -10855, -10925},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58298, -13993, 26466}
    },
    /* Vector 776: Angle -0.761 rad */
    {
        .input_axis = {52264, 26542, 29309},
        .input_angle_rad = -49863,
        .expected_quaternion = {60851, -19406, -9856, -10883},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58957, -14373, 24747}
    },
    /* Vector 777: Angle -0.758 rad */
    {
        .input_axis = {53525, 23897, 29309},
        .input_angle_rad = -49662,
        .expected_quaternion = {60888, -19798, -8839, -10841},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59565, -14804, 22975}
    },
    /* Vector 778: Angle -0.755 rad */
    {
        .input_axis = {54652, 21192, 29309},
        .input_angle_rad = -49461,
        .expected_quaternion = {60925, -20138, -7808, -10799},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60116, -15280, 21155}
    },
    /* Vector 779: Angle -0.752 rad */
    {
        .input_axis = {55643, 18434, 29309},
        .input_angle_rad = -49260,
        .expected_quaternion = {60962, -20423, -6766, -10757},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60607, -15796, 19292}
    },
    /* Vector 780: Angle -0.749 rad */
    {
        .input_axis = {56495, 15630, 29309},
        .input_angle_rad = -49059,
        .expected_quaternion = {60999, -20655, -5714, -10716},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61035, -16345, 17392}
    },
    /* Vector 781: Angle -0.746 rad */
    {
        .input_axis = {57206, 12787, 29309},
        .input_angle_rad = -48858,
        .expected_quaternion = {61035, -20833, -4657, -10674},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61397, -16921, 15460}
    },
    /* Vector 782: Angle -0.742 rad */
    {
        .input_axis = {57773, 9912, 29309},
        .input_angle_rad = -48657,
        .expected_quaternion = {61072, -20958, -3596, -10632},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61692, -17516, 13501}
    },
    /* Vector 783: Angle -0.739 rad */
    {
        .input_axis = {58196, 7012, 29309},
        .input_angle_rad = -48456,
        .expected_quaternion = {61108, -21028, -2534, -10590},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61918, -18123, 11521}
    },
    /* Vector 784: Angle -0.736 rad */
    {
        .input_axis = {58474, 4094, 29309},
        .input_angle_rad = -48255,
        .expected_quaternion = {61145, -21045, -1474, -10548},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62074, -18736, 9524}
    },
    /* Vector 785: Angle -0.733 rad */
    {
        .input_axis = {58606, 1167, 29309},
        .input_angle_rad = -48054,
        .expected_quaternion = {61181, -21008, -418, -10506},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62162, -19348, 7517}
    },
    /* Vector 786: Angle -0.730 rad */
    {
        .input_axis = {58591, -1764, 29309},
        .input_angle_rad = -47853,
        .expected_quaternion = {61217, -20919, 630, -10464},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62182, -19951, 5504}
    },
    /* Vector 787: Angle -0.727 rad */
    {
        .input_axis = {58429, -4690, 29309},
        .input_angle_rad = -47652,
        .expected_quaternion = {61253, -20777, 1668, -10422},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62136, -20539, 3491}
    },
    /* Vector 788: Angle -0.724 rad */
    {
        .input_axis = {58122, -7604, 29309},
        .input_angle_rad = -47451,
        .expected_quaternion = {61288, -20585, 2693, -10380},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62027, -21106, 1484}
    },
    /* Vector 789: Angle -0.721 rad */
    {
        .input_axis = {57669, -10500, 29309},
        .input_angle_rad = -47250,
        .expected_quaternion = {61324, -20342, 3703, -10338},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61856, -21646, -513}
    },
    /* Vector 790: Angle -0.718 rad */
    {
        .input_axis = {57072, -13369, 29309},
        .input_angle_rad = -47048,
        .expected_quaternion = {61359, -20049, 4696, -10296},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61628, -22153, -2494}
    },
    /* Vector 791: Angle -0.715 rad */
    {
        .input_axis = {56333, -16204, 29309},
        .input_angle_rad = -46847,
        .expected_quaternion = {61394, -19708, 5669, -10254},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61347, -22621, -4455}
    },
    /* Vector 792: Angle -0.712 rad */
    {
        .input_axis = {55453, -19000, 29309},
        .input_angle_rad = -46646,
        .expected_quaternion = {61429, -19321, 6620, -10212},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61016, -23047, -6389}
    },
    /* Vector 793: Angle -0.709 rad */
    {
        .input_axis = {54434, -21747, 29309},
        .input_angle_rad = -46445,
        .expected_quaternion = {61464, -18887, 7546, -10170},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60642, -23425, -8292}
    },
    /* Vector 794: Angle -0.706 rad */
    {
        .input_axis = {53279, -24441, 29309},
        .input_angle_rad = -46244,
        .expected_quaternion = {61499, -18410, 8445, -10127},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60229, -23752, -10160}
    },
    /* Vector 795: Angle -0.703 rad */
    {
        .input_axis = {51991, -27073, 29309},
        .input_angle_rad = -46043,
        .expected_quaternion = {61534, -17890, 9316, -10085},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59784, -24025, -11988}
    },
    /* Vector 796: Angle -0.699 rad */
    {
        .input_axis = {50573, -29638, 29309},
        .input_angle_rad = -45842,
        .expected_quaternion = {61568, -17329, 10156, -10043},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59311, -24241, -13770}
    },
    /* Vector 797: Angle -0.696 rad */
    {
        .input_axis = {49028, -32128, 29309},
        .input_angle_rad = -45641,
        .expected_quaternion = {61603, -16729, 10963, -10001},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58816, -24398, -15504}
    },
    /* Vector 798: Angle -0.693 rad */
    {
        .input_axis = {47361, -34538, 29309},
        .input_angle_rad = -45440,
        .expected_quaternion = {61637, -16092, 11735, -9958},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58307, -24495, -17184}
    },
    /* Vector 799: Angle -0.690 rad */
    {
        .input_axis = {45576, -36862, 29309},
        .input_angle_rad = -45239,
        .expected_quaternion = {61671, -15420, 12472, -9916},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57788, -24531, -18806}
    },
    /* Vector 800: Angle -0.687 rad */
    {
        .input_axis = {43676, -39094, 29309},
        .input_angle_rad = -45038,
        .expected_quaternion = {61705, -14714, 13170, -9874},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57267, -24507, -20367}
    },
    /* Vector 801: Angle -0.684 rad */
    {
        .input_axis = {41668, -41228, 29309},
        .input_angle_rad = -44837,
        .expected_quaternion = {61739, -13977, 13830, -9831},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56749, -24423, -21863}
    },
    /* Vector 802: Angle -0.681 rad */
    {
        .input_axis = {39555, -43259, 29309},
        .input_angle_rad = -44636,
        .expected_quaternion = {61772, -13211, 14449, -9789},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56241, -24279, -23291}
    },
    /* Vector 803: Angle -0.678 rad */
    {
        .input_axis = {37344, -45182, 29309},
        .input_angle_rad = -44435,
        .expected_quaternion = {61806, -12419, 15025, -9747},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55747, -24078, -24647}
    },
    /* Vector 804: Angle -0.675 rad */
    {
        .input_axis = {35039, -46992, 29309},
        .input_angle_rad = -44234,
        .expected_quaternion = {61839, -11602, 15559, -9704},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55274, -23823, -25928}
    },
    /* Vector 805: Angle -0.672 rad */
    {
        .input_axis = {32647, -48684, 29309},
        .input_angle_rad = -44033,
        .expected_quaternion = {61873, -10762, 16049, -9662},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54827, -23515, -27131}
    },
    /* Vector 806: Angle -0.669 rad */
    {
        .input_axis = {30173, -50255, 29309},
        .input_angle_rad = -43832,
        .expected_quaternion = {61906, -9903, 16494, -9619},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54410, -23158, -28254}
    },
    /* Vector 807: Angle -0.666 rad */
    {
        .input_axis = {27623, -51700, 29309},
        .input_angle_rad = -43630,
        .expected_quaternion = {61939, -9026, 16894, -9577},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54027, -22756, -29295}
    },
    /* Vector 808: Angle -0.663 rad */
    {
        .input_axis = {25005, -53016, 29309},
        .input_angle_rad = -43429,
        .expected_quaternion = {61971, -8134, 17247, -9534},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53684, -22313, -30251}
    },
    /* Vector 809: Angle -0.660 rad */
    {
        .input_axis = {22324, -54200, 29309},
        .input_angle_rad = -43228,
        .expected_quaternion = {62004, -7230, 17553, -9492},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53384, -21833, -31120}
    },
    /* Vector 810: Angle -0.657 rad */
    {
        .input_axis = {19587, -55248, 29309},
        .input_angle_rad = -43027,
        .expected_quaternion = {62036, -6315, 17812, -9449},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53129, -21322, -31901}
    },
    /* Vector 811: Angle -0.653 rad */
    {
        .input_axis = {16801, -56158, 29309},
        .input_angle_rad = -42826,
        .expected_quaternion = {62069, -5392, 18024, -9407},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52921, -20784, -32593}
    },
    /* Vector 812: Angle -0.650 rad */
    {
        .input_axis = {13974, -56927, 29309},
        .input_angle_rad = -42625,
        .expected_quaternion = {62101, -4465, 18188, -9364},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52764, -20225, -33194}
    },
    /* Vector 813: Angle -0.647 rad */
    {
        .input_axis = {11111, -57555, 29309},
        .input_angle_rad = -42424,
        .expected_quaternion = {62133, -3534, 18305, -9322},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52659, -19649, -33704}
    },
    /* Vector 814: Angle -0.644 rad */
    {
        .input_axis = {8221, -58038, 29309},
        .input_angle_rad = -42223,
        .expected_quaternion = {62165, -2603, 18374, -9279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52605, -19063, -34122}
    },
    /* Vector 815: Angle -0.641 rad */
    {
        .input_axis = {5310, -58376, 29309},
        .input_angle_rad = -42022,
        .expected_quaternion = {62197, -1673, 18397, -9236},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52604, -18471, -34447}
    },
    /* Vector 816: Angle -0.638 rad */
    {
        .input_axis = {2385, -58569, 29309},
        .input_angle_rad = -41821,
        .expected_quaternion = {62228, -748, 18372, -9194},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52656, -17879, -34679}
    },
    /* Vector 817: Angle -0.635 rad */
    {
        .input_axis = {-545, -58615, 29309},
        .input_angle_rad = -41620,
        .expected_quaternion = {62260, 170, 18301, -9151},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52759, -17292, -34820}
    },
    /* Vector 818: Angle -0.632 rad */
    {
        .input_axis = {-3474, -58514, 29309},
        .input_angle_rad = -41419,
        .expected_quaternion = {62291, 1080, 18184, -9108},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52913, -16715, -34868}
    },
    /* Vector 819: Angle -0.629 rad */
    {
        .input_axis = {-6394, -58267, 29309},
        .input_angle_rad = -41218,
        .expected_quaternion = {62322, 1978, 18023, -9065},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53115, -16154, -34825}
    },
    /* Vector 820: Angle -0.626 rad */
    {
        .input_axis = {-9298, -57875, 29309},
        .input_angle_rad = -41017,
        .expected_quaternion = {62353, 2862, 17817, -9023},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53364, -15613, -34691}
    },
    /* Vector 821: Angle -0.623 rad */
    {
        .input_axis = {-12179, -57338, 29309},
        .input_angle_rad = -40816,
        .expected_quaternion = {62384, 3732, 17568, -8980},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53657, -15095, -34468}
    },
    /* Vector 822: Angle -0.620 rad */
    {
        .input_axis = {-15029, -56658, 29309},
        .input_angle_rad = -40615,
        .expected_quaternion = {62415, 4583, 17277, -8937},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53990, -14607, -34157}
    },
    /* Vector 823: Angle -0.617 rad */
    {
        .input_axis = {-17842, -55836, 29309},
        .input_angle_rad = -40413,
        .expected_quaternion = {62445, 5415, 16944, -8894},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54360, -14150, -33760}
    },
    /* Vector 824: Angle -0.614 rad */
    {
        .input_axis = {-20611, -54874, 29309},
        .input_angle_rad = -40212,
        .expected_quaternion = {62476, 6225, 16572, -8851},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54764, -13728, -33278}
    },
    /* Vector 825: Angle -0.611 rad */
    {
        .input_axis = {-23327, -53775, 29309},
        .input_angle_rad = -40011,
        .expected_quaternion = {62506, 7011, 16162, -8808},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55197, -13345, -32714}
    },
    /* Vector 826: Angle -0.607 rad */
    {
        .input_axis = {-25986, -52542, 29309},
        .input_angle_rad = -39810,
        .expected_quaternion = {62536, 7772, 15714, -8766},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55655, -13002, -32069}
    },
    /* Vector 827: Angle -0.604 rad */
    {
        .input_axis = {-28580, -51178, 29309},
        .input_angle_rad = -39609,
        .expected_quaternion = {62566, 8506, 15231, -8723},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56134, -12701, -31347}
    },
    /* Vector 828: Angle -0.601 rad */
    {
        .input_axis = {-31102, -49686, 29309},
        .input_angle_rad = -39408,
        .expected_quaternion = {62596, 9211, 14714, -8680},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56629, -12445, -30549}
    },
    /* Vector 829: Angle -0.598 rad */
    {
        .input_axis = {-33546, -48069, 29309},
        .input_angle_rad = -39207,
        .expected_quaternion = {62626, 9886, 14165, -8637},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57136, -12233, -29678}
    },
    /* Vector 830: Angle -0.595 rad */
    {
        .input_axis = {-35907, -46332, 29309},
        .input_angle_rad = -39006,
        .expected_quaternion = {62655, 10528, 13586, -8594},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57650, -12067, -28738}
    },
    /* Vector 831: Angle -0.592 rad */
    {
        .input_axis = {-38177, -44480, 29309},
        .input_angle_rad = -38805,
        .expected_quaternion = {62685, 11138, 12977, -8551},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58165, -11947, -27732}
    },
    /* Vector 832: Angle -0.589 rad */
    {
        .input_axis = {-40353, -42516, 29309},
        .input_angle_rad = -38604,
        .expected_quaternion = {62714, 11714, 12342, -8508},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58679, -11871, -26662}
    },
    /* Vector 833: Angle -0.586 rad */
    {
        .input_axis = {-42427, -40446, 29309},
        .input_angle_rad = -38403,
        .expected_quaternion = {62743, 12254, 11682, -8465},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59185, -11840, -25533}
    },
    /* Vector 834: Angle -0.583 rad */
    {
        .input_axis = {-44396, -38275, 29309},
        .input_angle_rad = -38202,
        .expected_quaternion = {62772, 12757, 10998, -8422},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59680, -11851, -24348}
    },
    /* Vector 835: Angle -0.580 rad */
    {
        .input_axis = {-46253, -36009, 29309},
        .input_angle_rad = -38001,
        .expected_quaternion = {62801, 13223, 10294, -8379},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60160, -11904, -23110}
    },
    /* Vector 836: Angle -0.577 rad */
    {
        .input_axis = {-47995, -33652, 29309},
        .input_angle_rad = -37800,
        .expected_quaternion = {62830, 13650, 9571, -8336},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60620, -11996, -21824}
    },
    /* Vector 837: Angle -0.574 rad */
    {
        .input_axis = {-49617, -31211, 29309},
        .input_angle_rad = -37599,
        .expected_quaternion = {62858, 14038, 8831, -8292},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61058, -12124, -20493}
    },
    /* Vector 838: Angle -0.571 rad */
    {
        .input_axis = {-51115, -28692, 29309},
        .input_angle_rad = -37398,
        .expected_quaternion = {62886, 14387, 8076, -8249},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61469, -12286, -19121}
    },
    /* Vector 839: Angle -0.568 rad */
    {
        .input_axis = {-52485, -26102, 29309},
        .input_angle_rad = -37196,
        .expected_quaternion = {62915, 14695, 7308, -8206},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61851, -12478, -17712}
    },
    /* Vector 840: Angle -0.565 rad */
    {
        .input_axis = {-53724, -23446, 29309},
        .input_angle_rad = -36995,
        .expected_quaternion = {62943, 14963, 6530, -8163},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62201, -12698, -16271}
    },
    /* Vector 841: Angle -0.561 rad */
    {
        .input_axis = {-54829, -20732, 29309},
        .input_angle_rad = -36794,
        .expected_quaternion = {62971, 15190, 5744, -8120},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62517, -12941, -14802}
    },
    /* Vector 842: Angle -0.558 rad */
    {
        .input_axis = {-55796, -17965, 29309},
        .input_angle_rad = -36593,
        .expected_quaternion = {62998, 15376, 4951, -8077},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62797, -13205, -13308}
    },
    /* Vector 843: Angle -0.555 rad */
    {
        .input_axis = {-56624, -15154, 29309},
        .input_angle_rad = -36392,
        .expected_quaternion = {63026, 15521, 4154, -8033},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63040, -13484, -11794}
    },
    /* Vector 844: Angle -0.552 rad */
    {
        .input_axis = {-57311, -12305, 29309},
        .input_angle_rad = -36191,
        .expected_quaternion = {63054, 15624, 3355, -7990},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63244, -13775, -10265}
    },
    /* Vector 845: Angle -0.549 rad */
    {
        .input_axis = {-57854, -9426, 29309},
        .input_angle_rad = -35990,
        .expected_quaternion = {63081, 15687, 2556, -7947},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63409, -14075, -8724}
    },
    /* Vector 846: Angle -0.546 rad */
    {
        .input_axis = {-58253, -6522, 29309},
        .input_angle_rad = -35789,
        .expected_quaternion = {63108, 15709, 1759, -7904},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63535, -14378, -7176}
    },
    /* Vector 847: Angle -0.543 rad */
    {
        .input_axis = {-58506, -3603, 29309},
        .input_angle_rad = -35588,
        .expected_quaternion = {63135, 15691, 966, -7860},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63622, -14682, -5626}
    },
    /* Vector 848: Angle -0.540 rad */
    {
        .input_axis = {-58613, -674, 29309},
        .input_angle_rad = -35387,
        .expected_quaternion = {63162, 15633, 180, -7817},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63670, -14982, -4076}
    },
    /* Vector 849: Angle -0.537 rad */
    {
        .input_axis = {-58574, 2256, 29309},
        .input_angle_rad = -35186,
        .expected_quaternion = {63189, 15536, -598, -7774},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63681, -15274, -2532}
    },
    /* Vector 850: Angle -0.534 rad */
    {
        .input_axis = {-58388, 5181, 29309},
        .input_angle_rad = -34985,
        .expected_quaternion = {63215, 15400, -1366, -7730},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63655, -15555, -997}
    },
    /* Vector 851: Angle -0.531 rad */
    {
        .input_axis = {-58056, 8093, 29309},
        .input_angle_rad = -34784,
        .expected_quaternion = {63242, 15227, -2122, -7687},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63595, -15822, 524}
    },
    /* Vector 852: Angle -0.528 rad */
    {
        .input_axis = {-57579, 10984, 29309},
        .input_angle_rad = -34583,
        .expected_quaternion = {63268, 15016, -2865, -7644},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63503, -16071, 2028}
    },
    /* Vector 853: Angle -0.525 rad */
    {
        .input_axis = {-56958, 13848, 29309},
        .input_angle_rad = -34382,
        .expected_quaternion = {63294, 14770, -3591, -7600},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63380, -16299, 3511}
    },
    /* Vector 854: Angle -0.522 rad */
    {
        .input_axis = {-56195, 16677, 29309},
        .input_angle_rad = -34181,
        .expected_quaternion = {63320, 14489, -4300, -7557},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63229, -16504, 4968}
    },
    /* Vector 855: Angle -0.518 rad */
    {
        .input_axis = {-55291, 19465, 29309},
        .input_angle_rad = -33979,
        .expected_quaternion = {63346, 14174, -4990, -7513},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63053, -16683, 6396}
    },
    /* Vector 856: Angle -0.515 rad */
    {
        .input_axis = {-54249, 22204, 29309},
        .input_angle_rad = -33778,
        .expected_quaternion = {63372, 13826, -5659, -7470},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62856, -16834, 7793}
    },
    /* Vector 857: Angle -0.512 rad */
    {
        .input_axis = {-53071, 24888, 29309},
        .input_angle_rad = -33577,
        .expected_quaternion = {63397, 13447, -6306, -7426},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62639, -16956, 9153}
    },
    /* Vector 858: Angle -0.509 rad */
    {
        .input_axis = {-51761, 27509, 29309},
        .input_angle_rad = -33376,
        .expected_quaternion = {63423, 13039, -6929, -7383},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62407, -17047, 10474}
    },
    /* Vector 859: Angle -0.506 rad */
    {
        .input_axis = {-50322, 30062, 29309},
        .input_angle_rad = -33175,
        .expected_quaternion = {63448, 12601, -7528, -7339},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62163, -17106, 11754}
    },
    /* Vector 860: Angle -0.503 rad */
    {
        .input_axis = {-48756, 32539, 29309},
        .input_angle_rad = -32974,
        .expected_quaternion = {63473, 12137, -8100, -7296},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61909, -17132, 12988}
    },
    /* Vector 861: Angle -0.500 rad */
    {
        .input_axis = {-47069, 34935, 29309},
        .input_angle_rad = -32773,
        .expected_quaternion = {63498, 11647, -8644, -7252},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61650, -17126, 14174}
    },
    /* Vector 862: Angle -0.497 rad */
    {
        .input_axis = {-45264, 37244, 29309},
        .input_angle_rad = -32572,
        .expected_quaternion = {63523, 11133, -9160, -7209},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61389, -17087, 15309}
    },
    /* Vector 863: Angle -0.494 rad */
    {
        .input_axis = {-43346, 39460, 29309},
        .input_angle_rad = -32371,
        .expected_quaternion = {63547, 10597, -9647, -7165},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61129, -17015, 16391}
    },
    /* Vector 864: Angle -0.491 rad */
    {
        .input_axis = {-41320, 41577, 29309},
        .input_angle_rad = -32170,
        .expected_quaternion = {63572, 10040, -10102, -7121},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60874, -16911, 17417}
    },
    /* Vector 865: Angle -0.488 rad */
    {
        .input_axis = {-39190, 43590, 29309},
        .input_angle_rad = -31969,
        .expected_quaternion = {63596, 9464, -10527, -7078},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60626, -16777, 18386}
    },
    /* Vector 866: Angle -0.485 rad */
    {
        .input_axis = {-36963, 45494, 29309},
        .input_angle_rad = -31768,
        .expected_quaternion = {63621, 8871, -10919, -7034},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60388, -16613, 19295}
    },
    /* Vector 867: Angle -0.482 rad */
    {
        .input_axis = {-34643, 47285, 29309},
        .input_angle_rad = -31567,
        .expected_quaternion = {63645, 8263, -11278, -6990},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60163, -16421, 20142}
    },
    /* Vector 868: Angle -0.479 rad */
    {
        .input_axis = {-32236, 48957, 29309},
        .input_angle_rad = -31366,
        .expected_quaternion = {63668, 7641, -11604, -6947},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59954, -16204, 20927}
    },
    /* Vector 869: Angle -0.476 rad */
    {
        .input_axis = {-29749, 50507, 29309},
        .input_angle_rad = -31165,
        .expected_quaternion = {63692, 7007, -11896, -6903},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59763, -15962, 21647}
    },
    /* Vector 870: Angle -0.472 rad */
    {
        .input_axis = {-27188, 51931, 29309},
        .input_angle_rad = -30964,
        .expected_quaternion = {63716, 6363, -12154, -6859},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59592, -15698, 22301}
    },
    /* Vector 871: Angle -0.469 rad */
    {
        .input_axis = {-24558, 53225, 29309},
        .input_angle_rad = -30762,
        .expected_quaternion = {63739, 5711, -12377, -6816},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59443, -15415, 22888}
    },
    /* Vector 872: Angle -0.466 rad */
    {
        .input_axis = {-21867, 54386, 29309},
        .input_angle_rad = -30561,
        .expected_quaternion = {63763, 5053, -12566, -6772},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59317, -15115, 23408}
    },
    /* Vector 873: Angle -0.463 rad */
    {
        .input_axis = {-19122, 55411, 29309},
        .input_angle_rad = -30360,
        .expected_quaternion = {63786, 4390, -12720, -6728},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59217, -14801, 23860}
    },
    /* Vector 874: Angle -0.460 rad */
    {
        .input_axis = {-16329, 56297, 29309},
        .input_angle_rad = -30159,
        .expected_quaternion = {63809, 3724, -12840, -6684},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59141, -14476, 24243}
    },
    /* Vector 875: Angle -0.457 rad */
    {
        .input_axis = {-13494, 57043, 29309},
        .input_angle_rad = -29958,
        .expected_quaternion = {63832, 3058, -12925, -6641},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59092, -14142, 24557}
    },
    /* Vector 876: Angle -0.454 rad */
    {
        .input_axis = {-10627, 57646, 29309},
        .input_angle_rad = -29757,
        .expected_quaternion = {63854, 2392, -12975, -6597},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59070, -13802, 24803}
    },
    /* Vector 877: Angle -0.451 rad */
    {
        .input_axis = {-7732, 58105, 29309},
        .input_angle_rad = -29556,
        .expected_quaternion = {63877, 1729, -12992, -6553},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59075, -13460, 24980}
    },
    /* Vector 878: Angle -0.448 rad */
    {
        .input_axis = {-4819, 58419, 29309},
        .input_angle_rad = -29355,
        .expected_quaternion = {63899, 1070, -12974, -6509},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59106, -13117, 25088}
    },
    /* Vector 879: Angle -0.445 rad */
    {
        .input_axis = {-1893, 58587, 29309},
        .input_angle_rad = -29154,
        .expected_quaternion = {63922, 418, -12924, -6465},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59163, -12777, 25129}
    },
    /* Vector 880: Angle -0.442 rad */
    {
        .input_axis = {1038, 58608, 29309},
        .input_angle_rad = -28953,
        .expected_quaternion = {63944, -227, -12841, -6422},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59245, -12442, 25103}
    },
    /* Vector 881: Angle -0.439 rad */
    {
        .input_axis = {3966, 58483, 29309},
        .input_angle_rad = -28752,
        .expected_quaternion = {63966, -863, -12726, -6378},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59352, -12115, 25010}
    },
    /* Vector 882: Angle -0.436 rad */
    {
        .input_axis = {6883, 58212, 29309},
        .input_angle_rad = -28551,
        .expected_quaternion = {63987, -1488, -12580, -6334},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59482, -11797, 24853}
    },
    /* Vector 883: Angle -0.433 rad */
    {
        .input_axis = {9784, 57795, 29309},
        .input_angle_rad = -28350,
        .expected_quaternion = {64009, -2100, -12403, -6290},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59634, -11492, 24632}
    },
    /* Vector 884: Angle -0.430 rad */
    {
        .input_axis = {12661, 57234, 29309},
        .input_angle_rad = -28149,
        .expected_quaternion = {64031, -2698, -12197, -6246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59805, -11201, 24348}
    },
    /* Vector 885: Angle -0.426 rad */
    {
        .input_axis = {15505, 56529, 29309},
        .input_angle_rad = -27948,
        .expected_quaternion = {64052, -3281, -11962, -6202},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59995, -10925, 24004}
    },
    /* Vector 886: Angle -0.423 rad */
    {
        .input_axis = {18311, 55684, 29309},
        .input_angle_rad = -27747,
        .expected_quaternion = {64073, -3847, -11700, -6158},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60201, -10668, 23600}
    },
    /* Vector 887: Angle -0.420 rad */
    {
        .input_axis = {21071, 54699, 29309},
        .input_angle_rad = -27545,
        .expected_quaternion = {64094, -4396, -11411, -6114},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60422, -10428, 23140}
    },
    /* Vector 888: Angle -0.417 rad */
    {
        .input_axis = {23779, 53577, 29309},
        .input_angle_rad = -27344,
        .expected_quaternion = {64115, -4925, -11097, -6070},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60654, -10209, 22624}
    },
    /* Vector 889: Angle -0.414 rad */
    {
        .input_axis = {26427, 52322, 29309},
        .input_angle_rad = -27143,
        .expected_quaternion = {64136, -5434, -10758, -6026},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60896, -10011, 22055}
    },
    /* Vector 890: Angle -0.411 rad */
    {
        .input_axis = {29009, 50936, 29309},
        .input_angle_rad = -26942,
        .expected_quaternion = {64156, -5921, -10396, -5982},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61145, -9834, 21436}
    },
    /* Vector 891: Angle -0.408 rad */
    {
        .input_axis = {31518, 49422, 29309},
        .input_angle_rad = -26741,
        .expected_quaternion = {64177, -6386, -10013, -5938},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61400, -9679, 20769}
    },
    /* Vector 892: Angle -0.405 rad */
    {
        .input_axis = {33949, 47785, 29309},
        .input_angle_rad = -26540,
        .expected_quaternion = {64197, -6827, -9610, -5894},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61658, -9545, 20055}
    },
    /* Vector 893: Angle -0.402 rad */
    {
        .input_axis = {36295, 46029, 29309},
        .input_angle_rad = -26339,
        .expected_quaternion = {64217, -7245, -9187, -5850},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61916, -9433, 19299}
    },
    /* Vector 894: Angle -0.399 rad */
    {
        .input_axis = {38550, 44157, 29309},
        .input_angle_rad = -26138,
        .expected_quaternion = {64237, -7637, -8748, -5806},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62172, -9343, 18501}
    },
    /* Vector 895: Angle -0.396 rad */
    {
        .input_axis = {40709, 42176, 29309},
        .input_angle_rad = -25937,
        .expected_quaternion = {64257, -8003, -8291, -5762},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62425, -9274, 17667}
    },
    /* Vector 896: Angle -0.393 rad */
    {
        .input_axis = {42766, 40088, 29309},
        .input_angle_rad = -25736,
        .expected_quaternion = {64277, -8343, -7821, -5718},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62672, -9225, 16797}
    },
    /* Vector 897: Angle -0.390 rad */
    {
        .input_axis = {44716, 37901, 29309},
        .input_angle_rad = -25535,
        .expected_quaternion = {64296, -8656, -7337, -5674},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62911, -9195, 15895}
    },
    /* Vector 898: Angle -0.387 rad */
    {
        .input_axis = {46554, 35619, 29309},
        .input_angle_rad = -25334,
        .expected_quaternion = {64316, -8942, -6842, -5630},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63140, -9183, 14965}
    },
    /* Vector 899: Angle -0.383 rad */
    {
        .input_axis = {48276, 33247, 29309},
        .input_angle_rad = -25133,
        .expected_quaternion = {64335, -9200, -6336, -5585},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63359, -9187, 14008}
    },
    /* Vector 900: Angle -0.380 rad */
    {
        .input_axis = {49878, 30793, 29309},
        .input_angle_rad = -24932,
        .expected_quaternion = {64354, -9430, -5822, -5541},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63565, -9207, 13029}
    },
    /* Vector 901: Angle -0.377 rad */
    {
        .input_axis = {51354, 28262, 29309},
        .input_angle_rad = -24731,
        .expected_quaternion = {64373, -9632, -5301, -5497},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63756, -9241, 12029}
    },
    /* Vector 902: Angle -0.374 rad */
    {
        .input_axis = {52703, 25660, 29309},
        .input_angle_rad = -24530,
        .expected_quaternion = {64392, -9806, -4774, -5453},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63933, -9287, 11013}
    },
    /* Vector 903: Angle -0.371 rad */
    {
        .input_axis = {53919, 22994, 29309},
        .input_angle_rad = -24328,
        .expected_quaternion = {64410, -9951, -4243, -5409},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64094, -9343, 9984}
    },
    /* Vector 904: Angle -0.368 rad */
    {
        .input_axis = {55001, 20270, 29309},
        .input_angle_rad = -24127,
        .expected_quaternion = {64429, -10067, -3710, -5365},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64238, -9408, 8943}
    },
    /* Vector 905: Angle -0.365 rad */
    {
        .input_axis = {55945, 17496, 29309},
        .input_angle_rad = -23926,
        .expected_quaternion = {64447, -10156, -3176, -5320},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64364, -9480, 7895}
    },
    /* Vector 906: Angle -0.362 rad */
    {
        .input_axis = {56750, 14678, 29309},
        .input_angle_rad = -23725,
        .expected_quaternion = {64465, -10216, -2642, -5276},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64473, -9556, 6843}
    },
    /* Vector 907: Angle -0.359 rad */
    {
        .input_axis = {57412, 11823, 29309},
        .input_angle_rad = -23524,
        .expected_quaternion = {64483, -10249, -2111, -5232},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64565, -9636, 5790}
    },
    /* Vector 908: Angle -0.356 rad */
    {
        .input_axis = {57932, 8939, 29309},
        .input_angle_rad = -23323,
        .expected_quaternion = {64501, -10254, -1582, -5188},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64638, -9717, 4738}
    },
    /* Vector 909: Angle -0.353 rad */
    {
        .input_axis = {58306, 6032, 29309},
        .input_angle_rad = -23122,
        .expected_quaternion = {64519, -10232, -1059, -5143},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64694, -9797, 3691}
    },
    /* Vector 910: Angle -0.350 rad */
    {
        .input_axis = {58535, 3111, 29309},
        .input_angle_rad = -22921,
        .expected_quaternion = {64536, -10184, -541, -5099},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64734, -9875, 2651}
    },
    /* Vector 911: Angle -0.347 rad */
    {
        .input_axis = {58617, 181, 29309},
        .input_angle_rad = -22720,
        .expected_quaternion = {64554, -10110, -31, -5055},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64756, -9949, 1621}
    },
    /* Vector 912: Angle -0.344 rad */
    {
        .input_axis = {58553, -2749, 29309},
        .input_angle_rad = -22519,
        .expected_quaternion = {64571, -10010, 470, -5011},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64763, -10017, 605}
    },
    /* Vector 913: Angle -0.341 rad */
    {
        .input_axis = {58342, -5672, 29309},
        .input_angle_rad = -22318,
        .expected_quaternion = {64588, -9886, 961, -4966},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64755, -10079, -396}
    },
    /* Vector 914: Angle -0.337 rad */
    {
        .input_axis = {57986, -8580, 29309},
        .input_angle_rad = -22117,
        .expected_quaternion = {64605, -9738, 1441, -4922},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64733, -10132, -1378}
    },
    /* Vector 915: Angle -0.334 rad */
    {
        .input_axis = {57484, -11468, 29309},
        .input_angle_rad = -21916,
        .expected_quaternion = {64622, -9567, 1909, -4878},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64699, -10177, -2340}
    },
    /* Vector 916: Angle -0.331 rad */
    {
        .input_axis = {56840, -14326, 29309},
        .input_angle_rad = -21715,
        .expected_quaternion = {64639, -9374, 2363, -4833},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64653, -10210, -3278}
    },
    /* Vector 917: Angle -0.328 rad */
    {
        .input_axis = {56052, -17149, 29309},
        .input_angle_rad = -21514,
        .expected_quaternion = {64655, -9159, 2802, -4789},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64596, -10233, -4190}
    },
    /* Vector 918: Angle -0.325 rad */
    {
        .input_axis = {55125, -19929, 29309},
        .input_angle_rad = -21313,
        .expected_quaternion = {64672, -8924, 3226, -4745},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64531, -10243, -5075}
    },
    /* Vector 919: Angle -0.322 rad */
    {
        .input_axis = {54060, -22659, 29309},
        .input_angle_rad = -21112,
        .expected_quaternion = {64688, -8670, 3634, -4700},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64459, -10240, -5930}
    },
    /* Vector 920: Angle -0.319 rad */
    {
        .input_axis = {52860, -25333, 29309},
        .input_angle_rad = -20910,
        .expected_quaternion = {64704, -8397, 4024, -4656},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64380, -10225, -6753}
    },
    /* Vector 921: Angle -0.316 rad */
    {
        .input_axis = {51528, -27943, 29309},
        .input_angle_rad = -20709,
        .expected_quaternion = {64720, -8108, 4397, -4612},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64297, -10196, -7543}
    },
    /* Vector 922: Angle -0.313 rad */
    {
        .input_axis = {50067, -30484, 29309},
        .input_angle_rad = -20508,
        .expected_quaternion = {64735, -7802, 4750, -4567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64211, -10154, -8297}
    },
    /* Vector 923: Angle -0.310 rad */
    {
        .input_axis = {48481, -32948, 29309},
        .input_angle_rad = -20307,
        .expected_quaternion = {64751, -7481, 5084, -4523},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64123, -10098, -9014}
    },
    /* Vector 924: Angle -0.307 rad */
    {
        .input_axis = {46774, -35330, 29309},
        .input_angle_rad = -20106,
        .expected_quaternion = {64766, -7147, 5398, -4478},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64035, -10029, -9693}
    },
    /* Vector 925: Angle -0.304 rad */
    {
        .input_axis = {44949, -37623, 29309},
        .input_angle_rad = -19905,
        .expected_quaternion = {64782, -6800, 5692, -4434},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63947, -9947, -10332}
    },
    /* Vector 926: Angle -0.301 rad */
    {
        .input_axis = {43013, -39823, 29309},
        .input_angle_rad = -19704,
        .expected_quaternion = {64797, -6442, 5964, -4389},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63863, -9852, -10931}
    },
    /* Vector 927: Angle -0.298 rad */
    {
        .input_axis = {40969, -41923, 29309},
        .input_angle_rad = -19503,
        .expected_quaternion = {64812, -6074, 6215, -4345},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63781, -9746, -11487}
    },
    /* Vector 928: Angle -0.295 rad */
    {
        .input_axis = {38822, -43918, 29309},
        .input_angle_rad = -19302,
        .expected_quaternion = {64827, -5696, 6444, -4300},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63704, -9628, -12001}
    },
    /* Vector 929: Angle -0.291 rad */
    {
        .input_axis = {36579, -45803, 29309},
        .input_angle_rad = -19101,
        .expected_quaternion = {64841, -5312, 6651, -4256},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63633, -9500, -12472}
    },
    /* Vector 930: Angle -0.288 rad */
    {
        .input_axis = {34244, -47574, 29309},
        .input_angle_rad = -18900,
        .expected_quaternion = {64856, -4921, 6836, -4211},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63569, -9362, -12898}
    },
    /* Vector 931: Angle -0.285 rad */
    {
        .input_axis = {31823, -49226, 29309},
        .input_angle_rad = -18699,
        .expected_quaternion = {64870, -4525, 6999, -4167},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63511, -9216, -13280}
    },
    /* Vector 932: Angle -0.282 rad */
    {
        .input_axis = {29323, -50755, 29309},
        .input_angle_rad = -18498,
        .expected_quaternion = {64884, -4125, 7139, -4122},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63462, -9062, -13617}
    },
    /* Vector 933: Angle -0.279 rad */
    {
        .input_axis = {26750, -52158, 29309},
        .input_angle_rad = -18297,
        .expected_quaternion = {64899, -3722, 7257, -4078},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63421, -8901, -13910}
    },
    /* Vector 934: Angle -0.276 rad */
    {
        .input_axis = {24110, -53429, 29309},
        .input_angle_rad = -18096,
        .expected_quaternion = {64912, -3318, 7353, -4033},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63390, -8735, -14158}
    },
    /* Vector 935: Angle -0.273 rad */
    {
        .input_axis = {21409, -54568, 29309},
        .input_angle_rad = -17895,
        .expected_quaternion = {64926, -2914, 7427, -3989},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63367, -8564, -14360}
    },
    /* Vector 936: Angle -0.270 rad */
    {
        .input_axis = {18655, -55569, 29309},
        .input_angle_rad = -17693,
        .expected_quaternion = {64940, -2511, 7479, -3944},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63354, -8390, -14519}
    },
    /* Vector 937: Angle -0.267 rad */
    {
        .input_axis = {15855, -56432, 29309},
        .input_angle_rad = -17492,
        .expected_quaternion = {64953, -2110, 7509, -3900},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63351, -8214, -14633}
    },
    /* Vector 938: Angle -0.264 rad */
    {
        .input_axis = {13014, -57154, 29309},
        .input_angle_rad = -17291,
        .expected_quaternion = {64967, -1712, 7518, -3855},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63358, -8036, -14704}
    },
    /* Vector 939: Angle -0.261 rad */
    {
        .input_axis = {10142, -57733, 29309},
        .input_angle_rad = -17090,
        .expected_quaternion = {64980, -1319, 7506, -3811},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63373, -7859, -14732}
    },
    /* Vector 940: Angle -0.258 rad */
    {
        .input_axis = {7244, -58168, 29309},
        .input_angle_rad = -16889,
        .expected_quaternion = {64993, -931, 7474, -3766},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63398, -7682, -14718}
    },
    /* Vector 941: Angle -0.255 rad */
    {
        .input_axis = {4327, -58457, 29309},
        .input_angle_rad = -16688,
        .expected_quaternion = {65006, -549, 7423, -3722},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63432, -7507, -14663}
    },
    /* Vector 942: Angle -0.252 rad */
    {
        .input_axis = {1400, -58600, 29309},
        .input_angle_rad = -16487,
        .expected_quaternion = {65018, -176, 7352, -3677},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63474, -7335, -14568}
    },
    /* Vector 943: Angle -0.249 rad */
    {
        .input_axis = {-1530, -58597, 29309},
        .input_angle_rad = -16286,
        .expected_quaternion = {65031, 190, 7262, -3632},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63524, -7167, -14433}
    },
    /* Vector 944: Angle -0.245 rad */
    {
        .input_axis = {-4457, -58447, 29309},
        .input_angle_rad = -16085,
        .expected_quaternion = {65043, 546, 7155, -3588},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63581, -7002, -14261}
    },
    /* Vector 945: Angle -0.242 rad */
    {
        .input_axis = {-7373, -58152, 29309},
        .input_angle_rad = -15884,
        .expected_quaternion = {65055, 891, 7030, -3543},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63645, -6843, -14053}
    },
    /* Vector 946: Angle -0.239 rad */
    {
        .input_axis = {-10270, -57711, 29309},
        .input_angle_rad = -15683,
        .expected_quaternion = {65067, 1226, 6889, -3498},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63714, -6689, -13810}
    },
    /* Vector 947: Angle -0.236 rad */
    {
        .input_axis = {-13141, -57125, 29309},
        .input_angle_rad = -15482,
        .expected_quaternion = {65079, 1549, 6732, -3454},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63789, -6541, -13533}
    },
    /* Vector 948: Angle -0.233 rad */
    {
        .input_axis = {-15980, -56397, 29309},
        .input_angle_rad = -15281,
        .expected_quaternion = {65091, 1859, 6560, -3409},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63868, -6400, -13224}
    },
    /* Vector 949: Angle -0.230 rad */
    {
        .input_axis = {-18779, -55528, 29309},
        .input_angle_rad = -15080,
        .expected_quaternion = {65103, 2156, 6374, -3364},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63951, -6265, -12886}
    },
    /* Vector 950: Angle -0.227 rad */
    {
        .input_axis = {-21530, -54520, 29309},
        .input_angle_rad = -14879,
        .expected_quaternion = {65114, 2439, 6176, -3320},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64036, -6137, -12519}
    },
    /* Vector 951: Angle -0.224 rad */
    {
        .input_axis = {-24228, -53376, 29309},
        .input_angle_rad = -14678,
        .expected_quaternion = {65126, 2707, 5965, -3275},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64123, -6016, -12125}
    },
    /* Vector 952: Angle -0.221 rad */
    {
        .input_axis = {-26866, -52098, 29309},
        .input_angle_rad = -14476,
        .expected_quaternion = {65137, 2961, 5742, -3230},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64211, -5903, -11707}
    },
    /* Vector 953: Angle -0.218 rad */
    {
        .input_axis = {-29436, -50690, 29309},
        .input_angle_rad = -14275,
        .expected_quaternion = {65148, 3200, 5510, -3186},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64300, -5796, -11266}
    },
    /* Vector 954: Angle -0.215 rad */
    {
        .input_axis = {-31933, -49156, 29309},
        .input_angle_rad = -14074,
        .expected_quaternion = {65159, 3422, 5268, -3141},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64388, -5696, -10804}
    },
    /* Vector 955: Angle -0.212 rad */
    {
        .input_axis = {-34350, -47498, 29309},
        .input_angle_rad = -13873,
        .expected_quaternion = {65169, 3629, 5018, -3096},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64475, -5602, -10323}
    },
    /* Vector 956: Angle -0.209 rad */
    {
        .input_axis = {-36681, -45722, 29309},
        .input_angle_rad = -13672,
        .expected_quaternion = {65180, 3819, 4761, -3052},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64560, -5515, -9825}
    },
    /* Vector 957: Angle -0.206 rad */
    {
        .input_axis = {-38920, -43832, 29309},
        .input_angle_rad = -13471,
        .expected_quaternion = {65190, 3993, 4497, -3007},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64643, -5434, -9313}
    },
    /* Vector 958: Angle -0.202 rad */
    {
        .input_axis = {-41062, -41832, 29309},
        .input_angle_rad = -13270,
        .expected_quaternion = {65200, 4150, 4228, -2962},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64723, -5359, -8788}
    },
    /* Vector 959: Angle -0.199 rad */
    {
        .input_axis = {-43101, -39727, 29309},
        .input_angle_rad = -13069,
        .expected_quaternion = {65210, 4290, 3955, -2917},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64799, -5288, -8252}
    },
    /* Vector 960: Angle -0.196 rad */
    {
        .input_axis = {-45033, -37523, 29309},
        .input_angle_rad = -12868,
        .expected_quaternion = {65220, 4414, 3678, -2873},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64871, -5222, -7707}
    },
    /* Vector 961: Angle -0.193 rad */
    {
        .input_axis = {-46852, -35226, 29309},
        .input_angle_rad = -12667,
        .expected_quaternion = {65230, 4521, 3399, -2828},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64939, -5161, -7156}
    },
    /* Vector 962: Angle -0.190 rad */
    {
        .input_axis = {-48554, -32840, 29309},
        .input_angle_rad = -12466,
        .expected_quaternion = {65240, 4611, 3119, -2783},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65003, -5103, -6601}
    },
    /* Vector 963: Angle -0.187 rad */
    {
        .input_axis = {-50135, -30372, 29309},
        .input_angle_rad = -12265,
        .expected_quaternion = {65249, 4684, 2838, -2738},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65061, -5047, -6042}
    },
    /* Vector 964: Angle -0.184 rad */
    {
        .input_axis = {-51590, -27829, 29309},
        .input_angle_rad = -12064,
        .expected_quaternion = {65259, 4742, 2558, -2694},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65115, -4995, -5484}
    },
    /* Vector 965: Angle -0.181 rad */
    {
        .input_axis = {-52916, -25216, 29309},
        .input_angle_rad = -11863,
        .expected_quaternion = {65268, 4783, 2279, -2649},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65163, -4944, -4926}
    },
    /* Vector 966: Angle -0.178 rad */
    {
        .input_axis = {-54111, -22539, 29309},
        .input_angle_rad = -11662,
        .expected_quaternion = {65277, 4808, 2003, -2604},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65207, -4894, -4372}
    },
    /* Vector 967: Angle -0.175 rad */
    {
        .input_axis = {-55169, -19807, 29309},
        .input_angle_rad = -11461,
        .expected_quaternion = {65286, 4818, 1730, -2559},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65245, -4845, -3822}
    },
    /* Vector 968: Angle -0.172 rad */
    {
        .input_axis = {-56090, -17025, 29309},
        .input_angle_rad = -11259,
        .expected_quaternion = {65294, 4812, 1461, -2515},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65278, -4796, -3280}
    },
    /* Vector 969: Angle -0.169 rad */
    {
        .input_axis = {-56871, -14200, 29309},
        .input_angle_rad = -11058,
        .expected_quaternion = {65303, 4792, 1197, -2470},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65306, -4747, -2746}
    },
    /* Vector 970: Angle -0.166 rad */
    {
        .input_axis = {-57510, -11340, 29309},
        .input_angle_rad = -10857,
        .expected_quaternion = {65311, 4758, 938, -2425},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65330, -4697, -2222}
    },
    /* Vector 971: Angle -0.163 rad */
    {
        .input_axis = {-58005, -8452, 29309},
        .input_angle_rad = -10656,
        .expected_quaternion = {65320, 4711, 686, -2380},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65349, -4646, -1710}
    },
    /* Vector 972: Angle -0.160 rad */
    {
        .input_axis = {-58355, -5542, 29309},
        .input_angle_rad = -10455,
        .expected_quaternion = {65328, 4650, 442, -2335},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65364, -4593, -1212}
    },
    /* Vector 973: Angle -0.156 rad */
    {
        .input_axis = {-58559, -2619, 29309},
        .input_angle_rad = -10254,
        .expected_quaternion = {65336, 4577, 205, -2291},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65375, -4539, -728}
    },
    /* Vector 974: Angle -0.153 rad */
    {
        .input_axis = {-58616, 311, 29309},
        .input_angle_rad = -10053,
        .expected_quaternion = {65343, 4491, -24, -2246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65382, -4482, -260}
    },
    /* Vector 975: Angle -0.150 rad */
    {
        .input_axis = {-58528, 3241, 29309},
        .input_angle_rad = -9852,
        .expected_quaternion = {65351, 4395, -243, -2201},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65386, -4422, 190}
    },
    /* Vector 976: Angle -0.147 rad */
    {
        .input_axis = {-58292, 6162, 29309},
        .input_angle_rad = -9651,
        .expected_quaternion = {65358, 4288, -453, -2156},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65388, -4360, 622}
    },
    /* Vector 977: Angle -0.144 rad */
    {
        .input_axis = {-57912, 9068, 29309},
        .input_angle_rad = -9450,
        .expected_quaternion = {65366, 4172, -653, -2111},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65387, -4295, 1034}
    },
    /* Vector 978: Angle -0.141 rad */
    {
        .input_axis = {-57386, 11951, 29309},
        .input_angle_rad = -9249,
        .expected_quaternion = {65373, 4046, -843, -2066},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65384, -4227, 1426}
    },
    /* Vector 979: Angle -0.138 rad */
    {
        .input_axis = {-56717, 14804, 29309},
        .input_angle_rad = -9048,
        .expected_quaternion = {65380, 3912, -1021, -2022},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65379, -4155, 1796}
    },
    /* Vector 980: Angle -0.135 rad */
    {
        .input_axis = {-55906, 17620, 29309},
        .input_angle_rad = -8847,
        .expected_quaternion = {65387, 3771, -1188, -1977},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65374, -4081, 2144}
    },
    /* Vector 981: Angle -0.132 rad */
    {
        .input_axis = {-54956, 20392, 29309},
        .input_angle_rad = -8646,
        .expected_quaternion = {65393, 3622, -1344, -1932},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65367, -4004, 2469}
    },
    /* Vector 982: Angle -0.129 rad */
    {
        .input_axis = {-53868, 23113, 29309},
        .input_angle_rad = -8445,
        .expected_quaternion = {65400, 3468, -1488, -1887},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65360, -3924, 2770}
    },
    /* Vector 983: Angle -0.126 rad */
    {
        .input_axis = {-52645, 25777, 29309},
        .input_angle_rad = -8244,
        .expected_quaternion = {65406, 3309, -1620, -1842},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65352, -3841, 3048}
    },
    /* Vector 984: Angle -0.123 rad */
    {
        .input_axis = {-51291, 28376, 29309},
        .input_angle_rad = -8042,
        .expected_quaternion = {65413, 3145, -1740, -1797},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65345, -3755, 3301}
    },
    /* Vector 985: Angle -0.120 rad */
    {
        .input_axis = {-49809, 30904, 29309},
        .input_angle_rad = -7841,
        .expected_quaternion = {65419, 2978, -1848, -1752},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65338, -3666, 3530}
    },
    /* Vector 986: Angle -0.117 rad */
    {
        .input_axis = {-48202, 33354, 29309},
        .input_angle_rad = -7640,
        .expected_quaternion = {65425, 2808, -1943, -1707},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65332, -3576, 3733}
    },
    /* Vector 987: Angle -0.114 rad */
    {
        .input_axis = {-46475, 35722, 29309},
        .input_angle_rad = -7439,
        .expected_quaternion = {65430, 2636, -2026, -1663},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65326, -3483, 3912}
    },
    /* Vector 988: Angle -0.110 rad */
    {
        .input_axis = {-44632, 38000, 29309},
        .input_angle_rad = -7238,
        .expected_quaternion = {65436, 2463, -2097, -1618},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65322, -3388, 4067}
    },
    /* Vector 989: Angle -0.107 rad */
    {
        .input_axis = {-42677, 40183, 29309},
        .input_angle_rad = -7037,
        .expected_quaternion = {65442, 2290, -2156, -1573},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65319, -3292, 4197}
    },
    /* Vector 990: Angle -0.104 rad */
    {
        .input_axis = {-40615, 42266, 29309},
        .input_angle_rad = -6836,
        .expected_quaternion = {65447, 2117, -2203, -1528},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65317, -3194, 4302}
    },
    /* Vector 991: Angle -0.101 rad */
    {
        .input_axis = {-38452, 44243, 29309},
        .input_angle_rad = -6635,
        .expected_quaternion = {65452, 1946, -2239, -1483},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65316, -3095, 4384}
    },
    /* Vector 992: Angle -0.098 rad */
    {
        .input_axis = {-36193, 46109, 29309},
        .input_angle_rad = -6434,
        .expected_quaternion = {65457, 1776, -2262, -1438},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65317, -2995, 4442}
    },
    /* Vector 993: Angle -0.095 rad */
    {
        .input_axis = {-33843, 47861, 29309},
        .input_angle_rad = -6233,
        .expected_quaternion = {65462, 1609, -2275, -1393},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65319, -2895, 4477}
    },
    /* Vector 994: Angle -0.092 rad */
    {
        .input_axis = {-31408, 49492, 29309},
        .input_angle_rad = -6032,
        .expected_quaternion = {65467, 1445, -2277, -1348},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65322, -2794, 4489}
    },
    /* Vector 995: Angle -0.089 rad */
    {
        .input_axis = {-28896, 51000, 29309},
        .input_angle_rad = -5831,
        .expected_quaternion = {65471, 1285, -2268, -1303},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65327, -2693, 4480}
    },
    /* Vector 996: Angle -0.086 rad */
    {
        .input_axis = {-26311, 52381, 29309},
        .input_angle_rad = -5630,
        .expected_quaternion = {65476, 1130, -2249, -1258},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65333, -2592, 4451}
    },
    /* Vector 997: Angle -0.083 rad */
    {
        .input_axis = {-23660, 53630, 29309},
        .input_angle_rad = -5429,
        .expected_quaternion = {65480, 980, -2221, -1214},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65341, -2491, 4401}
    },
    /* Vector 998: Angle -0.080 rad */
    {
        .input_axis = {-20950, 54746, 29309},
        .input_angle_rad = -5228,
        .expected_quaternion = {65484, 835, -2183, -1169},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65349, -2391, 4332}
    },
    /* Vector 999: Angle -0.077 rad */
    {
        .input_axis = {-18187, 55724, 29309},
        .input_angle_rad = -5027,
        .expected_quaternion = {65488, 697, -2136, -1124},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65358, -2291, 4246}
    },
    /* Vector 1000: Angle -0.074 rad */
    {
        .input_axis = {-15380, 56564, 29309},
        .input_angle_rad = -4825,
        .expected_quaternion = {65492, 566, -2082, -1079},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65368, -2192, 4142}
    },
    /* Vector 1001: Angle -0.071 rad */
    {
        .input_axis = {-12533, 57262, 29309},
        .input_angle_rad = -4624,
        .expected_quaternion = {65495, 442, -2020, -1034},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65379, -2094, 4023}
    },
    /* Vector 1002: Angle -0.067 rad */
    {
        .input_axis = {-9656, 57816, 29309},
        .input_angle_rad = -4423,
        .expected_quaternion = {65499, 326, -1951, -989},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65390, -1996, 3890}
    },
    /* Vector 1003: Angle -0.064 rad */
    {
        .input_axis = {-6754, 58227, 29309},
        .input_angle_rad = -4222,
        .expected_quaternion = {65502, 218, -1875, -944},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65401, -1899, 3743}
    },
    /* Vector 1004: Angle -0.061 rad */
    {
        .input_axis = {-3836, 58492, 29309},
        .input_angle_rad = -4021,
        .expected_quaternion = {65505, 118, -1794, -899},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65413, -1804, 3584}
    },
    /* Vector 1005: Angle -0.058 rad */
    {
        .input_axis = {-908, 58610, 29309},
        .input_angle_rad = -3820,
        .expected_quaternion = {65508, 26, -1708, -854},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65425, -1709, 3414}
    },
    /* Vector 1006: Angle -0.055 rad */
    {
        .input_axis = {2023, 58582, 29309},
        .input_angle_rad = -3619,
        .expected_quaternion = {65511, -56, -1617, -809},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65436, -1615, 3235}
    },
    /* Vector 1007: Angle -0.052 rad */
    {
        .input_axis = {4948, 58408, 29309},
        .input_angle_rad = -3418,
        .expected_quaternion = {65514, -129, -1523, -764},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65447, -1522, 3048}
    },
    /* Vector 1008: Angle -0.049 rad */
    {
        .input_axis = {7861, 58088, 29309},
        .input_angle_rad = -3217,
        .expected_quaternion = {65516, -193, -1426, -719},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65458, -1430, 2854}
    },
    /* Vector 1009: Angle -0.046 rad */
    {
        .input_axis = {10755, 57622, 29309},
        .input_angle_rad = -3016,
        .expected_quaternion = {65519, -247, -1326, -674},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65468, -1338, 2656}
    },
    /* Vector 1010: Angle -0.043 rad */
    {
        .input_axis = {13621, 57013, 29309},
        .input_angle_rad = -2815,
        .expected_quaternion = {65521, -293, -1224, -629},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65478, -1248, 2454}
    },
    /* Vector 1011: Angle -0.040 rad */
    {
        .input_axis = {16454, 56261, 29309},
        .input_angle_rad = -2614,
        .expected_quaternion = {65523, -328, -1122, -584},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65487, -1157, 2249}
    },
    /* Vector 1012: Angle -0.037 rad */
    {
        .input_axis = {19245, 55368, 29309},
        .input_angle_rad = -2413,
        .expected_quaternion = {65525, -354, -1019, -539},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65495, -1068, 2044}
    },
    /* Vector 1013: Angle -0.034 rad */
    {
        .input_axis = {21988, 54337, 29309},
        .input_angle_rad = -2212,
        .expected_quaternion = {65527, -371, -917, -495},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65503, -979, 1839}
    },
    /* Vector 1014: Angle -0.031 rad */
    {
        .input_axis = {24676, 53170, 29309},
        .input_angle_rad = -2011,
        .expected_quaternion = {65528, -379, -816, -450},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65510, -890, 1636}
    },
    /* Vector 1015: Angle -0.028 rad */
    {
        .input_axis = {27303, 51870, 29309},
        .input_angle_rad = -1810,
        .expected_quaternion = {65530, -377, -716, -405},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65515, -801, 1437}
    },
    /* Vector 1016: Angle -0.025 rad */
    {
        .input_axis = {29861, 50441, 29309},
        .input_angle_rad = -1608,
        .expected_quaternion = {65531, -366, -619, -360},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65520, -712, 1242}
    },
    /* Vector 1017: Angle -0.021 rad */
    {
        .input_axis = {32345, 48885, 29309},
        .input_angle_rad = -1407,
        .expected_quaternion = {65532, -347, -525, -315},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65525, -624, 1053}
    },
    /* Vector 1018: Angle -0.018 rad */
    {
        .input_axis = {34748, 47208, 29309},
        .input_angle_rad = -1206,
        .expected_quaternion = {65533, -320, -434, -270},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65528, -535, 872}
    },
    /* Vector 1019: Angle -0.015 rad */
    {
        .input_axis = {37064, 45412, 29309},
        .input_angle_rad = -1005,
        .expected_quaternion = {65534, -284, -348, -225},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65531, -447, 699}
    },
    /* Vector 1020: Angle -0.012 rad */
    {
        .input_axis = {39287, 43503, 29309},
        .input_angle_rad = -804,
        .expected_quaternion = {65535, -241, -267, -180},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65533, -358, 535}
    },
    /* Vector 1021: Angle -0.009 rad */
    {
        .input_axis = {41412, 41485, 29309},
        .input_angle_rad = -603,
        .expected_quaternion = {65535, -191, -191, -135},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65534, -269, 383}
    },
    /* Vector 1022: Angle -0.006 rad */
    {
        .input_axis = {43434, 39364, 29309},
        .input_angle_rad = -402,
        .expected_quaternion = {65536, -133, -121, -90},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65535, -179, 242}
    },
    /* Vector 1023: Angle -0.003 rad */
    {
        .input_axis = {45347, 37144, 29309},
        .input_angle_rad = -201,
        .expected_quaternion = {65536, -70, -57, -45},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65536, -90, 114}
    },
    /* Vector 1024: Angle 0.000 rad */
    {
        .input_axis = {47147, 34831, 29309},
        .input_angle_rad = 0,
        .expected_quaternion = {65536, 0, 0, 0},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65536, 0, 0}
    },
    /* Vector 1025: Angle 0.003 rad */
    {
        .input_axis = {48828, 32431, 29309},
        .input_angle_rad = 201,
        .expected_quaternion = {65536, 75, 50, 45},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65536, 90, -99}
    },
    /* Vector 1026: Angle 0.006 rad */
    {
        .input_axis = {50388, 29950, 29309},
        .input_angle_rad = 402,
        .expected_quaternion = {65536, 155, 92, 90},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65535, 180, -183}
    },
    /* Vector 1027: Angle 0.009 rad */
    {
        .input_axis = {51822, 27394, 29309},
        .input_angle_rad = 603,
        .expected_quaternion = {65535, 238, 126, 135},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65535, 271, -251}
    },
    /* Vector 1028: Angle 0.012 rad */
    {
        .input_axis = {53127, 24770, 29309},
        .input_angle_rad = 804,
        .expected_quaternion = {65535, 326, 152, 180},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65534, 361, -302}
    },
    /* Vector 1029: Angle 0.015 rad */
    {
        .input_axis = {54298, 22084, 29309},
        .input_angle_rad = 1005,
        .expected_quaternion = {65534, 416, 169, 225},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65534, 452, -336}
    },
    /* Vector 1030: Angle 0.018 rad */
    {
        .input_axis = {55334, 19342, 29309},
        .input_angle_rad = 1206,
        .expected_quaternion = {65533, 509, 178, 270},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65533, 542, -352}
    },
    /* Vector 1031: Angle 0.021 rad */
    {
        .input_axis = {56232, 16553, 29309},
        .input_angle_rad = 1407,
        .expected_quaternion = {65532, 604, 178, 315},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65532, 633, -350}
    },
    /* Vector 1032: Angle 0.025 rad */
    {
        .input_axis = {56989, 13721, 29309},
        .input_angle_rad = 1608,
        .expected_quaternion = {65531, 699, 168, 360},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65531, 723, -329}
    },
    /* Vector 1033: Angle 0.028 rad */
    {
        .input_axis = {57603, 10856, 29309},
        .input_angle_rad = 1810,
        .expected_quaternion = {65530, 795, 150, 405},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65530, 813, -290}
    },
    /* Vector 1034: Angle 0.031 rad */
    {
        .input_axis = {58074, 7964, 29309},
        .input_angle_rad = 2011,
        .expected_quaternion = {65528, 891, 122, 450},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65529, 902, -232}
    },
    /* Vector 1035: Angle 0.034 rad */
    {
        .input_axis = {58399, 5051, 29309},
        .input_angle_rad = 2212,
        .expected_quaternion = {65527, 985, 85, 495},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65528, 991, -156}
    },
    /* Vector 1036: Angle 0.037 rad */
    {
        .input_axis = {58579, 2126, 29309},
        .input_angle_rad = 2413,
        .expected_quaternion = {65525, 1078, 39, 539},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65527, 1080, -61}
    },
    /* Vector 1037: Angle 0.040 rad */
    {
        .input_axis = {58612, -804, 29309},
        .input_angle_rad = 2614,
        .expected_quaternion = {65523, 1169, -16, 584},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65526, 1168, 53}
    },
    /* Vector 1038: Angle 0.043 rad */
    {
        .input_axis = {58498, -3733, 29309},
        .input_angle_rad = 2815,
        .expected_quaternion = {65521, 1256, -80, 629},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65524, 1255, 184}
    },
    /* Vector 1039: Angle 0.046 rad */
    {
        .input_axis = {58239, -6652, 29309},
        .input_angle_rad = 3016,
        .expected_quaternion = {65519, 1340, -153, 674},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65521, 1342, 334}
    },
    /* Vector 1040: Angle 0.049 rad */
    {
        .input_axis = {57833, -9554, 29309},
        .input_angle_rad = 3217,
        .expected_quaternion = {65516, 1419, -234, 719},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65519, 1428, 500}
    },
    /* Vector 1041: Angle 0.052 rad */
    {
        .input_axis = {57284, -12433, 29309},
        .input_angle_rad = 3418,
        .expected_quaternion = {65514, 1494, -324, 764},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65515, 1513, 683}
    },
    /* Vector 1042: Angle 0.055 rad */
    {
        .input_axis = {56591, -15280, 29309},
        .input_angle_rad = 3619,
        .expected_quaternion = {65511, 1562, -422, 809},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65511, 1598, 882}
    },
    /* Vector 1043: Angle 0.058 rad */
    {
        .input_axis = {55756, -18089, 29309},
        .input_angle_rad = 3820,
        .expected_quaternion = {65508, 1625, -527, 854},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65505, 1681, 1096}
    },
    /* Vector 1044: Angle 0.061 rad */
    {
        .input_axis = {54782, -20853, 29309},
        .input_angle_rad = 4021,
        .expected_quaternion = {65505, 1680, -640, 899},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65499, 1764, 1325}
    },
    /* Vector 1045: Angle 0.064 rad */
    {
        .input_axis = {53672, -23565, 29309},
        .input_angle_rad = 4222,
        .expected_quaternion = {65502, 1729, -759, 944},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65491, 1847, 1567}
    },
    /* Vector 1046: Angle 0.067 rad */
    {
        .input_axis = {52427, -26218, 29309},
        .input_angle_rad = 4423,
        .expected_quaternion = {65499, 1769, -885, 989},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65482, 1929, 1822}
    },
    /* Vector 1047: Angle 0.071 rad */
    {
        .input_axis = {51051, -28806, 29309},
        .input_angle_rad = 4624,
        .expected_quaternion = {65495, 1801, -1016, 1034},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65472, 2011, 2088}
    },
    /* Vector 1048: Angle 0.074 rad */
    {
        .input_axis = {49547, -31321, 29309},
        .input_angle_rad = 4825,
        .expected_quaternion = {65492, 1824, -1153, 1079},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65460, 2092, 2364}
    },
    /* Vector 1049: Angle 0.077 rad */
    {
        .input_axis = {47920, -33758, 29309},
        .input_angle_rad = 5027,
        .expected_quaternion = {65488, 1837, -1294, 1124},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65446, 2173, 2650}
    },
    /* Vector 1050: Angle 0.080 rad */
    {
        .input_axis = {46173, -36111, 29309},
        .input_angle_rad = 5228,
        .expected_quaternion = {65484, 1841, -1440, 1169},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65431, 2254, 2943}
    },
    /* Vector 1051: Angle 0.083 rad */
    {
        .input_axis = {44311, -38374, 29309},
        .input_angle_rad = 5429,
        .expected_quaternion = {65480, 1835, -1589, 1214},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65414, 2336, 3243}
    },
    /* Vector 1052: Angle 0.086 rad */
    {
        .input_axis = {42337, -40540, 29309},
        .input_angle_rad = 5630,
        .expected_quaternion = {65476, 1818, -1741, 1258},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65395, 2418, 3548}
    },
    /* Vector 1053: Angle 0.089 rad */
    {
        .input_axis = {40258, -42606, 29309},
        .input_angle_rad = 5831,
        .expected_quaternion = {65471, 1790, -1895, 1303},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65375, 2501, 3857}
    },
    /* Vector 1054: Angle 0.092 rad */
    {
        .input_axis = {38078, -44565, 29309},
        .input_angle_rad = 6032,
        .expected_quaternion = {65467, 1752, -2050, 1348},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65352, 2584, 4168}
    },
    /* Vector 1055: Angle 0.095 rad */
    {
        .input_axis = {35804, -46412, 29309},
        .input_angle_rad = 6233,
        .expected_quaternion = {65462, 1702, -2206, 1393},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65328, 2669, 4480}
    },
    /* Vector 1056: Angle 0.098 rad */
    {
        .input_axis = {33439, -48143, 29309},
        .input_angle_rad = 6434,
        .expected_quaternion = {65457, 1641, -2362, 1438},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65303, 2754, 4791}
    },
    /* Vector 1057: Angle 0.101 rad */
    {
        .input_axis = {30991, -49755, 29309},
        .input_angle_rad = 6635,
        .expected_quaternion = {65452, 1568, -2518, 1483},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65275, 2842, 5100}
    },
    /* Vector 1058: Angle 0.104 rad */
    {
        .input_axis = {28466, -51241, 29309},
        .input_angle_rad = 6836,
        .expected_quaternion = {65447, 1484, -2671, 1528},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65247, 2931, 5405}
    },
    /* Vector 1059: Angle 0.107 rad */
    {
        .input_axis = {25869, -52600, 29309},
        .input_angle_rad = 7037,
        .expected_quaternion = {65442, 1388, -2823, 1573},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65217, 3021, 5704}
    },
    /* Vector 1060: Angle 0.110 rad */
    {
        .input_axis = {23208, -53827, 29309},
        .input_angle_rad = 7238,
        .expected_quaternion = {65436, 1281, -2971, 1618},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65187, 3114, 5996}
    },
    /* Vector 1061: Angle 0.114 rad */
    {
        .input_axis = {20489, -54920, 29309},
        .input_angle_rad = 7439,
        .expected_quaternion = {65430, 1162, -3115, 1663},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65155, 3209, 6280}
    },
    /* Vector 1062: Angle 0.117 rad */
    {
        .input_axis = {17718, -55875, 29309},
        .input_angle_rad = 7640,
        .expected_quaternion = {65425, 1032, -3255, 1707},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65124, 3307, 6553}
    },
    /* Vector 1063: Angle 0.120 rad */
    {
        .input_axis = {14904, -56691, 29309},
        .input_angle_rad = 7841,
        .expected_quaternion = {65419, 891, -3390, 1752},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65092, 3406, 6815}
    },
    /* Vector 1064: Angle 0.123 rad */
    {
        .input_axis = {12052, -57365, 29309},
        .input_angle_rad = 8042,
        .expected_quaternion = {65413, 739, -3518, 1797},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65060, 3508, 7063}
    },
    /* Vector 1065: Angle 0.126 rad */
    {
        .input_axis = {9169, -57896, 29309},
        .input_angle_rad = 8244,
        .expected_quaternion = {65406, 576, -3639, 1842},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65028, 3613, 7296}
    },
    /* Vector 1066: Angle 0.129 rad */
    {
        .input_axis = {6264, -58281, 29309},
        .input_angle_rad = 8445,
        .expected_quaternion = {65400, 403, -3752, 1887},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64998, 3720, 7512}
    },
    /* Vector 1067: Angle 0.132 rad */
    {
        .input_axis = {3344, -58522, 29309},
        .input_angle_rad = 8646,
        .expected_quaternion = {65393, 220, -3857, 1932},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64968, 3829, 7711}
    },
    /* Vector 1068: Angle 0.135 rad */
    {
        .input_axis = {415, -58616, 29309},
        .input_angle_rad = 8847,
        .expected_quaternion = {65387, 28, -3953, 1977},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64940, 3941, 7890}
    },
    /* Vector 1069: Angle 0.138 rad */
    {
        .input_axis = {-2515, -58563, 29309},
        .input_angle_rad = 9048,
        .expected_quaternion = {65380, -173, -4039, 2022},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64913, 4055, 8049}
    },
    /* Vector 1070: Angle 0.141 rad */
    {
        .input_axis = {-5439, -58364, 29309},
        .input_angle_rad = 9249,
        .expected_quaternion = {65373, -383, -4115, 2066},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64889, 4171, 8185}
    },
    /* Vector 1071: Angle 0.144 rad */
    {
        .input_axis = {-8349, -58019, 29309},
        .input_angle_rad = 9450,
        .expected_quaternion = {65366, -601, -4179, 2111},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64867, 4288, 8298}
    },
    /* Vector 1072: Angle 0.147 rad */
    {
        .input_axis = {-11239, -57530, 29309},
        .input_angle_rad = 9651,
        .expected_quaternion = {65358, -827, -4232, 2156},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64848, 4407, 8387}
    },
    /* Vector 1073: Angle 0.150 rad */
    {
        .input_axis = {-14100, -56896, 29309},
        .input_angle_rad = 9852,
        .expected_quaternion = {65351, -1059, -4273, 2201},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64831, 4527, 8450}
    },
    /* Vector 1074: Angle 0.153 rad */
    {
        .input_axis = {-16926, -56120, 29309},
        .input_angle_rad = 10053,
        .expected_quaternion = {65343, -1297, -4300, 2246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64818, 4648, 8486}
    },
    /* Vector 1075: Angle 0.156 rad */
    {
        .input_axis = {-19710, -55204, 29309},
        .input_angle_rad = 10254,
        .expected_quaternion = {65336, -1540, -4314, 2291},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64808, 4770, 8495}
    },
    /* Vector 1076: Angle 0.160 rad */
    {
        .input_axis = {-22444, -54150, 29309},
        .input_angle_rad = 10455,
        .expected_quaternion = {65328, -1788, -4315, 2335},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64801, 4891, 8475}
    },
    /* Vector 1077: Angle 0.163 rad */
    {
        .input_axis = {-25122, -52961, 29309},
        .input_angle_rad = 10656,
        .expected_quaternion = {65320, -2040, -4301, 2380},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64799, 5012, 8425}
    },
    /* Vector 1078: Angle 0.166 rad */
    {
        .input_axis = {-27738, -51639, 29309},
        .input_angle_rad = 10857,
        .expected_quaternion = {65311, -2295, -4273, 2425},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64799, 5133, 8346}
    },
    /* Vector 1079: Angle 0.169 rad */
    {
        .input_axis = {-30284, -50188, 29309},
        .input_angle_rad = 11058,
        .expected_quaternion = {65303, -2552, -4229, 2470},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64804, 5251, 8236}
    },
    /* Vector 1080: Angle 0.172 rad */
    {
        .input_axis = {-32755, -48612, 29309},
        .input_angle_rad = 11259,
        .expected_quaternion = {65294, -2810, -4171, 2515},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64812, 5368, 8095}
    },
    /* Vector 1081: Angle 0.175 rad */
    {
        .input_axis = {-35143, -46914, 29309},
        .input_angle_rad = 11461,
        .expected_quaternion = {65286, -3069, -4097, 2559},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64824, 5483, 7923}
    },
    /* Vector 1082: Angle 0.178 rad */
    {
        .input_axis = {-37444, -45099, 29309},
        .input_angle_rad = 11662,
        .expected_quaternion = {65277, -3327, -4007, 2604},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64839, 5595, 7718}
    },
    /* Vector 1083: Angle 0.181 rad */
    {
        .input_axis = {-39651, -43171, 29309},
        .input_angle_rad = 11863,
        .expected_quaternion = {65268, -3584, -3902, 2649},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64857, 5703, 7482}
    },
    /* Vector 1084: Angle 0.184 rad */
    {
        .input_axis = {-41759, -41135, 29309},
        .input_angle_rad = 12064,
        .expected_quaternion = {65259, -3838, -3781, 2694},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64878, 5807, 7214}
    },
    /* Vector 1085: Angle 0.187 rad */
    {
        .input_axis = {-43763, -38997, 29309},
        .input_angle_rad = 12265,
        .expected_quaternion = {65249, -4089, -3644, 2738},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64902, 5908, 6914}
    },
    /* Vector 1086: Angle 0.190 rad */
    {
        .input_axis = {-45657, -36761, 29309},
        .input_angle_rad = 12466,
        .expected_quaternion = {65240, -4336, -3491, 2783},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64928, 6003, 6582}
    },
    /* Vector 1087: Angle 0.193 rad */
    {
        .input_axis = {-47438, -34433, 29309},
        .input_angle_rad = 12667,
        .expected_quaternion = {65230, -4577, -3322, 2828},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64955, 6094, 6219}
    },
    /* Vector 1088: Angle 0.196 rad */
    {
        .input_axis = {-49099, -32019, 29309},
        .input_angle_rad = 12868,
        .expected_quaternion = {65220, -4813, -3138, 2873},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64984, 6179, 5825}
    },
    /* Vector 1089: Angle 0.199 rad */
    {
        .input_axis = {-50638, -29525, 29309},
        .input_angle_rad = 13069,
        .expected_quaternion = {65210, -5041, -2939, 2917},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65013, 6258, 5400}
    },
    /* Vector 1090: Angle 0.202 rad */
    {
        .input_axis = {-52051, -26957, 29309},
        .input_angle_rad = 13270,
        .expected_quaternion = {65200, -5261, -2725, 2962},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65042, 6332, 4946}
    },
    /* Vector 1091: Angle 0.206 rad */
    {
        .input_axis = {-53333, -24322, 29309},
        .input_angle_rad = 13471,
        .expected_quaternion = {65190, -5472, -2495, 3007},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65070, 6399, 4462}
    },
    /* Vector 1092: Angle 0.209 rad */
    {
        .input_axis = {-54482, -21626, 29309},
        .input_angle_rad = 13672,
        .expected_quaternion = {65180, -5673, -2252, 3052},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65097, 6460, 3951}
    },
    /* Vector 1093: Angle 0.212 rad */
    {
        .input_axis = {-55495, -18876, 29309},
        .input_angle_rad = 13873,
        .expected_quaternion = {65169, -5863, -1994, 3096},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65122, 6515, 3412}
    },
    /* Vector 1094: Angle 0.215 rad */
    {
        .input_axis = {-56369, -16079, 29309},
        .input_angle_rad = 14074,
        .expected_quaternion = {65159, -6041, -1723, 3141},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65144, 6564, 2848}
    },
    /* Vector 1095: Angle 0.218 rad */
    {
        .input_axis = {-57102, -13242, 29309},
        .input_angle_rad = 14275,
        .expected_quaternion = {65148, -6207, -1439, 3186},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65163, 6606, 2258}
    },
    /* Vector 1096: Angle 0.221 rad */
    {
        .input_axis = {-57692, -10371, 29309},
        .input_angle_rad = 14476,
        .expected_quaternion = {65137, -6359, -1143, 3230},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65178, 6643, 1645}
    },
    /* Vector 1097: Angle 0.224 rad */
    {
        .input_axis = {-58139, -7475, 29309},
        .input_angle_rad = 14678,
        .expected_quaternion = {65126, -6497, -835, 3275},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65187, 6675, 1011}
    },
    /* Vector 1098: Angle 0.227 rad */
    {
        .input_axis = {-58440, -4560, 29309},
        .input_angle_rad = 14879,
        .expected_quaternion = {65114, -6620, -517, 3320},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65192, 6701, 356}
    },
    /* Vector 1099: Angle 0.230 rad */
    {
        .input_axis = {-58594, -1634, 29309},
        .input_angle_rad = 15080,
        .expected_quaternion = {65103, -6726, -188, 3364},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65189, 6723, -318}
    },
    /* Vector 1100: Angle 0.233 rad */
    {
        .input_axis = {-58603, 1297, 29309},
        .input_angle_rad = 15281,
        .expected_quaternion = {65091, -6817, 151, 3409},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65181, 6741, -1009}
    },
    /* Vector 1101: Angle 0.236 rad */
    {
        .input_axis = {-58465, 4224, 29309},
        .input_angle_rad = 15482,
        .expected_quaternion = {65079, -6890, 498, 3454},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65164, 6755, -1715}
    },
    /* Vector 1102: Angle 0.239 rad */
    {
        .input_axis = {-58181, 7141, 29309},
        .input_angle_rad = 15683,
        .expected_quaternion = {65067, -6945, 852, 3498},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65140, 6766, -2434}
    },
    /* Vector 1103: Angle 0.242 rad */
    {
        .input_axis = {-57751, 10040, 29309},
        .input_angle_rad = 15884,
        .expected_quaternion = {65055, -6981, 1214, 3543},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65108, 6776, -3164}
    },
    /* Vector 1104: Angle 0.245 rad */
    {
        .input_axis = {-57177, 12914, 29309},
        .input_angle_rad = 16085,
        .expected_quaternion = {65043, -6999, 1581, 3588},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65067, 6784, -3904}
    },
    /* Vector 1105: Angle 0.249 rad */
    {
        .input_axis = {-56460, 15755, 29309},
        .input_angle_rad = 16286,
        .expected_quaternion = {65031, -6997, 1953, 3632},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {65017, 6792, -4651}
    },
    /* Vector 1106: Angle 0.252 rad */
    {
        .input_axis = {-55602, 18557, 29309},
        .input_angle_rad = 16487,
        .expected_quaternion = {65018, -6976, 2328, 3677},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64958, 6800, -5402}
    },
    /* Vector 1107: Angle 0.255 rad */
    {
        .input_axis = {-54605, 21313, 29309},
        .input_angle_rad = 16688,
        .expected_quaternion = {65006, -6934, 2706, 3722},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64890, 6810, -6156}
    },
    /* Vector 1108: Angle 0.258 rad */
    {
        .input_axis = {-53472, 24016, 29309},
        .input_angle_rad = 16889,
        .expected_quaternion = {64993, -6871, 3086, 3766},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64813, 6823, -6910}
    },
    /* Vector 1109: Angle 0.261 rad */
    {
        .input_axis = {-52205, 26658, 29309},
        .input_angle_rad = 17090,
        .expected_quaternion = {64980, -6788, 3466, 3811},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64726, 6839, -7663}
    },
    /* Vector 1110: Angle 0.264 rad */
    {
        .input_axis = {-50807, 29234, 29309},
        .input_angle_rad = 17291,
        .expected_quaternion = {64967, -6683, 3845, 3855},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64631, 6859, -8410}
    },
    /* Vector 1111: Angle 0.267 rad */
    {
        .input_axis = {-49282, 31737, 29309},
        .input_angle_rad = 17492,
        .expected_quaternion = {64953, -6558, 4223, 3900},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64528, 6885, -9151}
    },
    /* Vector 1112: Angle 0.270 rad */
    {
        .input_axis = {-47635, 34160, 29309},
        .input_angle_rad = 17693,
        .expected_quaternion = {64940, -6411, 4597, 3944},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64416, 6918, -9883}
    },
    /* Vector 1113: Angle 0.273 rad */
    {
        .input_axis = {-45868, 36498, 29309},
        .input_angle_rad = 17895,
        .expected_quaternion = {64926, -6243, 4967, 3989},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64297, 6957, -10602}
    },
    /* Vector 1114: Angle 0.276 rad */
    {
        .input_axis = {-43986, 38745, 29309},
        .input_angle_rad = 18096,
        .expected_quaternion = {64912, -6053, 5332, 4033},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64172, 7005, -11308}
    },
    /* Vector 1115: Angle 0.279 rad */
    {
        .input_axis = {-41995, 40895, 29309},
        .input_angle_rad = 18297,
        .expected_quaternion = {64899, -5843, 5690, 4078},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64040, 7062, -11997}
    },
    /* Vector 1116: Angle 0.282 rad */
    {
        .input_axis = {-39899, 42943, 29309},
        .input_angle_rad = 18498,
        .expected_quaternion = {64884, -5612, 6040, 4122},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63904, 7129, -12666}
    },
    /* Vector 1117: Angle 0.285 rad */
    {
        .input_axis = {-37702, 44883, 29309},
        .input_angle_rad = 18699,
        .expected_quaternion = {64870, -5360, 6381, 4167},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63763, 7205, -13315}
    },
    /* Vector 1118: Angle 0.288 rad */
    {
        .input_axis = {-35412, 46711, 29309},
        .input_angle_rad = 18900,
        .expected_quaternion = {64856, -5089, 6712, 4211},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63620, 7293, -13939}
    },
    /* Vector 1119: Angle 0.291 rad */
    {
        .input_axis = {-33033, 48423, 29309},
        .input_angle_rad = 19101,
        .expected_quaternion = {64841, -4797, 7032, 4256},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63474, 7392, -14537}
    },
    /* Vector 1120: Angle 0.295 rad */
    {
        .input_axis = {-30572, 50013, 29309},
        .input_angle_rad = 19302,
        .expected_quaternion = {64827, -4486, 7338, 4300},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63328, 7503, -15107}
    },
    /* Vector 1121: Angle 0.298 rad */
    {
        .input_axis = {-28034, 51479, 29309},
        .input_angle_rad = 19503,
        .expected_quaternion = {64812, -4156, 7632, 4345},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63182, 7626, -15646}
    },
    /* Vector 1122: Angle 0.301 rad */
    {
        .input_axis = {-25426, 52816, 29309},
        .input_angle_rad = 19704,
        .expected_quaternion = {64797, -3808, 7910, 4389},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63039, 7761, -16151}
    },
    /* Vector 1123: Angle 0.304 rad */
    {
        .input_axis = {-22755, 54020, 29309},
        .input_angle_rad = 19905,
        .expected_quaternion = {64782, -3442, 8172, 4434},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62898, 7907, -16622}
    },
    /* Vector 1124: Angle 0.307 rad */
    {
        .input_axis = {-20026, 55090, 29309},
        .input_angle_rad = 20106,
        .expected_quaternion = {64766, -3060, 8418, 4478},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62762, 8065, -17056}
    },
    /* Vector 1125: Angle 0.310 rad */
    {
        .input_axis = {-17248, 56022, 29309},
        .input_angle_rad = 20307,
        .expected_quaternion = {64751, -2662, 8645, 4523},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62631, 8235, -17450}
    },
    /* Vector 1126: Angle 0.313 rad */
    {
        .input_axis = {-14426, 56814, 29309},
        .input_angle_rad = 20508,
        .expected_quaternion = {64735, -2248, 8853, 4567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62507, 8415, -17804}
    },
    /* Vector 1127: Angle 0.316 rad */
    {
        .input_axis = {-11569, 57464, 29309},
        .input_angle_rad = 20709,
        .expected_quaternion = {64720, -1820, 9042, 4612},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62392, 8606, -18114}
    },
    /* Vector 1128: Angle 0.319 rad */
    {
        .input_axis = {-8682, 57971, 29309},
        .input_angle_rad = 20910,
        .expected_quaternion = {64704, -1379, 9209, 4656},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62286, 8806, -18380}
    },
    /* Vector 1129: Angle 0.322 rad */
    {
        .input_axis = {-5774, 58332, 29309},
        .input_angle_rad = 21112,
        .expected_quaternion = {64688, -926, 9355, 4700},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62191, 9015, -18600}
    },
    /* Vector 1130: Angle 0.325 rad */
    {
        .input_axis = {-2852, 58548, 29309},
        .input_angle_rad = 21313,
        .expected_quaternion = {64672, -462, 9478, 4745},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62107, 9231, -18773}
    },
    /* Vector 1131: Angle 0.328 rad */
    {
        .input_axis = {78, 58617, 29309},
        .input_angle_rad = 21514,
        .expected_quaternion = {64655, 13, 9578, 4789},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62036, 9453, -18897}
    },
    /* Vector 1132: Angle 0.331 rad */
    {
        .input_axis = {3008, 58540, 29309},
        .input_angle_rad = 21715,
        .expected_quaternion = {64639, 496, 9654, 4833},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61979, 9681, -18970}
    },
    /* Vector 1133: Angle 0.334 rad */
    {
        .input_axis = {5930, 58316, 29309},
        .input_angle_rad = 21916,
        .expected_quaternion = {64622, 987, 9705, 4878},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61935, 9912, -18993}
    },
    /* Vector 1134: Angle 0.337 rad */
    {
        .input_axis = {8837, 57947, 29309},
        .input_angle_rad = 22117,
        .expected_quaternion = {64605, 1484, 9732, 4922},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61907, 10145, -18964}
    },
    /* Vector 1135: Angle 0.341 rad */
    {
        .input_axis = {11722, 57433, 29309},
        .input_angle_rad = 22318,
        .expected_quaternion = {64588, 1986, 9732, 4966},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61893, 10379, -18882}
    },
    /* Vector 1136: Angle 0.344 rad */
    {
        .input_axis = {14578, 56776, 29309},
        .input_angle_rad = 22519,
        .expected_quaternion = {64571, 2492, 9706, 5011},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61895, 10612, -18746}
    },
    /* Vector 1137: Angle 0.347 rad */
    {
        .input_axis = {17397, 55976, 29309},
        .input_angle_rad = 22720,
        .expected_quaternion = {64554, 3001, 9654, 5055},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61912, 10842, -18556}
    },
    /* Vector 1138: Angle 0.350 rad */
    {
        .input_axis = {20173, 55037, 29309},
        .input_angle_rad = 22921,
        .expected_quaternion = {64536, 3510, 9575, 5099},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61944, 11069, -18313}
    },
    /* Vector 1139: Angle 0.353 rad */
    {
        .input_axis = {22899, 53960, 29309},
        .input_angle_rad = 23122,
        .expected_quaternion = {64519, 4019, 9470, 5143},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61992, 11289, -18014}
    },
    /* Vector 1140: Angle 0.356 rad */
    {
        .input_axis = {25567, 52748, 29309},
        .input_angle_rad = 23323,
        .expected_quaternion = {64501, 4525, 9337, 5188},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62054, 11501, -17662}
    },
    /* Vector 1141: Angle 0.359 rad */
    {
        .input_axis = {28171, 51404, 29309},
        .input_angle_rad = 23524,
        .expected_quaternion = {64483, 5029, 9176, 5232},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62131, 11704, -17255}
    },
    /* Vector 1142: Angle 0.362 rad */
    {
        .input_axis = {30705, 49932, 29309},
        .input_angle_rad = 23725,
        .expected_quaternion = {64465, 5528, 8989, 5276},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62221, 11896, -16794}
    },
    /* Vector 1143: Angle 0.365 rad */
    {
        .input_axis = {33162, 48335, 29309},
        .input_angle_rad = 23926,
        .expected_quaternion = {64447, 6020, 8774, 5320},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62323, 12076, -16280}
    },
    /* Vector 1144: Angle 0.368 rad */
    {
        .input_axis = {35536, 46617, 29309},
        .input_angle_rad = 24127,
        .expected_quaternion = {64429, 6505, 8533, 5365},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62436, 12242, -15712}
    },
    /* Vector 1145: Angle 0.371 rad */
    {
        .input_axis = {37822, 44783, 29309},
        .input_angle_rad = 24328,
        .expected_quaternion = {64410, 6980, 8265, 5409},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62559, 12392, -15093}
    },
    /* Vector 1146: Angle 0.374 rad */
    {
        .input_axis = {40013, 42836, 29309},
        .input_angle_rad = 24530,
        .expected_quaternion = {64392, 7445, 7970, 5453},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62690, 12526, -14423}
    },
    /* Vector 1147: Angle 0.377 rad */
    {
        .input_axis = {42104, 40783, 29309},
        .input_angle_rad = 24731,
        .expected_quaternion = {64373, 7897, 7649, 5497},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62828, 12643, -13702}
    },
    /* Vector 1148: Angle 0.380 rad */
    {
        .input_axis = {44089, 38628, 29309},
        .input_angle_rad = 24932,
        .expected_quaternion = {64354, 8336, 7303, 5541},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62971, 12741, -12933}
    },
    /* Vector 1149: Angle 0.383 rad */
    {
        .input_axis = {45965, 36376, 29309},
        .input_angle_rad = 25133,
        .expected_quaternion = {64335, 8760, 6932, 5585},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63117, 12819, -12117}
    },
    /* Vector 1150: Angle 0.387 rad */
    {
        .input_axis = {47726, 34033, 29309},
        .input_angle_rad = 25334,
        .expected_quaternion = {64316, 9167, 6537, 5630},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63265, 12878, -11256}
    },
    /* Vector 1151: Angle 0.390 rad */
    {
        .input_axis = {49367, 31605, 29309},
        .input_angle_rad = 25535,
        .expected_quaternion = {64296, 9557, 6118, 5674},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63411, 12917, -10350}
    },
    /* Vector 1152: Angle 0.393 rad */
    {
        .input_axis = {50885, 29098, 29309},
        .input_angle_rad = 25736,
        .expected_quaternion = {64277, 9927, 5677, 5718},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63555, 12936, -9403}
    },
    /* Vector 1153: Angle 0.396 rad */
    {
        .input_axis = {52275, 26519, 29309},
        .input_angle_rad = 25937,
        .expected_quaternion = {64257, 10277, 5213, 5762},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63693, 12934, -8416}
    },
    /* Vector 1154: Angle 0.399 rad */
    {
        .input_axis = {53536, 23873, 29309},
        .input_angle_rad = 26138,
        .expected_quaternion = {64237, 10605, 4729, 5806},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63825, 12912, -7392}
    },
    /* Vector 1155: Angle 0.402 rad */
    {
        .input_axis = {54662, 21168, 29309},
        .input_angle_rad = 26339,
        .expected_quaternion = {64217, 10911, 4225, 5850},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63947, 12871, -6332}
    },
    /* Vector 1156: Angle 0.405 rad */
    {
        .input_axis = {55651, 18409, 29309},
        .input_angle_rad = 26540,
        .expected_quaternion = {64197, 11192, 3702, 5894},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64058, 12812, -5240}
    },
    /* Vector 1157: Angle 0.408 rad */
    {
        .input_axis = {56502, 15605, 29309},
        .input_angle_rad = 26741,
        .expected_quaternion = {64177, 11448, 3162, 5938},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64155, 12734, -4118}
    },
    /* Vector 1158: Angle 0.411 rad */
    {
        .input_axis = {57211, 12761, 29309},
        .input_angle_rad = 26942,
        .expected_quaternion = {64156, 11677, 2605, 5982},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64237, 12641, -2968}
    },
    /* Vector 1159: Angle 0.414 rad */
    {
        .input_axis = {57778, 9886, 29309},
        .input_angle_rad = 27143,
        .expected_quaternion = {64136, 11880, 2033, 6026},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64302, 12532, -1794}
    },
    /* Vector 1160: Angle 0.417 rad */
    {
        .input_axis = {58199, 6986, 29309},
        .input_angle_rad = 27344,
        .expected_quaternion = {64115, 12054, 1447, 6070},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64348, 12409, -598}
    },
    /* Vector 1161: Angle 0.420 rad */
    {
        .input_axis = {58476, 4069, 29309},
        .input_angle_rad = 27545,
        .expected_quaternion = {64094, 12199, 849, 6114},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64373, 12275, 616}
    },
    /* Vector 1162: Angle 0.423 rad */
    {
        .input_axis = {58606, 1141, 29309},
        .input_angle_rad = 27747,
        .expected_quaternion = {64073, 12314, 240, 6158},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64377, 12131, 1845}
    },
    /* Vector 1163: Angle 0.426 rad */
    {
        .input_axis = {58590, -1790, 29309},
        .input_angle_rad = 27948,
        .expected_quaternion = {64052, 12398, -379, 6202},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64358, 11980, 3087}
    },
    /* Vector 1164: Angle 0.430 rad */
    {
        .input_axis = {58427, -4716, 29309},
        .input_angle_rad = 28149,
        .expected_quaternion = {64031, 12451, -1005, 6246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64315, 11823, 4337}
    },
    /* Vector 1165: Angle 0.433 rad */
    {
        .input_axis = {58118, -7630, 29309},
        .input_angle_rad = 28350,
        .expected_quaternion = {64009, 12473, -1637, 6290},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64247, 11663, 5593}
    },
    /* Vector 1166: Angle 0.436 rad */
    {
        .input_axis = {57665, -10525, 29309},
        .input_angle_rad = 28551,
        .expected_quaternion = {63987, 12462, -2275, 6334},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64154, 11503, 6850}
    },
    /* Vector 1167: Angle 0.439 rad */
    {
        .input_axis = {57066, -13394, 29309},
        .input_angle_rad = 28752,
        .expected_quaternion = {63966, 12418, -2915, 6378},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {64035, 11345, 8106}
    },
    /* Vector 1168: Angle 0.442 rad */
    {
        .input_axis = {56326, -16229, 29309},
        .input_angle_rad = 28953,
        .expected_quaternion = {63944, 12341, -3556, 6422},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63892, 11192, 9357}
    },
    /* Vector 1169: Angle 0.445 rad */
    {
        .input_axis = {55444, -19024, 29309},
        .input_angle_rad = 29154,
        .expected_quaternion = {63922, 12231, -4197, 6465},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63723, 11046, 10600}
    },
    /* Vector 1170: Angle 0.448 rad */
    {
        .input_axis = {54424, -21771, 29309},
        .input_angle_rad = 29355,
        .expected_quaternion = {63899, 12087, -4835, 6509},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63529, 10910, 11830}
    },
    /* Vector 1171: Angle 0.451 rad */
    {
        .input_axis = {53268, -24464, 29309},
        .input_angle_rad = 29556,
        .expected_quaternion = {63877, 11910, -5470, 6553},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63312, 10786, 13045}
    },
    /* Vector 1172: Angle 0.454 rad */
    {
        .input_axis = {51979, -27096, 29309},
        .input_angle_rad = 29757,
        .expected_quaternion = {63854, 11700, -6099, 6597},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63073, 10678, 14240}
    },
    /* Vector 1173: Angle 0.457 rad */
    {
        .input_axis = {50559, -29660, 29309},
        .input_angle_rad = 29958,
        .expected_quaternion = {63832, 11456, -6720, 6641},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62812, 10587, 15413}
    },
    /* Vector 1174: Angle 0.460 rad */
    {
        .input_axis = {49014, -32150, 29309},
        .input_angle_rad = 30159,
        .expected_quaternion = {63809, 11179, -7332, 6684},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62532, 10515, 16559}
    },
    /* Vector 1175: Angle 0.463 rad */
    {
        .input_axis = {47346, -34559, 29309},
        .input_angle_rad = 30360,
        .expected_quaternion = {63786, 10869, -7934, 6728},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62234, 10466, 17675}
    },
    /* Vector 1176: Angle 0.466 rad */
    {
        .input_axis = {45559, -36882, 29309},
        .input_angle_rad = 30561,
        .expected_quaternion = {63763, 10527, -8522, 6772},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61920, 10440, 18758}
    },
    /* Vector 1177: Angle 0.469 rad */
    {
        .input_axis = {43659, -39113, 29309},
        .input_angle_rad = 30762,
        .expected_quaternion = {63739, 10153, -9096, 6816},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61593, 10439, 19805}
    },
    /* Vector 1178: Angle 0.472 rad */
    {
        .input_axis = {41650, -41247, 29309},
        .input_angle_rad = 30964,
        .expected_quaternion = {63716, 9748, -9653, 6859},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61256, 10466, 20811}
    },
    /* Vector 1179: Angle 0.476 rad */
    {
        .input_axis = {39536, -43277, 29309},
        .input_angle_rad = 31165,
        .expected_quaternion = {63692, 9312, -10193, 6903},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60911, 10521, 21774}
    },
    /* Vector 1180: Angle 0.479 rad */
    {
        .input_axis = {37324, -45199, 29309},
        .input_angle_rad = 31366,
        .expected_quaternion = {63668, 8847, -10713, 6947},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60561, 10605, 22691}
    },
    /* Vector 1181: Angle 0.482 rad */
    {
        .input_axis = {35018, -47007, 29309},
        .input_angle_rad = 31567,
        .expected_quaternion = {63645, 8352, -11212, 6990},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60208, 10720, 23558}
    },
    /* Vector 1182: Angle 0.485 rad */
    {
        .input_axis = {32625, -48699, 29309},
        .input_angle_rad = 31768,
        .expected_quaternion = {63621, 7830, -11688, 7034},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59857, 10864, 24373}
    },
    /* Vector 1183: Angle 0.488 rad */
    {
        .input_axis = {30150, -50269, 29309},
        .input_angle_rad = 31969,
        .expected_quaternion = {63596, 7281, -12139, 7078},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59510, 11039, 25133}
    },
    /* Vector 1184: Angle 0.491 rad */
    {
        .input_axis = {27600, -51713, 29309},
        .input_angle_rad = 32170,
        .expected_quaternion = {63572, 6706, -12565, 7121},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59170, 11244, 25835}
    },
    /* Vector 1185: Angle 0.494 rad */
    {
        .input_axis = {24981, -53027, 29309},
        .input_angle_rad = 32371,
        .expected_quaternion = {63547, 6107, -12964, 7165},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58841, 11479, 26476}
    },
    /* Vector 1186: Angle 0.497 rad */
    {
        .input_axis = {22300, -54210, 29309},
        .input_angle_rad = 32572,
        .expected_quaternion = {63523, 5485, -13333, 7209},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58525, 11743, 27054}
    },
    /* Vector 1187: Angle 0.500 rad */
    {
        .input_axis = {19562, -55257, 29309},
        .input_angle_rad = 32773,
        .expected_quaternion = {63498, 4841, -13673, 7252},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58226, 12034, 27566}
    },
    /* Vector 1188: Angle 0.503 rad */
    {
        .input_axis = {16776, -56165, 29309},
        .input_angle_rad = 32974,
        .expected_quaternion = {63473, 4176, -13981, 7296},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57946, 12350, 28012}
    },
    /* Vector 1189: Angle 0.506 rad */
    {
        .input_axis = {13948, -56933, 29309},
        .input_angle_rad = 33175,
        .expected_quaternion = {63448, 3493, -14257, 7339},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57689, 12691, 28388}
    },
    /* Vector 1190: Angle 0.509 rad */
    {
        .input_axis = {11085, -57559, 29309},
        .input_angle_rad = 33376,
        .expected_quaternion = {63423, 2792, -14499, 7383},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57457, 13054, 28692}
    },
    /* Vector 1191: Angle 0.512 rad */
    {
        .input_axis = {8195, -58042, 29309},
        .input_angle_rad = 33577,
        .expected_quaternion = {63397, 2076, -14707, 7426},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57252, 13436, 28924}
    },
    /* Vector 1192: Angle 0.515 rad */
    {
        .input_axis = {5284, -58379, 29309},
        .input_angle_rad = 33778,
        .expected_quaternion = {63372, 1347, -14879, 7470},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57077, 13835, 29082}
    },
    /* Vector 1193: Angle 0.518 rad */
    {
        .input_axis = {2359, -58570, 29309},
        .input_angle_rad = 33979,
        .expected_quaternion = {63346, 605, -15014, 7513},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56934, 14247, 29164}
    },
    /* Vector 1194: Angle 0.522 rad */
    {
        .input_axis = {-571, -58614, 29309},
        .input_angle_rad = 34181,
        .expected_quaternion = {63320, -147, -15113, 7557},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56823, 14670, 29169}
    },
    /* Vector 1195: Angle 0.525 rad */
    {
        .input_axis = {-3500, -58513, 29309},
        .input_angle_rad = 34382,
        .expected_quaternion = {63294, -908, -15173, 7600},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56747, 15100, 29098}
    },
    /* Vector 1196: Angle 0.528 rad */
    {
        .input_axis = {-6420, -58265, 29309},
        .input_angle_rad = 34583,
        .expected_quaternion = {63268, -1674, -15195, 7644},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56707, 15534, 28948}
    },
    /* Vector 1197: Angle 0.531 rad */
    {
        .input_axis = {-9324, -57871, 29309},
        .input_angle_rad = 34784,
        .expected_quaternion = {63242, -2445, -15178, 7687},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56702, 15968, 28720}
    },
    /* Vector 1198: Angle 0.534 rad */
    {
        .input_axis = {-12204, -57333, 29309},
        .input_angle_rad = 34985,
        .expected_quaternion = {63215, -3219, -15122, 7730},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56734, 16399, 28413}
    },
    /* Vector 1199: Angle 0.537 rad */
    {
        .input_axis = {-15055, -56651, 29309},
        .input_angle_rad = 35186,
        .expected_quaternion = {63189, -3993, -15026, 7774},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56802, 16821, 28028}
    },
    /* Vector 1200: Angle 0.540 rad */
    {
        .input_axis = {-17867, -55828, 29309},
        .input_angle_rad = 35387,
        .expected_quaternion = {63162, -4765, -14890, 7817},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56905, 17233, 27564}
    },
    /* Vector 1201: Angle 0.543 rad */
    {
        .input_axis = {-20635, -54865, 29309},
        .input_angle_rad = 35588,
        .expected_quaternion = {63135, -5534, -14714, 7860},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57043, 17630, 27023}
    },
    /* Vector 1202: Angle 0.546 rad */
    {
        .input_axis = {-23351, -53765, 29309},
        .input_angle_rad = 35789,
        .expected_quaternion = {63108, -6297, -14499, 7904},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57214, 18008, 26404}
    },
    /* Vector 1203: Angle 0.549 rad */
    {
        .input_axis = {-26009, -52531, 29309},
        .input_angle_rad = 35990,
        .expected_quaternion = {63081, -7052, -14243, 7947},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57417, 18364, 25709}
    },
    /* Vector 1204: Angle 0.552 rad */
    {
        .input_axis = {-28602, -51165, 29309},
        .input_angle_rad = 36191,
        .expected_quaternion = {63054, -7798, -13949, 7990},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57650, 18694, 24939}
    },
    /* Vector 1205: Angle 0.555 rad */
    {
        .input_axis = {-31124, -49672, 29309},
        .input_angle_rad = 36392,
        .expected_quaternion = {63026, -8531, -13615, 8033},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57910, 18996, 24096}
    },
    /* Vector 1206: Angle 0.558 rad */
    {
        .input_axis = {-33567, -48054, 29309},
        .input_angle_rad = 36593,
        .expected_quaternion = {62998, -9250, -13242, 8077},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58194, 19266, 23179}
    },
    /* Vector 1207: Angle 0.561 rad */
    {
        .input_axis = {-35927, -46316, 29309},
        .input_angle_rad = 36794,
        .expected_quaternion = {62971, -9953, -12832, 8120},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58499, 19502, 22193}
    },
    /* Vector 1208: Angle 0.565 rad */
    {
        .input_axis = {-38197, -44463, 29309},
        .input_angle_rad = 36995,
        .expected_quaternion = {62943, -10639, -12384, 8163},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58822, 19701, 21137}
    },
    /* Vector 1209: Angle 0.568 rad */
    {
        .input_axis = {-40372, -42498, 29309},
        .input_angle_rad = 37196,
        .expected_quaternion = {62915, -11304, -11899, 8206},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59160, 19861, 20016}
    },
    /* Vector 1210: Angle 0.571 rad */
    {
        .input_axis = {-42445, -40428, 29309},
        .input_angle_rad = 37398,
        .expected_quaternion = {62886, -11947, -11379, 8249},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59508, 19980, 18830}
    },
    /* Vector 1211: Angle 0.574 rad */
    {
        .input_axis = {-44413, -38256, 29309},
        .input_angle_rad = 37599,
        .expected_quaternion = {62858, -12566, -10824, 8292},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59862, 20058, 17583}
    },
    /* Vector 1212: Angle 0.577 rad */
    {
        .input_axis = {-46269, -35988, 29309},
        .input_angle_rad = 37800,
        .expected_quaternion = {62830, -13159, -10235, 8336},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60219, 20093, 16278}
    },
    /* Vector 1213: Angle 0.580 rad */
    {
        .input_axis = {-48010, -33631, 29309},
        .input_angle_rad = 38001,
        .expected_quaternion = {62801, -13725, -9614, 8379},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60573, 20085, 14917}
    },
    /* Vector 1214: Angle 0.583 rad */
    {
        .input_axis = {-49631, -31189, 29309},
        .input_angle_rad = 38202,
        .expected_quaternion = {62772, -14261, -8962, 8422},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60920, 20034, 13503}
    },
    /* Vector 1215: Angle 0.586 rad */
    {
        .input_axis = {-51128, -28670, 29309},
        .input_angle_rad = 38403,
        .expected_quaternion = {62743, -14766, -8280, 8465},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61257, 19940, 12040}
    },
    /* Vector 1216: Angle 0.589 rad */
    {
        .input_axis = {-52497, -26078, 29309},
        .input_angle_rad = 38604,
        .expected_quaternion = {62714, -15239, -7570, 8508},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61578, 19804, 10532}
    },
    /* Vector 1217: Angle 0.592 rad */
    {
        .input_axis = {-53734, -23422, 29309},
        .input_angle_rad = 38805,
        .expected_quaternion = {62685, -15677, -6833, 8551},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61880, 19627, 8981}
    },
    /* Vector 1218: Angle 0.595 rad */
    {
        .input_axis = {-54838, -20707, 29309},
        .input_angle_rad = 39006,
        .expected_quaternion = {62655, -16079, -6072, 8594},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62157, 19412, 7393}
    },
    /* Vector 1219: Angle 0.598 rad */
    {
        .input_axis = {-55804, -17941, 29309},
        .input_angle_rad = 39207,
        .expected_quaternion = {62626, -16445, -5287, 8637},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62407, 19160, 5770}
    },
    /* Vector 1220: Angle 0.601 rad */
    {
        .input_axis = {-56631, -15129, 29309},
        .input_angle_rad = 39408,
        .expected_quaternion = {62596, -16771, -4481, 8680},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62624, 18874, 4117}
    },
    /* Vector 1221: Angle 0.604 rad */
    {
        .input_axis = {-57316, -12280, 29309},
        .input_angle_rad = 39609,
        .expected_quaternion = {62566, -17058, -3655, 8723},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62806, 18557, 2437}
    },
    /* Vector 1222: Angle 0.607 rad */
    {
        .input_axis = {-57859, -9400, 29309},
        .input_angle_rad = 39810,
        .expected_quaternion = {62536, -17304, -2811, 8766},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62950, 18213, 736}
    },
    /* Vector 1223: Angle 0.611 rad */
    {
        .input_axis = {-58256, -6496, 29309},
        .input_angle_rad = 40011,
        .expected_quaternion = {62506, -17508, -1952, 8808},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63052, 17846, -982}
    },
    /* Vector 1224: Angle 0.614 rad */
    {
        .input_axis = {-58508, -3577, 29309},
        .input_angle_rad = 40212,
        .expected_quaternion = {62476, -17670, -1080, 8851},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63109, 17459, -2713}
    },
    /* Vector 1225: Angle 0.617 rad */
    {
        .input_axis = {-58614, -648, 29309},
        .input_angle_rad = 40413,
        .expected_quaternion = {62445, -17787, -197, 8894},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63121, 17056, -4453}
    },
    /* Vector 1226: Angle 0.620 rad */
    {
        .input_axis = {-58573, 2282, 29309},
        .input_angle_rad = 40615,
        .expected_quaternion = {62415, -17861, 696, 8937},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {63084, 16644, -6197}
    },
    /* Vector 1227: Angle 0.623 rad */
    {
        .input_axis = {-58385, 5207, 29309},
        .input_angle_rad = 40816,
        .expected_quaternion = {62384, -17889, 1595, 8980},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62997, 16225, -7939}
    },
    /* Vector 1228: Angle 0.626 rad */
    {
        .input_axis = {-58052, 8118, 29309},
        .input_angle_rad = 41017,
        .expected_quaternion = {62353, -17871, 2499, 9023},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62861, 15806, -9677}
    },
    /* Vector 1229: Angle 0.629 rad */
    {
        .input_axis = {-57574, 11010, 29309},
        .input_angle_rad = 41218,
        .expected_quaternion = {62322, -17808, 3405, 9065},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62674, 15391, -11403}
    },
    /* Vector 1230: Angle 0.632 rad */
    {
        .input_axis = {-56952, 13873, 29309},
        .input_angle_rad = 41419,
        .expected_quaternion = {62291, -17699, 4311, 9108},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62437, 14986, -13115}
    },
    /* Vector 1231: Angle 0.635 rad */
    {
        .input_axis = {-56187, 16702, 29309},
        .input_angle_rad = 41620,
        .expected_quaternion = {62260, -17543, 5215, 9151},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {62151, 14595, -14807}
    },
    /* Vector 1232: Angle 0.638 rad */
    {
        .input_axis = {-55282, 19490, 29309},
        .input_angle_rad = 41821,
        .expected_quaternion = {62228, -17341, 6114, 9194},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61816, 14224, -16475}
    },
    /* Vector 1233: Angle 0.641 rad */
    {
        .input_axis = {-54239, 22228, 29309},
        .input_angle_rad = 42022,
        .expected_quaternion = {62197, -17093, 7005, 9236},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61435, 13877, -18114}
    },
    /* Vector 1234: Angle 0.644 rad */
    {
        .input_axis = {-53060, 24911, 29309},
        .input_angle_rad = 42223,
        .expected_quaternion = {62165, -16799, 7887, 9279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61010, 13560, -19719}
    },
    /* Vector 1235: Angle 0.647 rad */
    {
        .input_axis = {-51749, 27532, 29309},
        .input_angle_rad = 42424,
        .expected_quaternion = {62133, -16459, 8757, 9322},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60544, 13277, -21286}
    },
    /* Vector 1236: Angle 0.650 rad */
    {
        .input_axis = {-50308, 30084, 29309},
        .input_angle_rad = 42625,
        .expected_quaternion = {62101, -16074, 9612, 9364},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60041, 13032, -22810}
    },
    /* Vector 1237: Angle 0.653 rad */
    {
        .input_axis = {-48742, 32561, 29309},
        .input_angle_rad = 42826,
        .expected_quaternion = {62069, -15644, 10451, 9407},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59503, 12829, -24286}
    },
    /* Vector 1238: Angle 0.657 rad */
    {
        .input_axis = {-47054, 34956, 29309},
        .input_angle_rad = 43027,
        .expected_quaternion = {62036, -15170, 11270, 9449},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58935, 12672, -25711}
    },
    /* Vector 1239: Angle 0.660 rad */
    {
        .input_axis = {-45248, 37264, 29309},
        .input_angle_rad = 43228,
        .expected_quaternion = {62004, -14654, 12068, 9492},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58342, 12564, -27081}
    },
    /* Vector 1240: Angle 0.663 rad */
    {
        .input_axis = {-43329, 39479, 29309},
        .input_angle_rad = 43429,
        .expected_quaternion = {61971, -14095, 12843, 9534},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57728, 12507, -28390}
    },
    /* Vector 1241: Angle 0.666 rad */
    {
        .input_axis = {-41301, 41595, 29309},
        .input_angle_rad = 43630,
        .expected_quaternion = {61939, -13496, 13592, 9577},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57099, 12505, -29636}
    },
    /* Vector 1242: Angle 0.669 rad */
    {
        .input_axis = {-39171, 43608, 29309},
        .input_angle_rad = 43832,
        .expected_quaternion = {61906, -12856, 14312, 9619},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56461, 12558, -30813}
    },
    /* Vector 1243: Angle 0.672 rad */
    {
        .input_axis = {-36942, 45511, 29309},
        .input_angle_rad = 44033,
        .expected_quaternion = {61873, -12178, 15003, 9662},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55818, 12668, -31920}
    },
    /* Vector 1244: Angle 0.675 rad */
    {
        .input_axis = {-34622, 47300, 29309},
        .input_angle_rad = 44234,
        .expected_quaternion = {61839, -11463, 15661, 9704},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55177, 12835, -32951}
    },
    /* Vector 1245: Angle 0.678 rad */
    {
        .input_axis = {-32214, 48971, 29309},
        .input_angle_rad = 44435,
        .expected_quaternion = {61806, -10713, 16286, 9747},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54543, 13060, -33904}
    },
    /* Vector 1246: Angle 0.681 rad */
    {
        .input_axis = {-29727, 50520, 29309},
        .input_angle_rad = 44636,
        .expected_quaternion = {61772, -9929, 16874, 9789},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53923, 13341, -34776}
    },
    /* Vector 1247: Angle 0.684 rad */
    {
        .input_axis = {-27164, 51943, 29309},
        .input_angle_rad = 44837,
        .expected_quaternion = {61739, -9112, 17424, 9831},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53321, 13678, -35563}
    },
    /* Vector 1248: Angle 0.687 rad */
    {
        .input_axis = {-24534, 53236, 29309},
        .input_angle_rad = 45038,
        .expected_quaternion = {61705, -8265, 17935, 9874},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52745, 14069, -36263}
    },
    /* Vector 1249: Angle 0.690 rad */
    {
        .input_axis = {-21843, 54395, 29309},
        .input_angle_rad = 45239,
        .expected_quaternion = {61671, -7390, 18404, 9916},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52199, 14512, -36873}
    },
    /* Vector 1250: Angle 0.693 rad */
    {
        .input_axis = {-19097, 55419, 29309},
        .input_angle_rad = 45440,
        .expected_quaternion = {61637, -6489, 18830, 9958},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51689, 15003, -37392}
    },
    /* Vector 1251: Angle 0.696 rad */
    {
        .input_axis = {-16304, 56304, 29309},
        .input_angle_rad = 45641,
        .expected_quaternion = {61603, -5563, 19212, 10001},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51220, 15539, -37816}
    },
    /* Vector 1252: Angle 0.699 rad */
    {
        .input_axis = {-13469, 57049, 29309},
        .input_angle_rad = 45842,
        .expected_quaternion = {61568, -4615, 19548, 10043},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50796, 16116, -38144}
    },
    /* Vector 1253: Angle 0.703 rad */
    {
        .input_axis = {-10601, 57651, 29309},
        .input_angle_rad = 46043,
        .expected_quaternion = {61534, -3648, 19838, 10085},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50422, 16730, -38375}
    },
    /* Vector 1254: Angle 0.706 rad */
    {
        .input_axis = {-7706, 58108, 29309},
        .input_angle_rad = 46244,
        .expected_quaternion = {61499, -2663, 20079, 10127},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50103, 17375, -38507}
    },
    /* Vector 1255: Angle 0.709 rad */
    {
        .input_axis = {-4793, 58421, 29309},
        .input_angle_rad = 46445,
        .expected_quaternion = {61464, -1663, 20271, 10170},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49840, 18047, -38539}
    },
    /* Vector 1256: Angle 0.712 rad */
    {
        .input_axis = {-1867, 58587, 29309},
        .input_angle_rad = 46646,
        .expected_quaternion = {61429, -650, 20413, 10212},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49637, 18738, -38470}
    },
    /* Vector 1257: Angle 0.715 rad */
    {
        .input_axis = {1064, 58608, 29309},
        .input_angle_rad = 46847,
        .expected_quaternion = {61394, 372, 20504, 10254},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49497, 19444, -38300}
    },
    /* Vector 1258: Angle 0.718 rad */
    {
        .input_axis = {3991, 58481, 29309},
        .input_angle_rad = 47048,
        .expected_quaternion = {61359, 1402, 20544, 10296},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49421, 20158, -38029}
    },
    /* Vector 1259: Angle 0.721 rad */
    {
        .input_axis = {6909, 58209, 29309},
        .input_angle_rad = 47250,
        .expected_quaternion = {61324, 2437, 20532, 10338},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49410, 20874, -37655}
    },
    /* Vector 1260: Angle 0.724 rad */
    {
        .input_axis = {9810, 57790, 29309},
        .input_angle_rad = 47451,
        .expected_quaternion = {61288, 3474, 20467, 10380},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49464, 21585, -37181}
    },
    /* Vector 1261: Angle 0.727 rad */
    {
        .input_axis = {12686, 57228, 29309},
        .input_angle_rad = 47652,
        .expected_quaternion = {61253, 4511, 20350, 10422},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49583, 22283, -36605}
    },
    /* Vector 1262: Angle 0.730 rad */
    {
        .input_axis = {15530, 56522, 29309},
        .input_angle_rad = 47853,
        .expected_quaternion = {61217, 5545, 20180, 10464},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49766, 22964, -35930}
    },
    /* Vector 1263: Angle 0.733 rad */
    {
        .input_axis = {18336, 55676, 29309},
        .input_angle_rad = 48054,
        .expected_quaternion = {61181, 6573, 19958, 10506},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50012, 23619, -35155}
    },
    /* Vector 1264: Angle 0.736 rad */
    {
        .input_axis = {21096, 54690, 29309},
        .input_angle_rad = 48255,
        .expected_quaternion = {61145, 7592, 19683, 10548},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50318, 24243, -34283}
    },
    /* Vector 1265: Angle 0.739 rad */
    {
        .input_axis = {23803, 53567, 29309},
        .input_angle_rad = 48456,
        .expected_quaternion = {61108, 8600, 19355, 10590},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50681, 24829, -33315}
    },
    /* Vector 1266: Angle 0.742 rad */
    {
        .input_axis = {26450, 52310, 29309},
        .input_angle_rad = 48657,
        .expected_quaternion = {61072, 9595, 18976, 10632},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51098, 25372, -32253}
    },
    /* Vector 1267: Angle 0.746 rad */
    {
        .input_axis = {29031, 50923, 29309},
        .input_angle_rad = 48858,
        .expected_quaternion = {61035, 10573, 18545, 10674},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51563, 25865, -31100}
    },
    /* Vector 1268: Angle 0.749 rad */
    {
        .input_axis = {31540, 49408, 29309},
        .input_angle_rad = 49059,
        .expected_quaternion = {60999, 11532, 18064, 10716},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52073, 26305, -29856}
    },
    /* Vector 1269: Angle 0.752 rad */
    {
        .input_axis = {33970, 47770, 29309},
        .input_angle_rad = 49260,
        .expected_quaternion = {60962, 12468, 17534, 10757},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52622, 26685, -28526}
    },
    /* Vector 1270: Angle 0.755 rad */
    {
        .input_axis = {36315, 46013, 29309},
        .input_angle_rad = 49461,
        .expected_quaternion = {60925, 13381, 16954, 10799},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53205, 27002, -27113}
    },
    /* Vector 1271: Angle 0.758 rad */
    {
        .input_axis = {38570, 44140, 29309},
        .input_angle_rad = 49662,
        .expected_quaternion = {60888, 14267, 16327, 10841},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53814, 27253, -25618}
    },
    /* Vector 1272: Angle 0.761 rad */
    {
        .input_axis = {40727, 42157, 29309},
        .input_angle_rad = 49863,
        .expected_quaternion = {60851, 15123, 15654, 10883},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54444, 27434, -24047}
    },
    /* Vector 1273: Angle 0.764 rad */
    {
        .input_axis = {42784, 40069, 29309},
        .input_angle_rad = 50064,
        .expected_quaternion = {60813, 15947, 14935, 10925},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55086, 27543, -22402}
    },
    /* Vector 1274: Angle 0.767 rad */
    {
        .input_axis = {44733, 37881, 29309},
        .input_angle_rad = 50265,
        .expected_quaternion = {60776, 16737, 14174, 10966},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55735, 27579, -20687}
    },
    /* Vector 1275: Angle 0.770 rad */
    {
        .input_axis = {46570, 35598, 29309},
        .input_angle_rad = 50467,
        .expected_quaternion = {60738, 17491, 13370, 11008},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56383, 27541, -18907}
    },
    /* Vector 1276: Angle 0.773 rad */
    {
        .input_axis = {48291, 33226, 29309},
        .input_angle_rad = 50668,
        .expected_quaternion = {60700, 18206, 12526, 11050},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57022, 27428, -17065}
    },
    /* Vector 1277: Angle 0.776 rad */
    {
        .input_axis = {49891, 30771, 29309},
        .input_angle_rad = 50869,
        .expected_quaternion = {60662, 18880, 11645, 11091},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57644, 27242, -15167}
    },
    /* Vector 1278: Angle 0.779 rad */
    {
        .input_axis = {51367, 28239, 29309},
        .input_angle_rad = 51070,
        .expected_quaternion = {60624, 19512, 10726, 11133},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58242, 26984, -13216}
    },
    /* Vector 1279: Angle 0.782 rad */
    {
        .input_axis = {52714, 25636, 29309},
        .input_angle_rad = 51271,
        .expected_quaternion = {60586, 20098, 9774, 11174},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58810, 26656, -11218}
    },
    /* Vector 1280: Angle 0.785 rad */
    {
        .input_axis = {53929, 22970, 29309},
        .input_angle_rad = 51472,
        .expected_quaternion = {60547, 20638, 8790, 11216},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59339, 26260, -9178}
    },
    /* Vector 1281: Angle 0.788 rad */
    {
        .input_axis = {55010, 20246, 29309},
        .input_angle_rad = 51673,
        .expected_quaternion = {60509, 21129, 7776, 11257},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59823, 25802, -7101}
    },
    /* Vector 1282: Angle 0.792 rad */
    {
        .input_axis = {55953, 17471, 29309},
        .input_angle_rad = 51874,
        .expected_quaternion = {60470, 21571, 6735, 11299},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60256, 25285, -4991}
    },
    /* Vector 1283: Angle 0.795 rad */
    {
        .input_axis = {56756, 14653, 29309},
        .input_angle_rad = 52075,
        .expected_quaternion = {60431, 21961, 5670, 11340},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60630, 24714, -2856}
    },
    /* Vector 1284: Angle 0.798 rad */
    {
        .input_axis = {57418, 11798, 29309},
        .input_angle_rad = 52276,
        .expected_quaternion = {60392, 22298, 4582, 11382},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60942, 24095, -699}
    },
    /* Vector 1285: Angle 0.801 rad */
    {
        .input_axis = {57936, 8913, 29309},
        .input_angle_rad = 52477,
        .expected_quaternion = {60353, 22581, 3474, 11423},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61185, 23434, 1473}
    },
    /* Vector 1286: Angle 0.804 rad */
    {
        .input_axis = {58309, 6006, 29309},
        .input_angle_rad = 52678,
        .expected_quaternion = {60314, 22809, 2350, 11465},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61356, 22738, 3655}
    },
    /* Vector 1287: Angle 0.807 rad */
    {
        .input_axis = {58536, 3085, 29309},
        .input_angle_rad = 52879,
        .expected_quaternion = {60275, 22980, 1211, 11506},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61451, 22014, 5842}
    },
    /* Vector 1288: Angle 0.810 rad */
    {
        .input_axis = {58617, 155, 29309},
        .input_angle_rad = 53080,
        .expected_quaternion = {60235, 23095, 61, 11547},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61467, 21270, 8026}
    },
    /* Vector 1289: Angle 0.813 rad */
    {
        .input_axis = {58551, -2775, 29309},
        .input_angle_rad = 53281,
        .expected_quaternion = {60195, 23151, -1097, 11589},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61401, 20513, 10203}
    },
    /* Vector 1290: Angle 0.816 rad */
    {
        .input_axis = {58340, -5697, 29309},
        .input_angle_rad = 53482,
        .expected_quaternion = {60156, 23150, -2261, 11630},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61252, 19753, 12367}
    },
    /* Vector 1291: Angle 0.819 rad */
    {
        .input_axis = {57982, -8606, 29309},
        .input_angle_rad = 53684,
        .expected_quaternion = {60116, 23089, -3427, 11671},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {61021, 18997, 14511}
    },
    /* Vector 1292: Angle 0.822 rad */
    {
        .input_axis = {57479, -11493, 29309},
        .input_angle_rad = 53885,
        .expected_quaternion = {60075, 22970, -4593, 11712},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60706, 18253, 16631}
    },
    /* Vector 1293: Angle 0.825 rad */
    {
        .input_axis = {56833, -14352, 29309},
        .input_angle_rad = 54086,
        .expected_quaternion = {60035, 22792, -5755, 11754},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {60309, 17531, 18720}
    },
    /* Vector 1294: Angle 0.828 rad */
    {
        .input_axis = {56045, -17174, 29309},
        .input_angle_rad = 54287,
        .expected_quaternion = {59995, 22554, -6911, 11795},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59833, 16838, 20773}
    },
    /* Vector 1295: Angle 0.831 rad */
    {
        .input_axis = {55116, -19954, 29309},
        .input_angle_rad = 54488,
        .expected_quaternion = {59954, 22258, -8058, 11836},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59279, 16182, 22783}
    },
    /* Vector 1296: Angle 0.834 rad */
    {
        .input_axis = {54050, -22683, 29309},
        .input_angle_rad = 54689,
        .expected_quaternion = {59914, 21903, -9192, 11877},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58652, 15572, 24746}
    },
    /* Vector 1297: Angle 0.838 rad */
    {
        .input_axis = {52849, -25357, 29309},
        .input_angle_rad = 54890,
        .expected_quaternion = {59873, 21491, -10311, 11918},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57957, 15014, 26657}
    },
    /* Vector 1298: Angle 0.841 rad */
    {
        .input_axis = {51516, -27966, 29309},
        .input_angle_rad = 55091,
        .expected_quaternion = {59832, 21021, -11411, 11959},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57197, 14516, 28508}
    },
    /* Vector 1299: Angle 0.844 rad */
    {
        .input_axis = {50054, -30506, 29309},
        .input_angle_rad = 55292,
        .expected_quaternion = {59791, 20494, -12490, 12000},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56380, 14085, 30296}
    },
    /* Vector 1300: Angle 0.847 rad */
    {
        .input_axis = {48466, -32969, 29309},
        .input_angle_rad = 55493,
        .expected_quaternion = {59750, 19912, -13545, 12041},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55512, 13725, 32016}
    },
    /* Vector 1301: Angle 0.850 rad */
    {
        .input_axis = {46758, -35351, 29309},
        .input_angle_rad = 55694,
        .expected_quaternion = {59708, 19276, -14573, 12082},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54600, 13443, 33661}
    },
    /* Vector 1302: Angle 0.853 rad */
    {
        .input_axis = {44933, -37643, 29309},
        .input_angle_rad = 55895,
        .expected_quaternion = {59667, 18586, -15571, 12123},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53652, 13243, 35229}
    },
    /* Vector 1303: Angle 0.856 rad */
    {
        .input_axis = {42995, -39842, 29309},
        .input_angle_rad = 56096,
        .expected_quaternion = {59625, 17845, -16536, 12164},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52676, 13129, 36713}
    },
    /* Vector 1304: Angle 0.859 rad */
    {
        .input_axis = {40950, -41941, 29309},
        .input_angle_rad = 56297,
        .expected_quaternion = {59583, 17053, -17465, 12205},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51681, 13103, 38110}
    },
    /* Vector 1305: Angle 0.862 rad */
    {
        .input_axis = {38803, -43935, 29309},
        .input_angle_rad = 56498,
        .expected_quaternion = {59541, 16213, -18357, 12246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50676, 13169, 39415}
    },
    /* Vector 1306: Angle 0.865 rad */
    {
        .input_axis = {36559, -45820, 29309},
        .input_angle_rad = 56699,
        .expected_quaternion = {59499, 15326, -19208, 12287},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49669, 13326, 40625}
    },
    /* Vector 1307: Angle 0.868 rad */
    {
        .input_axis = {34223, -47590, 29309},
        .input_angle_rad = 56901,
        .expected_quaternion = {59457, 14394, -20017, 12327},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48671, 13575, 41735}
    },
    /* Vector 1308: Angle 0.871 rad */
    {
        .input_axis = {31802, -49241, 29309},
        .input_angle_rad = 57102,
        .expected_quaternion = {59415, 13420, -20780, 12368},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47690, 13916, 42743}
    },
    /* Vector 1309: Angle 0.874 rad */
    {
        .input_axis = {29301, -50768, 29309},
        .input_angle_rad = 57303,
        .expected_quaternion = {59372, 12406, -21495, 12409},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46737, 14346, 43644}
    },
    /* Vector 1310: Angle 0.877 rad */
    {
        .input_axis = {26727, -52169, 29309},
        .input_angle_rad = 57504,
        .expected_quaternion = {59330, 11353, -22161, 12450},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45819, 14863, 44437}
    },
    /* Vector 1311: Angle 0.881 rad */
    {
        .input_axis = {24086, -53440, 29309},
        .input_angle_rad = 57705,
        .expected_quaternion = {59287, 10265, -22774, 12490},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44946, 15465, 45118}
    },
    /* Vector 1312: Angle 0.884 rad */
    {
        .input_axis = {21385, -54577, 29309},
        .input_angle_rad = 57906,
        .expected_quaternion = {59244, 9143, -23335, 12531},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44127, 16145, 45685}
    },
    /* Vector 1313: Angle 0.887 rad */
    {
        .input_axis = {18631, -55578, 29309},
        .input_angle_rad = 58107,
        .expected_quaternion = {59201, 7991, -23840, 12572},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43369, 16899, 46136}
    },
    /* Vector 1314: Angle 0.890 rad */
    {
        .input_axis = {15830, -56439, 29309},
        .input_angle_rad = 58308,
        .expected_quaternion = {59158, 6812, -24287, 12612},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42680, 17721, 46469}
    },
    /* Vector 1315: Angle 0.893 rad */
    {
        .input_axis = {12989, -57160, 29309},
        .input_angle_rad = 58509,
        .expected_quaternion = {59114, 5608, -24677, 12653},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42067, 18603, 46682}
    },
    /* Vector 1316: Angle 0.896 rad */
    {
        .input_axis = {10116, -57738, 29309},
        .input_angle_rad = 58710,
        .expected_quaternion = {59071, 4381, -25006, 12693},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41537, 19539, 46775}
    },
    /* Vector 1317: Angle 0.899 rad */
    {
        .input_axis = {7218, -58171, 29309},
        .input_angle_rad = 58911,
        .expected_quaternion = {59027, 3136, -25274, 12734},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41094, 20520, 46746}
    },
    /* Vector 1318: Angle 0.902 rad */
    {
        .input_axis = {4301, -58459, 29309},
        .input_angle_rad = 59112,
        .expected_quaternion = {58983, 1875, -25480, 12774},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40743, 21536, 46595}
    },
    /* Vector 1319: Angle 0.905 rad */
    {
        .input_axis = {1374, -58601, 29309},
        .input_angle_rad = 59313,
        .expected_quaternion = {58940, 601, -25623, 12815},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40489, 22580, 46322}
    },
    /* Vector 1320: Angle 0.908 rad */
    {
        .input_axis = {-1556, -58597, 29309},
        .input_angle_rad = 59514,
        .expected_quaternion = {58896, -683, -25701, 12855},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40334, 23641, 45927}
    },
    /* Vector 1321: Angle 0.911 rad */
    {
        .input_axis = {-4483, -58445, 29309},
        .input_angle_rad = 59715,
        .expected_quaternion = {58851, -1973, -25716, 12896},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40280, 24709, 45409}
    },
    /* Vector 1322: Angle 0.914 rad */
    {
        .input_axis = {-7398, -58148, 29309},
        .input_angle_rad = 59916,
        .expected_quaternion = {58807, -3265, -25665, 12936},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40327, 25773, 44771}
    },
    /* Vector 1323: Angle 0.917 rad */
    {
        .input_axis = {-10295, -57706, 29309},
        .input_angle_rad = 60118,
        .expected_quaternion = {58763, -4558, -25549, 12976},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40477, 26824, 44012}
    },
    /* Vector 1324: Angle 0.920 rad */
    {
        .input_axis = {-13167, -57119, 29309},
        .input_angle_rad = 60319,
        .expected_quaternion = {58718, -5848, -25368, 13017},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40726, 27852, 43135}
    },
    /* Vector 1325: Angle 0.923 rad */
    {
        .input_axis = {-16005, -56390, 29309},
        .input_angle_rad = 60520,
        .expected_quaternion = {58673, -7130, -25121, 13057},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41074, 28845, 42141}
    },
    /* Vector 1326: Angle 0.927 rad */
    {
        .input_axis = {-18803, -55519, 29309},
        .input_angle_rad = 60721,
        .expected_quaternion = {58628, -8403, -24810, 13097},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41517, 29795, 41031}
    },
    /* Vector 1327: Angle 0.930 rad */
    {
        .input_axis = {-21555, -54510, 29309},
        .input_angle_rad = 60922,
        .expected_quaternion = {58583, -9662, -24434, 13137},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42050, 30691, 39810}
    },
    /* Vector 1328: Angle 0.933 rad */
    {
        .input_axis = {-24252, -53365, 29309},
        .input_angle_rad = 61123,
        .expected_quaternion = {58538, -10904, -23993, 13177},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42668, 31525, 38478}
    },
    /* Vector 1329: Angle 0.936 rad */
    {
        .input_axis = {-26889, -52086, 29309},
        .input_angle_rad = 61324,
        .expected_quaternion = {58493, -12126, -23490, 13218},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43366, 32287, 37040}
    },
    /* Vector 1330: Angle 0.939 rad */
    {
        .input_axis = {-29458, -50677, 29309},
        .input_angle_rad = 61525,
        .expected_quaternion = {58448, -13326, -22924, 13258},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44135, 32970, 35497}
    },
    /* Vector 1331: Angle 0.942 rad */
    {
        .input_axis = {-31954, -49141, 29309},
        .input_angle_rad = 61726,
        .expected_quaternion = {58402, -14498, -22296, 13298},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44968, 33566, 33855}
    },
    /* Vector 1332: Angle 0.945 rad */
    {
        .input_axis = {-34371, -47483, 29309},
        .input_angle_rad = 61927,
        .expected_quaternion = {58356, -15641, -21609, 13338},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45857, 34068, 32116}
    },
    /* Vector 1333: Angle 0.948 rad */
    {
        .input_axis = {-36701, -45706, 29309},
        .input_angle_rad = 62128,
        .expected_quaternion = {58311, -16752, -20862, 13378},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46792, 34471, 30285}
    },
    /* Vector 1334: Angle 0.951 rad */
    {
        .input_axis = {-38939, -43814, 29309},
        .input_angle_rad = 62329,
        .expected_quaternion = {58265, -17827, -20059, 13418},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47763, 34771, 28367}
    },
    /* Vector 1335: Angle 0.954 rad */
    {
        .input_axis = {-41080, -41814, 29309},
        .input_angle_rad = 62530,
        .expected_quaternion = {58219, -18863, -19200, 13458},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48759, 34963, 26365}
    },
    /* Vector 1336: Angle 0.957 rad */
    {
        .input_axis = {-43119, -39708, 29309},
        .input_angle_rad = 62731,
        .expected_quaternion = {58172, -19858, -18287, 13498},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49770, 35045, 24285}
    },
    /* Vector 1337: Angle 0.960 rad */
    {
        .input_axis = {-45050, -37503, 29309},
        .input_angle_rad = 62932,
        .expected_quaternion = {58126, -20808, -17323, 13538},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50785, 35014, 22132}
    },
    /* Vector 1338: Angle 0.963 rad */
    {
        .input_axis = {-46868, -35205, 29309},
        .input_angle_rad = 63133,
        .expected_quaternion = {58079, -21712, -16309, 13577},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51793, 34872, 19911}
    },
    /* Vector 1339: Angle 0.966 rad */
    {
        .input_axis = {-48569, -32819, 29309},
        .input_angle_rad = 63335,
        .expected_quaternion = {58033, -22566, -15248, 13617},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52782, 34617, 17627}
    },
    /* Vector 1340: Angle 0.969 rad */
    {
        .input_axis = {-50148, -30350, 29309},
        .input_angle_rad = 63536,
        .expected_quaternion = {57986, -23368, -14142, 13657},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53740, 34253, 15287}
    },
    /* Vector 1341: Angle 0.973 rad */
    {
        .input_axis = {-51602, -27806, 29309},
        .input_angle_rad = 63737,
        .expected_quaternion = {57939, -24116, -12995, 13697},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54658, 33782, 12896}
    },
    /* Vector 1342: Angle 0.976 rad */
    {
        .input_axis = {-52928, -25192, 29309},
        .input_angle_rad = 63938,
        .expected_quaternion = {57892, -24807, -11807, 13737},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55523, 33207, 10461}
    },
    /* Vector 1343: Angle 0.979 rad */
    {
        .input_axis = {-54121, -22515, 29309},
        .input_angle_rad = 64139,
        .expected_quaternion = {57845, -25439, -10583, 13776},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56326, 32535, 7987}
    },
    /* Vector 1344: Angle 0.982 rad */
    {
        .input_axis = {-55178, -19782, 29309},
        .input_angle_rad = 64340,
        .expected_quaternion = {57798, -26011, -9325, 13816},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57057, 31772, 5481}
    },
    /* Vector 1345: Angle 0.985 rad */
    {
        .input_axis = {-56098, -17000, 29309},
        .input_angle_rad = 64541,
        .expected_quaternion = {57750, -26520, -8037, 13856},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57706, 30923, 2950}
    },
    /* Vector 1346: Angle 0.988 rad */
    {
        .input_axis = {-56877, -14175, 29309},
        .input_angle_rad = 64742,
        .expected_quaternion = {57703, -26966, -6720, 13895},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58266, 29999, 399}
    },
    /* Vector 1347: Angle 0.991 rad */
    {
        .input_axis = {-57515, -11314, 29309},
        .input_angle_rad = 64943,
        .expected_quaternion = {57655, -27345, -5379, 13935},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58727, 29007, -2164}
    },
    /* Vector 1348: Angle 0.994 rad */
    {
        .input_axis = {-58008, -8426, 29309},
        .input_angle_rad = 65144,
        .expected_quaternion = {57607, -27658, -4017, 13974},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59084, 27958, -4733}
    },
    /* Vector 1349: Angle 0.997 rad */
    {
        .input_axis = {-58357, -5516, 29309},
        .input_angle_rad = 65345,
        .expected_quaternion = {57559, -27903, -2637, 14014},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59330, 26862, -7300}
    },
    /* Vector 1350: Angle 1.000 rad */
    {
        .input_axis = {-58560, -2592, 29309},
        .input_angle_rad = 65546,
        .expected_quaternion = {57511, -28079, -1243, 14053},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59462, 25730, -9861}
    },
    /* Vector 1351: Angle 1.003 rad */
    {
        .input_axis = {-58616, 338, 29309},
        .input_angle_rad = 65747,
        .expected_quaternion = {57463, -28185, 162, 14093},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59474, 24574, -12406}
    },
    /* Vector 1352: Angle 1.006 rad */
    {
        .input_axis = {-58526, 3267, 29309},
        .input_angle_rad = 65948,
        .expected_quaternion = {57414, -28220, 1575, 14132},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59365, 23405, -14931}
    },
    /* Vector 1353: Angle 1.009 rad */
    {
        .input_axis = {-58290, 6188, 29309},
        .input_angle_rad = 66149,
        .expected_quaternion = {57366, -28185, 2992, 14171},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {59134, 22236, -17427}
    },
    /* Vector 1354: Angle 1.012 rad */
    {
        .input_axis = {-57908, 9093, 29309},
        .input_angle_rad = 66350,
        .expected_quaternion = {57317, -28078, 4409, 14211},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58780, 21079, -19889}
    },
    /* Vector 1355: Angle 1.015 rad */
    {
        .input_axis = {-57381, 11976, 29309},
        .input_angle_rad = 66551,
        .expected_quaternion = {57268, -27899, 5823, 14250},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {58304, 19947, -22309}
    },
    /* Vector 1356: Angle 1.019 rad */
    {
        .input_axis = {-56710, 14829, 29309},
        .input_angle_rad = 66753,
        .expected_quaternion = {57219, -27649, 7230, 14289},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57710, 18852, -24682}
    },
    /* Vector 1357: Angle 1.022 rad */
    {
        .input_axis = {-55898, 17645, 29309},
        .input_angle_rad = 66954,
        .expected_quaternion = {57170, -27328, 8626, 14329},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57000, 17805, -27000}
    },
    /* Vector 1358: Angle 1.025 rad */
    {
        .input_axis = {-54947, 20416, 29309},
        .input_angle_rad = 67155,
        .expected_quaternion = {57121, -26936, 10009, 14368},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56179, 16818, -29258}
    },
    /* Vector 1359: Angle 1.028 rad */
    {
        .input_axis = {-53858, 23137, 29309},
        .input_angle_rad = 67356,
        .expected_quaternion = {57072, -26474, 11373, 14407},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55254, 15904, -31449}
    },
    /* Vector 1360: Angle 1.031 rad */
    {
        .input_axis = {-52634, 25800, 29309},
        .input_angle_rad = 67557,
        .expected_quaternion = {57022, -25943, 12717, 14446},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54232, 15071, -33567}
    },
    /* Vector 1361: Angle 1.034 rad */
    {
        .input_axis = {-51279, 28398, 29309},
        .input_angle_rad = 67758,
        .expected_quaternion = {56972, -25344, 14035, 14485},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53121, 14330, -35606}
    },
    /* Vector 1362: Angle 1.037 rad */
    {
        .input_axis = {-49795, 30926, 29309},
        .input_angle_rad = 67959,
        .expected_quaternion = {56923, -24677, 15326, 14524},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51930, 13689, -37561}
    },
    /* Vector 1363: Angle 1.040 rad */
    {
        .input_axis = {-48187, 33376, 29309},
        .input_angle_rad = 68160,
        .expected_quaternion = {56873, -23944, 16584, 14563},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50670, 13158, -39426}
    },
    /* Vector 1364: Angle 1.043 rad */
    {
        .input_axis = {-46459, 35742, 29309},
        .input_angle_rad = 68361,
        .expected_quaternion = {56823, -23147, 17808, 14602},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49351, 12742, -41196}
    },
    /* Vector 1365: Angle 1.046 rad */
    {
        .input_axis = {-44615, 38020, 29309},
        .input_angle_rad = 68562,
        .expected_quaternion = {56773, -22288, 18993, 14641},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47985, 12449, -42865}
    },
    /* Vector 1366: Angle 1.049 rad */
    {
        .input_axis = {-42659, 40202, 29309},
        .input_angle_rad = 68763,
        .expected_quaternion = {56722, -21367, 20137, 14680},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46585, 12281, -44430}
    },
    /* Vector 1367: Angle 1.052 rad */
    {
        .input_axis = {-40596, 42284, 29309},
        .input_angle_rad = 68964,
        .expected_quaternion = {56672, -20388, 21235, 14719},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45163, 12244, -45885}
    },
    /* Vector 1368: Angle 1.055 rad */
    {
        .input_axis = {-38432, 44260, 29309},
        .input_angle_rad = 69165,
        .expected_quaternion = {56621, -19352, 22287, 14758},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43731, 12339, -47226}
    },
    /* Vector 1369: Angle 1.058 rad */
    {
        .input_axis = {-36172, 46125, 29309},
        .input_angle_rad = 69366,
        .expected_quaternion = {56571, -18262, 23287, 14797},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42305, 12567, -48449}
    },
    /* Vector 1370: Angle 1.062 rad */
    {
        .input_axis = {-33822, 47876, 29309},
        .input_angle_rad = 69567,
        .expected_quaternion = {56520, -17120, 24234, 14836},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40897, 12928, -49551}
    },
    /* Vector 1371: Angle 1.065 rad */
    {
        .input_axis = {-31386, 49506, 29309},
        .input_angle_rad = 69768,
        .expected_quaternion = {56469, -15929, 25125, 14874},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39520, 13419, -50528}
    },
    /* Vector 1372: Angle 1.068 rad */
    {
        .input_axis = {-28873, 51013, 29309},
        .input_angle_rad = 69970,
        .expected_quaternion = {56418, -14691, 25957, 14913},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38187, 14039, -51377}
    },
    /* Vector 1373: Angle 1.071 rad */
    {
        .input_axis = {-26287, 52392, 29309},
        .input_angle_rad = 70171,
        .expected_quaternion = {56367, -13410, 26728, 14952},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36912, 14781, -52096}
    },
    /* Vector 1374: Angle 1.074 rad */
    {
        .input_axis = {-23636, 53641, 29309},
        .input_angle_rad = 70372,
        .expected_quaternion = {56315, -12089, 27436, 14990},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35707, 15641, -52681}
    },
    /* Vector 1375: Angle 1.077 rad */
    {
        .input_axis = {-20925, 54755, 29309},
        .input_angle_rad = 70573,
        .expected_quaternion = {56264, -10730, 28078, 15029},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34584, 16611, -53132}
    },
    /* Vector 1376: Angle 1.080 rad */
    {
        .input_axis = {-18163, 55732, 29309},
        .input_angle_rad = 70774,
        .expected_quaternion = {56212, -9337, 28652, 15068},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33554, 17683, -53445}
    },
    /* Vector 1377: Angle 1.083 rad */
    {
        .input_axis = {-15355, 56570, 29309},
        .input_angle_rad = 70975,
        .expected_quaternion = {56160, -7914, 29157, 15106},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32627, 18848, -53621}
    },
    /* Vector 1378: Angle 1.086 rad */
    {
        .input_axis = {-12508, 57267, 29309},
        .input_angle_rad = 71176,
        .expected_quaternion = {56108, -6463, 29592, 15145},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31813, 20095, -53657}
    },
    /* Vector 1379: Angle 1.089 rad */
    {
        .input_axis = {-9630, 57821, 29309},
        .input_angle_rad = 71377,
        .expected_quaternion = {56056, -4989, 29954, 15183},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31120, 21414, -53554}
    },
    /* Vector 1380: Angle 1.092 rad */
    {
        .input_axis = {-6728, 58230, 29309},
        .input_angle_rad = 71578,
        .expected_quaternion = {56004, -3494, 30242, 15222},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30555, 22790, -53310}
    },
    /* Vector 1381: Angle 1.095 rad */
    {
        .input_axis = {-3810, 58493, 29309},
        .input_angle_rad = 71779,
        .expected_quaternion = {55952, -1984, 30455, 15260},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30123, 24213, -52927}
    },
    /* Vector 1382: Angle 1.098 rad */
    {
        .input_axis = {-881, 58611, 29309},
        .input_angle_rad = 71980,
        .expected_quaternion = {55900, -460, 30593, 15298},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29831, 25668, -52405}
    },
    /* Vector 1383: Angle 1.101 rad */
    {
        .input_axis = {2049, 58581, 29309},
        .input_angle_rad = 72181,
        .expected_quaternion = {55847, 1072, 30655, 15337},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29680, 27142, -51743}
    },
    /* Vector 1384: Angle 1.104 rad */
    {
        .input_axis = {4974, 58406, 29309},
        .input_angle_rad = 72382,
        .expected_quaternion = {55794, 2609, 30639, 15375},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29674, 28619, -50945}
    },
    /* Vector 1385: Angle 1.108 rad */
    {
        .input_axis = {7887, 58084, 29309},
        .input_angle_rad = 72583,
        .expected_quaternion = {55742, 4148, 30546, 15413},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29811, 30086, -50011}
    },
    /* Vector 1386: Angle 1.111 rad */
    {
        .input_axis = {10780, 57617, 29309},
        .input_angle_rad = 72784,
        .expected_quaternion = {55689, 5683, 30376, 15451},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30092, 31528, -48943}
    },
    /* Vector 1387: Angle 1.114 rad */
    {
        .input_axis = {13646, 57007, 29309},
        .input_angle_rad = 72985,
        .expected_quaternion = {55636, 7212, 30128, 15490},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30513, 32930, -47744}
    },
    /* Vector 1388: Angle 1.117 rad */
    {
        .input_axis = {16478, 56253, 29309},
        .input_angle_rad = 73187,
        .expected_quaternion = {55582, 8730, 29803, 15528},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31071, 34279, -46416}
    },
    /* Vector 1389: Angle 1.120 rad */
    {
        .input_axis = {19269, 55359, 29309},
        .input_angle_rad = 73388,
        .expected_quaternion = {55529, 10234, 29402, 15566},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31761, 35561, -44963}
    },
    /* Vector 1390: Angle 1.123 rad */
    {
        .input_axis = {22012, 54327, 29309},
        .input_angle_rad = 73589,
        .expected_quaternion = {55476, 11719, 28924, 15604},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32575, 36762, -43387}
    },
    /* Vector 1391: Angle 1.126 rad */
    {
        .input_axis = {24700, 53159, 29309},
        .input_angle_rad = 73790,
        .expected_quaternion = {55422, 13182, 28371, 15642},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33505, 37870, -41693}
    },
    /* Vector 1392: Angle 1.129 rad */
    {
        .input_axis = {27326, 51858, 29309},
        .input_angle_rad = 73991,
        .expected_quaternion = {55368, 14619, 27744, 15680},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34543, 38872, -39884}
    },
    /* Vector 1393: Angle 1.132 rad */
    {
        .input_axis = {29884, 50428, 29309},
        .input_angle_rad = 74192,
        .expected_quaternion = {55314, 16026, 27044, 15718},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35677, 39760, -37965}
    },
    /* Vector 1394: Angle 1.135 rad */
    {
        .input_axis = {32366, 48871, 29309},
        .input_angle_rad = 74393,
        .expected_quaternion = {55260, 17400, 26272, 15756},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36896, 40522, -35940}
    },
    /* Vector 1395: Angle 1.138 rad */
    {
        .input_axis = {34769, 47192, 29309},
        .input_angle_rad = 74594,
        .expected_quaternion = {55206, 18736, 25431, 15794},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38187, 41150, -33815}
    },
    /* Vector 1396: Angle 1.141 rad */
    {
        .input_axis = {37084, 45396, 29309},
        .input_angle_rad = 74795,
        .expected_quaternion = {55152, 20032, 24521, 15832},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39537, 41637, -31594}
    },
    /* Vector 1397: Angle 1.144 rad */
    {
        .input_axis = {39306, 43486, 29309},
        .input_angle_rad = 74996,
        .expected_quaternion = {55098, 21283, 23546, 15869},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40931, 41977, -29284}
    },
    /* Vector 1398: Angle 1.147 rad */
    {
        .input_axis = {41431, 41467, 29309},
        .input_angle_rad = 75197,
        .expected_quaternion = {55043, 22486, 22506, 15907},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42356, 42165, -26889}
    },
    /* Vector 1399: Angle 1.150 rad */
    {
        .input_axis = {43451, 39344, 29309},
        .input_angle_rad = 75398,
        .expected_quaternion = {54989, 23639, 21405, 15945},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43795, 42199, -24417}
    },
    /* Vector 1400: Angle 1.154 rad */
    {
        .input_axis = {45363, 37123, 29309},
        .input_angle_rad = 75599,
        .expected_quaternion = {54934, 24738, 20244, 15983},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45233, 42077, -21873}
    },
    /* Vector 1401: Angle 1.157 rad */
    {
        .input_axis = {47162, 34810, 29309},
        .input_angle_rad = 75800,
        .expected_quaternion = {54879, 25779, 19027, 16020},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46655, 41800, -19263}
    },
    /* Vector 1402: Angle 1.160 rad */
    {
        .input_axis = {48843, 32409, 29309},
        .input_angle_rad = 76001,
        .expected_quaternion = {54824, 26761, 17757, 16058},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48044, 41368, -16595}
    },
    /* Vector 1403: Angle 1.163 rad */
    {
        .input_axis = {50402, 29928, 29309},
        .input_angle_rad = 76202,
        .expected_quaternion = {54769, 27679, 16435, 16096},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49386, 40786, -13874}
    },
    /* Vector 1404: Angle 1.166 rad */
    {
        .input_axis = {51834, 27371, 29309},
        .input_angle_rad = 76404,
        .expected_quaternion = {54714, 28533, 15067, 16133},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50665, 40057, -11109}
    },
    /* Vector 1405: Angle 1.169 rad */
    {
        .input_axis = {53138, 24746, 29309},
        .input_angle_rad = 76605,
        .expected_quaternion = {54658, 29318, 13653, 16171},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51867, 39189, -8306}
    },
    /* Vector 1406: Angle 1.172 rad */
    {
        .input_axis = {54308, 22060, 29309},
        .input_angle_rad = 76806,
        .expected_quaternion = {54603, 30033, 12199, 16208},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52977, 38189, -5473}
    },
    /* Vector 1407: Angle 1.175 rad */
    {
        .input_axis = {55343, 19318, 29309},
        .input_angle_rad = 77007,
        .expected_quaternion = {54547, 30676, 10708, 16246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53983, 37067, -2616}
    },
    /* Vector 1408: Angle 1.178 rad */
    {
        .input_axis = {56239, 16528, 29309},
        .input_angle_rad = 77208,
        .expected_quaternion = {54491, 31245, 9182, 16283},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54872, 35833, 256}
    },
    /* Vector 1409: Angle 1.181 rad */
    {
        .input_axis = {56995, 13696, 29309},
        .input_angle_rad = 77409,
        .expected_quaternion = {54435, 31737, 7627, 16320},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55632, 34499, 3137}
    },
    /* Vector 1410: Angle 1.184 rad */
    {
        .input_axis = {57608, 10831, 29309},
        .input_angle_rad = 77610,
        .expected_quaternion = {54379, 32152, 6045, 16358},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56255, 33077, 6019}
    },
    /* Vector 1411: Angle 1.187 rad */
    {
        .input_axis = {58077, 7938, 29309},
        .input_angle_rad = 77811,
        .expected_quaternion = {54323, 32488, 4440, 16395},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56731, 31582, 8894}
    },
    /* Vector 1412: Angle 1.190 rad */
    {
        .input_axis = {58401, 5025, 29309},
        .input_angle_rad = 78012,
        .expected_quaternion = {54267, 32743, 2817, 16432},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57053, 30029, 11754}
    },
    /* Vector 1413: Angle 1.193 rad */
    {
        .input_axis = {58580, 2100, 29309},
        .input_angle_rad = 78213,
        .expected_quaternion = {54210, 32918, 1180, 16469},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57216, 28432, 14592}
    },
    /* Vector 1414: Angle 1.197 rad */
    {
        .input_axis = {58611, -830, 29309},
        .input_angle_rad = 78414,
        .expected_quaternion = {54154, 33010, -468, 16507},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57214, 26808, 17401}
    },
    /* Vector 1415: Angle 1.200 rad */
    {
        .input_axis = {58497, -3759, 29309},
        .input_angle_rad = 78615,
        .expected_quaternion = {54097, 33019, -2122, 16544},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {57046, 25174, 20173}
    },
    /* Vector 1416: Angle 1.203 rad */
    {
        .input_axis = {58236, -6678, 29309},
        .input_angle_rad = 78816,
        .expected_quaternion = {54040, 32946, -3778, 16581},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56711, 23547, 22901}
    },
    /* Vector 1417: Angle 1.206 rad */
    {
        .input_axis = {57829, -9580, 29309},
        .input_angle_rad = 79017,
        .expected_quaternion = {53983, 32789, -5432, 16618},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {56208, 21942, 25577}
    },
    /* Vector 1418: Angle 1.209 rad */
    {
        .input_axis = {57278, -12458, 29309},
        .input_angle_rad = 79218,
        .expected_quaternion = {53926, 32549, -7079, 16655},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {55541, 20377, 28194}
    },
    /* Vector 1419: Angle 1.212 rad */
    {
        .input_axis = {56584, -15305, 29309},
        .input_angle_rad = 79419,
        .expected_quaternion = {53869, 32226, -8717, 16692},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54715, 18868, 30745}
    },
    /* Vector 1420: Angle 1.215 rad */
    {
        .input_axis = {55748, -18114, 29309},
        .input_angle_rad = 79621,
        .expected_quaternion = {53812, 31820, -10339, 16729},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53733, 17432, 33224}
    },
    /* Vector 1421: Angle 1.218 rad */
    {
        .input_axis = {54773, -20878, 29309},
        .input_angle_rad = 79822,
        .expected_quaternion = {53754, 31332, -11943, 16766},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52605, 16084, 35623}
    },
    /* Vector 1422: Angle 1.221 rad */
    {
        .input_axis = {53661, -23589, 29309},
        .input_angle_rad = 80023,
        .expected_quaternion = {53697, 30764, -13524, 16803},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51339, 14838, 37936}
    },
    /* Vector 1423: Angle 1.224 rad */
    {
        .input_axis = {52415, -26242, 29309},
        .input_angle_rad = 80224,
        .expected_quaternion = {53639, 30115, -15077, 16839},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49945, 13708, 40156}
    },
    /* Vector 1424: Angle 1.227 rad */
    {
        .input_axis = {51038, -28828, 29309},
        .input_angle_rad = 80425,
        .expected_quaternion = {53581, 29388, -16600, 16876},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48435, 12708, 42279}
    },
    /* Vector 1425: Angle 1.230 rad */
    {
        .input_axis = {49534, -31343, 29309},
        .input_angle_rad = 80626,
        .expected_quaternion = {53523, 28584, -18087, 16913},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46823, 11848, 44297}
    },
    /* Vector 1426: Angle 1.233 rad */
    {
        .input_axis = {47905, -33780, 29309},
        .input_angle_rad = 80827,
        .expected_quaternion = {53465, 27704, -19535, 16950},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45122, 11139, 46205}
    },
    /* Vector 1427: Angle 1.236 rad */
    {
        .input_axis = {46157, -36132, 29309},
        .input_angle_rad = 81028,
        .expected_quaternion = {53407, 26751, -20941, 16986},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43348, 10590, 47997}
    },
    /* Vector 1428: Angle 1.239 rad */
    {
        .input_axis = {44293, -38393, 29309},
        .input_angle_rad = 81229,
        .expected_quaternion = {53349, 25726, -22299, 17023},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41517, 10207, 49670}
    },
    /* Vector 1429: Angle 1.243 rad */
    {
        .input_axis = {42319, -40559, 29309},
        .input_angle_rad = 81430,
        .expected_quaternion = {53290, 24632, -23608, 17059},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39646, 9997, 51217}
    },
    /* Vector 1430: Angle 1.246 rad */
    {
        .input_axis = {40239, -42624, 29309},
        .input_angle_rad = 81631,
        .expected_quaternion = {53232, 23472, -24863, 17096},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37752, 9963, 52635}
    },
    /* Vector 1431: Angle 1.249 rad */
    {
        .input_axis = {38059, -44582, 29309},
        .input_angle_rad = 81832,
        .expected_quaternion = {53173, 22247, -26060, 17132},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35853, 10108, 53920}
    },
    /* Vector 1432: Angle 1.252 rad */
    {
        .input_axis = {35783, -46428, 29309},
        .input_angle_rad = 82033,
        .expected_quaternion = {53114, 20962, -27197, 17169},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33967, 10431, 55068}
    },
    /* Vector 1433: Angle 1.255 rad */
    {
        .input_axis = {33418, -48158, 29309},
        .input_angle_rad = 82234,
        .expected_quaternion = {53055, 19618, -28271, 17205},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32111, 10932, 56074}
    },
    /* Vector 1434: Angle 1.258 rad */
    {
        .input_axis = {30969, -49768, 29309},
        .input_angle_rad = 82435,
        .expected_quaternion = {52996, 18219, -29278, 17242},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30305, 11607, 56937}
    },
    /* Vector 1435: Angle 1.261 rad */
    {
        .input_axis = {28443, -51254, 29309},
        .input_angle_rad = 82636,
        .expected_quaternion = {52937, 16768, -30215, 17278},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28564, 12451, 57654}
    },
    /* Vector 1436: Angle 1.264 rad */
    {
        .input_axis = {25846, -52611, 29309},
        .input_angle_rad = 82838,
        .expected_quaternion = {52878, 15269, -31081, 17314},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26907, 13458, 58223}
    },
    /* Vector 1437: Angle 1.267 rad */
    {
        .input_axis = {23184, -53837, 29309},
        .input_angle_rad = 83039,
        .expected_quaternion = {52818, 13725, -31872, 17351},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25349, 14618, 58641}
    },
    /* Vector 1438: Angle 1.270 rad */
    {
        .input_axis = {20464, -54929, 29309},
        .input_angle_rad = 83240,
        .expected_quaternion = {52759, 12140, -32586, 17387},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23906, 15921, 58906}
    },
    /* Vector 1439: Angle 1.273 rad */
    {
        .input_axis = {17694, -55883, 29309},
        .input_angle_rad = 83441,
        .expected_quaternion = {52699, 10518, -33221, 17423},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22593, 17357, 59019}
    },
    /* Vector 1440: Angle 1.276 rad */
    {
        .input_axis = {14878, -56698, 29309},
        .input_angle_rad = 83642,
        .expected_quaternion = {52639, 8863, -33775, 17459},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21421, 18911, 58978}
    },
    /* Vector 1441: Angle 1.279 rad */
    {
        .input_axis = {12026, -57370, 29309},
        .input_angle_rad = 83843,
        .expected_quaternion = {52579, 7179, -34246, 17495},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20404, 20570, 58784}
    },
    /* Vector 1442: Angle 1.282 rad */
    {
        .input_axis = {9144, -57900, 29309},
        .input_angle_rad = 84044,
        .expected_quaternion = {52519, 5469, -34633, 17531},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19552, 22317, 58435}
    },
    /* Vector 1443: Angle 1.285 rad */
    {
        .input_axis = {6239, -58284, 29309},
        .input_angle_rad = 84245,
        .expected_quaternion = {52459, 3739, -34935, 17567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18873, 24137, 57933}
    },
    /* Vector 1444: Angle 1.289 rad */
    {
        .input_axis = {3318, -58523, 29309},
        .input_angle_rad = 84446,
        .expected_quaternion = {52398, 1993, -35150, 17603},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18374, 26011, 57278}
    },
    /* Vector 1445: Angle 1.292 rad */
    {
        .input_axis = {389, -58616, 29309},
        .input_angle_rad = 84647,
        .expected_quaternion = {52338, 234, -35278, 17639},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18062, 27922, 56472}
    },
    /* Vector 1446: Angle 1.295 rad */
    {
        .input_axis = {-2541, -58562, 29309},
        .input_angle_rad = 84848,
        .expected_quaternion = {52277, -1533, -35317, 17675},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17938, 29850, 55517}
    },
    /* Vector 1447: Angle 1.298 rad */
    {
        .input_axis = {-5465, -58362, 29309},
        .input_angle_rad = 85049,
        .expected_quaternion = {52217, -3302, -35267, 17711},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18006, 31777, 54415}
    },
    /* Vector 1448: Angle 1.301 rad */
    {
        .input_axis = {-8375, -58016, 29309},
        .input_angle_rad = 85250,
        .expected_quaternion = {52156, -5071, -35129, 17747},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18264, 33684, 53168}
    },
    /* Vector 1449: Angle 1.304 rad */
    {
        .input_axis = {-11264, -57525, 29309},
        .input_angle_rad = 85451,
        .expected_quaternion = {52095, -6834, -34902, 17782},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18711, 35550, 51779}
    },
    /* Vector 1450: Angle 1.307 rad */
    {
        .input_axis = {-14125, -56890, 29309},
        .input_angle_rad = 85652,
        .expected_quaternion = {52034, -8587, -34586, 17818},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19342, 37358, 50252}
    },
    /* Vector 1451: Angle 1.310 rad */
    {
        .input_axis = {-16951, -56113, 29309},
        .input_angle_rad = 85853,
        .expected_quaternion = {51973, -10326, -34182, 17854},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20151, 39089, 48589}
    },
    /* Vector 1452: Angle 1.313 rad */
    {
        .input_axis = {-19734, -55195, 29309},
        .input_angle_rad = 86055,
        .expected_quaternion = {51911, -12045, -33690, 17889},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21131, 40725, 46797}
    },
    /* Vector 1453: Angle 1.316 rad */
    {
        .input_axis = {-22468, -54140, 29309},
        .input_angle_rad = 86256,
        .expected_quaternion = {51850, -13741, -33112, 17925},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22271, 42249, 44877}
    },
    /* Vector 1454: Angle 1.319 rad */
    {
        .input_axis = {-25146, -52950, 29309},
        .input_angle_rad = 86457,
        .expected_quaternion = {51789, -15410, -32448, 17961},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23560, 43645, 42837}
    },
    /* Vector 1455: Angle 1.322 rad */
    {
        .input_axis = {-27761, -51627, 29309},
        .input_angle_rad = 86658,
        .expected_quaternion = {51727, -17046, -31700, 17996},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24986, 44898, 40679}
    },
    /* Vector 1456: Angle 1.325 rad */
    {
        .input_axis = {-30306, -50175, 29309},
        .input_angle_rad = 86859,
        .expected_quaternion = {51665, -18645, -30869, 18032},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26533, 45995, 38411}
    },
    /* Vector 1457: Angle 1.328 rad */
    {
        .input_axis = {-32776, -48597, 29309},
        .input_angle_rad = 87060,
        .expected_quaternion = {51603, -20205, -29957, 18067},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28187, 46923, 36037}
    },
    /* Vector 1458: Angle 1.331 rad */
    {
        .input_axis = {-35164, -46898, 29309},
        .input_angle_rad = 87261,
        .expected_quaternion = {51541, -21719, -28967, 18102},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29929, 47673, 33564}
    },
    /* Vector 1459: Angle 1.335 rad */
    {
        .input_axis = {-37464, -45082, 29309},
        .input_angle_rad = 87462,
        .expected_quaternion = {51479, -23185, -27899, 18138},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31742, 48235, 30997}
    },
    /* Vector 1460: Angle 1.338 rad */
    {
        .input_axis = {-39670, -43154, 29309},
        .input_angle_rad = 87663,
        .expected_quaternion = {51417, -24598, -26758, 18173},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33608, 48602, 28344}
    },
    /* Vector 1461: Angle 1.341 rad */
    {
        .input_axis = {-41778, -41117, 29309},
        .input_angle_rad = 87864,
        .expected_quaternion = {51354, -25955, -25544, 18208},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35505, 48769, 25611}
    },
    /* Vector 1462: Angle 1.344 rad */
    {
        .input_axis = {-43780, -38978, 29309},
        .input_angle_rad = 88065,
        .expected_quaternion = {51292, -27252, -24262, 18243},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37415, 48734, 22805}
    },
    /* Vector 1463: Angle 1.347 rad */
    {
        .input_axis = {-45674, -36741, 29309},
        .input_angle_rad = 88266,
        .expected_quaternion = {51229, -28485, -22914, 18279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39317, 48495, 19934}
    },
    /* Vector 1464: Angle 1.350 rad */
    {
        .input_axis = {-47453, -34412, 29309},
        .input_angle_rad = 88467,
        .expected_quaternion = {51166, -29651, -21503, 18314},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41190, 48054, 17004}
    },
    /* Vector 1465: Angle 1.353 rad */
    {
        .input_axis = {-49114, -31997, 29309},
        .input_angle_rad = 88668,
        .expected_quaternion = {51104, -30748, -20032, 18349},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43015, 47413, 14024}
    },
    /* Vector 1466: Angle 1.356 rad */
    {
        .input_axis = {-50651, -29503, 29309},
        .input_angle_rad = 88869,
        .expected_quaternion = {51041, -31771, -18506, 18384},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44771, 46578, 11000}
    },
    /* Vector 1467: Angle 1.359 rad */
    {
        .input_axis = {-52063, -26934, 29309},
        .input_angle_rad = 89070,
        .expected_quaternion = {50977, -32719, -16927, 18419},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46439, 45556, 7942}
    },
    /* Vector 1468: Angle 1.362 rad */
    {
        .input_axis = {-53344, -24299, 29309},
        .input_angle_rad = 89271,
        .expected_quaternion = {50914, -33587, -15299, 18454},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48000, 44355, 4857}
    },
    /* Vector 1469: Angle 1.365 rad */
    {
        .input_axis = {-54491, -21602, 29309},
        .input_angle_rad = 89473,
        .expected_quaternion = {50851, -34375, -13627, 18489},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49437, 42987, 1752}
    },
    /* Vector 1470: Angle 1.368 rad */
    {
        .input_axis = {-55503, -18852, 29309},
        .input_angle_rad = 89674,
        .expected_quaternion = {50787, -35079, -11915, 18524},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50732, 41465, -1363}
    },
    /* Vector 1471: Angle 1.371 rad */
    {
        .input_axis = {-56376, -16054, 29309},
        .input_angle_rad = 89875,
        .expected_quaternion = {50724, -35698, -10166, 18558},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51872, 39802, -4482}
    },
    /* Vector 1472: Angle 1.374 rad */
    {
        .input_axis = {-57108, -13217, 29309},
        .input_angle_rad = 90076,
        .expected_quaternion = {50660, -36229, -8384, 18593},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52841, 38015, -7594}
    },
    /* Vector 1473: Angle 1.378 rad */
    {
        .input_axis = {-57697, -10346, 29309},
        .input_angle_rad = 90277,
        .expected_quaternion = {50596, -36671, -6576, 18628},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53627, 36122, -10693}
    },
    /* Vector 1474: Angle 1.381 rad */
    {
        .input_axis = {-58142, -7449, 29309},
        .input_angle_rad = 90478,
        .expected_quaternion = {50532, -37023, -4743, 18663},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54220, 34139, -13771}
    },
    /* Vector 1475: Angle 1.384 rad */
    {
        .input_axis = {-58442, -4534, 29309},
        .input_angle_rad = 90679,
        .expected_quaternion = {50468, -37282, -2892, 18697},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54612, 32088, -16818}
    },
    /* Vector 1476: Angle 1.387 rad */
    {
        .input_axis = {-58595, -1608, 29309},
        .input_angle_rad = 90880,
        .expected_quaternion = {50404, -37450, -1027, 18732},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54796, 29988, -19828}
    },
    /* Vector 1477: Angle 1.390 rad */
    {
        .input_axis = {-58602, 1323, 29309},
        .input_angle_rad = 91081,
        .expected_quaternion = {50340, -37523, 847, 18766},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54766, 27860, -22791}
    },
    /* Vector 1478: Angle 1.393 rad */
    {
        .input_axis = {-58463, 4250, 29309},
        .input_angle_rad = 91282,
        .expected_quaternion = {50275, -37503, 2726, 18801},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54522, 25725, -25701}
    },
    /* Vector 1479: Angle 1.396 rad */
    {
        .input_axis = {-58177, 7167, 29309},
        .input_angle_rad = 91483,
        .expected_quaternion = {50211, -37388, 4606, 18835},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {54062, 23606, -28549}
    },
    /* Vector 1480: Angle 1.399 rad */
    {
        .input_axis = {-57746, 10066, 29309},
        .input_angle_rad = 91684,
        .expected_quaternion = {50146, -37179, 6481, 18870},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {53388, 21524, -31327}
    },
    /* Vector 1481: Angle 1.402 rad */
    {
        .input_axis = {-57171, 12939, 29309},
        .input_angle_rad = 91885,
        .expected_quaternion = {50081, -36876, 8346, 18904},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52504, 19500, -34029}
    },
    /* Vector 1482: Angle 1.405 rad */
    {
        .input_axis = {-56453, 15780, 29309},
        .input_angle_rad = 92086,
        .expected_quaternion = {50016, -36479, 10197, 18939},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51417, 17556, -36647}
    },
    /* Vector 1483: Angle 1.408 rad */
    {
        .input_axis = {-55594, 18582, 29309},
        .input_angle_rad = 92287,
        .expected_quaternion = {49951, -35988, 12029, 18973},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50135, 15711, -39174}
    },
    /* Vector 1484: Angle 1.411 rad */
    {
        .input_axis = {-54596, 21337, 29309},
        .input_angle_rad = 92488,
        .expected_quaternion = {49886, -35406, 13838, 19007},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48667, 13985, -41604}
    },
    /* Vector 1485: Angle 1.414 rad */
    {
        .input_axis = {-53461, 24039, 29309},
        .input_angle_rad = 92690,
        .expected_quaternion = {49821, -34733, 15618, 19041},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47027, 12396, -43929}
    },
    /* Vector 1486: Angle 1.417 rad */
    {
        .input_axis = {-52193, 26681, 29309},
        .input_angle_rad = 92891,
        .expected_quaternion = {49756, -33969, 17365, 19075},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45229, 10962, -46143}
    },
    /* Vector 1487: Angle 1.420 rad */
    {
        .input_axis = {-50794, 29256, 29309},
        .input_angle_rad = 93092,
        .expected_quaternion = {49690, -33118, 19076, 19110},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43287, 9699, -48240}
    },
    /* Vector 1488: Angle 1.424 rad */
    {
        .input_axis = {-49268, 31759, 29309},
        .input_angle_rad = 93293,
        .expected_quaternion = {49624, -32181, 20744, 19144},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41220, 8619, -50215}
    },
    /* Vector 1489: Angle 1.427 rad */
    {
        .input_axis = {-47619, 34181, 29309},
        .input_angle_rad = 93494,
        .expected_quaternion = {49559, -31159, 22366, 19178},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39046, 7737, -52062}
    },
    /* Vector 1490: Angle 1.430 rad */
    {
        .input_axis = {-45852, 36519, 29309},
        .input_angle_rad = 93695,
        .expected_quaternion = {49493, -30055, 23938, 19212},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36786, 7061, -53777}
    },
    /* Vector 1491: Angle 1.433 rad */
    {
        .input_axis = {-43969, 38765, 29309},
        .input_angle_rad = 93896,
        .expected_quaternion = {49427, -28872, 25455, 19246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34459, 6601, -55353}
    },
    /* Vector 1492: Angle 1.436 rad */
    {
        .input_axis = {-41977, 40914, 29309},
        .input_angle_rad = 94097,
        .expected_quaternion = {49361, -27613, 26913, 19279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32088, 6363, -56788}
    },
    /* Vector 1493: Angle 1.439 rad */
    {
        .input_axis = {-39879, 42960, 29309},
        .input_angle_rad = 94298,
        .expected_quaternion = {49295, -26279, 28309, 19313},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29696, 6351, -58076}
    },
    /* Vector 1494: Angle 1.442 rad */
    {
        .input_axis = {-37683, 44900, 29309},
        .input_angle_rad = 94499,
        .expected_quaternion = {49228, -24875, 29639, 19347},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27304, 6566, -59214}
    },
    /* Vector 1495: Angle 1.445 rad */
    {
        .input_axis = {-35391, 46727, 29309},
        .input_angle_rad = 94700,
        .expected_quaternion = {49162, -23403, 30899, 19381},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24936, 7009, -60200}
    },
    /* Vector 1496: Angle 1.448 rad */
    {
        .input_axis = {-33012, 48438, 29309},
        .input_angle_rad = 94901,
        .expected_quaternion = {49095, -21868, 32086, 19414},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22615, 7676, -61029}
    },
    /* Vector 1497: Angle 1.451 rad */
    {
        .input_axis = {-30550, 50027, 29309},
        .input_angle_rad = 95102,
        .expected_quaternion = {49029, -20272, 33196, 19448},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20364, 8563, -61701}
    },
    /* Vector 1498: Angle 1.454 rad */
    {
        .input_axis = {-28011, 51491, 29309},
        .input_angle_rad = 95303,
        .expected_quaternion = {48962, -18619, 34227, 19482},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18203, 9661, -62212}
    },
    /* Vector 1499: Angle 1.457 rad */
    {
        .input_axis = {-25403, 52827, 29309},
        .input_angle_rad = 95504,
        .expected_quaternion = {48895, -16915, 35175, 19515},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16154, 10963, -62560}
    },
    /* Vector 1500: Angle 1.460 rad */
    {
        .input_axis = {-22731, 54030, 29309},
        .input_angle_rad = 95705,
        .expected_quaternion = {48828, -15161, 36038, 19549},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {14238, 12456, -62746}
    },
    /* Vector 1501: Angle 1.463 rad */
    {
        .input_axis = {-20002, 55099, 29309},
        .input_angle_rad = 95907,
        .expected_quaternion = {48761, -13364, 36814, 19582},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12474, 14126, -62768}
    },
    /* Vector 1502: Angle 1.466 rad */
    {
        .input_axis = {-17223, 56030, 29309},
        .input_angle_rad = 96108,
        .expected_quaternion = {48694, -11527, 37500, 19616},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10879, 15958, -62626}
    },
    /* Vector 1503: Angle 1.470 rad */
    {
        .input_axis = {-14401, 56821, 29309},
        .input_angle_rad = 96309,
        .expected_quaternion = {48626, -9655, 38094, 19649},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {9468, 17934, -62319}
    },
    /* Vector 1504: Angle 1.473 rad */
    {
        .input_axis = {-11543, 57469, 29309},
        .input_angle_rad = 96510,
        .expected_quaternion = {48559, -7752, 38594, 19682},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8258, 20037, -61849}
    },
    /* Vector 1505: Angle 1.476 rad */
    {
        .input_axis = {-8657, 57974, 29309},
        .input_angle_rad = 96711,
        .expected_quaternion = {48491, -5823, 38999, 19716},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {7258, 22245, -61216}
    },
    /* Vector 1506: Angle 1.479 rad */
    {
        .input_axis = {-5748, 58335, 29309},
        .input_angle_rad = 96912,
        .expected_quaternion = {48424, -3873, 39308, 19749},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {6481, 24538, -60422}
    },
    /* Vector 1507: Angle 1.482 rad */
    {
        .input_axis = {-2826, 58549, 29309},
        .input_angle_rad = 97113,
        .expected_quaternion = {48356, -1907, 39518, 19782},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5934, 26893, -59469}
    },
    /* Vector 1508: Angle 1.485 rad */
    {
        .input_axis = {104, 58617, 29309},
        .input_angle_rad = 97314,
        .expected_quaternion = {48288, 70, 39631, 19815},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5623, 29286, -58358}
    },
    /* Vector 1509: Angle 1.488 rad */
    {
        .input_axis = {3034, 58539, 29309},
        .input_angle_rad = 97515,
        .expected_quaternion = {48220, 2054, 39644, 19848},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5551, 31694, -57093}
    },
    /* Vector 1510: Angle 1.491 rad */
    {
        .input_axis = {5956, 58314, 29309},
        .input_angle_rad = 97716,
        .expected_quaternion = {48152, 4040, 39557, 19881},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5720, 34092, -55677}
    },
    /* Vector 1511: Angle 1.494 rad */
    {
        .input_axis = {8863, 57943, 29309},
        .input_angle_rad = 97917,
        .expected_quaternion = {48084, 6022, 39371, 19914},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {6128, 36458, -54113}
    },
    /* Vector 1512: Angle 1.497 rad */
    {
        .input_axis = {11747, 57428, 29309},
        .input_angle_rad = 98118,
        .expected_quaternion = {48015, 7995, 39086, 19947},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {6772, 38766, -52405}
    },
    /* Vector 1513: Angle 1.500 rad */
    {
        .input_axis = {14603, 56769, 29309},
        .input_angle_rad = 98319,
        .expected_quaternion = {47947, 9955, 38701, 19980},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {7645, 40993, -50558}
    },
    /* Vector 1514: Angle 1.503 rad */
    {
        .input_axis = {17422, 55968, 29309},
        .input_angle_rad = 98520,
        .expected_quaternion = {47878, 11897, 38218, 20013},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8739, 43117, -48575}
    },
    /* Vector 1515: Angle 1.506 rad */
    {
        .input_axis = {20197, 55028, 29309},
        .input_angle_rad = 98721,
        .expected_quaternion = {47809, 13814, 37637, 20046},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10043, 45115, -46462}
    },
    /* Vector 1516: Angle 1.509 rad */
    {
        .input_axis = {22922, 53949, 29309},
        .input_angle_rad = 98922,
        .expected_quaternion = {47741, 15704, 36960, 20079},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {11545, 46966, -44225}
    },
    /* Vector 1517: Angle 1.513 rad */
    {
        .input_axis = {25590, 52736, 29309},
        .input_angle_rad = 99124,
        .expected_quaternion = {47672, 17560, 36188, 20112},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13228, 48651, -41869}
    },
    /* Vector 1518: Angle 1.516 rad */
    {
        .input_axis = {28194, 51391, 29309},
        .input_angle_rad = 99325,
        .expected_quaternion = {47603, 19378, 35322, 20144},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15077, 50152, -39400}
    },
    /* Vector 1519: Angle 1.519 rad */
    {
        .input_axis = {30727, 49918, 29309},
        .input_angle_rad = 99526,
        .expected_quaternion = {47534, 21153, 34365, 20177},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17072, 51453, -36825}
    },
    /* Vector 1520: Angle 1.522 rad */
    {
        .input_axis = {33184, 48320, 29309},
        .input_angle_rad = 99727,
        .expected_quaternion = {47464, 22881, 33319, 20209},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19194, 52539, -34150}
    },
    /* Vector 1521: Angle 1.525 rad */
    {
        .input_axis = {35557, 46601, 29309},
        .input_angle_rad = 99928,
        .expected_quaternion = {47395, 24558, 32185, 20242},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21419, 53398, -31382}
    },
    /* Vector 1522: Angle 1.528 rad */
    {
        .input_axis = {37842, 44766, 29309},
        .input_angle_rad = 100129,
        .expected_quaternion = {47325, 26177, 30967, 20274},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23726, 54020, -28528}
    },
    /* Vector 1523: Angle 1.531 rad */
    {
        .input_axis = {40032, 42818, 29309},
        .input_angle_rad = 100330,
        .expected_quaternion = {47256, 27737, 29667, 20307},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26091, 54398, -25595}
    },
    /* Vector 1524: Angle 1.534 rad */
    {
        .input_axis = {42122, 40764, 29309},
        .input_angle_rad = 100531,
        .expected_quaternion = {47186, 29231, 28289, 20339},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28489, 54525, -22592}
    },
    /* Vector 1525: Angle 1.537 rad */
    {
        .input_axis = {44107, 38608, 29309},
        .input_angle_rad = 100732,
        .expected_quaternion = {47116, 30657, 26836, 20372},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30894, 54399, -19527}
    },
    /* Vector 1526: Angle 1.540 rad */
    {
        .input_axis = {45981, 36355, 29309},
        .input_angle_rad = 100933,
        .expected_quaternion = {47046, 32011, 25310, 20404},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33282, 54020, -16406}
    },
    /* Vector 1527: Angle 1.543 rad */
    {
        .input_axis = {47741, 34012, 29309},
        .input_angle_rad = 101134,
        .expected_quaternion = {46976, 33288, 23716, 20436},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35627, 53390, -13238}
    },
    /* Vector 1528: Angle 1.546 rad */
    {
        .input_axis = {49381, 31583, 29309},
        .input_angle_rad = 101335,
        .expected_quaternion = {46906, 34486, 22057, 20468},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37903, 52514, -10032}
    },
    /* Vector 1529: Angle 1.549 rad */
    {
        .input_axis = {50898, 29076, 29309},
        .input_angle_rad = 101536,
        .expected_quaternion = {46836, 35602, 20338, 20501},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40087, 51398, -6796}
    },
    /* Vector 1530: Angle 1.552 rad */
    {
        .input_axis = {52287, 26496, 29309},
        .input_angle_rad = 101737,
        .expected_quaternion = {46765, 36631, 18562, 20533},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42155, 50054, -3538}
    },
    /* Vector 1531: Angle 1.555 rad */
    {
        .input_axis = {53546, 23849, 29309},
        .input_angle_rad = 101938,
        .expected_quaternion = {46695, 37571, 16734, 20565},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44084, 48492, -267}
    },
    /* Vector 1532: Angle 1.559 rad */
    {
        .input_axis = {54671, 21143, 29309},
        .input_angle_rad = 102139,
        .expected_quaternion = {46624, 38420, 14859, 20597},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45852, 46728, 3008}
    },
    /* Vector 1533: Angle 1.562 rad */
    {
        .input_axis = {55660, 18384, 29309},
        .input_angle_rad = 102341,
        .expected_quaternion = {46554, 39176, 12940, 20629},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47440, 44778, 6279}
    },
    /* Vector 1534: Angle 1.565 rad */
    {
        .input_axis = {56509, 15580, 29309},
        .input_angle_rad = 102542,
        .expected_quaternion = {46483, 39835, 10983, 20661},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48828, 42659, 9537}
    },
    /* Vector 1535: Angle 1.568 rad */
    {
        .input_axis = {57217, 12736, 29309},
        .input_angle_rad = 102743,
        .expected_quaternion = {46412, 40396, 8992, 20692},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50002, 40394, 12774}
    },
    /* Vector 1536: Angle 1.571 rad */
    {
        .input_axis = {57782, 9860, 29309},
        .input_angle_rad = 102944,
        .expected_quaternion = {46341, 40858, 6972, 20724},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50945, 38002, 15980}
    },
    /* Vector 1537: Angle 1.574 rad */
    {
        .input_axis = {58202, 6960, 29309},
        .input_angle_rad = 103145,
        .expected_quaternion = {46270, 41218, 4929, 20756},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51647, 35509, 19149}
    },
    /* Vector 1538: Angle 1.577 rad */
    {
        .input_axis = {58478, 4043, 29309},
        .input_angle_rad = 103346,
        .expected_quaternion = {46199, 41477, 2867, 20788},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52097, 32937, 22270}
    },
    /* Vector 1539: Angle 1.580 rad */
    {
        .input_axis = {58607, 1115, 29309},
        .input_angle_rad = 103547,
        .expected_quaternion = {46127, 41631, 792, 20819},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52289, 30313, 25336}
    },
    /* Vector 1540: Angle 1.583 rad */
    {
        .input_axis = {58589, -1816, 29309},
        .input_angle_rad = 103748,
        .expected_quaternion = {46056, 41682, -1292, 20851},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {52217, 27663, 28339}
    },
    /* Vector 1541: Angle 1.586 rad */
    {
        .input_axis = {58425, -4742, 29309},
        .input_angle_rad = 103949,
        .expected_quaternion = {45984, 41628, -3378, 20883},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51879, 25013, 31270}
    },
    /* Vector 1542: Angle 1.589 rad */
    {
        .input_axis = {58115, -7656, 29309},
        .input_angle_rad = 104150,
        .expected_quaternion = {45912, 41470, -5463, 20914},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {51277, 22390, 34123}
    },
    /* Vector 1543: Angle 1.592 rad */
    {
        .input_axis = {57660, -10551, 29309},
        .input_angle_rad = 104351,
        .expected_quaternion = {45841, 41207, -7540, 20946},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {50412, 19820, 36888}
    },
    /* Vector 1544: Angle 1.595 rad */
    {
        .input_axis = {57060, -13419, 29309},
        .input_angle_rad = 104552,
        .expected_quaternion = {45769, 40840, -9605, 20977},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49292, 17329, 39560}
    },
    /* Vector 1545: Angle 1.598 rad */
    {
        .input_axis = {56318, -16254, 29309},
        .input_angle_rad = 104753,
        .expected_quaternion = {45697, 40369, -11651, 21008},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47924, 14944, 42130}
    },
    /* Vector 1546: Angle 1.601 rad */
    {
        .input_axis = {55436, -19049, 29309},
        .input_angle_rad = 104954,
        .expected_quaternion = {45625, 39796, -13675, 21040},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46320, 12687, 44592}
    },
    /* Vector 1547: Angle 1.605 rad */
    {
        .input_axis = {54414, -21796, 29309},
        .input_angle_rad = 105155,
        .expected_quaternion = {45552, 39121, -15670, 21071},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44493, 10584, 46939}
    },
    /* Vector 1548: Angle 1.608 rad */
    {
        .input_axis = {53257, -24488, 29309},
        .input_angle_rad = 105356,
        .expected_quaternion = {45480, 38345, -17631, 21102},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42459, 8656, 49165}
    },
    /* Vector 1549: Angle 1.611 rad */
    {
        .input_axis = {51967, -27119, 29309},
        .input_angle_rad = 105558,
        .expected_quaternion = {45408, 37471, -19555, 21133},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40237, 6924, 51264}
    },
    /* Vector 1550: Angle 1.614 rad */
    {
        .input_axis = {50546, -29682, 29309},
        .input_angle_rad = 105759,
        .expected_quaternion = {45335, 36501, -21435, 21165},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37845, 5405, 53231}
    },
    /* Vector 1551: Angle 1.617 rad */
    {
        .input_axis = {49000, -32172, 29309},
        .input_angle_rad = 105960,
        .expected_quaternion = {45262, 35436, -23266, 21196},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35306, 4117, 55059}
    },
    /* Vector 1552: Angle 1.620 rad */
    {
        .input_axis = {47330, -34580, 29309},
        .input_angle_rad = 106161,
        .expected_quaternion = {45190, 34279, -25045, 21227},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32644, 3074, 56744}
    },
    /* Vector 1553: Angle 1.623 rad */
    {
        .input_axis = {45543, -36903, 29309},
        .input_angle_rad = 106362,
        .expected_quaternion = {45117, 33033, -26766, 21258},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29883, 2287, 58282}
    },
    /* Vector 1554: Angle 1.626 rad */
    {
        .input_axis = {43642, -39133, 29309},
        .input_angle_rad = 106563,
        .expected_quaternion = {45044, 31700, -28424, 21289},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27049, 1766, 59668}
    },
    /* Vector 1555: Angle 1.629 rad */
    {
        .input_axis = {41631, -41265, 29309},
        .input_angle_rad = 106764,
        .expected_quaternion = {44971, 30283, -30017, 21319},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24169, 1518, 60898}
    },
    /* Vector 1556: Angle 1.632 rad */
    {
        .input_axis = {39517, -43294, 29309},
        .input_angle_rad = 106965,
        .expected_quaternion = {44898, 28787, -31538, 21350},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21270, 1547, 61969}
    },
    /* Vector 1557: Angle 1.635 rad */
    {
        .input_axis = {37304, -45215, 29309},
        .input_angle_rad = 107166,
        .expected_quaternion = {44824, 27214, -32985, 21381},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18381, 1854, 62878}
    },
    /* Vector 1558: Angle 1.638 rad */
    {
        .input_axis = {34997, -47023, 29309},
        .input_angle_rad = 107367,
        .expected_quaternion = {44751, 25568, -34353, 21412},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15530, 2437, 63623}
    },
    /* Vector 1559: Angle 1.641 rad */
    {
        .input_axis = {32603, -48713, 29309},
        .input_angle_rad = 107568,
        .expected_quaternion = {44677, 23853, -35639, 21442},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12743, 3293, 64201}
    },
    /* Vector 1560: Angle 1.644 rad */
    {
        .input_axis = {30128, -50282, 29309},
        .input_angle_rad = 107769,
        .expected_quaternion = {44604, 22073, -36839, 21473},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10048, 4413, 64611}
    },
    /* Vector 1561: Angle 1.647 rad */
    {
        .input_axis = {27577, -51725, 29309},
        .input_angle_rad = 107970,
        .expected_quaternion = {44530, 20233, -37950, 21504},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {7472, 5789, 64851}
    },
    /* Vector 1562: Angle 1.651 rad */
    {
        .input_axis = {24958, -53039, 29309},
        .input_angle_rad = 108171,
        .expected_quaternion = {44456, 18337, -38970, 21534},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5040, 7408, 64921}
    },
    /* Vector 1563: Angle 1.654 rad */
    {
        .input_axis = {22276, -54220, 29309},
        .input_angle_rad = 108372,
        .expected_quaternion = {44382, 16390, -39894, 21565},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2775, 9254, 64820}
    },
    /* Vector 1564: Angle 1.657 rad */
    {
        .input_axis = {19538, -55265, 29309},
        .input_angle_rad = 108573,
        .expected_quaternion = {44308, 14396, -40720, 21595},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {702, 11311, 64549}
    },
    /* Vector 1565: Angle 1.660 rad */
    {
        .input_axis = {16751, -56173, 29309},
        .input_angle_rad = 108775,
        .expected_quaternion = {44234, 12360, -41447, 21625},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1161, 13559, 64108}
    },
    /* Vector 1566: Angle 1.663 rad */
    {
        .input_axis = {13923, -56940, 29309},
        .input_angle_rad = 108976,
        .expected_quaternion = {44160, 10288, -42072, 21656},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-2794, 15976, 63497}
    },
    /* Vector 1567: Angle 1.666 rad */
    {
        .input_axis = {11060, -57564, 29309},
        .input_angle_rad = 109177,
        .expected_quaternion = {44086, 8183, -42593, 21686},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4180, 18539, 62720}
    },
    /* Vector 1568: Angle 1.669 rad */
    {
        .input_axis = {8169, -58045, 29309},
        .input_angle_rad = 109378,
        .expected_quaternion = {44011, 6053, -43009, 21716},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-5306, 21223, 61777}
    },
    /* Vector 1569: Angle 1.672 rad */
    {
        .input_axis = {5258, -58381, 29309},
        .input_angle_rad = 109579,
        .expected_quaternion = {43937, 3901, -43317, 21746},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6159, 24001, 60671}
    },
    /* Vector 1570: Angle 1.675 rad */
    {
        .input_axis = {2333, -58571, 29309},
        .input_angle_rad = 109780,
        .expected_quaternion = {43862, 1734, -43519, 21777},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6732, 26847, 59405}
    },
    /* Vector 1571: Angle 1.678 rad */
    {
        .input_axis = {-597, -58614, 29309},
        .input_angle_rad = 109981,
        .expected_quaternion = {43787, -444, -43611, 21807},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-7018, 29731, 57981}
    },
    /* Vector 1572: Angle 1.681 rad */
    {
        .input_axis = {-3526, -58511, 29309},
        .input_angle_rad = 110182,
        .expected_quaternion = {43713, -2627, -43594, 21837},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-7013, 32625, 56404}
    },
    /* Vector 1573: Angle 1.684 rad */
    {
        .input_axis = {-6446, -58262, 29309},
        .input_angle_rad = 110383,
        .expected_quaternion = {43638, -4809, -43468, 21867},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6718, 35499, 54678}
    },
    /* Vector 1574: Angle 1.687 rad */
    {
        .input_axis = {-9349, -57867, 29309},
        .input_angle_rad = 110584,
        .expected_quaternion = {43562, -6985, -43232, 21896},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6134, 38325, 52806}
    },
    /* Vector 1575: Angle 1.690 rad */
    {
        .input_axis = {-12230, -57327, 29309},
        .input_angle_rad = 110785,
        .expected_quaternion = {43487, -9149, -42888, 21926},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-5268, 41074, 50795}
    },
    /* Vector 1576: Angle 1.694 rad */
    {
        .input_axis = {-15080, -56644, 29309},
        .input_angle_rad = 110986,
        .expected_quaternion = {43412, -11297, -42434, 21956},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4128, 43717, 48649}
    },
    /* Vector 1577: Angle 1.697 rad */
    {
        .input_axis = {-17892, -55820, 29309},
        .input_angle_rad = 111187,
        .expected_quaternion = {43337, -13422, -41873, 21986},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-2724, 46228, 46373}
    },
    /* Vector 1578: Angle 1.700 rad */
    {
        .input_axis = {-20659, -54856, 29309},
        .input_angle_rad = 111388,
        .expected_quaternion = {43261, -15519, -41206, 22016},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1072, 48580, 43975}
    },
    /* Vector 1579: Angle 1.703 rad */
    {
        .input_axis = {-23375, -53755, 29309},
        .input_angle_rad = 111589,
        .expected_quaternion = {43186, -17582, -40433, 22045},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {814, 50749, 41459}
    },
    /* Vector 1580: Angle 1.706 rad */
    {
        .input_axis = {-26033, -52519, 29309},
        .input_angle_rad = 111790,
        .expected_quaternion = {43110, -19607, -39557, 22075},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2913, 52712, 38833}
    },
    /* Vector 1581: Angle 1.709 rad */
    {
        .input_axis = {-28625, -51153, 29309},
        .input_angle_rad = 111991,
        .expected_quaternion = {43034, -21589, -38579, 22104},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5204, 54447, 36103}
    },
    /* Vector 1582: Angle 1.712 rad */
    {
        .input_axis = {-31146, -49658, 29309},
        .input_angle_rad = 112193,
        .expected_quaternion = {42958, -23521, -37502, 22134},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {7666, 55937, 33276}
    },
    /* Vector 1583: Angle 1.715 rad */
    {
        .input_axis = {-33589, -48039, 29309},
        .input_angle_rad = 112394,
        .expected_quaternion = {42882, -25400, -36328, 22163},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10271, 57164, 30361}
    },
    /* Vector 1584: Angle 1.718 rad */
    {
        .input_axis = {-35948, -46301, 29309},
        .input_angle_rad = 112595,
        .expected_quaternion = {42806, -27220, -35059, 22193},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12995, 58115, 27364}
    },
    /* Vector 1585: Angle 1.721 rad */
    {
        .input_axis = {-38217, -44446, 29309},
        .input_angle_rad = 112796,
        .expected_quaternion = {42730, -28976, -33699, 22222},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15809, 58778, 24294}
    },
    /* Vector 1586: Angle 1.724 rad */
    {
        .input_axis = {-40390, -42480, 29309},
        .input_angle_rad = 112997,
        .expected_quaternion = {42654, -30665, -32252, 22251},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18683, 59146, 21158}
    },
    /* Vector 1587: Angle 1.727 rad */
    {
        .input_axis = {-42463, -40409, 29309},
        .input_angle_rad = 113198,
        .expected_quaternion = {42578, -32281, -30719, 22281},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21588, 59213, 17966}
    },
    /* Vector 1588: Angle 1.730 rad */
    {
        .input_axis = {-44430, -38236, 29309},
        .input_angle_rad = 113399,
        .expected_quaternion = {42501, -33820, -29105, 22310},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24495, 58976, 14724}
    },
    /* Vector 1589: Angle 1.733 rad */
    {
        .input_axis = {-46285, -35968, 29309},
        .input_angle_rad = 113600,
        .expected_quaternion = {42424, -35278, -27414, 22339},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27372, 58437, 11443}
    },
    /* Vector 1590: Angle 1.736 rad */
    {
        .input_axis = {-48025, -33609, 29309},
        .input_angle_rad = 113801,
        .expected_quaternion = {42348, -36652, -25650, 22368},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30189, 57598, 8130}
    },
    /* Vector 1591: Angle 1.740 rad */
    {
        .input_axis = {-49645, -31167, 29309},
        .input_angle_rad = 114002,
        .expected_quaternion = {42271, -37937, -23817, 22397},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32916, 56467, 4794}
    },
    /* Vector 1592: Angle 1.743 rad */
    {
        .input_axis = {-51140, -28647, 29309},
        .input_angle_rad = 114203,
        .expected_quaternion = {42194, -39131, -21920, 22426},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35525, 55053, 1445}
    },
    /* Vector 1593: Angle 1.746 rad */
    {
        .input_axis = {-52508, -26055, 29309},
        .input_angle_rad = 114404,
        .expected_quaternion = {42117, -40229, -19962, 22455},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37987, 53369, -1910}
    },
    /* Vector 1594: Angle 1.749 rad */
    {
        .input_axis = {-53745, -23398, 29309},
        .input_angle_rad = 114605,
        .expected_quaternion = {42040, -41230, -17950, 22484},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40276, 51431, -5261}
    },
    /* Vector 1595: Angle 1.752 rad */
    {
        .input_axis = {-54847, -20683, 29309},
        .input_angle_rad = 114806,
        .expected_quaternion = {41963, -42129, -15887, 22513},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42367, 49255, -8599}
    },
    /* Vector 1596: Angle 1.755 rad */
    {
        .input_axis = {-55812, -17916, 29309},
        .input_angle_rad = 115007,
        .expected_quaternion = {41886, -42925, -13779, 22541},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44235, 46864, -11915}
    },
    /* Vector 1597: Angle 1.758 rad */
    {
        .input_axis = {-56638, -15104, 29309},
        .input_angle_rad = 115208,
        .expected_quaternion = {41808, -43616, -11631, 22570},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45861, 44279, -15202}
    },
    /* Vector 1598: Angle 1.761 rad */
    {
        .input_axis = {-57322, -12254, 29309},
        .input_angle_rad = 115410,
        .expected_quaternion = {41731, -44199, -9449, 22599},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47226, 41525, -18449}
    },
    /* Vector 1599: Angle 1.764 rad */
    {
        .input_axis = {-57863, -9374, 29309},
        .input_angle_rad = 115611,
        .expected_quaternion = {41653, -44672, -7237, 22627},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48313, 38629, -21648}
    },
    /* Vector 1600: Angle 1.767 rad */
    {
        .input_axis = {-58259, -6471, 29309},
        .input_angle_rad = 115812,
        .expected_quaternion = {41576, -45035, -5002, 22656},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49108, 35620, -24791}
    },
    /* Vector 1601: Angle 1.770 rad */
    {
        .input_axis = {-58510, -3551, 29309},
        .input_angle_rad = 116013,
        .expected_quaternion = {41498, -45285, -2748, 22684},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49602, 32526, -27869}
    },
    /* Vector 1602: Angle 1.773 rad */
    {
        .input_axis = {-58614, -622, 29309},
        .input_angle_rad = 116214,
        .expected_quaternion = {41420, -45423, -482, 22713},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49786, 29378, -30875}
    },
    /* Vector 1603: Angle 1.776 rad */
    {
        .input_axis = {-58572, 2308, 29309},
        .input_angle_rad = 116415,
        .expected_quaternion = {41342, -45447, 1791, 22741},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49656, 26208, -33800}
    },
    /* Vector 1604: Angle 1.779 rad */
    {
        .input_axis = {-58383, 5233, 29309},
        .input_angle_rad = 116616,
        .expected_quaternion = {41264, -45357, 4065, 22770},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {49210, 23046, -36637}
    },
    /* Vector 1605: Angle 1.782 rad */
    {
        .input_axis = {-58049, 8144, 29309},
        .input_angle_rad = 116817,
        .expected_quaternion = {41186, -45153, 6335, 22798},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {48450, 19925, -39377}
    },
    /* Vector 1606: Angle 1.786 rad */
    {
        .input_axis = {-57569, 11035, 29309},
        .input_angle_rad = 117018,
        .expected_quaternion = {41108, -44836, 8594, 22826},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47381, 16876, -42014}
    },
    /* Vector 1607: Angle 1.789 rad */
    {
        .input_axis = {-56946, 13899, 29309},
        .input_angle_rad = 117219,
        .expected_quaternion = {41029, -44405, 10838, 22854},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46012, 13929, -44541}
    },
    /* Vector 1608: Angle 1.792 rad */
    {
        .input_axis = {-56180, 16727, 29309},
        .input_angle_rad = 117420,
        .expected_quaternion = {40951, -43862, 13060, 22882},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44352, 11115, -46950}
    },
    /* Vector 1609: Angle 1.795 rad */
    {
        .input_axis = {-55274, 19514, 29309},
        .input_angle_rad = 117621,
        .expected_quaternion = {40872, -43207, 15254, 22910},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42417, 8463, -49236}
    },
    /* Vector 1610: Angle 1.798 rad */
    {
        .input_axis = {-54229, 22252, 29309},
        .input_angle_rad = 117822,
        .expected_quaternion = {40794, -42443, 17416, 22938},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40222, 5999, -51392}
    },
    /* Vector 1611: Angle 1.801 rad */
    {
        .input_axis = {-53049, 24935, 29309},
        .input_angle_rad = 118023,
        .expected_quaternion = {40715, -41570, 19539, 22966},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37789, 3749, -53413}
    },
    /* Vector 1612: Angle 1.804 rad */
    {
        .input_axis = {-51737, 27555, 29309},
        .input_angle_rad = 118224,
        .expected_quaternion = {40636, -40590, 21619, 22994},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35138, 1736, -55293}
    },
    /* Vector 1613: Angle 1.807 rad */
    {
        .input_axis = {-50295, 30106, 29309},
        .input_angle_rad = 118425,
        .expected_quaternion = {40557, -39507, 23649, 23022},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32294, -18, -57027}
    },
    /* Vector 1614: Angle 1.810 rad */
    {
        .input_axis = {-48727, 32582, 29309},
        .input_angle_rad = 118627,
        .expected_quaternion = {40478, -38322, 25625, 23050},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29284, -1494, -58611}
    },
    /* Vector 1615: Angle 1.813 rad */
    {
        .input_axis = {-47038, 34977, 29309},
        .input_angle_rad = 118828,
        .expected_quaternion = {40399, -37038, 27541, 23078},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26135, -2678, -60040}
    },
    /* Vector 1616: Angle 1.816 rad */
    {
        .input_axis = {-45231, 37284, 29309},
        .input_angle_rad = 119029,
        .expected_quaternion = {40320, -35658, 29393, 23105},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22878, -3555, -61310}
    },
    /* Vector 1617: Angle 1.819 rad */
    {
        .input_axis = {-43311, 39498, 29309},
        .input_angle_rad = 119230,
        .expected_quaternion = {40241, -34185, 31176, 23133},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19544, -4116, -62418}
    },
    /* Vector 1618: Angle 1.822 rad */
    {
        .input_axis = {-41283, 41614, 29309},
        .input_angle_rad = 119431,
        .expected_quaternion = {40161, -32623, 32884, 23161},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16165, -4353, -63362}
    },
    /* Vector 1619: Angle 1.825 rad */
    {
        .input_axis = {-39152, 43625, 29309},
        .input_angle_rad = 119632,
        .expected_quaternion = {40082, -30975, 34515, 23188},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12773, -4263, -64138}
    },
    /* Vector 1620: Angle 1.829 rad */
    {
        .input_axis = {-36922, 45527, 29309},
        .input_angle_rad = 119833,
        .expected_quaternion = {40002, -29246, 36062, 23216},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {9400, -3846, -64744}
    },
    /* Vector 1621: Angle 1.832 rad */
    {
        .input_axis = {-34601, 47316, 29309},
        .input_angle_rad = 120034,
        .expected_quaternion = {39922, -27440, 37523, 23243},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {6081, -3104, -65179}
    },
    /* Vector 1622: Angle 1.835 rad */
    {
        .input_axis = {-32193, 48986, 29309},
        .input_angle_rad = 120235,
        .expected_quaternion = {39843, -25560, 38894, 23270},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2846, -2044, -65442}
    },
    /* Vector 1623: Angle 1.838 rad */
    {
        .input_axis = {-29704, 50534, 29309},
        .input_angle_rad = 120436,
        .expected_quaternion = {39763, -23612, 40169, 23298},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-271, -675, -65532}
    },
    /* Vector 1624: Angle 1.841 rad */
    {
        .input_axis = {-27141, 51955, 29309},
        .input_angle_rad = 120637,
        .expected_quaternion = {39683, -21600, 41348, 23325},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-3241, 991, -65448}
    },
    /* Vector 1625: Angle 1.844 rad */
    {
        .input_axis = {-24511, 53247, 29309},
        .input_angle_rad = 120838,
        .expected_quaternion = {39603, -19529, 42425, 23352},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6034, 2938, -65191}
    },
    /* Vector 1626: Angle 1.847 rad */
    {
        .input_axis = {-21819, 54405, 29309},
        .input_angle_rad = 121039,
        .expected_quaternion = {39523, -17405, 43398, 23379},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-8622, 5147, -64762}
    },
    /* Vector 1627: Angle 1.850 rad */
    {
        .input_axis = {-19073, 55428, 29309},
        .input_angle_rad = 121240,
        .expected_quaternion = {39442, -15232, 44265, 23406},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-10980, 7598, -64161}
    },
    /* Vector 1628: Angle 1.853 rad */
    {
        .input_axis = {-16279, 56311, 29309},
        .input_angle_rad = 121441,
        .expected_quaternion = {39362, -13015, 45023, 23433},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13083, 10266, -63391}
    },
    /* Vector 1629: Angle 1.856 rad */
    {
        .input_axis = {-13444, 57055, 29309},
        .input_angle_rad = 121642,
        .expected_quaternion = {39282, -10761, 45670, 23460},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14912, 13125, -62453}
    },
    /* Vector 1630: Angle 1.859 rad */
    {
        .input_axis = {-10575, 57655, 29309},
        .input_angle_rad = 121844,
        .expected_quaternion = {39201, -8475, 46204, 23487},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16447, 16148, -61349}
    },
    /* Vector 1631: Angle 1.862 rad */
    {
        .input_axis = {-7681, 58112, 29309},
        .input_angle_rad = 122045,
        .expected_quaternion = {39120, -6162, 46623, 23514},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-17673, 19305, -60083}
    },
    /* Vector 1632: Angle 1.865 rad */
    {
        .input_axis = {-4767, 58423, 29309},
        .input_angle_rad = 122246,
        .expected_quaternion = {39040, -3829, 46926, 23541},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18577, 22564, -58658}
    },
    /* Vector 1633: Angle 1.868 rad */
    {
        .input_axis = {-1841, 58588, 29309},
        .input_angle_rad = 122447,
        .expected_quaternion = {38959, -1480, 47112, 23568},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19150, 25892, -57078}
    },
    /* Vector 1634: Angle 1.871 rad */
    {
        .input_axis = {1090, 58607, 29309},
        .input_angle_rad = 122648,
        .expected_quaternion = {38878, 877, 47181, 23594},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19385, 29257, -55346}
    },
    /* Vector 1635: Angle 1.875 rad */
    {
        .input_axis = {4017, 58479, 29309},
        .input_angle_rad = 122849,
        .expected_quaternion = {38797, 3238, 47131, 23621},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19281, 32624, -53469}
    },
    /* Vector 1636: Angle 1.878 rad */
    {
        .input_axis = {6935, 58205, 29309},
        .input_angle_rad = 123050,
        .expected_quaternion = {38716, 5596, 46963, 23648},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18837, 35960, -51449}
    },
    /* Vector 1637: Angle 1.881 rad */
    {
        .input_axis = {9836, 57786, 29309},
        .input_angle_rad = 123251,
        .expected_quaternion = {38635, 7945, 46677, 23674},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18058, 39230, -49294}
    },
    /* Vector 1638: Angle 1.884 rad */
    {
        .input_axis = {12711, 57222, 29309},
        .input_angle_rad = 123452,
        .expected_quaternion = {38554, 10279, 46273, 23701},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16951, 42401, -47009}
    },
    /* Vector 1639: Angle 1.887 rad */
    {
        .input_axis = {15555, 56516, 29309},
        .input_angle_rad = 123653,
        .expected_quaternion = {38472, 12593, 45753, 23727},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-15527, 45440, -44599}
    },
    /* Vector 1640: Angle 1.890 rad */
    {
        .input_axis = {18361, 55667, 29309},
        .input_angle_rad = 123854,
        .expected_quaternion = {38391, 14880, 45116, 23753},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13800, 48317, -42071}
    },
    /* Vector 1641: Angle 1.893 rad */
    {
        .input_axis = {21120, 54680, 29309},
        .input_angle_rad = 124055,
        .expected_quaternion = {38309, 17136, 44365, 23780},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11787, 51001, -39432}
    },
    /* Vector 1642: Angle 1.896 rad */
    {
        .input_axis = {23826, 53556, 29309},
        .input_angle_rad = 124256,
        .expected_quaternion = {38228, 19353, 43501, 23806},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-9509, 53464, -36689}
    },
    /* Vector 1643: Angle 1.899 rad */
    {
        .input_axis = {26473, 52299, 29309},
        .input_angle_rad = 124457,
        .expected_quaternion = {38146, 21527, 42526, 23832},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6988, 55681, -33850}
    },
    /* Vector 1644: Angle 1.902 rad */
    {
        .input_axis = {29054, 50910, 29309},
        .input_angle_rad = 124658,
        .expected_quaternion = {38064, 23651, 41443, 23858},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4249, 57627, -30921}
    },
    /* Vector 1645: Angle 1.905 rad */
    {
        .input_axis = {31562, 49394, 29309},
        .input_angle_rad = 124859,
        .expected_quaternion = {37982, 25721, 40253, 23884},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1320, 59281, -27910}
    },
    /* Vector 1646: Angle 1.908 rad */
    {
        .input_axis = {33991, 47755, 29309},
        .input_angle_rad = 125061,
        .expected_quaternion = {37900, 27731, 38959, 23910},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {1768, 60626, -24827}
    },
    /* Vector 1647: Angle 1.911 rad */
    {
        .input_axis = {36336, 45997, 29309},
        .input_angle_rad = 125262,
        .expected_quaternion = {37818, 29675, 37565, 23936},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {4986, 61646, -21678}
    },
    /* Vector 1648: Angle 1.914 rad */
    {
        .input_axis = {38589, 44123, 29309},
        .input_angle_rad = 125463,
        .expected_quaternion = {37736, 31550, 36074, 23962},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8299, 62329, -18472}
    },
    /* Vector 1649: Angle 1.917 rad */
    {
        .input_axis = {40746, 42139, 29309},
        .input_angle_rad = 125664,
        .expected_quaternion = {37654, 33349, 34490, 23988},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {11673, 62667, -15219}
    },
    /* Vector 1650: Angle 1.921 rad */
    {
        .input_axis = {42801, 40050, 29309},
        .input_angle_rad = 125865,
        .expected_quaternion = {37572, 35069, 32815, 24014},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15075, 62654, -11925}
    },
    /* Vector 1651: Angle 1.924 rad */
    {
        .input_axis = {44750, 37861, 29309},
        .input_angle_rad = 126066,
        .expected_quaternion = {37489, 36705, 31055, 24040},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18469, 62289, -8601}
    },
    /* Vector 1652: Angle 1.927 rad */
    {
        .input_axis = {46586, 35577, 29309},
        .input_angle_rad = 126267,
        .expected_quaternion = {37407, 38252, 29212, 24065},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21819, 61573, -5255}
    },
    /* Vector 1653: Angle 1.930 rad */
    {
        .input_axis = {48306, 33204, 29309},
        .input_angle_rad = 126468,
        .expected_quaternion = {37324, 39706, 27293, 24091},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25091, 60513, -1896}
    },
    /* Vector 1654: Angle 1.933 rad */
    {
        .input_axis = {49905, 30749, 29309},
        .input_angle_rad = 126669,
        .expected_quaternion = {37241, 41064, 25301, 24117},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28250, 59116, 1467}
    },
    /* Vector 1655: Angle 1.936 rad */
    {
        .input_axis = {51379, 28216, 29309},
        .input_angle_rad = 126870,
        .expected_quaternion = {37159, 42322, 23242, 24142},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31264, 57396, 4825}
    },
    /* Vector 1656: Angle 1.939 rad */
    {
        .input_axis = {52725, 25613, 29309},
        .input_angle_rad = 127071,
        .expected_quaternion = {37076, 43477, 21120, 24168},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34099, 55367, 8169}
    },
    /* Vector 1657: Angle 1.942 rad */
    {
        .input_axis = {53940, 22946, 29309},
        .input_angle_rad = 127272,
        .expected_quaternion = {36993, 44525, 18941, 24193},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36726, 53048, 11490}
    },
    /* Vector 1658: Angle 1.945 rad */
    {
        .input_axis = {55019, 20221, 29309},
        .input_angle_rad = 127473,
        .expected_quaternion = {36910, 45463, 16709, 24218},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39116, 50462, 14780}
    },
    /* Vector 1659: Angle 1.948 rad */
    {
        .input_axis = {55961, 17446, 29309},
        .input_angle_rad = 127674,
        .expected_quaternion = {36827, 46290, 14431, 24244},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41244, 47633, 18029}
    },
    /* Vector 1660: Angle 1.951 rad */
    {
        .input_axis = {56763, 14627, 29309},
        .input_angle_rad = 127875,
        .expected_quaternion = {36744, 47002, 12112, 24269},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43085, 44587, 21229}
    },
    /* Vector 1661: Angle 1.954 rad */
    {
        .input_axis = {57423, 11772, 29309},
        .input_angle_rad = 128076,
        .expected_quaternion = {36660, 47598, 9758, 24294},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44619, 41354, 24372}
    },
    /* Vector 1662: Angle 1.957 rad */
    {
        .input_axis = {57940, 8887, 29309},
        .input_angle_rad = 128278,
        .expected_quaternion = {36577, 48076, 7374, 24319},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45828, 37965, 27448}
    },
    /* Vector 1663: Angle 1.960 rad */
    {
        .input_axis = {58311, 5981, 29309},
        .input_angle_rad = 128479,
        .expected_quaternion = {36493, 48434, 4968, 24344},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46697, 34454, 30451}
    },
    /* Vector 1664: Angle 1.963 rad */
    {
        .input_axis = {58537, 3059, 29309},
        .input_angle_rad = 128680,
        .expected_quaternion = {36410, 48672, 2543, 24369},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47216, 30855, 33371}
    },
    /* Vector 1665: Angle 1.967 rad */
    {
        .input_axis = {58617, 129, 29309},
        .input_angle_rad = 128881,
        .expected_quaternion = {36326, 48788, 108, 24394},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47375, 27203, 36201}
    },
    /* Vector 1666: Angle 1.970 rad */
    {
        .input_axis = {58550, -2801, 29309},
        .input_angle_rad = 129082,
        .expected_quaternion = {36243, 48782, -2333, 24419},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {47173, 23535, 38934}
    },
    /* Vector 1667: Angle 1.973 rad */
    {
        .input_axis = {58337, -5723, 29309},
        .input_angle_rad = 129283,
        .expected_quaternion = {36159, 48654, -4773, 24444},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {46606, 19886, 41562}
    },
    /* Vector 1668: Angle 1.976 rad */
    {
        .input_axis = {57978, -8632, 29309},
        .input_angle_rad = 129484,
        .expected_quaternion = {36075, 48404, -7206, 24469},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45680, 16293, 44078}
    },
    /* Vector 1669: Angle 1.979 rad */
    {
        .input_axis = {57474, -11519, 29309},
        .input_angle_rad = 129685,
        .expected_quaternion = {35991, 48032, -9626, 24493},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44400, 12792, 46476}
    },
    /* Vector 1670: Angle 1.982 rad */
    {
        .input_axis = {56827, -14377, 29309},
        .input_angle_rad = 129886,
        .expected_quaternion = {35907, 47538, -12027, 24518},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42777, 9418, 48749}
    },
    /* Vector 1671: Angle 1.985 rad */
    {
        .input_axis = {56037, -17199, 29309},
        .input_angle_rad = 130087,
        .expected_quaternion = {35823, 46925, -14402, 24543},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40824, 6206, 50891}
    },
    /* Vector 1672: Angle 1.988 rad */
    {
        .input_axis = {55108, -19978, 29309},
        .input_angle_rad = 130288,
        .expected_quaternion = {35738, 46193, -16746, 24567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38559, 3187, 52896}
    },
    /* Vector 1673: Angle 1.991 rad */
    {
        .input_axis = {54040, -22707, 29309},
        .input_angle_rad = 130489,
        .expected_quaternion = {35654, 45343, -19053, 24592},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36002, 393, 54760}
    },
    /* Vector 1674: Angle 1.994 rad */
    {
        .input_axis = {52838, -25380, 29309},
        .input_angle_rad = 130690,
        .expected_quaternion = {35570, 44378, -21316, 24616},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33177, -2148, 56477}
    },
    /* Vector 1675: Angle 1.997 rad */
    {
        .input_axis = {51503, -27989, 29309},
        .input_angle_rad = 130891,
        .expected_quaternion = {35485, 43300, -23531, 24640},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30109, -4411, 58043}
    },
    /* Vector 1676: Angle 2.000 rad */
    {
        .input_axis = {50040, -30528, 29309},
        .input_angle_rad = 131092,
        .expected_quaternion = {35401, 42111, -25691, 24665},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26828, -6370, 59453}
    },
    /* Vector 1677: Angle 2.003 rad */
    {
        .input_axis = {48452, -32991, 29309},
        .input_angle_rad = 131293,
        .expected_quaternion = {35316, 40815, -27791, 24689},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23364, -8007, 60704}
    },
    /* Vector 1678: Angle 2.006 rad */
    {
        .input_axis = {46742, -35371, 29309},
        .input_angle_rad = 131495,
        .expected_quaternion = {35231, 39413, -29825, 24713},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19751, -9303, 61793}
    },
    /* Vector 1679: Angle 2.010 rad */
    {
        .input_axis = {44916, -37663, 29309},
        .input_angle_rad = 131696,
        .expected_quaternion = {35146, 37911, -31789, 24737},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16022, -10245, 62716}
    },
    /* Vector 1680: Angle 2.013 rad */
    {
        .input_axis = {42978, -39861, 29309},
        .input_angle_rad = 131897,
        .expected_quaternion = {35062, 36310, -33677, 24761},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12214, -10822, 63472}
    },
    /* Vector 1681: Angle 2.016 rad */
    {
        .input_axis = {40932, -41959, 29309},
        .input_angle_rad = 132098,
        .expected_quaternion = {34977, 34615, -35484, 24785},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8364, -11028, 64058}
    },
    /* Vector 1682: Angle 2.019 rad */
    {
        .input_axis = {38783, -43953, 29309},
        .input_angle_rad = 132299,
        .expected_quaternion = {34892, 32830, -37205, 24809},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {4508, -10858, 64473}
    },
    /* Vector 1683: Angle 2.022 rad */
    {
        .input_axis = {36538, -45836, 29309},
        .input_angle_rad = 132500,
        .expected_quaternion = {34806, 30959, -38837, 24833},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {686, -10315, 64716}
    },
    /* Vector 1684: Angle 2.025 rad */
    {
        .input_axis = {34202, -47605, 29309},
        .input_angle_rad = 132701,
        .expected_quaternion = {34721, 29007, -40375, 24857},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-3067, -9402, 64786}
    },
    /* Vector 1685: Angle 2.028 rad */
    {
        .input_axis = {31780, -49255, 29309},
        .input_angle_rad = 132902,
        .expected_quaternion = {34636, 26979, -41814, 24881},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6713, -8127, 64683}
    },
    /* Vector 1686: Angle 2.031 rad */
    {
        .input_axis = {29278, -50781, 29309},
        .input_angle_rad = 133103,
        .expected_quaternion = {34551, 24879, -43151, 24905},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-10217, -6503, 64407}
    },
    /* Vector 1687: Angle 2.034 rad */
    {
        .input_axis = {26704, -52181, 29309},
        .input_angle_rad = 133304,
        .expected_quaternion = {34465, 22713, -44383, 24928},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13543, -4544, 63960}
    },
    /* Vector 1688: Angle 2.037 rad */
    {
        .input_axis = {24062, -53451, 29309},
        .input_angle_rad = 133505,
        .expected_quaternion = {34380, 20486, -45506, 24952},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16659, -2270, 63343}
    },
    /* Vector 1689: Angle 2.040 rad */
    {
        .input_axis = {21361, -54587, 29309},
        .input_angle_rad = 133706,
        .expected_quaternion = {34294, 18203, -46516, 24976},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19533, 298, 62557}
    },
    /* Vector 1690: Angle 2.043 rad */
    {
        .input_axis = {18606, -55586, 29309},
        .input_angle_rad = 133907,
        .expected_quaternion = {34208, 15870, -47413, 24999},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22138, 3135, 61604}
    },
    /* Vector 1691: Angle 2.046 rad */
    {
        .input_axis = {15805, -56446, 29309},
        .input_angle_rad = 134108,
        .expected_quaternion = {34122, 13493, -48192, 25023},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-24447, 6212, 60487}
    },
    /* Vector 1692: Angle 2.049 rad */
    {
        .input_axis = {12964, -57166, 29309},
        .input_angle_rad = 134309,
        .expected_quaternion = {34037, 11078, -48851, 25046},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26437, 9500, 59210}
    },
    /* Vector 1693: Angle 2.052 rad */
    {
        .input_axis = {10090, -57742, 29309},
        .input_angle_rad = 134510,
        .expected_quaternion = {33951, 8631, -49390, 25069},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-28087, 12965, 57775}
    },
    /* Vector 1694: Angle 2.056 rad */
    {
        .input_axis = {7192, -58174, 29309},
        .input_angle_rad = 134711,
        .expected_quaternion = {33865, 6157, -49806, 25092},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-29381, 16573, 56187}
    },
    /* Vector 1695: Angle 2.059 rad */
    {
        .input_axis = {4275, -58461, 29309},
        .input_angle_rad = 134913,
        .expected_quaternion = {33778, 3664, -50098, 25116},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30306, 20289, 54450}
    },
    /* Vector 1696: Angle 2.062 rad */
    {
        .input_axis = {1348, -58602, 29309},
        .input_angle_rad = 135114,
        .expected_quaternion = {33692, 1156, -50264, 25139},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30853, 24074, 52569}
    },
    /* Vector 1697: Angle 2.065 rad */
    {
        .input_axis = {-1582, -58596, 29309},
        .input_angle_rad = 135315,
        .expected_quaternion = {33606, -1358, -50305, 25162},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-31014, 27891, 50549}
    },
    /* Vector 1698: Angle 2.068 rad */
    {
        .input_axis = {-4509, -58444, 29309},
        .input_angle_rad = 135516,
        .expected_quaternion = {33520, -3875, -50221, 25185},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30789, 31701, 48395}
    },
    /* Vector 1699: Angle 2.071 rad */
    {
        .input_axis = {-7424, -58145, 29309},
        .input_angle_rad = 135717,
        .expected_quaternion = {33433, -6386, -50010, 25208},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30180, 35465, 46113}
    },
    /* Vector 1700: Angle 2.074 rad */
    {
        .input_axis = {-10321, -57701, 29309},
        .input_angle_rad = 135918,
        .expected_quaternion = {33347, -8885, -49673, 25231},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-29191, 39145, 43709}
    },
    /* Vector 1701: Angle 2.077 rad */
    {
        .input_axis = {-13192, -57113, 29309},
        .input_angle_rad = 136119,
        .expected_quaternion = {33260, -11367, -49212, 25254},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-27833, 42704, 41190}
    },
    /* Vector 1702: Angle 2.080 rad */
    {
        .input_axis = {-16030, -56383, 29309},
        .input_angle_rad = 136320,
        .expected_quaternion = {33173, -13825, -48626, 25276},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26120, 46104, 38563}
    },
    /* Vector 1703: Angle 2.083 rad */
    {
        .input_axis = {-18828, -55511, 29309},
        .input_angle_rad = 136521,
        .expected_quaternion = {33087, -16252, -47917, 25299},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-24067, 49311, 35835}
    },
    /* Vector 1704: Angle 2.086 rad */
    {
        .input_axis = {-21579, -54501, 29309},
        .input_angle_rad = 136722,
        .expected_quaternion = {33000, -18644, -47087, 25322},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-21695, 52291, 33013}
    },
    /* Vector 1705: Angle 2.089 rad */
    {
        .input_axis = {-24276, -53354, 29309},
        .input_angle_rad = 136923,
        .expected_quaternion = {32913, -20992, -46138, 25344},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19029, 55014, 30105}
    },
    /* Vector 1706: Angle 2.092 rad */
    {
        .input_axis = {-26912, -52074, 29309},
        .input_angle_rad = 137124,
        .expected_quaternion = {32826, -23293, -45071, 25367},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16095, 57450, 27119}
    },
    /* Vector 1707: Angle 2.095 rad */
    {
        .input_axis = {-29481, -50664, 29309},
        .input_angle_rad = 137325,
        .expected_quaternion = {32739, -25539, -43889, 25389},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-12921, 59574, 24062}
    },
    /* Vector 1708: Angle 2.098 rad */
    {
        .input_axis = {-31976, -49127, 29309},
        .input_angle_rad = 137526,
        .expected_quaternion = {32652, -27725, -42596, 25412},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-9542, 61362, 20944}
    },
    /* Vector 1709: Angle 2.102 rad */
    {
        .input_axis = {-34392, -47468, 29309},
        .input_angle_rad = 137727,
        .expected_quaternion = {32565, -29845, -41193, 25434},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-5990, 62795, 17772}
    },
    /* Vector 1710: Angle 2.105 rad */
    {
        .input_axis = {-36721, -45690, 29309},
        .input_angle_rad = 137928,
        .expected_quaternion = {32477, -31895, -39685, 25457},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-2302, 63858, 14554}
    },
    /* Vector 1711: Angle 2.108 rad */
    {
        .input_axis = {-38959, -43797, 29309},
        .input_angle_rad = 138130,
        .expected_quaternion = {32390, -33868, -38074, 25479},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {1485, 64537, 11301}
    },
    /* Vector 1712: Angle 2.111 rad */
    {
        .input_axis = {-41099, -41795, 29309},
        .input_angle_rad = 138331,
        .expected_quaternion = {32303, -35760, -36366, 25501},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5332, 64825, 8020}
    },
    /* Vector 1713: Angle 2.114 rad */
    {
        .input_axis = {-43137, -39689, 29309},
        .input_angle_rad = 138532,
        .expected_quaternion = {32215, -37565, -34563, 25523},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {9200, 64715, 4720}
    },
    /* Vector 1714: Angle 2.117 rad */
    {
        .input_axis = {-45066, -37483, 29309},
        .input_angle_rad = 138733,
        .expected_quaternion = {32127, -39280, -32670, 25545},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13048, 64208, 1410}
    },
    /* Vector 1715: Angle 2.120 rad */
    {
        .input_axis = {-46883, -35184, 29309},
        .input_angle_rad = 138934,
        .expected_quaternion = {32040, -40898, -30693, 25567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16838, 63307, -1900}
    },
    /* Vector 1716: Angle 2.123 rad */
    {
        .input_axis = {-48583, -32797, 29309},
        .input_angle_rad = 139135,
        .expected_quaternion = {31952, -42418, -28635, 25589},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20530, 62020, -5203}
    },
    /* Vector 1717: Angle 2.126 rad */
    {
        .input_axis = {-50162, -30328, 29309},
        .input_angle_rad = 139336,
        .expected_quaternion = {31864, -43833, -26502, 25611},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24085, 60356, -8489}
    },
    /* Vector 1718: Angle 2.129 rad */
    {
        .input_axis = {-51615, -27783, 29309},
        .input_angle_rad = 139537,
        .expected_quaternion = {31776, -45142, -24299, 25633},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27466, 58331, -11749}
    },
    /* Vector 1719: Angle 2.132 rad */
    {
        .input_axis = {-52939, -25169, 29309},
        .input_angle_rad = 139738,
        .expected_quaternion = {31688, -46339, -22031, 25655},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30639, 55964, -14975}
    },
    /* Vector 1720: Angle 2.135 rad */
    {
        .input_axis = {-54131, -22491, 29309},
        .input_angle_rad = 139939,
        .expected_quaternion = {31600, -47422, -19704, 25676},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33568, 53277, -18157}
    },
    /* Vector 1721: Angle 2.138 rad */
    {
        .input_axis = {-55187, -19758, 29309},
        .input_angle_rad = 140140,
        .expected_quaternion = {31512, -48388, -17324, 25698},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36224, 50295, -21288}
    },
    /* Vector 1722: Angle 2.141 rad */
    {
        .input_axis = {-56105, -16975, 29309},
        .input_angle_rad = 140341,
        .expected_quaternion = {31424, -49235, -14896, 25720},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38577, 47047, -24359}
    },
    /* Vector 1723: Angle 2.145 rad */
    {
        .input_axis = {-56884, -14150, 29309},
        .input_angle_rad = 140542,
        .expected_quaternion = {31336, -49960, -12427, 25741},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40602, 43563, -27362}
    },
    /* Vector 1724: Angle 2.148 rad */
    {
        .input_axis = {-57520, -11289, 29309},
        .input_angle_rad = 140743,
        .expected_quaternion = {31248, -50561, -9923, 25763},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42276, 39878, -30289}
    },
    /* Vector 1725: Angle 2.151 rad */
    {
        .input_axis = {-58012, -8400, 29309},
        .input_angle_rad = 140944,
        .expected_quaternion = {31159, -51036, -7390, 25784},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43581, 36028, -33131}
    },
    /* Vector 1726: Angle 2.154 rad */
    {
        .input_axis = {-58360, -5490, 29309},
        .input_angle_rad = 141145,
        .expected_quaternion = {31071, -51384, -4834, 25805},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44501, 32049, -35882}
    },
    /* Vector 1727: Angle 2.157 rad */
    {
        .input_axis = {-58561, -2566, 29309},
        .input_angle_rad = 141347,
        .expected_quaternion = {30982, -51604, -2262, 25827},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45024, 27981, -38534}
    },
    /* Vector 1728: Angle 2.160 rad */
    {
        .input_axis = {-58616, 364, 29309},
        .input_angle_rad = 141548,
        .expected_quaternion = {30893, -51695, 321, 25848},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {45144, 23863, -41080}
    },
    /* Vector 1729: Angle 2.163 rad */
    {
        .input_axis = {-58525, 3293, 29309},
        .input_angle_rad = 141749,
        .expected_quaternion = {30805, -51656, 2906, 25869},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44856, 19738, -43513}
    },
    /* Vector 1730: Angle 2.166 rad */
    {
        .input_axis = {-58287, 6214, 29309},
        .input_angle_rad = 141950,
        .expected_quaternion = {30716, -51489, 5489, 25890},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {44161, 15644, -45826}
    },
    /* Vector 1731: Angle 2.169 rad */
    {
        .input_axis = {-57904, 9119, 29309},
        .input_angle_rad = 142151,
        .expected_quaternion = {30627, -51191, 8062, 25911},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43063, 11624, -48015}
    },
    /* Vector 1732: Angle 2.172 rad */
    {
        .input_axis = {-57375, 12002, 29309},
        .input_angle_rad = 142352,
        .expected_quaternion = {30538, -50766, 10619, 25932},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41572, 7716, -50072}
    },
    /* Vector 1733: Angle 2.175 rad */
    {
        .input_axis = {-56704, 14854, 29309},
        .input_angle_rad = 142553,
        .expected_quaternion = {30449, -50212, 13153, 25953},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39701, 3961, -51992}
    },
    /* Vector 1734: Angle 2.178 rad */
    {
        .input_axis = {-55891, 17670, 29309},
        .input_angle_rad = 142754,
        .expected_quaternion = {30360, -49532, 15659, 25974},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37464, 395, -53770}
    },
    /* Vector 1735: Angle 2.181 rad */
    {
        .input_axis = {-54938, 20441, 29309},
        .input_angle_rad = 142955,
        .expected_quaternion = {30271, -48726, 18130, 25995},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34884, -2945, -55402}
    },
    /* Vector 1736: Angle 2.184 rad */
    {
        .input_axis = {-53847, 23161, 29309},
        .input_angle_rad = 143156,
        .expected_quaternion = {30182, -47797, 20559, 26015},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31983, -6026, -56884}
    },
    /* Vector 1737: Angle 2.187 rad */
    {
        .input_axis = {-52623, 25823, 29309},
        .input_angle_rad = 143357,
        .expected_quaternion = {30093, -46747, 22940, 26036},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28789, -8816, -58210}
    },
    /* Vector 1738: Angle 2.191 rad */
    {
        .input_axis = {-51266, 28421, 29309},
        .input_angle_rad = 143558,
        .expected_quaternion = {30003, -45578, 25268, 26057},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25332, -11287, -59379}
    },
    /* Vector 1739: Angle 2.194 rad */
    {
        .input_axis = {-49782, 30948, 29309},
        .input_angle_rad = 143759,
        .expected_quaternion = {29914, -44293, 27536, 26077},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21644, -13415, -60387}
    },
    /* Vector 1740: Angle 2.197 rad */
    {
        .input_axis = {-48173, 33397, 29309},
        .input_angle_rad = 143960,
        .expected_quaternion = {29824, -42895, 29738, 26098},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17762, -15176, -61231}
    },
    /* Vector 1741: Angle 2.200 rad */
    {
        .input_axis = {-46443, 35763, 29309},
        .input_angle_rad = 144161,
        .expected_quaternion = {29735, -41388, 31870, 26118},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13721, -16553, -61909}
    },
    /* Vector 1742: Angle 2.203 rad */
    {
        .input_axis = {-44598, 38040, 29309},
        .input_angle_rad = 144362,
        .expected_quaternion = {29645, -39774, 33925, 26139},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {9562, -17531, -62419}
    },
    /* Vector 1743: Angle 2.206 rad */
    {
        .input_axis = {-42641, 40221, 29309},
        .input_angle_rad = 144564,
        .expected_quaternion = {29555, -38058, 35899, 26159},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5325, -18100, -62761}
    },
    /* Vector 1744: Angle 2.209 rad */
    {
        .input_axis = {-40577, 42302, 29309},
        .input_angle_rad = 144765,
        .expected_quaternion = {29466, -36245, 37785, 26179},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {1051, -18253, -62934}
    },
    /* Vector 1745: Angle 2.212 rad */
    {
        .input_axis = {-38412, 44277, 29309},
        .input_angle_rad = 144966,
        .expected_quaternion = {29376, -34337, 39580, 26199},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-3219, -17988, -62937}
    },
    /* Vector 1746: Angle 2.215 rad */
    {
        .input_axis = {-36152, 46142, 29309},
        .input_angle_rad = 145167,
        .expected_quaternion = {29286, -32341, 41278, 26219},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-7442, -17307, -62770}
    },
    /* Vector 1747: Angle 2.218 rad */
    {
        .input_axis = {-33800, 47891, 29309},
        .input_angle_rad = 145368,
        .expected_quaternion = {29196, -30261, 42876, 26240},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11577, -16216, -62434}
    },
    /* Vector 1748: Angle 2.221 rad */
    {
        .input_axis = {-31364, 49520, 29309},
        .input_angle_rad = 145569,
        .expected_quaternion = {29106, -28102, 44368, 26260},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-15583, -14725, -61930}
    },
    /* Vector 1749: Angle 2.224 rad */
    {
        .input_axis = {-28850, 51026, 29309},
        .input_angle_rad = 145770,
        .expected_quaternion = {29016, -25869, 45752, 26279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19421, -12848, -61259}
    },
    /* Vector 1750: Angle 2.227 rad */
    {
        .input_axis = {-26264, 52404, 29309},
        .input_angle_rad = 145971,
        .expected_quaternion = {28926, -23567, 47023, 26299},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-23052, -10605, -60424}
    },
    /* Vector 1751: Angle 2.230 rad */
    {
        .input_axis = {-23612, 53651, 29309},
        .input_angle_rad = 146172,
        .expected_quaternion = {28835, -21204, 48179, 26319},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26441, -8015, -59427}
    },
    /* Vector 1752: Angle 2.233 rad */
    {
        .input_axis = {-20901, 54764, 29309},
        .input_angle_rad = 146373,
        .expected_quaternion = {28745, -18783, 49215, 26339},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-29553, -5106, -58271}
    },
    /* Vector 1753: Angle 2.237 rad */
    {
        .input_axis = {-18138, 55740, 29309},
        .input_angle_rad = 146574,
        .expected_quaternion = {28655, -16312, 50130, 26359},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-32358, -1905, -56959}
    },
    /* Vector 1754: Angle 2.240 rad */
    {
        .input_axis = {-15329, 56577, 29309},
        .input_angle_rad = 146775,
        .expected_quaternion = {28564, -13797, 50920, 26378},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-34827, 1555, -55494}
    },
    /* Vector 1755: Angle 2.243 rad */
    {
        .input_axis = {-12483, 57273, 29309},
        .input_angle_rad = 146976,
        .expected_quaternion = {28474, -11243, 51585, 26398},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-36936, 5239, -53882}
    },
    /* Vector 1756: Angle 2.246 rad */
    {
        .input_axis = {-9605, 57825, 29309},
        .input_angle_rad = 147177,
        .expected_quaternion = {28383, -8657, 52120, 26417},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-38664, 9112, -52125}
    },
    /* Vector 1757: Angle 2.249 rad */
    {
        .input_axis = {-6703, 58233, 29309},
        .input_angle_rad = 147378,
        .expected_quaternion = {28293, -6046, 52527, 26437},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39992, 13135, -50230}
    },
    /* Vector 1758: Angle 2.252 rad */
    {
        .input_axis = {-3784, 58495, 29309},
        .input_angle_rad = 147579,
        .expected_quaternion = {28202, -3415, 52802, 26456},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-40908, 17266, -48202}
    },
    /* Vector 1759: Angle 2.255 rad */
    {
        .input_axis = {-855, 58611, 29309},
        .input_angle_rad = 147781,
        .expected_quaternion = {28111, -773, 52945, 26475},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-41402, 21464, -46045}
    },
    /* Vector 1760: Angle 2.258 rad */
    {
        .input_axis = {2075, 58580, 29309},
        .input_angle_rad = 147982,
        .expected_quaternion = {28020, 1876, 52956, 26495},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-41468, 25687, -43767}
    },
    /* Vector 1761: Angle 2.261 rad */
    {
        .input_axis = {5000, 58404, 29309},
        .input_angle_rad = 148183,
        .expected_quaternion = {27929, 4523, 52834, 26514},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-41106, 29892, -41373}
    },
    /* Vector 1762: Angle 2.264 rad */
    {
        .input_axis = {7913, 58081, 29309},
        .input_angle_rad = 148384,
        .expected_quaternion = {27838, 7163, 52580, 26533},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-40320, 34036, -38870}
    },
    /* Vector 1763: Angle 2.267 rad */
    {
        .input_axis = {10806, 57613, 29309},
        .input_angle_rad = 148585,
        .expected_quaternion = {27747, 9789, 52194, 26552},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39115, 38077, -36264}
    },
    /* Vector 1764: Angle 2.270 rad */
    {
        .input_axis = {13672, 57001, 29309},
        .input_angle_rad = 148786,
        .expected_quaternion = {27656, 12395, 51676, 26571},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-37506, 41973, -33564}
    },
    /* Vector 1765: Angle 2.273 rad */
    {
        .input_axis = {16503, 56246, 29309},
        .input_angle_rad = 148987,
        .expected_quaternion = {27565, 14973, 51029, 26590},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-35506, 45684, -30777}
    },
    /* Vector 1766: Angle 2.276 rad */
    {
        .input_axis = {19294, 55351, 29309},
        .input_angle_rad = 149188,
        .expected_quaternion = {27474, 17517, 50252, 26609},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-33137, 49173, -27909}
    },
    /* Vector 1767: Angle 2.279 rad */
    {
        .input_axis = {22036, 54317, 29309},
        .input_angle_rad = 149389,
        .expected_quaternion = {27382, 20021, 49349, 26628},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30422, 52402, -24969}
    },
    /* Vector 1768: Angle 2.283 rad */
    {
        .input_axis = {24723, 53148, 29309},
        .input_angle_rad = 149590,
        .expected_quaternion = {27291, 22478, 48321, 26646},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-27387, 55339, -21966}
    },
    /* Vector 1769: Angle 2.286 rad */
    {
        .input_axis = {27349, 51846, 29309},
        .input_angle_rad = 149791,
        .expected_quaternion = {27200, 24882, 47170, 26665},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-24064, 57952, -18906}
    },
    /* Vector 1770: Angle 2.289 rad */
    {
        .input_axis = {29906, 50414, 29309},
        .input_angle_rad = 149992,
        .expected_quaternion = {27108, 27228, 45899, 26684},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-20486, 60213, -15799}
    },
    /* Vector 1771: Angle 2.292 rad */
    {
        .input_axis = {32388, 48857, 29309},
        .input_angle_rad = 150193,
        .expected_quaternion = {27017, 29508, 44512, 26702},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-16689, 62099, -12654}
    },
    /* Vector 1772: Angle 2.295 rad */
    {
        .input_axis = {34790, 47177, 29309},
        .input_angle_rad = 150394,
        .expected_quaternion = {26925, 31718, 43011, 26721},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-12711, 63589, -9477}
    },
    /* Vector 1773: Angle 2.298 rad */
    {
        .input_axis = {37104, 45379, 29309},
        .input_angle_rad = 150595,
        .expected_quaternion = {26833, 33851, 41401, 26739},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-8592, 64666, -6280}
    },
    /* Vector 1774: Angle 2.301 rad */
    {
        .input_axis = {39326, 43468, 29309},
        .input_angle_rad = 150796,
        .expected_quaternion = {26742, 35903, 39685, 26758},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4375, 65318, -3069}
    },
    /* Vector 1775: Angle 2.304 rad */
    {
        .input_axis = {41449, 41448, 29309},
        .input_angle_rad = 150998,
        .expected_quaternion = {26650, 37867, 37867, 26776},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-102, 65536, 146}
    },
    /* Vector 1776: Angle 2.307 rad */
    {
        .input_axis = {43469, 39325, 29309},
        .input_angle_rad = 151199,
        .expected_quaternion = {26558, 39739, 35951, 26794},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {4183, 65316, 3357}
    },
    /* Vector 1777: Angle 2.310 rad */
    {
        .input_axis = {45380, 37103, 29309},
        .input_angle_rad = 151400,
        .expected_quaternion = {26466, 41515, 33943, 26812},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8436, 64659, 6554}
    },
    /* Vector 1778: Angle 2.313 rad */
    {
        .input_axis = {47177, 34789, 29309},
        .input_angle_rad = 151601,
        .expected_quaternion = {26374, 43189, 31847, 26831},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12614, 63570, 9730}
    },
    /* Vector 1779: Angle 2.316 rad */
    {
        .input_axis = {48857, 32387, 29309},
        .input_angle_rad = 151802,
        .expected_quaternion = {26282, 44756, 29669, 26849},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16674, 62058, 12875}
    },
    /* Vector 1780: Angle 2.319 rad */
    {
        .input_axis = {50415, 29905, 29309},
        .input_angle_rad = 152003,
        .expected_quaternion = {26190, 46214, 27413, 26867},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20574, 60135, 15981}
    },
    /* Vector 1781: Angle 2.322 rad */
    {
        .input_axis = {51846, 27348, 29309},
        .input_angle_rad = 152204,
        .expected_quaternion = {26098, 47558, 25086, 26885},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24273, 57821, 19040}
    },
    /* Vector 1782: Angle 2.326 rad */
    {
        .input_axis = {53149, 24723, 29309},
        .input_angle_rad = 152405,
        .expected_quaternion = {26005, 48785, 22693, 26902},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27734, 55136, 22043}
    },
    /* Vector 1783: Angle 2.329 rad */
    {
        .input_axis = {54318, 22035, 29309},
        .input_angle_rad = 152606,
        .expected_quaternion = {25913, 49891, 20240, 26920},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30919, 52105, 24982}
    },
    /* Vector 1784: Angle 2.332 rad */
    {
        .input_axis = {55351, 19293, 29309},
        .input_angle_rad = 152807,
        .expected_quaternion = {25821, 50874, 17733, 26938},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33795, 48757, 27850}
    },
    /* Vector 1785: Angle 2.335 rad */
    {
        .input_axis = {56246, 16503, 29309},
        .input_angle_rad = 153008,
        .expected_quaternion = {25728, 51731, 15178, 26956},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36332, 45126, 30638}
    },
    /* Vector 1786: Angle 2.338 rad */
    {
        .input_axis = {57001, 13671, 29309},
        .input_angle_rad = 153209,
        .expected_quaternion = {25636, 52459, 12582, 26973},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38502, 41244, 33339}
    },
    /* Vector 1787: Angle 2.341 rad */
    {
        .input_axis = {57613, 10805, 29309},
        .input_angle_rad = 153410,
        .expected_quaternion = {25543, 53057, 9950, 26991},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40282, 37151, 35946}
    },
    /* Vector 1788: Angle 2.344 rad */
    {
        .input_axis = {58081, 7912, 29309},
        .input_angle_rad = 153611,
        .expected_quaternion = {25451, 53522, 7291, 27008},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41653, 32886, 38452}
    },
    /* Vector 1789: Angle 2.347 rad */
    {
        .input_axis = {58404, 4999, 29309},
        .input_angle_rad = 153812,
        .expected_quaternion = {25358, 53854, 4610, 27026},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42598, 28491, 40850}
    },
    /* Vector 1790: Angle 2.350 rad */
    {
        .input_axis = {58580, 2074, 29309},
        .input_angle_rad = 154013,
        .expected_quaternion = {25265, 54052, 1914, 27043},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43106, 24008, 43133}
    },
    /* Vector 1791: Angle 2.353 rad */
    {
        .input_axis = {58611, -856, 29309},
        .input_angle_rad = 154215,
        .expected_quaternion = {25172, 54115, -791, 27060},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {43170, 19482, 45296}
    },
    /* Vector 1792: Angle 2.356 rad */
    {
        .input_axis = {58495, -3785, 29309},
        .input_angle_rad = 154416,
        .expected_quaternion = {25080, 54042, -3497, 27078},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {42788, 14958, 47334}
    },
    /* Vector 1793: Angle 2.359 rad */
    {
        .input_axis = {58233, -6703, 29309},
        .input_angle_rad = 154617,
        .expected_quaternion = {24987, 53834, -6197, 27095},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41960, 10480, 49239}
    },
    /* Vector 1794: Angle 2.362 rad */
    {
        .input_axis = {57825, -9605, 29309},
        .input_angle_rad = 154818,
        .expected_quaternion = {24894, 53491, -8885, 27112},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40694, 6092, 51008}
    },
    /* Vector 1795: Angle 2.365 rad */
    {
        .input_axis = {57272, -12483, 29309},
        .input_angle_rad = 155019,
        .expected_quaternion = {24801, 53013, -11555, 27129},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39001, 1839, 52636}
    },
    /* Vector 1796: Angle 2.368 rad */
    {
        .input_axis = {56577, -15330, 29309},
        .input_angle_rad = 155220,
        .expected_quaternion = {24708, 52402, -14199, 27146},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36895, -2239, 54118}
    },
    /* Vector 1797: Angle 2.372 rad */
    {
        .input_axis = {55740, -18139, 29309},
        .input_angle_rad = 155421,
        .expected_quaternion = {24614, 51659, -16811, 27163},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34395, -6099, 55450}
    },
    /* Vector 1798: Angle 2.375 rad */
    {
        .input_axis = {54764, -20902, 29309},
        .input_angle_rad = 155622,
        .expected_quaternion = {24521, 50786, -19384, 27180},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31525, -9703, 56630}
    },
    /* Vector 1799: Angle 2.378 rad */
    {
        .input_axis = {53651, -23613, 29309},
        .input_angle_rad = 155823,
        .expected_quaternion = {24428, 49784, -21911, 27196},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28312, -13015, 57654}
    },
    /* Vector 1800: Angle 2.381 rad */
    {
        .input_axis = {52404, -26265, 29309},
        .input_angle_rad = 156024,
        .expected_quaternion = {24335, 48657, -24387, 27213},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24786, -16003, 58519}
    },
    /* Vector 1801: Angle 2.384 rad */
    {
        .input_axis = {51025, -28851, 29309},
        .input_angle_rad = 156225,
        .expected_quaternion = {24241, 47406, -26805, 27230},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20981, -18635, 59224}
    },
    /* Vector 1802: Angle 2.387 rad */
    {
        .input_axis = {49520, -31365, 29309},
        .input_angle_rad = 156426,
        .expected_quaternion = {24148, 46035, -29158, 27246},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16934, -20886, 59766}
    },
    /* Vector 1803: Angle 2.390 rad */
    {
        .input_axis = {47890, -33801, 29309},
        .input_angle_rad = 156627,
        .expected_quaternion = {24054, 44548, -31442, 27263},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12684, -22732, 60145}
    },
    /* Vector 1804: Angle 2.393 rad */
    {
        .input_axis = {46141, -36152, 29309},
        .input_angle_rad = 156828,
        .expected_quaternion = {23961, 42946, -33649, 27279},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8271, -24154, 60358}
    },
    /* Vector 1805: Angle 2.396 rad */
    {
        .input_axis = {44276, -38413, 29309},
        .input_angle_rad = 157029,
        .expected_quaternion = {23867, 41236, -35775, 27296},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {3740, -25139, 60407}
    },
    /* Vector 1806: Angle 2.399 rad */
    {
        .input_axis = {42301, -40578, 29309},
        .input_angle_rad = 157230,
        .expected_quaternion = {23774, 39420, -37814, 27312},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-866, -25675, 60291}
    },
    /* Vector 1807: Angle 2.402 rad */
    {
        .input_axis = {40220, -42642, 29309},
        .input_angle_rad = 157431,
        .expected_quaternion = {23680, 37503, -39761, 27328},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-5501, -25757, 60011}
    },
    /* Vector 1808: Angle 2.405 rad */
    {
        .input_axis = {38039, -44598, 29309},
        .input_angle_rad = 157633,
        .expected_quaternion = {23586, 35490, -41610, 27345},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-10121, -25384, 59567}
    },
    /* Vector 1809: Angle 2.408 rad */
    {
        .input_axis = {35762, -46444, 29309},
        .input_angle_rad = 157834,
        .expected_quaternion = {23492, 33386, -43357, 27361},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14679, -24559, 58961}
    },
    /* Vector 1810: Angle 2.411 rad */
    {
        .input_axis = {33396, -48173, 29309},
        .input_angle_rad = 158035,
        .expected_quaternion = {23398, 31195, -44998, 27377},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19130, -23290, 58195}
    },
    /* Vector 1811: Angle 2.414 rad */
    {
        .input_axis = {30947, -49782, 29309},
        .input_angle_rad = 158236,
        .expected_quaternion = {23304, 28924, -46528, 27393},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-23430, -21589, 57270}
    },
    /* Vector 1812: Angle 2.418 rad */
    {
        .input_axis = {28420, -51267, 29309},
        .input_angle_rad = 158437,
        .expected_quaternion = {23210, 26578, -47944, 27409},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-27538, -19473, 56191}
    },
    /* Vector 1813: Angle 2.421 rad */
    {
        .input_axis = {25823, -52623, 29309},
        .input_angle_rad = 158638,
        .expected_quaternion = {23116, 24163, -49241, 27425},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-31411, -16963, 54960}
    },
    /* Vector 1814: Angle 2.424 rad */
    {
        .input_axis = {23160, -53848, 29309},
        .input_angle_rad = 158839,
        .expected_quaternion = {23022, 21684, -50416, 27441},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-35011, -14083, 53580}
    },
    /* Vector 1815: Angle 2.427 rad */
    {
        .input_axis = {20440, -54938, 29309},
        .input_angle_rad = 159040,
        .expected_quaternion = {22928, 19148, -51466, 27456},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-38303, -10863, 52056}
    },
    /* Vector 1816: Angle 2.430 rad */
    {
        .input_axis = {17669, -55891, 29309},
        .input_angle_rad = 159241,
        .expected_quaternion = {22834, 16562, -52389, 27472},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-41254, -7335, 50391}
    },
    /* Vector 1817: Angle 2.433 rad */
    {
        .input_axis = {14853, -56704, 29309},
        .input_angle_rad = 159442,
        .expected_quaternion = {22740, 13930, -53181, 27488},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43833, -3533, 48591}
    },
    /* Vector 1818: Angle 2.436 rad */
    {
        .input_axis = {12001, -57376, 29309},
        .input_angle_rad = 159643,
        .expected_quaternion = {22645, 11261, -53841, 27503},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-46016, 503, 46661}
    },
    /* Vector 1819: Angle 2.439 rad */
    {
        .input_axis = {9118, -57904, 29309},
        .input_angle_rad = 159844,
        .expected_quaternion = {22551, 8561, -54368, 27519},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-47780, 4734, 44606}
    },
    /* Vector 1820: Angle 2.442 rad */
    {
        .input_axis = {6213, -58287, 29309},
        .input_angle_rad = 160045,
        .expected_quaternion = {22457, 5837, -54758, 27534},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-49106, 9116, 42431}
    },
    /* Vector 1821: Angle 2.445 rad */
    {
        .input_axis = {3292, -58525, 29309},
        .input_angle_rad = 160246,
        .expected_quaternion = {22362, 3094, -55012, 27550},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-49983, 13606, 40144}
    },
    /* Vector 1822: Angle 2.448 rad */
    {
        .input_axis = {363, -58616, 29309},
        .input_angle_rad = 160447,
        .expected_quaternion = {22268, 341, -55129, 27565},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-50400, 18158, 37750}
    },
    /* Vector 1823: Angle 2.451 rad */
    {
        .input_axis = {-2567, -58561, 29309},
        .input_angle_rad = 160648,
        .expected_quaternion = {22173, -2416, -55107, 27580},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-50354, 22726, 35256}
    },
    /* Vector 1824: Angle 2.454 rad */
    {
        .input_axis = {-5491, -58359, 29309},
        .input_angle_rad = 160850,
        .expected_quaternion = {22078, -5170, -54948, 27595},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-49844, 27263, 32669}
    },
    /* Vector 1825: Angle 2.457 rad */
    {
        .input_axis = {-8401, -58012, 29309},
        .input_angle_rad = 161051,
        .expected_quaternion = {21984, -7914, -54651, 27610},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-48876, 31723, 29996}
    },
    /* Vector 1826: Angle 2.461 rad */
    {
        .input_axis = {-11290, -57520, 29309},
        .input_angle_rad = 161252,
        .expected_quaternion = {21889, -10641, -54217, 27625},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-47458, 36061, 27245}
    },
    /* Vector 1827: Angle 2.464 rad */
    {
        .input_axis = {-14150, -56884, 29309},
        .input_angle_rad = 161453,
        .expected_quaternion = {21794, -13345, -53646, 27640},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-45606, 40232, 24423}
    },
    /* Vector 1828: Angle 2.467 rad */
    {
        .input_axis = {-16976, -56105, 29309},
        .input_angle_rad = 161654,
        .expected_quaternion = {21699, -16018, -52941, 27655},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43336, 44193, 21539}
    },
    /* Vector 1829: Angle 2.470 rad */
    {
        .input_axis = {-19759, -55187, 29309},
        .input_angle_rad = 161855,
        .expected_quaternion = {21604, -18654, -52102, 27670},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-40672, 47904, 18599}
    },
    /* Vector 1830: Angle 2.473 rad */
    {
        .input_axis = {-22492, -54130, 29309},
        .input_angle_rad = 162056,
        .expected_quaternion = {21510, -21246, -51132, 27685},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-37641, 51326, 15613}
    },
    /* Vector 1831: Angle 2.476 rad */
    {
        .input_axis = {-25169, -52938, 29309},
        .input_angle_rad = 162257,
        .expected_quaternion = {21415, -23788, -50032, 27700},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-34272, 54423, 12589}
    },
    /* Vector 1832: Angle 2.479 rad */
    {
        .input_axis = {-27784, -51614, 29309},
        .input_angle_rad = 162458,
        .expected_quaternion = {21320, -26273, -48807, 27714},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30600, 57164, 9534}
    },
    /* Vector 1833: Angle 2.482 rad */
    {
        .input_axis = {-30329, -50161, 29309},
        .input_angle_rad = 162659,
        .expected_quaternion = {21224, -28694, -47458, 27729},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26662, 59518, 6458}
    },
    /* Vector 1834: Angle 2.485 rad */
    {
        .input_axis = {-32798, -48583, 29309},
        .input_angle_rad = 162860,
        .expected_quaternion = {21129, -31046, -45988, 27744},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22496, 61462, 3368}
    },
    /* Vector 1835: Angle 2.488 rad */
    {
        .input_axis = {-35185, -46883, 29309},
        .input_angle_rad = 163061,
        .expected_quaternion = {21034, -33323, -44402, 27758},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18146, 62973, 274}
    },
    /* Vector 1836: Angle 2.491 rad */
    {
        .input_axis = {-37484, -45066, 29309},
        .input_angle_rad = 163262,
        .expected_quaternion = {20939, -35519, -42704, 27772},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13654, 64036, -2817}
    },
    /* Vector 1837: Angle 2.494 rad */
    {
        .input_axis = {-39690, -43136, 29309},
        .input_angle_rad = 163463,
        .expected_quaternion = {20844, -37629, -40896, 27787},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-9067, 64638, -5895}
    },
    /* Vector 1838: Angle 2.497 rad */
    {
        .input_axis = {-41796, -41098, 29309},
        .input_angle_rad = 163664,
        .expected_quaternion = {20748, -39646, -38984, 27801},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4431, 64770, -8952}
    },
    /* Vector 1839: Angle 2.500 rad */
    {
        .input_axis = {-43798, -38958, 29309},
        .input_angle_rad = 163865,
        .expected_quaternion = {20653, -41566, -36973, 27815},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {207, 64431, -11980}
    },
    /* Vector 1840: Angle 2.503 rad */
    {
        .input_axis = {-45690, -36720, 29309},
        .input_angle_rad = 164067,
        .expected_quaternion = {20557, -43384, -34867, 27829},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {4800, 63622, -14971}
    },
    /* Vector 1841: Angle 2.507 rad */
    {
        .input_axis = {-47468, -34391, 29309},
        .input_angle_rad = 164268,
        .expected_quaternion = {20462, -45095, -32672, 27843},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {9301, 62350, -17916}
    },
    /* Vector 1842: Angle 2.510 rad */
    {
        .input_axis = {-49128, -31976, 29309},
        .input_angle_rad = 164469,
        .expected_quaternion = {20366, -46695, -30392, 27857},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {13664, 60624, -20808}
    },
    /* Vector 1843: Angle 2.513 rad */
    {
        .input_axis = {-50664, -29480, 29309},
        .input_angle_rad = 164670,
        .expected_quaternion = {20271, -48180, -28035, 27871},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17845, 58462, -23638}
    },
    /* Vector 1844: Angle 2.516 rad */
    {
        .input_axis = {-52075, -26911, 29309},
        .input_angle_rad = 164871,
        .expected_quaternion = {20175, -49546, -25604, 27885},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21799, 55883, -26398}
    },
    /* Vector 1845: Angle 2.519 rad */
    {
        .input_axis = {-53354, -24275, 29309},
        .input_angle_rad = 165072,
        .expected_quaternion = {20080, -50788, -23107, 27899},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25487, 52911, -29082}
    },
    /* Vector 1846: Angle 2.522 rad */
    {
        .input_axis = {-54501, -21578, 29309},
        .input_angle_rad = 165273,
        .expected_quaternion = {19984, -51905, -20550, 27913},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28871, 49575, -31682}
    },
    /* Vector 1847: Angle 2.525 rad */
    {
        .input_axis = {-55511, -18827, 29309},
        .input_angle_rad = 165474,
        .expected_quaternion = {19888, -52894, -17939, 27926},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31915, 45907, -34190}
    },
    /* Vector 1848: Angle 2.528 rad */
    {
        .input_axis = {-56383, -16029, 29309},
        .input_angle_rad = 165675,
        .expected_quaternion = {19792, -53750, -15281, 27940},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34587, 41941, -36601}
    },
    /* Vector 1849: Angle 2.531 rad */
    {
        .input_axis = {-57114, -13191, 29309},
        .input_angle_rad = 165876,
        .expected_quaternion = {19696, -54473, -12581, 27954},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36859, 37718, -38907}
    },
    /* Vector 1850: Angle 2.534 rad */
    {
        .input_axis = {-57702, -10320, 29309},
        .input_angle_rad = 166077,
        .expected_quaternion = {19600, -55060, -9848, 27967},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38707, 33276, -41103}
    },
    /* Vector 1851: Angle 2.537 rad */
    {
        .input_axis = {-58145, -7423, 29309},
        .input_angle_rad = 166278,
        .expected_quaternion = {19505, -55510, -7087, 27980},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40111, 28661, -43182}
    },
    /* Vector 1852: Angle 2.540 rad */
    {
        .input_axis = {-58444, -4508, 29309},
        .input_angle_rad = 166479,
        .expected_quaternion = {19409, -55822, -4306, 27994},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41055, 23916, -45138}
    },
    /* Vector 1853: Angle 2.543 rad */
    {
        .input_axis = {-58596, -1581, 29309},
        .input_angle_rad = 166680,
        .expected_quaternion = {19313, -55994, -1511, 28007},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41528, 19089, -46968}
    },
    /* Vector 1854: Angle 2.546 rad */
    {
        .input_axis = {-58602, 1349, 29309},
        .input_angle_rad = 166881,
        .expected_quaternion = {19216, -56026, 1290, 28020},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41525, 14227, -48665}
    },
    /* Vector 1855: Angle 2.549 rad */
    {
        .input_axis = {-58461, 4276, 29309},
        .input_angle_rad = 167082,
        .expected_quaternion = {19120, -55918, 4090, 28033},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {41042, 9378, -50225}
    },
    /* Vector 1856: Angle 2.553 rad */
    {
        .input_axis = {-58174, 7193, 29309},
        .input_angle_rad = 167284,
        .expected_quaternion = {19024, -55669, 6883, 28047},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40085, 4590, -51644}
    },
    /* Vector 1857: Angle 2.556 rad */
    {
        .input_axis = {-57742, 10091, 29309},
        .input_angle_rad = 167485,
        .expected_quaternion = {18928, -55281, 9661, 28060},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38660, -91, -52919}
    },
    /* Vector 1858: Angle 2.559 rad */
    {
        .input_axis = {-57166, 12965, 29309},
        .input_angle_rad = 167686,
        .expected_quaternion = {18832, -54755, 12418, 28073},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36780, -4617, -54045}
    },
    /* Vector 1859: Angle 2.562 rad */
    {
        .input_axis = {-56446, 15805, 29309},
        .input_angle_rad = 167887,
        .expected_quaternion = {18735, -54090, 15146, 28085},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34463, -8943, -55021}
    },
    /* Vector 1860: Angle 2.565 rad */
    {
        .input_axis = {-55586, 18607, 29309},
        .input_angle_rad = 168088,
        .expected_quaternion = {18639, -53290, 17838, 28098},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31731, -13028, -55843}
    },
    /* Vector 1861: Angle 2.568 rad */
    {
        .input_axis = {-54586, 21362, 29309},
        .input_angle_rad = 168289,
        .expected_quaternion = {18543, -52356, 20489, 28111},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28609, -16829, -56509}
    },
    /* Vector 1862: Angle 2.571 rad */
    {
        .input_axis = {-53450, 24063, 29309},
        .input_angle_rad = 168490,
        .expected_quaternion = {18446, -51289, 23090, 28124},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {25128, -20310, -57018}
    },
    /* Vector 1863: Angle 2.574 rad */
    {
        .input_axis = {-52181, 26704, 29309},
        .input_angle_rad = 168691,
        .expected_quaternion = {18350, -50094, 25636, 28136},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {21320, -23435, -57369}
    },
    /* Vector 1864: Angle 2.577 rad */
    {
        .input_axis = {-50781, 29279, 29309},
        .input_angle_rad = 168892,
        .expected_quaternion = {18253, -48772, 28120, 28149},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {17223, -26174, -57561}
    },
    /* Vector 1865: Angle 2.580 rad */
    {
        .input_axis = {-49254, 31780, 29309},
        .input_angle_rad = 169093,
        .expected_quaternion = {18156, -47326, 30536, 28161},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12877, -28499, -57593}
    },
    /* Vector 1866: Angle 2.583 rad */
    {
        .input_axis = {-47604, 34202, 29309},
        .input_angle_rad = 169294,
        .expected_quaternion = {18060, -45761, 32878, 28174},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {8324, -30387, -57466}
    },
    /* Vector 1867: Angle 2.586 rad */
    {
        .input_axis = {-45835, 36539, 29309},
        .input_angle_rad = 169495,
        .expected_quaternion = {17963, -44080, 35140, 28186},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {3608, -31819, -57180}
    },
    /* Vector 1868: Angle 2.589 rad */
    {
        .input_axis = {-43952, 38784, 29309},
        .input_angle_rad = 169696,
        .expected_quaternion = {17867, -42287, 37315, 28198},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-1223, -32780, -56736}
    },
    /* Vector 1869: Angle 2.592 rad */
    {
        .input_axis = {-41959, 40932, 29309},
        .input_angle_rad = 169897,
        .expected_quaternion = {17770, -40387, 39399, 28211},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-6123, -33261, -56135}
    },
    /* Vector 1870: Angle 2.595 rad */
    {
        .input_axis = {-39860, 42978, 29309},
        .input_angle_rad = 170098,
        .expected_quaternion = {17673, -38384, 41386, 28223},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-11042, -33257, -55381}
    },
    /* Vector 1871: Angle 2.599 rad */
    {
        .input_axis = {-37663, 44917, 29309},
        .input_angle_rad = 170299,
        .expected_quaternion = {17576, -36283, 43271, 28235},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-15934, -32768, -54473}
    },
    /* Vector 1872: Angle 2.602 rad */
    {
        .input_axis = {-35371, 46743, 29309},
        .input_angle_rad = 170501,
        .expected_quaternion = {17479, -34089, 45050, 28247},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-20748, -31799, -53417}
    },
    /* Vector 1873: Angle 2.605 rad */
    {
        .input_axis = {-32990, 48452, 29309},
        .input_angle_rad = 170702,
        .expected_quaternion = {17382, -31809, 46717, 28259},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-25438, -30359, -52213}
    },
    /* Vector 1874: Angle 2.608 rad */
    {
        .input_axis = {-30527, 50040, 29309},
        .input_angle_rad = 170903,
        .expected_quaternion = {17285, -29446, 48269, 28271},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-29956, -28463, -50867}
    },
    /* Vector 1875: Angle 2.611 rad */
    {
        .input_axis = {-27988, 51504, 29309},
        .input_angle_rad = 171104,
        .expected_quaternion = {17188, -27009, 49701, 28283},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-34258, -26129, -49382}
    },
    /* Vector 1876: Angle 2.614 rad */
    {
        .input_axis = {-25379, 52838, 29309},
        .input_angle_rad = 171305,
        .expected_quaternion = {17091, -24501, 51010, 28294},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-38302, -23382, -47762}
    },
    /* Vector 1877: Angle 2.617 rad */
    {
        .input_axis = {-22707, 54041, 29309},
        .input_angle_rad = 171506,
        .expected_quaternion = {16994, -21930, 52192, 28306},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-42046, -20249, -46012}
    },
    /* Vector 1878: Angle 2.620 rad */
    {
        .input_axis = {-19977, 55108, 29309},
        .input_angle_rad = 171707,
        .expected_quaternion = {16897, -19302, 53245, 28318},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-45453, -16761, -44137}
    },
    /* Vector 1879: Angle 2.623 rad */
    {
        .input_axis = {-17198, 56037, 29309},
        .input_angle_rad = 171908,
        .expected_quaternion = {16800, -16623, 54165, 28329},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-48489, -12954, -42142}
    },
    /* Vector 1880: Angle 2.626 rad */
    {
        .input_axis = {-14376, 56827, 29309},
        .input_angle_rad = 172109,
        .expected_quaternion = {16703, -13901, 54950, 28341},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-51125, -8866, -40033}
    },
    /* Vector 1881: Angle 2.629 rad */
    {
        .input_axis = {-11518, 57474, 29309},
        .input_angle_rad = 172310,
        .expected_quaternion = {16606, -11142, 55599, 28352},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53332, -4537, -37816}
    },
    /* Vector 1882: Angle 2.632 rad */
    {
        .input_axis = {-8631, 57978, 29309},
        .input_angle_rad = 172511,
        .expected_quaternion = {16508, -8353, 56109, 28364},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-55090, -13, -35497}
    },
    /* Vector 1883: Angle 2.635 rad */
    {
        .input_axis = {-5722, 58337, 29309},
        .input_angle_rad = 172712,
        .expected_quaternion = {16411, -5540, 56479, 28375},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-56380, 4662, -33083}
    },
    /* Vector 1884: Angle 2.638 rad */
    {
        .input_axis = {-2800, 58550, 29309},
        .input_angle_rad = 172913,
        .expected_quaternion = {16314, -2712, 56707, 28386},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-57190, 9440, -30581}
    },
    /* Vector 1885: Angle 2.642 rad */
    {
        .input_axis = {130, 58617, 29309},
        .input_angle_rad = 173114,
        .expected_quaternion = {16216, 126, 56794, 28397},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-57510, 14272, -27997}
    },
    /* Vector 1886: Angle 2.645 rad */
    {
        .input_axis = {3060, 58537, 29309},
        .input_angle_rad = 173315,
        .expected_quaternion = {16119, 2966, 56739, 28408},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-57339, 19109, -25339}
    },
    /* Vector 1887: Angle 2.648 rad */
    {
        .input_axis = {5981, 58311, 29309},
        .input_angle_rad = 173516,
        .expected_quaternion = {16021, 5800, 56542, 28419},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-56676, 23903, -22615}
    },
    /* Vector 1888: Angle 2.651 rad */
    {
        .input_axis = {8888, 57939, 29309},
        .input_angle_rad = 173718,
        .expected_quaternion = {15924, 8622, 56203, 28430},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-55529, 28604, -19832}
    },
    /* Vector 1889: Angle 2.654 rad */
    {
        .input_axis = {11773, 57423, 29309},
        .input_angle_rad = 173919,
        .expected_quaternion = {15826, 11425, 55723, 28441},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53909, 33164, -16997}
    },
    /* Vector 1890: Angle 2.657 rad */
    {
        .input_axis = {14628, 56763, 29309},
        .input_angle_rad = 174120,
        .expected_quaternion = {15729, 14201, 55104, 28452},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-51832, 37537, -14120}
    },
    /* Vector 1891: Angle 2.660 rad */
    {
        .input_axis = {17447, 55961, 29309},
        .input_angle_rad = 174321,
        .expected_quaternion = {15631, 16943, 54345, 28463},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-49319, 41678, -11207}
    },
    /* Vector 1892: Angle 2.663 rad */
    {
        .input_axis = {20222, 55019, 29309},
        .input_angle_rad = 174522,
        .expected_quaternion = {15534, 19646, 53451, 28473},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-46394, 45544, -8267}
    },
    /* Vector 1893: Angle 2.666 rad */
    {
        .input_axis = {22946, 53939, 29309},
        .input_angle_rad = 174723,
        .expected_quaternion = {15436, 22301, 52422, 28484},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43087, 49094, -5309}
    },
    /* Vector 1894: Angle 2.669 rad */
    {
        .input_axis = {25614, 52725, 29309},
        .input_angle_rad = 174924,
        .expected_quaternion = {15338, 24902, 51261, 28495},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39432, 52294, -2340}
    },
    /* Vector 1895: Angle 2.672 rad */
    {
        .input_axis = {28217, 51379, 29309},
        .input_angle_rad = 175125,
        .expected_quaternion = {15240, 27443, 49970, 28505},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-35464, 55108, 632}
    },
    /* Vector 1896: Angle 2.675 rad */
    {
        .input_axis = {30749, 49904, 29309},
        .input_angle_rad = 175326,
        .expected_quaternion = {15143, 29917, 48554, 28515},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-31224, 57507, 3597}
    },
    /* Vector 1897: Angle 2.678 rad */
    {
        .input_axis = {33205, 48305, 29309},
        .input_angle_rad = 175527,
        .expected_quaternion = {15045, 32318, 47015, 28526},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-26754, 59467, 6548}
    },
    /* Vector 1898: Angle 2.681 rad */
    {
        .input_axis = {35578, 46585, 29309},
        .input_angle_rad = 175728,
        .expected_quaternion = {14947, 34640, 45358, 28536},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-22099, 60966, 9477}
    },
    /* Vector 1899: Angle 2.684 rad */
    {
        .input_axis = {37862, 44749, 29309},
        .input_angle_rad = 175929,
        .expected_quaternion = {14849, 36877, 43585, 28546},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-17306, 61987, 12375}
    },
    /* Vector 1900: Angle 2.688 rad */
    {
        .input_axis = {40051, 42801, 29309},
        .input_angle_rad = 176130,
        .expected_quaternion = {14751, 39023, 41702, 28557},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-12423, 62518, 15235}
    },
    /* Vector 1901: Angle 2.691 rad */
    {
        .input_axis = {42140, 40746, 29309},
        .input_angle_rad = 176331,
        .expected_quaternion = {14653, 41073, 39714, 28567},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-7500, 62554, 18048}
    },
    /* Vector 1902: Angle 2.694 rad */
    {
        .input_axis = {44124, 38588, 29309},
        .input_angle_rad = 176532,
        .expected_quaternion = {14555, 43022, 37625, 28577},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-2587, 62092, 20806}
    },
    /* Vector 1903: Angle 2.697 rad */
    {
        .input_axis = {45997, 36335, 29309},
        .input_angle_rad = 176733,
        .expected_quaternion = {14457, 44864, 35440, 28587},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2268, 61134, 23503}
    },
    /* Vector 1904: Angle 2.700 rad */
    {
        .input_axis = {47756, 33991, 29309},
        .input_angle_rad = 176934,
        .expected_quaternion = {14359, 46595, 33165, 28596},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {7014, 59691, 26131}
    },
    /* Vector 1905: Angle 2.703 rad */
    {
        .input_axis = {49395, 31561, 29309},
        .input_angle_rad = 177136,
        .expected_quaternion = {14261, 48211, 30805, 28606},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {11603, 57773, 28681}
    },
    /* Vector 1906: Angle 2.706 rad */
    {
        .input_axis = {50911, 29053, 29309},
        .input_angle_rad = 177337,
        .expected_quaternion = {14163, 49708, 28367, 28616},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15989, 55399, 31149}
    },
    /* Vector 1907: Angle 2.709 rad */
    {
        .input_axis = {52299, 26472, 29309},
        .input_angle_rad = 177538,
        .expected_quaternion = {14065, 51080, 25856, 28626},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20127, 52592, 33525}
    },
    /* Vector 1908: Angle 2.712 rad */
    {
        .input_axis = {53557, 23826, 29309},
        .input_angle_rad = 177739,
        .expected_quaternion = {13966, 52326, 23278, 28635},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23975, 49377, 35805}
    },
    /* Vector 1909: Angle 2.715 rad */
    {
        .input_axis = {54681, 21119, 29309},
        .input_angle_rad = 177940,
        .expected_quaternion = {13868, 53442, 20641, 28645},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {27494, 45787, 37982}
    },
    /* Vector 1910: Angle 2.718 rad */
    {
        .input_axis = {55668, 18360, 29309},
        .input_angle_rad = 178141,
        .expected_quaternion = {13770, 54425, 17950, 28654},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30646, 41855, 40050}
    },
    /* Vector 1911: Angle 2.721 rad */
    {
        .input_axis = {56516, 15555, 29309},
        .input_angle_rad = 178342,
        .expected_quaternion = {13672, 55272, 15212, 28664},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33400, 37619, 42002}
    },
    /* Vector 1912: Angle 2.724 rad */
    {
        .input_axis = {57223, 12711, 29309},
        .input_angle_rad = 178543,
        .expected_quaternion = {13573, 55982, 12435, 28673},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35727, 33121, 43835}
    },
    /* Vector 1913: Angle 2.727 rad */
    {
        .input_axis = {57786, 9835, 29309},
        .input_angle_rad = 178744,
        .expected_quaternion = {13475, 56552, 9625, 28682},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37603, 28405, 45543}
    },
    /* Vector 1914: Angle 2.730 rad */
    {
        .input_axis = {58206, 6934, 29309},
        .input_angle_rad = 178945,
        .expected_quaternion = {13376, 56980, 6788, 28692},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39007, 23517, 47121}
    },
    /* Vector 1915: Angle 2.734 rad */
    {
        .input_axis = {58479, 4017, 29309},
        .input_angle_rad = 179146,
        .expected_quaternion = {13278, 57267, 3933, 28701},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39926, 18504, 48565}
    },
    /* Vector 1916: Angle 2.737 rad */
    {
        .input_axis = {58607, 1089, 29309},
        .input_angle_rad = 179347,
        .expected_quaternion = {13180, 57410, 1067, 28710},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40347, 13416, 49871}
    },
    /* Vector 1917: Angle 2.740 rad */
    {
        .input_axis = {58588, -1842, 29309},
        .input_angle_rad = 179548,
        .expected_quaternion = {13081, 57409, -1805, 28719},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {40267, 8303, 51036}
    },
    /* Vector 1918: Angle 2.743 rad */
    {
        .input_axis = {58423, -4768, 29309},
        .input_angle_rad = 179749,
        .expected_quaternion = {12983, 57265, -4673, 28728},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39684, 3215, 52056}
    },
    /* Vector 1919: Angle 2.746 rad */
    {
        .input_axis = {58112, -7682, 29309},
        .input_angle_rad = 179950,
        .expected_quaternion = {12884, 56978, -7532, 28737},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38604, -1797, 52929}
    },
    /* Vector 1920: Angle 2.749 rad */
    {
        .input_axis = {57655, -10576, 29309},
        .input_angle_rad = 180151,
        .expected_quaternion = {12785, 56547, -10373, 28745},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37036, -6685, 53653}
    },
    /* Vector 1921: Angle 2.752 rad */
    {
        .input_axis = {57054, -13445, 29309},
        .input_angle_rad = 180353,
        .expected_quaternion = {12687, 55975, -13190, 28754},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {34994, -11399, 54226}
    },
    /* Vector 1922: Angle 2.755 rad */
    {
        .input_axis = {56311, -16279, 29309},
        .input_angle_rad = 180554,
        .expected_quaternion = {12588, 55263, -15976, 28763},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32499, -15894, 54645}
    },
    /* Vector 1923: Angle 2.758 rad */
    {
        .input_axis = {55427, -19073, 29309},
        .input_angle_rad = 180755,
        .expected_quaternion = {12490, 54411, -18724, 28771},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {29575, -20125, 54912}
    },
    /* Vector 1924: Angle 2.761 rad */
    {
        .input_axis = {54405, -21820, 29309},
        .input_angle_rad = 180956,
        .expected_quaternion = {12391, 53423, -21426, 28780},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26249, -24050, 55024}
    },
    /* Vector 1925: Angle 2.764 rad */
    {
        .input_axis = {53246, -24512, 29309},
        .input_angle_rad = 181157,
        .expected_quaternion = {12292, 52301, -24077, 28788},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {22553, -27630, 54981}
    },
    /* Vector 1926: Angle 2.767 rad */
    {
        .input_axis = {51955, -27142, 29309},
        .input_angle_rad = 181358,
        .expected_quaternion = {12193, 51047, -26668, 28797},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {18525, -30829, 54784}
    },
    /* Vector 1927: Angle 2.770 rad */
    {
        .input_axis = {50533, -29705, 29309},
        .input_angle_rad = 181559,
        .expected_quaternion = {12095, 49665, -29195, 28805},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {14203, -33617, 54434}
    },
    /* Vector 1928: Angle 2.773 rad */
    {
        .input_axis = {48985, -32193, 29309},
        .input_angle_rad = 181760,
        .expected_quaternion = {11996, 48158, -31650, 28813},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {9631, -35966, 53932}
    },
    /* Vector 1929: Angle 2.777 rad */
    {
        .input_axis = {47315, -34601, 29309},
        .input_angle_rad = 181961,
        .expected_quaternion = {11897, 46529, -34027, 28822},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {4852, -37852, 53279}
    },
    /* Vector 1930: Angle 2.780 rad */
    {
        .input_axis = {45527, -36923, 29309},
        .input_angle_rad = 182162,
        .expected_quaternion = {11798, 44783, -36320, 28830},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-85, -39257, 52477}
    },
    /* Vector 1931: Angle 2.783 rad */
    {
        .input_axis = {43624, -39152, 29309},
        .input_angle_rad = 182363,
        .expected_quaternion = {11699, 42924, -38523, 28838},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-5132, -40167, 51529}
    },
    /* Vector 1932: Angle 2.786 rad */
    {
        .input_axis = {41613, -41284, 29309},
        .input_angle_rad = 182564,
        .expected_quaternion = {11600, 40956, -40632, 28846},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-10240, -40573, 50438}
    },
    /* Vector 1933: Angle 2.789 rad */
    {
        .input_axis = {39498, -43312, 29309},
        .input_angle_rad = 182765,
        .expected_quaternion = {11501, 38885, -42640, 28854},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-15356, -40472, 49206}
    },
    /* Vector 1934: Angle 2.792 rad */
    {
        .input_axis = {37284, -45232, 29309},
        .input_angle_rad = 182966,
        .expected_quaternion = {11402, 36715, -44542, 28862},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-20431, -39864, 47837}
    },
    /* Vector 1935: Angle 2.795 rad */
    {
        .input_axis = {34976, -47039, 29309},
        .input_angle_rad = 183167,
        .expected_quaternion = {11303, 34452, -46334, 28869},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-25414, -38757, 46336}
    },
    /* Vector 1936: Angle 2.798 rad */
    {
        .input_axis = {32582, -48728, 29309},
        .input_angle_rad = 183368,
        .expected_quaternion = {11204, 32102, -48010, 28877},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30255, -37161, 44706}
    },
    /* Vector 1937: Angle 2.801 rad */
    {
        .input_axis = {30106, -50295, 29309},
        .input_angle_rad = 183570,
        .expected_quaternion = {11105, 29670, -49568, 28885},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-34907, -35093, 42953}
    },
    /* Vector 1938: Angle 2.804 rad */
    {
        .input_axis = {27554, -51737, 29309},
        .input_angle_rad = 183771,
        .expected_quaternion = {11006, 27163, -51002, 28892},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39323, -32574, 41081}
    },
    /* Vector 1939: Angle 2.807 rad */
    {
        .input_axis = {24934, -53050, 29309},
        .input_angle_rad = 183972,
        .expected_quaternion = {10907, 24586, -52310, 28900},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43458, -29630, 39095}
    },
    /* Vector 1940: Angle 2.810 rad */
    {
        .input_axis = {22252, -54230, 29309},
        .input_angle_rad = 184173,
        .expected_quaternion = {10808, 21947, -53487, 28907},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-47272, -26289, 37003}
    },
    /* Vector 1941: Angle 2.813 rad */
    {
        .input_axis = {19513, -55274, 29309},
        .input_angle_rad = 184374,
        .expected_quaternion = {10709, 19251, -54531, 28915},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-50726, -22588, 34808}
    },
    /* Vector 1942: Angle 2.816 rad */
    {
        .input_axis = {16726, -56180, 29309},
        .input_angle_rad = 184575,
        .expected_quaternion = {10609, 16506, -55439, 28922},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53787, -18561, 32518}
    },
    /* Vector 1943: Angle 2.819 rad */
    {
        .input_axis = {13898, -56946, 29309},
        .input_angle_rad = 184776,
        .expected_quaternion = {10510, 13718, -56209, 28929},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-56422, -14252, 30139}
    },
    /* Vector 1944: Angle 2.823 rad */
    {
        .input_axis = {11034, -57569, 29309},
        .input_angle_rad = 184977,
        .expected_quaternion = {10411, 10894, -56838, 28936},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-58606, -9703, 27679}
    },
    /* Vector 1945: Angle 2.826 rad */
    {
        .input_axis = {8143, -58049, 29309},
        .input_angle_rad = 185178,
        .expected_quaternion = {10312, 8042, -57326, 28944},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-60317, -4960, 25143}
    },
    /* Vector 1946: Angle 2.829 rad */
    {
        .input_axis = {5232, -58383, 29309},
        .input_angle_rad = 185379,
        .expected_quaternion = {10212, 5168, -57670, 28951},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-61538, -73, 22539}
    },
    /* Vector 1947: Angle 2.832 rad */
    {
        .input_axis = {2307, -58572, 29309},
        .input_angle_rad = 185580,
        .expected_quaternion = {10113, 2280, -57870, 28958},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-62256, 4911, 19875}
    },
    /* Vector 1948: Angle 2.835 rad */
    {
        .input_axis = {-623, -58614, 29309},
        .input_angle_rad = 185781,
        .expected_quaternion = {10014, -616, -57926, 28964},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-62464, 9940, 17158}
    },
    /* Vector 1949: Angle 2.838 rad */
    {
        .input_axis = {-3552, -58509, 29309},
        .input_angle_rad = 185982,
        .expected_quaternion = {9914, -3511, -57836, 28971},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-62160, 14962, 14395}
    },
    /* Vector 1950: Angle 2.841 rad */
    {
        .input_axis = {-6471, -58259, 29309},
        .input_angle_rad = 186183,
        .expected_quaternion = {9815, -6398, -57602, 28978},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-61347, 19927, 11595}
    },
    /* Vector 1951: Angle 2.844 rad */
    {
        .input_axis = {-9375, -57863, 29309},
        .input_angle_rad = 186384,
        .expected_quaternion = {9716, -9272, -57223, 28985},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-60032, 24785, 8765}
    },
    /* Vector 1952: Angle 2.847 rad */
    {
        .input_axis = {-12255, -57322, 29309},
        .input_angle_rad = 186585,
        .expected_quaternion = {9616, -12123, -56701, 28991},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-58229, 29485, 5914}
    },
    /* Vector 1953: Angle 2.850 rad */
    {
        .input_axis = {-15105, -56638, 29309},
        .input_angle_rad = 186787,
        .expected_quaternion = {9517, -14945, -56037, 28998},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-55956, 33979, 3049}
    },
    /* Vector 1954: Angle 2.853 rad */
    {
        .input_axis = {-17917, -55812, 29309},
        .input_angle_rad = 186988,
        .expected_quaternion = {9417, -17731, -55233, 29004},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53235, 38222, 179}
    },
    /* Vector 1955: Angle 2.856 rad */
    {
        .input_axis = {-20684, -54847, 29309},
        .input_angle_rad = 187189,
        .expected_quaternion = {9318, -20474, -54289, 29011},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-50094, 42170, -2689}
    },
    /* Vector 1956: Angle 2.859 rad */
    {
        .input_axis = {-23399, -53744, 29309},
        .input_angle_rad = 187390,
        .expected_quaternion = {9218, -23166, -53210, 29017},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-46564, 45782, -5546}
    },
    /* Vector 1957: Angle 2.862 rad */
    {
        .input_axis = {-26056, -52508, 29309},
        .input_angle_rad = 187591,
        .expected_quaternion = {9119, -25803, -51997, 29024},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-42681, 49021, -8384}
    },
    /* Vector 1958: Angle 2.865 rad */
    {
        .input_axis = {-28648, -51140, 29309},
        .input_angle_rad = 187792,
        .expected_quaternion = {9019, -28375, -50653, 29030},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-38482, 51853, -11196}
    },
    /* Vector 1959: Angle 2.869 rad */
    {
        .input_axis = {-31168, -49644, 29309},
        .input_angle_rad = 187993,
        .expected_quaternion = {8919, -30878, -49182, 29036},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-34011, 54249, -13973}
    },
    /* Vector 1960: Angle 2.872 rad */
    {
        .input_axis = {-33610, -48024, 29309},
        .input_angle_rad = 188194,
        .expected_quaternion = {8820, -33304, -47587, 29042},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-29313, 56183, -16709}
    },
    /* Vector 1961: Angle 2.875 rad */
    {
        .input_axis = {-35968, -46285, 29309},
        .input_angle_rad = 188395,
        .expected_quaternion = {8720, -35648, -45873, 29048},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-24433, 57636, -19394}
    },
    /* Vector 1962: Angle 2.878 rad */
    {
        .input_axis = {-38237, -44429, 29309},
        .input_angle_rad = 188596,
        .expected_quaternion = {8621, -37904, -44043, 29054},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19422, 58590, -22021}
    },
    /* Vector 1963: Angle 2.881 rad */
    {
        .input_axis = {-40409, -42462, 29309},
        .input_angle_rad = 188797,
        .expected_quaternion = {8521, -40066, -42102, 29060},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14330, 59036, -24584}
    },
    /* Vector 1964: Angle 2.884 rad */
    {
        .input_axis = {-42481, -40390, 29309},
        .input_angle_rad = 188998,
        .expected_quaternion = {8421, -42129, -40055, 29066},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-9208, 58967, -27075}
    },
    /* Vector 1965: Angle 2.887 rad */
    {
        .input_axis = {-44447, -38216, 29309},
        .input_angle_rad = 189199,
        .expected_quaternion = {8322, -44087, -37907, 29071},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4107, 58383, -29487}
    },
    /* Vector 1966: Angle 2.890 rad */
    {
        .input_axis = {-46301, -35947, 29309},
        .input_angle_rad = 189400,
        .expected_quaternion = {8222, -45935, -35663, 29077},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {920, 57289, -31813}
    },
    /* Vector 1967: Angle 2.893 rad */
    {
        .input_axis = {-48040, -33588, 29309},
        .input_angle_rad = 189601,
        .expected_quaternion = {8122, -47669, -33329, 29083},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5825, 55694, -34047}
    },
    /* Vector 1968: Angle 2.896 rad */
    {
        .input_axis = {-49658, -31145, 29309},
        .input_angle_rad = 189802,
        .expected_quaternion = {8022, -49285, -30911, 29088},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10556, 53613, -36183}
    },
    /* Vector 1969: Angle 2.899 rad */
    {
        .input_axis = {-51153, -28624, 29309},
        .input_angle_rad = 190004,
        .expected_quaternion = {7923, -50778, -28414, 29094},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15066, 51065, -38214}
    },
    /* Vector 1970: Angle 2.902 rad */
    {
        .input_axis = {-52520, -26032, 29309},
        .input_angle_rad = 190205,
        .expected_quaternion = {7823, -52144, -25846, 29099},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19309, 48075, -40136}
    },
    /* Vector 1971: Angle 2.905 rad */
    {
        .input_axis = {-53755, -23374, 29309},
        .input_angle_rad = 190406,
        .expected_quaternion = {7723, -53381, -23212, 29104},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23243, 44672, -41942}
    },
    /* Vector 1972: Angle 2.908 rad */
    {
        .input_axis = {-54856, -20659, 29309},
        .input_angle_rad = 190607,
        .expected_quaternion = {7623, -54484, -20518, 29110},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26828, 40888, -43628}
    },
    /* Vector 1973: Angle 2.911 rad */
    {
        .input_axis = {-55820, -17891, 29309},
        .input_angle_rad = 190808,
        .expected_quaternion = {7523, -55451, -17773, 29115},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30027, 36760, -45189}
    },
    /* Vector 1974: Angle 2.915 rad */
    {
        .input_axis = {-56645, -15079, 29309},
        .input_angle_rad = 191009,
        .expected_quaternion = {7423, -56280, -14982, 29120},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32808, 32329, -46620}
    },
    /* Vector 1975: Angle 2.918 rad */
    {
        .input_axis = {-57327, -12229, 29309},
        .input_angle_rad = 191210,
        .expected_quaternion = {7323, -56968, -12152, 29125},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35142, 27637, -47919}
    },
    /* Vector 1976: Angle 2.921 rad */
    {
        .input_axis = {-57867, -9349, 29309},
        .input_angle_rad = 191411,
        .expected_quaternion = {7224, -57514, -9292, 29130},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37005, 22730, -49081}
    },
    /* Vector 1977: Angle 2.924 rad */
    {
        .input_axis = {-58262, -6445, 29309},
        .input_angle_rad = 191612,
        .expected_quaternion = {7124, -57917, -6407, 29135},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38379, 17657, -50103}
    },
    /* Vector 1978: Angle 2.927 rad */
    {
        .input_axis = {-58511, -3525, 29309},
        .input_angle_rad = 191813,
        .expected_quaternion = {7024, -58174, -3504, 29140},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39248, 12468, -50982}
    },
    /* Vector 1979: Angle 2.930 rad */
    {
        .input_axis = {-58614, -596, 29309},
        .input_angle_rad = 192014,
        .expected_quaternion = {6924, -58286, -593, 29145},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39603, 7212, -51716}
    },
    /* Vector 1980: Angle 2.933 rad */
    {
        .input_axis = {-58571, 2334, 29309},
        .input_angle_rad = 192215,
        .expected_quaternion = {6824, -58252, 2322, 29149},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39441, 1943, -52303}
    },
    /* Vector 1981: Angle 2.936 rad */
    {
        .input_axis = {-58381, 5259, 29309},
        .input_angle_rad = 192416,
        .expected_quaternion = {6724, -58073, 5231, 29154},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38763, -3288, -52741}
    },
    /* Vector 1982: Angle 2.939 rad */
    {
        .input_axis = {-58045, 8170, 29309},
        .input_angle_rad = 192617,
        .expected_quaternion = {6624, -57748, 8128, 29159},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37573, -8430, -53030}
    },
    /* Vector 1983: Angle 2.942 rad */
    {
        .input_axis = {-57564, 11061, 29309},
        .input_angle_rad = 192818,
        .expected_quaternion = {6524, -57278, 11006, 29163},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35885, -13432, -53168}
    },
    /* Vector 1984: Angle 2.945 rad */
    {
        .input_axis = {-56939, 13924, 29309},
        .input_angle_rad = 193019,
        .expected_quaternion = {6424, -56665, 13857, 29167},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {33714, -18245, -53155}
    },
    /* Vector 1985: Angle 2.948 rad */
    {
        .input_axis = {-56172, 16752, 29309},
        .input_angle_rad = 193221,
        .expected_quaternion = {6324, -55910, 16674, 29172},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {31081, -22820, -52992}
    },
    /* Vector 1986: Angle 2.951 rad */
    {
        .input_axis = {-55265, 19539, 29309},
        .input_angle_rad = 193422,
        .expected_quaternion = {6224, -55015, 19450, 29176},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {28013, -27115, -52679}
    },
    /* Vector 1987: Angle 2.954 rad */
    {
        .input_axis = {-54219, 22276, 29309},
        .input_angle_rad = 193623,
        .expected_quaternion = {6123, -53982, 22179, 29180},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {24539, -31085, -52216}
    },
    /* Vector 1988: Angle 2.958 rad */
    {
        .input_axis = {-53038, 24958, 29309},
        .input_angle_rad = 193824,
        .expected_quaternion = {6023, -52814, 24853, 29185},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {20694, -34692, -51606}
    },
    /* Vector 1989: Angle 2.961 rad */
    {
        .input_axis = {-51725, 27578, 29309},
        .input_angle_rad = 194025,
        .expected_quaternion = {5923, -51513, 27465, 29189},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {16515, -37900, -50851}
    },
    /* Vector 1990: Angle 2.964 rad */
    {
        .input_axis = {-50282, 30129, 29309},
        .input_angle_rad = 194226,
        .expected_quaternion = {5823, -50083, 30010, 29193},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {12045, -40679, -49951}
    },
    /* Vector 1991: Angle 2.967 rad */
    {
        .input_axis = {-48713, 32604, 29309},
        .input_angle_rad = 194427,
        .expected_quaternion = {5723, -48527, 32480, 29197},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {7328, -43000, -48910}
    },
    /* Vector 1992: Angle 2.970 rad */
    {
        .input_axis = {-47022, 34998, 29309},
        .input_angle_rad = 194628,
        .expected_quaternion = {5623, -46849, 34869, 29201},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {2410, -44842, -47732}
    },
    /* Vector 1993: Angle 2.973 rad */
    {
        .input_axis = {-45215, 37304, 29309},
        .input_angle_rad = 194829,
        .expected_quaternion = {5523, -45054, 37172, 29204},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-2659, -46186, -46419}
    },
    /* Vector 1994: Angle 2.976 rad */
    {
        .input_axis = {-43294, 39518, 29309},
        .input_angle_rad = 195030,
        .expected_quaternion = {5422, -43145, 39382, 29208},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-7830, -47020, -44975}
    },
    /* Vector 1995: Angle 2.979 rad */
    {
        .input_axis = {-41264, 41632, 29309},
        .input_angle_rad = 195231,
        .expected_quaternion = {5322, -41128, 41494, 29212},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-13050, -47336, -43404}
    },
    /* Vector 1996: Angle 2.982 rad */
    {
        .input_axis = {-39132, 43642, 29309},
        .input_angle_rad = 195432,
        .expected_quaternion = {5222, -39008, 43504, 29215},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-18268, -47132, -41712}
    },
    /* Vector 1997: Angle 2.985 rad */
    {
        .input_axis = {-36902, 45544, 29309},
        .input_angle_rad = 195633,
        .expected_quaternion = {5122, -36789, 45404, 29219},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-23432, -46409, -39902}
    },
    /* Vector 1998: Angle 2.988 rad */
    {
        .input_axis = {-34580, 47331, 29309},
        .input_angle_rad = 195834,
        .expected_quaternion = {5022, -34478, 47192, 29222},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-28489, -45176, -37979}
    },
    /* Vector 1999: Angle 2.991 rad */
    {
        .input_axis = {-32171, 49000, 29309},
        .input_angle_rad = 196035,
        .expected_quaternion = {4921, -32080, 48862, 29226},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-33390, -43447, -35951}
    },
    /* Vector 2000: Angle 2.994 rad */
    {
        .input_axis = {-29682, 50547, 29309},
        .input_angle_rad = 196236,
        .expected_quaternion = {4821, -29601, 50410, 29229},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-38086, -41238, -33821}
    },
    /* Vector 2001: Angle 2.997 rad */
    {
        .input_axis = {-27118, 51967, 29309},
        .input_angle_rad = 196438,
        .expected_quaternion = {4721, -27048, 51832, 29232},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-42530, -38573, -31597}
    },
    /* Vector 2002: Angle 3.000 rad */
    {
        .input_axis = {-24487, 53257, 29309},
        .input_angle_rad = 196639,
        .expected_quaternion = {4621, -24426, 53125, 29236},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-46676, -35478, -29284}
    },
    /* Vector 2003: Angle 3.004 rad */
    {
        .input_axis = {-21795, 54415, 29309},
        .input_angle_rad = 196840,
        .expected_quaternion = {4520, -21743, 54285, 29239},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-50485, -31987, -26890}
    },
    /* Vector 2004: Angle 3.007 rad */
    {
        .input_axis = {-19048, 55436, 29309},
        .input_angle_rad = 197041,
        .expected_quaternion = {4420, -19005, 55310, 29242},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-53918, -28134, -24420}
    },
    /* Vector 2005: Angle 3.010 rad */
    {
        .input_axis = {-16254, 56319, 29309},
        .input_angle_rad = 197242,
        .expected_quaternion = {4320, -16218, 56196, 29245},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-56940, -23959, -21883}
    },
    /* Vector 2006: Angle 3.013 rad */
    {
        .input_axis = {-13418, 57061, 29309},
        .input_angle_rad = 197443,
        .expected_quaternion = {4219, -13391, 56942, 29248},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-59521, -19503, -19284}
    },
    /* Vector 2007: Angle 3.016 rad */
    {
        .input_axis = {-10550, 57660, 29309},
        .input_angle_rad = 197644,
        .expected_quaternion = {4119, -10529, 57546, 29251},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-61635, -14814, -16633}
    },
    /* Vector 2008: Angle 3.019 rad */
    {
        .input_axis = {-7655, 58115, 29309},
        .input_angle_rad = 197845,
        .expected_quaternion = {4019, -7640, 58006, 29253},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-63262, -9937, -13935}
    },
    /* Vector 2009: Angle 3.022 rad */
    {
        .input_axis = {-4741, 58425, 29309},
        .input_angle_rad = 198046,
        .expected_quaternion = {3918, -4732, 58321, 29256},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-64384, -4924, -11199}
    },
    /* Vector 2010: Angle 3.025 rad */
    {
        .input_axis = {-1815, 58589, 29309},
        .input_angle_rad = 198247,
        .expected_quaternion = {3818, -1812, 58490, 29259},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-64991, 175, -8433}
    },
    /* Vector 2011: Angle 3.028 rad */
    {
        .input_axis = {1116, 58607, 29309},
        .input_angle_rad = 198448,
        .expected_quaternion = {3718, 1114, 58512, 29261},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-65076, 5309, -5644}
    },
    /* Vector 2012: Angle 3.031 rad */
    {
        .input_axis = {4043, 58478, 29309},
        .input_angle_rad = 198649,
        .expected_quaternion = {3617, 4037, 58388, 29264},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-64639, 10424, -2840}
    },
    /* Vector 2013: Angle 3.034 rad */
    {
        .input_axis = {6961, 58202, 29309},
        .input_angle_rad = 198850,
        .expected_quaternion = {3517, 6951, 58119, 29266},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-63684, 15470, -29}
    },
    /* Vector 2014: Angle 3.037 rad */
    {
        .input_axis = {9861, 57782, 29309},
        .input_angle_rad = 199051,
        .expected_quaternion = {3417, 9848, 57703, 29269},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-62220, 20393, 2780}
    },
    /* Vector 2015: Angle 3.040 rad */
    {
        .input_axis = {12737, 57217, 29309},
        .input_angle_rad = 199252,
        .expected_quaternion = {3316, 12720, 57143, 29271},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-60262, 25145, 5580}
    },
    /* Vector 2016: Angle 3.043 rad */
    {
        .input_axis = {15581, 56509, 29309},
        .input_angle_rad = 199453,
        .expected_quaternion = {3216, 15562, 56441, 29273},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-57830, 29677, 8363}
    },
    /* Vector 2017: Angle 3.046 rad */
    {
        .input_axis = {18385, 55659, 29309},
        .input_angle_rad = 199654,
        .expected_quaternion = {3115, 18365, 55596, 29275},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-54948, 33942, 11122}
    },
    /* Vector 2018: Angle 3.050 rad */
    {
        .input_axis = {21144, 54671, 29309},
        .input_angle_rad = 199856,
        .expected_quaternion = {3015, 21122, 54613, 29278},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-51644, 37896, 13847}
    },
    /* Vector 2019: Angle 3.053 rad */
    {
        .input_axis = {23850, 53546, 29309},
        .input_angle_rad = 200057,
        .expected_quaternion = {2914, 23827, 53493, 29280},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-47952, 41500, 16532}
    },
    /* Vector 2020: Angle 3.056 rad */
    {
        .input_axis = {26496, 52287, 29309},
        .input_angle_rad = 200258,
        .expected_quaternion = {2814, 26472, 52239, 29282},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-43909, 44716, 19169}
    },
    /* Vector 2021: Angle 3.059 rad */
    {
        .input_axis = {29077, 50897, 29309},
        .input_angle_rad = 200459,
        .expected_quaternion = {2714, 29052, 50854, 29283},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-39554, 47511, 21751}
    },
    /* Vector 2022: Angle 3.062 rad */
    {
        .input_axis = {31584, 49380, 29309},
        .input_angle_rad = 200660,
        .expected_quaternion = {2613, 31559, 49341, 29285},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-34933, 49856, 24270}
    },
    /* Vector 2023: Angle 3.065 rad */
    {
        .input_axis = {34013, 47740, 29309},
        .input_angle_rad = 200861,
        .expected_quaternion = {2513, 33988, 47705, 29287},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-30091, 51726, 26719}
    },
    /* Vector 2024: Angle 3.068 rad */
    {
        .input_axis = {36356, 45981, 29309},
        .input_angle_rad = 201062,
        .expected_quaternion = {2412, 36331, 45949, 29289},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-25076, 53102, 29091}
    },
    /* Vector 2025: Angle 3.071 rad */
    {
        .input_axis = {38609, 44106, 29309},
        .input_angle_rad = 201263,
        .expected_quaternion = {2312, 38585, 44079, 29290},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-19939, 53969, 31380}
    },
    /* Vector 2026: Angle 3.074 rad */
    {
        .input_axis = {40765, 42121, 29309},
        .input_angle_rad = 201464,
        .expected_quaternion = {2211, 40742, 42097, 29292},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-14731, 54318, 33579}
    },
    /* Vector 2027: Angle 3.077 rad */
    {
        .input_axis = {42819, 40031, 29309},
        .input_angle_rad = 201665,
        .expected_quaternion = {2111, 42797, 40010, 29293},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-9505, 54143, 35682}
    },
    /* Vector 2028: Angle 3.080 rad */
    {
        .input_axis = {44766, 37841, 29309},
        .input_angle_rad = 201866,
        .expected_quaternion = {2010, 44745, 37823, 29295},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {-4312, 53446, 37682}
    },
    /* Vector 2029: Angle 3.083 rad */
    {
        .input_axis = {46602, 35556, 29309},
        .input_angle_rad = 202067,
        .expected_quaternion = {1910, 46582, 35541, 29296},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {794, 52232, 39575}
    },
    /* Vector 2030: Angle 3.086 rad */
    {
        .input_axis = {48320, 33183, 29309},
        .input_angle_rad = 202268,
        .expected_quaternion = {1809, 48302, 33170, 29297},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {5764, 50513, 41355}
    },
    /* Vector 2031: Angle 3.089 rad */
    {
        .input_axis = {49919, 30726, 29309},
        .input_angle_rad = 202469,
        .expected_quaternion = {1709, 49902, 30716, 29299},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {10547, 48304, 43016}
    },
    /* Vector 2032: Angle 3.093 rad */
    {
        .input_axis = {51392, 28193, 29309},
        .input_angle_rad = 202670,
        .expected_quaternion = {1608, 51376, 28185, 29300},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {15095, 45628, 44555}
    },
    /* Vector 2033: Angle 3.096 rad */
    {
        .input_axis = {52737, 25589, 29309},
        .input_angle_rad = 202871,
        .expected_quaternion = {1508, 52723, 25583, 29301},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {19363, 42510, 45967}
    },
    /* Vector 2034: Angle 3.099 rad */
    {
        .input_axis = {53950, 22922, 29309},
        .input_angle_rad = 203073,
        .expected_quaternion = {1407, 53937, 22916, 29302},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {23307, 38980, 47248}
    },
    /* Vector 2035: Angle 3.102 rad */
    {
        .input_axis = {55028, 20197, 29309},
        .input_angle_rad = 203274,
        .expected_quaternion = {1307, 55017, 20193, 29303},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {26889, 35072, 48394}
    },
    /* Vector 2036: Angle 3.105 rad */
    {
        .input_axis = {55969, 17421, 29309},
        .input_angle_rad = 203475,
        .expected_quaternion = {1206, 55959, 17418, 29304},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {30072, 30824, 49402}
    },
    /* Vector 2037: Angle 3.108 rad */
    {
        .input_axis = {56769, 14602, 29309},
        .input_angle_rad = 203676,
        .expected_quaternion = {1106, 56761, 14600, 29304},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32824, 26279, 50269}
    },
    /* Vector 2038: Angle 3.111 rad */
    {
        .input_axis = {57428, 11747, 29309},
        .input_angle_rad = 203877,
        .expected_quaternion = {1005, 57421, 11745, 29305},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35118, 21481, 50993}
    },
    /* Vector 2039: Angle 3.114 rad */
    {
        .input_axis = {57943, 8862, 29309},
        .input_angle_rad = 204078,
        .expected_quaternion = {905, 57938, 8861, 29306},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {36931, 16476, 51572}
    },
    /* Vector 2040: Angle 3.117 rad */
    {
        .input_axis = {58314, 5955, 29309},
        .input_angle_rad = 204279,
        .expected_quaternion = {804, 58310, 5954, 29306},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38244, 11315, 52004}
    },
    /* Vector 2041: Angle 3.120 rad */
    {
        .input_axis = {58539, 3033, 29309},
        .input_angle_rad = 204480,
        .expected_quaternion = {704, 58535, 3033, 29307},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39044, 6047, 52287}
    },
    /* Vector 2042: Angle 3.123 rad */
    {
        .input_axis = {58617, 103, 29309},
        .input_angle_rad = 204681,
        .expected_quaternion = {603, 58615, 103, 29307},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39323, 724, 52422}
    },
    /* Vector 2043: Angle 3.126 rad */
    {
        .input_axis = {58549, -2827, 29309},
        .input_angle_rad = 204882,
        .expected_quaternion = {503, 58547, -2826, 29308},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {39079, -4600, 52408}
    },
    /* Vector 2044: Angle 3.129 rad */
    {
        .input_axis = {58335, -5749, 29309},
        .input_angle_rad = 205083,
        .expected_quaternion = {402, 58333, -5749, 29308},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {38314, -9875, 52245}
    },
    /* Vector 2045: Angle 3.132 rad */
    {
        .input_axis = {57974, -8658, 29309},
        .input_angle_rad = 205284,
        .expected_quaternion = {302, 57974, -8657, 29308},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {37035, -15047, 51932}
    },
    /* Vector 2046: Angle 3.135 rad */
    {
        .input_axis = {57469, -11544, 29309},
        .input_angle_rad = 205485,
        .expected_quaternion = {201, 57469, -11544, 29308},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {35255, -20066, 51472}
    },
    /* Vector 2047: Angle 3.139 rad */
    {
        .input_axis = {56820, -14402, 29309},
        .input_angle_rad = 205686,
        .expected_quaternion = {101, 56820, -14402, 29309},
        .test_vector = {65536, 0, 0},
        .rotated_vector = {32992, -24884, 50866}
    }
};

bool verify_extensive_kinematic_vectors(void) {
    for (int i = 0; i < EXTENSIVE_VECTOR_COUNT; i++) {
        quat_q16_t q = quat_from_axis_angle(g_extensive_vectors[i].input_axis, g_extensive_vectors[i].input_angle_rad);
        vec3_q16_t rot = quat_rotate_vec(q, g_extensive_vectors[i].test_vector);
        vec3_q16_t err = vec3_sub(rot, g_extensive_vectors[i].rotated_vector);
        if (vec3_mag(err) > 128) return false;
    }
    return true;
}
