#ifndef METRONOME_BEEP_SAMPLES_H
#define METRONOME_BEEP_SAMPLES_H


#include <stdint.h>


// 8KHz PCM samples for basic high beep sound
const int16_t high_beep_samples[430] =
{
    0,
    3417,
    11663,
    19605,
    21281,
    12722,
    -4878,
    -20968,
    -30842,
    -31640,
    -23170,
    -7958,
    9510,
    24112,
    31584,
    29774,
    19255,
    3151,
    -13797,
    -26624,
    -31595,
    -27292,
    -15020,
    1588,
    17649,
    28465,
    30895,
    24267,
    10569,
    -6153,
    -20992,
    -29611,
    -29519,
    -20785,
    -6012,
    10446,
    23763,
    30057,
    27520,
    16940,
    1456,
    -14373,
    -25915,
    -29812,
    -24962,
    -12830,
    2992,
    17858,
    27418,
    28903,
    21920,
    8558,
    -7237,
    -20831,
    -28256,
    -27370,
    -18479,
    -4227,
    11185,
    23241,
    28430,
    25270,
    14731,
    -60,
    -14755,
    -25050,
    -27957,
    -22667,
    -10771,
    4209,
    17876,
    26235,
    26870,
    19637,
    6698,
    -8126,
    -20490,
    -26788,
    -25210,
    -16264,
    -2609,
    11729,
    22555,
    26718,
    23038,
    12638,
    -1398,
    -14945,
    -24038,
    -26046,
    -20419,
    -8851,
    5237,
    17710,
    24927,
    24808,
    17429,
    4996,
    -8823,
    -19978,
    -25221,
    -23052,
    -14152,
    -1165,
    12081,
    21713,
    24932,
    20837,
    10671,
    -2552,
    -14947,
    -22892,
    -24089,
    -18232,
    -7077,
    6075,
    17368,
    23508,
    22731,
    15310,
    3457,
    -9328,
    -19303,
    -23567,
    -20909,
    -12152,
    100,
    12244,
    20725,
    23088,
    18682,
    8840,
    -3519,
    -14768,
    -21621,
    -22103,
    -16117,
    -5458,
    6722,
    16855,
    21988,
    20655,
    13289,
    2089,
    -9642,
    -18473,
    -21840,
    -18795,
    -10275,
    1187,
    12222,
    19603,
    21199,
    16584,
    7153,
    -4300,
    -14413,
    -20237,
    -20101,
    -14089,
    -4000,
    7181,
    16180,
    20382,
    18592,
    11380,
    895,
    -9770,
    -17499,
    -20053,
    -16724,
    -8532,
    2090,
    12019,
    18357,
    19279,
    14557,
    5617,
    -4893,
    -13890,
    -18755,
    -18098,
    -12156,
    -2710,
    7451,
    15352,
    18702,
    16557,
    9590,
    -119,
    -9715,
    -16391,
    -18220,
    -14709,
    -6929,
    2809,
    11643,
    17000,
    17342,
    12613,
    4239,
    -5298,
    -13206,
    -17185,
    -16107,
    -10332,
    -1590,
    7536,
    14381,
    16961,
    14563,
    7931,
    -955,
    -9482,
    -15159,
    -16355,
    -12762,
    -5475,
    3342,
    11100,
    15542,
    15401,
    10764,
    3027,
    -5518,
    -12369,
    -15540,
    -14141,
    -8626,
    -646,
    7441,
    13275,
    15174,
    12622,
    6411,
    -1608,
    -9076,
    -13816,
    -14472,
    -10897,
    -4177,
    3690,
    10397,
    13997,
    13472,
    9020,
    1983,
    -5555,
    -11389,
    -13835,
    -12215,
    -7048,
    117,
    7168,
    12047,
    13354,
    10748,
    5037,
    -2077,
    -8503,
    -12372,
    -12585,
    -9123,
    -3041,
    3854,
    9542,
    12377,
    11566,
    7392,
    1113,
    -5412,
    -10277,
    -12082,
    -10341,
    -5606,
    702,
    6724,
    10707,
    11514,
    8954,
    3817,
    -2363,
    -7772,
    -10841,
    -10707,
    -7454,
    -2073,
    3836,
    8545,
    10696,
    9699,
    5890,
    419,
    -5093,
    -9041,
    -10296,
    -8532,
    -4309,
    1105,
    6115,
    9266,
    9670,
    7250,
    2757,
    -2467,
    -6890,
    -9235,
    -8852,
    -5899,
    -1276,
    3640,
    7414,
    8967,
    7882,
    4522,
    -94,
    -4603,
    -7693,
    -8489,
    -6801,
    -3164,
    1325,
    5347,
    7738,
    7833,
    5648,
    1863,
    -2389,
    -5865,
    -7566,
    -7034,
    -4468,
    -655,
    3268,
    6161,
    7203,
    6130,
    3299,
    -428,
    -3949,
    -6245,
    -6676,
    -5159,
    -2178,
    1364,
    4428,
    6134,
    6019,
    4160,
    1140,
    -2134,
    -4707,
    -5849,
    -5266,
    -3171,
    -212,
    2727,
    4796,
    5418,
    4453,
    2226,
    -580,
    -3137,
    -4709,
    -4871,
    -3618,
    -1357,
    1223,
    3367,
    4468,
    4240,
    2794,
    591,
    -1705,
    -3427,
    -4097,
    -3560,
    -2016,
    50,
    2021,
    3330,
    3626,
    2865,
    1311,
    -552,
    -2174,
    -3097,
    -3086,
    -2189,
    -706,
    905,
    2174,
    2752,
    2510,
    1561,
    219,
    -1105,
    -2034,
    -2323,
    -1929,
    -1010,
    133,
    1157,
    1776,
    1841,
    1377,
    560,
    -344,
    -1069,
    -1422,
    -1336,
    -881,
    -228,
    412,
    857,
    1000,
    841,
    469,
    27,
    -342,
    -541,
    -541,
    -386,
    -162,
    35,
    143,
    145,
    76,
    0,
    0
};

// 8KHz PCM samples for basic low beep sound
const int16_t low_beep_samples[430] =
{
    0,
    1775,
    6834,
    14412,
    23327,
    32117,
    32597,
    30643,
    26413,
    20231,
    12571,
    4010,
    -4809,
    -13228,
    -20619,
    -26437,
    -30255,
    -31797,
    -30961,
    -27818,
    -22616,
    -15753,
    -7748,
    793,
    9236,
    16947,
    23356,
    27992,
    30516,
    30752,
    28692,
    24504,
    18508,
    11161,
    3020,
    -5304,
    -13191,
    -20055,
    -25387,
    -28799,
    -30046,
    -29046,
    -25883,
    -20807,
    -14206,
    -6578,
    1498,
    9421,
    16601,
    22505,
    26699,
    28881,
    28896,
    26757,
    22633,
    16844,
    9829,
    2118,
    -5706,
    -13065,
    -19410,
    -24273,
    -27299,
    -28273,
    -27133,
    -23976,
    -19049,
    -12728,
    -5493,
    2110,
    9516,
    16170,
    21581,
    25352,
    27212,
    27033,
    24838,
    20805,
    15241,
    8574,
    1306,
    -6017,
    -12851,
    -18687,
    -23095,
    -25757,
    -26482,
    -25229,
    -22102,
    -17345,
    -11323,
    -4492,
    2632,
    9518,
    15655,
    20586,
    23955,
    25516,
    25166,
    22942,
    19020,
    13705,
    7400,
    582,
    -6236,
    -12548,
    -17887,
    -21859,
    -24176,
    -24677,
    -23335,
    -20262,
    -15697,
    -9992,
    -3577,
    3061,
    9432,
    15058,
    19525,
    22508,
    23795,
    23299,
    21069,
    17284,
    12236,
    6307,
    -49,
    -6363,
    -12160,
    -17013,
    -20564,
    -22560,
    -22860,
    -21455,
    -18460,
    -14109,
    -8737,
    -2749,
    3400,
    9255,
    14380,
    18399,
    21017,
    22052,
    21435,
    19225,
    15599,
    10835,
    5298,
    -593,
    -6399,
    -11687,
    -16066,
    -19216,
    -20912,
    -21036,
    -19592,
    -16699,
    -12582,
    -7559,
    -2009,
    3647,
    8991,
    13625,
    17209,
    19484,
    20290,
    19578,
    17412,
    13966,
    9506,
    4372,
    -1046,
    -6346,
    -11132,
    -15050,
    -17816,
    -19234,
    -19208,
    -17750,
    -14982,
    -11120,
    -6461,
    -1358,
    3803,
    8640,
    12793,
    15960,
    17912,
    18513,
    17730,
    15633,
    12389,
    8250,
    3532,
    -1409,
    -6203,
    -10494,
    -13966,
    -16369,
    -17531,
    -17378,
    -15932,
    -13312,
    -9724,
    -5443,
    -796,
    3868,
    8203,
    11888,
    14654,
    16304,
    16724,
    15896,
    13891,
    10871,
    7070,
    2777,
    -1680,
    -5971,
    -9776,
    -12818,
    -14876,
    -15806,
    -15551,
    -14140,
    -11691,
    -8396,
    -4508,
    -323,
    3843,
    7682,
    10910,
    13294,
    14664,
    14928,
    14079,
    12190,
    9414,
    5966,
    2110,
    -1860,
    -5651,
    -8981,
    -11607,
    -13340,
    -14063,
    -13730,
    -12380,
    -10123,
    -7138,
    -3656,
    58,
    3727,
    7078,
    9863,
    11882,
    12994,
    13127,
    12281,
    10532,
    8019,
    4941,
    1531,
    -1950,
    -5245,
    -8109,
    -10336,
    -11766,
    -12303,
    -11919,
    -10653,
    -8610,
    -5954,
    -2888,
    351,
    3523,
    6392,
    8748,
    10422,
    11299,
    11324,
    10507,
    8920,
    6691,
    3996,
    1040,
    -1949,
    -4753,
    -7164,
    -9008,
    -10156,
    -10532,
    -10120,
    -8962,
    -7155,
    -4844,
    -2207,
    553,
    3231,
    5627,
    7568,
    8917,
    9582,
    9524,
    8760,
    7357,
    5430,
    3132,
    640,
    -1858,
    -4177,
    -6146,
    -7626,
    -8513,
    -8753,
    -8338,
    -7312,
    -5761,
    -3810,
    -1612,
    665,
    2850,
    4784,
    6326,
    7369,
    7845,
    7729,
    7042,
    5846,
    4240,
    2352,
    328,
    -1678,
    -3518,
    -5059,
    -6192,
    -6841,
    -6968,
    -6575,
    -5704,
    -4429,
    -2855,
    -1105,
    686,
    2384,
    3865,
    5024,
    5781,
    6092,
    5943,
    5358,
    4390,
    3122,
    1656,
    106,
    -1409,
    -2779,
    -3904,
    -4709,
    -5142,
    -5181,
    -4836,
    -4142,
    -3162,
    -1979,
    -685,
    617,
    1832,
    2872,
    3664,
    4158,
    4327,
    4170,
    3709,
    2991,
    2078,
    1045,
    -25,
    -1051,
    -1959,
    -2685,
    -3181,
    -3421,
    -3397,
    -3122,
    -2628,
    -1963,
    -1184,
    -355,
    458,
    1196,
    1806,
    2250,
    2502,
    2553,
    2412,
    2100,
    1652,
    1109,
    520,
    -66,
    -606,
    -1061,
    -1402,
    -1610,
    -1680,
    -1617,
    -1438,
    -1166,
    -834,
    -472,
    -114,
    209,
    477,
    671,
    784,
    816,
    774,
    674,
    534,
    375,
    219,
    83,
    -17,
    -75,
    -88,
    -59,
    0,
    0,
};

#endif // METRONOME_BEEP_SAMPLES_H
