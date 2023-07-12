#ifndef METRONOME_BEEP_SAMPLES_H
#define METRONOME_BEEP_SAMPLES_H


#include <stdint.h>

#define BEEP_SAMPLE_COUNT (860)

// 8KHz PCM samples for basic high beep sound
const int16_t high_beep_samples[BEEP_SAMPLE_COUNT] =
{
    0,
    0,
    3417,
    3417,
    11663,
    11663,
    19605,
    19605,
    21281,
    21281,
    12722,
    12722,
    -4878,
    -4878,
    -20968,
    -20968,
    -30842,
    -30842,
    -31640,
    -31640,
    -23170,
    -23170,
    -7958,
    -7958,
    9510,
    9510,
    24112,
    24112,
    31584,
    31584,
    29774,
    29774,
    19255,
    19255,
    3151,
    3151,
    -13797,
    -13797,
    -26624,
    -26624,
    -31595,
    -31595,
    -27292,
    -27292,
    -15020,
    -15020,
    1588,
    1588,
    17649,
    17649,
    28465,
    28465,
    30895,
    30895,
    24267,
    24267,
    10569,
    10569,
    -6153,
    -6153,
    -20992,
    -20992,
    -29611,
    -29611,
    -29519,
    -29519,
    -20785,
    -20785,
    -6012,
    -6012,
    10446,
    10446,
    23763,
    23763,
    30057,
    30057,
    27520,
    27520,
    16940,
    16940,
    1456,
    1456,
    -14373,
    -14373,
    -25915,
    -25915,
    -29812,
    -29812,
    -24962,
    -24962,
    -12830,
    -12830,
    2992,
    2992,
    17858,
    17858,
    27418,
    27418,
    28903,
    28903,
    21920,
    21920,
    8558,
    8558,
    -7237,
    -7237,
    -20831,
    -20831,
    -28256,
    -28256,
    -27370,
    -27370,
    -18479,
    -18479,
    -4227,
    -4227,
    11185,
    11185,
    23241,
    23241,
    28430,
    28430,
    25270,
    25270,
    14731,
    14731,
    -60,
    -60,
    -14755,
    -14755,
    -25050,
    -25050,
    -27957,
    -27957,
    -22667,
    -22667,
    -10771,
    -10771,
    4209,
    4209,
    17876,
    17876,
    26235,
    26235,
    26870,
    26870,
    19637,
    19637,
    6698,
    6698,
    -8126,
    -8126,
    -20490,
    -20490,
    -26788,
    -26788,
    -25210,
    -25210,
    -16264,
    -16264,
    -2609,
    -2609,
    11729,
    11729,
    22555,
    22555,
    26718,
    26718,
    23038,
    23038,
    12638,
    12638,
    -1398,
    -1398,
    -14945,
    -14945,
    -24038,
    -24038,
    -26046,
    -26046,
    -20419,
    -20419,
    -8851,
    -8851,
    5237,
    5237,
    17710,
    17710,
    24927,
    24927,
    24808,
    24808,
    17429,
    17429,
    4996,
    4996,
    -8823,
    -8823,
    -19978,
    -19978,
    -25221,
    -25221,
    -23052,
    -23052,
    -14152,
    -14152,
    -1165,
    -1165,
    12081,
    12081,
    21713,
    21713,
    24932,
    24932,
    20837,
    20837,
    10671,
    10671,
    -2552,
    -2552,
    -14947,
    -14947,
    -22892,
    -22892,
    -24089,
    -24089,
    -18232,
    -18232,
    -7077,
    -7077,
    6075,
    6075,
    17368,
    17368,
    23508,
    23508,
    22731,
    22731,
    15310,
    15310,
    3457,
    3457,
    -9328,
    -9328,
    -19303,
    -19303,
    -23567,
    -23567,
    -20909,
    -20909,
    -12152,
    -12152,
    100,
    100,
    12244,
    12244,
    20725,
    20725,
    23088,
    23088,
    18682,
    18682,
    8840,
    8840,
    -3519,
    -3519,
    -14768,
    -14768,
    -21621,
    -21621,
    -22103,
    -22103,
    -16117,
    -16117,
    -5458,
    -5458,
    6722,
    6722,
    16855,
    16855,
    21988,
    21988,
    20655,
    20655,
    13289,
    13289,
    2089,
    2089,
    -9642,
    -9642,
    -18473,
    -18473,
    -21840,
    -21840,
    -18795,
    -18795,
    -10275,
    -10275,
    1187,
    1187,
    12222,
    12222,
    19603,
    19603,
    21199,
    21199,
    16584,
    16584,
    7153,
    7153,
    -4300,
    -4300,
    -14413,
    -14413,
    -20237,
    -20237,
    -20101,
    -20101,
    -14089,
    -14089,
    -4000,
    -4000,
    7181,
    7181,
    16180,
    16180,
    20382,
    20382,
    18592,
    18592,
    11380,
    11380,
    895,
    895,
    -9770,
    -9770,
    -17499,
    -17499,
    -20053,
    -20053,
    -16724,
    -16724,
    -8532,
    -8532,
    2090,
    2090,
    12019,
    12019,
    18357,
    18357,
    19279,
    19279,
    14557,
    14557,
    5617,
    5617,
    -4893,
    -4893,
    -13890,
    -13890,
    -18755,
    -18755,
    -18098,
    -18098,
    -12156,
    -12156,
    -2710,
    -2710,
    7451,
    7451,
    15352,
    15352,
    18702,
    18702,
    16557,
    16557,
    9590,
    9590,
    -119,
    -119,
    -9715,
    -9715,
    -16391,
    -16391,
    -18220,
    -18220,
    -14709,
    -14709,
    -6929,
    -6929,
    2809,
    2809,
    11643,
    11643,
    17000,
    17000,
    17342,
    17342,
    12613,
    12613,
    4239,
    4239,
    -5298,
    -5298,
    -13206,
    -13206,
    -17185,
    -17185,
    -16107,
    -16107,
    -10332,
    -10332,
    -1590,
    -1590,
    7536,
    7536,
    14381,
    14381,
    16961,
    16961,
    14563,
    14563,
    7931,
    7931,
    -955,
    -955,
    -9482,
    -9482,
    -15159,
    -15159,
    -16355,
    -16355,
    -12762,
    -12762,
    -5475,
    -5475,
    3342,
    3342,
    11100,
    11100,
    15542,
    15542,
    15401,
    15401,
    10764,
    10764,
    3027,
    3027,
    -5518,
    -5518,
    -12369,
    -12369,
    -15540,
    -15540,
    -14141,
    -14141,
    -8626,
    -8626,
    -646,
    -646,
    7441,
    7441,
    13275,
    13275,
    15174,
    15174,
    12622,
    12622,
    6411,
    6411,
    -1608,
    -1608,
    -9076,
    -9076,
    -13816,
    -13816,
    -14472,
    -14472,
    -10897,
    -10897,
    -4177,
    -4177,
    3690,
    3690,
    10397,
    10397,
    13997,
    13997,
    13472,
    13472,
    9020,
    9020,
    1983,
    1983,
    -5555,
    -5555,
    -11389,
    -11389,
    -13835,
    -13835,
    -12215,
    -12215,
    -7048,
    -7048,
    117,
    117,
    7168,
    7168,
    12047,
    12047,
    13354,
    13354,
    10748,
    10748,
    5037,
    5037,
    -2077,
    -2077,
    -8503,
    -8503,
    -12372,
    -12372,
    -12585,
    -12585,
    -9123,
    -9123,
    -3041,
    -3041,
    3854,
    3854,
    9542,
    9542,
    12377,
    12377,
    11566,
    11566,
    7392,
    7392,
    1113,
    1113,
    -5412,
    -5412,
    -10277,
    -10277,
    -12082,
    -12082,
    -10341,
    -10341,
    -5606,
    -5606,
    702,
    702,
    6724,
    6724,
    10707,
    10707,
    11514,
    11514,
    8954,
    8954,
    3817,
    3817,
    -2363,
    -2363,
    -7772,
    -7772,
    -10841,
    -10841,
    -10707,
    -10707,
    -7454,
    -7454,
    -2073,
    -2073,
    3836,
    3836,
    8545,
    8545,
    10696,
    10696,
    9699,
    9699,
    5890,
    5890,
    419,
    419,
    -5093,
    -5093,
    -9041,
    -9041,
    -10296,
    -10296,
    -8532,
    -8532,
    -4309,
    -4309,
    1105,
    1105,
    6115,
    6115,
    9266,
    9266,
    9670,
    9670,
    7250,
    7250,
    2757,
    2757,
    -2467,
    -2467,
    -6890,
    -6890,
    -9235,
    -9235,
    -8852,
    -8852,
    -5899,
    -5899,
    -1276,
    -1276,
    3640,
    3640,
    7414,
    7414,
    8967,
    8967,
    7882,
    7882,
    4522,
    4522,
    -94,
    -94,
    -4603,
    -4603,
    -7693,
    -7693,
    -8489,
    -8489,
    -6801,
    -6801,
    -3164,
    -3164,
    1325,
    1325,
    5347,
    5347,
    7738,
    7738,
    7833,
    7833,
    5648,
    5648,
    1863,
    1863,
    -2389,
    -2389,
    -5865,
    -5865,
    -7566,
    -7566,
    -7034,
    -7034,
    -4468,
    -4468,
    -655,
    -655,
    3268,
    3268,
    6161,
    6161,
    7203,
    7203,
    6130,
    6130,
    3299,
    3299,
    -428,
    -428,
    -3949,
    -3949,
    -6245,
    -6245,
    -6676,
    -6676,
    -5159,
    -5159,
    -2178,
    -2178,
    1364,
    1364,
    4428,
    4428,
    6134,
    6134,
    6019,
    6019,
    4160,
    4160,
    1140,
    1140,
    -2134,
    -2134,
    -4707,
    -4707,
    -5849,
    -5849,
    -5266,
    -5266,
    -3171,
    -3171,
    -212,
    -212,
    2727,
    2727,
    4796,
    4796,
    5418,
    5418,
    4453,
    4453,
    2226,
    2226,
    -580,
    -580,
    -3137,
    -3137,
    -4709,
    -4709,
    -4871,
    -4871,
    -3618,
    -3618,
    -1357,
    -1357,
    1223,
    1223,
    3367,
    3367,
    4468,
    4468,
    4240,
    4240,
    2794,
    2794,
    591,
    591,
    -1705,
    -1705,
    -3427,
    -3427,
    -4097,
    -4097,
    -3560,
    -3560,
    -2016,
    -2016,
    50,
    50,
    2021,
    2021,
    3330,
    3330,
    3626,
    3626,
    2865,
    2865,
    1311,
    1311,
    -552,
    -552,
    -2174,
    -2174,
    -3097,
    -3097,
    -3086,
    -3086,
    -2189,
    -2189,
    -706,
    -706,
    905,
    905,
    2174,
    2174,
    2752,
    2752,
    2510,
    2510,
    1561,
    1561,
    219,
    219,
    -1105,
    -1105,
    -2034,
    -2034,
    -2323,
    -2323,
    -1929,
    -1929,
    -1010,
    -1010,
    133,
    133,
    1157,
    1157,
    1776,
    1776,
    1841,
    1841,
    1377,
    1377,
    560,
    560,
    -344,
    -344,
    -1069,
    -1069,
    -1422,
    -1422,
    -1336,
    -1336,
    -881,
    -881,
    -228,
    -228,
    412,
    412,
    857,
    857,
    1000,
    1000,
    841,
    841,
    469,
    469,
    27,
    27,
    -342,
    -342,
    -541,
    -541,
    -541,
    -541,
    -386,
    -386,
    -162,
    -162,
    35,
    35,
    143,
    143,
    145,
    145,
    76,
    76,
    0,
    0,
    0,
    0
};

// 8KHz PCM samples for basic low beep sound
const int16_t low_beep_samples[BEEP_SAMPLE_COUNT] =
{
    0,
    0,
    1775,
    1775,
    6834,
    6834,
    14412,
    14412,
    23327,
    23327,
    32117,
    32117,
    32597,
    32597,
    30643,
    30643,
    26413,
    26413,
    20231,
    20231,
    12571,
    12571,
    4010,
    4010,
    -4809,
    -4809,
    -13228,
    -13228,
    -20619,
    -20619,
    -26437,
    -26437,
    -30255,
    -30255,
    -31797,
    -31797,
    -30961,
    -30961,
    -27818,
    -27818,
    -22616,
    -22616,
    -15753,
    -15753,
    -7748,
    -7748,
    793,
    793,
    9236,
    9236,
    16947,
    16947,
    23356,
    23356,
    27992,
    27992,
    30516,
    30516,
    30752,
    30752,
    28692,
    28692,
    24504,
    24504,
    18508,
    18508,
    11161,
    11161,
    3020,
    3020,
    -5304,
    -5304,
    -13191,
    -13191,
    -20055,
    -20055,
    -25387,
    -25387,
    -28799,
    -28799,
    -30046,
    -30046,
    -29046,
    -29046,
    -25883,
    -25883,
    -20807,
    -20807,
    -14206,
    -14206,
    -6578,
    -6578,
    1498,
    1498,
    9421,
    9421,
    16601,
    16601,
    22505,
    22505,
    26699,
    26699,
    28881,
    28881,
    28896,
    28896,
    26757,
    26757,
    22633,
    22633,
    16844,
    16844,
    9829,
    9829,
    2118,
    2118,
    -5706,
    -5706,
    -13065,
    -13065,
    -19410,
    -19410,
    -24273,
    -24273,
    -27299,
    -27299,
    -28273,
    -28273,
    -27133,
    -27133,
    -23976,
    -23976,
    -19049,
    -19049,
    -12728,
    -12728,
    -5493,
    -5493,
    2110,
    2110,
    9516,
    9516,
    16170,
    16170,
    21581,
    21581,
    25352,
    25352,
    27212,
    27212,
    27033,
    27033,
    24838,
    24838,
    20805,
    20805,
    15241,
    15241,
    8574,
    8574,
    1306,
    1306,
    -6017,
    -6017,
    -12851,
    -12851,
    -18687,
    -18687,
    -23095,
    -23095,
    -25757,
    -25757,
    -26482,
    -26482,
    -25229,
    -25229,
    -22102,
    -22102,
    -17345,
    -17345,
    -11323,
    -11323,
    -4492,
    -4492,
    2632,
    2632,
    9518,
    9518,
    15655,
    15655,
    20586,
    20586,
    23955,
    23955,
    25516,
    25516,
    25166,
    25166,
    22942,
    22942,
    19020,
    19020,
    13705,
    13705,
    7400,
    7400,
    582,
    582,
    -6236,
    -6236,
    -12548,
    -12548,
    -17887,
    -17887,
    -21859,
    -21859,
    -24176,
    -24176,
    -24677,
    -24677,
    -23335,
    -23335,
    -20262,
    -20262,
    -15697,
    -15697,
    -9992,
    -9992,
    -3577,
    -3577,
    3061,
    3061,
    9432,
    9432,
    15058,
    15058,
    19525,
    19525,
    22508,
    22508,
    23795,
    23795,
    23299,
    23299,
    21069,
    21069,
    17284,
    17284,
    12236,
    12236,
    6307,
    6307,
    -49,
    -49,
    -6363,
    -6363,
    -12160,
    -12160,
    -17013,
    -17013,
    -20564,
    -20564,
    -22560,
    -22560,
    -22860,
    -22860,
    -21455,
    -21455,
    -18460,
    -18460,
    -14109,
    -14109,
    -8737,
    -8737,
    -2749,
    -2749,
    3400,
    3400,
    9255,
    9255,
    14380,
    14380,
    18399,
    18399,
    21017,
    21017,
    22052,
    22052,
    21435,
    21435,
    19225,
    19225,
    15599,
    15599,
    10835,
    10835,
    5298,
    5298,
    -593,
    -593,
    -6399,
    -6399,
    -11687,
    -11687,
    -16066,
    -16066,
    -19216,
    -19216,
    -20912,
    -20912,
    -21036,
    -21036,
    -19592,
    -19592,
    -16699,
    -16699,
    -12582,
    -12582,
    -7559,
    -7559,
    -2009,
    -2009,
    3647,
    3647,
    8991,
    8991,
    13625,
    13625,
    17209,
    17209,
    19484,
    19484,
    20290,
    20290,
    19578,
    19578,
    17412,
    17412,
    13966,
    13966,
    9506,
    9506,
    4372,
    4372,
    -1046,
    -1046,
    -6346,
    -6346,
    -11132,
    -11132,
    -15050,
    -15050,
    -17816,
    -17816,
    -19234,
    -19234,
    -19208,
    -19208,
    -17750,
    -17750,
    -14982,
    -14982,
    -11120,
    -11120,
    -6461,
    -6461,
    -1358,
    -1358,
    3803,
    3803,
    8640,
    8640,
    12793,
    12793,
    15960,
    15960,
    17912,
    17912,
    18513,
    18513,
    17730,
    17730,
    15633,
    15633,
    12389,
    12389,
    8250,
    8250,
    3532,
    3532,
    -1409,
    -1409,
    -6203,
    -6203,
    -10494,
    -10494,
    -13966,
    -13966,
    -16369,
    -16369,
    -17531,
    -17531,
    -17378,
    -17378,
    -15932,
    -15932,
    -13312,
    -13312,
    -9724,
    -9724,
    -5443,
    -5443,
    -796,
    -796,
    3868,
    3868,
    8203,
    8203,
    11888,
    11888,
    14654,
    14654,
    16304,
    16304,
    16724,
    16724,
    15896,
    15896,
    13891,
    13891,
    10871,
    10871,
    7070,
    7070,
    2777,
    2777,
    -1680,
    -1680,
    -5971,
    -5971,
    -9776,
    -9776,
    -12818,
    -12818,
    -14876,
    -14876,
    -15806,
    -15806,
    -15551,
    -15551,
    -14140,
    -14140,
    -11691,
    -11691,
    -8396,
    -8396,
    -4508,
    -4508,
    -323,
    -323,
    3843,
    3843,
    7682,
    7682,
    10910,
    10910,
    13294,
    13294,
    14664,
    14664,
    14928,
    14928,
    14079,
    14079,
    12190,
    12190,
    9414,
    9414,
    5966,
    5966,
    2110,
    2110,
    -1860,
    -1860,
    -5651,
    -5651,
    -8981,
    -8981,
    -11607,
    -11607,
    -13340,
    -13340,
    -14063,
    -14063,
    -13730,
    -13730,
    -12380,
    -12380,
    -10123,
    -10123,
    -7138,
    -7138,
    -3656,
    -3656,
    58,
    58,
    3727,
    3727,
    7078,
    7078,
    9863,
    9863,
    11882,
    11882,
    12994,
    12994,
    13127,
    13127,
    12281,
    12281,
    10532,
    10532,
    8019,
    8019,
    4941,
    4941,
    1531,
    1531,
    -1950,
    -1950,
    -5245,
    -5245,
    -8109,
    -8109,
    -10336,
    -10336,
    -11766,
    -11766,
    -12303,
    -12303,
    -11919,
    -11919,
    -10653,
    -10653,
    -8610,
    -8610,
    -5954,
    -5954,
    -2888,
    -2888,
    351,
    351,
    3523,
    3523,
    6392,
    6392,
    8748,
    8748,
    10422,
    10422,
    11299,
    11299,
    11324,
    11324,
    10507,
    10507,
    8920,
    8920,
    6691,
    6691,
    3996,
    3996,
    1040,
    1040,
    -1949,
    -1949,
    -4753,
    -4753,
    -7164,
    -7164,
    -9008,
    -9008,
    -10156,
    -10156,
    -10532,
    -10532,
    -10120,
    -10120,
    -8962,
    -8962,
    -7155,
    -7155,
    -4844,
    -4844,
    -2207,
    -2207,
    553,
    553,
    3231,
    3231,
    5627,
    5627,
    7568,
    7568,
    8917,
    8917,
    9582,
    9582,
    9524,
    9524,
    8760,
    8760,
    7357,
    7357,
    5430,
    5430,
    3132,
    3132,
    640,
    640,
    -1858,
    -1858,
    -4177,
    -4177,
    -6146,
    -6146,
    -7626,
    -7626,
    -8513,
    -8513,
    -8753,
    -8753,
    -8338,
    -8338,
    -7312,
    -7312,
    -5761,
    -5761,
    -3810,
    -3810,
    -1612,
    -1612,
    665,
    665,
    2850,
    2850,
    4784,
    4784,
    6326,
    6326,
    7369,
    7369,
    7845,
    7845,
    7729,
    7729,
    7042,
    7042,
    5846,
    5846,
    4240,
    4240,
    2352,
    2352,
    328,
    328,
    -1678,
    -1678,
    -3518,
    -3518,
    -5059,
    -5059,
    -6192,
    -6192,
    -6841,
    -6841,
    -6968,
    -6968,
    -6575,
    -6575,
    -5704,
    -5704,
    -4429,
    -4429,
    -2855,
    -2855,
    -1105,
    -1105,
    686,
    686,
    2384,
    2384,
    3865,
    3865,
    5024,
    5024,
    5781,
    5781,
    6092,
    6092,
    5943,
    5943,
    5358,
    5358,
    4390,
    4390,
    3122,
    3122,
    1656,
    1656,
    106,
    106,
    -1409,
    -1409,
    -2779,
    -2779,
    -3904,
    -3904,
    -4709,
    -4709,
    -5142,
    -5142,
    -5181,
    -5181,
    -4836,
    -4836,
    -4142,
    -4142,
    -3162,
    -3162,
    -1979,
    -1979,
    -685,
    -685,
    617,
    617,
    1832,
    1832,
    2872,
    2872,
    3664,
    3664,
    4158,
    4158,
    4327,
    4327,
    4170,
    4170,
    3709,
    3709,
    2991,
    2991,
    2078,
    2078,
    1045,
    1045,
    -25,
    -25,
    -1051,
    -1051,
    -1959,
    -1959,
    -2685,
    -2685,
    -3181,
    -3181,
    -3421,
    -3421,
    -3397,
    -3397,
    -3122,
    -3122,
    -2628,
    -2628,
    -1963,
    -1963,
    -1184,
    -1184,
    -355,
    -355,
    458,
    458,
    1196,
    1196,
    1806,
    1806,
    2250,
    2250,
    2502,
    2502,
    2553,
    2553,
    2412,
    2412,
    2100,
    2100,
    1652,
    1652,
    1109,
    1109,
    520,
    520,
    -66,
    -66,
    -606,
    -606,
    -1061,
    -1061,
    -1402,
    -1402,
    -1610,
    -1610,
    -1680,
    -1680,
    -1617,
    -1617,
    -1438,
    -1438,
    -1166,
    -1166,
    -834,
    -834,
    -472,
    -472,
    -114,
    -114,
    209,
    209,
    477,
    477,
    671,
    671,
    784,
    784,
    816,
    816,
    774,
    774,
    674,
    674,
    534,
    534,
    375,
    375,
    219,
    219,
    83,
    83,
    -17,
    -17,
    -75,
    -75,
    -88,
    -88,
    -59,
    -59,
    0,
    0,
    0,
    0
};

#endif // METRONOME_BEEP_SAMPLES_H
