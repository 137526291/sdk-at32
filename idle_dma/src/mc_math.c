
// 1 myself h
#include "mc_math.h"

// 2 system etc... <stdint.h>  <string.h>
#include <math.h>

// 3 external module.

// 256
const int16_t sin_lut[] = {
    0,
    807,
    1614,
    2419,
    3224,
    4026,
    4826,
    5623,
    6417,
    7206,
    7992,
    8772,
    9547,
    10317,
    11080,
    11836,
    12585,
    13327,
    14060,
    14785,
    15501,
    16208,
    16905,
    17591,
    18267,
    18931,
    19585,
    20226,
    20855,
    21471,
    22074,
    22664,
    23241,
    23803,
    24350,
    24883,
    25401,
    25903,
    26389,
    26860,
    27314,
    27752,
    28173,
    28576,
    28963,
    29331,
    29682,
    30015,
    30330,
    30626,
    30904,
    31163,
    31403,
    31624,
    31825,
    32008,
    32171,
    32314,
    32438,
    32542,
    32627,
    32691,
    32736,
    32761,
    32766,
    32751,
    32716,
    32661,
    32587,
    32493,
    32379,
    32245,
    32092,
    31919,
    31727,
    31516,
    31285,
    31036,
    30767,
    30480,
    30175,
    29851,
    29509,
    29149,
    28772,
    28377,
    27964,
    27535,
    27089,
    26627,
    26148,
    25654,
    25144,
    24618,
    24078,
    23523,
    22954,
    22371,
    21774,
    21165,
    20542,
    19907,
    19259,
    18600,
    17930,
    17249,
    16557,
    15856,
    15145,
    14424,
    13695,
    12957,
    12212,
    11459,
    10699,
    9933,
    9161,
    8383,
    7600,
    6812,
    6020,
    5225,
    4426,
    3625,
    2822,
    2017,
    1210,
    403,
    -403,
    -1210,
    -2017,
    -2822,
    -3625,
    -4426,
    -5225,
    -6020,
    -6812,
    -7600,
    -8383,
    -9161,
    -9933,
    -10699,
    -11459,
    -12212,
    -12957,
    -13695,
    -14424,
    -15145,
    -15856,
    -16557,
    -17249,
    -17930,
    -18600,
    -19259,
    -19907,
    -20542,
    -21165,
    -21774,
    -22371,
    -22954,
    -23523,
    -24078,
    -24618,
    -25144,
    -25654,
    -26148,
    -26627,
    -27089,
    -27535,
    -27964,
    -28377,
    -28772,
    -29149,
    -29509,
    -29851,
    -30175,
    -30480,
    -30767,
    -31036,
    -31285,
    -31516,
    -31727,
    -31919,
    -32092,
    -32245,
    -32379,
    -32493,
    -32587,
    -32661,
    -32716,
    -32751,
    -32766,
    -32761,
    -32736,
    -32691,
    -32627,
    -32542,
    -32438,
    -32314,
    -32171,
    -32008,
    -31825,
    -31624,
    -31403,
    -31163,
    -30904,
    -30626,
    -30330,
    -30015,
    -29682,
    -29331,
    -28963,
    -28576,
    -28173,
    -27752,
    -27314,
    -26860,
    -26389,
    -25903,
    -25401,
    -24883,
    -24350,
    -23803,
    -23241,
    -22664,
    -22074,
    -21471,
    -20855,
    -20226,
    -19585,
    -18931,
    -18267,
    -17591,
    -16905,
    -16208,
    -15501,
    -14785,
    -14060,
    -13327,
    -12585,
    -11836,
    -11080,
    -10317,
    -9547,
    -8772,
    -7992,
    -7206,
    -6417,
    -5623,
    -4826,
    -4026,
    -3224,
    -2419,
    -1614,
    -807,
    0,
};

// 256. use result = cos_lut[(uint8_t) u1 >> 8],  u1 is q15
const int16_t cos_lut[] = {
    32767,
    32757,
    32727,
    32677,
    32607,
    32518,
    32409,
    32280,
    32132,
    31964,
    31777,
    31570,
    31345,
    31100,
    30836,
    30554,
    30253,
    29934,
    29596,
    29241,
    28868,
    28477,
    28069,
    27644,
    27202,
    26744,
    26269,
    25779,
    25273,
    24751,
    24215,
    23663,
    23098,
    22518,
    21925,
    21318,
    20699,
    20067,
    19422,
    18766,
    18099,
    17420,
    16731,
    16032,
    15323,
    14605,
    13878,
    13142,
    12399,
    11648,
    10890,
    10125,
    9354,
    8578,
    7796,
    7009,
    6219,
    5424,
    4626,
    3826,
    3023,
    2218,
    1412,
    605,
    -201,
    -1009,
    -1815,
    -2621,
    -3425,
    -4226,
    -5026,
    -5822,
    -6615,
    -7403,
    -8187,
    -8967,
    -9740,
    -10508,
    -11270,
    -12024,
    -12772,
    -13511,
    -14243,
    -14965,
    -15679,
    -16383,
    -17077,
    -17761,
    -18434,
    -19096,
    -19746,
    -20384,
    -21010,
    -21623,
    -22223,
    -22810,
    -23382,
    -23941,
    -24485,
    -25014,
    -25528,
    -26026,
    -26509,
    -26975,
    -27425,
    -27859,
    -28275,
    -28674,
    -29056,
    -29421,
    -29767,
    -30096,
    -30406,
    -30697,
    -30970,
    -31225,
    -31460,
    -31676,
    -31873,
    -32050,
    -32209,
    -32347,
    -32466,
    -32565,
    -32645,
    -32704,
    -32744,
    -32764,
    -32764,
    -32744,
    -32704,
    -32645,
    -32565,
    -32466,
    -32347,
    -32209,
    -32050,
    -31873,
    -31676,
    -31460,
    -31225,
    -30970,
    -30697,
    -30406,
    -30096,
    -29767,
    -29421,
    -29056,
    -28674,
    -28275,
    -27859,
    -27425,
    -26975,
    -26509,
    -26026,
    -25528,
    -25014,
    -24485,
    -23941,
    -23382,
    -22810,
    -22223,
    -21623,
    -21010,
    -20384,
    -19746,
    -19096,
    -18434,
    -17761,
    -17077,
    -16383,
    -15679,
    -14965,
    -14243,
    -13511,
    -12772,
    -12024,
    -11270,
    -10508,
    -9740,
    -8967,
    -8187,
    -7403,
    -6615,
    -5822,
    -5026,
    -4226,
    -3425,
    -2621,
    -1815,
    -1009,
    -201,
    605,
    1412,
    2218,
    3023,
    3826,
    4626,
    5424,
    6219,
    7009,
    7796,
    8578,
    9354,
    10125,
    10890,
    11648,
    12399,
    13142,
    13878,
    14605,
    15323,
    16032,
    16731,
    17420,
    18099,
    18766,
    19422,
    20067,
    20699,
    21318,
    21925,
    22518,
    23098,
    23663,
    24215,
    24751,
    25273,
    25779,
    26269,
    26744,
    27202,
    27644,
    28069,
    28477,
    28868,
    29241,
    29596,
    29934,
    30253,
    30554,
    30836,
    31100,
    31345,
    31570,
    31777,
    31964,
    32132,
    32280,
    32409,
    32518,
    32607,
    32677,
    32727,
    32757,
    32767,
};

// int32_t q15  -5 ~ 5. result /= 32768
const int32_t exp_lut[] = {
    220,
    229,
    238,
    248,
    258,
    268,
    279,
    290,
    302,
    314,
    326,
    339,
    353,
    367,
    382,
    397,
    413,
    430,
    447,
    465,
    483,
    503,
    523,
    544,
    565,
    588,
    612,
    636,
    661,
    688,
    715,
    744,
    774,
    805,
    837,
    871,
    905,
    942,
    979,
    1019,
    1059,
    1102,
    1146,
    1192,
    1239,
    1289,
    1340,
    1394,
    1450,
    1508,
    1568,
    1631,
    1696,
    1764,
    1835,
    1908,
    1984,
    2064,
    2146,
    2232,
    2321,
    2414,
    2511,
    2611,
    2716,
    2824,
    2937,
    3055,
    3177,
    3304,
    3436,
    3574,
    3717,
    3865,
    4020,
    4181,
    4348,
    4522,
    4703,
    4891,
    5086,
    5290,
    5501,
    5722,
    5950,
    6188,
    6436,
    6693,
    6961,
    7239,
    7529,
    7830,
    8143,
    8469,
    8808,
    9160,
    9526,
    9908,
    10304,
    10716,
    11144,
    11590,
    12054,
    12536,
    13037,
    13559,
    14101,
    14665,
    15252,
    15862,
    16496,
    17156,
    17842,
    18556,
    19298,
    20069,
    20872,
    21707,
    22575,
    23478,
    24417,
    25394,
    26409,
    27466,
    28564,
    29706,
    30895,
    32130,
    33415,
    34752,
    36142,
    37587,
    39091,
    40654,
    42280,
    43971,
    45730,
    47558,
    49461,
    51439,
    53496,
    55636,
    57861,
    60175,
    62582,
    65085,
    67688,
    70395,
    73210,
    76138,
    79184,
    82350,
    85644,
    89069,
    92632,
    96337,
    100190,
    104197,
    108364,
    112698,
    117205,
    121893,
    126768,
    131838,
    137111,
    142595,
    148298,
    154229,
    160397,
    166812,
    173484,
    180423,
    187639,
    195143,
    202948,
    211065,
    219506,
    228285,
    237416,
    246911,
    256786,
    267056,
    277737,
    288845,
    300397,
    312412,
    324907,
    337901,
    351416,
    365470,
    380087,
    395289,
    411098,
    427540,
    444640,
    462423,
    480917,
    500152,
    520155,
    540958,
    562594,
    585095,
    608496,
    632832,
    658142,
    684465,
    711840,
    740309,
    769918,
    800711,
    832735,
    866040,
    900677,
    936700,
    974163,
    1013124,
    1053644,
    1095784,
    1139610,
    1185188,
    1232589,
    1281887,
    1333155,
    1386475,
    1441926,
    1499596,
    1559572,
    1621947,
    1686816,
    1754280,
    1824442,
    1897410,
    1973297,
    2052218,
    2134296,
    2219657,
    2308432,
    2400757,
    2496775,
    2596633,
    2700484,
    2808490,
    2920815,
    3037632,
    3159121,
    3285470,
    3416871,
    3553528,
    3695651,
    3843458,
    3997176,
    4157042,
    4323302,
    4496211,
    4676036,
    4863053,
};

// u1 range [-5 ~ 5]. u2 range[-1, 1],  k=32768
const int16_t sigmoid_lut[] = {
    -32328,
    -32310,
    -32292,
    -32274,
    -32254,
    -32234,
    -32213,
    -32191,
    -32168,
    -32144,
    -32119,
    -32094,
    -32067,
    -32039,
    -32011,
    -31981,
    -31950,
    -31918,
    -31884,
    -31849,
    -31813,
    -31776,
    -31737,
    -31696,
    -31654,
    -31610,
    -31565,
    -31518,
    -31469,
    -31418,
    -31365,
    -31310,
    -31253,
    -31194,
    -31133,
    -31069,
    -31003,
    -30935,
    -30864,
    -30790,
    -30713,
    -30634,
    -30552,
    -30466,
    -30377,
    -30285,
    -30190,
    -30091,
    -29989,
    -29883,
    -29773,
    -29658,
    -29540,
    -29418,
    -29291,
    -29160,
    -29024,
    -28883,
    -28737,
    -28586,
    -28430,
    -28269,
    -28101,
    -27929,
    -27750,
    -27565,
    -27374,
    -27177,
    -26973,
    -26763,
    -26546,
    -26321,
    -26090,
    -25851,
    -25604,
    -25350,
    -25089,
    -24819,
    -24541,
    -24255,
    -23960,
    -23657,
    -23345,
    -23024,
    -22694,
    -22355,
    -22007,
    -21650,
    -21283,
    -20907,
    -20521,
    -20126,
    -19721,
    -19306,
    -18882,
    -18448,
    -18005,
    -17551,
    -17088,
    -16616,
    -16134,
    -15642,
    -15142,
    -14632,
    -14113,
    -13585,
    -13049,
    -12504,
    -11951,
    -11390,
    -10822,
    -10246,
    -9662,
    -9072,
    -8476,
    -7874,
    -7265,
    -6652,
    -6034,
    -5411,
    -4784,
    -4153,
    -3520,
    -2883,
    -2245,
    -1604,
    -963,
    -321,
    321,
    963,
    1604,
    2245,
    2883,
    3520,
    4153,
    4784,
    5411,
    6034,
    6652,
    7265,
    7874,
    8476,
    9072,
    9662,
    10246,
    10822,
    11390,
    11951,
    12504,
    13049,
    13585,
    14113,
    14632,
    15142,
    15642,
    16134,
    16616,
    17088,
    17551,
    18005,
    18448,
    18882,
    19306,
    19721,
    20126,
    20521,
    20907,
    21283,
    21650,
    22007,
    22355,
    22694,
    23024,
    23345,
    23657,
    23960,
    24255,
    24541,
    24819,
    25089,
    25350,
    25604,
    25851,
    26090,
    26321,
    26546,
    26763,
    26973,
    27177,
    27374,
    27565,
    27750,
    27929,
    28101,
    28269,
    28430,
    28586,
    28737,
    28883,
    29024,
    29160,
    29291,
    29418,
    29540,
    29658,
    29773,
    29883,
    29989,
    30091,
    30190,
    30285,
    30377,
    30466,
    30552,
    30634,
    30713,
    30790,
    30864,
    30935,
    31003,
    31069,
    31133,
    31194,
    31253,
    31310,
    31365,
    31418,
    31469,
    31518,
    31565,
    31610,
    31654,
    31696,
    31737,
    31776,
    31813,
    31849,
    31884,
    31918,
    31950,
    31981,
    32011,
    32039,
    32067,
    32094,
    32119,
    32144,
    32168,
    32191,
    32213,
    32234,
    32254,
    32274,
    32292,
    32310,
    32328,
};

int16_t inline sigmoid_q15(int32_t u1)
{
    if (u1 > 32767 * 5)
    {
        return 1 * 32767;
    }
    else if (u1 < -32767 * 5)
    {
        return -1 * 32767;
    }

    u1 /= 5; // scale to [-32767 32767]
    uint8_t idx = u1 / 256 + 127;
    return sigmoid_lut[idx];
}

float inline sigmoid_float(float u1)
{
    float expx = expf(-u1);

    return 2.0f / (1 + expx) - 1;
}

float inline sigmoid_fake_float(float u1)
{
#if 1
    const float a = 0.2f;
    const float bound = (1 / a);
    if (u1 > bound)
    {
        return 1;
    }
    else if (u1 < -bound)
    {
        return -1;
    }

    return a * u1;
#else
    if (u1 > 1)
    {
        return 1;
    }
    else if (u1 < -1)
    {
        return -1;
    }

    return u1;
#endif
}

int16_t inline sin_q15(uint16_t u1)
{
    return sin_lut[u1 >> 8];
}

int16_t inline cos_q15(uint16_t u1)
{
    return cos_lut[u1 >> 8];
}

const int16_t sin_cos_table[] = {
    0x0000,
    0x00C9,
    0x0192,
    0x025B,
    0x0324,
    0x03ED,
    0x04B6,
    0x057F,
    0x0648,
    0x0711,
    0x07D9,
    0x08A2,
    0x096A,
    0x0A33,
    0x0AFB,
    0x0BC4,
    0x0C8C,
    0x0D54,
    0x0E1C,
    0x0EE3,
    0x0FAB,
    0x1072,
    0x113A,
    0x1201,
    0x12C8,
    0x138F,
    0x1455,
    0x151C,
    0x15E2,
    0x16A8,
    0x176E,
    0x1833,
    0x18F9,
    0x19BE,
    0x1A82,
    0x1B47,
    0x1C0B,
    0x1CCF,
    0x1D93,
    0x1E57,
    0x1F1A,
    0x1FDD,
    0x209F,
    0x2161,
    0x2223,
    0x22E5,
    0x23A6,
    0x2467,
    0x2528,
    0x25E8,
    0x26A8,
    0x2767,
    0x2826,
    0x28E5,
    0x29A3,
    0x2A61,
    0x2B1F,
    0x2BDC,
    0x2C99,
    0x2D55,
    0x2E11,
    0x2ECC,
    0x2F87,
    0x3041,
    0x30FB,
    0x31B5,
    0x326E,
    0x3326,
    0x33DF,
    0x3496,
    0x354D,
    0x3604,
    0x36BA,
    0x376F,
    0x3824,
    0x38D9,
    0x398C,
    0x3A40,
    0x3AF2,
    0x3BA5,
    0x3C56,
    0x3D07,
    0x3DB8,
    0x3E68,
    0x3F17,
    0x3FC5,
    0x4073,
    0x4121,
    0x41CE,
    0x427A,
    0x4325,
    0x43D0,
    0x447A,
    0x4524,
    0x45CD,
    0x4675,
    0x471C,
    0x47C3,
    0x4869,
    0x490F,
    0x49B4,
    0x4A58,
    0x4AFB,
    0x4B9D,
    0x4C3F,
    0x4CE0,
    0x4D81,
    0x4E20,
    0x4EBF,
    0x4F5D,
    0x4FFB,
    0x5097,
    0x5133,
    0x51CE,
    0x5268,
    0x5302,
    0x539B,
    0x5432,
    0x54C9,
    0x5560,
    0x55F5,
    0x568A,
    0x571D,
    0x57B0,
    0x5842,
    0x58D3,
    0x5964,
    0x59F3,
    0x5A82,
    0x5B0F,
    0x5B9C,
    0x5C28,
    0x5CB3,
    0x5D3E,
    0x5DC7,
    0x5E4F,
    0x5ED7,
    0x5F5D,
    0x5FE3,
    0x6068,
    0x60EB,
    0x616E,
    0x61F0,
    0x6271,
    0x62F1,
    0x6370,
    0x63EE,
    0x646C,
    0x64E8,
    0x6563,
    0x65DD,
    0x6656,
    0x66CF,
    0x6746,
    0x67BC,
    0x6832,
    0x68A6,
    0x6919,
    0x698B,
    0x69FD,
    0x6A6D,
    0x6ADC,
    0x6B4A,
    0x6BB7,
    0x6C23,
    0x6C8E,
    0x6CF8,
    0x6D61,
    0x6DC9,
    0x6E30,
    0x6E96,
    0x6EFB,
    0x6F5E,
    0x6FC1,
    0x7022,
    0x7083,
    0x70E2,
    0x7140,
    0x719D,
    0x71F9,
    0x7254,
    0x72AE,
    0x7307,
    0x735E,
    0x73B5,
    0x740A,
    0x745F,
    0x74B2,
    0x7504,
    0x7555,
    0x75A5,
    0x75F3,
    0x7641,
    0x768D,
    0x76D8,
    0x7722,
    0x776B,
    0x77B3,
    0x77FA,
    0x783F,
    0x7884,
    0x78C7,
    0x7909,
    0x794A,
    0x7989,
    0x79C8,
    0x7A05,
    0x7A41,
    0x7A7C,
    0x7AB6,
    0x7AEE,
    0x7B26,
    0x7B5C,
    0x7B91,
    0x7BC5,
    0x7BF8,
    0x7C29,
    0x7C59,
    0x7C88,
    0x7CB6,
    0x7CE3,
    0x7D0E,
    0x7D39,
    0x7D62,
    0x7D89,
    0x7DB0,
    0x7DD5,
    0x7DFA,
    0x7E1D,
    0x7E3E,
    0x7E5F,
    0x7E7E,
    0x7E9C,
    0x7EB9,
    0x7ED5,
    0x7EEF,
    0x7F09,
    0x7F21,
    0x7F37,
    0x7F4D,
    0x7F61,
    0x7F74,
    0x7F86,
    0x7F97,
    0x7FA6,
    0x7FB4,
    0x7FC1,
    0x7FCD,
    0x7FD8,
    0x7FE1,
    0x7FE9,
    0x7FF0,
    0x7FF5,
    0x7FF9,
    0x7FFD,
    0x7FFE,
};

trig_value_t calc_sin_cos(int16_t u1, int16_t *sinx, int16_t *cosx)
{
    uint16_t index;
    trig_value_t local_trig;
    /* 10 bit index computation  */
    index = (uint16_t)u1 >> 6; // /= 64

    switch (index & 0x300)
    {
    case 0x000: // 0-90
        local_trig.sinx = sin_cos_table[(uint8_t)index];
        local_trig.cosx = sin_cos_table[0xFF - (uint8_t)index];
        break;

    case 0x100:
        local_trig.sinx = sin_cos_table[0xFF - (uint8_t)index];
        local_trig.cosx = -sin_cos_table[(uint8_t)index];
        break;

    case 0x200:
        local_trig.sinx = -sin_cos_table[(uint8_t)index];
        local_trig.cosx = -sin_cos_table[0xFF - (uint8_t)index];
        break;

    case 0x300:
        local_trig.sinx = -sin_cos_table[0xFF - (uint8_t)index];
        local_trig.cosx = sin_cos_table[(uint8_t)index];
        break;
    default:
        break;
    }

    *sinx = local_trig.sinx;
    *cosx = local_trig.cosx;
    return (local_trig);
}

/*******************************************************************************
 * Function Name  : Clarke Transformation
 * Description    : This function transforms stator currents qIas and
 *                  qIbs (which are directed along axes each displaced by
 *                  120 degrees) into currents qIalpha and qIbeta in a
 *                  stationary qd reference frame.
 *                  qIalpha = qIas
 *                  qIbeta = (2*Ib+Ia)/sqrt(3)
 *   ialfa 与 ia 轴水平向右👉->  ibeta 垂直向上↑👆
 *   \ib              ↑ ibeta
 *    \____ia   ==>   |_______ ialfa.
 *    /
 *   /ic
 * fv.Ialfa = fv.ia;
 * fv.Ibeta = (fv.ia + 2*fv.ib) * 0.57735026918963f;
 * Input          : ia, ib
 * Output         : ialfa, ibeta.
 * Return         : none.
 *******************************************************************************/

void clarke_q15(int16_t ia,
                int16_t ib,
                int16_t *ialfa,
                int16_t *ibeta)
{
    /* 1/sqrt(3) in q1.15 format=0.5773315*/
    const int16_t divSQRT_3 = 0x49E6;
    // sqrt3/2 in 1.15
    const int16_t qSqrt3div2 = 0x6ED9;
    int32_t qIa_divSQRT3_tmp;
    int32_t qIb_divSQRT3_tmp;
    int32_t sum_tmp;

    // qIalpha = qIas
    *ialfa = ia;

    qIa_divSQRT3_tmp = divSQRT_3 * ia;
    qIa_divSQRT3_tmp /= 32768;

    qIb_divSQRT3_tmp = divSQRT_3 * ib;
    qIb_divSQRT3_tmp /= 32768;

    // so far everything is good.
    // qIbeta = (2*qIbs+qIas)/sqrt(3)  //st原本是-
    // 3个q15相加， 有可能溢出的！！sum_tmp
    sum_tmp = qIa_divSQRT3_tmp + qIb_divSQRT3_tmp + qIb_divSQRT3_tmp;
    if (sum_tmp > 32767)
    {
        sum_tmp = 32767;
    }
    else if (sum_tmp < -32767)
    {
        sum_tmp = -32767;
    }

    *ibeta = sum_tmp;
}

/*******************************************************************************
 * Function Name  : Park Transformation
 * Description    : This function transforms stator currents qIalpha and qIbeta,
 *                  which belong to a stationary qd reference frame, to a rotor
 *                  flux synchronous reference frame (properly oriented), so as
 *                  to obtain qIq and qIds.
 * modified langgo
 * fv.id = fv.cos_theta * fv.Ialfa + fv.sin_theta * fv.Ibeta;
 * fv.iq = fv.cos_theta * fv.Ibeta - fv.sin_theta * fv.Ialfa;
 * Input          : alfa, beta, theta.
 * Output         : id, iq.
 * Return         : none.
 *******************************************************************************/
static int16_t park_sinx; // used for rev park fast calc.
static int16_t park_cosx; // used for rev park fast calc.

void park_q15(int16_t ialfa,
              int16_t ibeta,
              uint16_t theta,
              int16_t *id,
              int16_t *iq)
{
    int32_t id_tmp1, id_tmp2;
    int32_t iq_tmp1, iq_tmp2;
    int16_t sinx = sin_q15(theta);
    int16_t cosx = cos_q15(theta);

    // No overflow guaranteed
    id_tmp1 = ialfa * cosx;
    id_tmp1 /= 32768;

    // No overflow guaranteed
    id_tmp2 = ibeta * sinx;
    id_tmp2 /= 32768;

    // Id component in Q1.15 Format
    *id = (int16_t)id_tmp1 + (int16_t)id_tmp2;

    // No overflow guaranteed
    iq_tmp1 = ibeta * cosx;
    iq_tmp1 /= 32768;

    // No overflow guaranteed
    iq_tmp2 = ialfa * sinx;
    iq_tmp2 /= 32768;

    // Iq component in Q1.15 Format
    *iq = (int16_t)iq_tmp1 - (int16_t)iq_tmp2;
}

/*******************************************************************************
* Function Name  : Rev_Park Transformation
* Description    : This function transforms stator voltage qVq and qVd, that
*                  belong to a rotor flux synchronous rotating frame, to a
*                 stationary reference frame, so as to obtain qValpha and qVbeta
* Input          : vq, vd, theta.
* Output         : valfa, vbeta
* modified langgo:
    fv.Ualfa = -fv.sin_theta * fv.vq + fv.cos_theta * fv.vd ;
    fv.Ubeta = fv.cos_theta * fv.vq + fv.sin_theta * fv.vd ;
* Return         : none.
*******************************************************************************/

void inverse_park_q15(int16_t vq,
                      int16_t vd,
                      uint16_t theta,
                      int16_t *valfa,
                      int16_t *vbeta)
{
    int32_t valfa1, valfa2, vbeta1, vbeta2;

    int16_t sinx = sin_q15(theta);
    int16_t cosx = cos_q15(theta);

    // No overflow guaranteed  input q d
    valfa1 = -vq * sinx;
    valfa1 /= 32768;

    valfa2 = vd * cosx;
    valfa2 /= 32768;

    *valfa = (int16_t)valfa1 + (int16_t)(valfa2);

    vbeta1 = vq * cosx;
    vbeta1 /= 32768;

    vbeta2 = vd * sinx;
    vbeta2 /= 32768;

    *vbeta = (int16_t)vbeta1 + (int16_t)vbeta2;
}

// ta tb tc range[-1, 1] => alfa*[-1, 1]
void svpwm_q15(int16_t valfa, int16_t vbeta, int16_t tabc[])
{
    int tmp1, tmp2, tmp3;
    int sector;
    // disk judgement
    // sqrt(3)/2 = 0.866
    // 0<arctan(Uβ/ Uα) <60 in sector1
    const int16_t qSqrt3div2 = 0x6ED9;
    tmp1 = vbeta;
    // tmp2 = (qSqrt3div2 * valfa + vbeta) / 2 / 32768;
    tmp2 = (qSqrt3div2 * valfa / 32768) + vbeta / 2;
    // t1 = vbeta  ###  t2= 0.866f*fv.Ualfa+0.5f*fv.Ubeta;
    tmp3 = tmp2 - tmp1;
    sector = 3;
    sector = (tmp2 > 0) ? (sector - 1) : sector;
    sector = (tmp3 > 0) ? (sector - 1) : sector;
    sector = (tmp1 < 0) ? (7 - sector) : sector;

    switch (sector)
    {
    case 1:
    case 4:
        tabc[0] = tmp2;
        tabc[1] = tmp1 - tmp3;
        tabc[2] = -tmp2;
        break;
    case 2:
    case 5:
        tabc[0] = tmp3 + tmp2;
        tabc[1] = tmp1;
        tabc[2] = -tmp1;
        break;
    case 3:
    case 6:
        tabc[0] = tmp3;
        tabc[1] = -tmp3;
        tabc[2] = -(tmp1 + tmp2);
        break;
    default:
        tabc[0] = 0;
        tabc[1] = 0;
        tabc[2] = 0;
        break;
    }
}

// ta tb tc range[-1, 1] => alfa*[-1, 1]. copy from ti source code.
// 归一化的svpwm valfa和vbeta范围[-1,1]对应tabc[-1,1]
void svpwm_float(float valfa, float vbeta, float tabc[])
{
    float tmp1, tmp2, tmp3;
    int sector;
    // disk judgement
    // sqrt(3)/2 = 0.866
    // 0<arctan(Uβ/ Uα) <60 in sector1
    const float qSqrt3div2 = 0.8660254f;
    // 使用了AN1078的逆clarke变换
    // v1 = vbeta;
    // v2 =
    tmp1 = vbeta;
    tmp2 = qSqrt3div2 * valfa + vbeta * 0.5f;
    // tmp2 = (qSqrt3div2 * valfa / 32768) + vbeta / 2;
    // t1 = vbeta  ###  t2= 0.866f*fv.Ualfa+0.5f*fv.Ubeta;
    tmp3 = tmp2 - tmp1;
    sector = 3;
    sector = (tmp2 > 0) ? (sector - 1) : sector;
    sector = (tmp3 > 0) ? (sector - 1) : sector;
    sector = (tmp1 < 0) ? (7 - sector) : sector;

    switch (sector)
    {
    case 1:
    case 4:
        tabc[0] = tmp2;
        tabc[1] = tmp1 - tmp3;
        tabc[2] = -tmp2;
        break;
    case 2:
    case 5:
        tabc[0] = tmp3 + tmp2;
        tabc[1] = tmp1;
        tabc[2] = -tmp1;
        break;
    case 3:
    case 6:
        tabc[0] = tmp3;
        tabc[1] = -tmp3;
        tabc[2] = -(tmp1 + tmp2);
        break;
    default:
        tabc[0] = 0;
        tabc[1] = 0;
        tabc[2] = 0;
        break;
    }
}

void svpwm_float_lianzhang(float valfa, float vbeta, float tabc[], uint16_t pwm_full, uint16_t ccr[])
{
    const float sqrt3div2 = 0.866f;
    int n, a, b, c, sector;
    float u1 = vbeta;
    float u2 = sqrt3div2 * valfa - 0.5f * vbeta;
    float u3 = -sqrt3div2 * valfa - 0.5f * vbeta;
    float t0, t7;
    float t1, t2, t3, t4, t5, t6; // sector1, use t4, t6.
    float tx, ty;                 // 1st vector time. 2rd vector time.
    float ta, tb, tc;
    // const float Ts = 1; //10^-4s 但是与svpwm同周期 所以此处=1
    // const float Udc = 100;
    // float m = sqrt3div2 * Ts / Udc; //10^-6

    a = u1 > 0 ? 1 : 0;
    b = u2 > 0 ? 1 : 0;
    c = u3 > 0 ? 1 : 0;
    n = 4 * c + 2 * b + a;

    switch (n)
    {
    case 3:
        sector = 1;
        break;
    case 1:
        sector = 2;
        break;
    case 5:
        sector = 3;
        break;
    case 4:
        sector = 4;
        break;
    case 6:
        sector = 5;
        break;
    case 2:
        sector = 6;
        break;
    default:
        sector = 1;
        break;
    }

    switch (sector)
    {
    case 1: // 0467
    {
        tx = t4 = u2;
        ty = t6 = u1;
        t0 = 0.5f * (1 - tx - ty);
        tabc[0] = t0;
        tabc[1] = t0 + tx;
        tabc[2] = t0 + tx + ty;
    }
    break;

    case 2: // 0267
    {
        ty = t6 = -u3;
        tx = t2 = -u2;
        t0 = 0.5f * (1 - tx - ty);
        tabc[1] = t0;
        tabc[0] = t0 + tx;
        tabc[2] = t0 + tx + ty;
    }
    break;

    case 3: // 0237
    {
        tx = t2 = u1;
        ty = t3 = u3;
        t0 = 0.5f * (1 - tx - ty);
        tabc[1] = t0;
        tabc[2] = t0 + tx;
        tabc[0] = t0 + tx + ty;
    }
    break;

    case 4: // 0137
    {
        ty = t3 = -u2;
        tx = t1 = -u1;
        t0 = 0.5f * (1 - tx - ty);
        tabc[2] = t0;
        tabc[1] = t0 + tx;
        tabc[0] = t0 + tx + ty;
    }
    break;

    case 5: // 0157
    {
        tx = t1 = u3;
        ty = t5 = u2;
        t0 = 0.5f * (1 - tx - ty);
        tabc[2] = t0;
        tabc[0] = t0 + tx;
        tabc[1] = t0 + tx + ty;
    }
    break;

    case 6: // 0457
    {
        ty = t5 = -u1;
        tx = t4 = -u3;
        t0 = 0.5f * (1 - tx - ty);
        tabc[0] = t0;
        tabc[2] = t0 + tx;
        tabc[1] = t0 + tx + ty;
    }
    break;
    }

    // t0 = 0.5f*(1-tx-ty);
    // tabc[0] = t0;
    // tabc[1] = t0 + tx;
    // tabc[2] = t0 + tx + ty;
    ccr[0] = pwm_full * tabc[0];
    ccr[1] = pwm_full * tabc[1];
    ccr[2] = pwm_full * tabc[2];
}

int16_t slide_filter_int16(int16_t cur,
                           uint8_t *idx,
                           int16_t fil_buffer[],
                           uint8_t len)
{
    int sum = 0;

    fil_buffer[*idx % len] = cur;
    (*idx)++;

    for (int i = 0; i < len; ++i)
    {
        sum += fil_buffer[i];
    }
    return sum / len;
}

int max_of_3(int a, int b, int c)
{
    int tmp = MAX(a, b);
    return MAX(tmp, c);
    // int max3 = MAX(tmp, c);
    // return max3;
}

int min_of_3(int a, int b, int c)
{
    int tmp = MIN(a, b);
    return MIN(tmp, c);
    // int max3 = MAX(tmp, c);
    // return max3;
}

void clarke_float(float ia,
                  float ib,
                  float *ialfa,
                  float *ibeta)
{
    static const float ONE_DIV_SQUAEROOT3 = 0.577350269f;
    *ialfa = ia;
    *ibeta = (2.0f * ib + ia) * ONE_DIV_SQUAEROOT3;
    // beta = 2/sqrt3 * ib + 1/sqrt3 * a
}

// * fv.id = fv.cos_theta * fv.Ialfa + fv.sin_theta * fv.Ibeta;
// * fv.iq = fv.cos_theta * fv.Ibeta - fv.sin_theta * fv.Ialfa;
void park_float(float ialfa, float ibeta, float theta, float *id, float *iq)
{
    // TODO: use look up tab to reduce time.
    *id = ialfa * cosf(theta) + ibeta * sinf(theta);
    *iq = -ialfa * sinf(theta) + ibeta * cosf(theta);
}

/// DQ0 Transform ///
/// Phase current amplitude = lengh of dq vector///
/// i.e. iq = 1, id = 0, peak phase current of 1///
void dq0(float theta, float a, float b, float *d, float *q)
{
    float cos = fast_sin(theta);
    float sin = fast_cos(theta);
    const float ONE_DIV_SQRT3 = 0.577350269f;
    float ibeta = 0.577350269f * (2.0f * b + a) * sin;
    *d = a * cos + ibeta * sin;
    *q = -a * sin + ibeta * cos;
}

// * fv.id = fv.cos_theta * fv.Ialfa + fv.sin_theta * fv.Ibeta;
// * fv.iq = fv.cos_theta * fv.Ibeta - fv.sin_theta * fv.Ialfa;
// TODO: fixme
void park_float_lut(float ialfa, float ibeta, float theta, float *id, float *iq)
{
    // TODO: use look up tab to reduce time.

    // *id = ialfa * cosf(theta) + ibeta * sinf(theta);
    // *iq = -ialfa * sinf(theta) + ibeta * cosf(theta);
}

// fv.Ualfa = -fv.sin_theta * fv.vq + fv.cos_theta * fv.vd ;
// fv.Ubeta = fv.cos_theta * fv.vq + fv.sin_theta * fv.vd ;
void inverse_park_float(float vd,
                        float vq,
                        float theta,
                        float *valfa,
                        float *vbeta)
{
    *valfa = vd * cosf(theta) - vq * sinf(theta);
    *vbeta = vd * sinf(theta) + vq * cosf(theta);
}

#include "log.h"
static volatile int fuck_bug;
int32_t calc_theta_delta_raw(uint16_t *last, uint16_t cur, const int cnt_full_range)
{
    static int delta, res1, res2;

    // #define ABS(x) (((x) > 0) ? (x) : -(x))

    // 考虑两种情况 正传或者反转越界
    if (cur <= *last)
    {
        res1 = cur + cnt_full_range - *last; //+
        res2 = cur - *last;                  //-
    }
    else
    {                                        // cur > last
        res1 = cur - *last;                  //+
        res2 = cur - cnt_full_range - *last; //-
    }

    // 不管正反转， 转过的角度小的那个是真的
    if (ABS(res1) < ABS(res2))
    {
        delta = res1;
    }
    else
    {
        delta = res2;
    }

    // update last.
    *last = cur;

    return delta;
}

// will auto update last!
float calc_theta_delta_f32(float *last, float cur, const float cnt_full_range)
{
    static float delta, res1, res2;

    // 考虑两种情况 正传或者反转越界
    if (cur <= *last)
    {
        res1 = cur + cnt_full_range - *last; //+
        res2 = cur - *last;                  //-
    }
    else
    {                                        // cur > last
        res1 = cur - *last;                  //+
        res2 = cur - cnt_full_range - *last; //-
    }

    // 不管正反转， 转过的角度小的那个是真的
    if (ABS(res1) < ABS(res2))
    {
        delta = res1;
    }
    else
    {
        delta = res2;
    }

    // update last.
    *last = cur;

    return delta;
}

// call example : ref = ramp_calc(cur, set); called freq = 1000 hz.  //1s done.
int32_t ramp_calc(int32_t target, int32_t cur, int32_t dy)
{
    int delta = target - cur;
    if (cur == target || ABS(delta) < dy)
    {
        return target;
    }

    if (delta > 0)
    {
        return cur + dy;
    }
    else
    {
        return cur - dy;
    }
}

#define RAMP_DELTA (0.2f)
float ramp_calc_float(float cur, float target, float ramp_delta)
{
    if (ABS(cur - target) < 1.1f)
    {
        return cur;
    }
    float delta = target - cur;
    if (delta > 0)
    {
        return cur + ramp_delta;
    }
    else
    {
        return cur - ramp_delta;
    }
}

// void lpf_1rd_init(lpf1_t *lpf1rd, float tc, float z)
// {
//     lpf1rd->z1 = 0;
//     lpf1rd->tc = 0;
//     lpf1rd->out = 0;
//     lpf1rd->tc = tc;
// }

void lpf_1rd_init(lpf1_t *lpf1rd, float ts, float f_cutoff)
{
    // lpf1rd->a = 2*3.1416*ts*f_cutoff;
    lpf1rd->a = ts / (ts + (1.f / (2.f * 3.1415926f * f_cutoff)));
    lpf1rd->_1_a = 1 - lpf1rd->a;
}

// tsample unit: s, f_cutoff unit: hz, 注意这里截止是指角频率。
// 实测结果，输入频率100hz的正弦波（角频率w=2pi*f = 628hz), 截止频率设置为角频率100,
void lpf1_init(lpf1_t *lpf1, float t_sample, float f_cutoff)
{
    const float pi = 3.14159265358979323846;
    // Calculate the filter coefficient alpha based on sample rate and cutoff frequency
    float omega = 2 * pi * f_cutoff; // Angular frequency
    // float omega = 1 * f_cutoff; // Angular frequency

    // Calculate the filter time constant tau
    float tau = 1 / (2 * pi * f_cutoff);

    // Calculate the filter coefficient alpha
    float alpha = t_sample / (tau + t_sample);

    lpf1->a = alpha;
    lpf1->_1_a = 1 - lpf1->a;
}

float lpf1_calc(lpf1_t *lpf1, float new_sample)
{
    // lpf1->in =  new_sample;
    // lpf1->yn = lpf1->a * new_sample + (1-lpf1->a) * lpf1->yn_1;
    // lpf1->_1_a  = 1 - lpf1->a;
    lpf1->yn = lpf1->a * new_sample + lpf1->_1_a * lpf1->yn_1;
    lpf1->yn_1 = lpf1->yn;

    return lpf1->yn;
}

void hpf1_init(hpf1_t *hpf1, float ts, float f_cutoff)
{
    const float pi = 3.14159265358979323846;
    float tau = 1 / (2 * pi * f_cutoff); // tau = RC
    hpf1->a = tau / (ts + tau);
    // hpf1->_1_a = 1 - hpf1->a;
}

float hpf1_calc(hpf1_t *hpf1, float new_sample)
{
    hpf1->yn = hpf1->a * (hpf1->yn_1 + new_sample - hpf1->xn_1);
    hpf1->xn_1 = new_sample;
    hpf1->yn_1 = hpf1->yn;
    return hpf1->yn;
}

// a=T*Wc = T*2pi*fc
float lpf1_calc_coef(float Ts, float fcutoff)
{
    return 2 * 3.1416 * Ts * fcutoff;
}

// 512*4 = 2048
const float sin_table[] = {
    0, 0.012296, 0.024589, 0.036879, 0.049164, 0.061441, 0.073708, 0.085965, 0.098208, 0.11044, 0.12265, 0.13484, 0.14702, 0.15917, 0.17129, 0.18339, 0.19547, 0.20751, 0.21952, 0.2315, 0.24345, 0.25535, 0.26722, 0.27905, 0.29084, 0.30258, 0.31427, 0.32592, 0.33752, 0.34907, 0.36057, 0.37201, 0.38339, 0.39472, 0.40599, 0.41719, 0.42834, 0.43941, 0.45043, 0.46137, 0.47224, 0.48305, 0.49378, 0.50443, 0.51501, 0.52551, 0.53593, 0.54627, 0.55653, 0.5667, 0.57679, 0.58679, 0.5967, 0.60652, 0.61625, 0.62589, 0.63543, 0.64488, 0.65423, 0.66348, 0.67263, 0.68167, 0.69062, 0.69946, 0.70819, 0.71682, 0.72534, 0.73375, 0.74205, 0.75023, 0.75831, 0.76626, 0.77411, 0.78183, 0.78944, 0.79693, 0.80429, 0.81154, 0.81866, 0.82566, 0.83254, 0.83928, 0.84591, 0.8524, 0.85876, 0.865, 0.8711, 0.87708, 0.88292, 0.88862, 0.89419, 0.89963, 0.90493, 0.9101, 0.91512, 0.92001, 0.92476, 0.92937, 0.93384, 0.93816, 0.94235, 0.94639, 0.95029, 0.95405, 0.95766, 0.96113, 0.96445, 0.96763, 0.97066, 0.97354, 0.97628, 0.97887, 0.98131, 0.9836, 0.98574, 0.98774, 0.98958, 0.99128, 0.99282, 0.99422, 0.99546, 0.99656, 0.9975, 0.99829, 0.99894, 0.99943, 0.99977, 0.99996, 1, 0.99988, 0.99962, 0.9992, 0.99863, 0.99792, 0.99705, 0.99603, 0.99486, 0.99354, 0.99207, 0.99045, 0.98868, 0.98676, 0.98469, 0.98247, 0.9801, 0.97759, 0.97493, 0.97212, 0.96916, 0.96606, 0.96281, 0.95941, 0.95587, 0.95219, 0.94836, 0.94439, 0.94028, 0.93602, 0.93162, 0.92708, 0.9224, 0.91758, 0.91263, 0.90753, 0.9023, 0.89693, 0.89142, 0.88579, 0.88001, 0.87411, 0.86807, 0.8619, 0.8556, 0.84917, 0.84261, 0.83593, 0.82911, 0.82218, 0.81512, 0.80793, 0.80062, 0.7932, 0.78565, 0.77798, 0.7702, 0.7623, 0.75428, 0.74615, 0.73791, 0.72956, 0.72109, 0.71252, 0.70384, 0.69505, 0.68616, 0.67716, 0.66806, 0.65886, 0.64956, 0.64017, 0.63067, 0.62108, 0.6114, 0.60162, 0.59176, 0.5818, 0.57176, 0.56163, 0.55141, 0.54111, 0.53073, 0.52027, 0.50973, 0.49911, 0.48842, 0.47765, 0.46682, 0.45591, 0.44493, 0.43388, 0.42277, 0.4116, 0.40036, 0.38906, 0.37771, 0.36629, 0.35483, 0.3433, 0.33173, 0.32011, 0.30843, 0.29671, 0.28495, 0.27314, 0.26129, 0.2494, 0.23748, 0.22552, 0.21352, 0.20149, 0.18943, 0.17735, 0.16523, 0.15309, 0.14093, 0.12875, 0.11655, 0.10432, 0.092088, 0.079838, 0.067576, 0.055303, 0.043022, 0.030735, 0.018443, 0.0061479, -0.0061479, -0.018443, -0.030735, -0.043022, -0.055303, -0.067576, -0.079838, -0.092088, -0.10432, -0.11655, -0.12875, -0.14093, -0.15309, -0.16523, -0.17735, -0.18943, -0.20149, -0.21352, -0.22552, -0.23748, -0.2494, -0.26129, -0.27314, -0.28495, -0.29671, -0.30843, -0.32011, -0.33173, -0.3433, -0.35483, -0.36629, -0.37771, -0.38906, -0.40036, -0.4116, -0.42277, -0.43388, -0.44493, -0.45591, -0.46682, -0.47765, -0.48842, -0.49911, -0.50973, -0.52027, -0.53073, -0.54111, -0.55141, -0.56163, -0.57176, -0.5818, -0.59176, -0.60162, -0.6114, -0.62108, -0.63067, -0.64017, -0.64956, -0.65886, -0.66806, -0.67716, -0.68616, -0.69505, -0.70384, -0.71252, -0.72109, -0.72956, -0.73791, -0.74615, -0.75428, -0.7623, -0.7702, -0.77798, -0.78565, -0.7932, -0.80062, -0.80793, -0.81512, -0.82218, -0.82911, -0.83593, -0.84261, -0.84917, -0.8556, -0.8619, -0.86807, -0.87411, -0.88001, -0.88579, -0.89142, -0.89693, -0.9023, -0.90753, -0.91263, -0.91758, -0.9224, -0.92708, -0.93162, -0.93602, -0.94028, -0.94439, -0.94836, -0.95219, -0.95587, -0.95941, -0.96281, -0.96606, -0.96916, -0.97212, -0.97493, -0.97759, -0.9801, -0.98247, -0.98469, -0.98676, -0.98868, -0.99045, -0.99207, -0.99354, -0.99486, -0.99603, -0.99705, -0.99792, -0.99863, -0.9992, -0.99962, -0.99988, -1, -0.99996, -0.99977, -0.99943, -0.99894, -0.99829, -0.9975, -0.99656, -0.99546, -0.99422, -0.99282, -0.99128, -0.98958, -0.98774, -0.98574, -0.9836, -0.98131, -0.97887, -0.97628, -0.97354, -0.97066, -0.96763, -0.96445, -0.96113, -0.95766, -0.95405, -0.95029, -0.94639, -0.94235, -0.93816, -0.93384, -0.92937, -0.92476, -0.92001, -0.91512, -0.9101, -0.90493, -0.89963, -0.89419, -0.88862, -0.88292, -0.87708, -0.8711, -0.865, -0.85876, -0.8524, -0.84591, -0.83928, -0.83254, -0.82566, -0.81866, -0.81154, -0.80429, -0.79693, -0.78944, -0.78183, -0.77411, -0.76626, -0.75831, -0.75023, -0.74205, -0.73375, -0.72534, -0.71682, -0.70819, -0.69946, -0.69062, -0.68167, -0.67263, -0.66348, -0.65423, -0.64488, -0.63543, -0.62589, -0.61625, -0.60652, -0.5967, -0.58679, -0.57679, -0.5667, -0.55653, -0.54627, -0.53593, -0.52551, -0.51501, -0.50443, -0.49378, -0.48305, -0.47224, -0.46137, -0.45043, -0.43941, -0.42834, -0.41719, -0.40599, -0.39472, -0.38339, -0.37201, -0.36057, -0.34907, -0.33752, -0.32592, -0.31427, -0.30258, -0.29084, -0.27905, -0.26722, -0.25535, -0.24345, -0.2315, -0.21952, -0.20751, -0.19547, -0.18339, -0.17129, -0.15917, -0.14702, -0.13484, -0.12265, -0.11044, -0.098208, -0.085965, -0.073708, -0.061441, -0.049164, -0.036879, -0.024589, -0.012296, 0};

const float multiplier = 81.4873308631f; // 2pi分成512 索引=multiplier*theta(rad)
// MIT cheeta
float fast_sin(float theta)
{
    while (theta < 0.0f)
        theta += 6.28318530718f;
    while (theta >= 6.28318530718f)
        theta -= 6.28318530718f;
    return sin_table[(int)(multiplier * theta)];
}

float fast_cos(float theta)
{
    return fast_sin(1.57079632679f - theta);
}

int16_t value_limit(int32_t x, int16_t MIN, int16_t MAX)
{
    if (x > MAX)
        return MAX;
    if (x < MIN)
        return MIN;
    return x;
}

//
void luenburger_observer_init(lbg_observer_t *ob, float J, float B, float Kt, float l1, float l2)
{
    ob->J = J;
    ob->one_by_J = 1.f / J;
    ob->B = B;
    ob->Kt = Kt;
    ob->l1 = l1;
    ob->l2 = l2;
}

// input iq * Kt = Te
// input w = omega_mechenical
// eval output = Tl
void luenburger_observer_update(lbg_observer_t *ob, float iq, float w)
{
    const float TS = 50e-6; // sample rate = 50us. TODO. add ts to ob structure.
    ob->Te = iq * ob->Kt;
    ob->w = w;
    ob->dtload_est = ob->l2 * (ob->w - ob->w_est);
    ob->tload_est += ob->dtload_est * TS;
    ob->sum4 = ob->Te + ob->J * ob->l1 * (ob->w - ob->w_est) - ob->tload_est - ob->B * ob->w;
    ob->dw_est = ob->one_by_J * ob->sum4;
    ob->w_est += ob->dw_est * TS;
}

/*
    magnetic flux linkage online identify.(Latex)
    v_alfa = Rs*ialfa + d(phi_alfa)/dt
    phi_alfa = Ls * ialfa + psif * cos_theta
    vector: psif = psif_cos + psif_sin
*/

//ts = sample time delta
void flux_linkage_estimate_init(flux_linkage_t *pfl, float ts, float ls, float rs, float hpf_cutoff)
{
    pfl->Ls = ls;
    pfl->Rs = rs;
    pfl->ts = ts;
    hpf1_init(&pfl->hpf, ts, hpf_cutoff);
}

// fl: flux context struct
// i : ialfa or ibeta,
// u : ualfa or ubeta
float flux_linkage_estimate(flux_linkage_t *pfl, float i, float u)
{
    //
    pfl->dphi = u - pfl->Rs * i;
    pfl->phi += pfl->dphi * pfl->ts;
    pfl->phi_filtered = hpf1_calc(&pfl->hpf, pfl->phi);
    // pfl->phi += pfl->dphi_filtered;
    pfl->flux = pfl->phi_filtered - pfl->Ls * i;
    //final result psif^2 = fla.flux^2 + flb.flux^2;
    return pfl->flux;
}

//input cos as alfa, sin as beta.
float flux_psif_calc(flux_linkage_t *pfl_cos, flux_linkage_t *pfl_sin)
{
    return sqrtf(pfl_cos->flux * pfl_cos->flux + pfl_sin->flux * pfl_sin->flux);
}


/**
 * @brief     calculate position PID and position PID (parallel )
 * @param[in] pid: control pid struct
 * @param[in] get: measure feedback value
 * @param[in] set: target value
 * @retval    pid calculate output
 */
float pid_calc_parallel_f32(struct pid_f32 *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;
    if ((pid->input_max_err != 0) && (fabs(pid->err) > pid->input_max_err))
        return 0;

    pid->pout = pid->p * pid->err;
    pid->iout += pid->i * pid->err;
    //    if ( fabs( pid->d ) <= __FLT32_EPSILON__)   //判断浮点约为0的方法
    if (pid->d == 0.0f) // 判断浮点约为0的方法
        pid->dout = pid->d * (pid->err - pid->last_err);

    VA_LIMIT(pid->iout, -pid->integral_limit, pid->integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    VA_LIMIT(pid->out, -pid->max_out, pid->max_out);

    return pid->out;
}

/**
 * @brief     calculate position PID use serial pi
 * @param[in] pid: control pid struct
 * @param[in] get: measure feedback value
 * @param[in] set: target value
 * @retval    pid calculate output
 */
float pid_calc_serial_f32(struct pid_f32 *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;
    if ((pid->input_max_err != 0) && (fabs(pid->err) > pid->input_max_err))
        return 0;

    pid->pout = pid->p * pid->err;
    pid->iout += pid->p * pid->i * pid->err;
    // if ( fabs( pid->d ) <= __FLT32_EPSILON__)   //判断浮点约为0的方法
    if (pid->d != 0.0f) // 判断浮点约为0的方法
    {
        pid->dout = pid->d * (pid->err - pid->last_err);
        pid->last_err = pid->err;
    }

    VA_LIMIT(pid->iout, -pid->integral_limit, pid->integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    VA_LIMIT(pid->out, -pid->max_out, pid->max_out);

    return pid->out;
}

/**
 * @brief     calculate position PI use serial pi
 * @param[in] pid: control pid struct
 * @param[in] get: measure feedback value
 * @param[in] set: target value
 * @retval    pid calculate output
 */
float pi_calc_serial_f32(struct pid_f32 *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;
    if ((pid->input_max_err != 0) && (fabs(pid->err) > pid->input_max_err))
        return 0;

    pid->pout = pid->p * pid->err;
    pid->iout += pid->p * pid->i * pid->err;
    VA_LIMIT(pid->iout, -pid->integral_limit, pid->integral_limit);

    pid->out = pid->pout + pid->iout;
    VA_LIMIT(pid->out, -pid->max_out, pid->max_out);

    return pid->out;
}

/**
 * @brief     initialize pid parameter
 * @retval    none
 */
void pid_struct_init_f32(
    struct pid_f32 *pid,
    float maxout,
    float inte_limit,

    float kp,
    float ki,
    float kd)
{
    pid->integral_limit = inte_limit;
    pid->max_out = maxout;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    pid->pi_coef = kp * ki;
}

void pid_struct_init_q15(pid_q15_t *pid,
                         int32_t max_out,
                         int32_t integral_limit,

                         int32_t kp,
                         int32_t ki,
                         int32_t kd)
{
    pid->max_out = max_out;
    pid->integral_limit = integral_limit;
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
 * @brief     calculate delta PID and position PID
 * @param[in] pid: control pid struct
 * @param[in] get: measure feedback value
 * @param[in] set: target value
 * @retval    pid calculate output
 */
int32_t pid_calculate_q15(pid_q15_t *pid, int16_t get, int16_t set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;

    if (pid->err_limit)
    {
        VA_LIMIT(pid->err, -pid->err_limit, pid->err_limit);
    }

    pid->pout = pid->p * pid->err / 32768;
    pid->iout += pid->i * pid->err / 32768;
    if (!!pid->d)
    {
        pid->dout = pid->d * (pid->err - pid->last_err) / 32768;
    }

    VA_LIMIT(pid->iout, -pid->integral_limit, pid->integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    VA_LIMIT(pid->out, -pid->max_out, pid->max_out);

    return pid->out;
}

//calc y = 1.f/sqrt(x). famous fast inverse sqrt algorithm first used in quake3 by id software. big God john carmack.
float fast_inverse_sqrt(float x) {
    float xhalf = 0.5f * x;
    int i = *(int*)&x;            // Store floating-point bits in integer
    i = 0x5f3759df - (i >> 1);    // Initial guess for Newton's method
    x = *(float*)&i;              // Convert new bits into float
    x = x * (1.5f - xhalf * x * x); // One iteration of Newton's method
    return x;
}

/**
 * @brief output vref circle limit.
 * 
 * @param d can be id or vd
 * @param q 
 * @param limit range [0, 1]
 * @return float = norm2 of x,y or dq
 */
float circle_limitation(float *d, float *q, float limit)
{
    // float r = sqrtf(*x * *x + *y * *y);
    // we use fast invsqrt faster than sqrtf. DO NOT use sqrt(x)
    float norm2 = *d * *d + *q * *q;
    if (limit > 1.f) {
        limit = 1.f;
    }
    float r = fast_inverse_sqrt(norm2);
    if (norm2 > limit*limit)
    {
        float scale = limit * r;
        *d = *d * scale;
        *q = *q * scale;
    }
    return norm2;
}