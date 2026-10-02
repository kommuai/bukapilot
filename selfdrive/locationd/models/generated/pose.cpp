#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1862087488242639687) {
   out_1862087488242639687[0] = delta_x[0] + nom_x[0];
   out_1862087488242639687[1] = delta_x[1] + nom_x[1];
   out_1862087488242639687[2] = delta_x[2] + nom_x[2];
   out_1862087488242639687[3] = delta_x[3] + nom_x[3];
   out_1862087488242639687[4] = delta_x[4] + nom_x[4];
   out_1862087488242639687[5] = delta_x[5] + nom_x[5];
   out_1862087488242639687[6] = delta_x[6] + nom_x[6];
   out_1862087488242639687[7] = delta_x[7] + nom_x[7];
   out_1862087488242639687[8] = delta_x[8] + nom_x[8];
   out_1862087488242639687[9] = delta_x[9] + nom_x[9];
   out_1862087488242639687[10] = delta_x[10] + nom_x[10];
   out_1862087488242639687[11] = delta_x[11] + nom_x[11];
   out_1862087488242639687[12] = delta_x[12] + nom_x[12];
   out_1862087488242639687[13] = delta_x[13] + nom_x[13];
   out_1862087488242639687[14] = delta_x[14] + nom_x[14];
   out_1862087488242639687[15] = delta_x[15] + nom_x[15];
   out_1862087488242639687[16] = delta_x[16] + nom_x[16];
   out_1862087488242639687[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7512207833057071994) {
   out_7512207833057071994[0] = -nom_x[0] + true_x[0];
   out_7512207833057071994[1] = -nom_x[1] + true_x[1];
   out_7512207833057071994[2] = -nom_x[2] + true_x[2];
   out_7512207833057071994[3] = -nom_x[3] + true_x[3];
   out_7512207833057071994[4] = -nom_x[4] + true_x[4];
   out_7512207833057071994[5] = -nom_x[5] + true_x[5];
   out_7512207833057071994[6] = -nom_x[6] + true_x[6];
   out_7512207833057071994[7] = -nom_x[7] + true_x[7];
   out_7512207833057071994[8] = -nom_x[8] + true_x[8];
   out_7512207833057071994[9] = -nom_x[9] + true_x[9];
   out_7512207833057071994[10] = -nom_x[10] + true_x[10];
   out_7512207833057071994[11] = -nom_x[11] + true_x[11];
   out_7512207833057071994[12] = -nom_x[12] + true_x[12];
   out_7512207833057071994[13] = -nom_x[13] + true_x[13];
   out_7512207833057071994[14] = -nom_x[14] + true_x[14];
   out_7512207833057071994[15] = -nom_x[15] + true_x[15];
   out_7512207833057071994[16] = -nom_x[16] + true_x[16];
   out_7512207833057071994[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_5595754603494457870) {
   out_5595754603494457870[0] = 1.0;
   out_5595754603494457870[1] = 0.0;
   out_5595754603494457870[2] = 0.0;
   out_5595754603494457870[3] = 0.0;
   out_5595754603494457870[4] = 0.0;
   out_5595754603494457870[5] = 0.0;
   out_5595754603494457870[6] = 0.0;
   out_5595754603494457870[7] = 0.0;
   out_5595754603494457870[8] = 0.0;
   out_5595754603494457870[9] = 0.0;
   out_5595754603494457870[10] = 0.0;
   out_5595754603494457870[11] = 0.0;
   out_5595754603494457870[12] = 0.0;
   out_5595754603494457870[13] = 0.0;
   out_5595754603494457870[14] = 0.0;
   out_5595754603494457870[15] = 0.0;
   out_5595754603494457870[16] = 0.0;
   out_5595754603494457870[17] = 0.0;
   out_5595754603494457870[18] = 0.0;
   out_5595754603494457870[19] = 1.0;
   out_5595754603494457870[20] = 0.0;
   out_5595754603494457870[21] = 0.0;
   out_5595754603494457870[22] = 0.0;
   out_5595754603494457870[23] = 0.0;
   out_5595754603494457870[24] = 0.0;
   out_5595754603494457870[25] = 0.0;
   out_5595754603494457870[26] = 0.0;
   out_5595754603494457870[27] = 0.0;
   out_5595754603494457870[28] = 0.0;
   out_5595754603494457870[29] = 0.0;
   out_5595754603494457870[30] = 0.0;
   out_5595754603494457870[31] = 0.0;
   out_5595754603494457870[32] = 0.0;
   out_5595754603494457870[33] = 0.0;
   out_5595754603494457870[34] = 0.0;
   out_5595754603494457870[35] = 0.0;
   out_5595754603494457870[36] = 0.0;
   out_5595754603494457870[37] = 0.0;
   out_5595754603494457870[38] = 1.0;
   out_5595754603494457870[39] = 0.0;
   out_5595754603494457870[40] = 0.0;
   out_5595754603494457870[41] = 0.0;
   out_5595754603494457870[42] = 0.0;
   out_5595754603494457870[43] = 0.0;
   out_5595754603494457870[44] = 0.0;
   out_5595754603494457870[45] = 0.0;
   out_5595754603494457870[46] = 0.0;
   out_5595754603494457870[47] = 0.0;
   out_5595754603494457870[48] = 0.0;
   out_5595754603494457870[49] = 0.0;
   out_5595754603494457870[50] = 0.0;
   out_5595754603494457870[51] = 0.0;
   out_5595754603494457870[52] = 0.0;
   out_5595754603494457870[53] = 0.0;
   out_5595754603494457870[54] = 0.0;
   out_5595754603494457870[55] = 0.0;
   out_5595754603494457870[56] = 0.0;
   out_5595754603494457870[57] = 1.0;
   out_5595754603494457870[58] = 0.0;
   out_5595754603494457870[59] = 0.0;
   out_5595754603494457870[60] = 0.0;
   out_5595754603494457870[61] = 0.0;
   out_5595754603494457870[62] = 0.0;
   out_5595754603494457870[63] = 0.0;
   out_5595754603494457870[64] = 0.0;
   out_5595754603494457870[65] = 0.0;
   out_5595754603494457870[66] = 0.0;
   out_5595754603494457870[67] = 0.0;
   out_5595754603494457870[68] = 0.0;
   out_5595754603494457870[69] = 0.0;
   out_5595754603494457870[70] = 0.0;
   out_5595754603494457870[71] = 0.0;
   out_5595754603494457870[72] = 0.0;
   out_5595754603494457870[73] = 0.0;
   out_5595754603494457870[74] = 0.0;
   out_5595754603494457870[75] = 0.0;
   out_5595754603494457870[76] = 1.0;
   out_5595754603494457870[77] = 0.0;
   out_5595754603494457870[78] = 0.0;
   out_5595754603494457870[79] = 0.0;
   out_5595754603494457870[80] = 0.0;
   out_5595754603494457870[81] = 0.0;
   out_5595754603494457870[82] = 0.0;
   out_5595754603494457870[83] = 0.0;
   out_5595754603494457870[84] = 0.0;
   out_5595754603494457870[85] = 0.0;
   out_5595754603494457870[86] = 0.0;
   out_5595754603494457870[87] = 0.0;
   out_5595754603494457870[88] = 0.0;
   out_5595754603494457870[89] = 0.0;
   out_5595754603494457870[90] = 0.0;
   out_5595754603494457870[91] = 0.0;
   out_5595754603494457870[92] = 0.0;
   out_5595754603494457870[93] = 0.0;
   out_5595754603494457870[94] = 0.0;
   out_5595754603494457870[95] = 1.0;
   out_5595754603494457870[96] = 0.0;
   out_5595754603494457870[97] = 0.0;
   out_5595754603494457870[98] = 0.0;
   out_5595754603494457870[99] = 0.0;
   out_5595754603494457870[100] = 0.0;
   out_5595754603494457870[101] = 0.0;
   out_5595754603494457870[102] = 0.0;
   out_5595754603494457870[103] = 0.0;
   out_5595754603494457870[104] = 0.0;
   out_5595754603494457870[105] = 0.0;
   out_5595754603494457870[106] = 0.0;
   out_5595754603494457870[107] = 0.0;
   out_5595754603494457870[108] = 0.0;
   out_5595754603494457870[109] = 0.0;
   out_5595754603494457870[110] = 0.0;
   out_5595754603494457870[111] = 0.0;
   out_5595754603494457870[112] = 0.0;
   out_5595754603494457870[113] = 0.0;
   out_5595754603494457870[114] = 1.0;
   out_5595754603494457870[115] = 0.0;
   out_5595754603494457870[116] = 0.0;
   out_5595754603494457870[117] = 0.0;
   out_5595754603494457870[118] = 0.0;
   out_5595754603494457870[119] = 0.0;
   out_5595754603494457870[120] = 0.0;
   out_5595754603494457870[121] = 0.0;
   out_5595754603494457870[122] = 0.0;
   out_5595754603494457870[123] = 0.0;
   out_5595754603494457870[124] = 0.0;
   out_5595754603494457870[125] = 0.0;
   out_5595754603494457870[126] = 0.0;
   out_5595754603494457870[127] = 0.0;
   out_5595754603494457870[128] = 0.0;
   out_5595754603494457870[129] = 0.0;
   out_5595754603494457870[130] = 0.0;
   out_5595754603494457870[131] = 0.0;
   out_5595754603494457870[132] = 0.0;
   out_5595754603494457870[133] = 1.0;
   out_5595754603494457870[134] = 0.0;
   out_5595754603494457870[135] = 0.0;
   out_5595754603494457870[136] = 0.0;
   out_5595754603494457870[137] = 0.0;
   out_5595754603494457870[138] = 0.0;
   out_5595754603494457870[139] = 0.0;
   out_5595754603494457870[140] = 0.0;
   out_5595754603494457870[141] = 0.0;
   out_5595754603494457870[142] = 0.0;
   out_5595754603494457870[143] = 0.0;
   out_5595754603494457870[144] = 0.0;
   out_5595754603494457870[145] = 0.0;
   out_5595754603494457870[146] = 0.0;
   out_5595754603494457870[147] = 0.0;
   out_5595754603494457870[148] = 0.0;
   out_5595754603494457870[149] = 0.0;
   out_5595754603494457870[150] = 0.0;
   out_5595754603494457870[151] = 0.0;
   out_5595754603494457870[152] = 1.0;
   out_5595754603494457870[153] = 0.0;
   out_5595754603494457870[154] = 0.0;
   out_5595754603494457870[155] = 0.0;
   out_5595754603494457870[156] = 0.0;
   out_5595754603494457870[157] = 0.0;
   out_5595754603494457870[158] = 0.0;
   out_5595754603494457870[159] = 0.0;
   out_5595754603494457870[160] = 0.0;
   out_5595754603494457870[161] = 0.0;
   out_5595754603494457870[162] = 0.0;
   out_5595754603494457870[163] = 0.0;
   out_5595754603494457870[164] = 0.0;
   out_5595754603494457870[165] = 0.0;
   out_5595754603494457870[166] = 0.0;
   out_5595754603494457870[167] = 0.0;
   out_5595754603494457870[168] = 0.0;
   out_5595754603494457870[169] = 0.0;
   out_5595754603494457870[170] = 0.0;
   out_5595754603494457870[171] = 1.0;
   out_5595754603494457870[172] = 0.0;
   out_5595754603494457870[173] = 0.0;
   out_5595754603494457870[174] = 0.0;
   out_5595754603494457870[175] = 0.0;
   out_5595754603494457870[176] = 0.0;
   out_5595754603494457870[177] = 0.0;
   out_5595754603494457870[178] = 0.0;
   out_5595754603494457870[179] = 0.0;
   out_5595754603494457870[180] = 0.0;
   out_5595754603494457870[181] = 0.0;
   out_5595754603494457870[182] = 0.0;
   out_5595754603494457870[183] = 0.0;
   out_5595754603494457870[184] = 0.0;
   out_5595754603494457870[185] = 0.0;
   out_5595754603494457870[186] = 0.0;
   out_5595754603494457870[187] = 0.0;
   out_5595754603494457870[188] = 0.0;
   out_5595754603494457870[189] = 0.0;
   out_5595754603494457870[190] = 1.0;
   out_5595754603494457870[191] = 0.0;
   out_5595754603494457870[192] = 0.0;
   out_5595754603494457870[193] = 0.0;
   out_5595754603494457870[194] = 0.0;
   out_5595754603494457870[195] = 0.0;
   out_5595754603494457870[196] = 0.0;
   out_5595754603494457870[197] = 0.0;
   out_5595754603494457870[198] = 0.0;
   out_5595754603494457870[199] = 0.0;
   out_5595754603494457870[200] = 0.0;
   out_5595754603494457870[201] = 0.0;
   out_5595754603494457870[202] = 0.0;
   out_5595754603494457870[203] = 0.0;
   out_5595754603494457870[204] = 0.0;
   out_5595754603494457870[205] = 0.0;
   out_5595754603494457870[206] = 0.0;
   out_5595754603494457870[207] = 0.0;
   out_5595754603494457870[208] = 0.0;
   out_5595754603494457870[209] = 1.0;
   out_5595754603494457870[210] = 0.0;
   out_5595754603494457870[211] = 0.0;
   out_5595754603494457870[212] = 0.0;
   out_5595754603494457870[213] = 0.0;
   out_5595754603494457870[214] = 0.0;
   out_5595754603494457870[215] = 0.0;
   out_5595754603494457870[216] = 0.0;
   out_5595754603494457870[217] = 0.0;
   out_5595754603494457870[218] = 0.0;
   out_5595754603494457870[219] = 0.0;
   out_5595754603494457870[220] = 0.0;
   out_5595754603494457870[221] = 0.0;
   out_5595754603494457870[222] = 0.0;
   out_5595754603494457870[223] = 0.0;
   out_5595754603494457870[224] = 0.0;
   out_5595754603494457870[225] = 0.0;
   out_5595754603494457870[226] = 0.0;
   out_5595754603494457870[227] = 0.0;
   out_5595754603494457870[228] = 1.0;
   out_5595754603494457870[229] = 0.0;
   out_5595754603494457870[230] = 0.0;
   out_5595754603494457870[231] = 0.0;
   out_5595754603494457870[232] = 0.0;
   out_5595754603494457870[233] = 0.0;
   out_5595754603494457870[234] = 0.0;
   out_5595754603494457870[235] = 0.0;
   out_5595754603494457870[236] = 0.0;
   out_5595754603494457870[237] = 0.0;
   out_5595754603494457870[238] = 0.0;
   out_5595754603494457870[239] = 0.0;
   out_5595754603494457870[240] = 0.0;
   out_5595754603494457870[241] = 0.0;
   out_5595754603494457870[242] = 0.0;
   out_5595754603494457870[243] = 0.0;
   out_5595754603494457870[244] = 0.0;
   out_5595754603494457870[245] = 0.0;
   out_5595754603494457870[246] = 0.0;
   out_5595754603494457870[247] = 1.0;
   out_5595754603494457870[248] = 0.0;
   out_5595754603494457870[249] = 0.0;
   out_5595754603494457870[250] = 0.0;
   out_5595754603494457870[251] = 0.0;
   out_5595754603494457870[252] = 0.0;
   out_5595754603494457870[253] = 0.0;
   out_5595754603494457870[254] = 0.0;
   out_5595754603494457870[255] = 0.0;
   out_5595754603494457870[256] = 0.0;
   out_5595754603494457870[257] = 0.0;
   out_5595754603494457870[258] = 0.0;
   out_5595754603494457870[259] = 0.0;
   out_5595754603494457870[260] = 0.0;
   out_5595754603494457870[261] = 0.0;
   out_5595754603494457870[262] = 0.0;
   out_5595754603494457870[263] = 0.0;
   out_5595754603494457870[264] = 0.0;
   out_5595754603494457870[265] = 0.0;
   out_5595754603494457870[266] = 1.0;
   out_5595754603494457870[267] = 0.0;
   out_5595754603494457870[268] = 0.0;
   out_5595754603494457870[269] = 0.0;
   out_5595754603494457870[270] = 0.0;
   out_5595754603494457870[271] = 0.0;
   out_5595754603494457870[272] = 0.0;
   out_5595754603494457870[273] = 0.0;
   out_5595754603494457870[274] = 0.0;
   out_5595754603494457870[275] = 0.0;
   out_5595754603494457870[276] = 0.0;
   out_5595754603494457870[277] = 0.0;
   out_5595754603494457870[278] = 0.0;
   out_5595754603494457870[279] = 0.0;
   out_5595754603494457870[280] = 0.0;
   out_5595754603494457870[281] = 0.0;
   out_5595754603494457870[282] = 0.0;
   out_5595754603494457870[283] = 0.0;
   out_5595754603494457870[284] = 0.0;
   out_5595754603494457870[285] = 1.0;
   out_5595754603494457870[286] = 0.0;
   out_5595754603494457870[287] = 0.0;
   out_5595754603494457870[288] = 0.0;
   out_5595754603494457870[289] = 0.0;
   out_5595754603494457870[290] = 0.0;
   out_5595754603494457870[291] = 0.0;
   out_5595754603494457870[292] = 0.0;
   out_5595754603494457870[293] = 0.0;
   out_5595754603494457870[294] = 0.0;
   out_5595754603494457870[295] = 0.0;
   out_5595754603494457870[296] = 0.0;
   out_5595754603494457870[297] = 0.0;
   out_5595754603494457870[298] = 0.0;
   out_5595754603494457870[299] = 0.0;
   out_5595754603494457870[300] = 0.0;
   out_5595754603494457870[301] = 0.0;
   out_5595754603494457870[302] = 0.0;
   out_5595754603494457870[303] = 0.0;
   out_5595754603494457870[304] = 1.0;
   out_5595754603494457870[305] = 0.0;
   out_5595754603494457870[306] = 0.0;
   out_5595754603494457870[307] = 0.0;
   out_5595754603494457870[308] = 0.0;
   out_5595754603494457870[309] = 0.0;
   out_5595754603494457870[310] = 0.0;
   out_5595754603494457870[311] = 0.0;
   out_5595754603494457870[312] = 0.0;
   out_5595754603494457870[313] = 0.0;
   out_5595754603494457870[314] = 0.0;
   out_5595754603494457870[315] = 0.0;
   out_5595754603494457870[316] = 0.0;
   out_5595754603494457870[317] = 0.0;
   out_5595754603494457870[318] = 0.0;
   out_5595754603494457870[319] = 0.0;
   out_5595754603494457870[320] = 0.0;
   out_5595754603494457870[321] = 0.0;
   out_5595754603494457870[322] = 0.0;
   out_5595754603494457870[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3090951408168040086) {
   out_3090951408168040086[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3090951408168040086[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3090951408168040086[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3090951408168040086[3] = dt*state[12] + state[3];
   out_3090951408168040086[4] = dt*state[13] + state[4];
   out_3090951408168040086[5] = dt*state[14] + state[5];
   out_3090951408168040086[6] = state[6];
   out_3090951408168040086[7] = state[7];
   out_3090951408168040086[8] = state[8];
   out_3090951408168040086[9] = state[9];
   out_3090951408168040086[10] = state[10];
   out_3090951408168040086[11] = state[11];
   out_3090951408168040086[12] = state[12];
   out_3090951408168040086[13] = state[13];
   out_3090951408168040086[14] = state[14];
   out_3090951408168040086[15] = state[15];
   out_3090951408168040086[16] = state[16];
   out_3090951408168040086[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6851591457791860138) {
   out_6851591457791860138[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6851591457791860138[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6851591457791860138[2] = 0;
   out_6851591457791860138[3] = 0;
   out_6851591457791860138[4] = 0;
   out_6851591457791860138[5] = 0;
   out_6851591457791860138[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6851591457791860138[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6851591457791860138[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6851591457791860138[9] = 0;
   out_6851591457791860138[10] = 0;
   out_6851591457791860138[11] = 0;
   out_6851591457791860138[12] = 0;
   out_6851591457791860138[13] = 0;
   out_6851591457791860138[14] = 0;
   out_6851591457791860138[15] = 0;
   out_6851591457791860138[16] = 0;
   out_6851591457791860138[17] = 0;
   out_6851591457791860138[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6851591457791860138[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6851591457791860138[20] = 0;
   out_6851591457791860138[21] = 0;
   out_6851591457791860138[22] = 0;
   out_6851591457791860138[23] = 0;
   out_6851591457791860138[24] = 0;
   out_6851591457791860138[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6851591457791860138[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6851591457791860138[27] = 0;
   out_6851591457791860138[28] = 0;
   out_6851591457791860138[29] = 0;
   out_6851591457791860138[30] = 0;
   out_6851591457791860138[31] = 0;
   out_6851591457791860138[32] = 0;
   out_6851591457791860138[33] = 0;
   out_6851591457791860138[34] = 0;
   out_6851591457791860138[35] = 0;
   out_6851591457791860138[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6851591457791860138[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6851591457791860138[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6851591457791860138[39] = 0;
   out_6851591457791860138[40] = 0;
   out_6851591457791860138[41] = 0;
   out_6851591457791860138[42] = 0;
   out_6851591457791860138[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6851591457791860138[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6851591457791860138[45] = 0;
   out_6851591457791860138[46] = 0;
   out_6851591457791860138[47] = 0;
   out_6851591457791860138[48] = 0;
   out_6851591457791860138[49] = 0;
   out_6851591457791860138[50] = 0;
   out_6851591457791860138[51] = 0;
   out_6851591457791860138[52] = 0;
   out_6851591457791860138[53] = 0;
   out_6851591457791860138[54] = 0;
   out_6851591457791860138[55] = 0;
   out_6851591457791860138[56] = 0;
   out_6851591457791860138[57] = 1;
   out_6851591457791860138[58] = 0;
   out_6851591457791860138[59] = 0;
   out_6851591457791860138[60] = 0;
   out_6851591457791860138[61] = 0;
   out_6851591457791860138[62] = 0;
   out_6851591457791860138[63] = 0;
   out_6851591457791860138[64] = 0;
   out_6851591457791860138[65] = 0;
   out_6851591457791860138[66] = dt;
   out_6851591457791860138[67] = 0;
   out_6851591457791860138[68] = 0;
   out_6851591457791860138[69] = 0;
   out_6851591457791860138[70] = 0;
   out_6851591457791860138[71] = 0;
   out_6851591457791860138[72] = 0;
   out_6851591457791860138[73] = 0;
   out_6851591457791860138[74] = 0;
   out_6851591457791860138[75] = 0;
   out_6851591457791860138[76] = 1;
   out_6851591457791860138[77] = 0;
   out_6851591457791860138[78] = 0;
   out_6851591457791860138[79] = 0;
   out_6851591457791860138[80] = 0;
   out_6851591457791860138[81] = 0;
   out_6851591457791860138[82] = 0;
   out_6851591457791860138[83] = 0;
   out_6851591457791860138[84] = 0;
   out_6851591457791860138[85] = dt;
   out_6851591457791860138[86] = 0;
   out_6851591457791860138[87] = 0;
   out_6851591457791860138[88] = 0;
   out_6851591457791860138[89] = 0;
   out_6851591457791860138[90] = 0;
   out_6851591457791860138[91] = 0;
   out_6851591457791860138[92] = 0;
   out_6851591457791860138[93] = 0;
   out_6851591457791860138[94] = 0;
   out_6851591457791860138[95] = 1;
   out_6851591457791860138[96] = 0;
   out_6851591457791860138[97] = 0;
   out_6851591457791860138[98] = 0;
   out_6851591457791860138[99] = 0;
   out_6851591457791860138[100] = 0;
   out_6851591457791860138[101] = 0;
   out_6851591457791860138[102] = 0;
   out_6851591457791860138[103] = 0;
   out_6851591457791860138[104] = dt;
   out_6851591457791860138[105] = 0;
   out_6851591457791860138[106] = 0;
   out_6851591457791860138[107] = 0;
   out_6851591457791860138[108] = 0;
   out_6851591457791860138[109] = 0;
   out_6851591457791860138[110] = 0;
   out_6851591457791860138[111] = 0;
   out_6851591457791860138[112] = 0;
   out_6851591457791860138[113] = 0;
   out_6851591457791860138[114] = 1;
   out_6851591457791860138[115] = 0;
   out_6851591457791860138[116] = 0;
   out_6851591457791860138[117] = 0;
   out_6851591457791860138[118] = 0;
   out_6851591457791860138[119] = 0;
   out_6851591457791860138[120] = 0;
   out_6851591457791860138[121] = 0;
   out_6851591457791860138[122] = 0;
   out_6851591457791860138[123] = 0;
   out_6851591457791860138[124] = 0;
   out_6851591457791860138[125] = 0;
   out_6851591457791860138[126] = 0;
   out_6851591457791860138[127] = 0;
   out_6851591457791860138[128] = 0;
   out_6851591457791860138[129] = 0;
   out_6851591457791860138[130] = 0;
   out_6851591457791860138[131] = 0;
   out_6851591457791860138[132] = 0;
   out_6851591457791860138[133] = 1;
   out_6851591457791860138[134] = 0;
   out_6851591457791860138[135] = 0;
   out_6851591457791860138[136] = 0;
   out_6851591457791860138[137] = 0;
   out_6851591457791860138[138] = 0;
   out_6851591457791860138[139] = 0;
   out_6851591457791860138[140] = 0;
   out_6851591457791860138[141] = 0;
   out_6851591457791860138[142] = 0;
   out_6851591457791860138[143] = 0;
   out_6851591457791860138[144] = 0;
   out_6851591457791860138[145] = 0;
   out_6851591457791860138[146] = 0;
   out_6851591457791860138[147] = 0;
   out_6851591457791860138[148] = 0;
   out_6851591457791860138[149] = 0;
   out_6851591457791860138[150] = 0;
   out_6851591457791860138[151] = 0;
   out_6851591457791860138[152] = 1;
   out_6851591457791860138[153] = 0;
   out_6851591457791860138[154] = 0;
   out_6851591457791860138[155] = 0;
   out_6851591457791860138[156] = 0;
   out_6851591457791860138[157] = 0;
   out_6851591457791860138[158] = 0;
   out_6851591457791860138[159] = 0;
   out_6851591457791860138[160] = 0;
   out_6851591457791860138[161] = 0;
   out_6851591457791860138[162] = 0;
   out_6851591457791860138[163] = 0;
   out_6851591457791860138[164] = 0;
   out_6851591457791860138[165] = 0;
   out_6851591457791860138[166] = 0;
   out_6851591457791860138[167] = 0;
   out_6851591457791860138[168] = 0;
   out_6851591457791860138[169] = 0;
   out_6851591457791860138[170] = 0;
   out_6851591457791860138[171] = 1;
   out_6851591457791860138[172] = 0;
   out_6851591457791860138[173] = 0;
   out_6851591457791860138[174] = 0;
   out_6851591457791860138[175] = 0;
   out_6851591457791860138[176] = 0;
   out_6851591457791860138[177] = 0;
   out_6851591457791860138[178] = 0;
   out_6851591457791860138[179] = 0;
   out_6851591457791860138[180] = 0;
   out_6851591457791860138[181] = 0;
   out_6851591457791860138[182] = 0;
   out_6851591457791860138[183] = 0;
   out_6851591457791860138[184] = 0;
   out_6851591457791860138[185] = 0;
   out_6851591457791860138[186] = 0;
   out_6851591457791860138[187] = 0;
   out_6851591457791860138[188] = 0;
   out_6851591457791860138[189] = 0;
   out_6851591457791860138[190] = 1;
   out_6851591457791860138[191] = 0;
   out_6851591457791860138[192] = 0;
   out_6851591457791860138[193] = 0;
   out_6851591457791860138[194] = 0;
   out_6851591457791860138[195] = 0;
   out_6851591457791860138[196] = 0;
   out_6851591457791860138[197] = 0;
   out_6851591457791860138[198] = 0;
   out_6851591457791860138[199] = 0;
   out_6851591457791860138[200] = 0;
   out_6851591457791860138[201] = 0;
   out_6851591457791860138[202] = 0;
   out_6851591457791860138[203] = 0;
   out_6851591457791860138[204] = 0;
   out_6851591457791860138[205] = 0;
   out_6851591457791860138[206] = 0;
   out_6851591457791860138[207] = 0;
   out_6851591457791860138[208] = 0;
   out_6851591457791860138[209] = 1;
   out_6851591457791860138[210] = 0;
   out_6851591457791860138[211] = 0;
   out_6851591457791860138[212] = 0;
   out_6851591457791860138[213] = 0;
   out_6851591457791860138[214] = 0;
   out_6851591457791860138[215] = 0;
   out_6851591457791860138[216] = 0;
   out_6851591457791860138[217] = 0;
   out_6851591457791860138[218] = 0;
   out_6851591457791860138[219] = 0;
   out_6851591457791860138[220] = 0;
   out_6851591457791860138[221] = 0;
   out_6851591457791860138[222] = 0;
   out_6851591457791860138[223] = 0;
   out_6851591457791860138[224] = 0;
   out_6851591457791860138[225] = 0;
   out_6851591457791860138[226] = 0;
   out_6851591457791860138[227] = 0;
   out_6851591457791860138[228] = 1;
   out_6851591457791860138[229] = 0;
   out_6851591457791860138[230] = 0;
   out_6851591457791860138[231] = 0;
   out_6851591457791860138[232] = 0;
   out_6851591457791860138[233] = 0;
   out_6851591457791860138[234] = 0;
   out_6851591457791860138[235] = 0;
   out_6851591457791860138[236] = 0;
   out_6851591457791860138[237] = 0;
   out_6851591457791860138[238] = 0;
   out_6851591457791860138[239] = 0;
   out_6851591457791860138[240] = 0;
   out_6851591457791860138[241] = 0;
   out_6851591457791860138[242] = 0;
   out_6851591457791860138[243] = 0;
   out_6851591457791860138[244] = 0;
   out_6851591457791860138[245] = 0;
   out_6851591457791860138[246] = 0;
   out_6851591457791860138[247] = 1;
   out_6851591457791860138[248] = 0;
   out_6851591457791860138[249] = 0;
   out_6851591457791860138[250] = 0;
   out_6851591457791860138[251] = 0;
   out_6851591457791860138[252] = 0;
   out_6851591457791860138[253] = 0;
   out_6851591457791860138[254] = 0;
   out_6851591457791860138[255] = 0;
   out_6851591457791860138[256] = 0;
   out_6851591457791860138[257] = 0;
   out_6851591457791860138[258] = 0;
   out_6851591457791860138[259] = 0;
   out_6851591457791860138[260] = 0;
   out_6851591457791860138[261] = 0;
   out_6851591457791860138[262] = 0;
   out_6851591457791860138[263] = 0;
   out_6851591457791860138[264] = 0;
   out_6851591457791860138[265] = 0;
   out_6851591457791860138[266] = 1;
   out_6851591457791860138[267] = 0;
   out_6851591457791860138[268] = 0;
   out_6851591457791860138[269] = 0;
   out_6851591457791860138[270] = 0;
   out_6851591457791860138[271] = 0;
   out_6851591457791860138[272] = 0;
   out_6851591457791860138[273] = 0;
   out_6851591457791860138[274] = 0;
   out_6851591457791860138[275] = 0;
   out_6851591457791860138[276] = 0;
   out_6851591457791860138[277] = 0;
   out_6851591457791860138[278] = 0;
   out_6851591457791860138[279] = 0;
   out_6851591457791860138[280] = 0;
   out_6851591457791860138[281] = 0;
   out_6851591457791860138[282] = 0;
   out_6851591457791860138[283] = 0;
   out_6851591457791860138[284] = 0;
   out_6851591457791860138[285] = 1;
   out_6851591457791860138[286] = 0;
   out_6851591457791860138[287] = 0;
   out_6851591457791860138[288] = 0;
   out_6851591457791860138[289] = 0;
   out_6851591457791860138[290] = 0;
   out_6851591457791860138[291] = 0;
   out_6851591457791860138[292] = 0;
   out_6851591457791860138[293] = 0;
   out_6851591457791860138[294] = 0;
   out_6851591457791860138[295] = 0;
   out_6851591457791860138[296] = 0;
   out_6851591457791860138[297] = 0;
   out_6851591457791860138[298] = 0;
   out_6851591457791860138[299] = 0;
   out_6851591457791860138[300] = 0;
   out_6851591457791860138[301] = 0;
   out_6851591457791860138[302] = 0;
   out_6851591457791860138[303] = 0;
   out_6851591457791860138[304] = 1;
   out_6851591457791860138[305] = 0;
   out_6851591457791860138[306] = 0;
   out_6851591457791860138[307] = 0;
   out_6851591457791860138[308] = 0;
   out_6851591457791860138[309] = 0;
   out_6851591457791860138[310] = 0;
   out_6851591457791860138[311] = 0;
   out_6851591457791860138[312] = 0;
   out_6851591457791860138[313] = 0;
   out_6851591457791860138[314] = 0;
   out_6851591457791860138[315] = 0;
   out_6851591457791860138[316] = 0;
   out_6851591457791860138[317] = 0;
   out_6851591457791860138[318] = 0;
   out_6851591457791860138[319] = 0;
   out_6851591457791860138[320] = 0;
   out_6851591457791860138[321] = 0;
   out_6851591457791860138[322] = 0;
   out_6851591457791860138[323] = 1;
}
void h_4(double *state, double *unused, double *out_6888167971702813252) {
   out_6888167971702813252[0] = state[6] + state[9];
   out_6888167971702813252[1] = state[7] + state[10];
   out_6888167971702813252[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_443236218455377135) {
   out_443236218455377135[0] = 0;
   out_443236218455377135[1] = 0;
   out_443236218455377135[2] = 0;
   out_443236218455377135[3] = 0;
   out_443236218455377135[4] = 0;
   out_443236218455377135[5] = 0;
   out_443236218455377135[6] = 1;
   out_443236218455377135[7] = 0;
   out_443236218455377135[8] = 0;
   out_443236218455377135[9] = 1;
   out_443236218455377135[10] = 0;
   out_443236218455377135[11] = 0;
   out_443236218455377135[12] = 0;
   out_443236218455377135[13] = 0;
   out_443236218455377135[14] = 0;
   out_443236218455377135[15] = 0;
   out_443236218455377135[16] = 0;
   out_443236218455377135[17] = 0;
   out_443236218455377135[18] = 0;
   out_443236218455377135[19] = 0;
   out_443236218455377135[20] = 0;
   out_443236218455377135[21] = 0;
   out_443236218455377135[22] = 0;
   out_443236218455377135[23] = 0;
   out_443236218455377135[24] = 0;
   out_443236218455377135[25] = 1;
   out_443236218455377135[26] = 0;
   out_443236218455377135[27] = 0;
   out_443236218455377135[28] = 1;
   out_443236218455377135[29] = 0;
   out_443236218455377135[30] = 0;
   out_443236218455377135[31] = 0;
   out_443236218455377135[32] = 0;
   out_443236218455377135[33] = 0;
   out_443236218455377135[34] = 0;
   out_443236218455377135[35] = 0;
   out_443236218455377135[36] = 0;
   out_443236218455377135[37] = 0;
   out_443236218455377135[38] = 0;
   out_443236218455377135[39] = 0;
   out_443236218455377135[40] = 0;
   out_443236218455377135[41] = 0;
   out_443236218455377135[42] = 0;
   out_443236218455377135[43] = 0;
   out_443236218455377135[44] = 1;
   out_443236218455377135[45] = 0;
   out_443236218455377135[46] = 0;
   out_443236218455377135[47] = 1;
   out_443236218455377135[48] = 0;
   out_443236218455377135[49] = 0;
   out_443236218455377135[50] = 0;
   out_443236218455377135[51] = 0;
   out_443236218455377135[52] = 0;
   out_443236218455377135[53] = 0;
}
void h_10(double *state, double *unused, double *out_2926285046309371719) {
   out_2926285046309371719[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2926285046309371719[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2926285046309371719[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_7083396780515160844) {
   out_7083396780515160844[0] = 0;
   out_7083396780515160844[1] = 9.8100000000000005*cos(state[1]);
   out_7083396780515160844[2] = 0;
   out_7083396780515160844[3] = 0;
   out_7083396780515160844[4] = -state[8];
   out_7083396780515160844[5] = state[7];
   out_7083396780515160844[6] = 0;
   out_7083396780515160844[7] = state[5];
   out_7083396780515160844[8] = -state[4];
   out_7083396780515160844[9] = 0;
   out_7083396780515160844[10] = 0;
   out_7083396780515160844[11] = 0;
   out_7083396780515160844[12] = 1;
   out_7083396780515160844[13] = 0;
   out_7083396780515160844[14] = 0;
   out_7083396780515160844[15] = 1;
   out_7083396780515160844[16] = 0;
   out_7083396780515160844[17] = 0;
   out_7083396780515160844[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_7083396780515160844[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_7083396780515160844[20] = 0;
   out_7083396780515160844[21] = state[8];
   out_7083396780515160844[22] = 0;
   out_7083396780515160844[23] = -state[6];
   out_7083396780515160844[24] = -state[5];
   out_7083396780515160844[25] = 0;
   out_7083396780515160844[26] = state[3];
   out_7083396780515160844[27] = 0;
   out_7083396780515160844[28] = 0;
   out_7083396780515160844[29] = 0;
   out_7083396780515160844[30] = 0;
   out_7083396780515160844[31] = 1;
   out_7083396780515160844[32] = 0;
   out_7083396780515160844[33] = 0;
   out_7083396780515160844[34] = 1;
   out_7083396780515160844[35] = 0;
   out_7083396780515160844[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_7083396780515160844[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_7083396780515160844[38] = 0;
   out_7083396780515160844[39] = -state[7];
   out_7083396780515160844[40] = state[6];
   out_7083396780515160844[41] = 0;
   out_7083396780515160844[42] = state[4];
   out_7083396780515160844[43] = -state[3];
   out_7083396780515160844[44] = 0;
   out_7083396780515160844[45] = 0;
   out_7083396780515160844[46] = 0;
   out_7083396780515160844[47] = 0;
   out_7083396780515160844[48] = 0;
   out_7083396780515160844[49] = 0;
   out_7083396780515160844[50] = 1;
   out_7083396780515160844[51] = 0;
   out_7083396780515160844[52] = 0;
   out_7083396780515160844[53] = 1;
}
void h_13(double *state, double *unused, double *out_101780861794540917) {
   out_101780861794540917[0] = state[3];
   out_101780861794540917[1] = state[4];
   out_101780861794540917[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2769037606876955666) {
   out_2769037606876955666[0] = 0;
   out_2769037606876955666[1] = 0;
   out_2769037606876955666[2] = 0;
   out_2769037606876955666[3] = 1;
   out_2769037606876955666[4] = 0;
   out_2769037606876955666[5] = 0;
   out_2769037606876955666[6] = 0;
   out_2769037606876955666[7] = 0;
   out_2769037606876955666[8] = 0;
   out_2769037606876955666[9] = 0;
   out_2769037606876955666[10] = 0;
   out_2769037606876955666[11] = 0;
   out_2769037606876955666[12] = 0;
   out_2769037606876955666[13] = 0;
   out_2769037606876955666[14] = 0;
   out_2769037606876955666[15] = 0;
   out_2769037606876955666[16] = 0;
   out_2769037606876955666[17] = 0;
   out_2769037606876955666[18] = 0;
   out_2769037606876955666[19] = 0;
   out_2769037606876955666[20] = 0;
   out_2769037606876955666[21] = 0;
   out_2769037606876955666[22] = 1;
   out_2769037606876955666[23] = 0;
   out_2769037606876955666[24] = 0;
   out_2769037606876955666[25] = 0;
   out_2769037606876955666[26] = 0;
   out_2769037606876955666[27] = 0;
   out_2769037606876955666[28] = 0;
   out_2769037606876955666[29] = 0;
   out_2769037606876955666[30] = 0;
   out_2769037606876955666[31] = 0;
   out_2769037606876955666[32] = 0;
   out_2769037606876955666[33] = 0;
   out_2769037606876955666[34] = 0;
   out_2769037606876955666[35] = 0;
   out_2769037606876955666[36] = 0;
   out_2769037606876955666[37] = 0;
   out_2769037606876955666[38] = 0;
   out_2769037606876955666[39] = 0;
   out_2769037606876955666[40] = 0;
   out_2769037606876955666[41] = 1;
   out_2769037606876955666[42] = 0;
   out_2769037606876955666[43] = 0;
   out_2769037606876955666[44] = 0;
   out_2769037606876955666[45] = 0;
   out_2769037606876955666[46] = 0;
   out_2769037606876955666[47] = 0;
   out_2769037606876955666[48] = 0;
   out_2769037606876955666[49] = 0;
   out_2769037606876955666[50] = 0;
   out_2769037606876955666[51] = 0;
   out_2769037606876955666[52] = 0;
   out_2769037606876955666[53] = 0;
}
void h_14(double *state, double *unused, double *out_616030427240878133) {
   out_616030427240878133[0] = state[6];
   out_616030427240878133[1] = state[7];
   out_616030427240878133[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3526024650750749431) {
   out_3526024650750749431[0] = 0;
   out_3526024650750749431[1] = 0;
   out_3526024650750749431[2] = 0;
   out_3526024650750749431[3] = 0;
   out_3526024650750749431[4] = 0;
   out_3526024650750749431[5] = 0;
   out_3526024650750749431[6] = 1;
   out_3526024650750749431[7] = 0;
   out_3526024650750749431[8] = 0;
   out_3526024650750749431[9] = 0;
   out_3526024650750749431[10] = 0;
   out_3526024650750749431[11] = 0;
   out_3526024650750749431[12] = 0;
   out_3526024650750749431[13] = 0;
   out_3526024650750749431[14] = 0;
   out_3526024650750749431[15] = 0;
   out_3526024650750749431[16] = 0;
   out_3526024650750749431[17] = 0;
   out_3526024650750749431[18] = 0;
   out_3526024650750749431[19] = 0;
   out_3526024650750749431[20] = 0;
   out_3526024650750749431[21] = 0;
   out_3526024650750749431[22] = 0;
   out_3526024650750749431[23] = 0;
   out_3526024650750749431[24] = 0;
   out_3526024650750749431[25] = 1;
   out_3526024650750749431[26] = 0;
   out_3526024650750749431[27] = 0;
   out_3526024650750749431[28] = 0;
   out_3526024650750749431[29] = 0;
   out_3526024650750749431[30] = 0;
   out_3526024650750749431[31] = 0;
   out_3526024650750749431[32] = 0;
   out_3526024650750749431[33] = 0;
   out_3526024650750749431[34] = 0;
   out_3526024650750749431[35] = 0;
   out_3526024650750749431[36] = 0;
   out_3526024650750749431[37] = 0;
   out_3526024650750749431[38] = 0;
   out_3526024650750749431[39] = 0;
   out_3526024650750749431[40] = 0;
   out_3526024650750749431[41] = 0;
   out_3526024650750749431[42] = 0;
   out_3526024650750749431[43] = 0;
   out_3526024650750749431[44] = 1;
   out_3526024650750749431[45] = 0;
   out_3526024650750749431[46] = 0;
   out_3526024650750749431[47] = 0;
   out_3526024650750749431[48] = 0;
   out_3526024650750749431[49] = 0;
   out_3526024650750749431[50] = 0;
   out_3526024650750749431[51] = 0;
   out_3526024650750749431[52] = 0;
   out_3526024650750749431[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_1862087488242639687) {
  err_fun(nom_x, delta_x, out_1862087488242639687);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7512207833057071994) {
  inv_err_fun(nom_x, true_x, out_7512207833057071994);
}
void pose_H_mod_fun(double *state, double *out_5595754603494457870) {
  H_mod_fun(state, out_5595754603494457870);
}
void pose_f_fun(double *state, double dt, double *out_3090951408168040086) {
  f_fun(state,  dt, out_3090951408168040086);
}
void pose_F_fun(double *state, double dt, double *out_6851591457791860138) {
  F_fun(state,  dt, out_6851591457791860138);
}
void pose_h_4(double *state, double *unused, double *out_6888167971702813252) {
  h_4(state, unused, out_6888167971702813252);
}
void pose_H_4(double *state, double *unused, double *out_443236218455377135) {
  H_4(state, unused, out_443236218455377135);
}
void pose_h_10(double *state, double *unused, double *out_2926285046309371719) {
  h_10(state, unused, out_2926285046309371719);
}
void pose_H_10(double *state, double *unused, double *out_7083396780515160844) {
  H_10(state, unused, out_7083396780515160844);
}
void pose_h_13(double *state, double *unused, double *out_101780861794540917) {
  h_13(state, unused, out_101780861794540917);
}
void pose_H_13(double *state, double *unused, double *out_2769037606876955666) {
  H_13(state, unused, out_2769037606876955666);
}
void pose_h_14(double *state, double *unused, double *out_616030427240878133) {
  h_14(state, unused, out_616030427240878133);
}
void pose_H_14(double *state, double *unused, double *out_3526024650750749431) {
  H_14(state, unused, out_3526024650750749431);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
