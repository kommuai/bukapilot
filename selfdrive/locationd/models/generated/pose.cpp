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
void err_fun(double *nom_x, double *delta_x, double *out_4430958223127505081) {
   out_4430958223127505081[0] = delta_x[0] + nom_x[0];
   out_4430958223127505081[1] = delta_x[1] + nom_x[1];
   out_4430958223127505081[2] = delta_x[2] + nom_x[2];
   out_4430958223127505081[3] = delta_x[3] + nom_x[3];
   out_4430958223127505081[4] = delta_x[4] + nom_x[4];
   out_4430958223127505081[5] = delta_x[5] + nom_x[5];
   out_4430958223127505081[6] = delta_x[6] + nom_x[6];
   out_4430958223127505081[7] = delta_x[7] + nom_x[7];
   out_4430958223127505081[8] = delta_x[8] + nom_x[8];
   out_4430958223127505081[9] = delta_x[9] + nom_x[9];
   out_4430958223127505081[10] = delta_x[10] + nom_x[10];
   out_4430958223127505081[11] = delta_x[11] + nom_x[11];
   out_4430958223127505081[12] = delta_x[12] + nom_x[12];
   out_4430958223127505081[13] = delta_x[13] + nom_x[13];
   out_4430958223127505081[14] = delta_x[14] + nom_x[14];
   out_4430958223127505081[15] = delta_x[15] + nom_x[15];
   out_4430958223127505081[16] = delta_x[16] + nom_x[16];
   out_4430958223127505081[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4290768674008276673) {
   out_4290768674008276673[0] = -nom_x[0] + true_x[0];
   out_4290768674008276673[1] = -nom_x[1] + true_x[1];
   out_4290768674008276673[2] = -nom_x[2] + true_x[2];
   out_4290768674008276673[3] = -nom_x[3] + true_x[3];
   out_4290768674008276673[4] = -nom_x[4] + true_x[4];
   out_4290768674008276673[5] = -nom_x[5] + true_x[5];
   out_4290768674008276673[6] = -nom_x[6] + true_x[6];
   out_4290768674008276673[7] = -nom_x[7] + true_x[7];
   out_4290768674008276673[8] = -nom_x[8] + true_x[8];
   out_4290768674008276673[9] = -nom_x[9] + true_x[9];
   out_4290768674008276673[10] = -nom_x[10] + true_x[10];
   out_4290768674008276673[11] = -nom_x[11] + true_x[11];
   out_4290768674008276673[12] = -nom_x[12] + true_x[12];
   out_4290768674008276673[13] = -nom_x[13] + true_x[13];
   out_4290768674008276673[14] = -nom_x[14] + true_x[14];
   out_4290768674008276673[15] = -nom_x[15] + true_x[15];
   out_4290768674008276673[16] = -nom_x[16] + true_x[16];
   out_4290768674008276673[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3054748865441164755) {
   out_3054748865441164755[0] = 1.0;
   out_3054748865441164755[1] = 0.0;
   out_3054748865441164755[2] = 0.0;
   out_3054748865441164755[3] = 0.0;
   out_3054748865441164755[4] = 0.0;
   out_3054748865441164755[5] = 0.0;
   out_3054748865441164755[6] = 0.0;
   out_3054748865441164755[7] = 0.0;
   out_3054748865441164755[8] = 0.0;
   out_3054748865441164755[9] = 0.0;
   out_3054748865441164755[10] = 0.0;
   out_3054748865441164755[11] = 0.0;
   out_3054748865441164755[12] = 0.0;
   out_3054748865441164755[13] = 0.0;
   out_3054748865441164755[14] = 0.0;
   out_3054748865441164755[15] = 0.0;
   out_3054748865441164755[16] = 0.0;
   out_3054748865441164755[17] = 0.0;
   out_3054748865441164755[18] = 0.0;
   out_3054748865441164755[19] = 1.0;
   out_3054748865441164755[20] = 0.0;
   out_3054748865441164755[21] = 0.0;
   out_3054748865441164755[22] = 0.0;
   out_3054748865441164755[23] = 0.0;
   out_3054748865441164755[24] = 0.0;
   out_3054748865441164755[25] = 0.0;
   out_3054748865441164755[26] = 0.0;
   out_3054748865441164755[27] = 0.0;
   out_3054748865441164755[28] = 0.0;
   out_3054748865441164755[29] = 0.0;
   out_3054748865441164755[30] = 0.0;
   out_3054748865441164755[31] = 0.0;
   out_3054748865441164755[32] = 0.0;
   out_3054748865441164755[33] = 0.0;
   out_3054748865441164755[34] = 0.0;
   out_3054748865441164755[35] = 0.0;
   out_3054748865441164755[36] = 0.0;
   out_3054748865441164755[37] = 0.0;
   out_3054748865441164755[38] = 1.0;
   out_3054748865441164755[39] = 0.0;
   out_3054748865441164755[40] = 0.0;
   out_3054748865441164755[41] = 0.0;
   out_3054748865441164755[42] = 0.0;
   out_3054748865441164755[43] = 0.0;
   out_3054748865441164755[44] = 0.0;
   out_3054748865441164755[45] = 0.0;
   out_3054748865441164755[46] = 0.0;
   out_3054748865441164755[47] = 0.0;
   out_3054748865441164755[48] = 0.0;
   out_3054748865441164755[49] = 0.0;
   out_3054748865441164755[50] = 0.0;
   out_3054748865441164755[51] = 0.0;
   out_3054748865441164755[52] = 0.0;
   out_3054748865441164755[53] = 0.0;
   out_3054748865441164755[54] = 0.0;
   out_3054748865441164755[55] = 0.0;
   out_3054748865441164755[56] = 0.0;
   out_3054748865441164755[57] = 1.0;
   out_3054748865441164755[58] = 0.0;
   out_3054748865441164755[59] = 0.0;
   out_3054748865441164755[60] = 0.0;
   out_3054748865441164755[61] = 0.0;
   out_3054748865441164755[62] = 0.0;
   out_3054748865441164755[63] = 0.0;
   out_3054748865441164755[64] = 0.0;
   out_3054748865441164755[65] = 0.0;
   out_3054748865441164755[66] = 0.0;
   out_3054748865441164755[67] = 0.0;
   out_3054748865441164755[68] = 0.0;
   out_3054748865441164755[69] = 0.0;
   out_3054748865441164755[70] = 0.0;
   out_3054748865441164755[71] = 0.0;
   out_3054748865441164755[72] = 0.0;
   out_3054748865441164755[73] = 0.0;
   out_3054748865441164755[74] = 0.0;
   out_3054748865441164755[75] = 0.0;
   out_3054748865441164755[76] = 1.0;
   out_3054748865441164755[77] = 0.0;
   out_3054748865441164755[78] = 0.0;
   out_3054748865441164755[79] = 0.0;
   out_3054748865441164755[80] = 0.0;
   out_3054748865441164755[81] = 0.0;
   out_3054748865441164755[82] = 0.0;
   out_3054748865441164755[83] = 0.0;
   out_3054748865441164755[84] = 0.0;
   out_3054748865441164755[85] = 0.0;
   out_3054748865441164755[86] = 0.0;
   out_3054748865441164755[87] = 0.0;
   out_3054748865441164755[88] = 0.0;
   out_3054748865441164755[89] = 0.0;
   out_3054748865441164755[90] = 0.0;
   out_3054748865441164755[91] = 0.0;
   out_3054748865441164755[92] = 0.0;
   out_3054748865441164755[93] = 0.0;
   out_3054748865441164755[94] = 0.0;
   out_3054748865441164755[95] = 1.0;
   out_3054748865441164755[96] = 0.0;
   out_3054748865441164755[97] = 0.0;
   out_3054748865441164755[98] = 0.0;
   out_3054748865441164755[99] = 0.0;
   out_3054748865441164755[100] = 0.0;
   out_3054748865441164755[101] = 0.0;
   out_3054748865441164755[102] = 0.0;
   out_3054748865441164755[103] = 0.0;
   out_3054748865441164755[104] = 0.0;
   out_3054748865441164755[105] = 0.0;
   out_3054748865441164755[106] = 0.0;
   out_3054748865441164755[107] = 0.0;
   out_3054748865441164755[108] = 0.0;
   out_3054748865441164755[109] = 0.0;
   out_3054748865441164755[110] = 0.0;
   out_3054748865441164755[111] = 0.0;
   out_3054748865441164755[112] = 0.0;
   out_3054748865441164755[113] = 0.0;
   out_3054748865441164755[114] = 1.0;
   out_3054748865441164755[115] = 0.0;
   out_3054748865441164755[116] = 0.0;
   out_3054748865441164755[117] = 0.0;
   out_3054748865441164755[118] = 0.0;
   out_3054748865441164755[119] = 0.0;
   out_3054748865441164755[120] = 0.0;
   out_3054748865441164755[121] = 0.0;
   out_3054748865441164755[122] = 0.0;
   out_3054748865441164755[123] = 0.0;
   out_3054748865441164755[124] = 0.0;
   out_3054748865441164755[125] = 0.0;
   out_3054748865441164755[126] = 0.0;
   out_3054748865441164755[127] = 0.0;
   out_3054748865441164755[128] = 0.0;
   out_3054748865441164755[129] = 0.0;
   out_3054748865441164755[130] = 0.0;
   out_3054748865441164755[131] = 0.0;
   out_3054748865441164755[132] = 0.0;
   out_3054748865441164755[133] = 1.0;
   out_3054748865441164755[134] = 0.0;
   out_3054748865441164755[135] = 0.0;
   out_3054748865441164755[136] = 0.0;
   out_3054748865441164755[137] = 0.0;
   out_3054748865441164755[138] = 0.0;
   out_3054748865441164755[139] = 0.0;
   out_3054748865441164755[140] = 0.0;
   out_3054748865441164755[141] = 0.0;
   out_3054748865441164755[142] = 0.0;
   out_3054748865441164755[143] = 0.0;
   out_3054748865441164755[144] = 0.0;
   out_3054748865441164755[145] = 0.0;
   out_3054748865441164755[146] = 0.0;
   out_3054748865441164755[147] = 0.0;
   out_3054748865441164755[148] = 0.0;
   out_3054748865441164755[149] = 0.0;
   out_3054748865441164755[150] = 0.0;
   out_3054748865441164755[151] = 0.0;
   out_3054748865441164755[152] = 1.0;
   out_3054748865441164755[153] = 0.0;
   out_3054748865441164755[154] = 0.0;
   out_3054748865441164755[155] = 0.0;
   out_3054748865441164755[156] = 0.0;
   out_3054748865441164755[157] = 0.0;
   out_3054748865441164755[158] = 0.0;
   out_3054748865441164755[159] = 0.0;
   out_3054748865441164755[160] = 0.0;
   out_3054748865441164755[161] = 0.0;
   out_3054748865441164755[162] = 0.0;
   out_3054748865441164755[163] = 0.0;
   out_3054748865441164755[164] = 0.0;
   out_3054748865441164755[165] = 0.0;
   out_3054748865441164755[166] = 0.0;
   out_3054748865441164755[167] = 0.0;
   out_3054748865441164755[168] = 0.0;
   out_3054748865441164755[169] = 0.0;
   out_3054748865441164755[170] = 0.0;
   out_3054748865441164755[171] = 1.0;
   out_3054748865441164755[172] = 0.0;
   out_3054748865441164755[173] = 0.0;
   out_3054748865441164755[174] = 0.0;
   out_3054748865441164755[175] = 0.0;
   out_3054748865441164755[176] = 0.0;
   out_3054748865441164755[177] = 0.0;
   out_3054748865441164755[178] = 0.0;
   out_3054748865441164755[179] = 0.0;
   out_3054748865441164755[180] = 0.0;
   out_3054748865441164755[181] = 0.0;
   out_3054748865441164755[182] = 0.0;
   out_3054748865441164755[183] = 0.0;
   out_3054748865441164755[184] = 0.0;
   out_3054748865441164755[185] = 0.0;
   out_3054748865441164755[186] = 0.0;
   out_3054748865441164755[187] = 0.0;
   out_3054748865441164755[188] = 0.0;
   out_3054748865441164755[189] = 0.0;
   out_3054748865441164755[190] = 1.0;
   out_3054748865441164755[191] = 0.0;
   out_3054748865441164755[192] = 0.0;
   out_3054748865441164755[193] = 0.0;
   out_3054748865441164755[194] = 0.0;
   out_3054748865441164755[195] = 0.0;
   out_3054748865441164755[196] = 0.0;
   out_3054748865441164755[197] = 0.0;
   out_3054748865441164755[198] = 0.0;
   out_3054748865441164755[199] = 0.0;
   out_3054748865441164755[200] = 0.0;
   out_3054748865441164755[201] = 0.0;
   out_3054748865441164755[202] = 0.0;
   out_3054748865441164755[203] = 0.0;
   out_3054748865441164755[204] = 0.0;
   out_3054748865441164755[205] = 0.0;
   out_3054748865441164755[206] = 0.0;
   out_3054748865441164755[207] = 0.0;
   out_3054748865441164755[208] = 0.0;
   out_3054748865441164755[209] = 1.0;
   out_3054748865441164755[210] = 0.0;
   out_3054748865441164755[211] = 0.0;
   out_3054748865441164755[212] = 0.0;
   out_3054748865441164755[213] = 0.0;
   out_3054748865441164755[214] = 0.0;
   out_3054748865441164755[215] = 0.0;
   out_3054748865441164755[216] = 0.0;
   out_3054748865441164755[217] = 0.0;
   out_3054748865441164755[218] = 0.0;
   out_3054748865441164755[219] = 0.0;
   out_3054748865441164755[220] = 0.0;
   out_3054748865441164755[221] = 0.0;
   out_3054748865441164755[222] = 0.0;
   out_3054748865441164755[223] = 0.0;
   out_3054748865441164755[224] = 0.0;
   out_3054748865441164755[225] = 0.0;
   out_3054748865441164755[226] = 0.0;
   out_3054748865441164755[227] = 0.0;
   out_3054748865441164755[228] = 1.0;
   out_3054748865441164755[229] = 0.0;
   out_3054748865441164755[230] = 0.0;
   out_3054748865441164755[231] = 0.0;
   out_3054748865441164755[232] = 0.0;
   out_3054748865441164755[233] = 0.0;
   out_3054748865441164755[234] = 0.0;
   out_3054748865441164755[235] = 0.0;
   out_3054748865441164755[236] = 0.0;
   out_3054748865441164755[237] = 0.0;
   out_3054748865441164755[238] = 0.0;
   out_3054748865441164755[239] = 0.0;
   out_3054748865441164755[240] = 0.0;
   out_3054748865441164755[241] = 0.0;
   out_3054748865441164755[242] = 0.0;
   out_3054748865441164755[243] = 0.0;
   out_3054748865441164755[244] = 0.0;
   out_3054748865441164755[245] = 0.0;
   out_3054748865441164755[246] = 0.0;
   out_3054748865441164755[247] = 1.0;
   out_3054748865441164755[248] = 0.0;
   out_3054748865441164755[249] = 0.0;
   out_3054748865441164755[250] = 0.0;
   out_3054748865441164755[251] = 0.0;
   out_3054748865441164755[252] = 0.0;
   out_3054748865441164755[253] = 0.0;
   out_3054748865441164755[254] = 0.0;
   out_3054748865441164755[255] = 0.0;
   out_3054748865441164755[256] = 0.0;
   out_3054748865441164755[257] = 0.0;
   out_3054748865441164755[258] = 0.0;
   out_3054748865441164755[259] = 0.0;
   out_3054748865441164755[260] = 0.0;
   out_3054748865441164755[261] = 0.0;
   out_3054748865441164755[262] = 0.0;
   out_3054748865441164755[263] = 0.0;
   out_3054748865441164755[264] = 0.0;
   out_3054748865441164755[265] = 0.0;
   out_3054748865441164755[266] = 1.0;
   out_3054748865441164755[267] = 0.0;
   out_3054748865441164755[268] = 0.0;
   out_3054748865441164755[269] = 0.0;
   out_3054748865441164755[270] = 0.0;
   out_3054748865441164755[271] = 0.0;
   out_3054748865441164755[272] = 0.0;
   out_3054748865441164755[273] = 0.0;
   out_3054748865441164755[274] = 0.0;
   out_3054748865441164755[275] = 0.0;
   out_3054748865441164755[276] = 0.0;
   out_3054748865441164755[277] = 0.0;
   out_3054748865441164755[278] = 0.0;
   out_3054748865441164755[279] = 0.0;
   out_3054748865441164755[280] = 0.0;
   out_3054748865441164755[281] = 0.0;
   out_3054748865441164755[282] = 0.0;
   out_3054748865441164755[283] = 0.0;
   out_3054748865441164755[284] = 0.0;
   out_3054748865441164755[285] = 1.0;
   out_3054748865441164755[286] = 0.0;
   out_3054748865441164755[287] = 0.0;
   out_3054748865441164755[288] = 0.0;
   out_3054748865441164755[289] = 0.0;
   out_3054748865441164755[290] = 0.0;
   out_3054748865441164755[291] = 0.0;
   out_3054748865441164755[292] = 0.0;
   out_3054748865441164755[293] = 0.0;
   out_3054748865441164755[294] = 0.0;
   out_3054748865441164755[295] = 0.0;
   out_3054748865441164755[296] = 0.0;
   out_3054748865441164755[297] = 0.0;
   out_3054748865441164755[298] = 0.0;
   out_3054748865441164755[299] = 0.0;
   out_3054748865441164755[300] = 0.0;
   out_3054748865441164755[301] = 0.0;
   out_3054748865441164755[302] = 0.0;
   out_3054748865441164755[303] = 0.0;
   out_3054748865441164755[304] = 1.0;
   out_3054748865441164755[305] = 0.0;
   out_3054748865441164755[306] = 0.0;
   out_3054748865441164755[307] = 0.0;
   out_3054748865441164755[308] = 0.0;
   out_3054748865441164755[309] = 0.0;
   out_3054748865441164755[310] = 0.0;
   out_3054748865441164755[311] = 0.0;
   out_3054748865441164755[312] = 0.0;
   out_3054748865441164755[313] = 0.0;
   out_3054748865441164755[314] = 0.0;
   out_3054748865441164755[315] = 0.0;
   out_3054748865441164755[316] = 0.0;
   out_3054748865441164755[317] = 0.0;
   out_3054748865441164755[318] = 0.0;
   out_3054748865441164755[319] = 0.0;
   out_3054748865441164755[320] = 0.0;
   out_3054748865441164755[321] = 0.0;
   out_3054748865441164755[322] = 0.0;
   out_3054748865441164755[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_717525079663500723) {
   out_717525079663500723[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_717525079663500723[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_717525079663500723[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_717525079663500723[3] = dt*state[12] + state[3];
   out_717525079663500723[4] = dt*state[13] + state[4];
   out_717525079663500723[5] = dt*state[14] + state[5];
   out_717525079663500723[6] = state[6];
   out_717525079663500723[7] = state[7];
   out_717525079663500723[8] = state[8];
   out_717525079663500723[9] = state[9];
   out_717525079663500723[10] = state[10];
   out_717525079663500723[11] = state[11];
   out_717525079663500723[12] = state[12];
   out_717525079663500723[13] = state[13];
   out_717525079663500723[14] = state[14];
   out_717525079663500723[15] = state[15];
   out_717525079663500723[16] = state[16];
   out_717525079663500723[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8758534082296255942) {
   out_8758534082296255942[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8758534082296255942[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8758534082296255942[2] = 0;
   out_8758534082296255942[3] = 0;
   out_8758534082296255942[4] = 0;
   out_8758534082296255942[5] = 0;
   out_8758534082296255942[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8758534082296255942[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8758534082296255942[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8758534082296255942[9] = 0;
   out_8758534082296255942[10] = 0;
   out_8758534082296255942[11] = 0;
   out_8758534082296255942[12] = 0;
   out_8758534082296255942[13] = 0;
   out_8758534082296255942[14] = 0;
   out_8758534082296255942[15] = 0;
   out_8758534082296255942[16] = 0;
   out_8758534082296255942[17] = 0;
   out_8758534082296255942[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8758534082296255942[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8758534082296255942[20] = 0;
   out_8758534082296255942[21] = 0;
   out_8758534082296255942[22] = 0;
   out_8758534082296255942[23] = 0;
   out_8758534082296255942[24] = 0;
   out_8758534082296255942[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8758534082296255942[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8758534082296255942[27] = 0;
   out_8758534082296255942[28] = 0;
   out_8758534082296255942[29] = 0;
   out_8758534082296255942[30] = 0;
   out_8758534082296255942[31] = 0;
   out_8758534082296255942[32] = 0;
   out_8758534082296255942[33] = 0;
   out_8758534082296255942[34] = 0;
   out_8758534082296255942[35] = 0;
   out_8758534082296255942[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8758534082296255942[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8758534082296255942[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8758534082296255942[39] = 0;
   out_8758534082296255942[40] = 0;
   out_8758534082296255942[41] = 0;
   out_8758534082296255942[42] = 0;
   out_8758534082296255942[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8758534082296255942[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8758534082296255942[45] = 0;
   out_8758534082296255942[46] = 0;
   out_8758534082296255942[47] = 0;
   out_8758534082296255942[48] = 0;
   out_8758534082296255942[49] = 0;
   out_8758534082296255942[50] = 0;
   out_8758534082296255942[51] = 0;
   out_8758534082296255942[52] = 0;
   out_8758534082296255942[53] = 0;
   out_8758534082296255942[54] = 0;
   out_8758534082296255942[55] = 0;
   out_8758534082296255942[56] = 0;
   out_8758534082296255942[57] = 1;
   out_8758534082296255942[58] = 0;
   out_8758534082296255942[59] = 0;
   out_8758534082296255942[60] = 0;
   out_8758534082296255942[61] = 0;
   out_8758534082296255942[62] = 0;
   out_8758534082296255942[63] = 0;
   out_8758534082296255942[64] = 0;
   out_8758534082296255942[65] = 0;
   out_8758534082296255942[66] = dt;
   out_8758534082296255942[67] = 0;
   out_8758534082296255942[68] = 0;
   out_8758534082296255942[69] = 0;
   out_8758534082296255942[70] = 0;
   out_8758534082296255942[71] = 0;
   out_8758534082296255942[72] = 0;
   out_8758534082296255942[73] = 0;
   out_8758534082296255942[74] = 0;
   out_8758534082296255942[75] = 0;
   out_8758534082296255942[76] = 1;
   out_8758534082296255942[77] = 0;
   out_8758534082296255942[78] = 0;
   out_8758534082296255942[79] = 0;
   out_8758534082296255942[80] = 0;
   out_8758534082296255942[81] = 0;
   out_8758534082296255942[82] = 0;
   out_8758534082296255942[83] = 0;
   out_8758534082296255942[84] = 0;
   out_8758534082296255942[85] = dt;
   out_8758534082296255942[86] = 0;
   out_8758534082296255942[87] = 0;
   out_8758534082296255942[88] = 0;
   out_8758534082296255942[89] = 0;
   out_8758534082296255942[90] = 0;
   out_8758534082296255942[91] = 0;
   out_8758534082296255942[92] = 0;
   out_8758534082296255942[93] = 0;
   out_8758534082296255942[94] = 0;
   out_8758534082296255942[95] = 1;
   out_8758534082296255942[96] = 0;
   out_8758534082296255942[97] = 0;
   out_8758534082296255942[98] = 0;
   out_8758534082296255942[99] = 0;
   out_8758534082296255942[100] = 0;
   out_8758534082296255942[101] = 0;
   out_8758534082296255942[102] = 0;
   out_8758534082296255942[103] = 0;
   out_8758534082296255942[104] = dt;
   out_8758534082296255942[105] = 0;
   out_8758534082296255942[106] = 0;
   out_8758534082296255942[107] = 0;
   out_8758534082296255942[108] = 0;
   out_8758534082296255942[109] = 0;
   out_8758534082296255942[110] = 0;
   out_8758534082296255942[111] = 0;
   out_8758534082296255942[112] = 0;
   out_8758534082296255942[113] = 0;
   out_8758534082296255942[114] = 1;
   out_8758534082296255942[115] = 0;
   out_8758534082296255942[116] = 0;
   out_8758534082296255942[117] = 0;
   out_8758534082296255942[118] = 0;
   out_8758534082296255942[119] = 0;
   out_8758534082296255942[120] = 0;
   out_8758534082296255942[121] = 0;
   out_8758534082296255942[122] = 0;
   out_8758534082296255942[123] = 0;
   out_8758534082296255942[124] = 0;
   out_8758534082296255942[125] = 0;
   out_8758534082296255942[126] = 0;
   out_8758534082296255942[127] = 0;
   out_8758534082296255942[128] = 0;
   out_8758534082296255942[129] = 0;
   out_8758534082296255942[130] = 0;
   out_8758534082296255942[131] = 0;
   out_8758534082296255942[132] = 0;
   out_8758534082296255942[133] = 1;
   out_8758534082296255942[134] = 0;
   out_8758534082296255942[135] = 0;
   out_8758534082296255942[136] = 0;
   out_8758534082296255942[137] = 0;
   out_8758534082296255942[138] = 0;
   out_8758534082296255942[139] = 0;
   out_8758534082296255942[140] = 0;
   out_8758534082296255942[141] = 0;
   out_8758534082296255942[142] = 0;
   out_8758534082296255942[143] = 0;
   out_8758534082296255942[144] = 0;
   out_8758534082296255942[145] = 0;
   out_8758534082296255942[146] = 0;
   out_8758534082296255942[147] = 0;
   out_8758534082296255942[148] = 0;
   out_8758534082296255942[149] = 0;
   out_8758534082296255942[150] = 0;
   out_8758534082296255942[151] = 0;
   out_8758534082296255942[152] = 1;
   out_8758534082296255942[153] = 0;
   out_8758534082296255942[154] = 0;
   out_8758534082296255942[155] = 0;
   out_8758534082296255942[156] = 0;
   out_8758534082296255942[157] = 0;
   out_8758534082296255942[158] = 0;
   out_8758534082296255942[159] = 0;
   out_8758534082296255942[160] = 0;
   out_8758534082296255942[161] = 0;
   out_8758534082296255942[162] = 0;
   out_8758534082296255942[163] = 0;
   out_8758534082296255942[164] = 0;
   out_8758534082296255942[165] = 0;
   out_8758534082296255942[166] = 0;
   out_8758534082296255942[167] = 0;
   out_8758534082296255942[168] = 0;
   out_8758534082296255942[169] = 0;
   out_8758534082296255942[170] = 0;
   out_8758534082296255942[171] = 1;
   out_8758534082296255942[172] = 0;
   out_8758534082296255942[173] = 0;
   out_8758534082296255942[174] = 0;
   out_8758534082296255942[175] = 0;
   out_8758534082296255942[176] = 0;
   out_8758534082296255942[177] = 0;
   out_8758534082296255942[178] = 0;
   out_8758534082296255942[179] = 0;
   out_8758534082296255942[180] = 0;
   out_8758534082296255942[181] = 0;
   out_8758534082296255942[182] = 0;
   out_8758534082296255942[183] = 0;
   out_8758534082296255942[184] = 0;
   out_8758534082296255942[185] = 0;
   out_8758534082296255942[186] = 0;
   out_8758534082296255942[187] = 0;
   out_8758534082296255942[188] = 0;
   out_8758534082296255942[189] = 0;
   out_8758534082296255942[190] = 1;
   out_8758534082296255942[191] = 0;
   out_8758534082296255942[192] = 0;
   out_8758534082296255942[193] = 0;
   out_8758534082296255942[194] = 0;
   out_8758534082296255942[195] = 0;
   out_8758534082296255942[196] = 0;
   out_8758534082296255942[197] = 0;
   out_8758534082296255942[198] = 0;
   out_8758534082296255942[199] = 0;
   out_8758534082296255942[200] = 0;
   out_8758534082296255942[201] = 0;
   out_8758534082296255942[202] = 0;
   out_8758534082296255942[203] = 0;
   out_8758534082296255942[204] = 0;
   out_8758534082296255942[205] = 0;
   out_8758534082296255942[206] = 0;
   out_8758534082296255942[207] = 0;
   out_8758534082296255942[208] = 0;
   out_8758534082296255942[209] = 1;
   out_8758534082296255942[210] = 0;
   out_8758534082296255942[211] = 0;
   out_8758534082296255942[212] = 0;
   out_8758534082296255942[213] = 0;
   out_8758534082296255942[214] = 0;
   out_8758534082296255942[215] = 0;
   out_8758534082296255942[216] = 0;
   out_8758534082296255942[217] = 0;
   out_8758534082296255942[218] = 0;
   out_8758534082296255942[219] = 0;
   out_8758534082296255942[220] = 0;
   out_8758534082296255942[221] = 0;
   out_8758534082296255942[222] = 0;
   out_8758534082296255942[223] = 0;
   out_8758534082296255942[224] = 0;
   out_8758534082296255942[225] = 0;
   out_8758534082296255942[226] = 0;
   out_8758534082296255942[227] = 0;
   out_8758534082296255942[228] = 1;
   out_8758534082296255942[229] = 0;
   out_8758534082296255942[230] = 0;
   out_8758534082296255942[231] = 0;
   out_8758534082296255942[232] = 0;
   out_8758534082296255942[233] = 0;
   out_8758534082296255942[234] = 0;
   out_8758534082296255942[235] = 0;
   out_8758534082296255942[236] = 0;
   out_8758534082296255942[237] = 0;
   out_8758534082296255942[238] = 0;
   out_8758534082296255942[239] = 0;
   out_8758534082296255942[240] = 0;
   out_8758534082296255942[241] = 0;
   out_8758534082296255942[242] = 0;
   out_8758534082296255942[243] = 0;
   out_8758534082296255942[244] = 0;
   out_8758534082296255942[245] = 0;
   out_8758534082296255942[246] = 0;
   out_8758534082296255942[247] = 1;
   out_8758534082296255942[248] = 0;
   out_8758534082296255942[249] = 0;
   out_8758534082296255942[250] = 0;
   out_8758534082296255942[251] = 0;
   out_8758534082296255942[252] = 0;
   out_8758534082296255942[253] = 0;
   out_8758534082296255942[254] = 0;
   out_8758534082296255942[255] = 0;
   out_8758534082296255942[256] = 0;
   out_8758534082296255942[257] = 0;
   out_8758534082296255942[258] = 0;
   out_8758534082296255942[259] = 0;
   out_8758534082296255942[260] = 0;
   out_8758534082296255942[261] = 0;
   out_8758534082296255942[262] = 0;
   out_8758534082296255942[263] = 0;
   out_8758534082296255942[264] = 0;
   out_8758534082296255942[265] = 0;
   out_8758534082296255942[266] = 1;
   out_8758534082296255942[267] = 0;
   out_8758534082296255942[268] = 0;
   out_8758534082296255942[269] = 0;
   out_8758534082296255942[270] = 0;
   out_8758534082296255942[271] = 0;
   out_8758534082296255942[272] = 0;
   out_8758534082296255942[273] = 0;
   out_8758534082296255942[274] = 0;
   out_8758534082296255942[275] = 0;
   out_8758534082296255942[276] = 0;
   out_8758534082296255942[277] = 0;
   out_8758534082296255942[278] = 0;
   out_8758534082296255942[279] = 0;
   out_8758534082296255942[280] = 0;
   out_8758534082296255942[281] = 0;
   out_8758534082296255942[282] = 0;
   out_8758534082296255942[283] = 0;
   out_8758534082296255942[284] = 0;
   out_8758534082296255942[285] = 1;
   out_8758534082296255942[286] = 0;
   out_8758534082296255942[287] = 0;
   out_8758534082296255942[288] = 0;
   out_8758534082296255942[289] = 0;
   out_8758534082296255942[290] = 0;
   out_8758534082296255942[291] = 0;
   out_8758534082296255942[292] = 0;
   out_8758534082296255942[293] = 0;
   out_8758534082296255942[294] = 0;
   out_8758534082296255942[295] = 0;
   out_8758534082296255942[296] = 0;
   out_8758534082296255942[297] = 0;
   out_8758534082296255942[298] = 0;
   out_8758534082296255942[299] = 0;
   out_8758534082296255942[300] = 0;
   out_8758534082296255942[301] = 0;
   out_8758534082296255942[302] = 0;
   out_8758534082296255942[303] = 0;
   out_8758534082296255942[304] = 1;
   out_8758534082296255942[305] = 0;
   out_8758534082296255942[306] = 0;
   out_8758534082296255942[307] = 0;
   out_8758534082296255942[308] = 0;
   out_8758534082296255942[309] = 0;
   out_8758534082296255942[310] = 0;
   out_8758534082296255942[311] = 0;
   out_8758534082296255942[312] = 0;
   out_8758534082296255942[313] = 0;
   out_8758534082296255942[314] = 0;
   out_8758534082296255942[315] = 0;
   out_8758534082296255942[316] = 0;
   out_8758534082296255942[317] = 0;
   out_8758534082296255942[318] = 0;
   out_8758534082296255942[319] = 0;
   out_8758534082296255942[320] = 0;
   out_8758534082296255942[321] = 0;
   out_8758534082296255942[322] = 0;
   out_8758534082296255942[323] = 1;
}
void h_4(double *state, double *unused, double *out_4992833986378518702) {
   out_4992833986378518702[0] = state[6] + state[9];
   out_4992833986378518702[1] = state[7] + state[10];
   out_4992833986378518702[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6397593698087977158) {
   out_6397593698087977158[0] = 0;
   out_6397593698087977158[1] = 0;
   out_6397593698087977158[2] = 0;
   out_6397593698087977158[3] = 0;
   out_6397593698087977158[4] = 0;
   out_6397593698087977158[5] = 0;
   out_6397593698087977158[6] = 1;
   out_6397593698087977158[7] = 0;
   out_6397593698087977158[8] = 0;
   out_6397593698087977158[9] = 1;
   out_6397593698087977158[10] = 0;
   out_6397593698087977158[11] = 0;
   out_6397593698087977158[12] = 0;
   out_6397593698087977158[13] = 0;
   out_6397593698087977158[14] = 0;
   out_6397593698087977158[15] = 0;
   out_6397593698087977158[16] = 0;
   out_6397593698087977158[17] = 0;
   out_6397593698087977158[18] = 0;
   out_6397593698087977158[19] = 0;
   out_6397593698087977158[20] = 0;
   out_6397593698087977158[21] = 0;
   out_6397593698087977158[22] = 0;
   out_6397593698087977158[23] = 0;
   out_6397593698087977158[24] = 0;
   out_6397593698087977158[25] = 1;
   out_6397593698087977158[26] = 0;
   out_6397593698087977158[27] = 0;
   out_6397593698087977158[28] = 1;
   out_6397593698087977158[29] = 0;
   out_6397593698087977158[30] = 0;
   out_6397593698087977158[31] = 0;
   out_6397593698087977158[32] = 0;
   out_6397593698087977158[33] = 0;
   out_6397593698087977158[34] = 0;
   out_6397593698087977158[35] = 0;
   out_6397593698087977158[36] = 0;
   out_6397593698087977158[37] = 0;
   out_6397593698087977158[38] = 0;
   out_6397593698087977158[39] = 0;
   out_6397593698087977158[40] = 0;
   out_6397593698087977158[41] = 0;
   out_6397593698087977158[42] = 0;
   out_6397593698087977158[43] = 0;
   out_6397593698087977158[44] = 1;
   out_6397593698087977158[45] = 0;
   out_6397593698087977158[46] = 0;
   out_6397593698087977158[47] = 1;
   out_6397593698087977158[48] = 0;
   out_6397593698087977158[49] = 0;
   out_6397593698087977158[50] = 0;
   out_6397593698087977158[51] = 0;
   out_6397593698087977158[52] = 0;
   out_6397593698087977158[53] = 0;
}
void h_10(double *state, double *unused, double *out_8934627023979057486) {
   out_8934627023979057486[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8934627023979057486[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8934627023979057486[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_456432144623306927) {
   out_456432144623306927[0] = 0;
   out_456432144623306927[1] = 9.8100000000000005*cos(state[1]);
   out_456432144623306927[2] = 0;
   out_456432144623306927[3] = 0;
   out_456432144623306927[4] = -state[8];
   out_456432144623306927[5] = state[7];
   out_456432144623306927[6] = 0;
   out_456432144623306927[7] = state[5];
   out_456432144623306927[8] = -state[4];
   out_456432144623306927[9] = 0;
   out_456432144623306927[10] = 0;
   out_456432144623306927[11] = 0;
   out_456432144623306927[12] = 1;
   out_456432144623306927[13] = 0;
   out_456432144623306927[14] = 0;
   out_456432144623306927[15] = 1;
   out_456432144623306927[16] = 0;
   out_456432144623306927[17] = 0;
   out_456432144623306927[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_456432144623306927[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_456432144623306927[20] = 0;
   out_456432144623306927[21] = state[8];
   out_456432144623306927[22] = 0;
   out_456432144623306927[23] = -state[6];
   out_456432144623306927[24] = -state[5];
   out_456432144623306927[25] = 0;
   out_456432144623306927[26] = state[3];
   out_456432144623306927[27] = 0;
   out_456432144623306927[28] = 0;
   out_456432144623306927[29] = 0;
   out_456432144623306927[30] = 0;
   out_456432144623306927[31] = 1;
   out_456432144623306927[32] = 0;
   out_456432144623306927[33] = 0;
   out_456432144623306927[34] = 1;
   out_456432144623306927[35] = 0;
   out_456432144623306927[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_456432144623306927[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_456432144623306927[38] = 0;
   out_456432144623306927[39] = -state[7];
   out_456432144623306927[40] = state[6];
   out_456432144623306927[41] = 0;
   out_456432144623306927[42] = state[4];
   out_456432144623306927[43] = -state[3];
   out_456432144623306927[44] = 0;
   out_456432144623306927[45] = 0;
   out_456432144623306927[46] = 0;
   out_456432144623306927[47] = 0;
   out_456432144623306927[48] = 0;
   out_456432144623306927[49] = 0;
   out_456432144623306927[50] = 1;
   out_456432144623306927[51] = 0;
   out_456432144623306927[52] = 0;
   out_456432144623306927[53] = 1;
}
void h_13(double *state, double *unused, double *out_6687713468022338325) {
   out_6687713468022338325[0] = state[3];
   out_6687713468022338325[1] = state[4];
   out_6687713468022338325[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8836876550289241657) {
   out_8836876550289241657[0] = 0;
   out_8836876550289241657[1] = 0;
   out_8836876550289241657[2] = 0;
   out_8836876550289241657[3] = 1;
   out_8836876550289241657[4] = 0;
   out_8836876550289241657[5] = 0;
   out_8836876550289241657[6] = 0;
   out_8836876550289241657[7] = 0;
   out_8836876550289241657[8] = 0;
   out_8836876550289241657[9] = 0;
   out_8836876550289241657[10] = 0;
   out_8836876550289241657[11] = 0;
   out_8836876550289241657[12] = 0;
   out_8836876550289241657[13] = 0;
   out_8836876550289241657[14] = 0;
   out_8836876550289241657[15] = 0;
   out_8836876550289241657[16] = 0;
   out_8836876550289241657[17] = 0;
   out_8836876550289241657[18] = 0;
   out_8836876550289241657[19] = 0;
   out_8836876550289241657[20] = 0;
   out_8836876550289241657[21] = 0;
   out_8836876550289241657[22] = 1;
   out_8836876550289241657[23] = 0;
   out_8836876550289241657[24] = 0;
   out_8836876550289241657[25] = 0;
   out_8836876550289241657[26] = 0;
   out_8836876550289241657[27] = 0;
   out_8836876550289241657[28] = 0;
   out_8836876550289241657[29] = 0;
   out_8836876550289241657[30] = 0;
   out_8836876550289241657[31] = 0;
   out_8836876550289241657[32] = 0;
   out_8836876550289241657[33] = 0;
   out_8836876550289241657[34] = 0;
   out_8836876550289241657[35] = 0;
   out_8836876550289241657[36] = 0;
   out_8836876550289241657[37] = 0;
   out_8836876550289241657[38] = 0;
   out_8836876550289241657[39] = 0;
   out_8836876550289241657[40] = 0;
   out_8836876550289241657[41] = 1;
   out_8836876550289241657[42] = 0;
   out_8836876550289241657[43] = 0;
   out_8836876550289241657[44] = 0;
   out_8836876550289241657[45] = 0;
   out_8836876550289241657[46] = 0;
   out_8836876550289241657[47] = 0;
   out_8836876550289241657[48] = 0;
   out_8836876550289241657[49] = 0;
   out_8836876550289241657[50] = 0;
   out_8836876550289241657[51] = 0;
   out_8836876550289241657[52] = 0;
   out_8836876550289241657[53] = 0;
}
void h_14(double *state, double *unused, double *out_6685023201088185678) {
   out_6685023201088185678[0] = state[6];
   out_6685023201088185678[1] = state[7];
   out_6685023201088185678[2] = state[8];
}
void H_14(double *state, double *unused, double *out_1083552117191763266) {
   out_1083552117191763266[0] = 0;
   out_1083552117191763266[1] = 0;
   out_1083552117191763266[2] = 0;
   out_1083552117191763266[3] = 0;
   out_1083552117191763266[4] = 0;
   out_1083552117191763266[5] = 0;
   out_1083552117191763266[6] = 1;
   out_1083552117191763266[7] = 0;
   out_1083552117191763266[8] = 0;
   out_1083552117191763266[9] = 0;
   out_1083552117191763266[10] = 0;
   out_1083552117191763266[11] = 0;
   out_1083552117191763266[12] = 0;
   out_1083552117191763266[13] = 0;
   out_1083552117191763266[14] = 0;
   out_1083552117191763266[15] = 0;
   out_1083552117191763266[16] = 0;
   out_1083552117191763266[17] = 0;
   out_1083552117191763266[18] = 0;
   out_1083552117191763266[19] = 0;
   out_1083552117191763266[20] = 0;
   out_1083552117191763266[21] = 0;
   out_1083552117191763266[22] = 0;
   out_1083552117191763266[23] = 0;
   out_1083552117191763266[24] = 0;
   out_1083552117191763266[25] = 1;
   out_1083552117191763266[26] = 0;
   out_1083552117191763266[27] = 0;
   out_1083552117191763266[28] = 0;
   out_1083552117191763266[29] = 0;
   out_1083552117191763266[30] = 0;
   out_1083552117191763266[31] = 0;
   out_1083552117191763266[32] = 0;
   out_1083552117191763266[33] = 0;
   out_1083552117191763266[34] = 0;
   out_1083552117191763266[35] = 0;
   out_1083552117191763266[36] = 0;
   out_1083552117191763266[37] = 0;
   out_1083552117191763266[38] = 0;
   out_1083552117191763266[39] = 0;
   out_1083552117191763266[40] = 0;
   out_1083552117191763266[41] = 0;
   out_1083552117191763266[42] = 0;
   out_1083552117191763266[43] = 0;
   out_1083552117191763266[44] = 1;
   out_1083552117191763266[45] = 0;
   out_1083552117191763266[46] = 0;
   out_1083552117191763266[47] = 0;
   out_1083552117191763266[48] = 0;
   out_1083552117191763266[49] = 0;
   out_1083552117191763266[50] = 0;
   out_1083552117191763266[51] = 0;
   out_1083552117191763266[52] = 0;
   out_1083552117191763266[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4430958223127505081) {
  err_fun(nom_x, delta_x, out_4430958223127505081);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4290768674008276673) {
  inv_err_fun(nom_x, true_x, out_4290768674008276673);
}
void pose_H_mod_fun(double *state, double *out_3054748865441164755) {
  H_mod_fun(state, out_3054748865441164755);
}
void pose_f_fun(double *state, double dt, double *out_717525079663500723) {
  f_fun(state,  dt, out_717525079663500723);
}
void pose_F_fun(double *state, double dt, double *out_8758534082296255942) {
  F_fun(state,  dt, out_8758534082296255942);
}
void pose_h_4(double *state, double *unused, double *out_4992833986378518702) {
  h_4(state, unused, out_4992833986378518702);
}
void pose_H_4(double *state, double *unused, double *out_6397593698087977158) {
  H_4(state, unused, out_6397593698087977158);
}
void pose_h_10(double *state, double *unused, double *out_8934627023979057486) {
  h_10(state, unused, out_8934627023979057486);
}
void pose_H_10(double *state, double *unused, double *out_456432144623306927) {
  H_10(state, unused, out_456432144623306927);
}
void pose_h_13(double *state, double *unused, double *out_6687713468022338325) {
  h_13(state, unused, out_6687713468022338325);
}
void pose_H_13(double *state, double *unused, double *out_8836876550289241657) {
  H_13(state, unused, out_8836876550289241657);
}
void pose_h_14(double *state, double *unused, double *out_6685023201088185678) {
  h_14(state, unused, out_6685023201088185678);
}
void pose_H_14(double *state, double *unused, double *out_1083552117191763266) {
  H_14(state, unused, out_1083552117191763266);
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
