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
void err_fun(double *nom_x, double *delta_x, double *out_5997402376607838659) {
   out_5997402376607838659[0] = delta_x[0] + nom_x[0];
   out_5997402376607838659[1] = delta_x[1] + nom_x[1];
   out_5997402376607838659[2] = delta_x[2] + nom_x[2];
   out_5997402376607838659[3] = delta_x[3] + nom_x[3];
   out_5997402376607838659[4] = delta_x[4] + nom_x[4];
   out_5997402376607838659[5] = delta_x[5] + nom_x[5];
   out_5997402376607838659[6] = delta_x[6] + nom_x[6];
   out_5997402376607838659[7] = delta_x[7] + nom_x[7];
   out_5997402376607838659[8] = delta_x[8] + nom_x[8];
   out_5997402376607838659[9] = delta_x[9] + nom_x[9];
   out_5997402376607838659[10] = delta_x[10] + nom_x[10];
   out_5997402376607838659[11] = delta_x[11] + nom_x[11];
   out_5997402376607838659[12] = delta_x[12] + nom_x[12];
   out_5997402376607838659[13] = delta_x[13] + nom_x[13];
   out_5997402376607838659[14] = delta_x[14] + nom_x[14];
   out_5997402376607838659[15] = delta_x[15] + nom_x[15];
   out_5997402376607838659[16] = delta_x[16] + nom_x[16];
   out_5997402376607838659[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4197439267135692258) {
   out_4197439267135692258[0] = -nom_x[0] + true_x[0];
   out_4197439267135692258[1] = -nom_x[1] + true_x[1];
   out_4197439267135692258[2] = -nom_x[2] + true_x[2];
   out_4197439267135692258[3] = -nom_x[3] + true_x[3];
   out_4197439267135692258[4] = -nom_x[4] + true_x[4];
   out_4197439267135692258[5] = -nom_x[5] + true_x[5];
   out_4197439267135692258[6] = -nom_x[6] + true_x[6];
   out_4197439267135692258[7] = -nom_x[7] + true_x[7];
   out_4197439267135692258[8] = -nom_x[8] + true_x[8];
   out_4197439267135692258[9] = -nom_x[9] + true_x[9];
   out_4197439267135692258[10] = -nom_x[10] + true_x[10];
   out_4197439267135692258[11] = -nom_x[11] + true_x[11];
   out_4197439267135692258[12] = -nom_x[12] + true_x[12];
   out_4197439267135692258[13] = -nom_x[13] + true_x[13];
   out_4197439267135692258[14] = -nom_x[14] + true_x[14];
   out_4197439267135692258[15] = -nom_x[15] + true_x[15];
   out_4197439267135692258[16] = -nom_x[16] + true_x[16];
   out_4197439267135692258[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3181760894870259401) {
   out_3181760894870259401[0] = 1.0;
   out_3181760894870259401[1] = 0.0;
   out_3181760894870259401[2] = 0.0;
   out_3181760894870259401[3] = 0.0;
   out_3181760894870259401[4] = 0.0;
   out_3181760894870259401[5] = 0.0;
   out_3181760894870259401[6] = 0.0;
   out_3181760894870259401[7] = 0.0;
   out_3181760894870259401[8] = 0.0;
   out_3181760894870259401[9] = 0.0;
   out_3181760894870259401[10] = 0.0;
   out_3181760894870259401[11] = 0.0;
   out_3181760894870259401[12] = 0.0;
   out_3181760894870259401[13] = 0.0;
   out_3181760894870259401[14] = 0.0;
   out_3181760894870259401[15] = 0.0;
   out_3181760894870259401[16] = 0.0;
   out_3181760894870259401[17] = 0.0;
   out_3181760894870259401[18] = 0.0;
   out_3181760894870259401[19] = 1.0;
   out_3181760894870259401[20] = 0.0;
   out_3181760894870259401[21] = 0.0;
   out_3181760894870259401[22] = 0.0;
   out_3181760894870259401[23] = 0.0;
   out_3181760894870259401[24] = 0.0;
   out_3181760894870259401[25] = 0.0;
   out_3181760894870259401[26] = 0.0;
   out_3181760894870259401[27] = 0.0;
   out_3181760894870259401[28] = 0.0;
   out_3181760894870259401[29] = 0.0;
   out_3181760894870259401[30] = 0.0;
   out_3181760894870259401[31] = 0.0;
   out_3181760894870259401[32] = 0.0;
   out_3181760894870259401[33] = 0.0;
   out_3181760894870259401[34] = 0.0;
   out_3181760894870259401[35] = 0.0;
   out_3181760894870259401[36] = 0.0;
   out_3181760894870259401[37] = 0.0;
   out_3181760894870259401[38] = 1.0;
   out_3181760894870259401[39] = 0.0;
   out_3181760894870259401[40] = 0.0;
   out_3181760894870259401[41] = 0.0;
   out_3181760894870259401[42] = 0.0;
   out_3181760894870259401[43] = 0.0;
   out_3181760894870259401[44] = 0.0;
   out_3181760894870259401[45] = 0.0;
   out_3181760894870259401[46] = 0.0;
   out_3181760894870259401[47] = 0.0;
   out_3181760894870259401[48] = 0.0;
   out_3181760894870259401[49] = 0.0;
   out_3181760894870259401[50] = 0.0;
   out_3181760894870259401[51] = 0.0;
   out_3181760894870259401[52] = 0.0;
   out_3181760894870259401[53] = 0.0;
   out_3181760894870259401[54] = 0.0;
   out_3181760894870259401[55] = 0.0;
   out_3181760894870259401[56] = 0.0;
   out_3181760894870259401[57] = 1.0;
   out_3181760894870259401[58] = 0.0;
   out_3181760894870259401[59] = 0.0;
   out_3181760894870259401[60] = 0.0;
   out_3181760894870259401[61] = 0.0;
   out_3181760894870259401[62] = 0.0;
   out_3181760894870259401[63] = 0.0;
   out_3181760894870259401[64] = 0.0;
   out_3181760894870259401[65] = 0.0;
   out_3181760894870259401[66] = 0.0;
   out_3181760894870259401[67] = 0.0;
   out_3181760894870259401[68] = 0.0;
   out_3181760894870259401[69] = 0.0;
   out_3181760894870259401[70] = 0.0;
   out_3181760894870259401[71] = 0.0;
   out_3181760894870259401[72] = 0.0;
   out_3181760894870259401[73] = 0.0;
   out_3181760894870259401[74] = 0.0;
   out_3181760894870259401[75] = 0.0;
   out_3181760894870259401[76] = 1.0;
   out_3181760894870259401[77] = 0.0;
   out_3181760894870259401[78] = 0.0;
   out_3181760894870259401[79] = 0.0;
   out_3181760894870259401[80] = 0.0;
   out_3181760894870259401[81] = 0.0;
   out_3181760894870259401[82] = 0.0;
   out_3181760894870259401[83] = 0.0;
   out_3181760894870259401[84] = 0.0;
   out_3181760894870259401[85] = 0.0;
   out_3181760894870259401[86] = 0.0;
   out_3181760894870259401[87] = 0.0;
   out_3181760894870259401[88] = 0.0;
   out_3181760894870259401[89] = 0.0;
   out_3181760894870259401[90] = 0.0;
   out_3181760894870259401[91] = 0.0;
   out_3181760894870259401[92] = 0.0;
   out_3181760894870259401[93] = 0.0;
   out_3181760894870259401[94] = 0.0;
   out_3181760894870259401[95] = 1.0;
   out_3181760894870259401[96] = 0.0;
   out_3181760894870259401[97] = 0.0;
   out_3181760894870259401[98] = 0.0;
   out_3181760894870259401[99] = 0.0;
   out_3181760894870259401[100] = 0.0;
   out_3181760894870259401[101] = 0.0;
   out_3181760894870259401[102] = 0.0;
   out_3181760894870259401[103] = 0.0;
   out_3181760894870259401[104] = 0.0;
   out_3181760894870259401[105] = 0.0;
   out_3181760894870259401[106] = 0.0;
   out_3181760894870259401[107] = 0.0;
   out_3181760894870259401[108] = 0.0;
   out_3181760894870259401[109] = 0.0;
   out_3181760894870259401[110] = 0.0;
   out_3181760894870259401[111] = 0.0;
   out_3181760894870259401[112] = 0.0;
   out_3181760894870259401[113] = 0.0;
   out_3181760894870259401[114] = 1.0;
   out_3181760894870259401[115] = 0.0;
   out_3181760894870259401[116] = 0.0;
   out_3181760894870259401[117] = 0.0;
   out_3181760894870259401[118] = 0.0;
   out_3181760894870259401[119] = 0.0;
   out_3181760894870259401[120] = 0.0;
   out_3181760894870259401[121] = 0.0;
   out_3181760894870259401[122] = 0.0;
   out_3181760894870259401[123] = 0.0;
   out_3181760894870259401[124] = 0.0;
   out_3181760894870259401[125] = 0.0;
   out_3181760894870259401[126] = 0.0;
   out_3181760894870259401[127] = 0.0;
   out_3181760894870259401[128] = 0.0;
   out_3181760894870259401[129] = 0.0;
   out_3181760894870259401[130] = 0.0;
   out_3181760894870259401[131] = 0.0;
   out_3181760894870259401[132] = 0.0;
   out_3181760894870259401[133] = 1.0;
   out_3181760894870259401[134] = 0.0;
   out_3181760894870259401[135] = 0.0;
   out_3181760894870259401[136] = 0.0;
   out_3181760894870259401[137] = 0.0;
   out_3181760894870259401[138] = 0.0;
   out_3181760894870259401[139] = 0.0;
   out_3181760894870259401[140] = 0.0;
   out_3181760894870259401[141] = 0.0;
   out_3181760894870259401[142] = 0.0;
   out_3181760894870259401[143] = 0.0;
   out_3181760894870259401[144] = 0.0;
   out_3181760894870259401[145] = 0.0;
   out_3181760894870259401[146] = 0.0;
   out_3181760894870259401[147] = 0.0;
   out_3181760894870259401[148] = 0.0;
   out_3181760894870259401[149] = 0.0;
   out_3181760894870259401[150] = 0.0;
   out_3181760894870259401[151] = 0.0;
   out_3181760894870259401[152] = 1.0;
   out_3181760894870259401[153] = 0.0;
   out_3181760894870259401[154] = 0.0;
   out_3181760894870259401[155] = 0.0;
   out_3181760894870259401[156] = 0.0;
   out_3181760894870259401[157] = 0.0;
   out_3181760894870259401[158] = 0.0;
   out_3181760894870259401[159] = 0.0;
   out_3181760894870259401[160] = 0.0;
   out_3181760894870259401[161] = 0.0;
   out_3181760894870259401[162] = 0.0;
   out_3181760894870259401[163] = 0.0;
   out_3181760894870259401[164] = 0.0;
   out_3181760894870259401[165] = 0.0;
   out_3181760894870259401[166] = 0.0;
   out_3181760894870259401[167] = 0.0;
   out_3181760894870259401[168] = 0.0;
   out_3181760894870259401[169] = 0.0;
   out_3181760894870259401[170] = 0.0;
   out_3181760894870259401[171] = 1.0;
   out_3181760894870259401[172] = 0.0;
   out_3181760894870259401[173] = 0.0;
   out_3181760894870259401[174] = 0.0;
   out_3181760894870259401[175] = 0.0;
   out_3181760894870259401[176] = 0.0;
   out_3181760894870259401[177] = 0.0;
   out_3181760894870259401[178] = 0.0;
   out_3181760894870259401[179] = 0.0;
   out_3181760894870259401[180] = 0.0;
   out_3181760894870259401[181] = 0.0;
   out_3181760894870259401[182] = 0.0;
   out_3181760894870259401[183] = 0.0;
   out_3181760894870259401[184] = 0.0;
   out_3181760894870259401[185] = 0.0;
   out_3181760894870259401[186] = 0.0;
   out_3181760894870259401[187] = 0.0;
   out_3181760894870259401[188] = 0.0;
   out_3181760894870259401[189] = 0.0;
   out_3181760894870259401[190] = 1.0;
   out_3181760894870259401[191] = 0.0;
   out_3181760894870259401[192] = 0.0;
   out_3181760894870259401[193] = 0.0;
   out_3181760894870259401[194] = 0.0;
   out_3181760894870259401[195] = 0.0;
   out_3181760894870259401[196] = 0.0;
   out_3181760894870259401[197] = 0.0;
   out_3181760894870259401[198] = 0.0;
   out_3181760894870259401[199] = 0.0;
   out_3181760894870259401[200] = 0.0;
   out_3181760894870259401[201] = 0.0;
   out_3181760894870259401[202] = 0.0;
   out_3181760894870259401[203] = 0.0;
   out_3181760894870259401[204] = 0.0;
   out_3181760894870259401[205] = 0.0;
   out_3181760894870259401[206] = 0.0;
   out_3181760894870259401[207] = 0.0;
   out_3181760894870259401[208] = 0.0;
   out_3181760894870259401[209] = 1.0;
   out_3181760894870259401[210] = 0.0;
   out_3181760894870259401[211] = 0.0;
   out_3181760894870259401[212] = 0.0;
   out_3181760894870259401[213] = 0.0;
   out_3181760894870259401[214] = 0.0;
   out_3181760894870259401[215] = 0.0;
   out_3181760894870259401[216] = 0.0;
   out_3181760894870259401[217] = 0.0;
   out_3181760894870259401[218] = 0.0;
   out_3181760894870259401[219] = 0.0;
   out_3181760894870259401[220] = 0.0;
   out_3181760894870259401[221] = 0.0;
   out_3181760894870259401[222] = 0.0;
   out_3181760894870259401[223] = 0.0;
   out_3181760894870259401[224] = 0.0;
   out_3181760894870259401[225] = 0.0;
   out_3181760894870259401[226] = 0.0;
   out_3181760894870259401[227] = 0.0;
   out_3181760894870259401[228] = 1.0;
   out_3181760894870259401[229] = 0.0;
   out_3181760894870259401[230] = 0.0;
   out_3181760894870259401[231] = 0.0;
   out_3181760894870259401[232] = 0.0;
   out_3181760894870259401[233] = 0.0;
   out_3181760894870259401[234] = 0.0;
   out_3181760894870259401[235] = 0.0;
   out_3181760894870259401[236] = 0.0;
   out_3181760894870259401[237] = 0.0;
   out_3181760894870259401[238] = 0.0;
   out_3181760894870259401[239] = 0.0;
   out_3181760894870259401[240] = 0.0;
   out_3181760894870259401[241] = 0.0;
   out_3181760894870259401[242] = 0.0;
   out_3181760894870259401[243] = 0.0;
   out_3181760894870259401[244] = 0.0;
   out_3181760894870259401[245] = 0.0;
   out_3181760894870259401[246] = 0.0;
   out_3181760894870259401[247] = 1.0;
   out_3181760894870259401[248] = 0.0;
   out_3181760894870259401[249] = 0.0;
   out_3181760894870259401[250] = 0.0;
   out_3181760894870259401[251] = 0.0;
   out_3181760894870259401[252] = 0.0;
   out_3181760894870259401[253] = 0.0;
   out_3181760894870259401[254] = 0.0;
   out_3181760894870259401[255] = 0.0;
   out_3181760894870259401[256] = 0.0;
   out_3181760894870259401[257] = 0.0;
   out_3181760894870259401[258] = 0.0;
   out_3181760894870259401[259] = 0.0;
   out_3181760894870259401[260] = 0.0;
   out_3181760894870259401[261] = 0.0;
   out_3181760894870259401[262] = 0.0;
   out_3181760894870259401[263] = 0.0;
   out_3181760894870259401[264] = 0.0;
   out_3181760894870259401[265] = 0.0;
   out_3181760894870259401[266] = 1.0;
   out_3181760894870259401[267] = 0.0;
   out_3181760894870259401[268] = 0.0;
   out_3181760894870259401[269] = 0.0;
   out_3181760894870259401[270] = 0.0;
   out_3181760894870259401[271] = 0.0;
   out_3181760894870259401[272] = 0.0;
   out_3181760894870259401[273] = 0.0;
   out_3181760894870259401[274] = 0.0;
   out_3181760894870259401[275] = 0.0;
   out_3181760894870259401[276] = 0.0;
   out_3181760894870259401[277] = 0.0;
   out_3181760894870259401[278] = 0.0;
   out_3181760894870259401[279] = 0.0;
   out_3181760894870259401[280] = 0.0;
   out_3181760894870259401[281] = 0.0;
   out_3181760894870259401[282] = 0.0;
   out_3181760894870259401[283] = 0.0;
   out_3181760894870259401[284] = 0.0;
   out_3181760894870259401[285] = 1.0;
   out_3181760894870259401[286] = 0.0;
   out_3181760894870259401[287] = 0.0;
   out_3181760894870259401[288] = 0.0;
   out_3181760894870259401[289] = 0.0;
   out_3181760894870259401[290] = 0.0;
   out_3181760894870259401[291] = 0.0;
   out_3181760894870259401[292] = 0.0;
   out_3181760894870259401[293] = 0.0;
   out_3181760894870259401[294] = 0.0;
   out_3181760894870259401[295] = 0.0;
   out_3181760894870259401[296] = 0.0;
   out_3181760894870259401[297] = 0.0;
   out_3181760894870259401[298] = 0.0;
   out_3181760894870259401[299] = 0.0;
   out_3181760894870259401[300] = 0.0;
   out_3181760894870259401[301] = 0.0;
   out_3181760894870259401[302] = 0.0;
   out_3181760894870259401[303] = 0.0;
   out_3181760894870259401[304] = 1.0;
   out_3181760894870259401[305] = 0.0;
   out_3181760894870259401[306] = 0.0;
   out_3181760894870259401[307] = 0.0;
   out_3181760894870259401[308] = 0.0;
   out_3181760894870259401[309] = 0.0;
   out_3181760894870259401[310] = 0.0;
   out_3181760894870259401[311] = 0.0;
   out_3181760894870259401[312] = 0.0;
   out_3181760894870259401[313] = 0.0;
   out_3181760894870259401[314] = 0.0;
   out_3181760894870259401[315] = 0.0;
   out_3181760894870259401[316] = 0.0;
   out_3181760894870259401[317] = 0.0;
   out_3181760894870259401[318] = 0.0;
   out_3181760894870259401[319] = 0.0;
   out_3181760894870259401[320] = 0.0;
   out_3181760894870259401[321] = 0.0;
   out_3181760894870259401[322] = 0.0;
   out_3181760894870259401[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5584365400920044031) {
   out_5584365400920044031[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5584365400920044031[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5584365400920044031[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5584365400920044031[3] = dt*state[12] + state[3];
   out_5584365400920044031[4] = dt*state[13] + state[4];
   out_5584365400920044031[5] = dt*state[14] + state[5];
   out_5584365400920044031[6] = state[6];
   out_5584365400920044031[7] = state[7];
   out_5584365400920044031[8] = state[8];
   out_5584365400920044031[9] = state[9];
   out_5584365400920044031[10] = state[10];
   out_5584365400920044031[11] = state[11];
   out_5584365400920044031[12] = state[12];
   out_5584365400920044031[13] = state[13];
   out_5584365400920044031[14] = state[14];
   out_5584365400920044031[15] = state[15];
   out_5584365400920044031[16] = state[16];
   out_5584365400920044031[17] = state[17];
}
void F_fun(double *state, double dt, double *out_467723448142448616) {
   out_467723448142448616[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_467723448142448616[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_467723448142448616[2] = 0;
   out_467723448142448616[3] = 0;
   out_467723448142448616[4] = 0;
   out_467723448142448616[5] = 0;
   out_467723448142448616[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_467723448142448616[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_467723448142448616[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_467723448142448616[9] = 0;
   out_467723448142448616[10] = 0;
   out_467723448142448616[11] = 0;
   out_467723448142448616[12] = 0;
   out_467723448142448616[13] = 0;
   out_467723448142448616[14] = 0;
   out_467723448142448616[15] = 0;
   out_467723448142448616[16] = 0;
   out_467723448142448616[17] = 0;
   out_467723448142448616[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_467723448142448616[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_467723448142448616[20] = 0;
   out_467723448142448616[21] = 0;
   out_467723448142448616[22] = 0;
   out_467723448142448616[23] = 0;
   out_467723448142448616[24] = 0;
   out_467723448142448616[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_467723448142448616[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_467723448142448616[27] = 0;
   out_467723448142448616[28] = 0;
   out_467723448142448616[29] = 0;
   out_467723448142448616[30] = 0;
   out_467723448142448616[31] = 0;
   out_467723448142448616[32] = 0;
   out_467723448142448616[33] = 0;
   out_467723448142448616[34] = 0;
   out_467723448142448616[35] = 0;
   out_467723448142448616[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_467723448142448616[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_467723448142448616[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_467723448142448616[39] = 0;
   out_467723448142448616[40] = 0;
   out_467723448142448616[41] = 0;
   out_467723448142448616[42] = 0;
   out_467723448142448616[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_467723448142448616[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_467723448142448616[45] = 0;
   out_467723448142448616[46] = 0;
   out_467723448142448616[47] = 0;
   out_467723448142448616[48] = 0;
   out_467723448142448616[49] = 0;
   out_467723448142448616[50] = 0;
   out_467723448142448616[51] = 0;
   out_467723448142448616[52] = 0;
   out_467723448142448616[53] = 0;
   out_467723448142448616[54] = 0;
   out_467723448142448616[55] = 0;
   out_467723448142448616[56] = 0;
   out_467723448142448616[57] = 1;
   out_467723448142448616[58] = 0;
   out_467723448142448616[59] = 0;
   out_467723448142448616[60] = 0;
   out_467723448142448616[61] = 0;
   out_467723448142448616[62] = 0;
   out_467723448142448616[63] = 0;
   out_467723448142448616[64] = 0;
   out_467723448142448616[65] = 0;
   out_467723448142448616[66] = dt;
   out_467723448142448616[67] = 0;
   out_467723448142448616[68] = 0;
   out_467723448142448616[69] = 0;
   out_467723448142448616[70] = 0;
   out_467723448142448616[71] = 0;
   out_467723448142448616[72] = 0;
   out_467723448142448616[73] = 0;
   out_467723448142448616[74] = 0;
   out_467723448142448616[75] = 0;
   out_467723448142448616[76] = 1;
   out_467723448142448616[77] = 0;
   out_467723448142448616[78] = 0;
   out_467723448142448616[79] = 0;
   out_467723448142448616[80] = 0;
   out_467723448142448616[81] = 0;
   out_467723448142448616[82] = 0;
   out_467723448142448616[83] = 0;
   out_467723448142448616[84] = 0;
   out_467723448142448616[85] = dt;
   out_467723448142448616[86] = 0;
   out_467723448142448616[87] = 0;
   out_467723448142448616[88] = 0;
   out_467723448142448616[89] = 0;
   out_467723448142448616[90] = 0;
   out_467723448142448616[91] = 0;
   out_467723448142448616[92] = 0;
   out_467723448142448616[93] = 0;
   out_467723448142448616[94] = 0;
   out_467723448142448616[95] = 1;
   out_467723448142448616[96] = 0;
   out_467723448142448616[97] = 0;
   out_467723448142448616[98] = 0;
   out_467723448142448616[99] = 0;
   out_467723448142448616[100] = 0;
   out_467723448142448616[101] = 0;
   out_467723448142448616[102] = 0;
   out_467723448142448616[103] = 0;
   out_467723448142448616[104] = dt;
   out_467723448142448616[105] = 0;
   out_467723448142448616[106] = 0;
   out_467723448142448616[107] = 0;
   out_467723448142448616[108] = 0;
   out_467723448142448616[109] = 0;
   out_467723448142448616[110] = 0;
   out_467723448142448616[111] = 0;
   out_467723448142448616[112] = 0;
   out_467723448142448616[113] = 0;
   out_467723448142448616[114] = 1;
   out_467723448142448616[115] = 0;
   out_467723448142448616[116] = 0;
   out_467723448142448616[117] = 0;
   out_467723448142448616[118] = 0;
   out_467723448142448616[119] = 0;
   out_467723448142448616[120] = 0;
   out_467723448142448616[121] = 0;
   out_467723448142448616[122] = 0;
   out_467723448142448616[123] = 0;
   out_467723448142448616[124] = 0;
   out_467723448142448616[125] = 0;
   out_467723448142448616[126] = 0;
   out_467723448142448616[127] = 0;
   out_467723448142448616[128] = 0;
   out_467723448142448616[129] = 0;
   out_467723448142448616[130] = 0;
   out_467723448142448616[131] = 0;
   out_467723448142448616[132] = 0;
   out_467723448142448616[133] = 1;
   out_467723448142448616[134] = 0;
   out_467723448142448616[135] = 0;
   out_467723448142448616[136] = 0;
   out_467723448142448616[137] = 0;
   out_467723448142448616[138] = 0;
   out_467723448142448616[139] = 0;
   out_467723448142448616[140] = 0;
   out_467723448142448616[141] = 0;
   out_467723448142448616[142] = 0;
   out_467723448142448616[143] = 0;
   out_467723448142448616[144] = 0;
   out_467723448142448616[145] = 0;
   out_467723448142448616[146] = 0;
   out_467723448142448616[147] = 0;
   out_467723448142448616[148] = 0;
   out_467723448142448616[149] = 0;
   out_467723448142448616[150] = 0;
   out_467723448142448616[151] = 0;
   out_467723448142448616[152] = 1;
   out_467723448142448616[153] = 0;
   out_467723448142448616[154] = 0;
   out_467723448142448616[155] = 0;
   out_467723448142448616[156] = 0;
   out_467723448142448616[157] = 0;
   out_467723448142448616[158] = 0;
   out_467723448142448616[159] = 0;
   out_467723448142448616[160] = 0;
   out_467723448142448616[161] = 0;
   out_467723448142448616[162] = 0;
   out_467723448142448616[163] = 0;
   out_467723448142448616[164] = 0;
   out_467723448142448616[165] = 0;
   out_467723448142448616[166] = 0;
   out_467723448142448616[167] = 0;
   out_467723448142448616[168] = 0;
   out_467723448142448616[169] = 0;
   out_467723448142448616[170] = 0;
   out_467723448142448616[171] = 1;
   out_467723448142448616[172] = 0;
   out_467723448142448616[173] = 0;
   out_467723448142448616[174] = 0;
   out_467723448142448616[175] = 0;
   out_467723448142448616[176] = 0;
   out_467723448142448616[177] = 0;
   out_467723448142448616[178] = 0;
   out_467723448142448616[179] = 0;
   out_467723448142448616[180] = 0;
   out_467723448142448616[181] = 0;
   out_467723448142448616[182] = 0;
   out_467723448142448616[183] = 0;
   out_467723448142448616[184] = 0;
   out_467723448142448616[185] = 0;
   out_467723448142448616[186] = 0;
   out_467723448142448616[187] = 0;
   out_467723448142448616[188] = 0;
   out_467723448142448616[189] = 0;
   out_467723448142448616[190] = 1;
   out_467723448142448616[191] = 0;
   out_467723448142448616[192] = 0;
   out_467723448142448616[193] = 0;
   out_467723448142448616[194] = 0;
   out_467723448142448616[195] = 0;
   out_467723448142448616[196] = 0;
   out_467723448142448616[197] = 0;
   out_467723448142448616[198] = 0;
   out_467723448142448616[199] = 0;
   out_467723448142448616[200] = 0;
   out_467723448142448616[201] = 0;
   out_467723448142448616[202] = 0;
   out_467723448142448616[203] = 0;
   out_467723448142448616[204] = 0;
   out_467723448142448616[205] = 0;
   out_467723448142448616[206] = 0;
   out_467723448142448616[207] = 0;
   out_467723448142448616[208] = 0;
   out_467723448142448616[209] = 1;
   out_467723448142448616[210] = 0;
   out_467723448142448616[211] = 0;
   out_467723448142448616[212] = 0;
   out_467723448142448616[213] = 0;
   out_467723448142448616[214] = 0;
   out_467723448142448616[215] = 0;
   out_467723448142448616[216] = 0;
   out_467723448142448616[217] = 0;
   out_467723448142448616[218] = 0;
   out_467723448142448616[219] = 0;
   out_467723448142448616[220] = 0;
   out_467723448142448616[221] = 0;
   out_467723448142448616[222] = 0;
   out_467723448142448616[223] = 0;
   out_467723448142448616[224] = 0;
   out_467723448142448616[225] = 0;
   out_467723448142448616[226] = 0;
   out_467723448142448616[227] = 0;
   out_467723448142448616[228] = 1;
   out_467723448142448616[229] = 0;
   out_467723448142448616[230] = 0;
   out_467723448142448616[231] = 0;
   out_467723448142448616[232] = 0;
   out_467723448142448616[233] = 0;
   out_467723448142448616[234] = 0;
   out_467723448142448616[235] = 0;
   out_467723448142448616[236] = 0;
   out_467723448142448616[237] = 0;
   out_467723448142448616[238] = 0;
   out_467723448142448616[239] = 0;
   out_467723448142448616[240] = 0;
   out_467723448142448616[241] = 0;
   out_467723448142448616[242] = 0;
   out_467723448142448616[243] = 0;
   out_467723448142448616[244] = 0;
   out_467723448142448616[245] = 0;
   out_467723448142448616[246] = 0;
   out_467723448142448616[247] = 1;
   out_467723448142448616[248] = 0;
   out_467723448142448616[249] = 0;
   out_467723448142448616[250] = 0;
   out_467723448142448616[251] = 0;
   out_467723448142448616[252] = 0;
   out_467723448142448616[253] = 0;
   out_467723448142448616[254] = 0;
   out_467723448142448616[255] = 0;
   out_467723448142448616[256] = 0;
   out_467723448142448616[257] = 0;
   out_467723448142448616[258] = 0;
   out_467723448142448616[259] = 0;
   out_467723448142448616[260] = 0;
   out_467723448142448616[261] = 0;
   out_467723448142448616[262] = 0;
   out_467723448142448616[263] = 0;
   out_467723448142448616[264] = 0;
   out_467723448142448616[265] = 0;
   out_467723448142448616[266] = 1;
   out_467723448142448616[267] = 0;
   out_467723448142448616[268] = 0;
   out_467723448142448616[269] = 0;
   out_467723448142448616[270] = 0;
   out_467723448142448616[271] = 0;
   out_467723448142448616[272] = 0;
   out_467723448142448616[273] = 0;
   out_467723448142448616[274] = 0;
   out_467723448142448616[275] = 0;
   out_467723448142448616[276] = 0;
   out_467723448142448616[277] = 0;
   out_467723448142448616[278] = 0;
   out_467723448142448616[279] = 0;
   out_467723448142448616[280] = 0;
   out_467723448142448616[281] = 0;
   out_467723448142448616[282] = 0;
   out_467723448142448616[283] = 0;
   out_467723448142448616[284] = 0;
   out_467723448142448616[285] = 1;
   out_467723448142448616[286] = 0;
   out_467723448142448616[287] = 0;
   out_467723448142448616[288] = 0;
   out_467723448142448616[289] = 0;
   out_467723448142448616[290] = 0;
   out_467723448142448616[291] = 0;
   out_467723448142448616[292] = 0;
   out_467723448142448616[293] = 0;
   out_467723448142448616[294] = 0;
   out_467723448142448616[295] = 0;
   out_467723448142448616[296] = 0;
   out_467723448142448616[297] = 0;
   out_467723448142448616[298] = 0;
   out_467723448142448616[299] = 0;
   out_467723448142448616[300] = 0;
   out_467723448142448616[301] = 0;
   out_467723448142448616[302] = 0;
   out_467723448142448616[303] = 0;
   out_467723448142448616[304] = 1;
   out_467723448142448616[305] = 0;
   out_467723448142448616[306] = 0;
   out_467723448142448616[307] = 0;
   out_467723448142448616[308] = 0;
   out_467723448142448616[309] = 0;
   out_467723448142448616[310] = 0;
   out_467723448142448616[311] = 0;
   out_467723448142448616[312] = 0;
   out_467723448142448616[313] = 0;
   out_467723448142448616[314] = 0;
   out_467723448142448616[315] = 0;
   out_467723448142448616[316] = 0;
   out_467723448142448616[317] = 0;
   out_467723448142448616[318] = 0;
   out_467723448142448616[319] = 0;
   out_467723448142448616[320] = 0;
   out_467723448142448616[321] = 0;
   out_467723448142448616[322] = 0;
   out_467723448142448616[323] = 1;
}
void h_4(double *state, double *unused, double *out_8344643797140242233) {
   out_8344643797140242233[0] = state[6] + state[9];
   out_8344643797140242233[1] = state[7] + state[10];
   out_8344643797140242233[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_5018543989102965366) {
   out_5018543989102965366[0] = 0;
   out_5018543989102965366[1] = 0;
   out_5018543989102965366[2] = 0;
   out_5018543989102965366[3] = 0;
   out_5018543989102965366[4] = 0;
   out_5018543989102965366[5] = 0;
   out_5018543989102965366[6] = 1;
   out_5018543989102965366[7] = 0;
   out_5018543989102965366[8] = 0;
   out_5018543989102965366[9] = 1;
   out_5018543989102965366[10] = 0;
   out_5018543989102965366[11] = 0;
   out_5018543989102965366[12] = 0;
   out_5018543989102965366[13] = 0;
   out_5018543989102965366[14] = 0;
   out_5018543989102965366[15] = 0;
   out_5018543989102965366[16] = 0;
   out_5018543989102965366[17] = 0;
   out_5018543989102965366[18] = 0;
   out_5018543989102965366[19] = 0;
   out_5018543989102965366[20] = 0;
   out_5018543989102965366[21] = 0;
   out_5018543989102965366[22] = 0;
   out_5018543989102965366[23] = 0;
   out_5018543989102965366[24] = 0;
   out_5018543989102965366[25] = 1;
   out_5018543989102965366[26] = 0;
   out_5018543989102965366[27] = 0;
   out_5018543989102965366[28] = 1;
   out_5018543989102965366[29] = 0;
   out_5018543989102965366[30] = 0;
   out_5018543989102965366[31] = 0;
   out_5018543989102965366[32] = 0;
   out_5018543989102965366[33] = 0;
   out_5018543989102965366[34] = 0;
   out_5018543989102965366[35] = 0;
   out_5018543989102965366[36] = 0;
   out_5018543989102965366[37] = 0;
   out_5018543989102965366[38] = 0;
   out_5018543989102965366[39] = 0;
   out_5018543989102965366[40] = 0;
   out_5018543989102965366[41] = 0;
   out_5018543989102965366[42] = 0;
   out_5018543989102965366[43] = 0;
   out_5018543989102965366[44] = 1;
   out_5018543989102965366[45] = 0;
   out_5018543989102965366[46] = 0;
   out_5018543989102965366[47] = 1;
   out_5018543989102965366[48] = 0;
   out_5018543989102965366[49] = 0;
   out_5018543989102965366[50] = 0;
   out_5018543989102965366[51] = 0;
   out_5018543989102965366[52] = 0;
   out_5018543989102965366[53] = 0;
}
void h_10(double *state, double *unused, double *out_2557953024540116991) {
   out_2557953024540116991[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2557953024540116991[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2557953024540116991[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3761195516917964483) {
   out_3761195516917964483[0] = 0;
   out_3761195516917964483[1] = 9.8100000000000005*cos(state[1]);
   out_3761195516917964483[2] = 0;
   out_3761195516917964483[3] = 0;
   out_3761195516917964483[4] = -state[8];
   out_3761195516917964483[5] = state[7];
   out_3761195516917964483[6] = 0;
   out_3761195516917964483[7] = state[5];
   out_3761195516917964483[8] = -state[4];
   out_3761195516917964483[9] = 0;
   out_3761195516917964483[10] = 0;
   out_3761195516917964483[11] = 0;
   out_3761195516917964483[12] = 1;
   out_3761195516917964483[13] = 0;
   out_3761195516917964483[14] = 0;
   out_3761195516917964483[15] = 1;
   out_3761195516917964483[16] = 0;
   out_3761195516917964483[17] = 0;
   out_3761195516917964483[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3761195516917964483[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3761195516917964483[20] = 0;
   out_3761195516917964483[21] = state[8];
   out_3761195516917964483[22] = 0;
   out_3761195516917964483[23] = -state[6];
   out_3761195516917964483[24] = -state[5];
   out_3761195516917964483[25] = 0;
   out_3761195516917964483[26] = state[3];
   out_3761195516917964483[27] = 0;
   out_3761195516917964483[28] = 0;
   out_3761195516917964483[29] = 0;
   out_3761195516917964483[30] = 0;
   out_3761195516917964483[31] = 1;
   out_3761195516917964483[32] = 0;
   out_3761195516917964483[33] = 0;
   out_3761195516917964483[34] = 1;
   out_3761195516917964483[35] = 0;
   out_3761195516917964483[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3761195516917964483[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3761195516917964483[38] = 0;
   out_3761195516917964483[39] = -state[7];
   out_3761195516917964483[40] = state[6];
   out_3761195516917964483[41] = 0;
   out_3761195516917964483[42] = state[4];
   out_3761195516917964483[43] = -state[3];
   out_3761195516917964483[44] = 0;
   out_3761195516917964483[45] = 0;
   out_3761195516917964483[46] = 0;
   out_3761195516917964483[47] = 0;
   out_3761195516917964483[48] = 0;
   out_3761195516917964483[49] = 0;
   out_3761195516917964483[50] = 1;
   out_3761195516917964483[51] = 0;
   out_3761195516917964483[52] = 0;
   out_3761195516917964483[53] = 1;
}
void h_13(double *state, double *unused, double *out_4615145277794404057) {
   out_4615145277794404057[0] = state[3];
   out_4615145277794404057[1] = state[4];
   out_4615145277794404057[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5817568876289885321) {
   out_5817568876289885321[0] = 0;
   out_5817568876289885321[1] = 0;
   out_5817568876289885321[2] = 0;
   out_5817568876289885321[3] = 1;
   out_5817568876289885321[4] = 0;
   out_5817568876289885321[5] = 0;
   out_5817568876289885321[6] = 0;
   out_5817568876289885321[7] = 0;
   out_5817568876289885321[8] = 0;
   out_5817568876289885321[9] = 0;
   out_5817568876289885321[10] = 0;
   out_5817568876289885321[11] = 0;
   out_5817568876289885321[12] = 0;
   out_5817568876289885321[13] = 0;
   out_5817568876289885321[14] = 0;
   out_5817568876289885321[15] = 0;
   out_5817568876289885321[16] = 0;
   out_5817568876289885321[17] = 0;
   out_5817568876289885321[18] = 0;
   out_5817568876289885321[19] = 0;
   out_5817568876289885321[20] = 0;
   out_5817568876289885321[21] = 0;
   out_5817568876289885321[22] = 1;
   out_5817568876289885321[23] = 0;
   out_5817568876289885321[24] = 0;
   out_5817568876289885321[25] = 0;
   out_5817568876289885321[26] = 0;
   out_5817568876289885321[27] = 0;
   out_5817568876289885321[28] = 0;
   out_5817568876289885321[29] = 0;
   out_5817568876289885321[30] = 0;
   out_5817568876289885321[31] = 0;
   out_5817568876289885321[32] = 0;
   out_5817568876289885321[33] = 0;
   out_5817568876289885321[34] = 0;
   out_5817568876289885321[35] = 0;
   out_5817568876289885321[36] = 0;
   out_5817568876289885321[37] = 0;
   out_5817568876289885321[38] = 0;
   out_5817568876289885321[39] = 0;
   out_5817568876289885321[40] = 0;
   out_5817568876289885321[41] = 1;
   out_5817568876289885321[42] = 0;
   out_5817568876289885321[43] = 0;
   out_5817568876289885321[44] = 0;
   out_5817568876289885321[45] = 0;
   out_5817568876289885321[46] = 0;
   out_5817568876289885321[47] = 0;
   out_5817568876289885321[48] = 0;
   out_5817568876289885321[49] = 0;
   out_5817568876289885321[50] = 0;
   out_5817568876289885321[51] = 0;
   out_5817568876289885321[52] = 0;
   out_5817568876289885321[53] = 0;
}
void h_14(double *state, double *unused, double *out_4531057776464354787) {
   out_4531057776464354787[0] = state[6];
   out_4531057776464354787[1] = state[7];
   out_4531057776464354787[2] = state[8];
}
void H_14(double *state, double *unused, double *out_8981784845442449895) {
   out_8981784845442449895[0] = 0;
   out_8981784845442449895[1] = 0;
   out_8981784845442449895[2] = 0;
   out_8981784845442449895[3] = 0;
   out_8981784845442449895[4] = 0;
   out_8981784845442449895[5] = 0;
   out_8981784845442449895[6] = 1;
   out_8981784845442449895[7] = 0;
   out_8981784845442449895[8] = 0;
   out_8981784845442449895[9] = 0;
   out_8981784845442449895[10] = 0;
   out_8981784845442449895[11] = 0;
   out_8981784845442449895[12] = 0;
   out_8981784845442449895[13] = 0;
   out_8981784845442449895[14] = 0;
   out_8981784845442449895[15] = 0;
   out_8981784845442449895[16] = 0;
   out_8981784845442449895[17] = 0;
   out_8981784845442449895[18] = 0;
   out_8981784845442449895[19] = 0;
   out_8981784845442449895[20] = 0;
   out_8981784845442449895[21] = 0;
   out_8981784845442449895[22] = 0;
   out_8981784845442449895[23] = 0;
   out_8981784845442449895[24] = 0;
   out_8981784845442449895[25] = 1;
   out_8981784845442449895[26] = 0;
   out_8981784845442449895[27] = 0;
   out_8981784845442449895[28] = 0;
   out_8981784845442449895[29] = 0;
   out_8981784845442449895[30] = 0;
   out_8981784845442449895[31] = 0;
   out_8981784845442449895[32] = 0;
   out_8981784845442449895[33] = 0;
   out_8981784845442449895[34] = 0;
   out_8981784845442449895[35] = 0;
   out_8981784845442449895[36] = 0;
   out_8981784845442449895[37] = 0;
   out_8981784845442449895[38] = 0;
   out_8981784845442449895[39] = 0;
   out_8981784845442449895[40] = 0;
   out_8981784845442449895[41] = 0;
   out_8981784845442449895[42] = 0;
   out_8981784845442449895[43] = 0;
   out_8981784845442449895[44] = 1;
   out_8981784845442449895[45] = 0;
   out_8981784845442449895[46] = 0;
   out_8981784845442449895[47] = 0;
   out_8981784845442449895[48] = 0;
   out_8981784845442449895[49] = 0;
   out_8981784845442449895[50] = 0;
   out_8981784845442449895[51] = 0;
   out_8981784845442449895[52] = 0;
   out_8981784845442449895[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_5997402376607838659) {
  err_fun(nom_x, delta_x, out_5997402376607838659);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4197439267135692258) {
  inv_err_fun(nom_x, true_x, out_4197439267135692258);
}
void pose_H_mod_fun(double *state, double *out_3181760894870259401) {
  H_mod_fun(state, out_3181760894870259401);
}
void pose_f_fun(double *state, double dt, double *out_5584365400920044031) {
  f_fun(state,  dt, out_5584365400920044031);
}
void pose_F_fun(double *state, double dt, double *out_467723448142448616) {
  F_fun(state,  dt, out_467723448142448616);
}
void pose_h_4(double *state, double *unused, double *out_8344643797140242233) {
  h_4(state, unused, out_8344643797140242233);
}
void pose_H_4(double *state, double *unused, double *out_5018543989102965366) {
  H_4(state, unused, out_5018543989102965366);
}
void pose_h_10(double *state, double *unused, double *out_2557953024540116991) {
  h_10(state, unused, out_2557953024540116991);
}
void pose_H_10(double *state, double *unused, double *out_3761195516917964483) {
  H_10(state, unused, out_3761195516917964483);
}
void pose_h_13(double *state, double *unused, double *out_4615145277794404057) {
  h_13(state, unused, out_4615145277794404057);
}
void pose_H_13(double *state, double *unused, double *out_5817568876289885321) {
  H_13(state, unused, out_5817568876289885321);
}
void pose_h_14(double *state, double *unused, double *out_4531057776464354787) {
  h_14(state, unused, out_4531057776464354787);
}
void pose_H_14(double *state, double *unused, double *out_8981784845442449895) {
  H_14(state, unused, out_8981784845442449895);
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
