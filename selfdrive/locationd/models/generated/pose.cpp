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
void err_fun(double *nom_x, double *delta_x, double *out_4436213307949090908) {
   out_4436213307949090908[0] = delta_x[0] + nom_x[0];
   out_4436213307949090908[1] = delta_x[1] + nom_x[1];
   out_4436213307949090908[2] = delta_x[2] + nom_x[2];
   out_4436213307949090908[3] = delta_x[3] + nom_x[3];
   out_4436213307949090908[4] = delta_x[4] + nom_x[4];
   out_4436213307949090908[5] = delta_x[5] + nom_x[5];
   out_4436213307949090908[6] = delta_x[6] + nom_x[6];
   out_4436213307949090908[7] = delta_x[7] + nom_x[7];
   out_4436213307949090908[8] = delta_x[8] + nom_x[8];
   out_4436213307949090908[9] = delta_x[9] + nom_x[9];
   out_4436213307949090908[10] = delta_x[10] + nom_x[10];
   out_4436213307949090908[11] = delta_x[11] + nom_x[11];
   out_4436213307949090908[12] = delta_x[12] + nom_x[12];
   out_4436213307949090908[13] = delta_x[13] + nom_x[13];
   out_4436213307949090908[14] = delta_x[14] + nom_x[14];
   out_4436213307949090908[15] = delta_x[15] + nom_x[15];
   out_4436213307949090908[16] = delta_x[16] + nom_x[16];
   out_4436213307949090908[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4288605168489656967) {
   out_4288605168489656967[0] = -nom_x[0] + true_x[0];
   out_4288605168489656967[1] = -nom_x[1] + true_x[1];
   out_4288605168489656967[2] = -nom_x[2] + true_x[2];
   out_4288605168489656967[3] = -nom_x[3] + true_x[3];
   out_4288605168489656967[4] = -nom_x[4] + true_x[4];
   out_4288605168489656967[5] = -nom_x[5] + true_x[5];
   out_4288605168489656967[6] = -nom_x[6] + true_x[6];
   out_4288605168489656967[7] = -nom_x[7] + true_x[7];
   out_4288605168489656967[8] = -nom_x[8] + true_x[8];
   out_4288605168489656967[9] = -nom_x[9] + true_x[9];
   out_4288605168489656967[10] = -nom_x[10] + true_x[10];
   out_4288605168489656967[11] = -nom_x[11] + true_x[11];
   out_4288605168489656967[12] = -nom_x[12] + true_x[12];
   out_4288605168489656967[13] = -nom_x[13] + true_x[13];
   out_4288605168489656967[14] = -nom_x[14] + true_x[14];
   out_4288605168489656967[15] = -nom_x[15] + true_x[15];
   out_4288605168489656967[16] = -nom_x[16] + true_x[16];
   out_4288605168489656967[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6602564711839087419) {
   out_6602564711839087419[0] = 1.0;
   out_6602564711839087419[1] = 0.0;
   out_6602564711839087419[2] = 0.0;
   out_6602564711839087419[3] = 0.0;
   out_6602564711839087419[4] = 0.0;
   out_6602564711839087419[5] = 0.0;
   out_6602564711839087419[6] = 0.0;
   out_6602564711839087419[7] = 0.0;
   out_6602564711839087419[8] = 0.0;
   out_6602564711839087419[9] = 0.0;
   out_6602564711839087419[10] = 0.0;
   out_6602564711839087419[11] = 0.0;
   out_6602564711839087419[12] = 0.0;
   out_6602564711839087419[13] = 0.0;
   out_6602564711839087419[14] = 0.0;
   out_6602564711839087419[15] = 0.0;
   out_6602564711839087419[16] = 0.0;
   out_6602564711839087419[17] = 0.0;
   out_6602564711839087419[18] = 0.0;
   out_6602564711839087419[19] = 1.0;
   out_6602564711839087419[20] = 0.0;
   out_6602564711839087419[21] = 0.0;
   out_6602564711839087419[22] = 0.0;
   out_6602564711839087419[23] = 0.0;
   out_6602564711839087419[24] = 0.0;
   out_6602564711839087419[25] = 0.0;
   out_6602564711839087419[26] = 0.0;
   out_6602564711839087419[27] = 0.0;
   out_6602564711839087419[28] = 0.0;
   out_6602564711839087419[29] = 0.0;
   out_6602564711839087419[30] = 0.0;
   out_6602564711839087419[31] = 0.0;
   out_6602564711839087419[32] = 0.0;
   out_6602564711839087419[33] = 0.0;
   out_6602564711839087419[34] = 0.0;
   out_6602564711839087419[35] = 0.0;
   out_6602564711839087419[36] = 0.0;
   out_6602564711839087419[37] = 0.0;
   out_6602564711839087419[38] = 1.0;
   out_6602564711839087419[39] = 0.0;
   out_6602564711839087419[40] = 0.0;
   out_6602564711839087419[41] = 0.0;
   out_6602564711839087419[42] = 0.0;
   out_6602564711839087419[43] = 0.0;
   out_6602564711839087419[44] = 0.0;
   out_6602564711839087419[45] = 0.0;
   out_6602564711839087419[46] = 0.0;
   out_6602564711839087419[47] = 0.0;
   out_6602564711839087419[48] = 0.0;
   out_6602564711839087419[49] = 0.0;
   out_6602564711839087419[50] = 0.0;
   out_6602564711839087419[51] = 0.0;
   out_6602564711839087419[52] = 0.0;
   out_6602564711839087419[53] = 0.0;
   out_6602564711839087419[54] = 0.0;
   out_6602564711839087419[55] = 0.0;
   out_6602564711839087419[56] = 0.0;
   out_6602564711839087419[57] = 1.0;
   out_6602564711839087419[58] = 0.0;
   out_6602564711839087419[59] = 0.0;
   out_6602564711839087419[60] = 0.0;
   out_6602564711839087419[61] = 0.0;
   out_6602564711839087419[62] = 0.0;
   out_6602564711839087419[63] = 0.0;
   out_6602564711839087419[64] = 0.0;
   out_6602564711839087419[65] = 0.0;
   out_6602564711839087419[66] = 0.0;
   out_6602564711839087419[67] = 0.0;
   out_6602564711839087419[68] = 0.0;
   out_6602564711839087419[69] = 0.0;
   out_6602564711839087419[70] = 0.0;
   out_6602564711839087419[71] = 0.0;
   out_6602564711839087419[72] = 0.0;
   out_6602564711839087419[73] = 0.0;
   out_6602564711839087419[74] = 0.0;
   out_6602564711839087419[75] = 0.0;
   out_6602564711839087419[76] = 1.0;
   out_6602564711839087419[77] = 0.0;
   out_6602564711839087419[78] = 0.0;
   out_6602564711839087419[79] = 0.0;
   out_6602564711839087419[80] = 0.0;
   out_6602564711839087419[81] = 0.0;
   out_6602564711839087419[82] = 0.0;
   out_6602564711839087419[83] = 0.0;
   out_6602564711839087419[84] = 0.0;
   out_6602564711839087419[85] = 0.0;
   out_6602564711839087419[86] = 0.0;
   out_6602564711839087419[87] = 0.0;
   out_6602564711839087419[88] = 0.0;
   out_6602564711839087419[89] = 0.0;
   out_6602564711839087419[90] = 0.0;
   out_6602564711839087419[91] = 0.0;
   out_6602564711839087419[92] = 0.0;
   out_6602564711839087419[93] = 0.0;
   out_6602564711839087419[94] = 0.0;
   out_6602564711839087419[95] = 1.0;
   out_6602564711839087419[96] = 0.0;
   out_6602564711839087419[97] = 0.0;
   out_6602564711839087419[98] = 0.0;
   out_6602564711839087419[99] = 0.0;
   out_6602564711839087419[100] = 0.0;
   out_6602564711839087419[101] = 0.0;
   out_6602564711839087419[102] = 0.0;
   out_6602564711839087419[103] = 0.0;
   out_6602564711839087419[104] = 0.0;
   out_6602564711839087419[105] = 0.0;
   out_6602564711839087419[106] = 0.0;
   out_6602564711839087419[107] = 0.0;
   out_6602564711839087419[108] = 0.0;
   out_6602564711839087419[109] = 0.0;
   out_6602564711839087419[110] = 0.0;
   out_6602564711839087419[111] = 0.0;
   out_6602564711839087419[112] = 0.0;
   out_6602564711839087419[113] = 0.0;
   out_6602564711839087419[114] = 1.0;
   out_6602564711839087419[115] = 0.0;
   out_6602564711839087419[116] = 0.0;
   out_6602564711839087419[117] = 0.0;
   out_6602564711839087419[118] = 0.0;
   out_6602564711839087419[119] = 0.0;
   out_6602564711839087419[120] = 0.0;
   out_6602564711839087419[121] = 0.0;
   out_6602564711839087419[122] = 0.0;
   out_6602564711839087419[123] = 0.0;
   out_6602564711839087419[124] = 0.0;
   out_6602564711839087419[125] = 0.0;
   out_6602564711839087419[126] = 0.0;
   out_6602564711839087419[127] = 0.0;
   out_6602564711839087419[128] = 0.0;
   out_6602564711839087419[129] = 0.0;
   out_6602564711839087419[130] = 0.0;
   out_6602564711839087419[131] = 0.0;
   out_6602564711839087419[132] = 0.0;
   out_6602564711839087419[133] = 1.0;
   out_6602564711839087419[134] = 0.0;
   out_6602564711839087419[135] = 0.0;
   out_6602564711839087419[136] = 0.0;
   out_6602564711839087419[137] = 0.0;
   out_6602564711839087419[138] = 0.0;
   out_6602564711839087419[139] = 0.0;
   out_6602564711839087419[140] = 0.0;
   out_6602564711839087419[141] = 0.0;
   out_6602564711839087419[142] = 0.0;
   out_6602564711839087419[143] = 0.0;
   out_6602564711839087419[144] = 0.0;
   out_6602564711839087419[145] = 0.0;
   out_6602564711839087419[146] = 0.0;
   out_6602564711839087419[147] = 0.0;
   out_6602564711839087419[148] = 0.0;
   out_6602564711839087419[149] = 0.0;
   out_6602564711839087419[150] = 0.0;
   out_6602564711839087419[151] = 0.0;
   out_6602564711839087419[152] = 1.0;
   out_6602564711839087419[153] = 0.0;
   out_6602564711839087419[154] = 0.0;
   out_6602564711839087419[155] = 0.0;
   out_6602564711839087419[156] = 0.0;
   out_6602564711839087419[157] = 0.0;
   out_6602564711839087419[158] = 0.0;
   out_6602564711839087419[159] = 0.0;
   out_6602564711839087419[160] = 0.0;
   out_6602564711839087419[161] = 0.0;
   out_6602564711839087419[162] = 0.0;
   out_6602564711839087419[163] = 0.0;
   out_6602564711839087419[164] = 0.0;
   out_6602564711839087419[165] = 0.0;
   out_6602564711839087419[166] = 0.0;
   out_6602564711839087419[167] = 0.0;
   out_6602564711839087419[168] = 0.0;
   out_6602564711839087419[169] = 0.0;
   out_6602564711839087419[170] = 0.0;
   out_6602564711839087419[171] = 1.0;
   out_6602564711839087419[172] = 0.0;
   out_6602564711839087419[173] = 0.0;
   out_6602564711839087419[174] = 0.0;
   out_6602564711839087419[175] = 0.0;
   out_6602564711839087419[176] = 0.0;
   out_6602564711839087419[177] = 0.0;
   out_6602564711839087419[178] = 0.0;
   out_6602564711839087419[179] = 0.0;
   out_6602564711839087419[180] = 0.0;
   out_6602564711839087419[181] = 0.0;
   out_6602564711839087419[182] = 0.0;
   out_6602564711839087419[183] = 0.0;
   out_6602564711839087419[184] = 0.0;
   out_6602564711839087419[185] = 0.0;
   out_6602564711839087419[186] = 0.0;
   out_6602564711839087419[187] = 0.0;
   out_6602564711839087419[188] = 0.0;
   out_6602564711839087419[189] = 0.0;
   out_6602564711839087419[190] = 1.0;
   out_6602564711839087419[191] = 0.0;
   out_6602564711839087419[192] = 0.0;
   out_6602564711839087419[193] = 0.0;
   out_6602564711839087419[194] = 0.0;
   out_6602564711839087419[195] = 0.0;
   out_6602564711839087419[196] = 0.0;
   out_6602564711839087419[197] = 0.0;
   out_6602564711839087419[198] = 0.0;
   out_6602564711839087419[199] = 0.0;
   out_6602564711839087419[200] = 0.0;
   out_6602564711839087419[201] = 0.0;
   out_6602564711839087419[202] = 0.0;
   out_6602564711839087419[203] = 0.0;
   out_6602564711839087419[204] = 0.0;
   out_6602564711839087419[205] = 0.0;
   out_6602564711839087419[206] = 0.0;
   out_6602564711839087419[207] = 0.0;
   out_6602564711839087419[208] = 0.0;
   out_6602564711839087419[209] = 1.0;
   out_6602564711839087419[210] = 0.0;
   out_6602564711839087419[211] = 0.0;
   out_6602564711839087419[212] = 0.0;
   out_6602564711839087419[213] = 0.0;
   out_6602564711839087419[214] = 0.0;
   out_6602564711839087419[215] = 0.0;
   out_6602564711839087419[216] = 0.0;
   out_6602564711839087419[217] = 0.0;
   out_6602564711839087419[218] = 0.0;
   out_6602564711839087419[219] = 0.0;
   out_6602564711839087419[220] = 0.0;
   out_6602564711839087419[221] = 0.0;
   out_6602564711839087419[222] = 0.0;
   out_6602564711839087419[223] = 0.0;
   out_6602564711839087419[224] = 0.0;
   out_6602564711839087419[225] = 0.0;
   out_6602564711839087419[226] = 0.0;
   out_6602564711839087419[227] = 0.0;
   out_6602564711839087419[228] = 1.0;
   out_6602564711839087419[229] = 0.0;
   out_6602564711839087419[230] = 0.0;
   out_6602564711839087419[231] = 0.0;
   out_6602564711839087419[232] = 0.0;
   out_6602564711839087419[233] = 0.0;
   out_6602564711839087419[234] = 0.0;
   out_6602564711839087419[235] = 0.0;
   out_6602564711839087419[236] = 0.0;
   out_6602564711839087419[237] = 0.0;
   out_6602564711839087419[238] = 0.0;
   out_6602564711839087419[239] = 0.0;
   out_6602564711839087419[240] = 0.0;
   out_6602564711839087419[241] = 0.0;
   out_6602564711839087419[242] = 0.0;
   out_6602564711839087419[243] = 0.0;
   out_6602564711839087419[244] = 0.0;
   out_6602564711839087419[245] = 0.0;
   out_6602564711839087419[246] = 0.0;
   out_6602564711839087419[247] = 1.0;
   out_6602564711839087419[248] = 0.0;
   out_6602564711839087419[249] = 0.0;
   out_6602564711839087419[250] = 0.0;
   out_6602564711839087419[251] = 0.0;
   out_6602564711839087419[252] = 0.0;
   out_6602564711839087419[253] = 0.0;
   out_6602564711839087419[254] = 0.0;
   out_6602564711839087419[255] = 0.0;
   out_6602564711839087419[256] = 0.0;
   out_6602564711839087419[257] = 0.0;
   out_6602564711839087419[258] = 0.0;
   out_6602564711839087419[259] = 0.0;
   out_6602564711839087419[260] = 0.0;
   out_6602564711839087419[261] = 0.0;
   out_6602564711839087419[262] = 0.0;
   out_6602564711839087419[263] = 0.0;
   out_6602564711839087419[264] = 0.0;
   out_6602564711839087419[265] = 0.0;
   out_6602564711839087419[266] = 1.0;
   out_6602564711839087419[267] = 0.0;
   out_6602564711839087419[268] = 0.0;
   out_6602564711839087419[269] = 0.0;
   out_6602564711839087419[270] = 0.0;
   out_6602564711839087419[271] = 0.0;
   out_6602564711839087419[272] = 0.0;
   out_6602564711839087419[273] = 0.0;
   out_6602564711839087419[274] = 0.0;
   out_6602564711839087419[275] = 0.0;
   out_6602564711839087419[276] = 0.0;
   out_6602564711839087419[277] = 0.0;
   out_6602564711839087419[278] = 0.0;
   out_6602564711839087419[279] = 0.0;
   out_6602564711839087419[280] = 0.0;
   out_6602564711839087419[281] = 0.0;
   out_6602564711839087419[282] = 0.0;
   out_6602564711839087419[283] = 0.0;
   out_6602564711839087419[284] = 0.0;
   out_6602564711839087419[285] = 1.0;
   out_6602564711839087419[286] = 0.0;
   out_6602564711839087419[287] = 0.0;
   out_6602564711839087419[288] = 0.0;
   out_6602564711839087419[289] = 0.0;
   out_6602564711839087419[290] = 0.0;
   out_6602564711839087419[291] = 0.0;
   out_6602564711839087419[292] = 0.0;
   out_6602564711839087419[293] = 0.0;
   out_6602564711839087419[294] = 0.0;
   out_6602564711839087419[295] = 0.0;
   out_6602564711839087419[296] = 0.0;
   out_6602564711839087419[297] = 0.0;
   out_6602564711839087419[298] = 0.0;
   out_6602564711839087419[299] = 0.0;
   out_6602564711839087419[300] = 0.0;
   out_6602564711839087419[301] = 0.0;
   out_6602564711839087419[302] = 0.0;
   out_6602564711839087419[303] = 0.0;
   out_6602564711839087419[304] = 1.0;
   out_6602564711839087419[305] = 0.0;
   out_6602564711839087419[306] = 0.0;
   out_6602564711839087419[307] = 0.0;
   out_6602564711839087419[308] = 0.0;
   out_6602564711839087419[309] = 0.0;
   out_6602564711839087419[310] = 0.0;
   out_6602564711839087419[311] = 0.0;
   out_6602564711839087419[312] = 0.0;
   out_6602564711839087419[313] = 0.0;
   out_6602564711839087419[314] = 0.0;
   out_6602564711839087419[315] = 0.0;
   out_6602564711839087419[316] = 0.0;
   out_6602564711839087419[317] = 0.0;
   out_6602564711839087419[318] = 0.0;
   out_6602564711839087419[319] = 0.0;
   out_6602564711839087419[320] = 0.0;
   out_6602564711839087419[321] = 0.0;
   out_6602564711839087419[322] = 0.0;
   out_6602564711839087419[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_976073864278997094) {
   out_976073864278997094[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_976073864278997094[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_976073864278997094[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_976073864278997094[3] = dt*state[12] + state[3];
   out_976073864278997094[4] = dt*state[13] + state[4];
   out_976073864278997094[5] = dt*state[14] + state[5];
   out_976073864278997094[6] = state[6];
   out_976073864278997094[7] = state[7];
   out_976073864278997094[8] = state[8];
   out_976073864278997094[9] = state[9];
   out_976073864278997094[10] = state[10];
   out_976073864278997094[11] = state[11];
   out_976073864278997094[12] = state[12];
   out_976073864278997094[13] = state[13];
   out_976073864278997094[14] = state[14];
   out_976073864278997094[15] = state[15];
   out_976073864278997094[16] = state[16];
   out_976073864278997094[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3492059102621666200) {
   out_3492059102621666200[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3492059102621666200[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3492059102621666200[2] = 0;
   out_3492059102621666200[3] = 0;
   out_3492059102621666200[4] = 0;
   out_3492059102621666200[5] = 0;
   out_3492059102621666200[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3492059102621666200[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3492059102621666200[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3492059102621666200[9] = 0;
   out_3492059102621666200[10] = 0;
   out_3492059102621666200[11] = 0;
   out_3492059102621666200[12] = 0;
   out_3492059102621666200[13] = 0;
   out_3492059102621666200[14] = 0;
   out_3492059102621666200[15] = 0;
   out_3492059102621666200[16] = 0;
   out_3492059102621666200[17] = 0;
   out_3492059102621666200[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3492059102621666200[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3492059102621666200[20] = 0;
   out_3492059102621666200[21] = 0;
   out_3492059102621666200[22] = 0;
   out_3492059102621666200[23] = 0;
   out_3492059102621666200[24] = 0;
   out_3492059102621666200[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3492059102621666200[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3492059102621666200[27] = 0;
   out_3492059102621666200[28] = 0;
   out_3492059102621666200[29] = 0;
   out_3492059102621666200[30] = 0;
   out_3492059102621666200[31] = 0;
   out_3492059102621666200[32] = 0;
   out_3492059102621666200[33] = 0;
   out_3492059102621666200[34] = 0;
   out_3492059102621666200[35] = 0;
   out_3492059102621666200[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3492059102621666200[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3492059102621666200[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3492059102621666200[39] = 0;
   out_3492059102621666200[40] = 0;
   out_3492059102621666200[41] = 0;
   out_3492059102621666200[42] = 0;
   out_3492059102621666200[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3492059102621666200[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3492059102621666200[45] = 0;
   out_3492059102621666200[46] = 0;
   out_3492059102621666200[47] = 0;
   out_3492059102621666200[48] = 0;
   out_3492059102621666200[49] = 0;
   out_3492059102621666200[50] = 0;
   out_3492059102621666200[51] = 0;
   out_3492059102621666200[52] = 0;
   out_3492059102621666200[53] = 0;
   out_3492059102621666200[54] = 0;
   out_3492059102621666200[55] = 0;
   out_3492059102621666200[56] = 0;
   out_3492059102621666200[57] = 1;
   out_3492059102621666200[58] = 0;
   out_3492059102621666200[59] = 0;
   out_3492059102621666200[60] = 0;
   out_3492059102621666200[61] = 0;
   out_3492059102621666200[62] = 0;
   out_3492059102621666200[63] = 0;
   out_3492059102621666200[64] = 0;
   out_3492059102621666200[65] = 0;
   out_3492059102621666200[66] = dt;
   out_3492059102621666200[67] = 0;
   out_3492059102621666200[68] = 0;
   out_3492059102621666200[69] = 0;
   out_3492059102621666200[70] = 0;
   out_3492059102621666200[71] = 0;
   out_3492059102621666200[72] = 0;
   out_3492059102621666200[73] = 0;
   out_3492059102621666200[74] = 0;
   out_3492059102621666200[75] = 0;
   out_3492059102621666200[76] = 1;
   out_3492059102621666200[77] = 0;
   out_3492059102621666200[78] = 0;
   out_3492059102621666200[79] = 0;
   out_3492059102621666200[80] = 0;
   out_3492059102621666200[81] = 0;
   out_3492059102621666200[82] = 0;
   out_3492059102621666200[83] = 0;
   out_3492059102621666200[84] = 0;
   out_3492059102621666200[85] = dt;
   out_3492059102621666200[86] = 0;
   out_3492059102621666200[87] = 0;
   out_3492059102621666200[88] = 0;
   out_3492059102621666200[89] = 0;
   out_3492059102621666200[90] = 0;
   out_3492059102621666200[91] = 0;
   out_3492059102621666200[92] = 0;
   out_3492059102621666200[93] = 0;
   out_3492059102621666200[94] = 0;
   out_3492059102621666200[95] = 1;
   out_3492059102621666200[96] = 0;
   out_3492059102621666200[97] = 0;
   out_3492059102621666200[98] = 0;
   out_3492059102621666200[99] = 0;
   out_3492059102621666200[100] = 0;
   out_3492059102621666200[101] = 0;
   out_3492059102621666200[102] = 0;
   out_3492059102621666200[103] = 0;
   out_3492059102621666200[104] = dt;
   out_3492059102621666200[105] = 0;
   out_3492059102621666200[106] = 0;
   out_3492059102621666200[107] = 0;
   out_3492059102621666200[108] = 0;
   out_3492059102621666200[109] = 0;
   out_3492059102621666200[110] = 0;
   out_3492059102621666200[111] = 0;
   out_3492059102621666200[112] = 0;
   out_3492059102621666200[113] = 0;
   out_3492059102621666200[114] = 1;
   out_3492059102621666200[115] = 0;
   out_3492059102621666200[116] = 0;
   out_3492059102621666200[117] = 0;
   out_3492059102621666200[118] = 0;
   out_3492059102621666200[119] = 0;
   out_3492059102621666200[120] = 0;
   out_3492059102621666200[121] = 0;
   out_3492059102621666200[122] = 0;
   out_3492059102621666200[123] = 0;
   out_3492059102621666200[124] = 0;
   out_3492059102621666200[125] = 0;
   out_3492059102621666200[126] = 0;
   out_3492059102621666200[127] = 0;
   out_3492059102621666200[128] = 0;
   out_3492059102621666200[129] = 0;
   out_3492059102621666200[130] = 0;
   out_3492059102621666200[131] = 0;
   out_3492059102621666200[132] = 0;
   out_3492059102621666200[133] = 1;
   out_3492059102621666200[134] = 0;
   out_3492059102621666200[135] = 0;
   out_3492059102621666200[136] = 0;
   out_3492059102621666200[137] = 0;
   out_3492059102621666200[138] = 0;
   out_3492059102621666200[139] = 0;
   out_3492059102621666200[140] = 0;
   out_3492059102621666200[141] = 0;
   out_3492059102621666200[142] = 0;
   out_3492059102621666200[143] = 0;
   out_3492059102621666200[144] = 0;
   out_3492059102621666200[145] = 0;
   out_3492059102621666200[146] = 0;
   out_3492059102621666200[147] = 0;
   out_3492059102621666200[148] = 0;
   out_3492059102621666200[149] = 0;
   out_3492059102621666200[150] = 0;
   out_3492059102621666200[151] = 0;
   out_3492059102621666200[152] = 1;
   out_3492059102621666200[153] = 0;
   out_3492059102621666200[154] = 0;
   out_3492059102621666200[155] = 0;
   out_3492059102621666200[156] = 0;
   out_3492059102621666200[157] = 0;
   out_3492059102621666200[158] = 0;
   out_3492059102621666200[159] = 0;
   out_3492059102621666200[160] = 0;
   out_3492059102621666200[161] = 0;
   out_3492059102621666200[162] = 0;
   out_3492059102621666200[163] = 0;
   out_3492059102621666200[164] = 0;
   out_3492059102621666200[165] = 0;
   out_3492059102621666200[166] = 0;
   out_3492059102621666200[167] = 0;
   out_3492059102621666200[168] = 0;
   out_3492059102621666200[169] = 0;
   out_3492059102621666200[170] = 0;
   out_3492059102621666200[171] = 1;
   out_3492059102621666200[172] = 0;
   out_3492059102621666200[173] = 0;
   out_3492059102621666200[174] = 0;
   out_3492059102621666200[175] = 0;
   out_3492059102621666200[176] = 0;
   out_3492059102621666200[177] = 0;
   out_3492059102621666200[178] = 0;
   out_3492059102621666200[179] = 0;
   out_3492059102621666200[180] = 0;
   out_3492059102621666200[181] = 0;
   out_3492059102621666200[182] = 0;
   out_3492059102621666200[183] = 0;
   out_3492059102621666200[184] = 0;
   out_3492059102621666200[185] = 0;
   out_3492059102621666200[186] = 0;
   out_3492059102621666200[187] = 0;
   out_3492059102621666200[188] = 0;
   out_3492059102621666200[189] = 0;
   out_3492059102621666200[190] = 1;
   out_3492059102621666200[191] = 0;
   out_3492059102621666200[192] = 0;
   out_3492059102621666200[193] = 0;
   out_3492059102621666200[194] = 0;
   out_3492059102621666200[195] = 0;
   out_3492059102621666200[196] = 0;
   out_3492059102621666200[197] = 0;
   out_3492059102621666200[198] = 0;
   out_3492059102621666200[199] = 0;
   out_3492059102621666200[200] = 0;
   out_3492059102621666200[201] = 0;
   out_3492059102621666200[202] = 0;
   out_3492059102621666200[203] = 0;
   out_3492059102621666200[204] = 0;
   out_3492059102621666200[205] = 0;
   out_3492059102621666200[206] = 0;
   out_3492059102621666200[207] = 0;
   out_3492059102621666200[208] = 0;
   out_3492059102621666200[209] = 1;
   out_3492059102621666200[210] = 0;
   out_3492059102621666200[211] = 0;
   out_3492059102621666200[212] = 0;
   out_3492059102621666200[213] = 0;
   out_3492059102621666200[214] = 0;
   out_3492059102621666200[215] = 0;
   out_3492059102621666200[216] = 0;
   out_3492059102621666200[217] = 0;
   out_3492059102621666200[218] = 0;
   out_3492059102621666200[219] = 0;
   out_3492059102621666200[220] = 0;
   out_3492059102621666200[221] = 0;
   out_3492059102621666200[222] = 0;
   out_3492059102621666200[223] = 0;
   out_3492059102621666200[224] = 0;
   out_3492059102621666200[225] = 0;
   out_3492059102621666200[226] = 0;
   out_3492059102621666200[227] = 0;
   out_3492059102621666200[228] = 1;
   out_3492059102621666200[229] = 0;
   out_3492059102621666200[230] = 0;
   out_3492059102621666200[231] = 0;
   out_3492059102621666200[232] = 0;
   out_3492059102621666200[233] = 0;
   out_3492059102621666200[234] = 0;
   out_3492059102621666200[235] = 0;
   out_3492059102621666200[236] = 0;
   out_3492059102621666200[237] = 0;
   out_3492059102621666200[238] = 0;
   out_3492059102621666200[239] = 0;
   out_3492059102621666200[240] = 0;
   out_3492059102621666200[241] = 0;
   out_3492059102621666200[242] = 0;
   out_3492059102621666200[243] = 0;
   out_3492059102621666200[244] = 0;
   out_3492059102621666200[245] = 0;
   out_3492059102621666200[246] = 0;
   out_3492059102621666200[247] = 1;
   out_3492059102621666200[248] = 0;
   out_3492059102621666200[249] = 0;
   out_3492059102621666200[250] = 0;
   out_3492059102621666200[251] = 0;
   out_3492059102621666200[252] = 0;
   out_3492059102621666200[253] = 0;
   out_3492059102621666200[254] = 0;
   out_3492059102621666200[255] = 0;
   out_3492059102621666200[256] = 0;
   out_3492059102621666200[257] = 0;
   out_3492059102621666200[258] = 0;
   out_3492059102621666200[259] = 0;
   out_3492059102621666200[260] = 0;
   out_3492059102621666200[261] = 0;
   out_3492059102621666200[262] = 0;
   out_3492059102621666200[263] = 0;
   out_3492059102621666200[264] = 0;
   out_3492059102621666200[265] = 0;
   out_3492059102621666200[266] = 1;
   out_3492059102621666200[267] = 0;
   out_3492059102621666200[268] = 0;
   out_3492059102621666200[269] = 0;
   out_3492059102621666200[270] = 0;
   out_3492059102621666200[271] = 0;
   out_3492059102621666200[272] = 0;
   out_3492059102621666200[273] = 0;
   out_3492059102621666200[274] = 0;
   out_3492059102621666200[275] = 0;
   out_3492059102621666200[276] = 0;
   out_3492059102621666200[277] = 0;
   out_3492059102621666200[278] = 0;
   out_3492059102621666200[279] = 0;
   out_3492059102621666200[280] = 0;
   out_3492059102621666200[281] = 0;
   out_3492059102621666200[282] = 0;
   out_3492059102621666200[283] = 0;
   out_3492059102621666200[284] = 0;
   out_3492059102621666200[285] = 1;
   out_3492059102621666200[286] = 0;
   out_3492059102621666200[287] = 0;
   out_3492059102621666200[288] = 0;
   out_3492059102621666200[289] = 0;
   out_3492059102621666200[290] = 0;
   out_3492059102621666200[291] = 0;
   out_3492059102621666200[292] = 0;
   out_3492059102621666200[293] = 0;
   out_3492059102621666200[294] = 0;
   out_3492059102621666200[295] = 0;
   out_3492059102621666200[296] = 0;
   out_3492059102621666200[297] = 0;
   out_3492059102621666200[298] = 0;
   out_3492059102621666200[299] = 0;
   out_3492059102621666200[300] = 0;
   out_3492059102621666200[301] = 0;
   out_3492059102621666200[302] = 0;
   out_3492059102621666200[303] = 0;
   out_3492059102621666200[304] = 1;
   out_3492059102621666200[305] = 0;
   out_3492059102621666200[306] = 0;
   out_3492059102621666200[307] = 0;
   out_3492059102621666200[308] = 0;
   out_3492059102621666200[309] = 0;
   out_3492059102621666200[310] = 0;
   out_3492059102621666200[311] = 0;
   out_3492059102621666200[312] = 0;
   out_3492059102621666200[313] = 0;
   out_3492059102621666200[314] = 0;
   out_3492059102621666200[315] = 0;
   out_3492059102621666200[316] = 0;
   out_3492059102621666200[317] = 0;
   out_3492059102621666200[318] = 0;
   out_3492059102621666200[319] = 0;
   out_3492059102621666200[320] = 0;
   out_3492059102621666200[321] = 0;
   out_3492059102621666200[322] = 0;
   out_3492059102621666200[323] = 1;
}
void h_4(double *state, double *unused, double *out_4637603071142990440) {
   out_4637603071142990440[0] = state[6] + state[9];
   out_4637603071142990440[1] = state[7] + state[10];
   out_4637603071142990440[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8072372152026556625) {
   out_8072372152026556625[0] = 0;
   out_8072372152026556625[1] = 0;
   out_8072372152026556625[2] = 0;
   out_8072372152026556625[3] = 0;
   out_8072372152026556625[4] = 0;
   out_8072372152026556625[5] = 0;
   out_8072372152026556625[6] = 1;
   out_8072372152026556625[7] = 0;
   out_8072372152026556625[8] = 0;
   out_8072372152026556625[9] = 1;
   out_8072372152026556625[10] = 0;
   out_8072372152026556625[11] = 0;
   out_8072372152026556625[12] = 0;
   out_8072372152026556625[13] = 0;
   out_8072372152026556625[14] = 0;
   out_8072372152026556625[15] = 0;
   out_8072372152026556625[16] = 0;
   out_8072372152026556625[17] = 0;
   out_8072372152026556625[18] = 0;
   out_8072372152026556625[19] = 0;
   out_8072372152026556625[20] = 0;
   out_8072372152026556625[21] = 0;
   out_8072372152026556625[22] = 0;
   out_8072372152026556625[23] = 0;
   out_8072372152026556625[24] = 0;
   out_8072372152026556625[25] = 1;
   out_8072372152026556625[26] = 0;
   out_8072372152026556625[27] = 0;
   out_8072372152026556625[28] = 1;
   out_8072372152026556625[29] = 0;
   out_8072372152026556625[30] = 0;
   out_8072372152026556625[31] = 0;
   out_8072372152026556625[32] = 0;
   out_8072372152026556625[33] = 0;
   out_8072372152026556625[34] = 0;
   out_8072372152026556625[35] = 0;
   out_8072372152026556625[36] = 0;
   out_8072372152026556625[37] = 0;
   out_8072372152026556625[38] = 0;
   out_8072372152026556625[39] = 0;
   out_8072372152026556625[40] = 0;
   out_8072372152026556625[41] = 0;
   out_8072372152026556625[42] = 0;
   out_8072372152026556625[43] = 0;
   out_8072372152026556625[44] = 1;
   out_8072372152026556625[45] = 0;
   out_8072372152026556625[46] = 0;
   out_8072372152026556625[47] = 1;
   out_8072372152026556625[48] = 0;
   out_8072372152026556625[49] = 0;
   out_8072372152026556625[50] = 0;
   out_8072372152026556625[51] = 0;
   out_8072372152026556625[52] = 0;
   out_8072372152026556625[53] = 0;
}
void h_10(double *state, double *unused, double *out_6628455498773735725) {
   out_6628455498773735725[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6628455498773735725[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6628455498773735725[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4205718837824556590) {
   out_4205718837824556590[0] = 0;
   out_4205718837824556590[1] = 9.8100000000000005*cos(state[1]);
   out_4205718837824556590[2] = 0;
   out_4205718837824556590[3] = 0;
   out_4205718837824556590[4] = -state[8];
   out_4205718837824556590[5] = state[7];
   out_4205718837824556590[6] = 0;
   out_4205718837824556590[7] = state[5];
   out_4205718837824556590[8] = -state[4];
   out_4205718837824556590[9] = 0;
   out_4205718837824556590[10] = 0;
   out_4205718837824556590[11] = 0;
   out_4205718837824556590[12] = 1;
   out_4205718837824556590[13] = 0;
   out_4205718837824556590[14] = 0;
   out_4205718837824556590[15] = 1;
   out_4205718837824556590[16] = 0;
   out_4205718837824556590[17] = 0;
   out_4205718837824556590[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4205718837824556590[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4205718837824556590[20] = 0;
   out_4205718837824556590[21] = state[8];
   out_4205718837824556590[22] = 0;
   out_4205718837824556590[23] = -state[6];
   out_4205718837824556590[24] = -state[5];
   out_4205718837824556590[25] = 0;
   out_4205718837824556590[26] = state[3];
   out_4205718837824556590[27] = 0;
   out_4205718837824556590[28] = 0;
   out_4205718837824556590[29] = 0;
   out_4205718837824556590[30] = 0;
   out_4205718837824556590[31] = 1;
   out_4205718837824556590[32] = 0;
   out_4205718837824556590[33] = 0;
   out_4205718837824556590[34] = 1;
   out_4205718837824556590[35] = 0;
   out_4205718837824556590[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4205718837824556590[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4205718837824556590[38] = 0;
   out_4205718837824556590[39] = -state[7];
   out_4205718837824556590[40] = state[6];
   out_4205718837824556590[41] = 0;
   out_4205718837824556590[42] = state[4];
   out_4205718837824556590[43] = -state[3];
   out_4205718837824556590[44] = 0;
   out_4205718837824556590[45] = 0;
   out_4205718837824556590[46] = 0;
   out_4205718837824556590[47] = 0;
   out_4205718837824556590[48] = 0;
   out_4205718837824556590[49] = 0;
   out_4205718837824556590[50] = 1;
   out_4205718837824556590[51] = 0;
   out_4205718837824556590[52] = 0;
   out_4205718837824556590[53] = 1;
}
void h_13(double *state, double *unused, double *out_3016281579283853723) {
   out_3016281579283853723[0] = state[3];
   out_3016281579283853723[1] = state[4];
   out_3016281579283853723[2] = state[5];
}
void H_13(double *state, double *unused, double *out_461740943709855696) {
   out_461740943709855696[0] = 0;
   out_461740943709855696[1] = 0;
   out_461740943709855696[2] = 0;
   out_461740943709855696[3] = 1;
   out_461740943709855696[4] = 0;
   out_461740943709855696[5] = 0;
   out_461740943709855696[6] = 0;
   out_461740943709855696[7] = 0;
   out_461740943709855696[8] = 0;
   out_461740943709855696[9] = 0;
   out_461740943709855696[10] = 0;
   out_461740943709855696[11] = 0;
   out_461740943709855696[12] = 0;
   out_461740943709855696[13] = 0;
   out_461740943709855696[14] = 0;
   out_461740943709855696[15] = 0;
   out_461740943709855696[16] = 0;
   out_461740943709855696[17] = 0;
   out_461740943709855696[18] = 0;
   out_461740943709855696[19] = 0;
   out_461740943709855696[20] = 0;
   out_461740943709855696[21] = 0;
   out_461740943709855696[22] = 1;
   out_461740943709855696[23] = 0;
   out_461740943709855696[24] = 0;
   out_461740943709855696[25] = 0;
   out_461740943709855696[26] = 0;
   out_461740943709855696[27] = 0;
   out_461740943709855696[28] = 0;
   out_461740943709855696[29] = 0;
   out_461740943709855696[30] = 0;
   out_461740943709855696[31] = 0;
   out_461740943709855696[32] = 0;
   out_461740943709855696[33] = 0;
   out_461740943709855696[34] = 0;
   out_461740943709855696[35] = 0;
   out_461740943709855696[36] = 0;
   out_461740943709855696[37] = 0;
   out_461740943709855696[38] = 0;
   out_461740943709855696[39] = 0;
   out_461740943709855696[40] = 0;
   out_461740943709855696[41] = 1;
   out_461740943709855696[42] = 0;
   out_461740943709855696[43] = 0;
   out_461740943709855696[44] = 0;
   out_461740943709855696[45] = 0;
   out_461740943709855696[46] = 0;
   out_461740943709855696[47] = 0;
   out_461740943709855696[48] = 0;
   out_461740943709855696[49] = 0;
   out_461740943709855696[50] = 0;
   out_461740943709855696[51] = 0;
   out_461740943709855696[52] = 0;
   out_461740943709855696[53] = 0;
}
void h_14(double *state, double *unused, double *out_8170658983540952892) {
   out_8170658983540952892[0] = state[6];
   out_8170658983540952892[1] = state[7];
   out_8170658983540952892[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7291583489387622695) {
   out_7291583489387622695[0] = 0;
   out_7291583489387622695[1] = 0;
   out_7291583489387622695[2] = 0;
   out_7291583489387622695[3] = 0;
   out_7291583489387622695[4] = 0;
   out_7291583489387622695[5] = 0;
   out_7291583489387622695[6] = 1;
   out_7291583489387622695[7] = 0;
   out_7291583489387622695[8] = 0;
   out_7291583489387622695[9] = 0;
   out_7291583489387622695[10] = 0;
   out_7291583489387622695[11] = 0;
   out_7291583489387622695[12] = 0;
   out_7291583489387622695[13] = 0;
   out_7291583489387622695[14] = 0;
   out_7291583489387622695[15] = 0;
   out_7291583489387622695[16] = 0;
   out_7291583489387622695[17] = 0;
   out_7291583489387622695[18] = 0;
   out_7291583489387622695[19] = 0;
   out_7291583489387622695[20] = 0;
   out_7291583489387622695[21] = 0;
   out_7291583489387622695[22] = 0;
   out_7291583489387622695[23] = 0;
   out_7291583489387622695[24] = 0;
   out_7291583489387622695[25] = 1;
   out_7291583489387622695[26] = 0;
   out_7291583489387622695[27] = 0;
   out_7291583489387622695[28] = 0;
   out_7291583489387622695[29] = 0;
   out_7291583489387622695[30] = 0;
   out_7291583489387622695[31] = 0;
   out_7291583489387622695[32] = 0;
   out_7291583489387622695[33] = 0;
   out_7291583489387622695[34] = 0;
   out_7291583489387622695[35] = 0;
   out_7291583489387622695[36] = 0;
   out_7291583489387622695[37] = 0;
   out_7291583489387622695[38] = 0;
   out_7291583489387622695[39] = 0;
   out_7291583489387622695[40] = 0;
   out_7291583489387622695[41] = 0;
   out_7291583489387622695[42] = 0;
   out_7291583489387622695[43] = 0;
   out_7291583489387622695[44] = 1;
   out_7291583489387622695[45] = 0;
   out_7291583489387622695[46] = 0;
   out_7291583489387622695[47] = 0;
   out_7291583489387622695[48] = 0;
   out_7291583489387622695[49] = 0;
   out_7291583489387622695[50] = 0;
   out_7291583489387622695[51] = 0;
   out_7291583489387622695[52] = 0;
   out_7291583489387622695[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4436213307949090908) {
  err_fun(nom_x, delta_x, out_4436213307949090908);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4288605168489656967) {
  inv_err_fun(nom_x, true_x, out_4288605168489656967);
}
void pose_H_mod_fun(double *state, double *out_6602564711839087419) {
  H_mod_fun(state, out_6602564711839087419);
}
void pose_f_fun(double *state, double dt, double *out_976073864278997094) {
  f_fun(state,  dt, out_976073864278997094);
}
void pose_F_fun(double *state, double dt, double *out_3492059102621666200) {
  F_fun(state,  dt, out_3492059102621666200);
}
void pose_h_4(double *state, double *unused, double *out_4637603071142990440) {
  h_4(state, unused, out_4637603071142990440);
}
void pose_H_4(double *state, double *unused, double *out_8072372152026556625) {
  H_4(state, unused, out_8072372152026556625);
}
void pose_h_10(double *state, double *unused, double *out_6628455498773735725) {
  h_10(state, unused, out_6628455498773735725);
}
void pose_H_10(double *state, double *unused, double *out_4205718837824556590) {
  H_10(state, unused, out_4205718837824556590);
}
void pose_h_13(double *state, double *unused, double *out_3016281579283853723) {
  h_13(state, unused, out_3016281579283853723);
}
void pose_H_13(double *state, double *unused, double *out_461740943709855696) {
  H_13(state, unused, out_461740943709855696);
}
void pose_h_14(double *state, double *unused, double *out_8170658983540952892) {
  h_14(state, unused, out_8170658983540952892);
}
void pose_H_14(double *state, double *unused, double *out_7291583489387622695) {
  H_14(state, unused, out_7291583489387622695);
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
