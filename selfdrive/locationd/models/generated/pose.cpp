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
void err_fun(double *nom_x, double *delta_x, double *out_4903838001852844195) {
   out_4903838001852844195[0] = delta_x[0] + nom_x[0];
   out_4903838001852844195[1] = delta_x[1] + nom_x[1];
   out_4903838001852844195[2] = delta_x[2] + nom_x[2];
   out_4903838001852844195[3] = delta_x[3] + nom_x[3];
   out_4903838001852844195[4] = delta_x[4] + nom_x[4];
   out_4903838001852844195[5] = delta_x[5] + nom_x[5];
   out_4903838001852844195[6] = delta_x[6] + nom_x[6];
   out_4903838001852844195[7] = delta_x[7] + nom_x[7];
   out_4903838001852844195[8] = delta_x[8] + nom_x[8];
   out_4903838001852844195[9] = delta_x[9] + nom_x[9];
   out_4903838001852844195[10] = delta_x[10] + nom_x[10];
   out_4903838001852844195[11] = delta_x[11] + nom_x[11];
   out_4903838001852844195[12] = delta_x[12] + nom_x[12];
   out_4903838001852844195[13] = delta_x[13] + nom_x[13];
   out_4903838001852844195[14] = delta_x[14] + nom_x[14];
   out_4903838001852844195[15] = delta_x[15] + nom_x[15];
   out_4903838001852844195[16] = delta_x[16] + nom_x[16];
   out_4903838001852844195[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7297705039785129404) {
   out_7297705039785129404[0] = -nom_x[0] + true_x[0];
   out_7297705039785129404[1] = -nom_x[1] + true_x[1];
   out_7297705039785129404[2] = -nom_x[2] + true_x[2];
   out_7297705039785129404[3] = -nom_x[3] + true_x[3];
   out_7297705039785129404[4] = -nom_x[4] + true_x[4];
   out_7297705039785129404[5] = -nom_x[5] + true_x[5];
   out_7297705039785129404[6] = -nom_x[6] + true_x[6];
   out_7297705039785129404[7] = -nom_x[7] + true_x[7];
   out_7297705039785129404[8] = -nom_x[8] + true_x[8];
   out_7297705039785129404[9] = -nom_x[9] + true_x[9];
   out_7297705039785129404[10] = -nom_x[10] + true_x[10];
   out_7297705039785129404[11] = -nom_x[11] + true_x[11];
   out_7297705039785129404[12] = -nom_x[12] + true_x[12];
   out_7297705039785129404[13] = -nom_x[13] + true_x[13];
   out_7297705039785129404[14] = -nom_x[14] + true_x[14];
   out_7297705039785129404[15] = -nom_x[15] + true_x[15];
   out_7297705039785129404[16] = -nom_x[16] + true_x[16];
   out_7297705039785129404[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4026989540286219337) {
   out_4026989540286219337[0] = 1.0;
   out_4026989540286219337[1] = 0.0;
   out_4026989540286219337[2] = 0.0;
   out_4026989540286219337[3] = 0.0;
   out_4026989540286219337[4] = 0.0;
   out_4026989540286219337[5] = 0.0;
   out_4026989540286219337[6] = 0.0;
   out_4026989540286219337[7] = 0.0;
   out_4026989540286219337[8] = 0.0;
   out_4026989540286219337[9] = 0.0;
   out_4026989540286219337[10] = 0.0;
   out_4026989540286219337[11] = 0.0;
   out_4026989540286219337[12] = 0.0;
   out_4026989540286219337[13] = 0.0;
   out_4026989540286219337[14] = 0.0;
   out_4026989540286219337[15] = 0.0;
   out_4026989540286219337[16] = 0.0;
   out_4026989540286219337[17] = 0.0;
   out_4026989540286219337[18] = 0.0;
   out_4026989540286219337[19] = 1.0;
   out_4026989540286219337[20] = 0.0;
   out_4026989540286219337[21] = 0.0;
   out_4026989540286219337[22] = 0.0;
   out_4026989540286219337[23] = 0.0;
   out_4026989540286219337[24] = 0.0;
   out_4026989540286219337[25] = 0.0;
   out_4026989540286219337[26] = 0.0;
   out_4026989540286219337[27] = 0.0;
   out_4026989540286219337[28] = 0.0;
   out_4026989540286219337[29] = 0.0;
   out_4026989540286219337[30] = 0.0;
   out_4026989540286219337[31] = 0.0;
   out_4026989540286219337[32] = 0.0;
   out_4026989540286219337[33] = 0.0;
   out_4026989540286219337[34] = 0.0;
   out_4026989540286219337[35] = 0.0;
   out_4026989540286219337[36] = 0.0;
   out_4026989540286219337[37] = 0.0;
   out_4026989540286219337[38] = 1.0;
   out_4026989540286219337[39] = 0.0;
   out_4026989540286219337[40] = 0.0;
   out_4026989540286219337[41] = 0.0;
   out_4026989540286219337[42] = 0.0;
   out_4026989540286219337[43] = 0.0;
   out_4026989540286219337[44] = 0.0;
   out_4026989540286219337[45] = 0.0;
   out_4026989540286219337[46] = 0.0;
   out_4026989540286219337[47] = 0.0;
   out_4026989540286219337[48] = 0.0;
   out_4026989540286219337[49] = 0.0;
   out_4026989540286219337[50] = 0.0;
   out_4026989540286219337[51] = 0.0;
   out_4026989540286219337[52] = 0.0;
   out_4026989540286219337[53] = 0.0;
   out_4026989540286219337[54] = 0.0;
   out_4026989540286219337[55] = 0.0;
   out_4026989540286219337[56] = 0.0;
   out_4026989540286219337[57] = 1.0;
   out_4026989540286219337[58] = 0.0;
   out_4026989540286219337[59] = 0.0;
   out_4026989540286219337[60] = 0.0;
   out_4026989540286219337[61] = 0.0;
   out_4026989540286219337[62] = 0.0;
   out_4026989540286219337[63] = 0.0;
   out_4026989540286219337[64] = 0.0;
   out_4026989540286219337[65] = 0.0;
   out_4026989540286219337[66] = 0.0;
   out_4026989540286219337[67] = 0.0;
   out_4026989540286219337[68] = 0.0;
   out_4026989540286219337[69] = 0.0;
   out_4026989540286219337[70] = 0.0;
   out_4026989540286219337[71] = 0.0;
   out_4026989540286219337[72] = 0.0;
   out_4026989540286219337[73] = 0.0;
   out_4026989540286219337[74] = 0.0;
   out_4026989540286219337[75] = 0.0;
   out_4026989540286219337[76] = 1.0;
   out_4026989540286219337[77] = 0.0;
   out_4026989540286219337[78] = 0.0;
   out_4026989540286219337[79] = 0.0;
   out_4026989540286219337[80] = 0.0;
   out_4026989540286219337[81] = 0.0;
   out_4026989540286219337[82] = 0.0;
   out_4026989540286219337[83] = 0.0;
   out_4026989540286219337[84] = 0.0;
   out_4026989540286219337[85] = 0.0;
   out_4026989540286219337[86] = 0.0;
   out_4026989540286219337[87] = 0.0;
   out_4026989540286219337[88] = 0.0;
   out_4026989540286219337[89] = 0.0;
   out_4026989540286219337[90] = 0.0;
   out_4026989540286219337[91] = 0.0;
   out_4026989540286219337[92] = 0.0;
   out_4026989540286219337[93] = 0.0;
   out_4026989540286219337[94] = 0.0;
   out_4026989540286219337[95] = 1.0;
   out_4026989540286219337[96] = 0.0;
   out_4026989540286219337[97] = 0.0;
   out_4026989540286219337[98] = 0.0;
   out_4026989540286219337[99] = 0.0;
   out_4026989540286219337[100] = 0.0;
   out_4026989540286219337[101] = 0.0;
   out_4026989540286219337[102] = 0.0;
   out_4026989540286219337[103] = 0.0;
   out_4026989540286219337[104] = 0.0;
   out_4026989540286219337[105] = 0.0;
   out_4026989540286219337[106] = 0.0;
   out_4026989540286219337[107] = 0.0;
   out_4026989540286219337[108] = 0.0;
   out_4026989540286219337[109] = 0.0;
   out_4026989540286219337[110] = 0.0;
   out_4026989540286219337[111] = 0.0;
   out_4026989540286219337[112] = 0.0;
   out_4026989540286219337[113] = 0.0;
   out_4026989540286219337[114] = 1.0;
   out_4026989540286219337[115] = 0.0;
   out_4026989540286219337[116] = 0.0;
   out_4026989540286219337[117] = 0.0;
   out_4026989540286219337[118] = 0.0;
   out_4026989540286219337[119] = 0.0;
   out_4026989540286219337[120] = 0.0;
   out_4026989540286219337[121] = 0.0;
   out_4026989540286219337[122] = 0.0;
   out_4026989540286219337[123] = 0.0;
   out_4026989540286219337[124] = 0.0;
   out_4026989540286219337[125] = 0.0;
   out_4026989540286219337[126] = 0.0;
   out_4026989540286219337[127] = 0.0;
   out_4026989540286219337[128] = 0.0;
   out_4026989540286219337[129] = 0.0;
   out_4026989540286219337[130] = 0.0;
   out_4026989540286219337[131] = 0.0;
   out_4026989540286219337[132] = 0.0;
   out_4026989540286219337[133] = 1.0;
   out_4026989540286219337[134] = 0.0;
   out_4026989540286219337[135] = 0.0;
   out_4026989540286219337[136] = 0.0;
   out_4026989540286219337[137] = 0.0;
   out_4026989540286219337[138] = 0.0;
   out_4026989540286219337[139] = 0.0;
   out_4026989540286219337[140] = 0.0;
   out_4026989540286219337[141] = 0.0;
   out_4026989540286219337[142] = 0.0;
   out_4026989540286219337[143] = 0.0;
   out_4026989540286219337[144] = 0.0;
   out_4026989540286219337[145] = 0.0;
   out_4026989540286219337[146] = 0.0;
   out_4026989540286219337[147] = 0.0;
   out_4026989540286219337[148] = 0.0;
   out_4026989540286219337[149] = 0.0;
   out_4026989540286219337[150] = 0.0;
   out_4026989540286219337[151] = 0.0;
   out_4026989540286219337[152] = 1.0;
   out_4026989540286219337[153] = 0.0;
   out_4026989540286219337[154] = 0.0;
   out_4026989540286219337[155] = 0.0;
   out_4026989540286219337[156] = 0.0;
   out_4026989540286219337[157] = 0.0;
   out_4026989540286219337[158] = 0.0;
   out_4026989540286219337[159] = 0.0;
   out_4026989540286219337[160] = 0.0;
   out_4026989540286219337[161] = 0.0;
   out_4026989540286219337[162] = 0.0;
   out_4026989540286219337[163] = 0.0;
   out_4026989540286219337[164] = 0.0;
   out_4026989540286219337[165] = 0.0;
   out_4026989540286219337[166] = 0.0;
   out_4026989540286219337[167] = 0.0;
   out_4026989540286219337[168] = 0.0;
   out_4026989540286219337[169] = 0.0;
   out_4026989540286219337[170] = 0.0;
   out_4026989540286219337[171] = 1.0;
   out_4026989540286219337[172] = 0.0;
   out_4026989540286219337[173] = 0.0;
   out_4026989540286219337[174] = 0.0;
   out_4026989540286219337[175] = 0.0;
   out_4026989540286219337[176] = 0.0;
   out_4026989540286219337[177] = 0.0;
   out_4026989540286219337[178] = 0.0;
   out_4026989540286219337[179] = 0.0;
   out_4026989540286219337[180] = 0.0;
   out_4026989540286219337[181] = 0.0;
   out_4026989540286219337[182] = 0.0;
   out_4026989540286219337[183] = 0.0;
   out_4026989540286219337[184] = 0.0;
   out_4026989540286219337[185] = 0.0;
   out_4026989540286219337[186] = 0.0;
   out_4026989540286219337[187] = 0.0;
   out_4026989540286219337[188] = 0.0;
   out_4026989540286219337[189] = 0.0;
   out_4026989540286219337[190] = 1.0;
   out_4026989540286219337[191] = 0.0;
   out_4026989540286219337[192] = 0.0;
   out_4026989540286219337[193] = 0.0;
   out_4026989540286219337[194] = 0.0;
   out_4026989540286219337[195] = 0.0;
   out_4026989540286219337[196] = 0.0;
   out_4026989540286219337[197] = 0.0;
   out_4026989540286219337[198] = 0.0;
   out_4026989540286219337[199] = 0.0;
   out_4026989540286219337[200] = 0.0;
   out_4026989540286219337[201] = 0.0;
   out_4026989540286219337[202] = 0.0;
   out_4026989540286219337[203] = 0.0;
   out_4026989540286219337[204] = 0.0;
   out_4026989540286219337[205] = 0.0;
   out_4026989540286219337[206] = 0.0;
   out_4026989540286219337[207] = 0.0;
   out_4026989540286219337[208] = 0.0;
   out_4026989540286219337[209] = 1.0;
   out_4026989540286219337[210] = 0.0;
   out_4026989540286219337[211] = 0.0;
   out_4026989540286219337[212] = 0.0;
   out_4026989540286219337[213] = 0.0;
   out_4026989540286219337[214] = 0.0;
   out_4026989540286219337[215] = 0.0;
   out_4026989540286219337[216] = 0.0;
   out_4026989540286219337[217] = 0.0;
   out_4026989540286219337[218] = 0.0;
   out_4026989540286219337[219] = 0.0;
   out_4026989540286219337[220] = 0.0;
   out_4026989540286219337[221] = 0.0;
   out_4026989540286219337[222] = 0.0;
   out_4026989540286219337[223] = 0.0;
   out_4026989540286219337[224] = 0.0;
   out_4026989540286219337[225] = 0.0;
   out_4026989540286219337[226] = 0.0;
   out_4026989540286219337[227] = 0.0;
   out_4026989540286219337[228] = 1.0;
   out_4026989540286219337[229] = 0.0;
   out_4026989540286219337[230] = 0.0;
   out_4026989540286219337[231] = 0.0;
   out_4026989540286219337[232] = 0.0;
   out_4026989540286219337[233] = 0.0;
   out_4026989540286219337[234] = 0.0;
   out_4026989540286219337[235] = 0.0;
   out_4026989540286219337[236] = 0.0;
   out_4026989540286219337[237] = 0.0;
   out_4026989540286219337[238] = 0.0;
   out_4026989540286219337[239] = 0.0;
   out_4026989540286219337[240] = 0.0;
   out_4026989540286219337[241] = 0.0;
   out_4026989540286219337[242] = 0.0;
   out_4026989540286219337[243] = 0.0;
   out_4026989540286219337[244] = 0.0;
   out_4026989540286219337[245] = 0.0;
   out_4026989540286219337[246] = 0.0;
   out_4026989540286219337[247] = 1.0;
   out_4026989540286219337[248] = 0.0;
   out_4026989540286219337[249] = 0.0;
   out_4026989540286219337[250] = 0.0;
   out_4026989540286219337[251] = 0.0;
   out_4026989540286219337[252] = 0.0;
   out_4026989540286219337[253] = 0.0;
   out_4026989540286219337[254] = 0.0;
   out_4026989540286219337[255] = 0.0;
   out_4026989540286219337[256] = 0.0;
   out_4026989540286219337[257] = 0.0;
   out_4026989540286219337[258] = 0.0;
   out_4026989540286219337[259] = 0.0;
   out_4026989540286219337[260] = 0.0;
   out_4026989540286219337[261] = 0.0;
   out_4026989540286219337[262] = 0.0;
   out_4026989540286219337[263] = 0.0;
   out_4026989540286219337[264] = 0.0;
   out_4026989540286219337[265] = 0.0;
   out_4026989540286219337[266] = 1.0;
   out_4026989540286219337[267] = 0.0;
   out_4026989540286219337[268] = 0.0;
   out_4026989540286219337[269] = 0.0;
   out_4026989540286219337[270] = 0.0;
   out_4026989540286219337[271] = 0.0;
   out_4026989540286219337[272] = 0.0;
   out_4026989540286219337[273] = 0.0;
   out_4026989540286219337[274] = 0.0;
   out_4026989540286219337[275] = 0.0;
   out_4026989540286219337[276] = 0.0;
   out_4026989540286219337[277] = 0.0;
   out_4026989540286219337[278] = 0.0;
   out_4026989540286219337[279] = 0.0;
   out_4026989540286219337[280] = 0.0;
   out_4026989540286219337[281] = 0.0;
   out_4026989540286219337[282] = 0.0;
   out_4026989540286219337[283] = 0.0;
   out_4026989540286219337[284] = 0.0;
   out_4026989540286219337[285] = 1.0;
   out_4026989540286219337[286] = 0.0;
   out_4026989540286219337[287] = 0.0;
   out_4026989540286219337[288] = 0.0;
   out_4026989540286219337[289] = 0.0;
   out_4026989540286219337[290] = 0.0;
   out_4026989540286219337[291] = 0.0;
   out_4026989540286219337[292] = 0.0;
   out_4026989540286219337[293] = 0.0;
   out_4026989540286219337[294] = 0.0;
   out_4026989540286219337[295] = 0.0;
   out_4026989540286219337[296] = 0.0;
   out_4026989540286219337[297] = 0.0;
   out_4026989540286219337[298] = 0.0;
   out_4026989540286219337[299] = 0.0;
   out_4026989540286219337[300] = 0.0;
   out_4026989540286219337[301] = 0.0;
   out_4026989540286219337[302] = 0.0;
   out_4026989540286219337[303] = 0.0;
   out_4026989540286219337[304] = 1.0;
   out_4026989540286219337[305] = 0.0;
   out_4026989540286219337[306] = 0.0;
   out_4026989540286219337[307] = 0.0;
   out_4026989540286219337[308] = 0.0;
   out_4026989540286219337[309] = 0.0;
   out_4026989540286219337[310] = 0.0;
   out_4026989540286219337[311] = 0.0;
   out_4026989540286219337[312] = 0.0;
   out_4026989540286219337[313] = 0.0;
   out_4026989540286219337[314] = 0.0;
   out_4026989540286219337[315] = 0.0;
   out_4026989540286219337[316] = 0.0;
   out_4026989540286219337[317] = 0.0;
   out_4026989540286219337[318] = 0.0;
   out_4026989540286219337[319] = 0.0;
   out_4026989540286219337[320] = 0.0;
   out_4026989540286219337[321] = 0.0;
   out_4026989540286219337[322] = 0.0;
   out_4026989540286219337[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5703061510448955959) {
   out_5703061510448955959[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5703061510448955959[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5703061510448955959[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5703061510448955959[3] = dt*state[12] + state[3];
   out_5703061510448955959[4] = dt*state[13] + state[4];
   out_5703061510448955959[5] = dt*state[14] + state[5];
   out_5703061510448955959[6] = state[6];
   out_5703061510448955959[7] = state[7];
   out_5703061510448955959[8] = state[8];
   out_5703061510448955959[9] = state[9];
   out_5703061510448955959[10] = state[10];
   out_5703061510448955959[11] = state[11];
   out_5703061510448955959[12] = state[12];
   out_5703061510448955959[13] = state[13];
   out_5703061510448955959[14] = state[14];
   out_5703061510448955959[15] = state[15];
   out_5703061510448955959[16] = state[16];
   out_5703061510448955959[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4198032333973082495) {
   out_4198032333973082495[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4198032333973082495[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4198032333973082495[2] = 0;
   out_4198032333973082495[3] = 0;
   out_4198032333973082495[4] = 0;
   out_4198032333973082495[5] = 0;
   out_4198032333973082495[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4198032333973082495[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4198032333973082495[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4198032333973082495[9] = 0;
   out_4198032333973082495[10] = 0;
   out_4198032333973082495[11] = 0;
   out_4198032333973082495[12] = 0;
   out_4198032333973082495[13] = 0;
   out_4198032333973082495[14] = 0;
   out_4198032333973082495[15] = 0;
   out_4198032333973082495[16] = 0;
   out_4198032333973082495[17] = 0;
   out_4198032333973082495[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4198032333973082495[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4198032333973082495[20] = 0;
   out_4198032333973082495[21] = 0;
   out_4198032333973082495[22] = 0;
   out_4198032333973082495[23] = 0;
   out_4198032333973082495[24] = 0;
   out_4198032333973082495[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4198032333973082495[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4198032333973082495[27] = 0;
   out_4198032333973082495[28] = 0;
   out_4198032333973082495[29] = 0;
   out_4198032333973082495[30] = 0;
   out_4198032333973082495[31] = 0;
   out_4198032333973082495[32] = 0;
   out_4198032333973082495[33] = 0;
   out_4198032333973082495[34] = 0;
   out_4198032333973082495[35] = 0;
   out_4198032333973082495[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4198032333973082495[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4198032333973082495[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4198032333973082495[39] = 0;
   out_4198032333973082495[40] = 0;
   out_4198032333973082495[41] = 0;
   out_4198032333973082495[42] = 0;
   out_4198032333973082495[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4198032333973082495[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4198032333973082495[45] = 0;
   out_4198032333973082495[46] = 0;
   out_4198032333973082495[47] = 0;
   out_4198032333973082495[48] = 0;
   out_4198032333973082495[49] = 0;
   out_4198032333973082495[50] = 0;
   out_4198032333973082495[51] = 0;
   out_4198032333973082495[52] = 0;
   out_4198032333973082495[53] = 0;
   out_4198032333973082495[54] = 0;
   out_4198032333973082495[55] = 0;
   out_4198032333973082495[56] = 0;
   out_4198032333973082495[57] = 1;
   out_4198032333973082495[58] = 0;
   out_4198032333973082495[59] = 0;
   out_4198032333973082495[60] = 0;
   out_4198032333973082495[61] = 0;
   out_4198032333973082495[62] = 0;
   out_4198032333973082495[63] = 0;
   out_4198032333973082495[64] = 0;
   out_4198032333973082495[65] = 0;
   out_4198032333973082495[66] = dt;
   out_4198032333973082495[67] = 0;
   out_4198032333973082495[68] = 0;
   out_4198032333973082495[69] = 0;
   out_4198032333973082495[70] = 0;
   out_4198032333973082495[71] = 0;
   out_4198032333973082495[72] = 0;
   out_4198032333973082495[73] = 0;
   out_4198032333973082495[74] = 0;
   out_4198032333973082495[75] = 0;
   out_4198032333973082495[76] = 1;
   out_4198032333973082495[77] = 0;
   out_4198032333973082495[78] = 0;
   out_4198032333973082495[79] = 0;
   out_4198032333973082495[80] = 0;
   out_4198032333973082495[81] = 0;
   out_4198032333973082495[82] = 0;
   out_4198032333973082495[83] = 0;
   out_4198032333973082495[84] = 0;
   out_4198032333973082495[85] = dt;
   out_4198032333973082495[86] = 0;
   out_4198032333973082495[87] = 0;
   out_4198032333973082495[88] = 0;
   out_4198032333973082495[89] = 0;
   out_4198032333973082495[90] = 0;
   out_4198032333973082495[91] = 0;
   out_4198032333973082495[92] = 0;
   out_4198032333973082495[93] = 0;
   out_4198032333973082495[94] = 0;
   out_4198032333973082495[95] = 1;
   out_4198032333973082495[96] = 0;
   out_4198032333973082495[97] = 0;
   out_4198032333973082495[98] = 0;
   out_4198032333973082495[99] = 0;
   out_4198032333973082495[100] = 0;
   out_4198032333973082495[101] = 0;
   out_4198032333973082495[102] = 0;
   out_4198032333973082495[103] = 0;
   out_4198032333973082495[104] = dt;
   out_4198032333973082495[105] = 0;
   out_4198032333973082495[106] = 0;
   out_4198032333973082495[107] = 0;
   out_4198032333973082495[108] = 0;
   out_4198032333973082495[109] = 0;
   out_4198032333973082495[110] = 0;
   out_4198032333973082495[111] = 0;
   out_4198032333973082495[112] = 0;
   out_4198032333973082495[113] = 0;
   out_4198032333973082495[114] = 1;
   out_4198032333973082495[115] = 0;
   out_4198032333973082495[116] = 0;
   out_4198032333973082495[117] = 0;
   out_4198032333973082495[118] = 0;
   out_4198032333973082495[119] = 0;
   out_4198032333973082495[120] = 0;
   out_4198032333973082495[121] = 0;
   out_4198032333973082495[122] = 0;
   out_4198032333973082495[123] = 0;
   out_4198032333973082495[124] = 0;
   out_4198032333973082495[125] = 0;
   out_4198032333973082495[126] = 0;
   out_4198032333973082495[127] = 0;
   out_4198032333973082495[128] = 0;
   out_4198032333973082495[129] = 0;
   out_4198032333973082495[130] = 0;
   out_4198032333973082495[131] = 0;
   out_4198032333973082495[132] = 0;
   out_4198032333973082495[133] = 1;
   out_4198032333973082495[134] = 0;
   out_4198032333973082495[135] = 0;
   out_4198032333973082495[136] = 0;
   out_4198032333973082495[137] = 0;
   out_4198032333973082495[138] = 0;
   out_4198032333973082495[139] = 0;
   out_4198032333973082495[140] = 0;
   out_4198032333973082495[141] = 0;
   out_4198032333973082495[142] = 0;
   out_4198032333973082495[143] = 0;
   out_4198032333973082495[144] = 0;
   out_4198032333973082495[145] = 0;
   out_4198032333973082495[146] = 0;
   out_4198032333973082495[147] = 0;
   out_4198032333973082495[148] = 0;
   out_4198032333973082495[149] = 0;
   out_4198032333973082495[150] = 0;
   out_4198032333973082495[151] = 0;
   out_4198032333973082495[152] = 1;
   out_4198032333973082495[153] = 0;
   out_4198032333973082495[154] = 0;
   out_4198032333973082495[155] = 0;
   out_4198032333973082495[156] = 0;
   out_4198032333973082495[157] = 0;
   out_4198032333973082495[158] = 0;
   out_4198032333973082495[159] = 0;
   out_4198032333973082495[160] = 0;
   out_4198032333973082495[161] = 0;
   out_4198032333973082495[162] = 0;
   out_4198032333973082495[163] = 0;
   out_4198032333973082495[164] = 0;
   out_4198032333973082495[165] = 0;
   out_4198032333973082495[166] = 0;
   out_4198032333973082495[167] = 0;
   out_4198032333973082495[168] = 0;
   out_4198032333973082495[169] = 0;
   out_4198032333973082495[170] = 0;
   out_4198032333973082495[171] = 1;
   out_4198032333973082495[172] = 0;
   out_4198032333973082495[173] = 0;
   out_4198032333973082495[174] = 0;
   out_4198032333973082495[175] = 0;
   out_4198032333973082495[176] = 0;
   out_4198032333973082495[177] = 0;
   out_4198032333973082495[178] = 0;
   out_4198032333973082495[179] = 0;
   out_4198032333973082495[180] = 0;
   out_4198032333973082495[181] = 0;
   out_4198032333973082495[182] = 0;
   out_4198032333973082495[183] = 0;
   out_4198032333973082495[184] = 0;
   out_4198032333973082495[185] = 0;
   out_4198032333973082495[186] = 0;
   out_4198032333973082495[187] = 0;
   out_4198032333973082495[188] = 0;
   out_4198032333973082495[189] = 0;
   out_4198032333973082495[190] = 1;
   out_4198032333973082495[191] = 0;
   out_4198032333973082495[192] = 0;
   out_4198032333973082495[193] = 0;
   out_4198032333973082495[194] = 0;
   out_4198032333973082495[195] = 0;
   out_4198032333973082495[196] = 0;
   out_4198032333973082495[197] = 0;
   out_4198032333973082495[198] = 0;
   out_4198032333973082495[199] = 0;
   out_4198032333973082495[200] = 0;
   out_4198032333973082495[201] = 0;
   out_4198032333973082495[202] = 0;
   out_4198032333973082495[203] = 0;
   out_4198032333973082495[204] = 0;
   out_4198032333973082495[205] = 0;
   out_4198032333973082495[206] = 0;
   out_4198032333973082495[207] = 0;
   out_4198032333973082495[208] = 0;
   out_4198032333973082495[209] = 1;
   out_4198032333973082495[210] = 0;
   out_4198032333973082495[211] = 0;
   out_4198032333973082495[212] = 0;
   out_4198032333973082495[213] = 0;
   out_4198032333973082495[214] = 0;
   out_4198032333973082495[215] = 0;
   out_4198032333973082495[216] = 0;
   out_4198032333973082495[217] = 0;
   out_4198032333973082495[218] = 0;
   out_4198032333973082495[219] = 0;
   out_4198032333973082495[220] = 0;
   out_4198032333973082495[221] = 0;
   out_4198032333973082495[222] = 0;
   out_4198032333973082495[223] = 0;
   out_4198032333973082495[224] = 0;
   out_4198032333973082495[225] = 0;
   out_4198032333973082495[226] = 0;
   out_4198032333973082495[227] = 0;
   out_4198032333973082495[228] = 1;
   out_4198032333973082495[229] = 0;
   out_4198032333973082495[230] = 0;
   out_4198032333973082495[231] = 0;
   out_4198032333973082495[232] = 0;
   out_4198032333973082495[233] = 0;
   out_4198032333973082495[234] = 0;
   out_4198032333973082495[235] = 0;
   out_4198032333973082495[236] = 0;
   out_4198032333973082495[237] = 0;
   out_4198032333973082495[238] = 0;
   out_4198032333973082495[239] = 0;
   out_4198032333973082495[240] = 0;
   out_4198032333973082495[241] = 0;
   out_4198032333973082495[242] = 0;
   out_4198032333973082495[243] = 0;
   out_4198032333973082495[244] = 0;
   out_4198032333973082495[245] = 0;
   out_4198032333973082495[246] = 0;
   out_4198032333973082495[247] = 1;
   out_4198032333973082495[248] = 0;
   out_4198032333973082495[249] = 0;
   out_4198032333973082495[250] = 0;
   out_4198032333973082495[251] = 0;
   out_4198032333973082495[252] = 0;
   out_4198032333973082495[253] = 0;
   out_4198032333973082495[254] = 0;
   out_4198032333973082495[255] = 0;
   out_4198032333973082495[256] = 0;
   out_4198032333973082495[257] = 0;
   out_4198032333973082495[258] = 0;
   out_4198032333973082495[259] = 0;
   out_4198032333973082495[260] = 0;
   out_4198032333973082495[261] = 0;
   out_4198032333973082495[262] = 0;
   out_4198032333973082495[263] = 0;
   out_4198032333973082495[264] = 0;
   out_4198032333973082495[265] = 0;
   out_4198032333973082495[266] = 1;
   out_4198032333973082495[267] = 0;
   out_4198032333973082495[268] = 0;
   out_4198032333973082495[269] = 0;
   out_4198032333973082495[270] = 0;
   out_4198032333973082495[271] = 0;
   out_4198032333973082495[272] = 0;
   out_4198032333973082495[273] = 0;
   out_4198032333973082495[274] = 0;
   out_4198032333973082495[275] = 0;
   out_4198032333973082495[276] = 0;
   out_4198032333973082495[277] = 0;
   out_4198032333973082495[278] = 0;
   out_4198032333973082495[279] = 0;
   out_4198032333973082495[280] = 0;
   out_4198032333973082495[281] = 0;
   out_4198032333973082495[282] = 0;
   out_4198032333973082495[283] = 0;
   out_4198032333973082495[284] = 0;
   out_4198032333973082495[285] = 1;
   out_4198032333973082495[286] = 0;
   out_4198032333973082495[287] = 0;
   out_4198032333973082495[288] = 0;
   out_4198032333973082495[289] = 0;
   out_4198032333973082495[290] = 0;
   out_4198032333973082495[291] = 0;
   out_4198032333973082495[292] = 0;
   out_4198032333973082495[293] = 0;
   out_4198032333973082495[294] = 0;
   out_4198032333973082495[295] = 0;
   out_4198032333973082495[296] = 0;
   out_4198032333973082495[297] = 0;
   out_4198032333973082495[298] = 0;
   out_4198032333973082495[299] = 0;
   out_4198032333973082495[300] = 0;
   out_4198032333973082495[301] = 0;
   out_4198032333973082495[302] = 0;
   out_4198032333973082495[303] = 0;
   out_4198032333973082495[304] = 1;
   out_4198032333973082495[305] = 0;
   out_4198032333973082495[306] = 0;
   out_4198032333973082495[307] = 0;
   out_4198032333973082495[308] = 0;
   out_4198032333973082495[309] = 0;
   out_4198032333973082495[310] = 0;
   out_4198032333973082495[311] = 0;
   out_4198032333973082495[312] = 0;
   out_4198032333973082495[313] = 0;
   out_4198032333973082495[314] = 0;
   out_4198032333973082495[315] = 0;
   out_4198032333973082495[316] = 0;
   out_4198032333973082495[317] = 0;
   out_4198032333973082495[318] = 0;
   out_4198032333973082495[319] = 0;
   out_4198032333973082495[320] = 0;
   out_4198032333973082495[321] = 0;
   out_4198032333973082495[322] = 0;
   out_4198032333973082495[323] = 1;
}
void h_4(double *state, double *unused, double *out_526370016706783213) {
   out_526370016706783213[0] = state[6] + state[9];
   out_526370016706783213[1] = state[7] + state[10];
   out_526370016706783213[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_1570636421269345888) {
   out_1570636421269345888[0] = 0;
   out_1570636421269345888[1] = 0;
   out_1570636421269345888[2] = 0;
   out_1570636421269345888[3] = 0;
   out_1570636421269345888[4] = 0;
   out_1570636421269345888[5] = 0;
   out_1570636421269345888[6] = 1;
   out_1570636421269345888[7] = 0;
   out_1570636421269345888[8] = 0;
   out_1570636421269345888[9] = 1;
   out_1570636421269345888[10] = 0;
   out_1570636421269345888[11] = 0;
   out_1570636421269345888[12] = 0;
   out_1570636421269345888[13] = 0;
   out_1570636421269345888[14] = 0;
   out_1570636421269345888[15] = 0;
   out_1570636421269345888[16] = 0;
   out_1570636421269345888[17] = 0;
   out_1570636421269345888[18] = 0;
   out_1570636421269345888[19] = 0;
   out_1570636421269345888[20] = 0;
   out_1570636421269345888[21] = 0;
   out_1570636421269345888[22] = 0;
   out_1570636421269345888[23] = 0;
   out_1570636421269345888[24] = 0;
   out_1570636421269345888[25] = 1;
   out_1570636421269345888[26] = 0;
   out_1570636421269345888[27] = 0;
   out_1570636421269345888[28] = 1;
   out_1570636421269345888[29] = 0;
   out_1570636421269345888[30] = 0;
   out_1570636421269345888[31] = 0;
   out_1570636421269345888[32] = 0;
   out_1570636421269345888[33] = 0;
   out_1570636421269345888[34] = 0;
   out_1570636421269345888[35] = 0;
   out_1570636421269345888[36] = 0;
   out_1570636421269345888[37] = 0;
   out_1570636421269345888[38] = 0;
   out_1570636421269345888[39] = 0;
   out_1570636421269345888[40] = 0;
   out_1570636421269345888[41] = 0;
   out_1570636421269345888[42] = 0;
   out_1570636421269345888[43] = 0;
   out_1570636421269345888[44] = 1;
   out_1570636421269345888[45] = 0;
   out_1570636421269345888[46] = 0;
   out_1570636421269345888[47] = 1;
   out_1570636421269345888[48] = 0;
   out_1570636421269345888[49] = 0;
   out_1570636421269345888[50] = 0;
   out_1570636421269345888[51] = 0;
   out_1570636421269345888[52] = 0;
   out_1570636421269345888[53] = 0;
}
void h_10(double *state, double *unused, double *out_5670777292095012690) {
   out_5670777292095012690[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5670777292095012690[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5670777292095012690[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8988757871693847640) {
   out_8988757871693847640[0] = 0;
   out_8988757871693847640[1] = 9.8100000000000005*cos(state[1]);
   out_8988757871693847640[2] = 0;
   out_8988757871693847640[3] = 0;
   out_8988757871693847640[4] = -state[8];
   out_8988757871693847640[5] = state[7];
   out_8988757871693847640[6] = 0;
   out_8988757871693847640[7] = state[5];
   out_8988757871693847640[8] = -state[4];
   out_8988757871693847640[9] = 0;
   out_8988757871693847640[10] = 0;
   out_8988757871693847640[11] = 0;
   out_8988757871693847640[12] = 1;
   out_8988757871693847640[13] = 0;
   out_8988757871693847640[14] = 0;
   out_8988757871693847640[15] = 1;
   out_8988757871693847640[16] = 0;
   out_8988757871693847640[17] = 0;
   out_8988757871693847640[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8988757871693847640[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8988757871693847640[20] = 0;
   out_8988757871693847640[21] = state[8];
   out_8988757871693847640[22] = 0;
   out_8988757871693847640[23] = -state[6];
   out_8988757871693847640[24] = -state[5];
   out_8988757871693847640[25] = 0;
   out_8988757871693847640[26] = state[3];
   out_8988757871693847640[27] = 0;
   out_8988757871693847640[28] = 0;
   out_8988757871693847640[29] = 0;
   out_8988757871693847640[30] = 0;
   out_8988757871693847640[31] = 1;
   out_8988757871693847640[32] = 0;
   out_8988757871693847640[33] = 0;
   out_8988757871693847640[34] = 1;
   out_8988757871693847640[35] = 0;
   out_8988757871693847640[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8988757871693847640[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8988757871693847640[38] = 0;
   out_8988757871693847640[39] = -state[7];
   out_8988757871693847640[40] = state[6];
   out_8988757871693847640[41] = 0;
   out_8988757871693847640[42] = state[4];
   out_8988757871693847640[43] = -state[3];
   out_8988757871693847640[44] = 0;
   out_8988757871693847640[45] = 0;
   out_8988757871693847640[46] = 0;
   out_8988757871693847640[47] = 0;
   out_8988757871693847640[48] = 0;
   out_8988757871693847640[49] = 0;
   out_8988757871693847640[50] = 1;
   out_8988757871693847640[51] = 0;
   out_8988757871693847640[52] = 0;
   out_8988757871693847640[53] = 1;
}
void h_13(double *state, double *unused, double *out_8332537746237142870) {
   out_8332537746237142870[0] = state[3];
   out_8332537746237142870[1] = state[4];
   out_8332537746237142870[2] = state[5];
}
void H_13(double *state, double *unused, double *out_1641637404062986913) {
   out_1641637404062986913[0] = 0;
   out_1641637404062986913[1] = 0;
   out_1641637404062986913[2] = 0;
   out_1641637404062986913[3] = 1;
   out_1641637404062986913[4] = 0;
   out_1641637404062986913[5] = 0;
   out_1641637404062986913[6] = 0;
   out_1641637404062986913[7] = 0;
   out_1641637404062986913[8] = 0;
   out_1641637404062986913[9] = 0;
   out_1641637404062986913[10] = 0;
   out_1641637404062986913[11] = 0;
   out_1641637404062986913[12] = 0;
   out_1641637404062986913[13] = 0;
   out_1641637404062986913[14] = 0;
   out_1641637404062986913[15] = 0;
   out_1641637404062986913[16] = 0;
   out_1641637404062986913[17] = 0;
   out_1641637404062986913[18] = 0;
   out_1641637404062986913[19] = 0;
   out_1641637404062986913[20] = 0;
   out_1641637404062986913[21] = 0;
   out_1641637404062986913[22] = 1;
   out_1641637404062986913[23] = 0;
   out_1641637404062986913[24] = 0;
   out_1641637404062986913[25] = 0;
   out_1641637404062986913[26] = 0;
   out_1641637404062986913[27] = 0;
   out_1641637404062986913[28] = 0;
   out_1641637404062986913[29] = 0;
   out_1641637404062986913[30] = 0;
   out_1641637404062986913[31] = 0;
   out_1641637404062986913[32] = 0;
   out_1641637404062986913[33] = 0;
   out_1641637404062986913[34] = 0;
   out_1641637404062986913[35] = 0;
   out_1641637404062986913[36] = 0;
   out_1641637404062986913[37] = 0;
   out_1641637404062986913[38] = 0;
   out_1641637404062986913[39] = 0;
   out_1641637404062986913[40] = 0;
   out_1641637404062986913[41] = 1;
   out_1641637404062986913[42] = 0;
   out_1641637404062986913[43] = 0;
   out_1641637404062986913[44] = 0;
   out_1641637404062986913[45] = 0;
   out_1641637404062986913[46] = 0;
   out_1641637404062986913[47] = 0;
   out_1641637404062986913[48] = 0;
   out_1641637404062986913[49] = 0;
   out_1641637404062986913[50] = 0;
   out_1641637404062986913[51] = 0;
   out_1641637404062986913[52] = 0;
   out_1641637404062986913[53] = 0;
}
void h_14(double *state, double *unused, double *out_1423557163227896364) {
   out_1423557163227896364[0] = state[6];
   out_1423557163227896364[1] = state[7];
   out_1423557163227896364[2] = state[8];
}
void H_14(double *state, double *unused, double *out_2392604435070138641) {
   out_2392604435070138641[0] = 0;
   out_2392604435070138641[1] = 0;
   out_2392604435070138641[2] = 0;
   out_2392604435070138641[3] = 0;
   out_2392604435070138641[4] = 0;
   out_2392604435070138641[5] = 0;
   out_2392604435070138641[6] = 1;
   out_2392604435070138641[7] = 0;
   out_2392604435070138641[8] = 0;
   out_2392604435070138641[9] = 0;
   out_2392604435070138641[10] = 0;
   out_2392604435070138641[11] = 0;
   out_2392604435070138641[12] = 0;
   out_2392604435070138641[13] = 0;
   out_2392604435070138641[14] = 0;
   out_2392604435070138641[15] = 0;
   out_2392604435070138641[16] = 0;
   out_2392604435070138641[17] = 0;
   out_2392604435070138641[18] = 0;
   out_2392604435070138641[19] = 0;
   out_2392604435070138641[20] = 0;
   out_2392604435070138641[21] = 0;
   out_2392604435070138641[22] = 0;
   out_2392604435070138641[23] = 0;
   out_2392604435070138641[24] = 0;
   out_2392604435070138641[25] = 1;
   out_2392604435070138641[26] = 0;
   out_2392604435070138641[27] = 0;
   out_2392604435070138641[28] = 0;
   out_2392604435070138641[29] = 0;
   out_2392604435070138641[30] = 0;
   out_2392604435070138641[31] = 0;
   out_2392604435070138641[32] = 0;
   out_2392604435070138641[33] = 0;
   out_2392604435070138641[34] = 0;
   out_2392604435070138641[35] = 0;
   out_2392604435070138641[36] = 0;
   out_2392604435070138641[37] = 0;
   out_2392604435070138641[38] = 0;
   out_2392604435070138641[39] = 0;
   out_2392604435070138641[40] = 0;
   out_2392604435070138641[41] = 0;
   out_2392604435070138641[42] = 0;
   out_2392604435070138641[43] = 0;
   out_2392604435070138641[44] = 1;
   out_2392604435070138641[45] = 0;
   out_2392604435070138641[46] = 0;
   out_2392604435070138641[47] = 0;
   out_2392604435070138641[48] = 0;
   out_2392604435070138641[49] = 0;
   out_2392604435070138641[50] = 0;
   out_2392604435070138641[51] = 0;
   out_2392604435070138641[52] = 0;
   out_2392604435070138641[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4903838001852844195) {
  err_fun(nom_x, delta_x, out_4903838001852844195);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7297705039785129404) {
  inv_err_fun(nom_x, true_x, out_7297705039785129404);
}
void pose_H_mod_fun(double *state, double *out_4026989540286219337) {
  H_mod_fun(state, out_4026989540286219337);
}
void pose_f_fun(double *state, double dt, double *out_5703061510448955959) {
  f_fun(state,  dt, out_5703061510448955959);
}
void pose_F_fun(double *state, double dt, double *out_4198032333973082495) {
  F_fun(state,  dt, out_4198032333973082495);
}
void pose_h_4(double *state, double *unused, double *out_526370016706783213) {
  h_4(state, unused, out_526370016706783213);
}
void pose_H_4(double *state, double *unused, double *out_1570636421269345888) {
  H_4(state, unused, out_1570636421269345888);
}
void pose_h_10(double *state, double *unused, double *out_5670777292095012690) {
  h_10(state, unused, out_5670777292095012690);
}
void pose_H_10(double *state, double *unused, double *out_8988757871693847640) {
  H_10(state, unused, out_8988757871693847640);
}
void pose_h_13(double *state, double *unused, double *out_8332537746237142870) {
  h_13(state, unused, out_8332537746237142870);
}
void pose_H_13(double *state, double *unused, double *out_1641637404062986913) {
  H_13(state, unused, out_1641637404062986913);
}
void pose_h_14(double *state, double *unused, double *out_1423557163227896364) {
  h_14(state, unused, out_1423557163227896364);
}
void pose_H_14(double *state, double *unused, double *out_2392604435070138641) {
  H_14(state, unused, out_2392604435070138641);
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
