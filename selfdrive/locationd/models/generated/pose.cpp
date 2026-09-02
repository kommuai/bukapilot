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
void err_fun(double *nom_x, double *delta_x, double *out_8262091712122760042) {
   out_8262091712122760042[0] = delta_x[0] + nom_x[0];
   out_8262091712122760042[1] = delta_x[1] + nom_x[1];
   out_8262091712122760042[2] = delta_x[2] + nom_x[2];
   out_8262091712122760042[3] = delta_x[3] + nom_x[3];
   out_8262091712122760042[4] = delta_x[4] + nom_x[4];
   out_8262091712122760042[5] = delta_x[5] + nom_x[5];
   out_8262091712122760042[6] = delta_x[6] + nom_x[6];
   out_8262091712122760042[7] = delta_x[7] + nom_x[7];
   out_8262091712122760042[8] = delta_x[8] + nom_x[8];
   out_8262091712122760042[9] = delta_x[9] + nom_x[9];
   out_8262091712122760042[10] = delta_x[10] + nom_x[10];
   out_8262091712122760042[11] = delta_x[11] + nom_x[11];
   out_8262091712122760042[12] = delta_x[12] + nom_x[12];
   out_8262091712122760042[13] = delta_x[13] + nom_x[13];
   out_8262091712122760042[14] = delta_x[14] + nom_x[14];
   out_8262091712122760042[15] = delta_x[15] + nom_x[15];
   out_8262091712122760042[16] = delta_x[16] + nom_x[16];
   out_8262091712122760042[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4072723838878715707) {
   out_4072723838878715707[0] = -nom_x[0] + true_x[0];
   out_4072723838878715707[1] = -nom_x[1] + true_x[1];
   out_4072723838878715707[2] = -nom_x[2] + true_x[2];
   out_4072723838878715707[3] = -nom_x[3] + true_x[3];
   out_4072723838878715707[4] = -nom_x[4] + true_x[4];
   out_4072723838878715707[5] = -nom_x[5] + true_x[5];
   out_4072723838878715707[6] = -nom_x[6] + true_x[6];
   out_4072723838878715707[7] = -nom_x[7] + true_x[7];
   out_4072723838878715707[8] = -nom_x[8] + true_x[8];
   out_4072723838878715707[9] = -nom_x[9] + true_x[9];
   out_4072723838878715707[10] = -nom_x[10] + true_x[10];
   out_4072723838878715707[11] = -nom_x[11] + true_x[11];
   out_4072723838878715707[12] = -nom_x[12] + true_x[12];
   out_4072723838878715707[13] = -nom_x[13] + true_x[13];
   out_4072723838878715707[14] = -nom_x[14] + true_x[14];
   out_4072723838878715707[15] = -nom_x[15] + true_x[15];
   out_4072723838878715707[16] = -nom_x[16] + true_x[16];
   out_4072723838878715707[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4020441109412953693) {
   out_4020441109412953693[0] = 1.0;
   out_4020441109412953693[1] = 0.0;
   out_4020441109412953693[2] = 0.0;
   out_4020441109412953693[3] = 0.0;
   out_4020441109412953693[4] = 0.0;
   out_4020441109412953693[5] = 0.0;
   out_4020441109412953693[6] = 0.0;
   out_4020441109412953693[7] = 0.0;
   out_4020441109412953693[8] = 0.0;
   out_4020441109412953693[9] = 0.0;
   out_4020441109412953693[10] = 0.0;
   out_4020441109412953693[11] = 0.0;
   out_4020441109412953693[12] = 0.0;
   out_4020441109412953693[13] = 0.0;
   out_4020441109412953693[14] = 0.0;
   out_4020441109412953693[15] = 0.0;
   out_4020441109412953693[16] = 0.0;
   out_4020441109412953693[17] = 0.0;
   out_4020441109412953693[18] = 0.0;
   out_4020441109412953693[19] = 1.0;
   out_4020441109412953693[20] = 0.0;
   out_4020441109412953693[21] = 0.0;
   out_4020441109412953693[22] = 0.0;
   out_4020441109412953693[23] = 0.0;
   out_4020441109412953693[24] = 0.0;
   out_4020441109412953693[25] = 0.0;
   out_4020441109412953693[26] = 0.0;
   out_4020441109412953693[27] = 0.0;
   out_4020441109412953693[28] = 0.0;
   out_4020441109412953693[29] = 0.0;
   out_4020441109412953693[30] = 0.0;
   out_4020441109412953693[31] = 0.0;
   out_4020441109412953693[32] = 0.0;
   out_4020441109412953693[33] = 0.0;
   out_4020441109412953693[34] = 0.0;
   out_4020441109412953693[35] = 0.0;
   out_4020441109412953693[36] = 0.0;
   out_4020441109412953693[37] = 0.0;
   out_4020441109412953693[38] = 1.0;
   out_4020441109412953693[39] = 0.0;
   out_4020441109412953693[40] = 0.0;
   out_4020441109412953693[41] = 0.0;
   out_4020441109412953693[42] = 0.0;
   out_4020441109412953693[43] = 0.0;
   out_4020441109412953693[44] = 0.0;
   out_4020441109412953693[45] = 0.0;
   out_4020441109412953693[46] = 0.0;
   out_4020441109412953693[47] = 0.0;
   out_4020441109412953693[48] = 0.0;
   out_4020441109412953693[49] = 0.0;
   out_4020441109412953693[50] = 0.0;
   out_4020441109412953693[51] = 0.0;
   out_4020441109412953693[52] = 0.0;
   out_4020441109412953693[53] = 0.0;
   out_4020441109412953693[54] = 0.0;
   out_4020441109412953693[55] = 0.0;
   out_4020441109412953693[56] = 0.0;
   out_4020441109412953693[57] = 1.0;
   out_4020441109412953693[58] = 0.0;
   out_4020441109412953693[59] = 0.0;
   out_4020441109412953693[60] = 0.0;
   out_4020441109412953693[61] = 0.0;
   out_4020441109412953693[62] = 0.0;
   out_4020441109412953693[63] = 0.0;
   out_4020441109412953693[64] = 0.0;
   out_4020441109412953693[65] = 0.0;
   out_4020441109412953693[66] = 0.0;
   out_4020441109412953693[67] = 0.0;
   out_4020441109412953693[68] = 0.0;
   out_4020441109412953693[69] = 0.0;
   out_4020441109412953693[70] = 0.0;
   out_4020441109412953693[71] = 0.0;
   out_4020441109412953693[72] = 0.0;
   out_4020441109412953693[73] = 0.0;
   out_4020441109412953693[74] = 0.0;
   out_4020441109412953693[75] = 0.0;
   out_4020441109412953693[76] = 1.0;
   out_4020441109412953693[77] = 0.0;
   out_4020441109412953693[78] = 0.0;
   out_4020441109412953693[79] = 0.0;
   out_4020441109412953693[80] = 0.0;
   out_4020441109412953693[81] = 0.0;
   out_4020441109412953693[82] = 0.0;
   out_4020441109412953693[83] = 0.0;
   out_4020441109412953693[84] = 0.0;
   out_4020441109412953693[85] = 0.0;
   out_4020441109412953693[86] = 0.0;
   out_4020441109412953693[87] = 0.0;
   out_4020441109412953693[88] = 0.0;
   out_4020441109412953693[89] = 0.0;
   out_4020441109412953693[90] = 0.0;
   out_4020441109412953693[91] = 0.0;
   out_4020441109412953693[92] = 0.0;
   out_4020441109412953693[93] = 0.0;
   out_4020441109412953693[94] = 0.0;
   out_4020441109412953693[95] = 1.0;
   out_4020441109412953693[96] = 0.0;
   out_4020441109412953693[97] = 0.0;
   out_4020441109412953693[98] = 0.0;
   out_4020441109412953693[99] = 0.0;
   out_4020441109412953693[100] = 0.0;
   out_4020441109412953693[101] = 0.0;
   out_4020441109412953693[102] = 0.0;
   out_4020441109412953693[103] = 0.0;
   out_4020441109412953693[104] = 0.0;
   out_4020441109412953693[105] = 0.0;
   out_4020441109412953693[106] = 0.0;
   out_4020441109412953693[107] = 0.0;
   out_4020441109412953693[108] = 0.0;
   out_4020441109412953693[109] = 0.0;
   out_4020441109412953693[110] = 0.0;
   out_4020441109412953693[111] = 0.0;
   out_4020441109412953693[112] = 0.0;
   out_4020441109412953693[113] = 0.0;
   out_4020441109412953693[114] = 1.0;
   out_4020441109412953693[115] = 0.0;
   out_4020441109412953693[116] = 0.0;
   out_4020441109412953693[117] = 0.0;
   out_4020441109412953693[118] = 0.0;
   out_4020441109412953693[119] = 0.0;
   out_4020441109412953693[120] = 0.0;
   out_4020441109412953693[121] = 0.0;
   out_4020441109412953693[122] = 0.0;
   out_4020441109412953693[123] = 0.0;
   out_4020441109412953693[124] = 0.0;
   out_4020441109412953693[125] = 0.0;
   out_4020441109412953693[126] = 0.0;
   out_4020441109412953693[127] = 0.0;
   out_4020441109412953693[128] = 0.0;
   out_4020441109412953693[129] = 0.0;
   out_4020441109412953693[130] = 0.0;
   out_4020441109412953693[131] = 0.0;
   out_4020441109412953693[132] = 0.0;
   out_4020441109412953693[133] = 1.0;
   out_4020441109412953693[134] = 0.0;
   out_4020441109412953693[135] = 0.0;
   out_4020441109412953693[136] = 0.0;
   out_4020441109412953693[137] = 0.0;
   out_4020441109412953693[138] = 0.0;
   out_4020441109412953693[139] = 0.0;
   out_4020441109412953693[140] = 0.0;
   out_4020441109412953693[141] = 0.0;
   out_4020441109412953693[142] = 0.0;
   out_4020441109412953693[143] = 0.0;
   out_4020441109412953693[144] = 0.0;
   out_4020441109412953693[145] = 0.0;
   out_4020441109412953693[146] = 0.0;
   out_4020441109412953693[147] = 0.0;
   out_4020441109412953693[148] = 0.0;
   out_4020441109412953693[149] = 0.0;
   out_4020441109412953693[150] = 0.0;
   out_4020441109412953693[151] = 0.0;
   out_4020441109412953693[152] = 1.0;
   out_4020441109412953693[153] = 0.0;
   out_4020441109412953693[154] = 0.0;
   out_4020441109412953693[155] = 0.0;
   out_4020441109412953693[156] = 0.0;
   out_4020441109412953693[157] = 0.0;
   out_4020441109412953693[158] = 0.0;
   out_4020441109412953693[159] = 0.0;
   out_4020441109412953693[160] = 0.0;
   out_4020441109412953693[161] = 0.0;
   out_4020441109412953693[162] = 0.0;
   out_4020441109412953693[163] = 0.0;
   out_4020441109412953693[164] = 0.0;
   out_4020441109412953693[165] = 0.0;
   out_4020441109412953693[166] = 0.0;
   out_4020441109412953693[167] = 0.0;
   out_4020441109412953693[168] = 0.0;
   out_4020441109412953693[169] = 0.0;
   out_4020441109412953693[170] = 0.0;
   out_4020441109412953693[171] = 1.0;
   out_4020441109412953693[172] = 0.0;
   out_4020441109412953693[173] = 0.0;
   out_4020441109412953693[174] = 0.0;
   out_4020441109412953693[175] = 0.0;
   out_4020441109412953693[176] = 0.0;
   out_4020441109412953693[177] = 0.0;
   out_4020441109412953693[178] = 0.0;
   out_4020441109412953693[179] = 0.0;
   out_4020441109412953693[180] = 0.0;
   out_4020441109412953693[181] = 0.0;
   out_4020441109412953693[182] = 0.0;
   out_4020441109412953693[183] = 0.0;
   out_4020441109412953693[184] = 0.0;
   out_4020441109412953693[185] = 0.0;
   out_4020441109412953693[186] = 0.0;
   out_4020441109412953693[187] = 0.0;
   out_4020441109412953693[188] = 0.0;
   out_4020441109412953693[189] = 0.0;
   out_4020441109412953693[190] = 1.0;
   out_4020441109412953693[191] = 0.0;
   out_4020441109412953693[192] = 0.0;
   out_4020441109412953693[193] = 0.0;
   out_4020441109412953693[194] = 0.0;
   out_4020441109412953693[195] = 0.0;
   out_4020441109412953693[196] = 0.0;
   out_4020441109412953693[197] = 0.0;
   out_4020441109412953693[198] = 0.0;
   out_4020441109412953693[199] = 0.0;
   out_4020441109412953693[200] = 0.0;
   out_4020441109412953693[201] = 0.0;
   out_4020441109412953693[202] = 0.0;
   out_4020441109412953693[203] = 0.0;
   out_4020441109412953693[204] = 0.0;
   out_4020441109412953693[205] = 0.0;
   out_4020441109412953693[206] = 0.0;
   out_4020441109412953693[207] = 0.0;
   out_4020441109412953693[208] = 0.0;
   out_4020441109412953693[209] = 1.0;
   out_4020441109412953693[210] = 0.0;
   out_4020441109412953693[211] = 0.0;
   out_4020441109412953693[212] = 0.0;
   out_4020441109412953693[213] = 0.0;
   out_4020441109412953693[214] = 0.0;
   out_4020441109412953693[215] = 0.0;
   out_4020441109412953693[216] = 0.0;
   out_4020441109412953693[217] = 0.0;
   out_4020441109412953693[218] = 0.0;
   out_4020441109412953693[219] = 0.0;
   out_4020441109412953693[220] = 0.0;
   out_4020441109412953693[221] = 0.0;
   out_4020441109412953693[222] = 0.0;
   out_4020441109412953693[223] = 0.0;
   out_4020441109412953693[224] = 0.0;
   out_4020441109412953693[225] = 0.0;
   out_4020441109412953693[226] = 0.0;
   out_4020441109412953693[227] = 0.0;
   out_4020441109412953693[228] = 1.0;
   out_4020441109412953693[229] = 0.0;
   out_4020441109412953693[230] = 0.0;
   out_4020441109412953693[231] = 0.0;
   out_4020441109412953693[232] = 0.0;
   out_4020441109412953693[233] = 0.0;
   out_4020441109412953693[234] = 0.0;
   out_4020441109412953693[235] = 0.0;
   out_4020441109412953693[236] = 0.0;
   out_4020441109412953693[237] = 0.0;
   out_4020441109412953693[238] = 0.0;
   out_4020441109412953693[239] = 0.0;
   out_4020441109412953693[240] = 0.0;
   out_4020441109412953693[241] = 0.0;
   out_4020441109412953693[242] = 0.0;
   out_4020441109412953693[243] = 0.0;
   out_4020441109412953693[244] = 0.0;
   out_4020441109412953693[245] = 0.0;
   out_4020441109412953693[246] = 0.0;
   out_4020441109412953693[247] = 1.0;
   out_4020441109412953693[248] = 0.0;
   out_4020441109412953693[249] = 0.0;
   out_4020441109412953693[250] = 0.0;
   out_4020441109412953693[251] = 0.0;
   out_4020441109412953693[252] = 0.0;
   out_4020441109412953693[253] = 0.0;
   out_4020441109412953693[254] = 0.0;
   out_4020441109412953693[255] = 0.0;
   out_4020441109412953693[256] = 0.0;
   out_4020441109412953693[257] = 0.0;
   out_4020441109412953693[258] = 0.0;
   out_4020441109412953693[259] = 0.0;
   out_4020441109412953693[260] = 0.0;
   out_4020441109412953693[261] = 0.0;
   out_4020441109412953693[262] = 0.0;
   out_4020441109412953693[263] = 0.0;
   out_4020441109412953693[264] = 0.0;
   out_4020441109412953693[265] = 0.0;
   out_4020441109412953693[266] = 1.0;
   out_4020441109412953693[267] = 0.0;
   out_4020441109412953693[268] = 0.0;
   out_4020441109412953693[269] = 0.0;
   out_4020441109412953693[270] = 0.0;
   out_4020441109412953693[271] = 0.0;
   out_4020441109412953693[272] = 0.0;
   out_4020441109412953693[273] = 0.0;
   out_4020441109412953693[274] = 0.0;
   out_4020441109412953693[275] = 0.0;
   out_4020441109412953693[276] = 0.0;
   out_4020441109412953693[277] = 0.0;
   out_4020441109412953693[278] = 0.0;
   out_4020441109412953693[279] = 0.0;
   out_4020441109412953693[280] = 0.0;
   out_4020441109412953693[281] = 0.0;
   out_4020441109412953693[282] = 0.0;
   out_4020441109412953693[283] = 0.0;
   out_4020441109412953693[284] = 0.0;
   out_4020441109412953693[285] = 1.0;
   out_4020441109412953693[286] = 0.0;
   out_4020441109412953693[287] = 0.0;
   out_4020441109412953693[288] = 0.0;
   out_4020441109412953693[289] = 0.0;
   out_4020441109412953693[290] = 0.0;
   out_4020441109412953693[291] = 0.0;
   out_4020441109412953693[292] = 0.0;
   out_4020441109412953693[293] = 0.0;
   out_4020441109412953693[294] = 0.0;
   out_4020441109412953693[295] = 0.0;
   out_4020441109412953693[296] = 0.0;
   out_4020441109412953693[297] = 0.0;
   out_4020441109412953693[298] = 0.0;
   out_4020441109412953693[299] = 0.0;
   out_4020441109412953693[300] = 0.0;
   out_4020441109412953693[301] = 0.0;
   out_4020441109412953693[302] = 0.0;
   out_4020441109412953693[303] = 0.0;
   out_4020441109412953693[304] = 1.0;
   out_4020441109412953693[305] = 0.0;
   out_4020441109412953693[306] = 0.0;
   out_4020441109412953693[307] = 0.0;
   out_4020441109412953693[308] = 0.0;
   out_4020441109412953693[309] = 0.0;
   out_4020441109412953693[310] = 0.0;
   out_4020441109412953693[311] = 0.0;
   out_4020441109412953693[312] = 0.0;
   out_4020441109412953693[313] = 0.0;
   out_4020441109412953693[314] = 0.0;
   out_4020441109412953693[315] = 0.0;
   out_4020441109412953693[316] = 0.0;
   out_4020441109412953693[317] = 0.0;
   out_4020441109412953693[318] = 0.0;
   out_4020441109412953693[319] = 0.0;
   out_4020441109412953693[320] = 0.0;
   out_4020441109412953693[321] = 0.0;
   out_4020441109412953693[322] = 0.0;
   out_4020441109412953693[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_501825406615906340) {
   out_501825406615906340[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_501825406615906340[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_501825406615906340[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_501825406615906340[3] = dt*state[12] + state[3];
   out_501825406615906340[4] = dt*state[13] + state[4];
   out_501825406615906340[5] = dt*state[14] + state[5];
   out_501825406615906340[6] = state[6];
   out_501825406615906340[7] = state[7];
   out_501825406615906340[8] = state[8];
   out_501825406615906340[9] = state[9];
   out_501825406615906340[10] = state[10];
   out_501825406615906340[11] = state[11];
   out_501825406615906340[12] = state[12];
   out_501825406615906340[13] = state[13];
   out_501825406615906340[14] = state[14];
   out_501825406615906340[15] = state[15];
   out_501825406615906340[16] = state[16];
   out_501825406615906340[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6707474879487101266) {
   out_6707474879487101266[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6707474879487101266[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6707474879487101266[2] = 0;
   out_6707474879487101266[3] = 0;
   out_6707474879487101266[4] = 0;
   out_6707474879487101266[5] = 0;
   out_6707474879487101266[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6707474879487101266[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6707474879487101266[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6707474879487101266[9] = 0;
   out_6707474879487101266[10] = 0;
   out_6707474879487101266[11] = 0;
   out_6707474879487101266[12] = 0;
   out_6707474879487101266[13] = 0;
   out_6707474879487101266[14] = 0;
   out_6707474879487101266[15] = 0;
   out_6707474879487101266[16] = 0;
   out_6707474879487101266[17] = 0;
   out_6707474879487101266[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6707474879487101266[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6707474879487101266[20] = 0;
   out_6707474879487101266[21] = 0;
   out_6707474879487101266[22] = 0;
   out_6707474879487101266[23] = 0;
   out_6707474879487101266[24] = 0;
   out_6707474879487101266[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6707474879487101266[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6707474879487101266[27] = 0;
   out_6707474879487101266[28] = 0;
   out_6707474879487101266[29] = 0;
   out_6707474879487101266[30] = 0;
   out_6707474879487101266[31] = 0;
   out_6707474879487101266[32] = 0;
   out_6707474879487101266[33] = 0;
   out_6707474879487101266[34] = 0;
   out_6707474879487101266[35] = 0;
   out_6707474879487101266[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6707474879487101266[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6707474879487101266[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6707474879487101266[39] = 0;
   out_6707474879487101266[40] = 0;
   out_6707474879487101266[41] = 0;
   out_6707474879487101266[42] = 0;
   out_6707474879487101266[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6707474879487101266[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6707474879487101266[45] = 0;
   out_6707474879487101266[46] = 0;
   out_6707474879487101266[47] = 0;
   out_6707474879487101266[48] = 0;
   out_6707474879487101266[49] = 0;
   out_6707474879487101266[50] = 0;
   out_6707474879487101266[51] = 0;
   out_6707474879487101266[52] = 0;
   out_6707474879487101266[53] = 0;
   out_6707474879487101266[54] = 0;
   out_6707474879487101266[55] = 0;
   out_6707474879487101266[56] = 0;
   out_6707474879487101266[57] = 1;
   out_6707474879487101266[58] = 0;
   out_6707474879487101266[59] = 0;
   out_6707474879487101266[60] = 0;
   out_6707474879487101266[61] = 0;
   out_6707474879487101266[62] = 0;
   out_6707474879487101266[63] = 0;
   out_6707474879487101266[64] = 0;
   out_6707474879487101266[65] = 0;
   out_6707474879487101266[66] = dt;
   out_6707474879487101266[67] = 0;
   out_6707474879487101266[68] = 0;
   out_6707474879487101266[69] = 0;
   out_6707474879487101266[70] = 0;
   out_6707474879487101266[71] = 0;
   out_6707474879487101266[72] = 0;
   out_6707474879487101266[73] = 0;
   out_6707474879487101266[74] = 0;
   out_6707474879487101266[75] = 0;
   out_6707474879487101266[76] = 1;
   out_6707474879487101266[77] = 0;
   out_6707474879487101266[78] = 0;
   out_6707474879487101266[79] = 0;
   out_6707474879487101266[80] = 0;
   out_6707474879487101266[81] = 0;
   out_6707474879487101266[82] = 0;
   out_6707474879487101266[83] = 0;
   out_6707474879487101266[84] = 0;
   out_6707474879487101266[85] = dt;
   out_6707474879487101266[86] = 0;
   out_6707474879487101266[87] = 0;
   out_6707474879487101266[88] = 0;
   out_6707474879487101266[89] = 0;
   out_6707474879487101266[90] = 0;
   out_6707474879487101266[91] = 0;
   out_6707474879487101266[92] = 0;
   out_6707474879487101266[93] = 0;
   out_6707474879487101266[94] = 0;
   out_6707474879487101266[95] = 1;
   out_6707474879487101266[96] = 0;
   out_6707474879487101266[97] = 0;
   out_6707474879487101266[98] = 0;
   out_6707474879487101266[99] = 0;
   out_6707474879487101266[100] = 0;
   out_6707474879487101266[101] = 0;
   out_6707474879487101266[102] = 0;
   out_6707474879487101266[103] = 0;
   out_6707474879487101266[104] = dt;
   out_6707474879487101266[105] = 0;
   out_6707474879487101266[106] = 0;
   out_6707474879487101266[107] = 0;
   out_6707474879487101266[108] = 0;
   out_6707474879487101266[109] = 0;
   out_6707474879487101266[110] = 0;
   out_6707474879487101266[111] = 0;
   out_6707474879487101266[112] = 0;
   out_6707474879487101266[113] = 0;
   out_6707474879487101266[114] = 1;
   out_6707474879487101266[115] = 0;
   out_6707474879487101266[116] = 0;
   out_6707474879487101266[117] = 0;
   out_6707474879487101266[118] = 0;
   out_6707474879487101266[119] = 0;
   out_6707474879487101266[120] = 0;
   out_6707474879487101266[121] = 0;
   out_6707474879487101266[122] = 0;
   out_6707474879487101266[123] = 0;
   out_6707474879487101266[124] = 0;
   out_6707474879487101266[125] = 0;
   out_6707474879487101266[126] = 0;
   out_6707474879487101266[127] = 0;
   out_6707474879487101266[128] = 0;
   out_6707474879487101266[129] = 0;
   out_6707474879487101266[130] = 0;
   out_6707474879487101266[131] = 0;
   out_6707474879487101266[132] = 0;
   out_6707474879487101266[133] = 1;
   out_6707474879487101266[134] = 0;
   out_6707474879487101266[135] = 0;
   out_6707474879487101266[136] = 0;
   out_6707474879487101266[137] = 0;
   out_6707474879487101266[138] = 0;
   out_6707474879487101266[139] = 0;
   out_6707474879487101266[140] = 0;
   out_6707474879487101266[141] = 0;
   out_6707474879487101266[142] = 0;
   out_6707474879487101266[143] = 0;
   out_6707474879487101266[144] = 0;
   out_6707474879487101266[145] = 0;
   out_6707474879487101266[146] = 0;
   out_6707474879487101266[147] = 0;
   out_6707474879487101266[148] = 0;
   out_6707474879487101266[149] = 0;
   out_6707474879487101266[150] = 0;
   out_6707474879487101266[151] = 0;
   out_6707474879487101266[152] = 1;
   out_6707474879487101266[153] = 0;
   out_6707474879487101266[154] = 0;
   out_6707474879487101266[155] = 0;
   out_6707474879487101266[156] = 0;
   out_6707474879487101266[157] = 0;
   out_6707474879487101266[158] = 0;
   out_6707474879487101266[159] = 0;
   out_6707474879487101266[160] = 0;
   out_6707474879487101266[161] = 0;
   out_6707474879487101266[162] = 0;
   out_6707474879487101266[163] = 0;
   out_6707474879487101266[164] = 0;
   out_6707474879487101266[165] = 0;
   out_6707474879487101266[166] = 0;
   out_6707474879487101266[167] = 0;
   out_6707474879487101266[168] = 0;
   out_6707474879487101266[169] = 0;
   out_6707474879487101266[170] = 0;
   out_6707474879487101266[171] = 1;
   out_6707474879487101266[172] = 0;
   out_6707474879487101266[173] = 0;
   out_6707474879487101266[174] = 0;
   out_6707474879487101266[175] = 0;
   out_6707474879487101266[176] = 0;
   out_6707474879487101266[177] = 0;
   out_6707474879487101266[178] = 0;
   out_6707474879487101266[179] = 0;
   out_6707474879487101266[180] = 0;
   out_6707474879487101266[181] = 0;
   out_6707474879487101266[182] = 0;
   out_6707474879487101266[183] = 0;
   out_6707474879487101266[184] = 0;
   out_6707474879487101266[185] = 0;
   out_6707474879487101266[186] = 0;
   out_6707474879487101266[187] = 0;
   out_6707474879487101266[188] = 0;
   out_6707474879487101266[189] = 0;
   out_6707474879487101266[190] = 1;
   out_6707474879487101266[191] = 0;
   out_6707474879487101266[192] = 0;
   out_6707474879487101266[193] = 0;
   out_6707474879487101266[194] = 0;
   out_6707474879487101266[195] = 0;
   out_6707474879487101266[196] = 0;
   out_6707474879487101266[197] = 0;
   out_6707474879487101266[198] = 0;
   out_6707474879487101266[199] = 0;
   out_6707474879487101266[200] = 0;
   out_6707474879487101266[201] = 0;
   out_6707474879487101266[202] = 0;
   out_6707474879487101266[203] = 0;
   out_6707474879487101266[204] = 0;
   out_6707474879487101266[205] = 0;
   out_6707474879487101266[206] = 0;
   out_6707474879487101266[207] = 0;
   out_6707474879487101266[208] = 0;
   out_6707474879487101266[209] = 1;
   out_6707474879487101266[210] = 0;
   out_6707474879487101266[211] = 0;
   out_6707474879487101266[212] = 0;
   out_6707474879487101266[213] = 0;
   out_6707474879487101266[214] = 0;
   out_6707474879487101266[215] = 0;
   out_6707474879487101266[216] = 0;
   out_6707474879487101266[217] = 0;
   out_6707474879487101266[218] = 0;
   out_6707474879487101266[219] = 0;
   out_6707474879487101266[220] = 0;
   out_6707474879487101266[221] = 0;
   out_6707474879487101266[222] = 0;
   out_6707474879487101266[223] = 0;
   out_6707474879487101266[224] = 0;
   out_6707474879487101266[225] = 0;
   out_6707474879487101266[226] = 0;
   out_6707474879487101266[227] = 0;
   out_6707474879487101266[228] = 1;
   out_6707474879487101266[229] = 0;
   out_6707474879487101266[230] = 0;
   out_6707474879487101266[231] = 0;
   out_6707474879487101266[232] = 0;
   out_6707474879487101266[233] = 0;
   out_6707474879487101266[234] = 0;
   out_6707474879487101266[235] = 0;
   out_6707474879487101266[236] = 0;
   out_6707474879487101266[237] = 0;
   out_6707474879487101266[238] = 0;
   out_6707474879487101266[239] = 0;
   out_6707474879487101266[240] = 0;
   out_6707474879487101266[241] = 0;
   out_6707474879487101266[242] = 0;
   out_6707474879487101266[243] = 0;
   out_6707474879487101266[244] = 0;
   out_6707474879487101266[245] = 0;
   out_6707474879487101266[246] = 0;
   out_6707474879487101266[247] = 1;
   out_6707474879487101266[248] = 0;
   out_6707474879487101266[249] = 0;
   out_6707474879487101266[250] = 0;
   out_6707474879487101266[251] = 0;
   out_6707474879487101266[252] = 0;
   out_6707474879487101266[253] = 0;
   out_6707474879487101266[254] = 0;
   out_6707474879487101266[255] = 0;
   out_6707474879487101266[256] = 0;
   out_6707474879487101266[257] = 0;
   out_6707474879487101266[258] = 0;
   out_6707474879487101266[259] = 0;
   out_6707474879487101266[260] = 0;
   out_6707474879487101266[261] = 0;
   out_6707474879487101266[262] = 0;
   out_6707474879487101266[263] = 0;
   out_6707474879487101266[264] = 0;
   out_6707474879487101266[265] = 0;
   out_6707474879487101266[266] = 1;
   out_6707474879487101266[267] = 0;
   out_6707474879487101266[268] = 0;
   out_6707474879487101266[269] = 0;
   out_6707474879487101266[270] = 0;
   out_6707474879487101266[271] = 0;
   out_6707474879487101266[272] = 0;
   out_6707474879487101266[273] = 0;
   out_6707474879487101266[274] = 0;
   out_6707474879487101266[275] = 0;
   out_6707474879487101266[276] = 0;
   out_6707474879487101266[277] = 0;
   out_6707474879487101266[278] = 0;
   out_6707474879487101266[279] = 0;
   out_6707474879487101266[280] = 0;
   out_6707474879487101266[281] = 0;
   out_6707474879487101266[282] = 0;
   out_6707474879487101266[283] = 0;
   out_6707474879487101266[284] = 0;
   out_6707474879487101266[285] = 1;
   out_6707474879487101266[286] = 0;
   out_6707474879487101266[287] = 0;
   out_6707474879487101266[288] = 0;
   out_6707474879487101266[289] = 0;
   out_6707474879487101266[290] = 0;
   out_6707474879487101266[291] = 0;
   out_6707474879487101266[292] = 0;
   out_6707474879487101266[293] = 0;
   out_6707474879487101266[294] = 0;
   out_6707474879487101266[295] = 0;
   out_6707474879487101266[296] = 0;
   out_6707474879487101266[297] = 0;
   out_6707474879487101266[298] = 0;
   out_6707474879487101266[299] = 0;
   out_6707474879487101266[300] = 0;
   out_6707474879487101266[301] = 0;
   out_6707474879487101266[302] = 0;
   out_6707474879487101266[303] = 0;
   out_6707474879487101266[304] = 1;
   out_6707474879487101266[305] = 0;
   out_6707474879487101266[306] = 0;
   out_6707474879487101266[307] = 0;
   out_6707474879487101266[308] = 0;
   out_6707474879487101266[309] = 0;
   out_6707474879487101266[310] = 0;
   out_6707474879487101266[311] = 0;
   out_6707474879487101266[312] = 0;
   out_6707474879487101266[313] = 0;
   out_6707474879487101266[314] = 0;
   out_6707474879487101266[315] = 0;
   out_6707474879487101266[316] = 0;
   out_6707474879487101266[317] = 0;
   out_6707474879487101266[318] = 0;
   out_6707474879487101266[319] = 0;
   out_6707474879487101266[320] = 0;
   out_6707474879487101266[321] = 0;
   out_6707474879487101266[322] = 0;
   out_6707474879487101266[323] = 1;
}
void h_4(double *state, double *unused, double *out_3489320313047190272) {
   out_3489320313047190272[0] = state[6] + state[9];
   out_3489320313047190272[1] = state[7] + state[10];
   out_3489320313047190272[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_317256653424909271) {
   out_317256653424909271[0] = 0;
   out_317256653424909271[1] = 0;
   out_317256653424909271[2] = 0;
   out_317256653424909271[3] = 0;
   out_317256653424909271[4] = 0;
   out_317256653424909271[5] = 0;
   out_317256653424909271[6] = 1;
   out_317256653424909271[7] = 0;
   out_317256653424909271[8] = 0;
   out_317256653424909271[9] = 1;
   out_317256653424909271[10] = 0;
   out_317256653424909271[11] = 0;
   out_317256653424909271[12] = 0;
   out_317256653424909271[13] = 0;
   out_317256653424909271[14] = 0;
   out_317256653424909271[15] = 0;
   out_317256653424909271[16] = 0;
   out_317256653424909271[17] = 0;
   out_317256653424909271[18] = 0;
   out_317256653424909271[19] = 0;
   out_317256653424909271[20] = 0;
   out_317256653424909271[21] = 0;
   out_317256653424909271[22] = 0;
   out_317256653424909271[23] = 0;
   out_317256653424909271[24] = 0;
   out_317256653424909271[25] = 1;
   out_317256653424909271[26] = 0;
   out_317256653424909271[27] = 0;
   out_317256653424909271[28] = 1;
   out_317256653424909271[29] = 0;
   out_317256653424909271[30] = 0;
   out_317256653424909271[31] = 0;
   out_317256653424909271[32] = 0;
   out_317256653424909271[33] = 0;
   out_317256653424909271[34] = 0;
   out_317256653424909271[35] = 0;
   out_317256653424909271[36] = 0;
   out_317256653424909271[37] = 0;
   out_317256653424909271[38] = 0;
   out_317256653424909271[39] = 0;
   out_317256653424909271[40] = 0;
   out_317256653424909271[41] = 0;
   out_317256653424909271[42] = 0;
   out_317256653424909271[43] = 0;
   out_317256653424909271[44] = 1;
   out_317256653424909271[45] = 0;
   out_317256653424909271[46] = 0;
   out_317256653424909271[47] = 1;
   out_317256653424909271[48] = 0;
   out_317256653424909271[49] = 0;
   out_317256653424909271[50] = 0;
   out_317256653424909271[51] = 0;
   out_317256653424909271[52] = 0;
   out_317256653424909271[53] = 0;
}
void h_10(double *state, double *unused, double *out_1107332844316926936) {
   out_1107332844316926936[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1107332844316926936[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1107332844316926936[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5630855996947644706) {
   out_5630855996947644706[0] = 0;
   out_5630855996947644706[1] = 9.8100000000000005*cos(state[1]);
   out_5630855996947644706[2] = 0;
   out_5630855996947644706[3] = 0;
   out_5630855996947644706[4] = -state[8];
   out_5630855996947644706[5] = state[7];
   out_5630855996947644706[6] = 0;
   out_5630855996947644706[7] = state[5];
   out_5630855996947644706[8] = -state[4];
   out_5630855996947644706[9] = 0;
   out_5630855996947644706[10] = 0;
   out_5630855996947644706[11] = 0;
   out_5630855996947644706[12] = 1;
   out_5630855996947644706[13] = 0;
   out_5630855996947644706[14] = 0;
   out_5630855996947644706[15] = 1;
   out_5630855996947644706[16] = 0;
   out_5630855996947644706[17] = 0;
   out_5630855996947644706[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5630855996947644706[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5630855996947644706[20] = 0;
   out_5630855996947644706[21] = state[8];
   out_5630855996947644706[22] = 0;
   out_5630855996947644706[23] = -state[6];
   out_5630855996947644706[24] = -state[5];
   out_5630855996947644706[25] = 0;
   out_5630855996947644706[26] = state[3];
   out_5630855996947644706[27] = 0;
   out_5630855996947644706[28] = 0;
   out_5630855996947644706[29] = 0;
   out_5630855996947644706[30] = 0;
   out_5630855996947644706[31] = 1;
   out_5630855996947644706[32] = 0;
   out_5630855996947644706[33] = 0;
   out_5630855996947644706[34] = 1;
   out_5630855996947644706[35] = 0;
   out_5630855996947644706[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5630855996947644706[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5630855996947644706[38] = 0;
   out_5630855996947644706[39] = -state[7];
   out_5630855996947644706[40] = state[6];
   out_5630855996947644706[41] = 0;
   out_5630855996947644706[42] = state[4];
   out_5630855996947644706[43] = -state[3];
   out_5630855996947644706[44] = 0;
   out_5630855996947644706[45] = 0;
   out_5630855996947644706[46] = 0;
   out_5630855996947644706[47] = 0;
   out_5630855996947644706[48] = 0;
   out_5630855996947644706[49] = 0;
   out_5630855996947644706[50] = 1;
   out_5630855996947644706[51] = 0;
   out_5630855996947644706[52] = 0;
   out_5630855996947644706[53] = 1;
}
void h_13(double *state, double *unused, double *out_8211807696530044364) {
   out_8211807696530044364[0] = state[3];
   out_8211807696530044364[1] = state[4];
   out_8211807696530044364[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3529530478757242072) {
   out_3529530478757242072[0] = 0;
   out_3529530478757242072[1] = 0;
   out_3529530478757242072[2] = 0;
   out_3529530478757242072[3] = 1;
   out_3529530478757242072[4] = 0;
   out_3529530478757242072[5] = 0;
   out_3529530478757242072[6] = 0;
   out_3529530478757242072[7] = 0;
   out_3529530478757242072[8] = 0;
   out_3529530478757242072[9] = 0;
   out_3529530478757242072[10] = 0;
   out_3529530478757242072[11] = 0;
   out_3529530478757242072[12] = 0;
   out_3529530478757242072[13] = 0;
   out_3529530478757242072[14] = 0;
   out_3529530478757242072[15] = 0;
   out_3529530478757242072[16] = 0;
   out_3529530478757242072[17] = 0;
   out_3529530478757242072[18] = 0;
   out_3529530478757242072[19] = 0;
   out_3529530478757242072[20] = 0;
   out_3529530478757242072[21] = 0;
   out_3529530478757242072[22] = 1;
   out_3529530478757242072[23] = 0;
   out_3529530478757242072[24] = 0;
   out_3529530478757242072[25] = 0;
   out_3529530478757242072[26] = 0;
   out_3529530478757242072[27] = 0;
   out_3529530478757242072[28] = 0;
   out_3529530478757242072[29] = 0;
   out_3529530478757242072[30] = 0;
   out_3529530478757242072[31] = 0;
   out_3529530478757242072[32] = 0;
   out_3529530478757242072[33] = 0;
   out_3529530478757242072[34] = 0;
   out_3529530478757242072[35] = 0;
   out_3529530478757242072[36] = 0;
   out_3529530478757242072[37] = 0;
   out_3529530478757242072[38] = 0;
   out_3529530478757242072[39] = 0;
   out_3529530478757242072[40] = 0;
   out_3529530478757242072[41] = 1;
   out_3529530478757242072[42] = 0;
   out_3529530478757242072[43] = 0;
   out_3529530478757242072[44] = 0;
   out_3529530478757242072[45] = 0;
   out_3529530478757242072[46] = 0;
   out_3529530478757242072[47] = 0;
   out_3529530478757242072[48] = 0;
   out_3529530478757242072[49] = 0;
   out_3529530478757242072[50] = 0;
   out_3529530478757242072[51] = 0;
   out_3529530478757242072[52] = 0;
   out_3529530478757242072[53] = 0;
}
void h_14(double *state, double *unused, double *out_4953897270879559792) {
   out_4953897270879559792[0] = state[6];
   out_4953897270879559792[1] = state[7];
   out_4953897270879559792[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4280497509764393800) {
   out_4280497509764393800[0] = 0;
   out_4280497509764393800[1] = 0;
   out_4280497509764393800[2] = 0;
   out_4280497509764393800[3] = 0;
   out_4280497509764393800[4] = 0;
   out_4280497509764393800[5] = 0;
   out_4280497509764393800[6] = 1;
   out_4280497509764393800[7] = 0;
   out_4280497509764393800[8] = 0;
   out_4280497509764393800[9] = 0;
   out_4280497509764393800[10] = 0;
   out_4280497509764393800[11] = 0;
   out_4280497509764393800[12] = 0;
   out_4280497509764393800[13] = 0;
   out_4280497509764393800[14] = 0;
   out_4280497509764393800[15] = 0;
   out_4280497509764393800[16] = 0;
   out_4280497509764393800[17] = 0;
   out_4280497509764393800[18] = 0;
   out_4280497509764393800[19] = 0;
   out_4280497509764393800[20] = 0;
   out_4280497509764393800[21] = 0;
   out_4280497509764393800[22] = 0;
   out_4280497509764393800[23] = 0;
   out_4280497509764393800[24] = 0;
   out_4280497509764393800[25] = 1;
   out_4280497509764393800[26] = 0;
   out_4280497509764393800[27] = 0;
   out_4280497509764393800[28] = 0;
   out_4280497509764393800[29] = 0;
   out_4280497509764393800[30] = 0;
   out_4280497509764393800[31] = 0;
   out_4280497509764393800[32] = 0;
   out_4280497509764393800[33] = 0;
   out_4280497509764393800[34] = 0;
   out_4280497509764393800[35] = 0;
   out_4280497509764393800[36] = 0;
   out_4280497509764393800[37] = 0;
   out_4280497509764393800[38] = 0;
   out_4280497509764393800[39] = 0;
   out_4280497509764393800[40] = 0;
   out_4280497509764393800[41] = 0;
   out_4280497509764393800[42] = 0;
   out_4280497509764393800[43] = 0;
   out_4280497509764393800[44] = 1;
   out_4280497509764393800[45] = 0;
   out_4280497509764393800[46] = 0;
   out_4280497509764393800[47] = 0;
   out_4280497509764393800[48] = 0;
   out_4280497509764393800[49] = 0;
   out_4280497509764393800[50] = 0;
   out_4280497509764393800[51] = 0;
   out_4280497509764393800[52] = 0;
   out_4280497509764393800[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8262091712122760042) {
  err_fun(nom_x, delta_x, out_8262091712122760042);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4072723838878715707) {
  inv_err_fun(nom_x, true_x, out_4072723838878715707);
}
void pose_H_mod_fun(double *state, double *out_4020441109412953693) {
  H_mod_fun(state, out_4020441109412953693);
}
void pose_f_fun(double *state, double dt, double *out_501825406615906340) {
  f_fun(state,  dt, out_501825406615906340);
}
void pose_F_fun(double *state, double dt, double *out_6707474879487101266) {
  F_fun(state,  dt, out_6707474879487101266);
}
void pose_h_4(double *state, double *unused, double *out_3489320313047190272) {
  h_4(state, unused, out_3489320313047190272);
}
void pose_H_4(double *state, double *unused, double *out_317256653424909271) {
  H_4(state, unused, out_317256653424909271);
}
void pose_h_10(double *state, double *unused, double *out_1107332844316926936) {
  h_10(state, unused, out_1107332844316926936);
}
void pose_H_10(double *state, double *unused, double *out_5630855996947644706) {
  H_10(state, unused, out_5630855996947644706);
}
void pose_h_13(double *state, double *unused, double *out_8211807696530044364) {
  h_13(state, unused, out_8211807696530044364);
}
void pose_H_13(double *state, double *unused, double *out_3529530478757242072) {
  H_13(state, unused, out_3529530478757242072);
}
void pose_h_14(double *state, double *unused, double *out_4953897270879559792) {
  h_14(state, unused, out_4953897270879559792);
}
void pose_H_14(double *state, double *unused, double *out_4280497509764393800) {
  H_14(state, unused, out_4280497509764393800);
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
