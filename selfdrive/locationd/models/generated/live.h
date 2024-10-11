#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_9020660452643543827);
void live_err_fun(double *nom_x, double *delta_x, double *out_1843239551603340712);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1521282881650925734);
void live_H_mod_fun(double *state, double *out_1170439287579884367);
void live_f_fun(double *state, double dt, double *out_3390961232896241683);
void live_F_fun(double *state, double dt, double *out_8780167090476733147);
void live_h_4(double *state, double *unused, double *out_6438109753214842765);
void live_H_4(double *state, double *unused, double *out_4817982154392079832);
void live_h_9(double *state, double *unused, double *out_4283408552487300595);
void live_H_9(double *state, double *unused, double *out_2469236780872367638);
void live_h_10(double *state, double *unused, double *out_4280956928070286054);
void live_H_10(double *state, double *unused, double *out_1312013271050478428);
void live_h_12(double *state, double *unused, double *out_8651509579694973384);
void live_H_12(double *state, double *unused, double *out_7247503542274738788);
void live_h_31(double *state, double *unused, double *out_3881993293111176745);
void live_H_31(double *state, double *unused, double *out_1451320097019472456);
void live_h_32(double *state, double *unused, double *out_6400073782943741741);
void live_H_32(double *state, double *unused, double *out_1253704415882277481);
void live_h_13(double *state, double *unused, double *out_4605120974302349542);
void live_H_13(double *state, double *unused, double *out_2583655545270933408);
void live_h_14(double *state, double *unused, double *out_4283408552487300595);
void live_H_14(double *state, double *unused, double *out_2469236780872367638);
void live_h_33(double *state, double *unused, double *out_8898009552539348579);
void live_H_33(double *state, double *unused, double *out_8745266196254241973);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}