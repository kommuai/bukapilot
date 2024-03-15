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
void live_H(double *in_vec, double *out_5611574007553285366);
void live_err_fun(double *nom_x, double *delta_x, double *out_4147073567660988626);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1544491652842928106);
void live_H_mod_fun(double *state, double *out_8263643880166488238);
void live_f_fun(double *state, double dt, double *out_7642037060686782526);
void live_F_fun(double *state, double dt, double *out_3874572245833220670);
void live_h_4(double *state, double *unused, double *out_4643783970579663950);
void live_H_4(double *state, double *unused, double *out_4430389058913291070);
void live_h_9(double *state, double *unused, double *out_766446161399831429);
void live_H_9(double *state, double *unused, double *out_6729136079531813076);
void live_h_10(double *state, double *unused, double *out_5121633474030443790);
void live_H_10(double *state, double *unused, double *out_5846836711668955195);
void live_h_12(double *state, double *unused, double *out_8647337037816411235);
void live_H_12(double *state, double *unused, double *out_8996898606764298751);
void live_h_31(double *state, double *unused, double *out_1603042384283341811);
void live_H_31(double *state, double *unused, double *out_6251335574439285042);
void live_h_32(double *state, double *unused, double *out_7556554457275842987);
void live_H_32(double *state, double *unused, double *out_3578351878613989376);
void live_h_13(double *state, double *unused, double *out_2667441994367976965);
void live_H_13(double *state, double *unused, double *out_2677123412797053228);
void live_h_14(double *state, double *unused, double *out_766446161399831429);
void live_H_14(double *state, double *unused, double *out_6729136079531813076);
void live_h_33(double *state, double *unused, double *out_179669223228452346);
void live_H_33(double *state, double *unused, double *out_3100778569800427438);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}