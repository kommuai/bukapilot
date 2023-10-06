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
void live_H(double *in_vec, double *out_8258159684906796376);
void live_err_fun(double *nom_x, double *delta_x, double *out_4171775211613654441);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6976058462501210933);
void live_H_mod_fun(double *state, double *out_6624433466277367999);
void live_f_fun(double *state, double dt, double *out_172455382222605160);
void live_F_fun(double *state, double dt, double *out_7224995257418520784);
void live_h_4(double *state, double *unused, double *out_7069147676246277548);
void live_H_4(double *state, double *unused, double *out_1364884941170878753);
void live_h_9(double *state, double *unused, double *out_1513721213030672913);
void live_H_9(double *state, double *unused, double *out_1123695294541288108);
void live_h_10(double *state, double *unused, double *out_3794490238091193516);
void live_H_10(double *state, double *unused, double *out_5043536150976328507);
void live_h_12(double *state, double *unused, double *out_4863556815412175426);
void live_H_12(double *state, double *unused, double *out_3654571466861083042);
void live_h_31(double *state, double *unused, double *out_6580061750576346924);
void live_H_31(double *state, double *unused, double *out_2001777116201728623);
void live_h_32(double *state, double *unused, double *out_6200866325421669931);
void live_H_32(double *state, double *unused, double *out_8498765234536296136);
void live_h_13(double *state, double *unused, double *out_2488188502180867364);
void live_H_13(double *state, double *unused, double *out_5161781832465829508);
void live_h_14(double *state, double *unused, double *out_1513721213030672913);
void live_H_14(double *state, double *unused, double *out_1123695294541288108);
void live_h_33(double *state, double *unused, double *out_5580216781490366221);
void live_H_33(double *state, double *unused, double *out_5152334120840586227);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}