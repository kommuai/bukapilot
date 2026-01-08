#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6948804884735936091);
void live_err_fun(double *nom_x, double *delta_x, double *out_6795074912963459434);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1087170830190907230);
void live_H_mod_fun(double *state, double *out_5695966478716623779);
void live_f_fun(double *state, double dt, double *out_1471907326055577653);
void live_F_fun(double *state, double dt, double *out_7784473733798291725);
void live_h_4(double *state, double *unused, double *out_4399432867778218096);
void live_H_4(double *state, double *unused, double *out_2693610421440170898);
void live_h_9(double *state, double *unused, double *out_8204512919463660659);
void live_H_9(double *state, double *unused, double *out_8465914717004933248);
void live_h_10(double *state, double *unused, double *out_1536689961865218856);
void live_H_10(double *state, double *unused, double *out_7720787407990492377);
void live_h_12(double *state, double *unused, double *out_7423468815343005607);
void live_H_12(double *state, double *unused, double *out_7713066829472132693);
void live_h_35(double *state, double *unused, double *out_5437269061742126147);
void live_H_35(double *state, double *unused, double *out_6060272478812778274);
void live_h_32(double *state, double *unused, double *out_3521627853924214293);
void live_H_32(double *state, double *unused, double *out_4415432969101878672);
void live_h_13(double *state, double *unused, double *out_8329870940311003117);
void live_H_13(double *state, double *unused, double *out_1913648843882525803);
void live_h_14(double *state, double *unused, double *out_8204512919463660659);
void live_H_14(double *state, double *unused, double *out_8465914717004933248);
void live_h_33(double *state, double *unused, double *out_3205039837522656566);
void live_H_33(double *state, double *unused, double *out_9210829483451635878);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}