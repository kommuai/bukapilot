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
void live_H(double *in_vec, double *out_1743540026141181874);
void live_err_fun(double *nom_x, double *delta_x, double *out_5565743702987662541);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1255803739842959215);
void live_H_mod_fun(double *state, double *out_4975874789397051720);
void live_f_fun(double *state, double dt, double *out_2584232306761842976);
void live_F_fun(double *state, double dt, double *out_592368950663937455);
void live_h_4(double *state, double *unused, double *out_5873820215404431538);
void live_H_4(double *state, double *unused, double *out_8673921672999115200);
void live_h_9(double *state, double *unused, double *out_7764150290825091314);
void live_H_9(double *state, double *unused, double *out_2485603465445988946);
void live_h_10(double *state, double *unused, double *out_1220605068792826588);
void live_H_10(double *state, double *unused, double *out_906019114330644216);
void live_h_12(double *state, double *unused, double *out_4430268483979426153);
void live_H_12(double *state, double *unused, double *out_2292663295956382204);
void live_h_31(double *state, double *unused, double *out_4816735749702269999);
void live_H_31(double *state, double *unused, double *out_639868945297027785);
void live_h_32(double *state, double *unused, double *out_8211176837072183083);
void live_H_32(double *state, double *unused, double *out_6274995558397088571);
void live_h_13(double *state, double *unused, double *out_1076522462262386092);
void live_H_13(double *state, double *unused, double *out_7450152402225292463);
void live_h_14(double *state, double *unused, double *out_7764150290825091314);
void live_H_14(double *state, double *unused, double *out_2485603465445988946);
void live_h_33(double *state, double *unused, double *out_1792146915948154025);
void live_H_33(double *state, double *unused, double *out_3790425949935885389);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}