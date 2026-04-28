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
void live_H(double *in_vec, double *out_2671826496490926622);
void live_err_fun(double *nom_x, double *delta_x, double *out_1963905421173129326);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_473782205777764513);
void live_H_mod_fun(double *state, double *out_5461635689962740905);
void live_f_fun(double *state, double dt, double *out_8656960024927113402);
void live_F_fun(double *state, double dt, double *out_5935410227501104677);
void live_h_4(double *state, double *unused, double *out_5197000484400146326);
void live_H_4(double *state, double *unused, double *out_3844372267291876155);
void live_h_9(double *state, double *unused, double *out_6582345379862127439);
void live_H_9(double *state, double *unused, double *out_4085561913921466800);
void live_h_10(double *state, double *unused, double *out_2839726496683331715);
void live_H_10(double *state, double *unused, double *out_2030732238242130451);
void live_h_12(double *state, double *unused, double *out_3447488418585844724);
void live_H_12(double *state, double *unused, double *out_8863828675323837950);
void live_h_35(double *state, double *unused, double *out_6326956079333274723);
void live_H_35(double *state, double *unused, double *out_6837352366060699957);
void live_h_32(double *state, double *unused, double *out_6890987619898826137);
void live_H_32(double *state, double *unused, double *out_4132327853158950020);
void live_h_13(double *state, double *unused, double *out_3119052173924743958);
void live_H_13(double *state, double *unused, double *out_6054933302477802239);
void live_h_14(double *state, double *unused, double *out_6582345379862127439);
void live_H_14(double *state, double *unused, double *out_4085561913921466800);
void live_h_33(double *state, double *unused, double *out_772811441892854419);
void live_H_33(double *state, double *unused, double *out_3686795361421842353);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}