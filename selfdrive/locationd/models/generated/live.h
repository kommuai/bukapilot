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
void live_H(double *in_vec, double *out_1998661839602729755);
void live_err_fun(double *nom_x, double *delta_x, double *out_2466892906700489491);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2536180834501843492);
void live_H_mod_fun(double *state, double *out_5419307824721098114);
void live_f_fun(double *state, double dt, double *out_7273720062422829636);
void live_F_fun(double *state, double dt, double *out_3922242401729891612);
void live_h_4(double *state, double *unused, double *out_3654887270876669146);
void live_H_4(double *state, double *unused, double *out_5453178511537958233);
void live_h_9(double *state, double *unused, double *out_6320520344561205074);
void live_H_9(double *state, double *unused, double *out_5694368158167548878);
void live_h_10(double *state, double *unused, double *out_8176262669582982758);
void live_H_10(double *state, double *unused, double *out_6257087297693610709);
void live_h_12(double *state, double *unused, double *out_7916368867517339320);
void live_H_12(double *state, double *unused, double *out_7974109154139631588);
void live_h_35(double *state, double *unused, double *out_3264191204747113609);
void live_H_35(double *state, double *unused, double *out_8819840568910565609);
void live_h_32(double *state, double *unused, double *out_5230761432636576671);
void live_H_32(double *state, double *unused, double *out_5541167816702062907);
void live_h_13(double *state, double *unused, double *out_3269457563410465337);
void live_H_13(double *state, double *unused, double *out_2501771908092028826);
void live_h_14(double *state, double *unused, double *out_6320520344561205074);
void live_H_14(double *state, double *unused, double *out_5694368158167548878);
void live_h_33(double *state, double *unused, double *out_4780242198208547957);
void live_H_33(double *state, double *unused, double *out_6476346500160128403);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}