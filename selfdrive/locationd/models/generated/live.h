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
void live_H(double *in_vec, double *out_7220558093438677494);
void live_err_fun(double *nom_x, double *delta_x, double *out_9043724947419933481);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1152112582838846515);
void live_H_mod_fun(double *state, double *out_5515159812904440844);
void live_f_fun(double *state, double dt, double *out_5913141148673885470);
void live_F_fun(double *state, double dt, double *out_5824171225366303602);
void live_h_4(double *state, double *unused, double *out_7069358795977024945);
void live_H_4(double *state, double *unused, double *out_5963564968460779607);
void live_h_9(double *state, double *unused, double *out_5258850399582755988);
void live_H_9(double *state, double *unused, double *out_1323653966803667863);
void live_h_10(double *state, double *unused, double *out_5862171915760362298);
void live_H_10(double *state, double *unused, double *out_840162557926928356);
void live_h_12(double *state, double *unused, double *out_4478610574562312006);
void live_H_12(double *state, double *unused, double *out_6101920728206039013);
void live_h_31(double *state, double *unused, double *out_312814249559426213);
void live_H_31(double *state, double *unused, double *out_2596902911088172231);
void live_h_32(double *state, double *unused, double *out_5184772214683631885);
void live_H_32(double *state, double *unused, double *out_8196313323955254464);
void live_h_13(double *state, double *unused, double *out_4128099975101749082);
void live_H_13(double *state, double *unused, double *out_8061570371766460313);
void live_h_14(double *state, double *unused, double *out_5258850399582755988);
void live_H_14(double *state, double *unused, double *out_1323653966803667863);
void live_h_33(double *state, double *unused, double *out_7176268788006454179);
void live_H_33(double *state, double *unused, double *out_553654093550685373);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}