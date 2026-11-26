#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4430958223127505081);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4290768674008276673);
void pose_H_mod_fun(double *state, double *out_3054748865441164755);
void pose_f_fun(double *state, double dt, double *out_717525079663500723);
void pose_F_fun(double *state, double dt, double *out_8758534082296255942);
void pose_h_4(double *state, double *unused, double *out_4992833986378518702);
void pose_H_4(double *state, double *unused, double *out_6397593698087977158);
void pose_h_10(double *state, double *unused, double *out_8934627023979057486);
void pose_H_10(double *state, double *unused, double *out_456432144623306927);
void pose_h_13(double *state, double *unused, double *out_6687713468022338325);
void pose_H_13(double *state, double *unused, double *out_8836876550289241657);
void pose_h_14(double *state, double *unused, double *out_6685023201088185678);
void pose_H_14(double *state, double *unused, double *out_1083552117191763266);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}