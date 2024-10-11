#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8975246762864864452);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4540036244876903697);
void car_H_mod_fun(double *state, double *out_3842189826305601254);
void car_f_fun(double *state, double dt, double *out_4130278799096072198);
void car_F_fun(double *state, double dt, double *out_2680083277103679996);
void car_h_25(double *state, double *unused, double *out_3861349082676883418);
void car_H_25(double *state, double *unused, double *out_4949154357715001298);
void car_h_24(double *state, double *unused, double *out_6369036304708791866);
void car_H_24(double *state, double *unused, double *out_202645030748492264);
void car_h_30(double *state, double *unused, double *out_4006868906897538389);
void car_H_30(double *state, double *unused, double *out_7467487316222249925);
void car_h_26(double *state, double *unused, double *out_994804360763307256);
void car_H_26(double *state, double *unused, double *out_1207651038840945074);
void car_h_27(double *state, double *unused, double *out_8863022350337729063);
void car_H_27(double *state, double *unused, double *out_8755662686303358474);
void car_h_29(double *state, double *unused, double *out_6635995733016337592);
void car_H_29(double *state, double *unused, double *out_7977718660536642109);
void car_h_28(double *state, double *unused, double *out_7919512497436066628);
void car_H_28(double *state, double *unused, double *out_2895319643467111535);
void car_h_31(double *state, double *unused, double *out_223899554422861068);
void car_H_31(double *state, double *unused, double *out_4979800319591961726);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}