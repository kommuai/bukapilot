#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7166728709141313111);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7904298925836893155);
void car_H_mod_fun(double *state, double *out_2723439132888064356);
void car_f_fun(double *state, double dt, double *out_9222521799993261203);
void car_F_fun(double *state, double dt, double *out_5859762924884839019);
void car_h_25(double *state, double *unused, double *out_3573010150517271071);
void car_H_25(double *state, double *unused, double *out_3773583827001965595);
void car_h_24(double *state, double *unused, double *out_4865118116114068912);
void car_H_24(double *state, double *unused, double *out_8642398692029672447);
void car_h_30(double *state, double *unused, double *out_2057634506830890826);
void car_H_30(double *state, double *unused, double *out_8301280157129573793);
void car_h_26(double *state, double *unused, double *out_2944038760829954819);
void car_H_26(double *state, double *unused, double *out_7515087145876021819);
void car_h_27(double *state, double *unused, double *out_8574683418178116716);
void car_H_27(double *state, double *unused, double *out_7970700604779552912);
void car_h_29(double *state, double *unused, double *out_7124052025549233549);
void car_H_29(double *state, double *unused, double *out_7791048812815181609);
void car_h_28(double *state, double *unused, double *out_4870387387870589255);
void car_H_28(double *state, double *unused, double *out_5573296243824839433);
void car_h_31(double *state, double *unused, double *out_288673325911230581);
void car_H_31(double *state, double *unused, double *out_8141295248109373295);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}