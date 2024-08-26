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
void car_err_fun(double *nom_x, double *delta_x, double *out_5652973485134955936);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5828134910451363179);
void car_H_mod_fun(double *state, double *out_5063456026301800763);
void car_f_fun(double *state, double dt, double *out_9008031268712533140);
void car_F_fun(double *state, double dt, double *out_399800080831945283);
void car_h_25(double *state, double *unused, double *out_8429616611094014799);
void car_H_25(double *state, double *unused, double *out_3751244995790767177);
void car_h_24(double *state, double *unused, double *out_6860572812747741310);
void car_H_24(double *state, double *unused, double *out_7752957074421275810);
void car_h_30(double *state, double *unused, double *out_1841788684617719636);
void car_H_30(double *state, double *unused, double *out_7778808736427167684);
void car_h_26(double *state, double *unused, double *out_898278831716057201);
void car_H_26(double *state, double *unused, double *out_9741676916710953);
void car_h_27(double *state, double *unused, double *out_780271437782680457);
void car_H_27(double *state, double *unused, double *out_8493172025481959021);
void car_h_29(double *state, double *unused, double *out_4878574736062052321);
void car_H_29(double *state, double *unused, double *out_7268577392112775500);
void car_h_28(double *state, double *unused, double *out_201812655552617655);
void car_H_28(double *state, double *unused, double *out_6095767664527245542);
void car_h_31(double *state, double *unused, double *out_5775956747982297834);
void car_H_31(double *state, double *unused, double *out_3781890957667727605);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}