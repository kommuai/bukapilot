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
void car_err_fun(double *nom_x, double *delta_x, double *out_9130459952118648993);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7134613961939753672);
void car_H_mod_fun(double *state, double *out_5012168072766857418);
void car_f_fun(double *state, double dt, double *out_9200361991106576411);
void car_F_fun(double *state, double dt, double *out_6355094581427914180);
void car_h_25(double *state, double *unused, double *out_3279746131878314915);
void car_H_25(double *state, double *unused, double *out_4695936252697041279);
void car_h_24(double *state, double *unused, double *out_6457534050077103686);
void car_H_24(double *state, double *unused, double *out_2523286653691541713);
void car_h_30(double *state, double *unused, double *out_5242583122295845460);
void car_H_30(double *state, double *unused, double *out_7214269211204289906);
void car_h_26(double *state, double *unused, double *out_6948336229454658902);
void car_H_26(double *state, double *unused, double *out_954432933822985055);
void car_h_27(double *state, double *unused, double *out_8123773559799036180);
void car_H_27(double *state, double *unused, double *out_5039505899403864995);
void car_h_29(double *state, double *unused, double *out_5127380634965125020);
void car_H_29(double *state, double *unused, double *out_7724500555518682090);
void car_h_28(double *state, double *unused, double *out_3122100292138190535);
void car_H_28(double *state, double *unused, double *out_2642101538449151516);
void car_h_31(double *state, double *unused, double *out_1188058135829320558);
void car_H_31(double *state, double *unused, double *out_328224831589633579);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}