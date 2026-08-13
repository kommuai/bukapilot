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
void car_err_fun(double *nom_x, double *delta_x, double *out_500439478528929704);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8177809488341246547);
void car_H_mod_fun(double *state, double *out_48951311923778637);
void car_f_fun(double *state, double dt, double *out_4996357532719299431);
void car_F_fun(double *state, double dt, double *out_78166648368307141);
void car_h_25(double *state, double *unused, double *out_5046435348237256990);
void car_H_25(double *state, double *unused, double *out_9204604535873535742);
void car_h_24(double *state, double *unused, double *out_892464665149108733);
void car_H_24(double *state, double *unused, double *out_14074351766820649);
void car_h_30(double *state, double *unused, double *out_5321629410521762879);
void car_H_30(double *state, double *unused, double *out_4676908205745927544);
void car_h_26(double *state, double *unused, double *out_4718313525845285362);
void car_H_26(double *state, double *unused, double *out_5463101216999479518);
void car_h_27(double *state, double *unused, double *out_4102925495335594555);
void car_H_27(double *state, double *unused, double *out_2502144893945502633);
void car_h_29(double *state, double *unused, double *out_3660339761485077886);
void car_H_29(double *state, double *unused, double *out_5187139550060319728);
void car_h_28(double *state, double *unused, double *out_9104598762996440200);
void car_H_28(double *state, double *unused, double *out_7150769821625645979);
void car_h_31(double *state, double *unused, double *out_4760431248753752742);
void car_H_31(double *state, double *unused, double *out_4836893114766128042);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}