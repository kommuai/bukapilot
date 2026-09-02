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
void car_err_fun(double *nom_x, double *delta_x, double *out_6184164552224961757);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1205517025722757514);
void car_H_mod_fun(double *state, double *out_8692073091461732284);
void car_f_fun(double *state, double dt, double *out_6022347720067266341);
void car_F_fun(double *state, double dt, double *out_2034100707598491156);
void car_h_25(double *state, double *unused, double *out_1603510504705161482);
void car_H_25(double *state, double *unused, double *out_561482756335582095);
void car_h_24(double *state, double *unused, double *out_8697386051471671882);
void car_H_24(double *state, double *unused, double *out_1611166842669917471);
void car_h_30(double *state, double *unused, double *out_6277762146048887703);
void car_H_30(double *state, double *unused, double *out_7478173097827198850);
void car_h_26(double *state, double *unused, double *out_7206601929979370681);
void car_H_26(double *state, double *unused, double *out_3180020562538474129);
void car_h_27(double *state, double *unused, double *out_3398162762955684163);
void car_H_27(double *state, double *unused, double *out_5303409786026773939);
void car_h_29(double *state, double *unused, double *out_7259846239384185815);
void car_H_29(double *state, double *unused, double *out_3590047059157222906);
void car_h_28(double *state, double *unused, double *out_585610659685830780);
void car_H_28(double *state, double *unused, double *out_1492351957912307668);
void car_h_31(double *state, double *unused, double *out_5240960032959183832);
void car_H_31(double *state, double *unused, double *out_592128718212542523);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}