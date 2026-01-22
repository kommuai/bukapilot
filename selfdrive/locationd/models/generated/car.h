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
void car_err_fun(double *nom_x, double *delta_x, double *out_8484248936701507238);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1881682802568593388);
void car_H_mod_fun(double *state, double *out_2795609792051367190);
void car_f_fun(double *state, double dt, double *out_2726355231791695527);
void car_F_fun(double *state, double dt, double *out_7787842145159715712);
void car_h_25(double *state, double *unused, double *out_6613396955861225554);
void car_H_25(double *state, double *unused, double *out_727370857938124487);
void car_h_24(double *state, double *unused, double *out_5560718132626239476);
void car_H_24(double *state, double *unused, double *out_4285296456740447623);
void car_h_30(double *state, double *unused, double *out_328275521956714507);
void car_H_30(double *state, double *unused, double *out_7644061199429741242);
void car_h_26(double *state, double *unused, double *out_5394693040675057230);
void car_H_26(double *state, double *unused, double *out_3014132460935931737);
void car_h_27(double *state, double *unused, double *out_4830584414055260283);
void car_H_27(double *state, double *unused, double *out_5469297887629316331);
void car_h_29(double *state, double *unused, double *out_4673397745704131138);
void car_H_29(double *state, double *unused, double *out_8154292543744133426);
void car_h_28(double *state, double *unused, double *out_2707755848683944289);
void car_H_28(double *state, double *unused, double *out_1326463856309765276);
void car_h_31(double *state, double *unused, double *out_446282915890091251);
void car_H_31(double *state, double *unused, double *out_758016819815084915);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}