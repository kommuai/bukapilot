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
void car_err_fun(double *nom_x, double *delta_x, double *out_7807722455415062785);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_403857763333909611);
void car_H_mod_fun(double *state, double *out_4427032663813456926);
void car_f_fun(double *state, double dt, double *out_6512143296045430298);
void car_F_fun(double *state, double dt, double *out_4489943215438999676);
void car_h_25(double *state, double *unused, double *out_6013164719541088623);
void car_H_25(double *state, double *unused, double *out_5302341408152459906);
void car_h_24(double *state, double *unused, double *out_2616353451905830894);
void car_H_24(double *state, double *unused, double *out_3129691809146960340);
void car_h_30(double *state, double *unused, double *out_3793701877449921109);
void car_H_30(double *state, double *unused, double *out_7820674366659708533);
void car_h_26(double *state, double *unused, double *out_6520286259268309355);
void car_H_26(double *state, double *unused, double *out_1560838089278403682);
void car_h_27(double *state, double *unused, double *out_4998435919739762180);
void car_H_27(double *state, double *unused, double *out_8402475635865899866);
void car_h_29(double *state, double *unused, double *out_5273629982024268069);
void car_H_29(double *state, double *unused, double *out_8330905710974100717);
void car_h_28(double *state, double *unused, double *out_4427764741529312994);
void car_H_28(double *state, double *unused, double *out_3248506693904570143);
void car_h_31(double *state, double *unused, double *out_757670506809262313);
void car_H_31(double *state, double *unused, double *out_934629987045052206);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}