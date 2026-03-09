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
void car_err_fun(double *nom_x, double *delta_x, double *out_970018053595319883);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2532034153412208165);
void car_H_mod_fun(double *state, double *out_628905704139337664);
void car_f_fun(double *state, double dt, double *out_2381603625928507972);
void car_F_fun(double *state, double dt, double *out_4153852161505245629);
void car_h_25(double *state, double *unused, double *out_4848459882080520497);
void car_H_25(double *state, double *unused, double *out_2894142934506027484);
void car_h_24(double *state, double *unused, double *out_880766775357849931);
void car_H_24(double *state, double *unused, double *out_1910077063664713816);
void car_h_30(double *state, double *unused, double *out_3648200799183219495);
void car_H_30(double *state, double *unused, double *out_4022547406985589271);
void car_h_26(double *state, double *unused, double *out_8046152880464848507);
void car_H_26(double *state, double *unused, double *out_410383035254773117);
void car_h_27(double *state, double *unused, double *out_1811673830636187812);
void car_H_27(double *state, double *unused, double *out_1847784095185164360);
void car_h_29(double *state, double *unused, double *out_1353472468477626150);
void car_H_29(double *state, double *unused, double *out_4532778751299981455);
void car_h_28(double *state, double *unused, double *out_868163977734525377);
void car_H_28(double *state, double *unused, double *out_549620265769549119);
void car_h_31(double *state, double *unused, double *out_4573265819796014608);
void car_H_31(double *state, double *unused, double *out_4182532316005789769);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}