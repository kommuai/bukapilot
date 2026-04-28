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
void car_err_fun(double *nom_x, double *delta_x, double *out_1873531205278547893);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1518006983088921967);
void car_H_mod_fun(double *state, double *out_4479952202393314568);
void car_f_fun(double *state, double dt, double *out_5909533147568057795);
void car_F_fun(double *state, double dt, double *out_2217763294652930197);
void car_h_25(double *state, double *unused, double *out_4474342398968624688);
void car_H_25(double *state, double *unused, double *out_4985286644593611280);
void car_h_24(double *state, double *unused, double *out_7898975840914953389);
void car_H_24(double *state, double *unused, double *out_6885885622524422235);
void car_h_30(double *state, double *unused, double *out_2113485527507670475);
void car_H_30(double *state, double *unused, double *out_6544767087624323581);
void car_h_26(double *state, double *unused, double *out_8512527044921519621);
void car_H_26(double *state, double *unused, double *out_1243783325719555056);
void car_h_27(double *state, double *unused, double *out_670547664147022849);
void car_H_27(double *state, double *unused, double *out_8719530399424748492);
void car_h_29(double *state, double *unused, double *out_7115158795168516120);
void car_H_29(double *state, double *unused, double *out_6034535743309931397);
void car_h_28(double *state, double *unused, double *out_4814536367193809606);
void car_H_28(double *state, double *unused, double *out_7329809313330089645);
void car_h_31(double *state, double *unused, double *out_8715513113601863671);
void car_H_31(double *state, double *unused, double *out_5015932606470571708);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}