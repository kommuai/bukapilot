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
void car_err_fun(double *nom_x, double *delta_x, double *out_8363476756673253557);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1977797309478282067);
void car_H_mod_fun(double *state, double *out_801006497277677196);
void car_f_fun(double *state, double dt, double *out_4787359001335959165);
void car_F_fun(double *state, double dt, double *out_4451177536231554248);
void car_h_25(double *state, double *unused, double *out_7740290648445303998);
void car_H_25(double *state, double *unused, double *out_4114592028682951273);
void car_h_24(double *state, double *unused, double *out_5892028796349262122);
void car_H_24(double *state, double *unused, double *out_6287241627688450839);
void car_h_30(double *state, double *unused, double *out_5319145870600894991);
void car_H_30(double *state, double *unused, double *out_2802098312808665482);
void car_h_26(double *state, double *unused, double *out_4224101575499120674);
void car_H_26(double *state, double *unused, double *out_810066058922150672);
void car_h_27(double *state, double *unused, double *out_441132223024443353);
void car_H_27(double *state, double *unused, double *out_627335001008240571);
void car_h_29(double *state, double *unused, double *out_8997568804065107999);
void car_H_29(double *state, double *unused, double *out_1086027725861310462);
void car_h_28(double *state, double *unused, double *out_3045537543833141004);
void car_H_28(double *state, double *unused, double *out_6168426742930841036);
void car_h_31(double *state, double *unused, double *out_9142255106595760966);
void car_H_31(double *state, double *unused, double *out_4083946066805990845);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}