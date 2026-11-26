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
void car_err_fun(double *nom_x, double *delta_x, double *out_3728982769992195016);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7936685168515433141);
void car_H_mod_fun(double *state, double *out_7513730895619064577);
void car_f_fun(double *state, double dt, double *out_2023060963930489365);
void car_F_fun(double *state, double dt, double *out_5000204650175358919);
void car_h_25(double *state, double *unused, double *out_6775712609208471806);
void car_H_25(double *state, double *unused, double *out_2610039070434399737);
void car_h_24(double *state, double *unused, double *out_6623912581949403498);
void car_H_24(double *state, double *unused, double *out_7478853935462106589);
void car_h_30(double *state, double *unused, double *out_8723305474145594849);
void car_H_30(double *state, double *unused, double *out_7137735400562007935);
void car_h_26(double *state, double *unused, double *out_5686519422701262164);
void car_H_26(double *state, double *unused, double *out_6351542389308455961);
void car_h_27(double *state, double *unused, double *out_5832202756306809371);
void car_H_27(double *state, double *unused, double *out_4914141329378064718);
void car_h_29(double *state, double *unused, double *out_3721632206484749204);
void car_H_29(double *state, double *unused, double *out_6627504056247615751);
void car_h_28(double *state, double *unused, double *out_2105693166963939297);
void car_H_28(double *state, double *unused, double *out_4663873784682289500);
void car_h_31(double *state, double *unused, double *out_4900196238150728874);
void car_H_31(double *state, double *unused, double *out_6977750491541807437);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}