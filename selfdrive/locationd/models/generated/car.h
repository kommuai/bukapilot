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
void car_err_fun(double *nom_x, double *delta_x, double *out_185066053000096811);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1419290231265680633);
void car_H_mod_fun(double *state, double *out_4094801764284795946);
void car_f_fun(double *state, double dt, double *out_5978480815451850811);
void car_F_fun(double *state, double dt, double *out_7673283050767777171);
void car_h_25(double *state, double *unused, double *out_2480916803033305557);
void car_H_25(double *state, double *unused, double *out_6430604276450895865);
void car_h_24(double *state, double *unused, double *out_8982522961258785433);
void car_H_24(double *state, double *unused, double *out_5017576028412487028);
void car_h_30(double *state, double *unused, double *out_4706036101424752772);
void car_H_30(double *state, double *unused, double *out_7488443467131047553);
void car_h_26(double *state, double *unused, double *out_2101630780616055121);
void car_H_26(double *state, double *unused, double *out_8274636478384599527);
void car_h_27(double *state, double *unused, double *out_7482590070694151202);
void car_H_27(double *state, double *unused, double *out_8734706535394560846);
void car_h_29(double *state, double *unused, double *out_8016428012659915453);
void car_H_29(double *state, double *unused, double *out_7998674811445439737);
void car_h_28(double *state, double *unused, double *out_6122962539684781374);
void car_H_28(double *state, double *unused, double *out_2916275794375909163);
void car_h_31(double *state, double *unused, double *out_1156532725220716793);
void car_H_31(double *state, double *unused, double *out_7648428376151248051);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}