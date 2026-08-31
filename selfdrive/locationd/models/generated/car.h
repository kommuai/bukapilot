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
void car_err_fun(double *nom_x, double *delta_x, double *out_2161272052723994744);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6541742512428809330);
void car_H_mod_fun(double *state, double *out_4810974423062247760);
void car_f_fun(double *state, double dt, double *out_3010562746147677696);
void car_F_fun(double *state, double dt, double *out_3081481798655148787);
void car_h_25(double *state, double *unused, double *out_1026424918856168907);
void car_H_25(double *state, double *unused, double *out_3110363609911612045);
void car_h_24(double *state, double *unused, double *out_3822164015639221808);
void car_H_24(double *state, double *unused, double *out_2713718305557752550);
void car_h_30(double *state, double *unused, double *out_5140689121114965396);
void car_H_30(double *state, double *unused, double *out_5628696568418860672);
void car_h_26(double *state, double *unused, double *out_1258352396298883750);
void car_H_26(double *state, double *unused, double *out_631139708962444179);
void car_h_27(double *state, double *unused, double *out_6028098186517014552);
void car_H_27(double *state, double *unused, double *out_3453933256618435761);
void car_h_29(double *state, double *unused, double *out_502040749965991439);
void car_H_29(double *state, double *unused, double *out_6138927912733252856);
void car_h_28(double *state, double *unused, double *out_5084588333615352117);
void car_H_28(double *state, double *unused, double *out_1056528895663722282);
void car_h_31(double *state, double *unused, double *out_8789690175676841348);
void car_H_31(double *state, double *unused, double *out_1257347811195795655);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}