#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5997402376607838659);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4197439267135692258);
void pose_H_mod_fun(double *state, double *out_3181760894870259401);
void pose_f_fun(double *state, double dt, double *out_5584365400920044031);
void pose_F_fun(double *state, double dt, double *out_467723448142448616);
void pose_h_4(double *state, double *unused, double *out_8344643797140242233);
void pose_H_4(double *state, double *unused, double *out_5018543989102965366);
void pose_h_10(double *state, double *unused, double *out_2557953024540116991);
void pose_H_10(double *state, double *unused, double *out_3761195516917964483);
void pose_h_13(double *state, double *unused, double *out_4615145277794404057);
void pose_H_13(double *state, double *unused, double *out_5817568876289885321);
void pose_h_14(double *state, double *unused, double *out_4531057776464354787);
void pose_H_14(double *state, double *unused, double *out_8981784845442449895);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}