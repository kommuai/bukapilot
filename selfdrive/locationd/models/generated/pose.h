#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4436213307949090908);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4288605168489656967);
void pose_H_mod_fun(double *state, double *out_6602564711839087419);
void pose_f_fun(double *state, double dt, double *out_976073864278997094);
void pose_F_fun(double *state, double dt, double *out_3492059102621666200);
void pose_h_4(double *state, double *unused, double *out_4637603071142990440);
void pose_H_4(double *state, double *unused, double *out_8072372152026556625);
void pose_h_10(double *state, double *unused, double *out_6628455498773735725);
void pose_H_10(double *state, double *unused, double *out_4205718837824556590);
void pose_h_13(double *state, double *unused, double *out_3016281579283853723);
void pose_H_13(double *state, double *unused, double *out_461740943709855696);
void pose_h_14(double *state, double *unused, double *out_8170658983540952892);
void pose_H_14(double *state, double *unused, double *out_7291583489387622695);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}