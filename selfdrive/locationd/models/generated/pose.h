#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4903838001852844195);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7297705039785129404);
void pose_H_mod_fun(double *state, double *out_4026989540286219337);
void pose_f_fun(double *state, double dt, double *out_5703061510448955959);
void pose_F_fun(double *state, double dt, double *out_4198032333973082495);
void pose_h_4(double *state, double *unused, double *out_526370016706783213);
void pose_H_4(double *state, double *unused, double *out_1570636421269345888);
void pose_h_10(double *state, double *unused, double *out_5670777292095012690);
void pose_H_10(double *state, double *unused, double *out_8988757871693847640);
void pose_h_13(double *state, double *unused, double *out_8332537746237142870);
void pose_H_13(double *state, double *unused, double *out_1641637404062986913);
void pose_h_14(double *state, double *unused, double *out_1423557163227896364);
void pose_H_14(double *state, double *unused, double *out_2392604435070138641);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}