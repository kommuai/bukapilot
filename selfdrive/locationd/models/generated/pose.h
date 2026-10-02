#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1862087488242639687);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7512207833057071994);
void pose_H_mod_fun(double *state, double *out_5595754603494457870);
void pose_f_fun(double *state, double dt, double *out_3090951408168040086);
void pose_F_fun(double *state, double dt, double *out_6851591457791860138);
void pose_h_4(double *state, double *unused, double *out_6888167971702813252);
void pose_H_4(double *state, double *unused, double *out_443236218455377135);
void pose_h_10(double *state, double *unused, double *out_2926285046309371719);
void pose_H_10(double *state, double *unused, double *out_7083396780515160844);
void pose_h_13(double *state, double *unused, double *out_101780861794540917);
void pose_H_13(double *state, double *unused, double *out_2769037606876955666);
void pose_h_14(double *state, double *unused, double *out_616030427240878133);
void pose_H_14(double *state, double *unused, double *out_3526024650750749431);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}