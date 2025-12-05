#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_4902815310777972944);
void live_err_fun(double *nom_x, double *delta_x, double *out_2825210736959872897);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_508074051030737404);
void live_H_mod_fun(double *state, double *out_7128586772680036655);
void live_f_fun(double *state, double dt, double *out_4215115184792228984);
void live_F_fun(double *state, double dt, double *out_2962932329016642907);
void live_h_4(double *state, double *unused, double *out_8348315870193401831);
void live_H_4(double *state, double *unused, double *out_5950516946179221358);
void live_h_9(double *state, double *unused, double *out_6005027635904854189);
void live_H_9(double *state, double *unused, double *out_5209008192265882788);
void live_h_10(double *state, double *unused, double *out_5906832640591852734);
void live_H_10(double *state, double *unused, double *out_1312614663441757344);
void live_h_12(double *state, double *unused, double *out_6927184728335595972);
void live_H_12(double *state, double *unused, double *out_7476770719498368463);
void live_h_35(double *state, double *unused, double *out_7107709877921068935);
void live_H_35(double *state, double *unused, double *out_4731207687173354754);
void live_h_32(double *state, double *unused, double *out_2662622144318950992);
void live_H_32(double *state, double *unused, double *out_8116125973669114629);
void live_h_13(double *state, double *unused, double *out_3700348359485734862);
void live_H_13(double *state, double *unused, double *out_1363426998167614485);
void live_h_14(double *state, double *unused, double *out_6005027635904854189);
void live_H_14(double *state, double *unused, double *out_5209008192265882788);
void live_h_33(double *state, double *unused, double *out_740749714656993439);
void live_H_33(double *state, double *unused, double *out_1580650682534497150);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}