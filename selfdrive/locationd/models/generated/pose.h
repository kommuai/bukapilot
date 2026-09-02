#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8262091712122760042);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4072723838878715707);
void pose_H_mod_fun(double *state, double *out_4020441109412953693);
void pose_f_fun(double *state, double dt, double *out_501825406615906340);
void pose_F_fun(double *state, double dt, double *out_6707474879487101266);
void pose_h_4(double *state, double *unused, double *out_3489320313047190272);
void pose_H_4(double *state, double *unused, double *out_317256653424909271);
void pose_h_10(double *state, double *unused, double *out_1107332844316926936);
void pose_H_10(double *state, double *unused, double *out_5630855996947644706);
void pose_h_13(double *state, double *unused, double *out_8211807696530044364);
void pose_H_13(double *state, double *unused, double *out_3529530478757242072);
void pose_h_14(double *state, double *unused, double *out_4953897270879559792);
void pose_H_14(double *state, double *unused, double *out_4280497509764393800);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}