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
void live_H(double *in_vec, double *out_1894823507694594399);
void live_err_fun(double *nom_x, double *delta_x, double *out_8630857021434385873);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5908712063546568381);
void live_H_mod_fun(double *state, double *out_2355039495119276295);
void live_f_fun(double *state, double dt, double *out_8957858313121654405);
void live_F_fun(double *state, double dt, double *out_1945673585192076908);
void live_h_4(double *state, double *unused, double *out_6206602237265521381);
void live_H_4(double *state, double *unused, double *out_3794931149444130665);
void live_h_9(double *state, double *unused, double *out_6212945661161122643);
void live_H_9(double *state, double *unused, double *out_3492287785820316805);
void live_h_10(double *state, double *unused, double *out_6742967031195039179);
void live_H_10(double *state, double *unused, double *out_1712353419570829068);
void live_h_12(double *state, double *unused, double *out_6309565259756635743);
void live_H_12(double *state, double *unused, double *out_8270554547222687955);
void live_h_35(double *state, double *unused, double *out_1015025099746934569);
void live_H_35(double *state, double *unused, double *out_6617760196563333536);
void live_h_32(double *state, double *unused, double *out_5923868283126289758);
void live_H_32(double *state, double *unused, double *out_297104307130782820);
void live_h_13(double *state, double *unused, double *out_5474494654195363573);
void live_H_13(double *state, double *unused, double *out_807062589687609471);
void live_h_14(double *state, double *unused, double *out_6212945661161122643);
void live_H_14(double *state, double *unused, double *out_3492287785820316805);
void live_h_33(double *state, double *unused, double *out_5929295816428676035);
void live_H_33(double *state, double *unused, double *out_8678426872507360476);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}