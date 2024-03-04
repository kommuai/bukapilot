#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_69284044656233370);
void live_err_fun(double *nom_x, double *delta_x, double *out_4029443404658213361);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3045923156268303314);
void live_H_mod_fun(double *state, double *out_3408748072006619981);
void live_f_fun(double *state, double dt, double *out_3128910193456553890);
void live_F_fun(double *state, double dt, double *out_8640737457769697043);
void live_h_4(double *state, double *unused, double *out_1029957331960551422);
void live_H_4(double *state, double *unused, double *out_7668422609226997312);
void live_h_9(double *state, double *unused, double *out_8439246969304359371);
void live_H_9(double *state, double *unused, double *out_7889459912202474962);
void live_h_10(double *state, double *unused, double *out_9153137073887306784);
void live_H_10(double *state, double *unused, double *out_7164784566698278522);
void live_h_12(double *state, double *unused, double *out_4151712998878462505);
void live_H_12(double *state, double *unused, double *out_8289521634274590979);
void live_h_31(double *state, double *unused, double *out_2182452030772518184);
void live_H_31(double *state, double *unused, double *out_7411659407109946928);
void live_h_32(double *state, double *unused, double *out_4005491774866910250);
void live_H_32(double *state, double *unused, double *out_8148603243751150192);
void live_h_13(double *state, double *unused, double *out_504688405213454300);
void live_H_13(double *state, double *unused, double *out_758218849203997542);
void live_h_14(double *state, double *unused, double *out_8439246969304359371);
void live_H_14(double *state, double *unused, double *out_7889459912202474962);
void live_h_33(double *state, double *unused, double *out_5584032496057928490);
void live_H_33(double *state, double *unused, double *out_4261102402471089324);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}