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
void live_H(double *in_vec, double *out_7830971254716250063);
void live_err_fun(double *nom_x, double *delta_x, double *out_4365604187244482342);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7658737680440832004);
void live_H_mod_fun(double *state, double *out_4671202476678888951);
void live_f_fun(double *state, double dt, double *out_559772793863358210);
void live_F_fun(double *state, double dt, double *out_8363789154424671762);
void live_h_4(double *state, double *unused, double *out_8566151119488780355);
void live_H_4(double *state, double *unused, double *out_3529679171784025507);
void live_h_9(double *state, double *unused, double *out_6927880459251202853);
void live_H_9(double *state, double *unused, double *out_3770868818413616152);
void live_h_10(double *state, double *unused, double *out_3208087567610553222);
void live_H_10(double *state, double *unused, double *out_3050187738412092720);
void live_h_12(double *state, double *unused, double *out_5917549616677067887);
void live_H_12(double *state, double *unused, double *out_8549135579815987302);
void live_h_35(double *state, double *unused, double *out_6639701368155423683);
void live_H_35(double *state, double *unused, double *out_7152045461568550605);
void live_h_32(double *state, double *unused, double *out_3269805427791286076);
void live_H_32(double *state, double *unused, double *out_3604201121581391876);
void live_h_13(double *state, double *unused, double *out_8126218195559969015);
void live_H_13(double *state, double *unused, double *out_439013008876913351);
void live_h_14(double *state, double *unused, double *out_6927880459251202853);
void live_H_14(double *state, double *unused, double *out_3770868818413616152);
void live_h_33(double *state, double *unused, double *out_6528938297909394329);
void live_H_33(double *state, double *unused, double *out_4001488456929693001);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}