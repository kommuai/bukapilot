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
void live_H(double *in_vec, double *out_6398733232337652556);
void live_err_fun(double *nom_x, double *delta_x, double *out_4487673675287653984);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3564779497782253460);
void live_H_mod_fun(double *state, double *out_8271283838552333742);
void live_f_fun(double *state, double dt, double *out_8060587478065872808);
void live_F_fun(double *state, double dt, double *out_1869954971794230849);
void live_h_4(double *state, double *unused, double *out_8299928938932305509);
void live_H_4(double *state, double *unused, double *out_5734928039648235607);
void live_h_9(double *state, double *unused, double *out_3776323740026048546);
void live_H_9(double *state, double *unused, double *out_5424597098796868539);
void live_h_10(double *state, double *unused, double *out_4248019012141070217);
void live_H_10(double *state, double *unused, double *out_88789431243406781);
void live_h_12(double *state, double *unused, double *out_4297402469638289135);
void live_H_12(double *state, double *unused, double *out_7692359626029354214);
void live_h_35(double *state, double *unused, double *out_5087796782319810294);
void live_H_35(double *state, double *unused, double *out_9101590097020842983);
void live_h_32(double *state, double *unused, double *out_3600710899185999736);
void live_H_32(double *state, double *unused, double *out_3502179684153760750);
void live_h_13(double *state, double *unused, double *out_8526739897726141195);
void live_H_13(double *state, double *unused, double *out_613418987771083003);
void live_h_14(double *state, double *unused, double *out_3776323740026048546);
void live_H_14(double *state, double *unused, double *out_5424597098796868539);
void live_h_33(double *state, double *unused, double *out_1248064760613936342);
void live_H_33(double *state, double *unused, double *out_6194596972049851029);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}