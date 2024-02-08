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
void live_H(double *in_vec, double *out_1837242654657482381);
void live_err_fun(double *nom_x, double *delta_x, double *out_1049287755312093099);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_51980762048143343);
void live_H_mod_fun(double *state, double *out_36641115734452068);
void live_f_fun(double *state, double dt, double *out_3639431347680991822);
void live_F_fun(double *state, double dt, double *out_6769763949434269633);
void live_h_4(double *state, double *unused, double *out_3106790310301024044);
void live_H_4(double *state, double *unused, double *out_1791578705534762859);
void live_h_9(double *state, double *unused, double *out_2731228141922491785);
void live_H_9(double *state, double *unused, double *out_1550389058905172214);
void live_h_10(double *state, double *unused, double *out_4460143636780078925);
void live_H_10(double *state, double *unused, double *out_8453040072692771662);
void live_h_12(double *state, double *unused, double *out_4735359724762829200);
void live_H_12(double *state, double *unused, double *out_3227877702497198936);
void live_h_31(double *state, double *unused, double *out_422741657314969244);
void live_H_31(double *state, double *unused, double *out_5973440734822212645);
void live_h_32(double *state, double *unused, double *out_6119675094862218689);
void live_H_32(double *state, double *unused, double *out_1632435959089105757);
void live_h_13(double *state, double *unused, double *out_6589541705407183911);
void live_H_13(double *state, double *unused, double *out_2741759176471186047);
void live_h_14(double *state, double *unused, double *out_2731228141922491785);
void live_H_14(double *state, double *unused, double *out_1550389058905172214);
void live_h_33(double *state, double *unused, double *out_6883257441562439238);
void live_H_33(double *state, double *unused, double *out_9123997739461070249);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}