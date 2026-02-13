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
void live_H(double *in_vec, double *out_4049447043521740533);
void live_err_fun(double *nom_x, double *delta_x, double *out_3730734691855177007);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6934695813362053517);
void live_H_mod_fun(double *state, double *out_1008068711703099321);
void live_f_fun(double *state, double dt, double *out_3411123678743551243);
void live_F_fun(double *state, double dt, double *out_9009844614575140533);
void live_h_4(double *state, double *unused, double *out_4268146737521773333);
void live_H_4(double *state, double *unused, double *out_502043663951517747);
void live_h_9(double *state, double *unused, double *out_8094251101279215540);
void live_H_9(double *state, double *unused, double *out_743233310581108392);
void live_h_10(double *state, double *unused, double *out_5431845352141897093);
void live_H_10(double *state, double *unused, double *out_5705311635526615796);
void live_h_12(double *state, double *unused, double *out_1007220560385549881);
void live_H_12(double *state, double *unused, double *out_5521500071983479542);
void live_h_35(double *state, double *unused, double *out_6165321942463348325);
void live_H_35(double *state, double *unused, double *out_8267063104308493251);
void live_h_32(double *state, double *unused, double *out_900422001410582414);
void live_H_32(double *state, double *unused, double *out_7396022804982640772);
void live_h_13(double *state, double *unused, double *out_5805045031099027380);
void live_H_13(double *state, double *unused, double *out_5739861060419527940);
void live_h_14(double *state, double *unused, double *out_8094251101279215540);
void live_H_14(double *state, double *unused, double *out_743233310581108392);
void live_h_33(double *state, double *unused, double *out_7468219983623598780);
void live_H_33(double *state, double *unused, double *out_7029123964762200761);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}