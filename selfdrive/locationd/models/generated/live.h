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
void live_H(double *in_vec, double *out_5981996195794704176);
void live_err_fun(double *nom_x, double *delta_x, double *out_3382311297148676679);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_9126087969886912714);
void live_H_mod_fun(double *state, double *out_8873173558200939950);
void live_f_fun(double *state, double dt, double *out_7964932075917777733);
void live_F_fun(double *state, double dt, double *out_3043151322596716258);
void live_h_4(double *state, double *unused, double *out_2039463821466064429);
void live_H_4(double *state, double *unused, double *out_29340977301384961);
void live_h_9(double *state, double *unused, double *out_5111051974793935586);
void live_H_9(double *state, double *unused, double *out_270530623930975606);
void live_h_10(double *state, double *unused, double *out_2510176841250443782);
void live_H_10(double *state, double *unused, double *out_7374308252425218806);
void live_h_12(double *state, double *unused, double *out_4707539137427834114);
void live_H_12(double *state, double *unused, double *out_650440002348978628);
void live_h_35(double *state, double *unused, double *out_2757158960835210838);
void live_H_35(double *state, double *unused, double *out_3396003034673992337);
void live_h_32(double *state, double *unused, double *out_4137431217939684901);
void live_H_32(double *state, double *unused, double *out_5611629559837696492);
void live_h_13(double *state, double *unused, double *out_6401343299066906868);
void live_H_13(double *state, double *unused, double *out_5662838703391315843);
void live_h_14(double *state, double *unused, double *out_5111051974793935586);
void live_H_14(double *state, double *unused, double *out_270530623930975606);
void live_h_33(double *state, double *unused, double *out_1356893102445955693);
void live_H_33(double *state, double *unused, double *out_6546560039312849941);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}