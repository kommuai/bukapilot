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
void live_H(double *in_vec, double *out_7859480754964530524);
void live_err_fun(double *nom_x, double *delta_x, double *out_8986050362125967762);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7525530853159689811);
void live_H_mod_fun(double *state, double *out_6186104941638907742);
void live_f_fun(double *state, double dt, double *out_8651086384178484332);
void live_F_fun(double *state, double dt, double *out_1248759915537762567);
void live_h_4(double *state, double *unused, double *out_2739690074395112302);
void live_H_4(double *state, double *unused, double *out_5341175974121445260);
void live_h_9(double *state, double *unused, double *out_6563200546140929039);
void live_H_9(double *state, double *unused, double *out_5818349164323658886);
void live_h_10(double *state, double *unused, double *out_6965480574925086950);
void live_H_10(double *state, double *unused, double *out_313045313705284117);
void live_h_12(double *state, double *unused, double *out_4043940044041615481);
void live_H_12(double *state, double *unused, double *out_8086111691556144561);
void live_h_31(double *state, double *unused, double *out_4110569958532914412);
void live_H_31(double *state, double *unused, double *out_5340548659231130852);
void live_h_32(double *state, double *unused, double *out_1914545394107241737);
void live_H_32(double *state, double *unused, double *out_8227030082079585348);
void live_h_13(double *state, double *unused, double *out_4130225914800510158);
void live_H_13(double *state, double *unused, double *out_8577704704024322280);
void live_h_14(double *state, double *unused, double *out_6563200546140929039);
void live_H_14(double *state, double *unused, double *out_5818349164323658886);
void live_h_33(double *state, double *unused, double *out_4401839487732620247);
void live_H_33(double *state, double *unused, double *out_6588349037576641376);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}