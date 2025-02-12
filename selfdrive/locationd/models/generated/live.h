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
void live_H(double *in_vec, double *out_1842037309239832165);
void live_err_fun(double *nom_x, double *delta_x, double *out_7459520144611025500);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7692144070246694375);
void live_H_mod_fun(double *state, double *out_8345365083708280516);
void live_f_fun(double *state, double dt, double *out_3081722599511375810);
void live_F_fun(double *state, double dt, double *out_506536653518304631);
void live_h_4(double *state, double *unused, double *out_2961707126876967185);
void live_H_4(double *state, double *unused, double *out_8596994171129817888);
void live_h_9(double *state, double *unused, double *out_1983343287718067603);
void live_H_9(double *state, double *unused, double *out_2562530967315286258);
void live_h_10(double *state, double *unused, double *out_579963622511965692);
void live_H_10(double *state, double *unused, double *out_3121802244627957316);
void live_h_12(double *state, double *unused, double *out_6856281857806408488);
void live_H_12(double *state, double *unused, double *out_2215735794087084892);
void live_h_31(double *state, double *unused, double *out_3109264816922422152);
void live_H_31(double *state, double *unused, double *out_6483087845207126352);
void live_h_32(double *state, double *unused, double *out_6928179830698587299);
void live_H_32(double *state, double *unused, double *out_6364245815635343031);
void live_h_13(double *state, double *unused, double *out_8981710361692711154);
void live_H_13(double *state, double *unused, double *out_8443080833909417963);
void live_h_14(double *state, double *unused, double *out_1983343287718067603);
void live_H_14(double *state, double *unused, double *out_2562530967315286258);
void live_h_33(double *state, double *unused, double *out_2482992789213280395);
void live_H_33(double *state, double *unused, double *out_3332530840568268748);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}