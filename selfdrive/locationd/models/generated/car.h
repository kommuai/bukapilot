#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8049540775897896583);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7074179482604986343);
void car_H_mod_fun(double *state, double *out_2468025013813546553);
void car_f_fun(double *state, double dt, double *out_9198223713481844381);
void car_F_fun(double *state, double dt, double *out_1931036568146832837);
void car_h_25(double *state, double *unused, double *out_2954337136221408228);
void car_H_25(double *state, double *unused, double *out_7651867888905377355);
void car_h_24(double *state, double *unused, double *out_4961849974984120864);
void car_H_24(double *state, double *unused, double *out_8622226585798674695);
void car_h_30(double *state, double *unused, double *out_4404968528850291395);
void car_H_30(double *state, double *unused, double *out_5133534930398128728);
void car_h_26(double *state, double *unused, double *out_5008284413526768912);
void car_H_26(double *state, double *unused, double *out_7053372865930118037);
void car_h_27(double *state, double *unused, double *out_321510676526048361);
void car_H_27(double *state, double *unused, double *out_4092416542876141152);
void car_h_29(double *state, double *unused, double *out_596704738810554250);
void car_H_29(double *state, double *unused, double *out_6777411198990958247);
void car_h_28(double *state, double *unused, double *out_6934633040567403348);
void car_H_28(double *state, double *unused, double *out_8741041470556284498);
void car_h_31(double *state, double *unused, double *out_3816498090128942708);
void car_H_31(double *state, double *unused, double *out_6427164763696766561);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}