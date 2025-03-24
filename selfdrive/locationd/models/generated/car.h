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
void car_err_fun(double *nom_x, double *delta_x, double *out_8607305045532154138);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7480351364201551916);
void car_H_mod_fun(double *state, double *out_710053212192307984);
void car_f_fun(double *state, double dt, double *out_8301108005870377691);
void car_F_fun(double *state, double dt, double *out_571695760952113270);
void car_h_25(double *state, double *unused, double *out_1568893959961481892);
void car_H_25(double *state, double *unused, double *out_7862409623760403888);
void car_h_24(double *state, double *unused, double *out_8539449502525998869);
void car_H_24(double *state, double *unused, double *out_8846475494601717556);
void car_h_30(double *state, double *unused, double *out_7789271142808495861);
void car_H_30(double *state, double *unused, double *out_3334713293632795690);
void car_h_26(double *state, double *unused, double *out_8053067610092358104);
void car_H_26(double *state, double *unused, double *out_4120906304886347664);
void car_h_27(double *state, double *unused, double *out_6570567227622327537);
void car_H_27(double *state, double *unused, double *out_5558307364816738907);
void car_h_29(double *state, double *unused, double *out_5591055412182713032);
void car_H_29(double *state, double *unused, double *out_3844944637947187874);
void car_h_28(double *state, double *unused, double *out_4226831672775889737);
void car_H_28(double *state, double *unused, double *out_1237454379122342700);
void car_h_31(double *state, double *unused, double *out_2292789516467019760);
void car_H_31(double *state, double *unused, double *out_7893055585637364316);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}