#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3310266499749110268);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9111078211151530773);
void car_H_mod_fun(double *state, double *out_8318288169772448250);
void car_f_fun(double *state, double dt, double *out_3923135191490236514);
void car_F_fun(double *state, double dt, double *out_2349988940558973348);
void car_h_25(double *state, double *unused, double *out_3861404861143990202);
void car_H_25(double *state, double *unused, double *out_4795307519782956573);
void car_h_24(double *state, double *unused, double *out_3341134745436053412);
void car_H_24(double *state, double *unused, double *out_2348295926063274966);
void car_h_30(double *state, double *unused, double *out_236898437135381662);
void car_H_30(double *state, double *unused, double *out_9123740223798986845);
void car_h_26(double *state, double *unused, double *out_8745070734871459103);
void car_H_26(double *state, double *unused, double *out_8536810838657012797);
void car_h_27(double *state, double *unused, double *out_8863078128804835847);
void car_H_27(double *state, double *unused, double *out_7099409778726621554);
void car_h_29(double *state, double *unused, double *out_4764774830525463983);
void car_H_29(double *state, double *unused, double *out_8812772505596172587);
void car_h_28(double *state, double *unused, double *out_5826292077360503162);
void car_H_28(double *state, double *unused, double *out_4551572551043848455);
void car_h_31(double *state, double *unused, double *out_3586210798859484313);
void car_H_31(double *state, double *unused, double *out_4764661557905996145);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}