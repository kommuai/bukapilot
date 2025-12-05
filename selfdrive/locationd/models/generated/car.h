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
void car_err_fun(double *nom_x, double *delta_x, double *out_3639798656427526341);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9141996593500649878);
void car_H_mod_fun(double *state, double *out_7268018853631160798);
void car_f_fun(double *state, double dt, double *out_3932083014329006242);
void car_F_fun(double *state, double dt, double *out_5001605271866367269);
void car_h_25(double *state, double *unused, double *out_8062841473513369717);
void car_H_25(double *state, double *unused, double *out_2364327028446495958);
void car_h_24(double *state, double *unused, double *out_2051114860567036014);
void car_H_24(double *state, double *unused, double *out_7233141893474202810);
void car_h_30(double *state, double *unused, double *out_4425391945259347367);
void car_H_30(double *state, double *unused, double *out_6892023358574104156);
void car_h_26(double *state, double *unused, double *out_1388605893815014682);
void car_H_26(double *state, double *unused, double *out_6105830347320552182);
void car_h_27(double *state, double *unused, double *out_9006351326415032152);
void car_H_27(double *state, double *unused, double *out_4668429287390160939);
void car_h_29(double *state, double *unused, double *out_5368901798161009802);
void car_H_29(double *state, double *unused, double *out_6381792014259711972);
void car_h_28(double *state, double *unused, double *out_6403606695850186779);
void car_H_28(double *state, double *unused, double *out_4418161742694385721);
void car_h_31(double *state, double *unused, double *out_6244759337255205356);
void car_H_31(double *state, double *unused, double *out_6732038449553903658);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}