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
void car_err_fun(double *nom_x, double *delta_x, double *out_4516876845569160053);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8817190908637309161);
void car_H_mod_fun(double *state, double *out_395299991356658731);
void car_f_fun(double *state, double dt, double *out_7860395240039354169);
void car_F_fun(double *state, double dt, double *out_4413235784017741404);
void car_h_25(double *state, double *unused, double *out_4300358826467228124);
void car_H_25(double *state, double *unused, double *out_3127680658632832946);
void car_h_24(double *state, double *unused, double *out_8384942898529154166);
void car_H_24(double *state, double *unused, double *out_6685606257435156082);
void car_h_30(double *state, double *unused, double *out_7375550020891972556);
void car_H_30(double *state, double *unused, double *out_5646013617140081573);
void car_h_26(double *state, double *unused, double *out_2373876753231126911);
void car_H_26(double *state, double *unused, double *out_613822660241223278);
void car_h_27(double *state, double *unused, double *out_701314441193617521);
void car_H_27(double *state, double *unused, double *out_7869607688324024790);
void car_h_29(double *state, double *unused, double *out_749316951435265646);
void car_H_29(double *state, double *unused, double *out_6156244961454473757);
void car_h_28(double *state, double *unused, double *out_242195411708044914);
void car_H_28(double *state, double *unused, double *out_1073845944384943183);
void car_h_31(double *state, double *unused, double *out_3462906430353444317);
void car_H_31(double *state, double *unused, double *out_3158326620509793374);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}