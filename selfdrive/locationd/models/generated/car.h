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
void car_err_fun(double *nom_x, double *delta_x, double *out_4427157744248383102);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7045651039375787549);
void car_H_mod_fun(double *state, double *out_6801501052206484974);
void car_f_fun(double *state, double dt, double *out_3034258192735895458);
void car_F_fun(double *state, double dt, double *out_742065270288018262);
void car_h_25(double *state, double *unused, double *out_3481936820668391358);
void car_H_25(double *state, double *unused, double *out_8705174448933309154);
void car_h_24(double *state, double *unused, double *out_3854856096366595906);
void car_H_24(double *state, double *unused, double *out_7568920025770742896);
void car_h_30(double *state, double *unused, double *out_7201541996220487817);
void car_H_30(double *state, double *unused, double *out_6186841490426060527);
void car_h_26(double *state, double *unused, double *out_8064484404317752036);
void car_H_26(double *state, double *unused, double *out_5400648479172508553);
void car_h_27(double *state, double *unused, double *out_1519736446992454287);
void car_H_27(double *state, double *unused, double *out_3963247419242117310);
void car_h_29(double *state, double *unused, double *out_7804857880896965334);
void car_H_29(double *state, double *unused, double *out_5676610146111668343);
void car_h_28(double *state, double *unused, double *out_8209170706848405003);
void car_H_28(double *state, double *unused, double *out_7687734910528352699);
void car_h_31(double *state, double *unused, double *out_5655780464015459824);
void car_H_31(double *state, double *unused, double *out_5373858203668834762);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}