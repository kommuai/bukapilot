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
void car_err_fun(double *nom_x, double *delta_x, double *out_111765226697154245);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4788536493034062394);
void car_H_mod_fun(double *state, double *out_264405265267972483);
void car_f_fun(double *state, double dt, double *out_5176658407878781139);
void car_F_fun(double *state, double dt, double *out_980642874926063009);
void car_h_25(double *state, double *unused, double *out_3677467358579401113);
void car_H_25(double *state, double *unused, double *out_5247560340259298588);
void car_h_24(double *state, double *unused, double *out_1057967543710495476);
void car_H_24(double *state, double *unused, double *out_953472339591862838);
void car_h_30(double *state, double *unused, double *out_7577727602274638707);
void car_H_30(double *state, double *unused, double *out_6282493391958636273);
void car_h_26(double *state, double *unused, double *out_5643109255599587962);
void car_H_26(double *state, double *unused, double *out_5904414404369610492);
void car_h_27(double *state, double *unused, double *out_8679140626240246758);
void car_H_27(double *state, double *unused, double *out_2943458081315633607);
void car_h_29(double *state, double *unused, double *out_1218581861991358433);
void car_H_29(double *state, double *unused, double *out_5772262047644244089);
void car_h_28(double *state, double *unused, double *out_2423315904281199504);
void car_H_28(double *state, double *unused, double *out_7592083008995776953);
void car_h_31(double *state, double *unused, double *out_40017830325378763);
void car_H_31(double *state, double *unused, double *out_5278206302136259016);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}