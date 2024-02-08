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
void car_err_fun(double *nom_x, double *delta_x, double *out_122908531796567845);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2538234936892293981);
void car_H_mod_fun(double *state, double *out_4589170859792487472);
void car_f_fun(double *state, double dt, double *out_2042901932542470359);
void car_F_fun(double *state, double dt, double *out_749452283589495020);
void car_h_25(double *state, double *unused, double *out_8910777389406178182);
void car_H_25(double *state, double *unused, double *out_4447527938048292635);
void car_h_24(double *state, double *unused, double *out_6983956359321902164);
void car_H_24(double *state, double *unused, double *out_2274878339042793069);
void car_h_30(double *state, double *unused, double *out_5437663386024001570);
void car_H_30(double *state, double *unused, double *out_4318188990905052565);
void car_h_26(double *state, double *unused, double *out_435990118363155925);
void car_H_26(double *state, double *unused, double *out_706024619174236411);
void car_h_27(double *state, double *unused, double *out_8152927244245359372);
void car_H_27(double *state, double *unused, double *out_2143425679104627654);
void car_h_29(double *state, double *unused, double *out_6040979270700479087);
void car_H_29(double *state, double *unused, double *out_430062952235076621);
void car_h_28(double *state, double *unused, double *out_9096437097147021807);
void car_H_28(double *state, double *unused, double *out_2393693223800402872);
void car_h_31(double *state, double *unused, double *out_5391335255085532576);
void car_H_31(double *state, double *unused, double *out_4478173899925253063);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}