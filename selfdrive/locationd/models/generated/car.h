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
void car_err_fun(double *nom_x, double *delta_x, double *out_5652492354992792635);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2185114572777528757);
void car_H_mod_fun(double *state, double *out_4442504359244018288);
void car_f_fun(double *state, double dt, double *out_6496366283119291181);
void car_F_fun(double *state, double dt, double *out_8657312923978741203);
void car_h_25(double *state, double *unused, double *out_2531010797698552526);
void car_H_25(double *state, double *unused, double *out_4750683866668218949);
void car_h_24(double *state, double *unused, double *out_8812780496296381441);
void car_H_24(double *state, double *unused, double *out_1060701764354558016);
void car_h_30(double *state, double *unused, double *out_988431336622093080);
void car_H_30(double *state, double *unused, double *out_2166006474823397806);
void car_h_26(double *state, double *unused, double *out_7090950772355287470);
void car_H_26(double *state, double *unused, double *out_8492187185542275173);
void car_h_27(double *state, double *unused, double *out_8266388102699664748);
void car_H_27(double *state, double *unused, double *out_8756836977027105);
void car_h_29(double *state, double *unused, double *out_4404704626271163096);
void car_H_29(double *state, double *unused, double *out_1722119563846578138);
void car_h_28(double *state, double *unused, double *out_2047600034018565716);
void car_H_28(double *state, double *unused, double *out_6804518580916108712);
void car_h_31(double *state, double *unused, double *out_1106438730555469824);
void car_H_31(double *state, double *unused, double *out_4720037904791258521);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}