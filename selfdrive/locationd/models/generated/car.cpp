#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5652492354992792635) {
   out_5652492354992792635[0] = delta_x[0] + nom_x[0];
   out_5652492354992792635[1] = delta_x[1] + nom_x[1];
   out_5652492354992792635[2] = delta_x[2] + nom_x[2];
   out_5652492354992792635[3] = delta_x[3] + nom_x[3];
   out_5652492354992792635[4] = delta_x[4] + nom_x[4];
   out_5652492354992792635[5] = delta_x[5] + nom_x[5];
   out_5652492354992792635[6] = delta_x[6] + nom_x[6];
   out_5652492354992792635[7] = delta_x[7] + nom_x[7];
   out_5652492354992792635[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2185114572777528757) {
   out_2185114572777528757[0] = -nom_x[0] + true_x[0];
   out_2185114572777528757[1] = -nom_x[1] + true_x[1];
   out_2185114572777528757[2] = -nom_x[2] + true_x[2];
   out_2185114572777528757[3] = -nom_x[3] + true_x[3];
   out_2185114572777528757[4] = -nom_x[4] + true_x[4];
   out_2185114572777528757[5] = -nom_x[5] + true_x[5];
   out_2185114572777528757[6] = -nom_x[6] + true_x[6];
   out_2185114572777528757[7] = -nom_x[7] + true_x[7];
   out_2185114572777528757[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4442504359244018288) {
   out_4442504359244018288[0] = 1.0;
   out_4442504359244018288[1] = 0.0;
   out_4442504359244018288[2] = 0.0;
   out_4442504359244018288[3] = 0.0;
   out_4442504359244018288[4] = 0.0;
   out_4442504359244018288[5] = 0.0;
   out_4442504359244018288[6] = 0.0;
   out_4442504359244018288[7] = 0.0;
   out_4442504359244018288[8] = 0.0;
   out_4442504359244018288[9] = 0.0;
   out_4442504359244018288[10] = 1.0;
   out_4442504359244018288[11] = 0.0;
   out_4442504359244018288[12] = 0.0;
   out_4442504359244018288[13] = 0.0;
   out_4442504359244018288[14] = 0.0;
   out_4442504359244018288[15] = 0.0;
   out_4442504359244018288[16] = 0.0;
   out_4442504359244018288[17] = 0.0;
   out_4442504359244018288[18] = 0.0;
   out_4442504359244018288[19] = 0.0;
   out_4442504359244018288[20] = 1.0;
   out_4442504359244018288[21] = 0.0;
   out_4442504359244018288[22] = 0.0;
   out_4442504359244018288[23] = 0.0;
   out_4442504359244018288[24] = 0.0;
   out_4442504359244018288[25] = 0.0;
   out_4442504359244018288[26] = 0.0;
   out_4442504359244018288[27] = 0.0;
   out_4442504359244018288[28] = 0.0;
   out_4442504359244018288[29] = 0.0;
   out_4442504359244018288[30] = 1.0;
   out_4442504359244018288[31] = 0.0;
   out_4442504359244018288[32] = 0.0;
   out_4442504359244018288[33] = 0.0;
   out_4442504359244018288[34] = 0.0;
   out_4442504359244018288[35] = 0.0;
   out_4442504359244018288[36] = 0.0;
   out_4442504359244018288[37] = 0.0;
   out_4442504359244018288[38] = 0.0;
   out_4442504359244018288[39] = 0.0;
   out_4442504359244018288[40] = 1.0;
   out_4442504359244018288[41] = 0.0;
   out_4442504359244018288[42] = 0.0;
   out_4442504359244018288[43] = 0.0;
   out_4442504359244018288[44] = 0.0;
   out_4442504359244018288[45] = 0.0;
   out_4442504359244018288[46] = 0.0;
   out_4442504359244018288[47] = 0.0;
   out_4442504359244018288[48] = 0.0;
   out_4442504359244018288[49] = 0.0;
   out_4442504359244018288[50] = 1.0;
   out_4442504359244018288[51] = 0.0;
   out_4442504359244018288[52] = 0.0;
   out_4442504359244018288[53] = 0.0;
   out_4442504359244018288[54] = 0.0;
   out_4442504359244018288[55] = 0.0;
   out_4442504359244018288[56] = 0.0;
   out_4442504359244018288[57] = 0.0;
   out_4442504359244018288[58] = 0.0;
   out_4442504359244018288[59] = 0.0;
   out_4442504359244018288[60] = 1.0;
   out_4442504359244018288[61] = 0.0;
   out_4442504359244018288[62] = 0.0;
   out_4442504359244018288[63] = 0.0;
   out_4442504359244018288[64] = 0.0;
   out_4442504359244018288[65] = 0.0;
   out_4442504359244018288[66] = 0.0;
   out_4442504359244018288[67] = 0.0;
   out_4442504359244018288[68] = 0.0;
   out_4442504359244018288[69] = 0.0;
   out_4442504359244018288[70] = 1.0;
   out_4442504359244018288[71] = 0.0;
   out_4442504359244018288[72] = 0.0;
   out_4442504359244018288[73] = 0.0;
   out_4442504359244018288[74] = 0.0;
   out_4442504359244018288[75] = 0.0;
   out_4442504359244018288[76] = 0.0;
   out_4442504359244018288[77] = 0.0;
   out_4442504359244018288[78] = 0.0;
   out_4442504359244018288[79] = 0.0;
   out_4442504359244018288[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6496366283119291181) {
   out_6496366283119291181[0] = state[0];
   out_6496366283119291181[1] = state[1];
   out_6496366283119291181[2] = state[2];
   out_6496366283119291181[3] = state[3];
   out_6496366283119291181[4] = state[4];
   out_6496366283119291181[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6496366283119291181[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6496366283119291181[7] = state[7];
   out_6496366283119291181[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8657312923978741203) {
   out_8657312923978741203[0] = 1;
   out_8657312923978741203[1] = 0;
   out_8657312923978741203[2] = 0;
   out_8657312923978741203[3] = 0;
   out_8657312923978741203[4] = 0;
   out_8657312923978741203[5] = 0;
   out_8657312923978741203[6] = 0;
   out_8657312923978741203[7] = 0;
   out_8657312923978741203[8] = 0;
   out_8657312923978741203[9] = 0;
   out_8657312923978741203[10] = 1;
   out_8657312923978741203[11] = 0;
   out_8657312923978741203[12] = 0;
   out_8657312923978741203[13] = 0;
   out_8657312923978741203[14] = 0;
   out_8657312923978741203[15] = 0;
   out_8657312923978741203[16] = 0;
   out_8657312923978741203[17] = 0;
   out_8657312923978741203[18] = 0;
   out_8657312923978741203[19] = 0;
   out_8657312923978741203[20] = 1;
   out_8657312923978741203[21] = 0;
   out_8657312923978741203[22] = 0;
   out_8657312923978741203[23] = 0;
   out_8657312923978741203[24] = 0;
   out_8657312923978741203[25] = 0;
   out_8657312923978741203[26] = 0;
   out_8657312923978741203[27] = 0;
   out_8657312923978741203[28] = 0;
   out_8657312923978741203[29] = 0;
   out_8657312923978741203[30] = 1;
   out_8657312923978741203[31] = 0;
   out_8657312923978741203[32] = 0;
   out_8657312923978741203[33] = 0;
   out_8657312923978741203[34] = 0;
   out_8657312923978741203[35] = 0;
   out_8657312923978741203[36] = 0;
   out_8657312923978741203[37] = 0;
   out_8657312923978741203[38] = 0;
   out_8657312923978741203[39] = 0;
   out_8657312923978741203[40] = 1;
   out_8657312923978741203[41] = 0;
   out_8657312923978741203[42] = 0;
   out_8657312923978741203[43] = 0;
   out_8657312923978741203[44] = 0;
   out_8657312923978741203[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8657312923978741203[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8657312923978741203[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8657312923978741203[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8657312923978741203[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8657312923978741203[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8657312923978741203[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8657312923978741203[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8657312923978741203[53] = -9.8000000000000007*dt;
   out_8657312923978741203[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8657312923978741203[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8657312923978741203[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8657312923978741203[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8657312923978741203[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8657312923978741203[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8657312923978741203[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8657312923978741203[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8657312923978741203[62] = 0;
   out_8657312923978741203[63] = 0;
   out_8657312923978741203[64] = 0;
   out_8657312923978741203[65] = 0;
   out_8657312923978741203[66] = 0;
   out_8657312923978741203[67] = 0;
   out_8657312923978741203[68] = 0;
   out_8657312923978741203[69] = 0;
   out_8657312923978741203[70] = 1;
   out_8657312923978741203[71] = 0;
   out_8657312923978741203[72] = 0;
   out_8657312923978741203[73] = 0;
   out_8657312923978741203[74] = 0;
   out_8657312923978741203[75] = 0;
   out_8657312923978741203[76] = 0;
   out_8657312923978741203[77] = 0;
   out_8657312923978741203[78] = 0;
   out_8657312923978741203[79] = 0;
   out_8657312923978741203[80] = 1;
}
void h_25(double *state, double *unused, double *out_2531010797698552526) {
   out_2531010797698552526[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4750683866668218949) {
   out_4750683866668218949[0] = 0;
   out_4750683866668218949[1] = 0;
   out_4750683866668218949[2] = 0;
   out_4750683866668218949[3] = 0;
   out_4750683866668218949[4] = 0;
   out_4750683866668218949[5] = 0;
   out_4750683866668218949[6] = 1;
   out_4750683866668218949[7] = 0;
   out_4750683866668218949[8] = 0;
}
void h_24(double *state, double *unused, double *out_8812780496296381441) {
   out_8812780496296381441[0] = state[4];
   out_8812780496296381441[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1060701764354558016) {
   out_1060701764354558016[0] = 0;
   out_1060701764354558016[1] = 0;
   out_1060701764354558016[2] = 0;
   out_1060701764354558016[3] = 0;
   out_1060701764354558016[4] = 1;
   out_1060701764354558016[5] = 0;
   out_1060701764354558016[6] = 0;
   out_1060701764354558016[7] = 0;
   out_1060701764354558016[8] = 0;
   out_1060701764354558016[9] = 0;
   out_1060701764354558016[10] = 0;
   out_1060701764354558016[11] = 0;
   out_1060701764354558016[12] = 0;
   out_1060701764354558016[13] = 0;
   out_1060701764354558016[14] = 1;
   out_1060701764354558016[15] = 0;
   out_1060701764354558016[16] = 0;
   out_1060701764354558016[17] = 0;
}
void h_30(double *state, double *unused, double *out_988431336622093080) {
   out_988431336622093080[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2166006474823397806) {
   out_2166006474823397806[0] = 0;
   out_2166006474823397806[1] = 0;
   out_2166006474823397806[2] = 0;
   out_2166006474823397806[3] = 0;
   out_2166006474823397806[4] = 1;
   out_2166006474823397806[5] = 0;
   out_2166006474823397806[6] = 0;
   out_2166006474823397806[7] = 0;
   out_2166006474823397806[8] = 0;
}
void h_26(double *state, double *unused, double *out_7090950772355287470) {
   out_7090950772355287470[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8492187185542275173) {
   out_8492187185542275173[0] = 0;
   out_8492187185542275173[1] = 0;
   out_8492187185542275173[2] = 0;
   out_8492187185542275173[3] = 0;
   out_8492187185542275173[4] = 0;
   out_8492187185542275173[5] = 0;
   out_8492187185542275173[6] = 0;
   out_8492187185542275173[7] = 1;
   out_8492187185542275173[8] = 0;
}
void h_27(double *state, double *unused, double *out_8266388102699664748) {
   out_8266388102699664748[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8756836977027105) {
   out_8756836977027105[0] = 0;
   out_8756836977027105[1] = 0;
   out_8756836977027105[2] = 0;
   out_8756836977027105[3] = 1;
   out_8756836977027105[4] = 0;
   out_8756836977027105[5] = 0;
   out_8756836977027105[6] = 0;
   out_8756836977027105[7] = 0;
   out_8756836977027105[8] = 0;
}
void h_29(double *state, double *unused, double *out_4404704626271163096) {
   out_4404704626271163096[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1722119563846578138) {
   out_1722119563846578138[0] = 0;
   out_1722119563846578138[1] = 1;
   out_1722119563846578138[2] = 0;
   out_1722119563846578138[3] = 0;
   out_1722119563846578138[4] = 0;
   out_1722119563846578138[5] = 0;
   out_1722119563846578138[6] = 0;
   out_1722119563846578138[7] = 0;
   out_1722119563846578138[8] = 0;
}
void h_28(double *state, double *unused, double *out_2047600034018565716) {
   out_2047600034018565716[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6804518580916108712) {
   out_6804518580916108712[0] = 1;
   out_6804518580916108712[1] = 0;
   out_6804518580916108712[2] = 0;
   out_6804518580916108712[3] = 0;
   out_6804518580916108712[4] = 0;
   out_6804518580916108712[5] = 0;
   out_6804518580916108712[6] = 0;
   out_6804518580916108712[7] = 0;
   out_6804518580916108712[8] = 0;
}
void h_31(double *state, double *unused, double *out_1106438730555469824) {
   out_1106438730555469824[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4720037904791258521) {
   out_4720037904791258521[0] = 0;
   out_4720037904791258521[1] = 0;
   out_4720037904791258521[2] = 0;
   out_4720037904791258521[3] = 0;
   out_4720037904791258521[4] = 0;
   out_4720037904791258521[5] = 0;
   out_4720037904791258521[6] = 0;
   out_4720037904791258521[7] = 0;
   out_4720037904791258521[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_5652492354992792635) {
  err_fun(nom_x, delta_x, out_5652492354992792635);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2185114572777528757) {
  inv_err_fun(nom_x, true_x, out_2185114572777528757);
}
void car_H_mod_fun(double *state, double *out_4442504359244018288) {
  H_mod_fun(state, out_4442504359244018288);
}
void car_f_fun(double *state, double dt, double *out_6496366283119291181) {
  f_fun(state,  dt, out_6496366283119291181);
}
void car_F_fun(double *state, double dt, double *out_8657312923978741203) {
  F_fun(state,  dt, out_8657312923978741203);
}
void car_h_25(double *state, double *unused, double *out_2531010797698552526) {
  h_25(state, unused, out_2531010797698552526);
}
void car_H_25(double *state, double *unused, double *out_4750683866668218949) {
  H_25(state, unused, out_4750683866668218949);
}
void car_h_24(double *state, double *unused, double *out_8812780496296381441) {
  h_24(state, unused, out_8812780496296381441);
}
void car_H_24(double *state, double *unused, double *out_1060701764354558016) {
  H_24(state, unused, out_1060701764354558016);
}
void car_h_30(double *state, double *unused, double *out_988431336622093080) {
  h_30(state, unused, out_988431336622093080);
}
void car_H_30(double *state, double *unused, double *out_2166006474823397806) {
  H_30(state, unused, out_2166006474823397806);
}
void car_h_26(double *state, double *unused, double *out_7090950772355287470) {
  h_26(state, unused, out_7090950772355287470);
}
void car_H_26(double *state, double *unused, double *out_8492187185542275173) {
  H_26(state, unused, out_8492187185542275173);
}
void car_h_27(double *state, double *unused, double *out_8266388102699664748) {
  h_27(state, unused, out_8266388102699664748);
}
void car_H_27(double *state, double *unused, double *out_8756836977027105) {
  H_27(state, unused, out_8756836977027105);
}
void car_h_29(double *state, double *unused, double *out_4404704626271163096) {
  h_29(state, unused, out_4404704626271163096);
}
void car_H_29(double *state, double *unused, double *out_1722119563846578138) {
  H_29(state, unused, out_1722119563846578138);
}
void car_h_28(double *state, double *unused, double *out_2047600034018565716) {
  h_28(state, unused, out_2047600034018565716);
}
void car_H_28(double *state, double *unused, double *out_6804518580916108712) {
  H_28(state, unused, out_6804518580916108712);
}
void car_h_31(double *state, double *unused, double *out_1106438730555469824) {
  h_31(state, unused, out_1106438730555469824);
}
void car_H_31(double *state, double *unused, double *out_4720037904791258521) {
  H_31(state, unused, out_4720037904791258521);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
