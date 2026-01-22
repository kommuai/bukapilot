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
void err_fun(double *nom_x, double *delta_x, double *out_8484248936701507238) {
   out_8484248936701507238[0] = delta_x[0] + nom_x[0];
   out_8484248936701507238[1] = delta_x[1] + nom_x[1];
   out_8484248936701507238[2] = delta_x[2] + nom_x[2];
   out_8484248936701507238[3] = delta_x[3] + nom_x[3];
   out_8484248936701507238[4] = delta_x[4] + nom_x[4];
   out_8484248936701507238[5] = delta_x[5] + nom_x[5];
   out_8484248936701507238[6] = delta_x[6] + nom_x[6];
   out_8484248936701507238[7] = delta_x[7] + nom_x[7];
   out_8484248936701507238[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1881682802568593388) {
   out_1881682802568593388[0] = -nom_x[0] + true_x[0];
   out_1881682802568593388[1] = -nom_x[1] + true_x[1];
   out_1881682802568593388[2] = -nom_x[2] + true_x[2];
   out_1881682802568593388[3] = -nom_x[3] + true_x[3];
   out_1881682802568593388[4] = -nom_x[4] + true_x[4];
   out_1881682802568593388[5] = -nom_x[5] + true_x[5];
   out_1881682802568593388[6] = -nom_x[6] + true_x[6];
   out_1881682802568593388[7] = -nom_x[7] + true_x[7];
   out_1881682802568593388[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2795609792051367190) {
   out_2795609792051367190[0] = 1.0;
   out_2795609792051367190[1] = 0.0;
   out_2795609792051367190[2] = 0.0;
   out_2795609792051367190[3] = 0.0;
   out_2795609792051367190[4] = 0.0;
   out_2795609792051367190[5] = 0.0;
   out_2795609792051367190[6] = 0.0;
   out_2795609792051367190[7] = 0.0;
   out_2795609792051367190[8] = 0.0;
   out_2795609792051367190[9] = 0.0;
   out_2795609792051367190[10] = 1.0;
   out_2795609792051367190[11] = 0.0;
   out_2795609792051367190[12] = 0.0;
   out_2795609792051367190[13] = 0.0;
   out_2795609792051367190[14] = 0.0;
   out_2795609792051367190[15] = 0.0;
   out_2795609792051367190[16] = 0.0;
   out_2795609792051367190[17] = 0.0;
   out_2795609792051367190[18] = 0.0;
   out_2795609792051367190[19] = 0.0;
   out_2795609792051367190[20] = 1.0;
   out_2795609792051367190[21] = 0.0;
   out_2795609792051367190[22] = 0.0;
   out_2795609792051367190[23] = 0.0;
   out_2795609792051367190[24] = 0.0;
   out_2795609792051367190[25] = 0.0;
   out_2795609792051367190[26] = 0.0;
   out_2795609792051367190[27] = 0.0;
   out_2795609792051367190[28] = 0.0;
   out_2795609792051367190[29] = 0.0;
   out_2795609792051367190[30] = 1.0;
   out_2795609792051367190[31] = 0.0;
   out_2795609792051367190[32] = 0.0;
   out_2795609792051367190[33] = 0.0;
   out_2795609792051367190[34] = 0.0;
   out_2795609792051367190[35] = 0.0;
   out_2795609792051367190[36] = 0.0;
   out_2795609792051367190[37] = 0.0;
   out_2795609792051367190[38] = 0.0;
   out_2795609792051367190[39] = 0.0;
   out_2795609792051367190[40] = 1.0;
   out_2795609792051367190[41] = 0.0;
   out_2795609792051367190[42] = 0.0;
   out_2795609792051367190[43] = 0.0;
   out_2795609792051367190[44] = 0.0;
   out_2795609792051367190[45] = 0.0;
   out_2795609792051367190[46] = 0.0;
   out_2795609792051367190[47] = 0.0;
   out_2795609792051367190[48] = 0.0;
   out_2795609792051367190[49] = 0.0;
   out_2795609792051367190[50] = 1.0;
   out_2795609792051367190[51] = 0.0;
   out_2795609792051367190[52] = 0.0;
   out_2795609792051367190[53] = 0.0;
   out_2795609792051367190[54] = 0.0;
   out_2795609792051367190[55] = 0.0;
   out_2795609792051367190[56] = 0.0;
   out_2795609792051367190[57] = 0.0;
   out_2795609792051367190[58] = 0.0;
   out_2795609792051367190[59] = 0.0;
   out_2795609792051367190[60] = 1.0;
   out_2795609792051367190[61] = 0.0;
   out_2795609792051367190[62] = 0.0;
   out_2795609792051367190[63] = 0.0;
   out_2795609792051367190[64] = 0.0;
   out_2795609792051367190[65] = 0.0;
   out_2795609792051367190[66] = 0.0;
   out_2795609792051367190[67] = 0.0;
   out_2795609792051367190[68] = 0.0;
   out_2795609792051367190[69] = 0.0;
   out_2795609792051367190[70] = 1.0;
   out_2795609792051367190[71] = 0.0;
   out_2795609792051367190[72] = 0.0;
   out_2795609792051367190[73] = 0.0;
   out_2795609792051367190[74] = 0.0;
   out_2795609792051367190[75] = 0.0;
   out_2795609792051367190[76] = 0.0;
   out_2795609792051367190[77] = 0.0;
   out_2795609792051367190[78] = 0.0;
   out_2795609792051367190[79] = 0.0;
   out_2795609792051367190[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2726355231791695527) {
   out_2726355231791695527[0] = state[0];
   out_2726355231791695527[1] = state[1];
   out_2726355231791695527[2] = state[2];
   out_2726355231791695527[3] = state[3];
   out_2726355231791695527[4] = state[4];
   out_2726355231791695527[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2726355231791695527[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2726355231791695527[7] = state[7];
   out_2726355231791695527[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7787842145159715712) {
   out_7787842145159715712[0] = 1;
   out_7787842145159715712[1] = 0;
   out_7787842145159715712[2] = 0;
   out_7787842145159715712[3] = 0;
   out_7787842145159715712[4] = 0;
   out_7787842145159715712[5] = 0;
   out_7787842145159715712[6] = 0;
   out_7787842145159715712[7] = 0;
   out_7787842145159715712[8] = 0;
   out_7787842145159715712[9] = 0;
   out_7787842145159715712[10] = 1;
   out_7787842145159715712[11] = 0;
   out_7787842145159715712[12] = 0;
   out_7787842145159715712[13] = 0;
   out_7787842145159715712[14] = 0;
   out_7787842145159715712[15] = 0;
   out_7787842145159715712[16] = 0;
   out_7787842145159715712[17] = 0;
   out_7787842145159715712[18] = 0;
   out_7787842145159715712[19] = 0;
   out_7787842145159715712[20] = 1;
   out_7787842145159715712[21] = 0;
   out_7787842145159715712[22] = 0;
   out_7787842145159715712[23] = 0;
   out_7787842145159715712[24] = 0;
   out_7787842145159715712[25] = 0;
   out_7787842145159715712[26] = 0;
   out_7787842145159715712[27] = 0;
   out_7787842145159715712[28] = 0;
   out_7787842145159715712[29] = 0;
   out_7787842145159715712[30] = 1;
   out_7787842145159715712[31] = 0;
   out_7787842145159715712[32] = 0;
   out_7787842145159715712[33] = 0;
   out_7787842145159715712[34] = 0;
   out_7787842145159715712[35] = 0;
   out_7787842145159715712[36] = 0;
   out_7787842145159715712[37] = 0;
   out_7787842145159715712[38] = 0;
   out_7787842145159715712[39] = 0;
   out_7787842145159715712[40] = 1;
   out_7787842145159715712[41] = 0;
   out_7787842145159715712[42] = 0;
   out_7787842145159715712[43] = 0;
   out_7787842145159715712[44] = 0;
   out_7787842145159715712[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7787842145159715712[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7787842145159715712[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7787842145159715712[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7787842145159715712[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7787842145159715712[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7787842145159715712[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7787842145159715712[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7787842145159715712[53] = -9.8000000000000007*dt;
   out_7787842145159715712[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7787842145159715712[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7787842145159715712[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7787842145159715712[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7787842145159715712[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7787842145159715712[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7787842145159715712[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7787842145159715712[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7787842145159715712[62] = 0;
   out_7787842145159715712[63] = 0;
   out_7787842145159715712[64] = 0;
   out_7787842145159715712[65] = 0;
   out_7787842145159715712[66] = 0;
   out_7787842145159715712[67] = 0;
   out_7787842145159715712[68] = 0;
   out_7787842145159715712[69] = 0;
   out_7787842145159715712[70] = 1;
   out_7787842145159715712[71] = 0;
   out_7787842145159715712[72] = 0;
   out_7787842145159715712[73] = 0;
   out_7787842145159715712[74] = 0;
   out_7787842145159715712[75] = 0;
   out_7787842145159715712[76] = 0;
   out_7787842145159715712[77] = 0;
   out_7787842145159715712[78] = 0;
   out_7787842145159715712[79] = 0;
   out_7787842145159715712[80] = 1;
}
void h_25(double *state, double *unused, double *out_6613396955861225554) {
   out_6613396955861225554[0] = state[6];
}
void H_25(double *state, double *unused, double *out_727370857938124487) {
   out_727370857938124487[0] = 0;
   out_727370857938124487[1] = 0;
   out_727370857938124487[2] = 0;
   out_727370857938124487[3] = 0;
   out_727370857938124487[4] = 0;
   out_727370857938124487[5] = 0;
   out_727370857938124487[6] = 1;
   out_727370857938124487[7] = 0;
   out_727370857938124487[8] = 0;
}
void h_24(double *state, double *unused, double *out_5560718132626239476) {
   out_5560718132626239476[0] = state[4];
   out_5560718132626239476[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4285296456740447623) {
   out_4285296456740447623[0] = 0;
   out_4285296456740447623[1] = 0;
   out_4285296456740447623[2] = 0;
   out_4285296456740447623[3] = 0;
   out_4285296456740447623[4] = 1;
   out_4285296456740447623[5] = 0;
   out_4285296456740447623[6] = 0;
   out_4285296456740447623[7] = 0;
   out_4285296456740447623[8] = 0;
   out_4285296456740447623[9] = 0;
   out_4285296456740447623[10] = 0;
   out_4285296456740447623[11] = 0;
   out_4285296456740447623[12] = 0;
   out_4285296456740447623[13] = 0;
   out_4285296456740447623[14] = 1;
   out_4285296456740447623[15] = 0;
   out_4285296456740447623[16] = 0;
   out_4285296456740447623[17] = 0;
}
void h_30(double *state, double *unused, double *out_328275521956714507) {
   out_328275521956714507[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7644061199429741242) {
   out_7644061199429741242[0] = 0;
   out_7644061199429741242[1] = 0;
   out_7644061199429741242[2] = 0;
   out_7644061199429741242[3] = 0;
   out_7644061199429741242[4] = 1;
   out_7644061199429741242[5] = 0;
   out_7644061199429741242[6] = 0;
   out_7644061199429741242[7] = 0;
   out_7644061199429741242[8] = 0;
}
void h_26(double *state, double *unused, double *out_5394693040675057230) {
   out_5394693040675057230[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3014132460935931737) {
   out_3014132460935931737[0] = 0;
   out_3014132460935931737[1] = 0;
   out_3014132460935931737[2] = 0;
   out_3014132460935931737[3] = 0;
   out_3014132460935931737[4] = 0;
   out_3014132460935931737[5] = 0;
   out_3014132460935931737[6] = 0;
   out_3014132460935931737[7] = 1;
   out_3014132460935931737[8] = 0;
}
void h_27(double *state, double *unused, double *out_4830584414055260283) {
   out_4830584414055260283[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5469297887629316331) {
   out_5469297887629316331[0] = 0;
   out_5469297887629316331[1] = 0;
   out_5469297887629316331[2] = 0;
   out_5469297887629316331[3] = 1;
   out_5469297887629316331[4] = 0;
   out_5469297887629316331[5] = 0;
   out_5469297887629316331[6] = 0;
   out_5469297887629316331[7] = 0;
   out_5469297887629316331[8] = 0;
}
void h_29(double *state, double *unused, double *out_4673397745704131138) {
   out_4673397745704131138[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8154292543744133426) {
   out_8154292543744133426[0] = 0;
   out_8154292543744133426[1] = 1;
   out_8154292543744133426[2] = 0;
   out_8154292543744133426[3] = 0;
   out_8154292543744133426[4] = 0;
   out_8154292543744133426[5] = 0;
   out_8154292543744133426[6] = 0;
   out_8154292543744133426[7] = 0;
   out_8154292543744133426[8] = 0;
}
void h_28(double *state, double *unused, double *out_2707755848683944289) {
   out_2707755848683944289[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1326463856309765276) {
   out_1326463856309765276[0] = 1;
   out_1326463856309765276[1] = 0;
   out_1326463856309765276[2] = 0;
   out_1326463856309765276[3] = 0;
   out_1326463856309765276[4] = 0;
   out_1326463856309765276[5] = 0;
   out_1326463856309765276[6] = 0;
   out_1326463856309765276[7] = 0;
   out_1326463856309765276[8] = 0;
}
void h_31(double *state, double *unused, double *out_446282915890091251) {
   out_446282915890091251[0] = state[8];
}
void H_31(double *state, double *unused, double *out_758016819815084915) {
   out_758016819815084915[0] = 0;
   out_758016819815084915[1] = 0;
   out_758016819815084915[2] = 0;
   out_758016819815084915[3] = 0;
   out_758016819815084915[4] = 0;
   out_758016819815084915[5] = 0;
   out_758016819815084915[6] = 0;
   out_758016819815084915[7] = 0;
   out_758016819815084915[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8484248936701507238) {
  err_fun(nom_x, delta_x, out_8484248936701507238);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1881682802568593388) {
  inv_err_fun(nom_x, true_x, out_1881682802568593388);
}
void car_H_mod_fun(double *state, double *out_2795609792051367190) {
  H_mod_fun(state, out_2795609792051367190);
}
void car_f_fun(double *state, double dt, double *out_2726355231791695527) {
  f_fun(state,  dt, out_2726355231791695527);
}
void car_F_fun(double *state, double dt, double *out_7787842145159715712) {
  F_fun(state,  dt, out_7787842145159715712);
}
void car_h_25(double *state, double *unused, double *out_6613396955861225554) {
  h_25(state, unused, out_6613396955861225554);
}
void car_H_25(double *state, double *unused, double *out_727370857938124487) {
  H_25(state, unused, out_727370857938124487);
}
void car_h_24(double *state, double *unused, double *out_5560718132626239476) {
  h_24(state, unused, out_5560718132626239476);
}
void car_H_24(double *state, double *unused, double *out_4285296456740447623) {
  H_24(state, unused, out_4285296456740447623);
}
void car_h_30(double *state, double *unused, double *out_328275521956714507) {
  h_30(state, unused, out_328275521956714507);
}
void car_H_30(double *state, double *unused, double *out_7644061199429741242) {
  H_30(state, unused, out_7644061199429741242);
}
void car_h_26(double *state, double *unused, double *out_5394693040675057230) {
  h_26(state, unused, out_5394693040675057230);
}
void car_H_26(double *state, double *unused, double *out_3014132460935931737) {
  H_26(state, unused, out_3014132460935931737);
}
void car_h_27(double *state, double *unused, double *out_4830584414055260283) {
  h_27(state, unused, out_4830584414055260283);
}
void car_H_27(double *state, double *unused, double *out_5469297887629316331) {
  H_27(state, unused, out_5469297887629316331);
}
void car_h_29(double *state, double *unused, double *out_4673397745704131138) {
  h_29(state, unused, out_4673397745704131138);
}
void car_H_29(double *state, double *unused, double *out_8154292543744133426) {
  H_29(state, unused, out_8154292543744133426);
}
void car_h_28(double *state, double *unused, double *out_2707755848683944289) {
  h_28(state, unused, out_2707755848683944289);
}
void car_H_28(double *state, double *unused, double *out_1326463856309765276) {
  H_28(state, unused, out_1326463856309765276);
}
void car_h_31(double *state, double *unused, double *out_446282915890091251) {
  h_31(state, unused, out_446282915890091251);
}
void car_H_31(double *state, double *unused, double *out_758016819815084915) {
  H_31(state, unused, out_758016819815084915);
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
