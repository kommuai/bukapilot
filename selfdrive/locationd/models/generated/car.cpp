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
void err_fun(double *nom_x, double *delta_x, double *out_970018053595319883) {
   out_970018053595319883[0] = delta_x[0] + nom_x[0];
   out_970018053595319883[1] = delta_x[1] + nom_x[1];
   out_970018053595319883[2] = delta_x[2] + nom_x[2];
   out_970018053595319883[3] = delta_x[3] + nom_x[3];
   out_970018053595319883[4] = delta_x[4] + nom_x[4];
   out_970018053595319883[5] = delta_x[5] + nom_x[5];
   out_970018053595319883[6] = delta_x[6] + nom_x[6];
   out_970018053595319883[7] = delta_x[7] + nom_x[7];
   out_970018053595319883[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2532034153412208165) {
   out_2532034153412208165[0] = -nom_x[0] + true_x[0];
   out_2532034153412208165[1] = -nom_x[1] + true_x[1];
   out_2532034153412208165[2] = -nom_x[2] + true_x[2];
   out_2532034153412208165[3] = -nom_x[3] + true_x[3];
   out_2532034153412208165[4] = -nom_x[4] + true_x[4];
   out_2532034153412208165[5] = -nom_x[5] + true_x[5];
   out_2532034153412208165[6] = -nom_x[6] + true_x[6];
   out_2532034153412208165[7] = -nom_x[7] + true_x[7];
   out_2532034153412208165[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_628905704139337664) {
   out_628905704139337664[0] = 1.0;
   out_628905704139337664[1] = 0.0;
   out_628905704139337664[2] = 0.0;
   out_628905704139337664[3] = 0.0;
   out_628905704139337664[4] = 0.0;
   out_628905704139337664[5] = 0.0;
   out_628905704139337664[6] = 0.0;
   out_628905704139337664[7] = 0.0;
   out_628905704139337664[8] = 0.0;
   out_628905704139337664[9] = 0.0;
   out_628905704139337664[10] = 1.0;
   out_628905704139337664[11] = 0.0;
   out_628905704139337664[12] = 0.0;
   out_628905704139337664[13] = 0.0;
   out_628905704139337664[14] = 0.0;
   out_628905704139337664[15] = 0.0;
   out_628905704139337664[16] = 0.0;
   out_628905704139337664[17] = 0.0;
   out_628905704139337664[18] = 0.0;
   out_628905704139337664[19] = 0.0;
   out_628905704139337664[20] = 1.0;
   out_628905704139337664[21] = 0.0;
   out_628905704139337664[22] = 0.0;
   out_628905704139337664[23] = 0.0;
   out_628905704139337664[24] = 0.0;
   out_628905704139337664[25] = 0.0;
   out_628905704139337664[26] = 0.0;
   out_628905704139337664[27] = 0.0;
   out_628905704139337664[28] = 0.0;
   out_628905704139337664[29] = 0.0;
   out_628905704139337664[30] = 1.0;
   out_628905704139337664[31] = 0.0;
   out_628905704139337664[32] = 0.0;
   out_628905704139337664[33] = 0.0;
   out_628905704139337664[34] = 0.0;
   out_628905704139337664[35] = 0.0;
   out_628905704139337664[36] = 0.0;
   out_628905704139337664[37] = 0.0;
   out_628905704139337664[38] = 0.0;
   out_628905704139337664[39] = 0.0;
   out_628905704139337664[40] = 1.0;
   out_628905704139337664[41] = 0.0;
   out_628905704139337664[42] = 0.0;
   out_628905704139337664[43] = 0.0;
   out_628905704139337664[44] = 0.0;
   out_628905704139337664[45] = 0.0;
   out_628905704139337664[46] = 0.0;
   out_628905704139337664[47] = 0.0;
   out_628905704139337664[48] = 0.0;
   out_628905704139337664[49] = 0.0;
   out_628905704139337664[50] = 1.0;
   out_628905704139337664[51] = 0.0;
   out_628905704139337664[52] = 0.0;
   out_628905704139337664[53] = 0.0;
   out_628905704139337664[54] = 0.0;
   out_628905704139337664[55] = 0.0;
   out_628905704139337664[56] = 0.0;
   out_628905704139337664[57] = 0.0;
   out_628905704139337664[58] = 0.0;
   out_628905704139337664[59] = 0.0;
   out_628905704139337664[60] = 1.0;
   out_628905704139337664[61] = 0.0;
   out_628905704139337664[62] = 0.0;
   out_628905704139337664[63] = 0.0;
   out_628905704139337664[64] = 0.0;
   out_628905704139337664[65] = 0.0;
   out_628905704139337664[66] = 0.0;
   out_628905704139337664[67] = 0.0;
   out_628905704139337664[68] = 0.0;
   out_628905704139337664[69] = 0.0;
   out_628905704139337664[70] = 1.0;
   out_628905704139337664[71] = 0.0;
   out_628905704139337664[72] = 0.0;
   out_628905704139337664[73] = 0.0;
   out_628905704139337664[74] = 0.0;
   out_628905704139337664[75] = 0.0;
   out_628905704139337664[76] = 0.0;
   out_628905704139337664[77] = 0.0;
   out_628905704139337664[78] = 0.0;
   out_628905704139337664[79] = 0.0;
   out_628905704139337664[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2381603625928507972) {
   out_2381603625928507972[0] = state[0];
   out_2381603625928507972[1] = state[1];
   out_2381603625928507972[2] = state[2];
   out_2381603625928507972[3] = state[3];
   out_2381603625928507972[4] = state[4];
   out_2381603625928507972[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2381603625928507972[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2381603625928507972[7] = state[7];
   out_2381603625928507972[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4153852161505245629) {
   out_4153852161505245629[0] = 1;
   out_4153852161505245629[1] = 0;
   out_4153852161505245629[2] = 0;
   out_4153852161505245629[3] = 0;
   out_4153852161505245629[4] = 0;
   out_4153852161505245629[5] = 0;
   out_4153852161505245629[6] = 0;
   out_4153852161505245629[7] = 0;
   out_4153852161505245629[8] = 0;
   out_4153852161505245629[9] = 0;
   out_4153852161505245629[10] = 1;
   out_4153852161505245629[11] = 0;
   out_4153852161505245629[12] = 0;
   out_4153852161505245629[13] = 0;
   out_4153852161505245629[14] = 0;
   out_4153852161505245629[15] = 0;
   out_4153852161505245629[16] = 0;
   out_4153852161505245629[17] = 0;
   out_4153852161505245629[18] = 0;
   out_4153852161505245629[19] = 0;
   out_4153852161505245629[20] = 1;
   out_4153852161505245629[21] = 0;
   out_4153852161505245629[22] = 0;
   out_4153852161505245629[23] = 0;
   out_4153852161505245629[24] = 0;
   out_4153852161505245629[25] = 0;
   out_4153852161505245629[26] = 0;
   out_4153852161505245629[27] = 0;
   out_4153852161505245629[28] = 0;
   out_4153852161505245629[29] = 0;
   out_4153852161505245629[30] = 1;
   out_4153852161505245629[31] = 0;
   out_4153852161505245629[32] = 0;
   out_4153852161505245629[33] = 0;
   out_4153852161505245629[34] = 0;
   out_4153852161505245629[35] = 0;
   out_4153852161505245629[36] = 0;
   out_4153852161505245629[37] = 0;
   out_4153852161505245629[38] = 0;
   out_4153852161505245629[39] = 0;
   out_4153852161505245629[40] = 1;
   out_4153852161505245629[41] = 0;
   out_4153852161505245629[42] = 0;
   out_4153852161505245629[43] = 0;
   out_4153852161505245629[44] = 0;
   out_4153852161505245629[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4153852161505245629[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4153852161505245629[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4153852161505245629[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4153852161505245629[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4153852161505245629[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4153852161505245629[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4153852161505245629[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4153852161505245629[53] = -9.8000000000000007*dt;
   out_4153852161505245629[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4153852161505245629[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4153852161505245629[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4153852161505245629[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4153852161505245629[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4153852161505245629[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4153852161505245629[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4153852161505245629[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4153852161505245629[62] = 0;
   out_4153852161505245629[63] = 0;
   out_4153852161505245629[64] = 0;
   out_4153852161505245629[65] = 0;
   out_4153852161505245629[66] = 0;
   out_4153852161505245629[67] = 0;
   out_4153852161505245629[68] = 0;
   out_4153852161505245629[69] = 0;
   out_4153852161505245629[70] = 1;
   out_4153852161505245629[71] = 0;
   out_4153852161505245629[72] = 0;
   out_4153852161505245629[73] = 0;
   out_4153852161505245629[74] = 0;
   out_4153852161505245629[75] = 0;
   out_4153852161505245629[76] = 0;
   out_4153852161505245629[77] = 0;
   out_4153852161505245629[78] = 0;
   out_4153852161505245629[79] = 0;
   out_4153852161505245629[80] = 1;
}
void h_25(double *state, double *unused, double *out_4848459882080520497) {
   out_4848459882080520497[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2894142934506027484) {
   out_2894142934506027484[0] = 0;
   out_2894142934506027484[1] = 0;
   out_2894142934506027484[2] = 0;
   out_2894142934506027484[3] = 0;
   out_2894142934506027484[4] = 0;
   out_2894142934506027484[5] = 0;
   out_2894142934506027484[6] = 1;
   out_2894142934506027484[7] = 0;
   out_2894142934506027484[8] = 0;
}
void h_24(double *state, double *unused, double *out_880766775357849931) {
   out_880766775357849931[0] = state[4];
   out_880766775357849931[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1910077063664713816) {
   out_1910077063664713816[0] = 0;
   out_1910077063664713816[1] = 0;
   out_1910077063664713816[2] = 0;
   out_1910077063664713816[3] = 0;
   out_1910077063664713816[4] = 1;
   out_1910077063664713816[5] = 0;
   out_1910077063664713816[6] = 0;
   out_1910077063664713816[7] = 0;
   out_1910077063664713816[8] = 0;
   out_1910077063664713816[9] = 0;
   out_1910077063664713816[10] = 0;
   out_1910077063664713816[11] = 0;
   out_1910077063664713816[12] = 0;
   out_1910077063664713816[13] = 0;
   out_1910077063664713816[14] = 1;
   out_1910077063664713816[15] = 0;
   out_1910077063664713816[16] = 0;
   out_1910077063664713816[17] = 0;
}
void h_30(double *state, double *unused, double *out_3648200799183219495) {
   out_3648200799183219495[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4022547406985589271) {
   out_4022547406985589271[0] = 0;
   out_4022547406985589271[1] = 0;
   out_4022547406985589271[2] = 0;
   out_4022547406985589271[3] = 0;
   out_4022547406985589271[4] = 1;
   out_4022547406985589271[5] = 0;
   out_4022547406985589271[6] = 0;
   out_4022547406985589271[7] = 0;
   out_4022547406985589271[8] = 0;
}
void h_26(double *state, double *unused, double *out_8046152880464848507) {
   out_8046152880464848507[0] = state[7];
}
void H_26(double *state, double *unused, double *out_410383035254773117) {
   out_410383035254773117[0] = 0;
   out_410383035254773117[1] = 0;
   out_410383035254773117[2] = 0;
   out_410383035254773117[3] = 0;
   out_410383035254773117[4] = 0;
   out_410383035254773117[5] = 0;
   out_410383035254773117[6] = 0;
   out_410383035254773117[7] = 1;
   out_410383035254773117[8] = 0;
}
void h_27(double *state, double *unused, double *out_1811673830636187812) {
   out_1811673830636187812[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1847784095185164360) {
   out_1847784095185164360[0] = 0;
   out_1847784095185164360[1] = 0;
   out_1847784095185164360[2] = 0;
   out_1847784095185164360[3] = 1;
   out_1847784095185164360[4] = 0;
   out_1847784095185164360[5] = 0;
   out_1847784095185164360[6] = 0;
   out_1847784095185164360[7] = 0;
   out_1847784095185164360[8] = 0;
}
void h_29(double *state, double *unused, double *out_1353472468477626150) {
   out_1353472468477626150[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4532778751299981455) {
   out_4532778751299981455[0] = 0;
   out_4532778751299981455[1] = 1;
   out_4532778751299981455[2] = 0;
   out_4532778751299981455[3] = 0;
   out_4532778751299981455[4] = 0;
   out_4532778751299981455[5] = 0;
   out_4532778751299981455[6] = 0;
   out_4532778751299981455[7] = 0;
   out_4532778751299981455[8] = 0;
}
void h_28(double *state, double *unused, double *out_868163977734525377) {
   out_868163977734525377[0] = state[0];
}
void H_28(double *state, double *unused, double *out_549620265769549119) {
   out_549620265769549119[0] = 1;
   out_549620265769549119[1] = 0;
   out_549620265769549119[2] = 0;
   out_549620265769549119[3] = 0;
   out_549620265769549119[4] = 0;
   out_549620265769549119[5] = 0;
   out_549620265769549119[6] = 0;
   out_549620265769549119[7] = 0;
   out_549620265769549119[8] = 0;
}
void h_31(double *state, double *unused, double *out_4573265819796014608) {
   out_4573265819796014608[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4182532316005789769) {
   out_4182532316005789769[0] = 0;
   out_4182532316005789769[1] = 0;
   out_4182532316005789769[2] = 0;
   out_4182532316005789769[3] = 0;
   out_4182532316005789769[4] = 0;
   out_4182532316005789769[5] = 0;
   out_4182532316005789769[6] = 0;
   out_4182532316005789769[7] = 0;
   out_4182532316005789769[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_970018053595319883) {
  err_fun(nom_x, delta_x, out_970018053595319883);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2532034153412208165) {
  inv_err_fun(nom_x, true_x, out_2532034153412208165);
}
void car_H_mod_fun(double *state, double *out_628905704139337664) {
  H_mod_fun(state, out_628905704139337664);
}
void car_f_fun(double *state, double dt, double *out_2381603625928507972) {
  f_fun(state,  dt, out_2381603625928507972);
}
void car_F_fun(double *state, double dt, double *out_4153852161505245629) {
  F_fun(state,  dt, out_4153852161505245629);
}
void car_h_25(double *state, double *unused, double *out_4848459882080520497) {
  h_25(state, unused, out_4848459882080520497);
}
void car_H_25(double *state, double *unused, double *out_2894142934506027484) {
  H_25(state, unused, out_2894142934506027484);
}
void car_h_24(double *state, double *unused, double *out_880766775357849931) {
  h_24(state, unused, out_880766775357849931);
}
void car_H_24(double *state, double *unused, double *out_1910077063664713816) {
  H_24(state, unused, out_1910077063664713816);
}
void car_h_30(double *state, double *unused, double *out_3648200799183219495) {
  h_30(state, unused, out_3648200799183219495);
}
void car_H_30(double *state, double *unused, double *out_4022547406985589271) {
  H_30(state, unused, out_4022547406985589271);
}
void car_h_26(double *state, double *unused, double *out_8046152880464848507) {
  h_26(state, unused, out_8046152880464848507);
}
void car_H_26(double *state, double *unused, double *out_410383035254773117) {
  H_26(state, unused, out_410383035254773117);
}
void car_h_27(double *state, double *unused, double *out_1811673830636187812) {
  h_27(state, unused, out_1811673830636187812);
}
void car_H_27(double *state, double *unused, double *out_1847784095185164360) {
  H_27(state, unused, out_1847784095185164360);
}
void car_h_29(double *state, double *unused, double *out_1353472468477626150) {
  h_29(state, unused, out_1353472468477626150);
}
void car_H_29(double *state, double *unused, double *out_4532778751299981455) {
  H_29(state, unused, out_4532778751299981455);
}
void car_h_28(double *state, double *unused, double *out_868163977734525377) {
  h_28(state, unused, out_868163977734525377);
}
void car_H_28(double *state, double *unused, double *out_549620265769549119) {
  H_28(state, unused, out_549620265769549119);
}
void car_h_31(double *state, double *unused, double *out_4573265819796014608) {
  h_31(state, unused, out_4573265819796014608);
}
void car_H_31(double *state, double *unused, double *out_4182532316005789769) {
  H_31(state, unused, out_4182532316005789769);
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
